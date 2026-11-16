#!/usr/bin/env python3
"""Replay the reviewed LLVM18 coverage mapping reconciliation for one run.

The run manifest is the only source of run identity, immutable input paths,
hashes, helper locations, and the expected full-suite count. This command is
diagnostic only: a successful replay always keeps ``coverage_claim=false``.
It runs bounded argv commands through ``slicer_process`` and never invokes a
shell.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import time
from typing import Any

SCRIPTS_ROOT = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPTS_ROOT / "coverage_replay"
CONTROLLER = PACKAGE_ROOT / "replay_controller.py"
sys.path.insert(0, str(SCRIPTS_ROOT))
from slicer_process import run_captured

EXPECTED_OBJECTS = ("libslic3r", "fff_print", "sla_print", "libnest2d", "filament_group")
HELPER_FILES = {
    "all_kind_matcher": PACKAGE_ROOT / "all_kinds_matcher.py",
    "strict_matcher": PACKAGE_ROOT / "strict_warning_matcher.py",
    "strict_decoder": PACKAGE_ROOT / "strict_llvm18_mapping.py",
    "coverage_warning_matcher": PACKAGE_ROOT / "coverage_warning_matcher.py",
    "elf_inspector": PACKAGE_ROOT / "elf_covfun_identity.py",
    "replay_controller": CONTROLLER,
    "process_runner": SCRIPTS_ROOT / "slicer_process.py",
}
HASH_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
PROFILE_MAP_WARNING_RE = re.compile(
    r"^warning: (?P<count>[1-9][0-9]*) functions have mismatched data$"
)


class ReplayError(RuntimeError):
    """The run manifest or one of its bound inputs is invalid."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ReplayError(message)


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReplayError(f"cannot load {label}: {exc}") from exc
    _require(isinstance(value, dict), f"{label} must be a JSON object")
    return value


def _regular_file(base: Path, raw: Any, label: str) -> Path:
    _require(isinstance(raw, str) and raw.strip() != "", f"{label} path is missing")
    path = Path(raw)
    if not path.is_absolute():
        path = base / path
    _require(not path.is_symlink(), f"{label} must not be a symlink: {path}")
    try:
        resolved = path.resolve(strict=True)
    except OSError as exc:
        raise ReplayError(f"{label} is missing: {path}") from exc
    _require(resolved.is_file(), f"{label} is not a regular file: {resolved}")
    return resolved


def _descriptor(base: Path, row: Any, label: str) -> tuple[Path, str]:
    _require(isinstance(row, dict), f"{label} descriptor must be an object")
    path = _regular_file(base, row.get("path"), label)
    expected = row.get("sha256")
    _require(isinstance(expected, str) and HASH_RE.fullmatch(expected) is not None,
            f"{label} must have a lowercase SHA-256")
    actual = _sha256(path)
    _require(actual == expected, f"{label} hash mismatch: expected {expected}, got {actual}")
    return path, expected


def _validate_helpers(base: Path, rows: Any) -> dict[str, Path]:
    _require(isinstance(rows, dict), "manifest helpers must be an object")
    _require(set(rows) == set(HELPER_FILES), "manifest must bind the complete reviewed helper closure")
    resolved: dict[str, Path] = {}
    for name, expected_path in HELPER_FILES.items():
        row = rows[name]
        _require(isinstance(row, dict) and isinstance(row.get("path"), str),
                f"helper {name} descriptor is malformed")
        candidate = Path(row["path"])
        if not candidate.is_absolute():
            candidate = base / candidate
        expected_path = expected_path.resolve(strict=True)
        _require(candidate.resolve(strict=False) == expected_path,
                f"helper {name} is not deployed at the packaged path: {candidate}")
        path, _digest = _descriptor(base, row, f"helper {name}")
        resolved[name] = path
    return resolved


def _validate_artifact_rows(base: Path, rows: Any, label: str) -> dict[str, tuple[Path, str]]:
    _require(isinstance(rows, dict), f"manifest {label} must be an object")
    resolved = {}
    for name, descriptor in rows.items():
        resolved[name] = _descriptor(base, descriptor, f"artifact {name}")
    return resolved


def preflight(manifest_path: Path) -> dict[str, Any]:
    """Validate all paths, hashes, run identity, and helper deployment pre-LLVM."""
    started = time.monotonic()
    manifest_path = manifest_path.resolve(strict=True)
    manifest_hash = _sha256(manifest_path)
    manifest = _read_json(manifest_path, "run manifest")
    _require(manifest.get("schema") == 1, "unsupported run manifest schema")
    contract = manifest.get("contract")
    _require(isinstance(contract, dict), "manifest contract is missing")
    commit = contract.get("expected_commit")
    test_count = contract.get("expected_tests")
    warning = contract.get("profile_mapping_warning")
    warning_count = contract.get("warning_occurrences")
    _require(isinstance(commit, str) and COMMIT_RE.fullmatch(commit) is not None,
            "expected_commit must be a full lowercase Git SHA")
    _require(isinstance(test_count, int) and not isinstance(test_count, bool) and test_count > 0,
            "expected_tests must be a positive integer")
    _require(isinstance(warning, str) and PROFILE_MAP_WARNING_RE.fullmatch(warning) is not None,
            "profile_mapping_warning is malformed")
    _require(isinstance(warning_count, int) and not isinstance(warning_count, bool) and
            warning_count > 0, "warning_occurrences must be a positive integer")
    warning_match = PROFILE_MAP_WARNING_RE.fullmatch(warning)
    assert warning_match is not None
    _require(int(warning_match.group("count")) == warning_count,
            "profile_mapping_warning count disagrees with warning_occurrences")

    base = manifest_path.parent
    helper_paths = _validate_helpers(base, manifest.get("helpers"))
    artifact_paths = _validate_artifact_rows(base, manifest.get("artifacts"), "artifacts")
    required = {
        "runtime_test_report", "coverage_report", "coverage_export_summary", "export_stderr",
        "merged_profile", "compile_commands", "source_manifest_libslic3r",
        "source_manifest_cgal", "llvm_cov", "source_filter",
    }
    _require(required.issubset(artifact_paths),
            f"manifest artifacts missing: {sorted(required - set(artifact_paths))}")
    object_rows = manifest.get("binary_objects")
    _require(isinstance(object_rows, list) and len(object_rows) == len(EXPECTED_OBJECTS),
            "manifest must bind exactly five archived test binaries")
    object_paths = {}
    object_hashes = {}
    for row in object_rows:
        _require(isinstance(row, dict) and isinstance(row.get("name"), str),
                "binary object row is malformed")
        name = row["name"]
        _require(name in EXPECTED_OBJECTS and name not in object_paths,
                f"unexpected or duplicate binary object: {name}")
        path, digest = _descriptor(base, row, f"binary object {name}")
        _require(path.name == f"{name}_tests", f"binary object basename mismatch: {name}")
        object_paths[name] = path
        object_hashes[name] = digest
    _require(set(object_paths) == set(EXPECTED_OBJECTS), "binary object set is incomplete")
    _require(len(set(object_hashes.values())) == len(EXPECTED_OBJECTS),
            "five archived test binaries must have distinct hashes")

    runtime_path, runtime_hash = artifact_paths["runtime_test_report"]
    runtime = _read_json(runtime_path, "runtime test report")
    coverage_path = artifact_paths["coverage_report"][0]
    coverage = _read_json(coverage_path, "coverage report")
    summary = _read_json(artifact_paths["coverage_export_summary"][0], "LLVM summary export")
    _require(runtime.get("schema") == 3 and runtime.get("status") == "pass" and
            runtime.get("expected_commit") == commit and
            runtime.get("junit_test_count") == test_count and
            runtime.get("minimum_tests") == test_count and
            runtime.get("failed_tests") == [] and runtime.get("error_tests") == [] and
            runtime.get("skipped_tests") == [], "runtime report is not the exact accepted full-suite run")
    test_report = coverage.get("test_report")
    _require(isinstance(test_report, dict) and test_report.get("sha256") == runtime_hash,
            "coverage test_report hash does not bind the runtime test report")
    profile_inventory = runtime.get("profile_inventory")
    _require(isinstance(profile_inventory, list) and profile_inventory and
            coverage.get("profiles") == profile_inventory,
            "coverage profile inventory does not match the runtime report")
    source_before = runtime.get("source_sha256_before")
    source_after = runtime.get("source_sha256_after")
    _require(isinstance(source_before, dict) and source_before and source_after == source_before,
            "runtime source hashes changed or are missing before/after the run")
    for before_key, after_key, label in (
            ("commit_before", "commit_after", "commit"),
            ("tree_before", "tree_after", "tree")):
        before_value = runtime.get(before_key)
        after_value = runtime.get(after_key)
        if before_key in runtime or after_key in runtime:
            _require(isinstance(before_value, str) and before_value and
                    after_value == before_value,
                    f"runtime {label} changed or is incomplete before/after the run")
            if label == "commit":
                _require(before_value == commit,
                        "runtime commit provenance does not match expected_commit")
            elif "expected_tree" in runtime:
                _require(runtime.get("expected_tree") == before_value,
                        "runtime tree provenance does not match expected_tree")
    for field in ("tracked_before", "tracked_after"):
        if field in runtime:
            _require(runtime[field] in ("", [], {}),
                    f"runtime checkout is not clean in {field}")
    _require(coverage.get("expected_commit") == commit and
            coverage.get("status") == "coverage_integrity_error" and
            coverage.get("coverage_claim") is False and coverage.get("provisional") is True and
            coverage.get("integrity_status") == "error" and
            coverage.get("junit_test_count") == test_count and
            coverage.get("minimum_tests") == test_count,
            "coverage report binding/status does not match diagnostic-only contract")
    failures = coverage.get("integrity_failures")
    _require(isinstance(failures, dict) and
            failures.get("profile_mapping") == [warning] and
            all(not value for key, value in failures.items() if key != "profile_mapping"),
            "coverage report has unexpected integrity failures")
    _require(coverage.get("raw_summary") == coverage.get("summary"),
            "raw coverage summary changed while preserving the integrity error")
    checkout_before = coverage.get("checkout_before")
    checkout_after = coverage.get("checkout_after")
    if checkout_before is not None or checkout_after is not None:
        _require(isinstance(checkout_before, dict) and isinstance(checkout_after, dict) and
                checkout_before.get("head") == commit and checkout_after.get("head") == commit and
                checkout_before.get("tracked_clean") is True and
                checkout_after.get("tracked_clean") is True and
                checkout_before.get("tracked_changes") == [] and
                checkout_after.get("tracked_changes") == [],
                "coverage report checkout provenance is not clean at the expected commit")
    _require(artifact_paths["export_stderr"][0].read_text(encoding="utf-8").strip() == warning,
            "export stderr does not contain the exact recorded profile warning")
    export_data = summary.get("data")
    _require(isinstance(export_data, list) and export_data and
            isinstance(export_data[0], dict) and
            not export_data[0].get("functions"),
            "summary export unexpectedly contains detailed replay data")

    merged_hash = artifact_paths["merged_profile"][1]
    merged_desc = coverage.get("merged_profile")
    _require(isinstance(merged_desc, dict) and merged_desc.get("sha256") == merged_hash,
            "merged profile hash is not bound by coverage report")
    compile_hash = artifact_paths["compile_commands"][1]
    compile_rows = coverage.get("compile_commands")
    _require(isinstance(compile_rows, list) and any(
        isinstance(row, dict) and row.get("sha256") == compile_hash for row in compile_rows),
        "compile commands hash is not bound by coverage report")
    source_manifest_inventory = coverage.get("manifest_inventory")
    _require(isinstance(source_manifest_inventory, list),
            "coverage report source manifest inventory is malformed")
    manifest_hashes = {row.get("sha256") for row in source_manifest_inventory
                       if isinstance(row, dict)}
    _require(artifact_paths["source_manifest_libslic3r"][1] in manifest_hashes and
            artifact_paths["source_manifest_cgal"][1] in manifest_hashes,
            "source manifests are not bound by coverage report")

    report_hashes = coverage.get("binary_sha256_before_test")
    _require(isinstance(report_hashes, dict) and len(report_hashes) == 5 and
            len(set(report_hashes.values())) == 5, "coverage report does not bind five distinct binaries")
    for field in ("binary_sha256_after_test", "binary_sha256_before_export", "binary_sha256_after_export"):
        _require(coverage.get(field) == report_hashes,
                f"coverage report binary hashes changed in {field}")
    report_by_name = {}
    for report_path, digest in report_hashes.items():
        _require(isinstance(report_path, str) and isinstance(digest, str),
                "coverage report binary identity row is malformed")
        basename = Path(report_path).name
        _require(basename.endswith("_tests"),
                f"coverage report binary has unexpected basename: {basename}")
        name = basename[:-6]
        _require(name not in report_by_name, f"duplicate coverage report binary name: {name}")
        report_by_name[name] = digest
    _require(set(report_by_name) == set(EXPECTED_OBJECTS),
            "coverage report binary names do not match required test objects")
    _require(report_by_name == object_hashes,
            "archived ELF hashes do not match coverage report binaries")
    input_report_hashes = runtime.get("binary_sha256_before")
    _require(isinstance(input_report_hashes, dict) and
            input_report_hashes == runtime.get("binary_sha256_after") and
            set(input_report_hashes.values()) == set(object_hashes.values()),
            "runtime report binary hashes do not bind the archived test objects")

    tools = manifest.get("tools")
    _require(isinstance(tools, dict) and isinstance(tools.get("llvm_cov"), dict),
            "manifest LLVM tool descriptor is missing")
    llvm_path, llvm_hash = _descriptor(base, tools["llvm_cov"], "LLVM cov tool")
    _require(llvm_path == artifact_paths["llvm_cov"][0] and llvm_hash == artifact_paths["llvm_cov"][1],
            "LLVM tool descriptor disagrees with its artifact binding")
    _require(os.access(llvm_path, os.X_OK), "LLVM cov tool is not executable")
    source_filter = artifact_paths["source_filter"][0]
    paths = manifest.get("paths")
    _require(isinstance(paths, dict) and isinstance(paths.get("report_binary_root"), str) and
            paths["report_binary_root"].startswith("/"),
            "report_binary_root must be an absolute path namespace")
    working_directory = paths.get("working_directory", str(base))
    _require(isinstance(working_directory, str), "working_directory must be a path")
    working_directory_path = Path(working_directory).resolve(strict=True)
    _require(working_directory_path.is_dir(), "working_directory must be a directory")
    _require(_sha256(manifest_path) == manifest_hash,
            "run manifest changed during preflight")

    return {
        "manifest": manifest,
        "manifest_path": manifest_path,
        "commit": commit,
        "test_count": test_count,
        "warning": warning,
        "warning_count": warning_count,
        "helpers": helper_paths,
        "artifacts": artifact_paths,
        "binary_objects": object_paths,
        "binary_hashes": object_hashes,
        "llvm_cov": llvm_path,
        "source_filter": source_filter,
        "report_binary_root": Path(paths["report_binary_root"]),
        "working_directory": working_directory_path,
        "input_hashes": {
            str(manifest_path): manifest_hash,
            **{str(path): digest for path, digest in artifact_paths.values()},
            **{str(path): _sha256(path) for path in helper_paths.values()},
            **{str(path): _sha256(path) for path in object_paths.values()},
        },
        "preflight_seconds": time.monotonic() - started,
    }


def _capture_stage(label: str, argv: list[str], out: Path, cwd: Path,
                   timeout: float, timings: dict[str, Any]) -> tuple[Path, Path]:
    stdout_path = out / f"{label}.stdout.txt"
    stderr_path = out / f"{label}.stderr.txt"
    started = time.monotonic()
    try:
        result = run_captured(argv, cwd=cwd, timeout=timeout)
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        code = result.returncode
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        code = 124
    except OSError as exc:
        stdout, stderr, code = "", str(exc), 127
    if isinstance(stdout, bytes):
        stdout = stdout.decode("utf-8", "replace")
    if isinstance(stderr, bytes):
        stderr = stderr.decode("utf-8", "replace")
    stdout_path.write_text(stdout, encoding="utf-8")
    stderr_path.write_text(stderr, encoding="utf-8")
    timings[label] = {
        "elapsed_wall_seconds": time.monotonic() - started,
        "returncode": code,
        "argv": argv,
        "stdout": {"path": str(stdout_path), "sha256": _sha256(stdout_path),
                   "size_bytes": stdout_path.stat().st_size},
        "stderr": {"path": str(stderr_path), "sha256": _sha256(stderr_path),
                   "size_bytes": stderr_path.stat().st_size},
    }
    if code != 0:
        raise ReplayError(f"{label} failed with exit status {code}")
    return stdout_path, stderr_path


def _write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _artifact(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": _sha256(path)}


def _build_bundle(prepared: dict[str, Any], out: Path, inspector: Path,
                  structural: Path, warning_dump: Path) -> dict[str, Any]:
    artifacts = prepared["artifacts"]
    layout = out / "binary-layout"
    rows: list[dict[str, str]] = []
    bundle_artifacts: dict[str, dict[str, str]] = {}
    for name in EXPECTED_OBJECTS:
        dest = layout / name / f"{name}_tests"
        _require(dest.is_file(), f"private binary layout is missing: {name}")
        digest = _sha256(dest)
        _require(digest == prepared["binary_hashes"][name],
                f"copied binary hash changed: {name}")
        key = f"binary_{name}"
        bundle_artifacts[key] = _artifact(dest)
        rows.append({"name": name, "artifact_key": key, "sha256": digest})

    keys = {
        "compile_commands": "compile_commands",
        "coverage_export_summary": "coverage_export_summary",
        "coverage_report": "coverage_report",
        "export_stderr": "export_stderr",
        "inspector_input_report": "runtime_test_report",
        "merged_profile": "merged_profile",
        "runtime_coverage_report": "coverage_report",
        "runtime_test_report": "runtime_test_report",
        "source_manifest_libslic3r": "source_manifest_libslic3r",
        "source_manifest_cgal": "source_manifest_cgal",
    }
    for bundle_key, manifest_key in keys.items():
        bundle_artifacts[bundle_key] = _artifact(artifacts[manifest_key][0])
    bundle_artifacts["inspector"] = _artifact(inspector)
    bundle_artifacts["structural_export"] = _artifact(structural)
    bundle_artifacts["warning_dump"] = _artifact(warning_dump)
    return {
        "schema": 1,
        "status": "native_archive_inputs_present",
        "expected_commit": prepared["commit"],
        "coverage_claim": False,
        "artifacts": bundle_artifacts,
        "binary_objects": rows,
    }


def execute(prepared: dict[str, Any], output: Path) -> dict[str, Any]:
    """Run LLVM exports, inspector, and the pinned diagnostic matcher chain."""
    output = output.resolve()
    _require(not output.exists(), f"output already exists: {output}")
    _require(output.parent.is_dir(), f"output parent is missing: {output.parent}")
    output.mkdir()
    timings: dict[str, Any] = {"schema": 1, "coverage_claim": False,
                               "preflight_wall_seconds": prepared["preflight_seconds"],
                               "stages": {}}
    started_total = time.monotonic()
    try:
        layout_start = time.monotonic()
        # The exact five names/order are part of the inspector contract.
        layout = output / "binary-layout"
        for name in EXPECTED_OBJECTS:
            source = prepared["binary_objects"][name]
            dest = layout / name / f"{name}_tests"
            dest.parent.mkdir(parents=True, exist_ok=True)
            try:
                os.link(source, dest)
            except OSError:
                shutil.copyfile(source, dest)
            _require(_sha256(dest) == prepared["binary_hashes"][name],
                    f"copied binary hash changed: {name}")
        timings["stages"]["binary-layout"] = {
            "elapsed_wall_seconds": time.monotonic() - layout_start,
            "status": "hash_verified",
        }
        extras = []
        for name in EXPECTED_OBJECTS[1:]:
            extras += ["-object", str(layout / name / f"{name}_tests")]
        structural_args = [str(prepared["llvm_cov"]), "export", "-format=text",
                           "-skip-expansions", "--num-threads=1", "-instr-profile",
                           str(prepared["artifacts"]["merged_profile"][0]),
                           str(layout / "libslic3r" / "libslic3r_tests"), *extras,
                           str(prepared["source_filter"])]
        dump_args = [str(prepared["llvm_cov"]), "export", "--summary-only", "--dump",
                     "-format=text", "--num-threads=1", "-instr-profile",
                     str(prepared["artifacts"]["merged_profile"][0]),
                     str(layout / "libslic3r" / "libslic3r_tests"), *extras]
        structural_stdout, _ = _capture_stage("structural-export", structural_args, output,
                                              prepared["working_directory"], 1100,
                                              timings["stages"])
        structural_path = output / "structural-export.json"
        shutil.copyfile(structural_stdout, structural_path)
        dump_stdout, dump_stderr = _capture_stage("warning-dump", dump_args, output,
                                                  prepared["working_directory"], 1100,
                                                  timings["stages"])
        # llvm-cov emits JSON to stdout and its hash-mismatch rows to stderr.
        warning_path = output / "warning-inventory.raw.txt"
        dump_bytes = dump_stdout.read_bytes()
        stderr_bytes = dump_stderr.read_bytes()
        warning_path.write_bytes(dump_bytes +
                                 (b"" if not dump_bytes or dump_bytes.endswith(b"\n") else b"\n") +
                                 stderr_bytes)
        lines = warning_path.read_text(encoding="utf-8", errors="replace").splitlines()
        observed_warnings = sum(line.startswith("hash-mismatch:") for line in lines)
        observed_headers = sum(line.startswith("warning:") for line in lines)
        _require(observed_warnings == prepared["warning_count"] and observed_headers == 1,
                f"warning inventory count mismatch: {observed_warnings}/{observed_headers}")

        inspector_path = output / "inspector.json"
        inspector_argv = [sys.executable, str(HELPER_FILES["elf_inspector"]),
                          "--report", str(prepared["artifacts"]["runtime_test_report"][0]),
                          "--warnings", str(warning_path), "--binary-root",
                          str(output / "binary-layout"), "--report-binary-root",
                          str(prepared["report_binary_root"]), "--output", str(inspector_path)]
        _capture_stage("elf-inspector", inspector_argv, output,
                       prepared["working_directory"], 300, timings["stages"])
        bundle = _build_bundle(prepared, output, inspector_path, structural_path, warning_path)
        bundle_path = output / "replay-input-bundle.json"
        _write_json(bundle_path, bundle)
        replay_path = output / "replay-result.json"
        controller_argv = [sys.executable, str(CONTROLLER), "--manifest",
                           str(prepared["manifest_path"]), "--bundle", str(bundle_path),
                           "--out", str(replay_path)]
        _capture_stage("strict-all-kind-controller", controller_argv, output,
                       prepared["working_directory"], 300, timings["stages"])
        replay = _read_json(replay_path, "replay result")
        _require(replay.get("status") == "reconciled_diagnostic_no_coverage_claim" and
                replay.get("coverage_claim") is False and replay.get("warning_waiver") is False,
                "reviewed controller did not preserve diagnostic-only status")

        rehash_start = time.monotonic()
        after = {path: _sha256(Path(path)) for path in prepared["input_hashes"]}
        _require(after == prepared["input_hashes"], "an immutable input, helper, or binary changed")
        timings["stages"]["immutable_rehash"] = {
            "elapsed_wall_seconds": time.monotonic() - rehash_start,
            "status": "unchanged",
        }
        _write_json(output / "input-hashes-before-after.json",
                    {"before": prepared["input_hashes"], "after": after})
        timings["execution_wall_seconds"] = time.monotonic() - started_total
        timings["total_wall_seconds"] = prepared["preflight_seconds"] + timings["execution_wall_seconds"]
        timings["status"] = replay["status"]
        _write_json(output / "timings.json", timings)
        _write_json(output / "terminal-summary.json", {
            "status": replay["status"], "coverage_claim": False,
            "warning_occurrences": observed_warnings, "warning_headers": observed_headers,
            "structural_sha256": _sha256(structural_path),
            "raw_warning_sha256": _sha256(warning_path),
            "inspector_sha256": _sha256(inspector_path),
            "replay_result_sha256": _sha256(replay_path),
            "input_hashes_unchanged": True,
        })
        return {"status": replay["status"], "coverage_claim": False,
                "output": str(output), "timings": str(output / "timings.json")}
    except Exception as exc:
        timings["execution_wall_seconds"] = time.monotonic() - started_total
        timings["total_wall_seconds"] = prepared["preflight_seconds"] + timings["execution_wall_seconds"]
        timings["status"] = "failed_diagnostic_no_coverage_claim"
        timings["error"] = str(exc)
        _write_json(output / "timings.json", timings)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        prepared = preflight(args.manifest)
        result = execute(prepared, args.out)
        print(json.dumps(result, sort_keys=True))
        return 0
    except (ReplayError, OSError) as exc:
        print(json.dumps({"status": "rejected", "coverage_claim": False,
                          "error": str(exc)}), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
