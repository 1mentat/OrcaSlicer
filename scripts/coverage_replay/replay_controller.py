#!/usr/bin/env python3
"""Fail-closed manifest-bound mapping-warning reconciliation.

This controller performs only bounded JSON/hash work and invokes the already
reviewed pure-Python strict and all-kind matchers.  It never runs LLVM, native
binaries, a compiler, or a coverage merge.  The current run remains
coverage_claim=false even after a successful replay.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import collections
import re
import sys
from pathlib import Path
from typing import Any

PACKAGE_ROOT = Path(__file__).resolve().parent
EXPECTED_COMMIT: str | None = None
EXPECTED_TESTS: int | None = None
EXPECTED_PROFILE_WARNING: str | None = None
EXPECTED_WARNING_OCCURRENCES: int | None = None
HASH_WARNING_RE = re.compile(
    r"^hash-mismatch: No profile record found for '([^']+)' with hash = (0x[0-9A-Fa-f]+)$"
)
EXPECTED_HELPERS: dict[str, tuple[Path, str]] = {}


class ReconciliationError(ValueError):
    pass


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ReconciliationError(message)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ReconciliationError(f"cannot load {label}: {exc}") from exc
    require(isinstance(value, dict), f"{label} is not a JSON object")
    return value


def configure_run(manifest_path: Path) -> dict[str, Any]:
    """Load run-specific identity and helper paths from the single manifest."""
    global EXPECTED_COMMIT, EXPECTED_TESTS, EXPECTED_PROFILE_WARNING
    global EXPECTED_WARNING_OCCURRENCES, EXPECTED_HELPERS
    manifest_path = manifest_path.resolve(strict=True)
    manifest = load_json(manifest_path, "run manifest")
    contract = manifest.get("contract")
    require(manifest.get("schema") == 1 and isinstance(contract, dict),
            "unsupported run manifest schema or missing contract")
    commit = contract.get("expected_commit")
    tests = contract.get("expected_tests")
    warning = contract.get("profile_mapping_warning")
    warning_count = contract.get("warning_occurrences")
    require(isinstance(commit, str) and re.fullmatch(r"[0-9a-f]{40}", commit) is not None,
            "manifest expected_commit must be a full lowercase Git SHA")
    require(isinstance(tests, int) and not isinstance(tests, bool) and tests > 0,
            "manifest expected_tests must be a positive integer")
    require(isinstance(warning, str) and warning.startswith("warning: ") and
            warning.endswith(" functions have mismatched data"),
            "manifest profile_mapping_warning is malformed")
    require(isinstance(warning_count, int) and not isinstance(warning_count, bool) and
            warning_count > 0, "manifest warning_occurrences must be a positive integer")
    helper_data = manifest.get("helpers")
    require(isinstance(helper_data, dict), "manifest helpers must be an object")
    helpers: dict[str, tuple[Path, str]] = {}
    for name in ("all_kind_matcher", "strict_matcher", "strict_decoder"):
        row = helper_data.get(name)
        require(isinstance(row, dict) and isinstance(row.get("path"), str) and
                isinstance(row.get("sha256"), str) and
                re.fullmatch(r"[0-9a-f]{64}", row["sha256"]) is not None,
                f"manifest helper descriptor is malformed: {name}")
        path = Path(row["path"])
        if not path.is_absolute():
            path = manifest_path.parent / path
        helpers[name] = (path.resolve(strict=False), row["sha256"])
    EXPECTED_COMMIT = commit
    EXPECTED_TESTS = tests
    EXPECTED_PROFILE_WARNING = warning
    EXPECTED_WARNING_OCCURRENCES = warning_count
    EXPECTED_HELPERS = helpers
    return manifest


def artifact_path(bundle: dict[str, Any], key: str) -> Path:
    value = bundle.get("artifacts", {}).get(key)
    require(isinstance(value, dict), f"missing artifact descriptor: {key}")
    raw_path = value.get("path")
    expected = value.get("sha256")
    require(isinstance(raw_path, str) and raw_path and isinstance(expected, str) and len(expected) == 64,
            f"malformed artifact descriptor: {key}")
    path = Path(raw_path)
    require(not raw_path.startswith("<"), f"native artifact is still a placeholder: {key}")
    require(path.is_file(), f"artifact does not exist: {key}: {path}")
    actual = sha256(path)
    require(actual == expected, f"artifact hash mismatch for {key}: expected {expected}, got {actual}")
    return path


def verify_helper_hashes() -> None:
    for name, (path, expected) in EXPECTED_HELPERS.items():
        require(path.is_file(), f"reviewed helper missing: {name}: {path}")
        require(sha256(path) == expected, f"reviewed helper changed: {name}")


def _empty_failure_map(failures: dict[str, Any]) -> bool:
    return all(not value for key, value in failures.items() if key != "profile_mapping")


def _validate_structural_shape(structural: dict[str, Any]) -> None:
    """Reject malformed exports before the reviewed matchers inspect geometry."""
    data = structural.get("data")
    require(isinstance(data, list) and data, "structural export has no data")
    function_count = 0
    for data_index, item in enumerate(data):
        require(isinstance(item, dict) and isinstance(item.get("functions"), list),
                f"structural data[{data_index}] lacks functions")
        for function_index, function in enumerate(item["functions"]):
            require(isinstance(function, dict) and isinstance(function.get("name"), str) and
                    function["name"] and isinstance(function.get("filenames"), list) and
                    function["filenames"] and all(isinstance(p, str) and p for p in function["filenames"]),
                    f"structural function {data_index}:{function_index} identity is malformed")
            regions = function.get("regions", [])
            branches = function.get("branches", [])
            mcdc = function.get("mcdc_records", [])
            require(isinstance(regions, list) and isinstance(branches, list) and isinstance(mcdc, list),
                    f"structural function {function['name']} mapping arrays are malformed")
            require(not mcdc, f"structural function {function['name']} contains unsupported MCDC records")
            for index, region in enumerate(regions):
                require(isinstance(region, list) and len(region) >= 8 and
                        all(isinstance(value, int) and not isinstance(value, bool) for value in region),
                        f"structural function {function['name']} region {index} is malformed")
            for index, branch in enumerate(branches):
                require(isinstance(branch, list) and len(branch) >= 9 and
                        all(isinstance(value, int) and not isinstance(value, bool) for value in branch),
                        f"structural function {function['name']} branch {index} is malformed")
            function_count += 1
    require(function_count > 0, "structural export contains no functions")


def _validate_inspector_identity(inspector: dict[str, Any], bundle: dict[str, Any],
                                 inspector_input_path: Path, warning_path: Path,
                                 raw_warning: str, object_hashes: dict[str, str],
                                 runtime_report_hash: str) -> dict[str, Any]:
    """Bind inspector output to the exact report, ELFs, and raw warning dump."""
    require(inspector.get("schema") == 2 and inspector.get("status") == "complete" and
            inspector.get("coverage_claim") is False, "ELF inspector is not complete and diagnostic-only")
    inspector_report = inspector.get("report")
    inspector_input = load_json(inspector_input_path, "ELF inspector input report")
    require(isinstance(inspector_report, dict) and
            inspector_report.get("sha256") == sha256(inspector_input_path) and
            sha256(inspector_input_path) == runtime_report_hash and
            inspector_input.get("schema") == 3 and inspector_input.get("status") == "pass" and
            inspector_input.get("binary_sha256_before") == inspector_input.get("binary_sha256_after"),
            "ELF inspector input is not the stable schema-3 native runtime identity report")
    input_hashes = inspector_input.get("binary_sha256_before")
    require(isinstance(input_hashes, dict) and len(input_hashes) == 5 and
            len(set(input_hashes.values())) == 5 and len(set(object_hashes.values())) == 5 and
            set(input_hashes.values()) == set(object_hashes.values()),
            "ELF inspector input report is not bound to the current five object hashes")
    inventory = inspector.get("warning_inventory")
    require(isinstance(inventory, dict) and inventory.get("sha256") == sha256(warning_path) and
            inventory.get("size_bytes") == len(raw_warning.encode("utf-8")),
            "ELF inspector warning inventory is not bound to the archived raw dump")
    records = inventory.get("records")
    unique_records = inventory.get("unique_records")
    require(isinstance(records, list) and isinstance(unique_records, list) and
            len(records) == EXPECTED_WARNING_OCCURRENCES and
            inventory.get("count") == EXPECTED_WARNING_OCCURRENCES,
            f"raw warning occurrence count is not exactly {EXPECTED_WARNING_OCCURRENCES}")
    require(all(isinstance(row, dict) and row.get("mapping_hash") == 0 for row in records),
            "warning inventory contains a nonzero or malformed mapping hash")

    raw_records = []
    for line in raw_warning.splitlines():
        match = HASH_WARNING_RE.match(line)
        if match:
            raw_records.append((match.group(1), int(match.group(2), 16)))
    require(len(raw_records) == EXPECTED_WARNING_OCCURRENCES and
            raw_records == [(row.get("name"), row.get("mapping_hash")) for row in records],
            "raw warning records do not exactly match inspector records")
    raw_counts = collections.Counter(name for name, _ in raw_records)
    inspector_counts = {row.get("name"): row.get("repeat_count") for row in unique_records}
    require(len(inspector_counts) == len(unique_records) and
            dict(raw_counts) == inspector_counts and
            all(isinstance(row.get("mapping_hash"), int) and row["mapping_hash"] == 0
                for row in unique_records),
            "raw warning multiplicity does not match inspector unique records")

    expected_rows = {row.get("name"): row for row in bundle.get("binary_objects", [])
                     if isinstance(row, dict)}
    binaries = inspector.get("binaries")
    require(isinstance(binaries, list) and len(binaries) == 5,
            "ELF inspector does not contain exactly five binary rows")
    observed_names = set()
    for binary in binaries:
        require(isinstance(binary, dict) and isinstance(binary.get("name"), str),
                "ELF inspector binary row is malformed")
        name = binary["name"]
        require(name in expected_rows and name not in observed_names,
                f"ELF inspector has unexpected or duplicate binary name: {name}")
        observed_names.add(name)
        artifact_key = expected_rows[name].get("artifact_key")
        require(isinstance(artifact_key, str), f"ELF inspector archive row lacks artifact key: {name}")
        artifact = artifact_path(bundle, artifact_key)
        require(binary.get("sha256") == sha256(artifact) == expected_rows[name]["sha256"] and
                binary.get("sha256") == object_hashes[name] and
                binary.get("size_bytes") == artifact.stat().st_size,
                f"ELF inspector hash/size is not bound for {name}")
        require(Path(binary.get("path", "")).name == artifact.name,
                f"ELF inspector path is not bound for {name}")
    require(observed_names == set(expected_rows), "ELF inspector binary names do not match archive rows")
    return {"warning_occurrences": len(records), "warning_names": len(unique_records),
            "binary_names": sorted(observed_names), "raw_dump_hash": sha256(warning_path)}


def validate_run_binding(bundle: dict[str, Any]) -> dict[str, Any]:
    require(bundle.get("schema") == 1, "unsupported bundle schema")
    require(bundle.get("expected_commit") == EXPECTED_COMMIT,
            f"bundle is not bound to manifest commit {EXPECTED_COMMIT}")
    require(bundle.get("coverage_claim") is False, "bundle enables a coverage claim")
    verify_helper_hashes()

    runtime_report_path = artifact_path(bundle, "runtime_test_report")
    runtime = load_json(runtime_report_path, "runtime test report")
    report = load_json(artifact_path(bundle, "coverage_report"), "coverage report")
    runtime_report = load_json(artifact_path(bundle, "runtime_coverage_report"), "runtime coverage report")
    export_summary = load_json(artifact_path(bundle, "coverage_export_summary"), "summary export")
    stderr = artifact_path(bundle, "export_stderr").read_text(encoding="utf-8")
    merged_profile = artifact_path(bundle, "merged_profile")
    compile_commands = artifact_path(bundle, "compile_commands")
    source_manifest = artifact_path(bundle, "source_manifest_libslic3r")
    source_manifest_cgal = artifact_path(bundle, "source_manifest_cgal")

    for name, document in (("runtime test report", runtime), ("coverage report", report),
                           ("runtime coverage report", runtime_report)):
        require(document.get("expected_commit") == EXPECTED_COMMIT or
                document.get("commit") == EXPECTED_COMMIT,
                f"{name} is not bound to {EXPECTED_COMMIT}")
    require(runtime.get("status") == "pass" and runtime.get("junit_test_count") == EXPECTED_TESTS and
            runtime.get("minimum_tests") == EXPECTED_TESTS and
            not runtime.get("failed_tests") and not runtime.get("error_tests") and
            not runtime.get("skipped_tests"),
            f"full native test report is not the accepted {EXPECTED_TESTS}-test run")
    require(report.get("status") == "coverage_integrity_error" and
            report.get("coverage_claim") is False and report.get("provisional") is True and
            report.get("integrity_status") == "error", "coverage report no longer preserves fail-closed status")
    failures = report.get("integrity_failures")
    require(isinstance(failures, dict) and
            failures.get("profile_mapping") == [EXPECTED_PROFILE_WARNING] and
            _empty_failure_map(failures), "coverage report has an unexpected integrity failure")
    require(report.get("raw_summary") == report.get("summary"), "summary was changed while retaining integrity error")
    require(stderr.strip() == EXPECTED_PROFILE_WARNING,
            "export stderr is not the exact manifest-bound profile warning")
    require(export_summary.get("data") and not export_summary["data"][0].get("functions"),
            "summary export unexpectedly became a detailed replay input")

    merged_descriptor = report.get("merged_profile", {})
    require(isinstance(merged_descriptor, dict) and
            merged_descriptor.get("sha256") == sha256(merged_profile),
            "merged profile is not hash-bound to the current coverage report")
    compile_descriptor = next((item for item in report.get("compile_commands", [])
                               if isinstance(item, dict)), None)
    require(isinstance(compile_descriptor, dict) and
            compile_descriptor.get("sha256") == sha256(compile_commands),
            "compile commands are not hash-bound to the current coverage report")
    manifests = report.get("manifest_inventory", [])
    manifest_hashes = {item.get("path"): item.get("sha256") for item in manifests
                       if isinstance(item, dict)}
    require(len(manifest_hashes) >= 2 and
            sha256(source_manifest) in manifest_hashes.values() and
            sha256(source_manifest_cgal) in manifest_hashes.values(),
            "source manifests are not hash-bound to the current coverage report")
    object_hashes = report.get("binary_sha256_before_test")
    require(isinstance(object_hashes, dict) and len(object_hashes) == 5,
            "current coverage report does not bind all five test objects")
    require(len(set(object_hashes.values())) == 5,
            "current coverage report does not bind five distinct test objects")
    require(report.get("binary_sha256_after_test") == object_hashes and
            report.get("binary_sha256_before_export") == object_hashes and
            report.get("binary_sha256_after_export") == object_hashes,
            "coverage report does not preserve binary hashes across test/export")
    binary_rows = bundle.get("binary_objects")
    require(isinstance(binary_rows, list) and len(binary_rows) == 5,
            "bundle must bind exactly five archived test objects")
    report_object_names = {}
    for report_path, digest in object_hashes.items():
        basename = Path(report_path).name
        require(basename.endswith("_tests"),
                f"current report object path has unexpected basename: {basename}")
        report_object_names[basename[:-6]] = digest
    require(len(report_object_names) == 5,
            "current report object names are not unique")
    observed_binary_hashes: set[str] = set()
    for index, row in enumerate(binary_rows):
        require(isinstance(row, dict) and isinstance(row.get("artifact_key"), str),
                f"malformed binary descriptor {index}")
        actual = sha256(artifact_path(bundle, row["artifact_key"]))
        expected = row.get("sha256")
        name = row.get("name")
        require(isinstance(name, str) and name in report_object_names and
                actual == expected == report_object_names[name],
                f"binary {index} hash is not bound to the current report")
        observed_binary_hashes.add(actual)
    require(observed_binary_hashes == set(object_hashes.values()),
            "archived object set does not exactly match the current five binaries")

    return {
        "expected_commit": EXPECTED_COMMIT,
        "full_tests": EXPECTED_TESTS,
        "profile_warning": EXPECTED_PROFILE_WARNING,
        "raw_summary": report["raw_summary"],
        "summary_export_is_not_replay_input": True,
        "five_object_hashes_bound": True,
        "profile_and_source_inputs_bound": True,
        "object_hashes": report_object_names,
        "runtime_report_sha256": sha256(runtime_report_path),
    }


def _load_module(name: str, path: Path) -> Any:
    if path.name == "strict_warning_matcher.py" and str(path.parent) not in sys.path:
        sys.path.insert(0, str(path.parent))
    spec = importlib.util.spec_from_file_location(name, path)
    require(spec is not None and spec.loader is not None, f"cannot import reviewed matcher: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def replay(bundle: dict[str, Any], output: Path) -> dict[str, Any]:
    binding = validate_run_binding(bundle)
    inspector_input_path = artifact_path(bundle, "inspector_input_report")
    warning_path = artifact_path(bundle, "warning_dump")
    warning_dump = warning_path.read_text(encoding="utf-8")
    inspector = load_json(artifact_path(bundle, "inspector"), "current ELF inspector")
    structural = load_json(artifact_path(bundle, "structural_export"), "current structural export")
    _validate_structural_shape(structural)
    _validate_inspector_identity(inspector, bundle, inspector_input_path, warning_path, warning_dump,
                                 binding["object_hashes"], binding["runtime_report_sha256"])
    inventory = inspector.get("warning_inventory")
    require("hash-mismatch:" in warning_dump and EXPECTED_PROFILE_WARNING in warning_dump,
            "raw warning dump does not preserve the current mapping warning inventory")

    strict_module = _load_module("manifest_strict_warning_matcher",
                                 EXPECTED_HELPERS["strict_matcher"][0])
    all_kind_module = _load_module("manifest_all_kind_matcher",
                                   EXPECTED_HELPERS["all_kind_matcher"][0])
    strict_result = strict_module.strict_match(inspector, structural)
    require(strict_result.get("status") == "match" and strict_result.get("coverage_claim") is False,
            f"strict CodeRegion replay rejected current inputs: {strict_result.get('failures')}")
    used = strict_result.get("used_mapping_validation", {})
    require(used.get("status") == "strict_match" and
            used.get("geometry_status") == "exact_ordered_filename_and_code_region_match" and
            used.get("decoded_used_records", 0) > 0,
            "strict replay did not establish exact ordered CodeRegion geometry")
    all_kind_result = all_kind_module.match(inspector, structural)
    require(all_kind_result.get("status") == "match" and all_kind_result.get("coverage_claim") is False,
            f"all-kind replay rejected current inputs: {all_kind_result.get('failures')}")
    require(all_kind_result.get("mapping_signature_scope") == "all_region_kinds_plus_branches" and
            all_kind_result.get("mcdc_status") == "none_observed_fail_closed_if_present" and
            all_kind_result.get("warning_occurrences") == len(inventory["records"]) and
            all_kind_result.get("warning_names") == len(inventory["unique_records"]),
            "all-kind replay totals do not match the current inspector inventory")

    result = {
        "schema": 1,
        "status": "reconciled_diagnostic_no_coverage_claim",
        "coverage_claim": False,
        "warning_waiver": False,
        "denominator_action": "preserve_all_sources_and_all_metrics",
        "raw_warning_preserved": True,
        "run_binding": binding,
        "strict_replay": strict_result,
        "all_kind_replay": all_kind_result,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def self_check() -> None:
    # Pure contract checks: no filesystem/native input is required.
    require(isinstance(EXPECTED_COMMIT, str) and len(EXPECTED_COMMIT) == 40,
            "run manifest commit contract is unavailable")
    require(isinstance(EXPECTED_TESTS, int) and EXPECTED_TESTS > 0,
            "run manifest test-count contract is unavailable")
    require(EXPECTED_PROFILE_WARNING.startswith("warning: ") and
            EXPECTED_PROFILE_WARNING.endswith(" functions have mismatched data"),
            "warning contract malformed")
    for path, expected in EXPECTED_HELPERS.values():
        require(len(expected) == 64 and path.name.endswith(".py"), "helper contract malformed")
    print(json.dumps({"status": "self-check-pass", "native_execution": False,
                      "coverage_claim": False}))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--bundle", type=Path)
    parser.add_argument("--out", type=Path)
    parser.add_argument("--self-check", action="store_true")
    args = parser.parse_args(argv)
    try:
        require(args.manifest is not None, "--manifest is required")
        configure_run(args.manifest)
        if args.self_check:
            require(args.bundle is None and args.out is None, "--self-check cannot take bundle/output")
            self_check()
            return 0
        require(args.bundle is not None and args.out is not None,
                "normal mode requires --bundle and --out")
        bundle = load_json(args.bundle, "manifest-bound input bundle")
        result = replay(bundle, args.out)
        print(json.dumps({"status": result["status"], "output": str(args.out)}))
        return 0
    except ReconciliationError as exc:
        print(json.dumps({"status": "rejected", "coverage_claim": False, "error": str(exc)}))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
