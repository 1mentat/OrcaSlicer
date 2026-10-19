#!/usr/bin/env python3
"""Build, run, and report coverage for one Catch2-tagged native module."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from slicer_process import run_captured


class WorkflowError(RuntimeError):
    pass


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def under(path: Path, parent: Path) -> bool:
    try:
        path.resolve(strict=True).relative_to(parent.resolve(strict=True))
        return True
    except (OSError, ValueError):
        return False


def git_snapshot(root: Path) -> dict:
    def git(*argv: str) -> str:
        result = run_captured(["git", *argv], cwd=root, timeout=30)
        if result.returncode:
            raise WorkflowError(f"git {' '.join(argv)} failed: {result.stderr.strip()}")
        return result.stdout.strip()

    top = Path(git("rev-parse", "--show-toplevel")).resolve()
    if top != root.resolve():
        raise WorkflowError(f"--root is not the Git root: {top}")
    return {"head": git("rev-parse", "HEAD"),
            "tracked_status": git("status", "--porcelain=v1", "-uno")}


def file_snapshot(paths: dict[str, Path]) -> dict[str, str]:
    return {name: sha256(path) for name, path in paths.items()}


def _record_output_path(value: str, directory: Path, build: Path) -> tuple[Path, str]:
    if not isinstance(value, str) or not value.strip():
        raise WorkflowError("compile database record has an empty output path")
    # CMake output metadata and MSVC command records may use Windows separators.
    normalized = value.strip().strip('"').replace("\\", os.sep)
    output = Path(normalized)
    if not output.is_absolute():
        output = directory / output
    try:
        resolved = output.resolve(strict=False)
        resolved.relative_to(build.resolve(strict=True))
    except (OSError, RuntimeError, ValueError) as exc:
        raise WorkflowError(f"compile output is outside the build tree: {value!r}") from exc
    owner_match = re.search(r"(?:^|[/\\])CMakeFiles[/\\]([^/\\]+)\.dir(?:[/\\]|$)", value)
    if owner_match is None:
        raise WorkflowError(f"unsupported CMake output ownership: {value!r}")
    return resolved, owner_match.group(1)


def _command_output(tokens: list[str]) -> str | None:
    found: list[str] = []
    index = 0
    while index < len(tokens):
        token = tokens[index].strip('"')
        if token in {"-o", "--output", "/Fo", "/fo"}:
            if index + 1 >= len(tokens):
                raise WorkflowError("compile command has an output option without a path")
            found.append(tokens[index + 1].strip('"'))
            index += 2
            continue
        if token.startswith("--output="):
            found.append(token.split("=", 1)[1].strip('"'))
        elif token.startswith(("/Fo", "/fo")) and len(token) > 3:
            found.append(token[3:].strip('"'))
        elif token.startswith("-o") and len(token) > 2:
            # Avoid treating unrelated flags such as -openmp as output paths.
            candidate = token[2:].strip('"')
            if "/" in candidate or "\\" in candidate or candidate.endswith((".o", ".obj")):
                found.append(candidate)
        index += 1
    if len(found) > 1:
        raise WorkflowError("compile command has ambiguous output paths")
    return found[0] if found else None


def _entry_command_tokens(entry: dict, label: str) -> list[list[str]]:
    representations: list[list[str]] = []
    if "arguments" in entry:
        arguments = entry["arguments"]
        if not isinstance(arguments, list) or not arguments or not all(isinstance(item, str) for item in arguments):
            raise WorkflowError(f"compile database {label} has invalid arguments")
        representations.append(arguments)
    if "command" in entry:
        command = entry["command"]
        if not isinstance(command, str) or not command.strip():
            raise WorkflowError(f"compile database {label} has invalid command")
        try:
            # CMake's command field is a quoted command string; tokenize only,
            # never execute it. POSIX shlex also preserves quoted Windows paths.
            representations.append(shlex.split(command, posix=True))
        except ValueError as exc:
            raise WorkflowError(f"compile database {label} command cannot be parsed: {exc}") from exc
    if not representations:
        raise WorkflowError(f"compile database {label} has neither command nor arguments")
    return representations


def validate_compile_target_membership(path: Path, build: Path, source: Path,
                                       target: str) -> dict:
    """Require CMake's compile database to show the requested TU in this target."""
    try:
        records = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise WorkflowError(f"cannot read compile database: {exc}") from exc
    if not isinstance(records, list) or not records:
        raise WorkflowError("compile database must contain a nonempty array of records")
    try:
        build_real = build.resolve(strict=True)
        source_real = source.resolve(strict=True)
    except OSError as exc:
        raise WorkflowError(f"cannot resolve build/source for compile database: {exc}") from exc

    source_rows: list[dict] = []
    for index, entry in enumerate(records):
        label = f"record {index}"
        if not isinstance(entry, dict):
            raise WorkflowError(f"compile database {label} is not an object")
        directory_value, file_value = entry.get("directory"), entry.get("file")
        if not isinstance(directory_value, str) or not directory_value.strip():
            raise WorkflowError(f"compile database {label} has no valid directory")
        if not isinstance(file_value, str) or not file_value.strip():
            raise WorkflowError(f"compile database {label} has no valid source file")
        directory = Path(directory_value)
        if not directory.is_absolute():
            directory = build_real / directory
        try:
            directory = directory.resolve(strict=True)
            record_source = Path(file_value)
            if not record_source.is_absolute():
                record_source = directory / record_source
            record_source = record_source.resolve(strict=False)
        except (OSError, RuntimeError) as exc:
            raise WorkflowError(f"compile database {label} has an unresolvable source/directory: {exc}") from exc
        if record_source != source_real:
            continue

        representations = _entry_command_tokens(entry, label)
        output_values: list[str] = []
        if "output" in entry:
            if not isinstance(entry["output"], str) or not entry["output"].strip():
                raise WorkflowError(f"compile database {label} has invalid output metadata")
            output_values.append(entry["output"])
        command_outputs = [_command_output(tokens) for tokens in representations]
        output_values.extend(value for value in command_outputs if value is not None)
        if not output_values:
            raise WorkflowError(f"unsupported target ownership: {label} has no CMake output path")
        resolved_outputs: list[tuple[Path, str]] = []
        for value in output_values:
            resolved_outputs.append(_record_output_path(value, directory, build_real))
        distinct = {resolved for resolved, _ in resolved_outputs}
        owners = {owner for _, owner in resolved_outputs}
        if len(distinct) != 1 or len(owners) != 1:
            raise WorkflowError(f"compile database {label} has conflicting output ownership")
        source_rows.append({"owner": next(iter(owners)), "output": str(next(iter(distinct)))})

    if not source_rows:
        raise WorkflowError(f"compile database has no record for requested test source: {source_real}")
    owners = sorted({row["owner"] for row in source_rows})
    if target not in owners:
        raise WorkflowError(
            f"requested target {target!r} does not own test source; compile database owners={owners}")
    return {"requested_target": target, "test_source": str(source_real),
            "matching_records": source_rows, "owners": owners}


def save_receipt(path: Path, receipt: dict) -> None:
    tmp = path.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(receipt, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    tmp.replace(path)


def xml_names(xml_path: Path, tag: str, root_dir: Path, build_dir: Path,
              source: Path, expected: int) -> list[str]:
    try:
        root = ET.parse(xml_path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise WorkflowError(f"invalid Catch XML inventory: {exc}") from exc
    listing = root if root.tag.rsplit("}", 1)[-1] == "MatchingTests" else next(
        (node for node in root.iter() if node.tag.rsplit("}", 1)[-1] == "MatchingTests"), None)
    cases = listing.findall("TestCase") if listing is not None else []
    names: list[str] = []
    for case in cases:
        name = (case.findtext("Name") or "").strip()
        tags = case.findtext("Tags") or ""
        filename = (case.findtext("SourceInfo/File") or "").strip()
        test_file = Path(filename)
        candidates = [test_file] if test_file.is_absolute() else [root_dir / test_file, build_dir / test_file]
        if not name or tag not in re.findall(r"\[([^\]]+)\]", tags):
            raise WorkflowError(f"inventory case has missing name or [{tag}] tag: {name!r}")
        same_source = False
        for candidate in candidates:
            try:
                same_source |= candidate.resolve(strict=True) == source.resolve(strict=True)
            except OSError:
                pass
        if not same_source:
            raise WorkflowError(f"inventory case {name!r} comes from {filename!r}, expected {source}")
        names.append(name)
    if len(names) != expected or len(set(names)) != len(names):
        raise WorkflowError(f"inventory has {len(names)} cases/{len(set(names))} unique; expected {expected} unique")
    return names


def validate_junit(path: Path, expected_names: set[str]) -> dict[str, int]:
    try:
        root = ET.parse(path).getroot()
    except (ET.ParseError, OSError) as exc:
        raise WorkflowError(f"invalid Catch JUnit report: {exc}") from exc
    local = lambda node: node.tag.rsplit("}", 1)[-1]
    suites = [node for node in root.iter() if local(node) == "testsuite"]
    cases = [node for node in root.iter() if local(node) == "testcase"]
    represented: set[str] = set()
    unknown: list[str] = []
    child_counts = {key: 0 for key in ("failures", "errors", "skipped")}
    for case in cases:
        name = case.get("name", "")
        owners = [expected for expected in expected_names
                  if name == expected or name.startswith(expected + "/")]
        if owners:
            represented.add(max(owners, key=len))
        else:
            unknown.append(name)
        for marker in list(case):
            tag = local(marker)
            if tag == "failure":
                child_counts["failures"] += 1
            elif tag == "error":
                child_counts["errors"] += 1
            elif tag == "skipped":
                child_counts["skipped"] += 1
    missing = expected_names - represented
    if not cases or unknown or missing:
        raise WorkflowError(
            f"JUnit identity set differs: missing={sorted(missing)}, extra={sorted(unknown)}")
    suite_counts = {key: sum(int(suite.get(key, "0")) for suite in suites)
                    for key in child_counts}
    counts = {key: max(child_counts[key], suite_counts[key]) for key in child_counts}
    return counts


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("root", "build", "binary", "test-source", "production-source", "out",
                 "cmake", "llvm-profdata", "llvm-cov"):
        parser.add_argument(f"--{name}", required=True)
    parser.add_argument("--target", required=True)
    parser.add_argument("--tag", required=True)
    parser.add_argument("--expected-cases", required=True, type=int)
    parser.add_argument("--compile-commands")
    parser.add_argument("--seed", type=int, default=21907)
    parser.add_argument("--timeout", type=float, default=1800)
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> int:
    started = time.perf_counter()
    out = Path(args.out).absolute()
    if out.exists() or out.is_symlink():
        raise WorkflowError(f"output path must be new: {out}")
    root = Path(args.root).resolve(strict=True)
    build_arg = Path(args.build)
    build = build_arg.resolve(strict=True)  # resolves the canonical build symlink
    try:
        # Resolve existing symlinks before making any missing evidence parents.
        out = out.resolve(strict=False)
        out.parent.relative_to(build)
    except (OSError, RuntimeError, ValueError) as exc:
        raise WorkflowError("output must resolve inside the build tree") from exc
    if out.exists() or out.is_symlink():
        raise WorkflowError(f"output path must be new: {out}")
    binary_arg = Path(os.path.abspath(args.binary))
    binary_existed_before_build = os.path.lexists(binary_arg)
    try:
        binary = binary_arg.resolve(strict=binary_existed_before_build)
    except OSError as exc:
        raise WorkflowError(f"cannot resolve test binary path before build: {binary_arg}: {exc}") from exc
    try:
        binary.relative_to(build)
    except ValueError as exc:
        raise WorkflowError("test binary must resolve inside the build tree") from exc
    tag = args.tag.strip("[]")
    if not tag or "]" in tag or "[" in tag or args.expected_cases < 1:
        raise WorkflowError("tag must be one plain Catch2 tag and expected-cases must be positive")
    test_source = Path(args.test_source)
    prod_source = Path(args.production_source)
    test_source = (root / test_source).resolve(strict=True) if not test_source.is_absolute() else test_source.resolve(strict=True)
    prod_source = (root / prod_source).resolve(strict=True) if not prod_source.is_absolute() else prod_source.resolve(strict=True)
    if not under(test_source, root) or not under(prod_source, root):
        raise WorkflowError("test/production source must be inside root")
    compile_commands = Path(args.compile_commands) if args.compile_commands else build / "compile_commands.json"
    if not compile_commands.is_absolute():
        compile_commands = root / compile_commands
    compile_commands = compile_commands.resolve(strict=True)
    if not under(compile_commands, build):
        raise WorkflowError("compile_commands.json must resolve inside the build tree")
    compile_membership = validate_compile_target_membership(
        compile_commands, build, test_source, args.target)
    for key in ("cmake", "llvm_profdata", "llvm_cov"):
        tool = Path(getattr(args, key)).resolve(strict=True)
        if not tool.is_file() or not os.access(tool, os.X_OK):
            raise WorkflowError(f"tool is not an executable file: {tool}")
        setattr(args, key, str(tool))
    if binary_existed_before_build and (not binary.is_file() or not os.access(binary, os.X_OK)):
        raise WorkflowError(f"test binary is not executable: {binary}")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise WorkflowError("timeout must be finite and positive")
    tracked_before = git_snapshot(root)
    if tracked_before["tracked_status"]:
        raise WorkflowError("tracked checkout is dirty before the workflow")
    files = {"test_source": test_source, "production_source": prod_source,
             "compile_commands": compile_commands}
    if binary_existed_before_build:
        files["elf_before_build"] = binary
    hashes_before = file_snapshot(files)
    out.parent.mkdir(parents=True, exist_ok=True)
    if not under(out.parent, build):
        raise WorkflowError("output parent no longer resolves inside the build tree")
    out.mkdir()
    profiles = out / "module-profiles"
    profiles.mkdir()
    build_profiles = out / "build-profiles"
    inventory_profiles = out / "inventory-profiles"
    build_profiles.mkdir()
    inventory_profiles.mkdir()
    receipt = {"status": "running", "started_utc": utc_now(), "arguments": vars(args),
               "root": str(root), "build_arg": str(build_arg), "build_resolved": str(build),
               "binary": str(binary), "binary_exists_before_build": binary_existed_before_build,
               "compile_target_preflight": compile_membership,
               "tracked_before": tracked_before,
               "hashes_before": hashes_before, "stages": [],
               "coverage_scope": "production_source_only", "global_threshold_claim": False}
    receipt_path = out / "run.json"
    save_receipt(receipt_path, receipt)

    env = os.environ.copy()
    env["LLVM_PROFILE_FILE"] = str(profiles / "%m.profraw")
    build_env = env.copy()
    build_env["LLVM_PROFILE_FILE"] = str(build_profiles / "%m.profraw")
    inventory_env = env.copy()
    inventory_env["LLVM_PROFILE_FILE"] = str(inventory_profiles / "%m.profraw")

    def stage(name: str, argv: list[str], log: Path, *, capture: bool = False,
              stage_env: dict[str, str] | None = None,
              allow_failure: bool = False) -> subprocess.CompletedProcess:
        record = {"name": name, "argv": argv, "log": str(log), "started_utc": utc_now()}
        tick = time.perf_counter()
        try:
            if capture:
                result = run_captured(argv, cwd=root, env=stage_env or env, timeout=args.timeout)
                log.write_text(result.stdout or "", encoding="utf-8")
                log.with_suffix(".stderr.txt").write_text(result.stderr or "", encoding="utf-8")
            else:
                with log.open("wb") as stream:
                    result = run_captured(argv, cwd=root, env=stage_env or env,
                                          timeout=args.timeout, output_file=stream)
            record["returncode"] = result.returncode
        except subprocess.TimeoutExpired:
            record.update({"returncode": 124, "timed_out": True})
            raise WorkflowError(f"stage timed out: {name}")
        except OSError as exc:
            record.update({"returncode": None, "error": str(exc)})
            raise WorkflowError(f"cannot run {name}: {exc}") from exc
        finally:
            record["elapsed_seconds"] = round(time.perf_counter() - tick, 3)
            record["ended_utc"] = utc_now()
            receipt["stages"].append(record)
            save_receipt(receipt_path, receipt)
        if result.returncode != 0 and not allow_failure:
            raise WorkflowError(f"stage {name} failed with exit {result.returncode}")
        return result

    try:
        stage("build", [args.cmake, "--build", str(build), "--config", "Release", "--target", args.target, "--parallel", "4"],
              out / "build.log", stage_env=build_env)
        try:
            binary = binary_arg.resolve(strict=True)
        except OSError as exc:
            raise WorkflowError(f"build did not produce test binary: {binary_arg}") from exc
        if not under(binary, build):
            raise WorkflowError("built test binary resolves outside the build tree")
        if not binary.is_file() or not os.access(binary, os.X_OK):
            raise WorkflowError(f"build did not produce an executable test binary: {binary}")
        receipt["binary"] = str(binary)
        hashes_after_build = {"elf": sha256(binary)}
        receipt["elf_after_build_sha256"] = hashes_after_build["elf"]
        save_receipt(receipt_path, receipt)
        stage("inventory", [str(binary), f"[{tag}]", "--list-tests", "--reporter", "xml"],
              out / "inventory.stdout.xml", capture=True, stage_env=inventory_env)
        inventory_log = out / "inventory.stdout.xml"
        names = xml_names(inventory_log, tag, root, build, test_source, args.expected_cases)
        receipt["inventory_names"] = names
        save_receipt(receipt_path, receipt)
        junit = out / "tests.junit.xml"
        test_result = stage("test", [str(binary), f"[{tag}]", "--order", "rand", "--rng-seed", str(args.seed),
                                      "--reporter", "junit", "--out", str(junit)], out / "test.log",
                            stage_env=env, allow_failure=True)
        junit_counts = validate_junit(junit, set(names))
        receipt["junit"] = {**junit_counts, "exit_code": test_result.returncode}
        save_receipt(receipt_path, receipt)
        raw = sorted(profiles.glob("*.profraw"))
        if not raw or any(p.stat().st_size == 0 for p in raw):
            raise WorkflowError("test run produced no nonempty LLVM profile")
        receipt["profiles"] = [{"path": str(p), "sha256": sha256(p), "size": p.stat().st_size} for p in raw]
        save_receipt(receipt_path, receipt)
        merged = out / "merged.profdata"
        stage("profile_merge", [args.llvm_profdata, "merge", "-sparse", "-o", str(merged), *map(str, raw)],
              out / "profile-merge.log")
        if not merged.is_file() or merged.stat().st_size == 0:
            raise WorkflowError("llvm-profdata produced no nonempty merged profile")
        summary = out / "coverage-summary.txt"
        stage("coverage_report", [args.llvm_cov, "report", str(binary), f"-instr-profile={merged}", str(prod_source)], summary)
        lines = out / "coverage-lines.txt"
        stage("coverage_lines", [args.llvm_cov, "show", str(binary), f"-instr-profile={merged}",
                                  str(prod_source), "--show-line-counts-or-regions"], lines)
        if not summary.stat().st_size or not lines.stat().st_size:
            raise WorkflowError("llvm-cov produced an empty source-scoped report")
        after = git_snapshot(root)
        hashes_after = file_snapshot({"test_source": test_source, "production_source": prod_source,
                                      "compile_commands": compile_commands})
        elf_final = sha256(binary)
        receipt.update({"tracked_after": after, "hashes_after": hashes_after,
                        "elf_after_tests_sha256": elf_final,
                        "profiles": [{"path": str(p), "sha256": sha256(p), "size": p.stat().st_size} for p in raw],
                        "merged_profile_sha256": sha256(merged),
                        "test_status": "failed" if test_result.returncode != 0 or any(junit_counts.values()) else "passed",
                        "test_exit_code": test_result.returncode,
                        "junit_counts": junit_counts,
                        "elapsed_seconds": round(time.perf_counter() - started, 3)})
        save_receipt(receipt_path, receipt)
        if after != tracked_before or hashes_after != {k: v for k, v in hashes_before.items() if k != "elf_before_build"}:
            raise WorkflowError("checkout/source/compile database changed during the workflow")
        if elf_final != hashes_after_build["elf"]:
            raise WorkflowError("test ELF changed after the build stage")
        if test_result.returncode != 0 or any(junit_counts.values()):
            raise WorkflowError(f"module tests failed after diagnostic coverage report: exit={test_result.returncode}, junit={junit_counts}")
        receipt["status"] = "passed"
        receipt["ended_utc"] = utc_now()
        save_receipt(receipt_path, receipt)
        return 0
    except Exception as exc:
        receipt["status"] = "failed"
        receipt["error"] = str(exc)
        receipt["ended_utc"] = utc_now()
        receipt["elapsed_seconds"] = round(time.perf_counter() - started, 3)
        try:
            receipt["tracked_after_failure"] = git_snapshot(root)
            receipt["hashes_after_failure"] = file_snapshot({"test_source": test_source,
                                                               "production_source": prod_source,
                                                               "compile_commands": compile_commands})
            if binary.is_file() and under(binary, build):
                receipt["elf_after_failure_sha256"] = sha256(binary)
            receipt["profiles_after_failure"] = [
                {"path": str(p), "sha256": sha256(p), "size": p.stat().st_size}
                for p in sorted(profiles.glob("*.profraw"))]
            merged = out / "merged.profdata"
            if merged.is_file():
                receipt["merged_profile_sha256"] = sha256(merged)
        except Exception as snapshot_error:
            receipt["snapshot_error"] = str(snapshot_error)
        save_receipt(receipt_path, receipt)
        print(f"module feedback failed: {exc}", file=sys.stderr)
        return 1


def main(argv: list[str] | None = None) -> int:
    try:
        return run(parse_args(argv))
    except (WorkflowError, OSError) as exc:
        print(f"module feedback failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
