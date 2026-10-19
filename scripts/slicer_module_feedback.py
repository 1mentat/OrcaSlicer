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
