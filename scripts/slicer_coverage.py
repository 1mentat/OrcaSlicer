#!/usr/bin/env python3
"""Run an LLVM source-coverage workload for the real libslic3r objects.

This is deliberately a small runner around llvm-profdata/llvm-cov.  CMake owns
which production sources are compiled (and writes a manifest); this file owns
the profile merge, report denominator, evidence, and gate.  It never invents
coverage for a source file that was not present in LLVM's export.

Typical invocation (after a coverage-configured build)::

    python scripts/slicer_coverage.py run \
      --object build/tests/libslic3r/libslic3r_tests \
      --manifest build/slicer-coverage/libslic3r.sources.txt \
      --test-command 'ctest --test-dir build/tests -C Release -LE "NotWorking|RequiresApp" --no-tests=error' \
      --out build/slicer-coverage

The command is argv based (``shell=False``); command strings are only split
with ``shlex`` so paths and arguments remain explicit and auditable.
"""

from __future__ import annotations

import argparse
import glob
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys
import time
from typing import Any, Iterable, Sequence


DEFAULT_THRESHOLD = 90.0
_TEST_COUNT_PATTERNS = (
    re.compile(r"out of\s+(\d+)\b", re.IGNORECASE),
    re.compile(r"\bof\s+(\d+)\s+tests?\b", re.IGNORECASE),
    re.compile(r"\b(\d+)\s+tests?\s+(?:passed|failed|skipped)\b", re.IGNORECASE),
    re.compile(r"test cases?:\s*(\d+)\b", re.IGNORECASE),
    re.compile(r"all tests passed\s*\((\d+)\s+assertions?", re.IGNORECASE),
)
_NO_EXECUTION_PATTERNS = (
    re.compile(r"no tests? (?:were )?found", re.IGNORECASE),
    re.compile(r"not run", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+(?:tests?|test cases?)\s+skipped\b", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+skipped\b", re.IGNORECASE),
)


class CoverageError(RuntimeError):
    """A coverage run cannot produce an honest report."""


def _which(name: str, explicit: str | None = None) -> str:
    candidate = explicit or shutil.which(name)
    if not candidate and sys.platform == "darwin" and shutil.which("xcrun"):
        found = _run(["xcrun", "--find", name])
        if found.returncode == 0 and found.stdout.strip():
            candidate = found.stdout.strip().splitlines()[-1]
    if not candidate:
        raise CoverageError(f"required LLVM tool not found: {name}")
    return os.path.abspath(candidate)


def _run(argv: Sequence[str], *, cwd: Path | None = None, env: dict[str, str] | None = None,
         timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    """Run a recorded argv without invoking a shell."""
    try:
        return subprocess.run(
            list(argv), cwd=str(cwd) if cwd else None, env=env,
            text=True, encoding="utf-8", errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=timeout, check=False,
        )
    except FileNotFoundError as exc:
        raise CoverageError(f"command not found: {argv[0]}") from exc
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        return subprocess.CompletedProcess(
            list(argv), 124, stdout, stderr + f"\n[command timed out after {timeout}s]\n",
        )


def split_command(command: str | Sequence[str]) -> list[str]:
    if isinstance(command, str):
        argv = shlex.split(command, posix=(os.name != "nt"))
    else:
        argv = [str(part) for part in command]
    if not argv:
        raise CoverageError("an empty test command cannot establish coverage evidence")
    return argv


def _expand_command(argv: Sequence[str], *, root: Path, build: Path, filter_name: str | None = None) -> list[str]:
    values = {"root": str(root), "build": str(build), "filter": filter_name or ""}
    # Literal replacement keeps JSON braces and other command arguments intact.
    return [part.replace("{root}", values["root"])
                .replace("{build}", values["build"])
                .replace("{filter}", values["filter"])
            for part in argv]


def test_evidence(output: str) -> int:
    """Return a discovered test count, or zero when no runner evidence exists.

    Requiring a count prevents a typo in a CTest/Catch filter from producing a
    green report with no executed tests.  CTest's ``--no-tests=error`` remains
    recommended; this check also protects callers that omit it.
    """
    if any(pattern.search(output) for pattern in _NO_EXECUTION_PATTERNS):
        # CTest reports a total even when a label matched only disabled or
        # unavailable tests.  Such a run must not create a green profile.
        executed = sum(int(match.group(1)) for match in re.finditer(
            r"\b([1-9]\d*)\s+tests?\s+(?:passed|failed)\b", output, re.IGNORECASE))
        return executed
    counts = [int(match.group(1)) for pattern in _TEST_COUNT_PATTERNS
              for match in pattern.finditer(output)]
    return max(counts, default=0)


def _path_key(path: str | Path) -> str:
    return os.path.normcase(os.path.realpath(os.fspath(path)))


def _under(path: Path, directory: Path) -> bool:
    try:
        return os.path.commonpath([_path_key(path), _path_key(directory)]) == _path_key(directory)
    except ValueError:
        return False


def _read_manifest(path: Path, root: Path) -> list[Path]:
    if not path.is_file():
        raise CoverageError(f"coverage source manifest does not exist: {path}")
    result: list[Path] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        value = raw.strip()
        if not value or value.startswith("#"):
            continue
        source = Path(value)
        if not source.is_absolute():
            source = root / source
        result.append(source.resolve())
    if not result:
        raise CoverageError(f"coverage source manifest is empty: {path}")
    return sorted(set(result), key=lambda value: str(value))


def _summary_count(summary: dict[str, Any], key: str) -> tuple[int, int]:
    value = summary.get(key, {})
    if not isinstance(value, dict):
        return 0, 0
    try:
        return int(value.get("covered", 0)), int(value.get("count", 0))
    except (TypeError, ValueError):
        return 0, 0


def _merge_summaries(files: Iterable[dict[str, Any]]) -> dict[str, dict[str, int]]:
    totals: dict[str, dict[str, int]] = {}
    for item in files:
        summary = item.get("summary", {})
        if not isinstance(summary, dict):
            continue
        for key in ("lines", "functions", "regions", "branches", "instantiations"):
            covered, count = _summary_count(summary, key)
            bucket = totals.setdefault(key, {"covered": 0, "count": 0})
            bucket["covered"] += covered
            bucket["count"] += count
    for value in totals.values():
        value["percent"] = (100.0 * value["covered"] / value["count"]
                             if value["count"] else 100.0)
    return totals


def _file_summary(item: dict[str, Any]) -> dict[str, dict[str, int | float]]:
    summary: dict[str, Any] = item.get("summary", {})
    result: dict[str, dict[str, int | float]] = {}
    if not isinstance(summary, dict):
        return result
    for key in ("lines", "functions", "regions", "branches", "instantiations"):
        covered, count = _summary_count(summary, key)
        result[key] = {
            "covered": covered,
            "count": count,
            "percent": 100.0 * covered / count if count else 100.0,
        }
    return result


def parse_export(export: dict[str, Any], *, source_root: Path,
                 manifest: Sequence[Path] = (), path_base: Path | None = None
                 ) -> tuple[dict[str, Any], list[str]]:
    """Filter LLVM JSON to production files and return (summary, missing).

    LLVM's JSON summary is the denominator.  A manifest is used only to detect
    production sources omitted from the instrumented link; missing entries are
    reported as an integrity error by the caller.
    """
    data = export.get("data", [])
    if not isinstance(data, list):
        raise CoverageError("llvm-cov export has no data array")
    selected: list[dict[str, Any]] = []
    seen: set[str] = set()
    for chunk in data:
        if not isinstance(chunk, dict):
            continue
        files = chunk.get("files", [])
        if not isinstance(files, list):
            continue
        for item in files:
            if not isinstance(item, dict) or not item.get("filename"):
                continue
            path = Path(str(item["filename"]))
            if not path.is_absolute() and path_base is not None:
                path = path_base / path
            path = path.resolve()
            if not _under(path, source_root):
                continue
            key = _path_key(path)
            if key in seen:
                continue
            item = dict(item)
            item["filename"] = str(path)
            selected.append(item)
            seen.add(key)
    if not selected:
        raise CoverageError(f"llvm-cov export contains no files under {source_root}")

    summary = _merge_summaries(selected)
    report = {
        "source_root": str(source_root.resolve()),
        "files": [
            {"filename": item["filename"], "summary": _file_summary(item)}
            for item in sorted(selected, key=lambda value: value["filename"])
        ],
        "summary": summary,
    }
    exported = {_path_key(item["filename"]) for item in selected}
    missing = [str(Path(path).resolve()) for path in manifest if _path_key(path) not in exported]
    return report, missing


def _percent(summary: dict[str, Any], key: str) -> float:
    value = summary.get(key, {})
    return float(value.get("percent", 100.0)) if isinstance(value, dict) else 100.0


def _meets_gate(summary: dict[str, Any], key: str, threshold: float) -> bool:
    # The gate is deliberately strict: a reported 90.00% does not satisfy a
    # "more than 90%" policy.  A missing summary is an integrity failure, not
    # an empty denominator that should silently pass.
    value = summary.get(key)
    if not isinstance(value, dict):
        return False
    try:
        count = int(value.get("count", 0))
    except (TypeError, ValueError):
        return False
    return count > 0 and _percent(summary, key) > threshold


def _sources_without_executable_mappings(report: dict[str, Any]) -> list[str]:
    """Return exported production files with no executable LLVM denominator."""
    result: list[str] = []
    for item in report.get("files", []):
        if not isinstance(item, dict):
            continue
        summaries = item.get("summary", {})
        if not isinstance(summaries, dict):
            continue
        counts = []
        for key in ("lines", "functions", "regions", "branches", "instantiations"):
            value = summaries.get(key, {})
            if isinstance(value, dict):
                try:
                    counts.append(int(value.get("count", 0)))
                except (TypeError, ValueError):
                    counts.append(0)
        if counts and not any(counts):
            result.append(str(item.get("filename", "")))
    return sorted(value for value in result if value)


def _profile_files(profile_dir: Path) -> list[Path]:
    return sorted(Path(value) for value in glob.glob(str(profile_dir / "*.profraw")))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_coverage(args: argparse.Namespace) -> int:
    if args.metrics is None:
        args.metrics = ["lines", "functions", "branches"]
    root = Path(args.root).resolve()
    out_arg = Path(args.out)
    out = (root / out_arg if not out_arg.is_absolute() else out_arg).resolve()
    out.mkdir(parents=True, exist_ok=True)
    source_root = (root / args.source_root).resolve() if not Path(args.source_root).is_absolute() else Path(args.source_root).resolve()
    llvm_cov = _which("llvm-cov", args.llvm_cov)
    llvm_profdata = _which("llvm-profdata", args.llvm_profdata)
    objects = [(root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
               for value in args.object]
    missing_objects = [str(value) for value in objects if not value.is_file()]
    if missing_objects:
        raise CoverageError("coverage object does not exist: " + ", ".join(missing_objects))
    manifest: list[Path] = []
    if args.manifest:
        for manifest_path in args.manifest:
            manifest_arg = Path(manifest_path)
            manifest_path = (root / manifest_arg if not manifest_arg.is_absolute() else manifest_arg).resolve()
            manifest.extend(_read_manifest(manifest_path, root))
        manifest = sorted(set(manifest), key=lambda value: str(value))

    command = split_command(args.test_command)
    command = _expand_command(command, root=root, build=(root / args.build_dir).resolve())
    if "{filter}" in args.test_command and not args.filter:
        raise CoverageError("test command contains {filter}, but --filter was not supplied")
    # CTest labels are the least surprising way to keep a coverage run focused.
    if args.filter and "{filter}" not in args.test_command and Path(command[0]).name == "ctest" and "-L" not in command:
        command.extend(["-L", args.filter])
    # CTest can launch thousands of short processes. LLVM's merge pool bounds
    # raw-profile storage to four files per binary and locks concurrent updates.
    # https://clang.llvm.org/docs/SourceBasedCodeCoverage.html
    profile_pattern = out / "profraw-%4m.profraw"
    for stale in _profile_files(out):
        stale.unlink()
    env = dict(os.environ)
    env["LLVM_PROFILE_FILE"] = str(profile_pattern)
    started = time.monotonic()
    result = _run(command, cwd=root, env=env, timeout=args.timeout)
    elapsed = time.monotonic() - started
    output = (result.stdout or "") + (result.stderr or "")
    evidence = test_evidence(output)
    run_record = {
        "argv": command,
        "profile_pattern": str(profile_pattern),
        "returncode": result.returncode,
        "timed_out": result.returncode == 124,
        "duration_seconds": elapsed,
        "test_evidence_count": evidence,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }
    profiles = _profile_files(out)
    if evidence <= 0 and not profiles:
        _write_json(out / "coverage-report.json", {
            "status": "test_failed_no_evidence" if result.returncode != 0 else "no_test_evidence",
            "tests_failed": result.returncode != 0,
            "run": run_record,
        })
        return 2
    if not profiles:
        _write_json(out / "coverage-report.json", {
            "status": "test_failed_no_profile" if result.returncode != 0 else "no_profile",
            "tests_failed": result.returncode != 0,
            "run": run_record,
            "objects": [str(value) for value in objects],
            "manifest": [str(Path(value).resolve()) for value in args.manifest] if args.manifest else [],
        })
        return 2

    merged = out / "merged.profdata"
    merge_argv = [llvm_profdata, "merge", "-sparse", "-o", str(merged), *map(str, profiles)]
    merge = _run(merge_argv, cwd=root, timeout=args.timeout)
    if merge.returncode != 0:
        _write_json(out / "coverage-report.json", {
            "status": "profile_merge_failed", "run": run_record,
            "merge": {"argv": merge_argv, "returncode": merge.returncode,
                       "stdout": merge.stdout, "stderr": merge.stderr},
        })
        return 2

    export_argv = [llvm_cov, "export", "-format=text", "-instr-profile", str(merged)]
    for obj in objects:
        export_argv.extend(["-object", str(obj)])
    export = _run(export_argv, cwd=root, timeout=args.timeout)
    if export.returncode != 0:
        _write_json(out / "coverage-report.json", {
            "status": "export_failed", "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode,
                       "stdout": export.stdout, "stderr": export.stderr},
        })
        return 2
    try:
        exported = json.loads(export.stdout)
    except json.JSONDecodeError as exc:
        _write_json(out / "coverage-report.json", {
            "status": "invalid_export", "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode,
                       "stdout": export.stdout, "stderr": export.stderr},
        })
        raise CoverageError(f"llvm-cov export was not JSON: {exc}") from exc
    try:
        report, missing = parse_export(exported, source_root=source_root,
                                       manifest=manifest, path_base=root)
    except CoverageError as exc:
        _write_json(out / "coverage-report.json", {
            "status": "invalid_export", "error": str(exc), "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode},
        })
        raise
    unmapped = _sources_without_executable_mappings(report)
    if result.returncode != 0:
        status = "test_failed"
    elif evidence <= 0:
        status = "no_test_evidence"
    elif missing or unmapped or any(
        key not in report["summary"] or int(report["summary"][key].get("count", 0)) <= 0
        for key in args.metrics
    ):
        status = "coverage_integrity_error"
    elif not all(_meets_gate(report["summary"], key, args.threshold) for key in args.metrics):
        status = "coverage_below_threshold"
    else:
        status = "pass"
    report.update({
        "status": status,
        "tests_failed": result.returncode != 0,
        "threshold": args.threshold,
        "metrics": args.metrics,
        "objects": [str(value) for value in objects],
        "profiles": [str(value) for value in profiles],
        "manifest": [str(Path(value).resolve()) for value in args.manifest] if args.manifest else [],
        "missing_manifest_sources": missing,
        "missing_compiled_source_mappings": missing,
        "sources_without_executable_mappings": unmapped,
        "run": run_record,
        "merge": {"argv": merge_argv, "returncode": merge.returncode},
        "export": {"argv": export_argv, "returncode": export.returncode},
        "tool_versions": {
            "llvm-cov": _run([llvm_cov, "--version"]).stdout.strip(),
            "llvm-profdata": _run([llvm_profdata, "--version"]).stdout.strip(),
        },
    })
    _write_json(out / "coverage-report.json", report)
    # Keep the raw export as evidence and for future report consumers.
    (out / "coverage-export.json").write_text(export.stdout, encoding="utf-8")
    if result.returncode != 0 or evidence <= 0:
        return 2
    if missing or unmapped or any(not _meets_gate(report["summary"], key, args.threshold) for key in args.metrics):
        return 1
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run", help="run tests, merge profiles, and enforce the coverage gate")
    run.add_argument("--root", default=".", help="repository root (default: current directory)")
    run.add_argument("--build-dir", default="build", help="build directory relative to root")
    run.add_argument("--source-root", default="src/libslic3r", help="production source root")
    run.add_argument("--object", action="append", required=True, help="instrumented test executable; repeatable")
    run.add_argument("--manifest", action="append",
                     help="CMake source manifest for an actual target; repeat for libslic3r_cgal")
    run.add_argument("--out", default="build/slicer-coverage", help="report/output directory")
    run.add_argument("--test-command", required=True, help="argv string; {root}, {build}, and {filter} expand")
    run.add_argument("--filter", help="CTest label used when the command has no {filter} placeholder")
    run.add_argument("--threshold", type=float, default=DEFAULT_THRESHOLD)
    run.add_argument("--metric", dest="metrics", action="append",
                     choices=("lines", "functions", "regions", "branches", "instantiations"),
                     default=None)
    run.add_argument("--timeout", type=float, default=1800.0)
    run.add_argument("--llvm-cov")
    run.add_argument("--llvm-profdata")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        return run_coverage(args) if args.action == "run" else 2
    except CoverageError as exc:
        if args.action == "run":
            try:
                out_arg = Path(args.out)
                out = ((Path(args.root).resolve() / out_arg) if not out_arg.is_absolute() else out_arg).resolve()
                _write_json(out / "coverage-report.json",
                            {"status": "error", "error": str(exc)})
            except OSError:
                pass
        print(f"slicer_coverage: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
