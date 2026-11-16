#!/usr/bin/env python3
"""Prove CTest discovery of one focused target against its control baseline.

Reconfigures an existing build directory with focused CTest discovery on, links
only the focused target (its post-build step registers the tests), then checks
the registered ``<prefix>`` names against the control inventory and runs exactly
those names. Prints one ``FOCUSED_CONTROL_SUMMARY`` line and exits non-zero on
any missing, duplicate, wrongly-owned, failed, or unexecuted test.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))
import slicer_mutation as mutation  # noqa: E402
import slicer_process  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
OPTION = "-DSLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY=ON"
TAIL_BYTES = 3000


def run(cmd: Sequence[str], timeout: float, log: Path, cwd: Path) -> int:
    """Run cmd in its own process group with output in ``log``; 124 on timeout, 127 if unrunnable."""
    with open(log, "wb") as handle:
        try:
            return slicer_process.run_captured(
                cmd, cwd=cwd, timeout=timeout, output_file=handle).returncode
        except subprocess.TimeoutExpired:
            handle.write(("\n[timed out after %ss]\n" % timeout).encode())
            return 124
        except OSError as error:
            handle.write(("cannot run: %s\n" % error).encode())
            return 127


def tail(log: Path, size: int = TAIL_BYTES) -> str:
    try:
        with open(log, "rb") as handle:
            handle.seek(max(0, log.stat().st_size - size))
            return handle.read().decode("utf-8", errors="replace")
    except OSError as error:
        return "(no log: %s)" % error


def find_executable(build_dir: Path, target: str) -> Path | None:
    tests = build_dir / "tests"
    found = [p for name in (target, target + ".exe") for p in tests.rglob(name)
             if p.is_file() and not p.is_symlink()]
    return found[0] if len(found) == 1 else None


def evaluate(discovery: Any, expected: Sequence[str], prefix: str, build_dir: Path,
             executable: Path) -> dict[str, Any]:
    """Pure checks of ctest json-v1 output; returns counts, selected names and problems."""
    result: dict[str, Any] = {"discovered": 0, "selected": [], "missing": [], "duplicates": [],
                              "extra": [], "problems": []}
    raw = discovery.get("tests") if isinstance(discovery, dict) else None
    if not isinstance(raw, list):
        result["problems"].append("ctest JSON has no tests array")
        return result
    names = [t.get("name") for t in raw if isinstance(t, dict)]
    if len(names) != len(raw) or not all(isinstance(n, str) and n for n in names):
        result["problems"].append("ctest JSON contains a test without a name")
        return result
    result["discovered"] = len(names)
    result["duplicates"] = sorted({n for n in names if names.count(n) > 1})
    result["missing"] = sorted(set(expected) - set(names))
    result["extra"] = sorted(set(names) - set(expected))
    outside = sorted(n for n in names if not n.startswith(prefix))
    if outside:
        result["problems"].append("discovered names outside prefix %r: %s" % (prefix, outside))
    if result["duplicates"]:
        result["problems"].append("duplicate discovered names: %s" % result["duplicates"])
    if result["missing"]:
        result["problems"].append("control names not discovered: %s" % result["missing"])
    if result["problems"]:
        return result
    try:
        tests = mutation._ctest_tests(discovery)
        selected = mutation._selected_ctest_names(
            tests, control_regex=mutation._ctest_exact_names_regex(expected), exclude_label_regex="")
        mutation._verify_ctest_executables(tests, set(names), build_dir, executable)
    except mutation.MutationError as error:
        result["problems"].append(str(error))
        return result
    result["selected"] = selected
    if sorted(selected) != sorted(expected):
        result["problems"].append("selected names differ from the control inventory")
    return result


def check(args: argparse.Namespace, scratch: Path) -> tuple[int, dict[str, Any]]:
    stats: dict[str, Any] = {"expected": 0, "discovered": 0, "selected": 0, "executed": 0,
                             "passed": 0, "failed": 0, "missing": 0, "duplicates": 0}
    build_dir = Path(args.build_dir).resolve()
    root = Path(args.root).resolve()
    try:
        expected, _ = mutation._expected_control_names(args.control, root)
    except mutation.MutationError as error:
        print("FAILED control inventory: %s" % error)
        return 1, stats
    stats["expected"] = len(expected)

    def step(name: str, cmd: Sequence[str], timeout: float) -> bool:
        log = scratch / (name + ".log")
        code = run(cmd, timeout, log, root)
        if code:
            print("FAILED %s (exit %d); last output:\n%s" % (name, code, tail(log)))
        return code == 0

    if not step("configure", [args.cmake, "-S", str(root), "-B", str(build_dir), OPTION],
                args.configure_timeout):
        return 1, stats
    if not step("build", [args.cmake, "--build", str(build_dir), "--config", args.config,
                          "--target", args.target, "-j", str(args.jobs)], args.build_timeout):
        return 1, stats
    executable = find_executable(build_dir, args.target)
    if executable is None:
        print("FAILED expected exactly one regular %s executable under %s/tests" % (
            args.target, build_dir))
        return 1, stats

    list_log = scratch / "list.log"
    discovery_cmd = [args.ctest, "--test-dir", str(build_dir), "-C", args.config,
                     "--show-only=json-v1", "-R", "^" + args.prefix]
    try:
        completed = slicer_process.run_captured(discovery_cmd, cwd=root, timeout=args.list_timeout)
        output, code = completed.stdout or "", completed.returncode
        errors = completed.stderr or ""
    except subprocess.TimeoutExpired:
        output, errors, code = "[timed out after %ss]" % args.list_timeout, "", 124
    except OSError as error:
        output, errors, code = "cannot run: %s" % error, "", 127
    list_log.write_text(output + errors, encoding="utf-8")
    try:
        if code:
            raise ValueError("ctest --show-only exit %d" % code)
        discovery = json.loads(output[output.index("{"):])
    except ValueError as error:
        print("FAILED discovery (%s); last output:\n%s" % (error, tail(list_log)))
        return 1, stats
    verdict = evaluate(discovery, expected, args.prefix, build_dir, executable)
    stats.update(discovered=verdict["discovered"], selected=len(verdict["selected"]),
                 missing=len(verdict["missing"]), duplicates=len(verdict["duplicates"]))
    if verdict["extra"]:
        print("note: %d discovered names not in the control (tolerated): %s" % (
            len(verdict["extra"]), verdict["extra"]))
    if verdict["problems"]:
        print("FAILED discovery:\n  " + "\n  ".join(verdict["problems"]))
        return 1, stats

    junit = scratch / "control.junit.xml"
    run_log = scratch / "run.log"
    code = run([args.ctest, "--test-dir", str(build_dir), "-C", args.config,
                "-R", mutation._ctest_exact_names_regex(expected), "--no-tests=error",
                "-j", str(args.jobs), "--timeout", str(args.test_timeout),
                "--output-on-failure", "--output-junit", str(junit)],
               args.run_timeout, run_log, root)
    print("ctest control run exit %d; last output:\n%s" % (code, tail(run_log)))
    evidence = mutation._validate_target_junit(junit, expected) if junit.is_file() else {
        "usable": False, "errors": ["JUnit file was not created"], "cases": {}}
    cases = evidence.get("cases", {})
    stats["executed"] = len(cases)
    stats["passed"] = sum(status == "passed" for status in cases.values())
    stats["failed"] = stats["executed"] - stats["passed"]
    problems = list(evidence.get("errors", []))
    if code:
        problems.append("ctest exit %d" % code)
    if stats["executed"] != len(expected) or stats["passed"] != len(expected):
        problems.append("executed=%d passed=%d, expected all %d to pass" % (
            stats["executed"], stats["passed"], len(expected)))
    if problems:
        print("FAILED control run:\n  " + "\n  ".join(dict.fromkeys(problems)))
        return 1, stats
    return 0, stats


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--root", default=str(ROOT))
    p.add_argument("--build-dir", default="build")
    p.add_argument("--config", default="Release")
    p.add_argument("--target", default="fff_print_wipe_tower_tests")
    p.add_argument("--prefix", default="FocusedWipeTower::")
    p.add_argument("--control", default="tests/controls/wipe_tower.json")
    p.add_argument("--jobs", type=int, default=2)
    p.add_argument("--cmake", default="cmake")
    p.add_argument("--ctest", default="ctest")
    p.add_argument("--configure-timeout", type=int, default=300)
    p.add_argument("--build-timeout", type=int, default=900)
    p.add_argument("--list-timeout", type=int, default=60)
    p.add_argument("--test-timeout", type=int, default=300)
    p.add_argument("--run-timeout", type=int, default=900)
    args = p.parse_args(argv)
    started = time.monotonic()
    with tempfile.TemporaryDirectory(prefix="focused-control-") as scratch:
        code, stats = check(args, Path(scratch))
    print("FOCUSED_CONTROL_SUMMARY target=%s expected=%d discovered=%d selected=%d executed=%d "
          "passed=%d failed=%d missing=%d duplicates=%d status=%s elapsed=%.0fs" % (
              args.target, stats["expected"], stats["discovered"], stats["selected"],
              stats["executed"], stats["passed"], stats["failed"], stats["missing"],
              stats["duplicates"], "ok" if code == 0 else "failed", time.monotonic() - started))
    return code


if __name__ == "__main__":
    sys.exit(main())
