#!/usr/bin/env python3
"""Run a small, reproducible mutation gate against production libslic3r tests.

The harness validates a clean repository and a passing baseline, then creates
one disposable detached git worktree.  It applies one exact byte replacement at
a time, rebuilds and runs the selected tests, records the result, restores the
source bytes, and removes the worktree.  No mutation is ever applied to the
caller's checkout.

Commands are split with ``shlex`` and executed with ``shell=False``.  Use
``{root}``, ``{build}``, and ``{filter}`` placeholders in command strings.  A
CTest command automatically receives ``-L <filter>`` for a mutant that names a
test filter unless the command already has a label selector.

Example::

    python scripts/slicer_mutation.py run \
      --configure-command 'cmake -S {root} -B {build} -G Ninja -DCMAKE_BUILD_TYPE=Release' \
      --build-command 'cmake --build {build} --config Release --target fff_print_tests' \
      --test-command 'ctest --test-dir {build}/tests -C Release --no-tests=error'
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import time
from typing import Any, Sequence


MAX_OUTPUT = 64 * 1024
DEFAULT_MAX_MUTANTS = 5


class MutationError(RuntimeError):
    """The baseline or mutation run cannot be trusted."""


@dataclass(frozen=True)
class Mutant:
    id: str
    path: str
    old: str
    new: str
    description: str
    test_filter: str = ""


# These are intentionally few, exact, and tied to the Flow tests.  They are
# useful smoke mutants for the harness itself; teams can supply a checked-in
# JSON manifest when they want to grow the bounded list.
DEFAULT_MUTANTS = (
    Mutant("flow-default-width", "src/libslic3r/Flow.cpp", "1.125f", "1.0f",
           "wrong automatic perimeter width multiplier", "Flow"),
    Mutant("flow-spacing-constant", "src/libslic3r/Flow.cpp",
           "auto out = width - height * float(1. - 0.25 * PI);",
           "auto out = width - height * float(1. - 0.5 * PI);",
           "wrong rounded extrusion spacing constant", "Flow"),
    Mutant("flow-manual-width-branch", "src/libslic3r/Flow.cpp",
           "!width.percent  && width.value <= 0.", "!width.percent  && width.value < 0.",
           "zero width no longer uses the automatic fallback", "Flow"),
)


def split_command(command: str | Sequence[str]) -> list[str]:
    if isinstance(command, str):
        argv = shlex.split(command, posix=(os.name != "nt"))
    else:
        argv = [str(value) for value in command]
    if not argv:
        raise MutationError("empty build/test command")
    return argv


def _expand(argv: Sequence[str], *, root: Path, build: Path, test_filter: str = "") -> list[str]:
    values = {"root": str(root), "build": str(build), "filter": test_filter}
    return [value.replace("{root}", values["root"])
                 .replace("{build}", values["build"])
                 .replace("{filter}", values["filter"])
             for value in argv]


def _truncate(value: str) -> str:
    if len(value) <= MAX_OUTPUT:
        return value
    return value[:MAX_OUTPUT] + "\n...[output truncated]...\n"


@dataclass
class CommandResult:
    argv: list[str]
    returncode: int | None
    duration_seconds: float
    stdout: str
    stderr: str
    timed_out: bool = False

    def json(self) -> dict[str, Any]:
        result = asdict(self)
        result["stdout"] = _truncate(result["stdout"])
        result["stderr"] = _truncate(result["stderr"])
        return result


def run_command(argv: Sequence[str], *, cwd: Path, timeout: float) -> CommandResult:
    started = time.monotonic()
    try:
        result = subprocess.run(
            list(argv), cwd=str(cwd), shell=False, text=True, encoding="utf-8", errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout, check=False,
        )
        return CommandResult(list(argv), result.returncode, time.monotonic() - started,
                             result.stdout or "", result.stderr or "")
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        return CommandResult(list(argv), None, time.monotonic() - started, stdout, stderr, True)
    except FileNotFoundError as exc:
        return CommandResult(list(argv), 127, time.monotonic() - started, "", str(exc))


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


def test_evidence(output: str) -> int:
    """Count runner-reported tests; zero means a filter likely matched none."""
    if any(pattern.search(output) for pattern in _NO_EXECUTION_PATTERNS):
        return sum(int(match.group(1)) for match in re.finditer(
            r"\b([1-9]\d*)\s+tests?\s+(?:passed|failed)\b", output, re.IGNORECASE))
    values = [int(match.group(1)) for pattern in _TEST_COUNT_PATTERNS
              for match in pattern.finditer(output)]
    return max(values, default=0)


def _path_under(path: Path, root: Path) -> bool:
    try:
        return os.path.commonpath([str(path.resolve()), str(root.resolve())]) == str(root.resolve())
    except ValueError:
        return False


def _git(repo: Path, *args: str) -> CommandResult:
    return run_command(["git", *args], cwd=repo, timeout=120)


def validate_clean_repo(repo: Path) -> str:
    result = _git(repo, "status", "--porcelain", "--untracked-files=all")
    if result.returncode != 0:
        raise MutationError(f"cannot inspect repository status: {_truncate(result.stderr)}")
    if result.stdout.strip():
        raise MutationError("mutation baseline requires a clean checkout; git status is not empty")
    head = _git(repo, "rev-parse", "HEAD")
    if head.returncode != 0:
        raise MutationError(f"cannot resolve HEAD: {_truncate(head.stderr)}")
    return head.stdout.strip()


def _load_mutants(path: str | list[str] | None, root: Path | None = None) -> list[Mutant]:
    if not path:
        return list(DEFAULT_MUTANTS)
    if isinstance(path, list):
        return [mutant for manifest in path for mutant in _load_mutants(manifest, root)]
    manifest_arg = Path(path)
    manifest = ((root / manifest_arg) if root is not None and not manifest_arg.is_absolute()
                else manifest_arg).resolve()
    try:
        payload = json.loads(manifest.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise MutationError(f"cannot read mutant manifest {manifest}: {exc}") from exc
    entries = payload.get("mutants") if isinstance(payload, dict) else payload
    if not isinstance(entries, list):
        raise MutationError("mutant manifest must be a JSON list or {\"mutants\": [...]}")
    mutants: list[Mutant] = []
    for index, item in enumerate(entries):
        if not isinstance(item, dict):
            raise MutationError(f"mutant #{index + 1} is not an object")
        required = ("id", "path", "old", "new")
        if any(key not in item for key in required):
            raise MutationError(f"mutant #{index + 1} must include id/path/old/new")
        mutants.append(Mutant(
            str(item["id"]), str(item["path"]), str(item["old"]), str(item["new"]),
            str(item.get("description", "")), str(item.get("test_filter", "")),
        ))
    return mutants


def _apply(mutant: Mutant, root: Path) -> tuple[Path, bytes]:
    path = (root / mutant.path).resolve()
    if not _path_under(path, root):
        raise MutationError(f"mutant {mutant.id} escapes repository root: {mutant.path}")
    if not path.is_file():
        raise MutationError(f"mutant {mutant.id} source does not exist: {mutant.path}")
    original = path.read_bytes()
    old = mutant.old.encode("utf-8")
    new = mutant.new.encode("utf-8")
    if not old:
        raise MutationError(f"mutant {mutant.id} has an empty match")
    count = original.count(old)
    if count != 1:
        raise MutationError(f"mutant {mutant.id} expected one source match, found {count}: {mutant.path}")
    if old == new:
        raise MutationError(f"mutant {mutant.id} replacement is unchanged")
    path.write_bytes(original.replace(old, new, 1))
    return path, original


def _test_command(argv: Sequence[str], mutant: Mutant) -> list[str]:
    result = [value.replace("{filter}", mutant.test_filter) for value in argv]
    if mutant.test_filter and "{filter}" not in " ".join(argv):
        if Path(result[0]).name == "ctest" and "-L" not in result:
            result.extend(["-L", mutant.test_filter])
    return result


def _classify_build(result: CommandResult) -> str:
    if result.timed_out:
        return "build_timeout"
    if result.returncode is not None and result.returncode < 0:
        return "build_killed"
    return "build_error"


def _classify_test(result: CommandResult, evidence: int) -> str:
    if result.timed_out:
        return "test_timeout"
    if result.returncode is not None and result.returncode < 0:
        return "test_killed"
    if evidence <= 0:
        return "test_no_evidence"
    if result.returncode != 0:
        return "killed"
    return "survived"


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def run_mutation(args: argparse.Namespace) -> int:
    repo = Path(args.root).resolve()
    if not repo.is_dir():
        raise MutationError(f"repository root does not exist: {repo}")
    commit = validate_clean_repo(repo)
    mutants = _load_mutants(args.mutants, repo)
    if not mutants:
        raise MutationError("mutation manifest is empty")
    if len(mutants) > args.max_mutants:
        raise MutationError(f"{len(mutants)} mutants exceed --max-mutants={args.max_mutants}; keep the gate bounded")
    ids = [mutant.id for mutant in mutants]
    if len(set(ids)) != len(ids):
        raise MutationError("mutant ids must be unique")
    build_argv = split_command(args.build_command)
    test_argv = split_command(args.test_command)
    configure_argv = split_command(args.configure_command) if args.configure_command else None
    out_arg = Path(args.out)
    out = ((repo / out_arg) if not out_arg.is_absolute() else out_arg).resolve()
    work_parent_arg = Path(args.work_root) if args.work_root else None
    work_parent = ((repo / work_parent_arg) if work_parent_arg is not None and not work_parent_arg.is_absolute()
                   else work_parent_arg)
    if work_parent is not None:
        work_parent = work_parent.resolve()
    if work_parent:
        work_parent.mkdir(parents=True, exist_ok=True)

    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "infrastructure_error",
        "git": {"commit": commit, "root": str(repo)},
        "limits": {"max_mutants": args.max_mutants, "timeout_seconds": args.timeout},
        "commands": {"configure": configure_argv, "build": build_argv, "test": test_argv},
        "mutants": [],
    }

    with tempfile.TemporaryDirectory(prefix="slicer-mutation-", dir=str(work_parent) if work_parent else None) as parent:
        worktree = Path(parent) / "repo"
        add = _git(repo, "worktree", "add", "--detach", str(worktree), commit)
        report["worktree"] = {"path": str(worktree), "add": add.json()}
        if add.returncode != 0:
            raise MutationError(f"cannot create disposable worktree: {_truncate(add.stderr)}")
        try:
            build_dir = (worktree / args.build_dir).resolve()
            if configure_argv:
                configure = run_command(_expand(configure_argv, root=worktree, build=build_dir),
                                        cwd=worktree, timeout=args.timeout)
                report["configure"] = configure.json()
                if configure.timed_out or configure.returncode != 0:
                    report["status"] = "configure_timeout" if configure.timed_out else "configure_error"
                    _write_report(out / "mutation-report.json", report)
                    return 2
            build = _expand(build_argv, root=worktree, build=(worktree / args.build_dir).resolve())
            baseline_build = run_command(build, cwd=worktree, timeout=args.timeout)
            report["baseline_build"] = baseline_build.json()
            if baseline_build.timed_out or baseline_build.returncode != 0:
                report["status"] = _classify_build(baseline_build)
                _write_report(out / "mutation-report.json", report)
                return 2
            # Validate each manifest filter against the unmutated build.  A
            # broad suite baseline can fail for unrelated reasons while still
            # hiding whether the selected test actually ran.
            filters = sorted(set(mutant.test_filter for mutant in mutants))
            baseline_tests: list[dict[str, Any]] = []
            for test_filter in filters:
                baseline_mutant = Mutant("baseline", "", "x", "y", "", test_filter)
                selected = _test_command(test_argv, baseline_mutant)
                baseline_test = run_command(_expand(selected, root=worktree,
                                                     build=(worktree / args.build_dir).resolve()),
                                            cwd=worktree, timeout=args.timeout)
                evidence = test_evidence(baseline_test.stdout + baseline_test.stderr)
                baseline_tests.append({
                    "filter": test_filter,
                    "test_evidence": evidence,
                    "result": baseline_test.json(),
                })
                if baseline_test.timed_out or baseline_test.returncode != 0 or evidence <= 0:
                    report["baseline_tests"] = baseline_tests
                    report["status"] = ("baseline_test_timeout" if baseline_test.timed_out else
                                         "baseline_test_failed" if baseline_test.returncode != 0 else
                                         "baseline_no_evidence")
                    _write_report(out / "mutation-report.json", report)
                    return 2
            report["baseline_tests"] = baseline_tests

            for mutant in mutants:
                entry: dict[str, Any] = {
                    "id": mutant.id, "path": mutant.path, "description": mutant.description,
                    "test_filter": mutant.test_filter,
                    "old": mutant.old, "new": mutant.new,
                    "old_sha256": hashlib.sha256(mutant.old.encode("utf-8")).hexdigest(),
                    "new_sha256": hashlib.sha256(mutant.new.encode("utf-8")).hexdigest(),
                }
                path: Path | None = None
                original: bytes | None = None
                try:
                    path, original = _apply(mutant, worktree)
                    entry["source_match"] = "exactly_one"
                    mutant_build = run_command(build, cwd=worktree, timeout=args.timeout)
                    entry["build"] = mutant_build.json()
                    if mutant_build.timed_out or mutant_build.returncode != 0:
                        entry["status"] = _classify_build(mutant_build)
                        report["mutants"].append(entry)
                        continue
                    selected_test = _test_command(test_argv, mutant)
                    mutant_test = run_command(_expand(selected_test, root=worktree,
                                                       build=(worktree / args.build_dir).resolve()),
                                              cwd=worktree, timeout=args.timeout)
                    mutant_evidence = test_evidence(mutant_test.stdout + mutant_test.stderr)
                    entry["test"] = mutant_test.json()
                    entry["test_evidence"] = mutant_evidence
                    entry["status"] = _classify_test(mutant_test, mutant_evidence)
                    report["mutants"].append(entry)
                except MutationError as exc:
                    entry["status"] = "mutation_error"
                    entry["error"] = str(exc)
                    report["mutants"].append(entry)
                finally:
                    if path is not None and original is not None:
                        path.write_bytes(original)
                        if path.read_bytes() != original:
                            raise MutationError(f"failed to restore mutant source: {path}")
        finally:
            remove = _git(repo, "worktree", "remove", "--force", str(worktree))
            report["worktree"]["remove"] = remove.json()
            # Also persist early baseline failures after the worktree cleanup,
            # so the report contains the exact remove command and outcome.
            _write_report(out / "mutation-report.json", report)
    statuses = [entry.get("status") for entry in report["mutants"]]
    report["counts"] = {status: statuses.count(status) for status in sorted(set(statuses))}
    if any(status == "survived" for status in statuses):
        report["status"] = "survivors"
        exit_code = 1
    elif any(status != "killed" for status in statuses):
        report["status"] = "infrastructure_error"
        exit_code = 2
    else:
        report["status"] = "pass"
        exit_code = 0
    _write_report(out / "mutation-report.json", report)
    return exit_code


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    defaults = sub.add_parser("list-defaults", help="print the bounded built-in mutants as JSON")
    defaults.set_defaults(list_defaults=True)
    run = sub.add_parser("run", help="validate baseline and run bounded mutants")
    run.add_argument("--root", default=".")
    run.add_argument("--build-dir", default="build")
    run.add_argument("--configure-command", help="optional CMake configure argv for the fresh worktree")
    run.add_argument("--build-command", required=True)
    run.add_argument("--test-command", required=True)
    run.add_argument("--mutants", action="append",
                     help="JSON mutant manifest; repeat to share one build across manifests (default: three Flow smoke mutants)")
    run.add_argument("--max-mutants", type=int, default=DEFAULT_MAX_MUTANTS)
    run.add_argument("--out", default="build/slicer-mutation")
    run.add_argument("--work-root", help="parent directory for disposable worktree")
    run.add_argument("--timeout", type=float, default=1800.0)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.action == "list-defaults":
        print(json.dumps([asdict(item) for item in DEFAULT_MUTANTS], indent=2))
        return 0
    try:
        return run_mutation(args)
    except MutationError as exc:
        # Keep an actionable machine-readable record even when validation
        # fails before the disposable worktree can be created (for example,
        # a dirty checkout or an empty/ambiguous source match).
        try:
            out_arg = Path(args.out)
            root = Path(args.root).resolve()
            out = (root / out_arg if not out_arg.is_absolute() else out_arg).resolve()
            report_path = out / "mutation-report.json"
            if not report_path.exists():
                _write_report(report_path, {
                    "schema_version": 1,
                    "status": "infrastructure_error",
                    "error": str(exc),
                    "git": {"root": str(root)},
                })
        except OSError:
            pass
        print(f"slicer_mutation: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
