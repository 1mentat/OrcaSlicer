#!/usr/bin/env python3
"""Validate a testing harness against exact performance PR heads.

The runner reads ``build/evidence/open-prs.json``, creates one disposable
detached worktree, and reuses that worktree and its native CMake build for the
baseline and every selected PR head.  The testing harness is the binary diff
between ``--harness-base`` and ``--harness-ref``; it is applied to each PR head
with ``git apply --3way --index``.  A conflicted overlay is recorded as a
conflict and is never guessed at or auto-resolved.

The caller's checkout is read only.  The temporary worktree is reset between
heads and removed in a ``finally`` block.  Every command, tested commit/tree,
duration, log path, JUnit path, and status is written to ``report.json``.

Example::

    python scripts/slicer_pr_validation.py \
      --snapshot build/evidence/open-prs.json \
      --harness-base main \
      --harness-ref codex/slicer-confidence-tests \
      --deps-prefix /path/to/deps/install \
      --output-dir build/evidence/pr-validation \
      --jobs 8
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable, Sequence
from xml.etree import ElementTree


from slicer_process import run_captured

MAX_OUTPUT = 64 * 1024
DEFAULT_TARGETS = (
    "libslic3r_tests",
    "fff_print_tests",
    "sla_print_tests",
    "libnest2d_tests",
    "filament_group_tests",
)
DEFAULT_EXCLUDED_LABELS = "NotWorking|RequiresApp"
DEFAULT_TIMEOUT = 3600.0
_HEX40 = re.compile(r"^[0-9a-f]{40}$")


class ValidationError(RuntimeError):
    """The validation run cannot produce an honest result."""


@dataclass
class CommandResult:
    argv: list[str]
    returncode: int | None
    duration_seconds: float
    stdout: str
    stderr: str
    timed_out: bool = False

    def json(self) -> dict[str, Any]:
        value = asdict(self)
        value["stdout"] = _truncate(value["stdout"])
        value["stderr"] = _truncate(value["stderr"])
        return value


def _truncate(value: str) -> str:
    if len(value) <= MAX_OUTPUT:
        return value
    return value[:MAX_OUTPUT] + "\n...[output truncated]...\n"


def run_command(argv: Sequence[str], *, cwd: Path, timeout: float = DEFAULT_TIMEOUT,
                input_data: str | bytes | None = None) -> CommandResult:
    """Run one recorded argv without a shell."""
    started = time.monotonic()
    try:
        completed = run_captured(argv, cwd=cwd, input_data=input_data, timeout=timeout)
        stdout = completed.stdout or ""
        stderr = completed.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        return CommandResult(list(argv), completed.returncode, time.monotonic() - started,
                             stdout, stderr)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        return CommandResult(list(argv), None, time.monotonic() - started,
                             stdout, stderr, True)
    except FileNotFoundError as exc:
        return CommandResult(list(argv), 127, time.monotonic() - started, "", str(exc))


def _git(repo: Path, *args: str, timeout: float = 120.0) -> CommandResult:
    return run_command(["git", *args], cwd=repo, timeout=timeout)


def _write_log(path: Path, result: CommandResult) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(result.stdout + ("\n" if result.stdout and result.stderr else "") +
                    result.stderr, encoding="utf-8", errors="replace")


def _write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _command_entry(result: CommandResult, log_path: Path) -> dict[str, Any]:
    value = result.json()
    value["log"] = str(log_path)
    return value


def test_evidence(output: str) -> int:
    """Extract a runner count; zero means no trustworthy tests ran."""
    patterns = (
        re.compile(r"out of\s+(\d+)\b", re.IGNORECASE),
        re.compile(r"\bof\s+(\d+)\s+tests?\b", re.IGNORECASE),
        re.compile(r"\b(\d+)\s+tests?\s+(?:passed|failed|skipped)\b", re.IGNORECASE),
        re.compile(r"test cases?:\s*(\d+)\b", re.IGNORECASE),
        re.compile(r"all tests passed\s*\((\d+)\s+assertions?", re.IGNORECASE),
    )
    values = [int(match.group(1)) for pattern in patterns for match in pattern.finditer(output)]
    return max(values, default=0)


def classify_test_result(result: CommandResult, evidence: int, *, skipped: int = 0,
                         no_tests: bool = False) -> str:
    if result.timed_out:
        return "test_timeout"
    if no_tests:
        return "no_tests"
    if result.returncode != 0:
        return "test_failed"
    if skipped > 0:
        return "test_skipped"
    if evidence <= 0:
        return "no_tests"
    return "pass"


def classify_test(result: CommandResult, evidence: int) -> str:
    """Classify a result when no separate JUnit skipped count is available."""
    return classify_test_result(result, evidence)


def no_test_evidence(output: str) -> bool:
    return bool(re.search(r"\bno\s+tests?(?:\s+were)?\s+found\b|\bno\s+test\s+cases?\b",
                          output, re.IGNORECASE))


def skipped_test_evidence(output: str, junit: Path | None = None) -> int:
    """Return the number of skipped/not-run tests reported by the harness."""
    patterns = (
        re.compile(r"\bskipped\s*[:=]\s*(\d+)\b", re.IGNORECASE),
        re.compile(r"\b(\d+)\s+tests?\s+(?:skipped|not\s+run)\b", re.IGNORECASE),
        re.compile(r"\b(\d+)\s+(?:skipped|not\s+run)\s+tests?\b", re.IGNORECASE),
    )
    count = max((int(match.group(1)) for pattern in patterns
                 for match in pattern.finditer(output)), default=0)
    if junit is not None and junit.is_file():
        try:
            root = ElementTree.parse(junit).getroot()
            for suite in root.iter():
                if suite.tag.rsplit("}", 1)[-1] not in {"testsuite", "testsuites"}:
                    continue
                try:
                    count = max(count, int(suite.attrib.get("skipped", "0")))
                except ValueError:
                    continue
        except (ElementTree.ParseError, OSError):
            # A partial JUnit file should not hide command output or a failure.
            pass
    return count


def classify_command(result: CommandResult, phase: str) -> str:
    if result.timed_out:
        return f"{phase}_timeout"
    return "pass" if result.returncode == 0 else f"{phase}_failed"


def parse_numbers(values: Sequence[Any] | None) -> list[int] | None:
    """Parse repeated, space-separated, or comma-separated PR numbers."""
    if not values:
        return None
    numbers: list[int] = []
    for value_group in values:
        group = value_group if isinstance(value_group, (list, tuple)) else [value_group]
        for value in group:
            for part in str(value).split(","):
                part = part.strip()
                if not part:
                    continue
                try:
                    number = int(part)
                except ValueError as exc:
                    raise ValidationError(f"invalid PR number: {part}") from exc
                if number <= 0:
                    raise ValidationError(f"PR number must be positive: {number}")
                numbers.append(number)
    if not numbers:
        raise ValidationError("--numbers did not contain a PR number")
    if len(set(numbers)) != len(numbers):
        raise ValidationError("--numbers contains duplicates")
    return numbers


def load_snapshot(path: Path) -> list[dict[str, Any]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise ValidationError(f"cannot read snapshot {path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValidationError(f"snapshot is not valid JSON: {path}: {exc}") from exc
    if not isinstance(payload, list) or not payload:
        raise ValidationError("snapshot must be a non-empty JSON list")
    records: list[dict[str, Any]] = []
    numbers: set[int] = set()
    for index, item in enumerate(payload):
        if not isinstance(item, dict):
            raise ValidationError(f"snapshot entry #{index + 1} is not an object")
        for key in ("number", "headRefOid"):
            if key not in item:
                raise ValidationError(f"snapshot entry #{index + 1} lacks {key}")
        try:
            number = int(item["number"])
        except (TypeError, ValueError) as exc:
            raise ValidationError(f"snapshot entry #{index + 1} has invalid number") from exc
        if number <= 0 or number in numbers:
            raise ValidationError(f"snapshot PR number is invalid or duplicated: {number}")
        oid = str(item["headRefOid"])
        if not _HEX40.fullmatch(oid):
            raise ValidationError(f"snapshot PR #{number} has a non-exact head OID: {oid}")
        records.append(dict(item, number=number, headRefOid=oid))
        numbers.add(number)
    return records


def select_snapshot(records: Sequence[dict[str, Any]], numbers: list[int] | None) -> list[dict[str, Any]]:
    if numbers is None:
        return [dict(item) for item in records]
    by_number = {int(item["number"]): item for item in records}
    missing = [number for number in numbers if number not in by_number]
    if missing:
        raise ValidationError("requested PR numbers are absent from snapshot: " +
                              ", ".join(str(number) for number in missing))
    return [dict(by_number[number]) for number in numbers]


def resolve_commit(repo: Path, ref: str) -> tuple[str | None, CommandResult]:
    result = _git(repo, "rev-parse", "--verify", f"{ref}^{{commit}}")
    value = result.stdout.strip()
    if result.returncode != 0 or not _HEX40.fullmatch(value):
        return None, result
    return value, result


def _tree_sha(repo: Path) -> tuple[str | None, CommandResult]:
    result = _git(repo, "write-tree")
    value = result.stdout.strip()
    if result.returncode != 0 or not _HEX40.fullmatch(value):
        return None, result
    return value, result


def _status(repo: Path) -> str:
    result = _git(repo, "status", "--short", "--untracked-files=all")
    return result.stdout if result.returncode == 0 else f"status failed: {result.stderr}"


def _resolve_uv(value: str | None) -> str:
    if value:
        return str(Path(value).expanduser().resolve())
    found = shutil.which("uv")
    return found or "uv"


def _format_args(args: Iterable[str], *, root: Path, build: Path, deps_prefix: str,
                 uv: str, jobs: int) -> list[str]:
    values = {
        "root": str(root), "build": str(build), "deps_prefix": deps_prefix,
        "uv": uv, "jobs": str(jobs),
    }
    return [value.format_map(values) for value in args]


def _default_configure(root: Path, build: Path, deps_prefix: str, uv: str) -> list[str]:
    return [
        "cmake", "-S", str(root), "-B", str(build), "-G", "Ninja",
        "-DCMAKE_BUILD_TYPE=Release", "-DSLIC3R_GUI=OFF", "-DBUILD_TESTS=ON",
        "-DSLIC3R_CAD=ON", f"-DCMAKE_PREFIX_PATH={deps_prefix}",
        f"-DORCA_BUNDLED_UV_EXECUTABLE={uv}",
    ]


def _default_build(root: Path, build: Path, jobs: int, targets: Sequence[str]) -> list[str]:
    return ["cmake", "--build", str(build), "--config", "Release", "--parallel", str(jobs),
            "--target", *targets]


def _default_test(build: Path, jobs: int, junit: Path, excluded_labels: str) -> list[str]:
    return ["ctest", "--test-dir", str(build / "tests"), "-C", "Release",
            "--output-on-failure", "--output-junit", str(junit), "--no-tests=error",
            "--parallel", str(jobs), "-LE", excluded_labels]


def _resolve_build_path(worktree: Path, build_dir: str) -> Path:
    """Resolve a build directory and keep it inside the disposable worktree."""
    root = worktree.resolve()
    candidate = Path(build_dir).expanduser()
    build = (candidate if candidate.is_absolute() else root / candidate).resolve()
    try:
        build.relative_to(root)
    except ValueError as exc:
        raise ValidationError(f"--build-dir must be inside the validation worktree: {build_dir}") from exc
    return build


def _git_diff(repo: Path, base: str, ref: str) -> tuple[bytes | None, CommandResult]:
    started = time.monotonic()
    try:
        result = subprocess.run(
            ["git", "diff", "--binary", "--full-index", base, ref], cwd=str(repo),
            shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=120, check=False,
        )
        stdout = result.stdout or b""
        stderr = (result.stderr or b"").decode("utf-8", "replace")
        command = CommandResult(["git", "diff", "--binary", "--full-index", base, ref],
                                result.returncode, time.monotonic() - started,
                                stdout.decode("utf-8", "replace"), stderr)
        return stdout if result.returncode == 0 else None, command
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        if isinstance(exc, subprocess.TimeoutExpired):
            result = CommandResult(["git", "diff", "--binary", "--full-index", base, ref],
                                   None, time.monotonic() - started, "", str(exc), True)
        else:
            result = CommandResult(["git", "diff", "--binary", "--full-index", base, ref],
                                   127, time.monotonic() - started, "", str(exc))
        return None, result


def _reset_worktree(worktree: Path, commit: str) -> list[dict[str, Any]]:
    operations: list[dict[str, Any]] = []
    reset = _git(worktree, "reset", "--hard", commit)
    operations.append(reset.json())
    if reset.returncode == 0:
        clean = _git(worktree, "clean", "-fd")
        operations.append(clean.json())
    return operations


def _run_phase(argv: Sequence[str], *, cwd: Path, timeout: float, log: Path) -> CommandResult:
    result = run_command(argv, cwd=cwd, timeout=timeout)
    _write_log(log, result)
    return result


def _run_tree(worktree: Path, root_output: Path, name: str, args: argparse.Namespace,
              configure_args: Sequence[str], build_args: Sequence[str],
              *, commit_sha: str, tree_sha: str | None) -> dict[str, Any]:
    run_dir = root_output / name
    run_dir.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    entry: dict[str, Any] = {
        "name": name,
        "commit_sha": commit_sha,
        "tree_sha": tree_sha,
        "tested_tree_sha": tree_sha,
        "status": "infrastructure_error",
        "commands": {},
    }

    def finish(status: str) -> dict[str, Any]:
        entry["status"] = status
        entry["elapsed_seconds"] = time.monotonic() - started
        return entry

    configure = _run_phase(configure_args, cwd=worktree, timeout=args.timeout,
                           log=run_dir / "configure.log")
    entry["commands"]["configure"] = _command_entry(configure, run_dir / "configure.log")
    if configure.returncode != 0 or configure.timed_out:
        return finish(classify_command(configure, "configure"))

    build = _run_phase(build_args, cwd=worktree, timeout=args.timeout,
                       log=run_dir / "build.log")
    entry["commands"]["build"] = _command_entry(build, run_dir / "build.log")
    if build.returncode != 0 or build.timed_out:
        return finish(classify_command(build, "build"))

    junit = run_dir / "tests.junit.xml"
    build_path = _resolve_build_path(worktree, args.build_dir)
    test_args = _default_test(build_path, args.jobs, junit, args.exclude_labels)
    test_args = _format_args(test_args, root=worktree, build=build_path,
                             deps_prefix=args.deps_prefix, uv=args.uv, jobs=args.jobs)
    entry["commands"]["test"] = {"argv": test_args, "junit": str(junit),
                                  "log": str(run_dir / "test.log")}
    test = _run_phase(test_args, cwd=worktree, timeout=args.timeout,
                      log=run_dir / "test.log")
    entry["commands"]["test"].update(test.json())
    test_output = test.stdout + test.stderr
    entry["test_evidence"] = test_evidence(test_output)
    entry["no_test_evidence"] = no_test_evidence(test_output)
    entry["skipped_test_evidence"] = skipped_test_evidence(test_output, junit)
    entry["junit_exists"] = junit.is_file()
    return finish(classify_test_result(test, entry["test_evidence"],
                                       skipped=entry["skipped_test_evidence"],
                                       no_tests=entry["no_test_evidence"]))


def _selected_statuses(report: dict[str, Any]) -> list[str]:
    return [str(item.get("status", "infrastructure_error")) for item in report.get("heads", [])]


def _overall_status(report: dict[str, Any]) -> tuple[str, int]:
    baseline = report.get("baseline", {})
    baseline_status = baseline.get("status")
    if baseline_status != "pass":
        return str(baseline_status or "baseline_failed"), 2
    statuses = _selected_statuses(report)
    if not statuses:
        return "no_heads", 2
    if all(status == "pass" for status in statuses):
        return "pass", 0
    if any(status in {"patch_conflict", "invalid_ref", "reset_failed",
                      "tree_resolution_failed", "infrastructure_error", "cleanup_failed"}
           for status in statuses):
        return "infrastructure_error", 2
    return "heads_failed", 1


def run_validation(args: argparse.Namespace) -> int:
    repo = Path(args.root).resolve()
    snapshot_path = Path(args.snapshot).resolve()
    output = Path(args.output_dir).resolve()
    output.mkdir(parents=True, exist_ok=True)
    numbers = parse_numbers(args.numbers)
    records = select_snapshot(load_snapshot(snapshot_path), numbers)

    original_head, original_head_result = resolve_commit(repo, "HEAD")
    if original_head is None:
        raise ValidationError("cannot resolve original checkout HEAD")
    original_status = _status(repo)
    harness_base, harness_base_result = resolve_commit(repo, args.harness_base)
    harness_ref, harness_ref_result = resolve_commit(repo, args.harness_ref)
    if harness_base is None:
        raise ValidationError(f"invalid --harness-base {args.harness_base}: {harness_base_result.stderr.strip()}")
    if harness_ref is None:
        raise ValidationError(f"invalid --harness-ref {args.harness_ref}: {harness_ref_result.stderr.strip()}")

    patch, patch_result = _git_diff(repo, harness_base, harness_ref)
    if patch is None:
        raise ValidationError(f"cannot compute testing diff: {patch_result.stderr.strip()}")
    patch_path = output / "testing-harness.patch"
    patch_path.write_bytes(patch)
    patch_sha = hashlib.sha256(patch).hexdigest()
    uv = _resolve_uv(args.uv_executable)
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "infrastructure_error",
        "started_at_epoch": time.time(),
        "snapshot": str(snapshot_path),
        "selected_numbers": [int(item["number"]) for item in records],
        "harness": {
            "base_ref": args.harness_base, "base_sha": harness_base,
            "ref": args.harness_ref, "ref_sha": harness_ref,
            "patch": str(patch_path), "patch_sha256": patch_sha,
            "patch_bytes": len(patch),
            "diff_command": patch_result.json(),
        },
        "original_checkout": {
            "root": str(repo), "head_sha": original_head,
            "status_before": original_status,
            "head_resolution": original_head_result.json(),
        },
        "configuration": {
            "deps_prefix": args.deps_prefix, "uv": uv, "jobs": args.jobs,
            "build_dir": args.build_dir, "exclude_labels": args.exclude_labels,
            "targets": list(args.targets), "timeout_seconds": args.timeout,
            "cmake_args": list(args.cmake_arg),
        },
        "baseline": {},
        "heads": [],
    }
    worktree: Path | None = None
    cleanup: dict[str, Any] = {}
    started = time.monotonic()
    try:
        parent = Path(tempfile.mkdtemp(prefix="slicer-pr-validation-", dir=str(output.parent)))
        worktree = parent / "repo"
        add = _git(repo, "worktree", "add", "--detach", str(worktree), harness_ref, timeout=300)
        report["worktree"] = {"path": str(worktree), "add": add.json()}
        if add.returncode != 0:
            raise ValidationError(f"cannot create validation worktree: {add.stderr.strip()}")

        build_path = _resolve_build_path(worktree, args.build_dir)
        configure_args = _default_configure(worktree, build_path, args.deps_prefix, uv)
        configure_args.extend(_format_args(args.cmake_arg, root=worktree, build=build_path,
                                           deps_prefix=args.deps_prefix, uv=uv, jobs=args.jobs))
        build_args = _default_build(worktree, build_path, args.jobs, args.targets)
        report["commands"] = {"configure": configure_args, "build": build_args}

        baseline_tree, baseline_tree_result = _tree_sha(worktree)
        baseline = _run_tree(worktree, output, "baseline", args, configure_args, build_args,
                             commit_sha=harness_ref, tree_sha=baseline_tree)
        baseline["tree_resolution"] = baseline_tree_result.json()
        baseline["harness_patch_applied"] = False
        report["baseline"] = baseline
        _write_json(output / "report.json", report)

        if baseline["status"] != "pass":
            for record in records:
                report["heads"].append({
                    "number": int(record["number"]),
                    "head_ref_name": record.get("headRefName", ""),
                    "head_ref_oid": str(record["headRefOid"]),
                    "status": "baseline_failed",
                    "reason": baseline["status"],
                })
            return_code = 2
        else:
            return_code = 0
            for record in records:
                number = int(record["number"])
                requested_oid = str(record["headRefOid"])
                entry: dict[str, Any] = {
                    "number": number,
                    "head_ref_name": record.get("headRefName", ""),
                    "head_ref_oid": requested_oid,
                    "status": "infrastructure_error",
                }
                resolved, resolution = resolve_commit(worktree, requested_oid)
                entry["head_resolution"] = resolution.json()
                if resolved is None:
                    entry["status"] = "invalid_ref"
                    report["heads"].append(entry)
                    return_code = 2
                    continue
                entry["commit_sha"] = resolved
                reset_ops = _reset_worktree(worktree, resolved)
                entry["reset"] = reset_ops
                if any(item.get("returncode") != 0 for item in reset_ops):
                    entry["status"] = "reset_failed"
                    report["heads"].append(entry)
                    return_code = 2
                    continue
                if patch:
                    apply = _git(worktree, "apply", "--3way", "--index", str(patch_path), timeout=300)
                    entry["patch_apply"] = apply.json()
                    if apply.returncode != 0:
                        entry["status"] = "patch_conflict"
                        entry["conflict_status"] = _status(worktree)
                        unresolved = _git(worktree, "diff", "--name-only", "--diff-filter=U")
                        entry["unresolved_paths"] = unresolved.stdout.splitlines()
                        report["heads"].append(entry)
                        return_code = 2
                        continue
                tree, tree_result = _tree_sha(worktree)
                entry["tree_resolution"] = tree_result.json()
                entry["tested_tree_sha"] = tree
                if tree is None:
                    entry["status"] = "tree_resolution_failed"
                    report["heads"].append(entry)
                    return_code = 2
                    continue
                run = _run_tree(worktree, output, f"pr-{number}", args, configure_args, build_args,
                                commit_sha=resolved, tree_sha=tree)
                entry.update(run)
                entry["harness_patch_applied"] = bool(patch)
                report["heads"].append(entry)
                if entry["status"] != "pass":
                    return_code = 1 if entry["status"] in {"configure_failed", "build_failed",
                                                               "test_failed", "test_skipped", "no_tests",
                                                               "test_timeout",
                                                               "configure_timeout", "build_timeout"} else 2
                _write_json(output / "report.json", report)
    finally:
        if worktree is not None:
            remove = _git(repo, "worktree", "remove", "--force", str(worktree), timeout=300)
            cleanup["remove"] = remove.json()
            cleanup["removed"] = remove.returncode == 0
            parent = worktree.parent
            if remove.returncode == 0:
                try:
                    parent.rmdir()
                    cleanup["parent_removed"] = True
                except OSError as exc:
                    cleanup["parent_removed"] = False
                    cleanup["parent_remove_error"] = str(exc)
            else:
                try:
                    parent.rmdir()
                    cleanup["parent_removed"] = True
                except OSError as exc:
                    cleanup["parent_removed"] = False
                    cleanup["parent_remove_error"] = str(exc)
            report["cleanup"] = cleanup
        report["elapsed_seconds"] = time.monotonic() - started
        report["finished_at_epoch"] = time.time()
        report["original_checkout_after"] = {
            "head_sha": resolve_commit(repo, "HEAD")[0],
            "status": _status(repo),
        }
        if report.get("original_checkout_after", {}).get("head_sha") != original_head:
            report["status"] = "original_checkout_mutated"
        elif report.get("original_checkout_after", {}).get("status") != original_status:
            report["status"] = "original_checkout_status_changed"
        elif cleanup and not cleanup.get("removed", False):
            report["status"] = "cleanup_failed"
        else:
            report["status"], return_code = _overall_status(report)
        _write_json(output / "report.json", report)
    return return_code


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".", help="original repository checkout")
    parser.add_argument("--snapshot", default="build/evidence/open-prs.json")
    parser.add_argument("--harness-base", required=True,
                        help="ref at the base of the testing harness diff")
    parser.add_argument("--harness-ref", required=True,
                        help="ref containing the testing harness")
    parser.add_argument("--deps-prefix", required=True,
                        help="native dependency install prefix for CMake")
    parser.add_argument("--output-dir", default="build/evidence/pr-validation")
    parser.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1))
    parser.add_argument("--numbers", action="append", nargs="+",
                        help="PR numbers, space/comma-separated; may be repeated")
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT)
    parser.add_argument("--uv-executable", help="local uv executable for ORCA_BUNDLED_UV_EXECUTABLE")
    parser.add_argument("--cmake-arg", action="append", default=[],
                        help="extra CMake cache argument; may use {root}, {build}, {deps_prefix}, {uv}, {jobs}")
    parser.add_argument("--target", dest="targets", action="append", default=None,
                        help="native test target (repeat to override defaults)")
    parser.add_argument("--exclude-labels", default=DEFAULT_EXCLUDED_LABELS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.jobs < 1:
        print("slicer_pr_validation: error: --jobs must be positive", file=sys.stderr)
        return 2
    if args.timeout <= 0:
        print("slicer_pr_validation: error: --timeout must be positive", file=sys.stderr)
        return 2
    args.targets = tuple(args.targets or DEFAULT_TARGETS)
    args.uv = _resolve_uv(args.uv_executable)
    try:
        return run_validation(args)
    except (ValidationError, OSError) as exc:
        print(f"slicer_pr_validation: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
