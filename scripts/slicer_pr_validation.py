#!/usr/bin/env python3
"""Validate a testing harness against exact performance PR heads.

The runner reads ``build/evidence/open-prs.json``, creates one disposable
detached worktree, and reuses that worktree and its native CMake build for the
harness guard, unmodified production reference, and every selected PR head.
By default the binary diff between ``--harness-base`` and ``--harness-ref``
is restricted to tests, scripts and build configuration. Production source
index hashes must remain unchanged when applying this test-only overlay.
``--patch-scope integration`` and ``full`` deliberately broaden that scope.
The selected patch is applied with ``git apply --3way --index``; conflicts
are recorded and never guessed at or auto-resolved.

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
      --jobs 4 --test-jobs 8
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
TEST_ONLY_PATHS = ("tests/", "scripts/", "cmake/", ":(top)CMakeLists.txt", ":(top).gitignore")
INTEGRATION_PATHS = TEST_ONLY_PATHS + ("src/CMakeLists.txt", "src/libslic3r/CMakeLists.txt")


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
    """Prefer CTest's invocation total over nested Catch assertion output."""
    ctest = re.findall(r"\b\d+(?:\.\d+)?% tests passed,\s*\d+ tests failed out of\s+(\d+)\b",
                       output, re.IGNORECASE)
    if ctest:
        return int(ctest[-1])
    patterns = (
        r"test cases?:\s*(\d+)\b",
        r"all tests passed\s*\(\d+\s+assertions?\s+in\s+(\d+)\s+test cases?\)",
    )
    return max((int(match.group(1)) for pattern in patterns
                for match in re.finditer(pattern, output, re.IGNORECASE)), default=0)


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
            suites = [suite for suite in root.iter()
                      if suite.tag.rsplit("}", 1)[-1] == "testsuite"
                      and not any(child.tag.rsplit("}", 1)[-1] == "testsuite"
                                  for child in suite)]
            count = max(count, sum(int(suite.attrib.get("skipped", "0")) for suite in suites))
        except (ElementTree.ParseError, OSError, ValueError):
            # A partial JUnit file should not hide command output or a failure.
            pass
    return count


def _xml_tag(element: ElementTree.Element) -> str:
    return element.tag.rsplit("}", 1)[-1]


def _execution_error(text: str) -> bool:
    return bool(re.search(
        r"fatal error condition|unexpected exception|exception:\s*(?:segfault|other)|"
        r"segmentation fault|subprocess (?:killed|aborted)|signal:\s*SIG[A-Z]+|"
        r"\*\*\*Timeout|test timed out|test terminated|\(SEGFAULT\)|\(Timeout\)",
        text, re.IGNORECASE))


def read_junit_evidence(junit: Path) -> dict[str, Any]:
    """Require complete, internally consistent testcase identities and totals."""
    evidence: dict[str, Any] = {
        "usable": False, "errors": [], "counts": {
            "tests": 0, "failures": 0, "errors": 0, "skipped": 0}, "cases": {},
    }
    try:
        if junit.stat().st_size > 64 * 1024 * 1024:
            raise ValueError("JUnit exceeds the 64 MiB evidence limit")
        root = ElementTree.parse(junit).getroot()
    except (ElementTree.ParseError, OSError, ValueError) as exc:
        evidence["errors"].append(str(exc))
        return evidence
    if _xml_tag(root) not in {"testsuite", "testsuites"}:
        evidence["errors"].append("JUnit root is not a test suite")
        return evidence
    statuses: dict[ElementTree.Element, str] = {}
    for case in root.iter():
        if _xml_tag(case) != "testcase":
            continue
        name = case.get("name", "")
        classname = case.get("classname", "")
        identity = f"{classname}::{name}" if classname else name
        if not name:
            evidence["errors"].append("testcase lacks a name")
        if identity in evidence["cases"]:
            evidence["errors"].append(f"duplicate testcase identity: {identity}")
        tags = {_xml_tag(child) for child in case}
        if "error" in tags or ("failure" in tags and
                               _execution_error(ElementTree.tostring(case, encoding="unicode"))):
            status = "error"
        elif "failure" in tags:
            status = "failed"
        elif "skipped" in tags or case.get("status") in {"notrun", "disabled"}:
            status = "skipped"
        else:
            status = "passed"
        statuses[case] = status
        evidence["cases"][identity] = status
    evidence["counts"] = {
        "tests": len(statuses), "failures": sum(s == "failed" for s in statuses.values()),
        "errors": sum(s == "error" for s in statuses.values()),
        "skipped": sum(s == "skipped" for s in statuses.values()),
    }
    for suite in root.iter():
        if _xml_tag(suite) not in {"testsuite", "testsuites"}:
            continue
        for key in ("tests", "failures", "errors", "skipped", "disabled"):
            if key in suite.attrib:
                try:
                    value = int(suite.attrib[key])
                    if value < 0 or (key == "disabled" and value != 0):
                        raise ValueError(f"invalid or incomplete {key} total: {value}")
                except ValueError as exc:
                    evidence["errors"].append(str(exc))
        children = [statuses[case] for case in suite.iter() if case in statuses]
        # CTest encodes crashes in <failure>, while Catch may use <error>.
        # Verify the reported failure/error sum, then retain the stronger
        # execution-error classification above.
        expected = {"tests": len(children), "skipped": children.count("skipped")}
        for key, actual in expected.items():
            if key in suite.attrib:
                try:
                    if int(suite.attrib[key]) != actual:
                        raise ValueError(f"{key} total differs from testcase evidence")
                except ValueError as exc:
                    evidence["errors"].append(str(exc))
        if "failures" in suite.attrib or "errors" in suite.attrib:
            try:
                total = int(suite.get("failures", "0")) + int(suite.get("errors", "0"))
                if total != children.count("failed") + children.count("error"):
                    raise ValueError("failure/error total differs from testcase evidence")
            except ValueError as exc:
                evidence["errors"].append(str(exc))
    if not statuses:
        evidence["errors"].append("JUnit contains no testcases")
    evidence["usable"] = not evidence["errors"]
    return evidence


def junit_case_statuses(junit: Path) -> dict[str, str]:
    evidence = read_junit_evidence(junit)
    return evidence["cases"] if evidence["usable"] else {}


def classify_junit_result(result: CommandResult, evidence: dict[str, Any],
                          console_count: int, *, no_tests: bool = False) -> str:
    if result.timed_out:
        return "test_timeout"
    if result.returncode is not None and result.returncode < 0:
        return "test_error"
    if no_tests:
        return ("test_evidence_incomplete" if evidence["usable"] and
                evidence["counts"]["tests"] > 0 else "no_tests")
    if not evidence["usable"] or console_count != evidence["counts"]["tests"]:
        return "test_evidence_incomplete"
    counts = evidence["counts"]
    if counts["errors"] or _execution_error(result.stdout + result.stderr):
        return "test_error"
    if counts["skipped"]:
        return "test_skipped"
    if counts["failures"]:
        return "test_failed" if result.returncode else "test_evidence_incomplete"
    return "pass" if result.returncode == 0 else "test_error"


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


def load_snapshot(path: Path, *, content: bytes | None = None) -> list[dict[str, Any]]:
    try:
        payload = json.loads(path.read_bytes() if content is None else content)
    except OSError as exc:
        raise ValidationError(f"cannot read snapshot {path}: {exc}") from exc
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
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


def _git_diff_paths(repo: Path, base: str, ref: str) -> tuple[list[str], CommandResult]:
    command = ["git", "diff", "--name-only", "--no-renames", "-z", base, ref, "--"]
    started = time.monotonic()
    try:
        result = subprocess.run(command, cwd=str(repo), shell=False,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                timeout=120, check=False)
        stdout = (result.stdout or b"").decode("utf-8", "replace")
        stderr = (result.stderr or b"").decode("utf-8", "replace")
        command_result = CommandResult(command, result.returncode,
                                       time.monotonic() - started, stdout, stderr)
        # ``--name-only`` is NUL-delimited so names containing newlines remain
        # one path in the scope report.  Empty trailing records are expected.
        return [path for path in stdout.split("\0") if path] if result.returncode == 0 else [], command_result
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        command_result = CommandResult(command, None if isinstance(exc, subprocess.TimeoutExpired) else 127,
                                       time.monotonic() - started, "", str(exc),
                                       isinstance(exc, subprocess.TimeoutExpired))
        return [], command_result


def _scope_pathspecs(scope: str, includes: Sequence[str]) -> list[str] | None:
    if scope == "full":
        return None
    paths = list(TEST_ONLY_PATHS if scope == "test-only" else INTEGRATION_PATHS)
    for include in includes:
        include = include.removeprefix("./")
        if include not in paths:
            paths.append(include)
    return paths


def _path_is_in_scope(path: str, scope: str, includes: Sequence[str]) -> bool:
    path = path.removeprefix("./")
    pathspecs = _scope_pathspecs(scope, includes)
    if pathspecs is None:
        return True
    for spec in pathspecs:
        spec = spec.removeprefix(":(top)")
        if spec.endswith("/") and path.startswith(spec):
            return True
        if path == spec:
            return True
    return False


def _git_diff(repo: Path, base: str, ref: str,
              pathspecs: Sequence[str] | None = None) -> tuple[bytes | None, CommandResult]:
    command = ["git", "diff", "--binary", "--full-index", base, ref]
    command += ["--"] + [_git_pathspec(path) for path in (pathspecs or [])]
    started = time.monotonic()
    try:
        result = subprocess.run(
            command, cwd=str(repo),
            shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            timeout=120, check=False,
        )
        stdout = result.stdout or b""
        stderr = (result.stderr or b"").decode("utf-8", "replace")
        command = CommandResult(command,
                                result.returncode, time.monotonic() - started,
                                stdout.decode("utf-8", "replace"), stderr)
        return stdout if result.returncode == 0 else None, command
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        if isinstance(exc, subprocess.TimeoutExpired):
            result = CommandResult(command,
                                   None, time.monotonic() - started, "", str(exc), True)
        else:
            result = CommandResult(command,
                                   127, time.monotonic() - started, "", str(exc))
        return None, result


def _git_pathspec(path: str) -> str:
    """Use literal top-level matching for caller-supplied include paths."""
    if path.startswith(":(") or path.startswith(":/"):
        return path
    return f":(top,literal){path.removeprefix('./')}"


def _tracked_source_index_hash(repo: Path) -> tuple[str | None, CommandResult]:
    """Hash the tracked ``src/`` index so test-only overlays cannot alter production."""
    result = _git(repo, "ls-files", "-s", "--", "src/")
    if result.returncode != 0:
        return None, result
    return hashlib.sha256(result.stdout.encode("utf-8")).hexdigest(), result


def _reset_worktree(worktree: Path, commit: str) -> list[dict[str, Any]]:
    operations: list[dict[str, Any]] = []
    reset = _git(worktree, "reset", "--hard", commit)
    operations.append(reset.json())
    if reset.returncode == 0:
        clean = _git(worktree, "clean", "-fd")
        operations.append(clean.json())
    return operations


def _run_phase(argv: Sequence[str], *, cwd: Path, timeout: float, log: Path) -> CommandResult:
    """Keep full native output on disk and only a bounded tail in the report."""
    log.parent.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()
    returncode, timed_out = None, False
    with log.open("wb") as stream:
        try:
            result = run_captured(argv, cwd=cwd, timeout=timeout, output_file=stream)
            returncode = result.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
        except OSError as exc:
            returncode = 127
            stream.write(str(exc).encode("utf-8", "replace"))
    with log.open("rb") as stream:
        size = stream.seek(0, os.SEEK_END)
        stream.seek(max(0, size - MAX_OUTPUT))
        tail = stream.read(MAX_OUTPUT).decode("utf-8", "replace")
    if size > MAX_OUTPUT:
        tail = "...[earlier output retained in log]...\n" + tail
    return CommandResult(list(argv), returncode, time.monotonic() - started,
                         tail, "", timed_out)


def _run_tree(worktree: Path, root_output: Path, name: str, args: argparse.Namespace,
              configure_args: Sequence[str], build_args: Sequence[str],
              *, commit_sha: str, tree_sha: str | None) -> dict[str, Any]:
    run_dir = root_output / name
    run_dir.mkdir(parents=True, exist_ok=False)
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
    test_jobs = args.test_jobs if args.test_jobs is not None else args.jobs
    test_args = _default_test(build_path, test_jobs, junit, args.exclude_labels)
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
    entry["junit_exists"] = junit.is_file()
    junit_evidence = read_junit_evidence(junit)
    entry["junit_evidence"] = junit_evidence
    entry["skipped_test_evidence"] = junit_evidence["counts"]["skipped"]
    case_statuses = junit_evidence["cases"] if junit_evidence["usable"] else {}
    entry["junit_case_statuses"] = case_statuses
    for field, wanted in (("failed_test_names", "failed"), ("skipped_test_names", "skipped"),
                          ("error_test_names", "error")):
        entry[field] = sorted(name for name, case_status in case_statuses.items()
                              if case_status == wanted)
    status = classify_junit_result(test, junit_evidence, entry["test_evidence"],
                                   no_tests=entry["no_test_evidence"])
    entry["junit_reference_usable"] = status in {"pass", "test_failed"}
    return finish(status)


def _selected_statuses(report: dict[str, Any]) -> list[str]:
    return [str(item.get("status", "infrastructure_error")) for item in report.get("heads", [])]


def classify_failure_names(reference: dict[str, Any] | None,
                          head: dict[str, Any]) -> dict[str, Any]:
    """Compare failed JUnit testcase names without treating missing evidence as pass."""
    reference_statuses = dict(reference.get("junit_case_statuses", {})) if reference else {}
    head_statuses = dict(head.get("junit_case_statuses", {}))
    head_names = set(head.get("failed_test_names", []))
    reference_names = {name for name, status in reference_statuses.items()
                       if status == "failed"}
    if (not reference or not reference.get("junit_reference_usable", False)
            or not head.get("junit_reference_usable", False)):
        return {
            "reference_available": False,
            "inherited": [], "new": [], "resolved": [],
            "raw_observed_head_failures": sorted(head_names),
            "uncompared_head_failures": sorted(head_names),
            "not_executed_reference_failures": sorted(reference_names),
            "classification_basis": (
                "No comparable reference testcase execution was available; matching names "
                "cannot establish a shared root cause."
            ),
            "reason": "reference or head JUnit evidence unavailable or noncomparable",
        }
    inherited = sorted(name for name in head_names if reference_statuses.get(name) == "failed")
    new = sorted(name for name in head_names if reference_statuses.get(name) == "passed")
    uncompared = sorted(name for name in head_names
                        if reference_statuses.get(name) not in {"failed", "passed"})
    resolved = sorted(name for name in reference_names if head_statuses.get(name) == "passed")
    not_executed = sorted(name for name in reference_names
                          if head_statuses.get(name) not in {"passed", "failed"})
    return {
        "reference_available": True,
        "inherited": inherited,
        "new": new,
        "resolved": resolved,
        "raw_observed_head_failures": sorted(head_names),
        "uncompared_head_failures": uncompared,
        "not_executed_reference_failures": not_executed,
        "reference_failed": sorted(reference_names),
        "head_failed": sorted(head_names),
        "classification_basis": (
            "Testcase names compare execution statuses only; matching failure names do not "
            "establish the same root cause."
        ),
    }


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
                      "tree_resolution_failed", "infrastructure_error", "cleanup_failed",
                      "test_error", "test_evidence_incomplete", "test_timeout",
                      "test_skipped", "no_tests", "configure_timeout", "build_timeout"}
           for status in statuses):
        return "infrastructure_error", 2
    if any(status in {"tracked_source_index_changed", "tracked_source_index_resolution_failed"}
           for status in statuses):
        return "infrastructure_error", 2
    return "heads_failed", 1


def run_validation(args: argparse.Namespace) -> int:
    repo = Path(args.root).resolve()
    snapshot_path = Path(args.snapshot).resolve()
    output = Path(args.output_dir).resolve()
    if output.exists() and any(output.iterdir()):
        raise ValidationError(f"output directory must be new or empty: {output}")
    output.mkdir(parents=True, exist_ok=True)
    numbers = parse_numbers(args.numbers)
    snapshot_bytes = snapshot_path.read_bytes()
    records = select_snapshot(load_snapshot(snapshot_path, content=snapshot_bytes), numbers)

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

    full_diff_paths, paths_result = _git_diff_paths(repo, harness_base, harness_ref)
    if paths_result.returncode != 0:
        raise ValidationError(f"cannot enumerate testing diff paths: {paths_result.stderr.strip()}")
    pathspecs = _scope_pathspecs(args.patch_scope, args.patch_include)
    applied_paths = [path for path in full_diff_paths
                     if _path_is_in_scope(path, args.patch_scope, args.patch_include)]
    excluded_paths = [path for path in full_diff_paths if path not in applied_paths]
    excluded_production_paths = [path for path in excluded_paths
                                 if path.startswith("src/")]
    patch, patch_result = _git_diff(repo, harness_base, harness_ref, pathspecs)
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
        "snapshot_sha256": hashlib.sha256(snapshot_bytes).hexdigest(),
        "output_device": output.stat().st_dev,
        "preflight_heads": [],
        "selected_numbers": [int(item["number"]) for item in records],
        "harness": {
            "base_ref": args.harness_base, "base_sha": harness_base,
            "ref": args.harness_ref, "ref_sha": harness_ref,
            "patch": str(patch_path), "patch_sha256": patch_sha,
            "patch_bytes": len(patch), "patch_scope": args.patch_scope,
            "patch_include": list(args.patch_include), "full_diff_paths": full_diff_paths,
            "applied_paths": applied_paths, "excluded_paths": excluded_paths,
            "excluded_production_diff_paths": excluded_production_paths,
            "path_enumeration": paths_result.json(),
            "diff_command": patch_result.json(),
        },
        "original_checkout": {
            "root": str(repo), "head_sha": original_head,
            "status_before": original_status,
            "head_resolution": original_head_result.json(),
        },
        "configuration": {
            "deps_prefix": args.deps_prefix, "uv": uv, "jobs": args.jobs,
            "test_jobs": args.test_jobs if args.test_jobs is not None else args.jobs,
            "build_dir": args.build_dir, "exclude_labels": args.exclude_labels,
            "targets": list(args.targets), "timeout_seconds": args.timeout,
            "cmake_args": list(args.cmake_arg), "patch_scope": args.patch_scope,
            "patch_include": list(args.patch_include),
            "reference_baseline": args.reference_baseline,
        },
        "baseline": {},
        "reference": {},
        "heads": [],
    }
    worktree: Path | None = None
    cleanup: dict[str, Any] = {}
    started = time.monotonic()
    try:
        for record in records:
            resolved, resolution = resolve_commit(repo, record["headRefOid"])
            report["preflight_heads"].append({
                "number": record["number"], "requested_oid": record["headRefOid"],
                "resolved_oid": resolved, "resolution": resolution.json(),
            })
        if any(item["resolved_oid"] != item["requested_oid"] for item in report["preflight_heads"]):
            report["baseline"] = {"status": "invalid_ref"}
            raise ValidationError("snapshot contains unavailable exact head commits")
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
        baseline_prod_hash, baseline_prod_result = _tracked_source_index_hash(worktree)
        baseline["tracked_source_index_hash_before_overlay"] = baseline_prod_hash
        baseline["tracked_source_index_hash_after_overlay"] = baseline_prod_hash
        baseline["tracked_source_index_unchanged"] = True
        baseline["tracked_source_index_resolution"] = baseline_prod_result.json()
        baseline["harness_patch_applied"] = False
        report["baseline"] = baseline
        _write_json(output / "report.json", report)

        if baseline["status"] != "pass":
            report["reference"] = {
                "status": "not_run",
                "reason": "harness_ref baseline guard did not pass",
            }
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
            if args.reference_baseline:
                reference: dict[str, Any] = {
                    "name": "reference",
                    "commit_sha": harness_base,
                    "status": "infrastructure_error",
                    "harness_patch_applied": False,
                }
                reference_reset = _reset_worktree(worktree, harness_base)
                reference["reset"] = reference_reset
                if any(item.get("returncode") != 0 for item in reference_reset):
                    reference["status"] = "reset_failed"
                else:
                    reference_before, reference_before_result = _tracked_source_index_hash(worktree)
                    reference["tracked_source_index_hash_before_overlay"] = reference_before
                    reference["tracked_source_index_resolution_before"] = reference_before_result.json()
                    if patch:
                        reference_apply = _git(worktree, "apply", "--3way", "--index",
                                                str(patch_path), timeout=300)
                        reference["patch_apply"] = reference_apply.json()
                        if reference_apply.returncode != 0:
                            reference["status"] = "patch_conflict"
                            reference["conflict_status"] = _status(worktree)
                            reference["unresolved_paths"] = _git(
                                worktree, "diff", "--name-only", "--diff-filter=U").stdout.splitlines()
                    if reference.get("status") != "patch_conflict":
                        reference_after, reference_after_result = _tracked_source_index_hash(worktree)
                        reference["tracked_source_index_hash_after_overlay"] = reference_after
                        reference["tracked_source_index_resolution_after"] = reference_after_result.json()
                        reference["tracked_source_index_unchanged"] = reference_before == reference_after
                        if (args.patch_scope == "test-only" and
                                (reference_after is None or not reference["tracked_source_index_unchanged"])):
                            reference["status"] = "tracked_source_index_changed"
                        else:
                            reference_tree, reference_tree_result = _tree_sha(worktree)
                            reference["tree_resolution"] = reference_tree_result.json()
                            if reference_tree is None:
                                reference["status"] = "tree_resolution_failed"
                            else:
                                reference_run = _run_tree(
                                    worktree, output, "reference", args, configure_args, build_args,
                                    commit_sha=harness_base, tree_sha=reference_tree)
                                reference.update(reference_run)
                                reference["harness_patch_applied"] = bool(patch)
                report["reference"] = reference
                _write_json(output / "report.json", report)
            else:
                report["reference"] = {
                    "status": "not_run", "reason": "--no-reference-baseline",
                }
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
                production_before, production_before_result = _tracked_source_index_hash(worktree)
                entry["tracked_source_index_hash_before_overlay"] = production_before
                entry["tracked_source_index_resolution_before"] = production_before_result.json()
                if production_before is None:
                    entry["status"] = "tracked_source_index_resolution_failed"
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
                production_after, production_after_result = _tracked_source_index_hash(worktree)
                entry["tracked_source_index_hash_after_overlay"] = production_after
                entry["tracked_source_index_resolution_after"] = production_after_result.json()
                entry["tracked_source_index_unchanged"] = production_before == production_after
                if (args.patch_scope == "test-only" and
                        (production_after is None or not entry["tracked_source_index_unchanged"])):
                    entry["status"] = "tracked_source_index_changed"
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
                entry["failure_classification"] = classify_failure_names(
                    report.get("reference"), entry)
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
        checkout_after = report["original_checkout_after"]
        if (checkout_after.get("head_sha") != original_head or
                checkout_after.get("status") != original_status):
            # This is deliberately fail-closed, but the runner cannot infer
            # causality when another process edits the shared checkout during
            # an independent worktree run.  Keep both snapshots and report the
            # observation without claiming the runner caused it.
            report["original_checkout_change"] = {
                "observed": True,
                "cause": "unknown; concurrent shared-checkout activity is possible",
                "head_before": original_head,
                "head_after": checkout_after.get("head_sha"),
                "status_before": original_status,
                "status_after": checkout_after.get("status"),
            }
            report["status"] = "original_checkout_changed_during_run"
            return_code = 2
        elif cleanup and not cleanup.get("removed", False):
            report["status"] = "cleanup_failed"
            return_code = 2
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
    parser.add_argument("--jobs", type=int, default=max(1, os.cpu_count() or 1),
                        help="parallel compiler jobs; also the default test slots")
    parser.add_argument("--test-jobs", type=int,
                        help="CTest scheduling slots (defaults to --jobs)")
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
    parser.add_argument("--patch-scope", choices=("test-only", "integration", "full"),
                        default="test-only",
                        help="overlay scope; test-only protects src/, integration allows build CMake, full applies the legacy complete diff")
    parser.add_argument("--patch-include", action="append", default=[],
                        help="additional exact path or directory allowed in test-only/integration overlay")
    parser.add_argument("--reference-baseline", action=argparse.BooleanOptionalAction,
                        default=True,
                        help="run harness-base plus the selected overlay as a failure reference (default: enabled)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.jobs < 1:
        print("slicer_pr_validation: error: --jobs must be positive", file=sys.stderr)
        return 2
    if args.test_jobs is not None and args.test_jobs < 1:
        print("slicer_pr_validation: error: --test-jobs must be positive", file=sys.stderr)
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
