#!/usr/bin/env python3
"""Run a small, reproducible mutation gate against production libslic3r tests.

The harness validates a clean repository and a passing baseline, then uses a
disposable worktree by default or an opt-in retained worktree. The explicit
--reuse-detached-checkout mode instead mutates one clean detached checkout in
place, restores source bytes, and verifies HEAD and tracked cleanliness. Retained
worktree and reuse modes may leave the last mutant in build artifacts; explicitly
rebuild the baseline before using those artifacts.

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
from contextlib import contextmanager
from dataclasses import asdict, dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile
import time
from typing import Any, Sequence


from slicer_process import run_captured

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


def _load_argv_file(argument: str | os.PathLike[str], root: Path,
                    command_name: str) -> tuple[list[str], dict[str, str]]:
    """Load one exact argv vector and bind its source bytes for preflight."""
    path = Path(argument)
    if not path.is_absolute():
        path = root / path
    path = path.resolve()
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise MutationError(f"cannot read {command_name} argv file {path}: {exc}") from exc
    try:
        value = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as exc:
        raise MutationError(f"invalid JSON in {command_name} argv file {path}: {exc}") from exc
    if not isinstance(value, list) or not value:
        raise MutationError(f"{command_name} argv file must contain a nonempty JSON array of strings: {path}")
    if any(not isinstance(item, str) for item in value):
        raise MutationError(f"{command_name} argv file must contain only strings: {path}")
    if not value[0]:
        raise MutationError(f"{command_name} argv file executable (argv[0]) must not be empty: {path}")
    if any("\0" in item for item in value):
        raise MutationError(f"{command_name} argv file arguments must not contain NUL: {path}")
    return split_command(value), {"path": str(path), "sha256": hashlib.sha256(raw).hexdigest()}


def _command_arguments(command: str | None, argv_file: str | None, *, root: Path,
                       command_name: str) -> tuple[list[str], dict[str, str] | None]:
    if argv_file is not None:
        return _load_argv_file(argv_file, root, command_name)
    if command is None:
        raise MutationError(f"missing {command_name} command")
    return split_command(command), None


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


def _temp_environment_variables() -> tuple[str, ...]:
    """Return Python's variables plus Boost's POSIX TEMPDIR fallback."""
    variables = ("TMPDIR", "TMP", "TEMP")
    return variables + (("TEMPDIR",) if os.name == "posix" else ())


_TEMP_ENVIRONMENT_VARIABLES = _temp_environment_variables()


def _write_temp_probe(path: Path) -> None:
    """Create, write, and remove one file inside a configured temp directory."""
    with path.open("xb") as stream:
        stream.write(b"slicer mutation temp-directory probe\n")
    path.unlink()


def _preflight_temp_environment() -> list[dict[str, str]]:
    """Reject invalid explicit temp variables before worktree/build activity.

    An absent variable retains tempfile's existing fallback behavior.  An
    explicitly empty value is rejected because Boost.Filesystem treats an
    empty getenv result as its selected path and reports ENOTDIR.  Relative
    values are rejected because child commands run from a disposable
    worktree, so their resolution would differ from this controller's cwd.
    Every explicit candidate is checked because silently falling through from
    a misspelled or unwritable higher-priority variable makes later Python and
    C++ temporary-file failures difficult to diagnose.
    """
    checked: list[dict[str, str]] = []
    for variable in _TEMP_ENVIRONMENT_VARIABLES:
        configured = os.environ.get(variable)
        if configured is None:
            continue
        if configured == "":
            raise MutationError(
                f"explicit {variable} is empty; unset it or set it to an absolute "
                "writable temporary directory")
        if not Path(configured).is_absolute():
            raise MutationError(
                f"explicit {variable}={configured!r} must be an absolute path; "
                "relative temporary paths are unsafe across the disposable worktree cwd")
        try:
            if not Path(configured).is_dir():
                raise MutationError(
                    f"explicit {variable}={configured!r} is not an existing directory")
            with tempfile.TemporaryDirectory(
                    prefix=".slicer-mutation-preflight-", dir=configured) as probe:
                _write_temp_probe(Path(probe) / "write-probe")
        except MutationError:
            raise
        except (OSError, ValueError) as exc:
            raise MutationError(
                f"explicit {variable}={configured!r} failed its temporary-directory "
                f"create/write probe: {exc}") from exc
        checked.append({"variable": variable, "path": configured})
    return checked


@dataclass
class CommandResult:
    argv: list[str]
    returncode: int | None
    duration_seconds: float
    stdout: str
    stderr: str
    timed_out: bool = False
    log_path: str | None = None
    log_sha256: str | None = None
    log_size_bytes: int | None = None

    def json(self) -> dict[str, Any]:
        result = asdict(self)
        result["stdout"] = _truncate(result["stdout"])
        result["stderr"] = _truncate(result["stderr"])
        if self.log_path is not None:
            result["log"] = {
                "path": self.log_path,
                "sha256": self.log_sha256,
                "size_bytes": self.log_size_bytes,
            }
        return result


def _log_metadata(path: Path) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as stream:
        while chunk := stream.read(64 * 1024):
            digest.update(chunk)
            size += len(chunk)
    return digest.hexdigest(), size


def run_command(argv: Sequence[str], *, cwd: Path, timeout: float,
                output_file: Path | None = None) -> CommandResult:
    started = time.monotonic()
    stream = None
    try:
        if output_file is not None:
            output_file.parent.mkdir(parents=True, exist_ok=True)
            stream = output_file.open("wb")
        if stream is None:
            # Keep the legacy call shape for callers and mocks that do not
            # request file-backed output.
            result = run_captured(argv, cwd=cwd, timeout=timeout)
        else:
            result = run_captured(argv, cwd=cwd, timeout=timeout,
                                  output_file=stream)
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        command = CommandResult(list(argv), result.returncode,
                                time.monotonic() - started, stdout, stderr)
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        command = CommandResult(list(argv), None, time.monotonic() - started,
                                stdout, stderr, True)
    except FileNotFoundError as exc:
        command = CommandResult(list(argv), 127, time.monotonic() - started, "", str(exc))
    finally:
        if stream is not None:
            stream.close()
    if output_file is not None and output_file.is_file():
        command.log_path = str(output_file.resolve())
        command.log_sha256, command.log_size_bytes = _log_metadata(output_file)
    return command


_TEST_COUNT_PATTERNS = (
    re.compile(r"out of\s+(\d+)\b", re.IGNORECASE),
    re.compile(r"\bof\s+(\d+)\s+tests?\b", re.IGNORECASE),
    re.compile(r"\b(\d+)\s+tests?\s+(?:passed|failed|skipped)\b", re.IGNORECASE),
    re.compile(r"test cases?:\s*(\d+)\b", re.IGNORECASE),
    re.compile(r"all tests passed\s*\(\d+\s+assertions?\s+in\s+(\d+)\s+test cases?", re.IGNORECASE),
)
_NO_EXECUTION_PATTERNS = (
    re.compile(r"no tests? (?:were )?found", re.IGNORECASE),
    re.compile(r"not run", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+(?:tests?|test cases?)\s+skipped\b", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+skipped\b", re.IGNORECASE),
)


@dataclass(frozen=True)
class LogEvidence:
    """Bounded summary collected from the complete command log."""

    test_count: int = 0
    no_execution: bool = False
    assertion_failure: bool = False
    failed_summary: bool = False
    infrastructure_failure: bool = False
    timed_out: bool = False
    complete: bool = True
    bytes_read: int = 0
    error: str = ""


_MAX_EVIDENCE_LINE = 1024 * 1024
_INFRASTRUCTURE_PATTERN = re.compile(
    r"\*{3}Exception:|fatal error condition:|unexpected exception with message:|"
    r"\bSubprocess killed\b|"
    r"\((?:SEGFAULT|Subprocess aborted|OTHER_FAULT|ILLEGAL|INTERRUPT|Bus error)\)\s*$",
    re.IGNORECASE,
)
_TIMEOUT_PATTERN = re.compile(r"\*{3}Timeout\b|\(Timeout\)\s*$", re.IGNORECASE)
_FAILED_SUMMARY_PATTERN = re.compile(
    r"\d+% tests passed,\s*[1-9]\d* tests failed out of", re.IGNORECASE)


def scan_log_evidence(path: Path) -> LogEvidence:
    """Scan a complete log with bounded line memory; never trust a truncated scan."""
    max_count = 0
    passed_failed = 0
    ctest_total: int | None = None
    no_execution = False
    assertion_failure = False
    failed_summary = False
    infrastructure_failure = False
    timed_out = False
    bytes_read = 0
    try:
        with path.open("rb") as stream:
            while True:
                raw = stream.readline(_MAX_EVIDENCE_LINE + 1)
                if not raw:
                    break
                bytes_read += len(raw)
                if len(raw) > _MAX_EVIDENCE_LINE:
                    return LogEvidence(complete=False, bytes_read=bytes_read,
                                       error="evidence line exceeds bounded scanner limit")
                text = raw.decode("utf-8", "replace")
                if any(pattern.search(text) for pattern in _NO_EXECUTION_PATTERNS):
                    no_execution = True
                if _INFRASTRUCTURE_PATTERN.search(text):
                    infrastructure_failure = True
                if _TIMEOUT_PATTERN.search(text):
                    timed_out = True
                if _FAILED_SUMMARY_PATTERN.search(text):
                    failed_summary = True
                assertion_summaries = re.findall(
                    r"^\s*assertions?\s*:\s*\d+\s*\|([^\n]*)",
                    text, re.IGNORECASE)
                if any(re.search(r"(?:^|\|)\s*[1-9]\d*\s+failed\s*(?:\||$)", summary,
                                 re.IGNORECASE) for summary in assertion_summaries):
                    assertion_failure = True
                ctest_matches = re.findall(
                    r"\d+% tests passed,\s*\d+ tests failed out of\s+(\d+)\b",
                    text, re.IGNORECASE)
                if ctest_matches:
                    ctest_total = int(ctest_matches[-1])
                passed_failed += sum(int(match.group(1)) for match in re.finditer(
                    r"\b([1-9]\d*)\s+tests?\s+(?:passed|failed)\b",
                    text, re.IGNORECASE))
                for pattern in _TEST_COUNT_PATTERNS:
                    for match in pattern.finditer(text):
                        max_count = max(max_count, int(match.group(1)))
    except OSError as exc:
        return LogEvidence(complete=False, bytes_read=bytes_read, error=str(exc))
    if no_execution:
        test_count = passed_failed
    elif ctest_total is not None:
        test_count = ctest_total
    else:
        test_count = max_count
    return LogEvidence(test_count=test_count, no_execution=no_execution,
                       assertion_failure=assertion_failure,
                       failed_summary=failed_summary,
                       infrastructure_failure=infrastructure_failure,
                       timed_out=timed_out, bytes_read=bytes_read)


def test_evidence(output: str) -> int:
    """Count runner-reported tests; zero means a filter likely matched none."""
    if any(pattern.search(output) for pattern in _NO_EXECUTION_PATTERNS):
        return sum(int(match.group(1)) for match in re.finditer(
            r"\b([1-9]\d*)\s+tests?\s+(?:passed|failed)\b", output, re.IGNORECASE))
    # CTest counts registered invocations; a grouped invocation may run several
    # Catch2 cases. Failure output includes both reporters, unlike a quiet
    # passing baseline, so their counts must not be mixed.
    ctest_totals = re.findall(r"\d+% tests passed,\s*\d+ tests failed out of\s+(\d+)\b",
                              output, re.IGNORECASE)
    if ctest_totals:
        return int(ctest_totals[-1])
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


def _load_mutants(path: str | list[str] | None, root: Path | None = None, *,
                  expected_sha256: str | list[str] | None = None) -> list[Mutant]:
    if not path:
        if expected_sha256 is not None:
            raise MutationError("--mutants-sha256 requires an explicit --mutants manifest")
        return list(DEFAULT_MUTANTS)
    if isinstance(path, list):
        digests = [None] * len(path) if expected_sha256 is None else expected_sha256
        if not isinstance(digests, list) or len(digests) != len(path):
            raise MutationError("provide one --mutants-sha256 per --mutants manifest, in the same order")
        return [mutant for manifest, digest in zip(path, digests)
                for mutant in _load_mutants(manifest, root, expected_sha256=digest)]
    if expected_sha256 is not None and (
            not isinstance(expected_sha256, str) or re.fullmatch(r"[0-9a-fA-F]{64}", expected_sha256) is None):
        raise MutationError("--mutants-sha256 must contain 64 hexadecimal characters")
    manifest_arg = Path(path)
    manifest = ((root / manifest_arg) if root is not None and not manifest_arg.is_absolute()
                else manifest_arg).resolve()
    try:
        raw = manifest.read_bytes()
        if expected_sha256 is not None and hashlib.sha256(raw).hexdigest() != expected_sha256.lower():
            raise MutationError(f"mutant manifest SHA-256 mismatch: {manifest}")
        payload = json.loads(raw.decode("utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
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


def _is_ctest(executable: str) -> bool:
    return executable.replace("\\", "/").rsplit("/", 1)[-1].lower() in {"ctest", "ctest.exe"}


def _test_command(argv: Sequence[str], mutant: Mutant) -> list[str]:
    result = [value.replace("{filter}", mutant.test_filter) for value in argv]
    if _is_ctest(result[0]):
        # Catch2's assertion summary is needed to distinguish a test failure
        # from a CTest child crash or another infrastructure failure.
        if "--output-on-failure" not in result:
            result.append("--output-on-failure")
    if mutant.test_filter and "{filter}" not in " ".join(argv):
        has_label = any(value == "--label-regex" or value.startswith("--label-regex=")
                        or (value.startswith("-L") and not value.startswith("-LE"))
                        for value in result[1:])
        if _is_ctest(result[0]) and not has_label:
            result.extend(["-L", mutant.test_filter])
    return result


def _classify_build(result: CommandResult) -> str:
    if result.timed_out:
        return "build_timeout"
    if result.returncode is not None and result.returncode < 0:
        return "build_killed"
    return "build_error"


def _test_timed_out(result: CommandResult) -> bool:
    # CTest can time out a child and exit normally with status 8 before the
    # outer command deadline. Such a run is incomplete evidence, not a kill.
    return result.timed_out or bool(re.search(
        r"\*{3}Timeout\b|\(Timeout\)\s*$",
        result.stdout + "\n" + result.stderr, re.IGNORECASE | re.MULTILINE))


def _classify_test(result: CommandResult, evidence: int,
                   expected_tests: int | None = None,
                   *, log_evidence: LogEvidence | None = None) -> str:
    if log_evidence is not None:
        if not log_evidence.complete:
            return "test_evidence_incomplete"
        if log_evidence.timed_out:
            return "test_timeout"
    if _test_timed_out(result):
        return "test_timeout"
    if result.returncode is not None and result.returncode < 0:
        return "test_killed"
    output = result.stdout + "\n" + result.stderr
    # CTest itself often returns 8 after a child dies. Catch2 can also report
    # a fatal signal or unexpected exception as a failed assertion. Neither
    # establishes that an ordinary behavioral assertion detected the mutant.
    infrastructure_failure = (log_evidence.infrastructure_failure
                               if log_evidence is not None else
                               bool(re.search(
                                   r"\*{3}Exception:|fatal error condition:|unexpected exception with message:|"
                                   r"\bSubprocess killed\b|"
                                   r"\((?:SEGFAULT|Subprocess aborted|OTHER_FAULT|ILLEGAL|INTERRUPT|Bus error)\)\s*$",
                                   output, re.IGNORECASE | re.MULTILINE)))
    if infrastructure_failure:
        return "test_infrastructure_failure"
    if evidence <= 0:
        return "test_no_evidence"
    if expected_tests is not None and evidence != expected_tests:
        return "test_count_mismatch"
    assertion_failure = (log_evidence.assertion_failure
                         if log_evidence is not None else any(
                             re.search(r"(?:^|\|)\s*[1-9]\d*\s+failed\s*(?:\||$)", summary,
                                       re.IGNORECASE)
                             for summary in re.findall(
                                 r"^\s*assertions?\s*:\s*\d+\s*\|([^\n]*)", output,
                                 re.IGNORECASE | re.MULTILINE)))
    if result.returncode != 0:
        if assertion_failure:
            return "killed"
        return "test_failure_unverified"
    failed_summary = (log_evidence.failed_summary if log_evidence is not None else
                      bool(re.search(r"\d+% tests passed,\s*[1-9]\d* tests failed out of",
                                     output)))
    if assertion_failure or failed_summary:
        return "test_infrastructure_failure"
    return "survived"


def _write_report(path: Path, report: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(report, indent=2, sort_keys=True) + "\n"
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp",
                                     dir=str(path.parent), text=True)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def _output_available(out: Path) -> None:
    """Reject prior evidence before any worktree or report can be created."""
    if out.is_symlink():
        raise MutationError(f"output evidence path must not be a symlink: {out}")
    if not out.exists():
        return
    if not out.is_dir():
        raise MutationError(f"output evidence path is not a directory: {out}")
    try:
        next(out.iterdir())
    except StopIteration:
        return
    raise MutationError(f"output evidence directory is not empty; refusing to clobber: {out}")


def _lexical_output_path(root: Path, argument: str | os.PathLike[str]) -> Path:
    """Make an absolute path without resolving a final symlink."""
    candidate = Path(argument)
    if not candidate.is_absolute():
        candidate = root / candidate
    return Path(os.path.abspath(candidate))


def _checkpoint(report_path: Path, report: dict[str, Any], phase: str) -> None:
    report["checkpoint"] = {"state": "in_progress", "phase": phase}
    _write_report(report_path, report)


_PERSISTENT_WORKTREE_SCHEMA = 1
_PERSISTENT_HARNESS = "slicer_mutation"
_PERSISTENT_MARKER = ".slicer-mutation-worktree.json"
_PERSISTENT_LOCK_SUFFIX = ".slicer-mutation.lock"
_REUSE_CHECKOUT_LOCK = ".slicer-mutation-reuse.lock"


def _reject_symlink_components(path: Path, *, label: str) -> None:
    """Reject symlinked path components before any harness-owned write."""
    current = Path(path.anchor) if path.anchor else Path()
    for component in path.parts[1:] if path.is_absolute() else path.parts:
        current /= component
        if current.is_symlink():
            # macOS exposes the ordinary temporary directory through these
            # system aliases.  They are safe canonical aliases, unlike a
            # caller-provided link in the worktree path.
            canonical = Path(os.path.realpath(current))
            if (current == Path("/var") and canonical == Path("/private/var")) or \
                    (current == Path("/tmp") and canonical == Path("/private/tmp")):
                continue
            raise MutationError(f"{label} must not contain symlink component: {current}")


def _canonical_path(path: Path) -> Path:
    """Canonicalize system aliases before ownership and descendant checks."""
    return Path(os.path.realpath(path))


def _canonical_git_path(repo: Path, value: str) -> Path:
    candidate = Path(value.strip())
    if not candidate.is_absolute():
        candidate = repo / candidate
    return _canonical_path(candidate)


def _persistent_git_common_dir(repo: Path) -> Path:
    result = _git(repo, "rev-parse", "--git-common-dir")
    if result.returncode != 0:
        raise MutationError(f"cannot resolve git common directory: {_truncate(result.stderr)}")
    return _canonical_git_path(repo, result.stdout)


def _registered_worktree(repo: Path, path: Path, expected_head: str) -> None:
    listing = _git(repo, "worktree", "list", "--porcelain")
    if listing.returncode != 0:
        raise MutationError(f"cannot inspect caller worktree registration: {_truncate(listing.stderr)}")
    record: dict[str, str] = {}
    records: list[dict[str, str]] = []
    for line in listing.stdout.splitlines() + [""]:
        if not line.strip():
            if record:
                records.append(record)
                record = {}
            continue
        key, _, value = line.partition(" ")
        record[key] = value
    canonical = _canonical_path(path)
    for candidate in records:
        if candidate.get("worktree") and _canonical_path(Path(candidate["worktree"])) == canonical:
            if candidate.get("HEAD") != expected_head:
                raise MutationError("persistent worktree registration has an unexpected HEAD")
            return
    raise MutationError("persistent worktree is not registered with the caller repository")


def _verify_persistent_worktree(repo: Path, path: Path, expected_head: str) -> None:
    top = _git(path, "rev-parse", "--show-toplevel")
    if top.returncode != 0 or _canonical_git_path(path, top.stdout) != _canonical_path(path):
        raise MutationError("persistent path is not a valid worktree for this harness")
    caller_common = _persistent_git_common_dir(repo)
    worktree_common = _persistent_git_common_dir(path)
    if caller_common != worktree_common:
        raise MutationError("persistent worktree belongs to another git repository")
    branch = _git(path, "symbolic-ref", "--quiet", "--short", "HEAD")
    if branch.returncode == 0:
        raise MutationError("persistent worktree must remain detached")
    if branch.returncode != 1:
        raise MutationError(f"cannot inspect persistent worktree HEAD state: {_truncate(branch.stderr)}")
    head = _git(path, "rev-parse", "HEAD")
    if head.returncode != 0 or head.stdout.strip() != expected_head:
        raise MutationError("persistent worktree has an unexpected HEAD")
    status = _git(path, "status", "--porcelain", "--untracked-files=no")
    if status.returncode != 0:
        raise MutationError(f"cannot inspect persistent worktree: {_truncate(status.stderr)}")
    if status.stdout.strip():
        raise MutationError("persistent worktree has dirty tracked files")
    _registered_worktree(repo, path, expected_head)


def _persistent_worktree_path(repo: Path, argument: str | os.PathLike[str]) -> Path:
    path = _lexical_output_path(repo, argument)
    _reject_symlink_components(path, label="persistent worktree")
    path = _canonical_path(path)
    repo = _canonical_path(repo)
    if path == repo or _path_under(path, repo):
        raise MutationError("persistent worktree must be outside the caller checkout")
    path.parent.mkdir(parents=True, exist_ok=True)
    return path


def _write_exclusive_json(path: Path, payload: dict[str, Any]) -> None:
    encoded = (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8")
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise MutationError(f"persistent worktree is already in use: {path}") from exc
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise


@dataclass
class _ReuseCheckoutLease:
    path: Path
    git_dir: Path
    lock: Path
    commit: str
    lock_identity: tuple[int, int]

    def report(self) -> dict[str, Any]:
        return {
            "persistent": False,
            "reused_detached_checkout": True,
            "path": str(self.path),
            "git_dir": str(self.git_dir),
            "lock": str(self.lock),
            "commit": self.commit,
            "binary_warning": "build artifacts may contain the last mutant; explicitly rebuild the baseline before use",
        }

    def release(self) -> None:
        try:
            stat = self.lock.lstat()
            if (stat.st_dev, stat.st_ino) != self.lock_identity:
                raise MutationError(f"reuse lock changed while held; refusing to remove: {self.lock}")
            self.lock.unlink()
        except OSError as exc:
            raise MutationError(f"cannot release reused-checkout lock {self.lock}: {exc}") from exc


def _verify_reuse_checkout(repo: Path, expected_head: str) -> None:
    repo = _canonical_path(repo)
    top = _git(repo, "rev-parse", "--show-toplevel")
    if top.returncode != 0 or _canonical_git_path(repo, top.stdout) != repo:
        raise MutationError("reuse path is not a git checkout root")
    branch = _git(repo, "symbolic-ref", "--quiet", "--short", "HEAD")
    if branch.returncode == 0:
        raise MutationError("reused checkout must be detached")
    if branch.returncode != 1:
        raise MutationError(f"cannot inspect reused checkout HEAD state: {_truncate(branch.stderr)}")
    head = _git(repo, "rev-parse", "HEAD")
    if head.returncode != 0 or head.stdout.strip() != expected_head:
        raise MutationError("reused checkout HEAD changed")
    status = _git(repo, "status", "--porcelain", "--untracked-files=no")
    if status.returncode != 0:
        raise MutationError(f"cannot inspect reused checkout: {_truncate(status.stderr)}")
    if status.stdout.strip():
        raise MutationError("reused checkout has dirty tracked files")


def _acquire_reuse_checkout(repo: Path, commit: str) -> _ReuseCheckoutLease:
    repo = _canonical_path(repo)
    _verify_reuse_checkout(repo, commit)
    result = _git(repo, "rev-parse", "--absolute-git-dir")
    if result.returncode != 0:
        raise MutationError(f"cannot resolve checkout git directory: {_truncate(result.stderr)}")
    git_dir = Path(result.stdout.strip()).resolve(strict=True)
    if not git_dir.is_dir():
        raise MutationError(f"checkout git directory is not a directory: {git_dir}")
    lock = git_dir / _REUSE_CHECKOUT_LOCK
    try:
        fd = os.open(lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as exc:
        raise MutationError(
            f"reused-checkout lock exists (active or stale); inspect before removing: {lock}") from exc
    stat = os.fstat(fd)
    identity = (stat.st_dev, stat.st_ino)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            json.dump({"pid": os.getpid(), "path": str(repo), "commit": commit}, stream,
                      sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        _verify_reuse_checkout(repo, commit)
    except BaseException:
        try:
            current = lock.lstat()
            if (current.st_dev, current.st_ino) == identity:
                lock.unlink()
        except OSError:
            pass
        raise
    return _ReuseCheckoutLease(repo, git_dir, lock, commit, identity)


@dataclass
class _PersistentLease:
    repo: Path
    path: Path
    lock: Path
    marker: Path
    reused: bool
    previous_head: str | None
    checkout: dict[str, Any] | None

    def report(self) -> dict[str, Any]:
        return {
            "persistent": True,
            "path": str(self.path),
            "marker": str(self.marker),
            "lock": str(self.lock),
            "reused": self.reused,
            "previous_head": self.previous_head,
            "checkout": self.checkout,
            "retained_untracked_build_capability": "preserved by non-forcing detached checkout",
            "binary_warning": "build artifacts may contain the last mutant; explicitly rebuild the baseline before use",
        }

    def release(self) -> None:
        try:
            (self.lock / "owner.json").unlink()
            self.lock.rmdir()
        except OSError as exc:
            raise MutationError(f"cannot release persistent worktree lock {self.lock}: {exc}") from exc


def _persistent_worktree(repo: Path, path: Path, commit: str) -> _PersistentLease:
    """Acquire a marked, detached worktree without deleting its untracked build."""
    repo = _canonical_path(Path(os.path.abspath(repo)))
    path = Path(os.path.abspath(path))
    _reject_symlink_components(path, label="persistent worktree")
    path = _canonical_path(path)
    if path == repo or _path_under(path, repo):
        raise MutationError("persistent worktree must be outside the caller checkout")
    path.parent.mkdir(parents=True, exist_ok=True)
    lock = path.with_name(path.name + _PERSISTENT_LOCK_SUFFIX)
    if lock.exists() or lock.is_symlink():
        raise MutationError(f"persistent worktree is already in use: {lock}")
    try:
        lock.mkdir(mode=0o700)
    except FileExistsError as exc:
        raise MutationError(f"persistent worktree is already in use: {lock}") from exc
    owner = lock / "owner.json"
    try:
        _write_exclusive_json(owner, {
            "schema_version": _PERSISTENT_WORKTREE_SCHEMA,
            "harness": _PERSISTENT_HARNESS,
            "pid": os.getpid(),
            "repo": str(repo),
            "path": str(path),
        })
    except Exception:
        lock.rmdir()
        raise
    marker = path / _PERSISTENT_MARKER
    created = False
    try:
        previous_head: str | None = None
        checkout: dict[str, Any] | None = None
        if path.exists():
            if not path.is_dir():
                raise MutationError(f"persistent worktree path is not a directory: {path}")
            if marker.is_symlink() or not marker.is_file():
                raise MutationError("existing persistent path is not a harness-owned worktree")
            try:
                identity = json.loads(marker.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError) as exc:
                raise MutationError(f"cannot read persistent worktree marker: {exc}") from exc
            expected = {
                "schema_version": _PERSISTENT_WORKTREE_SCHEMA,
                "harness": _PERSISTENT_HARNESS,
                "repo": str(repo),
                "path": str(path),
            }
            if any(identity.get(key) != value for key, value in expected.items()):
                raise MutationError("existing persistent worktree marker belongs to another repository or harness")
            current = _git(path, "rev-parse", "HEAD")
            if current.returncode != 0:
                raise MutationError(f"cannot resolve persistent worktree HEAD: {_truncate(current.stderr)}")
            previous_head = current.stdout.strip()
            _verify_persistent_worktree(repo, path, previous_head)
            if previous_head != commit:
                checkout_result = _git(path, "checkout", "--detach", commit)
                checkout = checkout_result.json()
                if checkout_result.returncode != 0:
                    raise MutationError(f"cannot move persistent worktree to {commit}: {_truncate(checkout_result.stderr)}")
                _verify_persistent_worktree(repo, path, commit)
            else:
                _verify_persistent_worktree(repo, path, commit)
            reused = True
        else:
            parent = path.parent
            parent.mkdir(parents=True, exist_ok=True)
            add = _git(repo, "worktree", "add", "--detach", str(path), commit)
            if add.returncode != 0:
                raise MutationError(f"cannot create persistent worktree: {_truncate(add.stderr)}")
            created = True
            _verify_persistent_worktree(repo, path, commit)
            _write_exclusive_json(marker, {
                "schema_version": _PERSISTENT_WORKTREE_SCHEMA,
                "harness": _PERSISTENT_HARNESS,
                "repo": str(repo),
                "path": str(path),
                "created_commit": commit,
            })
            reused = False
        return _PersistentLease(repo, path, lock, marker, reused, previous_head, checkout)
    except Exception:
        try:
            if created and path.exists():
                _git(repo, "worktree", "remove", "--force", str(path))
        finally:
            try:
                owner.unlink()
                lock.rmdir()
            except OSError:
                pass
        raise


@contextmanager
def _mutation_worktree(repo: Path, commit: str, work_parent: Path | None,
                       persistent_path: Path | None, report_path: Path,
                       report: dict[str, Any], reuse_detached_checkout: bool = False):
    if reuse_detached_checkout:
        lease = _acquire_reuse_checkout(repo, commit)
        body_failed = False
        try:
            try:
                report["worktree"] = lease.report()
                _checkpoint(report_path, report, "reuse_detached_checkout_ready")
                yield lease.path
            except BaseException:
                body_failed = True
                raise
        finally:
            postflight_error: Exception | None = None
            try:
                _verify_reuse_checkout(repo, commit)
                report["worktree"]["postflight_tracked_clean"] = True
            except MutationError as exc:
                postflight_error = exc
                report.setdefault("worktree", {})["postflight_error"] = str(exc)
                report["status"] = "postflight_dirty"
            try:
                lease.release()
                report.setdefault("worktree", {})["lock_released"] = True
                _checkpoint(report_path, report, "reuse_detached_checkout_released")
            except Exception as exc:
                report.setdefault("worktree", {})["release_error"] = str(exc)
                report["cleanup_failed"] = True
                report["status"] = "cleanup_failed"
                if not body_failed and postflight_error is None:
                    raise
            if not body_failed and postflight_error is not None:
                raise postflight_error
        return
    if persistent_path is not None:
        lease = _persistent_worktree(repo, persistent_path, commit)
        body_failed = False
        try:
            try:
                report["worktree"] = lease.report()
                _checkpoint(report_path, report, "persistent_worktree_ready")
                yield lease.path
            except BaseException:
                body_failed = True
                raise
        finally:
            postflight_error: Exception | None = None
            try:
                _verify_persistent_worktree(repo, lease.path, commit)
                report["worktree"]["postflight_tracked_clean"] = True
            except MutationError as exc:
                postflight_error = exc
                report.setdefault("worktree", {})["postflight_error"] = str(exc)
                report["status"] = "postflight_dirty"
            try:
                lease.release()
                report.setdefault("worktree", {})["lock_released"] = True
                _checkpoint(report_path, report, "persistent_worktree_released")
            except Exception as exc:
                report.setdefault("worktree", {})["release_error"] = str(exc)
                report["cleanup_failed"] = True
                report["status"] = "cleanup_failed"
                if not body_failed and postflight_error is None:
                    raise
            if not body_failed and postflight_error is not None:
                raise postflight_error
        return
    with tempfile.TemporaryDirectory(prefix="slicer-mutation-",
                                      dir=str(work_parent) if work_parent else None) as parent:
        worktree = Path(parent) / "repo"
        report["worktree"] = {"path": str(worktree), "persistent": False}
        body_failed = False
        try:
            try:
                add = _git(repo, "worktree", "add", "--detach", str(worktree), commit)
                report["worktree"]["add"] = add.json()
                _checkpoint(report_path, report, "worktree_add")
                if add.returncode != 0:
                    raise MutationError(f"cannot create disposable worktree: {_truncate(add.stderr)}")
                yield worktree
            except BaseException:
                body_failed = True
                raise
        finally:
            postflight_error: Exception | None = None
            try:
                if (worktree / ".git").exists():
                    _verify_persistent_worktree(repo, worktree, commit)
                    report["worktree"]["postflight_tracked_clean"] = True
            except MutationError as exc:
                postflight_error = exc
                report["worktree"]["postflight_error"] = str(exc)
                report["status"] = "postflight_dirty"
            try:
                remove = _git(repo, "worktree", "remove", "--force", str(worktree))
                report["worktree"]["remove"] = remove.json()
                if remove.timed_out or remove.returncode != 0:
                    report["cleanup_failed"] = True
                    report["status"] = "cleanup_failed"
                _checkpoint(report_path, report, "worktree_removed")
            except Exception as exc:
                report["worktree"]["cleanup_error"] = str(exc)
                report["cleanup_failed"] = True
                report["status"] = "cleanup_failed"
                if not body_failed and postflight_error is None:
                    raise
            if not body_failed and postflight_error is not None:
                raise postflight_error


def _result_log_evidence(result: CommandResult) -> LogEvidence | None:
    if result.log_path is None:
        return None
    return scan_log_evidence(Path(result.log_path))


def _command_log(log_root: Path, serial: int, phase: str, label: str = "") -> Path:
    safe_phase = re.sub(r"[^A-Za-z0-9_.-]+", "-", phase).strip("-") or "command"
    safe_label = re.sub(r"[^A-Za-z0-9_.-]+", "-", label).strip("-")
    suffix = f"-{safe_label}" if safe_label else ""
    return log_root / f"{serial:04d}-{safe_phase}{suffix}.log"


def run_mutation(args: argparse.Namespace) -> int:
    if args.reuse_detached_checkout and args.persistent_worktree is not None:
        raise MutationError("--reuse-detached-checkout cannot be combined with --persistent-worktree")
    if args.reuse_detached_checkout and args.work_root is not None:
        raise MutationError("--reuse-detached-checkout cannot be combined with --work-root")
    repo = Path(args.root).resolve()
    if not repo.is_dir():
        raise MutationError(f"repository root does not exist: {repo}")
    if args.max_mutants <= 0:
        raise MutationError("--max-mutants must be positive")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        raise MutationError("--timeout must be finite and positive")
    build_argv, build_argv_file = _command_arguments(
        args.build_command, args.build_argv_file, root=repo, command_name="build")
    test_argv, test_argv_file = _command_arguments(
        args.test_command, args.test_argv_file, root=repo, command_name="test")
    manifest_digests = getattr(args, "mutants_sha256", None)
    guarded_mutants = (_load_mutants(args.mutants, repo, expected_sha256=manifest_digests)
                       if manifest_digests is not None else None)
    temp_preflight = _preflight_temp_environment()
    out_lexical = _lexical_output_path(repo, args.out)
    _output_available(out_lexical)
    out = out_lexical.resolve()
    commit = validate_clean_repo(repo)
    mutants = guarded_mutants if guarded_mutants is not None else _load_mutants(args.mutants, repo)
    if not mutants:
        raise MutationError("mutation manifest is empty")
    if len(mutants) > args.max_mutants:
        raise MutationError(f"{len(mutants)} mutants exceed --max-mutants={args.max_mutants}; keep the gate bounded")
    ids = [mutant.id for mutant in mutants]
    if len(set(ids)) != len(ids):
        raise MutationError("mutant ids must be unique")
    configure_argv = split_command(args.configure_command) if args.configure_command else None
    work_parent_arg = Path(args.work_root) if args.work_root else None
    work_parent = ((repo / work_parent_arg) if work_parent_arg is not None and not work_parent_arg.is_absolute()
                   else work_parent_arg)
    if work_parent is not None:
        work_parent = work_parent.resolve()
    if work_parent:
        work_parent.mkdir(parents=True, exist_ok=True)

    preflight: dict[str, Any] = {"explicit_temp_directories": temp_preflight}
    if manifest_digests is not None:
        preflight["mutant_manifests"] = [
            {"path": str((repo / Path(path)).resolve()), "sha256": digest.lower()}
            for path, digest in zip(args.mutants, manifest_digests)]
    argv_files = {name: source for name, source in
                  (("build", build_argv_file), ("test", test_argv_file))
                  if source is not None}
    if argv_files:
        preflight["argv_files"] = argv_files
    report: dict[str, Any] = {
        "schema_version": 1,
        "status": "infrastructure_error",
        "git": {"commit": commit, "root": str(repo)},
        "limits": {"max_mutants": args.max_mutants, "timeout_seconds": args.timeout},
        "commands": {"configure": configure_argv, "build": build_argv, "test": test_argv},
        "preflight": preflight,
        "mutants": [],
    }
    report_path = out / "mutation-report.json"
    out.mkdir(parents=True, exist_ok=True)
    log_root = out / "command-logs"
    log_root.mkdir(parents=True, exist_ok=True)
    _checkpoint(report_path, report, "preflight")
    command_serial = 0

    def next_log(phase: str, label: str = "") -> Path:
        nonlocal command_serial
        command_serial += 1
        return _command_log(log_root, command_serial, phase, label)

    persistent_path = (_persistent_worktree_path(repo, args.persistent_worktree)
                       if args.persistent_worktree else None)
    with _mutation_worktree(repo, commit, work_parent, persistent_path,
                        report_path, report, args.reuse_detached_checkout) as worktree:
        build_dir = (worktree / args.build_dir).resolve()
        if configure_argv:
            configure = run_command(_expand(configure_argv, root=worktree, build=build_dir),
                                    cwd=worktree, timeout=args.timeout,
                                    output_file=next_log("configure"))
            report["configure"] = configure.json()
            if args.reuse_detached_checkout:
                _verify_reuse_checkout(repo, commit)
            _checkpoint(report_path, report, "configure")
            if configure.timed_out or configure.returncode != 0:
                report["status"] = "configure_timeout" if configure.timed_out else "configure_error"
                _checkpoint(report_path, report, "configure_failed")
                return 2
        build = _expand(build_argv, root=worktree, build=(worktree / args.build_dir).resolve())
        baseline_build = run_command(build, cwd=worktree, timeout=args.timeout,
                                     output_file=next_log("baseline-build"))
        report["baseline_build"] = baseline_build.json()
        if args.reuse_detached_checkout:
            _verify_reuse_checkout(repo, commit)
        _checkpoint(report_path, report, "baseline_build")
        if baseline_build.timed_out or baseline_build.returncode != 0:
            report["status"] = _classify_build(baseline_build)
            _checkpoint(report_path, report, "baseline_build_failed")
            return 2
        # Validate each manifest filter against the unmutated build.  A
        # broad suite baseline can fail for unrelated reasons while still
        # hiding whether the selected test actually ran.
        filters = sorted(set(mutant.test_filter for mutant in mutants))
        baseline_tests: list[dict[str, Any]] = []
        baseline_counts: dict[str, int] = {}
        for test_filter in filters:
            baseline_mutant = Mutant("baseline", "", "x", "y", "", test_filter)
            selected = _test_command(test_argv, baseline_mutant)
            baseline_test = run_command(_expand(selected, root=worktree,
                                                 build=(worktree / args.build_dir).resolve()),
                                        cwd=worktree, timeout=args.timeout,
                                        output_file=next_log("baseline-test", test_filter))
            if args.reuse_detached_checkout:
                _verify_reuse_checkout(repo, commit)
            log_evidence = _result_log_evidence(baseline_test)
            evidence = (log_evidence.test_count if log_evidence is not None
                        else test_evidence(baseline_test.stdout + baseline_test.stderr))
            baseline_counts[test_filter] = evidence
            classification = _classify_test(baseline_test, evidence,
                                            log_evidence=log_evidence)
            baseline_tests.append({
                "filter": test_filter,
                "test_evidence": evidence,
                "classification": classification,
                "result": baseline_test.json(),
            })
            if log_evidence is not None:
                baseline_tests[-1]["log_evidence"] = asdict(log_evidence)
            report["baseline_tests"] = baseline_tests
            _checkpoint(report_path, report, f"baseline_test:{test_filter or 'all'}")
            if classification != "survived":
                report["status"] = ("baseline_test_timeout" if
                                     (log_evidence is not None and log_evidence.timed_out) or
                                     _test_timed_out(baseline_test) else
                                     "baseline_test_failed" if evidence > 0 else
                                     "baseline_no_evidence")
                _checkpoint(report_path, report, "baseline_test_failed")
                return 2
        report["baseline_tests"] = baseline_tests
        _checkpoint(report_path, report, "baseline_complete")

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
                mutant_build = run_command(build, cwd=worktree, timeout=args.timeout,
                                           output_file=next_log("mutant-build", mutant.id))
                entry["build"] = mutant_build.json()
                if mutant_build.timed_out or mutant_build.returncode != 0:
                    entry["status"] = _classify_build(mutant_build)
                    report["mutants"].append(entry)
                    _checkpoint(report_path, report, f"mutant:{mutant.id}:build")
                    continue
                selected_test = _test_command(test_argv, mutant)
                mutant_test = run_command(_expand(selected_test, root=worktree,
                                                   build=(worktree / args.build_dir).resolve()),
                                          cwd=worktree, timeout=args.timeout,
                                          output_file=next_log("mutant-test", mutant.id))
                log_evidence = _result_log_evidence(mutant_test)
                mutant_evidence = (log_evidence.test_count if log_evidence is not None
                                   else test_evidence(mutant_test.stdout + mutant_test.stderr))
                entry["test"] = mutant_test.json()
                entry["test_evidence"] = mutant_evidence
                if log_evidence is not None:
                    entry["log_evidence"] = asdict(log_evidence)
                entry["expected_test_count"] = baseline_counts[mutant.test_filter]
                entry["status"] = _classify_test(
                    mutant_test, mutant_evidence, baseline_counts[mutant.test_filter],
                    log_evidence=log_evidence)
                report["mutants"].append(entry)
                _checkpoint(report_path, report, f"mutant:{mutant.id}:test")
            except MutationError as exc:
                entry["status"] = "mutation_error"
                entry["error"] = str(exc)
                report["mutants"].append(entry)
                _checkpoint(report_path, report, f"mutant:{mutant.id}:error")
            finally:
                if path is not None and original is not None:
                    path.write_bytes(original)
                    restored = path.read_bytes() == original
                    entry["restoration"] = {"path": str(path), "restored": restored,
                                            "sha256": hashlib.sha256(original).hexdigest()}
                    if not restored:
                        raise MutationError(f"failed to restore mutant source: {path}")
                if args.reuse_detached_checkout or persistent_path is not None or (worktree / ".git").exists():
                    try:
                        if args.reuse_detached_checkout:
                            _verify_reuse_checkout(repo, commit)
                        else:
                            _verify_persistent_worktree(repo, worktree, commit)
                        entry["postflight_tracked_clean"] = True
                    except MutationError as exc:
                        entry["postflight_tracked_clean"] = False
                        entry["postflight_error"] = str(exc)
                        report["status"] = "postflight_dirty"
                        _checkpoint(report_path, report, f"mutant:{mutant.id}:postflight_error")
                        raise
    statuses = [entry.get("status") for entry in report["mutants"]]
    report["counts"] = {status: statuses.count(status) for status in sorted(set(statuses))}
    if report.get("cleanup_failed"):
        report["status"] = "cleanup_failed"
        exit_code = 2
    elif any(status not in {"killed", "survived"} for status in statuses):
        report["status"] = "infrastructure_error"
        exit_code = 2
    elif any(status == "survived" for status in statuses):
        report["status"] = "survivors"
        exit_code = 1
    else:
        report["status"] = "pass"
        exit_code = 0
    report["checkpoint"] = {"state": "complete", "phase": "complete"}
    _write_report(report_path, report)
    return exit_code


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    defaults = sub.add_parser("list-defaults", help="print the bounded built-in mutants as JSON")
    defaults.set_defaults(list_defaults=True)
    run = sub.add_parser("run", help="validate baseline and run bounded mutants")
    run.add_argument("--root", default=".")
    run.add_argument("--build-dir", default="build")
    run.add_argument("--configure-command", help="optional CMake configure argv before the baseline build")
    build_command = run.add_mutually_exclusive_group(required=True)
    build_command.add_argument("--build-command", help="build argv as a shell-like command string")
    build_command.add_argument("--build-argv-file",
                               help="JSON argv array file, relative to --root unless absolute")
    test_command = run.add_mutually_exclusive_group(required=True)
    test_command.add_argument("--test-command", help="test argv as a shell-like command string")
    test_command.add_argument("--test-argv-file",
                              help="JSON argv array file, relative to --root unless absolute")
    run.add_argument("--mutants", action="append",
                     help="JSON mutant manifest; repeat to share one build across manifests (default: three Flow smoke mutants)")
    run.add_argument("--mutants-sha256", action="append",
                     help="require a reviewed manifest digest before repository/build work; repeat in --mutants order")
    run.add_argument("--max-mutants", type=int, default=DEFAULT_MAX_MUTANTS)
    run.add_argument("--out", default="build/slicer-mutation")
    run.add_argument("--work-root", help="parent directory for disposable worktree")
    run.add_argument("--persistent-worktree",
                     help="opt-in harness-owned detached worktree retained between runs; build may be mutant")
    run.add_argument("--reuse-detached-checkout", action="store_true",
                     help="reuse this clean detached checkout in place; build outputs may remain mutant")
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
            root = Path(args.root).resolve()
            out_lexical = _lexical_output_path(root, args.out)
            report_path = out_lexical / "mutation-report.json"
            output_is_absent_or_empty = not out_lexical.exists() and not out_lexical.is_symlink()
            if not out_lexical.is_symlink() and out_lexical.exists() and out_lexical.is_dir():
                try:
                    next(out_lexical.iterdir())
                except StopIteration:
                    output_is_absent_or_empty = True
            if not report_path.exists() and output_is_absent_or_empty:
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
