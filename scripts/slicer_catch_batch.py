#!/usr/bin/env python3
"""Opt-in Catch2 batch executor for a live CTest selection.

This candidate deliberately supports only the simple registration shape in
which CTest invokes one Catch executable with exactly one test name. It does
not replace CTest or change the default coverage runner.
"""

from __future__ import annotations

from collections import Counter
from contextlib import contextmanager, nullcontext
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET
from typing import Any, Callable, Mapping, Sequence


from slicer_process import run_recorded


class BatchValidationError(ValueError):
    """CTest/Catch metadata cannot be represented without losing semantics."""


@dataclass(frozen=True)
class SelectedCase:
    name: str
    executable: Path
    working_directory: Path
    labels: tuple[str, ...]
    processors: int
    processor_affinity: bool
    original_executable: str


@dataclass(frozen=True)
class Batch:
    executable: Path
    working_directory: Path
    processors: int
    processor_affinity: bool
    cases: tuple[SelectedCase, ...]


class LinuxAffinityController:
    """Apply a verified child-inherited CPU mask and restore the caller mask."""

    @staticmethod
    def available() -> frozenset[int]:
        if sys.platform != "linux" or not hasattr(os, "sched_getaffinity") or \
                not hasattr(os, "sched_setaffinity"):
            raise BatchValidationError(
                "PROCESSOR_AFFINITY requires Linux sched_getaffinity/sched_setaffinity support")
        return frozenset(os.sched_getaffinity(0))

    @contextmanager
    def limit(self, requested: Sequence[int]):
        before = self.available()
        wanted = frozenset(requested)
        if not wanted or not wanted <= before:
            raise BatchValidationError(
                f"requested CPU subset {sorted(wanted)!r} is not contained in caller mask {sorted(before)!r}")
        info: dict[str, Any] = {"caller_mask_before": sorted(before), "requested_mask": sorted(wanted)}
        try:
            os.sched_setaffinity(0, set(wanted))
            applied = frozenset(os.sched_getaffinity(0))
            info["applied_mask"] = sorted(applied)
            if applied != wanted:
                raise BatchValidationError(
                    f"kernel applied CPU mask {sorted(applied)!r}, expected {sorted(wanted)!r}")
            info["applied_and_verified"] = True
            yield info
        finally:
            os.sched_setaffinity(0, set(before))
            restored = frozenset(os.sched_getaffinity(0))
            info["caller_mask_restored"] = sorted(restored)
            info["restoration_verified"] = restored == before
            if restored != before:
                raise BatchValidationError(
                    f"failed to restore caller CPU mask: got {sorted(restored)!r}, "
                    f"expected {sorted(before)!r}")


_ALLOWED_PROPERTIES = {
    "LABELS", "PROCESSORS", "PROCESSOR_AFFINITY", "SKIP_RETURN_CODE",
    "WORKING_DIRECTORY",
}
_AUTOMAKE_STATUS = {"PASS", "XFAIL", "SKIP", "FAIL", "XPASS", "ERROR"}
_AUTOMAKE_PREFIX = ":test-result:"
_CATCH_FAILURE_EXIT = 42
_CATCH_ALL_SKIPPED_EXIT = 4


def validate_batch_timeout(timeout_seconds: float) -> float:
    if isinstance(timeout_seconds, bool) or not isinstance(timeout_seconds, (int, float)) or \
            not math.isfinite(timeout_seconds) or timeout_seconds <= 0:
        raise BatchValidationError("batch timeout must be a finite positive number")
    return float(timeout_seconds)


def validate_ctest_argv(argv: Sequence[str]) -> list[str]:
    """Validate an explicit CTest argv while preserving all selection options."""
    command = list(argv)
    if not command or not all(isinstance(arg, str) and arg for arg in command):
        raise BatchValidationError("CTest argv must be a nonempty sequence of nonempty strings")
    executable_name = command[0].replace("\\", "/").rsplit("/", 1)[-1].lower()
    if executable_name not in {"ctest", "ctest.exe"}:
        raise BatchValidationError("selection source must be the real CTest executable")
    whole_batch_only = {
        "--timeout", "--stop-on-failure", "--stop-time", "--repeat",
        "--repeat-until-fail", "--repeat-until-pass",
    }
    for argument in command[1:]:
        if argument in {"-N", "--show-only"} or argument.startswith("--show-only="):
            raise BatchValidationError("CTest argv must not already contain --show-only")
        if argument == "--output-junit" or argument.startswith("--output-junit="):
            raise BatchValidationError("CTest argv must not already contain --output-junit")
        if argument in whole_batch_only or any(
                argument.startswith(option + "=") for option in whole_batch_only):
            raise BatchValidationError(
                f"CTest execution option {argument} has no equivalent in a whole-batch Catch run")
    if "--no-tests" in command:
        raise BatchValidationError("use --no-tests=error, not bare --no-tests")
    for argument in command:
        if argument.startswith("--no-tests=") and argument != "--no-tests=error":
            raise BatchValidationError("only --no-tests=error is supported")
    if not any(arg == "--test-dir" or arg.startswith("--test-dir=") for arg in command):
        raise BatchValidationError("CTest argv must identify its registration directory with --test-dir")
    if "--no-tests=error" not in command:
        command.append("--no-tests=error")
    return command


def ctest_selection_argv(argv: Sequence[str]) -> list[str]:
    """Drop only CTest execution/output controls; retain all selection options."""
    command = list(argv)
    if not command:
        raise BatchValidationError("CTest argv is empty")
    value_options = {"--parallel", "-j", "--output-log", "--timeout"}
    no_value_options = {"--output-on-failure", "--verbose", "-V", "--quiet", "--progress",
                        "--debug", "--debug-output", "--schedule-random"}
    result = [command[0]]
    index = 1
    while index < len(command):
        argument = command[index]
        if argument in value_options:
            if index + 1 < len(command) and not command[index + 1].startswith("-"):
                index += 2
                continue
            if argument in {"--output-log", "--timeout"}:
                raise BatchValidationError(f"CTest {argument} is missing its value")
            index += 1
            continue
        if argument in no_value_options:
            index += 1
            continue
        if any(argument.startswith(option + "=") for option in value_options - {"-j"}):
            index += 1
            continue
        if argument.startswith("-j") and len(argument) > 2 and argument[2:].isdigit():
            index += 1
            continue
        result.append(argument)
        index += 1
    return validate_ctest_argv(result)


def _property_map(test: Mapping[str, Any]) -> dict[str, Any]:
    raw_properties = test.get("properties", [])
    if not isinstance(raw_properties, list):
        raise BatchValidationError(f"CTest properties are not a list for {test.get('name')!r}")
    result: dict[str, Any] = {}
    for row in raw_properties:
        if not isinstance(row, dict) or not {"name", "value"} <= set(row):
            raise BatchValidationError(f"malformed CTest property for {test.get('name')!r}")
        name = row["name"]
        if not isinstance(name, str) or not name:
            raise BatchValidationError(f"CTest property has no name for {test.get('name')!r}")
        if name in result:
            raise BatchValidationError(f"duplicate CTest property {name!r} for {test.get('name')!r}")
        if name not in _ALLOWED_PROPERTIES:
            raise BatchValidationError(f"unsupported CTest property {name!r} for {test.get('name')!r}")
        result[name] = row["value"]
    return result


def _resolve_existing(path_value: str, *, field: str, directory: bool = False) -> Path:
    candidate = Path(path_value)
    if not candidate.is_absolute():
        raise BatchValidationError(f"CTest {field} must be an absolute path: {path_value!r}")
    try:
        resolved = candidate.resolve(strict=True)
    except OSError as exc:
        raise BatchValidationError(f"CTest {field} does not exist: {path_value!r}") from exc
    if directory and not resolved.is_dir():
        raise BatchValidationError(f"CTest {field} is not a directory: {path_value!r}")
    if not directory and not resolved.is_file():
        raise BatchValidationError(f"CTest {field} is not a file: {path_value!r}")
    return resolved


def _is_beneath(path: Path, root: Path) -> bool:
    try:
        path.relative_to(root)
        return True
    except ValueError:
        return False


def _validate_config(discovery: Mapping[str, Any], ctest_argv: Sequence[str]) -> str | None:
    tests = discovery.get("tests", [])
    configs = [test.get("config") for test in tests if isinstance(test, dict)]
    present = [config for config in configs if config is not None]
    if any(not isinstance(config, str) or not config for config in present):
        raise BatchValidationError("CTest test configuration must be nonempty text")
    if present and len(present) != len(configs):
        raise BatchValidationError("CTest discovery mixes configured and unconfigured test records")
    unique_configs = set(present)
    if len(unique_configs) > 1:
        raise BatchValidationError(f"CTest discovery mixes configurations: {sorted(unique_configs)!r}")
    discovered = next(iter(unique_configs), None)
    requested: list[str] = []
    argv = list(ctest_argv)
    index = 1
    while index < len(argv):
        argument = argv[index]
        if argument in {"-C", "--build-config"}:
            if index + 1 >= len(argv):
                raise BatchValidationError(f"CTest {argument} has no value")
            requested.append(argv[index + 1])
            index += 2
            continue
        if argument.startswith("--build-config="):
            requested.append(argument.split("=", 1)[1])
        elif argument.startswith("-C") and len(argument) > 2:
            requested.append(argument[2:])
        index += 1
    if len(set(requested)) > 1:
        raise BatchValidationError("CTest argv requests multiple build configurations")
    if requested and discovered is not None and requested[0] != discovered:
        raise BatchValidationError(
            f"CTest discovery configuration {discovered!r} does not match argv {requested[0]!r}")
    return discovered


def plan_batches(discovery: Mapping[str, Any], *, build_dir: Path | str,
                 ctest_argv: Sequence[str] | None = None) -> list[Batch]:
    """Create ordered executable/cwd batches from CTest JSON only."""
    build_root = Path(build_dir).resolve(strict=True)
    if not build_root.is_dir():
        raise BatchValidationError(f"build directory is not a directory: {build_root}")
    tests = discovery.get("tests")
    if not isinstance(tests, list) or not tests:
        raise BatchValidationError("CTest discovery selected no tests or has no tests array")
    if ctest_argv is not None:
        _validate_config(discovery, ctest_argv)

    ordered: list[SelectedCase] = []
    names: set[str] = set()
    for index, test in enumerate(tests):
        if not isinstance(test, dict):
            raise BatchValidationError(f"CTest test #{index} is not an object")
        name = test.get("name")
        if not isinstance(name, str) or not name or name in names:
            raise BatchValidationError(f"CTest test #{index} has an empty or duplicate identity")
        if any(ord(char) < 32 or ord(char) == 127 for char in name):
            raise BatchValidationError(f"CTest identity contains control characters: {name!r}")
        names.add(name)

        command = test.get("command")
        if not isinstance(command, list) or len(command) != 2 or \
                not all(isinstance(part, str) and part for part in command):
            raise BatchValidationError(
                f"unsupported CTest command shape for {name!r}: expected executable plus one name")
        if command[1] != escape_ctest_catch_argument(name):
            raise BatchValidationError(
                f"CTest command argument is not the known Catch literal escape of registration {name!r}")
        executable = _resolve_existing(command[0], field="test executable")
        if not os.access(executable, os.X_OK):
            raise BatchValidationError(f"CTest test executable is not executable: {executable}")

        properties = _property_map(test)
        raw_labels = properties.get("LABELS", [])
        if not isinstance(raw_labels, list) or not all(isinstance(label, str) for label in raw_labels):
            raise BatchValidationError(f"CTest LABELS must be a string list for {name!r}")
        labels = tuple(raw_labels)
        if len(set(labels)) != len(labels):
            raise BatchValidationError(f"CTest LABELS contain duplicates for {name!r}")

        processors = properties.get("PROCESSORS", 1)
        if isinstance(processors, bool) or not isinstance(processors, int) or processors < 1:
            raise BatchValidationError(f"CTest PROCESSORS must be a positive integer for {name!r}")
        affinity = properties.get("PROCESSOR_AFFINITY", False)
        if not isinstance(affinity, bool):
            raise BatchValidationError(f"PROCESSOR_AFFINITY must be boolean for {name!r}")
        if properties.get("SKIP_RETURN_CODE") != 4:
            raise BatchValidationError(f"Catch batches require CTest SKIP_RETURN_CODE=4 for {name!r}")
        if "WORKING_DIRECTORY" not in properties or not isinstance(properties["WORKING_DIRECTORY"], str):
            raise BatchValidationError(f"CTest WORKING_DIRECTORY is required for {name!r}")
        working_directory = _resolve_existing(
            properties["WORKING_DIRECTORY"], field="WORKING_DIRECTORY", directory=True)
        if not _is_beneath(working_directory, build_root):
            raise BatchValidationError(
                f"CTest WORKING_DIRECTORY escapes the supplied build directory for {name!r}")

        ordered.append(SelectedCase(name, executable, working_directory, labels,
                                    processors, affinity, command[0]))

    groups: dict[tuple[Path, Path, int, bool], list[SelectedCase]] = {}
    for case in ordered:
        groups.setdefault((case.executable, case.working_directory,
                           case.processors, case.processor_affinity), []).append(case)
    return [Batch(exe, cwd, processors, affinity, tuple(cases))
            for (exe, cwd, processors, affinity), cases in groups.items()]


def escape_catch_test_name(name: str) -> str:
    """Encode one exact Catch test spec using the accepted batch syntax."""
    if not isinstance(name, str) or not name:
        raise BatchValidationError("Catch test name must be nonempty text")
    if any(ord(char) < 32 or ord(char) == 127 for char in name):
        raise BatchValidationError(f"Catch test name contains a control character: {name!r}")
    escaped = name.replace("\\", "\\\\")
    escaped = escaped.replace(",", "\\,").replace('"', '\\"').replace("'", "\\'")
    return f'"{escaped}"'


def escape_ctest_catch_argument(name: str) -> str:
    """Mirror CatchAddTests.cmake's escaped command argument in CTest JSON."""
    if not isinstance(name, str) or not name:
        raise BatchValidationError("CTest registered test name must be nonempty text")
    if any(ord(char) < 32 or ord(char) == 127 for char in name):
        raise BatchValidationError(f"CTest registered test name contains a control character: {name!r}")
    escaped = name.replace("\\", "\\\\")
    return escaped.replace(",", "\\,").replace("[", "\\[").replace("]", "\\]")


def _write_exclusive(path: Path, data: bytes) -> None:
    with path.open("xb") as stream:
        stream.write(data)
        stream.flush()
        os.fsync(stream.fileno())


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _profile_file_inventory(root: Path) -> list[dict[str, str]]:
    extensions = {".gcda", ".profraw"}
    return [
        {"path": str(path), "sha256": _sha256(path)}
        for path in sorted(root.rglob("*"))
        if path.is_file() and path.suffix in extensions
    ]


def _json_from_log(path: Path, *, description: str) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BatchValidationError(f"{description} output is not one clean JSON document: {exc}") from exc


def _parse_automake(path: Path) -> list[tuple[str, str]]:
    try:
        lines = path.read_text(encoding="utf-8", errors="strict").splitlines()
    except (OSError, UnicodeDecodeError) as exc:
        raise BatchValidationError(f"cannot read Automake output: {exc}") from exc
    records: list[tuple[str, str]] = []
    for line in lines:
        if not line.startswith(_AUTOMAKE_PREFIX):
            continue
        payload = line[len(_AUTOMAKE_PREFIX):].lstrip()
        status, separator, name = payload.partition(" ")
        if not separator or status not in _AUTOMAKE_STATUS or not name:
            raise BatchValidationError(f"malformed or unsupported Automake result line: {line!r}")
        records.append((status, name))
    return records


def validate_automake_records(cases: Sequence[SelectedCase], records: Sequence[tuple[str, str]],
                              returncode: int) -> dict[str, str]:
    expected = {case.name: case for case in cases}
    found: dict[str, str] = {}
    for status, name in records:
        if status not in _AUTOMAKE_STATUS:
            raise BatchValidationError(f"unsupported Automake status {status!r}")
        if name not in expected:
            raise BatchValidationError(f"Automake reported an unexpected test identity: {name!r}")
        if name in found:
            raise BatchValidationError(f"Automake reported a duplicate test identity: {name!r}")
        found[name] = status
    missing = sorted(expected.keys() - found.keys())
    if missing:
        raise BatchValidationError(f"Automake omitted selected test identities: {missing[:8]!r}")

    normalized: dict[str, str] = {}
    for name, status in found.items():
        labels = set(expected[name].labels)
        expects_failure = "!shouldfail" in labels or "!mayfail" in labels
        if status == "XFAIL":
            if not expects_failure:
                raise BatchValidationError(f"unexpected XFAIL for untagged CTest case {name!r}")
            normalized[name] = "PASS"
        elif status == "PASS":
            if "!shouldfail" in labels:
                normalized[name] = "FAIL"
            else:
                normalized[name] = "PASS"
        elif status == "SKIP":
            normalized[name] = "SKIP"
        elif status in {"FAIL", "XPASS"}:
            normalized[name] = "FAIL"
        elif status == "ERROR":
            normalized[name] = "ERROR"
        else:  # pragma: no cover - status set is exhaustive
            raise BatchValidationError(f"unhandled Automake status {status!r}")

    raw_statuses = set(found.values())
    normalized_failures = any(value in {"FAIL", "ERROR"} for value in normalized.values())
    all_skipped = bool(found) and raw_statuses == {"SKIP"}
    if returncode == _CATCH_ALL_SKIPPED_EXIT:
        if not all_skipped:
            raise BatchValidationError("Catch exit 4 is valid only when every selected case is SKIP")
    elif returncode == _CATCH_FAILURE_EXIT:
        if not normalized_failures:
            raise BatchValidationError("Catch failure exit 42 has no Automake failure/error/XPASS row")
    elif returncode == 0:
        if all_skipped:
            raise BatchValidationError("all-skipped Catch batch must exit 4, not 0")
        if normalized_failures:
            raise BatchValidationError("Catch exited 0 despite Automake failure/error/XPASS status")
    else:
        raise BatchValidationError(f"unexpected Catch process exit status {returncode}")
    return normalized


def _isolated_profile_env(parent_env: Mapping[str, str], profile_dir: Path) -> dict[str, str]:
    """Route any GCC/LLVM discovery writes away from the measured test epoch."""
    gcov_dir = profile_dir / "gcov"
    llvm_dir = profile_dir / "llvm"
    gcov_dir.mkdir(mode=0o700)
    llvm_dir.mkdir(mode=0o700)
    env = dict(parent_env)
    env["GCOV_PREFIX"] = str(gcov_dir)
    env.setdefault("GCOV_PREFIX_STRIP", "0")
    env["LLVM_PROFILE_FILE"] = str(llvm_dir / "%p-%m.profraw")
    return env


def _listing_names(payload: Any) -> list[str]:
    try:
        raw_tests = payload["listings"]["tests"]
    except (KeyError, TypeError) as exc:
        raise BatchValidationError("Catch --list-tests JSON has no listings.tests array") from exc
    if not isinstance(raw_tests, list):
        raise BatchValidationError("Catch --list-tests listings.tests is not an array")
    result: list[str] = []
    for row in raw_tests:
        if not isinstance(row, dict) or not isinstance(row.get("name"), str):
            raise BatchValidationError("Catch --list-tests contains a malformed test record")
        result.append(row["name"])
    return result


def _verify_preflight(batch: Batch, *, env: dict[str, str], out_dir: Path,
                      timeout: float, executor: Callable[..., subprocess.CompletedProcess],
                      ordinal: int, binary_sha_before: str, cpu_mask: Sequence[int] | None,
                      affinity_controller: Any) -> dict[str, Any]:
    batch_dir = out_dir / f"batch-{ordinal:02d}"
    batch_dir.mkdir(mode=0o700)
    input_path = batch_dir / "catch-input.txt"
    selected_names = [case.name for case in batch.cases]
    input_bytes = ("\n".join(escape_catch_test_name(name) for name in selected_names) + "\n").encode("utf-8")
    _write_exclusive(input_path, input_bytes)
    list_log = batch_dir / "catch-list.log"
    list_receipt = batch_dir / "catch-list.receipt.json"
    argv = [str(batch.executable), "--input-file", str(input_path),
            "--list-tests", "--reporter", "json"]
    affinity_context = affinity_controller.limit(cpu_mask) if cpu_mask else nullcontext(None)
    try:
        with affinity_context as cpu_affinity:
            result = executor(argv, cwd=batch.working_directory, env=env,
                              timeout=timeout, log_path=list_log, receipt_path=list_receipt)
    except subprocess.TimeoutExpired as exc:
        raise BatchValidationError(f"Catch list preflight timed out for {batch.executable}: {exc}") from exc
    if result.returncode != 0:
        raise BatchValidationError(
            f"Catch list preflight exited {result.returncode} for {batch.executable}")
    listing = _listing_names(_json_from_log(list_log, description="Catch list preflight"))
    listed_counts = Counter(listing)
    expected_counts = Counter(selected_names)
    missing = sorted((expected_counts - listed_counts).elements())
    unexpected = sorted((listed_counts - expected_counts).elements())
    duplicate = sorted(name for name, count in listed_counts.items()
                       if count > 1 and name in expected_counts)
    if missing or unexpected or duplicate or listed_counts != expected_counts:
        raise BatchValidationError(
            f"Catch list preflight did not exactly match CTest selection; missing={missing[:8]!r}, "
            f"unexpected={unexpected[:8]!r}, duplicate={duplicate[:8]!r}")
    binary_sha_after = _sha256(batch.executable)
    if binary_sha_after != binary_sha_before:
        raise BatchValidationError(f"binary changed during list preflight: {batch.executable}")
    return {
        "batch_dir": str(batch_dir), "selected_count": len(selected_names),
        "selected_names": selected_names, "listed_count": len(listing),
        "listed_selected_counts": {name: listed_counts[name] for name in selected_names},
        "list_argv": argv, "list_log": str(list_log), "list_receipt": str(list_receipt),
        "list_log_sha256": _sha256(list_log), "catch_input": str(input_path),
        "catch_input_sha256": _sha256(input_path), "binary_sha256_before_preflight": binary_sha_before,
        "binary_sha256_after_preflight": binary_sha_after,
        "preflight_profile_files": _profile_file_inventory(Path(env["GCOV_PREFIX"]).parent),
        "preflight_cpu_affinity": cpu_affinity,
    }


def _write_ctest_junit(path: Path, cases: Sequence[SelectedCase],
                       normalized_statuses: Mapping[str, str], *, batch_by_name: Mapping[str, str]) -> None:
    statuses = [normalized_statuses[case.name] for case in cases]
    failures = sum(status == "FAIL" for status in statuses)
    errors = sum(status == "ERROR" for status in statuses)
    skipped = sum(status == "SKIP" for status in statuses)
    suite = ET.Element("testsuite", {
        "name": "CTest Catch batch", "tests": str(len(cases)),
        "failures": str(failures), "errors": str(errors), "skipped": str(skipped),
    })
    for case in cases:
        status = normalized_statuses[case.name]
        testcase = ET.SubElement(suite, "testcase", {
            "name": case.name, "classname": "CTest", "batch": batch_by_name[case.name],
        })
        if status == "SKIP":
            ET.SubElement(testcase, "skipped", {
                "message": "Catch Automake reported SKIP (CTest SKIP_RETURN_CODE=4)"})
        elif status == "FAIL":
            failure = ET.SubElement(testcase, "failure", {
                "type": "CatchFailure", "message": "Catch Automake reported a failure status"})
            failure.text = "See the referenced raw Catch JUnit and Automake log for assertions."
        elif status == "ERROR":
            error = ET.SubElement(testcase, "error", {
                "type": "CatchError", "message": "Catch Automake reported ERROR"})
            error.text = "See the referenced raw Catch JUnit and Automake log for diagnostics."
    tree = ET.ElementTree(ET.Element("testsuites"))
    tree.getroot().append(suite)
    ET.indent(tree, space="  ")
    with path.open("xb") as stream:
        tree.write(stream, encoding="utf-8", xml_declaration=True)
        stream.write(b"\n")
        stream.flush()
        os.fsync(stream.fileno())


def run_discovered_batches(discovery: Mapping[str, Any], *, ctest_argv: Sequence[str],
                           build_dir: Path | str, out_dir: Path | str,
                           batch_timeout_seconds: float,
                           discovery_argv: Sequence[str], discovery_sha256: str,
                           discovery_raw_json: bytes | None = None,
                           parent_env: Mapping[str, str] | None = None,
                           executor: Callable[..., subprocess.CompletedProcess] | None = None,
                           affinity_controller: Any | None = None) -> dict[str, Any]:
    """Preflight, then run each executable/cwd batch serially with exact names.

    The caller must supply discovery obtained by executing the same validated
    CTest argv plus ``--show-only=json-v1``. The CLI below provides that path;
    the function is also the clean integration seam for the gcov backend.
    """
    timeout = validate_batch_timeout(batch_timeout_seconds)
    ctest_command = validate_ctest_argv(ctest_argv)
    expected_discovery_argv = [*ctest_command, "--show-only=json-v1"]
    if list(discovery_argv) != expected_discovery_argv:
        raise BatchValidationError("discovery argv is not the validated CTest selection plus --show-only=json-v1")
    if not re.fullmatch(r"[0-9a-f]{64}", discovery_sha256):
        raise BatchValidationError("discovery SHA-256 must be lowercase hexadecimal")
    raw_discovery = discovery_raw_json
    if raw_discovery is None:
        raw_discovery = json.dumps(discovery, ensure_ascii=False, indent=2).encode("utf-8") + b"\n"
    if hashlib.sha256(raw_discovery).hexdigest() != discovery_sha256:
        raise BatchValidationError("discovery bytes do not match the supplied SHA-256")
    try:
        if json.loads(raw_discovery) != discovery:
            raise BatchValidationError("discovery bytes do not parse to the supplied CTest object")
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise BatchValidationError(f"discovery bytes are invalid JSON: {exc}") from exc
    root = Path(out_dir).resolve()
    root.mkdir(mode=0o700, parents=True, exist_ok=False)
    active_executor = executor or run_recorded
    active_affinity = affinity_controller or LinuxAffinityController()
    env = dict(os.environ if parent_env is None else parent_env)
    batches = plan_batches(discovery, build_dir=build_dir, ctest_argv=ctest_command)
    affinity_batches = [item for item in batches if item.processor_affinity]
    caller_cpu_mask = sorted(active_affinity.available()) if affinity_batches else []
    batch_cpu_masks: dict[tuple[Path, Path, int, bool], list[int] | None] = {}
    for item in batches:
        group_key = (item.executable, item.working_directory,
                     item.processors, item.processor_affinity)
        if not item.processor_affinity:
            batch_cpu_masks[group_key] = None
            continue
        if len(caller_cpu_mask) < item.processors:
            raise BatchValidationError(
                f"CTest cohort for {item.executable.name} requests {item.processors} CPUs, "
                f"caller mask has {len(caller_cpu_mask)}: {caller_cpu_mask!r}")
        batch_cpu_masks[group_key] = caller_cpu_mask[:item.processors]
    _write_exclusive(root / "ctest-discovery.json", raw_discovery)
    preflight_root = root / "preflight-profiles"
    preflight_root.mkdir(mode=0o700)
    runs_root = root / "runs"
    runs_root.mkdir(mode=0o700)

    batch_infos: list[dict[str, Any]] = []
    for ordinal, batch in enumerate(batches, 1):
        profile_dir = preflight_root / f"batch-{ordinal:02d}"
        profile_dir.mkdir(mode=0o700)
        isolated_env = _isolated_profile_env(env, profile_dir)
        binary_sha_before = _sha256(batch.executable)
        group_key = (batch.executable, batch.working_directory,
                     batch.processors, batch.processor_affinity)
        info = _verify_preflight(batch, env=isolated_env, out_dir=runs_root,
                                 timeout=timeout, executor=active_executor,
                                 ordinal=ordinal, binary_sha_before=binary_sha_before,
                                 cpu_mask=batch_cpu_masks[group_key],
                                 affinity_controller=active_affinity)
        info.update({
            "executable": str(batch.executable),
            "original_executable_paths": sorted({case.original_executable for case in batch.cases}),
            "working_directory": str(batch.working_directory),
            "labels": {case.name: list(case.labels) for case in batch.cases},
            "processors": {case.name: case.processors for case in batch.cases},
            "processor_affinity": batch.processor_affinity,
            "batch_cpu_mask": batch_cpu_masks[group_key],
            "preflight_profile_dir": str(profile_dir),
            "preflight_gcov_prefix": isolated_env["GCOV_PREFIX"],
            "preflight_llvm_profile_file": isolated_env["LLVM_PROFILE_FILE"],
        })
        batch_infos.append(info)

    all_cases = [case for batch in batches for case in batch.cases]
    normalized: dict[str, str] = {}
    batch_names: dict[str, str] = {}
    execution_failed = False
    for ordinal, (batch, info) in enumerate(zip(batches, batch_infos), 1):
        batch_key = f"batch-{ordinal:02d}"
        group_key = (batch.executable, batch.working_directory,
                     batch.processors, batch.processor_affinity)
        for case in batch.cases:
            batch_names[case.name] = batch_key
        batch_dir = Path(info["batch_dir"])
        input_path = Path(info["catch_input"])
        raw_junit = batch_dir / "catch-raw.junit.xml"
        automake_log = batch_dir / "catch-automake.log"
        receipt = batch_dir / "catch-batch.receipt.json"
        argv = [
            str(batch.executable), "--input-file", str(input_path),
            "--order", "rand", "--rng-seed", "5",
            "--reporter", f"junit::out={raw_junit}",
            "--reporter", "automake",
        ]
        binary_sha_before = _sha256(batch.executable)
        if binary_sha_before != info["binary_sha256_after_preflight"]:
            raise BatchValidationError(f"binary changed after list preflight: {batch.executable}")
        info["binary_sha256_before_run"] = binary_sha_before
        info["batch_argv"] = argv
        info["batch_timeout_seconds"] = timeout
        info["timeout_scope"] = "whole executable-plus-working-directory Catch process"
        info["batch_log"] = str(automake_log)
        info["batch_receipt"] = str(receipt)
        info["raw_catch_junit"] = str(raw_junit)
        info["parent_profile_environment"] = {
            key: env.get(key) for key in ("GCOV_PREFIX", "GCOV_PREFIX_STRIP", "LLVM_PROFILE_FILE")
            if key in env
        }
        cpu_run_info = None
        cpu_context = active_affinity.limit(batch_cpu_masks[group_key]) \
            if batch_cpu_masks[group_key] else nullcontext(None)
        try:
            with cpu_context as cpu_run_info:
                result = active_executor(argv, cwd=batch.working_directory, env=env,
                                         timeout=timeout, log_path=automake_log,
                                         receipt_path=receipt)
        except subprocess.TimeoutExpired as exc:
            binary_sha_after_timeout = _sha256(batch.executable)
            info.update({"process_outcome": "timed_out", "timed_out": True,
                         "binary_sha256_after_timeout": binary_sha_after_timeout,
                         "binary_unchanged_after_timeout":
                             binary_sha_after_timeout == info["binary_sha256_before_run"],
                         "batch_cpu_affinity": cpu_run_info,
                         "timeout_exception": str(exc)})
            report = {
                "schema": "slicer-catch-batch-run-v1", "status": "incomplete_timeout",
                "ctest_argv": ctest_command, "selected_count": len(all_cases),
                "discovery_argv": list(discovery_argv), "discovery_sha256": discovery_sha256,
                "caller_cpu_mask": caller_cpu_mask,
                "batches": batch_infos, "batch_timeout_seconds": timeout,
                "timeout_semantics": "one whole batch process per executable, working directory, and CPU cohort",
                "junit": None,
            }
            _write_exclusive(root / "batch-report.json",
                             (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8"))
            return report

        binary_sha_after = _sha256(batch.executable)
        info["binary_sha256_after_run"] = binary_sha_after
        if binary_sha_after != binary_sha_before:
            raise BatchValidationError(f"binary changed during Catch batch run: {batch.executable}")
        info["returncode"] = result.returncode
        info["timed_out"] = False
        info["batch_cpu_affinity"] = cpu_run_info
        if not raw_junit.is_file():
            raise BatchValidationError(f"Catch did not produce raw JUnit output: {raw_junit}")
        try:
            ET.parse(raw_junit)
        except (ET.ParseError, OSError) as exc:
            raise BatchValidationError(f"Catch raw JUnit is not valid XML: {raw_junit}: {exc}") from exc
        info["raw_catch_junit_sha256"] = _sha256(raw_junit)
        records = _parse_automake(automake_log)
        batch_normalized = validate_automake_records(batch.cases, records, result.returncode)
        normalized.update(batch_normalized)
        info["automake_rows"] = len(records)
        info["automake_status_counts"] = dict(Counter(status for status, _ in records))
        info["normalized_status_counts"] = dict(Counter(batch_normalized.values()))
        info["normalized_statuses"] = batch_normalized
        info["automake_log_sha256"] = _sha256(automake_log)
        info["process_outcome"] = "exited"
        execution_failed = execution_failed or any(
            status in {"FAIL", "ERROR"} for status in batch_normalized.values())

    junit_path = root / "ctest-junit.xml"
    _write_ctest_junit(junit_path, all_cases, normalized, batch_by_name=batch_names)
    expected_names = Counter(case.name for case in all_cases)
    junit_root = ET.parse(junit_path).getroot()
    actual_names = Counter(node.get("name") for node in junit_root.iter("testcase"))
    if actual_names != expected_names:
        raise BatchValidationError("synthesized CTest JUnit identity inventory is not exact")
    report = {
        "schema": "slicer-catch-batch-run-v1",
        "status": "completed_with_test_failures" if execution_failed else "pass",
        "ctest_argv": ctest_command,
        "discovery_argv": list(discovery_argv), "discovery_sha256": discovery_sha256,
        "selection_source": "live CTest --show-only=json-v1 discovery passed by caller",
        "selected_count": len(all_cases), "selected_names": [case.name for case in all_cases],
        "batch_count": len(batches), "sessions_are_serial": True,
        "order": "Catch randomized order", "rng_seed": 5,
        "batch_timeout_seconds": timeout,
        "timeout_semantics": "one whole batch process per executable, working directory, and CPU cohort; not per-test",
        "caller_cpu_memory_and_environment": "inherited unchanged for test batches; caller must bound resources",
        "caller_cpu_mask": caller_cpu_mask,
        "discovery_and_preflight_profiles": "isolated from the measured test batch profile environment",
        "batches": batch_infos,
        "normalized_status_counts": dict(Counter(normalized.values())),
        "junit": str(junit_path), "junit_sha256": _sha256(junit_path),
        "raw_catch_junit_paths": [info["raw_catch_junit"] for info in batch_infos
                                   if Path(info["raw_catch_junit"]).is_file()],
        "raw_automake_log_paths": [info["batch_log"] for info in batch_infos
                                   if Path(info["batch_log"]).is_file()],
    }
    _write_exclusive(root / "batch-report.json",
                     (json.dumps(report, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    return report


def discover_ctest(ctest_argv: Sequence[str], *, build_dir: Path | str,
                   out_dir: Path | str, discovery_timeout_seconds: float,
                   cwd: Path | str | None = None,
                   parent_env: Mapping[str, str] | None = None,
                   executor: Callable[..., subprocess.CompletedProcess] | None = None
                   ) -> tuple[list[str], dict[str, Any], dict[str, Any]]:
    """Run the exact CTest selection command in an isolated discovery profile epoch."""
    timeout = validate_batch_timeout(discovery_timeout_seconds)
    command = validate_ctest_argv(ctest_argv)
    root = Path(out_dir).resolve()
    root.mkdir(mode=0o700, parents=True, exist_ok=False)
    profiles = root / "discovery-profiles"
    profiles.mkdir(mode=0o700)
    profile_dir = profiles / "ctest"
    profile_dir.mkdir(mode=0o700)
    env = dict(os.environ if parent_env is None else parent_env)
    isolated_env = _isolated_profile_env(env, profile_dir)
    argv = [*command, "--show-only=json-v1"]
    log_path = root / "ctest-discovery.log"
    receipt_path = root / "ctest-discovery.receipt.json"
    active_executor = executor or run_recorded
    discovery_cwd = Path(build_dir if cwd is None else cwd)
    try:
        result = active_executor(argv, cwd=discovery_cwd, env=isolated_env, timeout=timeout,
                                 log_path=log_path, receipt_path=receipt_path)
    except subprocess.TimeoutExpired as exc:
        raise BatchValidationError(f"CTest JSON discovery timed out: {exc}") from exc
    if result.returncode != 0:
        raise BatchValidationError(f"CTest --show-only=json-v1 exited {result.returncode}")
    payload = _json_from_log(log_path, description="CTest discovery")
    if not isinstance(payload, dict):
        raise BatchValidationError("CTest discovery JSON root is not an object")
    # Validate now so malformed selections never reach Catch or gcov.
    plan_batches(payload, build_dir=build_dir, ctest_argv=command)
    raw_path = root / "ctest-discovery.json"
    _write_exclusive(raw_path, log_path.read_bytes())
    receipt = {
        "schema": "slicer-catch-discovery-v1", "argv": argv,
        "cwd": str(discovery_cwd),
        "exit_code": result.returncode, "selected_count": len(payload["tests"]),
        "ctest_json": str(raw_path), "ctest_json_sha256": _sha256(raw_path),
        "log": str(log_path), "receipt": str(receipt_path),
        "discovery_gcov_prefix": isolated_env["GCOV_PREFIX"],
        "discovery_llvm_profile_file": isolated_env["LLVM_PROFILE_FILE"],
        "discovery_profile_files": _profile_file_inventory(profile_dir),
    }
    _write_exclusive(root / "discovery-report.json",
                     (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8"))
    return command, payload, receipt


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build-dir", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--batch-timeout", type=float, required=True,
                        help="timeout for each whole Catch process, not each test case")
    parser.add_argument("--discovery-timeout", type=float, default=120.0)
    parser.add_argument("ctest_argv", nargs=argparse.REMAINDER,
                        help="explicit CTest argv after --; selection flags are preserved")
    args = parser.parse_args(argv)
    ctest_argv = list(args.ctest_argv)
    if ctest_argv and ctest_argv[0] == "--":
        ctest_argv.pop(0)
    try:
        command, discovery, discovery_receipt = discover_ctest(
            ctest_argv, build_dir=args.build_dir,
            out_dir=args.out / "discovery", discovery_timeout_seconds=args.discovery_timeout)
        discovery_path = Path(discovery_receipt["ctest_json"])
        report = run_discovered_batches(
            discovery, ctest_argv=command, build_dir=args.build_dir,
            out_dir=args.out / "batch", batch_timeout_seconds=args.batch_timeout,
            discovery_argv=discovery_receipt["argv"],
            discovery_sha256=discovery_receipt["ctest_json_sha256"],
            discovery_raw_json=discovery_path.read_bytes())
    except (BatchValidationError, OSError, subprocess.SubprocessError) as exc:
        print(f"catch batch rejected: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "selected_count": report["selected_count"],
                      "batch_count": report.get("batch_count"),
                      "junit": report.get("junit")}, sort_keys=True))
    return 0 if report["status"] == "pass" else 2


if __name__ == "__main__":
    raise SystemExit(main())
