#!/usr/bin/env python3
"""Measure LLVM source-based coverage of the real libslic3r objects.

CMake decides which production sources are compiled and writes a manifest per
target (see cmake/SlicerCoverage.cmake). This runner executes the tests with a
private profile directory, merges the profiles, exports LLVM's file summaries
and checks that the result is honest: every compiled production source is
mapped or proven mapping-free, every profile came from one of the measured
executables, and no physical file is counted twice. It never invents coverage
for a file absent from LLVM's export.

Typical invocation, after a coverage build (-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON)::

    python3 scripts/slicer_coverage.py run --measure-only --html \\
      --object build/tests/libslic3r/libslic3r_tests \\
      --object build/tests/fff_print/fff_print_tests \\
      --manifest build/slicer-coverage/libslic3r.sources.txt \\
      --manifest build/slicer-coverage/libslic3r_cgal.sources.txt \\
      --test-command 'ctest --test-dir build/tests -C Release --no-tests=error' \\
      --out build/slicer-coverage-report

Modes:
  --measure-only      record the percentages; any valid percentage passes.
  --threshold N       additionally require every --metric to exceed N percent.
Without either option the run measures only; percentages never gate by default.

Exit status: 0 the evidence is valid (and above the threshold, if one was
given); 1 the evidence is valid but below --threshold; 2 the evidence is not
valid: tests failed or never ran, profiles are missing or foreign, mappings are
missing, or a tool failed.

The test command is run without a shell; a string is split with shlex.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import mmap
import os
from pathlib import Path
import re
import shlex
import shutil
import struct
import subprocess
import sys
import time
from typing import Any, Iterable, Sequence

from slicer_process import run_captured

REPORT_SCHEMA = "orca-slicer-coverage-report"
REPORT_SCHEMA_VERSION = 2
BACKEND = "llvm-source-based"
METRICS = ("lines", "functions", "regions", "branches", "instantiations")
DEFAULT_METRICS = ("lines", "functions", "branches")
MEASUREMENT_SEMANTICS = (
    "Each file under source_root appears once, keyed by its resolved physical path. "
    "Its denominator is the union of LLVM coverage mappings for that file across all "
    "objects, and an item is covered if any test process executed it. Files that "
    "compile to no executable mapping contribute nothing to the denominator. Static "
    "production objects are force-linked into one test image so that files no test "
    "reaches are reported with zero hits rather than omitted."
)
_HEADER_SUFFIXES = frozenset({".h", ".hh", ".hpp", ".hxx", ".inc", ".inl", ".ipp", ".tcc", ".txx"})
_TEST_COUNT_PATTERNS = (
    re.compile(r"out of\s+(\d+)\b", re.IGNORECASE),
    re.compile(r"\bof\s+(\d+)\s+tests?\b", re.IGNORECASE),
    re.compile(r"\b(\d+)\s+tests?\s+(?:passed|failed|skipped)\b", re.IGNORECASE),
    re.compile(r"test cases?:\s*(\d+)\b", re.IGNORECASE),
    re.compile(r"all tests passed\s*\(\s*\d+\s+assertions?\s+in\s+(\d+)\s+test cases?\s*\)", re.IGNORECASE),
)
_NO_EXECUTION_PATTERNS = (
    re.compile(r"no tests? (?:were )?found", re.IGNORECASE),
    re.compile(r"not run", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+(?:tests?|test cases?)\s+skipped\b", re.IGNORECASE),
    re.compile(r"\b[1-9]\d*\s+skipped\b", re.IGNORECASE),
)
_PROFILE_SUBDIR = "profiles"
_HTML_SUBDIR = "html"


class CoverageError(RuntimeError):
    """A coverage run cannot produce an honest report."""


# --------------------------------------------------------------------------- tools

def _run(argv: Sequence[str], *, cwd: Path | None = None, env: dict[str, str] | None = None,
         timeout: float | None = None) -> subprocess.CompletedProcess[str]:
    """Run an argv without a shell; a timeout becomes return code 124."""
    try:
        return run_captured(argv, cwd=cwd, env=env, timeout=timeout)
    except FileNotFoundError as exc:
        raise CoverageError(f"command not found: {argv[0]}") from exc
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout or ""
        stderr = exc.stderr or ""
        if isinstance(stdout, bytes):
            stdout = stdout.decode("utf-8", "replace")
        if isinstance(stderr, bytes):
            stderr = stderr.decode("utf-8", "replace")
        cleanup_note = ""
        if getattr(exc, "cleanup_incomplete", False):
            notes = getattr(exc, "__notes__", ())
            detail = "; ".join(str(note) for note in notes)
            if not detail:
                detail = "process helper reported incomplete cleanup"
            cleanup_note = "\n[" + detail[:512] + "]"
        return subprocess.CompletedProcess(
            list(argv), 124, stdout,
            stderr + f"\n[command timed out after {timeout}s]\n" + cleanup_note,
        )


def _read_toolchain_file(build_dir: Path) -> dict[str, str]:
    """Read the key=value toolchain record written by SlicerCoverage.cmake."""
    path = build_dir / "slicer-coverage" / "toolchain.txt"
    result: dict[str, str] = {}
    if path.is_file():
        for line in path.read_text(encoding="utf-8").splitlines():
            key, sep, value = line.partition("=")
            if sep and value:
                result[key.strip()] = value.strip()
    return result


def _which(name: str, explicit: str | None = None, configured: str | None = None) -> str:
    candidate = explicit or configured or shutil.which(name)
    if not candidate and sys.platform == "darwin" and shutil.which("xcrun"):
        found = _run(["xcrun", "--find", name])
        if found.returncode == 0 and found.stdout.strip():
            candidate = found.stdout.strip().splitlines()[-1]
    if not candidate or not (os.path.isfile(candidate) or shutil.which(candidate)):
        raise CoverageError(f"required LLVM tool not found: {name}")
    return os.path.abspath(candidate) if os.path.isfile(candidate) else str(shutil.which(candidate))


def _llvm_version(text: str) -> str | None:
    match = re.search(r"\b(?:LLVM|clang) version (\d+(?:\.\d+)*)", text)
    return match.group(1) if match else None


def _tool_version_evidence(compilers: Sequence[str], llvm_cov: str, llvm_profdata: str
                           ) -> dict[str, Any]:
    """Record versions and require the tools to match the compiler's LLVM major.

    llvm-profdata and llvm-cov read the raw-profile and mapping formats the
    compiler wrote; a different major can misread or silently drop data.
    AppleClang reports an Xcode version, so it is recorded but not compared.
    """
    if len(set(compilers)) != 1:
        raise CoverageError("production sources were compiled by different compilers: "
                            + ", ".join(sorted(set(compilers))))
    compiler = compilers[0]
    rows: dict[str, Any] = {}
    for label, tool in (("compiler", compiler), ("llvm-cov", llvm_cov),
                        ("llvm-profdata", llvm_profdata)):
        result = _run([tool, "--version"], timeout=60)
        text = (result.stdout or "") + (result.stderr or "")
        rows[label] = {"path": tool, "version_text": text.strip(), "version": _llvm_version(text)}
        if result.returncode != 0 or rows[label]["version"] is None:
            raise CoverageError(f"cannot determine the LLVM version of {tool}")
    apple = "Apple clang" in rows["compiler"]["version_text"]
    majors = {label: row["version"].split(".")[0] for label, row in rows.items()}
    if apple:
        rows["compatibility"] = "not_compared_apple_clang"
    elif len(set(majors.values())) != 1:
        raise CoverageError("LLVM tool versions do not match the compiler: " + ", ".join(
            f"{label} {rows[label]['version']}" for label in rows))
    else:
        rows["compatibility"] = "major_versions_match"
    return rows


# ------------------------------------------------------------------- test evidence

def split_command(command: str | Sequence[str]) -> list[str]:
    if isinstance(command, str):
        argv = shlex.split(command, posix=(os.name != "nt"))
    else:
        argv = [str(part) for part in command]
    if not argv:
        raise CoverageError("an empty test command cannot establish coverage evidence")
    return argv


def _expand_command(argv: Sequence[str], *, root: Path, build: Path, filter_name: str | None = None) -> list[str]:
    values = {"{root}": str(root), "{build}": str(build), "{filter}": filter_name or ""}
    result = []
    for part in argv:
        for key, value in values.items():
            part = part.replace(key, value)
        result.append(part)
    return result


def test_evidence(output: str) -> int:
    """Return the executed test count, or zero when no runner evidence exists.

    Requiring a count stops a typo in a CTest/Catch filter from producing a
    report with no executed tests. CTest's --no-tests=error is still advised.
    """
    if any(pattern.search(output) for pattern in _NO_EXECUTION_PATTERNS):
        # CTest reports a total even when a label matched only disabled tests.
        return sum(int(match.group(1)) for match in re.finditer(
            r"\b([1-9]\d*)\s+tests?\s+(?:passed|failed)\b", output, re.IGNORECASE))
    counts = [int(match.group(1)) for pattern in _TEST_COUNT_PATTERNS
              for match in pattern.finditer(output)]
    return max(counts, default=0)


# ----------------------------------------------------------------------- paths

def _path_key(path: str | Path) -> str:
    return os.path.normcase(os.path.realpath(os.fspath(path)))


def _under(path: str | Path, directory: Path) -> bool:
    try:
        return os.path.commonpath([_path_key(path), _path_key(directory)]) == _path_key(directory)
    except ValueError:
        return False


def _resolve(root: Path, value: str | Path) -> Path:
    path = Path(value)
    return (path if path.is_absolute() else root / path).resolve()


def _sha256(path: Path) -> str | None:
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError:
        return None
    return digest.hexdigest()


def _artifact_provenance(path: Path) -> dict[str, Any]:
    """Return identity metadata (path, size, SHA-256) for one artifact."""
    resolved = path.resolve()
    record: dict[str, Any] = {"path": str(resolved)}
    try:
        size = resolved.stat().st_size
    except OSError:
        record.update({"status": "missing", "sha256": None, "size_bytes": None})
        return record
    digest = _sha256(resolved)
    record.update({"status": "present" if digest else "unreadable",
                   "sha256": digest, "size_bytes": size})
    return record


# ------------------------------------------------------------------ provenance

def _git_provenance(root: Path) -> dict[str, Any]:
    """Record repository identity and clean/dirty state without file contents."""
    empty = {"head": None, "working_tree": None}
    if not (root / ".git").exists():
        return {"status": "absent", "reason": "git metadata is absent", **empty}
    try:
        head_result = _run(["git", "rev-parse", "--verify", "HEAD"], cwd=root, timeout=60)
    except CoverageError:
        return {"status": "unavailable", "reason": "git executable is unavailable", **empty}
    head = head_result.stdout.strip()
    if head_result.returncode != 0 or not re.fullmatch(r"[0-9a-fA-F]{40,64}", head):
        return {"status": "error", "reason": "git could not resolve HEAD", **empty}
    status_result = _run(["git", "status", "--porcelain=v1", "--untracked-files=normal"],
                         cwd=root, timeout=300)
    if status_result.returncode != 0:
        return {"status": "present", "head": head,
                "working_tree": {"status": "unknown", "dirty": None}}
    lines = [line for line in status_result.stdout.splitlines() if line]
    untracked = [line for line in lines if line[:2] == "??"]
    tracked = [line for line in lines if line[:2] not in {"??", "!!"}]
    dirty = bool(tracked or untracked)
    return {
        "status": "present",
        "head": head,
        "working_tree": {
            "status": "dirty" if dirty else "clean",
            "dirty": dirty,
            "tracked_changes": bool(tracked),
            "untracked_changes": bool(untracked),
            "staged_changes": any(line[:1] not in {" ", "?"} for line in tracked),
            "unstaged_changes": any(line[1:2] not in {" ", "?"} for line in tracked),
            "tracked_change_count": len(tracked),
            "untracked_change_count": len(untracked),
        },
    }


def _coverage_provenance(root: Path, objects: Sequence[Path], manifests: Sequence[Path],
                         build_dir: Path, compile_commands: Path) -> dict[str, Any]:
    """Collect reproducibility metadata without asserting source/binary equivalence."""
    return {
        "git": _git_provenance(root),
        "test_object_binaries": [_artifact_provenance(path) for path in objects],
        "cmake_manifests": [_artifact_provenance(path) for path in manifests],
        "cmake_cache": _artifact_provenance(build_dir / "CMakeCache.txt"),
        "compile_commands": _artifact_provenance(compile_commands),
        "artifact_source_verification": {
            "status": "not_performed",
            "reason": "HEAD and artifact hashes identify inputs but do not prove source/binary correspondence.",
        },
    }


# --------------------------------------------------------------- binary identity

def elf_build_id(path: Path) -> str | None:
    """Return the GNU build ID of an ELF file, or None for non-ELF/unidentified files."""
    try:
        with path.open("rb") as stream, mmap.mmap(stream.fileno(), 0, access=mmap.ACCESS_READ) as data:
            return _elf_build_id(data)
    except (OSError, ValueError):
        return None


def _elf_build_id(data: Any) -> str | None:
    if data[:4] != b"\x7fELF" or len(data) < 64:
        return None
    is64 = data[4] == 2
    endian = "<" if data[5] == 1 else ">"
    try:
        if is64:
            shoff, = struct.unpack_from(endian + "Q", data, 0x28)
            shentsize, shnum = struct.unpack_from(endian + "HH", data, 0x3A)
        else:
            shoff, = struct.unpack_from(endian + "I", data, 0x20)
            shentsize, shnum = struct.unpack_from(endian + "HH", data, 0x2E)
        for index in range(shnum):
            base = shoff + index * shentsize
            sh_type, = struct.unpack_from(endian + "I", data, base + 4)
            if sh_type != 7:  # SHT_NOTE
                continue
            if is64:
                offset, size = struct.unpack_from(endian + "QQ", data, base + 0x18)
            else:
                offset, size = struct.unpack_from(endian + "II", data, base + 0x10)
            position, end = offset, offset + size
            while position + 12 <= end:
                namesz, descsz, note_type = struct.unpack_from(endian + "III", data, position)
                name_start = position + 12
                desc_start = name_start + ((namesz + 3) & ~3)
                if note_type == 3 and data[name_start:name_start + namesz].rstrip(b"\0") == b"GNU":
                    return bytes(data[desc_start:desc_start + descsz]).hex()
                position = desc_start + ((descsz + 3) & ~3)
    except struct.error:
        return None
    return None


def _profile_binary_ids(llvm_profdata: str, profile: Path, timeout: float | None) -> list[str] | None:
    """Return the binary IDs recorded in a raw profile, or None if unreadable."""
    result = _run([llvm_profdata, "show", "--binary-ids", str(profile)], timeout=timeout)
    if result.returncode != 0:
        return None
    ids: list[str] = []
    in_ids = False
    for line in result.stdout.splitlines():
        if line.strip().startswith("Binary IDs:"):
            in_ids = True
            continue
        if in_ids:
            value = line.strip().lower()
            if re.fullmatch(r"[0-9a-f]{8,}", value):
                ids.append(value)
            elif value:
                in_ids = False
    return ids


def _profile_identity_evidence(llvm_profdata: str, profiles: Sequence[Path],
                               objects: Sequence[Path], timeout: float | None) -> dict[str, Any]:
    """Check that every raw profile came from one of the measured executables.

    A profile from another binary (a stale run, a discovery listing, a helper
    tool) would merge silently and change the hit counts. Executables with no
    profile mean a selected suite never wrote counters. Both are failures when
    build IDs are available; without them (non-ELF objects) the check reports
    "unavailable" and the LLVM hash-mismatch diagnostics remain the guard.
    """
    object_ids = {str(path): elf_build_id(path) for path in objects}
    if not all(object_ids.values()):
        return {"status": "unavailable", "reason": "not every object has an ELF GNU build ID",
                "object_build_ids": object_ids, "errors": []}
    known = {value: key for key, value in object_ids.items()}
    errors: list[dict[str, Any]] = []
    profile_rows = []
    seen: set[str] = set()
    for profile in profiles:
        ids = _profile_binary_ids(llvm_profdata, profile, timeout)
        profile_rows.append({"profile": str(profile), "binary_ids": ids})
        if ids is None:
            errors.append({"profile": str(profile), "reason": "unreadable_profile"})
        elif not ids:
            errors.append({"profile": str(profile), "reason": "profile_without_binary_id"})
        else:
            for value in ids:
                if value not in known:
                    errors.append({"profile": str(profile), "reason": "foreign_profile",
                                   "binary_id": value})
                seen.add(value)
    for path, value in object_ids.items():
        if value not in seen:
            errors.append({"object": path, "reason": "object_without_profile"})
    return {"status": "error" if errors else "ok", "object_build_ids": object_ids,
            "profiles": profile_rows, "errors": errors}


# --------------------------------------------------------------------- manifests

def _read_manifest(path: Path, root: Path) -> list[Path]:
    if not path.is_file():
        raise CoverageError(f"coverage source manifest does not exist: {path}")
    result: list[Path] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        value = raw.strip()
        if value and not value.startswith("#"):
            result.append(_resolve(root, value))
    if not result:
        raise CoverageError(f"coverage source manifest is empty: {path}")
    return result


# ---------------------------------------------------------------- llvm export

def _summary_count(summary: dict[str, Any], key: str) -> tuple[int, int]:
    value = summary.get(key, {})
    if not isinstance(value, dict):
        return 0, 0
    try:
        return int(value.get("covered", 0)), int(value.get("count", 0))
    except (TypeError, ValueError):
        return 0, 0


def _metric(covered: int, count: int) -> dict[str, int | float]:
    # A file with no items of a kind (e.g. no branches) is reported at 100%,
    # matching the previous report format; an empty overall metric is an
    # integrity failure handled by the caller.
    return {"covered": covered, "count": count,
            "percent": 100.0 * covered / count if count else 100.0}


def _file_summary(item: dict[str, Any]) -> dict[str, dict[str, int | float]]:
    summary = item.get("summary", {})
    if not isinstance(summary, dict):
        summary = {}
    return {key: _metric(*_summary_count(summary, key)) for key in METRICS}


def _merge_summaries(files: Iterable[dict[str, Any]]) -> dict[str, dict[str, int | float]]:
    totals = {key: [0, 0] for key in METRICS}
    for item in files:
        for key in METRICS:
            value = item["summary"][key]
            totals[key][0] += int(value["covered"])
            totals[key][1] += int(value["count"])
    return {key: _metric(*totals[key]) for key in METRICS}


def _has_executable_mapping(summary: dict[str, Any]) -> bool:
    return any(_summary_count(summary, key)[1] > 0 for key in METRICS)


def _export_files(export: dict[str, Any], path_base: Path) -> Iterable[tuple[Path, dict[str, Any]]]:
    data = export.get("data", [])
    if not isinstance(data, list):
        raise CoverageError("llvm-cov export has no data array")
    for chunk in data:
        files = chunk.get("files", []) if isinstance(chunk, dict) else []
        for item in files if isinstance(files, list) else []:
            if isinstance(item, dict) and isinstance(item.get("filename"), str) and item["filename"]:
                path = Path(item["filename"])
                yield (path if path.is_absolute() else path_base / path), item


def parse_export(export: dict[str, Any], *, source_root: Path,
                 manifest: Sequence[Path] = (), path_base: Path | None = None
                 ) -> tuple[dict[str, Any], list[str], list[dict[str, Any]]]:
    """Select production files from an LLVM export.

    Returns (report, missing manifest sources, duplicate entries). llvm-cov
    already merges one filename across all objects. Two entries resolving to
    the same physical file (a symlink or ``..`` spelling) cannot be merged from
    summaries without double counting, so they are returned as an integrity
    failure and only the first is kept in the totals.
    """
    selected: dict[str, dict[str, Any]] = {}
    duplicates: list[dict[str, Any]] = []
    for path, item in _export_files(export, path_base or Path.cwd()):
        if not _under(path, source_root):
            continue
        key = _path_key(path)
        if key in selected:
            duplicates.append({"path": key, "spellings": [selected[key]["llvm_filename"], item["filename"]]})
            continue
        selected[key] = {"filename": str(Path(key)), "llvm_filename": item["filename"],
                         "summary": _file_summary(item)}
    if not selected:
        raise CoverageError(f"llvm-cov export contains no files under {source_root}")
    files = sorted(selected.values(), key=lambda value: value["filename"])
    report = {
        "source_root": str(source_root.resolve()),
        "files": [{"filename": item["filename"], "summary": item["summary"]} for item in files],
        "summary": _merge_summaries(files),
    }
    missing = sorted({str(path.resolve()) for path in manifest if _path_key(path) not in selected})
    return report, missing, duplicates


def _percent(summary: dict[str, Any], key: str) -> float:
    value = summary.get(key, {})
    return float(value.get("percent", 100.0)) if isinstance(value, dict) else 100.0


def _meets_threshold(summary: dict[str, Any], key: str, threshold: float) -> bool:
    # Strict: a reported 90.00% does not satisfy a "more than 90%" policy.
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
    return sorted(item["filename"] for item in report.get("files", [])
                  if not _has_executable_mapping(item.get("summary", {})))


def _reconcile_unmapped_sources(
    report: dict[str, Any], compiler_evidence: dict[str, Any] | None
) -> tuple[list[str], list[str], list[str]]:
    """Separate metadata-only headers from missing executable mappings.

    LLVM can list a header that contributed no executable mapping (a
    constants-only header included by a mapped TU). It is waived only after a
    successful object audit shows no compiled object maps executable code to
    it; otherwise a zero-count file remains an integrity error.
    """
    unmapped = _sources_without_executable_mappings(report)
    if not compiler_evidence or compiler_evidence.get("status") != "ok":
        return unmapped, [], list(unmapped)
    waived = {_path_key(value) for value in compiler_evidence.get("mapping_free_sources", [])}
    waived |= {_path_key(value) for value in compiler_evidence.get("object_mappings_without_source", [])}
    executable = {_path_key(value) for value in compiler_evidence.get("object_executable_mappings", [])}
    headers: list[str] = []
    unexpected: list[str] = []
    for value in unmapped:
        key = _path_key(value)
        if key in waived:
            continue
        if Path(value).suffix.lower() in _HEADER_SUFFIXES and key not in executable:
            headers.append(value)
        else:
            unexpected.append(value)
    return unmapped, sorted(headers), sorted(unexpected)


def _profile_mapping_errors(stderr: str) -> list[str]:
    # llvm-cov can exit 0 after dropping functions whose profile hashes do not
    # match their coverage maps; their absence would shrink the denominator.
    return [line.strip() for line in stderr.splitlines()
            if re.search(r"\b[1-9][0-9]* functions? ha(?:ve|s) mismatched data\b", line)]


# ------------------------------------------------------------- compile commands

def _compile_command_arguments(entry: dict[str, Any]) -> list[str]:
    arguments = entry.get("arguments")
    if isinstance(arguments, list) and all(isinstance(value, str) for value in arguments):
        return list(arguments)
    command = entry.get("command")
    if isinstance(command, str) and command.strip():
        return shlex.split(command, posix=(os.name != "nt"))
    return []


def _compile_command_output(arguments: Sequence[str], directory: Path) -> Path | None:
    for index, value in enumerate(arguments):
        if value == "-o" and index + 1 < len(arguments):
            return _resolve(directory, arguments[index + 1])
    return None


def _coverage_compile_flags(arguments: Sequence[str]) -> bool:
    return "-fcoverage-mapping" in arguments and "-fprofile-instr-generate" in arguments


def _atomic_profile_updates(arguments: Sequence[str]) -> bool:
    # Clang uses the last setting; a missing or prefer-atomic setting is not
    # evidence that concurrent increments are atomic.
    if any(value.startswith("@") for value in arguments):
        return False  # An unresolved response file could override visible flags.
    updates = [value for value in arguments if value.startswith("-fprofile-update=")]
    return bool(updates) and updates[-1] == "-fprofile-update=atomic"


def _load_compile_commands(path: Path) -> dict[str, list[dict[str, Any]]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise CoverageError(f"cannot read compile commands {path}: {exc}") from exc
    if not isinstance(payload, list):
        raise CoverageError(f"compile commands must be a JSON array: {path}")
    by_source: dict[str, list[dict[str, Any]]] = {}
    for item in payload:
        if not isinstance(item, dict) or not isinstance(item.get("file"), str):
            continue
        directory = _resolve(path.parent, str(item.get("directory", path.parent)))
        arguments = _compile_command_arguments(item)
        source = _resolve(directory, item["file"])
        by_source.setdefault(_path_key(source), []).append({
            "source": source,
            "directory": directory,
            "compiler": arguments[0] if arguments else None,
            "object": _compile_command_output(arguments, directory),
            "coverage_flags": _coverage_compile_flags(arguments),
            "atomic_profile_updates": _atomic_profile_updates(arguments),
        })
    return by_source


def _require_atomic_profile_commands(manifest: Sequence[Path], compile_commands: Path) -> list[str]:
    """Reject unsafe or stale builds before an expensive test run.

    Returns the compilers used for the manifest sources.
    """
    if not manifest:
        raise CoverageError("coverage requires a non-empty compiled-source manifest")
    by_source = _load_compile_commands(compile_commands)
    compilers: list[str] = []
    for source in manifest:
        candidates = [item for item in by_source.get(_path_key(source), []) if item["coverage_flags"]]
        if not candidates or any(not item["atomic_profile_updates"] for item in candidates):
            raise CoverageError(f"coverage requires explicit atomic LLVM compile commands for {source}")
        for item in candidates:
            compiler = item["compiler"]
            if compiler:
                found = compiler if os.path.isabs(compiler) else shutil.which(compiler) or compiler
                compilers.append(os.path.realpath(found))
    return compilers


# --------------------------------------------------------- compiler object audit

_EMPTY_INDEXED_PROFILE_TEXT = ":ir\n"


def _create_empty_indexed_profile(llvm_profdata: str, output: Path, *,
                                  timeout: float | None) -> dict[str, Any]:
    """Create a valid empty indexed profile for compiler-object inspection.

    llvm-cov needs an indexed profile to read an object's coverage map. This
    one is separate from the runtime profile, so object audits never see hits.
    """
    output = Path(output).absolute()
    profile_text = output.with_suffix(".proftext")
    if output.is_symlink() or profile_text.is_symlink():
        raise CoverageError(f"refusing to replace symlinked inventory path: {output} or {profile_text}")
    try:
        output.unlink(missing_ok=True)
        profile_text.write_text(_EMPTY_INDEXED_PROFILE_TEXT, encoding="utf-8")
    except OSError as exc:
        raise CoverageError(f"cannot prepare empty indexed profile {profile_text}: {exc}") from exc
    merge_argv = [llvm_profdata, "merge", "-sparse", "-o", str(output), str(profile_text)]
    merge = _run(merge_argv, cwd=output.parent, timeout=timeout)
    if merge.returncode != 0:
        raise CoverageError("compiler inventory profile generation failed "
                            f"(returncode {merge.returncode}): {merge.stderr.strip()}")
    if output.is_symlink() or not output.is_file() or output.stat().st_size <= 0:
        raise CoverageError("compiler inventory profile generation produced no non-empty profile")
    return {"status": "ok", "profile": str(output), "argv": merge_argv,
            "purpose": "compiler object coverage-map inventory; runtime hit counts are not used"}


def _exported_paths(export: dict[str, Any], *, path_base: Path, executable_only: bool) -> set[str]:
    return {_path_key(path) for path, item in _export_files(export, path_base)
            if not executable_only or _has_executable_mapping(item.get("summary", {}))}


def _audit_object(llvm_cov: str, source: Path, candidate: dict[str, Any],
                  inventory_profile: Path, timeout: float | None) -> tuple[dict[str, Any], dict[str, Any] | None]:
    """Classify one compiled object; returns (record, error or None)."""
    object_path = candidate["object"]
    record: dict[str, Any] = {"object": str(object_path) if object_path else None}
    if object_path is None or not object_path.is_file():
        record["status"] = "missing_object"
        return record, {"source": str(source), "reason": "missing_compiler_object",
                        "object": record["object"]}
    export = _run([llvm_cov, "export", "-format=text", "-summary-only",
                   "-instr-profile", str(inventory_profile), "-object", str(object_path)],
                  cwd=candidate["directory"], timeout=timeout)
    if export.stderr:
        record["diagnostics"] = export.stderr
    if export.returncode != 0:
        if export.returncode != 124 and "no coverage data found" in (export.stderr or "").lower():
            record["status"] = "mapping_free"
            return record, None
        record["status"] = "export_error"
        return record, {"source": str(source), "reason": "compiler_export_failed",
                        "object": str(object_path), "stderr": export.stderr}
    mapping_errors = _profile_mapping_errors(export.stderr or "")
    if mapping_errors:
        record["status"] = "profile_mapping_error"
        return record, {"source": str(source), "reason": "compiler_profile_mapping_error",
                        "object": str(object_path), "diagnostics": mapping_errors}
    try:
        object_export = json.loads(export.stdout)
    except json.JSONDecodeError:
        record["status"] = "invalid_export"
        return record, {"source": str(source), "reason": "compiler_export_not_json",
                        "object": str(object_path)}
    executable = _exported_paths(object_export, path_base=candidate["directory"], executable_only=True)
    listed = _exported_paths(object_export, path_base=candidate["directory"], executable_only=False)
    record["executable_mappings"] = sorted(executable)
    record["status"] = ("mapped" if _path_key(source) in listed
                        else "header_mapping_only" if executable else "mapping_free")
    return record, None


def _compiler_source_evidence(llvm_cov: str, manifest: Sequence[Path],
                              compile_commands: Path, *, timeout: float | None,
                              inventory_profile: Path) -> dict[str, Any]:
    """Classify manifest TUs using their compiled objects' coverage maps.

    A source-level export cannot tell an include-only TU from a TU whose object
    was left out of the executable. The object's own map supplies that: an
    object with executable mappings must appear in the final export; an
    instrumented object with no executable map (e.g. a PCH source) is
    mapping-free. Missing objects and failed audits are errors, never waived.
    """
    by_source = _load_compile_commands(compile_commands)
    records: list[dict[str, Any]] = []
    classes: dict[str, list[str]] = {"mapped": [], "mapping_free_translation_unit": [],
                                     "object_mappings_without_manifest_source": []}
    errors: list[dict[str, Any]] = []
    for source in manifest:
        candidates = by_source.get(_path_key(source), [])
        coverage_candidates = [item for item in candidates if item["coverage_flags"]]
        if not candidates:
            reason = classification = "missing_compile_command"
        elif not coverage_candidates:
            reason, classification = "compile_command_missing_llvm_flags", "unprofiled_translation_unit"
        elif any(not item["atomic_profile_updates"] for item in coverage_candidates):
            reason, classification = "compile_command_missing_atomic_profile_updates", "unsafe_profile_counters"
        else:
            reason = None
        if reason:
            errors.append({"source": str(source), "reason": reason})
            records.append({"source": str(source), "classification": classification, "objects": []})
            continue
        object_records = []
        had_error = False
        for candidate in coverage_candidates:
            record, error = _audit_object(llvm_cov, source, candidate, inventory_profile, timeout)
            object_records.append(record)
            if error:
                errors.append(error)
                had_error = True
        statuses = {record["status"] for record in object_records}
        if had_error:
            classification = "compiler_evidence_error"
        elif "mapped" in statuses:
            classification = "mapped"
        elif "header_mapping_only" in statuses:
            classification = "object_mappings_without_manifest_source"
        else:
            classification = "mapping_free_translation_unit"
        if classification in classes:
            classes[classification].append(str(source))
        records.append({"source": str(source), "classification": classification,
                        "objects": object_records})
    return {
        "status": "error" if errors else "ok",
        "path": str(compile_commands.resolve()),
        "sources": records,
        "mapped_sources": sorted(classes["mapped"]),
        "mapping_free_sources": sorted(classes["mapping_free_translation_unit"]),
        "object_mappings_without_source": sorted(classes["object_mappings_without_manifest_source"]),
        "object_executable_mappings": sorted({
            path for record in records for obj in record["objects"]
            for path in obj.get("executable_mappings", [])}),
        "errors": errors,
    }


# --------------------------------------------------------------------- outputs

def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    os.replace(temporary, path)


def _base_report(args: argparse.Namespace) -> dict[str, Any]:
    measure_only = args.threshold is None
    return {
        "schema": REPORT_SCHEMA,
        "schema_version": REPORT_SCHEMA_VERSION,
        "backend": BACKEND,
        "measurement": {
            "mode": "measure_only" if measure_only else "threshold",
            "threshold": args.threshold,
            "metrics": list(args.metrics),
            "gates_on_percentage": not measure_only,
            "semantics": MEASUREMENT_SEMANTICS,
        },
        # Kept at top level for consumers of the previous report format.
        "threshold": args.threshold,
        "metrics": list(args.metrics),
    }


def _html_page(html_dir: Path, source: str) -> Path:
    # llvm-cov show -format=html mirrors each absolute source path under coverage/.
    drive, rest = os.path.splitdrive(source)
    relative = (drive.rstrip(":") + rest if drive else rest).lstrip("/\\")
    return html_dir / "coverage" / f"{relative}.html"


def _annotation_command(llvm_cov: str, merged: Path, objects: Sequence[Path],
                        html_dir: Path, sources: Sequence[str]) -> list[str]:
    """llvm-cov show argv over exactly the report's objects, profile and files."""
    argv = [llvm_cov, "show", "-format=html", f"-output-dir={html_dir}",
            "-instr-profile", str(merged), "-show-line-counts-or-regions",
            str(objects[0])]
    for obj in objects[1:]:
        argv.extend(["-object", str(obj)])
    argv.extend(sources)
    return argv


# ------------------------------------------------------------------------ run

def run_coverage(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    out = _resolve(root, args.out)
    out.mkdir(parents=True, exist_ok=True)
    source_root = _resolve(root, args.source_root)
    build_dir = _resolve(root, args.build_dir)
    objects = [_resolve(root, value) for value in args.object]
    compile_commands = _resolve(root, args.compile_commands or build_dir / "compile_commands.json")
    manifest_files = [_resolve(root, value) for value in args.manifest]
    report_path = out / "coverage-report.json"
    report = _base_report(args)
    report["source_root"] = str(source_root)
    report["provenance"] = _coverage_provenance(root, objects, manifest_files, build_dir, compile_commands)

    def fail(status: str, **fields: Any) -> int:
        report.update({"status": status, "integrity_status": "error"}, **fields)
        _write_json(report_path, report)
        return 2

    if len({_path_key(path) for path in objects}) != len(objects):
        raise CoverageError("the same coverage object was given more than once")
    missing_objects = [str(value) for value in objects if not value.is_file()]
    if missing_objects:
        raise CoverageError("coverage object does not exist: " + ", ".join(missing_objects))
    if not compile_commands.is_file():
        raise CoverageError(f"compile commands do not exist: {compile_commands}")
    toolchain = _read_toolchain_file(build_dir)
    llvm_cov = _which("llvm-cov", args.llvm_cov, toolchain.get("llvm_cov"))
    llvm_profdata = _which("llvm-profdata", args.llvm_profdata, toolchain.get("llvm_profdata"))

    manifest = sorted({path for manifest_path in manifest_files
                       for path in _read_manifest(manifest_path, root)}, key=str)
    outside = [str(path) for path in manifest if not _under(path, source_root)]
    if outside:
        raise CoverageError(
            f"source manifests list files outside --source-root {source_root} "
            f"(e.g. {outside[0]}); supply production target manifests only")
    compilers = _require_atomic_profile_commands(manifest, compile_commands)
    report["tool_versions"] = _tool_version_evidence(compilers, llvm_cov, llvm_profdata)
    report["objects"] = [str(value) for value in objects]
    report["manifest"] = [str(value) for value in manifest_files]

    if "{filter}" in args.test_command and not args.filter:
        raise CoverageError("test command contains {filter}, but --filter was not supplied")
    command = _expand_command(split_command(args.test_command), root=root, build=build_dir,
                              filter_name=args.filter)
    if args.filter and "{filter}" not in args.test_command and \
            Path(command[0]).name.startswith("ctest") and "-L" not in command:
        command.extend(["-L", args.filter])

    # A private profile directory: nothing but this run's test processes may
    # write here. %4m bounds raw profiles to four merge-pool files per binary.
    profile_dir = out / _PROFILE_SUBDIR
    profile_dir.mkdir(exist_ok=True)
    for stale in profile_dir.iterdir():
        if stale.is_file() and stale.suffix == ".profraw" and not stale.is_symlink():
            stale.unlink()
    leftovers = sorted(str(path) for path in profile_dir.iterdir())
    if leftovers:
        raise CoverageError(f"profile directory {profile_dir} is not empty: {leftovers[:3]}")
    profile_pattern = profile_dir / "%4m.profraw"
    env = dict(os.environ)
    env["LLVM_PROFILE_FILE"] = str(profile_pattern)
    started = time.monotonic()
    result = _run(command, cwd=root, env=env, timeout=args.timeout)
    output = (result.stdout or "") + (result.stderr or "")
    evidence = test_evidence(output)
    tests_failed = result.returncode != 0
    report["tests_failed"] = tests_failed
    report["run"] = {
        "argv": command,
        "profile_pattern": str(profile_pattern),
        "returncode": result.returncode,
        "timed_out": result.returncode == 124,
        "duration_seconds": round(time.monotonic() - started, 3),
        "test_evidence_count": evidence,
        "stdout": result.stdout,
        "stderr": result.stderr,
    }
    profiles = sorted(profile_dir.glob("*.profraw"))
    report["profiles"] = [str(value) for value in profiles]
    if evidence <= 0 and not profiles:
        return fail("test_failed_no_evidence" if tests_failed else "no_test_evidence")
    if not profiles:
        return fail("test_failed_no_profile" if tests_failed else "no_profile")

    identity = _profile_identity_evidence(llvm_profdata, profiles, objects, args.timeout)
    report["profile_identity"] = identity

    merged = out / "merged.profdata"
    merge_argv = [llvm_profdata, "merge", "-sparse", "-failure-mode=any",
                  "-o", str(merged), *map(str, profiles)]
    merge = _run(merge_argv, cwd=root, timeout=args.timeout)
    report["merge"] = {"argv": merge_argv, "returncode": merge.returncode, "stderr": merge.stderr}
    if merge.returncode != 0 or not merged.is_file():
        return fail("profile_merge_failed")
    report["merged_profile"] = _artifact_provenance(merged)

    # File summaries are all the report and integrity checks need; full
    # line/segment detail comes from the annotated HTML instead.
    export_argv = [llvm_cov, "export", "-format=text", "-summary-only", "-instr-profile", str(merged)]
    for obj in objects:
        export_argv.extend(["-object", str(obj)])
    export = _run(export_argv, cwd=root, timeout=args.timeout)
    report["export"] = {"argv": export_argv, "returncode": export.returncode,
                        "stderr": export.stderr, "path": str(out / "coverage-export.json")}
    if export.returncode != 0:
        return fail("export_failed")
    (out / "coverage-export.json").write_text(export.stdout, encoding="utf-8")
    try:
        exported = json.loads(export.stdout)
        parsed, missing, duplicates = parse_export(exported, source_root=source_root,
                                                   manifest=manifest, path_base=root)
    except (json.JSONDecodeError, CoverageError) as exc:
        return fail("invalid_export", error=str(exc))
    report.update(parsed)

    inventory = _create_empty_indexed_profile(
        llvm_profdata, out / "compiler-inventory-empty.profdata", timeout=args.timeout)
    compiler_evidence = _compiler_source_evidence(
        llvm_cov, manifest, compile_commands, timeout=args.timeout,
        inventory_profile=Path(inventory["profile"]))
    unmapped, mapping_free_headers, unexpected_unmapped = _reconcile_unmapped_sources(
        parsed, compiler_evidence)
    without_direct_mapping = {_path_key(value) for value in
                              compiler_evidence["mapping_free_sources"]
                              + compiler_evidence["object_mappings_without_source"]}
    linked_missing = [value for value in missing if _path_key(value) not in without_direct_mapping]
    exported_paths = {_path_key(item["filename"]) for item in parsed["files"]}
    missing_object_mappings = sorted(
        path for path in compiler_evidence["object_executable_mappings"]
        if _under(path, source_root) and _path_key(path) not in exported_paths)
    missing_sources = [item["filename"] for item in parsed["files"]
                       if not Path(item["filename"]).is_file()]
    for item in parsed["files"]:
        item["source_sha256"] = _sha256(Path(item["filename"]))
    empty_metrics = [key for key in args.metrics if int(parsed["summary"][key]["count"]) <= 0]
    integrity_failures = {
        "profile_mapping": _profile_mapping_errors(export.stderr or ""),
        "profile_identity": identity["errors"],
        "compiler_evidence": compiler_evidence["errors"],
        "missing_compiled_source_mappings": linked_missing,
        "missing_object_executable_mappings": missing_object_mappings,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "duplicate_source_entries": duplicates,
        "missing_source_files": missing_sources,
        "empty_metrics": empty_metrics,
    }
    integrity_failed = any(integrity_failures.values())

    sources_file = out / "coverage-sources.txt"
    sources_file.write_text("".join(item["filename"] + "\n" for item in parsed["files"]), encoding="utf-8")
    html_dir = out / _HTML_SUBDIR
    annotation_argv = _annotation_command(llvm_cov, merged, objects, html_dir,
                                          [item["filename"] for item in parsed["files"]])
    report["annotation"] = {
        "format": "llvm-cov-show-html",
        "requested": bool(args.html),
        "output_dir": str(html_dir),
        "index": str(html_dir / "index.html"),
        "sources_file": str(sources_file),
        "argv": annotation_argv,
        "status": "not_requested",
    }
    html_failed = False
    if args.html:
        if html_dir.exists():
            shutil.rmtree(html_dir)
        show = _run(annotation_argv, cwd=root, timeout=args.timeout)
        report["annotation"].update({"returncode": show.returncode, "stderr": show.stderr})
        missing_pages = []
        for item in report["files"]:
            page = _html_page(html_dir, item["filename"])
            if page.is_file():
                item["html"] = page.relative_to(out).as_posix()
            elif _has_executable_mapping(item["summary"]):
                missing_pages.append(item["filename"])
        html_failed = show.returncode != 0 or not (html_dir / "index.html").is_file() or bool(missing_pages)
        report["annotation"]["missing_pages"] = missing_pages
        report["annotation"]["status"] = "error" if html_failed else "ok"

    if tests_failed:
        status = "test_failed"
    elif evidence <= 0:
        status = "no_test_evidence"
    elif integrity_failed:
        status = "coverage_integrity_error"
    elif html_failed:
        status = "annotation_failed"
    elif args.threshold is not None and not all(
            _meets_threshold(parsed["summary"], key, args.threshold) for key in args.metrics):
        status = "coverage_below_threshold"
    else:
        status = "pass"
    report.update({
        "status": status,
        "integrity_status": "error" if integrity_failed else "pass",
        "integrity_failures": integrity_failures,
        "missing_manifest_sources": missing,
        "missing_compiled_source_mappings": linked_missing,
        "missing_object_executable_mappings": missing_object_mappings,
        "mapping_free_headers": mapping_free_headers,
        "mapping_free_sources": compiler_evidence["mapping_free_sources"],
        "object_mappings_without_source": compiler_evidence["object_mappings_without_source"],
        "sources_without_executable_mappings": unmapped,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "compiler_evidence": compiler_evidence,
        "compiler_inventory_profile": inventory,
    })
    _write_json(report_path, report)
    if status == "pass":
        return 0
    return 1 if status == "coverage_below_threshold" else 2


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="action", required=True)
    run = sub.add_parser("run", help="run tests, merge profiles and write the coverage report")
    run.add_argument("--root", default=".", help="repository root (default: current directory)")
    run.add_argument("--build-dir", default="build", help="build directory, relative to root")
    run.add_argument("--source-root", default="src/libslic3r", help="production source root")
    run.add_argument("--object", action="append", required=True,
                     help="instrumented test executable; repeatable")
    run.add_argument("--manifest", action="append", required=True,
                     help="CMake source manifest of a production target; repeatable")
    run.add_argument("--compile-commands",
                     help="compile_commands.json (default: BUILD_DIR/compile_commands.json)")
    run.add_argument("--out", default="build/slicer-coverage-report", help="report directory")
    run.add_argument("--test-command", required=True,
                     help="test argv as a string; {root}, {build} and {filter} expand")
    run.add_argument("--filter", help="CTest label used when the command has no {filter} placeholder")
    mode = run.add_mutually_exclusive_group()
    mode.add_argument("--measure-only", action="store_true",
                      help="record coverage without a percentage gate (the default)")
    mode.add_argument("--threshold", type=float,
                      help="also require each --metric to exceed this percentage (exit 1 if not)")
    run.add_argument("--metric", dest="metrics", action="append", choices=METRICS, default=None,
                     help="metric to require for --threshold and to check for an empty "
                          "denominator; repeatable (default: lines, functions, branches)")
    run.add_argument("--html", action="store_true",
                     help="write llvm-cov annotated HTML for the reported files to OUT/html")
    run.add_argument("--timeout", type=float, default=7200.0, help="per-command timeout in seconds")
    run.add_argument("--llvm-cov", help="llvm-cov (default: CMake's SLICER_LLVM_COV, then PATH)")
    run.add_argument("--llvm-profdata",
                     help="llvm-profdata (default: CMake's SLICER_LLVM_PROFDATA, then PATH)")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.threshold is not None and not 0.0 <= args.threshold <= 100.0:
        parser.error("--threshold must be between 0 and 100")
    args.metrics = list(dict.fromkeys(args.metrics or DEFAULT_METRICS))
    try:
        return run_coverage(args)
    except CoverageError as exc:
        try:
            root = Path(args.root).resolve()
            build_dir = _resolve(root, args.build_dir)
            report = _base_report(args)
            report.update({
                "status": "error",
                "integrity_status": "error",
                "error": str(exc),
                "provenance": _coverage_provenance(
                    root, [_resolve(root, value) for value in args.object],
                    [_resolve(root, value) for value in args.manifest], build_dir,
                    _resolve(root, args.compile_commands or build_dir / "compile_commands.json")),
            })
            _write_json(_resolve(root, args.out) / "coverage-report.json", report)
        except OSError:
            pass
        print(f"slicer_coverage: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
