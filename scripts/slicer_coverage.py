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
      --compile-commands build/compile_commands.json \
      --test-command 'ctest --test-dir build/tests -C Release -LE "NotWorking|RequiresApp" --no-tests=error' \
      --out build/slicer-coverage

The command is argv based (``shell=False``); command strings are only split
with ``shlex`` so paths and arguments remain explicit and auditable.
"""

from __future__ import annotations

import argparse
import glob
import hashlib
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


from slicer_process import run_captured

DEFAULT_THRESHOLD = 90.0
_HEADER_SUFFIXES = frozenset({".h", ".hh", ".hpp", ".hxx", ".inc", ".inl", ".ipp", ".tcc", ".txx"})
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


def _artifact_provenance(path: Path) -> dict[str, Any]:
    """Return non-content identity metadata for a report artifact."""
    resolved = path.resolve()
    record: dict[str, Any] = {"path": str(resolved)}
    try:
        stat = resolved.stat()
    except FileNotFoundError:
        record.update({"status": "missing", "sha256": None, "size_bytes": None})
        return record
    except OSError:
        record.update({"status": "unreadable", "sha256": None, "size_bytes": None})
        return record

    digest = hashlib.sha256()
    try:
        with resolved.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError:
        record.update({"status": "unreadable", "sha256": None, "size_bytes": stat.st_size})
        return record
    record.update({"status": "present", "sha256": digest.hexdigest(), "size_bytes": stat.st_size})
    return record


def _git_provenance(root: Path) -> dict[str, Any]:
    """Record repository identity and clean/dirty state without file contents."""
    git_metadata = root / ".git"
    if not git_metadata.exists():
        return {
            "status": "absent",
            "reason": "git metadata is absent",
            "head": None,
            "working_tree": None,
        }

    try:
        head_result = _run(["git", "rev-parse", "--verify", "HEAD"], cwd=root)
    except CoverageError:
        return {
            "status": "unavailable",
            "reason": "git executable is unavailable",
            "head": None,
            "working_tree": None,
        }
    if head_result.returncode != 0:
        return {
            "status": "error",
            "reason": "git could not resolve HEAD",
            "head": None,
            "working_tree": None,
        }
    head = head_result.stdout.strip()
    if not re.fullmatch(r"[0-9a-fA-F]{40,64}", head):
        return {
            "status": "error",
            "reason": "git returned an invalid HEAD identifier",
            "head": None,
            "working_tree": None,
        }

    try:
        status_result = _run(
            ["git", "status", "--porcelain=v1", "--untracked-files=normal"], cwd=root
        )
    except CoverageError:
        return {
            "status": "present",
            "head": head,
            "working_tree": {"status": "unknown", "dirty": None},
        }
    if status_result.returncode != 0:
        return {
            "status": "present",
            "head": head,
            "working_tree": {"status": "unknown", "dirty": None},
        }

    lines = [line for line in status_result.stdout.splitlines() if line]
    untracked = [line for line in lines if line[:2] == "??"]
    tracked = [line for line in lines if line[:2] not in {"??", "!!"}]
    staged = [line for line in tracked if line[:1] not in {" ", "?"}]
    unstaged = [line for line in tracked if line[1:2] not in {" ", "?"}]
    dirty = bool(tracked or untracked)
    return {
        "status": "present",
        "head": head,
        "working_tree": {
            "status": "dirty" if dirty else "clean",
            "dirty": dirty,
            "tracked_changes": bool(tracked),
            "untracked_changes": bool(untracked),
            "staged_changes": bool(staged),
            "unstaged_changes": bool(unstaged),
            "tracked_change_count": len(tracked),
            "untracked_change_count": len(untracked),
        },
    }


def _coverage_provenance(root: Path, objects: Sequence[Path], manifests: Sequence[Path],
                         build_dir: Path, compile_commands: Path | None = None
                         ) -> dict[str, Any]:
    """Collect reproducibility metadata without asserting source/binary equivalence."""
    compile_commands = compile_commands or (build_dir / "compile_commands.json")
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

    LLVM's JSON summary is the denominator. A manifest inventories production
    source paths; missing entries are reconciled by the caller with compiler
    object evidence, so mapping-free and header-only translation units remain
    distinguishable from omitted executable mappings.
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


def _reconcile_unmapped_sources(
    report: dict[str, Any], compiler_evidence: dict[str, Any] | None
) -> tuple[list[str], list[str], list[str]]:
    """Separate metadata-only headers from missing executable mappings.

    LLVM can emit a header that contributed no executable mapping to the final
    image (for example, a constants-only header included by a mapped TU). Such
    a file is useful provenance but cannot contribute to a source denominator.
    We waive it only after a successful object audit confirms that no retained
    object has executable mappings for that header. A header that the object
    audit says is executable but is absent or zero in the final export remains
    an integrity error.
    """
    unmapped = _sources_without_executable_mappings(report)
    if not compiler_evidence or compiler_evidence.get("status") != "ok":
        return unmapped, [], list(unmapped)

    mapping_free_keys = {
        _path_key(value) for value in compiler_evidence.get("mapping_free_sources", [])
    }
    header_mapping_keys = {
        _path_key(value)
        for value in compiler_evidence.get("object_mappings_without_source", [])
    }
    object_executable_keys = {
        _path_key(value)
        for value in compiler_evidence.get("object_executable_mappings", [])
    }
    mapping_free_headers: list[str] = []
    unexpected: list[str] = []
    for value in unmapped:
        key = _path_key(value)
        if key in mapping_free_keys or key in header_mapping_keys:
            continue
        if Path(value).suffix.lower() in _HEADER_SUFFIXES and key not in object_executable_keys:
            mapping_free_headers.append(value)
            continue
        unexpected.append(value)
    return unmapped, sorted(mapping_free_headers), sorted(unexpected)


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
        if value in {"-o", "/Fo"} and index + 1 < len(arguments):
            output = arguments[index + 1]
            return (directory / output).resolve() if not Path(output).is_absolute() else Path(output).resolve()
        if value.startswith("/Fo") and len(value) > 3:
            output = value[3:]
            return (directory / output).resolve() if not Path(output).is_absolute() else Path(output).resolve()
    return None


def _coverage_compile_flags(arguments: Sequence[str]) -> bool:
    flags = set(arguments)
    mapping = "-fcoverage-mapping" in flags or any(
        "-fcoverage-mapping" in value for value in arguments if value.startswith("/clang:")
    )
    profile = "-fprofile-instr-generate" in flags or any(
        "-fprofile-instr-generate" in value for value in arguments if value.startswith("/clang:")
    )
    return mapping and profile


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
        directory = Path(str(item.get("directory", path.parent)))
        if not directory.is_absolute():
            directory = (path.parent / directory).resolve()
        source = Path(item["file"])
        source = (directory / source).resolve() if not source.is_absolute() else source.resolve()
        arguments = _compile_command_arguments(item)
        output = _compile_command_output(arguments, directory)
        record = {
            "source": source,
            "directory": directory,
            "object": output,
            "arguments": arguments,
            "coverage_flags": _coverage_compile_flags(arguments),
        }
        by_source.setdefault(_path_key(source), []).append(record)
    return by_source


def _exported_paths(export: dict[str, Any], *, path_base: Path) -> set[str]:
    paths: set[str] = set()
    for chunk in export.get("data", []):
        if not isinstance(chunk, dict):
            continue
        for item in chunk.get("files", []):
            if not isinstance(item, dict) or not isinstance(item.get("filename"), str):
                continue
            path = Path(item["filename"])
            if not path.is_absolute():
                path = path_base / path
            paths.add(_path_key(path))
    return paths


def _exported_executable_paths(export: dict[str, Any], *, path_base: Path) -> set[str]:
    """Return source paths with at least one executable mapping in an export."""
    paths: set[str] = set()
    for chunk in export.get("data", []):
        if not isinstance(chunk, dict):
            continue
        for item in chunk.get("files", []):
            if not isinstance(item, dict) or not isinstance(item.get("filename"), str):
                continue
            summary = item.get("summary", {})
            if not isinstance(summary, dict):
                continue
            has_mappings = False
            for key in ("lines", "functions", "regions", "branches", "instantiations"):
                value = summary.get(key, {})
                if not isinstance(value, dict):
                    continue
                try:
                    if int(value.get("count", 0)) > 0:
                        has_mappings = True
                        break
                except (TypeError, ValueError):
                    continue
            if has_mappings:
                path = Path(item["filename"])
                if not path.is_absolute():
                    path = path_base / path
                paths.add(_path_key(path))
    return paths


def _compiler_source_evidence(llvm_cov: str, manifest: Sequence[Path],
                              compile_commands: Path, *, timeout: float | None,
                              profile: Path | None = None) -> dict[str, Any]:
    """Classify manifest TUs using their compiled object coverage maps.

    A source-level export cannot tell an include-only TU from a TU whose
    instrumented object was accidentally omitted from the executable.  The
    compiler's object output supplies that missing evidence: executable
    mappings in the object prove that it must be retained, including mappings
    contributed only by an inline or generated header. An instrumented object
    with no executable map is a mapping-free TU (for example a PCH-only
    source). A mapped object absent from the final executable remains an
    integrity failure in the caller.
    """
    by_source = _load_compile_commands(compile_commands)
    records: list[dict[str, Any]] = []
    mapping_free: list[str] = []
    mapped: list[str] = []
    object_mappings_without_source: list[str] = []
    errors: list[dict[str, Any]] = []
    for source in manifest:
        source_name = str(source)
        candidates = by_source.get(_path_key(source), [])
        coverage_candidates = [item for item in candidates if item["coverage_flags"]]
        if not candidates:
            errors.append({"source": source_name, "reason": "missing_compile_command"})
            records.append({"source": source_name, "classification": "missing_compile_command", "objects": []})
            continue
        if not coverage_candidates:
            errors.append({"source": source_name, "reason": "compile_command_missing_llvm_flags"})
            records.append({"source": source_name, "classification": "unprofiled_translation_unit", "objects": []})
            continue

        object_records: list[dict[str, Any]] = []
        source_mapped = False
        object_has_mappings = False
        source_had_error = False
        for candidate in coverage_candidates:
            object_path = candidate["object"]
            object_record = {"object": str(object_path) if object_path else None}
            if object_path is None or not object_path.is_file():
                object_record["status"] = "missing_object"
                errors.append({"source": source_name, "reason": "missing_compiler_object",
                               "object": str(object_path) if object_path else None})
                source_had_error = True
                object_records.append(object_record)
                continue
            # Object audits only need the file-level summaries.  Full line and
            # segment records make this one-export-per-TU audit needlessly
            # expensive, especially for a large production build.
            # LLVM 18 requires a profile even when only mapping counts are
            # inspected. Runtime hit counts never classify an object as mapped.
            profile_args = ["-instr-profile", str(profile)] if profile else ["-empty-profile"]
            export = _run([llvm_cov, "export", "-format=text", "-summary-only", *profile_args,
                           "-object", str(object_path)], cwd=candidate["directory"], timeout=timeout)
            if export.stderr:
                object_record["diagnostics"] = export.stderr
            if export.returncode != 0:
                no_mapping = (
                    export.returncode != 124
                    and "no coverage data found" in (export.stderr or "").lower()
                )
                object_record["status"] = "mapping_free" if no_mapping else "export_error"
                if not no_mapping:
                    errors.append({"source": source_name, "reason": "compiler_export_failed",
                                   "object": str(object_path), "stderr": export.stderr})
                    source_had_error = True
                object_records.append(object_record)
                continue
            try:
                object_export = json.loads(export.stdout)
            except json.JSONDecodeError:
                object_record["status"] = "invalid_export"
                errors.append({"source": source_name, "reason": "compiler_export_not_json",
                               "object": str(object_path)})
                source_had_error = True
                object_records.append(object_record)
                continue
            executable_paths = _exported_executable_paths(
                object_export, path_base=candidate["directory"]
            )
            has_object_mappings = bool(executable_paths)
            object_record["has_executable_mappings"] = has_object_mappings
            object_record["executable_mappings"] = sorted(executable_paths)
            object_has_mappings = object_has_mappings or has_object_mappings
            has_source_mapping = _path_key(source) in _exported_paths(
                object_export, path_base=candidate["directory"]
            )
            object_record["status"] = (
                "mapped" if has_source_mapping
                else "header_mapping_only" if has_object_mappings
                else "mapping_free"
            )
            object_records.append(object_record)
            if has_source_mapping:
                source_mapped = True

        if source_mapped:
            mapped.append(source_name)
            classification = "mapped"
        elif not object_has_mappings and not source_had_error:
            mapping_free.append(source_name)
            classification = "mapping_free_translation_unit"
        elif object_has_mappings:
            object_mappings_without_source.append(source_name)
            classification = "object_mappings_without_manifest_source"
        else:
            classification = "compiler_evidence_error"
        records.append({"source": source_name, "classification": classification, "objects": object_records})
    return {
        "status": "error" if errors else "ok",
        "path": str(compile_commands.resolve()),
        "sources": records,
        "mapped_sources": sorted(mapped),
        "mapping_free_sources": sorted(mapping_free),
        "object_mappings_without_source": sorted(object_mappings_without_source),
        "object_executable_mappings": sorted(
            {
                path
                for record in records
                for object_record in record.get("objects", [])
                for path in object_record.get("executable_mappings", [])
            }
        ),
        "errors": errors,
    }


def _profile_files(profile_dir: Path) -> list[Path]:
    return sorted(Path(value) for value in glob.glob(str(profile_dir / "*.profraw")))


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_report(path: Path, payload: dict[str, Any], provenance: dict[str, Any] | None = None) -> None:
    if provenance is not None:
        payload = dict(payload)
        payload["provenance"] = provenance
    _write_json(path, payload)


def run_coverage(args: argparse.Namespace) -> int:
    if args.metrics is None:
        args.metrics = ["lines", "functions", "branches"]
    root = Path(args.root).resolve()
    out_arg = Path(args.out)
    out = (root / out_arg if not out_arg.is_absolute() else out_arg).resolve()
    out.mkdir(parents=True, exist_ok=True)
    source_root = (root / args.source_root).resolve() if not Path(args.source_root).is_absolute() else Path(args.source_root).resolve()
    build_dir = (root / args.build_dir).resolve()
    objects = [(root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
               for value in args.object]
    compile_commands_arg = (Path(args.compile_commands) if args.compile_commands
                            else Path(args.build_dir) / "compile_commands.json")
    compile_commands = (root / compile_commands_arg
                        if not compile_commands_arg.is_absolute() else compile_commands_arg).resolve()
    manifest_files: list[Path] = []
    if args.manifest:
        for manifest_path in args.manifest:
            manifest_arg = Path(manifest_path)
            manifest_files.append(
                (root / manifest_arg if not manifest_arg.is_absolute() else manifest_arg).resolve()
            )
    provenance = _coverage_provenance(
        root, objects, manifest_files, build_dir, compile_commands
    )
    llvm_cov = _which("llvm-cov", args.llvm_cov)
    llvm_profdata = _which("llvm-profdata", args.llvm_profdata)
    missing_objects = [str(value) for value in objects if not value.is_file()]
    if missing_objects:
        raise CoverageError("coverage object does not exist: " + ", ".join(missing_objects))
    manifest: list[Path] = []
    for manifest_path in manifest_files:
        manifest.extend(_read_manifest(manifest_path, root))
    if manifest_files:
        manifest = sorted(set(manifest), key=lambda value: str(value))

    command = split_command(args.test_command)
    command = _expand_command(command, root=root, build=build_dir)
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
        _write_report(out / "coverage-report.json", {
            "status": "test_failed_no_evidence" if result.returncode != 0 else "no_test_evidence",
            "tests_failed": result.returncode != 0,
            "run": run_record,
        }, provenance)
        return 2
    if not profiles:
        _write_report(out / "coverage-report.json", {
            "status": "test_failed_no_profile" if result.returncode != 0 else "no_profile",
            "tests_failed": result.returncode != 0,
            "run": run_record,
            "objects": [str(value) for value in objects],
            "manifest": [str(Path(value).resolve()) for value in args.manifest] if args.manifest else [],
        }, provenance)
        return 2

    merged = out / "merged.profdata"
    merge_argv = [llvm_profdata, "merge", "-sparse", "-o", str(merged), *map(str, profiles)]
    merge = _run(merge_argv, cwd=root, timeout=args.timeout)
    if merge.returncode != 0:
        _write_report(out / "coverage-report.json", {
            "status": "profile_merge_failed", "run": run_record,
            "merge": {"argv": merge_argv, "returncode": merge.returncode,
                       "stdout": merge.stdout, "stderr": merge.stderr},
        }, provenance)
        return 2

    # The report and all integrity checks consume only file summaries.  Keep
    # the compact export as the machine-readable evidence artifact; callers
    # needing line/segment detail can rerun llvm-cov without --summary-only.
    export_argv = [llvm_cov, "export", "-format=text", "-summary-only",
                   "-instr-profile", str(merged)]
    for obj in objects:
        export_argv.extend(["-object", str(obj)])
    export = _run(export_argv, cwd=root, timeout=args.timeout)
    if export.returncode != 0:
        _write_report(out / "coverage-report.json", {
            "status": "export_failed", "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode,
                       "stdout": export.stdout, "stderr": export.stderr},
        }, provenance)
        return 2
    try:
        exported = json.loads(export.stdout)
    except json.JSONDecodeError as exc:
        _write_report(out / "coverage-report.json", {
            "status": "invalid_export", "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode,
                       "stdout": export.stdout, "stderr": export.stderr},
        }, provenance)
        raise CoverageError(f"llvm-cov export was not JSON: {exc}") from exc
    try:
        report, missing = parse_export(exported, source_root=source_root,
                                       manifest=manifest, path_base=root)
    except CoverageError as exc:
        _write_report(out / "coverage-report.json", {
            "status": "invalid_export", "error": str(exc), "run": run_record,
            "export": {"argv": export_argv, "returncode": export.returncode},
        }, provenance)
        raise
    compiler_evidence: dict[str, Any] | None = None
    if manifest:
        if compile_commands.is_file():
            compiler_evidence = _compiler_source_evidence(
                llvm_cov, manifest, compile_commands, timeout=args.timeout, profile=merged
            )
        else:
            compiler_evidence = {
                "status": "not_run",
                "reason": "compile_commands.json was not provided or generated",
                "path": str(compile_commands),
                "mapped_sources": [],
                "mapping_free_sources": [],
                "object_mappings_without_source": [],
                "object_executable_mappings": [],
                "errors": [],
            }
    unmapped, mapping_free_headers, unexpected_unmapped = _reconcile_unmapped_sources(
        report, compiler_evidence
    )
    mapping_free_keys = {
        _path_key(value) for value in (compiler_evidence or {}).get("mapping_free_sources", [])
    }
    header_mapping_keys = {
        _path_key(value)
        for value in (compiler_evidence or {}).get("object_mappings_without_source", [])
    }
    source_without_direct_mapping_keys = mapping_free_keys | header_mapping_keys
    linked_missing = [
        value for value in missing
        if _path_key(value) not in source_without_direct_mapping_keys
    ]
    exported_paths = {_path_key(item["filename"]) for item in report["files"]}
    missing_object_mappings = sorted(
        path for path in (compiler_evidence or {}).get("object_executable_mappings", [])
        if _under(Path(path), source_root) and _path_key(path) not in exported_paths
    )
    compiler_errors = list((compiler_evidence or {}).get("errors", []))
    empty_metrics = [
        key for key in args.metrics
        if key not in report["summary"] or int(report["summary"][key].get("count", 0)) <= 0
    ]
    integrity_failures = {
        "compiler_evidence": compiler_errors,
        "missing_compiled_source_mappings": linked_missing,
        "missing_object_executable_mappings": missing_object_mappings,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "empty_metrics": empty_metrics,
    }
    integrity_failed = any(integrity_failures.values())
    if result.returncode != 0:
        status = "test_failed"
    elif evidence <= 0:
        status = "no_test_evidence"
    elif integrity_failed:
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
        "missing_compiled_source_mappings": linked_missing,
        "missing_object_executable_mappings": missing_object_mappings,
        "mapping_free_headers": mapping_free_headers,
        "mapping_free_sources": sorted(
            (compiler_evidence or {}).get("mapping_free_sources", [])
        ),
        "object_mappings_without_source": sorted(
            (compiler_evidence or {}).get("object_mappings_without_source", [])
        ),
        "sources_without_executable_mappings": unmapped,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "integrity_status": "error" if integrity_failed else "pass",
        "integrity_failures": integrity_failures,
        "compiler_evidence": compiler_evidence,
        "run": run_record,
        "merge": {"argv": merge_argv, "returncode": merge.returncode},
        "export": {"argv": export_argv, "returncode": export.returncode},
        "tool_versions": {
            "llvm-cov": _run([llvm_cov, "--version"]).stdout.strip(),
            "llvm-profdata": _run([llvm_profdata, "--version"]).stdout.strip(),
        },
    })
    _write_report(out / "coverage-report.json", report, provenance)
    # Keep the raw export as evidence and for future report consumers.
    (out / "coverage-export.json").write_text(export.stdout, encoding="utf-8")
    if result.returncode != 0 or evidence <= 0:
        return 2
    if integrity_failed or any(
        not _meets_gate(report["summary"], key, args.threshold) for key in args.metrics
    ):
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
    run.add_argument("--compile-commands",
                     help="compile_commands.json used to classify mapping-free translation units")
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
                root = Path(args.root).resolve()
                build_dir = (root / args.build_dir).resolve()
                compile_commands_arg = (
                    Path(args.compile_commands) if args.compile_commands
                    else Path(args.build_dir) / "compile_commands.json"
                )
                compile_commands = (
                    root / compile_commands_arg
                    if not compile_commands_arg.is_absolute() else compile_commands_arg
                ).resolve()
                objects = [
                    (root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
                    for value in (getattr(args, "object", None) or ())
                ]
                manifests = [
                    (root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
                    for value in (getattr(args, "manifest", None) or ())
                ]
                _write_report(
                    out / "coverage-report.json",
                    {"status": "error", "error": str(exc)},
                    _coverage_provenance(
                        root, objects, manifests, build_dir, compile_commands
                    ),
                )
            except OSError:
                pass
        print(f"slicer_coverage: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
