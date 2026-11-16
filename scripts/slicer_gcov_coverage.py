"""GCC gcov backend for the shared slicer_coverage command-line runner.

This backend deliberately reports its own physical-line universe. It does not
import, approximate, or replace the LLVM denominator.
"""

from __future__ import annotations

from collections import Counter
from concurrent.futures import FIRST_COMPLETED, ThreadPoolExecutor, wait
import gzip
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET
from typing import Any


FULL_TEST_TARGETS = (
    "libslic3r_tests", "fff_print_tests", "sla_print_tests",
    "libnest2d_tests", "filament_group_tests",
)
PRODUCTION_TARGETS = ("libslic3r", "libslic3r_cgal")
REQUIRED_MANIFEST_TARGETS = set(PRODUCTION_TARGETS + FULL_TEST_TARGETS)


def validate_gcov_jobs(value: Any) -> int:
    """Require an explicit positive bound for concurrent gcov processes."""
    if isinstance(value, bool) or not isinstance(value, int) or value < 1:
        raise ValueError("--gcov-jobs must be a positive integer")
    return value


def validate_gcov_output(returncode: int, emitted_count: int, source: str, *,
                         stderr: str = "") -> None:
    """Keep the existing fail-closed gcov process/output cardinality gate."""
    if returncode == 0 and emitted_count <= 1:
        return
    details = [f"returncode={returncode}", f"emitted_json_count={emitted_count}"]
    diagnostic = stderr.strip()
    if diagnostic:
        details.append(f"stderr={diagnostic[:2000]}")
    raise ValueError(f"gcov output validation failed for {source}: {'; '.join(details)}")


def run_gcov_commands(
        commands: list[tuple[list[str], Path, Path, Path, str]], *, jobs: int,
        timeout: float, common: Any) -> list[Any]:
    """Run isolated gcov commands, returning results in input order.

    Each tuple is (argv, cwd, stdout log, stderr log, source name). At most
    ``jobs`` futures are submitted at once. A worker saves its logs and checks
    the process/output cardinality before it can succeed. On failure, no more
    work is submitted, pending futures are canceled, and running workers settle
    before this function raises. Successful results retain input order.
    """
    jobs = validate_gcov_jobs(jobs)

    def invoke(command: tuple[list[str], Path, Path, Path, str]) -> Any:
        argv, cwd, stdout_path, stderr_path, source = command
        result = common._run(argv, cwd=cwd, timeout=timeout)
        stdout_path.write_text(result.stdout or "", encoding="utf-8")
        stderr_path.write_text(result.stderr or "", encoding="utf-8")
        emitted_count = sum(path.is_file() for path in cwd.glob("*.gcov.json.gz"))
        validate_gcov_output(result.returncode, emitted_count, source, stderr=result.stderr or "")
        return result

    if not commands:
        return []
    if jobs == 1 or len(commands) == 1:
        return [invoke(command) for command in commands]
    worker_count = min(jobs, len(commands))
    executor = ThreadPoolExecutor(max_workers=worker_count, thread_name_prefix="gcov")
    pending: dict[Any, int] = {}
    results: list[Any] = [None] * len(commands)
    next_index = 0

    def submit_one() -> None:
        nonlocal next_index
        future = executor.submit(invoke, commands[next_index])
        pending[future] = next_index
        next_index += 1

    try:
        while next_index < worker_count:
            submit_one()
        while pending:
            done, _ = wait(tuple(pending), return_when=FIRST_COMPLETED)
            # Consume every result already ready before replenishing the pool;
            # this prevents a nearby completed failure from sitting behind a
            # successful result and causing avoidable new submissions.
            ready = sorted((future for future in pending if future in done or future.done()),
                           key=pending.__getitem__)
            for future in ready:
                index = pending.pop(future)
                results[index] = future.result()
            if any(future.done() for future in pending):
                continue
            while next_index < len(commands) and len(pending) < worker_count:
                if any(future.done() for future in pending):
                    break
                submit_one()
        return results
    except BaseException:
        # Future.cancel() only cancels work that has not started; running gcov
        # subprocesses are allowed to settle (or hit their own timeout).
        for future in pending:
            future.cancel()
        if pending:
            wait(tuple(pending))
            for future in pending:
                try:
                    future.result()
                except BaseException:
                    pass
        raise
    finally:
        executor.shutdown(wait=True)


def _fail(common: Any, message: str) -> None:
    raise common.CoverageError(message)


def _target_from_manifest(path: Path) -> str:
    try:
        first = path.read_text(encoding="utf-8").splitlines()[0]
    except (OSError, IndexError) as exc:
        raise ValueError(f"empty or unreadable source manifest: {path}") from exc
    match = re.fullmatch(r"# Sources compiled into ([A-Za-z0-9_+-]+)", first)
    if not match:
        raise ValueError(f"manifest has no CMake target identity: {path}")
    return match.group(1)


def _argv(entry: dict[str, Any]) -> list[str]:
    values = entry.get("arguments")
    if isinstance(values, list) and all(isinstance(value, str) for value in values):
        return values
    command = entry.get("command")
    if isinstance(command, str):
        import shlex
        return shlex.split(command)
    return []


def _output_arg(argv: list[str], directory: Path) -> Path | None:
    for index, value in enumerate(argv):
        if value == "-o" and index + 1 < len(argv):
            candidate = Path(argv[index + 1])
            return (directory / candidate).resolve() if not candidate.is_absolute() else candidate.resolve()
        if value.startswith("-o") and len(value) > 2:
            candidate = Path(value[2:])
            return (directory / candidate).resolve() if not candidate.is_absolute() else candidate.resolve()
    return None


def _effective_option(argv: list[str], prefix: str) -> str | None:
    found = [value[len(prefix):] for value in argv if value.startswith(prefix)]
    return found[-1] if found else None


def validate_gcc_compile_argv(argv: list[str], *, source: Path,
                              source_root: Path, filter_source: Path | None = None) -> bool:
    """Validate gcov counters/filter and return whether the source is selected."""
    if "--coverage" not in argv or _effective_option(argv, "-fprofile-update=") != "atomic":
        raise ValueError("GCC counters are not atomically instrumented")
    filters = [value.split("=", 1)[1] for value in argv if value.startswith("-fprofile-filter-files=")]
    if len(filters) != 1:
        raise ValueError("expected one explicit project source filter")
    try:
        matches = re.fullmatch(filters[0], str(filter_source or source)) is not None
    except re.error as exc:
        raise ValueError(f"invalid GCC source filter: {exc}") from exc
    try:
        under = os.path.commonpath([os.path.realpath(source), os.path.realpath(source_root)]) == \
            os.path.realpath(source_root)
    except ValueError:
        under = False
    if under and not matches:
        raise ValueError("project source excluded by GCC filter")
    if not under and matches:
        raise ValueError("non-project source included by GCC filter")
    return matches


def compare_test_identities(selected: list[str], junit_names: Counter[str]) -> dict[str, list[str]]:
    expected = Counter(selected)
    return {"missing": sorted((expected - junit_names).elements()),
            "unexpected": sorted((junit_names - expected).elements())}


def validate_note_inventory(expected: set[Path], actual: set[Path]) -> list[str]:
    missing = sorted(str(path) for path in expected - actual)
    if missing:
        raise ValueError("selected manifest notes missing from build inventory: " + ", ".join(missing))
    return sorted(str(path) for path in actual - expected)


def collect_build_notes(commands: list[dict[str, Any]], root: Path) -> set[Path]:
    """Inventory notes from current CDB objects, excluding staged report copies."""
    notes: set[Path] = set()
    for entry in commands:
        if not isinstance(entry, dict):
            continue
        directory = Path(entry.get("directory", root))
        if not directory.is_absolute():
            directory = root / directory
        output = _output_arg(_argv(entry), directory)
        if output is None:
            continue
        note = output.with_suffix(".gcno")
        if note.is_file():
            notes.add(note.resolve())
    return notes


def _cmake_compiler_provenance(build_dir: Path) -> tuple[str, str, Path]:
    candidates = sorted((build_dir / "CMakeFiles").glob("*/CMakeCXXCompiler.cmake"))
    if len(candidates) != 1:
        raise ValueError(f"expected one generated CMake C++ compiler record, found {len(candidates)}")
    text = candidates[0].read_text(encoding="utf-8")
    values = dict(re.findall(r'^set\(CMAKE_CXX_COMPILER_(ID|VERSION)\s+"([^"]*)"\)',
                             text, re.MULTILINE))
    if not values.get("ID") or not values.get("VERSION"):
        raise ValueError(f"generated CMake compiler record is incomplete: {candidates[0]}")
    return values["ID"], values["VERSION"], candidates[0]


_SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx",
                    ".inc", ".inl", ".ipp", ".tcc", ".txx"}
_COMPILE_SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".m", ".mm"}


def validate_threshold(threshold: float) -> float:
    """Reject NaN, infinities, and percentages outside the meaningful range."""
    if not math.isfinite(threshold) or not 0.0 <= threshold <= 100.0:
        raise ValueError("coverage threshold must be a finite percentage from 0 through 100")
    return threshold


def _is_pch_compile_source(source: Path, build_dir: Path) -> bool:
    """CMake's generated PCH compilation is not an independent manifest TU."""
    try:
        in_build = os.path.commonpath([os.path.realpath(source),
                                       os.path.realpath(build_dir)]) == os.path.realpath(build_dir)
    except ValueError:
        in_build = False
    return in_build and source.name.lower().startswith("cmake_pch")


def validate_manifest_compile_sources(manifest_sources: list[Path],
                                      compiled_sources: list[Path],
                                      build_dir: Path) -> list[str]:
    """Require each real target C/C++ compile row to appear in its manifest.

    Generated CMake PCH translation units are excluded; rows from other targets
    are excluded by the caller before this target-local check.
    """
    expected = {os.path.normcase(os.path.realpath(path)) for path in manifest_sources}
    compiled = {
        os.path.normcase(os.path.realpath(path))
        for path in compiled_sources
        if path.suffix.lower() in _COMPILE_SOURCE_SUFFIXES
        and not _is_pch_compile_source(path, build_dir)
    }
    missing = sorted(compiled - expected)
    if missing:
        raise ValueError("compiled target source missing from coverage manifest: " + ", ".join(missing))
    return sorted(expected - compiled)


def _source_tree_snapshot(source_root: Path) -> dict[str, str]:
    return {str(path.resolve()): _hash_file(path)
            for path in sorted(source_root.rglob("*"))
            if path.is_file() and path.suffix.lower() in _SOURCE_SUFFIXES}


def _snapshot_digest(values: dict[str, str]) -> str:
    content = "".join(f"{path}\0{digest}\n" for path, digest in sorted(values.items()))
    return hashlib.sha256(content.encode("utf-8")).hexdigest()


def _artifact_snapshot(common: Any, paths: set[Path], *, require_present: bool = True
                       ) -> dict[str, str | None]:
    snapshot: dict[str, str | None] = {}
    for path in sorted(paths):
        provenance = common._artifact_provenance(path)
        digest = provenance.get("sha256")
        if provenance.get("status") != "present" or not isinstance(digest, str) or not digest:
            if require_present:
                raise ValueError(f"required build artifact is missing or unreadable: {path}")
            snapshot[str(path)] = None
        else:
            snapshot[str(path)] = digest
    return snapshot


def gcda_path_for_object(prefix: Path, object_path: Path) -> Path:
    """Map GCOV_PREFIX/STRIP=0 output back from an absolute object filename."""
    gcda_source = object_path.with_suffix(".gcda")
    return prefix / Path(str(gcda_source).lstrip(os.sep))


def _read_gcov_header(path: Path, expected_kind: str) -> dict[str, Any]:
    """Read only the portable GCOV magic, version, and stamp header fields."""
    with path.open("rb") as stream:
        header = stream.read(12)
    if len(header) < 12:
        raise ValueError(f"GCOV file header is truncated: {path}")
    magic_kinds = {
        b"oncg": ("gcno", "little"), b"adcg": ("gcda", "little"),
        b"gcno": ("gcno", "big"), b"gcda": ("gcda", "big"),
    }
    detected = magic_kinds.get(header[:4])
    if detected is None:
        raise ValueError(f"GCOV file has an unknown magic header: {path}")
    kind, byteorder = detected
    if kind != expected_kind:
        raise ValueError(f"GCOV file has {kind} magic; expected {expected_kind}: {path}")
    version_bytes = header[4:8]
    if byteorder == "little":
        version_bytes = version_bytes[::-1]
    try:
        version = version_bytes.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ValueError(f"GCOV file version is not ASCII: {path}") from exc
    return {
        "kind": kind, "version": version,
        "stamp": int.from_bytes(header[8:12], byteorder=byteorder),
    }


def inspect_gcov_profile_epochs(profile_roots: list[Path], *,
                                expected_notes: set[Path],
                                required_gcda_note_groups: dict[str, set[Path]] | None = None) -> dict[str, Any]:
    """Check every GCDA header against the GCNO for its absolute object path.

    GCOV_PREFIX with GCOV_PREFIX_STRIP=0 preserves the absolute object path
    below the profile root, so removing the prefix reconstructs its .gcno path.
    """
    if not expected_notes:
        raise ValueError("GCOV epoch validation requires a nonempty build-note allowset")
    expected = {path.resolve(strict=True) for path in expected_notes}
    required_groups = required_gcda_note_groups or {}
    normalized_groups = {
        name: {path.resolve(strict=True) for path in notes}
        for name, notes in required_groups.items()
    }
    for name, notes in normalized_groups.items():
        if not name or not notes or not notes <= expected:
            raise ValueError(f"required GCDA note group is empty or outside the build-note allowset: {name!r}")
    report: dict[str, Any] = {
        "schema": "slicer-gcov-profile-epoch-v1", "status": "pass",
        "profile_roots": [], "checked_gcda_count": 0, "mismatch_count": 0,
        "entries": [], "mismatches": [],
        "required_gcda_note_groups": {
            name: {"notes": sorted(map(str, notes)), "matched_note": None}
            for name, notes in sorted(normalized_groups.items())
        },
    }
    for profile_root_value in profile_roots:
        profile_root = Path(profile_root_value)
        root_record: dict[str, Any] = {"path": str(profile_root), "gcda_count": 0}
        report["profile_roots"].append(root_record)
        if profile_root.is_symlink():
            report["mismatches"].append({"profile_root": str(profile_root),
                                         "status": "symlink_profile_root"})
            continue
        try:
            profile_root = profile_root.resolve(strict=True)
        except OSError as exc:
            report["mismatches"].append({"profile_root": str(profile_root),
                                         "status": "profile_root_unavailable",
                                         "error": str(exc)})
            continue
        if not profile_root.is_dir():
            report["mismatches"].append({"profile_root": str(profile_root),
                                         "status": "profile_root_not_directory"})
            continue
        try:
            gcda_paths = sorted(profile_root.rglob("*.gcda"))
        except OSError as exc:
            report["mismatches"].append({"profile_root": str(profile_root),
                                         "status": "profile_scan_error", "error": str(exc)})
            continue
        for gcda_path in gcda_paths:
            row: dict[str, Any] = {
                "profile_root": str(profile_root), "gcda_path": str(gcda_path),
            }
            report["checked_gcda_count"] += 1
            root_record["gcda_count"] += 1
            try:
                if gcda_path.is_symlink() or not gcda_path.is_file():
                    raise ValueError("GCDA profile is not a regular file")
                relative = gcda_path.relative_to(profile_root)
                if any(part in {".", ".."} for part in relative.parts):
                    raise ValueError("GCDA path escapes its profile root")
                gcno_path = Path(os.sep, *relative.parts).with_suffix(".gcno")
                row["gcno_path"] = str(gcno_path)
                if gcno_path.is_symlink() or not gcno_path.is_file():
                    raise ValueError(f"matching GCNO note is missing or unsafe: {gcno_path}")
                resolved_note = gcno_path.resolve(strict=True)
                if resolved_note not in expected:
                    raise ValueError(f"GCDA maps to a GCNO outside the selected build notes: {gcno_path}")
                for group_name, group_notes in normalized_groups.items():
                    if resolved_note in group_notes:
                        report["required_gcda_note_groups"][group_name]["matched_note"] = \
                            str(resolved_note)
                gcda_header = _read_gcov_header(gcda_path, "gcda")
                gcno_header = _read_gcov_header(gcno_path, "gcno")
                row.update({
                    "gcda_version": gcda_header["version"],
                    "gcno_version": gcno_header["version"],
                    "gcda_stamp": gcda_header["stamp"],
                    "gcno_stamp": gcno_header["stamp"],
                    "gcda_size_bytes": gcda_path.stat().st_size,
                    "gcno_size_bytes": gcno_path.stat().st_size,
                })
                if (gcda_header["version"] != gcno_header["version"] or
                        gcda_header["stamp"] != gcno_header["stamp"]):
                    row["status"] = "epoch_mismatch"
                else:
                    row["status"] = "match"
            except (OSError, ValueError) as exc:
                row["status"] = "invalid_profile_binding"
                row["error"] = str(exc)
            report["entries"].append(row)
            if row["status"] != "match":
                report["mismatches"].append(row)
    for name, group in report["required_gcda_note_groups"].items():
        if group["matched_note"] is None:
            missing = {
                "status": "required_gcda_missing", "required_group": name,
                "profile_roots": [str(Path(value)) for value in profile_roots],
                "error": "instrumented Catch listing emitted no GCDA for this executable's CDB notes",
            }
            report["mismatches"].append(missing)
    report["mismatch_count"] = len(report["mismatches"])
    if report["mismatch_count"]:
        report["status"] = "mismatch"
    return report


def format_gcov_profile_epoch_failure(report: dict[str, Any], *, limit: int = 5) -> str:
    """Return a concise diagnostic while the full per-file receipt stays saved."""
    mismatches = report.get("mismatches", [])
    descriptions: list[str] = []
    for row in mismatches[:limit]:
        if row.get("status") == "epoch_mismatch":
            descriptions.append(
                f"{row.get('gcda_path')} has version/stamp "
                f"{row.get('gcda_version')}/{row.get('gcda_stamp')}, but "
                f"{row.get('gcno_path')} has "
                f"{row.get('gcno_version')}/{row.get('gcno_stamp')}")
        else:
            descriptions.append(
                f"{row.get('gcda_path', row.get('profile_root'))}: {row.get('error', row.get('status'))}")
    extra = len(mismatches) - len(descriptions)
    if extra > 0:
        descriptions.append(f"and {extra} more mismatched GCDA files")
    return (f"GCOV profile epoch check found {report.get('mismatch_count', len(mismatches))} "
            f"mismatched files after checking {report.get('checked_gcda_count', 0)} GCDA files: "
            + "; ".join(descriptions))


def create_gcov_data_dir(root: Path, out: Path, override: str | None) -> Path:
    """Create an exclusive profile epoch, optionally on a separate filesystem."""
    candidate = Path(override) if override is not None else out / "gcov-data"
    if not candidate.is_absolute():
        candidate = root / candidate
    if candidate.is_symlink():
        raise ValueError(f"gcov data path must not be a symlink: {candidate}")
    prefix = candidate.resolve()
    try:
        prefix.mkdir(parents=True)
    except OSError as exc:
        raise ValueError(f"gcov data directory must be new and writable: {prefix}: {exc}") from exc
    return prefix


def _record_source_path(record: dict[str, Any], path_base: Path | None = None) -> Path:
    if not isinstance(record, dict) or not isinstance(record.get("file"), str):
        raise ValueError("gcov JSON file record is missing its source path")
    raw_path = Path(record["file"])
    if not raw_path.is_absolute():
        base = Path(record.get("current_working_directory", path_base or Path.cwd()))
        raw_path = base / raw_path
    return raw_path.resolve()


def _is_under_path(path: Path, root: Path) -> bool:
    try:
        return os.path.commonpath([os.path.realpath(path), os.path.realpath(root)]) == \
            os.path.realpath(root)
    except ValueError:
        return False


def project_gcov_records(records: list[dict[str, Any]], source_root: Path,
                         path_base: Path | None = None) -> list[dict[str, Any]]:
    """Keep only project records in memory; each full raw TU JSON remains saved."""
    return [record for record in records
            if _is_under_path(_record_source_path(record, path_base), source_root)]


def classify_translation_unit(source: Path, *, filter_matches_project: bool,
                              records: list[dict[str, Any]], source_root: Path,
                              gcov_output: str) -> tuple[str, list[str]]:
    """Classify source and header mappings; unknown missing output is an error."""
    project_records: list[tuple[str, bool]] = []
    for record in records:
        resolved = _record_source_path(record)
        if _is_under_path(resolved, source_root):
            project_records.append((str(resolved), bool(record.get("lines"))))
    own = str(source.resolve())
    if any(path == own and has_lines for path, has_lines in project_records):
        classification = "source_mapped"
    elif any(has_lines for path, has_lines in project_records):
        classification = "header_only_mapping"
    elif project_records:
        classification = "mapping_free_translation_unit"
    elif not filter_matches_project:
        classification = "excluded_by_project_source_filter"
    elif re.search(r"no executable lines", gcov_output, re.IGNORECASE):
        classification = "mapping_free_translation_unit"
    else:
        raise ValueError(f"gcov emitted no project mapping or explicit no-executable-lines record for {source}")
    return classification, sorted({path for path, _ in project_records})


def validate_gcov_line_records(records: list[dict[str, Any]], *, source_root: Path,
                               path_base: Path | None = None) -> dict[str, Any]:
    """Deduplicate gcov JSON lines by physical source path and line number.

    gcov can emit repeated line entries within one source record and the same
    inline/header line in multiple translation units. A physical line is hit
    when any record has a positive execution count, so duplicate counts merge
    with max rather than being summed.
    """
    physical: dict[str, dict[int, int]] = {}
    display_paths: dict[str, str] = {}
    for record in records:
        source = _record_source_path(record, path_base)
        key = os.path.normcase(os.path.realpath(source))
        if not _is_under_path(source, source_root):
            continue
        display_paths[key] = str(source)
        source_lines = physical.setdefault(key, {})
        lines = record.get("lines", [])
        if not isinstance(lines, list):
            raise ValueError(f"gcov lines field is not an array: {source}")
        for line in lines:
            if not isinstance(line, dict):
                raise ValueError(f"gcov line entry is not an object: {source}")
            try:
                number, count = int(line["line_number"]), int(line["count"])
            except (KeyError, TypeError, ValueError) as exc:
                raise ValueError(f"invalid gcov line entry: {source}") from exc
            if number < 1 or count < 0:
                raise ValueError(f"invalid gcov line number/count: {source}:{number}")
            source_lines[number] = max(source_lines.get(number, 0), count)

    by_file: dict[str, dict[str, int | float]] = {}
    physical_lines: list[dict[str, Any]] = []
    for path_key in sorted(physical):
        source_entries = sorted(physical[path_key].items())
        counts = [count for _, count in source_entries]
        mapped = len(counts)
        hit = sum(count > 0 for count in counts)
        by_file[display_paths[path_key]] = {
            "hit_lines": hit, "mapped_lines": mapped,
            "percent": 100.0 * hit / mapped if mapped else 0.0,
        }
        physical_lines.extend({"filename": display_paths[path_key], "line_number": number,
                               "count": count} for number, count in source_entries)
    mapped = sum(len(lines) for lines in physical.values())
    hit = sum(count > 0 for lines in physical.values() for count in lines.values())
    return {
        "files": by_file,
        "physical_lines": physical_lines,
        "summary": {"hit_lines": hit, "mapped_lines": mapped,
                    "percent": 100.0 * hit / mapped if mapped else 0.0},
        "mapped_source_paths": sorted(display_paths.values()),
    }


def _hash_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


class GcovProfileMergeError(ValueError):
    """A profile merge failed closed; no merged data may receive coverage credit."""

    def __init__(self, message: str, report: dict[str, Any]):
        super().__init__(message)
        self.report = report


def _is_transient_gcov_tool_allocator_abort(result: Any) -> bool:
    """Match only the observed GCC 13 allocator abort eligible for one retry."""
    stderr = result.stderr or ""
    return (result.returncode == -6 and isinstance(stderr, str) and
            "free(): invalid pointer" in stderr)


def resolve_gcov_tool(gcov: str, explicit: str | None = None) -> Path:
    """Find gcov-tool alongside the selected gcov, then verify it before use."""
    candidate: str | None = explicit
    if candidate is None:
        gcov_path = Path(gcov)
        if gcov_path.name.startswith("gcov"):
            sibling_name = gcov_path.name.replace("gcov", "gcov-tool", 1)
            sibling = gcov_path.with_name(sibling_name)
            if sibling.is_file():
                candidate = str(sibling)
        if candidate is None:
            candidate = shutil.which("gcov-tool")
    if candidate is None:
        raise ValueError("matching gcov-tool was not found; set --gcov-tool to the GCC versioned executable")
    resolved = Path(candidate)
    if not resolved.is_absolute():
        found = shutil.which(candidate)
        if found:
            resolved = Path(found)
    try:
        resolved = resolved.resolve(strict=True)
    except OSError as exc:
        raise ValueError(f"gcov-tool executable does not exist: {candidate}") from exc
    if not resolved.is_file() or not os.access(resolved, os.X_OK):
        raise ValueError(f"gcov-tool is not an executable file: {resolved}")
    return resolved


def _gcda_profile_inventory(root: Path) -> dict[str, str]:
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"gcov profile root must be a real directory: {root}")
    inventory: dict[str, str] = {}
    for path in root.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"gcov profile tree must not contain symlinks: {path}")
        if path.is_file() and path.suffix == ".gcda":
            inventory[path.relative_to(root).as_posix()] = _hash_file(path)
    return inventory


def _gcda_inventory_digest(inventory: dict[str, str]) -> str:
    encoded = "".join(f"{name}\0{digest}\n" for name, digest in sorted(inventory.items()))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _write_profile_merge_report(path: Path, report: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink():
        raise ValueError(f"refusing to replace gcov profile merge report: {path}")
    with path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())


def _copy_profile_inventory(source_root: Path, inventory: dict[str, str],
                            destination_root: Path) -> None:
    """Copy only the GCDA paths selected by an already validated inventory."""
    if destination_root.exists() or destination_root.is_symlink():
        if destination_root.is_symlink() or not destination_root.is_dir():
            raise ValueError(f"gcov profile copy destination is not a regular directory: {destination_root}")
    else:
        destination_root.mkdir(mode=0o700, parents=True, exist_ok=False)
    for relative_path in sorted(inventory):
        relative = Path(relative_path)
        if relative.is_absolute() or ".." in relative.parts or relative.suffix != ".gcda":
            raise ValueError(f"unsafe relative GCDA inventory path: {relative_path}")
        source = source_root / relative
        if source.is_symlink() or not source.is_file():
            raise ValueError(f"GCDA inventory changed before copy: {source}")
        expected_digest = inventory[relative_path]
        if _hash_file(source) != expected_digest:
            raise ValueError(f"GCDA source hash changed before copy: {source}")
        destination = destination_root / relative
        if destination.exists() or destination.is_symlink():
            raise ValueError(f"gcov profile copy would replace an existing path: {destination}")
        destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        if _hash_file(destination) != expected_digest:
            raise ValueError(f"GCDA copy hash does not match its input inventory: {destination}")


def _merge_profile_inventories(
        profiles: list[tuple[Path, dict[str, str]]], *, gcov_tool: Path,
        gcc_version: str,
        stage_root: Path, timeout: float, common: Any,
        commands: list[dict[str, Any]], phase: str,
        category: str | None = None) -> tuple[Path, set[str]]:
    """Merge profile trees and verify every intermediate path union."""
    if len(profiles) < 2:
        raise ValueError("gcov-tool merge requires at least two nonempty profiles")
    current_input, first_inventory = profiles[0]
    current_inventory = first_inventory
    current_paths = set(first_inventory)
    for merge_index, (right_input, right_inventory) in enumerate(profiles[1:], 1):
        expected_paths = current_paths | set(right_inventory)
        for attempt in range(1, 3):
            destination = stage_root / (
                f"{phase}-{category or 'all'}-step-{merge_index:02d}-attempt-{attempt:02d}")
            destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            if destination.exists() or destination.is_symlink():
                raise ValueError(f"gcov merge destination is not fresh: {destination}")
            argv = [str(gcov_tool), "merge", "--weight", "1,1", "--output",
                    str(destination), str(current_input), str(right_input)]
            record: dict[str, Any] = {
                "phase": phase, "category": category, "attempt": attempt, "argv": argv,
                "left_input": str(current_input), "right_input": str(right_input),
                "output": str(destination),
            }
            input_roots = ((current_input, current_inventory), (right_input, right_inventory))
            input_snapshots: list[dict[str, str]] = []
            for input_root, expected_inventory in input_roots:
                before_inventory = _gcda_profile_inventory(input_root)
                if before_inventory != expected_inventory:
                    raise ValueError(f"gcov-tool merge input changed before command: {input_root}")
                input_snapshots.append(before_inventory)

            started = time.monotonic()
            try:
                result = common._run(argv, cwd=destination.parent, timeout=timeout)
            except Exception as exc:
                record.update({
                    "duration_seconds": round(time.monotonic() - started, 3),
                    "exception": {"type": type(exc).__name__, "message": str(exc)},
                })
                commands.append(record)
                raise ValueError(f"gcov-tool merge command raised {type(exc).__name__}: {exc}") from exc
            record.update({
                "returncode": result.returncode, "stdout": result.stdout,
                "stderr": result.stderr,
                "duration_seconds": round(time.monotonic() - started, 3),
            })
            commands.append(record)
            for (input_root, _), before_inventory in zip(input_roots, input_snapshots):
                if _gcda_profile_inventory(input_root) != before_inventory:
                    record["input_profiles_unchanged"] = False
                    raise ValueError(f"gcov-tool merge modified an input profile: {input_root}")
            record["input_profiles_unchanged"] = True

            if result.returncode != 0:
                if (attempt == 1 and gcc_version.split(".", maxsplit=1)[0] == "13" and
                        _is_transient_gcov_tool_allocator_abort(result)):
                    record["retry_reason"] = "gcc_allocator_abort_free_invalid_pointer"
                    continue
                raise ValueError(f"gcov-tool merge exited {result.returncode}")
            if not destination.is_dir() or destination.is_symlink():
                raise ValueError("gcov-tool merge did not create a regular output directory")
            output_inventory = _gcda_profile_inventory(destination)
            record["output_gcda_count"] = len(output_inventory)
            record["output_inventory_sha256"] = _gcda_inventory_digest(output_inventory)
            if set(output_inventory) != expected_paths:
                missing = sorted(expected_paths - set(output_inventory))
                unexpected = sorted(set(output_inventory) - expected_paths)
                record["identity_mismatch"] = {
                    "missing": missing[:20], "unexpected": unexpected[:20],
                    "missing_count": len(missing), "unexpected_count": len(unexpected),
                }
                raise ValueError("gcov-tool merge output GCDA inventory differs from input union")
            current_input = destination
            current_inventory = output_inventory
            current_paths = expected_paths
            break
    return current_input, current_paths


def _partition_profiles_by_build_tree(
        roots: list[Path], inventories: list[dict[str, str]], *,
        build_dir: Path, expected_notes: set[Path]) -> dict[str, list[dict[str, str]]]:
    """Classify every profile by its verified build/src or build/tests note."""
    resolved_build = build_dir.resolve(strict=True)
    if not resolved_build.is_dir():
        raise ValueError(f"gcov fallback build directory is not a directory: {resolved_build}")
    category_roots: dict[str, Path] = {}
    for category in ("src", "tests"):
        path = resolved_build / category
        if path.is_symlink() or not path.is_dir():
            raise ValueError(f"gcov fallback requires a real build/{category} directory: {path}")
        category_roots[category] = path.resolve(strict=True)

    resolved_expected = {path.resolve(strict=True) for path in expected_notes}
    partitioned: dict[str, list[dict[str, str]]] = {
        "src": [dict() for _ in roots], "tests": [dict() for _ in roots],
    }
    for root_index, inventory in enumerate(inventories):
        for relative_path, digest in inventory.items():
            relative = Path(relative_path)
            if relative.is_absolute() or ".." in relative.parts or relative.suffix != ".gcda":
                raise ValueError(f"unsafe relative GCDA inventory path: {relative_path}")
            note_path = Path(os.sep, *relative.parts).with_suffix(".gcno")
            try:
                resolved_note = note_path.resolve(strict=True)
            except OSError as exc:
                raise ValueError(f"gcov fallback GCNO path is unavailable: {note_path}") from exc
            if resolved_note not in resolved_expected:
                raise ValueError(f"gcov fallback GCNO is outside verified build notes: {note_path}")
            matches: list[str] = []
            for category, tree_root in category_roots.items():
                try:
                    resolved_note.relative_to(tree_root)
                    matches.append(category)
                except ValueError:
                    pass
            if len(matches) != 1:
                raise ValueError(
                    f"gcov fallback GCDA is not under exactly one verified build src/tests tree: "
                    f"{relative_path}")
            partitioned[matches[0]][root_index][relative_path] = digest
    return partitioned


def _recover_split_profile_merge(
        roots: list[Path], inventories: list[dict[str, str]], *,
        build_dir: Path, expected_notes: set[Path], gcov_tool: Path,
        gcc_version: str, expected_tool_sha256: str,
        output_root: Path, stage_root: Path, timeout: float, common: Any,
        commands: list[dict[str, Any]]) -> dict[str, Any]:
    """Merge isolated build/src and build/tests trees, then union them."""
    partitioned = _partition_profiles_by_build_tree(
        roots, inventories, build_dir=build_dir, expected_notes=expected_notes)
    category_outputs: dict[str, Path] = {}
    category_reports: dict[str, Any] = {}
    for category in ("src", "tests"):
        category_profiles: list[tuple[Path, dict[str, str]]] = []
        for root_index, category_inventory in enumerate(partitioned[category]):
            if not category_inventory:
                continue
            isolated_input = stage_root / "inputs" / category / f"profile-{root_index:02d}"
            _copy_profile_inventory(roots[root_index], category_inventory, isolated_input)
            category_profiles.append((isolated_input, category_inventory))
        expected_category_paths = set().union(
            *(set(inventory) for _, inventory in category_profiles)) if category_profiles else set()
        if not expected_category_paths:
            category_reports[category] = {"input_gcda_count": 0, "output_gcda_count": 0,
                                          "strategy": "empty"}
            continue
        category_output = stage_root / f"merged-{category}"
        if len(category_profiles) == 1:
            source_root, only_inventory = category_profiles[0]
            _copy_profile_inventory(source_root, only_inventory, category_output)
            category_strategy = "copy_single_nonempty_profile"
        else:
            category_output, merged_paths = _merge_profile_inventories(
                category_profiles, gcov_tool=gcov_tool, gcc_version=gcc_version,
                stage_root=stage_root / "commands", timeout=timeout,
                common=common, commands=commands, phase="recovery", category=category)
            if merged_paths != expected_category_paths:
                raise ValueError(f"gcov fallback {category} path union differs from inputs")
            category_strategy = "gcov_tool_merge_weight_1_1"
        output_inventory = _gcda_profile_inventory(category_output)
        if set(output_inventory) != expected_category_paths:
            raise ValueError(f"gcov fallback {category} output GCDA inventory differs from input union")
        category_outputs[category] = category_output
        category_reports[category] = {
            "strategy": category_strategy,
            "input_gcda_count": len(expected_category_paths),
            "output_gcda_count": len(output_inventory),
            "output_inventory_sha256": _gcda_inventory_digest(output_inventory),
            "profile_gcda_counts": [len(value) for value in partitioned[category]],
        }

    assembled_output = stage_root / "assembled-output"
    assembled_output.mkdir(mode=0o700, parents=True, exist_ok=False)
    union_paths: set[str] = set()
    for category in ("src", "tests"):
        category_root = category_outputs.get(category)
        if category_root is None:
            continue
        category_inventory = _gcda_profile_inventory(category_root)
        overlap = union_paths & set(category_inventory)
        if overlap:
            raise ValueError(f"gcov fallback category outputs overlap: {sorted(overlap)[:20]}")
        union_paths.update(category_inventory)
        _copy_profile_inventory(category_root, category_inventory, assembled_output)
    final_inventory = _gcda_profile_inventory(assembled_output)
    expected_union = set().union(*(set(value) for value in inventories))
    if union_paths != expected_union or set(final_inventory) != expected_union:
        raise ValueError("gcov fallback disjoint outputs do not match the exact input GCDA path union")
    for root, before_inventory in zip(roots, inventories):
        if _gcda_profile_inventory(root) != before_inventory:
            raise ValueError("gcov fallback modified an input profile before publication")
    if _hash_file(gcov_tool) != expected_tool_sha256:
        raise ValueError("gcov-tool executable changed before split recovery publication")
    if output_root.exists() or output_root.is_symlink():
        raise ValueError(f"gcov fallback final output path is no longer fresh: {output_root}")
    recovery_report = {
        "status": "pass", "strategy": "verified_build_src_tests_partition",
        "build_dir": str(build_dir.resolve(strict=True)),
        "categories": category_reports,
        "output_gcda_count": len(final_inventory),
        "output_inventory_sha256": _gcda_inventory_digest(final_inventory),
    }
    # The stage directory is created beside output_root, so this rename publishes
    # the verified disjoint union atomically and fails without exposing partial data.
    os.replace(assembled_output, output_root)
    return recovery_report


def merge_gcov_profile_roots(profile_roots: list[Path], *, gcov_tool: Path,
                              gcc_version: str, output_root: Path,
                              report_path: Path, timeout: float,
                              common: Any,
                              expected_notes: set[Path],
                              build_dir: Path | None = None) -> dict[str, Any]:
    """Merge independent GCOV_PREFIX roots without changing any input profile."""
    roots: list[Path] = []
    for value in profile_roots:
        if value.is_symlink():
            raise GcovProfileMergeError(
                f"gcov profile input must not be a symlink: {value}",
                {"status": "merge_failed", "error": "symlink profile input"})
        try:
            resolved = value.resolve(strict=True)
        except OSError as exc:
            raise GcovProfileMergeError(
                f"gcov profile input is missing: {value}",
                {"status": "merge_failed", "error": str(exc)}) from exc
        if not resolved.is_dir() or resolved in roots:
            raise GcovProfileMergeError(
                f"gcov profile inputs must be distinct directories: {resolved}",
                {"status": "merge_failed", "error": "invalid profile inputs"})
        roots.append(resolved)
    if len(roots) < 2:
        raise GcovProfileMergeError(
            "gcov profile merge needs at least two distinct roots",
            {"status": "merge_failed", "error": "too few profile inputs"})
    output_candidate = output_root.absolute()
    if output_candidate.exists() or output_candidate.is_symlink():
        raise GcovProfileMergeError(
            f"gcov merged output must be a fresh path: {output_candidate}",
            {"status": "merge_failed", "error": "output path already exists"})
    output_root = output_candidate.resolve()
    if report_path.exists() or report_path.is_symlink():
        raise GcovProfileMergeError(
            f"gcov profile merge report must be a fresh path: {report_path}",
            {"status": "merge_failed", "error": "report path already exists"})
    if any(output_root == root or root in output_root.parents or output_root in root.parents
           for root in roots):
        raise GcovProfileMergeError(
            "gcov merged output must not be inside an input profile root",
            {"status": "merge_failed", "error": "output nested in input"})
    for left_index, left in enumerate(roots):
        if any(left in right.parents or right in left.parents for right in roots[left_index + 1:]):
            raise GcovProfileMergeError(
                "gcov profile input roots must not overlap",
                {"status": "merge_failed", "error": "overlapping profile inputs"})

    report: dict[str, Any] = {
        "schema": "slicer-gcov-profile-merge-v1", "status": "running",
        "gcov_tool": str(gcov_tool), "gcc_version": gcc_version,
        "input_profiles": [], "commands": [], "output_root": str(output_root),
    }
    input_inventories: list[dict[str, str]] = []
    try:
        report_path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        for root in roots:
            inventory = _gcda_profile_inventory(root)
            input_inventories.append(inventory)
            report["input_profiles"].append({
                "path": str(root), "gcda_count": len(inventory),
                "inventory_sha256": _gcda_inventory_digest(inventory),
                "gcda_inventory": inventory,
            })
        epoch_check = inspect_gcov_profile_epochs(roots, expected_notes=expected_notes)
        report["profile_epoch_check"] = epoch_check
        if epoch_check["status"] != "pass":
            raise ValueError(format_gcov_profile_epoch_failure(epoch_check))
        tool_sha_before = _hash_file(gcov_tool)
        version_result = common._run([str(gcov_tool), "--version"], timeout=timeout)
        version_matches = re.findall(r"\b\d+\.\d+(?:\.\d+)?\b", version_result.stdout or "")
        if version_result.returncode != 0 or not any(
                value == gcc_version or gcc_version.startswith(value + ".")
                for value in version_matches):
            report.update({
                "status": "merge_failed", "tool_sha256": tool_sha_before,
                "tool_version_argv": [str(gcov_tool), "--version"],
                "tool_version_stdout": version_result.stdout,
                "tool_version_stderr": version_result.stderr,
                "tool_version_returncode": version_result.returncode,
                "parsed_versions": version_matches,
            })
            _write_profile_merge_report(report_path, report)
            raise GcovProfileMergeError(
                f"gcov-tool version does not match GCC {gcc_version!r}", report)

        report["tool_sha256"] = tool_sha_before
        report["tool_version"] = version_result.stdout.strip()
        report["tool_version_argv"] = [str(gcov_tool), "--version"]

        output_root.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        active_roots = [(root, inventory) for root, inventory in zip(roots, input_inventories)
                        if inventory]
        if not active_roots:
            output_root.mkdir(mode=0o700)
            report["merge_strategy"] = "empty_profile_union"
        elif len(active_roots) == 1:
            output_root.mkdir(mode=0o700)
            only_root, only_inventory = active_roots[0]
            for relative_path in sorted(only_inventory):
                source = only_root / relative_path
                destination = output_root / relative_path
                destination.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                shutil.copy2(source, destination)
            report["merge_strategy"] = "copy_single_nonempty_profile"
        else:
            report["merge_strategy"] = "gcov_tool_merge_weight_1_1"
            output_root.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(
                    prefix=f".{output_root.name}.gcov-merge-", dir=output_root.parent) as stage:
                stage_root = Path(stage)
                try:
                    primary_output, primary_paths = _merge_profile_inventories(
                        active_roots, gcov_tool=gcov_tool, gcc_version=gcc_version,
                        stage_root=stage_root / "primary", timeout=timeout,
                        common=common, commands=report["commands"], phase="primary")
                    expected_active_paths = set().union(
                        *(set(inventory) for _, inventory in active_roots))
                    if primary_paths != expected_active_paths:
                        raise ValueError("primary gcov merge path union differs from inputs")
                    os.replace(primary_output, output_root)
                except Exception as primary_error:
                    primary_commands = [row for row in report["commands"]
                                        if row.get("phase") == "primary"]
                    report["primary_merge_failure"] = {
                        "error": {"type": type(primary_error).__name__,
                                  "message": str(primary_error)},
                        "command": primary_commands[-1] if primary_commands else None,
                    }
                    report["merge_strategy"] = "verified_build_src_tests_split_recovery"
                    try:
                        if build_dir is None:
                            raise ValueError("verified build directory is required for split recovery")
                        if _hash_file(gcov_tool) != tool_sha_before:
                            raise ValueError("gcov-tool executable changed after primary merge failure")
                        recovery = _recover_split_profile_merge(
                            roots, input_inventories, build_dir=build_dir,
                            expected_notes=expected_notes, gcov_tool=gcov_tool,
                            gcc_version=gcc_version,
                            expected_tool_sha256=tool_sha_before,
                            output_root=output_root, stage_root=stage_root / "recovery",
                            timeout=timeout, common=common, commands=report["commands"])
                    except Exception as recovery_error:
                        report["recovery_failure"] = {
                            "type": type(recovery_error).__name__,
                            "message": str(recovery_error),
                        }
                        report["recovery_commands"] = [
                            row["argv"] for row in report["commands"]
                            if row.get("phase") == "recovery"]
                        raise ValueError(
                            f"primary gcov-tool merge failed ({primary_error}); "
                            f"split recovery failed ({recovery_error})") from recovery_error
                    report["recovery"] = recovery
                    report["recovery_commands"] = [
                        row["argv"] for row in report["commands"]
                        if row.get("phase") == "recovery"]

        input_unchanged = True
        for index, (root, before_inventory) in enumerate(zip(roots, input_inventories)):
            after_inventory = _gcda_profile_inventory(root)
            unchanged = after_inventory == before_inventory
            report["input_profiles"][index]["unchanged_after_merge"] = unchanged
            input_unchanged = input_unchanged and unchanged
        if not input_unchanged:
            raise ValueError("gcov-tool merge modified an input profile root")
        if _hash_file(gcov_tool) != tool_sha_before:
            raise ValueError("gcov-tool executable changed during profile merging")
        final_inventory = _gcda_profile_inventory(output_root)
        expected_paths = set().union(*(set(inventory) for inventory in input_inventories))
        if set(final_inventory) != expected_paths:
            raise ValueError("final gcov profile tree does not match the input GCDA path union")
        report.update({
            "status": "pass", "input_files_unchanged": True,
            "output_gcda_count": len(final_inventory),
            "output_gcda_inventory_sha256": _gcda_inventory_digest(final_inventory),
        })
        _write_profile_merge_report(report_path, report)
        return report
    except GcovProfileMergeError:
        raise
    except Exception as exc:
        try:
            before_inventories = locals().get("input_inventories")
            if before_inventories is not None:
                for index, (root, before_inventory) in enumerate(zip(roots, before_inventories)):
                    after_inventory = _gcda_profile_inventory(root)
                    report["input_profiles"][index]["unchanged_after_merge"] = \
                        after_inventory == before_inventory
            report["tool_sha256_after"] = _hash_file(gcov_tool)
        except Exception as inventory_error:
            report["input_recheck_error"] = {
                "type": type(inventory_error).__name__, "message": str(inventory_error)}
        report.update({"status": "merge_failed",
                       "error": {"type": type(exc).__name__, "message": str(exc)}})
        if not report_path.exists():
            _write_profile_merge_report(report_path, report)
        raise GcovProfileMergeError(f"gcov profile merge failed: {exc}", report) from exc


def _read_gcov_json(path: Path, expected_gcc_version: str) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    with gzip.open(path, "rt", encoding="utf-8") as stream:
        payload = json.load(stream)
    files = payload.get("files")
    if not isinstance(files, list):
        raise ValueError(f"gcov JSON has no files array: {path}")
    json_gcc_version = payload.get("gcc_version")
    if not isinstance(json_gcc_version, str) or not (
            json_gcc_version == expected_gcc_version or
            json_gcc_version.startswith(expected_gcc_version + ".")):
        raise ValueError(f"gcov JSON GCC version {json_gcc_version!r} does not match "
                         f"CMake compiler {expected_gcc_version!r}: {path}")
    cwd = payload.get("current_working_directory")
    for item in files:
        if isinstance(item, dict) and cwd:
            item.setdefault("current_working_directory", cwd)
    return files, {"gcc_version": json_gcc_version, "format_version": payload.get("format_version")}


def _junit_summary(path: Path) -> tuple[Counter[str], list[dict[str, str]]]:
    root = ET.parse(path).getroot()
    names: Counter[str] = Counter()
    problems: list[dict[str, str]] = []
    for case in root.iter("testcase"):
        name = case.get("name")
        if not name:
            raise ValueError("JUnit testcase has no name")
        names[name] += 1
        for kind in ("failure", "error", "skipped"):
            child = case.find(kind)
            if child is not None:
                problems.append({"test": name, "kind": kind,
                                 "message": child.get("message", "")})
    return names, problems


def _junit_credit_assessment(
        path: Path, selected_names: list[str]
) -> tuple[Counter[str], list[dict[str, str]], dict[str, Any]]:
    names, problems = _junit_summary(path)
    mismatch = compare_test_identities(selected_names, names)
    assessment = {
        "test_count": sum(names.values()), "name_mismatch": mismatch,
        "problems": problems,
        "eligible_for_coverage": names == Counter(selected_names) and not problems,
    }
    return names, problems, assessment


def _sha_artifact(common: Any, path: Path) -> dict[str, Any]:
    return common._artifact_provenance(path)


def _copy_catch_junit(source: Path, destination: Path, receipt_path: Path) -> dict[str, Any]:
    """Copy adapter JUnit to the shared gcov gate path without replacement."""
    if destination.exists() or destination.is_symlink() or receipt_path.exists() or receipt_path.is_symlink():
        raise ValueError("refusing to replace catch batch JUnit or its copy receipt")
    source_hash = _hash_file(source)
    with source.open("rb") as reader, destination.open("xb") as writer:
        shutil.copyfileobj(reader, writer)
        writer.flush()
        os.fsync(writer.fileno())
    destination_hash = _hash_file(destination)
    if destination_hash != source_hash:
        raise ValueError("copied Catch batch JUnit hash does not match source")
    receipt = {
        "schema": "slicer-catch-batch-junit-copy-v1",
        "source": str(source.resolve()), "source_sha256": source_hash,
        "destination": str(destination.resolve()), "destination_sha256": destination_hash,
        "byte_identical": True,
    }
    with receipt_path.open("x", encoding="utf-8") as stream:
        json.dump(receipt, stream, indent=2, sort_keys=True)
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    return receipt


def _run_gcov_coverage(args: Any, *, common: Any) -> int:
    """Validate, run exact CTest selection, and produce a backend-specific report."""
    try:
        args.threshold = validate_threshold(float(args.threshold))
    except (TypeError, ValueError) as exc:
        _fail(common, str(exc))
    try:
        gcov_jobs = validate_gcov_jobs(getattr(args, "gcov_jobs", 1))
    except (TypeError, ValueError) as exc:
        _fail(common, str(exc))
    fff_shards = getattr(args, "gcov_catch_batch_fff_shards", 1)
    if isinstance(fff_shards, bool) or not isinstance(fff_shards, int) or fff_shards not in {1, 2}:
        _fail(common, "--gcov-catch-batch-fff-shards must be 1 or 2")
    if fff_shards == 2 and not getattr(args, "gcov_catch_batch", False):
        _fail(common, "two-way FFF sharding requires --gcov-catch-batch")
    if args.metrics is None:
        args.metrics = ["lines"]
    if any(metric != "lines" for metric in args.metrics):
        _fail(common, "the gcov backend currently gates only physical lines; use --metric lines")
    root = Path(args.root).resolve()
    build_dir = (root / args.build_dir).resolve()
    source_root = (root / args.source_root).resolve() if not Path(args.source_root).is_absolute() \
        else Path(args.source_root).resolve()
    out_arg = Path(args.out)
    out_candidate = root / out_arg if not out_arg.is_absolute() else out_arg
    if out_candidate.is_symlink():
        _fail(common, f"gcov output path must not be a symlink: {out_candidate}")
    out = out_candidate.resolve()
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        _fail(common, f"gcov output must be a fresh empty directory: {out}")
    out.mkdir(parents=True, exist_ok=True)

    manifests = [(root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
                 for value in (args.manifest or [])]
    manifest_targets: dict[str, Path] = {}
    source_by_target: dict[str, list[Path]] = {}
    for path in manifests:
        try:
            target = _target_from_manifest(path)
        except ValueError as exc:
            _fail(common, str(exc))
        if target in manifest_targets:
            _fail(common, f"duplicate coverage manifest for target {target}")
        manifest_targets[target] = path
        try:
            sources = common._read_manifest(path, root)
        except common.CoverageError:
            raise
        missing = [str(source) for source in sources if not source.is_file()]
        if missing:
            _fail(common, "manifest source does not exist: " + ", ".join(missing[:8]))
        source_by_target[target] = sources
    if set(manifest_targets) != REQUIRED_MANIFEST_TARGETS:
        _fail(common, "gcov needs exactly seven manifests for: " + ", ".join(sorted(REQUIRED_MANIFEST_TARGETS)))
    if len(args.object) != len(FULL_TEST_TARGETS):
        _fail(common, "gcov needs exactly the five canonical full-suite test executables")
    objects = [(root / Path(value) if not Path(value).is_absolute() else Path(value)).resolve()
               for value in args.object]
    if any(not obj.is_file() or not common._under(obj, build_dir) for obj in objects):
        _fail(common, "one or more gcov test executables are missing")
    if {obj.name for obj in objects} != set(FULL_TEST_TARGETS):
        _fail(common, "gcov test executable names must match the five canonical full-suite targets")

    ccdb_arg = Path(args.compile_commands) if args.compile_commands \
        else Path(args.build_dir) / "compile_commands.json"
    compile_commands = (root / ccdb_arg if not ccdb_arg.is_absolute() else ccdb_arg).resolve()
    try:
        commands = json.loads(compile_commands.read_text(encoding="utf-8"))
        cache_path = build_dir / "CMakeCache.txt"
        cache_lines = cache_path.read_text(encoding="utf-8").splitlines()
    except (OSError, json.JSONDecodeError) as exc:
        _fail(common, f"cannot read GCC compile evidence: {exc}")
    try:
        compiler_id, gcc_version, compiler_record = _cmake_compiler_provenance(build_dir)
    except (OSError, ValueError) as exc:
        _fail(common, str(exc))
    if compiler_id != "GNU":
        _fail(common, f"generated CMake compiler record is not GNU: {compiler_id}")
    cache = {line.split("=", 1)[0].split(":", 1)[0]: line.split("=", 1)[1]
             for line in cache_lines if "=" in line and not line.startswith("#")}
    if cache.get("SLICER_ENABLE_GCC_GCOV_COVERAGE") not in ("ON", "1"):
        _fail(common, "CMake cache does not enable SLICER_ENABLE_GCC_GCOV_COVERAGE")
    if cache.get("SLICER_ENABLE_LLVM_SOURCE_COVERAGE") in ("ON", "1"):
        _fail(common, "CMake cache also enables LLVM coverage; select exactly one backend")
    if not isinstance(commands, list):
        _fail(common, "compile_commands.json is not an array")

    by_target_source: dict[tuple[str, str], list[dict[str, Any]]] = {}
    compiled_sources_by_target: dict[str, list[Path]] = {}
    for entry in commands:
        if not isinstance(entry, dict):
            continue
        argv = _argv(entry)
        directory = Path(entry.get("directory", root))
        if not directory.is_absolute():
            directory = root / directory
        output = _output_arg(argv, directory)
        if output is None:
            continue
        target_match = re.search(r"(?:^|/)CMakeFiles/([^/]+)\.dir/", output.as_posix())
        if not target_match:
            continue
        target = target_match.group(1)
        if target not in REQUIRED_MANIFEST_TARGETS:
            continue
        source_arg = Path(entry.get("file", ""))
        directory = Path(entry.get("directory", root))
        if not directory.is_absolute():
            directory = root / directory
        lexical_source = directory / source_arg if not source_arg.is_absolute() else source_arg
        lexical_source = Path(os.path.abspath(lexical_source))
        source = lexical_source.resolve()
        compiled_sources_by_target.setdefault(target, []).append(source)
        by_target_source.setdefault((target, os.path.normcase(os.path.realpath(source))), []).append(
            {"source": source, "lexical_source": lexical_source, "object": output, "argv": argv}
        )

    compile_rows: list[dict[str, Any]] = []
    expected_notes: set[Path] = set()
    notes_by_target: dict[str, set[Path]] = {}
    for target, sources in source_by_target.items():
        try:
            validate_manifest_compile_sources(
                sources, compiled_sources_by_target.get(target, []), build_dir)
        except ValueError as exc:
            _fail(common, f"invalid compile manifest for {target}: {exc}")
        for source in sources:
            rows = by_target_source.get((target, os.path.normcase(os.path.realpath(source))), [])
            if len(rows) != 1:
                _fail(common, f"expected one compile command for {target}:{source}; found {len(rows)}")
            row = rows[0]
            argv = row["argv"]
            try:
                matches = validate_gcc_compile_argv(argv, source=source,
                                                    source_root=source_root,
                                                    filter_source=row["lexical_source"])
            except ValueError as exc:
                _fail(common, f"invalid compile coverage flags for {target}:{source}: {exc}")
            note = row["object"].with_suffix(".gcno")
            if not note.is_file():
                _fail(common, f"missing gcov notes file for {target}:{source}: {note}")
            expected_notes.add(note.resolve())
            notes_by_target.setdefault(target, set()).add(note.resolve())
            compile_rows.append({"target": target, "source": str(source),
                                 "object": str(row["object"]), "gcno": str(note),
                                 "compiler": argv[0],
                                 "coverage_flags": [value for value in argv
                                     if value == "--coverage" or value.startswith("-fprofile-")],
                                 "filter_matches_project": matches})

    all_notes = collect_build_notes(commands, root)
    try:
        extra_notes = validate_note_inventory(expected_notes, all_notes)
    except ValueError as exc:
        _fail(common, str(exc))

    gcov = common._which("gcov", args.gcov)
    version_result = common._run([gcov, "--version"], timeout=args.timeout)
    if version_result.returncode != 0:
        _fail(common, "gcov --version failed: " + version_result.stderr)
    versions = re.findall(r"\b\d+\.\d+(?:\.\d+)?\b", version_result.stdout)
    if not versions or not gcc_version or not any(v == gcc_version or gcc_version.startswith(v + ".")
                                                    for v in versions):
        _fail(common, f"gcov version does not match CMake GCC version {gcc_version!r}")

    gcov_tool: Path | None = None
    gcov_tool_version: str | None = None
    if fff_shards == 2:
        try:
            gcov_tool = resolve_gcov_tool(gcov, getattr(args, "gcov_tool", None))
        except ValueError as exc:
            _fail(common, str(exc))
        tool_version_result = common._run([str(gcov_tool), "--version"], timeout=args.timeout)
        tool_versions = re.findall(r"\b\d+\.\d+(?:\.\d+)?\b", tool_version_result.stdout or "")
        if tool_version_result.returncode != 0 or not any(
                value == gcc_version or gcc_version.startswith(value + ".")
                for value in tool_versions):
            _fail(common, f"gcov-tool version does not match CMake GCC version {gcc_version!r}")
        gcov_tool_version = tool_version_result.stdout.strip()

    command = common._expand_command(common.split_command(args.test_command), root=root,
                                     build=build_dir, filter_name=args.filter)
    if Path(command[0]).name != "ctest":
        _fail(common, "gcov requires a CTest command to bind selected identities to JUnit")
    if "{filter}" in args.test_command and not args.filter:
        _fail(common, "test command contains {filter}, but --filter was not supplied")
    if args.filter and "{filter}" not in args.test_command and "-L" not in command:
        command.extend(["-L", args.filter])
    if "--no-tests=error" not in command:
        command.append("--no-tests=error")
    if any(arg == "--show-only" or arg.startswith("--output-junit") for arg in command):
        _fail(common, "test command must not preselect CTest listing or JUnit output options")
    requested_ctest_argv = list(command)
    catch_batch_enabled = bool(getattr(args, "gcov_catch_batch", False))
    catch_discovery_receipt: dict[str, Any] | None = None
    catch_discovery_raw: bytes | None = None
    parallel_profile_dirs: list[Path] = []
    if catch_batch_enabled:
        from slicer_catch_batch import ctest_selection_argv, discover_ctest
        try:
            command = ctest_selection_argv(command)
            command, discovery, catch_discovery_receipt = discover_ctest(
                command, build_dir=build_dir, cwd=root,
                out_dir=out / "catch-batch-discovery",
                discovery_timeout_seconds=args.timeout)
            discovery_source = Path(catch_discovery_receipt["ctest_json"])
            catch_discovery_raw = discovery_source.read_bytes()
            selected_names = [test["name"] for test in discovery["tests"]]
        except (OSError, ValueError, KeyError, TypeError) as exc:
            _fail(common, f"Catch batch CTest discovery failed: {exc}")
        preview_stdout = catch_discovery_raw.decode("utf-8")
        preview_argv = list(catch_discovery_receipt["argv"])
    else:
        preview = common._run([*command, "--show-only=json-v1"], cwd=root, timeout=args.timeout)
        if preview.returncode != 0:
            _fail(common, f"CTest discovery failed: {preview.stderr}")
        try:
            discovery = json.loads(preview.stdout)
            selected_names = [test["name"] for test in discovery["tests"]]
        except (KeyError, TypeError, json.JSONDecodeError) as exc:
            _fail(common, f"CTest discovery JSON is invalid: {exc}")
        preview_stdout = preview.stdout
        preview_argv = [*command, "--show-only=json-v1"]
    if not selected_names or len(set(selected_names)) != len(selected_names):
        _fail(common, "CTest selection is empty or contains duplicate test identities")
    discovery_path = out / "ctest-discovery.json"
    discovery_path.write_text(preview_stdout, encoding="utf-8")

    source_tree_before = _source_tree_snapshot(source_root)
    source_hashes_before = {str(source): _hash_file(source)
                            for sources in source_by_target.values() for source in sources}
    artifact_paths = {compile_commands, cache_path, compiler_record, *manifests, *objects, *expected_notes}
    try:
        artifact_hashes_before = _artifact_snapshot(common, artifact_paths)
    except ValueError as exc:
        _fail(common, str(exc))
    try:
        gcov_prefix = create_gcov_data_dir(root, out, getattr(args, "gcov_data_dir", None))
    except ValueError as exc:
        _fail(common, str(exc))
    junit_path = out / "ctest-junit.xml"
    env = dict(os.environ)
    env["GCOV_PREFIX"] = str(gcov_prefix)
    env["GCOV_PREFIX_STRIP"] = "0"
    started = __import__("time").monotonic()
    catch_batch_report: dict[str, Any] | None = None
    catch_junit_copy_receipt: dict[str, Any] | None = None
    if catch_batch_enabled:
        from slicer_catch_batch import run_discovered_batches
        if catch_discovery_receipt is None or catch_discovery_raw is None:
            _fail(common, "Catch batch mode has no validated live CTest discovery artifact")
        batch_timeout = getattr(args, "gcov_catch_batch_timeout", None)
        if batch_timeout is None:
            batch_timeout = args.timeout
        batch_out = out / "catch-batch"
        fff_profile_root = None
        if fff_shards == 2:
            fff_profile_root = gcov_prefix.parent / f"{gcov_prefix.name}-fff-shards"
        catch_batch_report = run_discovered_batches(
            discovery, ctest_argv=command, build_dir=build_dir, out_dir=batch_out,
            batch_timeout_seconds=batch_timeout,
            discovery_argv=catch_discovery_receipt["argv"],
            discovery_sha256=catch_discovery_receipt["ctest_json_sha256"],
            discovery_raw_json=catch_discovery_raw,
            fff_shards=fff_shards, fff_profile_root=fff_profile_root,
            expected_notes=expected_notes,
            required_gcda_notes_by_executable={
                target: notes_by_target[target] for target in FULL_TEST_TARGETS
            },
            parent_env=env)
        for batch_info in catch_batch_report.get("batches", []):
            parallel_profile_dirs.extend(Path(path) for path in batch_info.get("parallel_profile_dirs", []))
        source_junit = catch_batch_report.get("junit")
        if source_junit:
            catch_junit_copy_receipt = _copy_catch_junit(
                Path(source_junit), junit_path, out / "catch-batch-junit-copy.json")
        catch_status = catch_batch_report.get("status")
        catch_returncode = 0 if catch_status == "pass" else (
            124 if catch_status == "incomplete_timeout" else 42)
        test_summary = {
            "executor": "catch_batch", "status": catch_status,
            "selected_count": catch_batch_report.get("selected_count"),
            "batch_count": catch_batch_report.get("batch_count"),
            "junit": str(junit_path) if junit_path.is_file() else None,
        }
        test = subprocess.CompletedProcess(
            ["catch_batch", str(batch_out / "batch-report.json")], catch_returncode,
            json.dumps(test_summary, sort_keys=True), "")
        test_stdout_path = out / "catch-batch.stdout.log"
        test_stderr_path = out / "catch-batch.stderr.log"
    else:
        test = common._run([*command, "--output-junit", str(junit_path)], cwd=root,
                           env=env, timeout=args.timeout)
        test_stdout_path = out / "ctest.stdout.log"
        test_stderr_path = out / "ctest.stderr.log"
    elapsed = __import__("time").monotonic() - started
    test_stdout_path.write_text(test.stdout or "", encoding="utf-8")
    test_stderr_path.write_text(test.stderr or "", encoding="utf-8")
    source_tree_after = _source_tree_snapshot(source_root)
    artifact_hashes_after = _artifact_snapshot(common, artifact_paths, require_present=False)
    source_tree_before_meta = {"file_count": len(source_tree_before),
                               "sha256": _snapshot_digest(source_tree_before)}
    source_tree_after_meta = {"file_count": len(source_tree_after),
                              "sha256": _snapshot_digest(source_tree_after)}
    artifact_hash_failures = sorted(
        path for path in artifact_hashes_before
        if artifact_hashes_before[path] != artifact_hashes_after.get(path))
    if catch_batch_enabled:
        if catch_batch_report is None or catch_discovery_receipt is None:
            _fail(common, "Catch batch mode produced no run report")
        batch_report_path = out / "catch-batch" / "batch-report.json"
        batch_argvs = []
        for info in catch_batch_report.get("batches", []):
            if info.get("batch_process_launch_attempted") and "batch_argv" in info:
                batch_argvs.append(info["batch_argv"])
            batch_argvs.extend(
                shard["batch_argv"] for shard in info.get("shards", [])
                if shard.get("batch_process_launch_attempted") and "batch_argv" in shard)
        run_record = {
            "executor": "catch_batch",
            "requested_ctest_argv": requested_ctest_argv,
            "selection_ctest_argv": command,
            "preview_argv": preview_argv,
            "discovery_argv": catch_discovery_receipt["argv"],
            "discovery_command_receipt": _sha_artifact(
                common, Path(catch_discovery_receipt["receipt"])),
            "discovery_gcov_prefix": catch_discovery_receipt["discovery_gcov_prefix"],
            "discovery_llvm_profile_file": catch_discovery_receipt["discovery_llvm_profile_file"],
            "selected_test_count": len(selected_names), "selected_test_names": selected_names,
            "discovery": _sha_artifact(common, discovery_path),
            "returncode": test.returncode, "timed_out": test.returncode == 124,
            "duration_seconds": elapsed,
            "stdout_path": str(test_stdout_path), "stdout_sha256": _hash_file(test_stdout_path),
            "stderr_path": str(test_stderr_path), "stderr_sha256": _hash_file(test_stderr_path),
            "junit": _sha_artifact(common, junit_path),
            "junit_copy_receipt": catch_junit_copy_receipt,
            "batch_runner_report": _sha_artifact(common, batch_report_path),
            "batch_status": catch_batch_report.get("status"),
            "batch_count": catch_batch_report.get("batch_count",
                                                   len(catch_batch_report.get("batches", []))),
            "batch_argvs_started_count": catch_batch_report.get(
                "batch_process_launch_attempted_count", len(batch_argvs)),
            "batch_argvs": batch_argvs,
            "batch_timeout_seconds": catch_batch_report.get("batch_timeout_seconds"),
            "batch_timeout_scope": catch_batch_report.get("timeout_semantics"),
            "gcov_prefix": str(gcov_prefix), "gcov_prefix_strip": 0,
            "gcov_jobs": gcov_jobs,
            "fff_parallel_shards": fff_shards,
            "gcov_parallel_profile_dirs": [str(path) for path in parallel_profile_dirs],
            "gcov_tool": (None if gcov_tool is None else {
                "path": str(gcov_tool), "sha256": _hash_file(gcov_tool),
                "version": gcov_tool_version,
            }),
        }
    else:
        run_record = {
            "argv": [*command, "--output-junit", str(junit_path)],
            "preview_argv": preview_argv,
            "selected_test_count": len(selected_names), "selected_test_names": selected_names,
            "discovery": _sha_artifact(common, discovery_path),
            "returncode": test.returncode, "timed_out": test.returncode == 124,
            "duration_seconds": elapsed,
            "stdout_sha256": _hash_file(test_stdout_path),
            "stderr_sha256": _hash_file(test_stderr_path),
            "junit": _sha_artifact(common, junit_path),
            "gcov_prefix": str(gcov_prefix), "gcov_prefix_strip": 0,
            "gcov_jobs": gcov_jobs,
        }
    if test.returncode != 0 or not junit_path.is_file():
        raw_runtime_gcda = sorted({
            path for profile_root in [gcov_prefix, *parallel_profile_dirs]
            for path in profile_root.rglob("*.gcda") if path.is_file()
        })
        preflight_failed = (catch_batch_report is not None and
                            catch_batch_report.get("status") == "incomplete_preflight")
        report = {"status": "preflight_failed" if preflight_failed else
                  ("test_failed" if test.returncode else "missing_junit"),
                  "backend": "gcc-gcov", "denominator": "gcc physical lines only",
                  "run": run_record, "notes": sorted(map(str, expected_notes)),
                  "extra_build_notes_unselected": extra_notes,
                  "input_artifact_sha256_before": artifact_hashes_before,
                  "input_artifact_sha256_after": artifact_hashes_after,
                  "input_artifact_hash_failures": artifact_hash_failures,
                  "source_tree_before": source_tree_before_meta,
                  "source_tree_after": source_tree_after_meta,
                  "raw_gcda": [_sha_artifact(common, path) for path in raw_runtime_gcda]}
        if preflight_failed:
            report["preflight_failure"] = {
                "reason": catch_batch_report.get("incomplete_reason"),
                "failed_batches": [
                    {key: info[key] for key in (
                        "executable", "process_outcome", "preflight_error",
                        "preflight_profile_epoch", "preflight_profile_epoch_receipt",
                        "preflight_profile_epoch_receipt_sha256",
                        "preflight_profile_files") if key in info}
                    for info in catch_batch_report.get("batches", [])
                    if info.get("process_outcome") == "preflight_epoch_mismatch"
                ],
            }
        (out / "coverage-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 2
    try:
        junit_names, junit_problems, junit_assessment = _junit_credit_assessment(
            junit_path, selected_names)
    except (ET.ParseError, OSError, ValueError) as exc:
        _fail(common, f"cannot validate CTest JUnit output: {exc}")
    expected_names = Counter(selected_names)
    run_record["junit_test_count"] = sum(junit_names.values())
    run_record["junit_name_mismatch"] = junit_assessment["name_mismatch"]
    run_record["junit_problems"] = junit_problems
    run_record["junit_eligible_for_coverage"] = junit_assessment["eligible_for_coverage"]
    if not junit_assessment["eligible_for_coverage"]:
        (out / "coverage-report.json").write_text(json.dumps({
            "status": "test_identity_or_junit_failure", "backend": "gcc-gcov",
            "denominator": "gcc physical lines only", "run": run_record,
        }, indent=2) + "\n", encoding="utf-8")
        return 2

    gcov_profile_root = gcov_prefix
    if parallel_profile_dirs:
        if gcov_tool is None:
            _fail(common, "parallel FFF profiles have no version-checked gcov-tool")
        merge_report_path = out / "gcov-profile-merge.json"
        merge_output_root = out / "gcov-profile-merge" / "merged"
        try:
            merge_report = merge_gcov_profile_roots(
                [gcov_prefix, *parallel_profile_dirs], gcov_tool=gcov_tool,
                gcc_version=gcc_version, output_root=merge_output_root,
                report_path=merge_report_path, timeout=args.timeout, common=common,
                expected_notes=expected_notes, build_dir=build_dir)
        except GcovProfileMergeError as exc:
            run_record["gcov_profile_merge"] = _sha_artifact(common, merge_report_path)
            failure = {
                "status": "gcov_profile_merge_failed", "backend": "gcc-gcov",
                "denominator": "gcc physical lines only", "run": run_record,
                "profile_merge": exc.report,
                "input_artifact_sha256_before": artifact_hashes_before,
                "input_artifact_sha256_after": artifact_hashes_after,
                "input_artifact_hash_failures": artifact_hash_failures,
                "source_tree_before": source_tree_before_meta,
                "source_tree_after": source_tree_after_meta,
            }
            (out / "coverage-report.json").write_text(
                json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            return 2
        gcov_profile_root = merge_output_root
        run_record["gcov_profile_merge"] = _sha_artifact(common, merge_report_path)
        run_record["gcov_export_prefix"] = str(gcov_profile_root)
    else:
        epoch_report = inspect_gcov_profile_epochs(
            [gcov_prefix], expected_notes=expected_notes)
        epoch_report_path = out / "gcov-profile-epoch-check.json"
        _write_profile_merge_report(epoch_report_path, epoch_report)
        run_record["gcov_profile_epoch_check"] = _sha_artifact(common, epoch_report_path)
        if epoch_report["status"] != "pass":
            failure = {
                "status": "gcov_profile_epoch_mismatch", "backend": "gcc-gcov",
                "denominator": "gcc physical lines only", "run": run_record,
                "profile_epoch_check": epoch_report,
                "input_artifact_sha256_before": artifact_hashes_before,
                "input_artifact_sha256_after": artifact_hashes_after,
                "input_artifact_hash_failures": artifact_hash_failures,
                "source_tree_before": source_tree_before_meta,
                "source_tree_after": source_tree_after_meta,
            }
            (out / "coverage-report.json").write_text(
                json.dumps(failure, indent=2, sort_keys=True) + "\n", encoding="utf-8")
            return 2

    raw_data_dir = out / "gcov-input"
    raw_data_dir.mkdir()
    gcda_paths = {path.resolve() for path in gcov_profile_root.rglob("*.gcda") if path.is_file()}
    expected_gcda: set[Path] = set()
    expected_gcda_notes: dict[Path, Path] = {}
    records: list[dict[str, Any]] = []
    translation_units: list[dict[str, Any]] = []
    gcov_invocations: list[dict[str, Any]] = []
    gcov_json_metadata: list[dict[str, Any]] = []
    selected_note_to_row = {Path(row["gcno"]).resolve(): row for row in compile_rows}
    gcov_jobs_to_run: list[dict[str, Any]] = []
    gcov_commands: list[tuple[list[str], Path, Path, Path, str]] = []
    for index, note in enumerate(sorted(expected_notes)):
        row = selected_note_to_row[note]
        object_path = Path(row["object"])
        # GCC preserves absolute object paths beneath GCOV_PREFIX when STRIP=0.
        gcda = gcda_path_for_object(gcov_profile_root, object_path)
        if gcda.is_file():
            expected_gcda.add(gcda.resolve())
            expected_gcda_notes[gcda.resolve()] = note
        staging = raw_data_dir / f"tu-{index:04d}"
        staging.mkdir()
        staged_note = staging / note.name
        shutil.copy2(note, staged_note)
        staged_gcda = staged_note.with_suffix(".gcda")
        if gcda.is_file():
            shutil.copy2(gcda, staged_gcda)
        output_dir = out / "gcov-json" / f"tu-{index:04d}"
        output_dir.mkdir(parents=True)
        gcov_jobs_to_run.append({"index": index, "note": note, "row": row,
                                 "object": object_path, "gcda": gcda,
                                 "staged_note": staged_note, "staged_gcda": staged_gcda,
                                 "output_dir": output_dir})
        gcov_commands.append(([gcov, "--json-format", str(staged_note)], output_dir,
                              output_dir / "gcov.stdout.log", output_dir / "gcov.stderr.log",
                              row["source"]))

    gcov_results = run_gcov_commands(gcov_commands, jobs=gcov_jobs,
                                     timeout=args.timeout, common=common)
    for job, result in zip(gcov_jobs_to_run, gcov_results):
        index = job["index"]
        note = job["note"]
        row = job["row"]
        object_path = job["object"]
        gcda = job["gcda"]
        staged_note = job["staged_note"]
        staged_gcda = job["staged_gcda"]
        output_dir = job["output_dir"]
        emitted = sorted(output_dir.glob("*.gcov.json.gz"))
        gcov_invocations.append({"target": row["target"], "source": row["source"],
                                 "argv": [gcov, "--json-format", str(staged_note)],
                                 "returncode": result.returncode, "stdout": result.stdout,
                                 "stderr": result.stderr,
                                 "stdout_log": _sha_artifact(common, output_dir / "gcov.stdout.log"),
                                 "stderr_log": _sha_artifact(common, output_dir / "gcov.stderr.log"),
                                 "json": [_sha_artifact(common, path) for path in emitted]})
        tu_records: list[dict[str, Any]] = []
        project_tu_records: list[dict[str, Any]] = []
        if emitted:
            try:
                tu_records, json_metadata = _read_gcov_json(emitted[0], gcc_version)
                project_tu_records = project_gcov_records(tu_records, source_root, path_base=root)
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                _fail(common, f"invalid gcov JSON: {exc}")
            gcov_json_metadata.append({"path": str(emitted[0]), **json_metadata})
        records.extend(project_tu_records)
        try:
            classification, project_paths = classify_translation_unit(
                Path(row["source"]), filter_matches_project=row["filter_matches_project"],
                records=tu_records, source_root=source_root,
                gcov_output=(result.stdout or "") + "\n" + (result.stderr or ""))
        except ValueError as exc:
            _fail(common, str(exc))
        translation_units.append({"target": row["target"], "source": row["source"],
                                  "object": str(object_path), "gcno": str(note),
                                  "gcda": str(gcda) if gcda.is_file() else None,
                                  "staged_gcno": _sha_artifact(common, staged_note),
                                  "staged_gcda": _sha_artifact(common, staged_gcda)
                                  if staged_gcda.is_file() else None,
                                  "gcda_status": "present" if gcda.is_file() else "zero_data_file_absent",
                                  "classification": classification,
                                  "project_records": project_paths,
                                  "gcov_json": [str(path) for path in emitted]})

    extra_gcda = sorted(str(path) for path in gcda_paths - expected_gcda)
    try:
        gcov_report = validate_gcov_line_records(records, source_root=source_root, path_base=root)
    except ValueError as exc:
        _fail(common, f"invalid gcov physical-line data: {exc}")
    mapped_source_paths = set(gcov_report["mapped_source_paths"])
    source_hashes_before_used: dict[str, str] = {}
    source_hashes_after_used: dict[str, str] = {}
    source_hash_failures: list[str] = []
    for source_string in sorted(set(source_hashes_before) | mapped_source_paths):
        before_hash = source_tree_before.get(source_string, source_hashes_before.get(source_string))
        after_hash = source_tree_after.get(source_string)
        if after_hash is None and source_string in source_hashes_before:
            source_path = Path(source_string)
            after_hash = _hash_file(source_path) if source_path.is_file() else None
        if before_hash is None or after_hash is None:
            source_hash_failures.append(source_string)
            continue
        source_hashes_before_used[source_string] = before_hash
        source_hashes_after_used[source_string] = after_hash
        if before_hash != after_hash:
            source_hash_failures.append(source_string)
    source_hash_failures.extend(f"build evidence changed: {path}" for path in artifact_hash_failures)
    if source_tree_before_meta != source_tree_after_meta:
        source_hash_failures.append("project source tree changed during the test run")
    if extra_gcda:
        source_hash_failures.extend([f"unbound gcda: {value}" for value in extra_gcda])
    gcda_inventory = [{**_sha_artifact(common, path),
                       "selected_note": (str(expected_gcda_notes[path]) if path in expected_gcda_notes
                                         else None)}
                      for path in sorted(gcda_paths | expected_gcda)]
    integrity_failures: list[str] = []
    if gcov_report["summary"]["mapped_lines"] <= 0:
        integrity_failures.append("gcov produced no mapped project source lines")
    integrity_failed = bool(source_hash_failures or junit_problems or integrity_failures)
    percent = gcov_report["summary"]["percent"]
    line_summary = {"lines": {"count": gcov_report["summary"]["mapped_lines"],
                              "percent": percent}}
    meets_threshold = common._meets_gate(line_summary, "lines", args.threshold)
    status = "coverage_integrity_error" if integrity_failed else (
        "coverage_below_threshold" if not meets_threshold else "pass")
    report = {
        "status": status,
        "backend": "gcc-gcov",
        "denominator": "GCC gcov physical source lines deduplicated by resolved path and line number",
        "comparison_policy": "backend-specific; LLVM denominator and accepted LLVM percentages are not reused",
        "source_root": str(source_root),
        "threshold": args.threshold,
        "metrics": ["lines"],
        "summary": gcov_report["summary"],
        "files": gcov_report["files"],
        "physical_lines": gcov_report["physical_lines"],
        "translation_units": translation_units,
        "gcno_inventory": [{**_sha_artifact(common, path), "selected": path in expected_notes}
                            for path in sorted(all_notes)],
        "unselected_build_gcno": extra_notes,
        "gcda_inventory": gcda_inventory,
        "unbound_gcda": extra_gcda,
        "gcov_invocations": gcov_invocations,
        "gcov_json_versions": gcov_json_metadata,
        "source_sha256": source_hashes_before_used,
        "source_sha256_before": source_hashes_before_used,
        "source_sha256_after": source_hashes_after_used,
        "source_tree_before": source_tree_before_meta,
        "source_tree_after": source_tree_after_meta,
        "source_hash_or_artifact_failures": source_hash_failures,
        "integrity_failures": integrity_failures,
        "input_artifact_sha256_before": artifact_hashes_before,
        "input_artifact_sha256_after": artifact_hashes_after,
        "compile_commands": _sha_artifact(common, compile_commands),
        "manifests": {target: _sha_artifact(common, path)
                      for target, path in sorted(manifest_targets.items())},
        "test_objects": [_sha_artifact(common, path) for path in objects],
        "compiler": {"id": compiler_id, "version": gcc_version,
                     "cmake_record": _sha_artifact(common, compiler_record),
                     "gcov": gcov, "gcov_version": version_result.stdout.strip()},
        "compile_units": compile_rows,
        "run": run_record,
    }
    (out / "coverage-report.json").write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                                               encoding="utf-8")
    (out / "gcov-physical-export.json").write_text(
        json.dumps({"source_root": str(source_root),
                    "source_sha256_before": source_hashes_before_used,
                    "source_sha256_after": source_hashes_after_used,
                    "files": gcov_report["files"], "physical_lines": gcov_report["physical_lines"],
                    "summary": gcov_report["summary"]}, indent=2, sort_keys=True) + "\n",
        encoding="utf-8")
    return 2 if integrity_failed else (1 if status == "coverage_below_threshold" else 0)


def run_gcov_coverage(args: Any, *, common: Any) -> int:
    """Normalize expected filesystem/parser errors into the shared CLI report."""
    try:
        return _run_gcov_coverage(args, common=common)
    except common.CoverageError:
        raise
    except (OSError, ValueError) as exc:
        _fail(common, f"GCC gcov backend error: {exc}")
