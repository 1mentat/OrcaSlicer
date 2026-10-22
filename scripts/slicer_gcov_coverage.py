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


def validate_gcov_output(returncode: int, emitted_count: int, source: str) -> None:
    """Keep the existing fail-closed gcov process/output cardinality gate."""
    if returncode != 0 or emitted_count > 1:
        raise ValueError(f"gcov failed or emitted ambiguous JSON for {source}")


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
        validate_gcov_output(result.returncode, emitted_count, source)
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


def _sha_artifact(common: Any, path: Path) -> dict[str, Any]:
    return common._artifact_provenance(path)


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
    preview = common._run([*command, "--show-only=json-v1"], cwd=root, timeout=args.timeout)
    if preview.returncode != 0:
        _fail(common, f"CTest discovery failed: {preview.stderr}")
    try:
        discovery = json.loads(preview.stdout)
        selected_names = [test["name"] for test in discovery["tests"]]
    except (KeyError, TypeError, json.JSONDecodeError) as exc:
        _fail(common, f"CTest discovery JSON is invalid: {exc}")
    if not selected_names or len(set(selected_names)) != len(selected_names):
        _fail(common, "CTest selection is empty or contains duplicate test identities")
    discovery_path = out / "ctest-discovery.json"
    discovery_path.write_text(preview.stdout, encoding="utf-8")

    source_tree_before = _source_tree_snapshot(source_root)
    source_hashes_before = {str(source): _hash_file(source)
                            for sources in source_by_target.values() for source in sources}
    artifact_paths = {compile_commands, cache_path, compiler_record, *manifests, *objects, *expected_notes}
    try:
        artifact_hashes_before = _artifact_snapshot(common, artifact_paths)
    except ValueError as exc:
        _fail(common, str(exc))
    gcov_prefix = out / "gcov-data"
    gcov_prefix.mkdir()
    junit_path = out / "ctest-junit.xml"
    env = dict(os.environ)
    env["GCOV_PREFIX"] = str(gcov_prefix)
    env["GCOV_PREFIX_STRIP"] = "0"
    started = __import__("time").monotonic()
    test = common._run([*command, "--output-junit", str(junit_path)], cwd=root,
                       env=env, timeout=args.timeout)
    elapsed = __import__("time").monotonic() - started
    (out / "ctest.stdout.log").write_text(test.stdout or "", encoding="utf-8")
    (out / "ctest.stderr.log").write_text(test.stderr or "", encoding="utf-8")
    source_tree_after = _source_tree_snapshot(source_root)
    artifact_hashes_after = _artifact_snapshot(common, artifact_paths, require_present=False)
    source_tree_before_meta = {"file_count": len(source_tree_before),
                               "sha256": _snapshot_digest(source_tree_before)}
    source_tree_after_meta = {"file_count": len(source_tree_after),
                              "sha256": _snapshot_digest(source_tree_after)}
    artifact_hash_failures = sorted(
        path for path in artifact_hashes_before
        if artifact_hashes_before[path] != artifact_hashes_after.get(path))
    run_record: dict[str, Any] = {
        "argv": [*command, "--output-junit", str(junit_path)],
        "preview_argv": [*command, "--show-only=json-v1"],
        "selected_test_count": len(selected_names), "selected_test_names": selected_names,
        "discovery": _sha_artifact(common, discovery_path),
        "returncode": test.returncode, "timed_out": test.returncode == 124,
        "duration_seconds": elapsed,
        "stdout_sha256": _hash_file(out / "ctest.stdout.log"),
        "stderr_sha256": _hash_file(out / "ctest.stderr.log"),
        "junit": _sha_artifact(common, junit_path),
        "gcov_prefix": str(gcov_prefix), "gcov_prefix_strip": 0,
        "gcov_jobs": gcov_jobs,
    }
    if test.returncode != 0 or not junit_path.is_file():
        report = {"status": "test_failed" if test.returncode else "missing_junit",
                  "backend": "gcc-gcov", "denominator": "gcc physical lines only",
                  "run": run_record, "notes": sorted(map(str, expected_notes)),
                  "extra_build_notes_unselected": extra_notes,
                  "input_artifact_sha256_before": artifact_hashes_before,
                  "input_artifact_sha256_after": artifact_hashes_after,
                  "input_artifact_hash_failures": artifact_hash_failures,
                  "source_tree_before": source_tree_before_meta,
                  "source_tree_after": source_tree_after_meta,
                  "raw_gcda": [_sha_artifact(common, path)
                               for path in sorted(gcov_prefix.rglob("*.gcda")) if path.is_file()]}
        (out / "coverage-report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return 2
    try:
        junit_names, junit_problems = _junit_summary(junit_path)
    except (ET.ParseError, OSError, ValueError) as exc:
        _fail(common, f"cannot validate CTest JUnit output: {exc}")
    expected_names = Counter(selected_names)
    run_record["junit_test_count"] = sum(junit_names.values())
    run_record["junit_name_mismatch"] = compare_test_identities(selected_names, junit_names)
    run_record["junit_problems"] = junit_problems
    if junit_names != expected_names or junit_problems:
        (out / "coverage-report.json").write_text(json.dumps({
            "status": "test_identity_or_junit_failure", "backend": "gcc-gcov",
            "denominator": "gcc physical lines only", "run": run_record,
        }, indent=2) + "\n", encoding="utf-8")
        return 2

    raw_data_dir = out / "gcov-input"
    raw_data_dir.mkdir()
    gcda_paths = {path.resolve() for path in gcov_prefix.rglob("*.gcda") if path.is_file()}
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
        gcda = gcda_path_for_object(gcov_prefix, object_path)
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
