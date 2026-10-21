#!/usr/bin/env python3
"""Offline, pair-aware LLVM coverage export with an LCOV-owned line union.

The input JSON binds each test executable to its own raw profile paths and
SHA-256 digests. This collector does not run tests and never attests that they
passed or that a coverage threshold has been met.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any

import slicer_coverage as sc


def _resolve(value: str | Path, root: Path) -> Path:
    path = Path(value)
    return (root / path if not path.is_absolute() else path).resolve()


def _load_pairs(path: Path, root: Path) -> list[dict[str, Any]]:
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise sc.CoverageError(f"cannot read pair manifest {path}: {exc}") from exc
    if not isinstance(document, dict) or type(document.get("schema")) is not int or document.get("schema") != 1:
        raise sc.CoverageError("pair manifest must be a schema 1 JSON object")
    rows = document.get("pairs")
    if not isinstance(rows, list) or not rows:
        raise sc.CoverageError("pair manifest must contain at least one object/profile pair")

    seen_objects: set[str] = set()
    seen_profiles: set[str] = set()
    pairs: list[dict[str, Any]] = []
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or set(row) - {
            "object", "object_sha256", "profiles", "native_reference"
        }:
            raise sc.CoverageError(f"pair {index} has invalid fields")
        if not isinstance(row.get("object"), str) or not isinstance(row.get("profiles"), list):
            raise sc.CoverageError(f"pair {index} requires an object path and profile list")
        obj = _resolve(row["object"], root)
        obj_key = sc._path_key(obj)
        if obj_key in seen_objects:
            raise sc.CoverageError(f"duplicate object association: {obj}")
        seen_objects.add(obj_key)
        if not obj.is_file():
            raise sc.CoverageError(f"paired object does not exist: {obj}")
        expected_object_hash = row.get("object_sha256")
        if not isinstance(expected_object_hash, str) or not re.fullmatch(r"[0-9a-f]{64}", expected_object_hash):
            raise sc.CoverageError(f"pair {index} requires a lowercase object_sha256")
        try:
            actual_object_hash = sc._artifact_provenance(obj)["sha256"]
        except OSError as exc:
            raise sc.CoverageError(f"cannot hash paired object {obj}: {exc}") from exc
        if actual_object_hash != expected_object_hash:
            raise sc.CoverageError(f"object SHA-256 mismatch for {obj}")
        if not row["profiles"]:
            raise sc.CoverageError(f"pair {index} has no raw profiles")

        profiles: list[Path] = []
        profile_records: list[dict[str, str]] = []
        for profile_index, raw in enumerate(row["profiles"]):
            if not isinstance(raw, dict) or set(raw) != {"path", "sha256"}:
                raise sc.CoverageError(
                    f"pair {index} profile {profile_index} requires path and sha256"
                )
            if not isinstance(raw["path"], str) or not isinstance(raw["sha256"], str):
                raise sc.CoverageError(f"pair {index} profile {profile_index} fields must be strings")
            if not re.fullmatch(r"[0-9a-f]{64}", raw["sha256"]):
                raise sc.CoverageError(f"pair {index} profile {profile_index} has invalid SHA-256")
            profile = _resolve(raw["path"], root)
            profile_key = sc._path_key(profile)
            if profile_key in seen_profiles:
                raise sc.CoverageError(f"raw profile is associated more than once: {profile}")
            seen_profiles.add(profile_key)
            try:
                valid_profile = profile.suffix == ".profraw" and profile.is_file() and profile.stat().st_size > 0
                actual_profile_hash = sc._artifact_provenance(profile)["sha256"] if valid_profile else ""
            except OSError as exc:
                raise sc.CoverageError(f"cannot inspect raw profile {profile}: {exc}") from exc
            if not valid_profile:
                raise sc.CoverageError(f"raw profile is missing, empty, or not .profraw: {profile}")
            if actual_profile_hash != raw["sha256"]:
                raise sc.CoverageError(f"raw profile SHA-256 mismatch for {profile}")
            profiles.append(profile)
            profile_records.append({"path": str(profile), "sha256": actual_profile_hash})

        pairs.append({
            "index": index,
            "object": obj,
            "object_sha256": actual_object_hash,
            "profiles": profiles,
            "profile_records": profile_records,
            "native_reference": row.get("native_reference"),
        })
    return pairs


def _prepare_output(path: Path, source_root: Path, inputs: list[Path]) -> None:
    if path.is_symlink():
        raise sc.CoverageError(f"refusing symlinked output directory: {path}")
    if path == source_root or sc._under(path, source_root):
        raise sc.CoverageError(f"output directory must be outside source root: {path}")
    for item in inputs:
        if path == item or sc._under(item, path):
            raise sc.CoverageError(f"output directory would contain an input artifact: {item}")
    if path.exists():
        if not path.is_dir():
            raise sc.CoverageError(f"output path is not a directory: {path}")
        try:
            if any(path.iterdir()):
                raise sc.CoverageError(f"output directory must be new or empty: {path}")
        except OSError as exc:
            raise sc.CoverageError(f"cannot inspect output directory {path}: {exc}") from exc
    try:
        path.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        raise sc.CoverageError(f"cannot create output directory {path}: {exc}") from exc


def _resolve_output(value: str, root: Path) -> Path:
    raw = Path(value)
    if ".." in raw.parts:
        raise sc.CoverageError(f"output path cannot contain parent traversal: {value}")
    unresolved = raw if raw.is_absolute() else root / raw
    if unresolved.is_symlink():
        raise sc.CoverageError(f"refusing symlinked output path: {unresolved}")
    resolved = unresolved.resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError:
        return resolved
    current = root
    for part in relative.parts:
        current /= part
        if current.is_symlink():
            raise sc.CoverageError(f"refusing symlinked output path component: {current}")
    return resolved


def _lcov_summary(output: str) -> dict[str, Any] | None:
    match = re.search(
        r"(?im)^\s*lines[. ]*:\s*([0-9.]+)%\s*\(\s*([0-9,]+)\s+of\s+([0-9,]+)\s+lines?\s*\)",
        output,
    )
    if not match:
        return None
    covered = int(match.group(2).replace(",", ""))
    count = int(match.group(3).replace(",", ""))
    if covered < 0 or count < 0 or covered > count:
        return None
    return {
        "percent": 100.0 * covered / count if count else 0.0,
        "reported_percent": float(match.group(1)),
        "covered": covered,
        "count": count,
    }


def _lcov_source_maps(path: Path) -> dict[str, bool]:
    sources: dict[str, bool] = {}
    current: str | None = None
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if line.startswith("SF:"):
            current = sc._path_key(Path(line[3:]))
            sources[current] = False
        elif current is not None and line.startswith("DA:"):
            sources[current] = True
    return sources


def _lcov_line_identities(paths: list[Path], *, path_base: Path) -> set[tuple[str, int]]:
    """Return the physical source/line identities present in LCOV DA records."""
    identities: set[tuple[str, int]] = set()
    for tracefile in paths:
        current: str | None = None
        for line in tracefile.read_text(encoding="utf-8", errors="replace").splitlines():
            if line.startswith("SF:"):
                source = Path(line[3:])
                if not source.is_absolute():
                    source = path_base / source
                current = sc._path_key(source)
            elif line.startswith("DA:"):
                match = re.match(r"^DA:([0-9]+),", line)
                if current is None or match is None:
                    raise sc.CoverageError(f"invalid LCOV DA record in {tracefile}: {line!r}")
                identities.add((current, int(match.group(1))))
    return identities


def _has_profile_execution(document: Any) -> bool:
    data = document.get("data") if isinstance(document, dict) else None
    if not isinstance(data, list):
        return False
    for item in data:
        totals = item.get("totals") if isinstance(item, dict) else None
        if not isinstance(totals, dict):
            continue
        for key in ("lines", "regions", "functions", "branches", "instantiations"):
            metric = totals.get(key)
            if isinstance(metric, dict) and type(metric.get("covered")) is int and metric["covered"] > 0:
                return True
    return False


def _source_line_bounds(identities: set[tuple[str, int]], *, source_root: Path) -> dict[str, Any]:
    """Reject physical-line mappings outside the source bytes, including cold rows."""
    by_source: dict[str, list[int]] = {}
    for source, number in identities:
        by_source.setdefault(source, []).append(number)
    sources = []
    failures = []
    for source, numbers in sorted(by_source.items()):
        if not sc._under(Path(source), source_root):
            failures.append({"source": source, "reason": "source_outside_root"})
            continue
        try:
            digest = hashlib.sha256()
            size = newlines = 0
            last = b""
            with Path(source).open("rb") as stream:
                for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                    digest.update(chunk)
                    size += len(chunk)
                    newlines += chunk.count(b"\n")
                    last = chunk[-1:]
            lines = newlines + int(bool(last) and last != b"\n")
            invalid = sorted(number for number in numbers if number < 1 or number > lines)
            sources.append({"path": source, "sha256": digest.hexdigest(),
                            "size_bytes": size, "line_count": lines})
            if invalid:
                failures.append({"source": source, "reason": "line_outside_source",
                                 "line_count": lines, "invalid_identity_count": len(invalid),
                                 "line_samples": invalid[:25]})
        except (OSError, ValueError) as exc:
            failures.append({"source": source, "reason": "source_unreadable", "error": str(exc)})
    return {"status": "error" if failures else "pass", "sources": sources,
            "failures": failures}


def _input_snapshot(paths: list[Path]) -> list[dict[str, Any]]:
    unique = {sc._path_key(path): path for path in paths}
    return [sc._artifact_provenance(unique[key]) for key in sorted(unique)]


def _changed_inputs(before: list[dict[str, Any]], after: list[dict[str, Any]]) -> list[dict[str, Any]]:
    after_by_path = {item["path"]: item for item in after}
    changed = []
    for original in before:
        current = after_by_path.get(original["path"], {})
        if (original.get("status") != "present" or current.get("status") != "present"
                or original.get("sha256") != current.get("sha256")
                or original.get("size_bytes") != current.get("size_bytes")):
            changed.append({"path": original["path"], "before": original, "after": current})
    return changed


def collect(args: argparse.Namespace) -> int:
    root = Path(args.root).resolve()
    source_root = _resolve(args.source_root, root)
    build_dir = _resolve(args.build_dir, root)
    compile_commands = _resolve(args.compile_commands or (build_dir / "compile_commands.json"), root)
    pair_manifest = _resolve(args.pairs, root)
    output = _resolve_output(args.out, root)
    pairs = _load_pairs(pair_manifest, root)
    manifests = [_resolve(item, root) for item in args.manifest]
    inputs = [pair_manifest, compile_commands, *manifests, *[pair["object"] for pair in pairs]]
    inputs += [profile for pair in pairs for profile in pair["profiles"]]
    manifest_sources: list[Path] = []
    for manifest_file in manifests:
        manifest_sources.extend(sc._read_manifest(manifest_file, root))
    manifest_sources = sorted(set(manifest_sources), key=str)
    sc._require_atomic_profile_commands(manifest_sources, compile_commands)
    inputs_before = _input_snapshot(inputs)
    snapshot_by_path = {item["path"]: item["sha256"] for item in inputs_before}
    for pair in pairs:
        bound = [(pair["object"], pair["object_sha256"])]
        bound.extend((Path(item["path"]), item["sha256"]) for item in pair["profile_records"])
        for path, expected in bound:
            if snapshot_by_path.get(str(path.resolve())) != expected:
                raise sc.CoverageError(f"paired input changed before collection began: {path}")

    _prepare_output(output, source_root, inputs)
    llvm_cov = sc._which("llvm-cov", args.llvm_cov)
    llvm_profdata = sc._which("llvm-profdata", args.llvm_profdata)
    lcov = sc._which("lcov", args.lcov)
    pair_exports: list[dict[str, Any]] = []
    tracefiles: list[Path] = []
    mapping_warnings: list[dict[str, Any]] = []
    source_mapping_presence: dict[str, bool] = {}
    errors: list[dict[str, Any]] = []

    for pair in pairs:
        pair_dir = output / f"pair-{pair['index']:03d}"
        pair_dir.mkdir()
        merged = pair_dir / "merged.profdata"
        merge_argv = [llvm_profdata, "merge", "-sparse", "-o", str(merged),
                      *[str(path) for path in pair["profiles"]]]
        merge = sc._run(merge_argv, cwd=root, timeout=args.timeout)
        record: dict[str, Any] = {
            "object": str(pair["object"]),
            "object_sha256": pair["object_sha256"],
            "profiles": pair["profile_records"],
            "native_reference": pair["native_reference"],
            "merge": {"argv": merge_argv, "returncode": merge.returncode,
                      "stdout": merge.stdout, "stderr": merge.stderr},
        }
        if merge.returncode != 0 or not merged.is_file() or merged.stat().st_size == 0:
            errors.append({"object": str(pair["object"]), "reason": "profile_merge_failed"})
            pair_exports.append(record)
            continue

        json_path = pair_dir / "llvm-export-summary.json"
        json_argv = [llvm_cov, "export", "-format=text", "-summary-only",
                     "-instr-profile", str(merged), "-object", str(pair["object"])]
        exported = sc._run(json_argv, cwd=root, timeout=args.timeout)
        json_path.write_text(exported.stdout or "", encoding="utf-8")
        record["llvm_export"] = {
            "argv": json_argv, "returncode": exported.returncode,
            "path": str(json_path), "stderr": exported.stderr,
        }
        diagnostics = sc._profile_mapping_errors(exported.stderr or "")
        if diagnostics:
            mapping_warnings.append({"object": str(pair["object"]), "diagnostics": diagnostics})
        if exported.returncode != 0:
            errors.append({"object": str(pair["object"]), "reason": "llvm_export_failed"})
            pair_exports.append(record)
            continue
        try:
            summary_document = json.loads(exported.stdout)
        except json.JSONDecodeError as exc:
            errors.append({"object": str(pair["object"]), "reason": "invalid_llvm_export", "detail": str(exc)})
            pair_exports.append(record)
            continue
        record["has_profile_execution"] = _has_profile_execution(summary_document)
        if not record["has_profile_execution"]:
            errors.append({"object": str(pair["object"]), "reason": "profile_has_zero_execution"})

        raw_lcov = pair_dir / "llvm-export.lcov"
        lcov_argv = [llvm_cov, "export", "-format=lcov", "-instr-profile", str(merged),
                     "-object", str(pair["object"])]
        lcov_export = sc._run(lcov_argv, cwd=root, timeout=args.timeout)
        raw_lcov.write_text(lcov_export.stdout or "", encoding="utf-8")
        record["llvm_lcov_export"] = {
            "argv": lcov_argv, "returncode": lcov_export.returncode,
            "path": str(raw_lcov), "stderr": lcov_export.stderr,
        }
        diagnostics = sc._profile_mapping_errors(lcov_export.stderr or "")
        if diagnostics:
            mapping_warnings.append({"object": str(pair["object"]), "diagnostics": diagnostics})
        if lcov_export.returncode != 0 or not raw_lcov.stat().st_size:
            errors.append({"object": str(pair["object"]), "reason": "llvm_lcov_export_failed"})
            pair_exports.append(record)
            continue

        source_lcov = pair_dir / "source-root.lcov"
        extract_argv = [lcov, "--extract", str(raw_lcov), str(source_root) + "/*",
                        "--output-file", str(source_lcov)]
        extracted = sc._run(extract_argv, cwd=root, timeout=args.timeout)
        record["lcov_extract"] = {
            "argv": extract_argv, "returncode": extracted.returncode,
            "stdout": extracted.stdout, "stderr": extracted.stderr,
        }
        if extracted.returncode != 0 or not source_lcov.is_file() or not source_lcov.stat().st_size:
            errors.append({"object": str(pair["object"]), "reason": "lcov_source_extract_failed"})
        else:
            tracefiles.append(source_lcov)
            try:
                for path, mapped in _lcov_source_maps(source_lcov).items():
                    source_mapping_presence[path] = source_mapping_presence.get(path, False) or mapped
            except OSError as exc:
                errors.append({"object": str(pair["object"]), "reason": "cannot_read_pair_lcov",
                               "detail": str(exc)})
        pair_exports.append(record)

    inventory: dict[str, Any] | None = None
    compiler_evidence: dict[str, Any] | None = None
    if compile_commands.is_file():
        inventory = sc._create_empty_indexed_profile(
            llvm_profdata, output / "compiler-inventory-empty.profdata", timeout=args.timeout
        )
        compiler_evidence = sc._compiler_source_evidence(
            llvm_cov, manifest_sources, compile_commands, timeout=args.timeout,
            inventory_profile=Path(inventory["profile"]),
        )
    else:
        compiler_evidence = {"status": "not_run", "errors": [
            {"reason": "compile_commands_missing", "path": str(compile_commands)}
        ], "mapped_sources": [], "mapping_free_sources": [],
            "object_mappings_without_source": [], "object_executable_mappings": []}

    union_info = output / "coverage-union.info"
    summary: dict[str, Any] | None = None
    summary_output = ""
    union_command: dict[str, Any] | None = None
    union_line_audit: dict[str, Any] = {"status": "not_run"}
    source_line_audit: dict[str, Any] = {"status": "not_run", "sources": [], "failures": []}
    if tracefiles:
        union_argv = [lcov]
        for tracefile in tracefiles:
            union_argv.extend(["--add-tracefile", str(tracefile)])
        union_argv.extend(["--output-file", str(union_info)])
        union = sc._run(union_argv, cwd=root, timeout=args.timeout)
        union_command = {"argv": union_argv, "returncode": union.returncode,
                         "stdout": union.stdout, "stderr": union.stderr}
        if union.returncode != 0 or not union_info.is_file() or not union_info.stat().st_size:
            errors.append({"reason": "lcov_union_failed"})
        else:
            try:
                expected_identities = _lcov_line_identities(tracefiles, path_base=root)
                union_identities = _lcov_line_identities([union_info], path_base=root)
                source_line_audit = _source_line_bounds(
                    expected_identities | union_identities, source_root=source_root)
                missing_identities = sorted(expected_identities - union_identities)
                unexpected_identities = sorted(union_identities - expected_identities)
                union_line_audit = {
                    "status": "error" if missing_identities or unexpected_identities else "pass",
                    "expected_identity_count": len(expected_identities),
                    "union_identity_count": len(union_identities),
                    "missing_identity_count": len(missing_identities),
                    "unexpected_identity_count": len(unexpected_identities),
                    "missing_identity_samples": [
                        {"source": source, "line": line_number}
                        for source, line_number in missing_identities[:25]
                    ],
                    "unexpected_identity_samples": [
                        {"source": source, "line": line_number}
                        for source, line_number in unexpected_identities[:25]
                    ],
                }
                if missing_identities:
                    errors.append({
                        "reason": "lcov_union_missing_line_identities",
                        "count": len(missing_identities),
                        "samples": union_line_audit["missing_identity_samples"],
                    })
                if unexpected_identities:
                    errors.append({
                        "reason": "lcov_union_has_unexpected_line_identities",
                        "count": len(unexpected_identities),
                        "samples": union_line_audit["unexpected_identity_samples"],
                    })
            except (OSError, sc.CoverageError) as exc:
                union_line_audit = {"status": "error", "error": str(exc)}
                errors.append({"reason": "lcov_union_identity_audit_failed", "detail": str(exc)})
            summary_result = sc._run([lcov, "--summary", str(union_info)], cwd=root, timeout=args.timeout)
            summary_output = (summary_result.stdout or "") + (summary_result.stderr or "")
            summary = _lcov_summary(summary_output)
            union_command["summary"] = {
                "argv": [lcov, "--summary", str(union_info)],
                "returncode": summary_result.returncode,
                "output": summary_output,
            }
            if summary_result.returncode != 0 or summary is None:
                errors.append({"reason": "lcov_summary_missing_or_failed"})
    else:
        errors.append({"reason": "no_source_tracefiles"})

    # This synthesized view contains map-presence booleans only, never counts.
    mapping_report = {"files": [{"filename": path, "summary": {
        "lines": {"count": int(mapped)}
    }} for path, mapped in source_mapping_presence.items()]}
    unmapped, mapping_free_headers, unexpected_unmapped = sc._reconcile_unmapped_sources(
        mapping_report, compiler_evidence
    )
    exempt_source_keys = {
        sc._path_key(path) for path in compiler_evidence.get("mapping_free_sources", [])
    } | {
        sc._path_key(path) for path in compiler_evidence.get("object_mappings_without_source", [])
    } | {sc._path_key(path) for path in mapping_free_headers}
    lcov_exported_sources = set(source_mapping_presence)
    missing_manifest = sorted(str(path) for path in manifest_sources
                              if sc._path_key(path) not in lcov_exported_sources
                              and sc._path_key(path) not in exempt_source_keys)
    object_executable_mappings = compiler_evidence.get("object_executable_mappings", [])
    missing_object_mappings = sorted(
        path for path in object_executable_mappings
        if sc._under(Path(path), source_root) and sc._path_key(path) not in lcov_exported_sources
    )
    # The compiler audit and LLVM exports establish the mapping gates. LCOV's
    # tracefile is used only for its own standard summary and source metadata.
    missing_from_union = sorted(
        str(path) for path in manifest_sources
        if sc._path_key(path) not in lcov_exported_sources
        and sc._path_key(path) not in exempt_source_keys
    )
    if missing_from_union:
        missing_manifest = sorted(set(missing_manifest) | set(missing_from_union))

    inputs_after = _input_snapshot(inputs)
    input_stability = {
        "status": "pass",
        "checked": inputs_after,
        "changed": _changed_inputs(inputs_before, inputs_after),
    }
    if input_stability["changed"]:
        input_stability["status"] = "error"
        errors.append({"reason": "coverage_input_changed_during_collection"})

    integrity_failures = {
        "source_line_bounds": source_line_audit["failures"],
        "profile_mapping": mapping_warnings,
        "compiler_evidence": compiler_evidence.get("errors", []),
        "missing_compiled_source_mappings": missing_manifest,
        "missing_object_executable_mappings": missing_object_mappings,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "empty_line_metric": [] if summary and summary["count"] > 0 else ["LCOV line denominator is empty"],
        "collector_errors": errors,
    }
    integrity_failed = any(integrity_failures.values())
    report = {
        "schema": 1,
        "status": "coverage_integrity_error" if integrity_failed else "observed",
        "coverage_claim": False,
        "threshold_claim": False,
        "test_status": "not_checked",
        "metric_definition": (
            "LCOV 2.0 line summary after standard lcov --add-tracefile union of each "
            "object's source-root-filtered llvm-cov LCOV export; physical unique source lines."
        ),
        "llvm_summary_note": (
            "Per-object LLVM JSON summaries are retained but never added together. LLVM function-line "
            "counts can count nested functions separately; only the LCOV physical-line summary is global."
        ),
        "line_metric": summary,
        "line_metric_lcov_output": summary_output,
        "pairs": pair_exports,
        "pair_manifest": sc._artifact_provenance(pair_manifest),
        "source_manifests": [sc._artifact_provenance(path) for path in manifests],
        "source_root": str(source_root),
        "compile_commands": sc._artifact_provenance(compile_commands),
        "input_stability": input_stability,
        "compiler_inventory_profile": inventory,
        "compiler_evidence": compiler_evidence,
        "missing_compiled_source_mappings": missing_manifest,
        "missing_object_executable_mappings": missing_object_mappings,
        "unexpected_sources_without_executable_mappings": unexpected_unmapped,
        "mapping_free_headers": mapping_free_headers,
        "mapping_free_sources": sorted(compiler_evidence.get("mapping_free_sources", [])),
        "object_mappings_without_source": sorted(
            compiler_evidence.get("object_mappings_without_source", [])
        ),
        "integrity_status": "error" if integrity_failed else "pass",
        "integrity_failures": integrity_failures,
        "lcov_union": union_command,
        "lcov_union_line_audit": union_line_audit,
        "source_line_bounds_audit": source_line_audit,
    }
    try:
        (output / "coverage-report.json").write_text(
            json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    except OSError as exc:
        raise sc.CoverageError(f"cannot write coverage report: {exc}") from exc
    return 2 if integrity_failed else 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".")
    parser.add_argument("--source-root", default="src")
    parser.add_argument("--build-dir", default="build")
    parser.add_argument("--compile-commands")
    parser.add_argument("--manifest", action="append", required=True)
    parser.add_argument("--pairs", required=True, help="schema 1 object/profile SHA-256 manifest")
    parser.add_argument("--out", required=True, help="new or empty output directory")
    parser.add_argument("--llvm-cov")
    parser.add_argument("--llvm-profdata")
    parser.add_argument("--lcov")
    parser.add_argument("--timeout", type=float, default=1800.0)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        return collect(args)
    except sc.CoverageError as exc:
        print(f"coverage_pair_collector: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
