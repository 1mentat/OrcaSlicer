#!/usr/bin/env python3
"""Private strict wrapper around the retained structural warning matcher."""

from __future__ import annotations

import importlib.util
from collections import Counter
from pathlib import Path

from strict_llvm18_mapping import MappingError, decode_mapping, has_code_witness


PACKAGE_ROOT = Path(__file__).resolve().parent
ORIGINAL_DIR = PACKAGE_ROOT


def _load_original():
    path = ORIGINAL_DIR / "coverage_warning_matcher.py"
    spec = importlib.util.spec_from_file_location("strict_wrapper_original_matcher", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


original = _load_original()


def _table_paths(binary: dict, record: dict) -> list[str]:
    tables = binary["elf"]["covmap_filename_tables"]
    ref_value = int(record["filenames_ref"], 16)
    table = tables[f"0x{ref_value:016X}"]
    return list(table["filenames"])


def strict_match(inspector_document: dict, export_document: dict) -> dict:
    """Run the retained checks plus exact used-map CodeRegion witnesses.

    Dummy and used records may be in different binaries.  The strict witness
    is searched across every correctly decoded used record, but each mapping
    is decoded against its own ordered FilenameRef table.
    """

    result = original.match(inspector_document, export_document)
    if result.get("status") != "match":
        return result
    try:
        counts = original._warnings(inspector_document)
        binaries = original._list(inspector_document.get("binaries"), "inspector.binaries")
        decoded_by_name: dict[str, list[dict]] = {name: [] for name in counts}
        dummies_by_name: dict[str, list[dict]] = {name: [] for name in counts}
        for binary in binaries:
            for name in counts:
                state = original._record_states(binary, name)
                for dummy in state["dummy"]:
                    dummies_by_name[name].append(dummy)
                for used in state["used"]:
                    record = used["record"]
                    raw = bytes.fromhex(record["mapping_hex"])
                    tu_paths = _table_paths(binary, record)
                    decoded = decode_mapping(raw, tu_paths)
                    if decoded["virtual_paths"] != used["paths"]:
                        raise MappingError(
                            f"{binary['name']} {name}: decoded virtual filename order disagrees"
                        )
                    decoded_by_name[name].append({
                        "binary": binary["name"],
                        "paths": list(used["paths"]),
                        "decoded": decoded,
                    })
        for name in sorted(counts):
            if not decoded_by_name[name]:
                raise MappingError(f"{name}: no decoded nonzero used mapping")
            for dummy in dummies_by_name[name]:
                interval = tuple(dummy["interval"])
                path = dummy["paths"][0]
                if not any(has_code_witness(item["decoded"], path, interval)
                           for item in decoded_by_name[name]):
                    raise MappingError(
                        f"{name}: no used mapping CodeRegion witness for {path} {interval}"
                    )
        # Bind each used record's complete CodeRegion geometry to one export
        # function with the exact same ordered filename array.  This prevents
        # two different used records from separately satisfying only a body
        # interval against one global function witness.
        functions = original._export_functions(export_document)
        geometry_records: list[dict] = []
        for name in sorted(counts):
            for item in decoded_by_name[name]:
                decoded = item["decoded"]
                used_signature = Counter(
                    (
                        decoded["virtual_paths"][region.file_id],
                        region.interval,
                        region.file_id,
                        region.expanded_file_id,
                        region.kind,
                    )
                    for region in decoded["regions"]
                    if region.kind == strict_code_region_kind()
                )
                candidates = [
                    (index, function)
                    for index, function in enumerate(functions.get(name, []))
                    if tuple(function["_filenames"]) == tuple(item["paths"])
                ]
                matched_index = None
                observed_signature: Counter | None = None
                for index, function in candidates:
                    signature = Counter()
                    for region_index, raw_region in enumerate(function["_regions"]):
                        region = original._region_tuple(
                            raw_region, f"export {name} region {region_index}"
                        )
                        file_id, expanded_file_id, kind = region[5], region[6], region[7]
                        if kind != strict_code_region_kind():
                            continue
                        if file_id < 0 or file_id >= len(function["_filenames"]):
                            continue
                        signature[(
                            function["_filenames"][file_id],
                            tuple(region[:4]),
                            file_id,
                            expanded_file_id,
                            kind,
                        )] += 1
                    if signature == used_signature:
                        matched_index = index
                        observed_signature = signature
                        break
                    if observed_signature is None:
                        observed_signature = signature
                record_result = {
                    "binary": item["binary"],
                    "name": name,
                    "ordered_filenames": item["paths"],
                    "used_code_region_count": sum(used_signature.values()),
                    "candidate_count": len(candidates),
                }
                if matched_index is None:
                    record_result["status"] = "mismatch"
                    record_result["matched_function_index"] = None
                    record_result["observed_candidate_code_region_count"] = (
                        sum(observed_signature.values()) if observed_signature is not None else None
                    )
                else:
                    record_result["status"] = "match"
                    record_result["matched_function_index"] = matched_index
                    record_result["observed_candidate_code_region_count"] = sum(observed_signature.values())
                geometry_records.append(record_result)
        geometry_failures = [record for record in geometry_records if record["status"] != "match"]
        if geometry_failures:
            return {
                "schema": 1,
                "status": "mismatch",
                "coverage_claim": False,
                "failures": [{
                    "reason": "used_export_geometry_mismatch",
                    "detail": f"{len(geometry_failures)} used records lack an exact ordered-file CodeRegion signature",
                    "records": geometry_failures,
                }],
                "used_mapping_validation": {
                    "status": "geometry_mismatch",
                    "decoded_used_records": len(geometry_records),
                    "geometry_records": geometry_records,
                    "same_binary_required": False,
                },
            }
        strict_result = dict(result)
        strict_result["used_mapping_validation"] = {
            "status": "strict_match",
            "decoded_used_records": sum(len(records) for records in decoded_by_name.values()),
            "geometry_status": "exact_ordered_filename_and_code_region_match",
            "geometry_records": geometry_records,
            "same_binary_required": False,
        }
        return strict_result
    except (KeyError, TypeError, ValueError, MappingError, original.MatcherError) as exc:
        return {
            "schema": 1,
            "status": "mismatch",
            "coverage_claim": False,
            "failures": [{"reason": "used_mapping_mismatch", "detail": str(exc)}],
        }


def strict_code_region_kind() -> int:
    # Kept as a named local contract so the wrapper does not depend on an
    # implementation-specific enum import from the decoder module.
    return 0
