#!/usr/bin/env python3
"""Pure matcher for inspector dummy records and LLVM export regions.

This module intentionally performs no provenance, profile, binary, source-root,
or coverage-percentage validation.  It only compares the already-produced
inspector schema-2 document, its warning inventory, and an LLVM full-export
JSON document.  A ``match`` result is a structural reconciliation result, not
an acceptance or coverage waiver.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re
from typing import Any


SYMBOL_RE = re.compile(r"^__covrec_([0-9A-Fa-f]{1,16})(u)?$")
REQUIRED_REGION_FIELDS = 8


class MatcherError(ValueError):
    pass


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise MatcherError(message)


def _int(value: Any, label: str, *, minimum: int | None = None) -> int:
    _require(isinstance(value, int) and not isinstance(value, bool), f"{label} is not an integer")
    if minimum is not None:
        _require(value >= minimum, f"{label} is below {minimum}")
    return value


def _string(value: Any, label: str) -> str:
    _require(isinstance(value, str) and value != "", f"{label} is not a nonempty string")
    return value


def _list(value: Any, label: str) -> list[Any]:
    _require(isinstance(value, list), f"{label} is not an array")
    return value


def _hex_ref(value: Any, label: str) -> int:
    if isinstance(value, int) and not isinstance(value, bool):
        _require(value >= 0, f"{label} is negative")
        return value
    _require(isinstance(value, str), f"{label} is not a hex string")
    _require(re.fullmatch(r"0x[0-9A-Fa-f]{1,16}", value) is not None,
             f"{label} is malformed")
    return int(value, 16)


def _path_list(value: Any, label: str) -> list[str]:
    entries = _list(value, label)
    result: list[str] = []
    for index, entry in enumerate(entries):
        result.append(_string(entry, f"{label}[{index}]").replace("\\", "/"))
    _require(result, f"{label} is empty")
    return result


def _name_hash(value: str) -> int:
    try:
        digest = hashlib.md5(value.encode("utf-8"), usedforsecurity=False).digest()
    except TypeError:
        digest = hashlib.md5(value.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "little")


def _region_tuple(region: Any, label: str) -> tuple[int, ...]:
    values = _list(region, label)
    _require(len(values) >= REQUIRED_REGION_FIELDS, f"{label} has fewer than eight fields")
    result = tuple(_int(value, f"{label}[{index}]") for index, value in enumerate(values))
    return result


def _dummy_interval(record: dict[str, Any], label: str) -> tuple[int, int, int, int]:
    shape = record.get("mapping_shape")
    _require(isinstance(shape, dict), f"{label} lacks mapping_shape")
    _require(shape.get("mapping_validation") == "one-region-complete",
             f"{label} is not a complete one-region mapping")
    _require(shape.get("dummy") is True and shape.get("trailing_bytes") == 0,
             f"{label} is not a complete dummy")
    _require(shape.get("file_mappings") == 1, f"{label} has multiple virtual files")
    indices = _list(shape.get("filename_indices"), f"{label}.filename_indices")
    _require(len(indices) == 1, f"{label} has no single filename index")
    file_index = _int(indices[0], f"{label}.filename_indices[0]", minimum=0)
    if shape.get("filename_index") != file_index:
        raise MatcherError(f"{label} filename_index disagrees with filename_indices")
    interval = record.get("dummy_region")
    _require(isinstance(interval, dict), f"{label} lacks absolute dummy_region")
    fields = tuple(
        _int(interval.get(name), f"{label}.dummy_region.{name}", minimum=0)
        for name in ("line_start", "column_start", "line_end", "column_end")
    )
    start_line, start_column, end_line, end_column = fields
    _require(end_line >= start_line, f"{label} has descending line interval")
    _require(start_line != end_line or end_column >= start_column,
             f"{label} has descending same-line interval")
    coordinates = shape.get("region_coordinates")
    _require(isinstance(coordinates, list) and tuple(coordinates) == fields,
             f"{label} absolute interval disagrees with mapping_shape")
    return fields


def _resolve_record_paths(binary: dict[str, Any], record: dict[str, Any], label: str) -> list[str]:
    elf = binary.get("elf")
    _require(isinstance(elf, dict), f"{label} binary lacks elf metadata")
    tables = elf.get("covmap_filename_tables")
    _require(isinstance(tables, dict) and tables, f"{label} binary lacks covmap filename tables")
    ref_text = _string(record.get("filenames_ref"), f"{label}.filenames_ref")
    ref_value = _hex_ref(ref_text, f"{label}.filenames_ref")
    canonical_ref = f"0x{ref_value:016X}"
    table_raw = tables.get(canonical_ref)
    if table_raw is None:
        # The inspector emits uppercase fixed-width keys. Accepting a key with
        # a different spelling would hide an inconsistent producer document.
        raise MatcherError(f"{label} FilenameRef {canonical_ref} is absent")
    _require(isinstance(table_raw, dict), f"{label} FilenameRef table is malformed")
    table_paths = _path_list(table_raw.get("filenames"), f"{label}.filename_table.filenames")
    shape = record.get("mapping_shape")
    _require(isinstance(shape, dict), f"{label} lacks mapping_shape")
    indices = _list(shape.get("filename_indices"), f"{label}.filename_indices")
    resolved: list[str] = []
    for index, raw_index in enumerate(indices):
        filename_index = _int(raw_index, f"{label}.filename_indices[{index}]", minimum=0)
        _require(filename_index < len(table_paths),
                 f"{label} filename index {filename_index} is out of range")
        resolved.append(table_paths[filename_index])
    emitted_paths = _path_list(record.get("filename_paths"), f"{label}.filename_paths")
    _require(emitted_paths == resolved, f"{label} filename_paths disagree with its TU table")
    return resolved


def _record_states(binary: dict[str, Any], warning_name: str) -> dict[str, list[dict[str, Any]]]:
    entries = _list(binary.get("warnings"), f"binary {binary.get('name')}.warnings")
    matching_entries: list[dict[str, Any]] = []
    found: list[dict[str, Any]] = []
    for entry_index, entry in enumerate(entries):
        _require(isinstance(entry, dict), f"binary warning entry {entry_index} is malformed")
        if entry.get("name") != warning_name:
            continue
        matching_entries.append(entry)
        records = _list(entry.get("records"), f"{binary.get('name')} warning {warning_name}.records")
        if entry.get("record_count") is not None:
            _require(entry.get("record_count") == len(records),
                     f"{binary.get('name')} warning {warning_name} record_count disagrees")
        found.extend(records)
    _require(len(matching_entries) == 1,
             f"{binary.get('name')} must have exactly one warning entry for {warning_name}")
    _require(len(found) <= 2, f"{binary.get('name')} has duplicate records for {warning_name}")
    state: dict[str, list[dict[str, Any]]] = {"dummy": [], "used": []}
    for index, record in enumerate(found):
        _require(isinstance(record, dict), f"{binary.get('name')} {warning_name} record {index} is malformed")
        symbol = _string(record.get("symbol"), f"{binary.get('name')} {warning_name} record symbol")
        match = SYMBOL_RE.fullmatch(symbol)
        _require(match is not None, f"{binary.get('name')} has malformed COVFUN symbol {symbol}")
        _require(int(match.group(1), 16) == _name_hash(warning_name),
                 f"{symbol} NameRef does not hash warning name {warning_name}")
        function_hash = _int(record.get("function_hash"), f"{symbol}.function_hash", minimum=0)
        used_suffix = bool(match.group(2))
        if not used_suffix:
            _require(function_hash == 0 and record.get("used_suffix") is False,
                     f"{symbol} is not a zero-hash unsuffixed record")
            _require(record.get("dummy") is True, f"{symbol} is not marked dummy")
            interval = _dummy_interval(record, symbol)
            paths = _resolve_record_paths(binary, record, symbol)
            state["dummy"].append({
                "record": record,
                "paths": paths,
                "interval": interval,
            })
        else:
            _require(function_hash != 0 and record.get("used_suffix") is True,
                     f"{symbol} is not a nonzero used record")
            _require(record.get("dummy") is False, f"{symbol} is incorrectly marked dummy")
            _require(isinstance(record.get("mapping_hex"), str) and record["mapping_hex"],
                     f"{symbol} lacks used mapping bytes")
            paths = _resolve_record_paths(binary, record, symbol)
            state["used"].append({"record": record, "paths": paths})
    _require(len(state["dummy"]) <= 1, f"{binary.get('name')} has multiple dummy records for {warning_name}")
    _require(len(state["used"]) <= 1, f"{binary.get('name')} has multiple used records for {warning_name}")
    return state


def _warnings(document: dict[str, Any]) -> dict[str, int]:
    inventory = document.get("warning_inventory")
    _require(isinstance(inventory, dict), "inspector document lacks warning_inventory")
    records = _list(inventory.get("records"), "warning_inventory.records")
    _require(records, "warning_inventory.records is empty")
    counts: dict[str, int] = {}
    for index, record in enumerate(records):
        _require(isinstance(record, dict), f"warning record {index} is malformed")
        name = _string(record.get("name"), f"warning record {index}.name")
        mapping_hash = _int(record.get("mapping_hash"), f"warning record {index}.mapping_hash", minimum=0)
        _require(mapping_hash == 0, f"warning {name} has nonzero mapping hash")
        counts[name] = counts.get(name, 0) + 1
    return counts


def _export_functions(export_document: dict[str, Any]) -> dict[str, list[dict[str, Any]]]:
    data = _list(export_document.get("data"), "export.data")
    _require(data, "export.data is empty")
    functions: dict[str, list[dict[str, Any]]] = {}
    for data_index, item in enumerate(data):
        _require(isinstance(item, dict), f"export.data[{data_index}] is malformed")
        for function_index, function in enumerate(_list(item.get("functions"), f"export.data[{data_index}].functions")):
            _require(isinstance(function, dict), f"export function {data_index}:{function_index} is malformed")
            name = _string(function.get("name"), f"export function {data_index}:{function_index}.name")
            filenames = _path_list(function.get("filenames"), f"export function {name}.filenames")
            regions = _list(function.get("regions"), f"export function {name}.regions")
            normalized = dict(function)
            normalized["_filenames"] = filenames
            normalized["_regions"] = regions
            functions.setdefault(name, []).append(normalized)
    return functions


def _match_name(name: str, occurrence_count: int, binaries: list[dict[str, Any]],
                functions: dict[str, list[dict[str, Any]]]) -> dict[str, Any]:
    dummy_records: list[dict[str, Any]] = []
    dummy_only_binaries: list[str] = []
    used_paths: list[list[str]] = []
    per_binary: list[dict[str, Any]] = []
    for binary_index, binary in enumerate(binaries):
        _require(isinstance(binary, dict), f"binary {binary_index} is malformed")
        binary_name = _string(binary.get("name"), f"binary {binary_index}.name")
        state = _record_states(binary, name)
        if state["dummy"]:
            dummy = state["dummy"][0]
            dummy_records.append({"binary": binary_name, **dummy})
            if not state["used"]:
                dummy_only_binaries.append(binary_name)
        if state["used"]:
            used_paths.append(state["used"][0]["paths"])
        per_binary.append({"binary": binary_name, "dummy": bool(state["dummy"]), "used": bool(state["used"])})

    _require(len(dummy_only_binaries) == occurrence_count,
             f"{name} warning occurrences {occurrence_count} != dummy-only binaries "
             f"{len(dummy_only_binaries)}")
    _require(used_paths, f"{name} has no nonzero used COVFUN record")
    candidates = functions.get(name, [])
    _require(candidates, f"{name} is absent from LLVM export functions")
    used_path_tuples = {tuple(paths) for paths in used_paths}
    accepted_functions = [
        function for function in candidates if tuple(function["_filenames"]) in used_path_tuples
    ]
    _require(accepted_functions, f"{name} export filenames do not match any used COVFUN ordering")

    matched: list[dict[str, Any]] = []
    for dummy in dummy_records:
        candidates_for_dummy = [
            function for function in accepted_functions
            if dummy["paths"][0] in function["_filenames"]
        ]
        _require(candidates_for_dummy,
                 f"{name} dummy filename does not occur in an ordered used/export list")
        interval = dummy["interval"]
        matching_regions: list[dict[str, Any]] = []
        for function in candidates_for_dummy:
            for region_index, raw_region in enumerate(function["_regions"]):
                region = _region_tuple(raw_region, f"export {name} region {region_index}")
                file_id = region[5]
                kind = region[7]
                if file_id < 0 or file_id >= len(function["_filenames"]):
                    continue
                if function["_filenames"][file_id] != dummy["paths"][0]:
                    continue
                if kind != 0:
                    continue
                if region[:4] != interval:
                    continue
                matching_regions.append({
                    "function_filenames": function["_filenames"],
                    "region_index": region_index,
                    "file_id": file_id,
                    "kind": kind,
                    "interval": list(region[:4]),
                })
        _require(matching_regions,
                 f"{name} has no export region with matching FileID, kind, filename, and interval")
        matched.append({"binary": dummy["binary"], "regions": matching_regions})
    return {
        "name": name,
        "warning_occurrences": occurrence_count,
        "dummy_record_count": len(dummy_records),
        "dummy_only_binary_count": len(dummy_only_binaries),
        "used_cfun_orders": used_paths,
        "per_binary": per_binary,
        "matched_dummy_regions": matched,
    }


def _run(inspector_document: dict[str, Any], export_document: dict[str, Any]) -> dict[str, Any]:
    _require(inspector_document.get("schema") == 2, "inspector schema must be 2")
    _require(inspector_document.get("status") == "complete", "inspector status must be complete")
    counts = _warnings(inspector_document)
    binaries = _list(inspector_document.get("binaries"), "inspector.binaries")
    _require(binaries, "inspector.binaries is empty")
    binary_names = [_string(binary.get("name"), f"binary {index}.name")
                    for index, binary in enumerate(binaries)]
    _require(len(set(binary_names)) == len(binary_names), "inspector has duplicate binary names")
    functions = _export_functions(export_document)
    names: list[dict[str, Any]] = []
    for name in sorted(counts):
        names.append(_match_name(name, counts[name], binaries, functions))
    return {
        "schema": 1,
        "status": "match",
        "coverage_claim": False,
        "warning_names": len(names),
        "warning_occurrences": sum(counts.values()),
        "names": names,
    }


def match(inspector_document: Any, export_document: Any) -> dict[str, Any]:
    """Return a structural match result; never returns an acceptance claim."""
    try:
        _require(isinstance(inspector_document, dict), "inspector document is not an object")
        _require(isinstance(export_document, dict), "export document is not an object")
        return _run(inspector_document, export_document)
    except MatcherError as exc:
        return {
            "schema": 1,
            "status": "mismatch",
            "coverage_claim": False,
            "failures": [{"reason": "structural_mismatch", "detail": str(exc)}],
        }


def _load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    _require(isinstance(value, dict), f"JSON root is not an object: {path}")
    return value


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspector", type=Path, required=True)
    parser.add_argument("--export", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = match(_load(args.inspector), _load(args.export))
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "output": str(args.output)}))
    return 0 if result["status"] == "match" else 1


if __name__ == "__main__":
    raise SystemExit(main())
