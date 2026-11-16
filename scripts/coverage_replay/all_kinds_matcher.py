#!/usr/bin/env python3
"""Fail-closed LLVM18 mapping reconciliation for every supported region kind.

This private diagnostic extends the immutable CodeRegion-only matcher.  It
reconciles CODE, EXPANSION, SKIPPED, GAP, and BRANCH records against the full
LLVM export and keeps MCDC unsupported: a future MCDC record is a hard
mismatch until this module has a verified MCDC identity model.  It does not
read ELF/profile data and does not make a coverage claim.
"""
from __future__ import annotations

import argparse
import collections
import importlib.util
import json
import re
import sys
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
ORIGINAL_PATH = HERE / "coverage_warning_matcher.py"
DECODER_PATH = HERE / "strict_llvm18_mapping.py"

# Load immutable private diagnostic modules by file path.  Registering the
# modules before exec is required by dataclasses on Python 3.14.
def _load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module

original = _load_module("a25_v2_original_matcher", ORIGINAL_PATH)
decoder = _load_module("a25_v2_mapping_decoder", DECODER_PATH)

CODE = int(decoder.CODE)
EXPANSION = int(decoder.EXPANSION)
SKIPPED = int(decoder.SKIPPED)
GAP = int(decoder.GAP)
BRANCH = int(decoder.BRANCH)
MCDC_DECISION = int(decoder.MCDC_DECISION)
MCDC_BRANCH = int(decoder.MCDC_BRANCH)
SUPPORTED_NON_MCDC = {CODE, EXPANSION, SKIPPED, GAP, BRANCH}
SYMBOL_RE = re.compile(r"^__covrec_([0-9A-Fa-f]{1,16})(u)?$")


class MatcherError(ValueError):
    pass


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise MatcherError(message)


def _load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    _require(isinstance(value, dict), f"JSON root is not an object: {path}")
    return value


def _signature_counter(decoded: dict[str, Any], paths: list[str]) -> collections.Counter[tuple[Any, ...]]:
    result: collections.Counter[tuple[Any, ...]] = collections.Counter()
    for index, region in enumerate(decoded["regions"]):
        kind = int(region.kind)
        if kind in (MCDC_DECISION, MCDC_BRANCH):
            raise MatcherError(f"MCDC region {index} kind {kind} is unsupported")
        if kind not in SUPPORTED_NON_MCDC:
            raise MatcherError(f"unknown decoded mapping region kind {kind}")
        file_id = int(region.file_id)
        expanded_id = int(region.expanded_file_id)
        _require(0 <= file_id < len(paths), f"decoded region {index} FileID is out of range")
        _require(0 <= expanded_id < len(paths),
                 f"decoded region {index} ExpandedFileID is out of range")
        result[(kind, paths[file_id], tuple(region.interval), file_id, expanded_id)] += 1
    return result


def _export_signature(function: dict[str, Any]) -> tuple[collections.Counter[tuple[Any, ...]], dict[str, int]]:
    filenames = function["_filenames"]
    _require(isinstance(filenames, list) and filenames, "export filename table is empty")
    result: collections.Counter[tuple[Any, ...]] = collections.Counter()
    counters = {"regions": 0, "branches": 0, "mcdc": 0}
    for index, raw in enumerate(function.get("_regions", [])):
        _require(isinstance(raw, list) and len(raw) >= 8,
                 f"export region {index} has fewer than eight fields")
        values = tuple(raw)
        if not all(isinstance(value, int) and not isinstance(value, bool) for value in values):
            raise MatcherError(f"export region {index} contains a non-integer field")
        line_start, column_start, line_end, column_end = values[:4]
        file_id, expanded_id, kind = values[5], values[6], values[7]
        _require(kind in (CODE, EXPANSION, SKIPPED, GAP),
                 f"export region {index} has unsupported kind {kind}")
        _require(0 <= file_id < len(filenames), f"export region {index} FileID is out of range")
        _require(0 <= expanded_id < len(filenames),
                 f"export region {index} ExpandedFileID is out of range")
        result[(kind, filenames[file_id],
                (line_start, column_start, line_end, column_end), file_id, expanded_id)] += 1
        counters["regions"] += 1
    for index, raw in enumerate(function.get("branches", [])):
        # LLVM export branch arrays are [range(4), true_count, false_count,
        # file_id, expanded_file_id, kind].  Counts are execution data and are
        # intentionally excluded from a mapping-identity signature.
        _require(isinstance(raw, list) and len(raw) >= 9,
                 f"export branch {index} has fewer than nine fields")
        values = tuple(raw)
        if not all(isinstance(value, int) and not isinstance(value, bool) for value in values):
            raise MatcherError(f"export branch {index} contains a non-integer field")
        line_start, column_start, line_end, column_end = values[:4]
        file_id, expanded_id, kind = values[6], values[7], values[8]
        _require(kind == BRANCH, f"export branch {index} has kind {kind}, expected {BRANCH}")
        _require(0 <= file_id < len(filenames), f"export branch {index} FileID is out of range")
        _require(0 <= expanded_id < len(filenames),
                 f"export branch {index} ExpandedFileID is out of range")
        result[(BRANCH, filenames[file_id],
                (line_start, column_start, line_end, column_end), file_id, expanded_id)] += 1
        counters["branches"] += 1
    mcdc_records = function.get("mcdc_records", [])
    _require(isinstance(mcdc_records, list), "export mcdc_records is not an array")
    if mcdc_records:
        counters["mcdc"] = len(mcdc_records)
        raise MatcherError("export contains MCDC records; MCDC identity is fail-closed")
    return result, counters


def _branch_export_fallbacks(decoded: dict[str, Any], paths: list[str],
                             export_signature: collections.Counter[tuple[Any, ...]],
                             function: dict[str, Any]) -> list[dict[str, Any]]:
    """Account for LLVM's known zero-counter template branch export omission.

    LLVM 18's raw map can retain a zero/zero BRANCH region for a compile-time
    template condition while llvm-cov export emits the identical interval as a
    CODE region and no branch array.  This is accepted only when the branch
    has both zero counters and an exact CODE export identity, and is reported
    explicitly.  Any nonzero branch or geometry mismatch remains a failure.
    """
    fallbacks: list[dict[str, Any]] = []
    for index, region in enumerate(decoded["regions"]):
        if int(region.kind) != BRANCH:
            continue
        identity = (BRANCH, paths[region.file_id], tuple(region.interval),
                    int(region.file_id), int(region.expanded_file_id))
        if export_signature.get(identity, 0):
            continue
        code_identity = (CODE, paths[region.file_id], tuple(region.interval),
                         int(region.file_id), int(region.expanded_file_id))
        # The decoder's branch counters are not exported as identity, but the
        # known omission is safe only for a genuinely unreachable zero/zero
        # branch.  Nonzero mappings must have an exported branch record.
        true_counter = region.counter
        false_counter = region.false_counter
        zero = (true_counter is not None and false_counter is not None and
                true_counter.kind == "zero" and true_counter.value == 0 and
                false_counter.kind == "zero" and false_counter.value == 0)
        if not zero or export_signature.get(code_identity, 0) != 1:
            raise MatcherError(
                f"branch region {index} lacks exact export branch and is not a known zero/zero CODE fallback")
        fallbacks.append({
            "region_index": index,
            "interval": list(region.interval),
            "file_id": int(region.file_id),
            "expanded_file_id": int(region.expanded_file_id),
            "reason": "llvm18_zero_counter_branch_exported_as_code",
        })
    return fallbacks


def _export_index(export_document: dict[str, Any]) -> dict[tuple[str, tuple[str, ...]], list[dict[str, Any]]]:
    data = export_document.get("data")
    _require(isinstance(data, list) and data, "export.data is empty")
    index: dict[tuple[str, tuple[str, ...]], list[dict[str, Any]]] = {}
    for data_index, item in enumerate(data):
        _require(isinstance(item, dict), f"export.data[{data_index}] is malformed")
        functions = item.get("functions")
        _require(isinstance(functions, list), f"export.data[{data_index}].functions is not an array")
        for function_index, function in enumerate(functions):
            _require(isinstance(function, dict), f"export function {data_index}:{function_index} is malformed")
            name = function.get("name")
            _require(isinstance(name, str) and name, f"export function {data_index}:{function_index}.name invalid")
            filenames = function.get("filenames")
            _require(isinstance(filenames, list) and filenames and all(isinstance(v, str) and v for v in filenames),
                     f"export function {name}.filenames invalid")
            normalized = dict(function)
            normalized["_filenames"] = filenames
            normalized["_regions"] = function.get("regions", [])
            key = (name, tuple(filenames))
            index.setdefault(key, []).append(normalized)
    return index


def reconcile_signatures(decoded: dict[str, Any], paths: list[str],
                         function: dict[str, Any]) -> dict[str, Any]:
    """Compare one decoded mapping with one export function.

    This small pure boundary is also used by the negative-control tests.  It
    deliberately leaves MCDC and unexplained branch/export differences
    fail-closed.
    """
    decoded_signature = _signature_counter(decoded, paths)
    export_signature, counters = _export_signature(function)
    fallbacks = _branch_export_fallbacks(decoded, paths, export_signature, function)
    decoded_for_compare = decoded_signature.copy()
    for item in fallbacks:
        region = decoded["regions"][item["region_index"]]
        branch_identity = (BRANCH, paths[region.file_id], tuple(region.interval),
                           int(region.file_id), int(region.expanded_file_id))
        decoded_for_compare[branch_identity] -= 1
        if decoded_for_compare[branch_identity] <= 0:
            del decoded_for_compare[branch_identity]
    if decoded_for_compare != export_signature:
        delta_decoded = list((decoded_for_compare - export_signature).items())[:3]
        delta_export = list((export_signature - decoded_for_compare).items())[:3]
        raise MatcherError(
            f"mapping/export signature differs decoded_only={delta_decoded} export_only={delta_export}")
    return {"export_counters": counters, "branch_export_fallbacks": fallbacks}


def _match_name(name: str, occurrence_count: int, binaries: list[dict[str, Any]],
                export_index: dict[tuple[str, tuple[str, ...]], list[dict[str, Any]]]) -> dict[str, Any]:
    dummy_records: list[dict[str, Any]] = []
    dummy_only = 0
    used_count = 0
    matched_count = 0
    binary_rows: list[dict[str, Any]] = []
    function_rows: list[dict[str, Any]] = []
    for binary in binaries:
        state = original._record_states(binary, name)
        if state["dummy"]:
            dummy = state["dummy"][0]
            dummy_records.append({"binary": binary["name"], **dummy})
            if not state["used"]:
                dummy_only += 1
        if not state["used"]:
            binary_rows.append({"binary": binary["name"], "dummy": bool(state["dummy"]), "used": False})
            continue
        used_count += 1
        used = state["used"][0]
        record = used["record"]
        table = binary["elf"]["covmap_filename_tables"].get(record["filenames_ref"])
        _require(isinstance(table, dict), f"{binary['name']} {name} FilenameRef table absent")
        table_paths = table.get("filenames")
        _require(isinstance(table_paths, list) and table_paths, f"{binary['name']} {name} filename table invalid")
        try:
            decoded = decoder.decode_mapping(bytes.fromhex(record["mapping_hex"]), table_paths)
        except Exception as exc:
            raise MatcherError(f"{binary['name']} {name} mapping decode failed: {exc}") from exc
        paths = decoded["virtual_paths"]
        _require(paths == used["paths"], f"{binary['name']} {name} decoded paths disagree with inspector paths")
        candidates = export_index.get((name, tuple(paths)), [])
        _require(candidates, f"{binary['name']} {name} export filename ordering is absent")
        accepted: list[tuple[dict[str, Any], collections.Counter[tuple[Any, ...]], dict[str, Any], list[dict[str, Any]]]] = []
        candidate_failures: list[str] = []
        for function in candidates:
            try:
                reconciliation = reconcile_signatures(decoded, paths, function)
                counters = reconciliation["export_counters"]
                fallbacks = reconciliation["branch_export_fallbacks"]
                export_signature, _ = _export_signature(function)
                accepted.append((function, export_signature, counters, fallbacks))
            except MatcherError as exc:
                candidate_failures.append(str(exc))
        _require(len(accepted) == 1,
                 f"{binary['name']} {name} requires exactly one all-kind export match; "
                 f"accepted={len(accepted)} failures={candidate_failures[:2]}")
        function, export_signature, counters, fallbacks = accepted[0]
        matched_count += 1
        binary_rows.append({"binary": binary["name"], "dummy": bool(state["dummy"]), "used": True,
                            "decoded_regions": len(decoded["regions"]),
                            "decoded_kind_counts": dict(collections.Counter(int(r.kind) for r in decoded["regions"])),
                            "export_region_count": counters["regions"],
                            "export_branch_count": counters["branches"],
                            "branch_export_fallbacks": fallbacks,
                            "filename_table_sha256": record.get("filename_table_sha256"),
                            "mapping_sha256": record.get("mapping_sha256")})
        function_rows.append({"binary": binary["name"], "function_filenames": paths,
                              "decoded_regions": len(decoded["regions"]),
                              "branch_export_fallbacks": fallbacks})
    _require(dummy_only == occurrence_count,
             f"{name} warning occurrences {occurrence_count} != dummy-only binaries {dummy_only}")
    _require(used_count > 0, f"{name} has no used mapping")
    # Preserve the immutable dummy-code witness requirement from the original
    # matcher; all-kind matching is an extension, not a replacement.
    for dummy in dummy_records:
        interval = dummy["interval"]
        path = dummy["paths"][0]
        witness = False
        for binary in binaries:
            state = original._record_states(binary, name)
            if not state["used"]:
                continue
            record = state["used"][0]["record"]
            table = binary["elf"]["covmap_filename_tables"][record["filenames_ref"]]["filenames"]
            decoded = decoder.decode_mapping(bytes.fromhex(record["mapping_hex"]), table)
            if decoder.has_code_witness(decoded, path, interval):
                witness = True
                break
        _require(witness, f"{name} dummy has no exact used CODE witness")
    return {"name": name, "warning_occurrences": occurrence_count,
            "dummy_only_binaries": dummy_only, "used_records": used_count,
            "matched_used_records": matched_count, "binary_rows": binary_rows,
            "function_rows": function_rows}


def match(inspector_document: Any, export_document: Any) -> dict[str, Any]:
    try:
        _require(isinstance(inspector_document, dict), "inspector document is not an object")
        _require(isinstance(export_document, dict), "export document is not an object")
        # Re-run the immutable matcher first.  This preserves its strict
        # warning multiplicity, dummy shape, hash, path, and CodeRegion checks.
        baseline = original.match(inspector_document, export_document)
        _require(baseline.get("status") == "match",
                 f"immutable CodeRegion matcher rejected: {baseline.get('failures')}")
        counts = original._warnings(inspector_document)
        binaries = inspector_document.get("binaries")
        _require(isinstance(binaries, list) and binaries, "inspector.binaries is empty")
        index = _export_index(export_document)
        names = [_match_name(name, counts[name], binaries, index) for name in sorted(counts)]
        decoded_records = sum(row["matched_used_records"] for row in names)
        fallbacks = sum(len(fallback) for row in names for f in row["function_rows"]
                        for fallback in [f["branch_export_fallbacks"]])
        kind_counts: collections.Counter[str] = collections.Counter()
        for row in names:
            for binary_row in row["binary_rows"]:
                for kind, count in binary_row.get("decoded_kind_counts", {}).items():
                    kind_counts[str(kind)] += int(count)
        return {
            "schema": 2,
            "status": "match",
            "coverage_claim": False,
            "mapping_signature_scope": "all_region_kinds_plus_branches",
            "mapping_signature_status": "exact_match_with_explicit_llvm_zero_branch_fallbacks" if fallbacks else "exact_match",
            "mcdc_status": "none_observed_fail_closed_if_present",
            "warning_names": len(names),
            "warning_occurrences": sum(counts.values()),
            "decoded_used_records": decoded_records,
            "decoded_region_kind_counts": dict(kind_counts),
            "explicit_branch_export_fallbacks": fallbacks,
            "names": names,
        }
    except (MatcherError, ValueError, KeyError, TypeError) as exc:
        return {
            "schema": 2,
            "status": "mismatch",
            "coverage_claim": False,
            "mapping_signature_scope": "all_region_kinds_plus_branches",
            "mcdc_status": "unsupported_if_observed",
            "failures": [{"reason": "all_kind_mapping_mismatch", "detail": str(exc)}],
        }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--inspector", type=Path, required=True)
    parser.add_argument("--export", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    result = match(_load_json(args.inspector), _load_json(args.export))
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"status": result["status"], "output": str(args.output)}))
    return 0 if result["status"] == "match" else 1


if __name__ == "__main__":
    raise SystemExit(main())
