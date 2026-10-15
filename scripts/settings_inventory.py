#!/usr/bin/env python3
"""Build an honest settings-variance inventory from PrintConfigDef.

The source definition is the inventory owner.  A checked-in evidence map may
claim only named dimensions exercised by a named test; every registered option
missing from that map is emitted as uncovered.  Constructing
``full_print_config`` or mentioning a key in a test source does not count as
pipeline evidence.

The report is intentionally static and small.  A native test can later replace
the evidence map with a runtime dump, but the source option set and the
coverage claims remain independently auditable.
"""

from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
from pathlib import Path
import re
import sys
from typing import Any, Iterable


class InventoryError(RuntimeError):
    """The source inventory or evidence map cannot be trusted."""


_DIRECT_ADD = re.compile(
    r"\b(?:this->)?add\(\s*\"([^\"]+)\"\s*,\s*(co[A-Za-z0-9_]+)\s*\)"
)
_DYNAMIC_ADD = re.compile(
    r"\b(?:this->)?add(?:_nullable)?\(\s*([A-Za-z_]\w*)\s*,"
)
_ENUM_PUSH = re.compile(
    r"enum_values\s*\.\s*(?:push_back|emplace_back)\(\s*\"([^\"]*)\"\s*\)"
)
_ENUM_ASSIGN = re.compile(r"enum_values\s*=\s*\{(?P<body>[^}]*)\}", re.DOTALL)
_ENUM_ALIAS = re.compile(r"enum_values\s*=\s*([A-Za-z_]\w*)->enum_values")
_QUOTED = re.compile(r'\"([^\"]*)\"')
_NUMBER_ASSIGN = re.compile(r"def->(?:min|max)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)")
_DIMENSIONS = {
    "enum_values",
    "boolean_values",
    "boundary_values",
    "representative_values",
    "pipeline_oracle",
    "interaction_matrix",
    "fixture_only",
}


def _line_number(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def _strip_cpp_comments(source: str) -> str:
    """Remove C++ comments while preserving literals and line positions."""
    out: list[str] = []
    index = 0
    state = "code"
    while index < len(source):
        char = source[index]
        next_char = source[index + 1] if index + 1 < len(source) else ""
        if state == "code":
            if char == "/" and next_char == "/":
                out.extend((" ", " "))
                index += 2
                state = "line_comment"
                continue
            if char == "/" and next_char == "*":
                out.extend((" ", " "))
                index += 2
                state = "block_comment"
                continue
            out.append(char)
            if char == '"':
                state = "string"
            elif char == "'":
                state = "character"
            index += 1
            continue
        if state == "line_comment":
            if char == "\n":
                out.append(char)
                state = "code"
            else:
                out.append(" ")
            index += 1
            continue
        if state == "block_comment":
            if char == "*" and next_char == "/":
                out.extend((" ", " "))
                index += 2
                state = "code"
            else:
                out.append("\n" if char == "\n" else " ")
                index += 1
            continue
        # String and character literals must retain comment-looking text.
        out.append(char)
        if char == "\\" and index + 1 < len(source):
            out.append(source[index + 1])
            index += 2
            continue
        if (state == "string" and char == '"') or (state == "character" and char == "'"):
            state = "code"
        index += 1
    return "".join(out)


def _print_config_segment(source: str) -> tuple[str, int]:
    start_marker = "void PrintConfigDef::init_common_params()"
    end_marker = "void PrintConfigDef::handle_legacy("
    start = source.find(start_marker)
    end = source.find(end_marker, start + len(start_marker))
    if start < 0 or end < 0 or end <= start:
        raise InventoryError("cannot locate the PrintConfigDef registration region")
    return source[start:end], start


def _extract_enum_values(body: str) -> list[str]:
    values = _ENUM_PUSH.findall(body)
    for assignment in _ENUM_ASSIGN.finditer(body):
        values.extend(_QUOTED.findall(assignment.group("body")))
    return list(dict.fromkeys(values))


def _registration(key: str, option_type: str, line: int, *, origin: str,
                  body: str = "", enum_values: Iterable[str] = (), nullable: bool = False,
                  default_declared: bool | None = None) -> dict[str, Any]:
    values = list(dict.fromkeys(enum_values))
    values.extend(value for value in _extract_enum_values(body) if value not in values)
    ranges = {match.group(0).split("->", 1)[1].split("=", 1)[0].strip()
              for match in _NUMBER_ASSIGN.finditer(body)}
    return {
        "key": key,
        "type": option_type,
        "source_line": line,
        "origin": origin,
        "enum_values": values,
        "enum_value_count": len(values),
        "range_declared": sorted(ranges),
        "nullable": nullable,
        "default_declared": ("set_default_value" in body
                              if default_declared is None else default_declared),
    }


def _filament_override_keys(source: str) -> list[str]:
    match = re.search(r"filament_extruder_override_keys\s*=\s*\{(?P<body>.*?)\};", source, re.DOTALL)
    if not match:
        raise InventoryError("cannot locate filament_extruder_override_keys")
    return _QUOTED.findall(match.group("body"))


def parse_print_config(source_path: Path) -> dict[str, Any]:
    """Extract the effective PrintConfigDef key set and registration metadata."""
    source = source_path.read_text(encoding="utf-8")
    source_without_comments = _strip_cpp_comments(source)
    segment, segment_offset = _print_config_segment(source_without_comments)
    direct_matches = list(_DIRECT_ADD.finditer(segment))
    events: list[dict[str, Any]] = []
    enum_aliases: dict[str, list[str]] = {}
    for index, match in enumerate(direct_matches):
        body_end = direct_matches[index + 1].start() if index + 1 < len(direct_matches) else len(segment)
        body = segment[match.start():body_end]
        event = _registration(
            match.group(1), match.group(2), _line_number(source, segment_offset + match.start()),
            origin="direct", body=body,
        )
        inherited: list[str] = []
        for alias in _ENUM_ALIAS.findall(body):
            inherited.extend(enum_aliases.get(alias, []))
        event["enum_values"] = list(dict.fromkeys(event["enum_values"] + inherited))
        event["enum_value_count"] = len(event["enum_values"])
        events.append(event)

        # Keep the local variable used by declarations such as
        # ``auto def_top_fill_pattern = def = this->add(...)`` so later
        # settings that copy its enum_values remain source-derived.
        line_start = segment.rfind("\n", 0, match.start()) + 1
        declaration = segment[line_start:match.start()]
        alias_match = re.search(r"([A-Za-z_]\w*)\s*=\s*def\s*=\s*$", declaration)
        if alias_match and event["enum_values"]:
            enum_aliases[alias_match.group(1)] = list(event["enum_values"])

    # The plugin override keys are a small literal loop in init_common_params.
    for loop in re.finditer(
        r"for\s*\([^)]*:\s*\{(?P<keys>[^}]*)\}\s*\)\s*\{(?P<body>.*?)\}",
        segment, re.DOTALL,
    ):
        add_match = re.search(r"add\(\s*key\s*,\s*(co[A-Za-z0-9_]+)", loop.group("body"))
        if not add_match:
            continue
        keys = _QUOTED.findall(loop.group("keys"))
        if not keys:
            continue
        line = _line_number(source, segment_offset + loop.start() + add_match.start())
        for key in keys:
            events.append(_registration(
                key, add_match.group(1), line, origin="literal_loop", default_declared=True,
            ))

    # Belt remap options are registered through one lambda with three literal
    # call sites.  The lambda's nine enum values apply to each call.
    belt_keys = re.findall(r"\badd_belt_remap\(\s*\"([^\"]+)\"", segment)
    belt_values = ["pos_x", "pos_y", "pos_z", "neg_x", "neg_y", "neg_z", "rev_x", "rev_y", "rev_z"]
    for match in re.finditer(r"\badd_belt_remap\(\s*\"([^\"]+)\"", segment):
        events.append(_registration(
            match.group(1), "coEnum", _line_number(source, segment_offset + match.start()),
            origin="literal_lambda", enum_values=belt_values, default_declared=True,
        ))

    # Filament overrides inherit the base option's type, enum values, and
    # bounds.  Resolve the list from the source rather than silently dropping
    # these generated settings from the denominator.
    by_key_direct: dict[str, dict[str, Any]] = {}
    for event in events:
        by_key_direct[event["key"]] = event
    override_loop = re.search(r"for\s*\(auto&\s+opt_key\s*:\s*filament_extruder_override_keys\s*\)", segment)
    if override_loop:
        line = _line_number(source, segment_offset + override_loop.start())
        for key in _filament_override_keys(source_without_comments):
            base_key = key.removeprefix("filament_")
            base = by_key_direct.get(base_key)
            if base is None:
                raise InventoryError(f"filament override {key} has no source base option {base_key}")
            inherited_event = _registration(
                key, str(base["type"]), line, origin="inherited_filament_override",
                enum_values=base.get("enum_values", ()), nullable=True,
                default_declared=bool(base.get("default_declared")),
            )
            inherited_event["range_declared"] = list(base.get("range_declared", ()))
            events.append(inherited_event)

    # Any remaining dynamic add must be one of the templates handled above.
    handled_dynamic = {"key", "opt_key"}
    unresolved: list[dict[str, Any]] = []
    for match in _DYNAMIC_ADD.finditer(segment):
        variable = match.group(1)
        if variable not in handled_dynamic:
            unresolved.append({
                "variable": variable,
                "source_line": _line_number(source, segment_offset + match.start()),
            })
    if unresolved:
        raise InventoryError("unresolved dynamic PrintConfigDef registrations: " + json.dumps(unresolved))

    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for event in sorted(events, key=lambda item: item["source_line"]):
        grouped[event["key"]].append(event)
    effective = []
    duplicates = []
    for key, registrations in sorted(grouped.items()):
        if len(registrations) > 1:
            duplicates.append({"key": key, "registrations": registrations})
        effective.append(registrations[-1])
    return {
        "source": str(source_path),
        "source_sha256": hashlib.sha256(source.encode("utf-8")).hexdigest(),
        "scope": "PrintConfigDef registrations (common, FFF, and SLA)",
        "registration_count": len(events),
        "option_count": len(effective),
        "duplicates": duplicates,
        "options": effective,
    }


def _required_dimensions(option_type: str) -> list[str]:
    if option_type in {"coEnum", "coEnums"}:
        return ["enum_values", "pipeline_oracle"]
    if option_type in {"coBool", "coBools"}:
        return ["boolean_values", "pipeline_oracle"]
    if option_type in {
        "coInt", "coInts", "coFloat", "coFloats", "coPercent", "coPercents",
        "coFloatOrPercent", "coFloatsOrPercents",
    }:
        return ["boundary_values", "pipeline_oracle"]
    return ["representative_values", "pipeline_oracle"]


def _load_evidence(path: Path, root: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise InventoryError(f"cannot read evidence map {path}: {exc}") from exc
    if not isinstance(payload, dict) or not isinstance(payload.get("entries"), dict):
        raise InventoryError("evidence map must contain an object-valued 'entries'")
    entries: dict[str, dict[str, Any]] = {}
    for key, item in payload["entries"].items():
        if not isinstance(item, dict):
            raise InventoryError(f"evidence entry for {key} is not an object")
        dimensions = item.get("dimensions", [])
        if not isinstance(dimensions, list) or any(value not in _DIMENSIONS for value in dimensions):
            raise InventoryError(f"evidence entry for {key} has invalid dimensions")
        reason = item.get("reason")
        test = item.get("test")
        tag = item.get("tag")
        if not isinstance(reason, str) or not reason.strip() or not isinstance(test, str) or not isinstance(tag, str):
            raise InventoryError(f"evidence entry for {key} needs test, tag, and non-empty reason")
        test_path = (root / test).resolve() if not Path(test).is_absolute() else Path(test).resolve()
        if not test_path.is_file():
            raise InventoryError(f"evidence test does not exist for {key}: {test_path}")
        test_text = test_path.read_text(encoding="utf-8")
        if tag not in test_text:
            raise InventoryError(f"evidence tag {tag!r} is absent from {test_path} for {key}")
        entries[str(key)] = {
            "dimensions": list(dict.fromkeys(dimensions)),
            "test": test,
            "tag": tag,
            "reason": reason,
        }
    return payload, entries


def build_report(*, root: Path, source_path: Path, evidence_path: Path) -> dict[str, Any]:
    inventory = parse_print_config(source_path)
    evidence_doc, evidence = _load_evidence(evidence_path, root)
    option_keys = {item["key"] for item in inventory["options"]}
    unknown = sorted(set(evidence) - option_keys)
    if unknown:
        raise InventoryError("evidence names unregistered settings: " + ", ".join(unknown))

    default_reason = str(evidence_doc.get(
        "default_uncovered_reason",
        "No dedicated settings variance evidence is declared; schema construction and source coverage do not count.",
    ))
    entries: list[dict[str, Any]] = []
    for option in inventory["options"]:
        key = option["key"]
        required = _required_dimensions(option["type"])
        claim = evidence.get(key)
        dimensions = claim["dimensions"] if claim else []
        missing = [value for value in required if value not in dimensions]
        if claim and not missing:
            status = "declared_covered"
            reason = claim["reason"]
        elif claim:
            status = "declared_partial"
            reason = claim["reason"] + " Missing required dimensions: " + ", ".join(missing) + "."
        else:
            status = "uncovered"
            reason = default_reason
        entry = dict(option)
        entry.update({
            "required_dimensions": required,
            "evidence_dimensions": dimensions,
            "status": status,
            "reason": reason,
        })
        if claim:
            entry["evidence"] = {key: claim[key] for key in ("test", "tag")}
        entries.append(entry)

    counts = Counter(item["status"] for item in entries)
    by_type: dict[str, Counter[str]] = defaultdict(Counter)
    for item in entries:
        by_type[item["type"]][item["status"]] += 1
    return {
        "schema_version": 1,
        "status": "declarations_ready" if not inventory["duplicates"] else "declarations_ready_with_duplicate_registrations",
        "coverage_claims": "declared_only",
        "execution_verified": False,
        "execution_verification": "Run the tagged native test suite separately; this static report does not assert that tests passed.",
        "scope": inventory["scope"],
        "source": inventory["source"],
        "source_sha256": inventory["source_sha256"],
        "evidence_map": str(evidence_path),
        "registration_count": inventory["registration_count"],
        "option_count": inventory["option_count"],
        "duplicate_registrations": inventory["duplicates"],
        "counts": dict(sorted(counts.items())),
        "counts_by_type": {key: dict(sorted(value.items())) for key, value in sorted(by_type.items())},
        "options": entries,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", default=".")
    parser.add_argument("--source", default="src/libslic3r/PrintConfig.cpp")
    parser.add_argument("--evidence", default="tests/settings_coverage.json")
    parser.add_argument("--out", default="build/evidence/settings-inventory.json")
    parser.add_argument("--runtime-registry",
                        help="JSON from slicer_settings_inventory; verify the static key set against the compiled registry")
    parser.add_argument("--fail-on-uncovered", action="store_true",
                        help="return 1 when any setting lacks complete evidence")
    return parser


def audit_runtime_registry(report: dict[str, Any], path: Path) -> dict[str, Any]:
    """Compare the independently compiled key set with the source parser."""
    raw = path.read_bytes()
    try:
        registry = json.loads(raw)
        options = registry["options"]
        keys = [option["key"] for option in options]
        if (registry.get("source") != "PrintConfigDef::options" or
                registry.get("schema_version") != 1 or
                registry.get("settings_count") != len(keys) or
                any(not isinstance(key, str) for key in keys) or len(set(keys)) != len(keys)):
            raise ValueError("invalid or duplicate registry keys")
    except (ValueError, TypeError, KeyError, AttributeError) as exc:
        raise InventoryError(f"invalid runtime registry {path}: {exc}") from exc
    parsed_keys = {option["key"] for option in report["options"]}
    actual_keys = set(keys)
    return {
        "path": str(path),
        "sha256": hashlib.sha256(raw).hexdigest(),
        "matches": actual_keys == parsed_keys,
        "settings_count": len(keys),
        "missing_from_source_inventory": sorted(actual_keys - parsed_keys),
        "absent_from_runtime_registry": sorted(parsed_keys - actual_keys),
    }


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    root = Path(args.root).resolve()
    source_arg = Path(args.source)
    evidence_arg = Path(args.evidence)
    out_arg = Path(args.out)
    source = (root / source_arg if not source_arg.is_absolute() else source_arg).resolve()
    evidence_path = (root / evidence_arg if not evidence_arg.is_absolute() else evidence_arg).resolve()
    out = (root / out_arg if not out_arg.is_absolute() else out_arg).resolve()
    try:
        report = build_report(root=root, source_path=source, evidence_path=evidence_path)
        report["runtime_registry_verified"] = False
        if args.runtime_registry:
            runtime_arg = Path(args.runtime_registry)
            runtime_path = (root / runtime_arg if not runtime_arg.is_absolute() else runtime_arg).resolve()
            audit = audit_runtime_registry(report, runtime_path)
            report["runtime_registry"] = audit
            report["runtime_registry_verified"] = audit["matches"]
            if not audit["matches"]:
                report["status"] = "runtime_registry_mismatch"
        if args.fail_on_uncovered and report["counts"].get("uncovered", 0) + report["counts"].get("declared_partial", 0):
            if report["status"] != "runtime_registry_mismatch":
                report["status"] = "below_required_evidence"
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps({"status": report["status"], "counts": report["counts"], "out": str(out)}))
        return 2 if report["status"] == "runtime_registry_mismatch" else 1 if report["status"] == "below_required_evidence" else 0
    except (InventoryError, OSError) as exc:
        print(f"settings_inventory: error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
