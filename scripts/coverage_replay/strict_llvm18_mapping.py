#!/usr/bin/env python3
"""Small, fail-closed LLVM 18 RawCoverageMappingReader model.

This private diagnostic decoder mirrors the byte grammar in LLVM 18.1.3's
RawCoverageMappingReader.  It decodes used COVFUN mappings only; it does not
read ELF files, profiles, or compiler output.
"""

from __future__ import annotations

from dataclasses import dataclass


UINT_MAX = (1 << 32) - 1
COUNTER_TAG_MASK = 0x3
EXPANSION_BIT = 1 << 2
EXPANSION_SHIFT = 3

CODE = 0
EXPANSION = 1
SKIPPED = 2
GAP = 3
BRANCH = 4
MCDC_DECISION = 5
MCDC_BRANCH = 6


class MappingError(ValueError):
    """Malformed or truncated LLVM18 mapping data."""


@dataclass(frozen=True)
class Counter:
    kind: str
    value: int


@dataclass(frozen=True)
class Region:
    file_id: int
    expanded_file_id: int
    line_start: int
    column_start: int
    line_end: int
    column_end: int
    kind: int
    counter: Counter
    false_counter: Counter | None = None
    mcdc: tuple[int, ...] = ()

    @property
    def interval(self) -> tuple[int, int, int, int]:
        return (self.line_start, self.column_start, self.line_end, self.column_end)


class _Reader:
    def __init__(self, data: bytes):
        self.data = data
        self.cursor = 0

    def remaining(self) -> int:
        return len(self.data) - self.cursor

    def uleb(self, label: str) -> int:
        if self.cursor >= len(self.data):
            raise MappingError(f"{label}: truncated")
        value = 0
        shift = 0
        while True:
            if self.cursor >= len(self.data):
                raise MappingError(f"{label}: truncated ULEB128")
            byte = self.data[self.cursor]
            self.cursor += 1
            payload = byte & 0x7F
            if shift >= 64 or (shift == 63 and payload > 1):
                raise MappingError(f"{label}: ULEB128 exceeds uint64")
            value |= payload << shift
            if not byte & 0x80:
                return value
            shift += 7
            if shift >= 64:
                raise MappingError(f"{label}: ULEB128 is too large")

    def read_int_max(self, label: str, maximum_exclusive: int) -> int:
        value = self.uleb(label)
        if value >= maximum_exclusive:
            raise MappingError(f"{label}: {value} is >= {maximum_exclusive}")
        return value

    def read_size(self, label: str) -> int:
        value = self.uleb(label)
        if value > self.remaining():
            raise MappingError(f"{label}: size {value} exceeds remaining bytes")
        return value


def _counter(reader: _Reader, expressions: list[tuple[Counter, Counter]], label: str) -> Counter:
    encoded = reader.read_int_max(label, UINT_MAX)
    tag = encoded & COUNTER_TAG_MASK
    if tag == 0:
        return Counter("zero", 0)
    if tag == 1:
        return Counter("counter", encoded >> 2)
    expression_id = encoded >> 2
    if expression_id >= len(expressions):
        raise MappingError(f"{label}: expression {expression_id} is invalid")
    if tag == 2:
        expressions[expression_id] = (expressions[expression_id][0], expressions[expression_id][1])
        return Counter("expression-subtract", expression_id)
    if tag == 3:
        expressions[expression_id] = (expressions[expression_id][0], expressions[expression_id][1])
        return Counter("expression-add", expression_id)
    raise MappingError(f"{label}: invalid counter tag {tag}")


def decode_mapping(mapping: bytes, translation_unit_paths: list[str]) -> dict:
    """Decode one LLVM18 used mapping and return ordered virtual files/regions.

    ``translation_unit_paths`` is the ordered filename table resolved for this
    COVFUN record.  The mapping's virtual-file indices are checked against it;
    each region is returned with its resolved virtual file path.
    """

    if not translation_unit_paths:
        raise MappingError("translation unit filename table is empty")
    reader = _Reader(mapping)
    virtual_indices_count = reader.read_size("virtual file mapping count")
    virtual_indices = [
        reader.read_int_max(f"virtual filename index {index}", len(translation_unit_paths))
        for index in range(virtual_indices_count)
    ]
    if not virtual_indices:
        raise MappingError("virtual file mapping is empty")
    virtual_paths = [translation_unit_paths[index] for index in virtual_indices]

    expression_count = reader.read_size("expression count")
    expressions: list[tuple[Counter, Counter]] = [
        (Counter("zero", 0), Counter("zero", 0)) for _ in range(expression_count)
    ]
    for index in range(expression_count):
        lhs = _counter(reader, expressions, f"expression {index} lhs")
        rhs = _counter(reader, expressions, f"expression {index} rhs")
        expressions[index] = (lhs, rhs)

    regions: list[Region] = []
    for inferred_file_id in range(len(virtual_indices)):
        region_count = reader.read_size(f"file {inferred_file_id} region count")
        line_start = 0
        for region_index in range(region_count):
            encoded = reader.read_int_max(
                f"file {inferred_file_id} region {region_index} counter/kind", UINT_MAX
            )
            tag = encoded & COUNTER_TAG_MASK
            expanded_file_id = 0
            kind = CODE
            counter = Counter("zero", 0)
            false_counter: Counter | None = None
            mcdc: tuple[int, ...] = ()
            if tag != 0:
                counter = _decode_counter_value(encoded, expressions)
            elif encoded & EXPANSION_BIT:
                kind = EXPANSION
                expanded_file_id = encoded >> EXPANSION_SHIFT
                if expanded_file_id >= len(virtual_indices):
                    raise MappingError(f"expanded file {expanded_file_id} is invalid")
            else:
                kind = encoded >> EXPANSION_SHIFT
                if kind == BRANCH:
                    counter = _counter(reader, expressions, "branch true counter")
                    false_counter = _counter(reader, expressions, "branch false counter")
                elif kind == MCDC_BRANCH:
                    counter = _counter(reader, expressions, "MCDC branch true counter")
                    false_counter = _counter(reader, expressions, "MCDC branch false counter")
                    mcdc = tuple(
                        reader.read_int_max(label, UINT_MAX)
                        for label in ("MCDC condition", "MCDC true", "MCDC false")
                    )
                elif kind == MCDC_DECISION:
                    mcdc = tuple(
                        reader.read_int_max(label, UINT_MAX)
                        for label in ("MCDC bitmap index", "MCDC condition count")
                    )
                elif kind not in (CODE, SKIPPED):
                    raise MappingError(f"invalid region kind {kind}")

            line_start_delta = reader.read_int_max(
                f"file {inferred_file_id} region {region_index} line delta", UINT_MAX
            )
            column_start = reader.uleb(f"file {inferred_file_id} region {region_index} start column")
            if column_start > UINT_MAX:
                raise MappingError("start column exceeds uint32")
            num_lines = reader.read_int_max(
                f"file {inferred_file_id} region {region_index} line count", UINT_MAX
            )
            column_end = reader.read_int_max(
                f"file {inferred_file_id} region {region_index} end column", UINT_MAX
            )
            if line_start > UINT_MAX - line_start_delta:
                raise MappingError("source line start exceeds uint32")
            line_start += line_start_delta
            if column_end & (1 << 31):
                kind = GAP
                column_end &= ~(1 << 31)
            if column_start == 0 and column_end == 0:
                column_start, column_end = 1, UINT_MAX
            if line_start > UINT_MAX - num_lines:
                raise MappingError("source line end exceeds uint32")
            line_end = line_start + num_lines
            if line_end < line_start or (line_start == line_end and column_start > column_end):
                raise MappingError("region source range is descending")
            regions.append(Region(
                file_id=inferred_file_id,
                expanded_file_id=expanded_file_id,
                line_start=line_start,
                column_start=column_start,
                line_end=line_end,
                column_end=column_end,
                kind=kind,
                counter=counter,
                false_counter=false_counter,
                mcdc=mcdc,
            ))

    if reader.remaining() != 0:
        raise MappingError(f"trailing mapping bytes: {reader.remaining()}")
    return {
        "virtual_filename_indices": virtual_indices,
        "virtual_paths": virtual_paths,
        "expressions": expressions,
        "regions": regions,
    }


def _decode_counter_value(encoded: int, expressions: list[tuple[Counter, Counter]]) -> Counter:
    tag = encoded & COUNTER_TAG_MASK
    if tag == 0:
        return Counter("zero", 0)
    if tag == 1:
        return Counter("counter", encoded >> 2)
    expression_id = encoded >> 2
    if expression_id >= len(expressions):
        raise MappingError(f"counter expression {expression_id} is invalid")
    if tag == 2:
        return Counter("expression-subtract", expression_id)
    if tag == 3:
        return Counter("expression-add", expression_id)
    raise MappingError(f"invalid counter tag {tag}")


def has_code_witness(decoded: dict, path: str, interval: tuple[int, int, int, int]) -> bool:
    """Return whether a decoded used map has the exact dummy code-region witness."""
    for region in decoded["regions"]:
        if region.kind != CODE:
            continue
        if decoded["virtual_paths"][region.file_id] == path and region.interval == interval:
            return True
    return False
