#!/usr/bin/env python3
"""Bounded, stdlib-only ELF64 COVFUN identity inspector.

This private tool reads an exact binary inventory from the frozen coverage
report and the complete hash-mismatch stderr inventory. It never invokes an
external command, follows symlinks, writes inputs, or merges profiles. It
hashes and parses the five named ELF objects, then reports every matching
``__covrec_<NameHash>`` and ``__covrec_<NameHash>u`` record for every warned
name. ``--report-binary-root`` optionally names the report's original path
namespace while ``--binary-root`` names the physical archive directory; the
default keeps both namespaces identical. ``--self-check`` exercises the
NameHash and COVFUN/dummy parsers using only synthetic bytes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import sys
import tempfile
from typing import Any
import zlib


MAX_REPORT_BYTES = 32 * 1024 * 1024
MAX_WARNING_BYTES = 128 * 1024 * 1024
MAX_ELF_BYTES = 512 * 1024 * 1024
MAX_SECTION_BYTES = 256 * 1024 * 1024
MAX_MAPPING_BYTES = 4 * 1024 * 1024
MAX_WARNINGS = 100_000
MAX_SECTION_COUNT = 100_000
MAX_SYMBOL_COUNT = 2_000_000
MAX_JSON_OUTPUT_BYTES = 128 * 1024 * 1024

EXPECTED_OBJECTS = (
    "libslic3r",
    "fff_print",
    "sla_print",
    "libnest2d",
    "filament_group",
)
ELF_HEADER = struct.Struct("<16sHHIQQQIHHHHHH")
ELF_SECTION = struct.Struct("<IIQQQQIIQQ")
ELF_SYMBOL = struct.Struct("<IBBHQQ")
COVFUN_HEADER = struct.Struct("<QIQQ")
EXPECTED_RELATIVE_PATHS = {
    "libslic3r": Path("libslic3r/libslic3r_tests"),
    "fff_print": Path("fff_print/fff_print_tests"),
    "sla_print": Path("sla_print/sla_print_tests"),
    "libnest2d": Path("libnest2d/libnest2d_tests"),
    "filament_group": Path("filament_group/filament_group_tests"),
}
HASH_MISMATCH_RE = re.compile(
    r"^hash-mismatch: No profile record found for '([^'\n]{1,4096})' "
    r"with hash = 0x([0-9A-Fa-f]{1,16})$"
)
WARNING_COUNT_RE = re.compile(r"^warning:\s+(\d+) functions have mismatched data$")


class InspectorError(RuntimeError):
    pass


def fail(message: str) -> None:
    raise InspectorError(message)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def read_bounded(path: Path, limit: int, label: str) -> bytes:
    """Read one regular, non-symlink file without a path-swap race."""
    try:
        before = path.stat()
    except OSError as exc:
        fail(f"cannot stat {label}: {path}: {exc}")
    if path.is_symlink() or not path.is_file() or not os.path.isfile(path):
        fail(f"{label} is not a regular non-symlink file: {path}")
    if before.st_size < 0 or before.st_size > limit:
        fail(f"{label} exceeds bounded size {limit}: {path}")
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0)
    try:
        fd = os.open(path, flags)
    except OSError as exc:
        fail(f"cannot open {label}: {path}: {exc}")
    try:
        after = os.fstat(fd)
        if not os.path.isfile(path):
            fail(f"{label} changed from regular file: {path}")
        if after.st_size != before.st_size or after.st_size > limit:
            fail(f"{label} changed size during read: {path}")
        with os.fdopen(fd, "rb", closefd=True) as stream:
            data = stream.read(limit + 1)
        if len(data) != after.st_size:
            fail(f"short read for {label}: {path}")
        return data
    except Exception:
        try:
            os.close(fd)
        except OSError:
            pass
        raise


def read_json(path: Path, limit: int, label: str) -> dict[str, Any]:
    data = read_bounded(path, limit, label)

    def no_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                fail(f"duplicate JSON key {key!r} in {label}")
            result[key] = value
        return result

    try:
        value = json.loads(data.decode("utf-8"), object_pairs_hook=no_duplicate_pairs)
    except (UnicodeDecodeError, json.JSONDecodeError, RecursionError, MemoryError) as exc:
        fail(f"invalid JSON {label}: {exc}")
    if not isinstance(value, dict):
        fail(f"JSON root is not an object: {label}")
    return value


def name_hash(name: str) -> int:
    try:
        digest = hashlib.md5(name.encode("utf-8"), usedforsecurity=False).digest()
    except TypeError:  # Python versions before the keyword was added.
        digest = hashlib.md5(name.encode("utf-8")).digest()
    return int.from_bytes(digest[:8], "little")


def uleb(data: bytes, cursor: int, label: str, max_value: int | None = None) -> tuple[int, int]:
    value = 0
    shift = 0
    for index in range(10):
        if cursor >= len(data):
            fail(f"truncated ULEB in {label}")
        byte = data[cursor]
        cursor += 1
        if index == 9 and (byte & 0x7f) > 1:
            fail(f"ULEB overflow in {label}")
        value |= (byte & 0x7f) << shift
        if not byte & 0x80:
            if max_value is not None and value > max_value:
                fail(f"ULEB exceeds limit in {label}")
            return value, cursor
        shift += 7
    fail(f"unterminated ULEB in {label}")


def enc_uleb(value: int) -> bytes:
    if value < 0:
        fail(f"cannot encode negative ULEB: {value}")
    encoded = bytearray()
    while True:
        byte = value & 0x7F
        value >>= 7
        if value:
            encoded.append(byte | 0x80)
        else:
            encoded.append(byte)
            return bytes(encoded)


def hash_bytes(data: bytes) -> int:
    try:
        digest = hashlib.md5(data, usedforsecurity=False).digest()
    except TypeError:  # Python versions before the keyword was added.
        digest = hashlib.md5(data).digest()
    return int.from_bytes(digest[:8], "little")


def read_string(data: bytes, cursor: int, label: str) -> tuple[str, int]:
    length, cursor = uleb(data, cursor, f"{label} length", MAX_SECTION_BYTES)
    end = cursor + length
    if end > len(data):
        fail(f"truncated {label}")
    raw = data[cursor:end]
    if b"\0" in raw:
        fail(f"NUL in {label}")
    try:
        value = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        fail(f"invalid UTF-8 in {label}: {exc}")
    return value, end


def normalize_covmap_path(value: str, cwd: str) -> str:
    if not value:
        return value
    if os.path.isabs(value):
        return os.path.normpath(value).replace("\\", "/")
    if cwd:
        return os.path.normpath(os.path.join(cwd, value)).replace("\\", "/")
    return os.path.normpath(value).replace("\\", "/")


def parse_covmap_filenames(region: bytes, version: int, label: str) -> list[str]:
    """Decode one LLVM 18 RawCoverageFilenamesReader payload."""
    count, cursor = uleb(region, 0, f"{label} filename count", MAX_SECTION_BYTES)
    if count == 0 or count > MAX_SECTION_COUNT:
        fail(f"{label} has invalid filename count: {count}")
    if version >= 3:
        uncompressed_size, cursor = uleb(
            region, cursor, f"{label} uncompressed size", MAX_SECTION_BYTES
        )
        compressed_size, cursor = uleb(
            region, cursor, f"{label} compressed size", MAX_SECTION_BYTES
        )
        if compressed_size:
            end = cursor + compressed_size
            if end > len(region):
                fail(f"truncated compressed {label}")
            compressed = region[cursor:end]
            try:
                decoder = zlib.decompressobj()
                decoded = decoder.decompress(compressed, uncompressed_size + 1)
                if decoder.unconsumed_tail:
                    fail(f"{label} decompressed output exceeds declared size")
                # A valid complete stream must reach EOF within the bounded
                # decompress call. flush(length) only sets an initial buffer
                # size, so it must not be used as an output limit here.
                if len(decoded) > uncompressed_size:
                    fail(f"{label} decompressed output exceeds declared size")
                if not decoder.eof:
                    fail(f"truncated compressed {label}")
                if decoder.unused_data:
                    fail(f"trailing compressed data in {label}")
            except zlib.error as exc:
                fail(f"cannot decompress {label}: {exc}")
            if len(decoded) != uncompressed_size:
                fail(
                    f"{label} decompressed length {len(decoded)} != "
                    f"declared {uncompressed_size}"
                )
            payload = decoded
            cursor = end
        else:
            payload = region[cursor:]
            if len(payload) != uncompressed_size:
                fail(
                    f"{label} uncompressed length {len(payload)} != "
                    f"declared {uncompressed_size}"
                )
            cursor = len(region)
    else:
        payload = region[cursor:]
        cursor = len(region)

    names: list[str] = []
    payload_cursor = 0
    cwd = ""
    for index in range(count):
        value, payload_cursor = read_string(payload, payload_cursor, f"{label} filename {index}")
        if version >= 5:
            if index == 0:
                cwd = value
                names.append(value)
            else:
                names.append(normalize_covmap_path(value, cwd))
        else:
            names.append(value)
    if payload_cursor != len(payload):
        fail(f"trailing bytes in {label} filename payload")
    if cursor != len(region):
        fail(f"trailing bytes in {label} filename region")
    return names


def parse_covmap_tables(covmap: bytes) -> dict[int, dict[str, Any]]:
    """Parse LLVM 18 __llvm_covmap headers, keyed by FilenameRef."""
    header = struct.Struct("<IIII")
    cursor = 0
    tables: dict[int, dict[str, Any]] = {}
    while cursor < len(covmap):
        remaining_size = len(covmap) - cursor
        if remaining_size < header.size:
            if any(covmap[cursor:]):
                fail("truncated __llvm_covmap header")
            break
        # A fully zero final padding area is not another header. Checking one
        # fixed-size prefix avoids copying/scanning the whole remaining
        # section for every TU.
        if covmap[cursor:cursor + header.size] == b"\0" * header.size:
            if any(covmap[cursor:]):
                fail("nonzero data follows __llvm_covmap records")
            break
        records, filenames_size, coverage_size, version = header.unpack_from(covmap, cursor)
        cursor += header.size
        if version < 3 or version > 6:
            fail(f"unsupported __llvm_covmap version {version}")
        if records != 0 or coverage_size != 0:
            fail("LLVM 18 __llvm_covmap header has unexpected records/coverage payload")
        end = cursor + filenames_size
        if end > len(covmap):
            fail("truncated __llvm_covmap filename region")
        region = covmap[cursor:end]
        filenames = parse_covmap_filenames(region, version, "__llvm_covmap")
        ref = hash_bytes(region)
        table = {
            "filenames_ref": f"0x{ref:016X}",
            "version": version,
            "filenames": filenames,
            "filename_region_size": len(region),
            "filename_region_sha256": sha256_bytes(region),
        }
        previous = tables.get(ref)
        if previous is not None and previous["filenames"] != filenames:
            fail(f"FilenameRef collision with different TU filename tables: 0x{ref:X}")
        tables[ref] = table
        aligned = (end + 7) & ~7
        if aligned > len(covmap):
            # The final TU record may occupy the exact end of a section; ELF
            # section padding is not required to be included in sh_size.
            if any(covmap[end:]):
                fail("__llvm_covmap alignment exceeds section")
            cursor = end
        else:
            if any(covmap[end:aligned]):
                fail("nonzero __llvm_covmap alignment padding")
            cursor = aligned
    if not tables:
        fail("__llvm_covmap has no filename tables")
    return tables


def mapping_shape(mapping: bytes, label: str,
                  filename_count: int | None = None) -> dict[str, Any]:
    """Mirror LLVM 18's dummy prefix and expose the leading file-index list."""
    cursor = 0
    files, cursor = uleb(mapping, cursor, f"{label} file count", MAX_SECTION_COUNT)
    if files == 0:
        fail(f"{label} has zero file mappings")
    filename_indices: list[int] = []
    for index in range(files):
        file_index, cursor = uleb(
            mapping, cursor, f"{label} filename index {index}", 0xFFFFFFFF
        )
        if filename_count is not None and file_index >= filename_count:
            fail(f"{label} filename index {file_index} is outside its TU table")
        filename_indices.append(file_index)
    file_index: int | None = filename_indices[0] if files == 1 else None
    if files != 1:
        return {
            "file_mappings": files,
            "filename_index": file_index,
            "filename_indices": filename_indices,
            "expressions": None,
            "regions": None,
            "encoded_counter_region": None,
            "counter_tag": None,
            "dummy": False,
            "bytes_consumed": cursor,
            "trailing_bytes": len(mapping) - cursor,
            "dummy_predicate_short_circuit": "file_mappings",
        }
    expressions, cursor = uleb(mapping, cursor, f"{label} expression count", MAX_SECTION_COUNT)
    if expressions != 0:
        return {
            "file_mappings": files,
            "filename_index": file_index,
            "filename_indices": filename_indices,
            "expressions": expressions,
            "regions": None,
            "encoded_counter_region": None,
            "counter_tag": None,
            "dummy": False,
            "bytes_consumed": cursor,
            "trailing_bytes": len(mapping) - cursor,
            "dummy_predicate_short_circuit": "expressions",
        }
    regions, cursor = uleb(mapping, cursor, f"{label} region count", MAX_SECTION_COUNT)
    if regions != 1:
        return {
            "file_mappings": files,
            "filename_index": file_index,
            "filename_indices": filename_indices,
            "expressions": expressions,
            "regions": regions,
            "encoded_counter_region": None,
            "counter_tag": None,
            "dummy": False,
            "bytes_consumed": cursor,
            "trailing_bytes": len(mapping) - cursor,
            "dummy_predicate_short_circuit": "regions",
        }
    encoded, cursor = uleb(mapping, cursor, f"{label} counter/region", 0xFFFFFFFF)
    tag = encoded & 0x3
    shape = {
        "file_mappings": files,
        "filename_index": file_index,
        "filename_indices": filename_indices,
        "expressions": expressions,
        "regions": regions,
        "encoded_counter_region": encoded,
        "counter_tag": tag,
        "dummy": False,
        "bytes_consumed": cursor,
        "trailing_bytes": len(mapping) - cursor,
        "dummy_predicate_short_circuit": None,
        "mapping_validation": "counter-prefix-only",
    }
    # LLVM stores line-start delta, start column, number-of-lines, and end
    # column. A dummy has one inferred file and one region, so its absolute
    # start line is the delta from zero and its end line is start + num-lines.
    raw_coordinates: list[int] = []
    for index in range(4):
        value, cursor = uleb(
            mapping, cursor, f"{label} region coordinate {index}", 0xFFFFFFFF
        )
        raw_coordinates.append(value)
    line_start_delta, column_start, num_lines, column_end = raw_coordinates
    gap_region = bool(column_end & (1 << 31))
    if gap_region:
        column_end &= ~(1 << 31)
    if column_start == 0 and column_end == 0:
        column_start, column_end = 1, 0xFFFFFFFF
    line_start = line_start_delta
    line_end = line_start + num_lines
    if line_end < line_start or (
        line_start == line_end and column_start > column_end
    ):
        fail(f"invalid absolute region interval in {label}")
    if cursor != len(mapping):
        fail(f"unexpected trailing bytes in complete one-region mapping for {label}")
    shape.update(
        region_coordinates=[line_start, column_start, line_end, column_end],
        absolute_region={
            "line_start": line_start,
            "column_start": column_start,
            "line_end": line_end,
            "column_end": column_end,
        },
        raw_region_coordinates=raw_coordinates,
        gap_region=gap_region,
        bytes_consumed=cursor,
        trailing_bytes=0,
        mapping_validation="one-region-complete",
        dummy=(tag == 0),
    )
    return shape

def covfun_record(
    covfun: bytes,
    offset: int,
    expected_name_hash: int,
    symbol_name: str,
    symbol_size: int,
    filename_tables: dict[int, dict[str, Any]],
) -> dict[str, Any]:
    if offset < 0 or offset + COVFUN_HEADER.size > len(covfun):
        fail(f"truncated COVFUN header for {symbol_name}")
    name_ref, data_size, function_hash, filenames_ref = COVFUN_HEADER.unpack_from(covfun, offset)
    if name_ref != expected_name_hash:
        fail(
            f"NameRef mismatch for {symbol_name}: "
            f"expected 0x{expected_name_hash:016X}, got 0x{name_ref:016X}"
        )
    if data_size <= 0 or data_size > MAX_MAPPING_BYTES:
        fail(f"unsupported mapping size for {symbol_name}: {data_size}")
    mapping_start = offset + COVFUN_HEADER.size
    record_end = mapping_start + data_size
    if record_end > len(covfun):
        fail(f"truncated COVFUN mapping for {symbol_name}")
    filename_table = filename_tables.get(filenames_ref)
    if filename_table is None:
        fail(
            f"COVFUN FilenameRef has no __llvm_covmap table for {symbol_name}: "
            f"0x{filenames_ref:016X}"
        )
    record_size = record_end - offset
    if symbol_size and record_size > symbol_size:
        fail(
            f"COVFUN record exceeds ELF symbol size for {symbol_name}: "
            f"record={record_size}, symbol={symbol_size}"
        )
    mapping = covfun[mapping_start:record_end]
    shape = mapping_shape(
        mapping,
        symbol_name,
        filename_count=len(filename_table["filenames"]),
    )
    filename_indices = shape["filename_indices"]
    filename_paths = [filename_table["filenames"][index] for index in filename_indices]
    record = {
        "symbol": symbol_name,
        "used_suffix": symbol_name.endswith("u"),
        "record_offset": offset,
        "record_size": record_size,
        "symbol_size": symbol_size,
        "name_ref": f"0x{name_ref:016X}",
        "function_hash": function_hash,
        "function_hash_hex": f"0x{function_hash:016X}",
        "filenames_ref": f"0x{filenames_ref:016X}",
        "filename_paths": filename_paths,
        "filename_table_sha256": filename_table["filename_region_sha256"],
        "mapping_size": data_size,
        "mapping_sha256": sha256_bytes(mapping),
        "mapping_hex": mapping.hex(),
        "mapping_shape": shape,
        "dummy": bool(function_hash == 0 and shape["dummy"]),
    }
    if record["dummy"]:
        record["dummy_region"] = shape["absolute_region"]
    return record


def cstring(data: bytes, offset: int, label: str) -> bytes:
    if offset < 0 or offset >= len(data):
        fail(f"string offset outside section for {label}")
    end = data.find(b"\0", offset)
    if end < 0:
        fail(f"unterminated string for {label}")
    return data[offset:end]


def section_bytes(image: bytes, section: dict[str, Any], label: str) -> bytes:
    offset = section["offset"]
    size = section["size"]
    if size > MAX_SECTION_BYTES:
        fail(f"{label} section exceeds bounded size: {size}")
    if offset > len(image) or size > len(image) - offset:
        fail(f"{label} section exceeds ELF file")
    return image[offset:offset + size]


def parse_elf_symbols(image: bytes, targets: set[str]) -> tuple[dict[str, Any], dict[str, list[dict[str, Any]]]]:
    if len(image) < ELF_HEADER.size:
        fail("ELF header is truncated")
    ident, e_type, e_machine, e_version, e_entry, e_phoff, e_shoff, e_flags, e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx = ELF_HEADER.unpack_from(image)
    if ident[:4] != b"\x7fELF" or ident[4] != 2 or ident[5] != 1 or ident[6] != 1:
        fail("unsupported ELF: expected ELF64 little-endian current-version object")
    if e_ehsize != ELF_HEADER.size or e_shentsize != ELF_SECTION.size:
        fail("unsupported ELF header/section-header size")
    if e_shnum <= 0 or e_shnum > MAX_SECTION_COUNT or e_shstrndx >= e_shnum:
        fail("unsupported ELF section count/string-table index")
    section_table_size = e_shnum * e_shentsize
    if e_shoff > len(image) or section_table_size > len(image) - e_shoff:
        fail("truncated ELF section table")
    raw_sections: list[dict[str, Any]] = []
    for index in range(e_shnum):
        fields = ELF_SECTION.unpack_from(image, e_shoff + index * e_shentsize)
        sh_name, sh_type, sh_flags, sh_addr, sh_offset, sh_size, sh_link, sh_info, sh_addralign, sh_entsize = fields
        raw_sections.append({
            "index": index,
            "name_offset": sh_name,
            "type": sh_type,
            "flags": sh_flags,
            "addr": sh_addr,
            "offset": sh_offset,
            "size": sh_size,
            "link": sh_link,
            "info": sh_info,
            "align": sh_addralign,
            "entsize": sh_entsize,
        })
    shstr = raw_sections[e_shstrndx]
    shstr_data = section_bytes(image, shstr, "section-name string table")
    for section in raw_sections:
        name = cstring(shstr_data, section["name_offset"], "section name") if section["name_offset"] else b""
        try:
            section["name"] = name.decode("ascii")
        except UnicodeDecodeError:
            section["name"] = ""
    covfun_candidates = [s for s in raw_sections if s["name"] in {"__llvm_covfun", ".llvm_covfun"}]
    if len(covfun_candidates) != 1:
        fail(f"expected exactly one LLVM COVFUN section, found {len(covfun_candidates)}")
    covmap_candidates = [s for s in raw_sections if s["name"] in {"__llvm_covmap", ".llvm_covmap"}]
    if len(covmap_candidates) != 1:
        fail(f"expected exactly one LLVM COVMAP section, found {len(covmap_candidates)}")
    covfun = covfun_candidates[0]
    covfun_data = section_bytes(image, covfun, "COVFUN")
    covmap = covmap_candidates[0]
    covmap_data = section_bytes(image, covmap, "COVMAP")
    filename_tables = parse_covmap_tables(covmap_data)
    target_bytes = {target.encode("ascii") for target in targets}
    symbol_records: dict[str, list[dict[str, Any]]] = {target: [] for target in targets}
    seen_symbols: set[tuple[Any, ...]] = set()
    symbol_tables = [s for s in raw_sections if s["type"] in {2, 11}]
    if not symbol_tables:
        fail("ELF has no symbol table")
    for symtab in symbol_tables:
        if symtab["link"] >= len(raw_sections):
            fail("symbol table string-table link is out of range")
        strtab = raw_sections[symtab["link"]]
        str_data = section_bytes(image, strtab, "symbol string table")
        entsize = symtab["entsize"] or ELF_SYMBOL.size
        if entsize != ELF_SYMBOL.size:
            fail(f"unsupported ELF symbol entry size: {entsize}")
        sym_data = section_bytes(image, symtab, "symbol table")
        if len(sym_data) % entsize:
            fail("truncated ELF symbol table entry")
        symbol_count = len(sym_data) // entsize
        if symbol_count > MAX_SYMBOL_COUNT:
            fail("ELF symbol table exceeds bounded entry count")
        for index in range(symbol_count):
            st_name, st_info, st_other, st_shndx, st_value, st_size = ELF_SYMBOL.unpack_from(sym_data, index * entsize)
            raw_name = cstring(str_data, st_name, "symbol name") if st_name else b""
            if raw_name not in target_bytes:
                continue
            name = raw_name.decode("ascii")
            if st_shndx >= len(raw_sections) or st_shndx == 0:
                fail(f"target symbol has unsupported section index: {name}")
            section = raw_sections[st_shndx]
            if section["index"] != covfun["index"]:
                fail(f"target symbol is not in COVFUN section: {name}")
            identity = (name, st_shndx, st_value, st_size)
            if identity in seen_symbols:
                continue
            seen_symbols.add(identity)
            if st_value < covfun["addr"]:
                fail(f"target symbol address precedes COVFUN section: {name}")
            offset = st_value - covfun["addr"]
            record = covfun_record(
                covfun_data,
                offset,
                name_hash_from_symbol(name),
                name,
                st_size,
                filename_tables,
            )
            record["symbol_table_section"] = symtab["name"]
            record["section_name"] = covfun["name"]
            symbol_records[name].append(record)
    for name, records in symbol_records.items():
        if len(records) > 1:
            fail(f"target symbol collision: {name} has {len(records)} distinct records")
    header = {
        "elf_class": "ELF64",
        "endianness": "little",
        "machine": e_machine,
        "type": e_type,
        "section_count": e_shnum,
        "covfun_section": covfun["name"],
        "covfun_section_index": covfun["index"],
        "covfun_section_size": covfun["size"],
        "covmap_section": covmap["name"],
        "covmap_section_index": covmap["index"],
        "covmap_section_size": covmap["size"],
        "covmap_filename_tables": {
            f"0x{ref:016X}": table for ref, table in filename_tables.items()
        },
    }
    return header, symbol_records


def name_hash_from_symbol(symbol: str) -> int:
    # llvm::utohexstr emits no leading zero padding.
    match = re.fullmatch(r"__covrec_([0-9A-Fa-f]{1,16})(u)?", symbol, re.IGNORECASE)
    if not match:
        fail(f"unsupported COVFUN symbol name: {symbol}")
    return int(match.group(1), 16)


def parse_warnings(path: Path) -> dict[str, Any]:
    data = read_bounded(path, MAX_WARNING_BYTES, "warning stderr")
    try:
        text = data.decode("utf-8")
    except UnicodeDecodeError as exc:
        fail(f"warning stderr is not UTF-8: {exc}")
    records: list[dict[str, Any]] = []
    count_headers: list[int] = []
    for line_number, line in enumerate(text.splitlines(), 1):
        count_match = WARNING_COUNT_RE.match(line)
        if count_match:
            count_headers.append(int(count_match.group(1)))
        if "hash-mismatch:" in line:
            match = HASH_MISMATCH_RE.match(line)
            if not match:
                fail(f"unparseable hash-mismatch line {line_number}")
            name = match.group(1)
            mapping_hash = int(match.group(2), 16)
            records.append({
                "warning_index": len(records),
                "stderr_line": line_number,
                "name": name,
                "mapping_hash": mapping_hash,
                "mapping_hash_hex": f"0x{mapping_hash:X}",
                "name_hash": name_hash(name),
                "name_hash_hex": f"0x{name_hash(name):016X}",
            })
            if len(records) > MAX_WARNINGS:
                fail("warning inventory exceeds bounded count")
    if len(count_headers) != 1:
        fail(f"expected exactly one mismatch count header, found {len(count_headers)}")
    if count_headers[0] != len(records):
        fail(f"warning count {count_headers[0]} differs from parsed records {len(records)}")
    if not records:
        fail("warning stderr contains no hash-mismatch records")
    names_by_hash: dict[int, str] = {}
    for record in records:
        previous = names_by_hash.setdefault(record["name_hash"], record["name"])
        if previous != record["name"]:
            fail(f"NameHash collision between warnings: {previous} and {record['name']}")
    unique_by_key: dict[tuple[str, int], dict[str, Any]] = {}
    unique_records: list[dict[str, Any]] = []
    for record in records:
        key = (record["name"], record["mapping_hash"])
        unique = unique_by_key.get(key)
        if unique is None:
            unique = {
                "name": record["name"],
                "mapping_hash": record["mapping_hash"],
                "mapping_hash_hex": record["mapping_hash_hex"],
                "name_hash": record["name_hash"],
                "name_hash_hex": record["name_hash_hex"],
                "repeat_count": 0,
                "warning_indices": [],
                "stderr_lines": [],
            }
            unique_by_key[key] = unique
            unique_records.append(unique)
        unique["repeat_count"] += 1
        unique["warning_indices"].append(record["warning_index"])
        unique["stderr_lines"].append(record["stderr_line"])
    return {
        "path": str(path),
        "sha256": sha256_bytes(data),
        "size_bytes": len(data),
        "count": len(records),
        "unique_count": len(unique_records),
        "records": records,
        "unique_records": unique_records,
    }


def binary_inventory(
    report: dict[str, Any],
    binary_root: Path,
    report_binary_root: Path | None = None,
) -> list[dict[str, Any]]:
    if report.get("schema") != 3 or report.get("status") != "pass":
        fail("report must be schema 3 with status pass")
    before = report.get("binary_sha256_before")
    after = report.get("binary_sha256_after")
    if not isinstance(before, dict) or not isinstance(after, dict):
        fail("report must contain binary_sha256_before/after dictionaries")
    if before != after:
        fail("binary_sha256_before and binary_sha256_after differ")
    if binary_root.is_symlink() or not binary_root.is_dir() or not binary_root.is_absolute():
        fail(f"binary root is not an absolute regular directory: {binary_root}")
    if report_binary_root is None:
        report_binary_root = binary_root
    elif report_binary_root.is_symlink() or not report_binary_root.is_absolute():
        fail(f"report binary root is not an absolute non-symlink namespace: {report_binary_root}")
    expected_report_paths = {
        str(report_binary_root / EXPECTED_RELATIVE_PATHS[name]): name
        for name in EXPECTED_OBJECTS
    }
    if set(before) != set(expected_report_paths):
        fail("report binary hash keys differ from the required five paths")
    result: list[dict[str, Any]] = []
    for name in EXPECTED_OBJECTS:
        source_path = report_binary_root / EXPECTED_RELATIVE_PATHS[name]
        path = binary_root / EXPECTED_RELATIVE_PATHS[name]
        expected_sha = before.get(str(source_path))
        if not isinstance(expected_sha, str) or not re.fullmatch(r"[0-9A-Fa-f]{64}", expected_sha):
            fail(f"invalid expected SHA-256 for {name}")
        image = read_bounded(path, MAX_ELF_BYTES, f"ELF {name}")
        actual_sha = sha256_bytes(image)
        if actual_sha != expected_sha.lower():
            fail(f"SHA-256 mismatch for {name}: expected {expected_sha}, got {actual_sha}")
        result.append({
            "name": name,
            "source_path": str(source_path),
            "path": str(path),
            "sha256": actual_sha,
            "size_bytes": len(image),
        })
    return result


def inspect(
    report_path: Path,
    warnings_path: Path,
    binary_root: Path,
    report_binary_root: Path | None = None,
) -> dict[str, Any]:
    report = read_json(report_path, MAX_REPORT_BYTES, "full report")
    warning_inventory = parse_warnings(warnings_path)
    unique_warnings = warning_inventory["unique_records"]
    warning_names = {record["name"] for record in unique_warnings}
    expected_hashes = {name: name_hash(name) for name in warning_names}
    binaries = binary_inventory(report, binary_root, report_binary_root)
    output_binaries: list[dict[str, Any]] = []
    for binary in binaries:
        image = read_bounded(Path(binary["path"]), MAX_ELF_BYTES, f"ELF {binary['name']}")
        if sha256_bytes(image) != binary["sha256"]:
            fail(f"ELF changed between verification and parse: {binary['name']}")
        targets = {f"__covrec_{value:X}" for value in expected_hashes.values()}
        targets |= {f"{symbol}u" for symbol in targets}
        elf_header, symbol_records = parse_elf_symbols(image, targets)
        del image
        by_warning: list[dict[str, Any]] = []
        for warning in unique_warnings:
            base = f"__covrec_{warning['name_hash']:X}"
            records = list(symbol_records.get(base, [])) + list(symbol_records.get(base + "u", []))
            by_warning.append({
                "name": warning["name"],
                "mapping_hash": warning["mapping_hash_hex"],
                "name_hash": warning["name_hash_hex"],
                "repeat_count": warning["repeat_count"],
                "warning_indices": warning["warning_indices"],
                "stderr_lines": warning["stderr_lines"],
                "records": records,
                "record_count": len(records),
            })
        output_binaries.append({
            "name": binary["name"],
            "source_path": binary["source_path"],
            "path": binary["path"],
            "sha256": binary["sha256"],
            "size_bytes": binary["size_bytes"],
            "elf": elf_header,
            "warnings": by_warning,
        })
    return {
        "schema": 2,
        "status": "complete",
        # This inventory is diagnostic metadata only.  Keep the marker at the
        # top level because the replay controller binds this field before it
        # inspects the warning and binary inventories.
        "coverage_claim": False,
        "native_execution": False,
        "warning_inventory": warning_inventory,
        "report": {
            "path": str(report_path),
            "sha256": sha256_bytes(read_bounded(report_path, MAX_REPORT_BYTES, "full report")),
        },
        "binaries": output_binaries,
        "interpretation": {
            "dummy_rule": "FuncHash==0 and LLVM RawCoverageMappingDummyChecker shape",
            "used_suffix_rule": "symbol ends with u",
            "coverage_waiver": False,
            "coverage_claim": False,
        },
    }


def write_json(path: Path, value: object) -> None:
    if path.exists() or path.is_symlink():
        fail(f"output already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(value, indent=2, sort_keys=True) + "\n").encode("utf-8")
    if len(encoded) > MAX_JSON_OUTPUT_BYTES:
        fail("JSON output exceeds bounded size")
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", dir=path.parent)
    try:
        with os.fdopen(fd, "wb", closefd=True) as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    except Exception:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def self_check() -> None:
    known = {
        "_ZN6Slic3r10MultiPoint3endEv": 0x63ADC68F55E5A0AE,
        "_ZNK6Slic3r10MultiPoint6cbeginEv": 0xF9E0BBB80FE6FA3E,
        "_ZNK6Slic3r10MultiPoint4cendEv": 0x5B08B2B498A1AA4B,
    }
    for text, expected in known.items():
        assert name_hash(text) == expected, text
    assert name_hash_from_symbol("__covrec_1") == 1
    assert name_hash_from_symbol("__covrec_1u") == 1
    dummy_mapping = bytes.fromhex("01000001000a01000c")
    used_mapping = bytes.fromhex("01050001017020003a")
    assert mapping_shape(dummy_mapping, "mock dummy")["dummy"]
    assert not mapping_shape(used_mapping, "mock used")["dummy"]
    filename_payload = enc_uleb(len(b"/tmp/mock.cpp")) + b"/tmp/mock.cpp"
    filename_region = (
        enc_uleb(1)
        + enc_uleb(len(filename_payload))
        + enc_uleb(0)
        + filename_payload
    )
    filename_ref = hash_bytes(filename_region)
    filename_tables = parse_covmap_tables(
        struct.pack("<IIII", 0, len(filename_region), 0, 3)
        + filename_region
    )
    mock = COVFUN_HEADER.pack(
        known[next(iter(known))], len(dummy_mapping), 0, filename_ref
    ) + dummy_mapping
    parsed = covfun_record(
        mock,
        0,
        known[next(iter(known))],
        "__covrec_63ADC68F55E5A0AE",
        0,
        filename_tables,
    )
    assert parsed["dummy"] and parsed["function_hash"] == 0
    # Build one tiny ELF64 image in memory so section, symbol, address, and
    # COVFUN offset handling are exercised without opening an input file.
    symbol_name = "__covrec_63ADC68F55E5A0AE"
    shstr = b"\0.shstrtab\0.strtab\0.symtab\0__llvm_covfun\0__llvm_covmap\0"
    shstr_offsets = {
        name: shstr.index(name.encode("ascii"))
        for name in (".shstrtab", ".strtab", ".symtab", "__llvm_covfun", "__llvm_covmap")
    }
    strtab = b"\0" + symbol_name.encode("ascii") + b"\0"
    symbol_name_offset = 1
    covfun_addr = 0x400000
    covfun_blob = mock
    image = bytearray(64)

    def append_aligned(blob: bytes, alignment: int = 8) -> int:
        while len(image) % alignment:
            image.append(0)
        offset = len(image)
        image.extend(blob)
        return offset

    shstr_offset = append_aligned(shstr)
    strtab_offset = append_aligned(strtab)
    covfun_offset = append_aligned(covfun_blob)
    covmap_blob = struct.pack("<IIII", 0, len(filename_region), 0, 3) + filename_region
    covmap_offset = append_aligned(covmap_blob)
    symtab_offset = append_aligned(
        ELF_SYMBOL.pack(0, 0, 0, 0, 0, 0)
        + ELF_SYMBOL.pack(symbol_name_offset, 0x10, 0, 4, covfun_addr, len(covfun_blob))
    )
    while len(image) % 8:
        image.append(0)
    section_table_offset = len(image)
    sections = [
        ELF_SECTION.pack(0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
        ELF_SECTION.pack(shstr_offsets[".shstrtab"], 3, 0, 0, shstr_offset, len(shstr), 0, 0, 1, 0),
        ELF_SECTION.pack(shstr_offsets[".strtab"], 3, 0, 0, strtab_offset, len(strtab), 0, 0, 1, 0),
        ELF_SECTION.pack(shstr_offsets[".symtab"], 2, 0, 0, symtab_offset, ELF_SYMBOL.size * 2, 2, 1, 8, ELF_SYMBOL.size),
        ELF_SECTION.pack(shstr_offsets["__llvm_covfun"], 1, 0, covfun_addr, covfun_offset, len(covfun_blob), 0, 0, 8, 0),
        ELF_SECTION.pack(shstr_offsets["__llvm_covmap"], 1, 0, 0x500000, covmap_offset, len(covmap_blob), 0, 0, 8, 0),
    ]
    image.extend(b"".join(sections))
    image[:ELF_HEADER.size] = ELF_HEADER.pack(
        b"\x7fELF" + bytes((2, 1, 1, 0)) + bytes(8),
        2, 62, 1, 0, 0, section_table_offset, 0,
        ELF_HEADER.size, 0, 0, ELF_SECTION.size, len(sections), 1,
    )
    elf_header, symbols = parse_elf_symbols(bytes(image), {symbol_name})
    assert elf_header["elf_class"] == "ELF64"
    assert len(symbols[symbol_name]) == 1 and symbols[symbol_name][0]["dummy"]
    # Exercise the current schema-3 hash dictionaries and the warning
    # occurrence/unique-pair split without opening any native artifact.
    with tempfile.TemporaryDirectory(prefix="elf-covfun-self-check-") as temporary:
        root = Path(temporary)
        for relative in EXPECTED_RELATIVE_PATHS.values():
            path = root / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(bytes(image))
        hashes = {
            str(root / relative): sha256_bytes(bytes(image))
            for relative in EXPECTED_RELATIVE_PATHS.values()
        }
        mock_report = {
            "schema": 3,
            "status": "pass",
            "binary_sha256_before": hashes,
            "binary_sha256_after": dict(hashes),
        }
        checked = binary_inventory(mock_report, root)
        assert len(checked) == len(EXPECTED_OBJECTS)
        warning_path = root / "warnings.stderr"
        warning_path.write_text(
            "warning: 2 functions have mismatched data\n"
            "hash-mismatch: No profile record found for "
            "'_ZN6Slic3r10MultiPoint3endEv' with hash = 0x0\n"
            "hash-mismatch: No profile record found for "
            "'_ZN6Slic3r10MultiPoint3endEv' with hash = 0x0\n",
            encoding="utf-8",
        )
        report_path = root / "report.json"
        report_path.write_text(json.dumps(mock_report), encoding="utf-8")
        warning_data = parse_warnings(warning_path)
        assert warning_data["count"] == 2 and warning_data["unique_count"] == 1
        assert warning_data["unique_records"][0]["repeat_count"] == 2
        inspected = inspect(report_path, warning_path, root)
        assert inspected["coverage_claim"] is False
        assert inspected["status"] == "complete"
        assert inspected["warning_inventory"]["unique_count"] == 1
        assert all(len(binary["warnings"]) == 1 for binary in inspected["binaries"])
    try:
        mapping_shape(b"\x80", "truncated")
    except InspectorError:
        pass
    else:
        fail("self-check accepted truncated ULEB")
    print(json.dumps({"status": "self-check-pass", "known_name_hashes": len(known),
                      "coverage_claim": False}))


def parser() -> argparse.ArgumentParser:
    value = argparse.ArgumentParser(description=__doc__)
    value.add_argument("--report", type=Path)
    value.add_argument("--warnings", type=Path)
    value.add_argument("--binary-root", type=Path)
    value.add_argument(
        "--report-binary-root",
        type=Path,
        help="absolute report-path namespace; defaults to --binary-root",
    )
    # Kept as a spelling-compatible alias for the private v1 invocation; it
    # now means the directory containing the five known test subdirectories,
    # not an archive with a fabricated inventory layout.
    value.add_argument("--archive-root", dest="binary_root_alias", type=Path)
    value.add_argument("--output", type=Path)
    value.add_argument("--self-check", action="store_true")
    return value


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.self_check:
            if any(value is not None for value in (args.report, args.warnings, args.binary_root, args.binary_root_alias, args.report_binary_root, args.output)):
                fail("--self-check cannot be combined with input/output paths")
            self_check()
            return 0
        if args.report is None or args.warnings is None or args.output is None:
            fail("normal mode requires --report, --warnings, --output, and --binary-root")
        if args.binary_root is not None and args.binary_root_alias is not None:
            fail("use only one of --binary-root and --archive-root")
        binary_root = args.binary_root or args.binary_root_alias
        if binary_root is None:
            fail("normal mode requires --binary-root")
        result = inspect(args.report, args.warnings, binary_root, args.report_binary_root)
        write_json(args.output, result)
        print(json.dumps({"status": "complete", "output": str(args.output)}))
        return 0
    except InspectorError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
