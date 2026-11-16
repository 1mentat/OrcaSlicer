#!/usr/bin/env python3
"""Pre-extract Catch2 test names and tags for CatchAddTests.cmake."""

import json
import sys
from pathlib import Path


def bracket_argument(value):
    """Quote one CMake argument without letting its contents terminate it."""
    equals = ""
    while f"]{equals}]" in value:
        equals += "="
    return f"[{equals}[{value}]{equals}]"


def records_from_listing(path):
    listing = json.loads(path.read_text(encoding="utf-8"))
    version = listing.get("version") if isinstance(listing, dict) else None
    if not ((type(version) is int and version == 1) or version == "1"):
        raise ValueError(f"unsupported Catch output version: {version!r}")

    try:
        records = listing["listings"]["tests"]
    except (KeyError, TypeError) as exc:
        raise ValueError("Catch listing is missing listings.tests") from exc
    if not isinstance(records, list):
        raise ValueError("Catch listings.tests must be an array")

    output = [f"set(_CATCH_PREPROCESSED_TEST_COUNT {len(records)})\n"]
    for index, record in enumerate(records):
        if not isinstance(record, dict):
            raise ValueError(f"Catch test record {index} must be an object")
        name = record.get("name")
        tags = record.get("tags")
        if not isinstance(name, str):
            raise ValueError(f"Catch test record {index} has no string name")
        if not isinstance(tags, list) or not all(isinstance(tag, str) for tag in tags):
            raise ValueError(f"Catch test record {index} has invalid tags")
        compact_record = json.dumps(
            {"name": name, "tags": tags}, ensure_ascii=True, separators=(",", ":")
        )
        output.append(f"set(_CATCH_TEST_RECORD_{index} {bracket_argument(compact_record)})\n")
    return output


def main() -> int:
    if len(sys.argv) != 2:
        raise SystemExit("usage: CatchAddTestsPreprocess.py LISTING.json")
    try:
        output = records_from_listing(Path(sys.argv[1]))
    except (OSError, UnicodeError, json.JSONDecodeError, ValueError) as exc:
        print(f"CatchAddTestsPreprocess: {exc}", file=sys.stderr)
        return 1
    sys.stdout.writelines(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
