#!/usr/bin/env python3
"""Summarize a gcovr --json-summary file and check it against a line floor.

Used by .github/workflows/coverage.yml. The numbers come straight from gcovr;
this script only formats them and compares the total with the floor.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def render(summary: dict, commit: str, floor: float, platform: str,
           worst: int = 25) -> str:
    covered, total = summary["line_covered"], summary["line_total"]
    files = sorted(
        summary["files"],
        key=lambda f: f["line_total"] - f["line_covered"],
        reverse=True,
    )
    lines = [
        f"## libslic3r line coverage (gcov, {platform})",
        "",
        f"**{summary['line_percent']:.2f}%** — {covered} of {total} lines, "
        f"{len(files)} files, floor {floor:g}%",
        "",
    ]
    if commit:
        lines += [f"Commit `{commit}`", ""]
    lines += [
        f"### {worst} files with the most uncovered lines",
        "",
        "| File | Uncovered | Lines | Coverage |",
        "|---|---:|---:|---:|",
    ]
    for f in files[:worst]:
        missed = f["line_total"] - f["line_covered"]
        lines.append(
            f"| {f['filename']} | {missed} | {f['line_total']} | {f['line_percent']:.1f}% |"
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("summary", type=Path)
    parser.add_argument("--commit", default="")
    parser.add_argument("--floor", type=float, default=90.0)
    parser.add_argument("--platform", default="Linux")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--check", action="store_true",
                        help="exit 1 when line coverage is below the floor")
    args = parser.parse_args()

    summary = json.loads(args.summary.read_text())
    if args.out:
        args.out.write_text(render(summary, args.commit, args.floor, args.platform))
    if args.check:
        percent = summary["line_percent"]
        if summary["line_total"] == 0 or percent < args.floor:
            print(f"::error::libslic3r line coverage {percent:.2f}% is below "
                  f"the {args.floor:g}% floor")
            return 1
        print(f"libslic3r line coverage {percent:.2f}% meets the {args.floor:g}% floor")
    return 0


if __name__ == "__main__":
    sys.exit(main())
