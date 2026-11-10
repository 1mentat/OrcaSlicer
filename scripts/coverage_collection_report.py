#!/usr/bin/env python3
"""Render independent executable coverage series without combining their metrics."""
from __future__ import annotations

import argparse
from datetime import datetime
import html
import itertools
import json
from pathlib import Path
import re
import shutil

import coverage_report as leaf
from coverage_collection_contract import SUMMARY_SCHEMA, HISTORY_SCHEMA, SUITES, configuration_id

LABELS = {
    "libslic3r_tests": "Core library",
    "fff_print_tests": "FFF printing",
    "sla_print_tests": "SLA printing",
    "libnest2d_tests": "Nesting",
    "filament_group_tests": "Filament groups",
}


def read_json(path: Path):
    if path.is_symlink() or path.stat().st_size > 16 * 1024**2:
        raise ValueError(f"unsafe or oversized metadata: {path.name}")
    return json.loads(path.read_text(encoding="utf-8"))


def suite_paths(index: dict, root: Path) -> list[tuple[str, Path]]:
    if (index.get("schema") != "orca-slicer-coverage-collection"
            or index.get("schema_version") != 1
            or index.get("measurement_mode") != "per-executable"):
        raise ValueError("unsupported coverage collection")
    suites = index.get("suites")
    if not isinstance(suites, list) or len(suites) != len(SUITES):
        raise ValueError("collection must contain all five canonical suites")
    result = []
    seen = set()
    for suite in suites:
        name = suite.get("name", "")
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", name) or name in seen:
            raise ValueError("invalid or duplicate suite name")
        seen.add(name)
        if suite.get("executable_name") != name:
            raise ValueError("suite name must match its executable name")
        expected = f"suites/{name}/coverage-report.json"
        if suite.get("report") != expected:
            raise ValueError("suite report must use its canonical relative path")
        path = root / expected
        if not path.resolve().is_relative_to(root.resolve()):
            raise ValueError("suite report escapes the collection")
        result.append((name, path))
    if seen != set(SUITES):
        raise ValueError("collection must contain all five canonical suites")
    return result


def collection_identity(suites: dict) -> str:
    return configuration_id(suites)


def copy_annotations(source: Path, destination: Path, remaining: int, entries_left: int) -> tuple[int, int]:
    if not source.exists():
        return 0, 0
    # LLVM's output is a static tree. Never follow a symlink into other inputs.
    total = 0
    entries = 0
    for path in itertools.chain((source,), source.rglob("*")):
        entries += 1
        if entries > entries_left:
            raise ValueError("annotation collection exceeds entry bounds")
        if path.is_symlink() or not (path.is_dir() or path.is_file()):
            raise ValueError("annotation tree contains a symlink or special file")
        if path.is_file():
            size = path.stat().st_size
            total += size
            if size > 32 * 1024**2 or total > remaining:
                raise ValueError("annotation tree exceeds size bounds")
    if destination.exists():
        raise ValueError("annotation destination already exists; use a fresh output directory")
    shutil.copytree(source, destination)
    return total, entries


def trend(report: dict, label: str) -> str:
    if not report["valid"]:
        return '<p class="series-empty">No valid observation from this run.</p>'
    observations = report["history"][-30:]
    if not observations:
        return '<p class="series-empty">No compatible history yet.</p>'
    times = [leaf.timestamp(item["measured_at"]) for item in observations]
    ticks = [datetime.fromisoformat(value).timestamp() for value in times]
    span = max(ticks) - min(ticks)
    points = []
    for tick, item in zip(ticks, observations):
        x = 32 + (tick - ticks[0]) / span * 516 if span else 290
        points.append((x, 92 - item["summary"]["lines"]["percent"] * .8))
    title = html.escape(f"{label}: line coverage, zero to one hundred percent; "
                        f"{len(points)} compatible observations")
    line = " ".join(f"{x:.2f},{y:.2f}" for x, y in points)
    dots = "".join(
        f'<circle cx="{x:.2f}" cy="{y:.2f}" r="3"><title>'
        f'{html.escape(when)}: {item["summary"]["lines"]["percent"]:.2f}%</title></circle>'
        for (x, y), when, item in zip(points, times, observations))
    indices = sorted({round(index * (len(points) - 1) / 3) for index in range(4)})
    tick_labels = []
    for index in indices:
        anchor = "middle"
        if len(points) > 1:
            if index == 0:
                anchor = "start"
            elif index == len(points) - 1:
                anchor = "end"
        tick_labels.append(f'<text x="{points[index][0]:.2f}" y="115" text-anchor="{anchor}">'
                           f'{html.escape(times[index][5:10])}</text>')
    ticks = "".join(tick_labels)
    records = "".join(
        f'<tr><td><time datetime="{html.escape(when, quote=True)}">{html.escape(when)}</time></td>'
        f'<td>{item["summary"]["lines"]["covered"]:,} / {item["summary"]["lines"]["count"]:,}</td>'
        f'<td>{item["summary"]["lines"]["percent"]:.2f}%</td></tr>'
        for when, item in zip(times, observations))
    dates = html.escape(times[0][:10]) + " — " + html.escape(times[-1][:10])
    note = "First compatible baseline" if len(points) == 1 else f"{len(points)} observations · {dates}"
    return (f'<svg class="series-plot" viewBox="0 0 560 130" role="img" aria-label="{title}">'
            f'<title>{title}</title><path class="series-grid" d="M32 12H548 M32 92H548"/>'
            '<g class="series-axis"><text x="0" y="16">100</text><text x="16" y="96">0</text></g>'
            f'<g class="series-data"><polyline points="{line}"/>{dots}</g>'
            f'<g class="series-axis">{ticks}</g></svg><p class="series-caption">{note}</p>'
            '<details class="series-observations"><summary>Observation data</summary>'
            '<div class="table-scroll" tabindex="0"><table><thead><tr><th>Measured (UTC)</th>'
            f'<th>Covered / total lines</th><th>Coverage</th></tr></thead><tbody>{records}</tbody></table></div></details>')


def index_html(summary: dict) -> str:
    template_dir = Path(__file__).with_name("coverage_report")
    incumbent = (template_dir / "index.html").read_text(encoding="utf-8")
    style = incumbent.split("<style>", 1)[1].split("</style>", 1)[0]
    rows = []
    for name, report in summary["suites"].items():
        label = LABELS.get(name, name)
        metric = report["summary"]["lines"] if report["valid"] else None
        value = leaf.percentage(metric) if metric else "Incomplete"
        counts = f"{metric['covered']:,} / {metric['count']:,} lines" if metric else "No baseline credit"
        change = (f"{report['change']:+.2f} pp" if report["change"] is not None else "No compatible comparison")
        rows.append(
            f'<section class="series-row" aria-labelledby="suite-{name}">'
            f'<div><h2 id="suite-{name}"><a href="suites/{name}/index.html">{html.escape(label)}</a></h2>'
            f'<p class="series-caption"><code>{html.escape(name)}</code></p>'
            f'<p><strong class="numeric">{html.escape(value)}</strong><br>{html.escape(counts)}'
            f'<br><span class="series-caption">{html.escape(change)}</span></p></div>'
            f'<div>{trend(report, label)}</div>'
            f'<a class="series-detail" href="suites/{name}/index.html#files-heading">Files and annotations</a></section>')
    run = (f'<a href="{html.escape(summary["run_url"], quote=True)}">CI run</a>'
           if summary["run_url"] else "")
    complete = sum(item["valid"] for item in summary["suites"].values())
    count = len(summary["suites"])
    notice = (f"{complete} of {count} suites complete. All suite measurements are valid."
              if summary["valid"] else f"{complete} of {count} suites complete. "
              "Incomplete collection. This run does not establish a complete coverage baseline.")
    substitutions = {
        "__COLLECTION_STYLE__": style,
        "__COLLECTION_ROWS__": "\n".join(rows),
        "__COMMIT__": html.escape(summary["commit"][:12]),
        "__MEASURED__": html.escape(summary["measured_at"]),
        "__MEASURED_LABEL__": datetime.fromisoformat(summary["measured_at"].replace("Z", "+00:00")).strftime("%d %b %Y · %H:%M UTC"),
        "__CONFIGURATION__": html.escape(summary["configuration"]),
        "__RUN_LINK__": run,
        "__NOTICE__": notice,
        "__SUITE_COUNT__": str(len(summary["suites"])),
    }
    template = (template_dir / "collection.html").read_text(encoding="utf-8")
    # Substitute once, so an input label cannot expand another placeholder.
    return re.sub("|".join(map(re.escape, substitutions)), lambda match: substitutions[match[0]], template)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collection", required=True, type=Path)
    parser.add_argument("--root", default=Path.cwd(), type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--repository", required=True)
    parser.add_argument("--run-url", default="")
    parser.add_argument("--branch", default="main")
    parser.add_argument("--configuration", required=True)
    parser.add_argument("--measured-at", required=True)
    parser.add_argument("--previous", type=Path, help="previous collection site, with suites/ histories")
    args = parser.parse_args(argv)
    try:
        raw = read_json(args.collection)
        entries = suite_paths(raw, args.collection.parent)
        args.out.mkdir(parents=True, exist_ok=True)
        if any(args.out.iterdir()):
            raise ValueError("use an empty output directory for a collection")
        suites = {}
        annotation_bytes, annotation_entries = 0, 0
        for name, report_path in entries:
            output = args.out / "suites" / name
            output.mkdir(parents=True)
            measurement = read_json(report_path)
            if (len(measurement.get("objects", [])) != 1
                    or Path(measurement["objects"][0]).name != name):
                raise ValueError("suite report does not identify its named executable")
            size, count = copy_annotations(report_path.parent / "html", output / "source",
                                           1024**3 - annotation_bytes, 20000 - annotation_entries)
            annotation_bytes += size
            annotation_entries += count
            command = ["--report", str(report_path), "--root", str(args.root), "--out", str(output),
                       "--commit", args.commit, "--repository", args.repository,
                       "--run-url", args.run_url, "--branch", args.branch,
                       "--configuration", args.configuration, "--measured-at", args.measured_at,
                       "--scope", f"Core C++ · Linux · Release · {name}"]
            if args.previous:
                previous = args.previous / "suites" / name
                for flag, filename in (("--baseline", "summary.json"), ("--history", "history.json")):
                    path = previous / filename
                    if not path.is_file():
                        raise ValueError("previous collection is missing a suite baseline/history")
                    command += [flag, str(path)]
            if leaf.main(command) != 0:
                raise ValueError(f"could not render suite {name}")
            suites[name] = read_json(output / "summary.json")
        first = next(iter(suites.values()))
        valid = (raw.get("integrity_status") == "pass"
                 and raw.get("status") in leaf.VALID_STATUSES
                 and all(item["valid"] for item in suites.values()))
        summary = {key: first[key] for key in
                   ("commit", "repository", "branch", "run_url", "measured_at", "configuration")}
        summary.update(schema=SUMMARY_SCHEMA, schema_version=1, measurement_mode="per-executable",
                       configuration_id=collection_identity(suites), valid=valid,
                       status="pass" if valid else "incomplete", suites=suites)
        history = {"schema": HISTORY_SCHEMA, "schema_version": 1,
                   "configuration_id": summary["configuration_id"],
                   "suites": {name: item["history"] for name, item in suites.items()}}
        leaf.write_atomic(args.out / "summary.json", json.dumps(summary, indent=2) + "\n")
        leaf.write_atomic(args.out / "history.json", json.dumps(history, indent=2) + "\n")
        leaf.write_atomic(args.out / "index.html", index_html(summary))
        leaf.write_atomic(args.out / "summary.md", "\n".join(
            ["## Coverage by test suite", "", "Source: " + summary["commit"], "", notice_text(summary)]
            + [f"- {name}: {leaf.percentage(item['summary']['lines']) if item['valid'] else 'incomplete'}"
               for name, item in suites.items()]) + "\n")
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f"coverage collection: {error}\n")
    return 0


def notice_text(summary: dict) -> str:
    return ("Each percentage describes one test executable. Files overlap; percentages are not combined."
            if summary["valid"] else "Incomplete collection; no complete baseline is established.")


if __name__ == "__main__":
    raise SystemExit(main())
