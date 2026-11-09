#!/usr/bin/env python3
"""Render a portable, offline coverage report without changing measurement policy."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import html
import json
import os
from pathlib import Path
import re
import tempfile
from urllib.parse import quote, urlsplit


METRICS = ("lines", "functions", "branches")
VALID_STATUSES = {"pass", "measured", "measurement", "coverage_below_threshold"}


def read_json(path: Path, limit: int = 32 * 1024 * 1024):
    if path.stat().st_size > limit:
        raise ValueError(f"report exceeds {limit} bytes: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def metric(value):
    if not isinstance(value, dict):
        return None
    covered, count = value.get("covered"), value.get("count")
    if type(covered) is not int or type(count) is not int or not 0 <= covered <= count:
        raise ValueError("coverage counts must be integers with 0 <= covered <= count")
    if count == 0:
        return None
    return {"covered": covered, "count": count, "percent": 100 * covered / count}


def metrics(value):
    if not isinstance(value, dict):
        raise ValueError("coverage metrics must be an object")
    return {key: metric(value.get(key)) for key in METRICS}


def https_url(value: str) -> str:
    if not value:
        return ""
    if any(ord(char) < 32 or ord(char) == 127 for char in value):
        raise ValueError("links cannot contain control characters")
    parsed = urlsplit(value)
    if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError("source and run links must be HTTPS URLs without credentials")
    return value.rstrip("/")


def timestamp(value: str) -> str:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None:
        raise ValueError("measurement time must include a timezone")
    return parsed.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def tool_identity(tools: dict) -> dict:
    # Keep full tool output in evidence, but relocation and LLVM's host-CPU
    # diagnostic do not change the compiler or coverage format of a series.
    return {name: {"version": value.get("version"),
                   "version_text": "\n".join(
                       line.strip() for line in value.get("version_text", "").splitlines()
                       if not line.strip().startswith(("InstalledDir:", "Host CPU:")))}
            if isinstance(value, dict) else value for name, value in tools.items()}


def write_atomic(path: Path, value: str):
    with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                     prefix=".coverage-", delete=False) as output:
        temporary = Path(output.name)
        try:
            output.write(value)
            output.close()
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)


def percentage(value) -> str:
    return f"{value['percent']:.2f}%" if value else "Not measured"


def render(raw: dict, *, root: Path, out: Path, commit: str, repository: str,
           run_url: str, branch: str, configuration: str, scope: str,
           measured_at: str, history: list, baseline: dict | None,
           source_template: str, annotations: str = "source") -> dict:
    if not re.fullmatch(r"[0-9a-fA-F]{40}", commit):
        raise ValueError("an exact 40-character source commit is required")
    commit = commit.lower()
    repository, run_url = https_url(repository), https_url(run_url)
    measured_at = timestamp(measured_at)
    source_identity = raw.get("provenance", {}).get("git", {})
    measured_commit = source_identity.get("head")
    if measured_commit and measured_commit.lower() != commit:
        raise ValueError("measurement provenance does not match the requested source commit")
    source_verified = (measured_commit == commit
                       and source_identity.get("working_tree", {}).get("tracked_changes") is False)
    backend = raw.get("backend", "llvm")
    tool_versions = raw.get("tool_versions", {})
    identity = {"configuration": configuration, "backend": backend,
                "scope": scope, "repository": repository, "tools": tool_identity(tool_versions),
                "measurement_schema": raw.get("schema_version"),
                "measurement_semantics": raw.get("measurement", {}).get("semantics")}
    configuration_id = hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()
    summary = metrics(raw.get("summary", {}))
    valid = (raw.get("status") in VALID_STATUSES and raw.get("integrity_status") == "pass"
             and summary["lines"] is not None and source_verified)
    result = {"schema_version": 1, "commit": commit, "repository": repository,
              "branch": branch, "run_url": run_url, "configuration": configuration,
              "configuration_id": configuration_id, "backend": backend, "scope": scope,
              "measured_at": measured_at, "valid": valid, "status": raw.get("status", "unknown"),
              "summary": summary, "tool_versions": tool_versions, "files": [],
              "integrity_status": raw.get("integrity_status", "unknown"),
              "integrity_failures": raw.get("integrity_failures", {}),
              "source_verified": source_verified,
              "exclusions": "GUI, other operating systems, uncompiled platform sources and external dependencies"}
    comparable = (baseline if baseline and baseline.get("valid") is True
                  and baseline.get("configuration_id") == configuration_id
                  and datetime.fromisoformat(timestamp(baseline["measured_at"]))
                  < datetime.fromisoformat(measured_at)
                  and metric(baseline.get("summary", {}).get("lines")) else None)
    previous_files = {item["path"]: item for item in comparable.get("files", [])} if comparable else {}
    seen = set()
    for item in raw.get("files", []):
        path = Path(item["filename"])
        path = (root / path).resolve() if not path.is_absolute() else path.resolve()
        try:
            relative = path.relative_to(root.resolve()).as_posix()
        except ValueError as error:
            raise ValueError("coverage file is outside the measured checkout") from error
        if relative in seen:
            raise ValueError("coverage contains a duplicate source file")
        seen.add(relative)
        values = metrics(item.get("summary", {}))
        previous = metric(previous_files.get(relative, {}).get("summary", {}).get("lines"))
        change = (values["lines"]["percent"] - previous["percent"]
                  if values["lines"] and previous else None)
        source_url = ""
        if source_template:
            source_url = https_url(source_template.format(
                repository=repository, commit=commit, path=quote(relative, safe="/")))
        annotation_url = ""
        # llvm-cov's HTML directory mirrors source paths; never link outside out.
        for source_path in (str(path).lstrip("/"), relative):
            candidate = out / annotations / "coverage" / (source_path + ".html")
            if candidate.is_file() and candidate.resolve().is_relative_to(out.resolve()):
                annotation_url = quote(candidate.relative_to(out).as_posix(), safe="/")
                break
        result["files"].append({"path": relative, "summary": values,
                                "change": change if valid else None,
                                "source_url": source_url, "annotation_url": annotation_url})
    result["files"].sort(key=lambda item: item["path"])
    result["baseline"] = ({"commit": comparable["commit"], "measured_at": comparable["measured_at"]}
                          if comparable and valid else None)
    result["change"] = (summary["lines"]["percent"] - metric(comparable["summary"]["lines"])["percent"]
                        if comparable and valid else None)
    observations = []
    for record in history:
        if record.get("valid") is not True or record.get("configuration_id") != configuration_id:
            continue
        if not re.fullmatch(r"[0-9a-f]{40}", record.get("commit", "")):
            raise ValueError("history contains an invalid commit")
        measured = timestamp(record["measured_at"])
        values = metrics(record["summary"])
        if values["lines"]:
            observations.append({"commit": record["commit"], "measured_at": measured,
                                 "valid": True, "configuration_id": configuration_id, "summary": values})
    if valid:
        observations.append({key: result[key] for key in
                             ("commit", "measured_at", "valid", "configuration_id", "summary")})
    observations = list({(item["commit"], item["measured_at"]): item for item in observations}.values())
    observations.sort(key=lambda item: datetime.fromisoformat(item["measured_at"]))
    result["history"] = observations[-365:]
    result["annotation_index"] = (f"{annotations}/index.html"
                                  if (out / annotations / "index.html").is_file() else "")
    return result


def markdown(report: dict) -> str:
    # JSON is the machine contract; escape all Markdown metacharacters in labels.
    def label(value):
        return re.sub(r"([\\`*_{}\[\]()#+.!|<>-])", r"\\\1", str(value)).replace("\n", " ")
    state = "Valid measurement" if report["valid"] else "Incomplete measurement"
    lines = [f"## Coverage · {state}", "", label(report["scope"]), "",
             f"Commit: `{report['commit']}` · {label(report['measured_at'])}", "",
             "| Metric | Coverage | Covered / executable |", "|---|---:|---:|"]
    for key in METRICS:
        value = report["summary"][key] if report["valid"] else None
        lines.append(f"| {key.title()} | {percentage(value)} | "
                     + (f"{value['covered']:,} / {value['count']:,} |" if value else "— |"))
    if report["change"] is not None:
        lines += ["", f"Line change: {report['change']:+.2f} percentage points against "
                  f"`{report['baseline']['commit'][:12]}`."]
    if not report["valid"]:
        lines += ["", f"Status: **{label(report['status'])}**. This run is not a coverage baseline."]
        if not report["source_verified"]:
            lines += ["", "The measurement needs matching commit provenance and an unchanged tracked source tree."]
    lines += ["", "Scope excludes: " + label(report["exclusions"]) + ".",
              "", "Download the HTML report and raw evidence from this run's artifacts.", ""]
    return "\n".join(lines)


def badge(report: dict) -> str:
    value = percentage(report["summary"]["lines"]) if report["valid"] else "incomplete"
    title = html.escape(f"Core line coverage: {value}; measured {report['measured_at']}")
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="190" height="24" role="img" aria-label="{title}">'
            f'<title>{title}</title><rect width="190" height="24" rx="4" fill="#243333"/>'
            '<path d="M102 0h84q4 0 4 4v16q0 4-4 4h-84z" fill="#087f75"/>'
            '<g font-family="Verdana,sans-serif" font-size="11" fill="white" text-anchor="middle">'
            f'<text x="51" y="16">core coverage</text><text x="146" y="16">{value}</text></g></svg>')


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--root", default=Path.cwd(), type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--commit", required=True)
    parser.add_argument("--repository", required=True)
    parser.add_argument("--run-url", default="")
    parser.add_argument("--branch", default="main")
    parser.add_argument("--configuration", required=True)
    parser.add_argument("--scope", default="Core C++ · Linux · Release · GUI off")
    parser.add_argument("--measured-at", default=datetime.now(timezone.utc).isoformat())
    parser.add_argument("--history", type=Path)
    parser.add_argument("--baseline", type=Path)
    parser.add_argument("--source-template", default="{repository}/blob/{commit}/{path}")
    parser.add_argument("--annotations", default="source")
    args = parser.parse_args(argv)
    try:
        if not re.fullmatch(r"[a-zA-Z0-9_-]+", args.annotations):
            raise ValueError("annotation directory must be one relative directory name")
        args.out.mkdir(parents=True, exist_ok=True)
        history = read_json(args.history) if args.history else []
        if not isinstance(history, list):
            raise ValueError("history must be an array")
        report = render(read_json(args.report), root=args.root.resolve(), out=args.out.resolve(),
                        commit=args.commit, repository=args.repository, run_url=args.run_url,
                        branch=args.branch, configuration=args.configuration, scope=args.scope,
                        measured_at=args.measured_at, history=history,
                        baseline=read_json(args.baseline) if args.baseline else None,
                        source_template=args.source_template, annotations=args.annotations)
        template = (Path(__file__).parent / "coverage_report" / "index.html").read_text(encoding="utf-8")
        payload = json.dumps(report, ensure_ascii=True).replace("<", "\\u003c").replace("&", "\\u0026")
        write_atomic(args.out / "index.html", template.replace("__COVERAGE_DATA__", payload))
        write_atomic(args.out / "summary.json", json.dumps(report, indent=2) + "\n")
        write_atomic(args.out / "history.json", json.dumps(report["history"], indent=2) + "\n")
        write_atomic(args.out / "summary.md", markdown(report))
        write_atomic(args.out / "coverage.svg", badge(report))
    except (OSError, ValueError, KeyError, TypeError) as error:
        parser.exit(2, f"coverage report: {error}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
