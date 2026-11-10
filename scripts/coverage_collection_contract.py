"""Shared validation for independent coverage series on either forge."""
from datetime import datetime
import hashlib
import json
import re


SUMMARY_SCHEMA = "orca-slicer-coverage-suite-summary"
HISTORY_SCHEMA = "orca-slicer-coverage-suite-history"
SUITES = ("libslic3r_tests", "fff_print_tests", "sla_print_tests",
          "libnest2d_tests", "filament_group_tests")


def configuration_id(suites):
    identity = {"schema": SUMMARY_SCHEMA,
                "series": {name: leaf["configuration_id"] for name, leaf in suites.items()}}
    return hashlib.sha256(json.dumps(identity, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def validate(summary, history, expected_commit, validate_leaf):
    """Validate every leaf with the existing measurement/history contract."""
    if not isinstance(summary, dict) or not isinstance(history, dict):
        raise ValueError("collection metadata must be JSON objects")
    if (summary.get("schema") != SUMMARY_SCHEMA or summary.get("schema_version") != 1
            or summary.get("measurement_mode") != "per-executable"
            or summary.get("valid") is not True or summary.get("status") != "pass"):
        raise ValueError("only complete valid suite collections may be published")
    if (history.get("schema") != HISTORY_SCHEMA or history.get("schema_version") != 1):
        raise ValueError("unsupported suite history schema")
    suites, histories = summary.get("suites"), history.get("suites")
    if (not isinstance(suites, dict) or not isinstance(histories, dict)
            or set(suites) != set(SUITES) or suites.keys() != histories.keys()):
        raise ValueError("suite summaries and histories must contain all five canonical suites")
    if summary.get("commit") != expected_commit:
        raise ValueError("collection source does not match the measured commit")
    shared = ("commit", "repository", "branch", "run_url", "measured_at", "configuration")
    identifiers = set()
    for name, report in suites.items():
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,79}", name):
            raise ValueError("invalid suite identity")
        if not isinstance(report, dict) or report.get("schema") == SUMMARY_SCHEMA:
            raise ValueError("suite must be one native measurement")
        if any(report.get(key) != summary.get(key) for key in shared):
            raise ValueError("collection contains mixed source or run identities")
        if report.get("scope") != "Core C++ · Linux · Release · " + name:
            raise ValueError("suite scope does not identify its executable")
        validate_leaf(report, histories[name], expected_commit)
        identifiers.add(report["configuration_id"])
    if len(identifiers) != len(suites):
        raise ValueError("suite configurations must identify independent series")
    identity = configuration_id(suites)
    if summary.get("configuration_id") != identity or history.get("configuration_id") != identity:
        raise ValueError("collection configuration does not match its series")
    when = datetime.fromisoformat(summary["measured_at"].replace("Z", "+00:00"))
    if when.tzinfo is None:
        raise ValueError("measurement time must include a timezone")
    return when
