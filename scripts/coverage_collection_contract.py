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


def require_history_extension(previous_summary, previous_history, summary, history):
    """Reject loss or mutation of compatible observations from the current revision."""
    previous_collection = previous_summary.get("schema") == SUMMARY_SCHEMA
    collection = summary.get("schema") == SUMMARY_SCHEMA
    if previous_collection and not collection:
        raise ValueError("cannot replace suite history with a legacy global report")
    if not previous_collection:
        return  # A legacy global history has no compatible per-suite series to extend.
    for name in SUITES:
        previous_report = previous_summary["suites"][name]
        report = summary["suites"][name]
        if previous_report["configuration_id"] != report["configuration_id"]:
            continue  # This suite's measurement configuration changed.
        previous = previous_history["suites"][name]
        current = history["suites"][name]
        previous_by_key = {}
        previous_time = None
        for item in previous:
            key = (item["commit"], item["measured_at"])
            measured = datetime.fromisoformat(item["measured_at"].replace("Z", "+00:00"))
            if key in previous_by_key or (previous_time is not None and measured < previous_time):
                raise ValueError(f"current history for {name} has duplicate or unordered observations")
            previous_by_key[key] = item
            previous_time = measured

        current_key = (report["commit"], report["measured_at"])
        current_observations = [item for item in current
                                if (item["commit"], item["measured_at"]) == current_key]
        if len(current_observations) != 1:
            raise ValueError(f"new measurement is absent or duplicated in {name} history")
        if current_key in previous_by_key and previous_by_key[current_key] != current_observations[0]:
            raise ValueError(f"history for {name} changed an existing observation")
        expected_by_key = dict(previous_by_key)
        expected_by_key[current_key] = current_observations[0]
        expected = sorted(expected_by_key.values(), key=lambda item: datetime.fromisoformat(
            item["measured_at"].replace("Z", "+00:00")))[-365:]
        if current != expected:
            raise ValueError(f"history for {name} is stale or drops compatible observations")


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
