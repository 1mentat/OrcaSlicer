"""Publication must preserve newer history and reject incomplete measurements."""

import copy
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[2] / ".github/scripts/coverage_history.py"
spec = importlib.util.spec_from_file_location("coverage_history", SCRIPT)
history = importlib.util.module_from_spec(spec)
spec.loader.exec_module(history)


class FakeGitHub:
    def __init__(self, previous=None, comparison="ahead"):
        self.previous = previous
        self.comparison = comparison
        self.writes = []

    def head(self):
        return "b" * 40 if self.previous else None

    def file(self, name, revision):
        return (self.previous.get("history", []) if name == "history.json"
                else self.previous)

    def request(self, path, body=None, method="GET"):
        if path.startswith("/compare/"):
            return {"status": self.comparison}
        self.writes.append((path, body, method))
        return {"sha": "c" * 40}


class FakeCollectionGitHub:
    def __init__(self, summary, history):
        self.summary = summary
        self.history = history
        self.writes = []

    def head(self):
        return "b" * 40

    def file(self, name, revision):
        return self.summary if name == "summary.json" else self.history

    def request(self, path, body=None, method="GET"):
        if path.startswith("/compare/"):
            return {"status": "ahead"}
        self.writes.append((path, body, method))
        return {"sha": "c" * 40}


def collection(entries, config_prefix="d"):
    suites, histories = {}, {}
    shared = {"repository": "https://example.com/owner/repo", "branch": "main",
              "run_url": "https://example.com/run/1", "configuration": "test-config"}
    commit, measured_at, _ = entries[-1]
    for index, name in enumerate(history.collection_contract.SUITES):
        configuration_id = config_prefix + f"{index:063x}"
        observations = [{"valid": True, "commit": source, "configuration_id": configuration_id,
                         "measured_at": measured,
                         "summary": {"lines": {"covered": covered, "count": 10}}}
                        for source, measured, covered in entries]
        suites[name] = {"schema_version": 1, "valid": True, "commit": commit,
                        "configuration_id": configuration_id, "source_verified": True,
                        "integrity_status": "pass", "measured_at": measured_at,
                        "summary": observations[-1]["summary"], "history": observations,
                        "scope": "Core C++ · Linux · Release · " + name, **shared}
        histories[name] = observations
    identity = history.collection_contract.configuration_id(suites)
    summary = {"schema": history.collection_contract.SUMMARY_SCHEMA, "schema_version": 1,
               "measurement_mode": "per-executable", "valid": True, "status": "pass",
               "commit": commit, "measured_at": measured_at, "configuration_id": identity,
               **shared, "suites": suites}
    history_data = {"schema": history.collection_contract.HISTORY_SCHEMA, "schema_version": 1,
                    "configuration_id": identity, "suites": histories}
    return summary, history_data


class CoverageHistoryTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.observations = [{"valid": True, "commit": "a" * 40,
                              "configuration_id": "d" * 64,
                              "measured_at": "2026-11-09T12:00:00Z",
                              "summary": {"lines": {"covered": 1, "count": 10}}}]
        self.summary = {"schema_version": 1, "valid": True, "commit": "a" * 40,
                        "configuration_id": "d" * 64,
                        "source_verified": True, "integrity_status": "pass",
                        "measured_at": "2026-11-09T12:00:00Z", "history": self.observations}
        (self.directory / "summary.json").write_text(json.dumps(self.summary))
        (self.directory / "history.json").write_text(json.dumps(self.observations))

    def test_incomplete_or_different_source_cannot_publish(self):
        for changes in ({"valid": False}, {"source_verified": False},
                        {"integrity_status": "error"}, {"commit": "b" * 40}):
            value = {**self.summary, **changes}
            with self.assertRaises(ValueError):
                history.validate(value, self.observations, "a" * 40)
        bad = copy.deepcopy(self.observations)
        bad[0]["summary"]["lines"]["covered"] = -1
        with self.assertRaises(ValueError):
            history.validate({**self.summary, "history": bad}, bad, "a" * 40)

    def test_older_run_does_not_replace_newer_data(self):
        api = FakeGitHub({**self.summary, "measured_at": "2026-11-10T12:00:00Z"})
        history.publish(api, self.directory, "a" * 40)
        self.assertEqual(api.writes, [])

    def test_first_publication_creates_data_only_tree(self):
        api = FakeGitHub()
        history.publish(api, self.directory, "a" * 40)
        tree = next(body["tree"] for path, body, method in api.writes if path == "/git/trees")
        self.assertEqual([item["path"] for item in tree], ["history.json", "summary.json"])
        commit = next(body for path, body, method in api.writes if path == "/git/commits")
        self.assertEqual(commit["parents"], [])
        self.assertEqual(api.writes[-1][1]["ref"], "refs/heads/coverage-data")

    def test_next_publication_uses_non_force_update(self):
        previous_history = copy.deepcopy(self.observations)
        previous_history[0]["measured_at"] = "2026-11-08T12:00:00Z"
        previous = {**self.summary, "measured_at": "2026-11-08T12:00:00Z",
                    "history": previous_history}
        api = FakeGitHub(previous)
        history.publish(api, self.directory, "a" * 40)
        commit = next(body for path, body, method in api.writes if path == "/git/commits")
        self.assertEqual(commit["parents"], ["b" * 40])
        self.assertIs(api.writes[-1][1]["force"], False)

    def test_rerunning_older_source_cannot_replace_newer_source(self):
        previous_history = copy.deepcopy(self.observations)
        previous_history[0].update(measured_at="2026-11-08T12:00:00Z", commit="e" * 40)
        previous = {**self.summary, "measured_at": "2026-11-08T12:00:00Z",
                    "commit": "e" * 40, "history": previous_history}
        for status in ("behind", "diverged"):
            api = FakeGitHub(previous, comparison=status)
            self.assertIs(history.publish(api, self.directory, "a" * 40), False)
            self.assertEqual(api.writes, [])
        api = FakeGitHub(previous, comparison="ahead")
        self.assertIs(history.publish(api, self.directory, "a" * 40), True)

    def publish_collection(self, api, entries, config_prefix="d"):
        summary, history_data = collection(entries, config_prefix)
        (self.directory / "summary.json").write_text(json.dumps(summary))
        (self.directory / "history.json").write_text(json.dumps(history_data))
        return history.publish(api, self.directory, summary["commit"])

    def test_collection_publisher_rejects_stale_or_mutated_history_before_writes(self):
        entries = [("a" * 40, "2026-11-07T12:00:00Z", 1),
                   ("b" * 40, "2026-11-08T12:00:00Z", 2)]
        previous_summary, previous_history = collection(entries)
        for incoming_entries in (
                [entries[0], ("c" * 40, "2026-11-09T12:00:00Z", 3)],
                [(entries[0][0], entries[0][1], 9), entries[1],
                 ("c" * 40, "2026-11-09T12:00:00Z", 3)]):
            with self.subTest(incoming_entries=incoming_entries):
                api = FakeCollectionGitHub(previous_summary, previous_history)
                with self.assertRaises(ValueError):
                    self.publish_collection(api, incoming_entries)
                self.assertEqual(api.writes, [])

    def test_collection_publisher_accepts_exact_365_entry_truncation(self):
        start = datetime(2025, 1, 1, tzinfo=timezone.utc)
        entries = [(f"{index + 1:040x}",
                    (start + timedelta(days=index)).strftime("%Y-%m-%dT%H:%M:%SZ"), index % 10)
                   for index in range(365)]
        previous_summary, previous_history = collection(entries)
        api = FakeCollectionGitHub(previous_summary, previous_history)
        new = ("f" * 40, (start + timedelta(days=365)).strftime("%Y-%m-%dT%H:%M:%SZ"), 5)
        self.assertTrue(self.publish_collection(api, entries[1:] + [new]))
        self.assertTrue(api.writes)

    def test_collection_publisher_allows_per_suite_configuration_reset(self):
        entries = [("a" * 40, "2026-11-08T12:00:00Z", 1)]
        previous_summary, previous_history = collection(entries)
        api = FakeCollectionGitHub(previous_summary, previous_history)
        summary, history_data = collection(entries + [("b" * 40, "2026-11-09T12:00:00Z", 2)])
        name = history.collection_contract.SUITES[0]
        leaf = summary["suites"][name]
        leaf["configuration_id"] = "e" * 64
        leaf["history"] = [leaf["history"][-1]]
        leaf["history"][0]["configuration_id"] = leaf["configuration_id"]
        history_data["suites"][name] = leaf["history"]
        identity = history.collection_contract.configuration_id(summary["suites"])
        summary["configuration_id"] = history_data["configuration_id"] = identity
        (self.directory / "summary.json").write_text(json.dumps(summary))
        (self.directory / "history.json").write_text(json.dumps(history_data))
        self.assertTrue(history.publish(api, self.directory, summary["commit"]))
        self.assertTrue(api.writes)


if __name__ == "__main__":
    unittest.main()
