"""Publication must preserve newer history and reject incomplete measurements."""

import copy
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
        return self.previous

    def request(self, path, body=None, method="GET"):
        if path.startswith("/compare/"):
            return {"status": self.comparison}
        self.writes.append((path, body, method))
        return {"sha": "c" * 40}


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
        api = FakeGitHub({**self.summary, "measured_at": "2026-11-08T12:00:00Z"})
        history.publish(api, self.directory, "a" * 40)
        commit = next(body for path, body, method in api.writes if path == "/git/commits")
        self.assertEqual(commit["parents"], ["b" * 40])
        self.assertIs(api.writes[-1][1]["force"], False)

    def test_rerunning_older_source_cannot_replace_newer_source(self):
        previous = {**self.summary, "measured_at": "2026-11-08T12:00:00Z", "commit": "e" * 40}
        for status in ("behind", "diverged"):
            api = FakeGitHub(previous, comparison=status)
            self.assertIs(history.publish(api, self.directory, "a" * 40), False)
            self.assertEqual(api.writes, [])
        api = FakeGitHub(previous, comparison="ahead")
        self.assertIs(history.publish(api, self.directory, "a" * 40), True)


if __name__ == "__main__":
    unittest.main()
