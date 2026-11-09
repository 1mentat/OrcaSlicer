"""Report integrity, comparable history, and offline output contracts."""

import copy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "coverage_report.py"
spec = importlib.util.spec_from_file_location("coverage_report", SCRIPT)
reporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reporter)


class CoverageReportTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.raw = {"status": "measured", "integrity_status": "pass", "backend": "llvm",
                    "provenance": {"git": {"head": "a" * 40,
                                            "working_tree": {"tracked_changes": False}}},
                    "tool_versions": {"clang": "18"},
                    "summary": {"lines": {"covered": 2, "count": 10, "percent": 99}},
                    "files": [{"filename": str(self.root / "src/a.cpp"),
                               "summary": {"lines": {"covered": 2, "count": 10}}}]}

    def render(self, raw=None, **kwargs):
        arguments = dict(root=self.root, out=self.root, commit="a" * 40,
                         repository="https://example.com/owner/repo", run_url="",
                         branch="main", configuration="linux-clang18-release",
                         scope="Core C++", measured_at="2026-11-09T12:00:00Z",
                         history=[], baseline=None,
                         source_template="{repository}/blob/{commit}/{path}")
        arguments.update(kwargs)
        return reporter.render(raw or self.raw, **arguments)

    def test_recomputes_percentages_and_preserves_unmeasured(self):
        result = self.render()
        self.assertTrue(result["valid"])
        self.assertEqual(result["summary"]["lines"]["percent"], 20)
        self.assertIsNone(result["summary"]["branches"])
        self.assertIsNone(reporter.metric({"covered": 0, "count": 0}))
        with self.assertRaises(ValueError):
            reporter.metric({"covered": True, "count": 10})

    def test_only_valid_compatible_older_baselines_compare(self):
        previous = self.render(measured_at="2026-11-08T12:00:00Z")
        raw = copy.deepcopy(self.raw)
        raw["summary"]["lines"]["covered"] = 3
        raw["files"][0]["summary"]["lines"]["covered"] = 3
        result = self.render(raw, baseline=previous, history=previous["history"])
        self.assertEqual(result["change"], 10)
        self.assertEqual(result["files"][0]["change"], 10)
        self.assertEqual(len(result["history"]), 2)
        for changes in ({"configuration": "different"},
                        {"measured_at": "2026-11-07T12:00:00Z"}):
            result = self.render(baseline=previous, **changes)
            self.assertIsNone(result["baseline"])

    def test_invalid_run_is_never_added_to_history_or_badge(self):
        previous = self.render(measured_at="2026-11-08T12:00:00Z")
        raw = copy.deepcopy(self.raw)
        raw["integrity_status"] = "fail"
        result = self.render(raw, baseline=previous, history=previous["history"])
        self.assertFalse(result["valid"])
        self.assertEqual(len(result["history"]), 1)
        self.assertIsNone(result["change"])
        self.assertIn("incomplete", reporter.badge(result))
        self.assertNotIn("20.00%", reporter.markdown(result))

    def test_empty_previous_file_is_no_comparison(self):
        previous = self.render(measured_at="2026-11-08T12:00:00Z")
        previous["files"][0]["summary"]["lines"] = {"covered": 0, "count": 0}
        self.assertIsNone(self.render(baseline=previous)["files"][0]["change"])

    def test_source_identity_must_match_and_tracked_sources_must_be_clean(self):
        raw = copy.deepcopy(self.raw)
        raw["provenance"]["git"]["head"] = "b" * 40
        with self.assertRaises(ValueError):
            self.render(raw)
        raw["provenance"]["git"]["head"] = "a" * 40
        raw["provenance"]["git"]["working_tree"]["tracked_changes"] = True
        result = self.render(raw)
        self.assertFalse(result["valid"])
        self.assertEqual(result["history"], [])

    def test_rejects_paths_outside_checkout_and_duplicates(self):
        raw = copy.deepcopy(self.raw)
        raw["files"][0]["filename"] = "../private.cpp"
        with self.assertRaises(ValueError):
            self.render(raw)
        raw = copy.deepcopy(self.raw)
        raw["files"].append(raw["files"][0])
        with self.assertRaises(ValueError):
            self.render(raw)

    def test_rejects_unsafe_links_and_non_exact_commits(self):
        for url in ("javascript:alert(1)", "https://secret@example.com/repo", "https://example.com/\nrepo"):
            with self.assertRaises(ValueError):
                self.render(run_url=url)
        with self.assertRaises(ValueError):
            self.render(commit="main")

    def test_cli_outputs_safe_standalone_artifacts(self):
        raw_path = self.root / "raw.json"
        raw_path.write_text(json.dumps(self.raw))
        out = self.root / "out"
        malicious_label = '</script><img src=x onerror="alert(1)">'
        self.assertEqual(reporter.main([
            "--report", str(raw_path), "--root", str(self.root), "--out", str(out),
            "--commit", "a" * 40, "--repository", "https://example.com/owner/repo",
            "--configuration", "release", "--scope", malicious_label]), 0)
        page = (out / "index.html").read_text()
        self.assertNotIn(malicious_label, page)
        self.assertIn("\\u003c/script>", page)
        self.assertEqual(json.loads((out / "summary.json").read_text())["scope"], malicious_label)
        self.assertEqual({p.name for p in out.iterdir()},
                         {"index.html", "summary.json", "history.json", "summary.md", "coverage.svg"})


if __name__ == "__main__":
    unittest.main()
