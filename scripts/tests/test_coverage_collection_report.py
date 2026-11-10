"""Independent suite baselines, collection validity, and safe offline packaging."""
import copy
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("coverage_collection_report", SCRIPTS / "coverage_collection_report.py")
reporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reporter)
history_spec = importlib.util.spec_from_file_location(
    "collection_history", SCRIPTS.parent / ".github/scripts/coverage_history.py")
history = importlib.util.module_from_spec(history_spec)
history_spec.loader.exec_module(history)


class CollectionReportTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.collection = self.root / "input"
        self.collection.mkdir()
        self.index = {"schema": "orca-slicer-coverage-collection", "schema_version": 1,
                      "measurement_mode": "per-executable", "status": "pass", "integrity_status": "pass",
                      "suites": []}
        for number, name in enumerate(reporter.SUITES):
            directory = self.collection / "suites" / name
            directory.mkdir(parents=True)
            metric = {"covered": number + 1, "count": 10}
            raw = {"status": "pass", "integrity_status": "pass", "backend": "llvm",
                   "schema_version": 1, "tool_versions": {"clang": "18"},
                   "objects": [f"/build/{name}"],
                   "provenance": {"git": {"head": "a" * 40,
                                          "working_tree": {"tracked_changes": False}}},
                   "summary": {"lines": metric},
                   "files": [{"filename": str(self.root / "src/core.cpp"), "summary": {"lines": metric}}]}
            (directory / "coverage-report.json").write_text(json.dumps(raw))
            self.index["suites"].append({"name": name, "executable_name": name,
                                         "report": f"suites/{name}/coverage-report.json",
                                         "status": "pass", "integrity_status": "pass"})
        self.save_index()

    def save_index(self):
        (self.collection / "coverage-collection.json").write_text(json.dumps(self.index))

    def render(self, name="site", previous=None, extra=()):
        output = self.root / name
        args = ["--collection", str(self.collection / "coverage-collection.json"),
                "--root", str(self.root), "--out", str(output), "--commit", "a" * 40,
                "--repository", "https://example.com/owner/repo", "--configuration", "ILLUSTRATIVE TEST DATA",
                "--measured-at", "2026-11-10T12:00:00Z" if previous else "2026-11-09T12:00:00Z"]
        if previous:
            args += ["--previous", str(previous)]
        self.assertEqual(reporter.main([*args, *extra]), 0)
        return output, json.loads((output / "summary.json").read_text())

    def test_histories_remain_independent_and_metrics_are_not_summed(self):
        first, baseline = self.render()
        _, current = self.render("next", previous=first)
        self.assertTrue(current["valid"])
        self.assertNotIn("summary", current)
        self.assertEqual(current["configuration_id"], baseline["configuration_id"])
        identifiers = set()
        for name, suite in current["suites"].items():
            identifiers.add(suite["configuration_id"])
            self.assertEqual(len(suite["history"]), 2)
            self.assertTrue(all(record["configuration_id"] == suite["configuration_id"] for record in suite["history"]))
            self.assertEqual(suite["summary"]["lines"], baseline["suites"][name]["summary"]["lines"])
        self.assertEqual(len(identifiers), len(reporter.SUITES))

    def test_every_canonical_suite_is_required_for_rendering_and_publication(self):
        output, summary = self.render()
        observations = json.loads((output / "history.json").read_text())
        for name in reporter.SUITES:
            incomplete = copy.deepcopy(self.index)
            incomplete["suites"] = [suite for suite in incomplete["suites"] if suite["name"] != name]
            with self.assertRaises(ValueError):
                reporter.suite_paths(incomplete, self.collection)
            changed = copy.deepcopy(summary)
            changed_history = copy.deepcopy(observations)
            del changed["suites"][name]
            del changed_history["suites"][name]
            with self.assertRaises(ValueError):
                history.validate(changed, changed_history, "a" * 40)
        changed = copy.deepcopy(summary)
        changed_history = copy.deepcopy(observations)
        changed["suites"]["extra_tests"] = copy.deepcopy(changed["suites"]["fff_print_tests"])
        changed_history["suites"]["extra_tests"] = copy.deepcopy(changed_history["suites"]["fff_print_tests"])
        with self.assertRaises(ValueError):
            history.validate(changed, changed_history, "a" * 40)

    def test_metadata_restores_one_complete_revision_and_rejects_cross_series_history(self):
        output, summary = self.render()
        observations = json.loads((output / "history.json").read_text())
        restored = self.root / "restored"
        history.restore(restored, summary, observations)
        for name, report in summary["suites"].items():
            self.assertEqual(json.loads((restored / "suites" / name / "summary.json").read_text()), report)
            self.assertEqual(json.loads((restored / "suites" / name / "history.json").read_text()), observations["suites"][name])
        mixed = copy.deepcopy(observations)
        mixed["suites"]["fff_print_tests"] = observations["suites"]["libslic3r_tests"]
        untouched = self.root / "rejected"
        with self.assertRaises(ValueError):
            history.restore(untouched, summary, mixed)
        self.assertFalse(untouched.exists())

    def test_collection_rejects_mixed_source_or_run_even_when_every_leaf_is_valid(self):
        output, summary = self.render()
        observations = json.loads((output / "history.json").read_text())
        for field, value in (("run_url", "https://example.com/other-run"),
                             ("commit", "b" * 40), ("configuration", "other-build")):
            changed = copy.deepcopy(summary)
            changed["suites"]["fff_print_tests"][field] = value
            with self.assertRaises(ValueError):
                history.validate(changed, observations, "a" * 40)

    def test_failed_suite_prevents_complete_collection(self):
        path = self.collection / "suites/fff_print_tests/coverage-report.json"
        raw = json.loads(path.read_text())
        raw.update(status="coverage_integrity_error", integrity_status="error")
        path.write_text(json.dumps(raw))
        self.index.update(status="coverage_integrity_error", integrity_status="error")
        self.save_index()
        output, summary = self.render()
        self.assertFalse(summary["valid"])
        self.assertEqual(summary["status"], "incomplete")
        self.assertEqual(summary["suites"]["fff_print_tests"]["history"], [])
        self.assertIn("Incomplete collection", (output / "index.html").read_text())

    def test_unsafe_duplicate_or_escaping_suite_paths_are_rejected(self):
        for mutation in (lambda value: value["suites"].append(copy.deepcopy(value["suites"][0])),
                         lambda value: value["suites"][0].update(name="../outside"),
                         lambda value: value["suites"][0].update(report="../outside.json"),
                         lambda value: value["suites"][0].update(executable_name="other_tests")):
            changed = copy.deepcopy(self.index)
            mutation(changed)
            with self.assertRaises(ValueError):
                reporter.suite_paths(changed, self.collection)

    def test_cross_wired_executable_cannot_seed_a_different_suite(self):
        path = self.collection / "suites/fff_print_tests/coverage-report.json"
        raw = json.loads(path.read_text())
        raw["objects"] = ["/build/libslic3r_tests"]
        path.write_text(json.dumps(raw))
        with self.assertRaises(SystemExit) as error:
            self.render()
        self.assertEqual(error.exception.code, 2)

    def test_annotation_budget_applies_before_copy(self):
        source = self.root / "annotations"
        source.mkdir()
        (source / "index.html").write_text("x" * 20)
        with self.assertRaises(ValueError):
            reporter.copy_annotations(source, self.root / "copied", 10, 10)
        self.assertFalse((self.root / "copied").exists())

    def test_symlinked_annotations_are_rejected(self):
        directory = self.collection / "suites/libslic3r_tests/html"
        directory.mkdir()
        (directory / "source.html").symlink_to(self.collection / "coverage-collection.json")
        with self.assertRaises(SystemExit) as error:
            self.render()
        self.assertEqual(error.exception.code, 2)

    def test_annotations_and_links_remain_with_their_suite_and_labels_are_escaped(self):
        for suite in self.index["suites"]:
            directory = self.collection / "suites" / suite["name"] / "html"
            directory.mkdir()
            (directory / "index.html").write_text("<!doctype html><title>Illustrative native fixture</title>")
        output, summary = self.render(extra=("--configuration", "</p><script>alert(1)</script>"))
        page = (output / "index.html").read_text()
        self.assertNotIn("</p><script>alert(1)</script>", page)
        self.assertIn("&lt;script&gt;alert(1)&lt;/script&gt;", page)
        for name, suite in summary["suites"].items():
            self.assertIn(f'suites/{name}/index.html', page)
            self.assertEqual(suite["annotation_index"], "source/index.html")
            self.assertTrue((output / "suites" / name / "source/index.html").is_file())


if __name__ == "__main__":
    unittest.main()
