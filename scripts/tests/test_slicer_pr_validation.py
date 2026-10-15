"""Unit tests for PR-head validation status and cleanup semantics."""

from pathlib import Path
import json
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_pr_validation  # noqa: E402


class ParsingTests(unittest.TestCase):
    def test_numbers_accept_repeated_and_comma_separated_values(self):
        self.assertEqual(slicer_pr_validation.parse_numbers(["1, 3", "8"]), [1, 3, 8])
        self.assertEqual(slicer_pr_validation.parse_numbers([["1", "3"], ["8"]]), [1, 3, 8])

    def test_numbers_reject_duplicates_and_zero(self):
        with self.assertRaises(slicer_pr_validation.ValidationError):
            slicer_pr_validation.parse_numbers(["1,1"])
        with self.assertRaises(slicer_pr_validation.ValidationError):
            slicer_pr_validation.parse_numbers(["0"])

    def test_empty_snapshot_and_non_exact_head_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            empty = root / "empty.json"
            empty.write_text("[]", encoding="utf-8")
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation.load_snapshot(empty)
            invalid = root / "invalid.json"
            invalid.write_text(json.dumps([{"number": 1, "headRefOid": "HEAD"}]), encoding="utf-8")
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation.load_snapshot(invalid)


class StatusTests(unittest.TestCase):
    def result(self, returncode, *, output="100% tests passed, 0 tests failed out of 3",
               timed_out=False):
        return slicer_pr_validation.CommandResult(["runner"], returncode, 0.01, output, "", timed_out)

    def test_empty_results_are_not_pass(self):
        self.assertEqual(slicer_pr_validation.classify_test(self.result(0, output="No tests were found"), 0),
                         "no_tests")
        self.assertEqual(
            slicer_pr_validation.classify_test_result(
                self.result(8, output="No tests were found!!!"), 0,
                no_tests=slicer_pr_validation.no_test_evidence("No tests were found!!!")),
            "no_tests",
        )

    def test_failure_timeout_and_success_statuses_are_distinct(self):
        self.assertEqual(slicer_pr_validation.classify_test(self.result(1), 3), "test_failed")
        self.assertEqual(slicer_pr_validation.classify_test(self.result(None, timed_out=True), 3),
                         "test_timeout")
        self.assertEqual(slicer_pr_validation.classify_test(self.result(0), 3), "pass")

    def test_skipped_tests_are_not_pass(self):
        result = self.result(0, output="100% tests passed, 0 tests failed out of 3\n2 tests skipped")
        self.assertEqual(slicer_pr_validation.skipped_test_evidence(result.stdout), 2)
        self.assertEqual(slicer_pr_validation.classify_test_result(result, 3, skipped=2),
                         "test_skipped")

    def test_junit_skips_are_not_hidden_by_clean_console_output(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = Path(temp) / "tests.xml"
            junit.write_text(
                '<testsuites><testsuite tests="3" skipped="1" /></testsuites>',
                encoding="utf-8",
            )
            self.assertEqual(slicer_pr_validation.skipped_test_evidence("", junit), 1)

    def test_baseline_failure_prevents_overall_pass(self):
        report = {"baseline": {"status": "no_tests"}, "heads": [{"status": "pass"}]}
        self.assertEqual(slicer_pr_validation._overall_status(report), ("no_tests", 2))

    def test_a_failed_head_is_not_a_fully_passing_run(self):
        report = {"baseline": {"status": "pass"}, "heads": [
            {"status": "pass"}, {"status": "build_failed"},
        ]}
        self.assertEqual(slicer_pr_validation._overall_status(report), ("heads_failed", 1))

    def test_invalid_head_ref_is_an_infrastructure_failure(self):
        report = {"baseline": {"status": "pass"}, "heads": [{"status": "invalid_ref"}]}
        self.assertEqual(slicer_pr_validation._overall_status(report),
                         ("infrastructure_error", 2))


class CleanupTests(unittest.TestCase):
    def test_reset_operations_are_recorded_and_do_not_touch_original_checkout(self):
        with tempfile.TemporaryDirectory() as temp:
            original = Path(temp) / "original"
            original.mkdir()
            marker = original / "marker.txt"
            marker.write_text("untouched", encoding="utf-8")
            # The helper only needs a real worktree; this also exercises the
            # command shape used before each exact head.
            subprocess.run(["git", "init", "-q"], cwd=original, check=True)
            subprocess.run(["git", "config", "user.email", "test@example.invalid"], cwd=original, check=True)
            subprocess.run(["git", "config", "user.name", "Test"], cwd=original, check=True)
            subprocess.run(["git", "add", "marker.txt"], cwd=original, check=True)
            subprocess.run(["git", "commit", "-qm", "init"], cwd=original, check=True)
            worktree = Path(temp) / "worktree"
            added = slicer_pr_validation._git(original, "worktree", "add", "--detach", str(worktree), "HEAD")
            self.assertEqual(added.returncode, 0)
            operations = slicer_pr_validation._reset_worktree(worktree, "HEAD")
            self.assertEqual(len(operations), 2)
            removed = slicer_pr_validation._git(original, "worktree", "remove", "--force", str(worktree))
            self.assertEqual(removed.returncode, 0)
            self.assertEqual(marker.read_text(encoding="utf-8"), "untouched")

    def test_invalid_commit_ref_is_reported_without_guessing(self):
        with tempfile.TemporaryDirectory() as temp:
            original = Path(temp) / "original"
            original.mkdir()
            subprocess.run(["git", "init", "-q"], cwd=original, check=True)
            subprocess.run(["git", "config", "user.email", "test@example.invalid"], cwd=original, check=True)
            subprocess.run(["git", "config", "user.name", "Test"], cwd=original, check=True)
            (original / "marker.txt").write_text("marker", encoding="utf-8")
            subprocess.run(["git", "add", "marker.txt"], cwd=original, check=True)
            subprocess.run(["git", "commit", "-qm", "init"], cwd=original, check=True)
            resolved, result = slicer_pr_validation.resolve_commit(original, "does-not-exist")
            self.assertIsNone(resolved)
            self.assertNotEqual(result.returncode, 0)

    def test_external_build_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "worktree"
            root.mkdir()
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._resolve_build_path(root, str(Path(temp) / "outside"))


class CommandTests(unittest.TestCase):
    def test_binary_stdin_is_supported_and_decoded_for_reports(self):
        result = slicer_pr_validation.run_command(
            [sys.executable, "-c", "import sys; sys.stdout.buffer.write(sys.stdin.buffer.read())"],
            cwd=Path.cwd(), input_data=b"payload",
        )
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "payload")


if __name__ == "__main__":
    unittest.main()
