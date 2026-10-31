"""Unit tests for PR-head validation status and cleanup semantics."""

from pathlib import Path
from argparse import Namespace
import hashlib
import json
import os
import shlex
import shutil
import subprocess
import sys
import tempfile
import time
import threading
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_pr_validation  # noqa: E402


class ParsingTests(unittest.TestCase):
    def test_explicit_tmpdir_must_be_an_existing_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing = root / "missing"
            regular_file = root / "file"
            regular_file.write_text("not a directory", encoding="utf-8")
            for invalid in (missing, regular_file):
                with self.subTest(path=invalid), \
                     mock.patch.dict(os.environ, {"TMPDIR": str(invalid)}, clear=True), \
                     self.assertRaisesRegex(
                         slicer_pr_validation.ValidationError,
                         "TMPDIR must name an existing directory",
                     ):
                    slicer_pr_validation._validate_explicit_tmpdir()

    def test_empty_explicit_tmpdir_is_rejected(self):
        with mock.patch.dict(os.environ, {"TMPDIR": ""}, clear=True), \
             self.assertRaisesRegex(
                 slicer_pr_validation.ValidationError,
                 "TMPDIR is set but empty",
             ):
            slicer_pr_validation._validate_explicit_tmpdir()

    def test_existing_relative_tmpdir_is_rejected(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temp:
            relative = os.path.relpath(temp, Path.cwd())
            with tempfile.TemporaryFile(dir=relative):
                pass
            with mock.patch.dict(os.environ, {"TMPDIR": relative}, clear=True), \
                 self.assertRaisesRegex(
                     slicer_pr_validation.ValidationError,
                     "TMPDIR must be an absolute path",
                 ):
                slicer_pr_validation._validate_explicit_tmpdir()

    def test_explicit_tmpdir_must_allow_temporary_file_creation(self):
        with tempfile.TemporaryDirectory() as temp, \
             mock.patch.dict(os.environ, {"TMPDIR": temp}, clear=True), \
             mock.patch.object(slicer_pr_validation.tempfile, "TemporaryFile",
                               side_effect=PermissionError("write denied")), \
             self.assertRaisesRegex(
                 slicer_pr_validation.ValidationError,
                 "TMPDIR is not usable for temporary files",
             ):
            slicer_pr_validation._validate_explicit_tmpdir()

    def test_valid_explicit_tmpdir_is_probed_without_leaving_files(self):
        with tempfile.TemporaryDirectory() as temp, \
             mock.patch.dict(os.environ, {"TMPDIR": temp}, clear=True):
            slicer_pr_validation._validate_explicit_tmpdir()
            self.assertEqual(list(Path(temp).iterdir()), [])

    def test_unset_tmpdir_keeps_the_platform_default(self):
        with mock.patch.dict(os.environ, {}, clear=True), \
             mock.patch.object(slicer_pr_validation.tempfile, "TemporaryFile") as probe:
            slicer_pr_validation._validate_explicit_tmpdir()
        probe.assert_not_called()

    def test_explicit_ccache_configpath_rejects_missing_and_nonfile_targets(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing = root / "missing.conf"
            directory = root / "config-dir"
            directory.mkdir()
            for invalid in (missing, directory):
                with self.subTest(path=invalid), \
                     mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": str(invalid)}, clear=True), \
                     self.assertRaisesRegex(
                         slicer_pr_validation.ValidationError,
                         "CCACHE_CONFIGPATH must name an existing readable regular file",
                     ):
                    slicer_pr_validation._validate_explicit_ccache_configpath()

    def test_empty_explicit_ccache_configpath_is_rejected(self):
        with mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": ""}, clear=True), \
             self.assertRaisesRegex(
                 slicer_pr_validation.ValidationError,
                 "CCACHE_CONFIGPATH is set but empty",
             ):
            slicer_pr_validation._validate_explicit_ccache_configpath()

    def test_relative_explicit_ccache_configpath_is_rejected_even_if_file_exists(self):
        with tempfile.TemporaryDirectory(dir=Path.cwd()) as temp:
            config = Path(temp) / "ccache.conf"
            config.write_text("cache_dir = /tmp/ccache\n", encoding="utf-8")
            relative = os.path.relpath(config, Path.cwd())
            self.assertTrue(Path(relative).is_file())
            with mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": relative}, clear=True), \
                 self.assertRaisesRegex(
                     slicer_pr_validation.ValidationError,
                     "CCACHE_CONFIGPATH must be an absolute path",
                 ):
                slicer_pr_validation._validate_explicit_ccache_configpath()

    def test_valid_explicit_ccache_configpath_file_is_accepted(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = root / "ccache.conf"
            config.write_text("cache_dir = /tmp/ccache\n", encoding="utf-8")

            with mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": str(config)}, clear=True):
                slicer_pr_validation._validate_explicit_ccache_configpath()

    def test_valid_explicit_ccache_configpath_symlink_is_accepted(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            config = root / "ccache.conf"
            config.write_text("cache_dir = /tmp/ccache\n", encoding="utf-8")
            alias = root / "ccache-link.conf"
            try:
                alias.symlink_to(config)
            except (NotImplementedError, OSError) as exc:
                self.skipTest(f"symlinks are unavailable: {exc}")

            with mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": str(alias)}, clear=True):
                slicer_pr_validation._validate_explicit_ccache_configpath()

    def test_invalid_explicit_ccache_configpath_fails_before_validation_or_uv_resolution(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing = root / "missing.conf"
            nonfile = root / "config-dir"
            nonfile.mkdir()
            relative = root / "relative.conf"
            relative.write_text("cache_dir = /tmp/ccache\n", encoding="utf-8")
            relative_path = os.path.relpath(relative, Path.cwd())
            argv = ["--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused"]
            for invalid, expected_error in (
                (missing, "CCACHE_CONFIGPATH must name an existing readable regular file"),
                (nonfile, "CCACHE_CONFIGPATH must name an existing readable regular file"),
                (relative_path, "CCACHE_CONFIGPATH must be an absolute path"),
            ):
                with self.subTest(path=invalid), \
                     mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": str(invalid)}, clear=True), \
                     mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                     mock.patch.object(slicer_pr_validation, "_resolve_uv") as resolve_uv, \
                     mock.patch("sys.stderr") as stderr:
                    result = slicer_pr_validation.main(argv)
                self.assertEqual(result, 2)
                run.assert_not_called()
                resolve_uv.assert_not_called()
                self.assertIn(expected_error, "".join(str(call.args[0]) for call in stderr.write.call_args_list))

    def test_unset_ccache_configpath_reaches_validation_without_file_probe(self):
        argv = ["--harness-base", "base", "--harness-ref", "ref",
                "--deps-prefix", "/unused"]
        with mock.patch.dict(os.environ, {}, clear=True), \
             mock.patch.object(slicer_pr_validation, "_resolve_uv", return_value="uv"), \
             mock.patch.object(slicer_pr_validation, "run_validation", return_value=0) as run, \
             mock.patch.object(Path, "is_file") as is_file, \
             mock.patch.object(os, "access") as access:
            result = slicer_pr_validation.main(argv)
        self.assertEqual(result, 0)
        run.assert_called_once()
        is_file.assert_not_called()
        access.assert_not_called()

    def test_valid_explicit_ccache_configpath_reaches_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            config = Path(temp) / "ccache.conf"
            config.write_text("cache_dir = /tmp/ccache\n", encoding="utf-8")
            argv = ["--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused"]
            with mock.patch.dict(os.environ, {"CCACHE_CONFIGPATH": str(config)}, clear=True), \
                 mock.patch.object(slicer_pr_validation, "_resolve_uv", return_value="uv"), \
                 mock.patch.object(slicer_pr_validation, "run_validation", return_value=0) as run:
                result = slicer_pr_validation.main(argv)
            self.assertEqual(result, 0)
            run.assert_called_once()

    def test_invalid_explicit_tmpdir_fails_before_validation_or_uv_resolution(self):
        with tempfile.TemporaryDirectory() as temp:
            invalid = Path(temp) / "missing"
            argv = ["--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused"]
            with mock.patch.dict(os.environ, {"TMPDIR": str(invalid)}, clear=True), \
                 mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                 mock.patch.object(slicer_pr_validation, "_resolve_uv") as resolve_uv, \
                 mock.patch("sys.stderr") as stderr:
                result = slicer_pr_validation.main(argv)
            self.assertEqual(result, 2)
            run.assert_not_called()
            resolve_uv.assert_not_called()
            self.assertIn("TMPDIR", "".join(str(call.args[0]) for call in stderr.write.call_args_list))

    def test_unset_tmpdir_reaches_validation_without_a_probe(self):
        argv = ["--harness-base", "base", "--harness-ref", "ref",
                "--deps-prefix", "/unused"]
        with mock.patch.dict(os.environ, {}, clear=True), \
             mock.patch.object(slicer_pr_validation, "_resolve_uv", return_value="uv"), \
             mock.patch.object(slicer_pr_validation, "run_validation", return_value=0) as run, \
             mock.patch.object(slicer_pr_validation.tempfile, "TemporaryFile") as probe:
            result = slicer_pr_validation.main(argv)
        self.assertEqual(result, 0)
        run.assert_called_once()
        probe.assert_not_called()

    def test_test_regex_is_optional_and_preserved_by_parser(self):
        base = ["--harness-base", "base", "--harness-ref", "ref",
                "--deps-prefix", "/unused"]
        self.assertIsNone(slicer_pr_validation._parser().parse_args(base).test_regex)
        regex = "^(Alpha [case]|Beta)$"
        parsed = slicer_pr_validation._parser().parse_args([*base, "--test-regex", regex])
        self.assertEqual(parsed.test_regex, regex)

    def test_keep_and_reuse_worktree_options_are_mutually_exclusive(self):
        conflicting_options = (
            ("--keep-worktree", "--reuse-worktree-from-report", "prior-report.json"),
            ("--keep-worktree", "--seed-worktree", "/seed/repo"),
            ("--reuse-worktree-from-report", "prior-report.json",
             "--seed-worktree", "/seed/repo"),
        )
        with mock.patch("sys.stderr"):
            for options in conflicting_options:
                with self.subTest(options=options), self.assertRaises(SystemExit):
                    slicer_pr_validation._parser().parse_args([
                        "--harness-base", "base", "--harness-ref", "ref",
                        "--deps-prefix", "/unused", *options,
                    ])

    def test_empty_test_regex_fails_before_validation(self):
        for regex in ("", "   "):
            with self.subTest(regex=regex), \
                 mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                 mock.patch("sys.stderr"):
                result = slicer_pr_validation.main([
                    "--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused", "--test-regex", regex,
                ])
            self.assertEqual(result, 2)
            run.assert_not_called()

    def test_nonpositive_test_jobs_fail_before_validation(self):
        for jobs in ("0", "-1"):
            with self.subTest(jobs=jobs), \
                 mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                 mock.patch("sys.stderr"):
                result = slicer_pr_validation.main([
                    "--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused", "--test-jobs", jobs,
                ])
            self.assertEqual(result, 2)
            run.assert_not_called()

    def test_nonpositive_or_nonfinite_test_timeout_fails_before_validation(self):
        for timeout in ("0", "-1", "nan", "inf"):
            with self.subTest(timeout=timeout), \
                 mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                 mock.patch("sys.stderr"):
                result = slicer_pr_validation.main([
                    "--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused", "--test-timeout", timeout,
                ])
            self.assertEqual(result, 2)
            run.assert_not_called()

    def test_default_test_command_records_positive_ctest_timeout(self):
        command = slicer_pr_validation._default_test(
            Path("/tmp/build"), 8, Path("/tmp/tests.xml"), "NotWorking")
        self.assertEqual(command[command.index("--timeout") + 1], "180.0")
        custom = slicer_pr_validation._default_test(
            Path("/tmp/build"), 8, Path("/tmp/tests.xml"), "NotWorking", 37.5)
        self.assertEqual(custom[custom.index("--timeout") + 1], "37.5")

    def test_default_test_keeps_full_suite_and_label_exclusion(self):
        command = slicer_pr_validation._default_test(
            Path("/tmp/build"), 4, Path("/tmp/tests.xml"),
            "NotWorking|RequiresApp")
        self.assertEqual(command[command.index("-LE") + 1], "NotWorking|RequiresApp")
        self.assertNotIn("-R", command)

    def test_nonfinite_phase_timeout_fails_before_validation(self):
        for timeout in ("nan", "inf"):
            with self.subTest(timeout=timeout), \
                 mock.patch.object(slicer_pr_validation, "run_validation") as run, \
                 mock.patch("sys.stderr"):
                result = slicer_pr_validation.main([
                    "--harness-base", "base", "--harness-ref", "ref",
                    "--deps-prefix", "/unused", "--timeout", timeout,
                ])
            self.assertEqual(result, 2)
            run.assert_not_called()

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
            invalid.write_text(json.dumps([{"number": 1, "headRefOid": "HEAD",
                                            "baseRefOid": "a" * 40}]), encoding="utf-8")
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation.load_snapshot(invalid)

    def test_snapshot_requires_exact_base_oid(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing = root / "missing-base.json"
            missing.write_text(json.dumps([{
                "number": 1, "headRefOid": "a" * 40,
            }]), encoding="utf-8")
            with self.assertRaisesRegex(slicer_pr_validation.ValidationError, "baseRefOid"):
                slicer_pr_validation.load_snapshot(missing)
            invalid = root / "invalid-base.json"
            invalid.write_text(json.dumps([{
                "number": 1, "headRefOid": "a" * 40, "baseRefOid": "b" * 39,
            }]), encoding="utf-8")
            with self.assertRaisesRegex(slicer_pr_validation.ValidationError, "base OID"):
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

    def test_qualified_failed_baseline_keeps_overall_status_nonzero(self):
        report = {
            "configuration": {"reference_baseline": False},
            "baseline": {"status": "test_failed"},
            "baseline_expectation": {"enabled": True, "status": "qualified"},
            "heads": [{"status": "pass"}],
        }
        self.assertEqual(slicer_pr_validation._overall_status(report), ("test_failed", 2))

    def test_rejected_baseline_expectation_blocks_even_a_passing_raw_baseline(self):
        report = {
            "baseline": {"status": "pass"},
            "baseline_expectation": {"enabled": True, "status": "rejected"},
            "heads": [{"status": "pass"}],
        }
        self.assertEqual(
            slicer_pr_validation._overall_status(report),
            ("baseline_expectation_invalid", 2),
        )

    def test_policy_identity_presence_requires_previously_failed_case_to_execute(self):
        entry = {
            "status": "pass",
            "junit_evidence": {"counts": {"tests": 1}},
            "junit_case_statuses": {"suite::previous pass": "passed"},
        }
        slicer_pr_validation._apply_minimum_test_gate(
            entry, minimum_tests=1,
            required_identity_cases={
                "suite::previous pass": "passed",
                "suite::previous failure": "failed",
            })
        self.assertEqual(entry["missing_baseline_expectation_testcases"],
                         ["suite::previous failure"])
        self.assertEqual(entry["status"], "test_evidence_incomplete")
        self.assertFalse(entry["junit_reference_usable"])

    def test_baseline_failure_comparison_distinguishes_inherited_resolved_and_new(self):
        known = [f"suite::known {index}" for index in range(3)]
        reference_statuses = {name: "failed" for name in known}
        reference_statuses["suite::stable"] = "passed"
        head_statuses = {
            known[0]: "failed", known[1]: "passed", known[2]: "passed",
            "suite::stable": "failed",
        }
        reference = {
            "status": "test_failed", "junit_reference_usable": True,
            "junit_case_statuses": reference_statuses,
            "failed_test_names": known,
        }
        head = {
            "number": 17, "reference_base_sha": "base",
            "status": "test_failed", "junit_reference_usable": True,
            "missing_baseline_expectation_testcases": [],
            "junit_case_statuses": head_statuses,
            "failed_test_names": [known[0], "suite::stable"],
        }
        head["failure_classification"] = slicer_pr_validation.classify_failure_names(
            reference, head)
        self.assertEqual(head["failure_classification"]["inherited"], [known[0]])
        self.assertEqual(head["failure_classification"]["resolved"], known[1:])
        self.assertEqual(head["failure_classification"]["new"], ["suite::stable"])
        report = {
            "configuration": {"reference_baseline": True},
            "baseline_expectation": {"status": "qualified"},
            "references": {"base": reference},
            "heads": [head],
        }
        comparison = slicer_pr_validation._baseline_expectation_comparison(report)
        self.assertEqual(comparison["status"], "new_failing_testcases")
        self.assertEqual(comparison["new_failing_testcases_by_head"],
                         {"17": ["suite::stable"]})

        head["junit_case_statuses"].pop(known[2])
        head["missing_baseline_expectation_testcases"] = [known[2]]
        comparison = slicer_pr_validation._baseline_expectation_comparison(report)
        self.assertEqual(comparison["status"], "incomplete")

    def test_no_new_failure_comparison_can_still_retain_raw_failures(self):
        reference = {
            "status": "test_failed", "junit_reference_usable": True,
            "junit_case_statuses": {"suite::old": "failed"},
            "failed_test_names": ["suite::old"],
        }
        head = {
            "number": 2, "reference_base_sha": "base", "status": "test_failed",
            "junit_reference_usable": True, "missing_baseline_expectation_testcases": [],
            "junit_case_statuses": {"suite::old": "failed"},
            "failed_test_names": ["suite::old"],
        }
        head["failure_classification"] = slicer_pr_validation.classify_failure_names(
            reference, head)
        report = {
            "configuration": {"reference_baseline": True},
            "baseline_expectation": {"status": "qualified"},
            "references": {"base": reference}, "heads": [head],
        }
        self.assertEqual(
            slicer_pr_validation._baseline_expectation_comparison(report)["status"],
            "no_new_failing_testcases",
        )

    def test_a_failed_head_is_not_a_fully_passing_run(self):
        report = {"baseline": {"status": "pass"}, "heads": [
            {"status": "pass"}, {"status": "build_failed"},
        ]}
        self.assertEqual(slicer_pr_validation._overall_status(report), ("heads_failed", 1))

    def test_invalid_head_ref_is_an_infrastructure_failure(self):
        report = {"baseline": {"status": "pass"}, "heads": [{"status": "invalid_ref"}]}
        self.assertEqual(slicer_pr_validation._overall_status(report),
                         ("infrastructure_error", 2))

    def test_incomplete_head_statuses_are_infrastructure_failures(self):
        for status in ("test_timeout", "test_skipped", "no_tests"):
            with self.subTest(status=status):
                report = {
                    "baseline": {"status": "pass"},
                    "heads": [{"status": status}],
                }
                self.assertEqual(
                    slicer_pr_validation._overall_status(report),
                    ("infrastructure_error", 2),
                )

    def test_requested_reference_accepts_only_comparable_statuses(self):
        for status in ("pass", "test_failed"):
            with self.subTest(status=status):
                report = {
                    "configuration": {"reference_baseline": True},
                    "baseline": {"status": "pass"},
                    "reference": {"status": status},
                    "heads": [{"status": "pass"}],
                }
                self.assertEqual(slicer_pr_validation._overall_status(report), ("pass", 0))

    def test_requested_reference_missing_or_error_status_is_infrastructure_failure(self):
        statuses = (None, "not_run", "tracked_source_index_changed",
                    "tracked_source_index_resolution_failed", "infrastructure_error",
                    "test_evidence_incomplete")
        for status in statuses:
            with self.subTest(status=status):
                report = {
                    "configuration": {"reference_baseline": True},
                    "baseline": {"status": "pass"},
                    "heads": [{"status": "pass"}],
                }
                if status is not None:
                    report["reference"] = {"status": status}
                self.assertEqual(slicer_pr_validation._overall_status(report),
                                 ("infrastructure_error", 2))

    def test_disabled_reference_does_not_require_reference_evidence(self):
        report = {
            "configuration": {"reference_baseline": False},
            "baseline": {"status": "pass"},
            "reference": {"status": "not_run"},
            "heads": [{"status": "pass"}],
        }
        self.assertEqual(slicer_pr_validation._overall_status(report), ("pass", 0))

    def test_multiple_reference_bases_fail_closed_when_one_is_not_comparable(self):
        report = {
            "baseline": {"status": "pass"},
            "configuration": {"reference_baseline": True},
            "references": {
                "a" * 40: {"status": "pass"},
                "b" * 40: {"status": "test_error"},
            },
            "heads": [{"status": "pass"}],
        }
        self.assertEqual(slicer_pr_validation._overall_status(report),
                         ("infrastructure_error", 2))

    def test_legacy_report_without_reference_request_keeps_existing_behavior(self):
        report = {"baseline": {"status": "pass"}, "heads": [{"status": "pass"}]}
        self.assertEqual(slicer_pr_validation._overall_status(report), ("pass", 0))

    def test_reference_failures_are_classified_without_suppressing_head(self):
        reference = {
            "junit_exists": True,
            "junit_reference_usable": True,
            "status": "test_failed",
            "junit_case_statuses": {
                "suite::known base regression": "failed",
                "suite::resolved on PR": "failed",
            },
            "failed_test_names": ["suite::known base regression", "suite::resolved on PR"],
        }
        head = {
            "status": "pass",
            "junit_reference_usable": True,
            "junit_case_statuses": {
                "suite::known base regression": "passed",
                "suite::resolved on PR": "passed",
            },
            "failed_test_names": [],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertEqual(classification["inherited"], [])
        self.assertEqual(classification["new"], [])
        self.assertEqual(classification["resolved"], [
            "suite::known base regression", "suite::resolved on PR",
        ])
        report = {"baseline": {"status": "pass"}, "reference": reference,
                  "heads": [head]}
        self.assertEqual(slicer_pr_validation._overall_status(report), ("pass", 0))

    def test_missing_reference_evidence_is_not_reported_as_inherited(self):
        classification = slicer_pr_validation.classify_failure_names(
            {"status": "test_timeout", "junit_exists": False},
            {"status": "test_failed", "failed_test_names": ["head failure"]})
        self.assertFalse(classification["reference_available"])
        self.assertEqual(classification["new"], [])
        self.assertEqual(classification["uncompared_head_failures"], ["head failure"])
        self.assertIn("cannot establish a shared root cause", classification["classification_basis"])
        self.assertIn("unavailable", classification["reason"])

    def test_malformed_reference_junit_cannot_claim_new_failure(self):
        classification = slicer_pr_validation.classify_failure_names(
            {"status": "test_failed", "junit_exists": True,
             "junit_reference_usable": False, "junit_case_statuses": {}},
            {"status": "test_failed", "failed_test_names": ["suite::new"]})
        self.assertFalse(classification["reference_available"])
        self.assertEqual(classification["new"], [])
        self.assertEqual(classification["raw_observed_head_failures"], ["suite::new"])
        self.assertEqual(classification["uncompared_head_failures"], ["suite::new"])

    def test_differential_classification_requires_case_execution(self):
        reference = {
            "junit_reference_usable": True,
            "junit_case_statuses": {
                "suite::inherited": "failed",
                "suite::resolved": "failed",
                "suite::new": "passed",
                "suite::reference_only": "failed",
                "suite::skipped_reference": "failed",
                "suite::reference_skipped": "skipped",
                "suite::reference_unknown": "unknown",
            },
            "failed_test_names": [
                "suite::inherited", "suite::resolved", "suite::reference_only",
            ],
        }
        head = {
            "status": "test_failed",
            "junit_reference_usable": True,
            "junit_case_statuses": {
                "suite::inherited": "failed",
                "suite::resolved": "passed",
                "suite::new": "failed",
                "suite::skipped": "skipped",
                "suite::skipped_reference": "skipped",
            },
            "failed_test_names": [
                "suite::inherited", "suite::new",
                "suite::reference_skipped", "suite::reference_unknown",
            ],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertEqual(classification["inherited"], ["suite::inherited"])
        self.assertEqual(classification["new"], ["suite::new"])
        self.assertEqual(classification["resolved"], ["suite::resolved"])
        self.assertEqual(classification["uncompared_head_failures"], [
            "suite::reference_skipped", "suite::reference_unknown",
        ])
        self.assertEqual(classification["not_executed_reference_failures"],
                         ["suite::reference_only", "suite::skipped_reference"])
        self.assertIn("matching failure names do not", classification["classification_basis"])

    def test_execution_error_identities_have_separate_differential_buckets(self):
        reference = {
            "status": "test_error",
            "junit_error_comparison_usable": True,
            "junit_case_statuses": {
                "suite::shared crash": "error",
                "suite::resolved crash": "error",
                "suite::shared assertion": "failed",
                "suite::head-only crash": "passed",
            },
            "failed_test_names": ["suite::shared assertion"],
            "error_test_names": ["suite::shared crash", "suite::resolved crash"],
        }
        head = {
            "status": "test_error",
            "junit_error_comparison_usable": True,
            "junit_case_statuses": {
                "suite::shared crash": "error",
                "suite::resolved crash": "passed",
                "suite::shared assertion": "failed",
                "suite::head-only crash": "error",
            },
            "failed_test_names": ["suite::shared assertion"],
            "error_test_names": ["suite::shared crash", "suite::head-only crash"],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertTrue(classification["reference_available"])
        self.assertEqual(classification["inherited"], ["suite::shared assertion"])
        self.assertEqual(classification["inherited_errors"], ["suite::shared crash"])
        self.assertEqual(classification["new_errors"], ["suite::head-only crash"])
        self.assertEqual(classification["resolved_errors"], ["suite::resolved crash"])
        self.assertEqual(classification["uncompared_head_errors"], [])

    def test_unusable_error_evidence_cannot_claim_new_or_resolved_crashes(self):
        reference = {
            "status": "test_error",
            "junit_reference_usable": False,
            "junit_error_comparison_usable": False,
            "junit_case_statuses": {"suite::reference crash": "error"},
            "error_test_names": ["suite::reference crash"],
        }
        head = {
            "status": "test_error",
            "junit_error_comparison_usable": True,
            "junit_case_statuses": {"suite::head crash": "error"},
            "error_test_names": ["suite::head crash"],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertFalse(classification["reference_available"])
        self.assertEqual(classification["inherited_errors"], [])
        self.assertEqual(classification["new_errors"], [])
        self.assertEqual(classification["resolved_errors"], [])
        self.assertEqual(classification["raw_observed_head_errors"], ["suite::head crash"])
        self.assertEqual(classification["uncompared_head_errors"], ["suite::head crash"])
        self.assertEqual(
            slicer_pr_validation._overall_status({
                "baseline": {"status": "pass"},
                "heads": [{"status": "test_error"}],
            }),
            ("infrastructure_error", 2),
        )
        head_unusable = dict(head)
        head_unusable["junit_error_comparison_usable"] = False
        comparison = slicer_pr_validation.classify_failure_names(
            {**reference, "junit_error_comparison_usable": True}, head_unusable)
        self.assertFalse(comparison["reference_available"])
        self.assertEqual(comparison["new_errors"], [])
        self.assertEqual(comparison["resolved_errors"], [])

    def test_complete_error_junit_is_comparable_but_status_remains_fail_closed(self):
        entry = {
            "status": "test_error",
            "junit_evidence": {
                "usable": True,
                "counts": {"tests": 2, "errors": 1, "timeouts": 0, "skipped": 0},
            },
            "minimum_tests_ok": True,
            "minimum_tests_observed": 2,
        }
        slicer_pr_validation._apply_minimum_test_gate(entry, minimum_tests=2)
        self.assertTrue(entry["junit_error_comparison_usable"])
        self.assertFalse(entry["junit_reference_usable"])
        self.assertEqual(
            slicer_pr_validation._overall_status({
                "baseline": {"status": "pass"},
                "heads": [{"status": entry["status"]}],
            }),
            ("infrastructure_error", 2),
        )
        incomplete = {
            "status": "test_error",
            "junit_evidence": {
                "usable": True,
                "counts": {"tests": 1, "errors": 1, "timeouts": 0, "skipped": 0},
            },
        }
        slicer_pr_validation._apply_minimum_test_gate(incomplete, minimum_tests=2)
        self.assertFalse(incomplete["junit_error_comparison_usable"])

    def test_build_failure_cannot_claim_reference_failure_resolved(self):
        reference = {
            "junit_reference_usable": True,
            "junit_case_statuses": {"suite::known": "failed"},
            "failed_test_names": ["suite::known"],
        }
        head = {
            "status": "build_failed",
            "junit_exists": False,
            "junit_case_statuses": {},
            "failed_test_names": [],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertEqual(classification["resolved"], [])
        self.assertEqual(classification["not_executed_reference_failures"], ["suite::known"])

    def test_duplicate_junit_names_are_qualified_and_never_overwritten(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = Path(temp) / "duplicate.xml"
            junit.write_text(
                "<testsuites><testsuite><testcase classname='a' name='same'>"
                "<failure/></testcase><testcase classname='a' name='same'/>"
                "</testsuite></testsuites>", encoding="utf-8")
            # Duplicate identities are deliberately non-comparable now; the
            # parser returns no cases rather than assigning order-sensitive
            # #2/#3 suffixes.
            self.assertEqual(slicer_pr_validation.junit_case_statuses(junit), {})

    def test_head_junit_evidence_is_required_for_differential_classification(self):
        reference = {
            "junit_reference_usable": True,
            "junit_case_statuses": {"suite::new": "passed"},
            "failed_test_names": [],
        }
        head = {
            "status": "test_failed",
            "junit_reference_usable": False,
            "junit_case_statuses": {"suite::new": "failed"},
            "failed_test_names": ["suite::new"],
        }
        classification = slicer_pr_validation.classify_failure_names(reference, head)
        self.assertFalse(classification["reference_available"])
        self.assertEqual(classification["new"], [])
        self.assertEqual(classification["uncompared_head_failures"], ["suite::new"])


def _minimum_junit(names):
    cases = "".join(
        f'<testcase classname="Harness" name="{name}" />' for name in names)
    return (f'<testsuite tests="{len(names)}" failures="0" errors="0" skipped="0">'
            f"{cases}</testsuite>")


class MinimumTestIntegrationTests(unittest.TestCase):
    """Exercise the count and identity gates through the real _run_tree path."""

    @staticmethod
    def _args(minimum_tests=1, test_regex=None):
        return Namespace(
            test_timeout=180.0,
            timeout=30.0,
            minimum_tests=minimum_tests,
            build_dir="build",
            jobs=1,
            test_jobs=1,
            exclude_labels="",
            test_regex=test_regex,
            deps_prefix="deps",
            uv="uv",
        )

    def _run_tree(self, *, minimum_tests, names, build_result=None, test_regex=None):
        with tempfile.TemporaryDirectory() as temp:
            root_output = Path(temp) / "output"
            worktree = Path(temp) / "worktree"
            worktree.mkdir()

            def fake_default_test(build, jobs, junit, excluded, timeout, regex=None):
                junit.write_text(_minimum_junit(names), encoding="utf-8")
                command = ["ctest", "--output-junit", str(junit)]
                if regex is not None:
                    command.extend(["-R", regex])
                return command

            def fake_phase(argv, *, cwd, timeout, log):
                log.parent.mkdir(parents=True, exist_ok=True)
                is_test = argv and argv[0] == "ctest"
                if build_result is not None and not is_test \
                        and not hasattr(fake_phase, "configured"):
                    fake_phase.configured = True
                    result = slicer_pr_validation.CommandResult(
                        list(argv), 0, 0.01, "", "")
                elif build_result is not None and not is_test:
                    result = build_result
                else:
                    result = slicer_pr_validation.CommandResult(
                        list(argv), 0, 0.01,
                        f"100% tests passed, 0 tests failed out of {len(names)}" if is_test else "",
                        "")
                log.write_text(result.stdout + result.stderr, encoding="utf-8")
                return result

            with mock.patch.object(slicer_pr_validation, "_default_test",
                                   side_effect=fake_default_test), \
                 mock.patch.object(slicer_pr_validation, "_run_phase",
                                   side_effect=fake_phase):
                return slicer_pr_validation._run_tree(
                    worktree, root_output, "baseline",
                    self._args(minimum_tests, test_regex),
                    ["cmake", "configure"], ["cmake", "build"],
                    commit_sha="a" * 40, tree_sha="b" * 40,
                )

    def test_test_regex_braces_reach_ctest_literally_and_minimum_gates_selected_cases(self):
        regex = r"^(Alpha{2}|\{root\})$"
        entry = self._run_tree(minimum_tests=2, names=["Alpha"], test_regex=regex)
        self.assertEqual(entry["status"], "test_evidence_incomplete")
        self.assertEqual(entry["minimum_tests_observed"], 1)
        argv = entry["commands"]["test"]["argv"]
        self.assertEqual(argv[argv.index("-R") + 1], regex)

    def test_count_floor_is_applied_after_junit_parsing(self):
        entry = self._run_tree(minimum_tests=4, names=["one", "two", "three"])
        self.assertEqual(entry["status"], "test_evidence_incomplete")
        self.assertEqual(entry["minimum_tests_required"], 4)
        self.assertEqual(entry["minimum_tests_observed"], 3)
        self.assertFalse(entry["minimum_tests_ok"])
        self.assertFalse(entry["junit_reference_usable"])

    def test_equal_count_replacement_fails_identity_gate(self):
        entry = self._run_tree(minimum_tests=3, names=["one", "replacement", "three"])
        self.assertEqual(entry["status"], "pass")
        slicer_pr_validation._apply_minimum_test_gate(
            entry, minimum_tests=3,
            baseline_cases={
                "Harness::one": "passed", "Harness::two": "passed",
                "Harness::three": "passed",
            })
        self.assertEqual(entry["status"], "test_evidence_incomplete")
        self.assertEqual(entry["missing_baseline_testcases"], ["Harness::two"])
        self.assertFalse(entry["junit_reference_usable"])

    def test_complete_identity_set_preserves_comparable_failure(self):
        entry = self._run_tree(minimum_tests=3, names=["one", "two", "three"])
        entry["status"] = "test_failed"
        slicer_pr_validation._apply_minimum_test_gate(
            entry, minimum_tests=3,
            baseline_cases={
                "Harness::one": "passed", "Harness::two": "passed",
                "Harness::three": "passed",
            })
        self.assertEqual(entry["status"], "test_failed")
        self.assertTrue(entry["junit_reference_usable"])
        self.assertEqual(entry["missing_baseline_testcases"], [])

    def test_build_timeout_is_preserved_and_floor_is_recorded(self):
        timed_out = slicer_pr_validation.CommandResult(
            ["cmake"], None, 0.01, "partial", "", True)
        entry = self._run_tree(minimum_tests=3212, names=[], build_result=timed_out)
        self.assertEqual(entry["status"], "build_timeout")
        self.assertEqual(entry["minimum_tests_required"], 3212)
        self.assertNotIn("minimum_tests_ok", entry)

    def test_existing_test_timeout_is_not_hidden_by_identity_gate(self):
        entry = {
            "status": "test_timeout",
            "junit_evidence": {"counts": {"tests": 0}},
            "junit_case_statuses": {},
        }
        slicer_pr_validation._apply_minimum_test_gate(
            entry, minimum_tests=3212, baseline_cases={"Harness::one": "passed"})
        self.assertEqual(entry["status"], "test_timeout")

    def test_parser_default_explicit_value_and_rejection(self):
        parser = slicer_pr_validation._parser()
        common = ["--harness-base", "a" * 40, "--harness-ref", "b" * 40,
                  "--deps-prefix", "deps"]
        self.assertEqual(parser.parse_args(common).minimum_tests, 1)
        self.assertEqual(
            parser.parse_args(common + ["--minimum-tests", "3212"]).minimum_tests, 3212)
        self.assertEqual(slicer_pr_validation.main(common + ["--minimum-tests", "0"]), 2)


class IdentityManifestTests(unittest.TestCase):
    @staticmethod
    def _git(repo, *args):
        return subprocess.run(["git", *args], cwd=repo, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)

    def _repo_fixture(self, root):
        root.mkdir(parents=True, exist_ok=True)
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "test")
        (repo / "tests").mkdir()
        (repo / "tests/identity.cpp").write_text(
            'TEST_CASE("Original A", "[Identity]") {}\n'
            'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        with (repo / "tests/identity.cpp").open("a", encoding="utf-8") as stream:
            stream.write('TEST_CASE("New C", "[Identity]") {}\n')
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "harness")
        harness_ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        self._git(repo, "checkout", "-q", "-b", "head", base)
        (repo / "tests/identity.cpp").write_text(
            'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "old head")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        return repo, base, harness_ref, head

    @staticmethod
    def _case(declaration):
        return {
            "runtime_identity": declaration["name"] + "::" + declaration["name"],
            "source": declaration,
        }

    def _manifest(self, repo, base, harness_ref, head):
        base_declarations = slicer_pr_validation._git_test_declarations(repo, base)
        ref_declarations = slicer_pr_validation._git_test_declarations(repo, harness_ref)
        original_a = next(x for x in base_declarations.values() if x["name"] == "Original A")
        new_c = next(x for x in ref_declarations.values() if x["name"] == "New C")
        return {
            "schema_version": 2,
            "harness_base_sha": base,
            "harness_ref_sha": harness_ref,
            "required_harness_cases": [self._case(new_c)],
            "exception_groups": {"none": [], "legacy": [self._case(original_a)]},
            "heads": [{"number": 1, "head_oid": head, "exception_group": "legacy",
                       "required_head_cases": []}],
        }

    def test_exact_head_manifest_validates_absent_base_case_and_required_new_case(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness_ref, head = self._repo_fixture(Path(temp))
            manifest = self._manifest(repo, base, harness_ref, head)
            records = [{"number": 1, "headRefOid": head, "baseRefOid": base}]
            policy = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=records, selected_records=records)
            self.assertEqual(policy["required_harness_runtime_ids"], ["New C::New C"])
            self.assertEqual(
                list(policy["heads"][1]["exception_cases"]), ["Original A::Original A"])

    def test_manifest_rejects_new_case_as_exception_and_present_case_as_exception(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness_ref, head = self._repo_fixture(Path(temp))
            manifest = self._manifest(repo, base, harness_ref, head)
            new_c = manifest["required_harness_cases"][0]
            manifest["exception_groups"]["bad"] = [new_c]
            manifest["heads"][0]["exception_group"] = "bad"
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}],
                    selected_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}])

            manifest = self._manifest(repo, base, harness_ref, base)
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": base, "baseRefOid": base}],
                    selected_records=[{"number": 1, "headRefOid": base, "baseRefOid": base}])

    def test_scenario_runtime_prefix_and_escaped_names_are_fail_closed(self):
        scenario = {
            "runtime_identity": "suite::Scenario: Scenario name",
            "source": {"path": "tests/scenario.cpp", "macro": "SCENARIO",
                       "name": "Scenario name", "line": 9},
        }
        self.assertEqual(
            slicer_pr_validation._manifest_case_runtime_name(scenario, label="scenario"),
            "suite::Scenario: Scenario name")
        invalid_prefix = dict(scenario, runtime_identity="suite::Scenario name")
        with self.assertRaises(slicer_pr_validation.ValidationError):
            slicer_pr_validation._manifest_case_runtime_name(
                invalid_prefix, label="invalid_scenario")
        escaped = {
            "runtime_identity": r"literal \"1\"",
            "source": {"path": "tests/escaped.cpp", "macro": "TEST_CASE",
                       "name": r'literal \"1\"', "line": 4},
        }
        with self.assertRaisesRegex(slicer_pr_validation.ValidationError, r"escaped C\+\+ test name"):
            slicer_pr_validation._manifest_case_runtime_name(escaped, label="escaped")

    def test_required_case_relocation_by_name_is_not_novel(self):
        relocated = {
            "runtime_identity": "New C::New C",
            "source": {"path": "tests/relocated.cpp", "macro": "TEST_CASE",
                       "name": "New C", "line": 1},
        }
        declarations = {("tests/relocated.cpp", "TEST_CASE", "New C"):
                        relocated["source"]}
        with self.assertRaises(slicer_pr_validation.ValidationError):
            slicer_pr_validation._validate_manifest_cases(
                [relocated], label="required_harness_cases", expected_commit="ref",
                declarations=declarations, forbidden_names={"New C"})

    def test_exception_adjustment_keeps_new_cases_mandatory_and_skips_visible(self):
        entry = {
            "status": "test_skipped",
            "junit_evidence": {"counts": {"tests": 2}},
            "junit_case_statuses": {
                "Original B::Original B": "passed",
                "New C::New C": "skipped",
            },
        }
        baseline = {
            "Original A::Original A": "passed",
            "Original B::Original B": "passed",
            "New C::New C": "passed",
        }
        slicer_pr_validation._apply_minimum_test_gate(
            entry, minimum_tests=2,
            baseline_cases={name: status for name, status in baseline.items()
                            if name != "Original A::Original A"})
        self.assertEqual(entry["missing_baseline_testcases"], [])
        self.assertEqual(entry["skipped_baseline_testcases"], ["New C::New C"])
        self.assertEqual(entry["minimum_tests_required"], 2)
        self.assertEqual(entry["status"], "test_skipped")

    def test_duplicate_identity_is_explicitly_recorded_and_unusable(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = Path(temp) / "duplicate.xml"
            junit.write_text(
                "<testsuite tests='2' failures='0' errors='0' skipped='0'>"
                "<testcase classname='suite' name='same'/>"
                "<testcase classname='suite' name='same'/>"
                "</testsuite>", encoding="utf-8")
            evidence = slicer_pr_validation.read_junit_evidence(junit)
        self.assertFalse(evidence["usable"])
        self.assertEqual(evidence["duplicate_testcase_identities"], ["suite::same"])


class BaselineExpectationManifestTests(unittest.TestCase):
    @staticmethod
    def _manifest(statuses=None):
        return {
            "schema_version": 1,
            "harness_ref_sha": "a" * 40,
            "harness_ref_tree_sha": "b" * 40,
            "tracked_source_index_sha256": hashlib.sha256(b"tracked src").hexdigest(),
            "testcase_statuses": statuses or {
                "suite::ordinary pass": "passed",
                "suite::ordinary failure": "failed",
            },
        }

    def test_manifest_binds_exact_source_and_accepts_generic_status_counts(self):
        manifest = self._manifest()
        raw = json.dumps(manifest)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "baseline.json"
            path.write_text(raw, encoding="utf-8")
            loaded, digest = slicer_pr_validation._load_baseline_expectation_manifest(path)
        policy = slicer_pr_validation._validate_baseline_expectation_manifest(
            loaded, harness_ref="a" * 40, harness_ref_tree_sha="b" * 40)
        self.assertEqual(digest, hashlib.sha256(raw.encode("utf-8")).hexdigest())
        self.assertEqual(policy["testcase_count"], 2)
        self.assertEqual(policy["status_counts"], {"failed": 1, "passed": 1})

    def test_manifest_rejects_duplicate_keys_unknown_status_and_extra_fields(self):
        duplicate = (
            '{"schema_version":1,"harness_ref_sha":"' + "a" * 40 +
            '","harness_ref_tree_sha":"' + "b" * 40 +
            '","tracked_source_index_sha256":"' + hashlib.sha256(b"tracked src").hexdigest() +
            '","testcase_statuses":{"suite::one":"passed","suite::one":"failed"}}'
        )
        invalid_documents = [
            duplicate,
            json.dumps(self._manifest({"suite::one": "skipped"})),
            json.dumps(dict(self._manifest(), unrecognized=True)),
            json.dumps(self._manifest({"suite::one": "passed"})),
        ]
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "baseline.json"
            for index, document in enumerate(invalid_documents):
                with self.subTest(document=index):
                    path.write_text(document, encoding="utf-8")
                    with self.assertRaises(slicer_pr_validation.ValidationError):
                        slicer_pr_validation._load_baseline_expectation_manifest(path)

    def test_manifest_rejects_stale_commit_or_tree(self):
        manifest = self._manifest()
        for ref, tree in (("c" * 40, "b" * 40), ("a" * 40, "d" * 40)):
            with self.subTest(ref=ref, tree=tree):
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError, "stale"):
                    slicer_pr_validation._validate_baseline_expectation_manifest(
                        manifest, harness_ref=ref, harness_ref_tree_sha=tree)

    @staticmethod
    def _observed_baseline(statuses):
        counts = {
            "tests": len(statuses),
            "failures": sum(status == "failed" for status in statuses.values()),
            "errors": 0, "skipped": 0, "timeouts": 0,
        }
        return {
            "status": "test_failed",
            "commands": {
                "configure": {"returncode": 0, "timed_out": False},
                "build": {"returncode": 0, "timed_out": False},
                "test": {"returncode": 8, "timed_out": False},
            },
            "test_timeout_source": None,
            "junit_exists": True,
            "junit_evidence": {
                "usable": True, "errors": [], "duplicate_testcase_identities": [],
                "counts": counts,
            },
            "junit_reference_usable": True,
            "test_log_evidence_available": True,
            "test_log_evidence_error": None,
            "no_test_evidence": False,
            "junit_timeout_evidence": 0,
            "skipped_test_evidence": 0,
            "test_evidence": len(statuses),
            "ctest_summary": {
                "tests_failed": counts["failures"], "tests_total": counts["tests"],
            },
            "junit_case_statuses": dict(statuses),
        }

    def test_only_exact_ordinary_test_failed_baseline_qualifies(self):
        manifest = self._manifest()
        expectation = slicer_pr_validation._validate_baseline_expectation_manifest(
            manifest, harness_ref="a" * 40, harness_ref_tree_sha="b" * 40)
        baseline = self._observed_baseline(expectation["testcase_statuses"])
        source_hash = expectation["tracked_source_index_sha256"]
        result = slicer_pr_validation._qualify_baseline_expectation(
            baseline, expectation=expectation,
            source_hash_before=source_hash, source_hash_after=source_hash)
        self.assertEqual(result["status"], "qualified")
        self.assertEqual(baseline["status"], "test_failed")
        self.assertEqual(result["expected_testcase_count"], len(expectation["testcase_statuses"]))

    def test_changed_map_or_nonordinary_evidence_never_qualifies(self):
        manifest = self._manifest()
        expectation = slicer_pr_validation._validate_baseline_expectation_manifest(
            manifest, harness_ref="a" * 40, harness_ref_tree_sha="b" * 40)
        source_hash = expectation["tracked_source_index_sha256"]
        changed = self._observed_baseline({
            "suite::ordinary pass": "failed",
            "suite::ordinary failure": "passed",
        })
        result = slicer_pr_validation._qualify_baseline_expectation(
            changed, expectation=expectation,
            source_hash_before=source_hash, source_hash_after=source_hash)
        self.assertEqual(result["status"], "rejected")
        self.assertTrue(result["status_mismatches"])

        baseline = self._observed_baseline(expectation["testcase_statuses"])
        baseline["junit_evidence"]["counts"]["errors"] = 1
        result = slicer_pr_validation._qualify_baseline_expectation(
            baseline, expectation=expectation,
            source_hash_before=source_hash, source_hash_after=source_hash)
        self.assertEqual(result["status"], "rejected")

        baseline = self._observed_baseline(expectation["testcase_statuses"])
        result = slicer_pr_validation._qualify_baseline_expectation(
            baseline, expectation=expectation,
            source_hash_before=source_hash, source_hash_after="0" * 64)
        self.assertEqual(result["status"], "rejected")

        rejected_baselines = []
        missing = self._observed_baseline({"suite::ordinary pass": "passed"})
        rejected_baselines.append(missing)
        extra = self._observed_baseline({
            **expectation["testcase_statuses"], "suite::unexpected": "passed",
        })
        rejected_baselines.append(extra)
        duplicate = self._observed_baseline(expectation["testcase_statuses"])
        duplicate["junit_evidence"]["usable"] = False
        duplicate["junit_evidence"]["duplicate_testcase_identities"] = ["suite::ordinary pass"]
        rejected_baselines.append(duplicate)
        missing_log = self._observed_baseline(expectation["testcase_statuses"])
        missing_log["test_log_evidence_available"] = False
        missing_log["test_log_evidence_error"] = "missing test log"
        rejected_baselines.append(missing_log)
        skipped = self._observed_baseline(expectation["testcase_statuses"])
        skipped["junit_evidence"]["counts"]["skipped"] = 1
        skipped["skipped_test_evidence"] = 1
        rejected_baselines.append(skipped)
        timed_out = self._observed_baseline(expectation["testcase_statuses"])
        timed_out["junit_evidence"]["counts"]["timeouts"] = 1
        timed_out["junit_timeout_evidence"] = 1
        timed_out["test_timeout_source"] = "junit_case_timeout"
        rejected_baselines.append(timed_out)
        contradictory = self._observed_baseline(expectation["testcase_statuses"])
        contradictory["ctest_summary"]["tests_failed"] = 0
        rejected_baselines.append(contradictory)
        failed_build = self._observed_baseline(expectation["testcase_statuses"])
        failed_build["commands"]["build"]["returncode"] = 1
        rejected_baselines.append(failed_build)
        for rejected in rejected_baselines:
            with self.subTest(baseline=rejected):
                result = slicer_pr_validation._qualify_baseline_expectation(
                    rejected, expectation=expectation,
                    source_hash_before=source_hash, source_hash_after=source_hash)
                self.assertEqual(result["status"], "rejected")


class IdentityManifestIntegrationTests(unittest.TestCase):
    @staticmethod
    def _git(repo, *args):
        return subprocess.run(["git", *args], cwd=repo, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)

    def _fixture(self, root, *, moved_exception=False):
        root.mkdir(parents=True, exist_ok=True)
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "Test")
        (repo / "tests").mkdir()
        (repo / "tests/old.cpp").write_text(
            'TEST_CASE("Original A", "[Identity]") {}\n'
            'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        (repo / "tests/harness.cpp").write_text(
            'TEST_CASE("Harness New", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "harness")
        harness_ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        self._git(repo, "checkout", "-q", "-b", "head", base)
        (repo / "tests/old.cpp").write_text(
            'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
        (repo / "tests/pr.cpp").write_text(
            'TEST_CASE("PR New", "[Identity]") {}\n', encoding="utf-8")
        if moved_exception:
            (repo / "tests/moved.cpp").write_text(
                'TEST_CASE("Original A", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "head")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        return repo, base, harness_ref, head

    def _reference_fixture(self, root, *, unexpected_gap=False):
        root.mkdir(parents=True, exist_ok=True)
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "Test")
        (repo / "tests").mkdir()
        (repo / "tests/old.cpp").write_text(
            'TEST_CASE("Original A", "[Identity]") {}\n'
            'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        (repo / "tests/harness.cpp").write_text(
            'TEST_CASE("Harness New", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "harness")
        harness_ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        self._git(repo, "checkout", "-q", "-b", "reference-base", base)
        (repo / "tests/old.cpp").write_text(
            ('' if unexpected_gap else 'TEST_CASE("Original B", "[Identity]") {}\n'),
            encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "reference base")
        reference_base = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        self._git(repo, "checkout", "-q", "-b", "head", reference_base)
        (repo / "tests/pr.cpp").write_text(
            'TEST_CASE("PR New", "[Identity]") {}\n', encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "head")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        return repo, base, harness_ref, reference_base, head

    @staticmethod
    def _case(declaration):
        return {
            "runtime_identity": declaration["name"] + "::" + declaration["name"],
            "source": declaration,
        }

    def _manifest(self, repo, base, harness_ref, head):
        base_declarations = slicer_pr_validation._git_test_declarations(repo, base)
        ref_declarations = slicer_pr_validation._git_test_declarations(repo, harness_ref)
        head_declarations = slicer_pr_validation._git_test_declarations(repo, head)
        pick = lambda declarations, name: next(
            item for item in declarations.values() if item["name"] == name)
        return {
            "schema_version": 2,
            "harness_base_sha": base,
            "harness_ref_sha": harness_ref,
            "required_harness_cases": [self._case(pick(ref_declarations, "Harness New"))],
            "exception_groups": {
                "none": [],
                "legacy": [self._case(pick(base_declarations, "Original A"))],
            },
            "heads": [{
                "number": 1, "head_oid": head, "exception_group": "legacy",
                "required_head_cases": [self._case(pick(head_declarations, "PR New"))],
            }],
        }

    def _args(self, repo, snapshot, output, manifest):
        args = slicer_pr_validation._parser().parse_args([
            "--root", str(repo), "--snapshot", str(snapshot),
            "--harness-base", "base", "--harness-ref", "harness",
            "--identity-manifest", str(manifest), "--deps-prefix", str(repo / "deps"),
            "--output-dir", str(output), "--build-dir", "build", "--jobs", "1",
            "--timeout", "1", "--minimum-tests", "3",
        ])
        args.targets = slicer_pr_validation.DEFAULT_TARGETS
        args.uv = "uv"
        return args

    @staticmethod
    def _snapshot(root, head, base):
        path = root / "snapshot.json"
        path.write_text(json.dumps([{
            "number": 1, "headRefName": "head", "headRefOid": head,
            "baseRefOid": base,
        }]), encoding="utf-8")
        return path

    @staticmethod
    def _run_tree(*run_args, **run_kwargs):
        name = run_args[2]
        common = {
            "junit_evidence": {"counts": {"tests": 3}},
            "minimum_tests_required": 3,
            "minimum_tests_observed": 3,
            "minimum_tests_ok": True,
            "missing_baseline_testcases": [],
            "failed_test_names": [],
            "junit_reference_usable": True,
        }
        cases = {
            "Original A::Original A": "passed",
            "Original B::Original B": "passed",
            "Harness New::Harness New": "passed",
        }
        if name == "pr-1":
            cases.pop("Original A::Original A")
            cases["PR New::PR New"] = "passed"
        return {**common, "name": name, "status": "pass",
                "junit_case_statuses": cases}

    def _write_manifest_and_args(self, root, *, moved_exception=False):
        repo, base, harness_ref, head = self._fixture(root, moved_exception=moved_exception)
        snapshot = self._snapshot(root, head, base)
        manifest = root / "identity.json"
        manifest.write_text(json.dumps(
            self._manifest(repo, base, harness_ref, head)), encoding="utf-8")
        args = self._args(repo, snapshot, root / "evidence", manifest)
        args.harness_base = base
        args.harness_ref = harness_ref
        return repo, base, harness_ref, head, args

    def _write_reference_manifest_and_args(self, root, *, unexpected_gap=False):
        repo, base, harness_ref, reference_base, head = self._reference_fixture(
            root, unexpected_gap=unexpected_gap)
        snapshot = self._snapshot(root, head, reference_base)
        manifest = root / "identity.json"
        manifest.write_text(json.dumps(
            self._manifest(repo, base, harness_ref, head)), encoding="utf-8")
        args = self._args(repo, snapshot, root / "evidence", manifest)
        args.harness_base = base
        args.harness_ref = harness_ref
        return repo, base, harness_ref, reference_base, head, args

    def test_run_validation_applies_exact_exception_floor_and_head_requirement(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(root)
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._run_tree):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 0)
            report = json.loads((root / "evidence/report.json").read_text(encoding="utf-8"))
            gate = report["heads"][0]["identity_gate"]
            self.assertEqual(gate["exception_count"], 1)
            self.assertEqual(gate["minimum_tests_adjusted"], 2)
            self.assertEqual(gate["required_head_runtime_ids"], ["PR New::PR New"])
            self.assertEqual(gate["overlay_identity"]["checked"], True)
            self.assertEqual(report["identity_gate"]["all_snapshot_heads_validated"], True)
            reference_gate = report["references"][base]["identity_gate"]
            self.assertEqual(reference_gate["exception_runtime_ids"], [])
            self.assertEqual(reference_gate["minimum_tests_adjustment"], 0)
            self.assertEqual(reference_gate["minimum_tests_adjusted"], 3)

    def test_old_reference_with_exact_approved_gap_remains_comparable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, reference_base, head, args = (
                self._write_reference_manifest_and_args(root))

            def run_tree(*run_args, **run_kwargs):
                name = run_args[2]
                if name == "baseline":
                    cases = {
                        "Original A::Original A": "passed",
                        "Original B::Original B": "passed",
                        "Harness New::Harness New": "passed",
                    }
                elif name.startswith("reference-base-"):
                    cases = {
                        "Original B::Original B": "passed",
                        "Harness New::Harness New": "passed",
                    }
                else:
                    cases = {
                        "Original B::Original B": "passed",
                        "Harness New::Harness New": "passed",
                        "PR New::PR New": "passed",
                    }
                return {
                    "name": name, "status": "pass",
                    "junit_evidence": {"counts": {"tests": len(cases)}},
                    "junit_case_statuses": cases,
                    "failed_test_names": [],
                    "junit_reference_usable": True,
                }

            with mock.patch.object(slicer_pr_validation, "_run_tree", side_effect=run_tree):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 0)
            report = json.loads((root / "evidence/report.json").read_text(encoding="utf-8"))
            reference = report["references"][reference_base]
            gate = reference["identity_gate"]
            self.assertEqual(reference["status"], "pass")
            self.assertTrue(reference["junit_reference_usable"])
            self.assertEqual(gate["reference_base_sha"], reference_base)
            self.assertEqual(gate["exception_group"], "legacy")
            self.assertEqual(gate["exception_runtime_ids"], ["Original A::Original A"])
            self.assertEqual(gate["minimum_tests_adjustment"], -1)
            self.assertEqual(gate["minimum_tests_adjusted"], 2)
            self.assertEqual(reference["missing_baseline_testcases"], [])

    def test_reference_base_policy_is_exact_and_rejects_unapproved_gaps(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, reference_base, head, args = (
                self._write_reference_manifest_and_args(root))
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            approved_records = [{
                "number": 1, "headRefOid": head, "baseRefOid": reference_base,
            }]
            approved = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=approved_records, selected_records=approved_records)
            self.assertEqual(
                approved["reference_bases"][reference_base]["exception_runtime_ids"],
                ["Original A::Original A"])

            current_records = [{"number": 1, "headRefOid": head, "baseRefOid": base}]
            current = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=current_records, selected_records=current_records)
            self.assertEqual(current["reference_bases"][base]["exception_runtime_ids"], [])
            self.assertIsNone(current["reference_bases"][base]["exception_group"])

            repo, base, harness_ref, unexpected_base, head, args = (
                self._write_reference_manifest_and_args(root / "unexpected", unexpected_gap=True))
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head, "baseRefOid": unexpected_base}]
            with self.assertRaisesRegex(
                    slicer_pr_validation.ValidationError,
                    "do not match one exact approved exception group"):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=records, selected_records=records)

    def test_reference_overlay_rejects_missing_harness_and_unexpected_original_gap(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, reference_base, head, args = (
                self._write_reference_manifest_and_args(root))
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head, "baseRefOid": reference_base}]
            identity = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=records, selected_records=records)
            policy = identity["reference_bases"][reference_base]
            self._git(repo, "checkout", "-q", reference_base)
            self._git(repo, "checkout", harness_ref, "--", "tests/harness.cpp")
            overlay = slicer_pr_validation._validate_reference_identity_overlay(
                repo, policy, identity["required_harness_cases"], base_sha=reference_base)
            self.assertTrue(overlay["checked"])

            (repo / "tests/harness.cpp").unlink()
            with self.assertRaisesRegex(
                    slicer_pr_validation.ValidationError, r"missing_required=\['Harness New'\]"):
                slicer_pr_validation._validate_reference_identity_overlay(
                    repo, policy, identity["required_harness_cases"], base_sha=reference_base)
            self._git(repo, "checkout", harness_ref, "--", "tests/harness.cpp")

            (repo / "tests/old.cpp").write_text("", encoding="utf-8")
            with self.assertRaisesRegex(
                    slicer_pr_validation.ValidationError, r"missing_original=\['Original B'\]"):
                slicer_pr_validation._validate_reference_identity_overlay(
                    repo, policy, identity["required_harness_cases"], base_sha=reference_base)

    def test_reference_overlay_rejects_a_policy_bound_to_another_base(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, reference_base, head, args = (
                self._write_reference_manifest_and_args(root))
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head, "baseRefOid": reference_base}]
            identity = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=records, selected_records=records)
            policy = identity["reference_bases"][reference_base]
            with self.assertRaisesRegex(slicer_pr_validation.ValidationError, "policy is stale"):
                slicer_pr_validation._validate_reference_identity_overlay(
                    repo, policy, identity["required_harness_cases"], base_sha=base)

    def test_reference_adjustment_preserves_skipped_error_and_timeout_statuses(self):
        baseline = {
            "Original A::Original A": "passed",
            "Original B::Original B": "passed",
            "Harness New::Harness New": "passed",
        }
        adjusted_baseline = {name: status for name, status in baseline.items()
                             if name != "Original A::Original A"}
        cases = {
            "Original B::Original B": "passed",
            "Harness New::Harness New": "passed",
        }
        for status, counts, current in (
                ("test_skipped", {"tests": 2, "skipped": 1},
                 {**cases, "Harness New::Harness New": "skipped"}),
                ("test_error", {"tests": 2, "errors": 1},
                 {**cases, "Harness New::Harness New": "error"}),
                ("test_timeout", {"tests": 2, "timeouts": 1},
                 {**cases, "Harness New::Harness New": "timeout"})):
            with self.subTest(status=status):
                entry = {
                    "status": status,
                    "junit_evidence": {"usable": True, "counts": counts},
                    "junit_case_statuses": current,
                }
                slicer_pr_validation._apply_minimum_test_gate(
                    entry, minimum_tests=2, baseline_cases=adjusted_baseline)
                self.assertEqual(entry["status"], status)
                self.assertFalse(entry["junit_reference_usable"])
                self.assertTrue(entry["minimum_tests_ok"])

    def test_overlay_rejects_exception_reintroduced_by_applied_harness(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(root)
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head, "baseRefOid": base}]
            policy = slicer_pr_validation._validate_identity_manifest(
                repo, manifest, harness_base=base, harness_ref=harness_ref,
                snapshot_records=records, selected_records=records)
            self._git(repo, "checkout", harness_ref, "--", "tests/harness.cpp")
            clean_overlay = slicer_pr_validation._validate_identity_overlay(
                repo, policy["heads"][1], policy["required_harness_cases"])
            self.assertTrue(clean_overlay["checked"])
            (repo / "tests/old.cpp").write_text(
                'TEST_CASE("Original A", "[Identity]") {}\n'
                'TEST_CASE("Original B", "[Identity]") {}\n', encoding="utf-8")
            with self.assertRaisesRegex(
                    slicer_pr_validation.ValidationError,
                    r"missing_required=\[\], present_exceptions=\['Original A'\]"):
                slicer_pr_validation._validate_identity_overlay(
                    repo, policy["heads"][1], policy["required_harness_cases"])

    def test_run_validation_rejects_missing_required_harness_case(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(root)

            def missing_harness(*run_args, **run_kwargs):
                entry = self._run_tree(*run_args, **run_kwargs)
                if run_args[2] == "baseline":
                    entry["junit_case_statuses"].pop("Harness New::Harness New")
                return entry

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=missing_harness):
                with self.assertRaises(slicer_pr_validation.ValidationError):
                    slicer_pr_validation.run_validation(args)

    def test_run_validation_rejects_missing_pr_added_case(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(root)

            def missing_pr_case(*run_args, **run_kwargs):
                entry = self._run_tree(*run_args, **run_kwargs)
                if run_args[2] == "pr-1":
                    entry["junit_case_statuses"].pop("PR New::PR New")
                return entry

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=missing_pr_case):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 2)
            report = json.loads((root / "evidence/report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["heads"][0]["status"], "test_evidence_incomplete")
            self.assertEqual(report["heads"][0]["missing_required_head_testcases"],
                             ["PR New::PR New"])

    def test_moved_exception_and_stale_manifest_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(
                root, moved_exception=True)
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head, "baseRefOid": base}]
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=records, selected_records=records)

            repo, base, harness_ref, head, args = self._write_manifest_and_args(root / "stale")
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            manifest["heads"][0]["head_oid"] = base
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}],
                    selected_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}])
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            manifest["required_harness_cases"][0]["source"]["line"] += 1
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}],
                    selected_records=[{"number": 1, "headRefOid": head, "baseRefOid": base}])


class JUnitEvidenceTests(unittest.TestCase):
    @staticmethod
    def _result(returncode=0, *, timed_out=False, output="100% tests passed, 0 tests failed out of 2"):
        return slicer_pr_validation.CommandResult(
            ["ctest"], returncode, 0.01, output, "", timed_out)

    @staticmethod
    def _write(root, name, contents):
        path = root / name
        path.write_text(contents, encoding="utf-8")
        return path

    def test_catch_case_count_and_explicit_ctest_summary_win(self):
        self.assertEqual(
            slicer_pr_validation.test_evidence(
                "All tests passed (43 assertions in 2 test cases)"), 2)
        self.assertEqual(
            slicer_pr_validation.test_evidence(
                "100% tests passed, 0 tests failed out of 5\n"
                "All tests passed (43 assertions in 2 test cases)"), 5)

    def test_valid_junit_preserves_error_as_distinct_case_status(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = self._write(
                Path(temp), "valid.xml",
                """<testsuites><testsuite name='suite' tests='4' failures='1'
                errors='1' skipped='1'>
                <testcase classname='suite' name='pass'/>
                <testcase classname='suite' name='assertion'><failure/></testcase>
                <testcase classname='suite' name='crash'><error/></testcase>
                <testcase classname='suite' name='skip'><skipped/></testcase>
                </testsuite></testsuites>""",
            )
            evidence = slicer_pr_validation.read_junit_evidence(junit)
        self.assertTrue(evidence["usable"])
        self.assertEqual(evidence["counts"], {
            "tests": 4, "failures": 1, "errors": 1, "skipped": 1, "timeouts": 0,
        })
        self.assertEqual(evidence["cases"], {
            "suite::pass": "passed",
            "suite::assertion": "failed",
            "suite::crash": "error",
            "suite::skip": "skipped",
        })

    def test_catch2_direct_profile_separates_case_and_assertion_totals(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            control = self._write(
                root, "catch-control.xml",
                """<testsuites><testsuite name='catch' tests='9' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='one'/>
                <testcase classname='suite.global' name='two'/>
                </testsuite></testsuites>""",
            )
            mutant = self._write(
                root, "catch-mutant.xml",
                """<testsuites><testsuite name='catch' tests='20' failures='6' errors='1' skipped='2'>
                <testcase classname='suite.global' name='passed'/>
                <testcase classname='suite.global' name='mutated'>
                  <failure type='CHECK_THAT'/><failure type='CHECK_THAT'/>
                  <failure type='CHECK_THAT'/><failure type='REQUIRE'/>
                </testcase>
                <testcase classname='suite.global' name='mixed'>
                  <failure/><failure/><error/><skipped/>
                </testcase>
                <testcase classname='suite.global' name='explicit-skip'><skipped/></testcase>
                </testsuite></testsuites>""",
            )
            strict_control = slicer_pr_validation.read_junit_evidence(control)
            control_evidence = slicer_pr_validation.read_junit_evidence(
                control, reporter_profile="catch2_direct")
            mutant_evidence = slicer_pr_validation.read_junit_evidence(
                mutant, reporter_profile="catch2_direct")

        self.assertFalse(strict_control["usable"])
        self.assertIn("tests total differs from testcase evidence", strict_control["errors"])
        self.assertTrue(control_evidence["usable"], control_evidence["errors"])
        self.assertEqual(control_evidence["counts"], {
            "tests": 2, "failures": 0, "errors": 0, "skipped": 0, "timeouts": 0,
        })
        self.assertEqual(control_evidence["reporter_totals"], {
            "assertions": {"tests": 9, "failures": 0, "errors": 0, "skipped": 0},
            "result_elements": {"failures": 0, "errors": 0, "skipped": 0},
        })
        self.assertTrue(mutant_evidence["usable"], mutant_evidence["errors"])
        self.assertEqual(mutant_evidence["counts"], {
            "tests": 4, "failures": 1, "errors": 1, "skipped": 1, "timeouts": 0,
        })
        self.assertEqual(mutant_evidence["cases"]["mutated"], "failed")
        self.assertEqual(mutant_evidence["cases"]["mixed"], "error")
        self.assertEqual(mutant_evidence["cases"]["explicit-skip"], "skipped")
        self.assertEqual(mutant_evidence["reporter_totals"], {
            "assertions": {"tests": 20, "failures": 6, "errors": 1, "skipped": 2},
            "result_elements": {"failures": 6, "errors": 1, "skipped": 2},
        })
        self.assertEqual(mutant_evidence["assertion_failure_types_by_case"]["mutated"], {
            "CHECK_THAT": 3, "REQUIRE": 1,
        })
        self.assertEqual(mutant_evidence["case_result_elements"]["mutated"], {
            "failure": 4, "error": 0, "skipped": 0,
        })
        self.assertEqual(mutant_evidence["case_result_elements"]["mixed"], {
            "failure": 2, "error": 1, "skipped": 1,
        })

    def test_catch2_direct_rejects_bad_totals_and_duplicate_case_identities(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            bad_totals = self._write(
                root, "bad-totals.xml",
                """<testsuites><testsuite name='catch' tests='8' failures='2' errors='0' skipped='0'>
                <testcase classname='suite.global' name='mutated'><failure/></testcase>
                </testsuite></testsuites>""",
            )
            impossible_total = self._write(
                root, "impossible-total.xml",
                """<testsuites><testsuite name='catch' tests='2' failures='2' errors='0' skipped='1'>
                <testcase classname='suite.global' name='mutated'><failure/><failure/></testcase>
                <testcase classname='suite.global' name='skipped'><skipped/></testcase>
                </testsuite></testsuites>""",
            )
            duplicate = self._write(
                root, "duplicate.xml",
                """<testsuites><testsuite name='catch' tests='8' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='same'/>
                <testcase classname='suite.global' name='same'/>
                </testsuite></testsuites>""",
            )
            bad = slicer_pr_validation.read_junit_evidence(
                bad_totals, reporter_profile="catch2_direct")
            impossible = slicer_pr_validation.read_junit_evidence(
                impossible_total, reporter_profile="catch2_direct")
            repeated = slicer_pr_validation.read_junit_evidence(
                duplicate, reporter_profile="catch2_direct")

        self.assertFalse(bad["usable"])
        self.assertIn("Catch2 failures total differs from assertion elements", bad["errors"])
        self.assertFalse(impossible["usable"])
        self.assertIn(
            "Catch2 tests total is smaller than reported assertion outcomes",
            impossible["errors"],
        )
        self.assertFalse(repeated["usable"])
        self.assertEqual(repeated["duplicate_testcase_identities"], ["same"])

    def test_catch2_direct_rejects_mayfail_markers(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            mixed_failures = self._write(
                root, "mixed-mayfail.xml",
                """<testsuites><testsuite name='catch' tests='2' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='expected'>
                  <skipped message='TEST_CASE tagged with !mayfail'/><failure type='CHECK'/>
                </testcase>
                <testcase classname='suite.global' name='unexpected'><failure type='CHECK'/></testcase>
                </testsuite></testsuites>""",
            )
            tagged_exception = self._write(
                root, "mayfail-exception.xml",
                """<testsuites><testsuite name='catch' tests='1' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='throws'>
                  <skipped message='TEST_CASE tagged with !mayfail'/><error type='REQUIRE'/>
                </testcase>
                </testsuite></testsuites>""",
            )
            mixed = slicer_pr_validation.read_junit_evidence(
                mixed_failures, reporter_profile="catch2_direct")
            exception = slicer_pr_validation.read_junit_evidence(
                tagged_exception, reporter_profile="catch2_direct")

        for evidence in (mixed, exception):
            self.assertFalse(evidence["usable"])
            self.assertTrue(any(
                "unsupported !mayfail testcase marker" in error
                for error in evidence["errors"]))
        self.assertEqual(mixed["reporter_totals"]["result_elements"], {
            "failures": 2, "errors": 0, "skipped": 1,
        })
        self.assertEqual(mixed["assertion_failure_types_by_case"]["unexpected"], {
            "CHECK": 1,
        })
        self.assertEqual(exception["reporter_totals"]["result_elements"], {
            "failures": 0, "errors": 1, "skipped": 1,
        })

    def test_catch2_direct_rejects_missing_totals_and_unknown_profile(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            missing_total = self._write(
                root, "missing-total.xml",
                """<testsuites><testsuite name='catch' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='one'/>
                </testsuite></testsuites>""",
            )
            valid = self._write(
                root, "valid.xml",
                """<testsuites><testsuite name='catch' tests='1' failures='0' errors='0' skipped='0'>
                <testcase classname='suite.global' name='one'/>
                </testsuite></testsuites>""",
            )
            missing = slicer_pr_validation.read_junit_evidence(
                missing_total, reporter_profile="catch2_direct")
            unknown = slicer_pr_validation.read_junit_evidence(
                valid, reporter_profile="catch2_unknown")

        self.assertFalse(missing["usable"])
        self.assertTrue(any(
            "invalid or missing Catch2 tests total" in error
            for error in missing["errors"]))
        self.assertFalse(unknown["usable"])
        self.assertTrue(any(
            "unknown JUnit reporter profile" in error
            for error in unknown["errors"]))

    def test_ctest_crash_encoded_as_failure_is_error_but_assertion_is_failed(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            junit = self._write(
                root, "crash.xml",
                """<testsuites><testsuite tests='2' failures='2' errors='0' skipped='0'>
                <testcase classname='suite' name='assertion'><failure message='CHECK failed'/></testcase>
                <testcase classname='suite' name='crash'><failure message='Subprocess killed (SEGFAULT)'/></testcase>
                </testsuite></testsuites>""",
            )
            evidence = slicer_pr_validation.read_junit_evidence(junit)
        self.assertTrue(evidence["usable"])
        self.assertEqual(evidence["cases"]["suite::assertion"], "failed")
        self.assertEqual(evidence["cases"]["suite::crash"], "error")
        self.assertEqual(
            slicer_pr_validation.classify_junit_result(
                self._result(8), evidence, 2), "test_error")

    def test_ctest_timeout_is_separate_from_assertion_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = self._write(
                Path(temp), "timeout.xml",
                """<testsuites><testsuite tests='2' failures='2' errors='0' skipped='0'>
                <testcase classname='suite' name='assertion'><failure message='CHECK failed'/></testcase>
                <testcase classname='suite' name='slow'><failure message='Timeout'>
                  Test timeout after 180 seconds</failure></testcase>
                </testsuite></testsuites>""",
            )
            evidence = slicer_pr_validation.read_junit_evidence(junit)
        self.assertTrue(evidence["usable"])
        self.assertEqual(evidence["counts"]["failures"], 1)
        self.assertEqual(evidence["counts"]["timeouts"], 1)
        self.assertEqual(evidence["cases"]["suite::assertion"], "failed")
        self.assertEqual(evidence["cases"]["suite::slow"], "timeout")
        self.assertEqual(
            slicer_pr_validation.classify_junit_result(
                self._result(8), evidence, 2), "test_timeout")

    def test_timeout_does_not_become_inherited_assertion_failure(self):
        classification = slicer_pr_validation.classify_failure_names(
            {
                "status": "test_failed", "junit_reference_usable": True,
                "junit_case_statuses": {"suite::slow": "failed"},
                "failed_test_names": ["suite::slow"],
            },
            {
                "status": "test_timeout", "junit_reference_usable": False,
                "junit_case_statuses": {"suite::slow": "timeout"},
                "failed_test_names": [],
                "timed_out_test_names": ["suite::slow"],
            },
        )
        self.assertFalse(classification["reference_available"])
        self.assertEqual(classification["inherited"], [])
        self.assertEqual(classification["new"], [])
        self.assertEqual(classification["raw_observed_head_failures"], [])
        self.assertEqual(classification["raw_observed_head_timeouts"], ["suite::slow"])
        self.assertIn("separately", classification["classification_basis"])

    def test_timeout_words_in_test_name_or_output_do_not_hide_assertion_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            junit = self._write(
                Path(temp), "timeout-words.xml",
                """<testsuite tests='1' failures='1' errors='0' skipped='0'>
                <testcase name='A timed out request preserves state'>
                  <failure message='CHECK failed'>CHECK(status == "timed out")</failure>
                  <system-out>Test timeout after 180 seconds</system-out>
                </testcase></testsuite>""",
            )
            evidence = slicer_pr_validation.read_junit_evidence(junit)
        self.assertTrue(evidence["usable"])
        self.assertEqual(evidence["counts"]["timeouts"], 0)
        self.assertEqual(evidence["counts"]["failures"], 1)
        self.assertEqual(slicer_pr_validation.classify_junit_result(
            self._result(8), evidence, 1), "test_failed")

    def test_empty_ctest_timeout_failure_is_recognized(self):
        with tempfile.TemporaryDirectory() as temp:
            for tag in ("failure", "error"):
                with self.subTest(tag=tag):
                    junit = self._write(
                        Path(temp), "timeout.xml",
                        f"<testsuite tests='1' failures='1'><testcase name='slow'>"
                        f"<{tag} message='Timeout'/></testcase></testsuite>",
                    )
                    evidence = slicer_pr_validation.read_junit_evidence(junit)
                    self.assertTrue(evidence["usable"])
                    self.assertEqual(evidence["cases"], {"slow": "timeout"})

    def test_exit_zero_with_reported_failure_is_evidence_incomplete(self):
        evidence = {
            "usable": True,
            "errors": [],
            "counts": {"tests": 2, "failures": 1, "errors": 0, "skipped": 0},
            "cases": {"suite::one": "failed", "suite::two": "passed"},
        }
        self.assertEqual(
            slicer_pr_validation.classify_junit_result(
                self._result(0), evidence, 2), "test_evidence_incomplete")

    def test_duplicate_invalid_and_contradictory_junit_are_unusable(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            duplicate = self._write(
                root, "duplicate.xml",
                """<testsuites><testsuite tests='2' failures='0' errors='0' skipped='0'>
                <testcase classname='suite' name='same'/>
                <testcase classname='suite' name='same'/>
                </testsuite></testsuites>""",
            )
            contradictory = self._write(
                root, "contradictory.xml",
                """<testsuites><testsuite tests='1' failures='0' errors='0' skipped='0'>
                <testcase classname='suite' name='failed'><failure/></testcase>
                </testsuite></testsuites>""",
            )
            malformed = self._write(root, "malformed.xml", "<testsuites>")
            negative = self._write(
                root, "negative.xml",
                """<testsuites><testsuite tests='-1' failures='0' errors='0' skipped='0'
                disabled='0'><testcase classname='suite' name='one'/></testsuite></testsuites>""",
            )
            disabled = self._write(
                root, "disabled.xml",
                """<testsuites><testsuite tests='1' failures='0' errors='0' skipped='0'
                disabled='1'><testcase classname='suite' name='one'/></testsuite></testsuites>""",
            )
            missing = root / "missing.xml"
            duplicate_evidence = slicer_pr_validation.read_junit_evidence(duplicate)
            contradictory_evidence = slicer_pr_validation.read_junit_evidence(contradictory)
            malformed_evidence = slicer_pr_validation.read_junit_evidence(malformed)
            negative_evidence = slicer_pr_validation.read_junit_evidence(negative)
            disabled_evidence = slicer_pr_validation.read_junit_evidence(disabled)
            missing_evidence = slicer_pr_validation.read_junit_evidence(missing)
        for evidence in (duplicate_evidence, contradictory_evidence,
                         malformed_evidence, negative_evidence, disabled_evidence,
                         missing_evidence):
            self.assertFalse(evidence["usable"])
            self.assertTrue(evidence["errors"])

    def test_classify_junit_result_is_fail_closed(self):
        good = {
            "usable": True,
            "errors": [],
            "counts": {"tests": 2, "failures": 0, "errors": 0, "skipped": 0},
            "cases": {"suite::one": "passed", "suite::two": "passed"},
        }
        failed = {
            **good,
            "counts": {"tests": 2, "failures": 1, "errors": 0, "skipped": 0},
            "cases": {"suite::one": "failed", "suite::two": "passed"},
        }
        errored = {
            **good,
            "counts": {"tests": 2, "failures": 0, "errors": 1, "skipped": 0},
            "cases": {"suite::one": "error", "suite::two": "passed"},
        }
        skipped = {
            **good,
            "counts": {"tests": 2, "failures": 0, "errors": 0, "skipped": 1},
            "cases": {"suite::one": "passed", "suite::two": "skipped"},
        }
        classify = slicer_pr_validation.classify_junit_result
        self.assertEqual(classify(self._result(), good, 2), "pass")
        self.assertEqual(classify(self._result(1), failed, 2), "test_failed")
        self.assertEqual(classify(self._result(8), errored, 2), "test_error")
        self.assertEqual(classify(self._result(), skipped, 2), "test_skipped")
        self.assertEqual(classify(self._result(None, timed_out=True), good, 2), "test_timeout")
        self.assertEqual(classify(self._result(), good, 2, no_tests=True),
                         "test_evidence_incomplete")
        empty = {
            "usable": False,
            "errors": ["JUnit contains no testcases"],
            "counts": {"tests": 0, "failures": 0, "errors": 0, "skipped": 0},
            "cases": {},
        }
        self.assertEqual(classify(self._result(), empty, 0, no_tests=True), "no_tests")
        self.assertEqual(classify(self._result(), {**good, "usable": False}, 2),
                         "test_evidence_incomplete")
        self.assertEqual(classify(self._result(), good, 3), "test_evidence_incomplete")


class CompleteTestLogEvidenceTests(unittest.TestCase):
    @staticmethod
    def _args():
        return Namespace(
            test_timeout=180.0,
            timeout=30.0,
            minimum_tests=1,
            build_dir="build",
            jobs=1,
            test_jobs=1,
            exclude_labels="",
            deps_prefix="deps",
            uv="uv",
        )

    def _run_tree(self, *, full_log, junit_xml, returncode=0, timed_out=False,
                  create_test_log=True, stdout=None):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            worktree = root / "worktree"
            worktree.mkdir()

            def fake_default_test(build, jobs, junit, excluded, timeout, regex=None):
                if junit_xml is not None:
                    junit.write_text(junit_xml, encoding="utf-8")
                return ["ctest", "--output-junit", str(junit)]

            def fake_phase(argv, *, cwd, timeout, log):
                log.parent.mkdir(parents=True, exist_ok=True)
                if argv and argv[0] == "ctest":
                    if create_test_log:
                        log.write_text(full_log, encoding="utf-8")
                    output = stdout if stdout is not None else full_log
                    return slicer_pr_validation.CommandResult(
                        list(argv), returncode, 0.01, output, "", timed_out)
                log.write_text("", encoding="utf-8")
                return slicer_pr_validation.CommandResult(list(argv), 0, 0.01, "", "")

            with mock.patch.object(slicer_pr_validation, "_default_test",
                                   side_effect=fake_default_test), \
                 mock.patch.object(slicer_pr_validation, "_run_phase",
                                   side_effect=fake_phase):
                return slicer_pr_validation._run_tree(
                    worktree, root / "output", "run", self._args(),
                    ["cmake", "configure"], ["cmake", "build"],
                    commit_sha="a" * 40, tree_sha="b" * 40,
                )

    def test_ctest_summary_before_large_failure_list_is_read_from_full_log(self):
        full_log = (
            "50% tests passed, 1 tests failed out of 2\n"
            + "test cases: 99\n"
            + "failed test output follows\n" * (slicer_pr_validation.MAX_OUTPUT // 8)
        )
        stdout_tail = "...[earlier output retained in log]...\n" + full_log[-slicer_pr_validation.MAX_OUTPUT:]
        self.assertEqual(slicer_pr_validation.test_evidence(stdout_tail), 0)
        self.assertGreater(len(stdout_tail), slicer_pr_validation.MAX_OUTPUT)
        junit = (
            '<testsuite tests="2" failures="1" errors="0" skipped="0">'
            '<testcase classname="suite" name="assertion"><failure message="CHECK failed"/>'
            '</testcase><testcase classname="suite" name="pass"/></testsuite>'
        )

        entry = self._run_tree(full_log=full_log, junit_xml=junit, returncode=8,
                               stdout=stdout_tail)

        self.assertEqual(entry["status"], "test_failed")
        self.assertEqual(entry["test_evidence"], 2)
        self.assertTrue(entry["test_log_evidence_available"])
        self.assertFalse(entry["no_test_evidence"])
        self.assertTrue(entry["junit_reference_usable"])

    def test_streamed_matches_survive_chunk_boundaries_and_later_ctest_wins(self):
        chunk_size = 64
        early_catch = "test cases: 999\n"
        first_ctest = "50% tests passed, 1 tests failed out of 123\n"
        prefix_len = (chunk_size - 1 - len(early_catch) - first_ctest.index("123")) % chunk_size
        prefix = "p" * prefix_len
        second_ctest = "100% tests passed, 0 tests failed out of 45\n"
        late_catch = "test cases: 10000\n"
        before_sentinel = early_catch + prefix + first_ctest + second_ctest + late_catch
        sentinel = "No tests were found"
        sentinel_prefix_len = (chunk_size - 2 - len(before_sentinel) - sentinel.index("found")) % chunk_size
        full_log = before_sentinel + "-" * sentinel_prefix_len + sentinel + "\n"

        self.assertEqual(
            (len(early_catch) + len(prefix) + first_ctest.index("123")) % chunk_size,
            chunk_size - 1,
        )
        self.assertEqual(
            (len(before_sentinel) + sentinel_prefix_len + sentinel.index("found")) % chunk_size,
            chunk_size - 2,
        )
        with tempfile.TemporaryDirectory() as temp:
            log_path = Path(temp) / "test.log"
            log_path.write_text(full_log, encoding="utf-8")
            with mock.patch.object(slicer_pr_validation, "MAX_OUTPUT", chunk_size):
                evidence = slicer_pr_validation._read_test_log_evidence(log_path)

        self.assertTrue(evidence["available"])
        self.assertEqual(evidence["count"], 45)
        self.assertTrue(evidence["no_tests"])

    def test_no_tests_sentinel_outside_bounded_tail_is_detected(self):
        full_log = "No tests were found!!!\n" + "ordinary output\n" * (
            slicer_pr_validation.MAX_OUTPUT // 8)
        stdout_tail = "...[earlier output retained in log]...\n" + full_log[-slicer_pr_validation.MAX_OUTPUT:]
        self.assertFalse(slicer_pr_validation.no_test_evidence(stdout_tail))

        entry = self._run_tree(full_log=full_log, junit_xml=None, stdout=stdout_tail)

        self.assertEqual(entry["status"], "no_tests")
        self.assertEqual(entry["test_evidence"], 0)
        self.assertTrue(entry["no_test_evidence"])

    def test_missing_log_or_junit_count_mismatch_fails_closed(self):
        passing_junit = '<testsuite tests="2"><testcase name="one"/><testcase name="two"/></testsuite>'
        missing_log = self._run_tree(
            full_log="100% tests passed, 0 tests failed out of 2\n",
            junit_xml=passing_junit,
            create_test_log=False,
            stdout="100% tests passed, 0 tests failed out of 2\n",
        )
        self.assertEqual(missing_log["status"], "test_evidence_incomplete")
        self.assertFalse(missing_log["test_log_evidence_available"])
        self.assertFalse(missing_log["junit_reference_usable"])

        contradictory = self._run_tree(
            full_log="100% tests passed, 0 tests failed out of 3\n",
            junit_xml=passing_junit,
        )
        self.assertEqual(contradictory["status"], "test_evidence_incomplete")
        self.assertEqual(contradictory["test_evidence"], 3)
        self.assertEqual(contradictory["junit_evidence"]["counts"]["tests"], 2)

    def test_timeout_and_execution_error_keep_their_statuses(self):
        junit = '<testsuite tests="1"><testcase name="one"/></testsuite>'
        log = "100% tests passed, 0 tests failed out of 1\n"
        timeout = self._run_tree(full_log=log, junit_xml=junit, timed_out=True,
                                 returncode=None)
        self.assertEqual(timeout["status"], "test_timeout")

        error = self._run_tree(full_log=log, junit_xml=junit, returncode=-11)
        self.assertEqual(error["status"], "test_error")


class ProcessCaptureTests(unittest.TestCase):
    def test_command_json_marks_truncated_streams_at_the_exact_limit(self):
        limit = slicer_pr_validation.MAX_OUTPUT
        cases = (
            ("", "", False, False),
            ("x" * limit, "y" * limit, False, False),
            ("x" * (limit + 1), "", True, False),
            ("", "y" * (limit + 1), False, True),
        )
        for stdout, stderr, stdout_truncated, stderr_truncated in cases:
            with self.subTest(stdout_truncated=stdout_truncated,
                              stderr_truncated=stderr_truncated):
                evidence = slicer_pr_validation.CommandResult(
                    ["command"], 0, 0.01, stdout, stderr).json()

                self.assertEqual(evidence["stdout_truncated"], stdout_truncated)
                self.assertEqual(evidence["stderr_truncated"], stderr_truncated)
                if stdout_truncated:
                    self.assertIn("...[output truncated]...", evidence["stdout"])
                else:
                    self.assertEqual(evidence["stdout"], stdout)
                if stderr_truncated:
                    self.assertIn("...[output truncated]...", evidence["stderr"])
                else:
                    self.assertEqual(evidence["stderr"], stderr)

    def test_run_phase_keeps_large_output_on_disk_and_bounds_result(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "phase.log"
            code = "import sys; sys.stdout.write('x' * 100000); sys.stderr.write('y' * 100000)"
            result = slicer_pr_validation._run_phase(
                [sys.executable, "-c", code], cwd=Path(temp), timeout=10, log=log)
            self.assertEqual(result.returncode, 0)
            self.assertFalse(result.timed_out)
            self.assertEqual(result.stderr, "")
            self.assertLessEqual(
                len(result.stdout), slicer_pr_validation.MAX_OUTPUT + 80)
            self.assertGreater(log.stat().st_size, 190000)

    def test_run_phase_marks_utf8_byte_tail_as_truncated(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "phase.log"
            code = "import sys; sys.stdout.buffer.write('😀'.encode() * 20000)"
            result = slicer_pr_validation._run_phase(
                [sys.executable, "-c", code], cwd=Path(temp), timeout=10, log=log)

            self.assertGreater(log.stat().st_size, slicer_pr_validation.MAX_OUTPUT)
            self.assertLess(len(result.stdout), slicer_pr_validation.MAX_OUTPUT)
            evidence = result.json()
            self.assertTrue(evidence["stdout_truncated"])
            self.assertFalse(evidence["stderr_truncated"])


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


class PatchScopeTests(unittest.TestCase):
    @staticmethod
    def _git(repo, *args):
        return subprocess.run(["git", *args], cwd=repo, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)

    def _fixture(self, root):
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "Test")
        (repo / "tests").mkdir()
        (repo / "scripts").mkdir()
        (repo / "src").mkdir()
        (repo / "cmake").mkdir()
        (repo / "tests/harness.txt").write_text("base\n", encoding="utf-8")
        (repo / "tests/blob.bin").write_bytes(b"base\x00\xff")
        (repo / "src/production.cpp").write_text("base\n", encoding="utf-8")
        (repo / "src/CMakeLists.txt").write_text("base\n", encoding="utf-8")
        (repo / "CMakeLists.txt").write_text("base\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        (repo / "tests/harness.txt").write_text("harness\n", encoding="utf-8")
        (repo / "tests/blob.bin").write_bytes(b"harness\x00\x01\xfe")
        (repo / "src/production.cpp").write_text("production change\n", encoding="utf-8")
        (repo / "src/CMakeLists.txt").write_text("integration change\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "harness and production")
        ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        return repo, base, ref

    def test_test_only_scope_preserves_production_and_binary_patch(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, ref = self._fixture(Path(temp))
            test_paths = slicer_pr_validation._scope_pathspecs("test-only", [])
            patch, result = slicer_pr_validation._git_diff(repo, base, ref, test_paths)
            self.assertEqual(result.returncode, 0)
            self.assertIsNotNone(patch)
            self.assertIn(b"tests/harness.txt", patch)
            self.assertIn(b"GIT binary patch", patch)
            self.assertNotIn(b"src/production.cpp", patch)
            self.assertNotIn(b"src/CMakeLists.txt", patch)

            worktree = Path(temp) / "worktree"
            self._git(repo, "worktree", "add", "--detach", str(worktree), base)
            patch_path = Path(temp) / "test-only.patch"
            patch_path.write_bytes(patch)
            before, _ = slicer_pr_validation._tracked_source_index_hash(worktree)
            apply = slicer_pr_validation._git(worktree, "apply", "--3way", "--index", str(patch_path))
            self.assertEqual(apply.returncode, 0, apply.stderr)
            self.assertEqual((worktree / "tests/harness.txt").read_text(encoding="utf-8"), "harness\n")
            self.assertEqual((worktree / "tests/blob.bin").read_bytes(), b"harness\x00\x01\xfe")
            after, _ = slicer_pr_validation._tracked_source_index_hash(worktree)
            self.assertEqual(before, after)
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_integration_and_full_scopes_are_explicit(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, ref = self._fixture(Path(temp))
            integration, result = slicer_pr_validation._git_diff(
                repo, base, ref, slicer_pr_validation._scope_pathspecs("integration", []))
            self.assertEqual(result.returncode, 0)
            self.assertIn(b"src/CMakeLists.txt", integration)
            self.assertNotIn(b"src/production.cpp", integration)
            full, result = slicer_pr_validation._git_diff(repo, base, ref, None)
            self.assertEqual(result.returncode, 0)
            self.assertIn(b"src/production.cpp", full)
            explicit, result = slicer_pr_validation._git_diff(
                repo, base, ref, slicer_pr_validation._scope_pathspecs("test-only", ["src/CMakeLists.txt"]))
            self.assertEqual(result.returncode, 0)
            self.assertIn(b"src/CMakeLists.txt", explicit)

    def test_diff_paths_and_literal_include_preserve_special_names(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, _, ref = self._fixture(Path(temp))
            (repo / "docs").mkdir()
            selected = "docs/special [x]*☃.txt"
            sibling = "docs/special ax☃.txt"
            (repo / selected).write_text("selected\n", encoding="utf-8")
            (repo / sibling).write_text("sibling\n", encoding="utf-8")
            self._git(repo, "add", ".")
            self._git(repo, "commit", "-qm", "special names")
            special_ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()

            paths, result = slicer_pr_validation._git_diff_paths(repo, ref, special_ref)
            self.assertEqual(result.returncode, 0)
            self.assertEqual(paths, [selected, sibling])
            patch, result = slicer_pr_validation._git_diff(
                repo, ref, special_ref,
                slicer_pr_validation._scope_pathspecs("test-only", [selected]))
            self.assertEqual(result.returncode, 0)
            self.assertTrue(any(":(top,literal)" in arg for arg in result.argv))
            self.assertIn(b"+selected\n", patch)
            self.assertNotIn(b"+sibling\n", patch)


class RunValidationOrchestrationTests(unittest.TestCase):
    @staticmethod
    def _git(repo, *args):
        return subprocess.run(["git", *args], cwd=repo, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              text=True)

    def _fixture(self, root):
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "Test")
        (repo / "tests").mkdir()
        (repo / "src").mkdir()
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "tests/harness.txt").write_text("base\n", encoding="utf-8")
        (repo / "src/production.cpp").write_text("production\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        (repo / "tests/harness.txt").write_text("harness\n", encoding="utf-8")
        self._git(repo, "add", "tests/harness.txt")
        self._git(repo, "commit", "-qm", "harness")
        harness_ref = self._git(repo, "rev-parse", "HEAD").stdout.strip()

        self._git(repo, "checkout", "-q", "-b", "head", base)
        (repo / "README.txt").write_text("head-only context\n", encoding="utf-8")
        self._git(repo, "add", "README.txt")
        self._git(repo, "commit", "-qm", "head")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        return repo, base, harness_ref, head

    def _args(self, repo, snapshot, output):
        args = slicer_pr_validation._parser().parse_args([
            "--root", str(repo),
            "--snapshot", str(snapshot),
            "--harness-base", "HEAD~1~1",
            "--harness-ref", "HEAD~1",
            "--deps-prefix", str(repo / "deps"),
            "--output-dir", str(output),
            "--build-dir", "build",
            "--jobs", "1",
            "--timeout", "1",
        ])
        args.targets = slicer_pr_validation.DEFAULT_TARGETS
        args.uv = "uv"
        return args

    def _write_snapshot(self, root, head, base):
        snapshot = root / "snapshot.json"
        snapshot.write_text(json.dumps([{
            "number": 1, "headRefName": "head", "headRefOid": head,
            "baseRefOid": base,
        }]), encoding="utf-8")
        return snapshot

    def _write_baseline_expectation_manifest(self, root, repo, harness_ref, statuses):
        tree = self._git(repo, "rev-parse", f"{harness_ref}^{{tree}}").stdout.strip()
        source_hash, source_result = slicer_pr_validation._tracked_source_index_hash(repo)
        self.assertEqual(source_result.returncode, 0)
        path = root / "baseline-expectation.json"
        path.write_text(json.dumps({
            "schema_version": 1,
            "harness_ref_sha": harness_ref,
            "harness_ref_tree_sha": tree,
            "tracked_source_index_sha256": source_hash,
            "testcase_statuses": statuses,
        }), encoding="utf-8")
        return path

    def _make_seed_worktree(self, root, repo, ref, *, build_dir="build"):
        worktree = root / "seed" / "repo"
        worktree.parent.mkdir()
        self._git(repo, "worktree", "add", "--quiet", "--detach", str(worktree), ref)
        build = worktree / build_dir
        build.mkdir()
        (build / "cache.marker").write_text("retained build cache\n", encoding="utf-8")
        return worktree.resolve()

    @staticmethod
    def _fake_native_phases():
        calls = []

        def fake_phase(argv, *, cwd, timeout, log):
            calls.append((Path(cwd), list(argv), log.name))
            log.parent.mkdir(parents=True, exist_ok=True)
            if argv and argv[0] == "ctest":
                junit = Path(argv[argv.index("--output-junit") + 1])
                junit.write_text(
                    '<testsuite tests="1" failures="0" errors="0" skipped="0">'
                    '<testcase classname="suite" name="ordinary baseline"/></testsuite>',
                    encoding="utf-8")
                output = "100% tests passed, 0 tests failed out of 1\n"
                log.write_text(output, encoding="utf-8")
                return slicer_pr_validation.CommandResult(list(argv), 0, 0.01, output, "")
            log.write_text("", encoding="utf-8")
            return slicer_pr_validation.CommandResult(list(argv), 0, 0.01, "", "")

        return calls, fake_phase

    def test_report_shows_active_rows_while_fake_children_are_running(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            started_dir = root / "fake-children-started"
            release_dir = root / "release-fake-children"
            started_dir.mkdir()
            release_dir.mkdir()
            active_runs = ("baseline", "reference", "pr-1")
            child_script = (
                "import pathlib,sys,time\n"
                "phase,run_name=sys.argv[1:3]\n"
                "started=pathlib.Path(sys.argv[3]); release=pathlib.Path(sys.argv[4])\n"
                "if phase == 'configure' and run_name in ('baseline','reference','pr-1'):\n"
                " marker=started / run_name; gate=release / run_name\n"
                " if not marker.exists():\n"
                "  marker.touch()\n"
                "  while not gate.exists(): time.sleep(0.01)\n"
                "if phase == 'test':\n"
                " pathlib.Path(sys.argv[5]).write_text('<testsuite tests=\"1\" failures=\"0\" errors=\"0\" skipped=\"0\"><testcase classname=\"suite\" name=\"fake child test\"/></testsuite>', encoding='utf-8')\n"
                " print('100% tests passed, 0 tests failed out of 1')\n"
            )

            def fake_phase(argv, *, cwd, timeout, log):
                phase = log.stem
                junit = str(Path(argv[argv.index("--output-junit") + 1])) if phase == "test" else ""
                completed = subprocess.run(
                    [sys.executable, "-c", child_script, phase, log.parent.name,
                     str(started_dir), str(release_dir), junit],
                    cwd=cwd, capture_output=True, text=True, timeout=timeout, check=False)
                log.parent.mkdir(parents=True, exist_ok=True)
                log.write_text(completed.stdout + completed.stderr, encoding="utf-8")
                return slicer_pr_validation.CommandResult(
                    list(argv), completed.returncode, 0.01, completed.stdout, completed.stderr)

            results = []
            errors = []
            finished = threading.Event()

            def run():
                try:
                    results.append(slicer_pr_validation.run_validation(args))
                except BaseException as exc:
                    errors.append(exc)
                finally:
                    finished.set()

            def wait_for_active_child(name):
                deadline = time.monotonic() + 10
                while not (started_dir / name).exists() and not finished.is_set() and time.monotonic() < deadline:
                    time.sleep(0.01)
                self.assertTrue((started_dir / name).exists(), f"fake {name} child did not reach the barrier")
                return json.loads((output / "report.json").read_text(encoding="utf-8"))

            worker = threading.Thread(target=run)
            with mock.patch.object(slicer_pr_validation, "_run_phase", side_effect=fake_phase):
                worker.start()
                try:
                    live = wait_for_active_child("baseline")
                    self.assertEqual(live["status"], "in_progress")
                    self.assertEqual(live["baseline"]["status"], "in_progress")
                    (release_dir / "baseline").touch()

                    live = wait_for_active_child("reference")
                    self.assertEqual(live["status"], "in_progress")
                    self.assertEqual(live["references"][base]["status"], "in_progress")
                    (release_dir / "reference").touch()

                    live = wait_for_active_child("pr-1")
                    self.assertEqual(live["status"], "in_progress")
                    self.assertEqual(live["heads"][0]["status"], "in_progress")
                finally:
                    for name in active_runs:
                        (release_dir / name).touch()
                    worker.join(timeout=30)
            self.assertFalse(worker.is_alive(), "fake validation thread did not finish")
            self.assertEqual(errors, [])
            self.assertEqual(results, [0])
            final = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(final["status"], "pass")
            self.assertEqual(final["baseline"]["status"], "pass")
            self.assertEqual(final["references"][base]["status"], "pass")
            self.assertEqual(final["heads"][0]["status"], "pass")

    def test_exception_marks_active_report_terminal_infrastructure_error(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "exception-evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=RuntimeError("fake child exception")):
                with self.assertRaisesRegex(RuntimeError, "fake child exception"):
                    slicer_pr_validation.run_validation(args)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "infrastructure_error")
            self.assertEqual(report["error"], "fake child exception")
            self.assertEqual(report["baseline"]["status"], "infrastructure_error")
            self.assertIn("finished_at_epoch", report)

    def test_seed_worktree_runs_real_phases_and_reuse_rechecks_final_state(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            output = root / "seed-evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)
            calls, fake_phase = self._fake_native_phases()
            with mock.patch.object(slicer_pr_validation, "_run_phase", side_effect=fake_phase):
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)

            report_path = output / "report.json"
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual([item[2] for item in calls],
                             ["configure.log", "build.log", "test.log"] * 3)
            self.assertTrue(all(cwd == worktree for cwd, _, _ in calls))
            self.assertEqual(report["baseline"]["status"], "pass")
            self.assertEqual(report["references"][base]["status"], "pass")
            self.assertEqual(report["heads"][0]["status"], "pass")
            self.assertNotIn("add", report["worktree"])
            self.assertEqual(report["worktree"]["seed"]["producer"],
                             "seed_existing_worktree")
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "passed")
            self.assertTrue(report["worktree"]["final_state"]["available"])
            self.assertEqual(report["cleanup"]["reason"], "--seed-worktree")
            self.assertTrue(report["cleanup"]["lock_released"])
            self.assertTrue((worktree / "build/cache.marker").is_file())
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(), head)

            reuse_args = self._args(repo, snapshot, root / "reuse-evidence")
            reuse_args.harness_base = base
            reuse_args.harness_ref = harness_ref
            reuse_args.reuse_worktree_from_report = str(report_path)
            reuse_calls, fake_reuse_phase = self._fake_native_phases()
            with mock.patch.object(slicer_pr_validation, "_run_phase",
                                   side_effect=fake_reuse_phase):
                self.assertEqual(slicer_pr_validation.run_validation(reuse_args), 0)
            self.assertEqual(len(reuse_calls), 9)
            reuse_report = json.loads(
                (root / "reuse-evidence/report.json").read_text(encoding="utf-8"))
            self.assertTrue(reuse_report["worktree"]["reused"])
            self.assertEqual(reuse_report["worktree"]["path"], str(worktree))
            self.assertTrue(reuse_report["cleanup"]["lock_released"])

            tracked_file = worktree / "README.txt"
            tracked_file.write_text("modified after seeded run\n", encoding="utf-8")
            rejected_args = self._args(repo, snapshot, root / "changed-evidence")
            rejected_args.harness_base = base
            rejected_args.harness_ref = harness_ref
            rejected_args.reuse_worktree_from_report = str(report_path)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "tracked tree/status differs"):
                    slicer_pr_validation.run_validation(rejected_args)
            run_phase.assert_not_called()
            self.assertEqual(tracked_file.read_text(encoding="utf-8"),
                             "modified after seeded run\n")
            self.assertTrue(worktree.is_dir())
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_wrong_head_and_preserves_it(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, base)
            args = self._args(repo, snapshot, root / "wrong-head-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "not detached at its recorded HEAD"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()
            report = json.loads((root / "wrong-head-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "pending")
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertFalse(report["cleanup"]["lock_acquired"])
            self.assertTrue((worktree / "build/cache.marker").is_file())
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(), base)
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_dirty_tracked_file_and_releases_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            tracked_file = worktree / "tests/harness.txt"
            tracked_file.write_text("preserve this dirty content\n", encoding="utf-8")
            git_dir = Path(self._git(
                worktree, "rev-parse", "--absolute-git-dir").stdout.strip())
            lock = git_dir / slicer_pr_validation._PR_REUSE_LOCK
            args = self._args(repo, snapshot, root / "dirty-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)

            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(
                        slicer_pr_validation.ValidationError,
                        "seed worktree must be clean at the exact harness-ref HEAD"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()

            report = json.loads((root / "dirty-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "pending")
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertTrue(report["cleanup"]["lock_acquired"])
            self.assertTrue(report["cleanup"]["lock_released"])
            self.assertFalse(lock.exists())
            self.assertEqual(tracked_file.read_text(encoding="utf-8"),
                             "preserve this dirty content\n")
            self.assertTrue((worktree / "build/cache.marker").is_file())
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(),
                             harness_ref)
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_foreign_common_dir_and_preserves_checkout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            foreign_repo = root / "foreign-repo"
            subprocess.run(["git", "clone", "--shared", "--quiet", str(repo),
                            str(foreign_repo)], check=True, cwd=root,
                           stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            worktree = root / "foreign-seed" / "repo"
            worktree.parent.mkdir()
            self._git(foreign_repo, "worktree", "add", "--quiet", "--detach",
                      str(worktree), harness_ref)
            worktree = worktree.resolve()
            build = worktree / "build"
            build.mkdir()
            cache_marker = build / "cache.marker"
            cache_marker.write_text("foreign cache stays intact\n", encoding="utf-8")
            args = self._args(repo, snapshot, root / "foreign-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)

            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(
                        slicer_pr_validation.ValidationError,
                        "different git common directory"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()

            report = json.loads((root / "foreign-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "pending")
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertFalse(report["cleanup"]["lock_acquired"])
            self.assertEqual(cache_marker.read_text(encoding="utf-8"),
                             "foreign cache stays intact\n")
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(),
                             harness_ref)
            foreign_git_dir = Path(self._git(
                worktree, "rev-parse", "--absolute-git-dir").stdout.strip())
            self.assertFalse(
                (foreign_git_dir / slicer_pr_validation._PR_REUSE_LOCK).exists())
            self.assertTrue(worktree.is_dir())
            self._git(foreign_repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_is_preserved_when_real_baseline_build_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            args = self._args(repo, snapshot, root / "failed-baseline-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)
            calls = []

            def fail_build(argv, *, cwd, timeout, log):
                calls.append((Path(cwd), list(argv)))
                log.parent.mkdir(parents=True, exist_ok=True)
                log.write_text("build failed\n" if "--build" in argv else "",
                               encoding="utf-8")
                returncode = 1 if "--build" in argv else 0
                return slicer_pr_validation.CommandResult(list(argv), returncode, 0.01, "", "")

            with mock.patch.object(slicer_pr_validation, "_run_phase", side_effect=fail_build):
                self.assertEqual(slicer_pr_validation.run_validation(args), 2)
            report = json.loads((root / "failed-baseline-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertEqual(report["baseline"]["status"], "build_failed")
            self.assertEqual(len(calls), 2)
            self.assertTrue(all(cwd == worktree for cwd, _ in calls))
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertTrue(report["cleanup"]["lock_released"])
            self.assertTrue((worktree / "build/cache.marker").is_file())
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(),
                             harness_ref)
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_existing_lock_and_preserves_worktree(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            git_dir = Path(self._git(worktree, "rev-parse", "--absolute-git-dir").stdout.strip())
            lock = git_dir / slicer_pr_validation._PR_REUSE_LOCK
            lock.write_text("existing owner\n", encoding="utf-8")
            args = self._args(repo, snapshot, root / "locked-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "lock exists"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()
            report = json.loads((root / "locked-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertFalse(report["cleanup"]["lock_acquired"])
            self.assertEqual(lock.read_text(encoding="utf-8"), "existing owner\n")
            self.assertTrue((worktree / "build/cache.marker").is_file())
            lock.unlink()
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_untracked_files_outside_build(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            unexpected = worktree / "unexpected.txt"
            unexpected.write_text("keep me\n", encoding="utf-8")
            args = self._args(repo, snapshot, root / "untracked-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(worktree)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "unexpected untracked files outside retained build"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()
            self.assertEqual(unexpected.read_text(encoding="utf-8"), "keep me\n")
            report = json.loads((root / "untracked-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "pending")
            self.assertTrue((worktree / "build/cache.marker").is_file())
            unexpected.unlink()
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_rejects_symlink_alias_before_creating_output(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref)
            alias = root / "seed-alias"
            alias.symlink_to(worktree.parent, target_is_directory=True)
            args = self._args(repo, snapshot, root / "alias-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.seed_worktree = str(alias / worktree.name)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "canonical path"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()
            self.assertFalse((root / "alias-evidence").exists())
            self.assertTrue((worktree / "build/cache.marker").is_file())
            self.assertEqual(self._git(worktree, "rev-parse", "HEAD").stdout.strip(),
                             harness_ref)
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_seed_worktree_requires_an_ignored_existing_build_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            worktree = self._make_seed_worktree(root, repo, harness_ref, build_dir="cache")
            args = self._args(repo, snapshot, root / "nonignored-build-evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.build_dir = "cache"
            args.seed_worktree = str(worktree)
            with mock.patch.object(slicer_pr_validation, "_run_phase") as run_phase:
                with self.assertRaisesRegex(slicer_pr_validation.ValidationError,
                                            "not ignored"):
                    slicer_pr_validation.run_validation(args)
            run_phase.assert_not_called()
            report = json.loads((root / "nonignored-build-evidence/report.json").read_text(
                encoding="utf-8"))
            self.assertTrue(report["cleanup"]["preserved"])
            self.assertTrue(report["cleanup"]["lock_released"])
            self.assertEqual(report["worktree"]["seed"]["validation_status"], "pending")
            self.assertTrue((worktree / "cache/cache.marker").is_file())
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_nonempty_output_directory_is_rejected_before_worktree_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            output.mkdir()
            (output / "stale-report.json").write_text("stale\n", encoding="utf-8")
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation.run_validation(args)

    def test_missing_snapshot_oid_fails_preflight_before_any_tree_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, _ = self._fixture(root)
            snapshot = root / "snapshot.json"
            missing = "0" * 40
            snapshot.write_text(json.dumps([{
                "number": 1, "headRefName": "missing", "headRefOid": missing,
                "baseRefOid": base,
            }]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                with self.assertRaises(slicer_pr_validation.ValidationError):
                    slicer_pr_validation.run_validation(args)
            run_tree.assert_not_called()
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["preflight_heads"][0]["requested_oid"], missing)
            self.assertIsNone(report["preflight_heads"][0]["resolved_oid"])

    def test_missing_snapshot_base_fails_preflight_before_any_tree_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            missing_base = "0" * 40
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([{
                "number": 1, "headRefName": "head", "headRefOid": head,
                "baseRefOid": missing_base,
            }]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                with self.assertRaises(slicer_pr_validation.ValidationError):
                    slicer_pr_validation.run_validation(args)
            run_tree.assert_not_called()
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["preflight_heads"][0]["resolved_oid"], head)
            self.assertEqual(report["preflight_bases"][0]["requested_oid"], missing_base)
            self.assertIsNone(report["preflight_bases"][0]["resolved_oid"])

    def test_report_records_snapshot_digest_and_preflight_heads(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            args.test_jobs = 8
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["configuration"]["jobs"], 1)
            self.assertEqual(report["configuration"]["test_jobs"], 8)
            self.assertEqual(report["configuration"]["test_timeout_seconds"], 180.0)
            self.assertEqual(
                report["configuration"]["timeout_provenance"]["ctest_option"], "--timeout")
            self.assertEqual(
                report["configuration"]["timeout_provenance"][
                    "default_ctest_per_case_timeout_seconds"], 180.0)
            self.assertEqual(
                report["snapshot_sha256"], hashlib.sha256(snapshot.read_bytes()).hexdigest())
            self.assertEqual(len(report["preflight_heads"]), 1)
            preflight = report["preflight_heads"][0]
            self.assertEqual(preflight["number"], 1)
            self.assertEqual(preflight["requested_oid"], head)
            self.assertEqual(preflight["resolved_oid"], head)
            self.assertEqual(preflight["resolution"]["returncode"], 0)

    def test_snapshot_digest_uses_bytes_loaded_before_path_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            original_bytes = snapshot.read_bytes()
            changed_bytes = json.dumps([{
                "number": 1,
                "headRefName": "head",
                "headRefOid": head,
                "baseRefOid": base,
                "changed_after_load": True,
            }]).encode("utf-8")
            real_load_snapshot = slicer_pr_validation.load_snapshot

            def replace_path_after_read(path, *, content=None):
                snapshot.write_bytes(changed_bytes)
                return real_load_snapshot(path, content=content)

            with mock.patch.object(slicer_pr_validation, "load_snapshot",
                                   side_effect=replace_path_after_read) as load_snapshot, \
                 mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 0)
            load_snapshot.assert_called_once()
            self.assertEqual(load_snapshot.call_args.kwargs["content"], original_bytes)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            recorded = report["snapshot_sha256"]
            self.assertEqual(recorded, hashlib.sha256(original_bytes).hexdigest())
            self.assertNotEqual(recorded, hashlib.sha256(changed_bytes).hexdigest())

    def test_test_slots_do_not_raise_compiler_parallelism(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            worktree = root / "repo"
            worktree.mkdir()
            args = self._args(worktree, root / "snapshot.json", root / "evidence")
            args.jobs = 4
            build_path = worktree / args.build_dir
            for test_jobs in (None, 8):
                with self.subTest(test_jobs=test_jobs):
                    args.test_jobs = test_jobs
                    calls = []

                    def fake_phase(argv, *, cwd, timeout, log):
                        calls.append(list(argv))
                        log.parent.mkdir(parents=True, exist_ok=True)
                        if argv[0] == "ctest":
                            junit = Path(argv[argv.index("--output-junit") + 1])
                            junit.write_text('<testsuite tests="1"><testcase name="contract" /></testsuite>')
                            log.write_text("100% tests passed, 0 tests failed out of 1\n",
                                           encoding="utf-8")
                            return slicer_pr_validation.CommandResult(
                                list(argv), 0, 0.01, "100% tests passed, 0 tests failed out of 1", "")
                        log.write_text("", encoding="utf-8")
                        return slicer_pr_validation.CommandResult(list(argv), 0, 0.01, "", "")

                    build = slicer_pr_validation._default_build(worktree, build_path, args.jobs, args.targets)
                    with mock.patch.object(slicer_pr_validation, "_run_phase", side_effect=fake_phase):
                        result = slicer_pr_validation._run_tree(
                            worktree, root, f"run-{test_jobs}", args, ["cmake-configure"], build,
                            commit_sha="a" * 40, tree_sha="b" * 40)
                    self.assertEqual(result["status"], "pass")
                    self.assertEqual(calls[1][calls[1].index("--parallel") + 1], "4")
                    self.assertEqual(calls[2][calls[2].index("--parallel") + 1], str(test_jobs or 4))
                    self.assertEqual(calls[2][calls[2].index("--timeout") + 1], "180.0")
                    self.assertEqual(result["test_timeout_seconds"], 180.0)

    def test_run_tree_requires_junit_when_ctest_console_reports_success(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            args = self._args(repo, snapshot, root / "evidence")
            worktree = root / "worktree"
            worktree.mkdir()
            root_output = root / "runs"
            root_output.mkdir()

            def run_case(name, malformed):
                configure = slicer_pr_validation.CommandResult(
                    ["cmake"], 0, 0.01, "", "")
                build = slicer_pr_validation.CommandResult(
                    ["cmake-build"], 0, 0.01, "", "")
                test = slicer_pr_validation.CommandResult(
                    ["ctest"], 0, 0.01,
                    "100% tests passed, 0 tests failed out of 1", "")
                results = [configure, build, test]

                def fake_phase(argv, *, cwd, timeout, log):
                    result = results.pop(0)
                    if log.name == "test.log" and malformed:
                        log.parent.joinpath("tests.junit.xml").write_text(
                            "<testsuite>", encoding="utf-8")
                    return result

                with mock.patch.object(slicer_pr_validation, "_run_phase",
                                       side_effect=fake_phase):
                    entry = slicer_pr_validation._run_tree(
                        worktree, root_output, name, args,
                        ["configure"], ["build"],
                        commit_sha=head, tree_sha="tree")
                self.assertEqual(entry["status"], "test_evidence_incomplete")
                self.assertFalse(entry["junit_reference_usable"])
                self.assertFalse(entry["junit_evidence"]["usable"])

            run_case("absent", False)
            run_case("malformed", True)

    @staticmethod
    def _fake_run_tree(worktree, root_output, name, args, configure_args, build_args, **kwargs):
        evidence = {"counts": {"tests": 1}}
        common = {
            "junit_evidence": evidence,
            "minimum_tests_required": getattr(args, "minimum_tests", 1),
            "minimum_tests_observed": 1,
            "minimum_tests_ok": True,
            "missing_baseline_testcases": [],
        }
        if name == "baseline":
            return {
                **common,
                "name": name,
                "status": "pass",
                "junit_reference_usable": True,
                "junit_case_statuses": {"suite::base regression": "passed"},
                "failed_test_names": [],
            }
        if name == "reference" or name.startswith("reference-base-"):
            return {
                **common,
                "name": name,
                "status": "test_failed",
                "junit_reference_usable": True,
                "junit_case_statuses": {"suite::base regression": "failed"},
                "failed_test_names": ["suite::base regression"],
            }
        return {
            **common,
            "name": name,
            "status": "pass",
            "junit_reference_usable": True,
            "junit_case_statuses": {"suite::base regression": "passed"},
            "failed_test_names": [],
        }

    @staticmethod
    def _baseline_policy_run(name, statuses, *, status=None):
        counts = {
            "tests": len(statuses),
            "failures": sum(value == "failed" for value in statuses.values()),
            "errors": 0, "skipped": 0, "timeouts": 0,
        }
        actual_status = status or ("test_failed" if counts["failures"] else "pass")
        return {
            "name": name,
            "status": actual_status,
            "junit_exists": True,
            "junit_evidence": {
                "usable": True, "errors": [], "duplicate_testcase_identities": [],
                "counts": counts,
            },
            "junit_reference_usable": actual_status in {"pass", "test_failed"},
            "junit_case_statuses": dict(statuses),
            "failed_test_names": sorted(
                identity for identity, value in statuses.items() if value == "failed"),
            "test_log_evidence_available": True,
            "test_log_evidence_error": None,
            "test_evidence": len(statuses),
            "ctest_summary": {
                "tests_failed": counts["failures"], "tests_total": counts["tests"],
            },
            "no_test_evidence": False,
            "junit_timeout_evidence": 0,
            "skipped_test_evidence": 0,
            "test_timeout_source": None,
            "commands": {
                "configure": {"returncode": 0, "timed_out": False},
                "build": {"returncode": 0, "timed_out": False},
                "test": {"returncode": 8 if counts["failures"] else 0,
                          "timed_out": False},
            },
            "minimum_tests_observed": len(statuses),
            "minimum_tests_ok": True,
            "missing_baseline_testcases": [],
        }

    def test_qualified_failed_baseline_runs_exact_comparisons_without_hiding_failures(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            expected = {
                "suite::baseline pass": "passed",
                "suite::previous failure": "failed",
                "suite::previous failure now resolved": "failed",
            }
            manifest = self._write_baseline_expectation_manifest(
                root, repo, harness_ref, expected)
            args = self._args(repo, snapshot, root / "evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.baseline_expectation_manifest = str(manifest)
            names = []

            def run_tree(worktree, output, name, _args, _configure, _build, **_kwargs):
                names.append(name)
                if name == "baseline":
                    return self._baseline_policy_run(name, expected)
                if name == "reference":
                    reference_statuses = dict(expected)
                    reference_statuses["suite::new failure"] = "passed"
                    return self._baseline_policy_run(name, reference_statuses)
                head_statuses = {
                    "suite::baseline pass": "passed",
                    "suite::previous failure": "failed",
                    "suite::previous failure now resolved": "passed",
                    "suite::new failure": "failed",
                }
                return self._baseline_policy_run(name, head_statuses)

            with mock.patch.object(slicer_pr_validation, "_run_tree", side_effect=run_tree):
                result = slicer_pr_validation.run_validation(args)
            report = json.loads((Path(args.output_dir) / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(result, 2)
            self.assertEqual(names, ["baseline", "reference", "pr-1"])
            self.assertEqual(report["baseline"]["status"], "test_failed")
            self.assertEqual(report["baseline"]["failed_test_names"],
                             ["suite::previous failure",
                              "suite::previous failure now resolved"])
            self.assertEqual(report["baseline_expectation"]["status"], "qualified")
            self.assertEqual(report["status"], "test_failed")
            self.assertEqual(report["baseline_failure_comparison"]["status"],
                             "new_failing_testcases")
            self.assertEqual(report["baseline_failure_comparison"]["new_failing_testcases_by_head"],
                             {"1": ["suite::new failure"]})
            self.assertEqual(report["heads"][0]["failure_classification"]["inherited"],
                             ["suite::previous failure"])
            self.assertEqual(report["heads"][0]["failure_classification"]["resolved"],
                             ["suite::previous failure now resolved"])

    def test_policy_status_map_change_blocks_comparison_and_preserves_raw_baseline(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            expected = {
                "suite::baseline pass": "passed",
                "suite::previous failure": "failed",
            }
            manifest = self._write_baseline_expectation_manifest(
                root, repo, harness_ref, expected)
            args = self._args(repo, snapshot, root / "evidence")
            args.harness_base = base
            args.harness_ref = harness_ref
            args.baseline_expectation_manifest = str(manifest)
            names = []

            def changed_baseline(_worktree, _output, name, _args, _configure, _build, **_kwargs):
                names.append(name)
                return self._baseline_policy_run(name, {
                    "suite::baseline pass": "failed",
                    "suite::previous failure": "passed",
                }, status="test_failed")

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=changed_baseline):
                result = slicer_pr_validation.run_validation(args)
            report = json.loads((Path(args.output_dir) / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(result, 2)
            self.assertEqual(names, ["baseline"])
            self.assertEqual(report["baseline"]["status"], "test_failed")
            self.assertEqual(report["baseline_expectation"]["status"], "rejected")
            self.assertTrue(report["baseline_expectation"]["status_mismatches"])
            self.assertEqual(report["references"], {})
            self.assertEqual(report["status"], "baseline_expectation_invalid")

    def test_no_reference_mode_does_not_run_or_require_reference_entries(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            args.reference_baseline = False
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["references"], {})
            self.assertEqual(report["reference"]["status"], "not_run")
            self.assertEqual([call.args[2] for call in run_tree.call_args_list],
                             ["baseline", "pr-1"])

    def test_each_snapshot_base_drives_reference_and_is_reported(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "base-two", base)
            (repo / "base-two.txt").write_text("base two\n", encoding="utf-8")
            self._git(repo, "add", "base-two.txt")
            self._git(repo, "commit", "-qm", "second base")
            base_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            self._git(repo, "checkout", "-q", "-b", "head-two", base_two)
            (repo / "README-two.txt").write_text("head two\n", encoding="utf-8")
            self._git(repo, "add", "README-two.txt")
            self._git(repo, "commit", "-qm", "second head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head", "headRefOid": head,
                 "baseRefOid": base},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": base_two},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(set(report["references"]), {base, base_two})
            self.assertEqual([item["reference_base_sha"] for item in report["heads"]],
                             [base, base_two])
            self.assertEqual([call.args[2] for call in run_tree.call_args_list], [
                "baseline", f"reference-base-{base[:12]}", "pr-1",
                f"reference-base-{base_two[:12]}", "pr-2",
            ])
            self.assertEqual(report["reference"]["status"], "multiple")

    def test_each_head_classification_uses_its_own_base_failure_set(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "base-two", base)
            (repo / "base-two.txt").write_text("base two\n", encoding="utf-8")
            self._git(repo, "add", "base-two.txt")
            self._git(repo, "commit", "-qm", "second base")
            base_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            self._git(repo, "checkout", "-q", "-b", "head-two", base_two)
            (repo / "README-two.txt").write_text("head two\n", encoding="utf-8")
            self._git(repo, "add", "README-two.txt")
            self._git(repo, "commit", "-qm", "second head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head", "headRefOid": head,
                 "baseRefOid": base},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": base_two},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref

            def comparison_run(*run_args, **run_kwargs):
                name = run_args[2]
                entry = self._fake_run_tree(*run_args, **run_kwargs)
                if name.startswith("reference-base-"):
                    if base[:12] in name:
                        case = "suite::base-a"
                    else:
                        case = "suite::base-b"
                    entry.update({"status": "test_failed",
                                  "junit_case_statuses": {"suite::base regression": "passed", case: "failed"},
                                  "failed_test_names": [case]})
                elif name == "pr-1":
                    case = "suite::base-a"
                    entry.update({"status": "test_failed",
                                  "junit_case_statuses": {"suite::base regression": "passed", case: "failed"},
                                  "failed_test_names": [case]})
                elif name == "pr-2":
                    case = "suite::base-b"
                    entry.update({"status": "test_failed",
                                  "junit_case_statuses": {"suite::base regression": "passed", case: "failed"},
                                  "failed_test_names": [case]})
                return entry

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=comparison_run):
                self.assertEqual(slicer_pr_validation.run_validation(args), 1)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["heads"][0]["failure_classification"]["inherited"],
                             ["suite::base-a"])
            self.assertEqual(report["heads"][1]["failure_classification"]["inherited"],
                             ["suite::base-b"])
            self.assertEqual(report["heads"][0]["failure_classification"]["new"], [])
            self.assertEqual(report["heads"][1]["failure_classification"]["new"], [])

    def test_shared_snapshot_base_runs_one_cached_reference(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "head-two", base)
            (repo / "README-two.txt").write_text("head two\n", encoding="utf-8")
            self._git(repo, "add", "README-two.txt")
            self._git(repo, "commit", "-qm", "second head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head", "headRefOid": head,
                 "baseRefOid": base},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": base},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(set(report["references"]), {base})
            self.assertEqual([call.args[2] for call in run_tree.call_args_list],
                             ["baseline", "reference", "pr-1", "pr-2"])

    def test_reference_run_for_future_head_can_seed_that_head_once(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head_one = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "head-two", base)
            (repo / "README-two.txt").write_text("head two\n", encoding="utf-8")
            self._git(repo, "add", "README-two.txt")
            self._git(repo, "commit", "-qm", "second head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head-one", "headRefOid": head_one,
                 "baseRefOid": head_two},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": base},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 1)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["heads"][1]["reused_from_exact_run"],
                             f"reference-base-{head_two[:12]}")
            self.assertEqual([call.args[2] for call in run_tree.call_args_list], [
                "baseline", f"reference-base-{head_two[:12]}", "pr-1",
                f"reference-base-{base[:12]}",
            ])

    def test_identity_policy_disables_exact_reference_reuse(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head_one = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "head-two", head_one)
            (repo / "README-three.txt").write_text("head three\n", encoding="utf-8")
            self._git(repo, "add", "README-three.txt")
            self._git(repo, "commit", "-qm", "third head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head-one", "headRefOid": head_one,
                 "baseRefOid": head_two},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": base},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            args.identity_manifest = str(root / "identity.json")
            identity_policy = {
                "required_harness_runtime_ids": [],
                "required_harness_cases": {},
                "reference_bases": {
                    oid: {
                        "base_sha": oid, "exception_group": None,
                        "exception_cases": {}, "exception_runtime_ids": [],
                        "required_original_source_names": [],
                    }
                    for oid in (head_two, base)
                },
                "heads": {
                    1: {"exception_cases": {}, "required_head_cases": {},
                        "exception_group": "none"},
                    2: {"exception_cases": {}, "required_head_cases": {},
                        "exception_group": "none"},
                },
                "selected_head_count": 2,
                "all_snapshot_heads_validated": True,
            }
            with mock.patch.object(slicer_pr_validation, "_load_identity_manifest",
                                   return_value=({}, "f" * 64)), \
                 mock.patch.object(slicer_pr_validation, "_validate_identity_manifest",
                                   return_value=identity_policy), \
                 mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)
            self.assertEqual([call.args[2] for call in run_tree.call_args_list], [
                "baseline", f"reference-base-{head_two[:12]}", "pr-1",
                f"reference-base-{base[:12]}", "pr-2",
            ])

    def test_prior_exact_head_can_seed_a_later_reference_base(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            self._git(repo, "checkout", "-q", "-b", "head-two", head)
            (repo / "README-three.txt").write_text("head three\n", encoding="utf-8")
            self._git(repo, "add", "README-three.txt")
            self._git(repo, "commit", "-qm", "third head")
            head_two = self._git(repo, "rev-parse", "HEAD").stdout.strip()
            snapshot = root / "snapshot.json"
            snapshot.write_text(json.dumps([
                {"number": 1, "headRefName": "head", "headRefOid": head,
                 "baseRefOid": base},
                {"number": 2, "headRefName": "head-two", "headRefOid": head_two,
                 "baseRefOid": head},
            ]), encoding="utf-8")
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                self.assertEqual(slicer_pr_validation.run_validation(args), 0)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["references"][head]["reused_from_head_number"], 1)
            self.assertEqual([call.args[2] for call in run_tree.call_args_list],
                             ["baseline", f"reference-base-{base[:12]}", "pr-1", "pr-2"])

    def test_reference_failure_does_not_suppress_head_and_src_stays_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree) as run_tree:
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 0)
            self.assertEqual([call.args[2] for call in run_tree.call_args_list],
                             ["baseline", "reference", "pr-1"])
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["reference"]["status"], "test_failed")
            self.assertEqual(report["heads"][0]["status"], "pass")
            self.assertEqual(report["heads"][0]["failure_classification"]["resolved"],
                             ["suite::base regression"])
            self.assertTrue(report["heads"][0]["tracked_source_index_unchanged"])

    def test_identity_gate_runs_before_head_failure_classification(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref

            def mismatched_head(*run_args, **run_kwargs):
                entry = self._fake_run_tree(*run_args, **run_kwargs)
                if run_args[2] == "pr-1":
                    entry["status"] = "test_failed"
                    entry["junit_case_statuses"] = {"suite::replacement": "failed"}
                    entry["failed_test_names"] = ["suite::replacement"]
                return entry

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=mismatched_head):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 2)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            head_entry = report["heads"][0]
            self.assertEqual(head_entry["status"], "test_evidence_incomplete")
            self.assertEqual(head_entry["missing_baseline_testcases"],
                             ["suite::base regression"])
            classification = head_entry["failure_classification"]
            self.assertFalse(classification["reference_available"])
            self.assertEqual(classification["new"], [])
            self.assertEqual(classification["inherited"], [])
            self.assertEqual(classification["resolved"], [])
            self.assertEqual(classification["uncompared_head_failures"],
                             ["suite::replacement"])

    def test_reference_identity_gap_is_incomplete_and_blocks_matrix_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref

            def mismatched_reference(*run_args, **run_kwargs):
                entry = self._fake_run_tree(*run_args, **run_kwargs)
                if run_args[2] == "reference":
                    entry["junit_case_statuses"] = {"suite::replacement": "failed"}
                    entry["failed_test_names"] = ["suite::replacement"]
                return entry

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=mismatched_reference):
                result = slicer_pr_validation.run_validation(args)
            self.assertEqual(result, 2)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["reference"]["status"], "test_evidence_incomplete")
            self.assertEqual(report["reference"]["missing_baseline_testcases"],
                             ["suite::base regression"])
            classification = report["heads"][0]["failure_classification"]
            self.assertFalse(classification["reference_available"])
            self.assertEqual(classification["new"], [])

    def test_original_checkout_change_is_fail_closed_with_nonzero_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref

            def mutate_original(*run_args, **run_kwargs):
                (repo / "concurrent-change.txt").write_text("changed\n", encoding="utf-8")
                return self._fake_run_tree(*run_args, **run_kwargs)

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=mutate_original):
                result = slicer_pr_validation.run_validation(args)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(result, 2)
            self.assertEqual(report["status"], "original_checkout_changed_during_run")

    def test_cleanup_failure_is_fail_closed_with_nonzero_result(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            real_git = slicer_pr_validation._git

            def fail_remove(git_repo, *git_args, **git_kwargs):
                if git_args[:3] == ("worktree", "remove", "--force"):
                    return slicer_pr_validation.CommandResult(
                        ["git", *git_args], 1, 0.01, "", "forced cleanup failure")
                return real_git(git_repo, *git_args, **git_kwargs)

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=self._fake_run_tree), \
                 mock.patch.object(slicer_pr_validation, "_git",
                                   side_effect=fail_remove):
                result = slicer_pr_validation.run_validation(args)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            self.assertEqual(result, 2)
            self.assertEqual(report["status"], "cleanup_failed")
            self.assertFalse(report["cleanup"]["removed"])
            # The mocked cleanup failure intentionally leaves the disposable
            # worktree registered; remove it after the fail-closed assertion.
            worktree = Path(report["worktree"]["path"])
            self._git(repo, "worktree", "remove", "--force", str(worktree))

    def test_keep_worktree_preserves_registered_worktree_and_build(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            args.keep_worktree = True

            def run_with_build_marker(worktree, *run_args, **run_kwargs):
                build = worktree / "build"
                build.mkdir(exist_ok=True)
                (build / "kept.txt").write_text("existing build output\n", encoding="utf-8")
                return self._fake_run_tree(worktree, *run_args, **run_kwargs)

            worktree = None
            try:
                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=run_with_build_marker):
                    result = slicer_pr_validation.run_validation(args)
                report = json.loads((output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(report["worktree"]["path"])
                self.assertEqual(result, 0)
                self.assertEqual(report["configuration"]["keep_worktree"], True)
                self.assertEqual(report["cleanup"], {
                    "preserved": True,
                    "path": str(worktree),
                    "parent_path": str(worktree.parent),
                    "reason": "--keep-worktree",
                })
                self.assertTrue(worktree.is_dir())
                self.assertTrue((worktree / "build/kept.txt").is_file())
                registered = self._git(repo, "worktree", "list", "--porcelain").stdout
                self.assertIn(str(worktree), registered)
                self.assertNotEqual(report["status"], "cleanup_failed")
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_report_keeps_build_and_reexecutes_all_selected_trees(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            first_output = root / "first-evidence"
            first_args = self._args(repo, snapshot, first_output)
            first_args.harness_base = base
            first_args.harness_ref = harness_ref
            first_args.keep_worktree = True

            def first_run_tree(worktree, root_output, name, *args, **kwargs):
                build = worktree / "build"
                build.mkdir(exist_ok=True)
                (build / "cache.marker").write_text("warm build\n", encoding="utf-8")
                result = self._fake_run_tree(worktree, root_output, name, *args, **kwargs)
                if name == "pr-1":
                    result["status"] = "test_timeout"
                    result["junit_case_statuses"] = {"suite::base regression": "timeout"}
                    result["timed_out_test_names"] = ["suite::base regression"]
                    result["junit_reference_usable"] = False
                return result

            worktree = None
            try:
                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=first_run_tree) as first_run:
                    self.assertEqual(slicer_pr_validation.run_validation(first_args), 2)
                prior = json.loads((first_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])
                self.assertEqual(first_run.call_count, 3)
                self.assertEqual(prior["status"], "infrastructure_error")
                self.assertEqual(prior["heads"][-1]["status"], "test_timeout")
                self.assertTrue(prior["cleanup"]["preserved"])
                self.assertEqual(prior["worktree"]["final_state"]["untracked_count"], 0)

                legacy = json.loads(json.dumps(prior))
                legacy["worktree"].pop("final_state")
                legacy_path = root / "legacy-report.json"
                legacy_path.write_text(json.dumps(legacy), encoding="utf-8")
                ambiguous = json.loads(json.dumps(legacy))
                ambiguous["heads"][-1].pop("tested_tree_sha")
                ambiguous_path = root / "ambiguous-report.json"
                ambiguous_path.write_text(json.dumps(ambiguous), encoding="utf-8")
                ambiguous_args = self._args(repo, snapshot, root / "ambiguous-evidence")
                ambiguous_args.harness_base = base
                ambiguous_args.harness_ref = harness_ref
                ambiguous_args.reuse_worktree_from_report = str(ambiguous_path)
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError,
                            "last-run index-tree binding"):
                        slicer_pr_validation.run_validation(ambiguous_args)
                run_tree.assert_not_called()
                self.assertTrue((worktree / "build/cache.marker").is_file())

                second_output = root / "second-evidence"
                second_args = self._args(repo, snapshot, second_output)
                second_args.harness_base = base
                second_args.harness_ref = harness_ref
                second_args.reuse_worktree_from_report = str(legacy_path)
                observed = []

                def reused_run_tree(actual_worktree, root_output, name, *args, **kwargs):
                    observed.append((actual_worktree.resolve(), name))
                    self.assertEqual(actual_worktree.resolve(), worktree.resolve())
                    self.assertEqual(
                        (actual_worktree / "build/cache.marker").read_text(encoding="utf-8"),
                        "warm build\n")
                    return self._fake_run_tree(
                        actual_worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=reused_run_tree) as second_run:
                    self.assertEqual(slicer_pr_validation.run_validation(second_args), 0)
                report = json.loads((second_output / "report.json").read_text(encoding="utf-8"))
                self.assertEqual(second_run.call_count, 3)
                self.assertEqual([name for _, name in observed], ["baseline", "reference", "pr-1"])
                self.assertTrue(all(path == worktree.resolve() for path, _ in observed))
                self.assertEqual(report["worktree"]["path"], str(worktree))
                self.assertTrue(report["worktree"]["reused"])
                self.assertTrue(report["cleanup"]["preserved"])
                self.assertTrue(report["cleanup"]["lock_acquired"])
                self.assertTrue(report["cleanup"]["lock_released"])
                self.assertEqual(report["worktree"]["prior_state_binding"]["source"], "last_head")
                self.assertTrue((worktree / "build/cache.marker").is_file())
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_rejects_untracked_change_before_running_and_preserves_checkout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            first_output = root / "first-evidence"
            first_args = self._args(repo, snapshot, first_output)
            first_args.harness_base = base
            first_args.harness_ref = harness_ref
            first_args.keep_worktree = True
            worktree = None
            try:
                def first_run_tree(worktree, root_output, name, *args, **kwargs):
                    build = worktree / "build"
                    build.mkdir(exist_ok=True)
                    (build / "cache.marker").write_text("warm build\n", encoding="utf-8")
                    return self._fake_run_tree(worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=first_run_tree):
                    self.assertEqual(slicer_pr_validation.run_validation(first_args), 0)
                prior = json.loads((first_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])
                unexpected = worktree / "unexpected.txt"
                unexpected.write_text("keep me\n", encoding="utf-8")

                second_args = self._args(repo, snapshot, root / "second-evidence")
                second_args.harness_base = base
                second_args.harness_ref = harness_ref
                second_args.reuse_worktree_from_report = str(first_output / "report.json")
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError,
                            "tracked tree/status differs"):
                        slicer_pr_validation.run_validation(second_args)
                run_tree.assert_not_called()
                self.assertEqual(unexpected.read_text(encoding="utf-8"), "keep me\n")
                self.assertTrue((worktree / "build/cache.marker").is_file())
                git_dir = Path(self._git(
                    worktree, "rev-parse", "--absolute-git-dir").stdout.strip())
                self.assertFalse((git_dir / slicer_pr_validation._PR_REUSE_LOCK).exists())
                failed = json.loads(
                    (root / "second-evidence/report.json").read_text(encoding="utf-8"))
                self.assertTrue(failed["cleanup"]["preserved"])
                self.assertTrue(failed["cleanup"]["lock_released"])

                unexpected.unlink()
                (repo / ".git/info/exclude").write_text("ignored-extra/\n", encoding="utf-8")
                ignored_extra = worktree / "ignored-extra/cache.bin"
                ignored_extra.parent.mkdir()
                ignored_extra.write_bytes(b"ignored but outside the retained build")
                third_args = self._args(repo, snapshot, root / "third-evidence")
                third_args.harness_base = base
                third_args.harness_ref = harness_ref
                third_args.reuse_worktree_from_report = str(first_output / "report.json")
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError,
                            "unexpected ignored files outside retained build"):
                        slicer_pr_validation.run_validation(third_args)
                run_tree.assert_not_called()
                self.assertEqual(ignored_extra.read_bytes(), b"ignored but outside the retained build")
                failed = json.loads(
                    (root / "third-evidence/report.json").read_text(encoding="utf-8"))
                self.assertTrue(failed["cleanup"]["preserved"])
                self.assertTrue(failed["cleanup"]["lock_released"])
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_rejects_tracked_change_before_running_and_preserves_checkout(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            first_output = root / "first-evidence"
            first_args = self._args(repo, snapshot, first_output)
            first_args.harness_base = base
            first_args.harness_ref = harness_ref
            first_args.keep_worktree = True
            worktree = None
            try:
                def first_run_tree(worktree, root_output, name, *args, **kwargs):
                    build = worktree / "build"
                    build.mkdir(exist_ok=True)
                    (build / "cache.marker").write_text("warm build\n", encoding="utf-8")
                    return self._fake_run_tree(worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=first_run_tree):
                    self.assertEqual(slicer_pr_validation.run_validation(first_args), 0)
                prior = json.loads((first_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])
                tracked = worktree / "tests/harness.txt"
                original = tracked.read_text(encoding="utf-8")
                tracked.write_text(original + "changed outside runner\n", encoding="utf-8")

                second_args = self._args(repo, snapshot, root / "second-evidence")
                second_args.harness_base = base
                second_args.harness_ref = harness_ref
                second_args.reuse_worktree_from_report = str(first_output / "report.json")
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError,
                            "tracked tree/status differs"):
                        slicer_pr_validation.run_validation(second_args)
                run_tree.assert_not_called()
                self.assertEqual(tracked.read_text(encoding="utf-8"),
                                 original + "changed outside runner\n")
                self.assertTrue((worktree / "build/cache.marker").is_file())
                failed = json.loads(
                    (root / "second-evidence/report.json").read_text(encoding="utf-8"))
                self.assertTrue(failed["cleanup"]["preserved"])
                self.assertTrue(failed["cleanup"]["lock_released"])
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_rejects_existing_lock_without_modifying_it(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            first_output = root / "first-evidence"
            first_args = self._args(repo, snapshot, first_output)
            first_args.harness_base = base
            first_args.harness_ref = harness_ref
            first_args.keep_worktree = True
            worktree = None
            try:
                def first_run_tree(worktree, root_output, name, *args, **kwargs):
                    build = worktree / "build"
                    build.mkdir(exist_ok=True)
                    (build / "cache.marker").write_text("warm build\n", encoding="utf-8")
                    return self._fake_run_tree(worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=first_run_tree):
                    self.assertEqual(slicer_pr_validation.run_validation(first_args), 0)
                prior = json.loads((first_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])
                git_dir = Path(self._git(worktree, "rev-parse", "--absolute-git-dir").stdout.strip())
                lock = git_dir / slicer_pr_validation._PR_REUSE_LOCK
                lock.write_text("existing owner\n", encoding="utf-8")

                second_args = self._args(repo, snapshot, root / "second-evidence")
                second_args.harness_base = base
                second_args.harness_ref = harness_ref
                second_args.reuse_worktree_from_report = str(first_output / "report.json")
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError, "lock exists"):
                        slicer_pr_validation.run_validation(second_args)
                run_tree.assert_not_called()
                self.assertEqual(lock.read_text(encoding="utf-8"), "existing owner\n")
                self.assertTrue((worktree / "build/cache.marker").is_file())
            finally:
                if worktree is not None and worktree.exists():
                    lock = Path(self._git(worktree, "rev-parse", "--absolute-git-dir").stdout.strip()) \
                        / slicer_pr_validation._PR_REUSE_LOCK
                    if lock.exists():
                        lock.unlink()
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_rejects_tracked_build_collision_before_reset(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            first_output = root / "first-evidence"
            first_args = self._args(repo, snapshot, first_output)
            first_args.harness_base = base
            first_args.harness_ref = harness_ref
            first_args.keep_worktree = True
            worktree = None
            try:
                def first_run_tree(worktree, root_output, name, *args, **kwargs):
                    build = worktree / "build"
                    build.mkdir(exist_ok=True)
                    (build / "cache.marker").write_text("warm build\n", encoding="utf-8")
                    return self._fake_run_tree(worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=first_run_tree):
                    self.assertEqual(slicer_pr_validation.run_validation(first_args), 0)
                prior = json.loads((first_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])

                self._git(repo, "checkout", "-q", "-b", "collision-harness", harness_ref)
                tracked_build_file = repo / "build/tracked.txt"
                tracked_build_file.parent.mkdir(exist_ok=True)
                tracked_build_file.write_text("tracked path collision\n", encoding="utf-8")
                self._git(repo, "add", "-f", "build/tracked.txt")
                self._git(repo, "commit", "-qm", "tracked build path")
                collision_harness = self._git(repo, "rev-parse", "HEAD").stdout.strip()
                self._git(repo, "checkout", "-q", "head")

                second_args = self._args(repo, snapshot, root / "collision-evidence")
                second_args.harness_base = base
                second_args.harness_ref = collision_harness
                second_args.reuse_worktree_from_report = str(first_output / "report.json")
                with mock.patch.object(slicer_pr_validation, "_run_tree") as run_tree:
                    with self.assertRaisesRegex(
                            slicer_pr_validation.ValidationError,
                            "collide with retained build"):
                        slicer_pr_validation.run_validation(second_args)
                run_tree.assert_not_called()
                self.assertEqual((worktree / "build/cache.marker").read_text(encoding="utf-8"),
                                 "warm build\n")
                self.assertFalse((worktree / "build/tracked.txt").exists())
                failed = json.loads(
                    (root / "collision-evidence/report.json").read_text(encoding="utf-8"))
                self.assertTrue(failed["cleanup"]["preserved"])
                self.assertTrue(failed["cleanup"]["lock_released"])
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_reuse_preserves_build_when_heads_stop_ignoring_it(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, original_head = self._fixture(root)
            prior_snapshot = self._write_snapshot(root, original_head, base)
            prior_output = root / "prior-evidence"
            prior_args = self._args(repo, prior_snapshot, prior_output)
            prior_args.harness_base = base
            prior_args.harness_ref = harness_ref
            prior_args.keep_worktree = True
            worktree = None
            try:
                def seed_build(worktree, root_output, name, *args, **kwargs):
                    build = worktree / "build"
                    build.mkdir(exist_ok=True)
                    marker = build / "cache.marker"
                    if not marker.exists():
                        marker.write_text("preserve through resets\n", encoding="utf-8")
                    return self._fake_run_tree(worktree, root_output, name, *args, **kwargs)

                with mock.patch.object(slicer_pr_validation, "_run_tree", side_effect=seed_build):
                    self.assertEqual(slicer_pr_validation.run_validation(prior_args), 0)
                prior = json.loads((prior_output / "report.json").read_text(encoding="utf-8"))
                worktree = Path(prior["worktree"]["path"])
                marker = worktree / "build/cache.marker"
                self.assertEqual(marker.read_text(encoding="utf-8"), "preserve through resets\n")

                heads = []
                for branch, filename in (("head-unignores-build-one", "one.txt"),
                                         ("head-unignores-build-two", "two.txt")):
                    self._git(repo, "checkout", "-q", "-b", branch, base)
                    (repo / ".gitignore").write_text("# build is no longer ignored\n",
                                                      encoding="utf-8")
                    (repo / filename).write_text(f"{filename}\n", encoding="utf-8")
                    self._git(repo, "add", ".gitignore", filename)
                    self._git(repo, "commit", "-qm", branch)
                    heads.append(self._git(repo, "rev-parse", "HEAD").stdout.strip())
                self._git(repo, "checkout", "-q", "head")
                snapshot = root / "unignored-heads.json"
                snapshot.write_text(json.dumps([
                    {"number": index, "headRefName": f"unignored-{index}",
                     "headRefOid": head, "baseRefOid": base}
                    for index, head in enumerate(heads, 1)
                ]), encoding="utf-8")

                reuse_args = self._args(repo, snapshot, root / "reuse-evidence")
                reuse_args.harness_base = base
                reuse_args.harness_ref = harness_ref
                reuse_args.reuse_worktree_from_report = str(prior_output / "report.json")
                observed = []

                def verify_build_survives(worktree, root_output, name, *args, **kwargs):
                    observed.append(name)
                    self.assertEqual(marker.read_text(encoding="utf-8"),
                                     "preserve through resets\n")
                    result = self._fake_run_tree(worktree, root_output, name, *args, **kwargs)
                    result["commands"] = {
                        "configure": {"returncode": 0, "timed_out": False},
                        "build": {"returncode": 0, "timed_out": False},
                    }
                    return result

                with mock.patch.object(slicer_pr_validation, "_run_tree",
                                       side_effect=verify_build_survives):
                    self.assertEqual(slicer_pr_validation.run_validation(reuse_args), 0)
                report = json.loads(
                    (root / "reuse-evidence/report.json").read_text(encoding="utf-8"))
                self.assertEqual(observed, ["baseline", "reference", "pr-1", "pr-2"])
                self.assertTrue(all(len(item["reset"]) == 1 for item in report["heads"]))
                for transition in [report["reference"], *report["heads"]]:
                    mtime = transition["build_input_mtime_preservation"]
                    self.assertEqual(mtime["scope"],
                                     "exact_applied_paths_regular_tracked_files_only")
                    self.assertIn(mtime["snapshot"]["status"], {"captured", "no_eligible_files"})
                    self.assertIn("status", mtime["restore"])
                self.assertTrue(marker.is_file())
                self.assertEqual(report["worktree"]["final_state"]["untracked_count"], 0)
                self.assertNotIn("?? build/", report["worktree"]["final_state"]["status"])
            finally:
                if worktree is not None and worktree.exists():
                    self._git(repo, "worktree", "remove", "--force", str(worktree))
                    worktree.parent.rmdir()

    def test_default_cleanup_still_removes_worktree_and_build(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head, base)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref

            def run_with_build_marker(worktree, *run_args, **run_kwargs):
                build = worktree / "build"
                build.mkdir(exist_ok=True)
                (build / "kept.txt").write_text("temporary build output\n", encoding="utf-8")
                return self._fake_run_tree(worktree, *run_args, **run_kwargs)

            with mock.patch.object(slicer_pr_validation, "_run_tree",
                                   side_effect=run_with_build_marker):
                result = slicer_pr_validation.run_validation(args)
            report = json.loads((output / "report.json").read_text(encoding="utf-8"))
            worktree = Path(report["worktree"]["path"])
            self.assertEqual(result, 0)
            self.assertEqual(report["configuration"]["keep_worktree"], False)
            self.assertTrue(report["cleanup"]["removed"])
            self.assertTrue(report["cleanup"]["parent_removed"])
            self.assertFalse(worktree.exists())
            self.assertFalse(worktree.parent.exists())


class OverlayResolutionTests(unittest.TestCase):
    @staticmethod
    def _git(repo, *args):
        return subprocess.run(["git", *args], cwd=repo, check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    def _fixture(self, root):
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "test@example.invalid")
        self._git(repo, "config", "user.name", "Test")
        (repo / "tests").mkdir()
        path = repo / "tests/overlay.txt"
        path.write_text("value=base\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.decode().strip()
        path.write_text("value=harness\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "harness")
        harness = self._git(repo, "rev-parse", "HEAD").stdout.decode().strip()
        self._git(repo, "checkout", "-qb", "pr", base)
        path.write_text("value=pr\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "pr")
        head = self._git(repo, "rev-parse", "HEAD").stdout.decode().strip()
        patch = self._git(repo, "diff", "--binary", "--full-index", base, harness,
                          "--", "tests/overlay.txt").stdout
        patch_path = root / "overlay.patch"
        patch_path.write_bytes(patch)
        patch_sha = hashlib.sha256(patch).hexdigest()
        # Record the real three-way stages from this exact base/head/overlay.
        apply = subprocess.run(["git", "apply", "--3way", "--index", str(patch_path)], cwd=repo,
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertNotEqual(apply.returncode, 0)
        stage_output = self._git(repo, "ls-files", "-u", "--", "tests/overlay.txt").stdout.decode()
        stages = {}
        for line in stage_output.splitlines():
            left, _, listed = line.partition("\t")
            mode, blob, stage = left.split()
            self.assertEqual(listed, "tests/overlay.txt")
            stages[stage] = blob
        self.assertEqual(set(stages), {"1", "2", "3"})
        self._git(repo, "reset", "--hard", head)
        manifest_dir = root / "manifest"
        manifest_dir.mkdir()
        payload = b"value=resolved\n"
        payload_path = manifest_dir / "resolved.txt"
        payload_path.write_bytes(payload)
        blob = hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()
        manifest = {
            "schema_version": 1,
            "resolutions": [{
                "base_sha": base, "head_sha": head, "overlay_patch_sha256": patch_sha,
                "paths": [{"path": "tests/overlay.txt", "stage_blobs": stages,
                           "resolved_path": "resolved.txt",
                           "resolved_sha256": hashlib.sha256(payload).hexdigest(),
                           "resolved_blob_sha1": blob}],
            }],
        }
        manifest_path = manifest_dir / "manifest.json"
        manifest_path.write_text(json.dumps(manifest), encoding="utf-8")
        args = Namespace(patch_scope="test-only", patch_include=[])
        return repo, base, harness, head, patch_path, patch_sha, manifest_path, args

    def test_exact_head_base_and_stage_payload_resolves_conflict(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            manifest, _ = slicer_pr_validation._load_overlay_resolution_manifest(
                manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])
            result = slicer_pr_validation._apply_overlay_patch(
                repo, patch, patch_sha256=patch_sha, base_sha=base, head_sha=head,
                resolution_manifest=manifest)
            self.assertTrue(result["applied"])
            self.assertEqual(result["status"], "resolved")
            self.assertEqual((repo / "tests/overlay.txt").read_bytes(), b"value=resolved\n")
            self.assertEqual(self._git(repo, "diff", "--name-only", "--diff-filter=U").stdout, b"")

    def test_matching_overlay_with_wrong_head_fails_closed_and_leaves_conflict(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            raw = json.loads(manifest_path.read_text())
            raw["resolutions"][0]["head_sha"] = "f" * 40
            manifest_path.write_text(json.dumps(raw), encoding="utf-8")
            manifest, _ = slicer_pr_validation._load_overlay_resolution_manifest(
                manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])
            result = slicer_pr_validation._apply_overlay_patch(
                repo, patch, patch_sha256=patch_sha, base_sha=base, head_sha=head,
                resolution_manifest=manifest)
            self.assertFalse(result["applied"])
            self.assertEqual(result["status"], "overlay_resolution_missing")
            self.assertEqual(self._git(repo, "diff", "--name-only", "--diff-filter=U").stdout.decode().strip(),
                             "tests/overlay.txt")

    def test_reference_resolution_pins_harness_overlay_to_reference_base(self):
        base = "b" * 40
        observed = {}
        command = slicer_pr_validation.CommandResult(["git"], 0, 0.0, "", "")

        def capture_overlay(worktree, patch_path, **kwargs):
            observed.update(kwargs)
            return {"applied": True}

        with tempfile.TemporaryDirectory() as temp, \
             mock.patch.object(slicer_pr_validation, "_reset_worktree", return_value=[]), \
             mock.patch.object(slicer_pr_validation, "_tracked_source_index_hash",
                               side_effect=[("index", command), ("index", command)]), \
             mock.patch.object(slicer_pr_validation, "_apply_overlay_patch", side_effect=capture_overlay), \
             mock.patch.object(slicer_pr_validation, "_tree_sha", return_value=("tree", command)), \
             mock.patch.object(slicer_pr_validation, "_run_tree", return_value={"status": "pass"}), \
             mock.patch.object(slicer_pr_validation, "_apply_minimum_test_gate"):
            result = slicer_pr_validation._run_reference_entry(
                Path(temp), Path(temp), "reference", base,
                Namespace(patch_scope="test-only", minimum_tests=1), (), (), Path(temp) / "overlay.patch",
                b"patch", "a" * 64, None, {})
        self.assertEqual(result["status"], "pass")
        self.assertEqual(observed["base_sha"], base)
        self.assertEqual(observed["head_sha"], base)

    def test_nominal_apply_with_unmerged_index_is_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            patch = root / "overlay.patch"
            patch.write_bytes(b"patch")
            apply_result = slicer_pr_validation.CommandResult(
                ["git", "apply"], 0, 0.0, "", "")
            unmerged_result = slicer_pr_validation.CommandResult(
                ["git", "ls-files", "-u"], 0, 0.0,
                "100644 " + "a" * 40 + " 1\ttests/overlay.txt\n", "")

            def fake_git(repo, *git_args, **kwargs):
                if git_args[:3] == ("apply", "--3way", "--index"):
                    return apply_result
                if git_args[:2] == ("ls-files", "-u"):
                    return unmerged_result
                raise AssertionError(git_args)

            with mock.patch.object(slicer_pr_validation, "_git", side_effect=fake_git):
                result = slicer_pr_validation._apply_overlay_patch(
                    root, patch, patch_sha256="a" * 64, base_sha="b" * 40,
                    head_sha="c" * 40, resolution_manifest=None)
            self.assertFalse(result["applied"])
            self.assertEqual(result["status"], "overlay_unmerged_after_apply")
            self.assertEqual(result["unresolved_paths"], ["tests/overlay.txt"])

    def test_overlay_payload_sha_mismatch_is_rejected_before_apply(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            raw = json.loads(manifest_path.read_text())
            raw["resolutions"][0]["overlay_patch_sha256"] = "0" * 64
            manifest_path.write_text(json.dumps(raw), encoding="utf-8")
            with self.assertRaisesRegex(slicer_pr_validation.ValidationError, "overlay patch SHA"):
                slicer_pr_validation._load_overlay_resolution_manifest(
                    manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])

    def test_wrong_base_row_fails_closed_before_payload_write(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            raw = json.loads(manifest_path.read_text())
            raw["resolutions"][0]["base_sha"] = "0" * 40
            manifest_path.write_text(json.dumps(raw), encoding="utf-8")
            manifest, _ = slicer_pr_validation._load_overlay_resolution_manifest(
                manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])
            result = slicer_pr_validation._apply_overlay_patch(
                repo, patch, patch_sha256=patch_sha, base_sha=base, head_sha=head,
                resolution_manifest=manifest)
            self.assertFalse(result["applied"])
            self.assertEqual(result["status"], "overlay_resolution_missing")
            self.assertEqual((repo / "tests/overlay.txt").read_text(),
                             "<<<<<<< ours\nvalue=pr\n=======\nvalue=harness\n>>>>>>> theirs\n")

    def test_unexpected_conflict_path_fails_before_payload_write(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            raw = json.loads(manifest_path.read_text())
            original = raw["resolutions"][0]["paths"][0]
            extra = dict(original)
            extra["path"] = "tests/unexpected.txt"
            raw["resolutions"][0]["paths"].append(extra)
            manifest_path.write_text(json.dumps(raw), encoding="utf-8")
            manifest, _ = slicer_pr_validation._load_overlay_resolution_manifest(
                manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])
            result = slicer_pr_validation._apply_overlay_patch(
                repo, patch, patch_sha256=patch_sha, base_sha=base, head_sha=head,
                resolution_manifest=manifest)
            self.assertFalse(result["applied"])
            self.assertEqual(result["status"], "overlay_resolution_invalid")
            self.assertIn("conflict paths mismatch", result["resolution_error"])
            self.assertEqual((repo / "tests/overlay.txt").read_text(),
                             "<<<<<<< ours\nvalue=pr\n=======\nvalue=harness\n>>>>>>> theirs\n")

    def test_overlay_resolution_statuses_are_infrastructure_failures(self):
        for status in ("overlay_resolution_missing", "overlay_resolution_invalid",
                       "overlay_state_inspection_failed", "overlay_unmerged_after_apply"):
            with self.subTest(status=status):
                report = {
                    "baseline": {"status": "pass"},
                    "configuration": {"reference_baseline": False},
                    "heads": [{"status": status}],
                }
                self.assertEqual(slicer_pr_validation._overall_status(report),
                                 ("infrastructure_error", 2))

    def test_stage_mismatch_is_rejected_before_resolved_bytes_are_written(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, base, harness, head, patch, patch_sha, manifest_path, args = self._fixture(Path(temp))
            raw = json.loads(manifest_path.read_text())
            raw["resolutions"][0]["paths"][0]["stage_blobs"]["2"] = "0" * 40
            manifest_path.write_text(json.dumps(raw), encoding="utf-8")
            manifest, _ = slicer_pr_validation._load_overlay_resolution_manifest(
                manifest_path, patch_sha256=patch_sha, scope="test-only", includes=[])
            result = slicer_pr_validation._apply_overlay_patch(
                repo, patch, patch_sha256=patch_sha, base_sha=base, head_sha=head,
                resolution_manifest=manifest)
            self.assertFalse(result["applied"])
            self.assertEqual(result["status"], "overlay_resolution_invalid")
            self.assertEqual(self._git(repo, "diff", "--name-only", "--diff-filter=U").stdout.decode().strip(),
                             "tests/overlay.txt")
            self.assertEqual((repo / "tests/overlay.txt").read_bytes(), b"<<<<<<< ours\nvalue=pr\n=======\nvalue=harness\n>>>>>>> theirs\n")


class OverlayMtimePreservationTests(unittest.TestCase):
    def _git(self, repo, *args, check=True):
        result = subprocess.run(["git", *args], cwd=repo, shell=False,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True, check=False)
        if check and result.returncode != 0:
            self.fail(f"git {' '.join(args)} failed: {result.stderr}")
        return result

    def _fixture(self, root):
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "mtime-test@example.invalid")
        self._git(repo, "config", "user.name", "mtime test")
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "input.txt").write_text("stable\n", encoding="utf-8")
        (repo / "nested").mkdir()
        (repo / "nested/leaf.txt").write_text("nested\n", encoding="utf-8")
        (repo / "copy.py").write_text(
            "import pathlib, sys\n"
            "source = pathlib.Path(sys.argv[1]).read_text(encoding='utf-8')\n"
            "generated = pathlib.Path(sys.argv[2]).read_text(encoding='utf-8')\n"
            "if source.strip() == 'fail':\n"
            "    raise SystemExit(7)\n"
            "pathlib.Path(sys.argv[3]).write_text(source + generated, encoding='utf-8')\n",
            encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "seed")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        worktree = root / "reuse"
        self._git(repo, "worktree", "add", "--detach", str(worktree), head)
        repo = repo.resolve()
        worktree = worktree.resolve()
        build = worktree / "build"
        build.mkdir()
        (build / "generated_config.hpp").write_text("config-v1\n", encoding="utf-8")
        self._write_ninja(build)
        lock = slicer_pr_validation._acquire_pr_validation_reuse_lock(repo, worktree, head)
        return repo, worktree, build, lock

    def _git_reset_overlay_fixture(self, root):
        repo = root / "git-cycle-repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "mtime-test@example.invalid")
        self._git(repo, "config", "user.name", "mtime test")
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "input.txt").write_text("stable\n", encoding="utf-8")
        (repo / "copy.py").write_text(
            "import pathlib, sys\n"
            "source = pathlib.Path(sys.argv[1]).read_text(encoding='utf-8')\n"
            "generated = pathlib.Path(sys.argv[2]).read_text(encoding='utf-8')\n"
            "pathlib.Path(sys.argv[3]).write_text(source + generated, encoding='utf-8')\n",
            encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "base")
        base = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        primary_branch = self._git(repo, "symbolic-ref", "--short", "HEAD").stdout.strip()
        (repo / "harness.txt").write_text("harness-only\n", encoding="utf-8")
        self._git(repo, "add", "harness.txt")
        self._git(repo, "commit", "-qm", "harness head")
        harness = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        self._git(repo, "switch", "-qc", "mtime-pr", base)
        (repo / "input.txt").write_text("pr-overlay\n", encoding="utf-8")
        self._git(repo, "add", "input.txt")
        self._git(repo, "commit", "-qm", "PR overlay")
        head = self._git(repo, "rev-parse", "HEAD").stdout.strip()
        patch_result = self._git(repo, "diff", "--binary", base, head, "--", "input.txt")
        patch_path = root / "actual-overlay.patch"
        patch_path.write_bytes(patch_result.stdout.encode("utf-8"))
        self._git(repo, "switch", "-q", primary_branch)
        worktree = root / "git-cycle-reuse"
        self._git(repo, "worktree", "add", "--detach", str(worktree), head)
        repo = repo.resolve()
        worktree = worktree.resolve()
        build = worktree / "build"
        build.mkdir()
        (build / "generated_config.hpp").write_text("config-v1\n", encoding="utf-8")
        self._write_ninja(build)
        lock = slicer_pr_validation._acquire_pr_validation_reuse_lock(repo, worktree, head)
        return repo.resolve(), worktree.resolve(), build, lock, base, harness, head, patch_path

    def _write_ninja(self, build):
        script = shlex.quote(sys.executable)
        (build / "build.ninja").write_text(
            "rule tiny_copy\n"
            f"  command = {script} ../copy.py ../input.txt generated_config.hpp output.txt\n"
            "  description = Write tiny output\n\n"
            "build output.txt: tiny_copy ../input.txt generated_config.hpp ../copy.py\n"
            "default output.txt\n", encoding="utf-8")

    def _ninja(self, build, *args, check=True):
        result = subprocess.run([shutil.which("ninja") or "ninja", "-C", str(build), *args],
                                shell=False, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, check=False)
        if check and result.returncode != 0:
            self.fail(f"ninja {' '.join(args)} failed: {result.stdout}")
        return result

    def _successful_run(self):
        command = {"returncode": 0, "timed_out": False}
        return {"commands": {"configure": dict(command), "build": dict(command)}}

    def _mtime(self, path, future_of=()):
        current = path.stat()
        later_than = [item.stat().st_mtime_ns for item in future_of if item.exists()]
        target = max([time.time_ns(), current.st_mtime_ns + 5_000_000_000,
                      *(stamp + 5_000_000_000 for stamp in later_than)])
        os.utime(path, ns=(current.st_atime_ns, target))
        return target

    def _snapshot(self, worktree, lock, paths=("input.txt",), previous_run=None):
        if previous_run is None:
            previous_run = self._successful_run()
        return slicer_pr_validation._snapshot_overlay_mtimes(
            worktree, list(paths), "build", previous_run, lock)

    def test_restored_timestamp_preserves_real_ninja_dirty_relation(self):
        if not shutil.which("ninja"):
            self.skipTest("Ninja is unavailable")
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, build, lock = self._fixture(Path(temp))
            output = build / "output.txt"
            self._ninja(build)
            self.assertEqual(output.read_text(encoding="utf-8"), "stable\nconfig-v1\n")

            snapshot, captured = self._snapshot(worktree, lock)
            self.assertEqual(captured["status"], "captured")
            self.assertEqual(captured["snapshot_file_count"], 1)
            original_mtime = (worktree / "input.txt").stat().st_mtime_ns
            fresh_mtime = self._mtime(worktree / "input.txt", future_of=(output,))
            self.assertGreater(fresh_mtime, output.stat().st_mtime_ns)
            before_restore = self._ninja(build, "-n")
            self.assertNotIn("no work to do", before_restore.stdout.lower())
            status_before_restore = self._git(worktree, "status", "--porcelain").stdout

            restored = slicer_pr_validation._restore_overlay_mtimes(worktree, snapshot, lock)
            self.assertEqual(restored["restored_file_count"], 1)
            self.assertEqual((worktree / "input.txt").stat().st_mtime_ns, original_mtime)
            self.assertEqual(self._git(worktree, "status", "--porcelain").stdout,
                             status_before_restore)
            after_restore = self._ninja(build, "-n")
            self.assertIn("no work to do", after_restore.stdout.lower())
            self._ninja(build)
            self.assertEqual(output.read_text(encoding="utf-8"), "stable\nconfig-v1\n")

            # A changed generated configuration header remains a legitimate Ninja input.
            config = build / "generated_config.hpp"
            config.write_text("config-v2\n", encoding="utf-8")
            self._mtime(config, future_of=(output,))
            changed_header = self._ninja(build, "-n")
            self.assertNotIn("no work to do", changed_header.stdout.lower())
            self._ninja(build)
            self.assertEqual(output.read_text(encoding="utf-8"), "stable\nconfig-v2\n")

    def test_git_reset_and_overlay_inode_replacement_preserves_ninja_dirty_relation(self):
        if not shutil.which("ninja"):
            self.skipTest("Ninja is unavailable")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            (repo, worktree, build, lock, base, harness, head,
             patch_path) = self._git_reset_overlay_fixture(root)
            output = build / "output.txt"
            self._ninja(build)
            self.assertEqual(output.read_text(encoding="utf-8"), "pr-overlay\nconfig-v1\n")
            snapshot, captured = self._snapshot(worktree, lock, paths=("input.txt",))
            self.assertEqual(captured["status"], "captured")
            self.assertEqual(captured["snapshot_file_count"], 1)
            source = worktree / "input.txt"
            original_mtime = source.stat().st_mtime_ns
            old_fd = os.open(source, os.O_RDONLY)
            try:
                old_identity = os.fstat(old_fd).st_dev, os.fstat(old_fd).st_ino
                reset = slicer_pr_validation._reset_worktree(
                    worktree, harness, preserve_build_dir="build", allowed_staged_paths=())
                self.assertTrue(all(row["returncode"] == 0 for row in reset), reset)
                patch_bytes = patch_path.read_bytes()
                applied = slicer_pr_validation._apply_overlay_patch(
                    worktree, patch_path, patch_sha256=hashlib.sha256(patch_bytes).hexdigest(),
                    base_sha=base, head_sha=harness, resolution_manifest=None)
                self.assertTrue(applied["applied"], applied)
                self.assertEqual(source.read_text(encoding="utf-8"), "pr-overlay\n")
                current = source.stat()
                self.assertNotEqual((current.st_dev, current.st_ino), old_identity)
                self.assertEqual(self._git(worktree, "status", "--porcelain").stdout.strip(),
                                 "M  input.txt")

                self._mtime(source, future_of=(output,))
                before_restore = self._ninja(build, "-n")
                self.assertNotIn("no work to do", before_restore.stdout.lower())
                status_before_restore = self._git(worktree, "status", "--porcelain").stdout
                restored = slicer_pr_validation._restore_overlay_mtimes(
                    worktree, snapshot, lock)
                self.assertEqual(restored["restored_file_count"], 1, restored)
                self.assertEqual(source.stat().st_mtime_ns, original_mtime)
                self.assertEqual(self._git(worktree, "status", "--porcelain").stdout,
                                 status_before_restore)
                after_restore = self._ninja(build, "-n")
                self.assertIn("no work to do", after_restore.stdout.lower())
                self._ninja(build)
                self.assertEqual(output.read_text(encoding="utf-8"),
                                 "pr-overlay\nconfig-v1\n")
            finally:
                os.close(old_fd)

    def test_changed_content_and_mode_are_never_restored(self):
        if not shutil.which("ninja"):
            self.skipTest("Ninja is unavailable")
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, build, lock = self._fixture(Path(temp))
            self._ninja(build)
            snapshot, captured = self._snapshot(worktree, lock)
            self.assertEqual(captured["status"], "captured")
            source = worktree / "input.txt"
            source.write_text("change\n", encoding="utf-8")
            changed_mtime = self._mtime(source, future_of=(build / "output.txt",))
            result = slicer_pr_validation._restore_overlay_mtimes(worktree, snapshot, lock)
            self.assertEqual(result["restored_file_count"], 0)
            self.assertEqual(result["skipped_reasons"], {"content_changed": 1})
            self.assertEqual(source.stat().st_mtime_ns, changed_mtime)
            self._ninja(build)
            self.assertTrue((build / "output.txt").read_text().startswith("change\n"))

            snapshot, captured = self._snapshot(worktree, lock)
            self.assertEqual(captured["status"], "captured")
            before = source.stat()
            source.chmod(0o600)
            changed_mtime = self._mtime(source, future_of=(build / "output.txt",))
            result = slicer_pr_validation._restore_overlay_mtimes(worktree, snapshot, lock)
            self.assertEqual(result["restored_file_count"], 0)
            self.assertEqual(result["skipped_reasons"], {"mode_changed": 1})
            self.assertEqual(source.stat().st_mtime_ns, changed_mtime)
            self.assertEqual(source.stat().st_mode & 0o777, 0o600)
            source.chmod(before.st_mode & 0o777)

    def test_parent_symlink_and_escaping_paths_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, worktree, _build, lock = self._fixture(root)
            snapshot, captured = self._snapshot(worktree, lock, paths=("nested/leaf.txt",))
            self.assertEqual(captured["snapshot_file_count"], 1)
            external = root / "external"
            external.mkdir()
            target = external / "leaf.txt"
            target.write_text("outside\n", encoding="utf-8")
            before = target.stat()
            (worktree / "nested").rename(worktree / "nested-old")
            (worktree / "nested").symlink_to(external, target_is_directory=True)
            result = slicer_pr_validation._restore_overlay_mtimes(worktree, snapshot, lock)
            self.assertEqual(result["restored_file_count"], 0)
            self.assertTrue(result["skipped_reasons"])
            after = target.stat()
            self.assertEqual((after.st_dev, after.st_ino, after.st_mtime_ns),
                             (before.st_dev, before.st_ino, before.st_mtime_ns))

            unsafe_snapshot, unsafe_stats = self._snapshot(
                worktree, lock, paths=("../external/leaf.txt",))
            self.assertEqual(unsafe_snapshot["entries"], {})
            self.assertEqual(unsafe_stats["skipped_reasons"], {"unsafe_path": 1})

    def test_external_hardlink_disables_mtime_restoration(self):
        if not shutil.which("ninja"):
            self.skipTest("Ninja is unavailable")
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            _, worktree, build, lock = self._fixture(root)
            self._ninja(build)
            snapshot, captured = self._snapshot(worktree, lock)
            self.assertEqual(captured["snapshot_file_count"], 1)
            source = worktree / "input.txt"
            original_mtime = source.stat().st_mtime_ns
            external_link = root / "outside-hardlink.txt"
            os.link(source, external_link)
            future_mtime = self._mtime(source, future_of=(build / "output.txt",))
            result = slicer_pr_validation._restore_overlay_mtimes(worktree, snapshot, lock)
            self.assertEqual(result["restored_file_count"], 0)
            self.assertEqual(result["skipped_reasons"], {"not_single_link_regular_file": 1})
            self.assertEqual(source.stat().st_mtime_ns, future_mtime)
            self.assertEqual(external_link.stat().st_mtime_ns, future_mtime)
            self.assertNotEqual(future_mtime, original_mtime)

    def test_untracked_deleted_and_build_paths_are_excluded(self):
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, _build, lock = self._fixture(Path(temp))
            (worktree / "new.txt").write_text("new\n", encoding="utf-8")
            (worktree / "nested/leaf.txt").unlink()
            snapshot, captured = self._snapshot(
                worktree, lock,
                paths=("input.txt", "new.txt", "nested/leaf.txt", "build/generated.hpp"))
            self.assertEqual(set(snapshot["entries"]), {"input.txt"})
            self.assertEqual(captured["skipped_reasons"].get("build_path"), 1)
            self.assertEqual(captured["skipped_reasons"].get("not_single_stage_zero_tracked_file"), 1)
            self.assertEqual(captured["skipped_reasons"].get("FileNotFoundError"), 1)

    def test_failed_build_keeps_old_stale_output_and_disables_restoration(self):
        if not shutil.which("ninja"):
            self.skipTest("Ninja is unavailable")
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, build, lock = self._fixture(Path(temp))
            output = build / "output.txt"
            self._ninja(build)
            stale_output_mtime = output.stat().st_mtime_ns

            source = worktree / "input.txt"
            source.write_text("fail\n", encoding="utf-8")
            self._mtime(source, future_of=(output,))
            failed = self._ninja(build, check=False)
            self.assertNotEqual(failed.returncode, 0)
            self.assertEqual(output.read_text(encoding="utf-8"), "stable\nconfig-v1\n")
            self.assertEqual(output.stat().st_mtime_ns, stale_output_mtime)
            self.assertLess(output.stat().st_mtime_ns, source.stat().st_mtime_ns)

            failed_run = {"commands": {
                "configure": {"returncode": 0, "timed_out": False},
                "build": {"returncode": failed.returncode, "timed_out": False},
            }}
            snapshot, captured = self._snapshot(worktree, lock, previous_run=failed_run)
            self.assertEqual(snapshot["entries"], {})
            self.assertEqual(captured["reason"], "previous_configure_or_build_not_successful")

            source.write_text("stable\n", encoding="utf-8")
            fresh_mtime = self._mtime(source, future_of=(output,))
            self.assertGreater(fresh_mtime, output.stat().st_mtime_ns)
            dirty = self._ninja(build, "-n")
            self.assertNotIn("no work to do", dirty.stdout.lower())
            self._ninja(build)
            self.assertGreater(output.stat().st_mtime_ns, stale_output_mtime)
            self.assertEqual(output.read_text(encoding="utf-8"), "stable\nconfig-v1\n")

    def test_current_reuse_lock_identity_is_required(self):
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, _build, lock = self._fixture(Path(temp))
            lock_ok, reason = slicer_pr_validation._validated_reuse_lock(worktree, lock)
            self.assertTrue(lock_ok, reason)
            snapshot, stats = self._snapshot(worktree, None)
            self.assertEqual(snapshot["entries"], {})
            self.assertEqual(stats["reason"], "reuse_lock_object_unavailable")
            valid_snapshot, valid_stats = self._snapshot(worktree, lock)
            self.assertEqual(valid_stats["status"], "captured")
            source = worktree / "input.txt"
            fresh_mtime = self._mtime(source)

            old_fd = os.open(lock.path, os.O_RDONLY)
            try:
                lock.path.unlink()
                lock.path.write_text(json.dumps({"pid": os.getpid(), "path": str(worktree)}),
                                     encoding="utf-8")
                replaced, reason = slicer_pr_validation._validated_reuse_lock(worktree, lock)
                self.assertFalse(replaced)
                self.assertEqual(reason, "reuse_lock_identity_changed")
                result = slicer_pr_validation._restore_overlay_mtimes(
                    worktree, valid_snapshot, lock)
                self.assertEqual(result["restored_file_count"], 0)
                self.assertEqual(result["reason"], "reuse_lock_identity_changed")
                self.assertEqual(source.stat().st_mtime_ns, fresh_mtime)
            finally:
                os.close(old_fd)

    def test_non_object_lock_json_fails_closed(self):
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, _build, lock = self._fixture(Path(temp))
            lock.path.write_text("[]\n", encoding="utf-8")
            lock_ok, reason = slicer_pr_validation._validated_reuse_lock(worktree, lock)
            self.assertFalse(lock_ok)
            self.assertEqual(reason, "reuse_lock_owner_mismatch")

    def test_fifo_paths_and_lock_are_rejected_without_blocking(self):
        with tempfile.TemporaryDirectory() as temp:
            _, worktree, _build, lock = self._fixture(Path(temp))
            os.mkfifo(worktree / "fifo")
            lock.path.unlink()
            os.mkfifo(lock.path)
            script_dir = Path(slicer_pr_validation.__file__).resolve().parent
            code = (
                "import os, sys\n"
                "from pathlib import Path\n"
                "sys.path.insert(0, sys.argv[1])\n"
                "import slicer_pr_validation as s\n"
                "worktree = Path(sys.argv[2])\n"
                "try:\n"
                "    s._open_worktree_file(worktree, 'fifo')\n"
                "except ValueError as exc:\n"
                "    assert str(exc) == 'not_single_link_regular_file', str(exc)\n"
                "else:\n"
                "    raise AssertionError('FIFO unexpectedly opened as a regular file')\n"
                "git_dir = Path(s._git(worktree, 'rev-parse', '--absolute-git-dir').stdout.strip())\n"
                "lock_path = git_dir / s._PR_REUSE_LOCK\n"
                "identity = lock_path.lstat()\n"
                "lock = s._PrValidationReuseLock(lock_path, (identity.st_dev, identity.st_ino))\n"
                "ok, reason = s._validated_reuse_lock(worktree, lock)\n"
                "assert not ok and reason == 'reuse_lock_identity_changed', (ok, reason)\n"
            )
            result = subprocess.run(
                [sys.executable, "-c", code, str(script_dir), str(worktree)],
                shell=False, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                text=True, timeout=5, check=False)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
