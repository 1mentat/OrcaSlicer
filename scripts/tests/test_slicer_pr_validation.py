"""Unit tests for PR-head validation status and cleanup semantics."""

from pathlib import Path
from argparse import Namespace
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_pr_validation  # noqa: E402


class ParsingTests(unittest.TestCase):
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
    def _args(minimum_tests=1):
        return Namespace(
            test_timeout=180.0,
            timeout=30.0,
            minimum_tests=minimum_tests,
            build_dir="build",
            jobs=1,
            test_jobs=1,
            exclude_labels="",
            deps_prefix="deps",
            uv="uv",
        )

    def _run_tree(self, *, minimum_tests, names, build_result=None):
        with tempfile.TemporaryDirectory() as temp:
            root_output = Path(temp) / "output"
            worktree = Path(temp) / "worktree"
            worktree.mkdir()

            def fake_default_test(build, jobs, junit, excluded, timeout):
                junit.write_text(_minimum_junit(names), encoding="utf-8")
                return ["ctest", "--output-junit", str(junit)]

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
                        "100% tests passed, 0 tests failed out of 3" if is_test else "",
                        "")
                log.write_text(result.stdout + result.stderr, encoding="utf-8")
                return result

            with mock.patch.object(slicer_pr_validation, "_default_test",
                                   side_effect=fake_default_test), \
                 mock.patch.object(slicer_pr_validation, "_run_phase",
                                   side_effect=fake_phase):
                return slicer_pr_validation._run_tree(
                    worktree, root_output, "baseline", self._args(minimum_tests),
                    ["cmake", "configure"], ["cmake", "build"],
                    commit_sha="a" * 40, tree_sha="b" * 40,
                )

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
            records = [{"number": 1, "headRefOid": head}]
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
                    snapshot_records=[{"number": 1, "headRefOid": head}],
                    selected_records=[{"number": 1, "headRefOid": head}])

            manifest = self._manifest(repo, base, harness_ref, base)
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": base}],
                    selected_records=[{"number": 1, "headRefOid": base}])

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
    def _snapshot(root, head):
        path = root / "snapshot.json"
        path.write_text(json.dumps([{
            "number": 1, "headRefName": "head", "headRefOid": head,
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
        snapshot = self._snapshot(root, head)
        manifest = root / "identity.json"
        manifest.write_text(json.dumps(
            self._manifest(repo, base, harness_ref, head)), encoding="utf-8")
        args = self._args(repo, snapshot, root / "evidence", manifest)
        args.harness_base = base
        args.harness_ref = harness_ref
        return repo, base, harness_ref, head, args

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

    def test_overlay_rejects_exception_reintroduced_by_applied_harness(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head, args = self._write_manifest_and_args(root)
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            records = [{"number": 1, "headRefOid": head}]
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
            records = [{"number": 1, "headRefOid": head}]
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
                    snapshot_records=[{"number": 1, "headRefOid": head}],
                    selected_records=[{"number": 1, "headRefOid": head}])
            manifest = json.loads(Path(args.identity_manifest).read_text(encoding="utf-8"))
            manifest["required_harness_cases"][0]["source"]["line"] += 1
            with self.assertRaises(slicer_pr_validation.ValidationError):
                slicer_pr_validation._validate_identity_manifest(
                    repo, manifest, harness_base=base, harness_ref=harness_ref,
                    snapshot_records=[{"number": 1, "headRefOid": head}],
                    selected_records=[{"number": 1, "headRefOid": head}])


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


class ProcessCaptureTests(unittest.TestCase):
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

    def _write_snapshot(self, root, head):
        snapshot = root / "snapshot.json"
        snapshot.write_text(json.dumps([{
            "number": 1, "headRefName": "head", "headRefOid": head,
        }]), encoding="utf-8")
        return snapshot

    def test_nonempty_output_directory_is_rejected_before_worktree_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head)
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

    def test_report_records_snapshot_digest_and_preflight_heads(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head)
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
            snapshot = self._write_snapshot(root, head)
            output = root / "evidence"
            args = self._args(repo, snapshot, output)
            args.harness_base = base
            args.harness_ref = harness_ref
            original_bytes = snapshot.read_bytes()
            changed_bytes = json.dumps([{
                "number": 1,
                "headRefName": "head",
                "headRefOid": head,
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
                        if argv[0] == "ctest":
                            junit = Path(argv[argv.index("--output-junit") + 1])
                            junit.write_text('<testsuite tests="1"><testcase name="contract" /></testsuite>')
                            return slicer_pr_validation.CommandResult(
                                list(argv), 0, 0.01, "100% tests passed, 0 tests failed out of 1", "")
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
            snapshot = self._write_snapshot(root, head)
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
        if name == "reference":
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

    def test_reference_failure_does_not_suppress_head_and_src_stays_unchanged(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, base, harness_ref, head = self._fixture(root)
            snapshot = self._write_snapshot(root, head)
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
            snapshot = self._write_snapshot(root, head)
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
            snapshot = self._write_snapshot(root, head)
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
            snapshot = self._write_snapshot(root, head)
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
            snapshot = self._write_snapshot(root, head)
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


if __name__ == "__main__":
    unittest.main()
