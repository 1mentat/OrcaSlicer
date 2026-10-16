#!/usr/bin/env python3
"""Unit tests for bounded mutation selection and result classification."""

from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_mutation  # noqa: E402


class MutationSelectionTests(unittest.TestCase):
    def test_default_mutants_are_bounded_and_exactly_named(self):
        self.assertGreater(len(slicer_mutation.DEFAULT_MUTANTS), 0)
        self.assertLessEqual(len(slicer_mutation.DEFAULT_MUTANTS, ), slicer_mutation.DEFAULT_MAX_MUTANTS)
        self.assertEqual(len({item.id for item in slicer_mutation.DEFAULT_MUTANTS}),
                         len(slicer_mutation.DEFAULT_MUTANTS))

    def test_empty_and_ambiguous_matches_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.cpp"
            source.write_text("same same", encoding="utf-8")
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._apply(
                    slicer_mutation.Mutant("empty", "source.cpp", "", "x", ""), root)
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._apply(
                    slicer_mutation.Mutant("ambiguous", "source.cpp", "same", "x", ""), root)

    def test_mutation_restores_original_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.cpp"
            source.write_bytes(b"before\n")
            mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "")
            path, original = slicer_mutation._apply(mutant, root)
            self.assertEqual(path.read_bytes(), b"after\n")
            path.write_bytes(original)
            self.assertEqual(path.read_bytes(), b"before\n")

    def test_manifest_path_is_relative_to_declared_root(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = root / "tests" / "mutations.json"
            manifest.parent.mkdir()
            manifest.write_text(json.dumps({"mutants": [{
                "id": "one", "path": "source.cpp", "old": "before", "new": "after",
            }]}), encoding="utf-8")
            source = root / "source.cpp"
            source.write_text("before", encoding="utf-8")
            mutants = slicer_mutation._load_mutants("tests/mutations.json", root)
            self.assertEqual([item.id for item in mutants], ["one"])

    def test_repeated_manifests_keep_each_filter_in_selection_order(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ("first", "second"):
                (root / f"{name}.json").write_text(json.dumps({"mutants": [{
                    "id": name, "path": "source.cpp", "old": "before", "new": "after",
                    "test_filter": name.title(),
                }]}), encoding="utf-8")
            args = slicer_mutation._parser().parse_args([
                "run", "--build-command", "build", "--test-command", "ctest",
                "--mutants", "first.json", "--mutants", "second.json",
            ])
            mutants = slicer_mutation._load_mutants(args.mutants, root)
            self.assertEqual([(item.id, item.test_filter) for item in mutants],
                             [("first", "First"), ("second", "Second")])


class MutationResultTests(unittest.TestCase):
    def result(self, returncode, *, output="1 tests passed", timed_out=False):
        return slicer_mutation.CommandResult(["runner"], returncode, 0.01, output, "", timed_out)

    def test_distinguishes_killed_survived_timeout_and_build_error(self):
        failure = self.result(1, output="test cases: 1 | 1 failed\nassertions: 3 | 2 passed | 1 failed")
        self.assertEqual(slicer_mutation._classify_test(failure, 1), "killed")
        self.assertEqual(slicer_mutation._classify_test(self.result(0), 1), "survived")
        self.assertEqual(slicer_mutation._classify_test(self.result(None, timed_out=True), 1), "test_timeout")
        self.assertEqual(slicer_mutation._classify_build(self.result(1)), "build_error")
        self.assertEqual(slicer_mutation._classify_build(self.result(None, timed_out=True)), "build_timeout")

    def test_ctest_child_timeouts_are_not_mutation_kills(self):
        outputs = [
            "1/2 Test #1: traversal ....***Timeout 0.01 sec\n50% tests passed, 1 tests failed out of 2",
            "The following tests FAILED:\n  1 - traversal (Timeout)\n",
        ]
        for output in outputs:
            with self.subTest(output=output):
                self.assertEqual(slicer_mutation._classify_test(self.result(8, output=output), 2),
                                 "test_timeout")

    def test_timeout_in_a_test_name_does_not_hide_an_assertion_failure(self):
        output = ("1/1 Test #1: Timeout settings contract ....***Failed\n"
                  "assertions: 1 | 1 failed\n0% tests passed, 1 tests failed out of 1")
        self.assertEqual(slicer_mutation._classify_test(self.result(8, output=output), 1), "killed")

    def test_ctest_crashes_are_not_kills_even_with_another_assertion_failure(self):
        for crash in ("***Exception: SegFault", "***Exception: Other",
                      "2 - crash (Subprocess aborted)", "2 - crash (SEGFAULT)",
                      "fatal error condition: SIGSEGV", "unexpected exception with message:",
                      "Subprocess killed",
                      "FAILED due to a fatal error condition:",
                      "FAILED due to unexpected exception with message:"):
            with self.subTest(crash=crash):
                output = f"{crash}\nassertions: 2 | 1 passed | 1 failed\n2 tests failed out of 2"
                self.assertEqual(slicer_mutation._classify_test(self.result(8, output=output), 2),
                                 "test_infrastructure_failure")

    def test_nonzero_exit_requires_positive_assertion_failure_evidence(self):
        for output in ("1 tests passed", "1 tests failed out of 1",
                       "assertions: 3 | 3 passed | 0 failed", "assertions: -1 | -1 failed",
                       "assertions: 1 | -1 failed", "assertions: 1 | 1 failed as expected"):
            with self.subTest(output=output):
                self.assertEqual(slicer_mutation._classify_test(self.result(8, output=output), 1),
                                 "test_failure_unverified")

    def test_mutant_must_execute_the_baseline_test_count(self):
        output = "test cases: 1 | 1 failed\nassertions: 1 | 1 failed"
        self.assertEqual(slicer_mutation._classify_test(self.result(1, output=output), 1, 2),
                         "test_count_mismatch")
        self.assertEqual(slicer_mutation._classify_test(self.result(0), 1, 2),
                         "test_count_mismatch")

    def test_catch_success_counts_cases_instead_of_assertions(self):
        self.assertEqual(slicer_mutation.test_evidence(
            "All tests passed (43 assertions in 2 test cases)"), 2)
        self.assertEqual(slicer_mutation.test_evidence(
            "All tests passed (1 assertion in 1 test case)"), 1)

    def test_grouped_ctest_counts_are_stable_when_catch_failure_output_is_added(self):
        baseline = "100% tests passed, 0 tests failed out of 1"
        failed = ("test cases: 3 | 2 passed | 1 failed\nassertions: 7 | 6 passed | 1 failed\n"
                  "0% tests passed, 1 tests failed out of 1")
        expected = slicer_mutation.test_evidence(baseline)
        observed = slicer_mutation.test_evidence(failed)
        self.assertEqual((expected, observed), (1, 1))
        self.assertEqual(slicer_mutation._classify_test(self.result(8, output=failed), observed, expected),
                         "killed")

    def test_zero_exit_cannot_hide_reported_assertion_failures(self):
        for output in ("assertions: 1 | 1 failed", "0% tests passed, 1 tests failed out of 1"):
            with self.subTest(output=output):
                self.assertEqual(slicer_mutation._classify_test(self.result(0, output=output), 1),
                                 "test_infrastructure_failure")

    def test_ctest_requests_failure_output_once_and_preserves_filter(self):
        mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "", "Flow")
        for executable in ("/usr/bin/ctest", "C:/tools/ctest.exe", r"C:\tools\ctest.exe"):
            with self.subTest(executable=executable):
                command = slicer_mutation._test_command([executable, "--output-on-failure"], mutant)
                self.assertEqual(command.count("--output-on-failure"), 1)
                self.assertEqual(command[-2:], ["-L", "Flow"])
                command = slicer_mutation._test_command([executable], mutant)
                self.assertIn("--output-on-failure", command)

    def test_ctest_preserves_existing_label_options(self):
        mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "", "Flow")
        for options in (["-L", "Other"], ["-LOther"], ["--label-regex", "Other"],
                        ["--label-regex=Other"]):
            with self.subTest(options=options):
                command = slicer_mutation._test_command(["ctest", *options], mutant)
                self.assertEqual(command, ["ctest", *options, "--output-on-failure"])

    def test_zero_evidence_is_not_a_survivor(self):
        self.assertEqual(slicer_mutation._classify_test(self.result(0, output=""), 0), "test_no_evidence")

    def test_all_skipped_or_not_run_is_not_evidence(self):
        self.assertEqual(
            slicer_mutation.test_evidence(
                "0% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(
            slicer_mutation.test_evidence(
                "100% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(slicer_mutation.test_evidence("test cases: 0 | 2 skipped"), 0)


if __name__ == "__main__":
    unittest.main()
