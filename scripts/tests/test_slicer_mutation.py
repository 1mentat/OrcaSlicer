#!/usr/bin/env python3
"""Unit tests for bounded mutation selection and result classification."""

from pathlib import Path
import hashlib
import json
import os
import shlex
import sys
import subprocess
import tempfile
import unittest
from unittest import mock

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

    def test_ctest_exclusions_do_not_suppress_mutant_selection(self):
        mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "", "Flow")
        for options in (["-LE", "NotWorking|RequiresApp"], ["-LENotWorking"],
                        ["--label-exclude", "NotWorking"], ["--label-exclude=NotWorking"]):
            with self.subTest(options=options):
                command = slicer_mutation._test_command(["ctest", *options], mutant)
                self.assertEqual(command, ["ctest", *options, "--output-on-failure", "-L", "Flow"])

    def test_ctest_explicit_filter_and_exclusions_are_preserved(self):
        mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "", "Flow")
        command = slicer_mutation._test_command(
            ["ctest", "--label-regex", "{filter}", "-LE", "NotWorking"], mutant)
        self.assertEqual(command, ["ctest", "--label-regex", "Flow", "-LE", "NotWorking",
                                   "--output-on-failure"])

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


class MutationEvidenceHardeningTests(unittest.TestCase):
    def test_file_backed_capture_scans_full_log_without_stdout_buffer(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "command.log"
            payload = (b"progress\n" * 90000) + (
                b"test cases: 1 | 1 passed\n"
                b"assertions: 2 | 2 passed\n"
            )

            def fake_capture(argv, *, cwd, timeout, output_file):
                self.assertIsNotNone(output_file)
                output_file.write(payload)
                return subprocess.CompletedProcess(list(argv), 0, None, None)

            with mock.patch.object(slicer_mutation, "run_captured", fake_capture):
                result = slicer_mutation.run_command(["fake"], cwd=Path(temp),
                                                     timeout=1, output_file=log)

            self.assertEqual(result.stdout, "")
            self.assertEqual(result.returncode, 0)
            self.assertEqual(result.log_path, str(log.resolve()))
            self.assertEqual(result.log_size_bytes, len(payload))
            self.assertEqual(result.log_sha256, hashlib.sha256(payload).hexdigest())
            self.assertGreater(result.log_size_bytes, slicer_mutation.MAX_OUTPUT)
            self.assertEqual(result.json()["log"], {
                "path": str(log.resolve()),
                "sha256": hashlib.sha256(payload).hexdigest(),
                "size_bytes": len(payload),
            })
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertTrue(evidence.complete)
            self.assertEqual(evidence.test_count, 1)
            self.assertEqual(slicer_mutation._classify_test(result, evidence.test_count,
                                                             log_evidence=evidence),
                             "survived")

    def test_complete_log_at_end_of_large_output_can_kill(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "mutant.log"
            log.write_bytes((b"noise\n" * 90000) +
                            b"test cases: 1 | 1 failed\n"
                            b"assertions: 2 | 1 passed | 1 failed\n")
            result = slicer_mutation.CommandResult(["test"], 1, 0.1, "", "",
                                                   log_path=str(log),
                                                   log_sha256="unused", log_size_bytes=log.stat().st_size)
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertTrue(evidence.complete)
            self.assertTrue(evidence.assertion_failure)
            self.assertEqual(slicer_mutation._classify_test(result, evidence.test_count,
                                                             1, log_evidence=evidence),
                             "killed")

    def test_oversized_line_fails_closed_instead_of_killing(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "truncated.log"
            log.write_bytes(b"x" * (slicer_mutation._MAX_EVIDENCE_LINE + 1) +
                            b"\nassertions: 1 | 1 failed\n")
            result = slicer_mutation.CommandResult(["test"], 1, 0.1, "", "",
                                                   log_path=str(log))
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertFalse(evidence.complete)
            self.assertEqual(slicer_mutation._classify_test(result, 1, 1,
                                                             log_evidence=evidence),
                             "test_evidence_incomplete")

    def test_assertion_test_name_aborted_is_not_a_crash_marker(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "assertion.log"
            log.write_text("test case: aborted transactions\n"
                           "test cases: 1 | 1 failed\n"
                           "assertions: 2 | 1 passed | 1 failed\n")
            result = slicer_mutation.CommandResult(["test"], 1, 0.1, "", "",
                                                   log_path=str(log))
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertFalse(evidence.infrastructure_failure)
            self.assertEqual(slicer_mutation._classify_test(result, evidence.test_count,
                                                             1, log_evidence=evidence),
                             "killed")

    def test_real_crash_marker_before_large_tail_is_infrastructure_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "crash.log"
            log.write_bytes(b"***Exception: SegFault\n" + (b"noise\n" * 90000) +
                            b"test cases: 1 | 1 failed\n"
                            b"assertions: 2 | 1 passed | 1 failed\n")
            result = slicer_mutation.CommandResult(["test"], 8, 0.1, "", "",
                                                   log_path=str(log))
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertTrue(evidence.infrastructure_failure)
            self.assertEqual(slicer_mutation._classify_test(result, evidence.test_count,
                                                             1, log_evidence=evidence),
                             "test_infrastructure_failure")

    def test_nonfinite_or_nonpositive_limits_fail_before_execution(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for option, value, message in (("--timeout", "nan", "finite and positive"),
                                            ("--timeout", "0", "finite and positive"),
                                            ("--max-mutants", "0", "must be positive")):
                with self.subTest(option=option, value=value):
                    args = slicer_mutation._parser().parse_args([
                        "run", "--root", str(root), "--out", str(root / "out"),
                        "--build-command", "build", "--test-command", "test",
                        option, value,
                    ])
                    with self.assertRaisesRegex(slicer_mutation.MutationError, message):
                        slicer_mutation.run_mutation(args)

    def test_checkpoint_is_atomic_and_marks_in_progress(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "mutation-report.json"
            report = {"schema_version": 2, "status": "infrastructure_error"}
            slicer_mutation._checkpoint(path, report, "preflight")
            self.assertEqual(json.loads(path.read_text())["checkpoint"],
                             {"phase": "preflight", "state": "in_progress"})
            self.assertFalse(list(Path(temp).glob("*.tmp")))

    def test_nonempty_output_is_rejected_before_worktree_creation(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "evidence"
            out.mkdir()
            marker = out / "mutation-report.json"
            marker.write_text("old evidence\n")
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._output_available(out)
            self.assertEqual(marker.read_text(), "old evidence\n")

    def test_symlink_output_is_rejected_without_touching_target(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            target = root / "target"
            target.mkdir()
            link = root / "evidence-link"
            try:
                link.symlink_to(target, target_is_directory=True)
            except (NotImplementedError, OSError):
                self.skipTest("symlinks unavailable")
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._output_available(link)
            self.assertEqual(list(target.iterdir()), [])

    def test_run_and_main_reject_symlink_output_before_report_creation(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "repo"
            root.mkdir()
            target = Path(temp) / "target"
            target.mkdir()
            link = Path(temp) / "evidence-link"
            try:
                link.symlink_to(target, target_is_directory=True)
            except (NotImplementedError, OSError):
                self.skipTest("symlinks unavailable")
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(root), "--out", str(link),
                "--build-command", "build", "--test-command", "test",
            ])
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation.run_mutation(args)
            self.assertEqual(list(target.iterdir()), [])
            self.assertEqual(slicer_mutation.main([
                "run", "--root", str(root), "--out", str(link),
                "--build-command", "build", "--test-command", "test",
            ]), 2)
            self.assertEqual(list(target.iterdir()), [])

    def test_runner_writes_preflight_checkpoint_before_fake_worktree(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
                "--max-mutants", "2",
            ])
            mutants = [
                slicer_mutation.Mutant("m1", "missing.cpp", "old", "new", ""),
                slicer_mutation.Mutant("m2", "missing.cpp", "old", "new", ""),
            ]
            seen_checkpoint = []

            def fake_git(_repo, *git_args):
                if git_args[:2] == ("worktree", "add"):
                    report = json.loads((out / "mutation-report.json").read_text())
                    seen_checkpoint.append(report["checkpoint"])
                return slicer_mutation.CommandResult(["git", *git_args], 0, 0.01, "deadbeef\n", "")

            def fake_command(argv, *, cwd, timeout, output_file=None):
                log_path = None
                log_sha256 = None
                log_size_bytes = None
                if output_file is not None:
                    payload = b"test cases: 1 | 1 passed\nassertions: 1 | 1 passed\n"
                    output_file.parent.mkdir(parents=True, exist_ok=True)
                    output_file.write_bytes(payload)
                    log_path = str(output_file.resolve())
                    log_sha256 = hashlib.sha256(payload).hexdigest()
                    log_size_bytes = len(payload)
                return slicer_mutation.CommandResult(list(argv), 0, 0.01,
                                                     "", "", log_path=log_path,
                                                     log_sha256=log_sha256,
                                                     log_size_bytes=log_size_bytes)

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="deadbeef"), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=mutants), \
                 mock.patch.object(slicer_mutation, "_git", side_effect=fake_git), \
                 mock.patch.object(slicer_mutation, "run_command", side_effect=fake_command):
                self.assertEqual(slicer_mutation.run_mutation(args), 2)
            self.assertEqual(seen_checkpoint,
                             [{"phase": "preflight", "state": "in_progress"}])
            final_report = json.loads((out / "mutation-report.json").read_text())
            self.assertEqual(final_report["checkpoint"]["state"], "complete")
            self.assertEqual([entry["status"] for entry in final_report["mutants"]],
                             ["mutation_error", "mutation_error"])
            self.assertEqual(len(final_report["mutants"]), 2)
            baseline_evidence = final_report["baseline_tests"][0]["log_evidence"]
            self.assertTrue(baseline_evidence["complete"])
            self.assertEqual(baseline_evidence["test_count"], 1)
            self.assertEqual(final_report["baseline_tests"][0]["result"]["log"]["size_bytes"],
                             len(b"test cases: 1 | 1 passed\nassertions: 1 | 1 passed\n"))

    def test_cleanup_failure_cannot_report_pass(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
                "--max-mutants", "1",
            ])
            source = repo / "source.cpp"
            source.write_text("old")
            mutant = slicer_mutation.Mutant("m", "source.cpp", "old", "new", "")
            git_calls = []

            def fake_git(_repo, *git_args):
                git_calls.append(git_args)
                if git_args[:2] == ("worktree", "remove"):
                    return slicer_mutation.CommandResult(["git", *git_args], 1, 0.01, "", "remove failed")
                return slicer_mutation.CommandResult(["git", *git_args], 0, 0.01, "deadbeef\n", "")

            def fake_command(argv, *, cwd, timeout, output_file=None):
                return slicer_mutation.CommandResult(list(argv), 0, 0.01,
                                                     "test cases: 1 | 1 passed\n", "")

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="deadbeef"), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                 mock.patch.object(slicer_mutation, "_git", side_effect=fake_git), \
                 mock.patch.object(slicer_mutation, "run_command", side_effect=fake_command):
                self.assertEqual(slicer_mutation.run_mutation(args), 2)
            report = json.loads((out / "mutation-report.json").read_text())
            self.assertEqual(report["status"], "cleanup_failed")
            self.assertTrue(report["cleanup_failed"])

    def test_interrupted_run_leaves_in_progress_checkpoint(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            source = repo / "source.cpp"
            source.write_text("old")
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
                "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("m", "source.cpp", "old", "new", "")
            command_count = 0

            def fake_git(_repo, *git_args):
                return slicer_mutation.CommandResult(["git", *git_args], 0, 0.01,
                                                     "deadbeef\n", "")

            def fake_command(argv, *, cwd, timeout, output_file=None):
                nonlocal command_count
                command_count += 1
                if command_count == 4:
                    raise KeyboardInterrupt
                return slicer_mutation.CommandResult(list(argv), 0, 0.01,
                                                     "test cases: 1 | 1 passed\n", "")

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="deadbeef"), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                 mock.patch.object(slicer_mutation, "_git", side_effect=fake_git), \
                 mock.patch.object(slicer_mutation, "_apply", return_value=(source, b"old")), \
                 mock.patch.object(slicer_mutation, "run_command", side_effect=fake_command):
                with self.assertRaises(KeyboardInterrupt):
                    slicer_mutation.run_mutation(args)
            report = json.loads((out / "mutation-report.json").read_text())
            self.assertNotEqual(report["status"], "pass")
            self.assertEqual(report["checkpoint"]["state"], "in_progress")

    def test_command_log_names_are_unique_and_sanitized(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            first = slicer_mutation._command_log(root, 1, "mutant/test", "id with spaces")
            second = slicer_mutation._command_log(root, 2, "mutant/test", "id with spaces")
            self.assertNotEqual(first, second)
            self.assertEqual(first.name, "0001-mutant-test-id-with-spaces.log")

    def test_main_error_does_not_clobber_existing_evidence(self):
        with tempfile.TemporaryDirectory() as temp:
            out = Path(temp) / "evidence"
            out.mkdir()
            marker = out / "mutation-report.json"
            marker.write_text("old evidence\n")
            code = slicer_mutation.main([
                "run", "--root", temp, "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
            ])
            self.assertEqual(code, 2)
            self.assertEqual(marker.read_text(), "old evidence\n")


class MutationCheckpointIntegrationTests(unittest.TestCase):
    def test_disposable_worktree_checkpoint_failure_still_attempts_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "build", "--test-command", "test",
                "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "old", "new", "")

            def git(_repo, *args):
                return slicer_mutation.CommandResult(["git", *args], 0, 0.01, "", "")

            def checkpoint(path, report, phase):
                if phase == "worktree_add":
                    raise OSError("report disk failure")
                slicer_mutation._write_report(path, report)

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                 mock.patch.object(slicer_mutation, "_git", side_effect=git) as run_git, \
                 mock.patch.object(slicer_mutation, "_checkpoint", side_effect=checkpoint):
                with self.assertRaises(OSError):
                    slicer_mutation.run_mutation(args)
            self.assertEqual([call.args[2] for call in run_git.call_args_list], ["add", "remove"])

    def test_failed_worktree_add_still_attempts_cleanup(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "build", "--test-command", "test",
                "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "old", "new", "")

            def git(_repo, *args):
                return slicer_mutation.CommandResult(
                    ["git", *args], 1 if args[1] == "add" else 0, 0.01, "", "add failed")

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                 mock.patch.object(slicer_mutation, "_git", side_effect=git) as run_git:
                with self.assertRaisesRegex(slicer_mutation.MutationError, "cannot create"):
                    slicer_mutation.run_mutation(args)
            self.assertEqual([call.args[2] for call in run_git.call_args_list], ["add", "remove"])
            report = json.loads((out / "mutation-report.json").read_text())
            self.assertNotEqual(report["status"], "pass")
            self.assertEqual(report["worktree"]["remove"]["returncode"], 0)

    def run_two_mutants(self, *, first_build_error=False):
        with tempfile.TemporaryDirectory() as temp:
            repo = Path(temp) / "repo"
            repo.mkdir()
            source = repo / "source.cpp"
            source.write_text("old", encoding="utf-8")
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "build", "--test-command", "test",
                "--max-mutants", "2",
            ])
            mutants = [slicer_mutation.Mutant(name, "source.cpp", "old", "new", "")
                       for name in ("first", "second")]
            checkpoints = []
            builds = 0
            tests = 0

            def command(argv, *, cwd, timeout, output_file=None):
                nonlocal builds, tests
                code = 0
                output = ""
                if argv[0] == "build":
                    builds += 1
                    if builds == 3:
                        checkpoints.append(json.loads((out / "mutation-report.json").read_text()))
                    if first_build_error and builds == 2:
                        code = 1
                else:
                    tests += 1
                    if tests == 1 or first_build_error:
                        output = "100% tests passed, 0 tests failed out of 1\n"
                    else:
                        code = 8
                        output = ("assertions: 1 | 1 failed\n"
                                  "0% tests passed, 1 tests failed out of 1\n")
                return slicer_mutation.CommandResult(list(argv), code, 0.01, output, "")

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                 mock.patch.object(slicer_mutation, "_load_mutants", return_value=mutants), \
                 mock.patch.object(slicer_mutation, "_git", return_value=
                                   slicer_mutation.CommandResult(["git"], 0, 0.01, "", "")), \
                 mock.patch.object(slicer_mutation, "_apply", return_value=(source, b"old")), \
                 mock.patch.object(slicer_mutation, "run_command", side_effect=command):
                code = slicer_mutation.run_mutation(args)
            return code, json.loads((out / "mutation-report.json").read_text()), checkpoints

    def test_completed_mutant_is_durable_before_next_build(self):
        code, report, checkpoints = self.run_two_mutants()
        self.assertEqual(code, 0)
        self.assertEqual(report["status"], "pass")
        self.assertEqual(len(checkpoints), 1)
        checkpoint = checkpoints[0]
        self.assertEqual(checkpoint["checkpoint"]["state"], "in_progress")
        self.assertNotEqual(checkpoint["status"], "pass")
        self.assertEqual([(entry["id"], entry["status"]) for entry in checkpoint["mutants"]],
                         [("first", "killed")])

    def test_survivor_does_not_mask_another_mutants_build_error(self):
        code, report, _ = self.run_two_mutants(first_build_error=True)
        self.assertEqual(code, 2)
        self.assertEqual(report["status"], "infrastructure_error")
        self.assertEqual(report["counts"], {"build_error": 1, "survived": 1})

    def test_crash_word_inside_test_name_is_not_a_crash_marker(self):
        output = ("1/1 Test #1: (SEGFAULT) recovery contract ....***Failed\n"
                  "assertions: 1 | 1 failed\n"
                  "0% tests passed, 1 tests failed out of 1\n")
        result = slicer_mutation.CommandResult(["ctest"], 8, 0.01, output, "")
        self.assertEqual(slicer_mutation._classify_test(result, 1, 1), "killed")
        with tempfile.TemporaryDirectory() as temp:
            log = Path(temp) / "test.log"
            log.write_text(output, encoding="utf-8")
            evidence = slicer_mutation.scan_log_evidence(log)
            self.assertFalse(evidence.infrastructure_failure)
            self.assertEqual(slicer_mutation._classify_test(
                result, evidence.test_count, 1, log_evidence=evidence), "killed")


class MutationTempDirectoryPreflightTests(unittest.TestCase):
    def _args(self, root: Path, out: Path):
        return slicer_mutation._parser().parse_args([
            "run", "--root", str(root), "--out", str(out),
            "--build-command", "configure-build", "--test-command", "ctest",
        ])

    def test_invalid_explicit_temp_paths_fail_before_any_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "repo"
            root.mkdir()
            out = Path(temp) / "out"
            for variable, value in (
                    ("TMPDIR", str(Path(temp) / "missing")),
                    ("TEMP", str(Path(temp) / "missing-file")),
                    ("TMP", str(Path(temp) / "missing-tmp"))):
                with self.subTest(variable=variable):
                    if variable == "TEMP":
                        value_path = Path(value)
                        value_path.write_text("not a directory", encoding="utf-8")
                    with mock.patch.dict(os.environ, {variable: value}, clear=True), \
                         mock.patch.object(slicer_mutation, "run_command") as command:
                        with self.assertRaisesRegex(slicer_mutation.MutationError, variable):
                            slicer_mutation.run_mutation(self._args(root, out))
                        command.assert_not_called()

    def test_empty_explicit_temp_paths_fail_before_any_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "repo"
            root.mkdir()
            out = Path(temp) / "out"
            for variable in slicer_mutation._TEMP_ENVIRONMENT_VARIABLES:
                with self.subTest(variable=variable):
                    with mock.patch.dict(os.environ, {variable: ""}, clear=True), \
                         mock.patch.object(slicer_mutation, "run_command") as command:
                        with self.assertRaisesRegex(slicer_mutation.MutationError,
                                                    variable + ".*empty"):
                            slicer_mutation.run_mutation(self._args(root, out))
                        command.assert_not_called()

    def test_windows_temp_environment_policy_keeps_python_variables(self):
        with mock.patch.object(slicer_mutation.os, "name", "nt"):
            self.assertEqual(slicer_mutation._temp_environment_variables(),
                             ("TMPDIR", "TMP", "TEMP"))

    def test_absent_temp_environment_preserves_fallback(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            self.assertEqual(slicer_mutation._preflight_temp_environment(), [])

    def test_relative_explicit_temp_path_fails_before_any_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "repo"
            root.mkdir()
            out = Path(temp) / "out"
            with mock.patch.dict(os.environ, {"TMPDIR": "relative-temp"}, clear=True), \
                 mock.patch.object(slicer_mutation, "run_command") as command:
                with self.assertRaisesRegex(slicer_mutation.MutationError,
                                            "TMPDIR.*absolute path"):
                    slicer_mutation.run_mutation(self._args(root, out))
                command.assert_not_called()

    def test_failed_write_probe_fails_before_any_command(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp) / "repo"
            root.mkdir()
            out = Path(temp) / "out"
            with mock.patch.dict(os.environ, {"TMPDIR": temp}, clear=True), \
                 mock.patch.object(slicer_mutation, "_write_temp_probe",
                                   side_effect=PermissionError("probe denied")), \
                 mock.patch.object(slicer_mutation, "run_command") as command:
                with self.assertRaisesRegex(slicer_mutation.MutationError,
                                            "TMPDIR.*write probe"):
                    slicer_mutation.run_mutation(self._args(root, out))
                command.assert_not_called()

    def test_valid_explicit_temp_path_is_checked_and_probe_is_removed(self):
        with tempfile.TemporaryDirectory() as temp:
            with mock.patch.dict(os.environ, {"TMPDIR": temp}, clear=True):
                checked = slicer_mutation._preflight_temp_environment()
            self.assertEqual(checked, [{"variable": "TMPDIR", "path": temp}])
            self.assertEqual(list(Path(temp).iterdir()), [])


class PersistentMutationWorktreeTests(unittest.TestCase):
    def _git(self, cwd: Path, *args: str) -> str:
        result = subprocess.run(["git", *args], cwd=cwd, check=True,
                                stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                text=True)
        return result.stdout.strip()

    def _repo(self, root: Path) -> tuple[Path, str, str]:
        repo = root / "repo"
        repo.mkdir()
        self._git(repo, "init", "-q")
        self._git(repo, "config", "user.email", "mutation-test@example.invalid")
        self._git(repo, "config", "user.name", "Mutation Test")
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "source.cpp").write_text("before\n", encoding="utf-8")
        self._git(repo, "add", ".")
        self._git(repo, "commit", "-qm", "first")
        first = self._git(repo, "rev-parse", "HEAD")
        (repo / "source.cpp").write_text("second\n", encoding="utf-8")
        self._git(repo, "add", "source.cpp")
        self._git(repo, "commit", "-qm", "second")
        second = self._git(repo, "rev-parse", "HEAD")
        return repo, first, second

    def test_persistent_worktree_reuses_ignored_build_and_moves_clean_commit(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, first, second = self._repo(Path(temp))
            persistent = Path(temp) / "retained"
            lease = slicer_mutation._persistent_worktree(repo, persistent, first)
            self.assertFalse(lease.reused)
            self.assertIsNone(lease.previous_head)
            (persistent / "build").mkdir()
            (persistent / "build" / "cache.bin").write_bytes(b"retained-build")
            lease.release()

            lease = slicer_mutation._persistent_worktree(repo, persistent, second)
            try:
                self.assertTrue(lease.reused)
                self.assertEqual(lease.previous_head, first)
                self.assertEqual(self._git(persistent, "rev-parse", "HEAD"), second)
                self.assertEqual((persistent / "source.cpp").read_text(encoding="utf-8"), "second\n")
                self.assertEqual((persistent / "build" / "cache.bin").read_bytes(), b"retained-build")
                detached = subprocess.run(
                    ["git", "symbolic-ref", "--quiet", "--short", "HEAD"],
                    cwd=persistent, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True)
                self.assertEqual(detached.returncode, 1)
            finally:
                lease.release()
            self.assertFalse((persistent.with_name(persistent.name + ".slicer-mutation.lock")).exists())

    def test_persistent_rejects_lock_dirty_branch_wrong_owner_and_symlink(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, first, _ = self._repo(root)
            persistent = root / "retained"
            lease = slicer_mutation._persistent_worktree(repo, persistent, first)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "already in use"):
                slicer_mutation._persistent_worktree(repo, persistent, first)
            lease.release()

            (persistent / "source.cpp").write_text("dirty\n", encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "dirty tracked"):
                slicer_mutation._persistent_worktree(repo, persistent, first)
            (persistent / "source.cpp").write_text("before\n", encoding="utf-8")

            self._git(persistent, "switch", "-c", "attached")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "detached"):
                slicer_mutation._persistent_worktree(repo, persistent, first)
            self._git(persistent, "switch", "--detach", first)

            marker = persistent / slicer_mutation._PERSISTENT_MARKER
            identity = json.loads(marker.read_text(encoding="utf-8"))
            other = root / "other"
            other.mkdir()
            identity["repo"] = str(other)
            marker.write_text(json.dumps(identity), encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "another repository"):
                slicer_mutation._persistent_worktree(repo, persistent, first)
            identity["repo"] = str(repo)
            marker.write_text(json.dumps(identity), encoding="utf-8")

            link = root / "retained-link"
            link.symlink_to(persistent, target_is_directory=True)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "symlink"):
                slicer_mutation._persistent_worktree_path(repo, link)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "outside"):
                slicer_mutation._persistent_worktree_path(repo, repo)

    def test_marker_cannot_forge_common_directory_ownership(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            left_root = root / "left"
            right_root = root / "right"
            left_root.mkdir()
            right_root.mkdir()
            left, left_head, _ = self._repo(left_root)
            right, right_head, _ = self._repo(right_root)
            persistent = root / "right-retained"
            lease = slicer_mutation._persistent_worktree(right, persistent, right_head)
            lease.release()
            marker = persistent / slicer_mutation._PERSISTENT_MARKER
            identity = json.loads(marker.read_text(encoding="utf-8"))
            identity["repo"] = str(left.resolve())
            identity["path"] = str(persistent.resolve())
            marker.write_text(json.dumps(identity), encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "another git repository"):
                slicer_mutation._persistent_worktree(left, persistent, left_head)
            self._git(right, "worktree", "remove", "--force", str(persistent))

    def test_report_checkpoint_failure_releases_persistent_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, first, _ = self._repo(root)
            persistent = root / "retained"
            report_path = root / "report.json"
            report = {}

            def fail_ready(_path, _report, phase):
                if phase == "persistent_worktree_ready":
                    raise OSError("report disk failure")

            with mock.patch.object(slicer_mutation, "_checkpoint", side_effect=fail_ready), \
                    self.assertRaises(OSError):
                with slicer_mutation._mutation_worktree(repo, first, None, persistent,
                                                        report_path, report):
                    pass
            self.assertFalse((persistent.with_name(persistent.name + ".slicer-mutation.lock")).exists())

    def test_tracked_edit_by_fake_build_is_rejected_after_source_restore(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, _, current = self._repo(root)
            (repo / "other.cpp").write_text("clean\n", encoding="utf-8")
            self._git(repo, "add", "other.cpp")
            self._git(repo, "commit", "-qm", "other")
            current = self._git(repo, "rev-parse", "HEAD")
            manifest = root / "mutants.json"
            manifest.write_text(json.dumps({"mutants": [{
                "id": "tracked-side-effect", "path": "source.cpp", "old": "second", "new": "after",
                "description": "fake build edits another tracked file", "test_filter": "FakeMutation",
            }]}), encoding="utf-8")
            persistent = root / "retained"
            out = root / "evidence"
            code = ("import pathlib; "
                    "pathlib.Path('other.cpp').write_text('dirty\\n') if "
                    "pathlib.Path('source.cpp').read_text() == 'after\\n' else None")
            test_code = "print('1 tests passed')"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--build-dir", "build",
                "--build-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(code)}",
                "--test-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(test_code)}",
                "--mutants", str(manifest), "--max-mutants", "1", "--out", str(out),
                "--persistent-worktree", str(persistent), "--timeout", "30",
            ])
            with self.assertRaisesRegex(slicer_mutation.MutationError, "dirty tracked"):
                slicer_mutation.run_mutation(args)
            self.assertEqual((persistent / "source.cpp").read_text(encoding="utf-8"), "second\n")
            self.assertEqual((persistent / "other.cpp").read_text(encoding="utf-8"), "dirty\n")
            self.assertFalse((persistent.with_name(persistent.name + ".slicer-mutation.lock")).exists())
            report = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "postflight_dirty")
            self.assertEqual(self._git(repo, "rev-parse", "HEAD"), current)

    def test_missing_git_metadata_stops_before_the_next_persistent_mutant(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, _, _ = self._repo(root)
            manifest = root / "mutants.json"
            manifest.write_text(json.dumps({"mutants": [
                {"id": name, "path": "source.cpp", "old": "second", "new": name,
                 "description": "fake build removes worktree metadata", "test_filter": "FakeMutation"}
                for name in ("first-mutant", "second-mutant")
            ]}), encoding="utf-8")
            persistent = root / "retained"
            code = ("import pathlib; "
                    "s=pathlib.Path('source.cpp').read_text().strip(); "
                    "p=pathlib.Path('.git'); "
                    "pathlib.Path('executed-mutants.txt').open('a').write(s+'\\n') if s!='second' else None; "
                    "p.unlink() if s!='second' and p.exists() else None")
            test_code = "print('1 tests passed')"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo),
                "--build-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(code)}",
                "--test-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(test_code)}",
                "--mutants", str(manifest), "--max-mutants", "2",
                "--out", str(root / "evidence"), "--persistent-worktree", str(persistent),
                "--timeout", "30",
            ])
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation.run_mutation(args)
            self.assertEqual((persistent / "executed-mutants.txt").read_text().splitlines(), ["first-mutant"])
            self.assertEqual((persistent / "source.cpp").read_text(), "second\n")
            self.assertFalse(persistent.with_name(persistent.name + ".slicer-mutation.lock").exists())
            report = json.loads((root / "evidence" / "mutation-report.json").read_text())
            self.assertEqual(report["status"], "postflight_dirty")
            self.assertEqual(len(report["mutants"]), 1)

    def test_run_restores_source_when_fake_mutant_build_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, _, current = self._repo(root)
            manifest = root / "mutants.json"
            manifest.write_text(json.dumps({"mutants": [{
                "id": "fail-build", "path": "source.cpp", "old": "second", "new": "after",
                "description": "fake build failure", "test_filter": "FakeMutation",
            }]}), encoding="utf-8")
            persistent = root / "retained"
            out = root / "evidence"
            code = ("import pathlib,sys; sys.exit(1 if "
                    "pathlib.Path('source.cpp').read_text() == 'after\\n' else 0)")
            test_code = "print('1 tests passed')"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--build-dir", "build",
                "--build-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(code)}",
                "--test-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(test_code)}",
                "--mutants", str(manifest), "--max-mutants", "1", "--out", str(out),
                "--persistent-worktree", str(persistent), "--timeout", "30",
            ])
            self.assertEqual(slicer_mutation.run_mutation(args), 2)
            report = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["mutants"][0]["status"], "build_error")
            self.assertEqual((persistent / "source.cpp").read_text(encoding="utf-8"), "second\n")
            self.assertEqual(report["worktree"]["persistent"], True)
            self.assertEqual(report["worktree"]["reused"], False)
            self.assertEqual(report["worktree"]["retained_untracked_build_capability"],
                             "preserved by non-forcing detached checkout")
            self.assertFalse((persistent.with_name(persistent.name + ".slicer-mutation.lock")).exists())
            self.assertEqual(self._git(repo, "rev-parse", "HEAD"), current)

            second_manifest = root / "mutants-second.json"
            second_manifest.write_text(json.dumps({"mutants": [{
                "id": "fail-build-again", "path": "source.cpp", "old": "second", "new": "third",
                "description": "fake build failure on reused worktree", "test_filter": "FakeMutation",
            }]}), encoding="utf-8")
            second_out = root / "evidence-second"
            second_code = ("import pathlib,sys; sys.exit(1 if "
                            "pathlib.Path('source.cpp').read_text() == 'third\\n' else 0)")
            second_args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--build-dir", "build",
                "--build-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(second_code)}",
                "--test-command", f"{shlex.quote(sys.executable)} -c {shlex.quote(test_code)}",
                "--mutants", str(second_manifest), "--max-mutants", "1", "--out", str(second_out),
                "--persistent-worktree", str(persistent), "--timeout", "30",
            ])
            self.assertEqual(slicer_mutation.run_mutation(second_args), 2)
            second_report = json.loads((second_out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertTrue(second_report["worktree"]["reused"])
            self.assertEqual(second_report["worktree"]["previous_head"], current)
            self.assertEqual((persistent / "source.cpp").read_text(encoding="utf-8"), "second\n")


if __name__ == "__main__":
    unittest.main()
