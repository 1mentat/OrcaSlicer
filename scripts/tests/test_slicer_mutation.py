#!/usr/bin/env python3
"""Unit tests for bounded mutation selection and result classification."""

from pathlib import Path
import hashlib
import json
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


if __name__ == "__main__":
    unittest.main()
