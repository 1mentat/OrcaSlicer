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
import threading
import time
from contextlib import contextmanager
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


class MutationManifestDigestTests(unittest.TestCase):
    def write_manifest(self, path, ident):
        payload = json.dumps({"mutants": [{
            "id": ident, "path": "source.cpp", "old": "before", "new": "after",
            "test_filter": ident.title(),
        }]}).encode("utf-8") + b"\r\n"
        path.write_bytes(payload)
        return hashlib.sha256(payload).hexdigest()

    def test_digest_binds_raw_manifest_bytes_and_each_repeated_selection(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            first = self.write_manifest(root / "first.json", "first")
            second = self.write_manifest(root / "second.json", "second")
            mutants = slicer_mutation._load_mutants(
                ["first.json", str(root / "second.json")], root,
                expected_sha256=[first.upper(), second])
            self.assertEqual([(m.id, m.test_filter) for m in mutants],
                             [("first", "First"), ("second", "Second")])
            with self.assertRaisesRegex(slicer_mutation.MutationError, "SHA-256 mismatch"):
                slicer_mutation._load_mutants(
                    ["second.json", "first.json"], root, expected_sha256=[first, second])
            (root / "first.json").write_bytes((root / "first.json").read_bytes() + b"\n")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "SHA-256 mismatch"):
                slicer_mutation._load_mutants("first.json", root, expected_sha256=first)

    def test_stale_catalog_fails_before_repository_build_or_source_work(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            reviewed = self.write_manifest(root / "reviewed.json", "new-contract")
            self.write_manifest(root / "stale.json", "old-contract")
            source = root / "source.cpp"
            source.write_bytes(b"before\n")
            output = root / "evidence"
            with mock.patch.object(slicer_mutation, "validate_clean_repo") as validate, \
                    mock.patch.object(slicer_mutation, "_mutation_worktree") as worktree, \
                    mock.patch.object(slicer_mutation, "run_command") as command:
                code = slicer_mutation.main([
                    "run", "--root", str(root), "--out", str(output),
                    "--build-command", "build", "--test-command", "test",
                    "--mutants", "stale.json", "--mutants-sha256", reviewed,
                ])
            self.assertEqual(code, 2)
            validate.assert_not_called()
            worktree.assert_not_called()
            command.assert_not_called()
            self.assertEqual(source.read_bytes(), b"before\n")
            self.assertFalse((root / "build").exists())
            report = json.loads((output / "mutation-report.json").read_text())
            self.assertEqual(report["status"], "infrastructure_error")
            self.assertIn("SHA-256 mismatch", report["error"])

    def test_digest_guards_require_unambiguous_explicit_manifest_pairing(self):
        for manifests, digests, error in [
                (None, ["a" * 64], "explicit --mutants"),
                (["first.json", "second.json"], ["a" * 64], "one --mutants-sha256"),
                (["first.json"], ["bad-digest"], "64 hexadecimal")]:
            with self.subTest(manifests=manifests, digests=digests):
                with self.assertRaisesRegex(slicer_mutation.MutationError, error):
                    slicer_mutation._load_mutants(manifests, expected_sha256=digests)

    def test_matching_digest_does_not_hide_invalid_manifest_encoding(self):
        with tempfile.TemporaryDirectory() as temp:
            manifest = Path(temp) / "bad.json"
            manifest.write_bytes(b"\xff")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "cannot read mutant manifest"):
                slicer_mutation._load_mutants(
                    str(manifest), expected_sha256=hashlib.sha256(b"\xff").hexdigest())


class MutationArgvFileTests(unittest.TestCase):
    def make_detached_repo(self, parent):
        repo = Path(parent) / "repo"
        repo.mkdir()

        def git(*args):
            return subprocess.run(["git", "-C", str(repo), *args], check=True,
                                  capture_output=True, text=True).stdout.strip()

        subprocess.run(["git", "init", "-q", str(repo)], check=True)
        git("config", "user.name", "Mutation Test")
        git("config", "user.email", "mutation@example.invalid")
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "source.cpp").write_text("before\n", encoding="utf-8")
        git("add", ".gitignore", "source.cpp")
        git("commit", "-m", "fixture")
        git("checkout", "--detach", "-q", "HEAD")
        (repo / "build").mkdir()
        return repo

    def test_argv_files_preserve_tokens_expand_placeholders_and_bind_raw_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            repo = self.make_detached_repo(temp)
            outside = Path(temp) / "commands"
            outside.mkdir()
            probe = repo / "argv_probe.py"
            probe.write_text(
                "import json, pathlib, sys\n"
                "args = sys.argv[1:]\n"
                "pathlib.Path(args[0]).write_text(json.dumps(args[1:]), encoding='utf-8')\n"
                "if '--report-test-evidence' in args[1:]:\n"
                "    root = next(value.split('=', 1)[1] for value in args if value.startswith('--root='))\n"
                "    junit = pathlib.Path(next(value.split('=', 1)[1] for value in args if value.startswith('--output-junit=')))\n"
                "    failed = (pathlib.Path(root) / 'source.cpp').read_text(encoding='utf-8') == 'after\\n'\n"
                "    junit.parent.mkdir(parents=True, exist_ok=True)\n"
                "    if failed:\n"
                "        junit.write_text('<testsuite name=\"probe\" tests=\"1\" failures=\"1\" errors=\"0\" skipped=\"0\"><testcase classname=\"probe\" name=\"one\"><failure message=\"assertion\">assert</failure></testcase></testsuite>', encoding='utf-8')\n"
                "        print('test cases: 1 | 1 failed\\nassertions: 1 | 1 failed')\n"
                "        sys.exit(1)\n"
                "    junit.write_text('<testsuite name=\"probe\" tests=\"1\" failures=\"0\" errors=\"0\" skipped=\"0\"><testcase classname=\"probe\" name=\"one\"/></testsuite>', encoding='utf-8')\n"
                "    print('test cases: 1 | 1 passed')\n",
                encoding="utf-8")
            subprocess.run(["git", "-C", str(repo), "add", "argv_probe.py"], check=True)
            subprocess.run(["git", "-C", str(repo), "commit", "-m", "probe"],
                           check=True, capture_output=True, text=True)
            build_file = repo / "build" / "commands" / "build.json"
            build_file.parent.mkdir()
            build_tokens = [
                sys.executable, "{root}/argv_probe.py", "{build}/build-argv-seen.json",
                "build value with spaces", "-leading-option", "literal;touch {build}/semicolon-marker",
                "$(touch {build}/substitution-marker)", "",
            ]
            build_bytes = (json.dumps(build_tokens, ensure_ascii=False) + "\n").encode("utf-8")
            build_file.write_bytes(build_bytes)
            test_file = outside / "test.json"
            test_tokens = [
                sys.executable, "{root}/argv_probe.py", "{build}/test-argv-seen.json",
                "--report-test-evidence", "--filter={filter}", "--root={root}",
                "--output-junit={junit}",
                "--build={build}", "test value with spaces", "-another-leading-option",
                "literal;touch {build}/test-marker", "$(touch {build}/test-substitution-marker)", "",
            ]
            test_bytes = json.dumps(test_tokens, ensure_ascii=False).encode("utf-8")
            test_file.write_bytes(test_bytes)
            output = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(output), "--build-dir", "build",
                "--build-argv-file", "build/commands/build.json", "--test-argv-file", str(test_file),
                "--reuse-detached-checkout", "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant(
                "one", "source.cpp", "before", "after", "", "Flow with spaces")
            real_run_command = slicer_mutation.run_command
            read_counts = {build_file.resolve(): 0, test_file.resolve(): 0}
            real_read_bytes = Path.read_bytes

            def counted_read_bytes(path):
                resolved = path.resolve()
                if resolved in read_counts:
                    read_counts[resolved] += 1
                return real_read_bytes(path)

            with mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(Path, "read_bytes", counted_read_bytes), \
                    mock.patch.object(slicer_mutation, "run_command", wraps=real_run_command):
                self.assertEqual(slicer_mutation.run_mutation(args), 0)

            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            root_token = str(repo.resolve())
            build_token = str((repo / "build").resolve())
            evidence_root = output.resolve()
            baseline_junit = evidence_root / "command-logs" / (
                "0002-baseline-test-Flow-with-spaces.junit.xml")
            mutant_junit = evidence_root / "command-logs" / "0004-mutant-test-one.junit.xml"
            expected_build = [
                sys.executable, f"{root_token}/argv_probe.py", f"{build_token}/build-argv-seen.json",
                "build value with spaces", "-leading-option", f"literal;touch {build_token}/semicolon-marker",
                f"$(touch {build_token}/substitution-marker)", "",
            ]
            expected_test = [
                sys.executable, f"{root_token}/argv_probe.py", f"{build_token}/test-argv-seen.json",
                "--report-test-evidence", "--filter=Flow with spaces", f"--root={root_token}",
                f"--output-junit={baseline_junit}",
                f"--build={build_token}", "test value with spaces", "-another-leading-option",
                f"literal;touch {build_token}/test-marker",
                f"$(touch {build_token}/test-substitution-marker)", "",
            ]
            self.assertEqual(read_counts, {build_file.resolve(): 1, test_file.resolve(): 1})
            self.assertEqual(report["preflight"]["argv_files"], {
                "build": {"path": str(build_file.resolve()),
                          "sha256": hashlib.sha256(build_bytes).hexdigest()},
                "test": {"path": str(test_file.resolve()),
                         "sha256": hashlib.sha256(test_bytes).hexdigest()},
            })
            self.assertEqual(report["baseline_build"]["argv"], expected_build)
            self.assertEqual(report["mutants"][0]["build"]["argv"], expected_build)
            self.assertEqual(report["baseline_tests"][0]["result"]["argv"], expected_test)
            mutant_test_argv = list(expected_test)
            mutant_test_argv[6] = f"--output-junit={mutant_junit}"
            self.assertEqual(report["mutants"][0]["test"]["argv"], mutant_test_argv)
            self.assertEqual(json.loads((repo / "build" / "build-argv-seen.json").read_text()),
                             expected_build[3:])
            self.assertEqual(json.loads((repo / "build" / "test-argv-seen.json").read_text()),
                             mutant_test_argv[3:])
            baseline_artifact = report["baseline_tests"][0]["result"]["junit_artifact"]
            mutant_artifact = report["mutants"][0]["test"]["junit_artifact"]
            self.assertEqual(baseline_artifact["path"], str(baseline_junit))
            self.assertEqual(mutant_artifact["path"], str(mutant_junit))
            self.assertNotEqual(baseline_artifact["path"], mutant_artifact["path"])
            for artifact, junit_path, failures in (
                    (baseline_artifact, baseline_junit, 0), (mutant_artifact, mutant_junit, 1)):
                self.assertEqual(junit_path.parent, evidence_root / "command-logs")
                self.assertTrue(junit_path.is_file())
                self.assertEqual(artifact["size_bytes"], junit_path.stat().st_size)
                self.assertEqual(artifact["sha256"], hashlib.sha256(junit_path.read_bytes()).hexdigest())
                self.assertTrue(artifact["usable"])
                self.assertEqual(artifact["counts"], {
                    "tests": 1, "failures": failures, "errors": 0, "skipped": 0, "timeouts": 0,
                })
            for marker in ("semicolon-marker", "substitution-marker", "test-marker",
                           "test-substitution-marker"):
                self.assertFalse((repo / "build" / marker).exists())

    def test_legacy_command_flags_remain_exclusive_and_supported(self):
        args = slicer_mutation._parser().parse_args([
            "run", "--build-command", "cmake --build {build}",
            "--test-command", "ctest --test-dir {build}",
        ])
        self.assertEqual(slicer_mutation.split_command(args.build_command),
                         ["cmake", "--build", "{build}"])
        self.assertEqual(slicer_mutation.split_command(args.test_command),
                         ["ctest", "--test-dir", "{build}"])
        self.assertIsNone(args.build_argv_file)
        self.assertIsNone(args.test_argv_file)
        with self.assertRaises(SystemExit):
            slicer_mutation._parser().parse_args([
                "run", "--build-command", "build", "--build-argv-file", "build.json",
                "--test-command", "test",
            ])

    def test_invalid_argv_files_fail_before_repository_or_build_work(self):
        invalid = {
            "malformed": b"[\n",
            "object": b'{"argv": ["tool"]}',
            "empty": b"[]",
            "nested": b'["tool", ["nested"]]',
            "nonstring": b'["tool", 1]',
            "empty-executable": b'["", "argument"]',
            "nul": b'["tool", "bad\\u0000argument"]',
        }
        for name, payload in invalid.items():
            with self.subTest(name=name), tempfile.TemporaryDirectory() as temp:
                root = Path(temp) / "repo"
                root.mkdir()
                argv_file = root / "bad.json"
                argv_file.write_bytes(payload)
                output = root / "evidence"
                with mock.patch.object(slicer_mutation, "validate_clean_repo") as validate, \
                        mock.patch.object(slicer_mutation, "_mutation_worktree") as worktree, \
                        mock.patch.object(slicer_mutation, "run_command") as command:
                    code = slicer_mutation.main([
                        "run", "--root", str(root), "--out", str(output),
                        "--build-argv-file", "bad.json", "--test-command", "test",
                    ])
                self.assertEqual(code, 2)
                validate.assert_not_called()
                worktree.assert_not_called()
                command.assert_not_called()
                self.assertFalse((root / "build").exists())
                self.assertEqual({path.name for path in output.iterdir()}, {"mutation-report.json"})
                error = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
                self.assertEqual(error["status"], "infrastructure_error")

    def test_invalid_test_argv_file_is_rejected_before_repository_validation(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            argv_file = root / "bad-test.json"
            argv_file.write_text('["tool", null]', encoding="utf-8")
            with mock.patch.object(slicer_mutation, "validate_clean_repo") as validate:
                with self.assertRaisesRegex(slicer_mutation.MutationError, "only strings"):
                    slicer_mutation.run_mutation(slicer_mutation._parser().parse_args([
                        "run", "--root", str(root), "--out", str(root / "evidence"),
                        "--build-command", "build", "--test-argv-file", str(argv_file),
                    ]))
            validate.assert_not_called()
            self.assertFalse((root / "evidence").exists())


class MutationReuseCheckoutTests(unittest.TestCase):
    def make_detached_repo(self, parent):
        repo = Path(parent) / "repo"
        repo.mkdir()

        def git(*args):
            return subprocess.run(["git", "-C", str(repo), *args], check=True,
                                  capture_output=True, text=True).stdout.strip()

        subprocess.run(["git", "init", "-q", str(repo)], check=True)
        git("config", "user.name", "Mutation Test")
        git("config", "user.email", "mutation@example.invalid")
        (repo / ".gitignore").write_text("build/\n", encoding="utf-8")
        (repo / "source.cpp").write_text("before\n", encoding="utf-8")
        git("add", ".gitignore", "source.cpp")
        git("commit", "-m", "fixture")
        git("checkout", "--detach", "-q", "HEAD")
        return repo, git("rev-parse", "HEAD")

    def test_reuse_requires_git_root_detached_exact_head_and_clean_tracked_files(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = self.make_detached_repo(temp)
            slicer_mutation._verify_reuse_checkout(repo, commit)

            subprocess.run(["git", "-C", str(repo), "switch", "-c", "feature"],
                           check=True, capture_output=True)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "detached"):
                slicer_mutation._verify_reuse_checkout(repo, commit)
            subprocess.run(["git", "-C", str(repo), "checkout", "--detach", commit],
                           check=True, capture_output=True)

            (repo / "source.cpp").write_text("dirty\n", encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "dirty tracked"):
                slicer_mutation._verify_reuse_checkout(repo, commit)
            (repo / "source.cpp").write_text("before\n", encoding="utf-8")
            (repo / "unexpected.tmp").write_text("untracked\n", encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "not empty"):
                slicer_mutation.validate_clean_repo(repo)
            (repo / "unexpected.tmp").unlink()
            with self.assertRaisesRegex(slicer_mutation.MutationError, "HEAD changed"):
                slicer_mutation._verify_reuse_checkout(repo, "0" * len(commit))

    def test_reuse_rejects_missing_git_metadata_and_existing_stale_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            missing_git = Path(temp) / "not-a-repository"
            missing_git.mkdir()
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._acquire_reuse_checkout(missing_git, "a" * 40)

            repo, commit = self.make_detached_repo(temp)
            git_dir = Path(subprocess.run(
                ["git", "-C", str(repo), "rev-parse", "--absolute-git-dir"],
                check=True, capture_output=True, text=True).stdout.strip())
            lock = git_dir / slicer_mutation._REUSE_CHECKOUT_LOCK
            lock.write_text("stale lock; do not clear automatically\n", encoding="utf-8")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "active or stale"):
                slicer_mutation._acquire_reuse_checkout(repo, commit)
            self.assertEqual(lock.read_text(encoding="utf-8"),
                             "stale lock; do not clear automatically\n")

    def test_reuse_lock_uses_linked_checkouts_private_git_directory(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = self.make_detached_repo(temp)
            linked = Path(temp) / "linked"
            subprocess.run(["git", "-C", str(repo), "worktree", "add", "--detach",
                            str(linked), commit], check=True, capture_output=True)
            lease = slicer_mutation._acquire_reuse_checkout(linked, commit)
            self.assertEqual(lease.git_dir, Path(subprocess.run(
                ["git", "-C", str(linked), "rev-parse", "--absolute-git-dir"],
                check=True, capture_output=True, text=True).stdout.strip()).resolve())
            self.assertNotEqual(lease.git_dir, (repo / ".git").resolve())
            self.assertTrue(lease.lock.exists())
            lease.release()
            self.assertFalse(lease.lock.exists())

    def test_reuse_lock_release_preserves_an_atomically_replaced_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = self.make_detached_repo(temp)
            lease = slicer_mutation._acquire_reuse_checkout(repo, commit)
            original_identity = lease.lock_identity
            replacement = lease.git_dir / "replacement-lock"
            replacement.write_text("replacement lock owner\n", encoding="utf-8")
            replacement.replace(lease.lock)
            current = lease.lock.lstat()
            self.assertNotEqual((current.st_dev, current.st_ino), original_identity)

            with self.assertRaisesRegex(slicer_mutation.MutationError, "lock changed"):
                lease.release()
            self.assertEqual(lease.lock.read_text(encoding="utf-8"), "replacement lock owner\n")
            lease.lock.unlink()

    def test_moved_detached_head_fails_postflight_without_resetting_it(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, original_commit = self.make_detached_repo(temp)
            report_path = Path(temp) / "evidence" / "report.json"
            report = {"status": "in_progress"}
            git_dir = Path(subprocess.run(
                ["git", "-C", str(repo), "rev-parse", "--absolute-git-dir"],
                check=True, capture_output=True, text=True).stdout.strip())
            with self.assertRaisesRegex(slicer_mutation.MutationError, "HEAD changed"):
                with slicer_mutation._mutation_worktree(
                        repo, original_commit, None, None, report_path, report, True):
                    subprocess.run([
                        "git", "-C", str(repo), "-c", "user.name=Mutation Test",
                        "-c", "user.email=mutation@example.invalid", "commit",
                        "--allow-empty", "-m", "move detached HEAD",
                    ], check=True, capture_output=True)

            moved_head = subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                                        check=True, capture_output=True, text=True).stdout.strip()
            self.assertNotEqual(moved_head, original_commit)
            branch = subprocess.run(["git", "-C", str(repo), "symbolic-ref", "--quiet", "HEAD"],
                                    capture_output=True, text=True)
            self.assertEqual(branch.returncode, 1)
            self.assertFalse((git_dir / slicer_mutation._REUSE_CHECKOUT_LOCK).exists())
            self.assertTrue(report["worktree"]["lock_released"])
            self.assertEqual(report["status"], "postflight_dirty")

    def test_reuse_lease_releases_on_success_error_and_keyboard_interrupt(self):
        for failure in (None, RuntimeError("failed body"), KeyboardInterrupt()):
            with self.subTest(failure=type(failure).__name__ if failure else "success"), \
                    tempfile.TemporaryDirectory() as temp:
                repo, commit = self.make_detached_repo(temp)
                git_dir = Path(subprocess.run(
                    ["git", "-C", str(repo), "rev-parse", "--absolute-git-dir"],
                    check=True, capture_output=True, text=True).stdout.strip())
                report_path = Path(temp) / "evidence" / "report.json"
                report = {"status": "in_progress"}
                if failure is None:
                    with slicer_mutation._mutation_worktree(
                            repo, commit, None, None, report_path, report, True) as path:
                        self.assertEqual(path.resolve(), repo.resolve())
                else:
                    with self.assertRaises(type(failure)):
                        with slicer_mutation._mutation_worktree(
                                repo, commit, None, None, report_path, report, True):
                            raise failure
                self.assertFalse((git_dir / slicer_mutation._REUSE_CHECKOUT_LOCK).exists())
                self.assertTrue(report["worktree"]["lock_released"])

    def test_run_reuses_detached_checkout_and_restores_source(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = self.make_detached_repo(temp)
            build = repo / "build"
            build.mkdir()
            (build / "preserve.txt").write_text("warm build", encoding="utf-8")
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--build-dir", "build", "--out", str(out),
                "--build-command", "fake-build {build}", "--test-command", "fake-test",
                "--reuse-detached-checkout", "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "")
            calls = []
            git_calls = []
            real_git = slicer_mutation._git
            real_run_command = slicer_mutation.run_command

            def command(argv, *, cwd, timeout, output_file=None):
                if argv[0] == "git":
                    return real_run_command(argv, cwd=cwd, timeout=timeout, output_file=output_file)
                calls.append((list(argv), Path(cwd)))
                if argv[0] == "fake-test" and sum(call[0][0] == "fake-test" for call in calls) == 2:
                    return slicer_mutation.CommandResult(list(argv), 1, .01,
                        "test cases: 1 | 1 failed\nassertions: 1 | 1 failed\n", "")
                output = "test cases: 1 | 1 passed\n" if argv[0] == "fake-test" else ""
                return slicer_mutation.CommandResult(list(argv), 0, .01, output, "")

            def recording_git(path, *git_args):
                git_calls.append(git_args)
                return real_git(path, *git_args)

            with mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(slicer_mutation, "_git", side_effect=recording_git), \
                    mock.patch.object(slicer_mutation, "run_command", side_effect=command):
                self.assertEqual(slicer_mutation.run_mutation(args), 0)

            report = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["git"]["commit"], commit)
            self.assertTrue(report["worktree"]["reused_detached_checkout"])
            self.assertIn("explicitly rebuild the baseline", report["worktree"]["binary_warning"])
            self.assertNotIn("junit_artifact", report["baseline_tests"][0]["result"])
            self.assertNotIn("junit_artifact", report["mutants"][0]["test"])
            self.assertEqual((repo / "source.cpp").read_text(encoding="utf-8"), "before\n")
            self.assertEqual((build / "preserve.txt").read_text(encoding="utf-8"), "warm build")
            self.assertTrue(all(cwd.resolve() == repo.resolve() for _, cwd in calls))
            self.assertFalse(any(args[0] in {"checkout", "reset", "worktree"} for args in git_calls))
            self.assertFalse((repo / ".git" / slicer_mutation._REUSE_CHECKOUT_LOCK).exists())
            self.assertEqual(subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                check=True, capture_output=True, text=True).stdout.strip(), commit)

    def test_keyboard_interrupt_restores_source_and_releases_reuse_lock(self):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = self.make_detached_repo(temp)
            out = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
                "--reuse-detached-checkout", "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "")
            git_dir = Path(subprocess.run(
                ["git", "-C", str(repo), "rev-parse", "--absolute-git-dir"],
                check=True, capture_output=True, text=True).stdout.strip())
            real_run_command = slicer_mutation.run_command
            command_count = 0

            def command(argv, *, cwd, timeout, output_file=None):
                nonlocal command_count
                if argv[0] == "git":
                    return real_run_command(argv, cwd=cwd, timeout=timeout, output_file=output_file)
                command_count += 1
                if command_count == 4:
                    raise KeyboardInterrupt
                output = "test cases: 1 | 1 passed\n" if argv[0] == "fake-test" else ""
                return slicer_mutation.CommandResult(list(argv), 0, .01, output, "")

            with mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(slicer_mutation, "run_command", side_effect=command):
                with self.assertRaises(KeyboardInterrupt):
                    slicer_mutation.run_mutation(args)

            self.assertEqual((repo / "source.cpp").read_text(encoding="utf-8"), "before\n")
            self.assertEqual(subprocess.run(["git", "-C", str(repo), "rev-parse", "HEAD"],
                check=True, capture_output=True, text=True).stdout.strip(), commit)
            self.assertFalse((git_dir / slicer_mutation._REUSE_CHECKOUT_LOCK).exists())
            checkpoint = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(checkpoint["checkpoint"]["state"], "in_progress")

    def test_reuse_options_cannot_mix_with_worktree_modes(self):
        with tempfile.TemporaryDirectory() as temp:
            for extra, message in ((["--persistent-worktree", str(Path(temp) / "wt")],
                                    "cannot be combined with --persistent-worktree"),
                                   (["--work-root", str(Path(temp) / "work")],
                                    "cannot be combined with --work-root")):
                args = slicer_mutation._parser().parse_args([
                    "run", "--root", temp, "--out", str(Path(temp) / "out"),
                    "--build-command", "build", "--test-command", "test",
                    "--reuse-detached-checkout", *extra,
                ])
                with self.assertRaisesRegex(slicer_mutation.MutationError, message):
                    slicer_mutation.run_mutation(args)

    def test_reuse_mode_is_opt_in(self):
        args = slicer_mutation._parser().parse_args([
            "run", "--build-command", "build", "--test-command", "test",
        ])
        self.assertFalse(args.reuse_detached_checkout)


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


class MutationJUnitArtifactTests(unittest.TestCase):
    @staticmethod
    def _junit_xml(failures=0, cases=1):
        failed_case = '<failure message="assertion">assert</failure>' if failures else ""
        testcases = '<testcase classname="probe" name="one">' + failed_case + '</testcase>'
        if cases == 2:
            testcases += '<testcase classname="probe" name="two"/>'
        return (f'<testsuite name="probe" tests="{cases}" failures="{failures}" '
                f'errors="0" skipped="0">{testcases}</testsuite>')

    def _run_with_bad_requested_junit(self, *, invalid_phase, artifact_kind):
        with tempfile.TemporaryDirectory() as temp:
            repo, commit = MutationReuseCheckoutTests().make_detached_repo(temp)
            output = Path(temp) / "evidence"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(output), "--build-dir", "build",
                "--build-command", "fake-build {build}",
                "--test-command", "fake-test --output-junit {junit}",
                "--reuse-detached-checkout", "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "")
            real_run_command = slicer_mutation.run_command
            test_count = 0
            calls = []

            def recorded(argv, *, cwd, timeout, output_file=None):
                nonlocal test_count
                if argv[0] == "git":
                    return real_run_command(argv, cwd=cwd, timeout=timeout,
                                            output_file=output_file)
                phase = "build"
                code = 0
                text = ""
                if argv[0] == "fake-test":
                    test_count += 1
                    phase = "baseline" if test_count == 1 else "mutant"
                    calls.append((phase, list(argv)))
                    failed = phase == "mutant"
                    code = 8 if failed else 0
                    text = ("test cases: 1 | 1 failed\nassertions: 1 | 1 failed\n"
                            "0% tests passed, 1 tests failed out of 1\n" if failed else
                            "test cases: 1 | 1 passed\n100% tests passed, 0 tests failed out of 1\n")
                    junit = Path(argv[argv.index("--output-junit") + 1])
                    if not (phase == invalid_phase and artifact_kind == "missing"):
                        junit.parent.mkdir(parents=True, exist_ok=True)
                        if phase == invalid_phase and artifact_kind == "malformed":
                            junit.write_bytes(b"<testsuite tests=\"1\">")
                        else:
                            junit.write_text(self._junit_xml(failures=int(failed)), encoding="utf-8")
                if output_file is not None:
                    output_file.parent.mkdir(parents=True, exist_ok=True)
                    output_file.write_text(text, encoding="utf-8")
                    digest, size = slicer_mutation._log_metadata(output_file)
                    return slicer_mutation.CommandResult(
                        list(argv), code, .01, text, "", log_path=str(output_file.resolve()),
                        log_sha256=digest, log_size_bytes=size)
                return slicer_mutation.CommandResult(list(argv), code, .01, text, "")

            with mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(slicer_mutation, "run_command", side_effect=recorded):
                code = slicer_mutation.run_mutation(args)

            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            return code, report, (repo / "source.cpp").read_bytes(), calls, commit

    def test_bad_baseline_junit_aborts_before_running_mutants(self):
        for artifact_kind in ("missing", "malformed"):
            with self.subTest(artifact_kind=artifact_kind):
                code, report, source, calls, commit = self._run_with_bad_requested_junit(
                    invalid_phase="baseline", artifact_kind=artifact_kind)
                self.assertEqual(code, 2)
                self.assertEqual(report["status"], "baseline_test_evidence_incomplete")
                self.assertEqual(report["mutants"], [])
                self.assertEqual([phase for phase, _ in calls], ["baseline"])
                self.assertEqual(source, b"before\n")
                self.assertTrue(report["worktree"]["postflight_tracked_clean"])
                artifact = report["baseline_tests"][0]["result"]["junit_artifact"]
                self.assertFalse(artifact["usable"])
                self.assertTrue(artifact["errors"])
                self.assertEqual(report["git"]["commit"], commit)

    def test_bad_mutant_junit_is_incomplete_and_restores_source(self):
        for artifact_kind in ("missing", "malformed"):
            with self.subTest(artifact_kind=artifact_kind):
                code, report, source, calls, _commit = self._run_with_bad_requested_junit(
                    invalid_phase="mutant", artifact_kind=artifact_kind)
                self.assertEqual(code, 2)
                self.assertEqual(report["status"], "infrastructure_error")
                self.assertEqual([phase for phase, _ in calls], ["baseline", "mutant"])
                self.assertEqual(source, b"before\n")
                self.assertTrue(report["worktree"]["postflight_tracked_clean"])
                mutant = report["mutants"][0]
                self.assertEqual(mutant["status"], "test_junit_evidence_incomplete")
                self.assertTrue(mutant["restoration"]["restored"])
                self.assertTrue(mutant["postflight_tracked_clean"])
                self.assertFalse(mutant["test"]["junit_artifact"]["usable"])
                self.assertTrue(mutant["test"]["junit_artifact"]["errors"])

    def test_missing_or_invalid_requested_junit_cannot_be_a_mutation_kill(self):
        log_evidence = slicer_mutation.LogEvidence(test_count=1, assertion_failure=True)
        for kind in ("missing", "invalid"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp:
                log_root = Path(temp) / "command-logs"
                log_root.mkdir()
                path = log_root / f"mutant-{kind}.junit.xml"
                if kind == "invalid":
                    path.write_bytes(b"<testsuite tests=\"1\">")
                result = slicer_mutation.CommandResult(["ctest"], 8, .01, "", "")

                artifact = slicer_mutation._capture_junit_artifact(
                    result, path, log_root=log_root, console_test_count=1,
                    log_evidence=log_evidence)

                self.assertEqual(artifact["path"], str(path))
                self.assertFalse(artifact["usable"])
                self.assertTrue(artifact["errors"])
                if kind == "missing":
                    self.assertIsNone(artifact["sha256"])
                    self.assertIsNone(artifact["size_bytes"])
                else:
                    self.assertEqual(artifact["size_bytes"], path.stat().st_size)
                    self.assertEqual(artifact["sha256"],
                                     hashlib.sha256(path.read_bytes()).hexdigest())
                self.assertEqual(slicer_mutation._classify_test(
                    result, 1, 1, log_evidence=log_evidence,
                    junit_artifact=artifact), "test_junit_evidence_incomplete")

    def test_oversized_requested_junit_is_hashed_but_rejected_without_parsing(self):
        with tempfile.TemporaryDirectory() as temp:
            log_root = Path(temp) / "command-logs"
            log_root.mkdir()
            path = log_root / "large.junit.xml"
            path.write_text(self._junit_xml(), encoding="utf-8")
            result = slicer_mutation.CommandResult(["ctest"], 0, .01, "", "")
            log_evidence = slicer_mutation.LogEvidence(test_count=1)

            with mock.patch.object(slicer_mutation, "MAX_JUNIT_BYTES", 1), \
                    mock.patch.object(slicer_mutation, "read_junit_evidence") as reader:
                artifact = slicer_mutation._capture_junit_artifact(
                    result, path, log_root=log_root, console_test_count=1,
                    log_evidence=log_evidence)

            reader.assert_not_called()
            self.assertFalse(artifact["usable"])
            self.assertTrue(any("evidence limit" in error for error in artifact["errors"]))
            self.assertEqual(artifact["size_bytes"], path.stat().st_size)
            self.assertEqual(artifact["sha256"], hashlib.sha256(path.read_bytes()).hexdigest())

    def test_valid_junit_case_count_must_match_console_count(self):
        with tempfile.TemporaryDirectory() as temp:
            log_root = Path(temp) / "command-logs"
            log_root.mkdir()
            path = log_root / "count-mismatch.junit.xml"
            path.write_text(self._junit_xml(cases=2), encoding="utf-8")
            parsed = slicer_mutation.read_junit_evidence(path, reporter_profile="ctest")
            self.assertTrue(parsed["usable"])
            self.assertEqual(parsed["counts"]["tests"], 2)
            result = slicer_mutation.CommandResult(["ctest"], 0, .01, "", "")
            log_evidence = slicer_mutation.LogEvidence(test_count=1)

            artifact = slicer_mutation._capture_junit_artifact(
                result, path, log_root=log_root, console_test_count=1,
                log_evidence=log_evidence)

            self.assertFalse(artifact["usable"])
            self.assertTrue(any("count differs" in error for error in artifact["errors"]))
            self.assertEqual(slicer_mutation._classify_test(
                result, 1, 1, log_evidence=log_evidence,
                junit_artifact=artifact), "test_junit_evidence_incomplete")


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
            prior_report = target / "mutation-report.json"
            prior_bytes = (b'{ "schema_version": 1, "status": "in_progress", '
                           b'"mutants": [{"id": "prior", "status": "in_progress"}] }\n')
            prior_report.write_bytes(prior_bytes)
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
            self.assertEqual(prior_report.read_bytes(), prior_bytes)
            self.assertEqual(slicer_mutation.main([
                "run", "--root", str(root), "--out", str(link),
                "--build-command", "build", "--test-command", "test",
            ]), 2)
            self.assertEqual(prior_report.read_bytes(), prior_bytes)

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
            self.assertNotIn("argv_files", final_report["preflight"])
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

    def test_report_shows_active_mutant_while_fake_child_is_running(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo = root / "repo"
            repo.mkdir()
            source = repo / "source.cpp"
            source.write_text("old", encoding="utf-8")
            out = root / "evidence"
            build_count = root / "build-count"
            test_count = root / "test-count"
            child_started = root / "child-started"
            release_child = root / "release-child"
            build_argv = root / "build.argv.json"
            test_argv = root / "test.argv.json"
            build_script = (
                "import pathlib,sys,time\n"
                "count=pathlib.Path(sys.argv[1]); n=int(count.read_text())+1 if count.exists() else 1\n"
                "count.write_text(str(n))\n"
                "if n == 2:\n"
                " pathlib.Path(sys.argv[2]).touch()\n"
                " while not pathlib.Path(sys.argv[3]).exists(): time.sleep(0.01)\n"
            )
            test_script = (
                "import pathlib,sys\n"
                "count=pathlib.Path(sys.argv[1]); n=int(count.read_text())+1 if count.exists() else 1\n"
                "count.write_text(str(n))\n"
                "if n == 1: print('test cases: 1 | 1 passed')\n"
                "else:\n"
                " print('assertions: 1 | 1 failed\\n0% tests passed, 1 tests failed out of 1')\n"
                " sys.exit(8)\n"
            )
            build_argv.write_text(json.dumps([
                sys.executable, "-c", build_script,
                str(build_count), str(child_started), str(release_child),
            ]), encoding="utf-8")
            test_argv.write_text(json.dumps([
                sys.executable, "-c", test_script, str(test_count),
            ]), encoding="utf-8")
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-argv-file", str(build_argv),
                "--test-argv-file", str(test_argv), "--max-mutants", "1",
            ])
            mutant = slicer_mutation.Mutant("one", "source.cpp", "old", "new", "")

            @contextmanager
            def use_repo(*_args, **_kwargs):
                yield repo

            result = []
            errors = []
            finished = threading.Event()

            def run():
                try:
                    result.append(slicer_mutation.run_mutation(args))
                except BaseException as exc:
                    errors.append(exc)
                finally:
                    finished.set()

            worker = threading.Thread(target=run)
            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                    mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(slicer_mutation, "_mutation_worktree", side_effect=use_repo):
                worker.start()
                try:
                    deadline = time.monotonic() + 10
                    while not child_started.exists() and not finished.is_set() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    self.assertTrue(child_started.exists(), "fake build child did not reach the barrier")
                    live = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
                    self.assertEqual(live["status"], "in_progress")
                    self.assertEqual(live["mutants"][0]["id"], "one")
                    self.assertEqual(live["mutants"][0]["status"], "in_progress")
                finally:
                    release_child.touch()
                    worker.join(timeout=20)
            self.assertFalse(worker.is_alive(), "fake validation thread did not finish")
            self.assertEqual(errors, [])
            self.assertEqual(result, [0])
            final = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(final["status"], "pass")
            self.assertEqual(final["mutants"][0]["status"], "killed")
            self.assertEqual(final["checkpoint"]["state"], "complete")

    def test_run_persists_infrastructure_error_before_propagating_exception(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo = root / "repo"
            repo.mkdir()
            out = root / "evidence"
            mutant = slicer_mutation.Mutant("one", "source.cpp", "old", "new", "")
            command_count = 0
            source = repo / "source.cpp"
            args = slicer_mutation._parser().parse_args([
                "run", "--root", str(repo), "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
                "--max-mutants", "1",
            ])

            @contextmanager
            def use_repo(*_args, **_kwargs):
                yield repo

            def fake_command(argv, *, cwd, timeout, output_file=None):
                nonlocal command_count
                command_count += 1
                if command_count == 3:
                    raise RuntimeError("fake child error")
                return slicer_mutation.CommandResult(
                    list(argv), 0, 0.01, "test cases: 1 | 1 passed\n", "")

            with mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                    mock.patch.object(slicer_mutation, "_load_mutants", return_value=[mutant]), \
                    mock.patch.object(slicer_mutation, "_mutation_worktree", side_effect=use_repo), \
                    mock.patch.object(slicer_mutation, "_apply", return_value=(source, b"old")), \
                    mock.patch.object(slicer_mutation, "run_command", side_effect=fake_command):
                with self.assertRaisesRegex(RuntimeError, "fake child error"):
                    slicer_mutation.run_mutation(args)
            report = json.loads((out / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "infrastructure_error")
            self.assertEqual(report["error"], "fake child error")
            self.assertEqual(report["mutants"][0]["status"], "infrastructure_error")

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
            prior_bytes = (b'{ "schema_version": 1, "status": "in_progress", '
                           b'"mutants": [{"id": "prior", "status": "in_progress"}] }\n')
            marker.write_bytes(prior_bytes)
            code = slicer_mutation.main([
                "run", "--root", temp, "--out", str(out),
                "--build-command", "fake-build", "--test-command", "fake-test",
            ])
            self.assertEqual(code, 2)
            self.assertEqual(marker.read_bytes(), prior_bytes)


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
                         [("first", "killed"), ("second", "in_progress")])

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


class MutationTargetModeTests(unittest.TestCase):
    def _write_file_api(self, build, *, configurations):
        reply = build / ".cmake" / "api" / "v1" / "reply"
        reply.mkdir(parents=True, exist_ok=True)
        config_rows = []
        for config_name, relative_artifact in configurations:
            target_file = f"target-{config_name or 'single'}.json"
            artifact = build / relative_artifact
            artifact.parent.mkdir(parents=True, exist_ok=True)
            artifact.write_text("normal", encoding="utf-8")
            artifact.chmod(0o755)
            (reply / target_file).write_text(json.dumps({
                "name": "fake_tests", "type": "EXECUTABLE",
                "artifacts": [{"path": relative_artifact}],
            }), encoding="utf-8")
            config_rows.append({"name": config_name, "targets": [
                {"name": "fake_tests", "jsonFile": target_file}]})
        (reply / "codemodel-v2.json").write_text(json.dumps({"configurations": config_rows}), encoding="utf-8")
        (reply / "index-20261030T120000.json").write_text(json.dumps({
            "reply": {"codemodel-v2": {"jsonFile": "codemodel-v2.json"}}}), encoding="utf-8")
        return artifact

    def test_file_api_resolves_single_and_multi_configuration_artifact_paths(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            single_build = root / "single"
            single_build.mkdir()
            single_artifact = single_build / "bin" / "fake_tests"
            self._write_file_api(single_build, configurations=[("", "bin/fake_tests")])
            single = slicer_mutation._file_api_target(single_build, "fake_tests", "Release")
            self.assertEqual(single["artifact"], str(single_artifact.resolve()))
            self.assertEqual(single["configuration"], "")

            multi_build = root / "multi"
            multi_build.mkdir()
            release_artifact = multi_build / "Release" / "fake_tests"
            self._write_file_api(multi_build, configurations=[
                ("Debug", "Debug/fake_tests"), ("Release", "Release/fake_tests")])
            multi = slicer_mutation._file_api_target(multi_build, "fake_tests", "Release")
            self.assertEqual(multi["artifact"], str(release_artifact.resolve()))
            self.assertEqual(multi["configuration"], "Release")

    def test_ctest_label_strings_and_lists_select_exact_tests_and_reject_bad_bindings(self):
        with tempfile.TemporaryDirectory() as temp:
            build = Path(temp) / "build"
            binary = build / "bin" / "test"
            other = build / "bin" / "other"
            binary.parent.mkdir(parents=True)
            binary.write_text("binary")
            other.write_text("other")
            tests = slicer_mutation._ctest_tests({"tests": [
                {"name": "Focused::one", "command": [str(binary)], "properties": [
                    {"name": "LABELS", "value": ["Group", "Other"]}]},
                {"name": "Focused::two", "command": [str(binary)], "properties": {
                    "LABELS": "Group;Settings"}},
                {"name": "other_NOT_BUILT", "properties": {
                    "WORKING_DIRECTORY": str(build)}},
            ]})
            selected = slicer_mutation._selected_ctest_names(
                tests, control_regex="^Focused::", exclude_label_regex="NotWorking",
                test_filter="^Group$")
            self.assertEqual(selected, ["Focused::one", "Focused::two"])
            inventory = slicer_mutation._compare_inventory(
                selected, ["Focused::one", "Focused::two"], "control")
            self.assertEqual(inventory["count"], 2)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "inventory differs"):
                slicer_mutation._compare_inventory(
                    selected, ["Focused::one", "Focused::missing"], "control")
            with self.assertRaisesRegex(slicer_mutation.MutationError, "executable mismatch"):
                slicer_mutation._verify_ctest_executables(
                    tests, {"Focused::one"}, build, other)
            with self.assertRaisesRegex(slicer_mutation.MutationError, "has no executable command"):
                slicer_mutation._verify_ctest_executables(
                    tests, {"other_NOT_BUILT"}, build, other)

    def _fixture(self, root, *, first_control_fails=False, failed_restore_build=False):
        repo = root / "repo"
        repo.mkdir()
        build = repo / "build"
        build.mkdir()
        (build / "CMakeCache.txt").write_text(
            f"CMAKE_HOME_DIRECTORY:INTERNAL={repo}\nCMAKE_BUILD_TYPE:STRING=Release\n",
            encoding="utf-8")
        source = repo / "source.cpp"
        source.write_text("before\n", encoding="utf-8")
        output = root / "evidence"
        expected = root / "expected.json"
        expected.write_text(json.dumps(["Focused::one"]), encoding="utf-8")
        first = root / "first.json"
        second = root / "second.json"
        for path, ident in ((first, "first"), (second, "second")):
            path.write_text(json.dumps({"mutants": [{
                "id": ident, "path": "source.cpp", "old": "before", "new": "after",
                "description": ident, "test_filter": "^MutationLabel$",
            }]}), encoding="utf-8")
        cmake = root / "cmake"
        ctest = root / "ctest"
        cmake.write_text(
            "#!/usr/bin/env python3\n"
            "import json,os,pathlib,sys\n"
            "args=sys.argv[1:]; repo=pathlib.Path(os.environ['TARGET_REPO']); build=pathlib.Path(args[args.index('-B')+1]) if '-B' in args else pathlib.Path(args[args.index('--build')+1])\n"
            "if '--build' in args:\n"
            " c=build/'build-count'; n=int(c.read_text())+1 if c.exists() else 1; c.write_text(str(n))\n"
            " if os.environ.get('FAIL_RESTORE_BUILD')=='1' and n==4: print('restore build failed'); sys.exit(17)\n"
            " artifact=build/'Release'/'fake_tests'; artifact.parent.mkdir(parents=True,exist_ok=True); artifact.write_text('mutant' if (repo/'source.cpp').read_text()=='after\\n' else 'normal'); artifact.chmod(0o755)\n"
            " reply=build/'.cmake/api/v1/reply'; reply.mkdir(parents=True,exist_ok=True)\n"
            " (reply/'target.json').write_text(json.dumps({'name':'fake_tests','type':'EXECUTABLE','artifacts':[{'path':'Release/fake_tests'}]}))\n"
            " (reply/'codemodel-v2.json').write_text(json.dumps({'configurations':[{'name':'Release','targets':[{'name':'fake_tests','jsonFile':'target.json'}]}]}))\n"
            " (reply/'index-20261030T120000.json').write_text(json.dumps({'reply':{'codemodel-v2':{'jsonFile':'codemodel-v2.json'}}}))\n"
            "print('cmake ok')\n", encoding="utf-8")
        ctest.write_text(
            "#!/usr/bin/env python3\n"
            "import json,os,pathlib,sys\n"
            "args=sys.argv[1:]; build=pathlib.Path(args[args.index('--test-dir')+1]); artifact=build/'Release'/'fake_tests'\n"
            "record=pathlib.Path(os.environ['TARGET_ENV_LOG']); record.parent.mkdir(parents=True,exist_ok=True)\n"
            "with record.open('a') as f: f.write(json.dumps({k:os.environ.get(k) for k in ['PATH','LC_ALL','LD_LIBRARY_PATH','KEEP_ENV','GIT_DIR','GIT_WORK_TREE','TMPDIR','TEMP','TEMPDIR','LLVM_PROFILE_FILE']})+'\\n')\n"
            "if '--show-only=json-v1' in args:\n"
            " print(json.dumps({'tests':[{'name':'Focused::one','command':[str(artifact)],'properties':[{'name':'LABELS','value':['MutationLabel']},{'name':'WORKING_DIRECTORY','value':str(build)}]}]})); sys.exit(0)\n"
            "junit=pathlib.Path(args[args.index('--output-junit')+1]); junit.parent.mkdir(parents=True,exist_ok=True)\n"
            "has_label='-L' in args; countfile=record.with_suffix('.controls'); count=int(countfile.read_text())+1 if countfile.exists() else 1\n"
            "if not has_label: countfile.write_text(str(count))\n"
            "fail=(has_label and pathlib.Path(os.environ['TARGET_SOURCE']).read_text()=='after\\n') or (os.environ.get('FAIL_FIRST_CONTROL')=='1' and not has_label and count==1)\n"
            "failure='<failure message=\"assertion\" type=\"ASSERTION\">ordinary failed assertion</failure>' if fail else ''\n"
            "junit.write_text(f'<testsuite name=\"Focused::one\" tests=\"1\" failures=\"{int(fail)}\" errors=\"0\" skipped=\"0\"><testcase classname=\"Focused::one\" name=\"Focused::one\">{failure}</testcase></testsuite>')\n"
            "print('assertions: 1 | 1 failed\\n0% tests passed, 1 tests failed out of 1' if fail else '100% tests passed, 0 tests failed out of 1')\n"
            "sys.exit(8 if fail else 0)\n", encoding="utf-8")
        cmake.chmod(0o755)
        ctest.chmod(0o755)
        args = slicer_mutation._parser().parse_args([
            "run", "--root", str(repo), "--build-dir", "build", "--target", "fake_tests",
            "--config", "Release", "--jobs", "2", "--cmake", str(cmake), "--ctest", str(ctest),
            "--control-regex", "^Focused::", "--expected-control-inventory", str(expected),
            "--mutants", str(first), "--mutants", str(second), "--max-mutants", "2",
            "--timeout", "30", "--out", str(output),
        ])
        env = {"TARGET_REPO": str(repo), "TARGET_SOURCE": str(source),
               "TARGET_ENV_LOG": str(root / "child-env.jsonl")}
        if first_control_fails:
            env["FAIL_FIRST_CONTROL"] = "1"
        if failed_restore_build:
            env["FAIL_RESTORE_BUILD"] = "1"
        return repo, source, output, args, env

    def _run_fake_target(self, root, **options):
        repo, source, output, args, child_env = self._fixture(root, **options)

        @contextmanager
        def use_repo(*_args, **_kwargs):
            yield repo

        inherited = {"TARGET_REPO": child_env["TARGET_REPO"],
                     "TARGET_SOURCE": child_env["TARGET_SOURCE"],
                     "TARGET_ENV_LOG": child_env["TARGET_ENV_LOG"],
                     "PATH": os.environ["PATH"], "LC_ALL": "C.UTF-8",
                     "KEEP_ENV": "preserved", "GIT_DIR": "/external/snapshot.git",
                     "GIT_WORK_TREE": "/external/worktree", "TMPDIR": str(root),
                     "TMP": str(root), "TEMP": str(root), "TEMPDIR": str(root)}
        if "FAIL_FIRST_CONTROL" in child_env:
            inherited["FAIL_FIRST_CONTROL"] = child_env["FAIL_FIRST_CONTROL"]
        if "FAIL_RESTORE_BUILD" in child_env:
            inherited["FAIL_RESTORE_BUILD"] = child_env["FAIL_RESTORE_BUILD"]
        inherited["LD_LIBRARY_PATH"] = os.environ.get("LD_LIBRARY_PATH", "/usr/lib")
        with mock.patch.dict(os.environ, inherited), \
                mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                mock.patch.object(slicer_mutation, "_mutation_worktree_with_report", side_effect=use_repo):
            code = slicer_mutation.run_mutation(args)
        return code, repo, source, output, root / "child-env.jsonl"

    def test_target_mode_batches_catalogs_with_one_filter_baseline_and_restores_normal(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, _repo, source, output, env_log = self._run_fake_target(root)
            self.assertEqual(code, 0)
            self.assertEqual(source.read_text(encoding="utf-8"), "before\n")
            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "pass")
            self.assertEqual(report["focused_control"]["test_evidence"], 1)
            self.assertEqual([row["status"] for row in report["mutants"]], ["killed", "killed"])
            self.assertTrue(all(row["log_evidence"]["assertion_failure"]
                                for row in report["mutants"]))
            self.assertEqual(len(report["baseline_tests"]), 1)
            self.assertEqual(report["normal_restore"]["status"], "passed")
            self.assertEqual(report["normal_restore"]["control"]["status"], "passed")
            preserved = sorted(path.name for path in (output / "artifacts").iterdir())
            self.assertEqual(preserved, [
                "first-fake_tests", "normal-baseline-fake_tests",
                "normal-restored-fake_tests", "second-fake_tests",
            ])
            self.assertEqual((output / "artifacts" / "first-fake_tests").read_text(), "mutant")
            self.assertEqual((output / "artifacts" / "second-fake_tests").read_text(), "mutant")
            self.assertEqual((output / "artifacts" / "normal-restored-fake_tests").read_text(), "normal")
            child_rows = [json.loads(line) for line in env_log.read_text().splitlines()]
            self.assertGreater(len(child_rows), 6)
            self.assertTrue(all(row["PATH"] == os.environ["PATH"] for row in child_rows))
            self.assertTrue(all(row["LC_ALL"] == "C.UTF-8" and row["KEEP_ENV"] == "preserved"
                                for row in child_rows))
            self.assertTrue(all(row["LD_LIBRARY_PATH"] == os.environ.get("LD_LIBRARY_PATH", "/usr/lib")
                                for row in child_rows))
            self.assertTrue(all(row["GIT_DIR"] is None and row["GIT_WORK_TREE"] is None
                                for row in child_rows))
            self.assertTrue(all(Path(row["TMPDIR"]).resolve().is_relative_to((output / "tmp").resolve())
                                and Path(row["LLVM_PROFILE_FILE"]).resolve().is_relative_to(
                                    (output / "profiles").resolve())
                                for row in child_rows))
            self.assertGreater(len({row["TMPDIR"] for row in child_rows}), 1)

    def test_failed_focused_control_stops_before_any_mutant_and_runs_restore_control(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            code, _repo, source, output, _env_log = self._run_fake_target(
                root, first_control_fails=True)
            self.assertEqual(code, 2)
            self.assertEqual(source.read_text(encoding="utf-8"), "before\n")
            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "focused_control_failed")
            self.assertEqual(report["focused_control"]["classification"], "killed")
            self.assertEqual(report["mutants"], [])
            self.assertEqual(report["normal_restore"]["status"], "passed")

    def test_normal_restore_build_failure_is_recorded_and_fails_the_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, source, output, args, child_env = self._fixture(root, failed_restore_build=True)

            @contextmanager
            def use_repo(*_args, **_kwargs):
                yield repo

            inherited = {**child_env, "PATH": os.environ["PATH"], "TMPDIR": str(root),
                         "TMP": str(root), "TEMP": str(root), "TEMPDIR": str(root)}
            with mock.patch.dict(os.environ, inherited), \
                    mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                    mock.patch.object(slicer_mutation, "_mutation_worktree_with_report", side_effect=use_repo):
                with self.assertRaisesRegex(slicer_mutation.MutationError, "normal target rebuild failed"):
                    slicer_mutation.run_mutation(args)
            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "normal_restore_failed")
            self.assertEqual(report["normal_restore"]["status"], "failed")
            self.assertEqual(source.read_text(encoding="utf-8"), "before\n")

    def test_keyboard_interrupt_runs_normal_restore_after_source_restoration(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            repo, source, output, args, child_env = self._fixture(root)

            @contextmanager
            def use_repo(*_args, **_kwargs):
                yield repo

            real_run_recorded = slicer_mutation.run_recorded
            label_runs = 0

            def interrupt_mutant(argv, **kwargs):
                nonlocal label_runs
                if "-L" in argv:
                    label_runs += 1
                    if label_runs == 2:
                        raise KeyboardInterrupt
                return real_run_recorded(argv, **kwargs)

            inherited = {**child_env, "PATH": os.environ["PATH"], "TMPDIR": str(root),
                         "TMP": str(root), "TEMP": str(root), "TEMPDIR": str(root)}
            with mock.patch.dict(os.environ, inherited), \
                    mock.patch.object(slicer_mutation, "validate_clean_repo", return_value="a" * 40), \
                    mock.patch.object(slicer_mutation, "_mutation_worktree_with_report", side_effect=use_repo), \
                    mock.patch.object(slicer_mutation, "run_recorded", side_effect=interrupt_mutant):
                with self.assertRaises(KeyboardInterrupt):
                    slicer_mutation.run_mutation(args)
            self.assertEqual(source.read_text(encoding="utf-8"), "before\n")
            report = json.loads((output / "mutation-report.json").read_text(encoding="utf-8"))
            self.assertEqual(report["normal_restore"]["status"], "passed")

class PersistentMutationWorktreeEdgeTests(unittest.TestCase):
    _git = PersistentMutationWorktreeTests._git
    _repo = PersistentMutationWorktreeTests._repo

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
