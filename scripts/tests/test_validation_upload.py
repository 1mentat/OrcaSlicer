"""Owned temporary files and synthetic Git/manager contexts; no uploads/network."""

import contextlib
from datetime import datetime
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import validation_artifacts as artifacts
import validation_upload as upload


class UploadPreflightTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve()
        self.workspace = self.root / "checkout"
        self.scratch = self.root / "runner-temp"
        self.workspace.mkdir(); self.scratch.mkdir()
        self.template = self.root / "git-template"
        (self.template / "info").mkdir(parents=True)
        (self.template / "info/exclude").write_text("")
        env = dict(os.environ, GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull,
                   GIT_TEMPLATE_DIR=str(self.template))
        subprocess.run(["git", "init", "-q", str(self.workspace)], env=env, check=True,
                       capture_output=True)
        subprocess.run(["git", "-C", str(self.workspace), "-c", "user.name=Upload test",
                        "-c", "user.email=upload@example.invalid", "-c", "core.hooksPath=" + str(self.template),
                        "commit", "-q", "--allow-empty", "-m", "owned fixture"],
                       env=env, check=True, capture_output=True)
        self.head = upload._head(self.workspace)
        self.env = {"GITHUB_WORKSPACE": str(self.workspace), "RUNNER_TEMP": str(self.scratch),
                    "RUNNER_ENVIRONMENT": "github-hosted", "GITHUB_ACTOR": "fixture-owner",
                    "GITHUB_REPOSITORY": "owner/repository", "GITHUB_RUN_ID": "123",
                    "GITHUB_RUN_ATTEMPT": "2", "GITHUB_JOB": "unit-test",
                    "VALIDATION_UPLOAD_NAME": "fixture reports", "VALIDATION_UPLOAD_PATH": "output"}

    def file(self, relative, data=b"abc"):
        path = self.workspace / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)
        return path

    def invoke(self, **extra):
        return upload.preflight(dict(self.env, **extra))

    def link(self, path, target, directory=False):
        try:
            path.symlink_to(target, target_is_directory=directory)
        except (OSError, NotImplementedError) as error:
            self.skipTest(f"owned symlink fixture unsupported: {error}")

    def manager(self):
        root = self.root / "managed"
        artifacts.init(root, 100000)
        receipt = artifacts.create(root, "fixture", self.head, "fixture-owner", 60, 10000)
        self.env.update(RUNNER_ENVIRONMENT="self-hosted", VALIDATION_ARTIFACT_ROOT=str(root),
                        VALIDATION_ARTIFACT_RUN_ID=receipt["run_id"],
                        VALIDATION_ARTIFACT_TOKEN=receipt["token"],
                        VALIDATION_ARTIFACT_DIR=receipt["payload_dir"], VALIDATION_SOURCE_SHA=self.head)
        return receipt

    def test_directory_actual_bytes_identity_and_expiry(self):
        self.file("output/a", b"123"); self.file("output/sub/b", b"12345")
        before = datetime.now().astimezone()
        receipt = self.invoke(GITHUB_SHA="f" * 40)
        self.assertEqual((receipt["selected_files"], receipt["selected_bytes"]), (2, 8))
        self.assertEqual(receipt["source_sha"], self.head)
        self.assertEqual(receipt["owner"], "fixture-owner")
        self.assertEqual(receipt["work_item"], "owner/repository/runs/123/attempts/2/jobs/unit-test")
        delta = datetime.fromisoformat(receipt["expires_at"]) - datetime.fromisoformat(receipt["checked_at"])
        self.assertEqual(delta.total_seconds(), 14 * 86400)
        self.assertGreaterEqual(datetime.fromisoformat(receipt["checked_at"]), before)
        self.assertIn("not locked", receipt["budget_enforcement"])

    def test_multiline_globs_zero_depth_directory_and_exclusion(self):
        self.file("output/a.txt"); self.file("output/sub/b.txt"); self.file("output/omit.txt")
        self.file("output/sub/c.bin"); self.file("other/selected", b"xy")
        receipt = self.invoke(VALIDATION_UPLOAD_PATH="output/**/*.txt\n!output/omit.txt\nother\noutput/a.txt")
        self.assertEqual((receipt["selected_files"], receipt["selected_bytes"]), (3, 8))

    def test_nonrecursive_glob_does_not_follow_unselected_link(self):
        self.file("output/selected.txt")
        outside = self.root / "outside"; outside.mkdir()
        self.link(self.workspace / "output/unselected", outside, True)
        receipt = self.invoke(VALIDATION_UPLOAD_PATH="output/*.txt")
        self.assertEqual(receipt["selected_files"], 1)

    def test_glob_matching_directory_includes_its_descendants(self):
        self.file("output/report.txt/inside.bin", b"1234")
        self.assertEqual(self.invoke(VALIDATION_UPLOAD_PATH="output/*.txt")["selected_bytes"], 4)

    def test_actual_oversize_fails_before_upload(self):
        self.file("output/a", b"1234")
        with self.assertRaisesRegex(upload.UploadError, "actual selected"):
            self.invoke(VALIDATION_UPLOAD_MAX_BYTES="3")

    def test_budget_and_retention_are_positive_bounded_finite_integers(self):
        for name, values in [("VALIDATION_UPLOAD_MAX_BYTES", ["0", "-1", "4294967297", "inf", "1.5"]),
                             ("VALIDATION_UPLOAD_RETENTION_DAYS", ["0", "31", "-1", "NaN", "1.5"])]:
            for value in values:
                with self.subTest(name=name, value=value), self.assertRaises(upload.UploadError):
                    self.invoke(**{name: value})

    def test_hidden_selection_and_exact_boundary(self):
        self.file("output/.secret", b"123"); self.file("output/plain", b"12")
        self.assertEqual(self.invoke(VALIDATION_UPLOAD_MAX_BYTES="2")["selected_bytes"], 2)
        self.assertEqual(self.invoke(VALIDATION_UPLOAD_INCLUDE_HIDDEN_FILES="true",
                                     VALIDATION_UPLOAD_MAX_BYTES="5")["selected_bytes"], 5)

    def test_empty_warn_ignore_and_error(self):
        for policy in ["warn", "ignore"]:
            self.assertEqual(self.invoke(VALIDATION_UPLOAD_IF_NO_FILES_FOUND=policy)["selected_files"], 0)
        with self.assertRaisesRegex(upload.UploadError, "no upload files"):
            self.invoke(VALIDATION_UPLOAD_IF_NO_FILES_FOUND="error")

    def test_workspace_and_runner_temp_absolute_paths_allowed(self):
        self.file("output/a")
        (self.scratch / "report").write_bytes(b"1234")
        self.assertEqual(self.invoke(VALIDATION_UPLOAD_PATH=str(self.scratch / "report"))["selected_bytes"], 4)
        self.assertEqual(self.invoke(VALIDATION_UPLOAD_PATH=str(self.workspace / "output"))["selected_files"], 1)

    def test_parent_and_absolute_escapes_rejected_even_when_missing(self):
        for path in ["../outside", str(self.root / "outside-missing"), "output/../../outside"]:
            with self.subTest(path=path), self.assertRaises(upload.UploadError):
                self.invoke(VALIDATION_UPLOAD_PATH=path)

    def test_symlink_escape_is_rejected(self):
        outside = self.root / "outside"; outside.write_bytes(b"secret")
        self.workspace.joinpath("output").mkdir()
        self.link(self.workspace / "output/link", outside)
        with self.assertRaisesRegex(upload.UploadError, "escapes"):
            self.invoke()

    def test_symlink_cycle_rejected_and_contained_alias_counted(self):
        self.file("output/a", b"123")
        self.link(self.workspace / "output/alias", self.workspace / "output/a")
        self.assertEqual(self.invoke()["selected_bytes"], 6)
        self.link(self.workspace / "output/cycle", self.workspace / "output", True)
        with self.assertRaisesRegex(upload.UploadError, "cycle"):
            self.invoke()

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX FIFO unsupported")
    def test_special_file_rejected_without_opening(self):
        self.workspace.joinpath("output").mkdir()
        os.mkfifo(self.workspace / "output/pipe")
        with self.assertRaisesRegex(upload.UploadError, "special file"):
            self.invoke()

    def test_unknown_runner_and_selfhost_without_context_fail_closed(self):
        for value in ["", "self-hosted", "unknown"]:
            with self.subTest(value=value), self.assertRaisesRegex(upload.UploadError, "authenticated"):
                self.invoke(RUNNER_ENVIRONMENT=value)

    def test_authenticated_manager_payload_only_and_wrong_source(self):
        receipt = self.manager()
        payload = Path(receipt["payload_dir"])
        (payload / "report").write_bytes(b"123")
        result = self.invoke(VALIDATION_UPLOAD_PATH=str(payload / "report"))
        self.assertEqual(result["selected_bytes"], 3)
        self.assertEqual(result["local_disposal"], "authenticated artifact manager lease")
        self.file("output/a")
        with self.assertRaises(upload.UploadError):
            self.invoke()
        with self.assertRaisesRegex(upload.UploadError, "source ownership"):
            self.invoke(VALIDATION_UPLOAD_PATH=str(payload / "report"), VALIDATION_SOURCE_SHA="f" * 40)

    def test_expired_manager_and_invalid_capability_fail_closed(self):
        receipt = self.manager()
        path = str(Path(receipt["payload_dir"]) / "report")
        Path(path).write_bytes(b"abc")
        with self.assertRaises(artifacts.ArtifactError):
            self.invoke(VALIDATION_UPLOAD_PATH=path, VALIDATION_ARTIFACT_TOKEN="wrong")
        with patch.object(artifacts.time, "time", return_value=time.time() + 120):
            with self.assertRaises(artifacts.ArtifactError):
                self.invoke(VALIDATION_UPLOAD_PATH=path)

    def test_cli_manager_failure_is_fail_closed_without_traceback(self):
        receipt = self.manager()
        path = str(Path(receipt["payload_dir"]) / "report")
        Path(path).write_bytes(b"abc")
        env = dict(self.env, VALIDATION_UPLOAD_PATH=path, VALIDATION_ARTIFACT_TOKEN="wrong")
        error = io.StringIO()
        with patch.dict(os.environ, env, clear=True), contextlib.redirect_stderr(error):
            self.assertEqual(upload.main(), 2)
        self.assertIn("Invalid capability", error.getvalue())
        self.assertNotIn("Traceback", error.getvalue())

    def test_ci_owner_is_required_and_spoofed_sha_not_used(self):
        self.file("output/a")
        with self.assertRaisesRegex(upload.UploadError, "owner"):
            self.invoke(GITHUB_ACTOR="")
        self.assertEqual(self.invoke(GITHUB_SHA="f" * 40)["source_sha"], self.head)

    def test_invalid_action_inputs_fail_before_selection(self):
        for name, value in [("VALIDATION_UPLOAD_COMPRESSION_LEVEL", "10"),
                            ("VALIDATION_UPLOAD_OVERWRITE", "yes"),
                            ("VALIDATION_UPLOAD_INCLUDE_HIDDEN_FILES", "yes"),
                            ("VALIDATION_UPLOAD_IF_NO_FILES_FOUND", "maybe")]:
            with self.subTest(name=name), self.assertRaises(upload.UploadError):
                self.invoke(**{name: value})

    def test_main_only_emits_small_json_and_runner_summary(self):
        self.file("output/a")
        summary = self.scratch / "summary"
        stdout = io.StringIO()
        with patch.dict(os.environ, dict(self.env, GITHUB_STEP_SUMMARY=str(summary)), clear=True), \
                contextlib.redirect_stdout(stdout):
            self.assertEqual(upload.main(), 0)
        receipt = json.loads(stdout.getvalue())
        self.assertNotIn("paths", receipt)
        self.assertIn(receipt["source_sha"], summary.read_text())
        self.assertEqual(list(self.workspace.joinpath("output").iterdir()), [self.workspace / "output/a"])

    def test_summary_symlink_escape_fails_without_mutation(self):
        self.file("output/a")
        outside = self.root / "outside"; outside.write_text("unchanged")
        self.link(self.scratch / "summary", outside)
        with patch.dict(os.environ, dict(self.env, GITHUB_STEP_SUMMARY=str(self.scratch / "summary")), clear=True), \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(upload.main(), 2)
        self.assertEqual(outside.read_text(), "unchanged")


if __name__ == "__main__":
    unittest.main()
