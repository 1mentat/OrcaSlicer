"""Exercise disposal and enforced cancellation using actual child commands."""

import contextlib
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPTS))
import validation_artifacts as artifacts
import validation_runner as runner


class ManagedRunTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name).resolve() / "artifacts"
        artifacts.init(self.root, 2_000_000)

    def arguments(self, command, extra=()):
        return ["--root", str(self.root), "--work-item", "PR-610", "--source-sha", "a" * 40,
                "--owner", "tests", "--ttl-seconds", "30", "--budget-bytes", "100000",
                *extra, "--", sys.executable, "-c", command]

    def invoke(self, command, extra=()):
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(io.StringIO()):
            code = runner.main(self.arguments(command, extra))
        return code, json.loads(output.getvalue())

    def assert_disposed(self):
        for directory in (self.root / "runs").iterdir():
            record = json.loads((directory / "run.json").read_text())
            if record["state"] != "retained":
                self.assertFalse((directory / "payload").exists())

    def test_success_and_failure_discard_unpromoted_data(self):
        for exit_code in (0, 3):
            code, receipt = self.invoke(
                "import os,pathlib; pathlib.Path(os.environ['VALIDATION_ARTIFACT_DIR'],"
                f"'intermediate.o').write_bytes(b'object'); raise SystemExit({exit_code})")
            self.assertEqual(code, exit_code)
            self.assertEqual(receipt["outcome"], "completed" if exit_code == 0 else "failed")
            self.assertNotIn("token", json.dumps(receipt))
            self.assert_disposed()

    def test_explicit_promotion_keeps_only_selected_small_report(self):
        code, receipt = self.invoke(
            "import os,pathlib; p=pathlib.Path(os.environ['VALIDATION_ARTIFACT_DIR']);"
            "(p/'report.json').write_text('{}'); (p/'intermediate.a').write_bytes(b'archive')",
            ["--promote", "report.json", "--retain-ttl-seconds", "60",
             "--retain-budget-bytes", "100"])
        self.assertEqual(code, 0)
        retained = Path(receipt["retained"]["payload_dir"])
        self.assertEqual([p.name for p in retained.iterdir()], ["report.json"])
        self.assert_disposed()

    def test_budget_failure_terminates_command_and_discards_payload(self):
        started = time.monotonic()
        code, receipt = self.invoke(
            "import os,pathlib,time; p=pathlib.Path(os.environ['VALIDATION_ARTIFACT_DIR']);"
            "(p/'too-large.bin').write_bytes(b'x'*200000); time.sleep(20)")
        self.assertEqual(code, 2)
        self.assertEqual(receipt["outcome"], "failed")
        self.assertLess(time.monotonic() - started, 10)
        self.assert_disposed()

    def test_fast_command_cannot_bypass_final_budget_check(self):
        code, receipt = self.invoke(
            "import os,pathlib; pathlib.Path(os.environ['VALIDATION_ARTIFACT_DIR'],"
            "'too-large.bin').write_bytes(b'x'*200000)")
        self.assertEqual(code, 2)
        self.assertEqual(receipt["outcome"], "failed")
        self.assert_disposed()

    @unittest.skipIf(os.name == "nt", "POSIX SIGTERM; Windows cancellation covered by helper tests")
    def test_sigterm_disposes_run(self):
        process = subprocess.Popen([sys.executable, str(SCRIPTS / "validation_runner.py"),
                                    *self.arguments("import time; time.sleep(20)")],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        deadline = time.monotonic() + 10
        while not list((self.root / "runs").glob("*/payload/command.log")):
            if process.poll() is not None or time.monotonic() > deadline:
                self.fail("run did not start")
            time.sleep(0.02)
        process.send_signal(signal.SIGTERM)
        stdout, stderr = process.communicate(timeout=10)
        self.assertEqual(process.returncode, 130, stderr)
        self.assertEqual(json.loads(stdout)["outcome"], "interrupted")
        self.assert_disposed()

    def test_writer_refuses_unmanaged_or_escaping_output(self):
        with patch.dict(os.environ, {}, clear=True), contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(runner.managed_main(lambda args: self.fail("writer ran"),
                                                output_option="--out", default_name="result"), 2)
        record = artifacts.create(self.root, "PR-610", "a" * 40, "tests", 60, 10000)
        environment = {"VALIDATION_ARTIFACT_ROOT": str(self.root),
                       "VALIDATION_ARTIFACT_RUN_ID": record["run_id"],
                       "VALIDATION_ARTIFACT_TOKEN": record["token"],
                       "VALIDATION_ARTIFACT_DIR": record["payload_dir"],
                       "VALIDATION_SOURCE_SHA": "a" * 40}
        with patch.dict(os.environ, environment), patch.object(subprocess, "check_output", return_value="a" * 40), \
                contextlib.redirect_stderr(io.StringIO()):
            self.assertEqual(runner.managed_main(lambda args: self.fail("writer ran"),
                                                output_option="--out", default_name="result",
                                                argv=["--out", "../escape"]), 2)
        artifacts.finish(self.root, record["run_id"], record["token"], "completed")

    def test_producer_parsers_reject_abbreviated_output_override(self):
        import settings_inventory
        import slicer_mutation
        import slicer_pr_validation
        import slicer_module_feedback
        for parser in (settings_inventory._parser(), slicer_mutation._parser(),
                       slicer_pr_validation._parser()):
            self.assertFalse(parser.allow_abbrev)
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            settings_inventory._parser().parse_args(["--out", "safe", "--ou", "/outside"])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            slicer_mutation._parser().parse_args(["run", "--out", "safe", "--ou", "/outside"])
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            slicer_module_feedback.parse_args(["--ou", "/outside"])


if __name__ == "__main__":
    unittest.main()
