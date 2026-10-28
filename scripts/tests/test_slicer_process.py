"""Native-runner process lifetime contracts, using real child processes."""

import json
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_process
from slicer_process import run_captured, run_recorded


class CapturedProcessTests(unittest.TestCase):
    def test_captures_text_input_and_both_output_streams(self):
        result = run_captured(
            [sys.executable, "-c", "import sys; print(sys.stdin.read()); print('diagnostic', file=sys.stderr)"],
            input_data="mesh Δ", timeout=5,
        )
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout, "mesh Δ\n")
        self.assertEqual(result.stderr, "diagnostic\n")

    def test_preserves_binary_input_and_nonzero_exit(self):
        result = run_captured(
            [sys.executable, "-c", "import sys; sys.stdout.buffer.write(sys.stdin.buffer.read()); sys.exit(3)"],
            input_data=b"\x00\xff patch\n", timeout=5,
        )
        self.assertEqual(result.returncode, 3)
        self.assertEqual(result.stdout, b"\x00\xff patch\n")

    def test_timeout_stops_descendant_before_worktree_can_change(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "late-write"
            child = ("from pathlib import Path; import sys,time; "
                     "print('child ready', flush=True); time.sleep(1.2); "
                     "Path(sys.argv[1]).write_text('still running')")
            parent = ("import subprocess,sys,time; "
                      "subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]]); "
                      "time.sleep(10)")
            with self.assertRaises(subprocess.TimeoutExpired) as caught:
                run_captured([sys.executable, "-c", parent, child, str(marker)], timeout=0.5)
            self.assertIn("child ready", caught.exception.output)
            time.sleep(1.3)
            self.assertFalse(marker.exists(), "a timed-out child continued writing into the worktree")

    def test_explicit_working_directory_and_environment_reach_the_child(self):
        with tempfile.TemporaryDirectory() as directory:
            environment = dict(os.environ, ORCA_PROCESS_TEST="scoped")
            result = run_captured(
                [sys.executable, "-c", "import os; print(os.getcwd()); print(os.environ['ORCA_PROCESS_TEST'])"],
                cwd=directory, env=environment, timeout=5,
            )
            lines = result.stdout.splitlines()
            self.assertEqual(Path(lines[0]).resolve(), Path(directory).resolve())
            self.assertEqual(lines[1], "scoped")

    def test_keyboard_interrupt_stops_grandchild_before_worktree_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "late-write"
            child = ("from pathlib import Path; import sys,time; "
                     "time.sleep(0.9); Path(sys.argv[1]).write_text('still running')")
            parent = ("import subprocess,sys,time; "
                      "subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]]); "
                      "time.sleep(10)")
            timer = threading.Timer(0.2, os.kill, args=(os.getpid(), signal.SIGINT))
            timer.start()
            try:
                with self.assertRaises(KeyboardInterrupt):
                    run_captured([sys.executable, "-c", parent, child, str(marker)], timeout=5)
            finally:
                timer.cancel()
            time.sleep(1.1)
            self.assertFalse(marker.exists(), "an interrupted descendant survived process cleanup")


class RecordedProcessTests(unittest.TestCase):
    def test_receipt_precedes_later_assessment_and_keeps_exit_42_as_process_outcome(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "combined.log"
            receipt_path = root / "command.json"
            environment = dict(os.environ, ORCA_RECORDED_TEST="explicit child environment")
            inherited_environment = dict(os.environ)
            command = (
                "import os,sys; print(os.getcwd()); "
                "print(os.environ['ORCA_RECORDED_TEST']); "
                "print('diagnostic', file=sys.stderr); sys.exit(42)"
            )
            result = run_recorded(
                [sys.executable, "-c", command], cwd=root, env=environment, timeout=5,
                log_path=log, receipt_path=receipt_path,
            )

            self.assertEqual(result.returncode, 42)
            self.assertIsNone(result.stdout)
            self.assertIsNone(result.stderr)
            evidence = json.loads(receipt_path.read_text(encoding="utf-8"))
            self.assertEqual(evidence["schema"], "slicer-recorded-command-v1")
            self.assertEqual(evidence["process_outcome"], "exited")
            self.assertEqual(evidence["returncode"], 42)
            self.assertFalse(evidence["timed_out"])
            self.assertNotIn("accepted", evidence)
            self.assertNotIn("env", evidence, "receipt must not persist environment values")
            self.assertEqual(evidence["log_path"], str(log))
            self.assertEqual(Path(evidence["cwd"]), root)
            combined = log.read_text(encoding="utf-8")
            self.assertIn(str(root), combined)
            self.assertIn("explicit child environment", combined)
            self.assertIn("diagnostic", combined)

            with self.assertRaisesRegex(RuntimeError, "later JUnit assessment"):
                raise RuntimeError("later JUnit assessment failed")
            self.assertEqual(json.loads(receipt_path.read_text(encoding="utf-8")), evidence)
            self.assertEqual(dict(os.environ), inherited_environment)
            self.assertEqual(list(root.glob("*.tmp")), [])

    def test_launch_error_is_recorded_and_original_exception_is_reraised(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            missing = root / "does-not-exist"
            receipt_path = root / "command.json"
            with self.assertRaises(FileNotFoundError):
                run_recorded(
                    [str(missing)], cwd=root, env=dict(os.environ), timeout=2,
                    log_path=root / "combined.log", receipt_path=receipt_path,
                )
            evidence = json.loads(receipt_path.read_text(encoding="utf-8"))
            self.assertEqual(evidence["process_outcome"], "execution_error")
            self.assertIsNone(evidence["returncode"])
            self.assertFalse(evidence["timed_out"])
            self.assertEqual(evidence["exception"]["type"], "FileNotFoundError")
            self.assertEqual(evidence["stage"], "run_captured")

    def test_interruption_is_recorded_and_original_exception_is_reraised(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            interruption = KeyboardInterrupt("cancel native command")
            with patch.object(slicer_process, "run_captured", side_effect=interruption):
                with self.assertRaises(KeyboardInterrupt) as caught:
                    run_recorded(
                        [sys.executable, "-c", "pass"], cwd=root, env=dict(os.environ), timeout=2,
                        log_path=root / "combined.log", receipt_path=root / "command.json",
                    )
            self.assertIs(caught.exception, interruption)
            evidence = json.loads((root / "command.json").read_text(encoding="utf-8"))
            self.assertEqual(evidence["process_outcome"], "interrupted")
            self.assertIsNone(evidence["returncode"])
            self.assertEqual(evidence["exception"]["type"], "KeyboardInterrupt")

    def test_existing_receipt_is_never_replaced_or_command_started(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            log = root / "combined.log"
            receipt_path = root / "command.json"
            receipt_path.write_text("previous evidence\n", encoding="utf-8")
            with self.assertRaises(FileExistsError):
                run_recorded(
                    [sys.executable, "-c", "raise SystemExit(42)"],
                    cwd=root, env=dict(os.environ), timeout=2,
                    log_path=log, receipt_path=receipt_path,
                )
            self.assertEqual(receipt_path.read_text(encoding="utf-8"), "previous evidence\n")
            self.assertFalse(log.exists())

    def test_timeout_receipt_keeps_descendant_cleanup_and_partial_log(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            marker = root / "late-write"
            log = root / "combined.log"
            receipt_path = root / "command.json"
            child = ("from pathlib import Path; import sys,time; "
                     "print('child ready', flush=True); time.sleep(1.0); "
                     "Path(sys.argv[1]).write_text('still running')")
            parent = ("import subprocess,sys,time; "
                      "subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2]]); "
                      "time.sleep(10)")
            with self.assertRaises(subprocess.TimeoutExpired):
                run_recorded(
                    [sys.executable, "-c", parent, child, str(marker)],
                    cwd=root, env=dict(os.environ), timeout=0.5,
                    log_path=log, receipt_path=receipt_path,
                )

            evidence = json.loads(receipt_path.read_text(encoding="utf-8"))
            self.assertEqual(evidence["process_outcome"], "timed_out")
            self.assertTrue(evidence["timed_out"])
            if os.name == "posix":
                self.assertEqual(evidence["returncode"], -signal.SIGKILL)
            else:
                self.assertIsInstance(evidence["returncode"], int)
            self.assertIn("child ready", log.read_text(encoding="utf-8"))
            time.sleep(1.1)
            self.assertFalse(marker.exists(), "a timed-out descendant continued writing into the worktree")

    def test_receipt_write_error_does_not_mask_interruption(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            interruption = KeyboardInterrupt("cancel native command")
            with patch.object(slicer_process, "run_captured", side_effect=interruption), \
                    patch.object(slicer_process, "_write_json_receipt", side_effect=OSError("disk full")):
                with self.assertRaises(KeyboardInterrupt) as caught:
                    run_recorded(
                        [sys.executable, "-c", "pass"], cwd=root, env=dict(os.environ), timeout=2,
                        log_path=root / "combined.log", receipt_path=root / "command.json",
                    )
            self.assertIs(caught.exception, interruption)


if __name__ == "__main__":
    unittest.main()
