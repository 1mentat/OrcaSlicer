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

    @unittest.skipUnless(os.name == "posix", "self SIGINT is safe only on POSIX")
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


    def test_incomplete_cleanup_is_recorded_without_masking_original_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            timeout = subprocess.TimeoutExpired(["native-tool"], .5, output="partial")
            timeout.returncode = 1
            timeout.cleanup_incomplete = True
            timeout.add_note("Windows command cleanup incomplete: bounded drain expired")
            with patch.object(slicer_process, "run_captured", side_effect=timeout):
                with self.assertRaises(subprocess.TimeoutExpired) as caught:
                    run_recorded(["native-tool"], cwd=root, env=dict(os.environ), timeout=.5,
                                 log_path=root / "output.log", receipt_path=root / "receipt.json")
            self.assertIs(caught.exception, timeout)
            receipt = json.loads((root / "receipt.json").read_text())
            self.assertTrue(receipt["timed_out"])
            self.assertTrue(receipt["cleanup_incomplete"])
            self.assertEqual(receipt["returncode"], 1)
            self.assertEqual(receipt["cleanup_notes"], timeout.__notes__)


class PortableInterruptionTests(unittest.TestCase):
    def test_main_thread_interruption_stops_ready_grandchild_before_cleanup(self):
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "late-write"
            ready = Path(directory) / "grandchild-ready"
            child = ("from pathlib import Path; import sys,time; "
                     "Path(sys.argv[2]).write_text('ready'); "
                     "time.sleep(0.9); Path(sys.argv[1]).write_text('still running')")
            parent = ("import subprocess,sys,time; "
                      "subprocess.Popen([sys.executable, '-c', sys.argv[1], sys.argv[2], sys.argv[3]]); "
                      "time.sleep(10)")
            communicate = subprocess.Popen.communicate
            injected = False
            ready_observed = False

            def interrupt_once(process, *args, **kwargs):
                nonlocal injected, ready_observed
                if not injected:
                    injected = True
                    self.assertIs(threading.current_thread(), threading.main_thread())
                    deadline = time.monotonic() + 5
                    while not ready.exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    ready_observed = ready.exists()
                    self.assertTrue(ready_observed, "grandchild did not become ready before interruption")
                    raise KeyboardInterrupt("portable main-thread interruption")
                return communicate(process, *args, **kwargs)

            with patch.object(subprocess.Popen, "communicate", new=interrupt_once):
                with self.assertRaises(KeyboardInterrupt):
                    run_captured([sys.executable, "-c", parent, child, str(marker), str(ready)], timeout=5)
            self.assertTrue(injected)
            self.assertTrue(ready_observed)
            time.sleep(1.1)
            self.assertFalse(marker.exists(), "an interrupted descendant survived process cleanup")


@unittest.skipUnless(os.name == "nt", "Windows Job Object ownership")
class WindowsJobOwnershipTests(unittest.TestCase):
    def run_tree_control(self, *, output_file=False, interrupt=False, parent_exits=False,
                         binary=False):
        import ctypes
        from ctypes import wintypes
        import _slicer_windows_job as jobs

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            ready, marker = root / "ready", root / "late-write"
            child_pid = root / "child-pid"
            child = ("from pathlib import Path; import os,sys,time; "
                     "Path(sys.argv[3]).write_text(str(os.getpid())); "
                     "print('child ready', flush=True); print('child diagnostic', file=sys.stderr, flush=True); "
                     "Path(sys.argv[2]).write_text('ready'); time.sleep(2); "
                     "Path(sys.argv[1]).write_text('survived')")
            parent = ("import subprocess,sys,time; from pathlib import Path; "
                      "subprocess.Popen([sys.executable,'-c',sys.argv[1],*sys.argv[2:5]]); "
                      "deadline=time.monotonic()+5; "
                      "exec('while not Path(sys.argv[3]).exists() and time.monotonic()<deadline: time.sleep(.01)'); "
                      "time.sleep(.3 if sys.argv[5]=='exit' else 10)")
            command = [sys.executable, "-c", parent, child, str(marker), str(ready),
                       str(child_pid), "exit" if parent_exits else "wait"]
            api = jobs._Api()
            api.kernel.IsProcessInJob.argtypes = [wintypes.HANDLE, wintypes.HANDLE,
                                                  ctypes.POINTER(wintypes.BOOL)]
            api.kernel.IsProcessInJob.restype = wintypes.BOOL
            original_assign = jobs._Api.assign
            original_communicate = subprocess.Popen.communicate
            child_handle = None
            injected = False

            def checked_assign(instance, job, broker):
                original_assign(instance, job, broker)
                handle = instance.checked("OpenProcess", 0x100000 | 0x1000, False, broker.pid)
                try:
                    belongs = wintypes.BOOL()
                    instance.checked("IsProcessInJob", handle, job, ctypes.byref(belongs))
                    self.assertTrue(belongs.value, "broker was not assigned before its gate opened")
                finally:
                    instance.checked("CloseHandle", handle)

            def observe_ready(process, *args, **kwargs):
                nonlocal child_handle, injected
                if child_handle is None:
                    deadline = time.monotonic() + 5
                    while not ready.exists() and time.monotonic() < deadline:
                        time.sleep(.01)
                    self.assertTrue(ready.exists(), "real grandchild never became ready")
                    child_handle = api.checked("OpenProcess", 0x100000, False,
                                               int(child_pid.read_text()))
                    if interrupt:
                        injected = True
                        raise KeyboardInterrupt("Windows owned-tree interruption")
                return original_communicate(process, *args, **kwargs)

            started = time.monotonic()
            try:
                with (root / "output.log").open("w+b") as output, \
                        patch.object(jobs._Api, "assign", new=checked_assign), \
                        patch.object(subprocess.Popen, "communicate", new=observe_ready):
                    if interrupt:
                        with self.assertRaises(KeyboardInterrupt):
                            run_captured(command, timeout=5, output_file=output if output_file else None)
                        self.assertTrue(injected)
                    elif parent_exits:
                        result = run_captured(command, timeout=5)
                        self.assertEqual(result.returncode, 0)
                        self.assertIn("child ready", result.stdout)
                    else:
                        with self.assertRaises(subprocess.TimeoutExpired) as caught:
                            run_captured(command, timeout=.5,
                                         input_data=b"" if binary else None,
                                         output_file=output if output_file else None)
                        error = caught.exception
                        self.assertEqual(error.cmd, command)
                        self.assertEqual(error.timeout, .5)
                        self.assertIsInstance(error.returncode, int)
                        if output_file:
                            output.flush()
                            output.seek(0)
                            self.assertIn(b"child ready", output.read())
                            self.assertIsNone(error.output)
                        else:
                            self.assertIn(b"child ready" if binary else "child ready", error.output)
                            self.assertIn(b"child diagnostic" if binary else "child diagnostic", error.stderr)
                self.assertLess(time.monotonic() - started, 8)
                self.assertIsNotNone(child_handle)
                self.assertEqual(api.kernel.WaitForSingleObject(child_handle, 1000), 0,
                                 "owned grandchild process did not exit")
                time.sleep(2.1)
                self.assertFalse(marker.exists(), "owned grandchild wrote after cleanup")
            finally:
                if child_handle is not None:
                    api.checked("CloseHandle", child_handle)

    def test_timeout_closes_real_descendant_text_pipes(self):
        self.run_tree_control()

    def test_timeout_closes_real_descendant_binary_pipes(self):
        self.run_tree_control(binary=True)

    def test_timeout_closes_real_descendant_output_file(self):
        self.run_tree_control(output_file=True)

    def test_interruption_closes_real_descendant_pipes(self):
        self.run_tree_control(interrupt=True)

    def test_interruption_closes_real_descendant_output_file(self):
        self.run_tree_control(interrupt=True, output_file=True)

    def test_normal_parent_exit_still_stops_pipe_holding_descendant(self):
        self.run_tree_control(parent_exits=True)

    def test_assignment_failure_never_starts_requested_target(self):
        import _slicer_windows_job as jobs
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "must-not-start"
            command = [sys.executable, "-c", "from pathlib import Path; import sys; Path(sys.argv[1]).touch()", str(marker)]
            started = time.monotonic()
            with patch.object(jobs._Api, "assign", side_effect=OSError("assignment denied")):
                with self.assertRaisesRegex(OSError, "assignment denied"):
                    run_captured(command, timeout=3)
            self.assertLess(time.monotonic() - started, 7)
            self.assertFalse(marker.exists())

    def test_gate_signal_failure_never_starts_requested_target(self):
        import _slicer_windows_job as jobs
        original_checked = jobs._Api.checked
        def fail_signal(instance, name, *args):
            if name == "SetEvent":
                raise OSError("gate signal denied")
            return original_checked(instance, name, *args)
        with tempfile.TemporaryDirectory() as directory:
            marker = Path(directory) / "must-not-start"
            command = [sys.executable, "-c", "from pathlib import Path; import sys; Path(sys.argv[1]).touch()", str(marker)]
            with patch.object(jobs._Api, "checked", new=fail_signal):
                with self.assertRaisesRegex(OSError, "gate signal denied"):
                    run_captured(command, timeout=3)
            self.assertFalse(marker.exists())


if __name__ == "__main__":
    unittest.main()
