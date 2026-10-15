"""Native-runner process lifetime contracts, using real child processes."""

from pathlib import Path
import os
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from slicer_process import run_captured


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


if __name__ == "__main__":
    unittest.main()
