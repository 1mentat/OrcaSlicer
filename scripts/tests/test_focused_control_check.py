#!/usr/bin/env python3
"""Hermetic tests for the focused CTest control check (no cmake or ctest needed)."""

from pathlib import Path
from unittest import mock
import argparse
import contextlib
import io
import json
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import focused_control_check as fcc  # noqa: E402

PREFIX = "FocusedWipeTower::"
TARGET = "fff_print_wipe_tower_tests"
NAMES = [PREFIX + "alpha [one]", PREFIX + "beta, two", PREFIX + "gamma"]


def discovery(names, exe):
    return {"tests": [{"name": n, "command": [str(exe), "--test-case", n]} for n in names]}


def junit(path, names, failed=()):
    cases = "".join(
        '<testcase name="%s" classname="%s" status="run">%s</testcase>' % (
            n, n, "<failure message='f'/>" if n in failed else "")
        for n in names)
    path.write_text('<testsuite tests="%d" failures="%d" skipped="0" errors="0">%s</testsuite>' % (
        len(names), len(failed), cases), encoding="utf-8")


class Fixture:
    def __init__(self, test):
        scratch = tempfile.TemporaryDirectory()
        test.addCleanup(scratch.cleanup)
        self.root = Path(scratch.name)
        self.build = self.root / "build"
        self.exe = self.build / "tests" / "fff_print" / "Release" / TARGET
        self.exe.parent.mkdir(parents=True)
        self.exe.write_text("x")
        self.control = self.root / "control.json"
        self.control.write_text(json.dumps({"schema": 1, "names": NAMES}), encoding="utf-8")
        self.work = self.root / "work"
        self.work.mkdir()

    def args(self, **over):
        values = dict(root=str(self.root), build_dir=str(self.build), config="Release",
                      target=TARGET, prefix=PREFIX, control=str(self.control), jobs=2,
                      cmake="cmake", ctest="ctest", configure_timeout=5, build_timeout=5,
                      list_timeout=5, test_timeout=5, run_timeout=5)
        values.update(over)
        return argparse.Namespace(**values)


class EvaluateTests(unittest.TestCase):
    def setUp(self):
        self.exe = Path(tempfile.gettempdir()) / TARGET
        self.build = self.exe.parent

    def evaluate(self, data):
        with mock.patch.object(Path, "resolve", lambda self, strict=False: self):
            return fcc.evaluate(data, NAMES, PREFIX, self.build, self.exe)

    def test_positive(self):
        result = self.evaluate(discovery(NAMES, self.exe))
        self.assertEqual(result["problems"], [])
        self.assertEqual(sorted(result["selected"]), sorted(NAMES))

    def test_missing_name_is_reported_exactly(self):
        result = self.evaluate(discovery(NAMES[:2], self.exe))
        self.assertEqual(result["missing"], [NAMES[2]])
        self.assertTrue(any(NAMES[2] in p for p in result["problems"]))

    def test_duplicate_name(self):
        result = self.evaluate(discovery(NAMES + [NAMES[0]], self.exe))
        self.assertEqual(result["duplicates"], [NAMES[0]])
        self.assertTrue(result["problems"])

    def test_wrong_executable(self):
        data = discovery(NAMES, self.exe)
        data["tests"][1]["command"] = [str(self.build / "fff_print_tests")]
        result = self.evaluate(data)
        self.assertTrue(any("executable mismatch" in p for p in result["problems"]))

    def test_extra_names_are_tolerated(self):
        result = self.evaluate(discovery(NAMES + [PREFIX + "extra"], self.exe))
        self.assertEqual(result["problems"], [])
        self.assertEqual(result["extra"], [PREFIX + "extra"])

    def test_name_outside_prefix_fails(self):
        result = self.evaluate(discovery(NAMES + ["Other::x"], self.exe))
        self.assertTrue(result["problems"])

    def test_malformed_json(self):
        self.assertTrue(self.evaluate({"nope": 1})["problems"])


class CheckTests(unittest.TestCase):
    def run_check(self, fixture, discovered=None, failed=(), executed=None, ctest_code=0,
                  step_codes=None):
        discovered = NAMES if discovered is None else discovered
        executed = NAMES if executed is None else executed
        calls = []

        def fake_run(cmd, timeout, log, cwd):
            calls.append(list(cmd))
            if "--output-junit" in cmd:
                junit(Path(cmd[cmd.index("--output-junit") + 1]), executed, failed)
            return (step_codes or {}).get(len(calls), ctest_code if "-R" in cmd else 0)

        listing = subprocess.CompletedProcess([], 0, json.dumps(discovery(discovered, fixture.exe)), "")
        out = io.StringIO()
        with mock.patch.object(fcc, "run", fake_run), \
                mock.patch.object(fcc.slicer_process, "run_captured", return_value=listing), \
                contextlib.redirect_stdout(out):
            code, stats = fcc.check(fixture.args(), fixture.work)
        return code, stats, calls, out.getvalue()

    def test_positive(self):
        code, stats, calls, _ = self.run_check(Fixture(self))
        self.assertEqual(code, 0)
        self.assertEqual((stats["expected"], stats["discovered"], stats["selected"],
                          stats["executed"], stats["passed"], stats["failed"]), (3, 3, 3, 3, 3, 0))
        self.assertIn(fcc.OPTION, calls[0])
        self.assertEqual(calls[1][calls[1].index("--target") + 1], TARGET)

    def test_missing_name_fails_without_running(self):
        code, stats, calls, text = self.run_check(Fixture(self), discovered=NAMES[:2])
        self.assertEqual(code, 1)
        self.assertEqual(stats["missing"], 1)
        self.assertEqual(len(calls), 2)  # configure and build only
        self.assertIn(NAMES[2], text)

    def test_duplicate_fails(self):
        code, stats, _, _ = self.run_check(Fixture(self), discovered=NAMES + [NAMES[1]])
        self.assertEqual((code, stats["duplicates"]), (1, 1))

    def test_failed_test_fails(self):
        code, stats, _, _ = self.run_check(Fixture(self), failed=[NAMES[0]], ctest_code=8)
        self.assertEqual((code, stats["failed"]), (1, 1))

    def test_unexecuted_name_fails(self):
        code, stats, _, _ = self.run_check(Fixture(self), executed=NAMES[:2])
        self.assertEqual((code, stats["executed"]), (1, 2))

    def test_configure_failure_stops(self):
        code, _, calls, _ = self.run_check(Fixture(self), step_codes={1: 1})
        self.assertEqual((code, len(calls)), (1, 1))

    def test_missing_executable_fails(self):
        fixture = Fixture(self)
        fixture.exe.unlink()
        code, *_ = self.run_check(fixture)
        self.assertEqual(code, 1)


class RunTests(unittest.TestCase):
    def test_timeout_maps_to_124_and_logs(self):
        with tempfile.TemporaryDirectory() as directory:
            log = Path(directory) / "out.log"
            code = fcc.run([sys.executable, "-c", "import time; time.sleep(30)"], 1, log,
                           Path(directory))
            self.assertEqual(code, 124)
            self.assertIn("timed out", log.read_text())

    def test_unrunnable_command_is_127(self):
        with tempfile.TemporaryDirectory() as directory:
            code = fcc.run([str(Path(directory) / "missing-exe")], 5, Path(directory) / "l", Path(directory))
            self.assertEqual(code, 127)


if __name__ == "__main__":
    unittest.main()
