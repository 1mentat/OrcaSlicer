#!/usr/bin/env python3
"""Tests for the focused test-target link inventory."""

from pathlib import Path
from unittest import mock
import io
import contextlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import focused_targets  # noqa: E402


class FocusedTargetInventoryTests(unittest.TestCase):
    def test_inventory_equals_cmake_declared_set(self):
        self.assertEqual(focused_targets.check(), [])
        self.assertEqual(len(focused_targets.load_inventory()), 52)

    def test_conditional_targets_are_marked_optional(self):
        declared = focused_targets.cmake_targets()
        inventory = focused_targets.load_inventory()
        self.assertEqual({n for n, c in declared.items() if c},
                         {n for n, r in inventory.items() if r})

    def test_pre_existing_helper_is_not_focused(self):
        self.assertNotIn("slicer_settings_inventory", focused_targets.cmake_targets())

    def test_drift_is_reported(self):
        with tempfile.TemporaryDirectory() as directory:
            inventory = Path(directory) / "inv.txt"
            names = [n for n in focused_targets.load_inventory() if n != "sla_pipeline_tests"]
            inventory.write_text("\n".join(names + ["ghost_tests"]) + "\n")
            problems = focused_targets.check(inventory=inventory)
        self.assertIn("in CMake, not inventory: sla_pipeline_tests", problems)
        self.assertIn("in inventory, not CMake: ghost_tests", problems)

    def test_build_reports_failures_and_skips_without_hiding(self):
        inventory = {"a_tests": None, "b_tests": None, "cad_tests": "cad", "gone_tests": None}
        calls = []

        def fake_run(cmd, timeout):
            calls.append(cmd)
            if cmd[-1] == "help":
                return 0, "... a_tests\n... b_tests\n"
            if "--target" in cmd and cmd[cmd.index("--target") + 1] == "b_tests":
                return 1, "link error"
            return 0, ""

        with tempfile.TemporaryDirectory() as directory:
            exe = Path(directory) / "tests" / "x" / "a_tests"
            exe.parent.mkdir(parents=True)
            exe.write_bytes(b"")
            out = io.StringIO()
            with mock.patch.object(focused_targets, "load_inventory", return_value=inventory), \
                    mock.patch.object(focused_targets, "_run", fake_run), \
                    contextlib.redirect_stdout(out):
                code = focused_targets.build(directory, "Release", 2, 10, 10)
            self.assertFalse(exe.exists())
        text = out.getvalue()
        self.assertEqual(code, 1)
        self.assertIn("FOCUSED_TARGETS_SUMMARY inventory=4 built=1 skipped_not_configured=1 "
                      "failed=2 smoked=1 failed_targets=b_tests,gone_tests", text)
        self.assertIn([str(exe.resolve()), "--list-tests"], calls)

    def test_multiline_declaration_is_found(self):
        with tempfile.TemporaryDirectory() as directory:
            for suite in focused_targets.SUITES:
                (Path(directory) / "tests" / suite).mkdir(parents=True)
                (Path(directory) / "tests" / suite / "CMakeLists.txt").write_text("")
            (Path(directory) / "tests" / "sla_print" / "CMakeLists.txt").write_text(
                "# add_executable(commented EXCLUDE_FROM_ALL x.cpp)\n"
                "add_executable(\n    split_tests\n    EXCLUDE_FROM_ALL\n    a.cpp)\n"
                "if (X)\nadd_executable(cond_tests EXCLUDE_FROM_ALL b.cpp)\nendif()\n"
                "add_executable(plain_tests c.cpp)\n")
            self.assertEqual(focused_targets.cmake_targets(directory),
                             {"split_tests": False, "cond_tests": True})

    def test_ninja_multi_config_help_format_is_recognised(self):
        inventory = {"a_tests": None}
        help_text = "a_tests: phony\na_tests:Release: phony\nother:Debug: phony\n"
        runs = []

        def fake_run(cmd, timeout):
            runs.append(cmd)
            return (0, help_text) if cmd[-1] == "help" else (1, "x")

        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(focused_targets, "load_inventory", return_value=inventory), \
                mock.patch.object(focused_targets, "_run", fake_run), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            focused_targets.build(directory, "Release", 2, 10, 10)
        self.assertIn("failed=1 smoked=0 failed_targets=a_tests", out.getvalue())
        self.assertEqual(len(runs), 2)  # it was attempted, not "not configured"

    def test_timeout_and_deadline_accounting(self):
        inventory = {"a_tests": None, "b_tests": None, "c_tests": None}
        timeouts = []

        def fake_run(cmd, timeout):
            if cmd[-1] == "help":
                return 0, "a_tests: phony\nb_tests: phony\nc_tests: phony\n"
            timeouts.append(timeout)
            return 124, "[timed out]"

        clock = iter([0, 0, 10, 50, 60, 5000])
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(focused_targets, "load_inventory", return_value=inventory), \
                mock.patch.object(focused_targets, "_run", fake_run), \
                mock.patch.object(focused_targets.time, "monotonic", lambda: next(clock)), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            code = focused_targets.build(directory, "Release", 2, 1500, 10, deadline=100)
        text = out.getvalue()
        self.assertEqual(code, 1)
        self.assertEqual(timeouts, [100, 50])
        self.assertIn("FAILED c_tests (not attempted: deadline)", text)
        self.assertIn("built=0 skipped_not_configured=0 failed=3 smoked=0 "
                      "failed_targets=a_tests,b_tests,c_tests", text)


if __name__ == "__main__":
    unittest.main()
