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


if __name__ == "__main__":
    unittest.main()
