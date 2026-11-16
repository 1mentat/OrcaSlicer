#!/usr/bin/env python3
"""Tests for the focused test-target link inventory."""

from pathlib import Path
from unittest import mock
import io
import contextlib
import os
import subprocess
import sys
import time
import shutil
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

    def harness(self, inventory, help_text, cache=None, make=(), pre=(), clock=None,
                deadline=3300, link=None, setup=None):
        """Run build() against a temp tree; `make` names get an exe when 'built'."""
        calls = []
        directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, directory, True)
        tests = Path(directory) / "tests" / "x"
        tests.mkdir(parents=True)
        if setup:
            setup(tests)
        for name in pre:
            (tests / name).write_bytes(b"")
        if cache is not None:
            (Path(directory) / "CMakeCache.txt").write_text(cache)

        def fake_run(cmd, timeout, tail=3000):
            calls.append((cmd, timeout))
            if cmd[-1] == "help":
                return 0, help_text
            if "--target" in cmd:
                target = cmd[cmd.index("--target") + 1]
                if target in make:
                    if link:
                        os.symlink(link, tests / target)
                    else:
                        (tests / target).write_bytes(b"")
                    return 0, ""
                return 1, "link error"
            return 0, ""

        patches = [mock.patch.object(focused_targets, "load_inventory", return_value=inventory),
                   mock.patch.object(focused_targets, "_run", fake_run)]
        if clock:
            patches.append(mock.patch.object(focused_targets.time, "monotonic",
                                             lambda it=iter(clock): next(it)))
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            for patch in patches:
                patch.start()
            try:
                code = focused_targets.build(directory, "Release", 2, 1500, 60, deadline=deadline)
            finally:
                mock.patch.stopall()
        return code, out.getvalue(), calls, tests

    def test_build_reports_failures_and_skips_without_hiding(self):
        inventory = {"a_tests": None, "b_tests": None, "gone_tests": None}
        code, text, calls, tests = self.harness(inventory, "a_tests: phony\nb_tests: phony\n",
                                                make=("a_tests",))
        self.assertEqual(code, 1)
        self.assertFalse((tests / "a_tests").exists())
        self.assertIn("built=1 skipped_not_configured=0 failed=2 smoked=1 "
                      "failed_targets=b_tests,gone_tests", text)
        self.assertTrue(any(c[0][-1] == "--list-tests" for c in calls))

    def test_optional_cad_binding(self):
        inventory = {"cad_tests": "SLIC3R_CAD"}
        for cache, expected in (("SLIC3R_CAD:BOOL=OFF\n", "skipped_not_configured=1 failed=0"),
                                ("SLIC3R_CAD:BOOL=ON\n", "skipped_not_configured=0 failed=1"),
                                ("OTHER:BOOL=ON\n", "skipped_not_configured=0 failed=1"),
                                (None, "skipped_not_configured=0 failed=1")):
            code, text, _, _ = self.harness(inventory, "", cache=cache)
            self.assertIn(expected, text, cache)
        _, text, _, _ = self.harness(inventory, "", cache="SLIC3R_CAD:BOOL=ON\n")
        self.assertIn("optional condition ON but target absent", text)

    def test_optional_reason_must_name_a_variable(self):
        with tempfile.TemporaryDirectory() as directory:
            inventory = Path(directory) / "inv.txt"
            inventory.write_text("a_tests optional: because cad\n")
            with self.assertRaises(ValueError):
                focused_targets.load_inventory(inventory)

    def test_pre_existing_output_is_failed_and_survives(self):
        code, text, calls, tests = self.harness({"a_tests": None}, "a_tests: phony\n",
                                                make=("a_tests",), pre=("a_tests",))
        self.assertEqual(code, 1)
        self.assertIn("pre-existing output", text)
        self.assertTrue((tests / "a_tests").exists())
        self.assertFalse(any("--target" in c[0] and c[0][-1] != "help" for c in calls[1:]))

    def test_empty_configure_time_object_dirs_are_normal(self):
        def before(tests):
            (tests / "CMakeFiles" / "a_tests.dir" / "Release").mkdir(parents=True)
        code, text, _, tests = self.harness({"a_tests": None}, "a_tests: phony\n",
                                            make=("a_tests",), setup=before)
        self.assertEqual(code, 0, text)
        self.assertIn("built=1", text)
        self.assertFalse((tests / "CMakeFiles" / "a_tests.dir").exists())

    def test_regular_file_in_object_dir_is_pre_existing(self):
        def before(tests):
            objects = tests / "CMakeFiles" / "a_tests.dir" / "Release"
            objects.mkdir(parents=True)
            (objects / "a.o").write_bytes(b"x")
        code, text, _, tests = self.harness({"a_tests": None}, "a_tests: phony\n",
                                            make=("a_tests",), setup=before)
        self.assertEqual(code, 1)
        self.assertIn("pre-existing output", text)
        self.assertTrue((tests / "CMakeFiles" / "a_tests.dir" / "Release" / "a.o").exists())

    def test_runner_oserror_fails_target_not_traceback(self):
        with mock.patch.object(focused_targets.slicer_process, "run_captured",
                               side_effect=PermissionError("denied")):
            code, text = focused_targets._run(["nope"], 5)
        self.assertEqual(code, 127)
        self.assertIn("denied", text)

    def test_symlinked_executable_is_failed_without_smoke_or_delete(self):
        with tempfile.TemporaryDirectory() as outside:
            victim = Path(outside) / "victim"
            victim.write_bytes(b"keep")
            code, text, calls, tests = self.harness({"a_tests": None}, "a_tests: phony\n",
                                                    make=("a_tests",), link=str(victim))
            self.assertEqual(code, 1)
            self.assertTrue(victim.exists())
            self.assertFalse(any(c[0][-1] == "--list-tests" for c in calls))
            self.assertIn("exactly one regular executable", text)

    def test_two_matches_are_failed(self):
        calls_made = []
        inventory = {"a_tests": None}
        directory = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, directory, True)
        for sub in ("x", "y"):
            (Path(directory) / "tests" / sub).mkdir(parents=True)

        def fake_run(cmd, timeout, tail=3000):
            calls_made.append(cmd)
            if cmd[-1] == "help":
                return 0, "a_tests: phony\n"
            for sub in ("x", "y"):
                (Path(directory) / "tests" / sub / "a_tests").write_bytes(b"")
            return 0, ""

        with mock.patch.object(focused_targets, "load_inventory", return_value=inventory), \
                mock.patch.object(focused_targets, "_run", fake_run), \
                contextlib.redirect_stdout(io.StringIO()) as out:
            code = focused_targets.build(directory, "Release", 2, 10, 10)
        self.assertEqual(code, 1)
        self.assertIn("found 2", out.getvalue())
        self.assertFalse(any(c[-1] == "--list-tests" for c in calls_made))

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
        help_text = "a_tests: phony\na_tests:Release: phony\nother:Debug: phony\n"
        _, text, calls, _ = self.harness({"a_tests": None}, help_text)
        self.assertIn("failed=1 smoked=0 failed_targets=a_tests", text)
        self.assertEqual(len(calls), 2)  # attempted, not "not configured"

    def test_timeout_and_deadline_accounting(self):
        inventory = {"a_tests": None, "b_tests": None, "c_tests": None}
        help_text = "a_tests: phony\nb_tests: phony\nc_tests: phony\n"
        code, text, calls, _ = self.harness(inventory, help_text, deadline=100,
                                            clock=[0, 0, 10, 50, 60, 5000])
        self.assertEqual(code, 1)
        self.assertEqual([c[1] for c in calls if c[0][-1] != "help"], [100, 50])
        self.assertIn("FAILED c_tests (not attempted: deadline)", text)
        self.assertIn("failed=3 smoked=0 failed_targets=a_tests,b_tests,c_tests", text)

    def test_deadline_before_smoke_is_failed(self):
        # end=100; build start=0; before smoke=500 -> beyond the deadline.
        code, text, calls, tests = self.harness({"a_tests": None}, "a_tests: phony\n",
                                                make=("a_tests",), deadline=100,
                                                clock=[0, 0, 500])
        self.assertEqual(code, 1)
        self.assertIn("deadline before --list-tests", text)
        self.assertFalse(any(c[0][-1] == "--list-tests" for c in calls))
        self.assertFalse((tests / "a_tests").exists())

    @unittest.skipIf(os.name == "nt", "POSIX process groups")
    def test_run_kills_backgrounded_holder_and_bounds_tail(self):
        started = time.monotonic()
        code, text = focused_targets._run(
            ["sh", "-c", "yes x | head -c 20000; sleep 30 & sleep 30"], 1)
        self.assertEqual(code, 124)
        self.assertLess(time.monotonic() - started, 5)
        self.assertLessEqual(len(text.encode()), 3000 + 40)


if __name__ == "__main__":
    unittest.main()
