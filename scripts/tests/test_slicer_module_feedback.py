from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
CLI = REPO_ROOT / "scripts" / "slicer_module_feedback.py"

FAKE = r'''#!{python}
import json, os, pathlib, subprocess, sys, time
a = sys.argv[1:]
mode = os.environ.get("FAKE_MODE", "ok")
names = json.loads(os.environ["FAKE_NAMES"])
if "--build" in a:
    pattern = os.environ.get("LLVM_PROFILE_FILE", "")
    if pattern:
        pathlib.Path(pattern.replace("%m", "build-fake")).write_bytes(b"build-profile")
    if mode == "timeout-build":
        code = "import os,time,pathlib; time.sleep(.7); pathlib.Path(os.environ['FAKE_MARKER']).write_text('survived')"
        subprocess.Popen([sys.executable, "-c", code])
        time.sleep(20)
elif "--list-tests" in a:
    print(os.environ["FAKE_INVENTORY"])
    pattern = os.environ.get("LLVM_PROFILE_FILE", "")
    if pattern:
        pathlib.Path(pattern.replace("%m", "inventory-fake")).write_bytes(b"inventory-profile")
elif "--reporter" in a and a[a.index("--reporter") + 1] == "junit":
    cases = "".join(f'<testcase name="{n}/section: nested"/>' for n in names)
    if mode == "ok":
        cases += f'<testcase name="{names[0]}/math: repeated"/>'
    failure = ' failures="1"' if mode == "junit-failure" else ' failures="0"'
    skipped = ' skipped="1"' if mode == "junit-skip" else ' skipped="0"'
    detail = '<failure message="expected fake failure"/>' if mode == "junit-failure" else ('<skipped/>' if mode == "junit-skip" else '')
    if detail:
        cases = cases.replace(f'<testcase name="{names[0]}/section: nested"/>',
                              f'<testcase name="{names[0]}/section: nested">{detail}</testcase>', 1)
    index = a.index("--out")
    pathlib.Path(a[index + 1]).write_text(f'<testsuites><testsuite tests="{len(names)}"{failure} errors="0"{skipped}>{cases}</testsuite></testsuites>')
    pattern = os.environ["LLVM_PROFILE_FILE"]
    if mode != "missing-profile":
        pathlib.Path(pattern.replace("%m", "fake-module")).write_bytes(b"fake-profile")
elif a and a[0] == "merge":
    pathlib.Path(a[a.index("-o") + 1]).write_bytes(b"fake-merged-profile")
elif a and a[0] in {"report", "show"}:
    print("fake coverage output")
'''


class ModuleFeedbackTests(unittest.TestCase):
    def fixture(self, mode="ok", expected=2, bad_source=False, timeout=5):
        tmp = tempfile.TemporaryDirectory(prefix="module feedback & safe ")
        base = Path(tmp.name)
        root = base / "repo"
        build = root / "build"
        evidence = build / "evidence"
        (root / "tests").mkdir(parents=True)
        evidence.mkdir(parents=True)
        source = root / "tests" / "module_tests.cpp"
        prod = root / "src" / "module.cpp"
        prod.parent.mkdir()
        source.write_text("// test source\n")
        prod.write_text("// production source\n")
        compile_db = build / "compile_commands.json"
        compile_db.write_text("[]\n")
        fake = build / "fake-tool"
        fake.write_text(FAKE.replace("{python}", sys.executable))
        fake.chmod(0o755)
        binary = build / "module-tests"
        binary.write_text(FAKE.replace("{python}", sys.executable))
        binary.chmod(0o755)
        xml_file = "tests/other.cpp" if bad_source else "tests/module_tests.cpp"
        inventory = (f'<MatchingTests>'
                     f'<TestCase><Name>Case A</Name><Tags>[ModuleTag]</Tags><SourceInfo><File>{xml_file}</File><Line>1</Line></SourceInfo></TestCase>'
                     f'<TestCase><Name>Case B</Name><Tags>[ModuleTag]</Tags><SourceInfo><File>{xml_file}</File><Line>2</Line></SourceInfo></TestCase>'
                     f'</MatchingTests>')
        subprocess.run(["git", "init", "-q", str(root)], check=True)
        subprocess.run(["git", "-C", str(root), "config", "user.name", "Test"], check=True)
        subprocess.run(["git", "-C", str(root), "config", "user.email", "test@example.invalid"], check=True)
        subprocess.run(["git", "-C", str(root), "add", "tests/module_tests.cpp", "src/module.cpp", "build/compile_commands.json"], check=True)
        subprocess.run(["git", "-C", str(root), "commit", "-qm", "test fixture"], check=True)
        out = evidence / "run"
        marker = base / "late-child-marker"
        env = os.environ.copy()
        env.update({"FAKE_MODE": mode, "FAKE_NAMES": json.dumps(["Case A", "Case B"]),
                    "FAKE_INVENTORY": inventory, "FAKE_MARKER": str(marker)})
        env["PYTHONPATH"] = os.pathsep.join(
            item for item in (str(REPO_ROOT / "scripts"), env.get("PYTHONPATH", "")) if item)
        args = [sys.executable, str(CLI), "--root", str(root), "--build", str(build),
                "--binary", str(binary), "--test-source", str(source), "--production-source", str(prod),
                "--compile-commands", str(compile_db), "--out", str(out), "--target", "module-tests",
                "--tag", "ModuleTag", "--expected-cases", str(expected), "--cmake", str(fake),
                "--llvm-profdata", str(fake), "--llvm-cov", str(fake), "--timeout", str(timeout)]
        return tmp, env, args, out, marker

    def invoke(self, mode="ok", expected=2, bad_source=False, cli_timeout=5, process_timeout=10):
        tmp, env, args, out, marker = self.fixture(mode, expected, bad_source, cli_timeout)
        completed = subprocess.run(args, env=env, text=True, stdout=subprocess.PIPE,
                                   stderr=subprocess.PIPE, timeout=process_timeout, check=False)
        return tmp, completed, out, marker

    def test_success_records_complete_scoped_cycle_and_duplicate_junit_rows(self):
        tmp, result, out, _ = self.invoke()
        self.addCleanup(tmp.cleanup)
        self.assertEqual(result.returncode, 0, result.stderr)
        receipt = json.loads((out / "run.json").read_text())
        self.assertEqual(receipt["status"], "passed")
        self.assertEqual([s["name"] for s in receipt["stages"]],
                         ["build", "inventory", "test", "profile_merge", "coverage_report", "coverage_lines"])
        self.assertEqual(len(receipt["inventory_names"]), 2)
        self.assertTrue((out / "coverage-summary.txt").is_file())
        self.assertTrue((out / "coverage-lines.txt").is_file())
        self.assertGreater(receipt["elapsed_seconds"], 0)
        self.assertEqual(receipt["coverage_scope"], "production_source_only")
        self.assertFalse(receipt["global_threshold_claim"])
        self.assertEqual(receipt["test_status"], "passed")
        self.assertEqual(len(receipt["merged_profile_sha256"]), 64)
        module_profiles = list((out / "module-profiles").glob("*.profraw"))
        self.assertEqual(len(module_profiles), 1)
        merge_argv = next(s["argv"] for s in receipt["stages"] if s["name"] == "profile_merge")
        self.assertTrue(all("module-profiles" in arg for arg in merge_argv[5:]))
        self.assertTrue(list((out / "build-profiles").glob("*.profraw")))
        self.assertTrue(list((out / "inventory-profiles").glob("*.profraw")))

    def test_wrong_inventory_count_stops_before_tests(self):
        tmp, result, out, _ = self.invoke(expected=3)
        self.addCleanup(tmp.cleanup)
        self.assertNotEqual(result.returncode, 0)
        receipt = json.loads((out / "run.json").read_text())
        self.assertEqual(receipt["status"], "failed")
        self.assertEqual([s["name"] for s in receipt["stages"]], ["build", "inventory"])
        self.assertIn("expected 3", receipt["error"])
        self.assertFalse((out / "coverage-summary.txt").exists())

    def test_wrong_source_translation_unit_stops_before_tests(self):
        tmp, env, args, out, _ = self.fixture(bad_source=True)
        self.addCleanup(tmp.cleanup)
        result = subprocess.run(args, env=env, text=True, capture_output=True, check=False)
        self.assertNotEqual(result.returncode, 0)
        receipt = json.loads((out / "run.json").read_text())
        self.assertIn("expected", receipt["error"])
        self.assertEqual([s["name"] for s in receipt["stages"]], ["build", "inventory"])
        self.assertFalse((out / "coverage-summary.txt").exists())

    def test_junit_failure_and_skip_keep_diagnostics_but_fail_closed(self):
        for mode in ("junit-failure", "junit-skip"):
            with self.subTest(mode=mode):
                tmp, result, out, _ = self.invoke(mode=mode)
                self.addCleanup(tmp.cleanup)
                self.assertNotEqual(result.returncode, 0)
                receipt = json.loads((out / "run.json").read_text())
                self.assertEqual(receipt["status"], "failed")
                self.assertEqual([s["name"] for s in receipt["stages"]],
                                 ["build", "inventory", "test", "profile_merge", "coverage_report", "coverage_lines"])
                self.assertIn("coverage report", receipt["error"])
                self.assertTrue((out / "coverage-summary.txt").is_file())
                self.assertTrue((out / "coverage-lines.txt").is_file())
                self.assertEqual(receipt["test_status"], "failed")
                self.assertEqual(len(receipt["merged_profile_sha256"]), 64)
                self.assertEqual(set(receipt["hashes_after"]),
                                 {"test_source", "production_source", "compile_commands"})
                self.assertEqual(len(receipt["elf_after_tests_sha256"]), 64)


    def test_empty_profile_fails_before_merge(self):
        tmp, result, out, _ = self.invoke(mode="missing-profile")
        self.addCleanup(tmp.cleanup)
        self.assertNotEqual(result.returncode, 0)
        receipt = json.loads((out / "run.json").read_text())
        self.assertIn("no nonempty LLVM profile", receipt["error"])
        self.assertFalse(any(stage["name"] == "profile_merge" for stage in receipt["stages"]))
        self.assertFalse((out / "coverage-summary.txt").exists())

    def test_nonfinite_timeout_is_rejected_before_output_creation(self):
        tmp, env, args, out, _ = self.fixture()
        self.addCleanup(tmp.cleanup)
        args.extend(["--timeout", "nan"])
        result = subprocess.run(args, env=env, text=True, capture_output=True, check=False)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("finite and positive", result.stderr)
        self.assertFalse(out.exists())

    def test_timed_out_build_kills_descendant_and_keeps_failure_receipt(self):
        tmp, result, out, marker = self.invoke(mode="timeout-build", cli_timeout=0.15, process_timeout=5)
        self.addCleanup(tmp.cleanup)
        self.assertNotEqual(result.returncode, 0)
        time.sleep(0.9)
        self.assertFalse(marker.exists(), "timed-out build descendant survived")
        receipt = json.loads((out / "run.json").read_text())
        self.assertEqual(receipt["status"], "failed")
        self.assertTrue(receipt["stages"][0]["timed_out"])
        self.assertEqual(receipt["stages"][0]["returncode"], 124)
        self.assertFalse((out / "coverage-summary.txt").exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
