from __future__ import annotations

from collections import Counter
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET


SCRIPT_DIR = Path(__file__).resolve().parents[1]
SCRIPT = SCRIPT_DIR / "slicer_catch_batch.py"
sys.path.insert(0, str(SCRIPT_DIR))
SPEC = importlib.util.spec_from_file_location("slicer_catch_batch_candidate", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
batch = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = batch
SPEC.loader.exec_module(batch)


def _gcov_header(kind: str, stamp: int, *, version: bytes = b"B33*") -> bytes:
    magic = b"oncg" if kind == "gcno" else b"adcg"
    return magic + version[::-1] + stamp.to_bytes(4, "little") + bytes(4)


class CatchBatchValidationTests(unittest.TestCase):
    def test_selection_argv_removes_execution_controls_but_keeps_filtering(self):
        argv = ["/usr/bin/ctest", "--test-dir", "/build/tests", "-C", "Release",
                "-LE", "NotWorking|RequiresApp", "-R", "SettingsVariance",
                "--parallel", "12", "--timeout", "2400", "--output-log", "/tmp/ctest.log",
                "--output-on-failure"]
        self.assertEqual(batch.ctest_selection_argv(argv), [
            "/usr/bin/ctest", "--test-dir", "/build/tests", "-C", "Release",
            "-LE", "NotWorking|RequiresApp", "-R", "SettingsVariance", "--no-tests=error"])
        with self.assertRaisesRegex(batch.BatchValidationError, "--output-junit"):
            batch.ctest_selection_argv(["ctest", "--test-dir", "/build", "--output-junit", "old.xml"])

    def test_ctest_argv_preserves_selection_and_rejects_wrappers_or_conflicts(self):
        argv = ["/usr/bin/ctest", "--test-dir", "/build", "-C", "Release",
                "-LE", "NotWorking|RequiresApp", "-R", "^Settings"]
        self.assertEqual(batch.validate_ctest_argv(argv), [*argv, "--no-tests=error"])
        with self.assertRaisesRegex(batch.BatchValidationError, "real CTest"):
            batch.validate_ctest_argv(["ctest-wrapper", "--test-dir", "/build"])
        with self.assertRaisesRegex(batch.BatchValidationError, "--output-junit"):
            batch.validate_ctest_argv(["ctest", "--test-dir", "/build", "--output-junit", "x.xml"])
        with self.assertRaisesRegex(batch.BatchValidationError, "--show-only"):
            batch.validate_ctest_argv(["ctest", "--test-dir", "/build", "--show-only=json-v1"])
        with self.assertRaisesRegex(batch.BatchValidationError, "whole-batch"):
            batch.validate_ctest_argv(["ctest", "--test-dir", "/build", "--timeout", "60"])

    def test_exact_name_escaping_handles_catch_syntax_and_rejects_controls(self):
        name = 'Case, [tag] "quoted" path\\segment; wildcard* apostrophe\'s'
        self.assertEqual(
            batch.escape_catch_test_name(name),
            '"Case\\, [tag] \\"quoted\\" path\\\\segment; wildcard* apostrophe\\\'s"')
        with self.assertRaisesRegex(batch.BatchValidationError, "control"):
            batch.escape_catch_test_name("test\nother")

    def test_automake_normalization_and_all_skipped_exit_contract(self):
        cases = [
            batch.SelectedCase("ordinary", Path("/tmp/a"), Path("/tmp"), (), 1, False, "/tmp/a"),
            batch.SelectedCase("expected", Path("/tmp/a"), Path("/tmp"), ("!shouldfail",), 1, False, "/tmp/a"),
            batch.SelectedCase("may fail", Path("/tmp/a"), Path("/tmp"), ("!mayfail",), 1, False, "/tmp/a"),
            batch.SelectedCase("skipped", Path("/tmp/a"), Path("/tmp"), (), 1, False, "/tmp/a"),
        ]
        normalized = batch.validate_automake_records(
            cases, [("PASS", "ordinary"), ("XFAIL", "expected"),
                    ("XFAIL", "may fail"), ("SKIP", "skipped")], 0)
        self.assertEqual(normalized, {
            "ordinary": "PASS", "expected": "PASS", "may fail": "PASS", "skipped": "SKIP"})
        only_skip = cases[-1:]
        self.assertEqual(batch.validate_automake_records(only_skip, [("SKIP", "skipped")], 4),
                         {"skipped": "SKIP"})
        with self.assertRaisesRegex(batch.BatchValidationError, "must exit 4"):
            batch.validate_automake_records(only_skip, [("SKIP", "skipped")], 0)
        with self.assertRaisesRegex(batch.BatchValidationError, "untagged"):
            batch.validate_automake_records(cases[:1], [("XFAIL", "ordinary")], 42)
        shouldfail_pass = [cases[1]]
        self.assertEqual(batch.validate_automake_records(shouldfail_pass, [("PASS", "expected")], 42),
                         {"expected": "FAIL"})
        with self.assertRaisesRegex(batch.BatchValidationError, "unexpected test identity"):
            batch.validate_automake_records(cases[:1], [("PASS", "extra")], 0)
        with self.assertRaisesRegex(batch.BatchValidationError, "duplicate"):
            batch.validate_automake_records(cases[:1], [("PASS", "ordinary"), ("PASS", "ordinary")], 0)
        with self.assertRaisesRegex(batch.BatchValidationError, "omitted"):
            batch.validate_automake_records(cases[:2], [("PASS", "ordinary")], 0)
        with self.assertRaisesRegex(batch.BatchValidationError, "exit 42"):
            batch.validate_automake_records(cases[:1], [("PASS", "ordinary")], 42)

    def test_ctest_junit_has_one_row_per_registered_case_and_exact_skip(self):
        cases = [
            batch.SelectedCase("case one", Path("/tmp/a"), Path("/tmp"), (), 1, False, "/tmp/a"),
            batch.SelectedCase("case two", Path("/tmp/a"), Path("/tmp"), (), 1, False, "/tmp/a"),
            batch.SelectedCase("case three", Path("/tmp/a"), Path("/tmp"), (), 1, False, "/tmp/a"),
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "junit.xml"
            batch._write_ctest_junit(path, cases,
                                     {"case one": "PASS", "case two": "SKIP", "case three": "FAIL"},
                                     batch_by_name={case.name: "batch-01" for case in cases})
            root = ET.parse(path).getroot()
            rows = list(root.iter("testcase"))
            self.assertEqual(Counter(row.get("name") for row in rows), Counter(c.name for c in cases))
            self.assertEqual(sum(row.find("skipped") is not None for row in rows), 1)
            self.assertEqual(sum(row.find("failure") is not None for row in rows), 1)


class FakeAffinityController:
    def __init__(self, available=(12, 13, 14)):
        self.mask = set(available)
        self.calls = []

    def available(self):
        return frozenset(self.mask)

    @contextmanager
    def limit(self, requested):
        before = set(self.mask)
        wanted = set(requested)
        if not wanted <= before:
            raise batch.BatchValidationError("fake mask is outside caller mask")
        record = {"caller_mask_before": sorted(before), "requested_mask": sorted(wanted),
                  "applied_mask": sorted(wanted), "applied_and_verified": True}
        self.mask = wanted
        self.calls.append(record)
        try:
            yield record
        finally:
            self.mask = before
            record["caller_mask_restored"] = sorted(self.mask)
            record["restoration_verified"] = self.mask == before


class ThreadLocalFakeAffinityController:
    """A per-thread affinity fake that can exercise concurrent shard contexts."""

    def __init__(self, available=(12, 13, 14)):
        self.initial = frozenset(available)
        self.local = threading.local()
        self.calls = []
        self.lock = threading.Lock()

    def available(self):
        return getattr(self.local, "mask", self.initial)

    @contextmanager
    def limit(self, requested):
        before = self.available()
        wanted = frozenset(requested)
        if not wanted or not wanted <= before:
            raise batch.BatchValidationError("fake mask is outside caller mask")
        record = {"caller_mask_before": sorted(before), "requested_mask": sorted(wanted),
                  "applied_mask": sorted(wanted), "applied_and_verified": True}
        self.local.mask = wanted
        with self.lock:
            self.calls.append(record)
        try:
            yield record
        finally:
            self.local.mask = before
            record["caller_mask_restored"] = sorted(self.available())
            record["restoration_verified"] = self.available() == before


class CatchBatchPlanningTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.build = self.root / "build"
        self.build.mkdir()
        self.test_dir = self.build / "tests"
        self.test_dir.mkdir()
        self.executables: dict[str, Path] = {}
        self.entries: list[dict[str, object]] = []
        self.case_names_by_exe: dict[str, list[str]] = {}
        self.statuses: dict[str, str] = {}
        self.batch_launch_envs: list[dict[str, str]] = []
        self.call_order: list[tuple[str, str]] = []
        self.unexpected_listed_by_exe: dict[str, list[str]] = {}
        self.raise_on_batch = False
        self.affinity = FakeAffinityController()

        fixtures = [
            ("libslic3r_tests", ["comma, identity", "bracket [identity]", "expected xfail"],
             ["ordinary", "ordinary", "!shouldfail"], [1, 1, 2]),
            ("fff_print_tests", ["skip exact"], ["ordinary"], [2]),
            ("sla_print_tests", ["sla one"], ["ordinary"], [1]),
            ("libnest2d_tests", ["nest one"], ["ordinary"], [1]),
            ("filament_group_tests", ["filament one"], ["ordinary"], [1]),
        ]
        for exe_name, names, labels, processors in fixtures:
            exe = self.root / "bin" / exe_name
            exe.parent.mkdir(exist_ok=True)
            exe.write_bytes(("dummy binary " + exe_name).encode())
            exe.chmod(0o700)
            self.executables[exe_name] = exe
            self.case_names_by_exe[exe_name] = names
            for name, label, processor_count in zip(names, labels, processors):
                properties = [
                    {"name": "SKIP_RETURN_CODE", "value": 4},
                    {"name": "PROCESSOR_AFFINITY", "value": True},
                    {"name": "PROCESSORS", "value": processor_count},
                    {"name": "WORKING_DIRECTORY", "value": str(self.test_dir)},
                ]
                if label != "ordinary":
                    properties.append({"name": "LABELS", "value": [label]})
                self.entries.append({
                    "name": name,
                    "command": [str(exe), batch.escape_ctest_catch_argument(name)],
                    "config": "Release",
                    "properties": properties,
                })
                self.statuses[name] = "PASS"
        self.statuses["expected xfail"] = "XFAIL"
        self.statuses["skip exact"] = "SKIP"

    def tearDown(self):
        self.temporary.cleanup()

    @property
    def discovery(self):
        return {"version": {"major": 1, "minor": 0}, "tests": list(self.entries)}

    @staticmethod
    def ctest_argv():
        return ["/usr/bin/ctest", "--test-dir", "/tmp/build/tests", "-C", "Release",
                "-LE", "NotWorking|RequiresApp", "--no-tests=error"]

    def test_groups_by_binary_and_working_directory_and_rejects_semantic_gaps(self):
        batches = batch.plan_batches(self.discovery, build_dir=self.build, ctest_argv=self.ctest_argv())
        self.assertEqual(len(batches), 6)
        self.assertEqual([len(group.cases) for group in batches], [2, 1, 1, 1, 1, 1])
        self.assertEqual(sum(len(group.cases) for group in batches), len(self.entries))
        self.assertEqual([(group.executable.name, group.processors, group.processor_affinity)
                          for group in batches[:2]],
                         [("libslic3r_tests", 1, True), ("libslic3r_tests", 2, True)])

        wrong = json.loads(json.dumps(self.discovery))
        wrong["tests"][0]["properties"].append({"name": "TIMEOUT", "value": 30})
        with self.assertRaisesRegex(batch.BatchValidationError, "unsupported CTest property"):
            batch.plan_batches(wrong, build_dir=self.build)
        wrong = json.loads(json.dumps(self.discovery))
        wrong["tests"][0]["command"].append("--extra-option")
        with self.assertRaisesRegex(batch.BatchValidationError, "command shape"):
            batch.plan_batches(wrong, build_dir=self.build)
        wrong = json.loads(json.dumps(self.discovery))
        for test in wrong["tests"]:
            test["config"] = "Debug"
        with self.assertRaisesRegex(batch.BatchValidationError, "does not match"):
            batch.plan_batches(wrong, build_dir=self.build, ctest_argv=self.ctest_argv())
        wrong = json.loads(json.dumps(self.discovery))
        wrong["tests"][0]["properties"][0]["value"] = 0
        with self.assertRaisesRegex(batch.BatchValidationError, "SKIP_RETURN_CODE=4"):
            batch.plan_batches(wrong, build_dir=self.build)
        for unsupported in ("ENVIRONMENT", "WILL_FAIL"):
            wrong = json.loads(json.dumps(self.discovery))
            wrong["tests"][0]["properties"].append({"name": unsupported, "value": True})
            with self.assertRaisesRegex(batch.BatchValidationError, "unsupported CTest property"):
                batch.plan_batches(wrong, build_dir=self.build)

    def fake_executor(self, argv, *, cwd, env, timeout, log_path, receipt_path):
        exe = Path(argv[0]).name if argv else ""
        self.call_order.append((exe, "preflight" if "--list-tests" in argv else "batch"))
        if "--list-tests" in argv:
            self.assert_isolated_preflight_env(env)
            self.assertIn("--input-file", argv)
            input_path = Path(argv[argv.index("--input-file") + 1])
            self.assertTrue(input_path.is_file())
            input_lines = input_path.read_text(encoding="utf-8").splitlines()
            name_by_spec = {batch.escape_catch_test_name(name): name
                            for name in self.case_names_by_exe[exe]}
            self.assertTrue(all(line in name_by_spec for line in input_lines))
            selected_names = [name_by_spec[line] for line in input_lines]
            selected_names.extend(self.unexpected_listed_by_exe.get(exe, []))
            output = {"version": {"major": 3, "minor": 11}, "listings": {
                "tests": [{"name": name, "class-name": "", "tags": []}
                          for name in selected_names]}}
            log_path.write_text(json.dumps(output), encoding="utf-8")
            rc = 0
        else:
            if self.raise_on_batch:
                raise RuntimeError("injected process failure")
            self.batch_launch_envs.append(dict(env))
            if env["GCOV_PREFIX"] == self.parent_env["GCOV_PREFIX"]:
                self.assertEqual(env, self.parent_env)
            else:
                self.assertEqual(env["GCOV_PREFIX_STRIP"], "0")
                self.assertNotEqual(env["GCOV_PREFIX"], self.parent_env["GCOV_PREFIX"])
                self.assertNotEqual(env["LLVM_PROFILE_FILE"], self.parent_env["LLVM_PROFILE_FILE"])
            input_path = Path(argv[argv.index("--input-file") + 1])
            name_by_spec = {batch.escape_catch_test_name(name): name
                            for name in self.case_names_by_exe[exe]}
            selected_names = [name_by_spec[line]
                              for line in input_path.read_text(encoding="utf-8").splitlines()]
            statuses = [(name, self.statuses[name]) for name in selected_names]
            log_path.write_text("\n".join(f":test-result: {status} {name}" for name, status in statuses) + "\n",
                                encoding="utf-8")
            junit_option = next(arg for arg in argv if arg.startswith("junit::out="))
            raw_junit = Path(junit_option.split("=", 1)[1])
            raw_junit.write_text("<testsuites><testsuite tests='1'/></testsuites>\n", encoding="utf-8")
            rc = 4 if statuses and all(status == "SKIP" for _, status in statuses) else \
                42 if any(status in {"FAIL", "ERROR", "XPASS"} for _, status in statuses) else 0
        receipt_path.write_text(json.dumps({"returncode": rc}), encoding="utf-8")
        return subprocess.CompletedProcess(list(argv), rc)

    def assert_isolated_preflight_env(self, env):
        self.assertEqual(env["GCOV_PREFIX_STRIP"], "0")
        self.assertIn("/preflight-profiles/", env["GCOV_PREFIX"])
        self.assertIn("/preflight-profiles/", env["LLVM_PROFILE_FILE"])
        self.assertNotEqual(env["GCOV_PREFIX"], self.parent_env["GCOV_PREFIX"])

    def test_processor_cohorts_run_serially_with_parent_env_and_exact_junit(self):
        self.parent_env = {
            "PATH": "/usr/bin", "LANG": "C", "GCOV_PREFIX": "/coverage/measured",
            "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p.profraw",
        }
        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        digest = hashlib.sha256(raw).hexdigest()
        out = self.root / "output"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=600,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=digest, discovery_raw_json=raw,
            parent_env=self.parent_env, executor=self.fake_executor,
            affinity_controller=self.affinity)
        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["selected_count"], 7)
        self.assertEqual(report["batch_count"], 6)
        self.assertEqual(len(self.batch_launch_envs), 6)
        self.assertEqual(len([call for call in self.call_order if call[1] == "batch"]), 6)
        self.assertEqual(report["normalized_status_counts"], {"PASS": 6, "SKIP": 1})
        self.assertEqual((out / "ctest-discovery.json").read_bytes(), raw)
        junit = ET.parse(out / "ctest-junit.xml").getroot()
        rows = list(junit.iter("testcase"))
        self.assertEqual(Counter(row.get("name") for row in rows), Counter(e["name"] for e in self.entries))
        self.assertEqual(sum(row.find("skipped") is not None for row in rows), 1)
        self.assertEqual(len(report["raw_catch_junit_paths"]), 6)
        self.assertTrue(all(Path(path).is_file() for path in report["raw_catch_junit_paths"]))
        self.assertIn("--rng-seed", report["batches"][0]["batch_argv"])
        self.assertEqual(report["batches"][0]["batch_argv"][
            report["batches"][0]["batch_argv"].index("--rng-seed") + 1], "5")
        lines = Path(report["batches"][0]["catch_input"]).read_text(encoding="utf-8").splitlines()
        self.assertEqual(lines, ['"comma\\, identity"', '"bracket [identity]"'])
        self.assertEqual(batch.escape_ctest_catch_argument("bracket [identity]"),
                         r"bracket \[identity\]")
        self.assertEqual(report["batches"][0]["batch_cpu_mask"], [12])
        self.assertEqual(report["batches"][1]["batch_cpu_mask"], [12, 13])
        self.assertTrue(report["batches"][1]["batch_cpu_affinity"]["restoration_verified"])
        for info in report["batches"]:
            self.assertEqual(info["binary_sha256_before_run"], info["binary_sha256_after_run"])

    def test_discovery_hash_must_bind_exact_live_argv_and_bytes(self):
        raw = json.dumps(self.discovery).encode()
        with tempfile.TemporaryDirectory() as output:
            with self.assertRaisesRegex(batch.BatchValidationError, "discovery argv"):
                batch.run_discovered_batches(
                    self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                    out_dir=Path(output) / "bad-argv", batch_timeout_seconds=30,
                    discovery_argv=["ctest", "--test-dir", "/different", "--show-only=json-v1"],
                    discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw)

    def test_unexpected_list_match_fails_before_any_native_batch_process(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        self.unexpected_listed_by_exe["libslic3r_tests"] = ["another test selected by bad escaping"]
        raw = json.dumps(self.discovery).encode("utf-8")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(batch.BatchValidationError, "unexpected="):
                batch.run_discovered_batches(
                    self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                    out_dir=Path(temporary) / "output", batch_timeout_seconds=60,
                    discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
                    discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
                    parent_env=self.parent_env, executor=self.fake_executor,
                    affinity_controller=self.affinity)
        self.assertFalse(any(kind == "batch" for _, kind in self.call_order))

    def test_preflight_rejects_gcda_stamp_mismatch_before_any_native_batch_process(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        note = self.build / "src" / "unit.cpp.gcno"
        note.parent.mkdir(parents=True)
        note.write_bytes(_gcov_header("gcno", 0x12345678))

        def stale_profile_executor(argv, *, cwd, env, timeout, log_path, receipt_path):
            result = self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                        log_path=log_path, receipt_path=receipt_path)
            if "--list-tests" in argv and Path(argv[0]).name == "libslic3r_tests":
                gcda = Path(env["GCOV_PREFIX"]) / str(note.with_suffix(".gcda")).lstrip("/")
                gcda.parent.mkdir(parents=True, exist_ok=True)
                gcda.write_bytes(_gcov_header("gcda", 0x87654321))
            return result

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        out = self.root / "epoch-mismatch-output"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, executor=stale_profile_executor,
            affinity_controller=self.affinity, expected_notes={note},
            required_gcda_notes_by_executable={"libslic3r_tests": {note}})

        self.assertEqual(report["status"], "incomplete_preflight")
        self.assertEqual(report["batch_process_launch_attempted_count"], 0)
        self.assertFalse(any(kind == "batch" for _, kind in self.call_order))
        failed_batch = report["batches"][0]
        self.assertEqual(failed_batch["preflight_profile_epoch"]["mismatch_count"], 1)
        mismatch = failed_batch["preflight_profile_epoch"]["mismatches"][0]
        self.assertEqual(mismatch["gcda_stamp"], 0x87654321)
        self.assertEqual(mismatch["gcno_stamp"], 0x12345678)
        self.assertTrue(Path(failed_batch["preflight_profile_epoch_receipt"]).is_file())
        self.assertTrue((out / "batch-report.json").is_file())

    def test_preflight_rejects_matching_note_outside_build_before_native_batch(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        allowed_note = self.build / "src" / "unit.cpp.gcno"
        allowed_note.parent.mkdir(parents=True)
        allowed_note.write_bytes(_gcov_header("gcno", 0x12345678))
        outside_note = self.root / "outside" / "unit.cpp.gcno"
        outside_note.parent.mkdir(parents=True)
        outside_note.write_bytes(_gcov_header("gcno", 0x12345678))

        def outside_note_executor(argv, *, cwd, env, timeout, log_path, receipt_path):
            result = self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                        log_path=log_path, receipt_path=receipt_path)
            if "--list-tests" in argv and Path(argv[0]).name == "libslic3r_tests":
                gcda = Path(env["GCOV_PREFIX"]) / str(
                    outside_note.with_suffix(".gcda")).lstrip("/")
                gcda.parent.mkdir(parents=True, exist_ok=True)
                gcda.write_bytes(_gcov_header("gcda", 0x12345678))
            return result

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        out = self.root / "outside-note-output"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, executor=outside_note_executor,
            affinity_controller=self.affinity, expected_notes={allowed_note},
            required_gcda_notes_by_executable={"libslic3r_tests": {allowed_note}})

        self.assertEqual(report["status"], "incomplete_preflight")
        self.assertEqual(report["batch_process_launch_attempted_count"], 0)
        self.assertFalse(any(kind == "batch" for _, kind in self.call_order))
        failed = report["batches"][0]
        self.assertEqual(failed["preflight_profile_epoch"]["mismatches"][0]["status"],
                         "invalid_profile_binding")
        self.assertIn("outside the selected build notes",
                      failed["preflight_profile_epoch"]["mismatches"][0]["error"])

    def test_instrumented_preflight_rejects_no_gcda_but_generic_batch_allows_it(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        note = self.build / "src" / "unit.cpp.gcno"
        note.parent.mkdir(parents=True)
        note.write_bytes(_gcov_header("gcno", 0x12345678))
        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")

        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=self.root / "empty-instrumented-output", batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, executor=self.fake_executor,
            affinity_controller=self.affinity, expected_notes={note},
            required_gcda_notes_by_executable={"libslic3r_tests": {note}})

        self.assertEqual(report["status"], "incomplete_preflight")
        self.assertEqual(report["batch_process_launch_attempted_count"], 0)
        self.assertEqual(report["batches"][0]["preflight_profile_epoch"]["checked_gcda_count"], 0)
        self.assertEqual(report["batches"][0]["preflight_profile_epoch"]["mismatches"][0]["status"],
                         "required_gcda_missing")
        self.assertFalse(any(kind == "batch" for _, kind in self.call_order))

    def test_preflight_rejects_notes_outside_the_supplied_build_before_listing(self):
        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        outside_note = self.root / "outside" / "unit.cpp.gcno"
        outside_note.parent.mkdir(parents=True)
        outside_note.write_bytes(_gcov_header("gcno", 0x12345678))
        with self.assertRaisesRegex(batch.BatchValidationError, "outside the supplied build"):
            batch.run_discovered_batches(
                self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                out_dir=self.root / "out-of-build-context", batch_timeout_seconds=60,
                discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
                discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
                expected_notes={outside_note},
                required_gcda_notes_by_executable={"libslic3r_tests": {outside_note}})
        self.assertEqual(self.call_order, [])

    def test_cpu_limit_restores_on_executor_exception(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        self.raise_on_batch = True
        raw = json.dumps(self.discovery).encode("utf-8")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(RuntimeError, "injected process failure"):
                batch.run_discovered_batches(
                    self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                    out_dir=Path(temporary) / "output", batch_timeout_seconds=60,
                    discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
                    discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
                    parent_env=self.parent_env, executor=self.fake_executor,
                    affinity_controller=self.affinity)
        self.assertEqual(self.affinity.mask, {12, 13, 14})
        self.assertTrue(self.affinity.calls[-1]["restoration_verified"])

    def _add_fff_shard_cases(self):
        exe = self.executables["fff_print_tests"]
        for name in ("fff shard alpha", "fff shard beta"):
            self.case_names_by_exe["fff_print_tests"].append(name)
            self.statuses[name] = "PASS"
            self.entries.append({
                "name": name,
                "command": [str(exe), batch.escape_ctest_catch_argument(name)],
                "config": "Release",
                "properties": [
                    {"name": "SKIP_RETURN_CODE", "value": 4},
                    {"name": "PROCESSOR_AFFINITY", "value": True},
                    {"name": "PROCESSORS", "value": 1},
                    {"name": "WORKING_DIRECTORY", "value": str(self.test_dir)},
                ],
            })

    def test_opt_in_fff_shards_run_concurrently_with_distinct_profiles_and_exact_union(self):
        self._add_fff_shard_cases()
        self.parent_env = {
            "PATH": "/usr/bin", "LANG": "C", "GCOV_PREFIX": "/coverage/measured",
            "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p-%m.profraw",
        }
        active = 0
        max_active = 0
        lock = threading.Lock()
        rendezvous = threading.Barrier(2)

        def concurrent_executor(argv, *, cwd, env, timeout, log_path, receipt_path):
            nonlocal active, max_active
            if "--list-tests" not in argv:
                input_path = Path(argv[argv.index("--input-file") + 1])
                if "fff shard" in input_path.read_text(encoding="utf-8"):
                    with lock:
                        active += 1
                        max_active = max(max_active, active)
                    try:
                        rendezvous.wait(timeout=5)
                        time.sleep(0.01)
                        return self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                                  log_path=log_path, receipt_path=receipt_path)
                    finally:
                        with lock:
                            active -= 1
            return self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                      log_path=log_path, receipt_path=receipt_path)

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        out = self.root / "sharded-output"
        profile_root = self.root / "parallel-profiles"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=600,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, fff_shards=2, fff_profile_root=profile_root,
            executor=concurrent_executor,
            affinity_controller=ThreadLocalFakeAffinityController())

        self.assertEqual(report["status"], "pass")
        self.assertFalse(report["sessions_are_serial"])
        self.assertEqual(report["selected_count"], len(self.entries))
        self.assertEqual(max_active, 2)
        shard_batch = next(info for info in report["batches"] if info.get("shards"))
        shards = shard_batch["shards"]
        self.assertEqual([row["selected_count"] for row in shards], [1, 1])
        self.assertEqual(shards[0]["batch_cpu_mask"], [12])
        self.assertEqual(shards[1]["batch_cpu_mask"], [13])
        self.assertNotEqual(shards[0]["gcov_prefix"], shards[1]["gcov_prefix"])
        self.assertTrue(all(Path(row["gcov_prefix"]).is_dir() for row in shards))
        self.assertEqual(Counter(name for row in shards for name in row["selected_names"]),
                         Counter(("fff shard alpha", "fff shard beta")))
        junit = ET.parse(report["junit"]).getroot()
        self.assertEqual(Counter(row.get("name") for row in junit.iter("testcase")),
                         Counter(entry["name"] for entry in self.entries))
        self.assertEqual(len(report["raw_catch_junit_paths"]), 8)
        self.assertEqual(len(report["raw_automake_log_paths"]), 8)

    def test_fff_shard_timeout_is_reported_without_junit_or_success(self):
        self._add_fff_shard_cases()
        self.parent_env = {
            "PATH": "/usr/bin", "LANG": "C", "GCOV_PREFIX": "/coverage/measured",
            "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p-%m.profraw",
        }

        def timeout_one_shard(argv, *, cwd, env, timeout, log_path, receipt_path):
            if "--list-tests" not in argv:
                input_path = Path(argv[argv.index("--input-file") + 1])
                if "fff shard alpha" in input_path.read_text(encoding="utf-8"):
                    raise subprocess.TimeoutExpired(list(argv), timeout)
            return self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                      log_path=log_path, receipt_path=receipt_path)

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        out = self.root / "timeout-output"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, fff_shards=2,
            fff_profile_root=self.root / "timeout-profiles", executor=timeout_one_shard,
            affinity_controller=ThreadLocalFakeAffinityController())

        self.assertEqual(report["status"], "incomplete_timeout")
        self.assertIsNone(report["junit"])
        self.assertFalse((out / "ctest-junit.xml").exists())
        shard_batch = next(info for info in report["batches"] if info.get("shards"))
        self.assertEqual({row["process_outcome"] for row in shard_batch["shards"]},
                         {"timed_out", "exited"})
        self.assertEqual(report["selected_count"], len(self.entries))

    def test_fff_shard_execution_error_is_incomplete_without_junit(self):
        self._add_fff_shard_cases()
        self.parent_env = {
            "PATH": "/usr/bin", "LANG": "C", "GCOV_PREFIX": "/coverage/measured",
            "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p-%m.profraw",
        }

        def fail_one_shard(argv, *, cwd, env, timeout, log_path, receipt_path):
            if "--list-tests" not in argv:
                input_path = Path(argv[argv.index("--input-file") + 1])
                if "fff shard alpha" in input_path.read_text(encoding="utf-8"):
                    raise OSError("injected shard launch error")
            return self.fake_executor(argv, cwd=cwd, env=env, timeout=timeout,
                                      log_path=log_path, receipt_path=receipt_path)

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        out = self.root / "error-output"
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=out, batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, fff_shards=2,
            fff_profile_root=self.root / "error-profiles", executor=fail_one_shard,
            affinity_controller=ThreadLocalFakeAffinityController())

        self.assertEqual(report["status"], "incomplete_execution_error")
        self.assertIsNone(report["junit"])
        self.assertFalse((out / "ctest-junit.xml").exists())
        shard_batch = next(info for info in report["batches"] if info.get("shards"))
        self.assertEqual({row["process_outcome"] for row in shard_batch["shards"]},
                         {"execution_error", "exited"})

    def test_fff_affinity_entry_failure_is_not_counted_as_a_process_launch(self):
        self._add_fff_shard_cases()
        self.parent_env = {
            "PATH": "/usr/bin", "LANG": "C", "GCOV_PREFIX": "/coverage/measured",
            "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p-%m.profraw",
        }

        class FailFirstShardAffinityEntry(ThreadLocalFakeAffinityController):
            @contextmanager
            def limit(self, requested):
                if (set(requested) == {12}
                        and threading.current_thread().name.startswith("fff-catch-shard")):
                    raise OSError("injected affinity setup failure")
                with super().limit(requested) as record:
                    yield record

        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        report = batch.run_discovered_batches(
            self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
            out_dir=self.root / "affinity-entry-failure", batch_timeout_seconds=60,
            discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
            discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
            parent_env=self.parent_env, fff_shards=2,
            fff_profile_root=self.root / "affinity-entry-profiles",
            executor=self.fake_executor, affinity_controller=FailFirstShardAffinityEntry())

        self.assertEqual(report["status"], "incomplete_execution_error")
        self.assertIsNone(report["junit"])
        shard_batch = next(info for info in report["batches"] if info.get("shards"))
        first, second = shard_batch["shards"]
        self.assertIn("batch_argv", first)
        self.assertFalse(first["batch_process_launch_attempted"])
        self.assertTrue(second["batch_process_launch_attempted"])
        attempts = sum(bool(info.get("batch_process_launch_attempted"))
                       for info in report["batches"])
        attempts += sum(bool(shard.get("batch_process_launch_attempted"))
                        for info in report["batches"] for shard in info.get("shards", []))
        self.assertEqual(report["batch_process_launch_attempted_count"], attempts)

    def test_fff_shard_selector_rejects_absent_onecpu_cohort_before_execution(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0"}
        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        with self.assertRaisesRegex(batch.BatchValidationError, "exactly one one-CPU"):
            batch.run_discovered_batches(
                self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                out_dir=self.root / "wrong-fff-cohort", batch_timeout_seconds=60,
                discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
                discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
                parent_env=self.parent_env, fff_shards=2,
                fff_profile_root=self.root / "wrong-fff-profiles",
                executor=self.fake_executor, affinity_controller=self.affinity)
        self.assertEqual(self.call_order, [])


    def test_insufficient_cpu_mask_and_unsupported_platform_fail_closed(self):
        raw = json.dumps(self.discovery).encode("utf-8")
        with tempfile.TemporaryDirectory() as temporary:
            with self.assertRaisesRegex(batch.BatchValidationError, "requests 2 CPUs"):
                batch.run_discovered_batches(
                    self.discovery, ctest_argv=self.ctest_argv(), build_dir=self.build,
                    out_dir=Path(temporary) / "too-small", batch_timeout_seconds=60,
                    discovery_argv=[*self.ctest_argv(), "--show-only=json-v1"],
                    discovery_sha256=hashlib.sha256(raw).hexdigest(), discovery_raw_json=raw,
                    parent_env={"PATH": "/usr/bin"}, executor=self.fake_executor,
                    affinity_controller=FakeAffinityController((12,)))
        with patch.object(batch.sys, "platform", "darwin"):
            with self.assertRaisesRegex(batch.BatchValidationError, "requires Linux"):
                batch.LinuxAffinityController.available()

    def test_live_ctest_discovery_uses_private_profiles_and_records_raw_hash(self):
        self.parent_env = {"PATH": "/usr/bin", "GCOV_PREFIX": "/coverage/measured",
                           "GCOV_PREFIX_STRIP": "0", "LLVM_PROFILE_FILE": "/coverage/measured/%p.profraw"}
        raw = json.dumps(self.discovery, ensure_ascii=False).encode("utf-8")
        observed_env = {}

        def fake_ctest(argv, *, cwd, env, timeout, log_path, receipt_path):
            observed_env.update(env)
            self.assertEqual(argv[-1], "--show-only=json-v1")
            log_path.write_bytes(raw)
            receipt_path.write_text('{"returncode":0}\n', encoding="utf-8")
            return subprocess.CompletedProcess(list(argv), 0)

        with tempfile.TemporaryDirectory() as temporary:
            out = Path(temporary) / "discovery"
            command, payload, receipt = batch.discover_ctest(
                self.ctest_argv(), build_dir=self.build, out_dir=out,
                discovery_timeout_seconds=30, parent_env=self.parent_env, executor=fake_ctest)
            saved = Path(receipt["ctest_json"])
            self.assertEqual(saved.read_bytes(), raw)
            self.assertEqual(receipt["ctest_json_sha256"], hashlib.sha256(raw).hexdigest())
            self.assertEqual(command, self.ctest_argv())
            self.assertEqual(payload, self.discovery)
            self.assertIn("/discovery-profiles/ctest/", observed_env["GCOV_PREFIX"])
            self.assertIn("/discovery-profiles/ctest/", observed_env["LLVM_PROFILE_FILE"])
            self.assertNotEqual(observed_env["GCOV_PREFIX"], self.parent_env["GCOV_PREFIX"])

if __name__ == "__main__":
    unittest.main()
