from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import shutil
import sys
import subprocess
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


coverage = load_module("slicer_coverage_batch_candidate", SCRIPT_DIR / "slicer_coverage.py")
gcov = load_module("slicer_gcov_batch_candidate", SCRIPT_DIR / "slicer_gcov_coverage.py")


def _gcov_header(kind: str, stamp: int, *, version: bytes = b"B33*") -> bytes:
    magic = b"oncg" if kind == "gcno" else b"adcg"
    return magic + version[::-1] + stamp.to_bytes(4, "little") + bytes(4)


def _profile_gcda(profile: Path, note: Path) -> Path:
    return profile / str(note.with_suffix(".gcda")).lstrip("/")


def _write_gcov_pair(profile: Path, note: Path, stamp: int, *, payload: bytes = b"") -> Path:
    note.parent.mkdir(parents=True, exist_ok=True)
    note.write_bytes(_gcov_header("gcno", stamp))
    gcda = _profile_gcda(profile, note)
    gcda.parent.mkdir(parents=True, exist_ok=True)
    gcda.write_bytes(_gcov_header("gcda", stamp) + payload)
    return gcda


class CatchBatchGcovIntegrationTests(unittest.TestCase):
    def test_parser_keeps_default_ctest_mode_and_requires_explicit_gcov_opt_in(self):
        base = ["run", "--object", "build/tests/libslic3r_tests",
                "--test-command", "ctest --test-dir build/tests -C Release"]
        default = coverage._parser().parse_args(base)
        self.assertEqual(default.backend, "llvm")
        self.assertFalse(default.gcov_catch_batch)
        self.assertIsNone(default.gcov_catch_batch_timeout)
        self.assertEqual(default.gcov_catch_batch_fff_shards, 1)

        opt_in = coverage._parser().parse_args([
            *base[0:1], "--backend", "gcov", *base[1:],
            "--gcov-catch-batch", "--gcov-catch-batch-timeout", "2400"])
        self.assertTrue(opt_in.gcov_catch_batch)
        self.assertEqual(opt_in.gcov_catch_batch_timeout, 2400)
        shard_opt_in = coverage._parser().parse_args([
            *base[0:1], "--backend", "gcov", *base[1:], "--gcov-catch-batch",
            "--gcov-catch-batch-fff-shards", "2", "--gcov-tool", "/usr/bin/gcov-tool-13"])
        self.assertEqual(shard_opt_in.gcov_catch_batch_fff_shards, 2)
        self.assertEqual(shard_opt_in.gcov_tool, "/usr/bin/gcov-tool-13")
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--gcov-catch-batch"])
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--gcov-catch-batch-timeout", "20"])
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--backend", "gcov", "--gcov-catch-batch-fff-shards", "2"])
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--backend", "gcov", "--gcov-tool", "/usr/bin/gcov-tool-13"])

    def test_gcov_tool_profile_merge_preserves_inputs_and_exact_relative_gcda_union(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tool = root / "gcov-tool-13"
            tool.write_bytes(b"test executable bytes")
            tool.chmod(0o700)
            inputs = [root / f"input-{index}" for index in range(3)]
            for profile in inputs:
                profile.mkdir()
            note_one = root / "build" / "a" / "one.gcno"
            note_two = root / "build" / "b" / "two.gcno"
            _write_gcov_pair(inputs[0], note_one, 101, payload=b"one")
            _write_gcov_pair(inputs[1], note_one, 101, payload=b"two")
            _write_gcov_pair(inputs[1], note_two, 202, payload=b"two")
            # The third, empty profile root is valid; filtered shards may have no data.
            before = [{path.relative_to(profile).as_posix(): path.read_bytes()
                       for path in profile.rglob("*.gcda")} for profile in inputs]
            output = root / "output" / "merged"
            report_path = root / "output" / "merge-report.json"

            class FakeCommon:
                calls = []

                def _run(self, argv, *, cwd=None, env=None, timeout=None):
                    self.calls.append(list(argv))
                    if argv[-1] == "--version":
                        return subprocess.CompletedProcess(argv, 0, "gcov-tool (GCC) 13.3.0\n", "")
                    destination = Path(argv[argv.index("--output") + 1])
                    left, right = Path(argv[-2]), Path(argv[-1])
                    destination.mkdir(parents=True)
                    for source in (left, right):
                        for path in source.rglob("*.gcda"):
                            target = destination / path.relative_to(source)
                            target.parent.mkdir(parents=True, exist_ok=True)
                            shutil.copy2(path, target)
                    return subprocess.CompletedProcess(argv, 0, "", "")

            common = FakeCommon()
            report = gcov.merge_gcov_profile_roots(
                inputs, gcov_tool=tool, gcc_version="13.3.0", output_root=output,
                report_path=report_path, timeout=30, common=common,
                expected_notes={note_one, note_two})

            self.assertEqual(report["status"], "pass")
            self.assertEqual([Path(row["path"]) for row in report["input_profiles"]],
                             [profile.resolve() for profile in inputs])
            self.assertTrue(all(row["unchanged_after_merge"] for row in report["input_profiles"]))
            self.assertEqual(set(report["input_profiles"][0]["gcda_inventory"]),
                             {str(note_one.with_suffix(".gcda")).lstrip("/")})
            self.assertEqual(report["output_gcda_count"], 2)
            self.assertTrue(report_path.is_file())
            self.assertEqual({path.relative_to(output).as_posix() for path in output.rglob("*.gcda")},
                             {str(note_one.with_suffix(".gcda")).lstrip("/"),
                              str(note_two.with_suffix(".gcda")).lstrip("/")})
            self.assertEqual(len([row for row in common.calls if len(row) > 1 and row[1] == "merge"]), 1)
            after = [{path.relative_to(profile).as_posix(): path.read_bytes()
                      for path in profile.rglob("*.gcda")} for profile in inputs]
            self.assertEqual(before, after)

    def test_gcov_profile_merge_failure_writes_failure_receipt_without_merged_root(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tool = root / "gcov-tool-13"
            tool.write_bytes(b"test executable bytes")
            tool.chmod(0o700)
            inputs = [root / "one", root / "two"]
            for profile in inputs:
                profile.mkdir()
            note = root / "build" / "source.gcno"
            _write_gcov_pair(inputs[0], note, 123, payload=b"one")
            _write_gcov_pair(inputs[1], note, 123, payload=b"two")

            class FailingCommon:
                def _run(self, argv, *, cwd=None, env=None, timeout=None):
                    if argv[-1] == "--version":
                        return subprocess.CompletedProcess(argv, 0, "gcov-tool (GCC) 13.3.0\n", "")
                    return subprocess.CompletedProcess(argv, 2, "", "merge rejected")

            report_path = root / "merge-report.json"
            output = root / "merged"
            with self.assertRaises(gcov.GcovProfileMergeError):
                gcov.merge_gcov_profile_roots(
                    inputs, gcov_tool=tool, gcc_version="13.3.0", output_root=output,
                    report_path=report_path, timeout=30, common=FailingCommon(),
                    expected_notes={note})
            receipt = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(receipt["status"], "merge_failed")
            self.assertEqual(receipt["commands"][0]["returncode"], 2)
            self.assertFalse(output.exists())

    def test_gcov_profile_merge_rejects_incomplete_path_union(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tool = root / "gcov-tool-13"
            tool.write_bytes(b"test executable bytes")
            tool.chmod(0o700)
            inputs = [root / "one", root / "two"]
            note_one = root / "build" / "one.gcno"
            note_two = root / "build" / "two.gcno"
            for profile in inputs:
                profile.mkdir()
            _write_gcov_pair(inputs[0], note_one, 111, payload=b"one")
            _write_gcov_pair(inputs[1], note_two, 222, payload=b"two")

            class IncompleteCommon:
                def _run(self, argv, *, cwd=None, env=None, timeout=None):
                    if argv[-1] == "--version":
                        return subprocess.CompletedProcess(argv, 0, "gcov-tool (GCC) 13.3.0\n", "")
                    output = Path(argv[argv.index("--output") + 1])
                    left = Path(argv[-2])
                    output.mkdir(parents=True)
                    for path in left.rglob("*.gcda"):
                        target = output / path.relative_to(left)
                        target.parent.mkdir(parents=True, exist_ok=True)
                        shutil.copy2(path, target)
                    return subprocess.CompletedProcess(argv, 0, "", "")

            report_path = root / "merge-report.json"
            with self.assertRaisesRegex(gcov.GcovProfileMergeError, "GCDA inventory"):
                gcov.merge_gcov_profile_roots(
                    inputs, gcov_tool=tool, gcc_version="13.3.0", output_root=root / "merged",
                    report_path=report_path, timeout=30, common=IncompleteCommon(),
                    expected_notes={note_one, note_two})
            report = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "merge_failed")
            self.assertEqual(report["commands"][0]["identity_mismatch"]["missing_count"], 1)
            self.assertTrue(all(row["unchanged_after_merge"] for row in report["input_profiles"]))

    def test_gcov_profile_epoch_check_matches_versions_and_stamps(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            note = root / "build" / "src" / "unit.cpp.gcno"
            note.parent.mkdir(parents=True)
            note.write_bytes(_gcov_header("gcno", 0x12345678))
            profile = root / "profile"
            gcda = profile / str(note.with_suffix(".gcda")).lstrip("/")
            gcda.parent.mkdir(parents=True)
            gcda.write_bytes(_gcov_header("gcda", 0x87654321))

            mismatch = gcov.inspect_gcov_profile_epochs([profile], expected_notes={note})
            self.assertEqual(mismatch["status"], "mismatch")
            self.assertEqual(mismatch["checked_gcda_count"], 1)
            self.assertEqual(mismatch["mismatches"][0]["gcda_stamp"], 0x87654321)
            self.assertEqual(mismatch["mismatches"][0]["gcno_stamp"], 0x12345678)

            gcda.write_bytes(_gcov_header("gcda", 0x12345678))
            match = gcov.inspect_gcov_profile_epochs([profile], expected_notes={note})
            self.assertEqual(match["status"], "pass")
            self.assertEqual(match["mismatch_count"], 0)
            self.assertEqual(match["entries"][0]["gcda_version"], "B33*")

    def test_profile_merge_epoch_guard_stops_before_gcov_tool(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            tool = root / "gcov-tool-13"
            tool.write_bytes(b"test executable bytes")
            tool.chmod(0o700)
            inputs = [root / "input-01", root / "input-02"]
            note = root / "build" / "src" / "unit.cpp.gcno"
            note.parent.mkdir(parents=True)
            note.write_bytes(_gcov_header("gcno", 0x12345678))
            for profile in inputs:
                gcda = _profile_gcda(profile, note)
                gcda.parent.mkdir(parents=True)
                gcda.write_bytes(_gcov_header("gcda", 0x87654321) + b"protected-profile")
            sentinel_before = [gcda.read_bytes() for gcda in
                               (_profile_gcda(profile, note) for profile in inputs)]

            class CountingCommon:
                calls = []

                def _run(self, argv, *, cwd=None, env=None, timeout=None):
                    self.calls.append(list(argv))
                    return subprocess.CompletedProcess(argv, 0, "gcov-tool (GCC) 13.3.0\n", "")

            common = CountingCommon()
            report_path = root / "output" / "merge-report.json"
            with self.assertRaisesRegex(gcov.GcovProfileMergeError, "epoch check"):
                gcov.merge_gcov_profile_roots(
                    inputs, gcov_tool=tool, gcc_version="13.3.0",
                    output_root=root / "output" / "merged", report_path=report_path,
                    timeout=30, common=common, expected_notes={note})
            receipt = json.loads(report_path.read_text(encoding="utf-8"))
            self.assertEqual(receipt["profile_epoch_check"]["status"], "mismatch")
            self.assertEqual(receipt["profile_epoch_check"]["checked_gcda_count"], 2)
            self.assertEqual(common.calls, [])
            self.assertFalse((root / "output" / "merged").exists())
            self.assertEqual([_profile_gcda(profile, note).read_bytes() for profile in inputs],
                             sentinel_before)

    def test_gcov_epoch_reader_checks_big_endian_magic_and_version(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            note = root / "build" / "unit.cpp.gcno"
            note.parent.mkdir(parents=True)
            note.write_bytes(b"gcno" + b"B33*" + (77).to_bytes(4, "big") + bytes(4))
            profile = root / "profile"
            gcda = _profile_gcda(profile, note)
            gcda.parent.mkdir(parents=True)
            gcda.write_bytes(b"gcda" + b"B34*" + (77).to_bytes(4, "big") + bytes(4))

            mismatch = gcov.inspect_gcov_profile_epochs([profile], expected_notes={note})
            self.assertEqual(mismatch["status"], "mismatch")
            self.assertEqual(mismatch["mismatches"][0]["gcda_version"], "B34*")
            self.assertEqual(mismatch["mismatches"][0]["gcno_version"], "B33*")

            gcda.write_bytes(b"gcda" + b"B33*" + (77).to_bytes(4, "big") + bytes(4))
            match = gcov.inspect_gcov_profile_epochs([profile], expected_notes={note})
            self.assertEqual(match["status"], "pass")
            self.assertEqual(match["entries"][0]["gcda_stamp"], 77)

    def test_gcov_process_failure_reports_returncode_even_with_one_json_file(self):
        with self.assertRaises(ValueError) as raised:
            gcov.validate_gcov_output(
                5, 1, "/src/bbs_3mf.cpp",
                stderr="profile.gcda:stamp mismatch with notes file")
        message = str(raised.exception)
        self.assertIn("returncode=5", message)
        self.assertIn("emitted_json_count=1", message)
        self.assertIn("stamp mismatch with notes file", message)

    def test_junit_copy_is_hash_bound_and_refuses_replacement(self):
        payload = b"<?xml version='1.0'?><testsuites><testsuite tests='1'/></testsuites>\n"
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source, destination, receipt = root / "raw.xml", root / "ctest.xml", root / "copy.json"
            source.write_bytes(payload)
            copied = gcov._copy_catch_junit(source, destination, receipt)
            self.assertEqual(destination.read_bytes(), payload)
            self.assertEqual(copied["source_sha256"], copied["destination_sha256"])
            self.assertTrue(copied["byte_identical"])
            self.assertEqual(json.loads(receipt.read_text())["destination_sha256"],
                             hashlib.sha256(payload).hexdigest())
            with self.assertRaisesRegex(ValueError, "refusing to replace"):
                gcov._copy_catch_junit(source, destination, receipt)

    def test_failure_skip_error_and_identity_mismatch_cannot_be_credited(self):
        for problem in ("failure", "skipped", "error"):
            with self.subTest(problem=problem), tempfile.TemporaryDirectory() as temporary:
                path = Path(temporary) / "junit.xml"
                path.write_text(
                    f"<testsuites><testsuite><testcase name='selected'><{problem}/></testcase>"
                    "</testsuite></testsuites>", encoding="utf-8")
                _, _, assessment = gcov._junit_credit_assessment(path, ["selected"])
                self.assertFalse(assessment["eligible_for_coverage"])
                self.assertEqual(assessment["test_count"], 1)
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "junit.xml"
            path.write_text("<testsuites><testsuite><testcase name='other'/></testsuite></testsuites>",
                            encoding="utf-8")
            _, _, assessment = gcov._junit_credit_assessment(path, ["selected"])
            self.assertFalse(assessment["eligible_for_coverage"])
            self.assertEqual(assessment["name_mismatch"], {"missing": ["selected"], "unexpected": ["other"]})

    def test_pass_only_exact_junit_is_the_only_credit_eligible_batch_result(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "junit.xml"
            path.write_text("<testsuites><testsuite><testcase name='selected'/></testsuite></testsuites>",
                            encoding="utf-8")
            _, _, assessment = gcov._junit_credit_assessment(path, ["selected"])
            self.assertTrue(assessment["eligible_for_coverage"])


if __name__ == "__main__":
    unittest.main()
