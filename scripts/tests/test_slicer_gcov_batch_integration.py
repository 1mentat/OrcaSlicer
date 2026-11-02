from __future__ import annotations

import contextlib
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import sys
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


class CatchBatchGcovIntegrationTests(unittest.TestCase):
    def test_parser_keeps_default_ctest_mode_and_requires_explicit_gcov_opt_in(self):
        base = ["run", "--object", "build/tests/libslic3r_tests",
                "--test-command", "ctest --test-dir build/tests -C Release"]
        default = coverage._parser().parse_args(base)
        self.assertEqual(default.backend, "llvm")
        self.assertFalse(default.gcov_catch_batch)
        self.assertIsNone(default.gcov_catch_batch_timeout)

        opt_in = coverage._parser().parse_args([
            *base[0:1], "--backend", "gcov", *base[1:],
            "--gcov-catch-batch", "--gcov-catch-batch-timeout", "2400"])
        self.assertTrue(opt_in.gcov_catch_batch)
        self.assertEqual(opt_in.gcov_catch_batch_timeout, 2400)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--gcov-catch-batch"])
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                coverage.main([*base, "--gcov-catch-batch-timeout", "20"])

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
