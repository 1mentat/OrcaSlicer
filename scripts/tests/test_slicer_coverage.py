#!/usr/bin/env python3
"""Unit tests for the LLVM coverage report boundary (stdlib only)."""

from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_coverage  # noqa: E402


class CoverageEvidenceTests(unittest.TestCase):
    def test_empty_filter_has_no_evidence(self):
        self.assertEqual(slicer_coverage.test_evidence("No tests were found!!!\n"), 0)

    def test_ctest_and_catch_counts_are_evidence(self):
        self.assertEqual(slicer_coverage.test_evidence("100% tests passed, 0 tests failed out of 17"), 17)
        self.assertEqual(slicer_coverage.test_evidence("test cases: 4 | 4 passed"), 4)

    def test_all_skipped_or_not_run_is_not_evidence(self):
        self.assertEqual(
            slicer_coverage.test_evidence(
                "0% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(
            slicer_coverage.test_evidence(
                "100% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(slicer_coverage.test_evidence("test cases: 0 | 2 skipped"), 0)


class CoverageExportTests(unittest.TestCase):
    def test_gate_is_strictly_greater_than_threshold(self):
        summary = {"lines": {"covered": 9, "count": 10, "percent": 90.0}}
        self.assertFalse(slicer_coverage._meets_gate(summary, "lines", 90.0))
        summary["lines"]["percent"] = 90.01
        self.assertTrue(slicer_coverage._meets_gate(summary, "lines", 90.0))
        self.assertFalse(slicer_coverage._meets_gate(
            {"branches": {"covered": 0, "count": 0, "percent": 100.0}}, "branches", 90.0
        ))

    def test_report_uses_llvm_denominator_and_reports_missing_manifest_sources(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source_root = root / "src" / "libslic3r"
            source_root.mkdir(parents=True)
            hit = source_root / "Hit.cpp"
            zero = source_root / "Zero.cpp"
            hit.write_text("int hit;\n", encoding="utf-8")
            zero.write_text("int zero;\n", encoding="utf-8")
            exported = {
                "data": [{"files": [{
                    "filename": str(hit),
                    "summary": {
                        "lines": {"covered": 3, "count": 5},
                        "functions": {"covered": 1, "count": 2},
                        "branches": {"covered": 2, "count": 4},
                    },
                }]}],
            }
            report, missing = slicer_coverage.parse_export(
                exported, source_root=source_root, manifest=[hit, zero])
            self.assertEqual(report["summary"]["lines"]["covered"], 3)
            self.assertEqual(report["summary"]["lines"]["count"], 5)
            self.assertEqual(report["summary"]["branches"]["count"], 4)
            self.assertEqual(missing, [str(zero.resolve())])

    def test_outside_source_files_are_not_silently_counted(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source_root = root / "src"
            source_root.mkdir()
            outside = root / "deps.cpp"
            exported = {"data": [{"files": [{"filename": str(outside), "summary": {}}]}]}
            with self.assertRaises(slicer_coverage.CoverageError):
                slicer_coverage.parse_export(exported, source_root=source_root)

    def test_relative_export_paths_resolve_against_declared_root(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source_root = root / "src" / "libslic3r"
            source_root.mkdir(parents=True)
            source = source_root / "Relative.cpp"
            source.write_text("int value;\n", encoding="utf-8")
            exported = {"data": [{"files": [{
                "filename": "src/libslic3r/Relative.cpp",
                "summary": {"lines": {"covered": 1, "count": 1}},
            }]}]}
            report, missing = slicer_coverage.parse_export(
                exported, source_root=source_root, manifest=[source], path_base=root)
            self.assertEqual(missing, [])
            self.assertEqual(report["files"][0]["filename"], str(source.resolve()))

    def test_exported_source_without_executable_mapping_is_explicit(self):
        report = {
            "files": [{"filename": "/tmp/NoCode.cpp", "summary": {
                "lines": {"covered": 0, "count": 0},
                "functions": {"covered": 0, "count": 0},
                "regions": {"covered": 0, "count": 0},
            }}]
        }
        self.assertEqual(
            slicer_coverage._sources_without_executable_mappings(report),
            ["/tmp/NoCode.cpp"],
        )


class CommandTests(unittest.TestCase):
    def test_command_splitting_does_not_use_a_shell(self):
        self.assertEqual(slicer_coverage.split_command("ctest --test-dir 'build/tests'"),
                         ["ctest", "--test-dir", "build/tests"])


if __name__ == "__main__":
    unittest.main()
