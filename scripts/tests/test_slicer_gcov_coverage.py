"""Integrity tests for the GCC gcov physical-line backend (stdlib only)."""

from collections import Counter
import gzip
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_gcov_coverage as gcov  # noqa: E402
import slicer_coverage as common  # noqa: E402


class GcovPhysicalLineTests(unittest.TestCase):
    def test_line_counts_deduplicate_within_and_across_records(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            source = root / "src" / "sample.cpp"
            source.parent.mkdir()
            source.touch()
            report = gcov.validate_gcov_line_records([
                {"file": str(source), "lines": [
                    {"line_number": 4, "count": 1},
                    {"line_number": 4, "count": 0},
                    {"line_number": 5, "count": 0},
                ]},
                {"file": str(source), "lines": [
                    {"line_number": 4, "count": 3},
                    {"line_number": 6, "count": 0},
                ]},
            ], source_root=root / "src")
            self.assertEqual(report["summary"], {"hit_lines": 1, "mapped_lines": 3,
                                                   "percent": 100.0 / 3.0})
            self.assertEqual(report["files"][str(source)]["mapped_lines"], 3)

    def test_source_root_keeps_project_src_records_and_excludes_test_paths(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            src = root / "src" / "code.cpp"
            test = root / "tests" / "test.cpp"
            src.parent.mkdir()
            test.parent.mkdir()
            src.touch()
            test.touch()
            report = gcov.validate_gcov_line_records([
                {"file": str(src), "lines": [{"line_number": 1, "count": 1}]},
                {"file": str(test), "lines": [{"line_number": 1, "count": 1}]},
            ], source_root=root / "src")
            self.assertEqual(report["summary"]["mapped_lines"], 1)
            self.assertEqual(report["mapped_source_paths"], [str(src)])

    def test_project_record_filter_discards_external_headers_before_aggregation(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            src = root / "src" / "code.cpp"
            inline = root / "src" / "inline.hpp"
            external = root / "deps" / "header.hpp"
            records = [
                {"file": str(src), "lines": [{"line_number": 1, "count": 1}]},
                {"file": str(inline), "lines": [{"line_number": 3, "count": 2}]},
                {"file": str(external), "lines": [{"line_number": 9, "count": 99}]},
            ]
            self.assertEqual(gcov.project_gcov_records(records, root / "src"), records[:2])

    def test_zero_mapping_record_is_retained_and_bad_line_data_fails(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            empty = root / "src" / "empty.cpp"
            empty.parent.mkdir()
            empty.touch()
            report = gcov.validate_gcov_line_records([
                {"file": str(empty), "lines": []},
            ], source_root=root / "src")
            self.assertIn(str(empty), report["files"])
            self.assertEqual(report["summary"]["mapped_lines"], 0)
            self.assertEqual(report["summary"]["percent"], 0.0)
            self.assertFalse(common._meets_gate(
                {"lines": {"count": report["summary"]["mapped_lines"],
                           "percent": report["summary"]["percent"]}}, "lines", 0.0))
            with self.assertRaisesRegex(ValueError, "invalid gcov line"):
                gcov.validate_gcov_line_records([
                    {"file": str(empty), "lines": [{"line_number": 0, "count": -1}]},
                ], source_root=root / "src")

    def test_compiler_validation_requires_atomic_coverage_and_src_only_filter(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            source_root = root / "src"
            project_source = source_root / "lib.cpp"
            test_source = root / "tests" / "test.cpp"
            pattern = str(source_root).replace(".", r"\.") + r"/.*"
            safe = ["g++", "--coverage", "-fprofile-update=atomic",
                    f"-fprofile-filter-files=^{pattern}"]
            self.assertTrue(gcov.validate_gcc_compile_argv(
                safe, source=project_source, source_root=source_root))
            self.assertFalse(gcov.validate_gcc_compile_argv(
                safe, source=test_source, source_root=source_root))
            for argv in (
                ["g++", "--coverage", "-fprofile-update=atomic", "-fprofile-update=single",
                 f"-fprofile-filter-files=^{pattern}"],
                ["g++", "--coverage", "-fprofile-update=atomic"],
                ["g++", "--coverage", "-fprofile-update=atomic",
                 f"-fprofile-filter-files=^{pattern}", "-fprofile-filter-files=.*"],
            ):
                with self.subTest(argv=argv), self.assertRaises(ValueError):
                    gcov.validate_gcc_compile_argv(argv, source=project_source,
                                                   source_root=source_root)

    def test_compiler_filter_uses_lexical_symlink_path_but_scope_is_canonical(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            source_root = root / "src"
            source = source_root / "sample.cpp"
            source_root.mkdir()
            source.touch()
            alias = root / "checkout-alias"
            alias.symlink_to(root, target_is_directory=True)
            lexical = alias / "src" / "sample.cpp"
            pattern = str(alias / "src").replace(".", r"\.") + r"/.*"
            argv = ["g++", "--coverage", "-fprofile-update=atomic",
                    f"-fprofile-filter-files=^{pattern}"]
            self.assertTrue(gcov.validate_gcc_compile_argv(
                argv, source=source, source_root=source_root, filter_source=lexical))

    def test_gcda_prefix_binding_uses_full_absolute_object_path(self):
        prefix = Path("/coverage/run/gcov-data")
        obj = Path("/mnt/build/tests/CMakeFiles/libslic3r_tests.dir/a.cpp.o")
        self.assertEqual(gcov.gcda_path_for_object(prefix, obj),
                         Path("/coverage/run/gcov-data/mnt/build/tests/CMakeFiles/"
                              "libslic3r_tests.dir/a.cpp.gcda"))

    def test_gcov_without_project_output_requires_explicit_exclusion_or_empty_note(self):
        root = Path("/checkout/src")
        source = root / "empty.cpp"
        self.assertEqual(gcov.classify_translation_unit(
            source, filter_matches_project=True,
            records=[{"file": str(source), "lines": []}], source_root=root,
            gcov_output=""), ("mapping_free_translation_unit", [str(source)]))
        self.assertEqual(gcov.classify_translation_unit(
            Path("/checkout/tests/test.cpp"), filter_matches_project=False,
            records=[], source_root=root, gcov_output=""),
            ("excluded_by_project_source_filter", []))
        header = root / "inline.hpp"
        self.assertEqual(gcov.classify_translation_unit(
            Path("/checkout/tests/test.cpp"), filter_matches_project=False,
            records=[{"file": str(header), "lines": [{"line_number": 3, "count": 0}]}],
            source_root=root, gcov_output=""), ("header_only_mapping", [str(header)]))
        with self.assertRaisesRegex(ValueError, "no project mapping or explicit"):
            gcov.classify_translation_unit(source, filter_matches_project=True,
                                           records=[], source_root=root, gcov_output="unknown output")
        self.assertEqual(gcov.classify_translation_unit(
            source, filter_matches_project=True, records=[], source_root=root,
            gcov_output="No executable lines"), ("mapping_free_translation_unit", []))

    def test_cmake_generated_compiler_provenance_is_authoritative(self):
        with tempfile.TemporaryDirectory() as temp:
            record = Path(temp) / "CMakeFiles" / "3.28.3" / "CMakeCXXCompiler.cmake"
            record.parent.mkdir(parents=True)
            record.write_text('set(CMAKE_CXX_COMPILER_ID "GNU")\n'
                              'set(CMAKE_CXX_COMPILER_VERSION "13.3.0")\n', encoding="utf-8")
            self.assertEqual(gcov._cmake_compiler_provenance(record.parents[2]),
                             ("GNU", "13.3.0", record))

    def test_note_inventory_fails_missing_selected_note_and_reports_unselected(self):
        selected = Path("build/CMakeFiles/libslic3r.dir/a.cpp.gcno")
        unselected = Path("build/CMakeFiles/focused_test.dir/b.cpp.gcno")
        with self.assertRaisesRegex(ValueError, "selected manifest notes missing"):
            gcov.validate_note_inventory({selected}, set())
        self.assertEqual(gcov.validate_note_inventory({selected}, {selected, unselected}),
                         [str(unselected)])

    def test_artifact_snapshot_fails_closed_for_unreadable_elf(self):
        elf = Path("build/tests/libslic3r/libslic3r_tests")
        common = SimpleNamespace(_artifact_provenance=Mock(return_value={
            "path": str(elf), "status": "unreadable", "sha256": None,
        }))
        with self.assertRaisesRegex(ValueError, "required build artifact is missing or unreadable"):
            gcov._artifact_snapshot(common, {elf})

    def test_build_note_scan_uses_nested_cdb_outputs_not_prior_report_copies(self):
        with tempfile.TemporaryDirectory() as temp:
            build = Path(temp) / "build"
            prod = build / "src" / "libslic3r" / "CMakeFiles" / "libslic3r.dir" / "source.cpp.o"
            tests = build / "tests" / "libslic3r" / "CMakeFiles" / "libslic3r_tests.dir" / "test.cpp.o"
            other = build / "tests" / "extra" / "CMakeFiles" / "extra_tests.dir" / "extra.cpp.o"
            prior_report_copy = build / "run-1" / "gcov-input" / "source.cpp.gcno"
            for path in (prod.with_suffix(".gcno"), tests.with_suffix(".gcno"),
                         other.with_suffix(".gcno"), prior_report_copy):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            commands = [
                {"directory": str(build), "arguments": ["g++", "-c", str(source), "-o", str(obj)]}
                for source, obj in ((prod.with_suffix(".cpp"), prod),
                                    (tests.with_suffix(".cpp"), tests),
                                    (other.with_suffix(".cpp"), other))
            ]
            self.assertEqual(gcov.collect_build_notes(commands, Path(temp)),
                             {prod.with_suffix(".gcno").resolve(),
                              tests.with_suffix(".gcno").resolve(),
                              other.with_suffix(".gcno").resolve()})

    def test_manifest_covers_every_real_target_compile_row_and_skips_pch(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            build = root / "build"
            source = root / "tests" / "test.cpp"
            omitted = root / "tests" / "helper.cpp"
            pch = build / "CMakeFiles" / "target.dir" / "cmake_pch.hxx.cxx"
            for path in (source, omitted, pch):
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
            self.assertEqual(gcov.validate_manifest_compile_sources(
                [source], [source, pch], build), [])
            with self.assertRaisesRegex(ValueError, "compiled target source missing"):
                gcov.validate_manifest_compile_sources([source], [source, omitted, pch], build)

    def test_threshold_must_be_finite_and_in_range_before_cli_preflight(self):
        for value in (float("nan"), float("inf"), float("-inf"), -0.01, 100.01):
            with self.subTest(value=value), self.assertRaisesRegex(
                    ValueError, "finite percentage"):
                gcov.validate_threshold(value)
        self.assertEqual(gcov.validate_threshold(0.0), 0.0)
        self.assertEqual(gcov.validate_threshold(100.0), 100.0)
        with self.assertRaisesRegex(common.CoverageError, "finite percentage"):
            gcov.run_gcov_coverage(SimpleNamespace(threshold=float("nan")), common=common)

    def test_line_threshold_uses_common_strict_positive_denominator_gate(self):
        self.assertFalse(common._meets_gate(
            {"lines": {"count": 0, "percent": 100.0}}, "lines", 0.0))
        self.assertFalse(common._meets_gate(
            {"lines": {"count": 10, "percent": 90.0}}, "lines", 90.0))

    def test_ctest_junit_names_are_multiset_compared(self):
        mismatch = gcov.compare_test_identities(["one", "two"], Counter(["one", "one"]))
        self.assertEqual(mismatch, {"missing": ["two"], "unexpected": ["one"]})
        self.assertEqual(gcov.compare_test_identities(["one", "two"], Counter(["one", "two"])),
                         {"missing": [], "unexpected": []})

    def test_gcov_json_must_identify_the_matching_compiler_version(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "fixture.gcov.json.gz"
            with gzip.open(path, "wt", encoding="utf-8") as stream:
                json.dump({"gcc_version": "13.3.0", "format_version": "1",
                           "current_working_directory": temp, "files": []}, stream)
            files, metadata = gcov._read_gcov_json(path, "13.3.0")
            self.assertEqual(files, [])
            self.assertEqual(metadata["format_version"], "1")
            with self.assertRaisesRegex(ValueError, "does not match"):
                gcov._read_gcov_json(path, "12.2.0")


if __name__ == "__main__":
    unittest.main()
