#!/usr/bin/env python3
"""Unit tests for the LLVM coverage report boundary (stdlib only)."""

from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest
from unittest import mock

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
    def test_compiler_evidence_separates_mapping_free_translation_units(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            mapped = root / "mapped.cpp"
            mapping_free = root / "include_only.cpp"
            header_only = root / "header_only.cpp"
            mapped.write_text("int f() { return 1; }\n", encoding="utf-8")
            mapping_free.write_text("#include \"only_header.hpp\"\n", encoding="utf-8")
            header_only.write_text("#include \"executable_header.hpp\"\n", encoding="utf-8")
            mapped_object = root / "mapped.o"
            mapping_free_object = root / "include_only.o"
            header_only_object = root / "header_only.o"
            mapped_object.touch()
            mapping_free_object.touch()
            header_only_object.touch()
            compile_commands = root / "compile_commands.json"
            compile_commands.write_text(json.dumps([
                {"directory": str(root), "file": str(mapped), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping", "-c",
                    str(mapped), "-o", str(mapped_object),
                ]},
                {"directory": str(root), "file": str(mapping_free), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping", "-c",
                    str(mapping_free), "-o", str(mapping_free_object),
                ]},
                {"directory": str(root), "file": str(header_only), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping", "-c",
                    str(header_only), "-o", str(header_only_object),
                ]},
            ]), encoding="utf-8")
            fake_cov = root / "fake-llvm-cov.py"
            fake_cov.write_text(
                "#!/usr/bin/env python3\n"
                "import json, pathlib, sys\n"
                "obj = pathlib.Path(sys.argv[sys.argv.index('-object') + 1])\n"
                f"mapped = {str(mapped)!r}\n"
                f"header = {str(root / 'executable_header.hpp')!r}\n"
                "if obj.name == 'include_only.o':\n"
                "    print('error: no coverage data found', file=sys.stderr)\n"
                "    raise SystemExit(1)\n"
                "if obj.name == 'header_only.o':\n"
                "    print(json.dumps({'data': [{'files': [{'filename': header, 'summary': {'lines': {'covered': 0, 'count': 1}}}]}]}))\n"
                "    raise SystemExit(0)\n"
                "print(json.dumps({'data': [{'files': [{'filename': mapped, 'summary': {}}]}]}))\n",
                encoding="utf-8",
            )
            fake_cov.chmod(0o755)

            evidence = slicer_coverage._compiler_source_evidence(
                str(fake_cov), [mapped, mapping_free, header_only], compile_commands, timeout=10,
            )

            self.assertEqual(evidence["status"], "ok")
            self.assertEqual(evidence["mapped_sources"], [str(mapped)])
            self.assertEqual(evidence["mapping_free_sources"], [str(mapping_free)])
            self.assertEqual(evidence["object_mappings_without_source"], [str(header_only)])
            self.assertEqual(
                evidence["object_executable_mappings"],
                [slicer_coverage._path_key(root / "executable_header.hpp")],
            )
            self.assertEqual(evidence["errors"], [])

    def test_compiler_evidence_timeout_is_not_mapping_free(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "slow.cpp"
            source.write_text("int slow() { return 1; }\n", encoding="utf-8")
            obj = root / "slow.o"
            obj.touch()
            compile_commands = root / "compile_commands.json"
            compile_commands.write_text(json.dumps([{
                "directory": str(root),
                "file": str(source),
                "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping", "-c",
                    str(source), "-o", str(obj),
                ],
            }]), encoding="utf-8")
            timeout_result = slicer_coverage.subprocess.CompletedProcess(
                ["llvm-cov"], 124, "", "error: no coverage data found\n"
            )
            with mock.patch.object(slicer_coverage, "_run", return_value=timeout_result):
                evidence = slicer_coverage._compiler_source_evidence(
                    "llvm-cov", [source], compile_commands, timeout=1,
                )

            self.assertEqual(evidence["mapping_free_sources"], [])
            self.assertEqual(evidence["errors"][0]["reason"], "compiler_export_failed")

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

    def test_metadata_only_header_is_not_an_integrity_failure(self):
        report = {"files": [
            {"filename": "/tmp/PrintConfigConstants.hpp", "summary": {
                "lines": {"covered": 0, "count": 0},
                "functions": {"covered": 0, "count": 0},
            }},
            {"filename": "/tmp/Executable.hpp", "summary": {
                "lines": {"covered": 0, "count": 0},
                "functions": {"covered": 0, "count": 0},
            }},
            {"filename": "/tmp/Mapped.cpp", "summary": {
                "lines": {"covered": 1, "count": 1},
            }},
        ]}
        evidence = {
            "status": "ok",
            "mapping_free_sources": [],
            "object_mappings_without_source": [],
            "object_executable_mappings": ["/tmp/Executable.hpp"],
        }
        unmapped, metadata_headers, unexpected = slicer_coverage._reconcile_unmapped_sources(
            report, evidence
        )
        self.assertEqual(unmapped, ["/tmp/Executable.hpp", "/tmp/PrintConfigConstants.hpp"])
        self.assertEqual(metadata_headers, ["/tmp/PrintConfigConstants.hpp"])
        self.assertEqual(unexpected, ["/tmp/Executable.hpp"])

    def test_header_reconciliation_requires_successful_object_audit(self):
        report = {"files": [{"filename": "/tmp/Constants.hpp", "summary": {
            "lines": {"covered": 0, "count": 0},
        }}]}
        unmapped, metadata_headers, unexpected = slicer_coverage._reconcile_unmapped_sources(
            report, {"status": "not_run"}
        )
        self.assertEqual(unmapped, ["/tmp/Constants.hpp"])
        self.assertEqual(metadata_headers, [])
        self.assertEqual(unexpected, ["/tmp/Constants.hpp"])


class CommandTests(unittest.TestCase):
    def test_command_splitting_does_not_use_a_shell(self):
        self.assertEqual(slicer_coverage.split_command("ctest --test-dir 'build/tests'"),
                         ["ctest", "--test-dir", "build/tests"])


class ProvenanceTests(unittest.TestCase):
    def test_provenance_hashes_artifacts_and_marks_absent_git(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            binary = root / "tests" / "libslic3r_tests"
            manifest = root / "build" / "libslic3r.sources.txt"
            cache = root / "build" / "CMakeCache.txt"
            binary.parent.mkdir(parents=True)
            manifest.parent.mkdir(parents=True)
            binary.write_bytes(b"test-binary")
            manifest.write_text("src/libslic3r/Print.cpp\n", encoding="utf-8")
            cache.write_text("CMAKE_BUILD_TYPE:STRING=Release\n", encoding="utf-8")
            compile_commands = cache.parent / "compile_commands.json"
            compile_commands.write_text("[]\n", encoding="utf-8")

            provenance = slicer_coverage._coverage_provenance(
                root, [binary], [manifest], cache.parent
            )

            self.assertEqual(provenance["git"]["status"], "absent")
            self.assertIsNone(provenance["git"]["head"])
            self.assertEqual(provenance["test_object_binaries"][0]["status"], "present")
            self.assertEqual(
                provenance["test_object_binaries"][0]["sha256"],
                hashlib.sha256(b"test-binary").hexdigest(),
            )
            self.assertEqual(provenance["cmake_manifests"][0]["status"], "present")
            self.assertEqual(provenance["cmake_cache"]["status"], "present")
            self.assertEqual(provenance["compile_commands"]["status"], "present")
            self.assertEqual(
                provenance["compile_commands"]["sha256"],
                hashlib.sha256(b"[]\n").hexdigest(),
            )
            self.assertEqual(provenance["artifact_source_verification"]["status"], "not_performed")
            self.assertNotIn("content", provenance["test_object_binaries"][0])


if __name__ == "__main__":
    unittest.main()
