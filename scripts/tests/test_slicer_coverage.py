#!/usr/bin/env python3
"""Unit tests for the LLVM coverage collector (stdlib only, no compiler needed).

tests/test_slicer_coverage_fixture.py drives the same collector against real
Clang output.
"""

from pathlib import Path
import hashlib
import json
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_coverage  # noqa: E402

ATOMIC_FLAGS = ["-fprofile-instr-generate", "-fcoverage-mapping", "-fprofile-update=atomic"]


def _completed(argv, returncode=0, stdout="", stderr=""):
    return subprocess.CompletedProcess(list(argv), returncode, stdout, stderr)


class Fixture:
    """A production source, its compile command, a fake test binary and fake LLVM tools."""

    def __init__(self, root: Path, *, export_stderr="", tests_output="100% tests passed, 0 tests failed out of 1\n",
                 tests_returncode=0, write_profile=True, coverage=(1, 1), versions=("18.1.3", "18.1.3", "18.1.3")):
        self.root = root
        self.source = root / "src" / "libslic3r" / "fixture.cpp"
        self.source.parent.mkdir(parents=True)
        self.source.write_text("int fixture() { return 1; }\n", encoding="utf-8")
        self.obj = root / "fixture.o"
        self.obj.touch()
        self.binary = root / "fixture-tests"
        self.binary.touch()
        self.build = root / "build"
        self.build.mkdir()
        self.manifest = self.build / "sources.txt"
        self.manifest.write_text("src/libslic3r/fixture.cpp\n", encoding="utf-8")
        self.commands = self.build / "compile_commands.json"
        self.commands.write_text(json.dumps([{
            "directory": str(root), "file": str(self.source),
            "arguments": ["/usr/bin/clang++", *ATOMIC_FLAGS, "-c", str(self.source), "-o", str(self.obj)],
        }]), encoding="utf-8")
        covered, count = coverage
        metric = {"covered": covered, "count": count}
        self.runtime_export = {"data": [{"files": [{"filename": str(self.source), "summary": {
            key: dict(metric) for key in slicer_coverage.METRICS}}]}]}
        self.export_stderr = export_stderr
        self.tests_output = tests_output
        self.tests_returncode = tests_returncode
        self.write_profile = write_profile
        self.versions = versions
        self.calls = []

    def args(self, *extra):
        return ["run", "--root", str(self.root), "--build-dir", str(self.build),
                "--object", str(self.binary), "--manifest", str(self.manifest),
                "--compile-commands", str(self.commands), "--test-command", "fixture-tests",
                "--out", str(self.root / "report"), "--llvm-cov", "llvm-cov",
                "--llvm-profdata", "llvm-profdata", *extra]

    def run(self, argv, **kwargs):
        argv = list(argv)
        self.calls.append(argv)
        if argv[0] == "fixture-tests":
            if self.write_profile:
                Path(kwargs["env"]["LLVM_PROFILE_FILE"].replace("%4m", "runtime")).write_bytes(b"raw")
            return _completed(argv, self.tests_returncode, self.tests_output)
        if argv[1:] == ["--version"]:
            tools = (str(Path("/usr/bin/clang++").resolve()), "llvm-cov", "llvm-profdata")
            version = dict(zip(tools, self.versions))[argv[0]]
            prefix = "clang" if "clang" in argv[0] else "LLVM"
            return _completed(argv, 0, f"Ubuntu {prefix} version {version}\n")
        if argv[0] == "llvm-profdata" and argv[1] == "merge":
            output = Path(argv[argv.index("-o") + 1])
            output.write_bytes(b"inventory" if output.name.startswith("compiler-") else b"runtime counts")
            return _completed(argv)
        if argv[0] == "llvm-cov" and argv[1] == "export":
            profile = Path(argv[argv.index("-instr-profile") + 1])
            exported = Path(argv[argv.index("-object") + 1])
            if exported == self.obj.resolve():
                assert profile.name == "compiler-inventory-empty.profdata"
                return _completed(argv, 0, json.dumps(self.runtime_export))
            assert profile.name == "merged.profdata"
            return _completed(argv, 0, json.dumps(self.runtime_export), self.export_stderr)
        raise AssertionError(f"unexpected command {argv}")

    def main(self, *extra):
        with mock.patch.object(slicer_coverage, "_run", side_effect=self.run), \
             mock.patch.object(slicer_coverage, "_which", side_effect=lambda name, *_: name):
            status = slicer_coverage.main(self.args(*extra))
        report = json.loads((self.root / "report" / "coverage-report.json").read_text(encoding="utf-8"))
        return status, report


class TestEvidenceTests(unittest.TestCase):
    def test_empty_filter_has_no_evidence(self):
        self.assertEqual(slicer_coverage.test_evidence("No tests were found!!!\n"), 0)

    def test_ctest_and_catch_counts_are_evidence(self):
        self.assertEqual(slicer_coverage.test_evidence("100% tests passed, 0 tests failed out of 17"), 17)
        self.assertEqual(slicer_coverage.test_evidence("test cases: 4 | 4 passed"), 4)
        self.assertEqual(slicer_coverage.test_evidence("All tests passed (43 assertions in 2 test cases)"), 2)
        self.assertEqual(slicer_coverage.test_evidence("All tests passed (43 assertions)"), 0)

    def test_all_skipped_or_not_run_is_not_evidence(self):
        self.assertEqual(slicer_coverage.test_evidence(
            "100% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"), 0)
        self.assertEqual(slicer_coverage.test_evidence("test cases: 0 | 2 skipped"), 0)


class CompileCommandTests(unittest.TestCase):
    def test_only_an_effective_atomic_update_is_accepted(self):
        for options, accepted in (
            ([], False),
            (["-fprofile-update=single"], False),
            (["-fprofile-update=prefer-atomic"], False),
            (["-fprofile-update=atomic", "-fprofile-update=single"], False),
            (["-fprofile-update=atomic", "@hidden-options.rsp"], False),
            (["-fprofile-update=single", "-fprofile-update=atomic"], True),
        ):
            with self.subTest(options=options):
                self.assertEqual(slicer_coverage._atomic_profile_updates(options), accepted)

    def test_unsafe_counters_are_rejected_before_any_object_export(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "concurrent.cpp"
            source.touch()
            commands = root / "compile_commands.json"
            commands.write_text(json.dumps([{"directory": str(root), "file": str(source), "arguments": [
                "clang", "-fprofile-instr-generate", "-fcoverage-mapping", "-c", str(source), "-o", "x.o"]}]))
            with mock.patch.object(slicer_coverage, "_run") as run:
                evidence = slicer_coverage._compiler_source_evidence(
                    "llvm-cov", [source], commands, timeout=5, inventory_profile=root / "unused")
                run.assert_not_called()
            self.assertEqual(evidence["sources"][0]["classification"], "unsafe_profile_counters")
            self.assertEqual(evidence["status"], "error")

    def test_preflight_rejects_unsafe_or_out_of_scope_builds_before_running_tests(self):
        for kind in ("mixed_commands", "no_commands", "outside_source_root", "empty_manifest"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp:
                fixture = Fixture(Path(temp))
                if kind == "mixed_commands":
                    entries = json.loads(fixture.commands.read_text())
                    entries.append(dict(entries[0], arguments=[
                        "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                        "-fprofile-update=single", "-c", str(fixture.source)]))
                    fixture.commands.write_text(json.dumps(entries))
                elif kind == "no_commands":
                    fixture.commands.write_text("[]")
                elif kind == "outside_source_root":
                    outside = fixture.root / "tests" / "fixture_test.cpp"
                    outside.parent.mkdir()
                    outside.touch()
                    fixture.manifest.write_text(str(outside) + "\n")
                else:
                    fixture.manifest.write_text("# nothing\n")
                status, report = fixture.main("--measure-only")
                self.assertEqual(status, 2)
                self.assertEqual(report["status"], "error")
                self.assertEqual(report["integrity_status"], "error")
                self.assertNotIn(["fixture-tests"], fixture.calls)


class ModeTests(unittest.TestCase):
    def test_measure_only_passes_any_valid_percentage(self):
        for coverage in ((0, 7), (1, 7), (7, 7)):
            with self.subTest(coverage=coverage), tempfile.TemporaryDirectory() as temp:
                status, report = Fixture(Path(temp), coverage=coverage).main("--measure-only")
                self.assertEqual(status, 0)
                self.assertEqual(report["status"], "pass")
                self.assertEqual(report["integrity_status"], "pass")
                self.assertEqual(report["measurement"]["mode"], "measure_only")
                self.assertFalse(report["measurement"]["gates_on_percentage"])
                self.assertIsNone(report["threshold"])
                self.assertEqual(report["summary"]["lines"]["covered"], coverage[0])

    def test_default_mode_does_not_gate(self):
        with tempfile.TemporaryDirectory() as temp:
            status, report = Fixture(Path(temp), coverage=(1, 10)).main()
            self.assertEqual((status, report["status"]), (0, "pass"))
            self.assertEqual(report["measurement"]["mode"], "measure_only")

    def test_threshold_mode_is_strict_and_opt_in(self):
        for coverage, expected in (((9, 10), (1, "coverage_below_threshold")), ((10, 10), (0, "pass"))):
            with self.subTest(coverage=coverage), tempfile.TemporaryDirectory() as temp:
                status, report = Fixture(Path(temp), coverage=coverage).main("--threshold", "90")
                self.assertEqual((status, report["status"]), expected)
                self.assertEqual(report["measurement"]["mode"], "threshold")
                self.assertEqual(report["measurement"]["threshold"], 90.0)

    def test_measure_only_and_threshold_are_exclusive(self):
        with self.assertRaises(SystemExit), mock.patch("sys.stderr"):
            slicer_coverage._parser().parse_args(
                ["run", "--object", "a", "--manifest", "m", "--test-command", "t",
                 "--measure-only", "--threshold", "1"])

    def test_threshold_is_strictly_greater(self):
        summary = {"lines": {"covered": 9, "count": 10, "percent": 90.0}}
        self.assertFalse(slicer_coverage._meets_threshold(summary, "lines", 90.0))
        summary["lines"]["percent"] = 90.01
        self.assertTrue(slicer_coverage._meets_threshold(summary, "lines", 90.0))
        self.assertFalse(slicer_coverage._meets_threshold(
            {"branches": {"covered": 0, "count": 0, "percent": 100.0}}, "branches", 0.0))


class FailureTests(unittest.TestCase):
    def test_incomplete_process_cleanup_survives_timeout_conversion(self):
        cases = (
            ("process cleanup", ["Windows command cleanup incomplete: process tree remains"]),
            ("handle cleanup", ["Windows command handle cleanup incomplete: pipe drain timed out"]),
            ("no notes", []),
        )
        for label, notes in cases:
            with self.subTest(label=label):
                timeout = slicer_coverage.subprocess.TimeoutExpired(
                    ["tests"], 1, output=b"partial test output\n", stderr=b"partial diagnostic\n"
                )
                timeout.cleanup_incomplete = True
                # Assign directly so this test works on Python 3.10, before add_note().
                timeout.__notes__ = notes
                with mock.patch.object(slicer_coverage, "run_captured", side_effect=timeout):
                    result = slicer_coverage._run(["tests"], timeout=1)

                self.assertEqual(result.returncode, 124)
                self.assertEqual(result.stdout, "partial test output\n")
                self.assertIn("partial diagnostic", result.stderr)
                self.assertIn("[command timed out after 1s]", result.stderr)
                if notes:
                    self.assertIn(notes[0], result.stderr)
                else:
                    self.assertIn("process helper reported incomplete cleanup", result.stderr)
                self.assertLessEqual(
                    len(result.stderr), len("partial diagnostic\n\n[command timed out after 1s]\n") + 512 + 3
                )

    def test_measure_only_still_fails_on_test_failure(self):
        with tempfile.TemporaryDirectory() as temp:
            status, report = Fixture(
                Path(temp), tests_returncode=8,
                tests_output="50% tests passed, 1 tests failed out of 2\n").main("--measure-only")
            self.assertEqual((status, report["status"]), (2, "test_failed"))
            self.assertTrue(report["tests_failed"])

    def test_measure_only_fails_without_profiles_or_evidence(self):
        for kwargs, expected in (
            ({"write_profile": False}, "no_profile"),
            ({"write_profile": False, "tests_output": "No tests were found!!!\n"}, "no_test_evidence"),
            ({"tests_output": "No tests were found!!!\n"}, "no_test_evidence"),
        ):
            with self.subTest(expected=expected, **kwargs), tempfile.TemporaryDirectory() as temp:
                status, report = Fixture(Path(temp), **kwargs).main("--measure-only")
                # status is authoritative; integrity_status describes only the mappings.
                self.assertEqual((status, report["status"]), (2, expected))

    def test_hash_mismatch_is_an_integrity_failure_even_at_full_coverage(self):
        with tempfile.TemporaryDirectory() as temp:
            status, report = Fixture(
                Path(temp), export_stderr="warning: 1 functions have mismatched data\n").main("--measure-only")
            self.assertEqual((status, report["status"]), (2, "coverage_integrity_error"))
            self.assertEqual(report["integrity_failures"]["profile_mapping"],
                             ["warning: 1 functions have mismatched data"])
            self.assertNotIn("stdout", report["export"])  # the export JSON has its own file

    def test_tool_version_mismatch_is_rejected_before_tests(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = Fixture(Path(temp), versions=("18.1.3", "17.0.6", "18.1.3"))
            status, report = fixture.main("--measure-only")
            self.assertEqual((status, report["status"]), (2, "error"))
            self.assertIn("do not match", report["error"])
            self.assertNotIn(["fixture-tests"], fixture.calls)

    def test_stale_profiles_are_removed_and_foreign_files_refused(self):
        with tempfile.TemporaryDirectory() as temp:
            fixture = Fixture(Path(temp))
            profiles = fixture.root / "report" / "profiles"
            profiles.mkdir(parents=True)
            (profiles / "old.profraw").write_bytes(b"stale")
            status, report = fixture.main("--measure-only")
            self.assertEqual(status, 0)
            self.assertEqual([Path(p).name for p in report["profiles"]], ["runtime.profraw"])
            (profiles / "unexpected.txt").write_text("x")
            status, report = fixture.main("--measure-only")
            self.assertEqual((status, report["status"]), (2, "error"))

    def test_unrelated_diagnostics_are_not_profile_mismatches(self):
        for diagnostics in ("", "informational diagnostic\n", "warning: 0 functions have mismatched data\n"):
            with self.subTest(diagnostics=diagnostics):
                self.assertEqual(slicer_coverage._profile_mapping_errors(diagnostics), [])


class ReportTests(unittest.TestCase):
    def test_report_declares_schema_backend_and_provenance(self):
        with tempfile.TemporaryDirectory() as temp:
            status, report = Fixture(Path(temp)).main("--measure-only")
            self.assertEqual(status, 0)
            self.assertEqual(report["schema"], "orca-slicer-coverage-report")
            self.assertEqual(report["schema_version"], slicer_coverage.REPORT_SCHEMA_VERSION)
            self.assertEqual(report["backend"], "llvm-source-based")
            self.assertEqual(report["tool_versions"]["compatibility"], "major_versions_match")
            self.assertIn("git", report["provenance"])
            self.assertEqual(report["files"][0]["filename"], str(Path(temp).resolve() / "src/libslic3r/fixture.cpp"))
            self.assertEqual(set(report["summary"]["lines"]), {"covered", "count", "percent"})
            self.assertEqual(report["annotation"]["status"], "not_requested")
            self.assertEqual(report["annotation"]["argv"][-1], report["files"][0]["filename"])
            self.assertEqual(report["profile_identity"]["status"], "unavailable")  # fake objects are not ELF

    def test_llvm_denominator_and_missing_manifest_sources(self):
        with tempfile.TemporaryDirectory() as temp:
            source_root = Path(temp) / "src" / "libslic3r"
            source_root.mkdir(parents=True)
            hit, zero = source_root / "Hit.cpp", source_root / "Zero.cpp"
            exported = {"data": [{"files": [{"filename": str(hit), "summary": {
                "lines": {"covered": 3, "count": 5}, "branches": {"covered": 2, "count": 4}}}]}]}
            report, missing, duplicates = slicer_coverage.parse_export(
                exported, source_root=source_root, manifest=[hit, zero])
            self.assertEqual((report["summary"]["lines"]["covered"], report["summary"]["lines"]["count"]), (3, 5))
            self.assertEqual(missing, [str(zero.resolve())])
            self.assertEqual(duplicates, [])

    def test_one_physical_file_is_never_counted_twice(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            source_root = root / "src"
            source_root.mkdir()
            real = source_root / "Real.cpp"
            real.write_text("int x;\n")
            (source_root / "alias").symlink_to(source_root)
            metric = {"covered": 1, "count": 2}
            exported = {"data": [{"files": [
                {"filename": str(real), "summary": {"lines": metric}},
                {"filename": str(source_root / "alias" / "Real.cpp"), "summary": {"lines": metric}},
                {"filename": "src/../src/Real.cpp", "summary": {"lines": metric}},
            ]}]}
            report, _, duplicates = slicer_coverage.parse_export(
                exported, source_root=source_root, path_base=root)
            self.assertEqual(len(report["files"]), 1)
            self.assertEqual(report["summary"]["lines"]["count"], 2)
            self.assertEqual(len(duplicates), 2)

    def test_outside_and_relative_paths(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            source_root = root / "src" / "libslic3r"
            source_root.mkdir(parents=True)
            with self.assertRaises(slicer_coverage.CoverageError):
                slicer_coverage.parse_export({"data": [{"files": [
                    {"filename": str(root / "deps.cpp"), "summary": {}}]}]}, source_root=source_root)
            report, missing, _ = slicer_coverage.parse_export(
                {"data": [{"files": [{"filename": "src/libslic3r/Relative.cpp",
                                      "summary": {"lines": {"covered": 1, "count": 1}}}]}]},
                source_root=source_root, manifest=[source_root / "Relative.cpp"], path_base=root)
            self.assertEqual(missing, [])
            self.assertEqual(report["files"][0]["filename"], str(source_root / "Relative.cpp"))

    def test_metadata_only_header_needs_a_successful_object_audit(self):
        report = {"files": [
            {"filename": "/tmp/Constants.hpp", "summary": {"lines": {"covered": 0, "count": 0}}},
            {"filename": "/tmp/Executable.hpp", "summary": {"lines": {"covered": 0, "count": 0}}},
            {"filename": "/tmp/Mapped.cpp", "summary": {"lines": {"covered": 1, "count": 1}}},
        ]}
        evidence = {"status": "ok", "mapping_free_sources": [], "object_mappings_without_source": [],
                    "object_executable_mappings": ["/tmp/Executable.hpp"]}
        unmapped, headers, unexpected = slicer_coverage._reconcile_unmapped_sources(report, evidence)
        self.assertEqual(unmapped, ["/tmp/Constants.hpp", "/tmp/Executable.hpp"])
        self.assertEqual(headers, ["/tmp/Constants.hpp"])
        self.assertEqual(unexpected, ["/tmp/Executable.hpp"])
        _, headers, unexpected = slicer_coverage._reconcile_unmapped_sources(report, {"status": "not_run"})
        self.assertEqual(headers, [])
        self.assertEqual(unexpected, unmapped)


class CompilerAuditTests(unittest.TestCase):
    def _commands(self, root, sources):
        commands = root / "compile_commands.json"
        commands.write_text(json.dumps([
            {"directory": str(root), "file": str(source), "arguments": [
                "clang", *ATOMIC_FLAGS, "-c", str(source), "-o", str(source.with_suffix(".o"))]}
            for source in sources]), encoding="utf-8")
        for source in sources:
            source.write_text("// fixture\n")
            source.with_suffix(".o").touch()
        profile = root / "inventory.profdata"
        profile.write_bytes(b"indexed")
        return commands, profile

    def test_mapped_mapping_free_and_header_only_objects_are_distinguished(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp).resolve()
            mapped, free, header_only = root / "mapped.cpp", root / "free.cpp", root / "header_only.cpp"
            commands, profile = self._commands(root, [mapped, free, header_only])
            header = root / "inline.hpp"

            def run(argv, **kwargs):
                obj = Path(argv[argv.index("-object") + 1]).name
                if obj == "free.o":
                    return _completed(argv, 1, "", "error: no coverage data found\n")
                filename = str(header if obj == "header_only.o" else mapped)
                return _completed(argv, 0, json.dumps({"data": [{"files": [
                    {"filename": filename, "summary": {"lines": {"covered": 0, "count": 1}}}]}]}))

            with mock.patch.object(slicer_coverage, "_run", side_effect=run):
                evidence = slicer_coverage._compiler_source_evidence(
                    "llvm-cov", [mapped, free, header_only], commands, timeout=5, inventory_profile=profile)
            self.assertEqual(evidence["status"], "ok")
            self.assertEqual(evidence["mapped_sources"], [str(mapped)])
            self.assertEqual(evidence["mapping_free_sources"], [str(free)])
            self.assertEqual(evidence["object_mappings_without_source"], [str(header_only)])
            self.assertEqual(evidence["object_executable_mappings"], sorted([str(header), str(mapped)]))

    def test_timeouts_mismatches_and_missing_objects_are_errors_not_mapping_free(self):
        for result, reason in (
            (_completed(["llvm-cov"], 124, "", "error: no coverage data found\n"), "compiler_export_failed"),
            (_completed(["llvm-cov"], 0, '{"data": [{"files": []}]}',
                        "warning: 3878 functions have mismatched data\n"), "compiler_profile_mapping_error"),
            (None, "missing_compiler_object"),
        ):
            with self.subTest(reason=reason), tempfile.TemporaryDirectory() as temp:
                root = Path(temp).resolve()
                source = root / "omitted.cpp"
                commands, profile = self._commands(root, [source])
                if result is None:
                    source.with_suffix(".o").unlink()
                with mock.patch.object(slicer_coverage, "_run", return_value=result):
                    evidence = slicer_coverage._compiler_source_evidence(
                        "llvm-cov", [source], commands, timeout=1, inventory_profile=profile)
                self.assertEqual(evidence["status"], "error")
                self.assertEqual(evidence["mapping_free_sources"], [])
                self.assertEqual(evidence["errors"][0]["reason"], reason)

    def test_empty_indexed_profile_fails_closed_and_refuses_symlinks(self):
        for returncode in (1, 0):
            with self.subTest(returncode=returncode), tempfile.TemporaryDirectory() as temp:
                output = Path(temp) / "compiler-inventory-empty.profdata"
                output.write_bytes(b"stale")
                with mock.patch.object(slicer_coverage, "_run", return_value=_completed([], returncode)):
                    with self.assertRaises(slicer_coverage.CoverageError):
                        slicer_coverage._create_empty_indexed_profile("llvm-profdata", output, timeout=5)
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "compiler-inventory-empty.profdata"
            target = Path(temp) / "protected"
            target.write_bytes(b"keep")
            output.symlink_to(target)
            with mock.patch.object(slicer_coverage, "_run") as run, \
                    self.assertRaises(slicer_coverage.CoverageError):
                slicer_coverage._create_empty_indexed_profile("llvm-profdata", output, timeout=5)
            run.assert_not_called()
            self.assertEqual(target.read_bytes(), b"keep")


class IdentityTests(unittest.TestCase):
    def test_binary_ids_are_parsed_from_llvm_profdata(self):
        output = ("Instrumentation level: Front-end\nTotal functions: 3\nBinary IDs: \n"
                  "26cf08ab877c33e2ad776619a9b26c92b1e94b9e\n4167e6d3231f9acd0e1dc56b0682c6170608f369\n")
        with mock.patch.object(slicer_coverage, "_run", return_value=_completed([], 0, output)):
            self.assertEqual(slicer_coverage._profile_binary_ids("llvm-profdata", Path("p"), 5),
                             ["26cf08ab877c33e2ad776619a9b26c92b1e94b9e",
                              "4167e6d3231f9acd0e1dc56b0682c6170608f369"])

    def test_foreign_and_unidentified_profiles_and_silent_objects_are_errors(self):
        with mock.patch.object(slicer_coverage, "elf_build_id", side_effect=lambda p: {"a": "aa11", "b": "bb22"}[p.name]), \
             mock.patch.object(slicer_coverage, "_profile_binary_ids",
                               side_effect=lambda tool, p, t: {"p1": ["aa11"], "p2": ["cc33"], "p3": []}[p.name]):
            evidence = slicer_coverage._profile_identity_evidence(
                "llvm-profdata", [Path("p1"), Path("p2"), Path("p3")], [Path("a"), Path("b")], 5)
        self.assertEqual(evidence["status"], "error")
        self.assertEqual(sorted(error["reason"] for error in evidence["errors"]),
                         ["foreign_profile", "object_without_profile", "profile_without_binary_id"])

    def test_elf_build_id_ignores_non_elf_files(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "script"
            path.write_text("#!/bin/sh\n")
            self.assertIsNone(slicer_coverage.elf_build_id(path))
            self.assertIsNone(slicer_coverage.elf_build_id(Path(temp) / "missing"))


class MiscTests(unittest.TestCase):
    def test_command_splitting_does_not_use_a_shell(self):
        self.assertEqual(slicer_coverage.split_command("ctest --test-dir 'build/tests'"),
                         ["ctest", "--test-dir", "build/tests"])

    def test_html_page_path_mirrors_llvm_cov_layout(self):
        self.assertEqual(slicer_coverage._html_page(Path("/o/html"), "/src/libslic3r/A.cpp"),
                         Path("/o/html/coverage/src/libslic3r/A.cpp.html"))

    def test_provenance_hashes_artifacts_and_marks_absent_git(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            binary = root / "libslic3r_tests"
            binary.write_bytes(b"test-binary")
            provenance = slicer_coverage._coverage_provenance(
                root, [binary], [], root / "build", root / "build" / "compile_commands.json")
            self.assertEqual(provenance["git"]["status"], "absent")
            self.assertIsNone(provenance["git"]["head"])
            self.assertEqual(provenance["test_object_binaries"][0]["sha256"],
                             hashlib.sha256(b"test-binary").hexdigest())
            self.assertEqual(provenance["compile_commands"]["status"], "missing")


if __name__ == "__main__":
    unittest.main()
