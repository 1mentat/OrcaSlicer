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

    def test_catch_summary_counts_cases_instead_of_assertions(self):
        self.assertEqual(slicer_coverage.test_evidence("All tests passed (43 assertions in 2 test cases)"), 2)
        self.assertEqual(slicer_coverage.test_evidence("All tests passed (1 assertion in 1 test case)"), 1)
        self.assertEqual(slicer_coverage.test_evidence("All tests passed (43 assertions)"), 0)

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
    def test_compiler_audit_requires_effective_atomic_updates_before_object_export(self):
        for options, accepted in (
            ([], False),
            (["-fprofile-update=single"], False),
            (["-fprofile-update=prefer-atomic"], False),
            (["-fprofile-update=atomic", "-fprofile-update=single"], False),
            (["-fprofile-update=atomic-invalid"], False),
            (["-fprofile-update=atomic", "@hidden-options.rsp"], False),
            (["-fprofile-update=single", "-fprofile-update=atomic"], True),
            (["/clang:-fprofile-update=atomic"], True),
            (["/clang:-fprofile-update=atomic", "/clang:-fprofile-update=single"], False),
        ):
            with self.subTest(options=options), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                source = root / "concurrent.cpp"
                source.write_text("int concurrent() { return 1; }\n")
                obj = root / "concurrent.o"
                obj.touch()
                profile = root / "inventory.profdata"
                profile.write_bytes(b"indexed inventory")
                commands = root / "compile_commands.json"
                commands.write_text(json.dumps([{
                    "directory": str(root), "file": str(source), "arguments": [
                        "clang-cl", "/clang:-fprofile-instr-generate",
                        "/clang:-fcoverage-mapping", *options, "-c", str(source), "-o", str(obj),
                    ],
                }]))
                export = {"data": [{"files": [{
                    "filename": str(source), "summary": {"lines": {"count": 1, "covered": 0}},
                }]}]}
                with mock.patch.object(slicer_coverage, "_run", return_value=
                                       slicer_coverage.subprocess.CompletedProcess(
                                           [], 0, json.dumps(export), "")) as run:
                    evidence = slicer_coverage._compiler_source_evidence(
                        "llvm-cov", [source], commands, timeout=5, inventory_profile=profile,
                    )
                if accepted:
                    self.assertEqual(evidence["status"], "ok")
                    run.assert_called_once()
                else:
                    run.assert_not_called()
                    self.assertEqual(evidence["errors"], [{
                        "source": str(source),
                        "reason": "compile_command_missing_atomic_profile_updates",
                    }])
                    self.assertEqual(evidence["sources"][0]["classification"], "unsafe_profile_counters")

    def test_preflight_rejects_missing_or_mixed_evidence_before_running_tests(self):
        for kind in ("no_manifest", "no_commands", "mixed_commands"):
            with self.subTest(kind=kind), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                source = root / "concurrent.cpp"
                source.touch()
                binary = root / "tests"
                binary.touch()
                manifest = root / "sources.txt"
                manifest.write_text(str(source) + "\n")
                commands = root / "compile_commands.json"
                if kind == "mixed_commands":
                    commands.write_text(json.dumps([
                        {"directory": str(root), "file": str(source), "command":
                         f"clang -fprofile-instr-generate -fcoverage-mapping -fprofile-update={mode} -c concurrent.cpp"}
                        for mode in ("atomic", "single")
                    ]))
                args = slicer_coverage._parser().parse_args([
                    "run", "--root", str(root), "--object", str(binary),
                    "--compile-commands", str(commands), "--test-command", "expensive-tests",
                    *([] if kind == "no_manifest" else ["--manifest", str(manifest)]),
                ])
                with mock.patch.object(slicer_coverage, "_which", side_effect=lambda name, explicit: name), \
                     mock.patch.object(slicer_coverage, "_run") as run:
                    with self.assertRaises(slicer_coverage.CoverageError):
                        slicer_coverage.run_coverage(args)
                    run.assert_not_called()

    def test_preflight_rejects_manifests_outside_production_scope_before_tests(self):
        for relative in ("tests/fff_print/fixture.cpp", "src/libslic3r-shadow/fixture.cpp"):
            with self.subTest(source=relative), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                source = root / relative
                source.parent.mkdir(parents=True)
                source.touch()
                binary = root / "fixture-tests"
                binary.touch()
                manifest = root / "sources.txt"
                manifest.write_text(str(source) + "\n", encoding="utf-8")
                commands = root / "compile_commands.json"
                commands.write_text(json.dumps([{
                    "directory": str(root), "file": str(source), "arguments": [
                        "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                        "-fprofile-update=atomic", "-c", str(source),
                    ],
                }]), encoding="utf-8")
                args = slicer_coverage._parser().parse_args([
                    "run", "--root", str(root), "--object", str(binary),
                    "--manifest", str(manifest), "--compile-commands", str(commands),
                    "--test-command", "expensive-tests",
                ])
                with mock.patch.object(slicer_coverage, "_which", side_effect=lambda name, explicit: name), \
                     mock.patch.object(slicer_coverage, "_run", side_effect=AssertionError("test command launched")) as run:
                    with self.assertRaisesRegex(slicer_coverage.CoverageError, "no sources under.*source-root"):
                        slicer_coverage.run_coverage(args)
                    run.assert_not_called()

    def test_empty_indexed_profile_generation_fails_closed(self):
        for returncode, reason in ((1, "failed"), (0, "missing")):
            with self.subTest(returncode=returncode, reason=reason), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                output = root / "compiler-inventory-empty.profdata"
                output.write_bytes(b"stale profile from an earlier run")
                calls = []

                def run(argv, **kwargs):
                    calls.append(list(argv))
                    if returncode:
                        output.write_bytes(b"partial profile from failed merge")
                    return slicer_coverage.subprocess.CompletedProcess(
                        argv, returncode, "", "failed" if returncode else ""
                    )

                with mock.patch.object(slicer_coverage, "_run", side_effect=run):
                    with self.assertRaises(slicer_coverage.CoverageError):
                        slicer_coverage._create_empty_indexed_profile(
                            "llvm-profdata", output, timeout=5
                        )

                self.assertEqual(len(calls), 1)
                self.assertEqual(calls[0][0:3], ["llvm-profdata", "merge", "-sparse"])
                self.assertNotIn("--text", calls[0])
                self.assertTrue((root / "compiler-inventory-empty.proftext").is_file())
                self.assertEqual(
                    (root / "compiler-inventory-empty.proftext").read_text(encoding="utf-8"),
                    ":ir\n",
                )

    def test_empty_indexed_profile_rejects_symlink_paths_without_touching_targets(self):
        for path_kind in ("output", "profile_text"):
            with self.subTest(path_kind=path_kind), tempfile.TemporaryDirectory() as temp:
                root = Path(temp)
                output = root / "compiler-inventory-empty.profdata"
                target = root / f"protected-{path_kind}"
                target.write_bytes(b"runtime profile that must remain unchanged")
                symlink = output if path_kind == "output" else output.with_suffix(".proftext")
                symlink.symlink_to(target)

                with mock.patch.object(slicer_coverage, "_run") as run:
                    with self.assertRaises(slicer_coverage.CoverageError):
                        slicer_coverage._create_empty_indexed_profile(
                            "llvm-profdata", output, timeout=5
                        )
                    run.assert_not_called()

                self.assertEqual(target.read_bytes(), b"runtime profile that must remain unchanged")

    def test_runtime_and_inventory_profiles_are_separate_in_coverage_run(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source_root = root / "src" / "libslic3r"
            source_root.mkdir(parents=True)
            source = source_root / "fixture.cpp"
            source.write_text("int fixture() { return 1; }\n", encoding="utf-8")
            obj = root / "fixture.o"
            obj.touch()
            binary = root / "fixture-tests"
            binary.touch()
            build = root / "build"
            build.mkdir()
            manifest = build / "sources.txt"
            manifest.write_text("src/libslic3r/fixture.cpp\n", encoding="utf-8")
            compile_commands = build / "compile_commands.json"
            compile_commands.write_text(json.dumps([{
                "directory": str(root),
                "file": str(source),
                "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(source), "-o", str(obj),
                ],
            }]), encoding="utf-8")

            metric = {"covered": 1, "count": 1, "percent": 100.0}
            runtime_export = {"data": [{"files": [{
                "filename": str(source),
                "summary": {"lines": metric, "functions": metric, "branches": metric},
            }]}]}
            inventory_metric = {"covered": 0, "count": 1, "percent": 0.0}
            inventory_export = {"data": [{"files": [{
                "filename": str(source),
                "summary": {"lines": inventory_metric, "functions": inventory_metric,
                            "branches": inventory_metric},
            }]}]}
            calls = []

            def run(argv, **kwargs):
                calls.append(list(argv))
                if argv[0] == "fixture-tests":
                    profile = Path(kwargs["env"]["LLVM_PROFILE_FILE"].replace("%4m", "runtime"))
                    profile.write_bytes(b"runtime profile")
                    return slicer_coverage.subprocess.CompletedProcess(
                        argv, 0, "100% tests passed, 0 tests failed out of 1\n", ""
                    )
                if argv[0] == "llvm-profdata" and argv[1] == "--version":
                    return slicer_coverage.subprocess.CompletedProcess(argv, 0, "fake LLVM\n", "")
                if argv[0] == "llvm-profdata":
                    output = Path(argv[argv.index("-o") + 1])
                    output.write_bytes(b"empty inventory" if output.name.startswith("compiler-")
                                       else b"runtime counts")
                    return slicer_coverage.subprocess.CompletedProcess(argv, 0, "", "")
                if argv[0] == "llvm-cov" and argv[1] == "--version":
                    return slicer_coverage.subprocess.CompletedProcess(argv, 0, "fake LLVM\n", "")
                if argv[0] == "llvm-cov" and argv[1] == "export":
                    profile = Path(argv[argv.index("-instr-profile") + 1])
                    self.assertNotIn("-empty-profile", argv)
                    exported_object = Path(argv[argv.index("-object") + 1])
                    self.assertIn(exported_object, {obj.resolve(), binary.resolve()})
                    inventory_call = exported_object == obj.resolve()
                    self.assertEqual(profile.name, "compiler-inventory-empty.profdata"
                                     if inventory_call else "merged.profdata")
                    self.assertEqual(profile.read_bytes(), b"empty inventory"
                                     if inventory_call else b"runtime counts")
                    return slicer_coverage.subprocess.CompletedProcess(
                        argv, 0, json.dumps(inventory_export if inventory_call else runtime_export), ""
                    )
                self.fail(f"unexpected command {argv}")

            with mock.patch.object(slicer_coverage, "_which", side_effect=lambda name, explicit: name), \
                 mock.patch.object(slicer_coverage, "_run", side_effect=run):
                status = slicer_coverage.main([
                    "run", "--root", str(root), "--build-dir", str(build),
                    "--source-root", str(source_root), "--object", str(binary),
                    "--manifest", str(manifest), "--compile-commands", str(compile_commands),
                    "--test-command", "fixture-tests", "--out", str(root / "report"),
                ])

            self.assertEqual(status, 0)
            report = json.loads((root / "report" / "coverage-report.json").read_text())
            runtime_profiles = report["profiles"]
            inventory = report["compiler_inventory_profile"]
            self.assertEqual(report["summary"]["lines"]["covered"], 1)
            self.assertEqual(report["summary"]["lines"]["count"], 1)
            self.assertEqual(len(runtime_profiles), 1)
            self.assertNotEqual(runtime_profiles[0], inventory["profile"])
            self.assertEqual(inventory["purpose"],
                             "compiler object coverage-map inventory; runtime hit counts are not used")
            self.assertNotIn("inventory_profile", report["compiler_evidence"])
            merge_calls = [call for call in calls if call[:2] == ["llvm-profdata", "merge"]]
            self.assertEqual(len(merge_calls), 2)
            export_calls = [call for call in calls if call[:2] == ["llvm-cov", "export"]]
            self.assertEqual(len(export_calls), 2)

    def test_profile_based_object_audit_retains_zero_hit_mappings(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "never_called.cpp"
            source.write_text("int never_called() { return 1; }\n")
            obj = root / "never_called.o"
            obj.touch()
            inventory_profile = root / "compiler-inventory-empty.profdata"
            inventory_profile.write_bytes(b"indexed inventory profile")
            commands = root / "compile_commands.json"
            commands.write_text(json.dumps([{
                "directory": str(root), "file": str(source), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(source), "-o", str(obj),
                ],
            }]))
            fake_cov = root / "llvm18.py"
            fake_cov.write_text(
                "#!/usr/bin/env python3\n"
                "import json, pathlib, sys\n"
                "assert '-empty-profile' not in sys.argv\n"
                "assert pathlib.Path(sys.argv[sys.argv.index('-instr-profile') + 1]).is_file()\n"
                f"print(json.dumps({{'data': [{{'files': [{{'filename': {str(source)!r}, "
                "'summary': {'lines': {'covered': 0, 'count': 1}}}]}]}))\n"
            )
            fake_cov.chmod(0o755)
            evidence = slicer_coverage._compiler_source_evidence(
                str(fake_cov), [source], commands, timeout=5,
                inventory_profile=inventory_profile,
            )
            self.assertEqual(evidence["status"], "ok")
            self.assertEqual(evidence["mapped_sources"], [str(source)])
            self.assertEqual(evidence["mapping_free_sources"], [])
            self.assertEqual(evidence["object_executable_mappings"], [slicer_coverage._path_key(source)])

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
            inventory_profile = root / "compiler-inventory-empty.profdata"
            inventory_profile.write_bytes(b"indexed inventory profile")
            compile_commands = root / "compile_commands.json"
            compile_commands.write_text(json.dumps([
                {"directory": str(root), "file": str(mapped), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(mapped), "-o", str(mapped_object),
                ]},
                {"directory": str(root), "file": str(mapping_free), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(mapping_free), "-o", str(mapping_free_object),
                ]},
                {"directory": str(root), "file": str(header_only), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
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
                inventory_profile=inventory_profile,
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
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(source), "-o", str(obj),
                ],
            }]), encoding="utf-8")
            timeout_result = slicer_coverage.subprocess.CompletedProcess(
                ["llvm-cov"], 124, "", "error: no coverage data found\n"
            )
            inventory_profile = root / "compiler-inventory-empty.profdata"
            inventory_profile.write_bytes(b"indexed inventory profile")
            with mock.patch.object(slicer_coverage, "_run", return_value=timeout_result):
                evidence = slicer_coverage._compiler_source_evidence(
                    "llvm-cov", [source], compile_commands, timeout=1,
                    inventory_profile=inventory_profile,
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


class CoverageDiagnosticTests(unittest.TestCase):
    def test_hash_mismatch_fails_gate_even_when_export_succeeds_at_full_coverage(self):
        import subprocess
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "src/libslic3r/fixture.cpp"
            source.parent.mkdir(parents=True)
            source.write_text("int fixture() { return 1; }\n")
            binary = root / "fixture-tests"
            binary.touch()
            obj = root / "fixture.o"
            obj.touch()
            manifest = root / "sources.txt"
            manifest.write_text(str(source) + "\n")
            commands = root / "compile_commands.json"
            commands.write_text(json.dumps([{
                "directory": str(root), "file": str(source), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c", str(source), "-o", str(obj),
                ],
            }]))
            metric = {"covered": 1, "count": 1, "percent": 100.0}
            exported = {"data": [{"files": [{"filename": str(source), "summary": {
                "lines": metric, "functions": metric, "branches": metric,
            }}]}]}

            def run(argv, **kwargs):
                stdout, stderr = "", ""
                if argv[0] == "fixture-tests":
                    profile = kwargs["env"]["LLVM_PROFILE_FILE"].replace("%4m", "fixture")
                    Path(profile).write_bytes(b"profile")
                    stdout = "100% tests passed, 0 tests failed out of 1\n"
                elif "merge" in argv:
                    Path(argv[argv.index("-o") + 1]).write_bytes(b"indexed profile")
                    stdout = "profile merge notice\n"
                    stderr = "profile merge diagnostic\n"
                elif "export" in argv:
                    stdout = json.dumps(exported)
                    stderr = "warning: 1 functions have mismatched data\n"
                elif "--version" in argv:
                    stdout = "fixture LLVM\n"
                else:
                    self.fail(f"unexpected command {argv}")
                return subprocess.CompletedProcess(argv, 0, stdout, stderr)

            with mock.patch.object(slicer_coverage, "_coverage_provenance", return_value={}), \
                 mock.patch.object(slicer_coverage, "_which", side_effect=lambda name, explicit: name), \
                 mock.patch.object(slicer_coverage, "_run", side_effect=run):
                status = slicer_coverage.main([
                    "run", "--root", str(root), "--object", str(binary),
                    "--manifest", str(manifest), "--compile-commands", str(commands),
                    "--test-command", "fixture-tests", "--out", "report",
                ])
            self.assertEqual(status, 1)
            report = json.loads((root / "report/coverage-report.json").read_text())
            self.assertEqual(report["status"], "coverage_integrity_error")
            self.assertEqual(report["integrity_status"], "error")
            self.assertEqual(report["integrity_failures"]["profile_mapping"],
                             ["warning: 1 functions have mismatched data"])
            self.assertEqual(report["export"]["stderr"], "warning: 1 functions have mismatched data\n")
            self.assertEqual(report["merge"]["stdout"], "profile merge notice\n")
            self.assertEqual(report["merge"]["stderr"], "profile merge diagnostic\n")
            self.assertNotIn("stdout", report["export"])  # The large JSON export has its own file.
            self.assertEqual(json.loads((root / "report/coverage-export.json").read_text()), exported)

    def test_unrelated_diagnostics_are_not_profile_mismatches(self):
        for diagnostics in ("", "informational diagnostic\n",
                            "warning: 0 functions have mismatched data\n"):
            with self.subTest(diagnostics=diagnostics):
                self.assertEqual(slicer_coverage._profile_mapping_errors(diagnostics), [])

    def test_mismatched_object_is_not_classified_as_mapping_free(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "omitted.cpp"
            source.write_text("int omitted() { return 1; }\n")
            obj = root / "omitted.o"
            obj.touch()
            commands = root / "compile_commands.json"
            commands.write_text(json.dumps([{
                "directory": str(root), "file": str(source), "arguments": [
                    "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                    "-fprofile-update=atomic", "-c",
                    str(source), "-o", str(obj),
                ],
            }]))
            result = slicer_coverage.subprocess.CompletedProcess(
                ["llvm-cov"], 0, '{"data": [{"files": []}]}',
                "warning: 3878 functions have mismatched data\n",
            )
            inventory_profile = root / "compiler-inventory-empty.profdata"
            inventory_profile.write_bytes(b"indexed inventory profile")
            with mock.patch.object(slicer_coverage, "_run", return_value=result):
                evidence = slicer_coverage._compiler_source_evidence(
                    "llvm-cov", [source], commands, timeout=1,
                    inventory_profile=inventory_profile,
                )
            self.assertEqual(evidence["status"], "error")
            self.assertEqual(evidence["mapping_free_sources"], [])
            self.assertEqual(evidence["errors"][0]["reason"], "compiler_profile_mapping_error")


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
