#!/usr/bin/env python3
"""Focused stdlib tests for the pair-aware collector."""

from pathlib import Path
import hashlib
import json
import sys
import tempfile
import unittest
from unittest import mock


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import slicer_coverage as sc  # noqa: E402
import slicer_coverage_pair_collector as collector  # noqa: E402


class PairCollectorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.source_root = self.root / "src" / "libslic3r"
        self.source_root.mkdir(parents=True)
        self.source = self.source_root / "fixture.cpp"
        self.source.write_text("int fixture() {\nreturn 1; }\n", encoding="utf-8")
        self.header = self.source_root / "metadata_only.h"
        self.header.write_text("#pragma once\n", encoding="utf-8")
        self.build = self.root / "build"
        self.build.mkdir()
        self.binary = self.build / "fixture-tests"
        self.binary.write_bytes(b"fixture executable")
        self.obj = self.build / "fixture.o"
        self.obj.write_bytes(b"fixture object")
        self.raw = self.build / "fixture.profraw"
        self.raw.write_bytes(b"raw profile bytes")
        self.source_manifest = self.build / "sources.txt"
        self.source_manifest.write_text("src/libslic3r/fixture.cpp\n", encoding="utf-8")
        self.compile_commands = self.build / "compile_commands.json"
        self.compile_commands.write_text(json.dumps([{
            "directory": str(self.build),
            "file": str(self.source),
            "arguments": [
                "clang", "-fprofile-instr-generate", "-fcoverage-mapping",
                "-fprofile-update=atomic", "-c", str(self.source), "-o", str(self.obj),
            ],
        }]), encoding="utf-8")
        self.pair_manifest = self.build / "pairs.json"
        self.write_pair_manifest()

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def sha(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def row(self, binary=None, profile=None):
        binary = binary or self.binary
        profile = profile or self.raw
        return {
            "object": str(binary),
            "object_sha256": self.sha(binary),
            "profiles": [{"path": str(profile), "sha256": self.sha(profile)}],
        }

    def write_pair_manifest(self, rows=None):
        self.pair_manifest.write_text(json.dumps({
            "schema": 1, "pairs": rows if rows is not None else [self.row()],
        }), encoding="utf-8")

    def args(self, out=None):
        out = out or self.root / "report"
        return [
            "--root", str(self.root),
            "--source-root", str(self.root / "src"),
            "--build-dir", str(self.build),
            "--compile-commands", str(self.compile_commands),
            "--manifest", str(self.source_manifest),
            "--pairs", str(self.pair_manifest),
            "--out", str(out),
        ]

    def fake_tools(self, *, mismatch=False, mutate_input=None, invalid_summary=False,
                   zero_execution=False, union_missing=None, union_extra=False,
                   mapping_free_header=False, stale_cold_mapping=False):
        source = str(self.source)
        json_export = json.dumps({"data": [{"files": [{
            "filename": source,
            "summary": {
                "lines": {"count": 3, "covered": 2, "percent": 66.67},
                "functions": {"count": 2, "covered": 1, "percent": 50.0},
                "regions": {"count": 3, "covered": 2, "percent": 66.67},
                "branches": {"count": 0, "covered": 0, "percent": 100.0},
                "instantiations": {"count": 0, "covered": 0, "percent": 100.0},
            },
        }], "totals": {
            "lines": {"count": 3, "covered": 0 if zero_execution else 2},
            "regions": {"count": 3, "covered": 0 if zero_execution else 2},
            "functions": {"count": 2, "covered": 0 if zero_execution else 1},
            "branches": {"count": 0, "covered": 0},
            "instantiations": {"count": 0, "covered": 0},
        }}]})
        hit = 0 if zero_execution else 1
        lcov_trace = f"TN:\nSF:{source}\nDA:1,{hit}\nDA:2,{hit}\nend_of_record\n"
        if stale_cold_mapping:
            lcov_trace += f"TN:\nSF:{source}\nDA:4056,0\nend_of_record\n"
        if mapping_free_header:
            lcov_trace += f"TN:\nSF:{self.header}\nend_of_record\n"
        calls = []
        mutated = False

        def run(argv, **kwargs):
            nonlocal mutated
            calls.append(list(argv))
            if mutate_input is not None and not mutated:
                mutate_input.write_bytes(mutate_input.read_bytes() + b"\n")
                mutated = True
            if argv[0] == "llvm-profdata" and argv[1] == "merge":
                output = Path(argv[argv.index("-o") + 1])
                output.write_bytes(b"indexed profile")
                return sc.subprocess.CompletedProcess(argv, 0, "", "")
            if argv[0] == "llvm-cov" and argv[1] == "export":
                if "-format=lcov" in argv:
                    return sc.subprocess.CompletedProcess(
                        argv, 0, lcov_trace,
                        "warning: 1 functions have mismatched data\n" if mismatch else "",
                    )
                return sc.subprocess.CompletedProcess(
                    argv, 0, json_export,
                    "warning: 1 functions have mismatched data\n" if mismatch else "",
                )
            if argv[0] == "lcov" and "--extract" in argv:
                output = Path(argv[argv.index("--output-file") + 1])
                output.write_text(lcov_trace, encoding="utf-8")
                return sc.subprocess.CompletedProcess(argv, 0, "", "")
            if argv[0] == "lcov" and "--add-tracefile" in argv:
                output = Path(argv[argv.index("--output-file") + 1])
                union_trace = lcov_trace
                if union_missing == "file":
                    union_trace = "TN:\nend_of_record\n"
                elif union_missing == "line":
                    union_trace = union_trace.replace("DA:2,1\n", "")
                if union_extra:
                    union_trace += f"TN:\nSF:{source}\nDA:3,0\nend_of_record\n"
                output.write_text(union_trace, encoding="utf-8")
                return sc.subprocess.CompletedProcess(argv, 0, "", "")
            if argv[0] == "lcov" and "--summary" in argv:
                coverage_summary = "4 of 3" if invalid_summary else ("0 of 3" if zero_execution else "2 of 3")
                return sc.subprocess.CompletedProcess(
                    argv, 0,
                    f"Summary coverage rate:\n  lines......: 66.67% ({coverage_summary} lines)\n",
                    "",
                )
            self.fail(f"unexpected tool command: {argv}")

        return calls, run

    def run_collector(self, *, mismatch=False, out=None, mutate_input=None, invalid_summary=False,
                      zero_execution=False, union_missing=None, union_extra=False,
                      mapping_free_header=False, stale_cold_mapping=False):
        calls, run = self.fake_tools(
            mismatch=mismatch, mutate_input=mutate_input, invalid_summary=invalid_summary,
            zero_execution=zero_execution, union_missing=union_missing,
            union_extra=union_extra, mapping_free_header=mapping_free_header,
            stale_cold_mapping=stale_cold_mapping,
        )
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args(out))
        return status, calls

    def test_pair_exports_use_only_the_bound_profile_and_standard_lcov_union(self):
        status, calls = self.run_collector()
        self.assertEqual(status, 0)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        self.assertEqual(report["status"], "observed")
        self.assertEqual(report["test_status"], "not_checked")
        self.assertFalse(report["coverage_claim"])
        self.assertFalse(report["threshold_claim"])
        self.assertEqual(report["line_metric"], {
            "percent": 100.0 * 2 / 3,
            "reported_percent": 66.67,
            "covered": 2,
            "count": 3,
        })
        self.assertTrue(report["llvm_summary_note"].startswith("Per-object LLVM JSON summaries"))
        merge = next(call for call in calls if call[:2] == ["llvm-profdata", "merge"]
                     and "compiler-inventory-empty.proftext" not in call[-1])
        self.assertEqual(merge[-1], str(self.raw.resolve()))
        cov_exports = [call for call in calls if call[:2] == ["llvm-cov", "export"]]
        runtime_exports = [call for call in cov_exports
                           if "compiler-inventory-empty.profdata" not in " ".join(call)]
        self.assertTrue(all(call[call.index("-object") + 1] == str(self.binary.resolve())
                            for call in runtime_exports))
        union = next(call for call in calls if call[0] == "lcov" and "--add-tracefile" in call)
        self.assertEqual(union.count("--add-tracefile"), 1)
        self.assertTrue((self.root / "report" / "pair-000" / "llvm-export.lcov").is_file())
        self.assertTrue((self.root / "report" / "coverage-union.info").is_file())
        self.assertEqual(report["lcov_union_line_audit"], {
            "status": "pass",
            "expected_identity_count": 2,
            "union_identity_count": 2,
            "missing_identity_count": 0,
            "unexpected_identity_count": 0,
            "missing_identity_samples": [],
            "unexpected_identity_samples": [],
        })

    def test_stale_zero_hit_mapping_fails_even_when_union_retains_every_identity(self):
        status, _ = self.run_collector(stale_cold_mapping=True)
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report/coverage-report.json").read_text())
        self.assertEqual(report["lcov_union_line_audit"]["status"], "pass")
        self.assertEqual(report["integrity_status"], "error")
        self.assertEqual(report["integrity_failures"]["source_line_bounds"], [{
            "source": str(self.source.resolve()), "reason": "line_outside_source", "line_count": 2,
            "invalid_identity_count": 1, "line_samples": [4056],
        }])
        self.assertIn("DA:4056,0", (self.root / "report/coverage-union.info").read_text())

    def test_source_bounds_count_crlf_and_unterminated_final_line_and_bind_bytes(self):
        self.source.write_bytes(b"first\r\nsecond")
        audit = collector._source_line_bounds(
            {(str(self.source), 1), (str(self.source), 2)}, source_root=self.source_root)
        self.assertEqual(audit["status"], "pass")
        self.assertEqual(audit["sources"], [{"path": str(self.source),
            "sha256": self.sha(self.source), "size_bytes": 13, "line_count": 2}])

    def test_empty_source_cannot_have_executable_lines_including_line_zero(self):
        self.source.write_bytes(b"")
        audit = collector._source_line_bounds(
            {(str(self.source), 0), (str(self.source), 1)}, source_root=self.source_root)
        self.assertEqual(audit["status"], "error")
        self.assertEqual(audit["failures"][0]["line_samples"], [0, 1])
        self.assertEqual(audit["failures"][0]["line_count"], 0)

    def test_missing_source_is_an_integrity_failure(self):
        self.source.unlink()
        audit = collector._source_line_bounds({(str(self.source), 1)}, source_root=self.source_root)
        self.assertEqual(audit["status"], "error")
        self.assertEqual(audit["failures"][0]["reason"], "source_unreadable")
        invalid_source = str(self.source_root / "invalid\0.cpp")
        invalid = collector._source_line_bounds({(invalid_source, 1)}, source_root=self.source_root)
        self.assertEqual(invalid["status"], "error")
        self.assertEqual(invalid["sources"], [])

    def test_source_bounds_reject_outside_and_symlink_escape_before_opening(self):
        outside = self.root / "outside.cpp"
        outside.write_bytes(b"outside\n")
        linked = self.source_root / "linked.cpp"
        linked.symlink_to(outside)
        for source in (outside, linked):
            with self.subTest(source=source), mock.patch.object(Path, "open") as opened:
                audit = collector._source_line_bounds({(str(source), 1)}, source_root=self.source_root)
                opened.assert_not_called()
                self.assertEqual(audit["status"], "error")
                self.assertEqual(audit["failures"][0]["reason"], "source_outside_root")

    def test_union_missing_source_file_fails_identity_audit(self):
        status, _ = self.run_collector(union_missing="file")
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        audit = report["lcov_union_line_audit"]
        self.assertEqual(audit["status"], "error")
        self.assertEqual(audit["missing_identity_count"], 2)
        self.assertTrue(all(item["source"] == str(self.source.resolve())
                            for item in audit["missing_identity_samples"]))

    def test_union_missing_da_line_fails_identity_audit(self):
        status, _ = self.run_collector(union_missing="line")
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        audit = report["lcov_union_line_audit"]
        self.assertEqual(audit["status"], "error")
        self.assertEqual(audit["missing_identity_count"], 1)
        self.assertEqual(audit["missing_identity_samples"], [
            {"source": str(self.source.resolve()), "line": 2},
        ])

    def test_union_unexpected_da_line_fails_identity_audit(self):
        status, _ = self.run_collector(union_extra=True)
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        audit = report["lcov_union_line_audit"]
        self.assertEqual(audit["status"], "error")
        self.assertEqual(audit["unexpected_identity_count"], 1)
        self.assertEqual(audit["unexpected_identity_samples"], [
            {"source": str(self.source.resolve()), "line": 3},
        ])

    def test_mapping_free_header_is_reported_without_failing_integrity(self):
        status, _ = self.run_collector(mapping_free_header=True)
        self.assertEqual(status, 0)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        self.assertEqual(report["status"], "observed")
        self.assertEqual(report["mapping_free_headers"], [str(self.header.resolve())])
        self.assertNotIn("mapping_free_headers", report["integrity_failures"])

    def test_any_llvm_profile_mapping_warning_fails_closed_but_keeps_lcov_metric(self):
        status, _ = self.run_collector(mismatch=True)
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        self.assertEqual(report["status"], "coverage_integrity_error")
        self.assertEqual(report["line_metric"]["count"], 3)
        self.assertGreaterEqual(len(report["integrity_failures"]["profile_mapping"]), 1)
        self.assertTrue(all("mismatched data" in warning
                            for item in report["integrity_failures"]["profile_mapping"]
                            for warning in item["diagnostics"]))

    def test_zero_execution_pair_fails_closed_even_when_exports_are_valid(self):
        status, _ = self.run_collector(zero_execution=True)
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        self.assertEqual(report["line_metric"]["covered"], 0)
        self.assertFalse(report["pairs"][0]["has_profile_execution"])
        self.assertIn("profile_has_zero_execution", [
            item["reason"] for item in report["integrity_failures"]["collector_errors"]
        ])

    def test_wrong_object_hash_is_rejected_before_tools_run(self):
        row = self.row()
        row["object_sha256"] = "0" * 64
        self.write_pair_manifest([row])
        calls, run = self.fake_tools()
        out = self.root / "report"
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args(out))
        self.assertEqual(status, 2)
        self.assertEqual(calls, [])
        self.assertFalse(out.exists())

    def test_profile_cannot_be_ambiguously_associated_twice(self):
        second_binary = self.build / "other-tests"
        second_binary.write_bytes(b"other test binary")
        self.write_pair_manifest([self.row(), self.row(binary=second_binary)])
        calls, run = self.fake_tools()
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args())
        self.assertEqual(status, 2)
        self.assertEqual(calls, [])
        self.assertFalse((self.root / "report").exists())

    def test_existing_nonempty_output_is_rejected_without_mutation(self):
        out = self.root / "occupied"
        out.mkdir()
        sentinel = out / "sentinel.txt"
        sentinel.write_text("keep", encoding="utf-8")
        calls, run = self.fake_tools()
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args(out))
        self.assertEqual(status, 2)
        self.assertEqual(sentinel.read_text(encoding="utf-8"), "keep")
        self.assertEqual(calls, [])

    def test_empty_output_symlink_is_rejected_before_resolving_its_target(self):
        target = self.root / "empty-target"
        target.mkdir()
        link = self.root / "linked-report"
        link.symlink_to(target, target_is_directory=True)
        calls, run = self.fake_tools()
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args(link))
        self.assertEqual(status, 2)
        self.assertEqual(calls, [])
        self.assertEqual(list(target.iterdir()), [])

    def test_relative_parent_traversal_is_rejected(self):
        calls, run = self.fake_tools()
        with mock.patch.object(collector.sc, "_which", side_effect=lambda name, explicit=None: name), \
             mock.patch.object(collector.sc, "_run", side_effect=run):
            status = collector.main(self.args("../escaped-report"))
        self.assertEqual(status, 2)
        self.assertEqual(calls, [])
        self.assertFalse((self.root.parent / "escaped-report").exists())

    def test_invalid_lcov_line_counts_fail_and_input_mutation_is_reported(self):
        status, _ = self.run_collector(invalid_summary=True)
        self.assertEqual(status, 2)
        report = json.loads((self.root / "report" / "coverage-report.json").read_text())
        self.assertIsNone(report["line_metric"])
        self.assertTrue(report["integrity_failures"]["empty_line_metric"])

        self.tearDown()
        self.setUp()
        for index, (label, field) in enumerate((
            ("profile", "raw"),
            ("compile_commands", "compile_commands"),
            ("source_manifest", "source_manifest"),
            ("pair_manifest", "pair_manifest"),
        )):
            if index:
                self.tearDown()
                self.setUp()
            path = getattr(self, field)
            with self.subTest(input=label):
                status, _ = self.run_collector(out=self.root / f"report-{label}", mutate_input=path)
                self.assertEqual(status, 2)
                report = json.loads((self.root / f"report-{label}" / "coverage-report.json").read_text())
                self.assertEqual(report["input_stability"]["status"], "error")
                self.assertIn("coverage_input_changed_during_collection", [
                    item["reason"] for item in report["integrity_failures"]["collector_errors"]
                ])


if __name__ == "__main__":
    unittest.main()
