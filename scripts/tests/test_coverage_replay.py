from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import sys
import tempfile
import unittest
from contextlib import redirect_stderr
from contextlib import redirect_stdout
from unittest.mock import patch

REPO = next(parent for parent in Path(__file__).resolve().parents
            if (parent / "AGENTS.md").is_file())
SCRIPTS = Path(__file__).resolve().parents[1]
PACKAGE = SCRIPTS / "coverage_replay"
sys.path.insert(0, str(SCRIPTS))
sys.path.insert(0, str(PACKAGE))

import all_kinds_matcher as all_kinds  # noqa: E402
import coverage_warning_matcher as warning_matcher  # noqa: E402
import elf_covfun_identity as inspector  # noqa: E402
import strict_llvm18_mapping as strict_decoder  # noqa: E402
import strict_warning_matcher as strict_matcher  # noqa: E402

SPEC = importlib.util.spec_from_file_location(
    "slicer_coverage_reconcile_candidate", SCRIPTS / "slicer_coverage_reconcile.py")
assert SPEC is not None and SPEC.loader is not None
reconcile = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = reconcile
SPEC.loader.exec_module(reconcile)


def sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def descriptor(path: Path) -> dict[str, str]:
    return {"path": str(path), "sha256": sha(path)}


def json_roundtrip(value):
    return json.loads(json.dumps(value, sort_keys=True))


def _region(kind: int, interval: tuple[int, int, int, int], *, expanded: int = 0,
            true: int = 0, false: int = 0):
    counter = all_kinds.decoder.Counter
    true_counter = counter("zero", 0) if true == 0 else counter("counter", true)
    false_counter = counter("zero", 0) if false == 0 else counter("counter", false)
    return all_kinds.decoder.Region(
        file_id=0, expanded_file_id=expanded, line_start=interval[0],
        column_start=interval[1], line_end=interval[2], column_end=interval[3],
        kind=kind,
        counter=true_counter if kind == all_kinds.BRANCH else counter("zero", 0),
        false_counter=false_counter if kind == all_kinds.BRANCH else counter("zero", 0),
    )


def all_kind_fixture():
    intervals = [(1, 1, 1, 4), (2, 1, 2, 4), (3, 1, 3, 4), (4, 1, 4, 4), (5, 1, 5, 4)]
    decoded = {"regions": [
        _region(all_kinds.CODE, intervals[0]),
        _region(all_kinds.EXPANSION, intervals[1], expanded=1),
        _region(all_kinds.SKIPPED, intervals[2]),
        _region(all_kinds.GAP, intervals[3]),
        _region(all_kinds.BRANCH, intervals[4], true=3, false=1),
    ]}
    function = {
        "_filenames": ["/tmp/unit.cpp", "/tmp/macro.hpp"],
        "_regions": [
            [1, 1, 1, 4, 1, 0, 0, all_kinds.CODE],
            [2, 1, 2, 4, 1, 0, 1, all_kinds.EXPANSION],
            [3, 1, 3, 4, 1, 0, 0, all_kinds.SKIPPED],
            [4, 1, 4, 4, 1, 0, 0, all_kinds.GAP],
        ],
        "branches": [[5, 1, 5, 4, 3, 1, 0, 0, all_kinds.BRANCH]],
        "mcdc_records": [],
    }
    return decoded, function


def _uleb(value: int) -> bytes:
    output = bytearray()
    while True:
        byte = value & 0x7f
        value >>= 7
        if value:
            output.append(byte | 0x80)
        else:
            output.append(byte)
            return bytes(output)


def _full_mapping() -> bytes:
    values = [
        2, 0, 1,
        1, 1, 0,
        5,
        1, 10, 2, 0, 12,
        0, 11, 0, 0, 0x80000005,
        32, 1, 1, 12, 1, 0, 7,
        12, 14, 1, 0, 9,
        16, 15, 1, 0, 9,
        1,
        3, 17, 1, 0, 4,
    ]
    return b"".join(_uleb(value) for value in values)


def _used_mapping_document(mapping_hex: str):
    name = "_Z3foov"
    name_hash = f"{warning_matcher._name_hash(name):X}"
    path = "/tu/one.cpp"
    ref = "0x1"
    table_ref = f"0x{int(ref, 16):016X}"
    dummy_interval = [10, 2, 10, 12]
    dummy = {
        "symbol": f"__covrec_{name_hash}", "used_suffix": False,
        "dummy": True, "function_hash": 0, "filenames_ref": ref,
        "filename_paths": [path],
        "dummy_region": {"line_start": 10, "column_start": 2,
                         "line_end": 10, "column_end": 12},
        "mapping_shape": {"file_mappings": 1, "filename_index": 0,
                          "filename_indices": [0], "mapping_validation": "one-region-complete",
                          "dummy": True, "trailing_bytes": 0,
                          "region_coordinates": dummy_interval},
    }
    used = {
        "symbol": f"__covrec_{name_hash}u", "used_suffix": True,
        "dummy": False, "function_hash": 24, "filenames_ref": ref,
        "filename_paths": [path], "mapping_hex": mapping_hex,
        "mapping_shape": {"filename_indices": [0], "filename_index": 0},
    }
    inspector_document = {
        "schema": 2, "status": "complete", "coverage_claim": False,
        "warning_inventory": {"records": [{"name": name, "mapping_hash": 0}]},
        "binaries": [
            {"name": "dummy-only", "elf": {"covmap_filename_tables": {
                table_ref: {"filenames": [path], "filenames_ref": table_ref}}},
             "warnings": [{"name": name, "record_count": 1, "records": [dummy]}]},
            {"name": "used-definition", "elf": {"covmap_filename_tables": {
                table_ref: {"filenames": [path], "filenames_ref": table_ref}}},
             "warnings": [{"name": name, "record_count": 1, "records": [used]}]},
        ],
    }
    export_document = {
        "type": "llvm.coverage.json.export", "version": "2.0.0",
        "data": [{"functions": [{
            "name": name, "count": 0, "filenames": [path],
            "regions": [[10, 2, 10, 12, 0, 0, 0, 0]],
        }]}],
    }
    return inspector_document, export_document


class ReconcilePreflightTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.manifest_path = self.root / "manifest.json"
        self.manifest = self.make_manifest()
        self.write_manifest()

    def tearDown(self) -> None:
        self.temp.cleanup()

    def write_manifest(self) -> None:
        self.manifest_path.write_text(json.dumps(self.manifest), encoding="utf-8")

    def rewrite_reports_with_bound_hashes(self, *, runtime=None, coverage=None) -> None:
        artifacts = self.manifest["artifacts"]
        runtime_path = Path(artifacts["runtime_test_report"]["path"])
        coverage_path = Path(artifacts["coverage_report"]["path"])
        runtime_doc = json.loads(runtime_path.read_text(encoding="utf-8"))
        coverage_doc = json.loads(coverage_path.read_text(encoding="utf-8"))
        if runtime is not None:
            runtime_doc = runtime
        if coverage is not None:
            coverage_doc = coverage
        runtime_path.write_text(json.dumps(runtime_doc), encoding="utf-8")
        artifacts["runtime_test_report"]["sha256"] = sha(runtime_path)
        coverage_doc["test_report"]["sha256"] = sha(runtime_path)
        coverage_path.write_text(json.dumps(coverage_doc), encoding="utf-8")
        artifacts["coverage_report"]["sha256"] = sha(coverage_path)
        self.write_manifest()

    def make_manifest(self) -> dict:
        commit = "a" * 40
        warning = "warning: 3 functions have mismatched data"
        data_dir = self.root / "inputs"
        data_dir.mkdir()
        runtime_path = data_dir / "runtime.json"
        compile_path = data_dir / "compile_commands.json"
        manifest_path = data_dir / "libslic3r.sources.txt"
        cgal_path = data_dir / "libslic3r_cgal.sources.txt"
        merged_path = data_dir / "merged.profdata"
        export_path = data_dir / "coverage-export.json"
        stderr_path = data_dir / "export.stderr.txt"
        source_path = data_dir / "AABBMesh.cpp"
        llvm_path = data_dir / "llvm-cov"
        for path, content in (
            (compile_path, b"[]"), (manifest_path, b"source-a.cpp\n"),
            (cgal_path, b"source-b.cpp\n"), (merged_path, b"merged profile"),
            (export_path, b'{"data":[{"functions":[]}]}'),
            (stderr_path, (warning + "\n").encode()), (source_path, b"int source_filter;\n"),
            (llvm_path, b"fake llvm executable"),
        ):
            path.write_bytes(content)
        llvm_path.chmod(0o755)
        hashes = {}
        binary_rows = []
        for name in reconcile.EXPECTED_OBJECTS:
            path = data_dir / f"{name}_tests"
            path.write_bytes(f"ELF stand-in {name}".encode())
            digest = sha(path)
            hashes[f"/worker/build/tests/{name}/{name}_tests"] = digest
            binary_rows.append({"name": name, **descriptor(path)})
        runtime = {
            "schema": 3, "status": "pass", "expected_commit": commit,
            "junit_test_count": 8, "minimum_tests": 8,
            "failed_tests": [], "error_tests": [], "skipped_tests": [],
            "binary_sha256_before": hashes, "binary_sha256_after": hashes,
            "profile_inventory": [
                {"path": "/profiles/a.profraw", "sha256": "1" * 64, "size_bytes": 11},
                {"path": "/profiles/b.profraw", "sha256": "2" * 64, "size_bytes": 22},
            ],
            "source_sha256_before": {"src/file.cpp": "3" * 64},
            "source_sha256_after": {"src/file.cpp": "3" * 64},
            "commit_before": commit, "commit_after": commit,
            "tree_before": "b" * 40, "tree_after": "b" * 40,
            "tracked_before": "", "tracked_after": "",
        }
        runtime_path.write_text(json.dumps(runtime), encoding="utf-8")
        merged_hash = sha(merged_path)
        compile_hash = sha(compile_path)
        manifest_hashes = [sha(manifest_path), sha(cgal_path)]
        coverage_path = data_dir / "coverage-report.json"
        summary = {"lines": {"count": 20, "covered": 15, "percent": 75.0}}
        coverage = {
            "expected_commit": commit, "junit_test_count": 8, "minimum_tests": 8,
            "status": "coverage_integrity_error", "coverage_claim": False,
            "provisional": True, "integrity_status": "error",
            "integrity_failures": {
                "profile_mapping": [warning], "other": [],
            },
            "raw_summary": summary, "summary": summary,
            "test_report": {"path": str(runtime_path), "sha256": sha(runtime_path)},
            "profiles": runtime["profile_inventory"],
            "checkout_before": {"head": commit, "tracked_clean": True, "tracked_changes": []},
            "checkout_after": {"head": commit, "tracked_clean": True, "tracked_changes": []},
            "merged_profile": {"sha256": merged_hash},
            "compile_commands": [{"sha256": compile_hash}],
            "manifest_inventory": [{"sha256": item} for item in manifest_hashes],
            "binary_sha256_before_test": hashes,
            "binary_sha256_after_test": hashes,
            "binary_sha256_before_export": hashes,
            "binary_sha256_after_export": hashes,
        }
        coverage_path.write_text(json.dumps(coverage), encoding="utf-8")
        helper_rows = {
            name: descriptor(path) for name, path in reconcile.HELPER_FILES.items()
        }
        return {
            "schema": 1,
            "contract": {
                "expected_commit": commit, "expected_tests": 8,
                "profile_mapping_warning": warning, "warning_occurrences": 3,
            },
            "paths": {"report_binary_root": "/worker/build/tests",
                      "working_directory": str(self.root)},
            "tools": {"llvm_cov": descriptor(llvm_path)},
            "helpers": helper_rows,
            "artifacts": {
                "runtime_test_report": descriptor(runtime_path),
                "coverage_report": descriptor(coverage_path),
                "coverage_export_summary": descriptor(export_path),
                "export_stderr": descriptor(stderr_path),
                "merged_profile": descriptor(merged_path),
                "compile_commands": descriptor(compile_path),
                "source_manifest_libslic3r": descriptor(manifest_path),
                "source_manifest_cgal": descriptor(cgal_path),
                "llvm_cov": descriptor(llvm_path),
                "source_filter": descriptor(source_path),
            },
            "binary_objects": binary_rows,
        }

    def assert_rejected_before_subprocess(self) -> str:
        output = self.root / "out"
        captured = io.StringIO()
        with patch.object(reconcile, "run_captured") as runner, redirect_stderr(captured):
            self.assertEqual(reconcile.main(["--manifest", str(self.manifest_path),
                                             "--out", str(output)]), 1)
        runner.assert_not_called()
        self.assertFalse(output.exists())
        return captured.getvalue()

    def test_missing_helper_fails_before_any_subprocess(self) -> None:
        row = self.manifest["helpers"]["all_kind_matcher"]
        row["path"] = str(PACKAGE / "not_deployed.py")
        row["sha256"] = "0" * 64
        self.write_manifest()
        self.assertIn("helper", self.assert_rejected_before_subprocess())

    def test_valid_manifest_preflights_without_running_tools(self) -> None:
        prepared = reconcile.preflight(self.manifest_path)
        self.assertEqual(prepared["commit"], "a" * 40)
        self.assertEqual(prepared["test_count"], 8)
        self.assertEqual(len(prepared["binary_hashes"]), 5)
        self.assertEqual(prepared["input_hashes"][str(self.manifest_path.resolve())], sha(self.manifest_path))
        self.assertFalse(prepared["manifest"].get("coverage_claim", False))
        import replay_controller
        replay_controller.configure_run(self.manifest_path)
        with redirect_stdout(io.StringIO()):
            replay_controller.self_check()

    def test_stale_commit_is_rejected_pre_llvm(self) -> None:
        self.manifest["contract"]["expected_commit"] = "b" * 40
        self.write_manifest()
        self.assertIn("runtime report is not", self.assert_rejected_before_subprocess())

    def test_wrong_test_count_is_rejected_pre_llvm(self) -> None:
        self.manifest["contract"]["expected_tests"] = 9
        self.write_manifest()
        self.assertIn("runtime report is not", self.assert_rejected_before_subprocess())

    def test_input_hash_mismatch_is_rejected_pre_llvm(self) -> None:
        self.manifest["artifacts"]["merged_profile"]["sha256"] = "0" * 64
        self.write_manifest()
        self.assertIn("merged_profile hash mismatch", self.assert_rejected_before_subprocess())

    def test_warning_count_disagreement_is_rejected_before_subprocess(self) -> None:
        self.manifest["contract"]["warning_occurrences"] = 4
        self.write_manifest()
        self.assertIn("warning_occurrences", self.assert_rejected_before_subprocess())

    def test_runtime_profile_inventory_conflict_is_rejected_with_rebound_hashes(self) -> None:
        coverage_path = Path(self.manifest["artifacts"]["coverage_report"]["path"])
        coverage = json.loads(coverage_path.read_text(encoding="utf-8"))
        coverage["profiles"].reverse()
        self.rewrite_reports_with_bound_hashes(coverage=coverage)
        self.assertIn("profile inventory", self.assert_rejected_before_subprocess())

    def test_coverage_runtime_report_hash_conflict_is_rejected_with_rebound_hashes(self) -> None:
        coverage_path = Path(self.manifest["artifacts"]["coverage_report"]["path"])
        coverage = json.loads(coverage_path.read_text(encoding="utf-8"))
        coverage["test_report"]["sha256"] = "0" * 64
        coverage_path.write_text(json.dumps(coverage), encoding="utf-8")
        self.manifest["artifacts"]["coverage_report"]["sha256"] = sha(coverage_path)
        self.write_manifest()
        self.assertIn("test_report hash", self.assert_rejected_before_subprocess())

    def test_runtime_source_provenance_conflict_is_rejected_with_rebound_hashes(self) -> None:
        runtime_path = Path(self.manifest["artifacts"]["runtime_test_report"]["path"])
        runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
        runtime["source_sha256_after"]["src/file.cpp"] = "4" * 64
        self.rewrite_reports_with_bound_hashes(runtime=runtime)
        self.assertIn("source hashes changed", self.assert_rejected_before_subprocess())

    def test_runtime_commit_tree_or_dirty_checkout_conflict_is_rejected_with_rebound_hashes(self) -> None:
        runtime_path = Path(self.manifest["artifacts"]["runtime_test_report"]["path"])
        runtime = json.loads(runtime_path.read_text(encoding="utf-8"))
        runtime["tree_after"] = "c" * 40
        runtime["tracked_after"] = " M src/file.cpp"
        self.rewrite_reports_with_bound_hashes(runtime=runtime)
        output = self.assert_rejected_before_subprocess()
        self.assertTrue("runtime tree changed" in output or "runtime checkout is not clean" in output)


class MatcherAndDecoderFixtureTests(unittest.TestCase):
    def test_all_region_kinds_and_branch_geometry_match(self) -> None:
        decoded, function = all_kind_fixture()
        result = all_kinds.reconcile_signatures(decoded, ["/tmp/unit.cpp", "/tmp/macro.hpp"], function)
        self.assertEqual(result["branch_export_fallbacks"], [])
        self.assertEqual(result["export_counters"], {"regions": 4, "branches": 1, "mcdc": 0})

    def test_geometry_order_and_branch_mismatches_fail_closed(self) -> None:
        decoded, function = all_kind_fixture()
        paths = ["/tmp/unit.cpp", "/tmp/macro.hpp"]
        function["_regions"][1][1] += 1
        with self.assertRaises(all_kinds.MatcherError):
            all_kinds.reconcile_signatures(decoded, paths, function)

        decoded, function = all_kind_fixture()
        function["branches"][0][0] += 1
        with self.assertRaises(all_kinds.MatcherError):
            all_kinds.reconcile_signatures(decoded, paths, function)

        decoded, function = all_kind_fixture()
        with self.assertRaises(all_kinds.MatcherError):
            all_kinds.reconcile_signatures(decoded, list(reversed(paths)), function)

    def test_mcdc_and_unexplained_nonzero_branch_are_rejected(self) -> None:
        decoded, function = all_kind_fixture()
        decoded["regions"].append(_region(all_kinds.MCDC_DECISION, (6, 1, 6, 4)))
        with self.assertRaises(all_kinds.MatcherError):
            all_kinds.reconcile_signatures(decoded, ["/tmp/unit.cpp", "/tmp/macro.hpp"], function)

        decoded, function = all_kind_fixture()
        function["branches"] = []
        with self.assertRaises(all_kinds.MatcherError):
            all_kinds.reconcile_signatures(decoded, ["/tmp/unit.cpp", "/tmp/macro.hpp"], function)

    def test_strict_decoder_preserves_filename_region_and_expansion_order(self) -> None:
        decoded = strict_decoder.decode_mapping(_full_mapping(), ["/tu/one.cpp", "/tu/two.cpp"])
        self.assertEqual(decoded["virtual_filename_indices"], [0, 1])
        self.assertEqual([region.kind for region in decoded["regions"]], [0, 3, 4, 1, 2, 0])
        self.assertEqual(decoded["regions"][3].expanded_file_id, 1)
        self.assertEqual(decoded["regions"][0].interval, (10, 2, 10, 12))

    def test_strict_decoder_rejects_trailing_and_overflow_data(self) -> None:
        matching = b"".join(_uleb(value) for value in (1, 0, 0, 1, 1, 10, 2, 0, 12))
        with self.assertRaises(strict_decoder.MappingError):
            strict_decoder.decode_mapping(matching + b"\x00", ["/tu/one.cpp"])
        overflow = b"".join(_uleb(value) for value in (1, 0, 0, 1, 1, 10, 1 << 32, 0, 12))
        with self.assertRaises(strict_decoder.MappingError):
            strict_decoder.decode_mapping(overflow, ["/tu/one.cpp"])

    def test_tiny_elf_inspector_contract_self_check(self) -> None:
        output = io.StringIO()
        with redirect_stdout(output):
            inspector.self_check()
        result = json.loads(output.getvalue())
        self.assertEqual(result["status"], "self-check-pass")
        self.assertIs(result["coverage_claim"], False)

    def test_used_mapping_gap_fixture_remains_diagnostic_and_strict_decode_rejects(self) -> None:
        wrong_interval = b"".join(_uleb(value) for value in (1, 0, 0, 1, 1, 12, 2, 3, 7))
        shape = inspector.mapping_shape(wrong_interval, "used-mismatch", filename_count=1)
        self.assertEqual(shape["mapping_validation"], "one-region-complete")
        self.assertNotEqual(shape["region_coordinates"], [10, 2, 10, 12])

        inspector_doc, export_doc = _used_mapping_document(wrong_interval.hex())
        result = strict_matcher.strict_match(inspector_doc, export_doc)
        self.assertEqual(result["status"], "mismatch")
        self.assertIn("CodeRegion witness", result["failures"][0]["detail"])

        inspector_doc, export_doc = _used_mapping_document("80")
        with self.assertRaises(inspector.InspectorError):
            inspector.mapping_shape(bytes.fromhex("80"), "truncated-used", filename_count=1)
        result = strict_matcher.strict_match(inspector_doc, export_doc)
        self.assertEqual(result["status"], "mismatch")
        self.assertFalse(result["coverage_claim"])
        with self.assertRaises(strict_decoder.MappingError):
            strict_decoder.decode_mapping(bytes.fromhex("80"), ["/tu/one.cpp"])


class ArchivedReplayParityTests(unittest.TestCase):
    def test_a511_and_6a06_matchers_equal_archived_replay_results(self) -> None:
        archived = (
            REPO / "build/evidence/native-successor-a51192e5a2-v1/coverage-summary-v1/reconcile-v4",
            REPO / "build/evidence/native-successor-6a06b7910f-v1/coverage-summary-v1/reconcile-v3",
        )
        if not all((folder / name).is_file()
                   for folder in archived
                   for name in ("inspector.json", "structural-export.json", "replay-result.json")):
            self.skipTest("local archived a511/6a06 replay JSON is unavailable")

        import all_kinds_matcher
        import strict_warning_matcher

        for folder in archived:
            inspector = json.loads((folder / "inspector.json").read_text(encoding="utf-8"))
            structural = json.loads((folder / "structural-export.json").read_text(encoding="utf-8"))
            archived_result = json.loads((folder / "replay-result.json").read_text(encoding="utf-8"))
            self.assertEqual(json_roundtrip(strict_warning_matcher.strict_match(inspector, structural)),
                             archived_result["strict_replay"], str(folder))
            self.assertEqual(json_roundtrip(all_kinds_matcher.match(inspector, structural)),
                             archived_result["all_kind_replay"], str(folder))
            self.assertIs(archived_result["coverage_claim"], False)
            self.assertIs(archived_result["warning_waiver"], False)


if __name__ == "__main__":
    unittest.main()
