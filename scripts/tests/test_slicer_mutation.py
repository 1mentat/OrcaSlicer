#!/usr/bin/env python3
"""Unit tests for bounded mutation selection and result classification."""

from pathlib import Path
import json
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_mutation  # noqa: E402


class MutationSelectionTests(unittest.TestCase):
    def test_default_mutants_are_bounded_and_exactly_named(self):
        self.assertGreater(len(slicer_mutation.DEFAULT_MUTANTS), 0)
        self.assertLessEqual(len(slicer_mutation.DEFAULT_MUTANTS, ), slicer_mutation.DEFAULT_MAX_MUTANTS)
        self.assertEqual(len({item.id for item in slicer_mutation.DEFAULT_MUTANTS}),
                         len(slicer_mutation.DEFAULT_MUTANTS))

    def test_empty_and_ambiguous_matches_are_rejected(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.cpp"
            source.write_text("same same", encoding="utf-8")
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._apply(
                    slicer_mutation.Mutant("empty", "source.cpp", "", "x", ""), root)
            with self.assertRaises(slicer_mutation.MutationError):
                slicer_mutation._apply(
                    slicer_mutation.Mutant("ambiguous", "source.cpp", "same", "x", ""), root)

    def test_mutation_restores_original_bytes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            source = root / "source.cpp"
            source.write_bytes(b"before\n")
            mutant = slicer_mutation.Mutant("one", "source.cpp", "before", "after", "")
            path, original = slicer_mutation._apply(mutant, root)
            self.assertEqual(path.read_bytes(), b"after\n")
            path.write_bytes(original)
            self.assertEqual(path.read_bytes(), b"before\n")

    def test_manifest_path_is_relative_to_declared_root(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            manifest = root / "tests" / "mutations.json"
            manifest.parent.mkdir()
            manifest.write_text(json.dumps({"mutants": [{
                "id": "one", "path": "source.cpp", "old": "before", "new": "after",
            }]}), encoding="utf-8")
            source = root / "source.cpp"
            source.write_text("before", encoding="utf-8")
            mutants = slicer_mutation._load_mutants("tests/mutations.json", root)
            self.assertEqual([item.id for item in mutants], ["one"])

    def test_repeated_manifests_keep_each_filter_in_selection_order(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            for name in ("first", "second"):
                (root / f"{name}.json").write_text(json.dumps({"mutants": [{
                    "id": name, "path": "source.cpp", "old": "before", "new": "after",
                    "test_filter": name.title(),
                }]}), encoding="utf-8")
            args = slicer_mutation._parser().parse_args([
                "run", "--build-command", "build", "--test-command", "ctest",
                "--mutants", "first.json", "--mutants", "second.json",
            ])
            mutants = slicer_mutation._load_mutants(args.mutants, root)
            self.assertEqual([(item.id, item.test_filter) for item in mutants],
                             [("first", "First"), ("second", "Second")])


class MutationResultTests(unittest.TestCase):
    def result(self, returncode, *, output="1 tests passed", timed_out=False):
        return slicer_mutation.CommandResult(["runner"], returncode, 0.01, output, "", timed_out)

    def test_distinguishes_killed_survived_timeout_and_build_error(self):
        self.assertEqual(slicer_mutation._classify_test(self.result(1), 1), "killed")
        self.assertEqual(slicer_mutation._classify_test(self.result(0), 1), "survived")
        self.assertEqual(slicer_mutation._classify_test(self.result(None, timed_out=True), 1), "test_timeout")
        self.assertEqual(slicer_mutation._classify_build(self.result(1)), "build_error")
        self.assertEqual(slicer_mutation._classify_build(self.result(None, timed_out=True)), "build_timeout")

    def test_zero_evidence_is_not_a_survivor(self):
        self.assertEqual(slicer_mutation._classify_test(self.result(0, output=""), 0), "test_no_evidence")

    def test_all_skipped_or_not_run_is_not_evidence(self):
        self.assertEqual(
            slicer_mutation.test_evidence(
                "0% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(
            slicer_mutation.test_evidence(
                "100% tests passed, 0 tests failed out of 3\nThe following tests did not run:\n"
            ),
            0,
        )
        self.assertEqual(slicer_mutation.test_evidence("test cases: 0 | 2 skipped"), 0)


if __name__ == "__main__":
    unittest.main()
