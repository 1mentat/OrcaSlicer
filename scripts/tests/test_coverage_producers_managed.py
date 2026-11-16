"""Composition tests: coverage producers run only through the validation runner."""

import contextlib
import io
import json
import os
from pathlib import Path
import runpy
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

SCRIPTS = Path(__file__).resolve().parents[1]
REPO = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
import validation_artifacts as artifacts
import validation_artifact_lint as lint
import validation_runner as runner

PRODUCERS = {
    "coverage-validation": ("slicer_coverage.py", "coverage", ["run"]),
    "coverage-pair-collector": ("slicer_coverage_pair_collector.py", "coverage-pair", []),
    "catch-batch": ("slicer_catch_batch.py", "catch-batch", []),
    "coverage-reconcile": ("slicer_coverage_reconcile.py", "coverage-reconcile", []),
}
SOURCE_SHA = "a" * 40
ENV_KEYS = ("VALIDATION_ARTIFACT_ROOT", "VALIDATION_ARTIFACT_RUN_ID", "VALIDATION_ARTIFACT_TOKEN",
            "VALIDATION_ARTIFACT_DIR", "VALIDATION_SOURCE_SHA")


def _clean_env(home: str) -> dict:
    env = {k: v for k, v in os.environ.items() if k not in ENV_KEYS}
    env.update({"HOME": home, "TMPDIR": home, "PYTHONDONTWRITEBYTECODE": "1"})
    return env


class CoverageProducersManagedTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.home = str(Path(self.temporary.name).resolve())

    def test_registry_entries_exist_and_lint_passes(self):
        document = json.loads((REPO / ".github/validation-producers.json").read_text(encoding="utf-8"))
        rows = {row["id"]: row for row in document["producers"]}
        for producer_id, (script, _, _) in PRODUCERS.items():
            row = rows[producer_id]
            self.assertEqual(row["path"], f"scripts/{script}")
            self.assertEqual(row["kind"], "cli")
            self.assertEqual(row["lifecycle"], "validation-runner")
            self.assertEqual(row["output_options"], ["--out"])
            self.assertEqual(row["payload_env"], "VALIDATION_ARTIFACT_DIR")
        self.assertEqual(lint.lint_producer_registry(REPO), [])

    def test_direct_write_without_runner_env_is_refused(self):
        for producer_id, (script, _, prefix) in PRODUCERS.items():
            with self.subTest(producer=producer_id):
                result = subprocess.run(
                    [sys.executable, "-B", str(SCRIPTS / script), *prefix, "--out", "x"],
                    capture_output=True, text=True, timeout=60, cwd=self.home,
                    env=_clean_env(self.home), check=False)
                self.assertEqual(result.returncode, 2, result.stderr)
                self.assertIn("run this writer through scripts/validation_runner.py", result.stderr)

    def test_help_stays_read_only(self):
        for producer_id, (script, _, prefix) in PRODUCERS.items():
            with self.subTest(producer=producer_id):
                result = subprocess.run(
                    [sys.executable, "-B", str(SCRIPTS / script), *prefix, "--help"],
                    capture_output=True, text=True, timeout=60, cwd=self.home,
                    env=_clean_env(self.home), check=False)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertIn("usage", result.stdout.lower())

    def _lease(self):
        root = Path(self.home) / "artifacts"
        artifacts.init(root, 2_000_000)
        record = artifacts.create(root, "PR-610", SOURCE_SHA, "tests", 60, 10000)
        self.addCleanup(artifacts.finish, root, record["run_id"], record["token"], "completed")
        environment = {"VALIDATION_ARTIFACT_ROOT": str(root),
                       "VALIDATION_ARTIFACT_RUN_ID": record["run_id"],
                       "VALIDATION_ARTIFACT_TOKEN": record["token"],
                       "VALIDATION_ARTIFACT_DIR": record["payload_dir"],
                       "VALIDATION_SOURCE_SHA": SOURCE_SHA}
        return Path(record["payload_dir"]).resolve(), environment

    def _run_main_block(self, script, prefix, extra):
        """Execute the script's real __main__ block, capturing the argv its main() receives."""
        captured = []
        real = runner.managed_main

        def spy(main, **kwargs):
            return real(lambda argv: captured.append(list(argv)) or 0, **kwargs)

        stderr = io.StringIO()
        with patch.object(runner, "managed_main", spy), \
                patch.object(subprocess, "check_output", return_value=SOURCE_SHA), \
                patch.object(sys, "argv", [script, *prefix, *extra]), \
                contextlib.redirect_stderr(stderr), self.assertRaises(SystemExit) as exit_info:
            runpy.run_path(str(SCRIPTS / script), run_name="__main__")
        return exit_info.exception.code, captured, stderr.getvalue()

    def test_runner_lease_defaults_output_into_payload_and_rejects_escape(self):
        payload, environment = self._lease()
        for producer_id, (script, default_name, prefix) in PRODUCERS.items():
            with self.subTest(producer=producer_id), patch.dict(os.environ, environment):
                code, captured, _ = self._run_main_block(script, prefix, [])
                self.assertEqual(code, 0)
                argv = captured[-1]
                self.assertEqual(argv[:len(prefix)], prefix)
                self.assertEqual(Path(argv[argv.index("--out") + 1]), payload / default_name)
                code, captured, _ = self._run_main_block(script, prefix, ["--out", "child"])
                self.assertEqual(code, 0)
                self.assertEqual(Path(captured[-1][captured[-1].index("--out") + 1]), payload / "child")
                outside = str(Path(self.home) / "escape")
                for bad in (outside, "../escape"):
                    code, captured, stderr = self._run_main_block(script, prefix, ["--out", bad])
                    self.assertEqual(code, 2)
                    self.assertEqual(captured, [])
                    self.assertIn("output must be a child of the leased payload directory", stderr)

    def test_build_and_work_root_constraints_apply(self):
        payload, environment = self._lease()
        script, _, prefix = PRODUCERS["catch-batch"]
        with patch.dict(os.environ, environment):
            code, captured, stderr = self._run_main_block(script, prefix, ["--build-dir", str(Path(self.home) / "build")])
            self.assertEqual(code, 2)
            self.assertEqual(captured, [])
            self.assertIn("build-dir must stay relative", stderr)
            code, captured, _ = self._run_main_block(script, prefix, ["--build-dir", "build"])
            self.assertEqual(code, 0)


if __name__ == "__main__":
    unittest.main()
