"""Tiny real GNU gcov fixture (Linux only): real g++ compile/link/run and real gcov.

Scope: a tiny real GNU fixture only. It drives the repository's actual
epoch-check, gcov runner, JSON reader and physical-line functions on one
two-function program; it makes no claim about the full pair/batch pipeline.
"""

import gzip
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import slicer_coverage as common  # noqa: E402
import slicer_gcov_coverage as gcov  # noqa: E402

TOOL_TIMEOUT = 60
# Line numbers are asserted below; keep this text and the numbers in step.
SOURCE = (b"int called_fn(int x) {\n"        # 1
          b"    return x + 1;\n"              # 2
          b"}\n"                              # 3
          b"int uncalled_fn(int x) {\n"       # 4
          b"    return x * 2;\n"              # 5
          b"}\n"                              # 6
          b"int main() {\n"                   # 7
          b"    return called_fn(0) == 1 ? 0 : 1;\n"  # 8
          b"}\n")                             # 9


@unittest.skipUnless(sys.platform.startswith("linux"), "GNU gcov fixture runs on Linux only")
class GnuGcovNativeFixtureTests(unittest.TestCase):
    def setUp(self):
        self.tools = {name: shutil.which(name) for name in ("g++", "gcov")}
        missing = [name for name, path in self.tools.items() if path is None]
        if missing:
            self.fail("required GNU tools are missing on Linux: " + ", ".join(missing))
        self._temp = tempfile.TemporaryDirectory(prefix="gcov-native-")
        self.addCleanup(self._temp.cleanup)
        self.root = Path(self._temp.name).resolve()
        self.env = {"PATH": os.environ.get("PATH", ""), "LC_ALL": "C",
                    "TMPDIR": str(self.root)}
        version = self._run([self.tools["g++"], "--version"], self.root)
        if "free software foundation" not in version.stdout.lower():
            self.fail("g++ is not GNU g++: " + version.stdout[:200])
        self.gcc_version = self._run([self.tools["g++"], "-dumpfullversion"],
                                     self.root).stdout.strip()
        self.assertTrue(self.gcc_version, "g++ -dumpfullversion printed nothing")
        self.source = self.root / "src" / "fixture.cpp"
        self.source.parent.mkdir()
        self.source.write_bytes(SOURCE)
        self.source_digest = hashlib.sha256(SOURCE).hexdigest()
        self.build = self.root / "build"
        self.build.mkdir()
        self.profile = self.root / "profile"
        self.profile.mkdir()
        self.cwd_before = sorted(os.listdir(os.getcwd()))

    def _run(self, argv, cwd, env=None):
        result = subprocess.run(argv, cwd=cwd, env=env or self.env, capture_output=True,
                                text=True, timeout=TOOL_TIMEOUT, check=False)
        self.assertEqual(result.returncode, 0, f"{argv} failed: {result.stderr[:2000]}")
        return result

    def _compile(self, source, name):
        obj = self.build / f"{name}.o"
        self._run([self.tools["g++"], "--coverage", "-O0", "-c", str(source), "-o", str(obj)],
                  self.build)
        exe = self.build / name
        self._run([self.tools["g++"], "--coverage", str(obj), "-o", str(exe)], self.build)
        return obj.with_suffix(".gcno"), exe

    def _execute(self, exe):
        env = dict(self.env, GCOV_PREFIX=str(self.profile), GCOV_PREFIX_STRIP="0")
        self._run([str(exe)], self.build, env=env)

    def _gcda(self, gcno):
        return gcov.gcda_path_for_object(self.profile, gcno.with_suffix(".o"))

    def _export(self, gcno, gcda, index=0):
        stage = self.root / f"stage-{index}"
        out = self.root / f"json-{index}"
        stage.mkdir()
        out.mkdir()
        staged = stage / gcno.name
        shutil.copy2(gcno, staged)
        shutil.copy2(gcda, staged.with_suffix(".gcda"))
        gcov.run_gcov_commands(
            [([self.tools["gcov"], "--json-format", str(staged)], out,
              out / "stdout.log", out / "stderr.log", str(self.source))],
            jobs=1, timeout=TOOL_TIMEOUT, common=common)
        emitted = sorted(out.glob("*.gcov.json.gz"))
        self.assertEqual(len(emitted), 1)
        return emitted[0], (out / "stdout.log").read_text(encoding="utf-8")

    def test_real_compile_run_and_gcov_export_through_repo_functions(self):
        gcno, exe = self._compile(self.source, "fixture")
        self._execute(exe)
        gcda = self._gcda(gcno)
        self.assertTrue(gcda.is_file(), f"program run produced no GCDA at {gcda}")

        epoch = gcov.inspect_gcov_profile_epochs([self.profile], expected_notes={gcno})
        self.assertEqual(epoch["status"], "pass", epoch["mismatches"])
        self.assertEqual(epoch["checked_gcda_count"], 1)

        json_path, gcov_stdout = self._export(gcno, gcda)
        records, meta = gcov._read_gcov_json(json_path, self.gcc_version)
        self.assertEqual(meta["gcc_version"].split(".")[0], self.gcc_version.split(".")[0])
        classification, mapped = gcov.classify_translation_unit(
            self.source, filter_matches_project=True, records=records,
            source_root=self.root / "src", gcov_output=gcov_stdout)
        self.assertEqual(classification, "source_mapped")
        self.assertEqual(mapped, [str(self.source)])

        result = gcov.validate_gcov_line_records(records, source_root=self.root / "src")
        counts = {row["line_number"]: row["count"] for row in result["physical_lines"]}
        self.assertEqual({row["filename"] for row in result["physical_lines"]}, {str(self.source)})
        self.assertGreater(counts[2], 0, "called function body must have hits")
        self.assertGreater(counts[8], 0, "main body must have hits")
        self.assertIn(5, counts, "uncalled function line must stay in the denominator")
        self.assertEqual(counts[5], 0)
        zero = {number for number, count in counts.items() if count == 0}
        hit = {number for number, count in counts.items() if count > 0}
        self.assertLessEqual(zero, {4, 5, 6})
        self.assertTrue(zero.isdisjoint({1, 2, 7, 8}))
        summary = result["summary"]
        self.assertEqual(summary["mapped_lines"], len(counts))
        self.assertEqual(summary["hit_lines"], len(hit))
        self.assertEqual(len(hit) + len(zero), len(counts))
        self.assertLess(summary["percent"], 100.0)
        self.assertGreater(summary["percent"], 0.0)

        # A source root that does not contain the recorded path maps nothing.
        foreign = gcov.validate_gcov_line_records(records, source_root=self.root / "build")
        self.assertEqual(foreign["summary"]["mapped_lines"], 0)
        # A compiler version that differs from the expected one is rejected.
        with self.assertRaises(ValueError):
            gcov._read_gcov_json(json_path, "0.0.0")

        self.assertEqual(hashlib.sha256(self.source.read_bytes()).hexdigest(), self.source_digest)
        self.assertEqual(sorted(os.listdir(os.getcwd())), self.cwd_before)
        self.assertEqual(sorted(p.name for p in self.root.iterdir()),
                         ["build", "json-0", "profile", "src", "stage-0"])

    def test_gcda_from_a_different_build_fails_epoch_check(self):
        gcno, exe = self._compile(self.source, "fixture")
        self._execute(exe)
        stale = self._gcda(gcno).read_bytes()
        # Rebuild the same object path later: new GCNO, so the old GCDA is foreign.
        time.sleep(1.1)
        gcno2, exe2 = self._compile(self.source, "fixture")
        self.assertEqual(gcno2, gcno)
        header_old = gcov._read_gcov_header(self._gcda(gcno), "gcda")
        header_new = gcov._read_gcov_header(gcno2, "gcno")
        self.assertNotEqual(header_old["stamp"], header_new["stamp"],
                            "fixture precondition: rebuilt GCNO must carry a new stamp")
        self.assertEqual(self._gcda(gcno).read_bytes(), stale)
        epoch = gcov.inspect_gcov_profile_epochs([self.profile], expected_notes={gcno2})
        self.assertEqual(epoch["status"], "mismatch")
        self.assertEqual([m["status"] for m in epoch["mismatches"]], ["epoch_mismatch"])


if __name__ == "__main__":
    unittest.main()
