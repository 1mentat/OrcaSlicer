#!/usr/bin/env python3
"""Exercise Catch discovery profile isolation with a fake instrumented binary."""
from __future__ import annotations

import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
EXTRAS = ROOT / "tests" / "catch2" / "extras"

FAKE_TEST = r'''#!/usr/bin/env python3
import json
import os
from pathlib import Path
import sys

record_path = Path(os.environ["FIXTURE_RECORD"])
args = sys.argv[1:]
row = {
    "args": args,
    "gcov_prefix": os.environ.get("GCOV_PREFIX"),
    "gcov_prefix_strip": os.environ.get("GCOV_PREFIX_STRIP"),
    "llvm_profile_file": os.environ.get("LLVM_PROFILE_FILE"),
    "ld_library_path": os.environ.get("LD_LIBRARY_PATH"),
    "keep_env": os.environ.get("KEEP_ENV"),
}

def expand_llvm_pattern(pattern):
    return pattern.replace("%p", str(os.getpid())).replace("%m", "fake-binary-signature")

def write_profiles():
    written = []
    gcov_prefix = os.environ.get("GCOV_PREFIX")
    if gcov_prefix:
        gcda = Path(gcov_prefix) / "absolute" / "object" / "fake.gcda"
        gcda.parent.mkdir(parents=True, exist_ok=True)
        gcda.write_bytes(b"fake gcov profile\n")
        written.append(str(gcda))
    llvm_pattern = os.environ.get("LLVM_PROFILE_FILE")
    if llvm_pattern:
        llvm_file = Path(expand_llvm_pattern(llvm_pattern))
        llvm_file.parent.mkdir(parents=True, exist_ok=True)
        llvm_file.write_bytes(b"fake llvm profile\n")
        written.append(str(llvm_file))
    return written

if os.environ.get("FAKE_INSTRUMENTED", "1") == "1":
    row["profile_files"] = write_profiles()
else:
    row["profile_files"] = []
with record_path.open("a", encoding="utf-8") as stream:
    stream.write(json.dumps(row, sort_keys=True) + "\n")

if "--list-tests" in args:
    print(json.dumps({"version": 1, "listings": {"tests": [{"name": "fake registered case", "tags": ["Fake"]}]}}))
    failure_code = os.environ.get("FAKE_LIST_EXIT")
    if failure_code is not None:
        print("fake list discovery diagnostic")
        sys.exit(int(failure_code))
elif "--list-reporters" in args:
    print("Available reporters: console")
    failure_code = os.environ.get("FAKE_REPORTER_EXIT")
    if failure_code is not None:
        print("fake reporter discovery diagnostic")
        sys.exit(int(failure_code))
'''

PROJECT = r'''cmake_minimum_required(VERSION 3.19)
project(CatchDiscoveryIsolation NONE)
enable_testing()
find_package(Python3 COMPONENTS Interpreter REQUIRED)

add_executable(fake_tests IMPORTED GLOBAL)
set_property(TARGET fake_tests PROPERTY IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/fake_tests.py")
include("@EXTRAS@/Catch.cmake")
include("@EXTRAS@/CatchAddTests.cmake")

# Exercise the implementation in the configure process too, so the probe below
# can verify that its caller's environment was restored after discovery.
catch_discover_tests_impl(
  TEST_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/fake_tests.py"
  TEST_WORKING_DIR "${CMAKE_CURRENT_BINARY_DIR}"
  TEST_REPORTER console
  TEST_PREFIX "Direct::"
  CTEST_FILE "${CMAKE_CURRENT_BINARY_DIR}/direct-tests.cmake"
  ADD_TAGS_AS_LABELS
)
execute_process(
  COMMAND "${Python3_EXECUTABLE}" -c
    "import json,os; print(json.dumps({k:os.environ.get(k) for k in ['GCOV_PREFIX','GCOV_PREFIX_STRIP','LLVM_PROFILE_FILE','LD_LIBRARY_PATH','KEEP_ENV']}))"
  OUTPUT_VARIABLE restored_environment
  RESULT_VARIABLE environment_probe_status
)
if(NOT environment_probe_status EQUAL 0)
  message(FATAL_ERROR "Could not inspect the environment after direct discovery")
endif()
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/restored-environment.json" "${restored_environment}")

catch_discover_tests(fake_tests
  TEST_PREFIX "Fake::"
  REPORTER console
  DISCOVERY_MODE PRE_TEST
  ADD_TAGS_AS_LABELS
)
'''


@unittest.skipUnless(shutil.which("cmake") and shutil.which("ctest"), "CMake and CTest are required")
@unittest.skipIf(os.name == "nt", "fixture executable uses a POSIX shebang")
class DiscoveryEnvironmentTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temp = tempfile.TemporaryDirectory(prefix="catch-discovery-isolation-")
        self.root = Path(self.temp.name)
        self.source = self.root / "source"
        self.build = self.root / "build"
        self.source.mkdir()
        fake = self.source / "fake_tests.py"
        fake.write_text(FAKE_TEST, encoding="utf-8")
        fake.chmod(0o755)
        cmake_lists = PROJECT.replace("@EXTRAS@", EXTRAS.as_posix())
        (self.source / "CMakeLists.txt").write_text(cmake_lists, encoding="utf-8")
        self.record = self.root / "calls.jsonl"

    def tearDown(self) -> None:
        self.temp.cleanup()

    def run_cmake_and_ctest(
        self, coverage_env: dict[str, str] | None, *, instrumented: bool = True
    ) -> tuple[dict[str, str | None], list[dict[str, object]]]:
        env = os.environ.copy()
        for key in ("GCOV_PREFIX", "GCOV_PREFIX_STRIP", "LLVM_PROFILE_FILE"):
            env.pop(key, None)
        env.update({
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        if coverage_env:
            env.update(coverage_env)
        env["FIXTURE_RECORD"] = str(self.record)
        env["FAKE_INSTRUMENTED"] = "1" if instrumented else "0"
        self.root.joinpath("ld-loader-path").mkdir()

        configure = subprocess.run(
            ["cmake", "-S", str(self.source), "-B", str(self.build)],
            env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30,
        )
        self.assertEqual(configure.returncode, 0, configure.stdout)
        ctest = subprocess.run(
            ["ctest", "--test-dir", str(self.build), "-C", "Release", "--output-on-failure", "-R", "Fake::fake registered case"],
            env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30,
        )
        self.assertEqual(ctest.returncode, 0, ctest.stdout)
        self.assertIn("100% tests passed", ctest.stdout)
        rows = [json.loads(line) for line in self.record.read_text(encoding="utf-8").splitlines()]
        restored = json.loads((self.build / "restored-environment.json").read_text(encoding="utf-8"))
        return restored, rows

    def assert_discovery_rows_are_isolated(
        self, rows: list[dict[str, object]], *, instrumented: bool = True
    ) -> list[dict[str, object]]:
        discovery = [row for row in rows if "--list-tests" in row["args"] or "--list-reporters" in row["args"]]
        self.assertEqual(sum("--list-tests" in row["args"] for row in discovery), 2, rows)
        self.assertEqual(sum("--list-reporters" in row["args"] for row in discovery), 2, rows)
        for row in discovery:
            self.assertTrue(str(row["gcov_prefix"]).endswith("/gcov"), row)
            self.assertIn(".catch-discovery/", str(row["gcov_prefix"]), row)
            self.assertNotIn("%p", str(row["gcov_prefix"]), "use a concrete GCOV directory and keep LLVM placeholders in LLVM_PROFILE_FILE")
            self.assertEqual(row["gcov_prefix_strip"], "0", row)
            self.assertIn(".catch-discovery/", str(row["llvm_profile_file"]), row)
            self.assertTrue(str(row["llvm_profile_file"]).endswith("/llvm/%p-%m.profraw"), row)
            self.assertEqual(row["ld_library_path"], str(self.root / "ld-loader-path"), row)
            self.assertEqual(row["keep_env"], "preserved", row)
            expected_profiles = 2 if instrumented else 0
            self.assertEqual(len(row["profile_files"]), expected_profiles, row)
            self.assertTrue(all(".catch-discovery" in profile for profile in row["profile_files"]), row)
        self.assertTrue(all(not Path(profile).exists() for row in discovery for profile in row["profile_files"]))
        private_roots = {str(row["gcov_prefix"]).split("/gcov", 1)[0] for row in discovery}
        self.assertGreaterEqual(len(private_roots), 2, "distinct discovery invocations must get separate namespaces")
        return discovery

    def test_normal_coverage_environment_is_private_to_discovery(self) -> None:
        normal_gcov = self.root / "normal" / "gcov"
        normal_llvm = self.root / "normal" / "llvm-%p-%m.profraw"
        original = {
            "GCOV_PREFIX": str(normal_gcov),
            "GCOV_PREFIX_STRIP": "4",
            "LLVM_PROFILE_FILE": str(normal_llvm),
        }
        restored, rows = self.run_cmake_and_ctest(original)
        self.assertEqual(restored, original | {
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        self.assert_discovery_rows_are_isolated(rows)

        runtime = [row for row in rows if "--list-tests" not in row["args"] and "--list-reporters" not in row["args"]]
        self.assertEqual(len(runtime), 1, rows)
        self.assertEqual(runtime[0]["gcov_prefix"], str(normal_gcov), runtime[0])
        self.assertEqual(runtime[0]["gcov_prefix_strip"], "4", runtime[0])
        self.assertEqual(runtime[0]["llvm_profile_file"], str(normal_llvm), runtime[0])
        self.assertEqual(runtime[0]["ld_library_path"], str(self.root / "ld-loader-path"), runtime[0])
        self.assertEqual(runtime[0]["keep_env"], "preserved", runtime[0])
        self.assertEqual(len(list(normal_gcov.rglob("fake.gcda"))), 1)
        self.assertEqual(len(list((self.root / "normal").glob("llvm-*-fake-binary-signature.profraw"))), 1)
        self.assertEqual(runtime[0]["args"], ["fake registered case", "--reporter console"])

        self.assertFalse(list(self.build.rglob("fake.gcda")))
        self.assertFalse(list(self.build.rglob("*-fake-binary-signature.profraw")))

    def test_post_build_script_mode_uses_private_profiles(self) -> None:
        normal_gcov = self.root / "normal" / "gcov"
        normal_llvm = self.root / "normal" / "llvm-%p-%m.profraw"
        env = os.environ.copy()
        for key in ("GCOV_PREFIX", "GCOV_PREFIX_STRIP", "LLVM_PROFILE_FILE"):
            env.pop(key, None)
        env.update({
            "GCOV_PREFIX": str(normal_gcov),
            "GCOV_PREFIX_STRIP": "5",
            "LLVM_PROFILE_FILE": str(normal_llvm),
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
            "FIXTURE_RECORD": str(self.record),
            "FAKE_INSTRUMENTED": "1",
        })
        (self.root / "ld-loader-path").mkdir()
        ctest_file = self.build / "post-build-tests.cmake"
        self.build.mkdir()
        discover = subprocess.run(
            [
                "cmake",
                f"-DTEST_EXECUTABLE={self.source / 'fake_tests.py'}",
                f"-DTEST_WORKING_DIR={self.build}",
                "-DTEST_REPORTER=console",
                f"-DCTEST_FILE={ctest_file.name}",
                "-P",
                str(EXTRAS / "CatchAddTests.cmake"),
            ],
            cwd=self.build, env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30,
        )
        self.assertEqual(discover.returncode, 0, discover.stdout)
        rows = [json.loads(line) for line in self.record.read_text(encoding="utf-8").splitlines()]
        self.assertEqual(len(rows), 2, rows)
        self.assertEqual(sum("--list-tests" in row["args"] for row in rows), 1, rows)
        self.assertEqual(sum("--list-reporters" in row["args"] for row in rows), 1, rows)
        for row in rows:
            self.assertIn(".catch-discovery/", str(row["gcov_prefix"]), row)
            self.assertNotEqual(row["gcov_prefix"], str(normal_gcov), row)
            self.assertEqual(row["gcov_prefix_strip"], "0", row)
            self.assertTrue(str(row["llvm_profile_file"]).endswith("/llvm/%p-%m.profraw"), row)
            self.assertEqual(row["ld_library_path"], str(self.root / "ld-loader-path"), row)
            self.assertEqual(row["keep_env"], "preserved", row)
            self.assertEqual(len(row["profile_files"]), 2, row)
            self.assertTrue(all(".catch-discovery" in profile for profile in row["profile_files"]), row)
            self.assertTrue(all(not Path(profile).exists() for profile in row["profile_files"]))
        self.assertTrue(all(Path(str(row["gcov_prefix"])).resolve().is_relative_to(self.build.resolve()) for row in rows))
        generated = ctest_file.read_text(encoding="utf-8")
        self.assertIn("fake registered case", generated)
        self.assertNotIn(".catch-discovery", generated)
        self.assertFalse(list(self.build.rglob("fake.gcda")))
        self.assertFalse(list(self.build.rglob("*-fake-binary-signature.profraw")))

    def test_absent_coverage_environment_stays_absent_for_registered_test(self) -> None:
        restored, rows = self.run_cmake_and_ctest(None)
        self.assertEqual(restored, {
            "GCOV_PREFIX": None,
            "GCOV_PREFIX_STRIP": None,
            "LLVM_PROFILE_FILE": None,
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        self.assert_discovery_rows_are_isolated(rows)
        runtime = [row for row in rows if "--list-tests" not in row["args"] and "--list-reporters" not in row["args"]]
        self.assertEqual(len(runtime), 1, rows)
        for variable in ("gcov_prefix", "gcov_prefix_strip", "llvm_profile_file"):
            self.assertIsNone(runtime[0][variable], runtime[0])
        self.assertEqual(runtime[0]["ld_library_path"], str(self.root / "ld-loader-path"), runtime[0])
        self.assertEqual(runtime[0]["keep_env"], "preserved", runtime[0])
        self.assertEqual(runtime[0]["args"], ["fake registered case", "--reporter console"])

    def test_partial_coverage_environment_is_restored_per_variable(self) -> None:
        original = {"GCOV_PREFIX": str(self.root / "normal" / "gcov")}
        restored, rows = self.run_cmake_and_ctest(original)
        self.assertEqual(restored, original | {
            "GCOV_PREFIX_STRIP": None,
            "LLVM_PROFILE_FILE": None,
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        self.assert_discovery_rows_are_isolated(rows)
        runtime = [row for row in rows if "--list-tests" not in row["args"] and "--list-reporters" not in row["args"]]
        self.assertEqual(len(runtime), 1, rows)
        self.assertEqual(runtime[0]["gcov_prefix"], original["GCOV_PREFIX"], runtime[0])
        self.assertIsNone(runtime[0]["gcov_prefix_strip"], runtime[0])
        self.assertIsNone(runtime[0]["llvm_profile_file"], runtime[0])

    def test_explicit_empty_coverage_values_remain_set_and_empty(self) -> None:
        original = {
            "GCOV_PREFIX": "",
            "GCOV_PREFIX_STRIP": "",
            "LLVM_PROFILE_FILE": "",
        }
        restored, rows = self.run_cmake_and_ctest(original)
        self.assertEqual(restored, original | {
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        self.assert_discovery_rows_are_isolated(rows)
        runtime = [row for row in rows if "--list-tests" not in row["args"] and "--list-reporters" not in row["args"]]
        self.assertEqual(len(runtime), 1, rows)
        for variable in ("gcov_prefix", "gcov_prefix_strip", "llvm_profile_file"):
            self.assertEqual(runtime[0][variable], "", runtime[0])

    def run_discovery_failure(self, failure_env: dict[str, str]) -> tuple[subprocess.CompletedProcess[str], list[dict[str, object]], Path, Path]:
        normal_gcov = self.root / "normal" / "gcov"
        normal_llvm = self.root / "normal" / "llvm-%p-%m.profraw"
        env = os.environ.copy()
        for key in ("GCOV_PREFIX", "GCOV_PREFIX_STRIP", "LLVM_PROFILE_FILE"):
            env.pop(key, None)
        env.update({
            "GCOV_PREFIX": str(normal_gcov),
            "GCOV_PREFIX_STRIP": "6",
            "LLVM_PROFILE_FILE": str(normal_llvm),
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
            "FIXTURE_RECORD": str(self.record),
            "FAKE_INSTRUMENTED": "1",
        })
        env.update(failure_env)
        (self.root / "ld-loader-path").mkdir()
        configure = subprocess.run(
            ["cmake", "-S", str(self.source), "-B", str(self.build)],
            env=env, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30,
        )
        rows = [json.loads(line) for line in self.record.read_text(encoding="utf-8").splitlines()]
        return configure, rows, normal_gcov, normal_llvm

    def assert_failed_discovery_is_isolated(
        self, rows: list[dict[str, object]], normal_gcov: Path, normal_llvm: Path
    ) -> None:
        self.assertTrue(rows, rows)
        self.assertTrue(all("--list-tests" in row["args"] or "--list-reporters" in row["args"] for row in rows), rows)
        for row in rows:
            self.assertIn(".catch-discovery/", str(row["gcov_prefix"]), row)
            self.assertEqual(row["gcov_prefix_strip"], "0", row)
            self.assertIn(".catch-discovery/", str(row["llvm_profile_file"]), row)
            self.assertEqual(len(row["profile_files"]), 2, row)
            self.assertTrue(all(not Path(profile).exists() for profile in row["profile_files"]))
        self.assertFalse(list(normal_gcov.rglob("fake.gcda")))
        self.assertFalse(list(normal_llvm.parent.glob("llvm-*-fake-binary-signature.profraw")))

    def test_failed_test_listing_preserves_error_and_cleans_profiles(self) -> None:
        configure, rows, normal_gcov, normal_llvm = self.run_discovery_failure({"FAKE_LIST_EXIT": "17"})
        self.assertNotEqual(configure.returncode, 0)
        self.assertIn("Error listing tests from executable", configure.stdout)
        self.assertIn("Result: 17", configure.stdout)
        self.assertIn("fake list discovery diagnostic", configure.stdout)
        self.assertEqual(len(rows), 1, rows)
        self.assertIn("--list-tests", rows[0]["args"])
        self.assert_failed_discovery_is_isolated(rows, normal_gcov, normal_llvm)

    def test_invalid_reporter_preserves_error_and_cleans_profiles(self) -> None:
        configure, rows, normal_gcov, normal_llvm = self.run_discovery_failure({"FAKE_REPORTER_EXIT": "255"})
        self.assertNotEqual(configure.returncode, 0)
        self.assertIn("\"console\" is not a valid reporter!", configure.stdout)
        self.assertEqual(sum("--list-tests" in row["args"] for row in rows), 1, rows)
        self.assertEqual(sum("--list-reporters" in row["args"] for row in rows), 1, rows)
        self.assert_failed_discovery_is_isolated(rows, normal_gcov, normal_llvm)

    def test_uninstrumented_binary_and_registered_runtime_are_unchanged(self) -> None:
        normal_gcov = self.root / "normal" / "gcov"
        normal_llvm = self.root / "normal" / "llvm-%p-%m.profraw"
        original = {
            "GCOV_PREFIX": str(normal_gcov),
            "GCOV_PREFIX_STRIP": "7",
            "LLVM_PROFILE_FILE": str(normal_llvm),
        }
        restored, rows = self.run_cmake_and_ctest(original, instrumented=False)
        self.assertEqual(restored, original | {
            "LD_LIBRARY_PATH": str(self.root / "ld-loader-path"),
            "KEEP_ENV": "preserved",
        })
        self.assert_discovery_rows_are_isolated(rows, instrumented=False)
        runtime = [row for row in rows if "--list-tests" not in row["args"] and "--list-reporters" not in row["args"]]
        self.assertEqual(len(runtime), 1, rows)
        self.assertEqual(runtime[0]["gcov_prefix"], str(normal_gcov), runtime[0])
        self.assertEqual(runtime[0]["gcov_prefix_strip"], "7", runtime[0])
        self.assertEqual(runtime[0]["llvm_profile_file"], str(normal_llvm), runtime[0])
        self.assertEqual(runtime[0]["ld_library_path"], str(self.root / "ld-loader-path"), runtime[0])
        self.assertTrue(all(not row["profile_files"] for row in rows if "--list-tests" in row["args"] or "--list-reporters" in row["args"]))
        self.assertFalse(list(self.root.rglob("fake.gcda")))
        self.assertFalse(list(self.root.rglob("*-fake-binary-signature.profraw")))


if __name__ == "__main__":
    unittest.main()
