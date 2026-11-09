#!/usr/bin/env python3
"""End-to-end checks of the Linux LLVM collector with real Clang and ELF build IDs.

A small CMake project mirrors Orca's layout: a static production library
under src/libslic3r with a called file, a file no test calls, an inline header
and a mapping-free translation unit, plus two test executables registered with
CTest. It is built with Ninja using one job. Skipped unless clang++, matching
llvm-cov/llvm-profdata, CMake and Ninja are installed (set SLICER_COVERAGE_CLANG
to choose a compiler, e.g. clang++-18).
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import unittest

REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
sys.path.insert(0, str(SCRIPTS))

import slicer_coverage  # noqa: E402

CMAKE_PROJECT = r"""
cmake_minimum_required(VERSION 3.16)
project(SlicerCoverageFixture LANGUAGES C CXX)
enable_testing()
set(CMAKE_REQUIRED_FLAGS "-DFIXTURE_CALLER_FLAG=1")
set(CMAKE_REQUIRED_LINK_OPTIONS "-Wl,--as-needed")
if (NOT FIXTURE_WITHOUT_COVERAGE_MODULE)
    include("${SLICER_COVERAGE_MODULE}")
else ()
    function(slicer_enable_llvm_source_coverage)
    endfunction()
    function(slicer_force_link_static_library)
    endfunction()
endif ()
if (NOT CMAKE_REQUIRED_FLAGS STREQUAL "-DFIXTURE_CALLER_FLAG=1" OR
    NOT CMAKE_REQUIRED_LINK_OPTIONS STREQUAL "-Wl,--as-needed")
    message(FATAL_ERROR "Coverage configuration changed the caller's compiler-check settings")
endif ()

add_library(fixture_dep INTERFACE)
add_library(libslic3r STATIC
    src/libslic3r/Called.cpp src/libslic3r/Called.hpp
    src/libslic3r/Uncalled.cpp
    src/libslic3r/ConstantsOnly.cpp)
target_include_directories(libslic3r PUBLIC src)
# Keyword signature on the library, plain signature on the tests, as in Orca.
target_link_libraries(libslic3r PRIVATE fixture_dep)

add_executable(full_tests tests/full_tests.cpp)
target_link_libraries(full_tests libslic3r)
add_executable(partial_tests tests/partial_tests.cpp)
target_link_libraries(partial_tests libslic3r)
add_executable(foreign_tool tests/foreign_tool.cpp)

add_test(NAME full COMMAND full_tests)
add_test(NAME partial COMMAND partial_tests)
add_test(NAME foreign COMMAND foreign_tool)
set_tests_properties(foreign PROPERTIES LABELS Foreign)
add_test(NAME failing COMMAND partial_tests --fail)
set_tests_properties(failing PROPERTIES LABELS Failing)

slicer_enable_llvm_source_coverage(libslic3r)
foreach (_test IN ITEMS full_tests partial_tests foreign_tool)
    slicer_enable_llvm_source_coverage(${_test})
endforeach ()
slicer_force_link_static_library(full_tests libslic3r)
"""

SOURCES = {
    "src/libslic3r/Called.hpp": """#pragma once
namespace Slic3r {
int called(int value);
inline int doubled(int value) { return value > 0 ? value * 2 : 0; }
}
""",
    "src/libslic3r/Called.cpp": """#include "libslic3r/Called.hpp"
namespace Slic3r {
int called(int value)
{
    if (value > 10)
        return value - 10;
    return doubled(value);
}
}
""",
    "src/libslic3r/Uncalled.cpp": """namespace Slic3r {
int never_called(int value)
{
    int total = 0;
    for (int i = 0; i < value; ++i)
        total += i % 3 == 0 ? i : -i;
    return total;
}
}
""",
    "src/libslic3r/ConstantsOnly.cpp": """namespace Slic3r {
extern const int constants_only_value;
const int constants_only_value = 42;
}
""",
    "tests/full_tests.cpp": """#include "libslic3r/Called.hpp"
#include <cstdio>
#include <thread>
int main()
{
    int a = 0, b = 0;
    std::thread first([&] { for (int i = 0; i < 1000; ++i) a += Slic3r::called(3); });
    std::thread second([&] { for (int i = 0; i < 1000; ++i) b += Slic3r::called(4); });
    first.join();
    second.join();
    std::puts("full ok");
    return a == 6000 && b == 8000 ? 0 : 1;
}
""",
    "tests/partial_tests.cpp": """#include "libslic3r/Called.hpp"
int main(int argc, char**) { return argc == 1 && Slic3r::called(12) == 2 ? 0 : 1; }
""",
    "tests/foreign_tool.cpp": """int main() { return 0; }
""",
}


def _find_tools():
    clang = os.environ.get("SLICER_COVERAGE_CLANG") or next(
        (name for name in ("clang++", "clang++-18", "clang++-19", "clang++-17") if shutil.which(name)), None)
    clang = shutil.which(clang) if clang else None
    tools = {"cmake": shutil.which("cmake"), "ninja": shutil.which("ninja"), "clang++": clang}
    if clang:
        tools["clang"] = shutil.which(re.sub(r"clang\+\+", "clang", Path(clang).name))
    return tools


def _command_lines(build: Path) -> list[str]:
    result = subprocess.run(["ninja", "-C", str(build), "-t", "commands"], capture_output=True,
                            text=True, check=True, timeout=60)
    return result.stdout.splitlines()


@unittest.skipUnless(sys.platform.startswith("linux"), "compiled profile-identity fixtures require Linux ELF")
class SlicerCoverageFixtureTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        tools = _find_tools()
        missing = [name for name, path in tools.items() if not path]
        if missing:
            raise unittest.SkipTest("missing tools: " + ", ".join(missing))
        cls.tools = tools
        cls.temp = tempfile.TemporaryDirectory(prefix="slicer-coverage-fixture-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.root = Path(cls.temp.name).resolve()
        cls.source = cls.root / "fixture"
        for relative, text in SOURCES.items():
            path = cls.source / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text, encoding="utf-8")
        (cls.source / "CMakeLists.txt").write_text(CMAKE_PROJECT, encoding="utf-8")
        cls.build = cls.configure("build-on", "-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON")
        subprocess.run(["cmake", "--build", str(cls.build), "--", "-j1"], check=True,
                       capture_output=True, timeout=600)

    @classmethod
    def configure(cls, name, *options):
        build = cls.root / name
        env = dict(os.environ, CC=cls.tools["clang"], CXX=cls.tools["clang++"])
        configured = subprocess.run(
            ["cmake", "-S", str(cls.source), "-B", str(build), "-G", "Ninja",
             "-DCMAKE_BUILD_TYPE=Release",
             f"-DSLICER_COVERAGE_MODULE={REPOSITORY / 'cmake' / 'SlicerCoverage.cmake'}", *options],
            env=env, capture_output=True, text=True, timeout=300)
        if configured.returncode:
            raise AssertionError("configure failed:\n" + configured.stdout + configured.stderr)
        return build

    def collect(self, out_name, *extra, objects=("full_tests", "partial_tests"),
                test_command="ctest --test-dir {build} -LE 'Foreign|Failing' --no-tests=error -j1"):
        out = self.root / out_name
        argv = ["run", "--root", str(self.source), "--build-dir", str(self.build),
                "--source-root", "src/libslic3r",
                "--manifest", str(self.build / "slicer-coverage" / "libslic3r.sources.txt"),
                "--test-command", test_command, "--out", str(out), *extra]
        for name in objects:
            argv.extend(["--object", str(self.build / name)])
        status = slicer_coverage.main(argv)
        report = json.loads((out / "coverage-report.json").read_text(encoding="utf-8"))
        return status, report

    def file_entry(self, report, name):
        matches = [item for item in report["files"] if Path(item["filename"]).name == name]
        self.assertEqual(len(matches), 1, [item["filename"] for item in report["files"]])
        return matches[0]

    def test_measure_only_reports_called_uncalled_and_mapping_free_files(self):
        status, report = self.collect("measure", "--measure-only", "--html")
        self.assertEqual(status, 0, json.dumps(report.get("integrity_failures"), indent=1))
        self.assertEqual(report["status"], "pass")
        self.assertEqual(report["integrity_status"], "pass")
        self.assertEqual(report["measurement"]["mode"], "measure_only")
        self.assertEqual(report["profile_identity"]["status"], "ok")
        self.assertEqual(report["tool_versions"]["compatibility"], "major_versions_match")

        called = self.file_entry(report, "Called.cpp")["summary"]
        uncalled = self.file_entry(report, "Uncalled.cpp")["summary"]
        header = self.file_entry(report, "Called.hpp")["summary"]
        self.assertGreater(called["lines"]["covered"], 0)
        self.assertEqual(called["lines"]["covered"], called["lines"]["count"])  # both suites together
        self.assertGreater(uncalled["lines"]["count"], 0)
        self.assertEqual(uncalled["lines"]["covered"], 0)
        self.assertEqual(uncalled["functions"]["covered"], 0)
        self.assertGreater(header["functions"]["covered"], 0)
        # Two executables use Called.cpp, but its one function is counted once.
        self.assertEqual(called["functions"]["count"], 1)
        names = [item["filename"] for item in report["files"]]
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(any(path.endswith("ConstantsOnly.cpp") for path in report["mapping_free_sources"]))
        self.assertLess(report["summary"]["lines"]["percent"], 100.0)

        out = self.root / "measure"
        self.assertTrue((out / "html" / "index.html").is_file())
        self.assertEqual(report["annotation"]["status"], "ok")
        for name in ("Called.cpp", "Uncalled.cpp"):
            page = out / self.file_entry(report, name)["html"]
            self.assertTrue(page.is_file(), page)
        self.assertTrue((out / "merged.profdata").is_file())
        self.assertTrue((out / "coverage-export.json").is_file())
        # Discovery-style profiles never land in the build tree.
        self.assertEqual(list(self.build.rglob("default.profraw")), [])

    def test_threshold_mode_reports_low_coverage_as_exit_1(self):
        status, report = self.collect("threshold", "--threshold", "99")
        self.assertEqual((status, report["status"], report["integrity_status"]),
                         (1, "coverage_below_threshold", "pass"))

    def test_foreign_profile_is_contamination(self):
        status, report = self.collect(
            "foreign", "--measure-only",
            test_command="ctest --test-dir {build} -LE Failing --no-tests=error -j1")
        self.assertEqual((status, report["status"]), (2, "coverage_integrity_error"))
        reasons = {error["reason"] for error in report["integrity_failures"]["profile_identity"]}
        self.assertEqual(reasons, {"foreign_profile"})

    def test_object_without_profile_is_a_failure(self):
        status, report = self.collect(
            "missing-profile", "--measure-only",
            test_command="ctest --test-dir {build} -R ^full$ --no-tests=error")
        self.assertEqual((status, report["status"]), (2, "coverage_integrity_error"))
        reasons = {error["reason"] for error in report["integrity_failures"]["profile_identity"]}
        self.assertEqual(reasons, {"object_without_profile"})

    def test_image_without_unreached_objects_is_an_incomplete_mapping(self):
        # partial_tests links libslic3r normally, so the linker drops Uncalled.o.
        status, report = self.collect(
            "incomplete", "--measure-only", objects=("partial_tests",),
            test_command="ctest --test-dir {build} -R ^partial$ --no-tests=error")
        self.assertEqual((status, report["status"]), (2, "coverage_integrity_error"))
        missing = report["integrity_failures"]["missing_compiled_source_mappings"]
        self.assertEqual([Path(path).name for path in missing], ["Uncalled.cpp"])

    def test_failing_tests_fail_measure_only(self):
        status, report = self.collect(
            "failing", "--measure-only",
            test_command="ctest --test-dir {build} -LE Foreign --no-tests=error -j1")
        self.assertEqual((status, report["status"]), (2, "test_failed"))
        self.assertTrue(report["tests_failed"])

    def test_static_runtime_propagates_without_changing_link_signatures(self):
        lines = _command_lines(self.build)
        compiles = [line for line in lines if " -c " in line and "libslic3r.dir" in line]
        self.assertTrue(compiles)
        for line in compiles:
            self.assertIn("-fprofile-instr-generate -fcoverage-mapping -fprofile-update=atomic", line)
        links = {name: next(line for line in lines if f"-o {name} " in line or line.endswith(f"-o {name}"))
                 for name in ("full_tests", "partial_tests")}
        for line in links.values():
            self.assertIn("-fprofile-instr-generate", line)
        self.assertIn("--whole-archive", links["full_tests"])
        self.assertNotIn("--whole-archive", links["partial_tests"])
        toolchain = (self.build / "slicer-coverage" / "toolchain.txt").read_text()
        self.assertIn("llvm_cov=", toolchain)

    def test_option_off_leaves_the_build_unchanged(self):
        off = self.configure("build-off")
        absent = self.configure("build-absent", "-DFIXTURE_WITHOUT_COVERAGE_MODULE=ON")
        normalize = lambda build: [line.replace(str(build), "<build>") for line in _command_lines(build)]  # noqa: E731
        self.assertEqual(normalize(off), normalize(absent))
        self.assertFalse((off / "slicer-coverage").exists())
        self.assertNotIn("profile-instr", "\n".join(normalize(off)))
        subprocess.run(["cmake", "--build", str(off), "--", "-j1"], check=True, capture_output=True, timeout=600)
        tested = subprocess.run(["ctest", "--test-dir", str(off), "-LE", "Failing", "--no-tests=error"],
                                capture_output=True, text=True, timeout=120)
        self.assertEqual(tested.returncode, 0, tested.stdout)

    def test_non_clang_compiler_is_rejected(self):
        gcc = shutil.which("g++")
        if not gcc or "clang" in subprocess.run([gcc, "--version"], capture_output=True, text=True).stdout:
            self.skipTest("GNU g++ is not installed")
        configured = subprocess.run(
            ["cmake", "-S", str(self.source), "-B", str(self.root / "build-gcc"), "-G", "Ninja",
             f"-DCMAKE_CXX_COMPILER={gcc}", f"-DCMAKE_C_COMPILER={shutil.which('gcc')}",
             "-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON",
             f"-DSLICER_COVERAGE_MODULE={REPOSITORY / 'cmake' / 'SlicerCoverage.cmake'}"],
            capture_output=True, text=True, timeout=300)
        self.assertNotEqual(configured.returncode, 0)
        self.assertIn("requires the Clang", configured.stdout + configured.stderr)


class CatchDiscoveryProfileIsolationTests(unittest.TestCase):
    """CatchAddTests.cmake must keep discovery profiles out of the caller's profile path."""

    def test_discovery_profiles_are_private_and_environment_is_restored(self):
        if not shutil.which("cmake"):
            self.skipTest("cmake is not installed")
        with tempfile.TemporaryDirectory(prefix="catch-discovery-") as temp:
            root = Path(temp).resolve()
            fake = root / "fake_tests.py"
            fake.write_text(
                "#!" + sys.executable + "\n"
                "import json, os, pathlib, sys\n"
                "pattern = os.environ.get('LLVM_PROFILE_FILE', 'default.profraw')\n"
                "path = pathlib.Path(pattern.replace('%p', str(os.getpid())).replace('%m', 'sig'))\n"
                "path.parent.mkdir(parents=True, exist_ok=True)\n"
                "path.write_bytes(b'profile')\n"
                "with open(os.environ['RECORD'], 'a') as stream: stream.write(str(path) + '\\n')\n"
                "if '--list-tests' in sys.argv:\n"
                "    print(json.dumps({'version': 1, 'listings': {'tests': [{'name': 'case', 'tags': []}]}}))\n",
                encoding="utf-8")
            fake.chmod(0o755)
            caller_profiles = root / "caller-profiles"
            script = root / "discover.cmake"
            # Script mode runs discovery on include, as CTest's PRE_TEST mode does in-process.
            script.write_text(
                f'set(TEST_EXECUTABLE "{fake.as_posix()}")\n'
                f'set(TEST_EXECUTOR "{Path(sys.executable).as_posix()}")\n'
                f'set(TEST_WORKING_DIR "{root.as_posix()}")\n'
                'set(TEST_REPORTER console)\n'
                'set(TEST_LIST fixture_tests)\n'
                f'set(CTEST_FILE "{(root / "out" / "tests.cmake").as_posix()}")\n'
                f'include("{(REPOSITORY / "tests/catch2/extras/CatchAddTests.cmake").as_posix()}")\n'
                'message(STATUS "AFTER=$ENV{LLVM_PROFILE_FILE}")\n', encoding="utf-8")
            (root / "out").mkdir()
            record = root / "record.txt"
            env = dict(os.environ, LLVM_PROFILE_FILE=str(caller_profiles / "%p.profraw"), RECORD=str(record))
            result = subprocess.run(["cmake", "-P", str(script)], env=env, capture_output=True,
                                    text=True, timeout=60)
            self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
            written = record.read_text().splitlines()
            self.assertEqual(len(written), 2)  # --list-tests and the reporter check
            for path in written:
                self.assertIn(".catch-discovery-", path)
                self.assertFalse(Path(path).exists(), "discovery profile was not removed")
            self.assertFalse(caller_profiles.exists())
            self.assertFalse((root / "default.profraw").exists())
            self.assertIn(f"AFTER={caller_profiles / '%p.profraw'}", result.stdout + result.stderr)
            self.assertIn("case", (root / "out" / "tests.cmake").read_text())


class FilamentGroupTimingTests(unittest.TestCase):
    """The FilamentGroup timing ceilings relax only in coverage builds."""

    SUITE = REPOSITORY / "tests" / "filament_group"

    def setUp(self):
        if not (self.SUITE / "filament_group_regression_main.cpp").is_file():
            self.skipTest("tests/filament_group sources are not checked out")

    def test_diagnostic_macro_is_defined_only_when_coverage_is_enabled(self):
        if not (shutil.which("cmake") and shutil.which("ninja")):
            self.skipTest("cmake and ninja are required")
        with tempfile.TemporaryDirectory(prefix="fg-timing-cmake-") as temp:
            root = Path(temp).resolve()
            # Stand-ins for what tests/CMakeLists.txt provides; configure only.
            (root / "CMakeLists.txt").write_text(f"""
cmake_minimum_required(VERSION 3.16)
project(FilamentGroupTimingFixture LANGUAGES CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
foreach(_target test_common libslic3r nlohmann_json)
    add_library(${{_target}} INTERFACE)
endforeach()
add_library(Catch2::Catch2WithMain INTERFACE IMPORTED)
function(orcaslicer_copy_test_dlls)
endfunction()
function(orcaslicer_discover_tests)
endfunction()
add_subdirectory("{self.SUITE.as_posix()}" filament_group)
""", encoding="utf-8")
            defines = {}
            for state in ("ON", "OFF"):
                build = root / f"build-{state}"
                configured = subprocess.run(
                    ["cmake", "-S", str(root), "-B", str(build), "-G", "Ninja",
                     f"-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE={state}"],
                    capture_output=True, text=True, timeout=120)
                self.assertEqual(configured.returncode, 0, configured.stdout + configured.stderr)
                commands = json.loads((build / "compile_commands.json").read_text(encoding="utf-8"))
                self.assertEqual(len(commands), 1)
                defines[state] = commands[0]["command"]
            self.assertRegex(defines["ON"], r"[-/]DSLICER_FG_COVERAGE_TIMING_DIAGNOSTIC_ONLY=1")
            self.assertNotIn("SLICER_FG_COVERAGE_TIMING_DIAGNOSTIC_ONLY", defines["OFF"])

    def test_release_ceiling_is_enforced_without_the_macro_and_recorded_with_it(self):
        compiler = shutil.which(os.environ.get("SLICER_COVERAGE_CLANG", "")) or shutil.which("c++") \
            or shutil.which("clang++")
        if not compiler:
            self.skipTest("no C++ compiler")
        source = (self.SUITE / "filament_group_regression_main.cpp").read_text(encoding="utf-8")
        match = re.search(r"static void check_elapsed_ms\(.*?\n}\n", source, re.DOTALL)
        self.assertIsNotNone(match, "check_elapsed_ms helper not found")
        # Minimal Catch2 stand-ins: INFO is ignored and REQUIRE failures exit 1.
        harness = (
            "#include <cmath>\n#include <cstdlib>\n#include <iostream>\n#include <string>\n"
            "#define INFO(x) do { } while (0)\n"
            "#define REQUIRE(x) do { if (!(x)) { std::exit(1); } } while (0)\n"
            + match.group(0)
            + "int main(int, char** argv) { check_elapsed_ms(\"case\", std::atof(argv[1]), 100.0); return 0; }\n")
        with tempfile.TemporaryDirectory(prefix="fg-timing-") as temp:
            root = Path(temp)
            (root / "harness.cpp").write_text(harness, encoding="utf-8")
            results = {}
            for name, defines in (("release", []), ("coverage", ["-DSLICER_FG_COVERAGE_TIMING_DIAGNOSTIC_ONLY=1"])):
                binary = root / name
                subprocess.run([compiler, "-std=c++17", *defines, str(root / "harness.cpp"), "-o", str(binary)],
                               check=True, capture_output=True, timeout=120)
                results[name] = {elapsed: subprocess.run([str(binary), elapsed], capture_output=True,
                                                         timeout=30).returncode
                                 for elapsed in ("50", "100", "250", "nan", "-1")}
            # Release keeps the exact strict ceiling.
            self.assertEqual(results["release"], {"50": 0, "100": 1, "250": 1, "nan": 1, "-1": 0})
            # Coverage records any finite, non-negative duration; nonsense still fails.
            self.assertEqual(results["coverage"], {"50": 0, "100": 0, "250": 0, "nan": 1, "-1": 1})


if __name__ == "__main__":
    unittest.main(verbosity=2)
