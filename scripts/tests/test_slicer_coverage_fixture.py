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
# Exercise the generated PCH system-header wrapper with project inline code.
target_precompile_headers(libslic3r PRIVATE src/libslic3r/Called.hpp)
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


@unittest.skipUnless(sys.platform.startswith("linux"), "compiled profile-identity fixtures require Linux ELF")
class PerExecutableCoveragePairFixtureTests(unittest.TestCase):
    def setUp(self):
        tools = _find_tools()
        missing = [name for name, tool in tools.items() if not tool]
        if missing:
            self.skipTest("missing tools: " + ", ".join(missing))
        self.tools = tools
        self.temp = tempfile.TemporaryDirectory(prefix="coverage-pair-fixture-")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name).resolve()

    def file_entry(self, report, name):
        matches = [item for item in report["files"] if Path(item["filename"]).name == name]
        self.assertEqual(len(matches), 1, [item["filename"] for item in report["files"]])
        return matches[0]

    def test_force_link_inherits_dependencies_without_an_existing_library_link(self):
        source = self.root / "dependency-project"
        build = self.root / "dependency-build"
        source.mkdir()
        (source / "CMakeLists.txt").write_text(f'''cmake_minimum_required(VERSION 3.16)
project(CoverageDependencyFixture LANGUAGES C CXX)
include("{REPOSITORY / "cmake" / "SlicerCoverage.cmake"}")
add_library(fixture_leaf STATIC leaf.cpp)
add_library(production STATIC production.cpp)
target_link_libraries(production PRIVATE fixture_leaf)
add_library(fixture_interface INTERFACE)
add_executable(orphan_tests main.cpp)
target_link_libraries(orphan_tests PRIVATE fixture_interface)
slicer_enable_llvm_source_coverage(production)
slicer_enable_llvm_source_coverage(orphan_tests)
slicer_force_link_static_library(orphan_tests production)
''', encoding="utf-8")
        (source / "leaf.cpp").write_text("int leaf_value() { return 42; }\n", encoding="utf-8")
        (source / "production.cpp").write_text(
            "extern int leaf_value();\nint uncalled_production() { return leaf_value(); }\n",
            encoding="utf-8")
        (source / "main.cpp").write_text("int main() { return 0; }\n", encoding="utf-8")
        env = dict(os.environ, CC=self.tools["clang"], CXX=self.tools["clang++"])
        configured = subprocess.run(
            ["cmake", "-S", str(source), "-B", str(build), "-G", "Ninja",
             "-DCMAKE_BUILD_TYPE=Release", "-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON"],
            env=env, capture_output=True, text=True, timeout=300)
        self.assertEqual(configured.returncode, 0, configured.stdout + configured.stderr)
        compiled = subprocess.run(["cmake", "--build", str(build), "--", "-j1"],
                                  capture_output=True, text=True, timeout=300)
        self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)
        executed = subprocess.run([str(build / "orphan_tests")], cwd=build,
                                  capture_output=True, text=True, timeout=30)
        self.assertEqual(executed.returncode, 0, executed.stdout + executed.stderr)

    def test_per_executable_pairing_isolates_cross_binary_dummy_mapping(self):
        source = self.root / "pair-project"
        build = self.root / "pair-build"
        files = {
            "CMakeLists.txt": f'''cmake_minimum_required(VERSION 3.16)
project(CoveragePairFixture LANGUAGES C CXX)
enable_testing()
include("{REPOSITORY / "cmake" / "SlicerCoverage.cmake"}")
add_library(libslic3r STATIC src/libslic3r/core.cpp src/libslic3r/uncalled.cpp)
target_include_directories(libslic3r PUBLIC src/libslic3r)
add_executable(suite_a tests/suite_a.cpp)
target_link_libraries(suite_a PRIVATE libslic3r)
add_executable(suite_b tests/suite_b.cpp)
target_link_libraries(suite_b PRIVATE libslic3r)
foreach(target IN ITEMS libslic3r suite_a suite_b)
  slicer_enable_llvm_source_coverage(${{target}})
endforeach()
foreach(target IN ITEMS suite_a suite_b)
  slicer_force_link_static_library(${{target}} libslic3r)
endforeach()
add_test(NAME suite_a COMMAND suite_a)
add_test(NAME suite_b COMMAND suite_b)
''',
            "src/libslic3r/shared.hpp": """#pragma once
inline int shared_inline(int value) { return value > 2 ? value + 1 : value - 1; }
int core_value(int value);
""",
            "src/libslic3r/core.cpp": """__attribute__((noinline)) void emitted_zero() {}

int core_value(int value)
{
    emitted_zero();
    if (value > 0)
        return value;
    return -value;
}
""",
            "src/libslic3r/uncalled.cpp": """int never_linked_by_reference() { return 42; }
""",
            "tests/suite_a.cpp": """#include "shared.hpp"
int main(int argc, char**) { return shared_inline(core_value(argc)) >= 0 ? 0 : 1; }
""",
            "tests/suite_b.cpp": """#include "shared.hpp"
int main(int argc, char**) { return core_value(argc) > 0 ? 0 : 1; }
""",
        }
        for relative, contents in files.items():
            path = source / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(contents, encoding="utf-8")
        env = dict(os.environ, CC=self.tools["clang"], CXX=self.tools["clang++"])
        configured = subprocess.run(
            ["cmake", "-S", str(source), "-B", str(build), "-G", "Ninja",
             "-DCMAKE_BUILD_TYPE=Release", "-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE=ON"],
            env=env, capture_output=True, text=True, timeout=300)
        self.assertEqual(configured.returncode, 0, configured.stdout + configured.stderr)
        compiled = subprocess.run(["cmake", "--build", str(build), "--", "-j1"],
                                  capture_output=True, text=True, timeout=300)
        self.assertEqual(compiled.returncode, 0, compiled.stdout + compiled.stderr)

        def collect(out_name, per_executable):
            out = self.root / out_name
            argv = ["run", "--root", str(source), "--build-dir", str(build),
                    "--source-root", "src/libslic3r",
                    "--manifest", str(build / "slicer-coverage" / "libslic3r.sources.txt"),
                    "--test-command", "ctest --test-dir {build} --no-tests=error -j1",
                    "--out", str(out), "--measure-only"]
            if per_executable:
                argv.extend(["--per-executable", "--html"])
            for name in ("suite_a", "suite_b"):
                argv.extend(["--object", str(build / name)])
            status = slicer_coverage.main(argv)
            result_name = "coverage-collection.json" if per_executable else "coverage-report.json"
            result = json.loads((out / result_name).read_text(encoding="utf-8"))
            return status, result, out

        global_status, global_report, _ = collect("pair-global", False)
        self.assertEqual(global_status, 2, json.dumps(global_report.get("integrity_failures"), indent=1))
        self.assertTrue(global_report["integrity_failures"]["profile_mapping"])

        pair_status, collection, pair_out = collect("pair-series", True)
        self.assertEqual(pair_status, 0, json.dumps(collection, indent=1))
        self.assertEqual(collection["status"], "pass")
        self.assertNotIn("summary", collection)
        toolchain = dict(line.split("=", 1) for line in
                         (build / "slicer-coverage" / "toolchain.txt").read_text().splitlines()
                         if "=" in line)
        for profile_row in collection["profile_identity"]["profiles"]:
            raw_show = subprocess.run(
                [toolchain["llvm_profdata"], "show", "--all-functions", "--counts",
                 profile_row["profile"]], capture_output=True, text=True, timeout=60)
            self.assertEqual(raw_show.returncode, 0, raw_show.stdout + raw_show.stderr)
            zero_function = re.search(
                r"(?:emitted_zero|_Z12emitted_zerov)[^:\n]*:\s+Hash: 0", raw_show.stdout)
            self.assertIsNotNone(zero_function, raw_show.stdout)
        for row in collection["suites"]:
            report = json.loads((pair_out / row["report"]).read_text(encoding="utf-8"))
            self.assertEqual(report["integrity_status"], "pass")
            self.assertEqual(report["tool_versions"]["compatibility"], "major_versions_match")
            self.assertEqual(report["annotation"]["status"], "ok")
            suite_dir = pair_out / "suites" / row["name"]
            self.assertTrue((suite_dir / "html" / "index.html").is_file())
            annotation_argv = report["annotation"]["argv"]
            profile_index = annotation_argv.index("-instr-profile")
            self.assertEqual(annotation_argv[profile_index + 1], str(suite_dir / "merged.profdata"))
            self.assertIn(str(build / row["executable_name"]), annotation_argv)
            uncalled = self.file_entry(report, "uncalled.cpp")["summary"]
            self.assertGreater(uncalled["lines"]["count"], 0)
            self.assertEqual(uncalled["lines"]["covered"], 0)
            core_functions = self.file_entry(report, "core.cpp")["summary"]["functions"]
            self.assertEqual(core_functions["count"], 2)
            self.assertEqual(core_functions["covered"], 2)
            inline = self.file_entry(report, "shared.hpp")["summary"]["functions"]
            self.assertGreater(inline["count"], 0)
            if row["name"] == "suite_b":
                self.assertEqual(inline["covered"], 0)


@unittest.skipUnless(sys.platform.startswith("linux"), "coverage dependency script is Linux-only")
class CoverageDepsWorkingDirectoryTests(unittest.TestCase):
    def test_deps_only_preserves_caller_paths_and_git_patch_context(self):
        required = (
            "bash", "cmake", "ninja", "clang-18", "clang++-18", "ld.lld-18",
            "llvm-profdata-18", "llvm-cov-18", "git", "taskset",
        )
        missing = [name for name in required if not shutil.which(name)]
        if missing:
            self.skipTest("missing tools: " + ", ".join(missing))

        with tempfile.TemporaryDirectory(prefix="coverage-deps-cwd-") as temp:
            root = Path(temp).resolve()
            source = root / "source"
            caller = root / "caller"
            foreign_cache = root / "foreign-cache"
            for directory in (source, caller, foreign_cache):
                directory.mkdir()

            git_env = {
                "PATH": os.environ.get("PATH", os.defpath),
                "HOME": str(root),
                "GIT_CONFIG_NOSYSTEM": "1",
                "GIT_CONFIG_GLOBAL": os.devnull,
                "GIT_TERMINAL_PROMPT": "0",
            }

            def git(directory, *args):
                return subprocess.run(
                    ["git", "-C", str(directory), *args], env=git_env,
                    capture_output=True, text=True, timeout=10, check=True)

            def initialize_repo(directory, commit):
                git(directory, "init", "--quiet")
                git(directory, "config", "core.hooksPath", os.devnull)
                git(directory, "config", "commit.gpgsign", "false")
                git(directory, "config", "user.name", "Coverage fixture")
                git(directory, "config", "user.email", "coverage-fixture@example.invalid")
                if commit:
                    git(directory, "add", "-A")
                    git(directory, "commit", "--quiet", "--no-verify", "--no-gpg-sign", "-m", "fixture")

            (source / "scripts").mkdir()
            shutil.copy2(REPOSITORY / "scripts" / "build_coverage.sh",
                         source / "scripts" / "build_coverage.sh")
            (source / "cmake/modules").mkdir(parents=True)
            (source / "cmake/modules/fixture.cmake").write_text("# tracked fixture input\n", encoding="utf-8")
            deps_source = source / "deps"
            (deps_source / "OCCT").mkdir(parents=True)
            (deps_source / "OCCT/patch.diff").write_text(
                "diff --git a/probe.txt b/probe.txt\n"
                "--- a/probe.txt\n"
                "+++ b/probe.txt\n"
                "@@ -1 +1 @@\n"
                "-before\n"
                "+after\n",
                encoding="utf-8")
            (deps_source / "CMakeLists.txt").write_text(r'''cmake_minimum_required(VERSION 3.10)
project(DepsWorkingDirectoryFixture LANGUAGES NONE)
find_package(Git REQUIRED)
execute_process(
    COMMAND "${GIT_EXECUTABLE}" rev-parse --is-inside-work-tree
    RESULT_VARIABLE REV_PARSE_RESULT
    OUTPUT_VARIABLE REV_PARSE_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE)
if((REV_PARSE_RESULT EQUAL 0) AND (REV_PARSE_OUTPUT STREQUAL "true"))
    set(IN_GIT_REPO TRUE)
    file(RELATIVE_PATH BINARY_DIR_REL "${CMAKE_SOURCE_DIR}/.." "${CMAKE_BINARY_DIR}")
endif()
set(OCCT_SOURCE_DIR "${CMAKE_BINARY_DIR}/dep_OCCT-prefix/src/dep_OCCT")
file(MAKE_DIRECTORY "${OCCT_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/OrcaSlicer_dep")
file(WRITE "${OCCT_SOURCE_DIR}/probe.txt" "before\n")
if(IN_GIT_REPO)
    set(OCCT_DIRECTORY_FLAG --directory "${BINARY_DIR_REL}/dep_OCCT-prefix/src/dep_OCCT")
else()
    set(OCCT_DIRECTORY_FLAG)
endif()
file(WRITE "${CMAKE_BINARY_DIR}/fixture-context.txt"
    "IN_GIT_REPO=${IN_GIT_REPO}\nBINARY_DIR_REL=${BINARY_DIR_REL}\n")
add_custom_target(fixture_patch
    COMMAND "${GIT_EXECUTABLE}" apply ${OCCT_DIRECTORY_FLAG} --verbose
        --ignore-space-change --whitespace=fix "${CMAKE_CURRENT_SOURCE_DIR}/OCCT/patch.diff"
    WORKING_DIRECTORY "${OCCT_SOURCE_DIR}"
    VERBATIM)
set(SELECTED_DEP_TARGETS
    dep_Boost dep_Eigen dep_TBB dep_Cereal dep_CGAL dep_Draco dep_NLopt
    dep_OpenVDB dep_SLVS dep_OCCT dep_OpenCV dep_Assimp dep_python3 dep_libnoise
    dep_Qhull dep_GLFW dep_wxWidgets)
foreach(_target IN LISTS SELECTED_DEP_TARGETS)
    add_custom_target(${_target})
    add_dependencies(${_target} fixture_patch)
endforeach()
''', encoding="utf-8")
            initialize_repo(source, commit=True)
            (caller / "caller.txt").write_text("caller repo\n", encoding="utf-8")
            initialize_repo(caller, commit=True)
            initialize_repo(foreign_cache, commit=False)

            script = source / "scripts/build_coverage.sh"
            tools_env = {
                "PATH": os.environ.get("PATH", os.defpath),
                "HOME": str(root),
                "LANG": "C.UTF-8",
                "LC_ALL": "C.UTF-8",
                "GIT_CONFIG_NOSYSTEM": "1",
                "GIT_CONFIG_GLOBAL": os.devnull,
                "GIT_TERMINAL_PROMPT": "0",
                "ORCA_COVERAGE_AFFINITY_SET": "1",
                "COVERAGE_BUILD_JOBS": "1",
            }

            def run_deps(label, deps_value):
                env = dict(tools_env)
                env.update({
                    "COVERAGE_BUILD_DIR": f"relative-build/{label}",
                    "COVERAGE_DEPS_DIR": deps_value,
                    "COVERAGE_OUTPUT_DIR": f"relative-output/{label}",
                })
                result = subprocess.run(
                    ["bash", "-x", str(script), "--deps-only"], cwd=caller, env=env,
                    capture_output=True, text=True, timeout=90)
                return result, env

            outside, outside_env = run_deps("outside", "../outside-cache")
            self.assertEqual(outside.returncode, 0, outside.stdout + outside.stderr)
            outside_deps = (caller / outside_env["COVERAGE_DEPS_DIR"]).resolve()
            outside_out = (caller / outside_env["COVERAGE_OUTPUT_DIR"]).resolve()
            outside_build = (caller / outside_env["COVERAGE_BUILD_DIR"]).resolve()
            outside_patch = outside_deps / "dep_OCCT-prefix/src/dep_OCCT/probe.txt"
            self.assertEqual(outside_patch.read_text(encoding="utf-8"), "after\n")
            self.assertEqual((outside_deps / "fixture-context.txt").read_text(encoding="utf-8"),
                             "IN_GIT_REPO=\nBINARY_DIR_REL=\n")
            self.assertEqual((outside_deps / "OrcaSlicer_dep/.coverage-ready").read_text().strip(),
                             subprocess.run(["bash", str(script), "--deps-key"], cwd=caller,
                                            env=tools_env, capture_output=True, text=True,
                                            timeout=30, check=True).stdout.strip())
            self.assertIn(f"build={outside_build}", outside.stderr)
            self.assertIn(f"deps={outside_deps}", outside.stderr)
            self.assertIn(f"out={outside_out}", outside.stderr)
            for name in ("deps-configure.log", "deps-build.log", "environment.txt"):
                self.assertTrue((outside_out / name).is_file(), name)
            self.assertIn("Checking patch probe.txt", (outside_out / "deps-build.log").read_text())

            inside, inside_env = run_deps("inside", "../source/deps/cache-inside")
            self.assertEqual(inside.returncode, 0, inside.stdout + inside.stderr)
            inside_deps = (caller / inside_env["COVERAGE_DEPS_DIR"]).resolve()
            inside_out = (caller / inside_env["COVERAGE_OUTPUT_DIR"]).resolve()
            inside_build = (caller / inside_env["COVERAGE_BUILD_DIR"]).resolve()
            self.assertEqual((inside_deps / "dep_OCCT-prefix/src/dep_OCCT/probe.txt").read_text(encoding="utf-8"),
                             "after\n")
            context = (inside_deps / "fixture-context.txt").read_text(encoding="utf-8")
            self.assertIn("IN_GIT_REPO=TRUE", context)
            self.assertIn(f"BINARY_DIR_REL={os.path.relpath(inside_deps, source)}", context)
            self.assertEqual((inside_deps / "OrcaSlicer_dep/.coverage-ready").read_text().strip(),
                             subprocess.run(["bash", str(script), "--deps-key"], cwd=caller,
                                            env=tools_env, capture_output=True, text=True,
                                            timeout=30, check=True).stdout.strip())
            self.assertIn(f"build={inside_build}", inside.stderr)
            self.assertIn(f"deps={inside_deps}", inside.stderr)
            self.assertIn(f"out={inside_out}", inside.stderr)
            for name in ("deps-configure.log", "deps-build.log", "environment.txt"):
                self.assertTrue((inside_out / name).is_file(), name)
            self.assertIn("--directory", (inside_out / "deps-build.log").read_text())

            rejected, rejected_env = run_deps("foreign", "../foreign-cache")
            self.assertEqual(rejected.returncode, 2, rejected.stdout + rejected.stderr)
            self.assertIn("Dependency builds must be outside Git or in the source worktree.",
                          rejected.stderr)
            rejected_deps = (caller / rejected_env["COVERAGE_DEPS_DIR"]).resolve()
            rejected_out = (caller / rejected_env["COVERAGE_OUTPUT_DIR"]).resolve()
            self.assertFalse((rejected_deps / "OrcaSlicer_dep/.coverage-ready").exists())
            self.assertFalse((rejected_out / "deps-configure.log").exists())
            self.assertFalse((rejected_out / "deps-build.log").exists())

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
