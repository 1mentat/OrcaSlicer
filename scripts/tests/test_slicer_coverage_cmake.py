"""Configure-only checks for coverage instrumentation and default-OFF behavior.

Portable NONE-language controls verify that the original helpers leave an
uninstrumented project untouched. The separate Linux/GNU-only fixture configures
C and C++ targets, then inspects Ninja's command list without building those
targets. That fixture keeps its original platform/tool requirements and skips;
the workflow selects it explicitly on a preflight-checked GNU/Linux runner.
"""

from collections import Counter
import json
import platform
import re
import shlex
import shutil
import subprocess
import tempfile
from pathlib import Path
import unittest


REPOSITORY = Path(__file__).resolve().parents[2]

EXPECTED_SKIP = {
    "case_unset": {"C": False, "CXX": False},
    "case_c_only": {"C": True, "CXX": False},
    "case_cxx_only": {"C": False, "CXX": True},
    "case_both": {"C": True, "CXX": True},
    "case_nonccache": {"C": False, "CXX": False},
    "case_ccache_args": {"C": False, "CXX": True},
}


def _command_argv(entry):
    arguments = entry.get("arguments")
    return arguments if isinstance(arguments, list) else shlex.split(entry["command"])


def _cmake_regex_escape(value):
    # Match the character class used by SlicerCoverage.cmake. CMake leaves
    # hyphens unescaped here, unlike Python's re.escape().
    special = set("[].+*?^$(){}|\\")
    return "".join("\\" + char if char in special else char for char in value)


def _strip_ninja_depfile_args(argv):
    result = []
    index = 0
    while index < len(argv):
        token = argv[index]
        if token == "-MD":
            index += 1
        elif token in {"-MT", "-MF"}:
            index += 2
        else:
            result.append(token)
            index += 1
    return result


def _source_is_within(source, root):
    try:
        source.relative_to(root)
        return True
    except ValueError:
        return False


class SlicerCoverageCMakeLauncherTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if platform.system() != "Linux":
            raise unittest.SkipTest("configure fixture requires Linux")

        tools = {name: shutil.which(name) for name in ("cmake", "ninja", "gcc", "g++")}
        missing = [name for name, path in tools.items() if path is None]
        if missing:
            raise unittest.SkipTest("missing configure-fixture tools: " + ", ".join(missing))
        for name in ("gcc", "g++"):
            version = subprocess.run(
                [tools[name], "--version"], capture_output=True, text=True,
                timeout=10, check=False,
            )
            if version.returncode or not re.search(r"\bGCC\b|\bgcc\b|Free Software Foundation",
                                                   version.stdout, re.IGNORECASE):
                raise unittest.SkipTest("configure fixture requires GNU gcc and g++")

        cls.tools = tools
        cls.tempdir = tempfile.TemporaryDirectory(prefix="slicer-coverage-cmake-")
        cls.addClassCleanup(cls.tempdir.cleanup)
        cls.root = Path(cls.tempdir.name).resolve()
        cls.source = cls.root / "fixture"
        cls.build = cls.root / "build"
        for relative in ("src", "tests"):
            (cls.source / relative).mkdir(parents=True, exist_ok=True)
        (cls.source / "src/fixture.c").write_text("int fixture_c(void) { return 1; }\n")
        (cls.source / "src/fixture.cpp").write_text("int fixture_cpp() { return 2; }\n")
        (cls.source / "tests/outside.c").write_text("int outside_c(void) { return 3; }\n")
        (cls.source / "tests/outside.cpp").write_text("int outside_cpp() { return 4; }\n")
        (cls.source / "CMakeLists.txt").write_text(
            """cmake_minimum_required(VERSION 3.13)
project(SlicerCoverageLauncherFixture LANGUAGES C CXX)
if (NOT CMAKE_C_COMPILER_ID STREQUAL \"GNU\" OR
    NOT CMAKE_CXX_COMPILER_ID STREQUAL \"GNU\")
    message(FATAL_ERROR \"fixture requires GNU compilers for C and CXX\")
endif()
if (NOT DEFINED SLICER_COVERAGE_MODULE OR NOT EXISTS \"${SLICER_COVERAGE_MODULE}\")
    message(FATAL_ERROR \"SLICER_COVERAGE_MODULE must name the repository module\")
endif()
set(SLICER_ENABLE_GCC_GCOV_COVERAGE ON CACHE BOOL \"\" FORCE)
include(\"${SLICER_COVERAGE_MODULE}\")

function(add_launcher_case target c_launcher cxx_launcher)
    add_library(${target} STATIC
        src/fixture.c src/fixture.cpp
        tests/outside.c tests/outside.cpp)
    if (NOT \"${c_launcher}\" STREQUAL \"__unset__\")
        set_property(TARGET ${target} PROPERTY C_COMPILER_LAUNCHER \"${c_launcher}\")
    endif()
    if (NOT \"${cxx_launcher}\" STREQUAL \"__unset__\")
        set_property(TARGET ${target} PROPERTY CXX_COMPILER_LAUNCHER \"${cxx_launcher}\")
    endif()
    slicer_enable_llvm_source_coverage(${target})
endfunction()

add_launcher_case(case_unset __unset__ __unset__)
add_launcher_case(case_c_only ccache __unset__)
add_launcher_case(case_cxx_only __unset__ ccache)
add_launcher_case(case_both ccache ccache)
add_launcher_case(case_nonccache __unset__ sccache)
add_launcher_case(case_ccache_args __unset__
    \"ccache;--config-path=${CMAKE_BINARY_DIR}/fixture-ccache.conf\")
""",
            encoding="utf-8",
        )

        cls.configure_command = [
            tools["cmake"], "-S", str(cls.source), "-B", str(cls.build), "-G", "Ninja",
            "-DCMAKE_BUILD_TYPE=Release",
            "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
            "-DCMAKE_C_COMPILER=" + tools["gcc"],
            "-DCMAKE_CXX_COMPILER=" + tools["g++"],
            "-DCMAKE_C_COMPILER_LAUNCHER=",
            "-DCMAKE_CXX_COMPILER_LAUNCHER=",
            "-DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY",
            "-DSLICER_COVERAGE_MODULE=" + str(REPOSITORY / "cmake/SlicerCoverage.cmake"),
        ]
        configured = subprocess.run(
            cls.configure_command, cwd=cls.source, capture_output=True, text=True,
            timeout=180, check=False,
        )
        if configured.returncode:
            raise RuntimeError(
                "CMake configure failed:\n" + configured.stdout + configured.stderr
            )

        command_list = subprocess.run(
            [tools["ninja"], "-C", str(cls.build), "-t", "commands"],
            cwd=cls.source, capture_output=True, text=True, timeout=30, check=False,
        )
        if command_list.returncode:
            raise RuntimeError("Ninja command query failed:\n" + command_list.stderr)

        cls.compile_commands = json.loads(
            (cls.build / "compile_commands.json").read_text(encoding="utf-8")
        )
        cls.expanded_by_output = {}
        for line in command_list.stdout.splitlines():
            expanded = shlex.split(line)
            if "-c" not in expanded or "-o" not in expanded:
                continue
            output = expanded[expanded.index("-o") + 1]
            if output in cls.expanded_by_output:
                raise RuntimeError("duplicate compile output in Ninja command query")
            cls.expanded_by_output[output] = expanded

    def test_ccache_skip_is_language_gated_and_preserves_coverage_flags(self):
        source_root = (self.source / "src").resolve()
        expected_filter = (
            "-fprofile-filter-files=^" + _cmake_regex_escape(str(source_root)) + "/.*"
        )
        expected_launchers = {
            "case_unset": {"C": [], "CXX": []},
            "case_c_only": {"C": ["ccache"], "CXX": []},
            "case_cxx_only": {"C": [], "CXX": ["ccache"]},
            "case_both": {"C": ["ccache"], "CXX": ["ccache"]},
            "case_nonccache": {"C": [], "CXX": ["sccache"]},
            "case_ccache_args": {
                "C": [],
                "CXX": ["ccache", "--config-path=" + str(self.build / "fixture-ccache.conf")],
            },
        }
        self.assertEqual(len(self.compile_commands), 24)
        self.assertEqual(len(self.expanded_by_output), 24)

        rows_per_target = Counter()
        seen = set()
        for entry in self.compile_commands:
            argv = _command_argv(entry)
            source = Path(entry["file"]).resolve()
            lang = {".c": "C", ".cpp": "CXX"}.get(source.suffix)
            self.assertIsNotNone(lang, entry["file"])
            self.assertIn("-o", argv)
            output = argv[argv.index("-o") + 1]
            match = re.search(r"CMakeFiles/(case_[^/]+)\.dir/", output)
            self.assertIsNotNone(match, output)
            target = match.group(1)
            self.assertIn(target, EXPECTED_SKIP)

            launcher = expected_launchers[target][lang]
            expanded = self.expanded_by_output[output]
            self.assertEqual(expanded[:len(launcher)], launcher)
            self.assertEqual(_strip_ninja_depfile_args(expanded[len(launcher):]), argv)

            skip = EXPECTED_SKIP[target][lang]
            self.assertEqual(argv.count("--ccache-skip"), int(skip), (target, lang, argv))
            filter_indexes = [i for i, token in enumerate(argv)
                              if token.startswith("-fprofile-filter-files=")]
            self.assertEqual(len(filter_indexes), 1, (target, lang, argv))
            filter_index = filter_indexes[0]
            self.assertEqual(argv[filter_index], expected_filter)

            coverage_index = argv.index("--coverage")
            atomic_index = argv.index("-fprofile-update=atomic")
            self.assertEqual(atomic_index, coverage_index + 1)
            if skip:
                self.assertEqual(filter_index, atomic_index + 2)
                self.assertEqual(argv[atomic_index + 1], "--ccache-skip")
            else:
                self.assertEqual(filter_index, atomic_index + 1)

            self.assertEqual(argv.count("--coverage"), 1)
            self.assertEqual(argv.count("-fprofile-update=atomic"), 1)
            self.assertIn("-O3", argv)
            self.assertIn("-DNDEBUG", argv)
            in_scope = _source_is_within(source, source_root)
            self.assertEqual(
                re.fullmatch(expected_filter.split("=", 1)[1], str(source)) is not None,
                in_scope,
                (target, lang, source),
            )
            rows_per_target[target] += 1
            seen.add((target, lang, source.relative_to(self.source).as_posix()))

        self.assertEqual(rows_per_target, Counter({target: 4 for target in EXPECTED_SKIP}))
        self.assertEqual(len(seen), 24)


class SlicerCoverageCMakeDefaultOffTests(unittest.TestCase):
    def test_default_off_helpers_leave_none_project_untouched(self):
        cmake = shutil.which("cmake")
        ninja = shutil.which("ninja")
        self.assertIsNotNone(cmake, "CMake is required for the NONE-language control")
        self.assertIsNotNone(ninja, "Ninja is required for the NONE-language control")

        with tempfile.TemporaryDirectory(prefix="slicer-coverage-default-off-") as temporary:
            root = Path(temporary).resolve()
            source = root / "fixture source"
            build = root / "fixture build"
            for relative in ("src", "tests"):
                (source / relative).mkdir(parents=True, exist_ok=True)
            (source / "src/fixture.cpp").write_text("// Not compiled.\n", encoding="utf-8")
            (source / "tests/fixture.cpp").write_text("// Not compiled.\n", encoding="utf-8")
            (source / "CMakeLists.txt").write_text(
                """cmake_minimum_required(VERSION 3.13)
project(SlicerCoverageDefaultOffFixture LANGUAGES NONE)
if (NOT DEFINED SLICER_COVERAGE_MODULE OR NOT EXISTS \"${SLICER_COVERAGE_MODULE}\")
    message(FATAL_ERROR \"SLICER_COVERAGE_MODULE must name the repository module\")
endif()
add_custom_target(production SOURCES src/fixture.cpp)
add_custom_target(test_image SOURCES tests/fixture.cpp)
set_property(TARGET production PROPERTY CUSTOM_MARKER \"production-preserved\")
set_property(TARGET test_image PROPERTY CUSTOM_MARKER \"test-preserved\")
get_directory_property(_includes_before INCLUDE_DIRECTORIES)
include(\"${SLICER_COVERAGE_MODULE}\")
if (SLICER_ENABLE_LLVM_SOURCE_COVERAGE OR SLICER_ENABLE_GCC_GCOV_COVERAGE OR
    SLICER_COVERAGE_PROJECT_FUNCTIONS_ONLY)
    message(FATAL_ERROR \"coverage options must default to OFF\")
endif()
slicer_enable_llvm_source_coverage(production)
slicer_enable_llvm_source_coverage(test_image)
slicer_force_link_static_library(test_image production)
if (CMAKE_EXPORT_COMPILE_COMMANDS)
    message(FATAL_ERROR \"default-OFF helpers enabled compile commands\")
endif()
if (EXISTS \"${CMAKE_BINARY_DIR}/slicer-coverage\")
    message(FATAL_ERROR \"default-OFF helpers created a manifest directory\")
endif()
get_property(_manifests GLOBAL PROPERTY SLICER_COVERAGE_MANIFESTS)
if (_manifests)
    message(FATAL_ERROR \"default-OFF helpers registered a source manifest\")
endif()
get_directory_property(_includes_after INCLUDE_DIRECTORIES)
if (NOT \"${_includes_after}\" STREQUAL \"${_includes_before}\")
    message(FATAL_ERROR \"default-OFF helpers changed include directories\")
endif()
foreach(_target IN ITEMS production test_image)
    foreach(_property IN ITEMS
        COMPILE_DEFINITIONS COMPILE_OPTIONS INCLUDE_DIRECTORIES
        LINK_OPTIONS LINK_LIBRARIES INTERFACE_LINK_LIBRARIES)
        get_target_property(_value ${_target} ${_property})
        if (_value)
            message(FATAL_ERROR
                \"default-OFF helpers changed ${_target} ${_property}: ${_value}\")
        endif()
    endforeach()
endforeach()
get_target_property(_production_marker production CUSTOM_MARKER)
get_target_property(_test_marker test_image CUSTOM_MARKER)
if (NOT \"${_production_marker}\" STREQUAL \"production-preserved\" OR
    NOT \"${_test_marker}\" STREQUAL \"test-preserved\")
    message(FATAL_ERROR \"default-OFF helpers changed existing target properties\")
endif()
file(WRITE \"${CMAKE_BINARY_DIR}/default-off-ok.txt\" \"default OFF preserved\\n\")
""",
                encoding="utf-8",
            )
            configured = subprocess.run(
                [
                    cmake, "-S", str(source), "-B", str(build), "-G", "Ninja",
                    "-DSLICER_COVERAGE_MODULE=" + str(REPOSITORY / "cmake/SlicerCoverage.cmake"),
                ],
                cwd=source, capture_output=True, text=True, encoding="utf-8",
                errors="replace", timeout=60, check=False,
            )
            self.assertEqual(configured.returncode, 0, configured.stdout + configured.stderr)
            self.assertEqual(
                (build / "default-off-ok.txt").read_text(encoding="utf-8"),
                "default OFF preserved\n",
            )

    def test_main_cmake_retains_system_include_and_existing_targets_when_off(self):
        root_cmake = (REPOSITORY / "CMakeLists.txt").read_text(encoding="utf-8")
        expected_include_branch = """include(cmake/SlicerCoverage.cmake)
# Coverage instrumentation can omit or inconsistently map system headers.
# Keep our own inline definitions non-system in coverage builds so their
# mappings stay consistent between the libraries and tests.
if(SLICER_ENABLE_LLVM_SOURCE_COVERAGE OR SLICER_ENABLE_GCC_GCOV_COVERAGE)
    include_directories(${LIBDIR})
else()
    # For the bundled boost libraries (boost::nowide)
    include_directories(SYSTEM ${LIBDIR})
endif()"""
        self.assertEqual(root_cmake.count(expected_include_branch), 1)
        self.assertEqual(root_cmake.count("include(cmake/SlicerCoverage.cmake)"), 1)
        self.assertEqual(root_cmake.count("slicer_enable_llvm_source_coverage(libslic3r)"), 1)
        self.assertEqual(root_cmake.count("slicer_enable_llvm_source_coverage(libslic3r_cgal)"), 1)
        test_loop = """foreach(_slicer_test IN ITEMS libslic3r_tests fff_print_tests sla_print_tests libnest2d_tests filament_group_tests)
        slicer_enable_llvm_source_coverage(${_slicer_test})"""
        self.assertEqual(root_cmake.count(test_loop), 1)
        self.assertEqual(root_cmake.count("slicer_force_link_static_library(${_slicer_test} libslic3r)"), 1)
        self.assertEqual(root_cmake.count("slicer_force_link_static_library(${_slicer_test} libslic3r_cgal)"), 1)

        filament = (REPOSITORY / "tests/filament_group/CMakeLists.txt").read_text(encoding="utf-8")
        self.assertEqual(filament.count("orcaslicer_discover_tests(${_TEST_NAME}_tests)"), 1)
        self.assertEqual(filament.count("orcaslicer_copy_test_dlls()"), 1)
        self.assertEqual(filament.count("SLICER_FG_COVERAGE_TIMING_DIAGNOSTIC_ONLY=1"), 1)


if __name__ == "__main__":
    unittest.main(verbosity=2)
