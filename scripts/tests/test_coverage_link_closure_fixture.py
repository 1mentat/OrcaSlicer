#!/usr/bin/env python3
"""Link the actual coverage-only CMake stanza in a small native project."""

from pathlib import Path
import os
import shutil
import subprocess
import tempfile
import unittest


REPOSITORY = Path(__file__).resolve().parents[2]
TOP_LEVEL = (REPOSITORY / "CMakeLists.txt").read_text(encoding="utf-8")
START = TOP_LEVEL.index("    # Each suite image carries the same production objects")
END = TOP_LEVEL.index("    if (NOT SLIC3R_BUNDLED_WARNINGS)", START)
COVERAGE_TEST_WIRING = TOP_LEVEL[START:END]


@unittest.skipUnless(os.name != "nt" and shutil.which("cmake") and shutil.which("c++")
                     and (shutil.which("ninja") or shutil.which("make")),
                     "POSIX CMake link fixture requires a C++ compiler and Ninja or Make")
class CoverageLinkClosureFixture(unittest.TestCase):
    def test_coverage_only_provider_closes_forced_production_references(self):
        with tempfile.TemporaryDirectory(prefix="coverage-link-closure-") as temp:
            source = Path(temp) / "source"
            source.mkdir()
            nanosvg_source = source / "tests/fff_print/fff_print_tests.cpp"
            nanosvg_source.parent.mkdir(parents=True)
            shutil.copy2(REPOSITORY / "tests/fff_print/fff_print_tests.cpp", nanosvg_source)
            (source / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.13)
project(CoverageLinkClosureFixture LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(REPOSITORY "{REPOSITORY}")
function(slicer_enable_llvm_source_coverage target)
endfunction()
function(slicer_force_link_static_library target library)
    set_property(TARGET ${{target}} APPEND PROPERTY LINK_LIBRARIES ${{library}})
endfunction()
add_library(libslic3r STATIC production.cpp)
add_library(libslic3r_cgal STATIC empty.cpp)
target_include_directories(libslic3r PRIVATE "${{REPOSITORY}}/deps_src")
add_library(slic3r_test_color_support STATIC color.cpp)
foreach(target IN ITEMS libslic3r_tests fff_print_tests sla_print_tests libnest2d_tests filament_group_tests)
    add_executable(${{target}} main.cpp)
    target_link_libraries(${{target}} libslic3r)
    target_include_directories(${{target}} PRIVATE "${{REPOSITORY}}/deps_src")
endforeach()
target_link_libraries(libslic3r_tests slic3r_test_color_support)
foreach(target IN ITEMS libslic3r_tests fff_print_tests sla_print_tests)
    target_sources(${{target}} PRIVATE "${{REPOSITORY}}/tests/fff_print/fff_print_tests.cpp")
endforeach()
if(SLICER_ENABLE_LLVM_SOURCE_COVERAGE)
    add_compile_definitions(CLOSURE_REQUIRED)
endif()
{COVERAGE_TEST_WIRING}
foreach(target IN ITEMS fff_print_tests sla_print_tests libnest2d_tests filament_group_tests)
    get_target_property(links ${{target}} LINK_LIBRARIES)
    if(SLICER_ENABLE_LLVM_SOURCE_COVERAGE AND NOT "slic3r_test_color_support" IN_LIST links)
        message(FATAL_ERROR "Missing RGB2HSV provider for ${{target}}")
    elseif(NOT SLICER_ENABLE_LLVM_SOURCE_COVERAGE AND "slic3r_test_color_support" IN_LIST links)
        message(FATAL_ERROR "Unexpected default-build provider for ${{target}}")
    endif()
endforeach()
foreach(target IN ITEMS libnest2d_tests filament_group_tests)
    get_target_property(sources ${{target}} SOURCES)
    if(SLICER_ENABLE_LLVM_SOURCE_COVERAGE AND NOT "${{CMAKE_CURRENT_SOURCE_DIR}}/tests/fff_print/fff_print_tests.cpp" IN_LIST sources)
        message(FATAL_ERROR "Missing NanoSVG implementation for ${{target}}")
    elseif(NOT SLICER_ENABLE_LLVM_SOURCE_COVERAGE AND "${{CMAKE_CURRENT_SOURCE_DIR}}/tests/fff_print/fff_print_tests.cpp" IN_LIST sources)
        message(FATAL_ERROR "Unexpected default-build NanoSVG implementation for ${{target}}")
    endif()
endforeach()
""", encoding="utf-8")
            (source / "production.cpp").write_text("""#include "nanosvg/nanosvg.h"
extern void RGB2HSV(float, float, float, float*, float*, float*);
int production()
{
    NSVGimage* image = nsvgParseFromFile("/missing.svg", "px", 96);
    if (image) nsvgDelete(image);
    char svg[] = "<svg xmlns='http://www.w3.org/2000/svg'/>";
    image = nsvgParse(svg, "px", 96);
    if (image) nsvgDelete(image);
    float h, s, v;
    RGB2HSV(0, 0, 0, &h, &s, &v);
    return 0;
}
""", encoding="utf-8")
            (source / "color.cpp").write_text(
                "void RGB2HSV(float, float, float, float* h, float* s, float* v) { *h = *s = *v = 0; }\n",
                encoding="utf-8")
            (source / "empty.cpp").write_text("int empty() { return 0; }\n", encoding="utf-8")
            (source / "main.cpp").write_text("""#ifdef CLOSURE_REQUIRED
extern int production();
int main() { return production(); }
#else
int main() { return 0; }
#endif
""", encoding="utf-8")
            for enabled in ("OFF", "ON"):
                build = Path(temp) / enabled.lower()
                for command in (
                    ["cmake", "-S", str(source), "-B", str(build),
                     "-G", "Ninja" if shutil.which("ninja") else "Unix Makefiles",
                     "-DCMAKE_BUILD_TYPE=Release", f"-DSLICER_ENABLE_LLVM_SOURCE_COVERAGE={enabled}"],
                    ["cmake", "--build", str(build), "--target", "libslic3r_tests", "fff_print_tests",
                     "sla_print_tests", "libnest2d_tests", "filament_group_tests", "-j2"],
                ):
                    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                for target in ("libslic3r_tests", "fff_print_tests", "sla_print_tests",
                               "libnest2d_tests", "filament_group_tests"):
                    result = subprocess.run([str(build / target)], capture_output=True, text=True, timeout=10)
                    self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
