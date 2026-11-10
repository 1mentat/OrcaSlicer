# Opt-in LLVM source-based coverage for production targets and their tests.
#
# With SLICER_ENABLE_LLVM_SOURCE_COVERAGE=OFF (the default) every function in
# this module returns immediately and the build is unchanged.
#
# Usage, after the targets are defined:
#
#   slicer_enable_llvm_source_coverage(libslic3r)
#   slicer_enable_llvm_source_coverage(libslic3r_tests)
#   slicer_force_link_static_library(libslic3r_tests libslic3r)
#
# Each instrumented target writes ${CMAKE_BINARY_DIR}/slicer-coverage/<target>.sources.txt
# from its SOURCES property, so scripts/slicer_coverage.py measures the sources
# the configured build actually compiles rather than a guessed glob.

include_guard(GLOBAL)

option(SLICER_ENABLE_LLVM_SOURCE_COVERAGE
       "Instrument selected OrcaSlicer targets with LLVM source-based coverage" OFF)

if (NOT SLICER_ENABLE_LLVM_SOURCE_COVERAGE)
    function(slicer_enable_llvm_source_coverage TARGET_NAME)
    endfunction()
    function(slicer_force_link_static_library TEST_TARGET LIBRARY_TARGET)
    endfunction()
    return()
endif ()

if (CMAKE_VERSION VERSION_LESS 3.16)
    message(FATAL_ERROR
        "SLICER_ENABLE_LLVM_SOURCE_COVERAGE requires CMake 3.16 or newer to disable precompiled headers")
endif ()

if (NOT CMAKE_CXX_COMPILER_ID MATCHES "^(Clang|AppleClang)$" OR
    CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
    message(FATAL_ERROR
        "SLICER_ENABLE_LLVM_SOURCE_COVERAGE requires the Clang or AppleClang GNU-style driver; "
        "got ${CMAKE_CXX_COMPILER_ID} (frontend '${CMAKE_CXX_COMPILER_FRONTEND_VARIANT}')")
endif ()
if (DEFINED CMAKE_C_COMPILER_ID AND CMAKE_C_COMPILER_ID AND
    NOT CMAKE_C_COMPILER_ID STREQUAL CMAKE_CXX_COMPILER_ID)
    message(FATAL_ERROR
        "SLICER_ENABLE_LLVM_SOURCE_COVERAGE requires matching C and C++ compilers; "
        "got ${CMAKE_C_COMPILER_ID} and ${CMAKE_CXX_COMPILER_ID}")
endif ()

# Slicing runs concurrently under TBB. Non-atomic counter increments can be
# lost, and LLVM derives some counters from others, so lost increments can
# credit regions that never ran. Refuse a compiler that cannot do atomic updates.
include(CheckCXXSourceCompiles)
include(CMakePushCheckState)
cmake_push_check_state(RESET)
set(CMAKE_REQUIRED_FLAGS "-fprofile-instr-generate -fcoverage-mapping -fprofile-update=atomic")
set(CMAKE_REQUIRED_LINK_OPTIONS "-fprofile-instr-generate")
check_cxx_source_compiles("int main() { return 0; }" SLICER_LLVM_COVERAGE_FLAGS_WORK)
cmake_pop_check_state()
if (NOT SLICER_LLVM_COVERAGE_FLAGS_WORK)
    message(FATAL_ERROR
        "${CMAKE_CXX_COMPILER} cannot compile and link with "
        "-fprofile-instr-generate -fcoverage-mapping -fprofile-update=atomic")
endif ()

# llvm-profdata and llvm-cov must read the profile and mapping formats this
# compiler writes. Prefer the tools beside the compiler, then the versioned
# names distributions use. scripts/slicer_coverage.py rechecks the versions.
string(REGEX MATCH "^[0-9]+" _slicer_llvm_major "${CMAKE_CXX_COMPILER_VERSION}")
get_filename_component(_slicer_compiler_dir "${CMAKE_CXX_COMPILER}" DIRECTORY)
get_filename_component(_slicer_compiler_real "${CMAKE_CXX_COMPILER}" REALPATH)
get_filename_component(_slicer_compiler_real_dir "${_slicer_compiler_real}" DIRECTORY)
foreach (_tool IN ITEMS cov profdata)
    string(TOUPPER "${_tool}" _tool_upper)
    find_program(SLICER_LLVM_${_tool_upper}
        NAMES llvm-${_tool}-${_slicer_llvm_major} llvm-${_tool}
        HINTS "${_slicer_compiler_real_dir}" "${_slicer_compiler_dir}"
        DOC "llvm-${_tool} matching the coverage compiler")
    if (NOT SLICER_LLVM_${_tool_upper} AND APPLE)
        execute_process(COMMAND xcrun --find llvm-${_tool}
            OUTPUT_VARIABLE _xcrun_tool OUTPUT_STRIP_TRAILING_WHITESPACE
            RESULT_VARIABLE _xcrun_result ERROR_QUIET)
        if (_xcrun_result EQUAL 0 AND EXISTS "${_xcrun_tool}")
            set(SLICER_LLVM_${_tool_upper} "${_xcrun_tool}" CACHE FILEPATH
                "llvm-${_tool} matching the coverage compiler" FORCE)
        endif ()
    endif ()
    if (NOT SLICER_LLVM_${_tool_upper})
        message(FATAL_ERROR "SLICER_ENABLE_LLVM_SOURCE_COVERAGE: llvm-${_tool} was not found; "
                            "set SLICER_LLVM_${_tool_upper}")
    endif ()
    execute_process(COMMAND "${SLICER_LLVM_${_tool_upper}}" --version
        OUTPUT_VARIABLE _tool_version ERROR_VARIABLE _tool_version_err
        RESULT_VARIABLE _tool_result)
    string(REGEX MATCH "LLVM version ([0-9]+)" _tool_match "${_tool_version}${_tool_version_err}")
    if (NOT _tool_result EQUAL 0 OR NOT CMAKE_MATCH_1)
        message(FATAL_ERROR "Cannot read the LLVM version of ${SLICER_LLVM_${_tool_upper}}")
    endif ()
    # AppleClang versions follow Xcode rather than LLVM, so only upstream Clang
    # can be compared by major version here.
    if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND NOT CMAKE_MATCH_1 STREQUAL _slicer_llvm_major)
        message(FATAL_ERROR
            "${SLICER_LLVM_${_tool_upper}} is LLVM ${CMAKE_MATCH_1}, but the compiler is "
            "Clang ${CMAKE_CXX_COMPILER_VERSION}; set SLICER_LLVM_${_tool_upper} to a matching tool")
    endif ()
endforeach ()

# Object-level compiler evidence lets the collector tell mapping-free
# translation units from executable code missing from the link.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(SLICER_COVERAGE_DIR "${CMAKE_BINARY_DIR}/slicer-coverage")
file(MAKE_DIRECTORY "${SLICER_COVERAGE_DIR}")
file(WRITE "${SLICER_COVERAGE_DIR}/toolchain.txt"
    "compiler=${CMAKE_CXX_COMPILER}\n"
    "compiler_id=${CMAKE_CXX_COMPILER_ID}\n"
    "compiler_version=${CMAKE_CXX_COMPILER_VERSION}\n"
    "llvm_cov=${SLICER_LLVM_COV}\n"
    "llvm_profdata=${SLICER_LLVM_PROFDATA}\n")

function(slicer_enable_llvm_source_coverage TARGET_NAME)
    if (NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "slicer_enable_llvm_source_coverage: unknown target '${TARGET_NAME}'")
    endif ()
    get_target_property(_target_type ${TARGET_NAME} TYPE)
    if (NOT _target_type MATCHES "^(STATIC_LIBRARY|EXECUTABLE)$")
        message(FATAL_ERROR
            "slicer_enable_llvm_source_coverage: '${TARGET_NAME}' is a ${_target_type}; "
            "only static libraries and executables are supported")
    endif ()

    # CMake's PCH wrapper marks included headers as system headers. That can
    # suppress inline coverage mappings while the instrumented code still
    # contributes profile counts, producing mismatched coverage data.
    set_property(TARGET ${TARGET_NAME} PROPERTY DISABLE_PRECOMPILE_HEADERS ON)

    target_compile_options(${TARGET_NAME} PRIVATE
        -fprofile-instr-generate -fcoverage-mapping -fprofile-update=atomic
        # Coverage builds include Orca's own headers as non-system headers (see
        # the root CMakeLists.txt), which exposes unused private fields that the
        # SYSTEM include normally hides. Keep them visible but not fatal.
        -Wno-error=unused-private-field)

    if (_target_type STREQUAL "STATIC_LIBRARY")
        # target_link_options() is not usable on static libraries, and
        # target_link_libraries() would have to pick the plain or keyword
        # signature the target's owner already uses. Appending to the interface
        # property propagates the runtime to every consuming executable.
        set_property(TARGET ${TARGET_NAME} APPEND PROPERTY
            INTERFACE_LINK_LIBRARIES "-fprofile-instr-generate")
    else ()
        target_link_options(${TARGET_NAME} PRIVATE -fprofile-instr-generate)
    endif ()

    get_target_property(_sources ${TARGET_NAME} SOURCES)
    get_target_property(_source_dir ${TARGET_NAME} SOURCE_DIR)
    if (NOT _sources)
        message(FATAL_ERROR "slicer_enable_llvm_source_coverage: '${TARGET_NAME}' has no sources")
    endif ()
    set(_manifest "${SLICER_COVERAGE_DIR}/${TARGET_NAME}.sources.txt")
    set(_contents "# Sources compiled into ${TARGET_NAME}\n")
    foreach (_source IN LISTS _sources)
        if (_source MATCHES "\\$<")
            message(FATAL_ERROR
                "slicer_enable_llvm_source_coverage: '${TARGET_NAME}' uses a generator "
                "expression in SOURCES ('${_source}'); the source manifest cannot list it")
        endif ()
        get_filename_component(_absolute "${_source}" ABSOLUTE BASE_DIR "${_source_dir}")
        if (_absolute MATCHES "\\.(c|cc|cpp|cxx|m|mm)$")
            string(APPEND _contents "${_absolute}\n")
        endif ()
    endforeach ()
    file(WRITE "${_manifest}" "${_contents}")
    set_property(GLOBAL APPEND PROPERTY SLICER_COVERAGE_MANIFESTS "${_manifest}")
endfunction()

# Link every object of LIBRARY_TARGET into TEST_TARGET. A static archive
# otherwise contributes only the objects the tests reference, so production
# files no test reaches would vanish from the denominator instead of being
# reported with zero hits. Per-executable reports need the complete denominator
# in each test image.
function(slicer_force_link_static_library TEST_TARGET LIBRARY_TARGET)
    if (NOT TARGET ${TEST_TARGET} OR NOT TARGET ${LIBRARY_TARGET})
        message(FATAL_ERROR "slicer_force_link_static_library: both targets must exist")
    endif ()
    get_target_property(_library_type ${LIBRARY_TARGET} TYPE)
    if (NOT _library_type STREQUAL "STATIC_LIBRARY")
        message(FATAL_ERROR "slicer_force_link_static_library: '${LIBRARY_TARGET}' is not a static library")
    endif ()
    # The test target already links the library (directly or transitively);
    # only build ordering and the whole-archive flag are added here, so the
    # target's existing target_link_libraries() signature is left alone.
    add_dependencies(${TEST_TARGET} ${LIBRARY_TARGET})
    if (APPLE)
        target_link_options(${TEST_TARGET} PRIVATE
            "LINKER:-force_load,$<TARGET_FILE:${LIBRARY_TARGET}>")
    else ()
        target_link_options(${TEST_TARGET} PRIVATE
            "SHELL:-Wl,--whole-archive \"$<TARGET_FILE:${LIBRARY_TARGET}>\" -Wl,--no-whole-archive")
    endif ()
endfunction()
