# LLVM source-based coverage for a production target and its test executable.
#
# This module is opt-in.  The caller should invoke, after the target is defined:
#
#   include(SlicerCoverage)
#   slicer_enable_llvm_source_coverage(libslic3r)
#   slicer_enable_llvm_source_coverage(libslic3r_tests)
#   slicer_force_link_static_library(libslic3r_tests libslic3r)
#
# The Python runner consumes the generated
# `${CMAKE_BINARY_DIR}/slicer-coverage/<target>.sources.txt` manifest.  Keeping
# the manifest from the target's SOURCES property is what makes the denominator
# match the actual conditional build, rather than a guessed source glob.

include_guard(GLOBAL)

option(SLICER_ENABLE_LLVM_SOURCE_COVERAGE
       "Instrument selected OrcaSlicer targets with LLVM source coverage" OFF)

option(SLICER_COVERAGE_PROJECT_FUNCTIONS_ONLY
       "Instrument only function definitions under CMAKE_SOURCE_DIR/src in LLVM coverage targets" OFF)

if(SLICER_ENABLE_LLVM_SOURCE_COVERAGE)
    # Object-level compiler evidence distinguishes mapping-free translation
    # units from executable code that was accidentally omitted from the link.
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

function(slicer_enable_llvm_source_coverage TARGET_NAME)
    if (NOT SLICER_ENABLE_LLVM_SOURCE_COVERAGE)
        return()
    endif()
    if (NOT TARGET ${TARGET_NAME})
        message(FATAL_ERROR "slicer_enable_llvm_source_coverage: unknown target '${TARGET_NAME}'")
    endif()

    # GCC's gcov flags are intentionally not substituted here: the runner
    # consumes LLVM's source mapping JSON and must fail loudly when that
    # contract is unavailable.  clang-cl is the one MSVC frontend supported.
    if (NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang|AppleClang" AND
        NOT (MSVC AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "Clang"))
        message(FATAL_ERROR
            "SLICER_ENABLE_LLVM_SOURCE_COVERAGE requires Clang/AppleClang or clang-cl; "
            "got ${CMAKE_CXX_COMPILER_ID} (${CMAKE_CXX_COMPILER_FRONTEND_VARIANT})")
    endif()

    # LLVM's source-based mapping is supported by clang-cl too, but the linker
    # flags differ from MSVC's normal options only for the whole-archive helper
    # below.  Keep the flags target-local so dependencies are not instrumented.
    # Slicing runs concurrently: lost non-atomic increments can make LLVM's
    # derived counters credit branches that never executed.
    target_compile_options(${TARGET_NAME} PRIVATE
        -fprofile-instr-generate -fcoverage-mapping -fprofile-update=atomic)
    # Coverage includes Orca headers as project headers to retain their inline
    # mappings. This exposes existing unused private fields that normal SYSTEM
    # includes hide. Keep these diagnostics visible without blocking test-only
    # overlays on older production trees; other warnings remain errors.
    target_compile_options(${TARGET_NAME} PRIVATE
        "$<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-Wno-error=unused-private-field>")
    get_target_property(_target_type ${TARGET_NAME} TYPE)
    if (NOT _target_type STREQUAL "STATIC_LIBRARY")
        target_link_options(${TARGET_NAME} PRIVATE -fprofile-instr-generate)
    else()
        # CMake does not allow target_link_options() on a STATIC_LIBRARY.
        # Propagate the runtime flag to every executable that consumes this
        # archive, including the force-linked coverage test image.
        target_link_libraries(${TARGET_NAME} INTERFACE "-fprofile-instr-generate")
    endif()

    get_target_property(_sources ${TARGET_NAME} SOURCES)
    get_target_property(_source_dir ${TARGET_NAME} SOURCE_DIR)
    if (NOT _sources)
        message(FATAL_ERROR "slicer_enable_llvm_source_coverage: target '${TARGET_NAME}' has no sources")
    endif()

    set(_manifest_dir "${CMAKE_BINARY_DIR}/slicer-coverage")
    file(MAKE_DIRECTORY "${_manifest_dir}")
    # This optional profile list changes only which functions receive counters;
    # the target's source manifest and normal coverage flags remain active. Keep
    # CMAKE_SOURCE_DIR's configured spelling (including a
    # symlink alias) so it matches the paths clang sees in this build.
    if (SLICER_COVERAGE_PROJECT_FUNCTIONS_ONLY)
        set(_profile_list "${_manifest_dir}/project-functions-only.profiler-list")
        set(_profile_list_contents
            "[clang]\nsource:${CMAKE_SOURCE_DIR}/src/*=allow\ndefault:forbid\n")
        if (EXISTS "${_profile_list}")
            file(READ "${_profile_list}" _profile_list_existing_contents)
        else()
            set(_profile_list_existing_contents "")
        endif()
        # Clang records this file as an input dependency. Preserve its mtime on
        # an identical configure so Ninja does not rebuild every instrumented TU.
        if (NOT "${_profile_list_existing_contents}" STREQUAL "${_profile_list_contents}")
            file(WRITE "${_profile_list}" "${_profile_list_contents}")
        endif()
        # One CMake argument preserves the profile-list path if the build path
        # contains spaces.
        target_compile_options(${TARGET_NAME} PRIVATE "-fprofile-list=${_profile_list}")
    endif()

    set(_manifest "${_manifest_dir}/${TARGET_NAME}.sources.txt")
    file(WRITE "${_manifest}" "# Sources compiled into ${TARGET_NAME}\n")
    foreach(_source IN LISTS _sources)
        if (IS_ABSOLUTE "${_source}")
            set(_absolute "${_source}")
        else()
            set(_absolute "${_source_dir}/${_source}")
        endif()
        get_filename_component(_extension "${_absolute}" LAST_EXT)
        if (_extension MATCHES "^\\.(c|cc|cpp|cxx|m|mm)$")
            file(APPEND "${_manifest}" "${_absolute}\n")
        endif()
    endforeach()

    # A global property gives the root build a discoverable list of manifests;
    # callers can also use the target-specific variable directly.
    set_property(GLOBAL APPEND PROPERTY SLICER_COVERAGE_MANIFESTS "${_manifest}")
    string(MAKE_C_IDENTIFIER "${TARGET_NAME}" _target_identifier)
    set("SLICER_COVERAGE_${_target_identifier}_MANIFEST" "${_manifest}" CACHE FILEPATH
        "LLVM source manifest for ${TARGET_NAME}")
endfunction()

function(slicer_force_link_static_library TEST_TARGET LIBRARY_TARGET)
    if (NOT SLICER_ENABLE_LLVM_SOURCE_COVERAGE)
        return()
    endif()
    if (NOT TARGET ${TEST_TARGET} OR NOT TARGET ${LIBRARY_TARGET})
        message(FATAL_ERROR "slicer_force_link_static_library: both targets must exist")
    endif()

    # Keep the target dependency and transitive libraries even for test suites
    # that normally exercise a separate library (for example libnest2d).
    target_link_libraries(${TEST_TARGET} ${LIBRARY_TARGET})

    # Static archives otherwise contribute only object files containing symbols
    # reached by the tests.  Whole-archive keeps every instrumented production
    # object in the test image, including objects with zero runtime hits.
    if (MSVC)
        target_link_options(${TEST_TARGET} PRIVATE
            "/WHOLEARCHIVE:$<TARGET_FILE:${LIBRARY_TARGET}>")
    elseif (APPLE)
        target_link_options(${TEST_TARGET} PRIVATE
            "-Wl,-force_load,$<TARGET_FILE:${LIBRARY_TARGET}>")
    else()
        target_link_options(${TEST_TARGET} PRIVATE
            "SHELL:-Wl,--whole-archive \"$<TARGET_FILE:${LIBRARY_TARGET}>\" -Wl,--no-whole-archive")
    endif()
endfunction()
