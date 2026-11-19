# Applied as PATCH_COMMAND for Boost 1.84.0.
#
# Boost.Locale groups its headers for IDEs with
#     source_group(TREE ".../include/boost/locale" ... FILES ${sources_inc})
# but the list also holds include/boost/locale.hpp, which lies beside that directory, not
# under it. CMake used to compare the two as plain strings and let it through; CMake 4.5
# compares them as paths and stops the configure. Root the tree one level up, which
# holds every listed header.

set(_lists "libs/locale/CMakeLists.txt")
if(NOT EXISTS "${_lists}")
    message(FATAL_ERROR "[Boost.Locale patch] Not found: ${_lists}")
endif()
file(READ "${_lists}" _content)
set(_old "source_group(TREE \"\${CMAKE_CURRENT_SOURCE_DIR}/include/boost/locale\" PREFIX")
set(_new "source_group(TREE \"\${CMAKE_CURRENT_SOURCE_DIR}/include/boost\" PREFIX")
string(FIND "${_content}" "${_new}" _done)
if(_done EQUAL -1)
    string(REPLACE "${_old}" "${_new}" _patched "${_content}")
    if(_patched STREQUAL _content)
        message(FATAL_ERROR "[Boost.Locale patch] Failed to match the header source_group in ${_lists}")
    endif()
    file(WRITE "${_lists}" "${_patched}")
endif()
