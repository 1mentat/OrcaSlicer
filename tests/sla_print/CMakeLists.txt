get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests ${_TEST_NAME}_tests_main.cpp
    sla_print_tests.cpp
    test_sla_pipeline_contracts.cpp
    sla_test_utils.hpp sla_test_utils.cpp
    sla_supptgen_tests.cpp
    sla_raycast_tests.cpp)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

# Explicitly built focused SLA support-tree contracts, without duplicate CTest registration.
add_executable(sla_pipeline_tests EXCLUDE_FROM_ALL
    test_sla_pipeline_contracts.cpp
    sla_print_tests_main.cpp)
target_link_libraries(sla_pipeline_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(sla_pipeline_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET sla_pipeline_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(sla_pipeline_tests)
slicer_enable_llvm_source_coverage(sla_pipeline_tests)

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)
