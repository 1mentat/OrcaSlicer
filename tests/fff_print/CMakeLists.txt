get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)
add_executable(${_TEST_NAME}_tests 
	${_TEST_NAME}_tests.cpp
	test_helpers.cpp
	test_helpers.hpp
	test_cooling.cpp
	test_extrusion_entity.cpp
	test_extrusion_processor.cpp
	test_fill.cpp
	test_flow.cpp
	test_gcode_processor.cpp
	test_gcode_timing.cpp
	test_gcode.cpp
	test_timelapse_pos_picker.cpp
	test_gcodeprocessor.cpp
    test_small_area_infill_flow_compensator.cpp
	test_gcodewriter.cpp
	test_interlocking.cpp
	test_model.cpp
	test_multifilament.cpp
    test_layer_height_variance.cpp
    test_painted_segmentation.cpp
	test_perimeters.cpp
    test_pipeline_invariants.cpp
    test_boolean_settings.cpp
	test_settings_variance.cpp
	test_gcode_settings_contracts.cpp
	test_fan_mover_contracts.cpp
	test_motion_flow_variance.cpp
	test_pressure_equalizer.cpp
	# Seam geometry and pipeline regressions share the existing Print fixtures.
	test_precise_seam.cpp
	test_print.cpp
	test_printobject.cpp
	test_mixed_filament.cpp
	test_seam_placer.cpp
	test_skirt_brim.cpp
	test_slicing_pipeline_hook.cpp
	test_support_material.cpp
	test_texture_displacement.cpp
	test_thumbnails.cpp
	test_tree_support.cpp
	test_trianglemesh.cpp
	test_wipe.cpp
	test_wipe_tower.cpp
	)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r Catch2::Catch2WithMain)
# PrintExtents.hpp uses a library-root include that libslic3r exposes privately.
set_property(SOURCE test_gcode.cpp APPEND PROPERTY INCLUDE_DIRECTORIES
    "${CMAKE_SOURCE_DIR}/src/libslic3r")
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)

# Explicitly built GCodeProcessor tests, without duplicate CTest registration.
add_executable(fff_print_gcodeprocessor_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_gcodeprocessor.cpp)
target_link_libraries(fff_print_gcodeprocessor_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_gcodeprocessor_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_gcodeprocessor_tests)
slicer_enable_llvm_source_coverage(fff_print_gcodeprocessor_tests)

# Explicitly built TimelapsePosPicker contract tests, without duplicate CTest registration.
add_executable(fff_print_timelapse_pos_picker_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_timelapse_pos_picker.cpp)
target_link_libraries(fff_print_timelapse_pos_picker_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_timelapse_pos_picker_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_timelapse_pos_picker_tests)
slicer_enable_llvm_source_coverage(fff_print_timelapse_pos_picker_tests)
