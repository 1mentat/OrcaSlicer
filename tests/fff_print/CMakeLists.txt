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
	test_gcode_motion_settings.cpp
	test_gcode_contract_helpers.hpp
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
	test_adaptive_bed_mesh.cpp
	test_fan_mover_contracts.cpp
	test_motion_flow_variance.cpp
	test_pressure_equalizer.cpp
	# Seam geometry and pipeline regressions share the existing Print fixtures.
	test_precise_seam.cpp
	test_print.cpp
	test_print_validation_clearance.cpp
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
    test_gcodeprocessor.cpp
    test_gcode_processor.cpp)
target_link_libraries(fff_print_gcodeprocessor_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_gcodeprocessor_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_gcodeprocessor_tests)
slicer_enable_llvm_source_coverage(fff_print_gcodeprocessor_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_gcodeprocessor_tests TEST_PREFIX "FocusedGCodeProcessor::")
endif()

# Explicitly built TimelapsePosPicker contract tests, without duplicate CTest registration.
add_executable(fff_print_timelapse_pos_picker_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_timelapse_pos_picker.cpp)
target_link_libraries(fff_print_timelapse_pos_picker_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_timelapse_pos_picker_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_timelapse_pos_picker_tests)
slicer_enable_llvm_source_coverage(fff_print_timelapse_pos_picker_tests)

# Focused painted-segmentation contracts, without duplicate CTest registration.
add_executable(fff_print_painted_segmentation_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_painted_segmentation.cpp)
target_link_libraries(fff_print_painted_segmentation_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_painted_segmentation_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_painted_segmentation_tests)
slicer_enable_llvm_source_coverage(fff_print_painted_segmentation_tests)

# Explicitly built Print workflow tests, including object-sharing contracts; no duplicate CTest registration.
add_executable(fff_print_workflow_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_print.cpp)
target_link_libraries(fff_print_workflow_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_workflow_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_workflow_tests)
slicer_enable_llvm_source_coverage(fff_print_workflow_tests)

# Explicit G-code export contracts, without duplicate CTest registration.
add_executable(fff_print_gcode_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_gcode.cpp)
target_link_libraries(fff_print_gcode_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_gcode_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_gcode_tests)
slicer_enable_llvm_source_coverage(fff_print_gcode_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_gcode_tests TEST_PREFIX "FocusedGCode::")
endif()

# Focused motion-setting G-code contracts, without duplicate default CTest registration.
add_executable(fff_print_motion_settings_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_gcode_motion_settings.cpp
    test_gcode_contract_helpers.hpp)
target_link_libraries(fff_print_motion_settings_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_motion_settings_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_motion_settings_tests)
slicer_enable_llvm_source_coverage(fff_print_motion_settings_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_motion_settings_tests TEST_PREFIX "FocusedMotionSettings::")
endif ()

# Focused cooling contracts, without duplicate default CTest registrations.
add_executable(fff_print_cooling_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_cooling.cpp)
target_link_libraries(fff_print_cooling_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_cooling_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_cooling_tests)
slicer_enable_llvm_source_coverage(fff_print_cooling_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_cooling_tests TEST_PREFIX "FocusedCooling::")
endif()

# Focused adaptive bed-mesh Print-to-G-code contracts, without duplicate CTest registration.
add_executable(fff_print_adaptive_bed_mesh_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_adaptive_bed_mesh.cpp)
target_link_libraries(fff_print_adaptive_bed_mesh_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_adaptive_bed_mesh_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_adaptive_bed_mesh_tests)
slicer_enable_llvm_source_coverage(fff_print_adaptive_bed_mesh_tests)

# Focused infill contracts and runtime comparisons, without duplicate CTest registration.
add_executable(fff_print_fill_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_fill.cpp)
target_link_libraries(fff_print_fill_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_fill_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_fill_tests)
slicer_enable_llvm_source_coverage(fff_print_fill_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_fill_tests TEST_PREFIX "FocusedFill::")
endif()

# Focused pressure-equalizer behavior, without duplicate default CTest registration.
add_executable(fff_print_pressure_equalizer_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_pressure_equalizer.cpp)
target_link_libraries(fff_print_pressure_equalizer_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_pressure_equalizer_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_pressure_equalizer_tests)
slicer_enable_llvm_source_coverage(fff_print_pressure_equalizer_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_pressure_equalizer_tests TEST_PREFIX "FocusedPressureEqualizer::")
endif()

# Focused thumbnail encoding and export contracts, without duplicate default CTest registration.
add_executable(fff_print_thumbnails_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_thumbnails.cpp)
target_link_libraries(fff_print_thumbnails_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_thumbnails_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_thumbnails_tests)
slicer_enable_llvm_source_coverage(fff_print_thumbnails_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_thumbnails_tests TEST_PREFIX "FocusedThumbnails::")
endif()

# Focused TreeSupport pipeline tests, without duplicate CTest registration.
add_executable(fff_print_tree_support_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_tree_support.cpp)
target_link_libraries(fff_print_tree_support_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_tree_support_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_tree_support_tests)
slicer_enable_llvm_source_coverage(fff_print_tree_support_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_tree_support_tests TEST_PREFIX "FocusedTreeSupport::")
endif()

# Focused filament-assignment contracts, without duplicate default CTest registration.
add_executable(fff_print_multifilament_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_multifilament.cpp)
target_link_libraries(fff_print_multifilament_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_multifilament_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_multifilament_tests)
slicer_enable_llvm_source_coverage(fff_print_multifilament_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_multifilament_tests TEST_PREFIX "FocusedMultiFilament::")
endif()

# Focused WipeTower contracts, without duplicate CTest registration.
add_executable(fff_print_wipe_tower_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_wipe_tower.cpp)
target_link_libraries(fff_print_wipe_tower_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_wipe_tower_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_wipe_tower_tests)
slicer_enable_llvm_source_coverage(fff_print_wipe_tower_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_wipe_tower_tests TEST_PREFIX "FocusedWipeTower::")
endif()

# Focused perimeter contracts, without duplicate CTest registration.
add_executable(fff_print_perimeters_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_perimeters.cpp)
target_link_libraries(fff_print_perimeters_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_perimeters_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_perimeters_tests)
slicer_enable_llvm_source_coverage(fff_print_perimeters_tests)

# Focused extrusion-entity contracts, without duplicate CTest registration.
add_executable(fff_print_extrusion_entity_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_extrusion_entity.cpp)
target_link_libraries(fff_print_extrusion_entity_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET fff_print_extrusion_entity_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_extrusion_entity_tests)
slicer_enable_llvm_source_coverage(fff_print_extrusion_entity_tests)

# Opt-in composite target for the current focused PR selection. Keep one FFF
# main (and therefore one NanoSVG implementation) and register shared helpers once.
add_executable(fff_print_composite_focused_tests EXCLUDE_FROM_ALL
    fff_print_tests.cpp
    test_helpers.cpp
    test_gcode.cpp
    test_gcode_motion_settings.cpp
    test_fill.cpp
    ../libslic3r/test_arachne_walls.cpp
    test_pressure_equalizer.cpp
    test_thumbnails.cpp
    ../libslic3r/test_utils.cpp
    test_gcode_processor.cpp
    test_precise_seam.cpp
    test_tree_support.cpp)
target_link_libraries(fff_print_composite_focused_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(fff_print_composite_focused_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET fff_print_composite_focused_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(fff_print_composite_focused_tests)
slicer_enable_llvm_source_coverage(fff_print_composite_focused_tests)
if (SLICER_ENABLE_FOCUSED_PR_CTEST_DISCOVERY)
    orcaslicer_discover_tests(fff_print_composite_focused_tests TEST_PREFIX "FocusedComposite::")
endif()
