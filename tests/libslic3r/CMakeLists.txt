get_filename_component(_TEST_NAME ${CMAKE_CURRENT_LIST_DIR} NAME)

add_executable(${_TEST_NAME}_tests
    ${_TEST_NAME}_tests.cpp
    test_3mf.cpp
    test_amf.cpp
    # Round-trip seam metadata and active/dormant volume settings in both formats.
    test_precise_seam_3mf.cpp
    # Pure perimeter extraction is independent of Print/Layer fixtures.
    test_precise_seam.cpp
    test_aabbindirect.cpp
    test_appconfig.cpp
    test_arachne_walls.cpp
    test_arrange.cpp
    test_assemble_list.cpp
    test_assimp_import.cpp
    test_bambu_networking.cpp
    test_belt_brim.cpp
    test_bridge_detector.cpp
    test_buildvolume.cpp
    test_calib.cpp
    test_circle.cpp
    test_clipper_offset.cpp
    test_clipper_utils.cpp
    test_color.cpp
    test_color_decompose_recipe.cpp
    test_rotfinder.cpp
    test_color_utils_contracts.cpp
    test_normal_utils_contracts.cpp
    test_config.cpp
    test_config_variant_expansion.cpp
    test_parameter_utils.cpp
    test_cut_surface_contracts.cpp
    test_surface_collection.cpp
    test_cut_utils.cpp
    test_texture_painting.cpp
    test_texture_bake_regularize.cpp
    test_texture_bake_relocate.cpp
    test_texture_bake_debug.cpp
    test_orient.cpp
    test_locales_utils.cpp
    test_drc.cpp
    test_edge_grid.cpp
    test_gcodereader.cpp
    test_toolordering_nozzle_group.cpp
    test_tool_order_utils.cpp
    test_parallel_resolve.cpp
    test_preset_bundle_loading.cpp
    test_preset_setting_id.cpp
    test_preset_diff.cpp
    test_printer_preset_collection.cpp
    test_vendor_cache.cpp
    test_preset_options.cpp
    test_elephant_foot_compensation.cpp
    test_emboss_contracts.cpp
    test_text_shape.cpp
    test_expolygon_contracts.cpp
    test_expolygons_index_contracts.cpp
    test_extrusion_simulator.cpp
    test_flush_volume_contracts.cpp
    test_intersection_points_contracts.cpp
    test_fill_corner_smoothing.cpp
    test_fuzzy_skin.cpp
    test_slicing.cpp
    test_aabb_mesh.cpp
    test_filament_group.cpp
    test_filament_group_utils.cpp
    test_filament_mixer.cpp
    test_fill_plane_path.cpp
    test_fill_tpms_adaptive.cpp
    test_geometry.cpp
    test_multimaterial_segmentation.cpp
    test_placeholder_parser.cpp
    test_png_read_write.cpp
    test_polygon.cpp
    test_polyline.cpp
    test_multipoint.cpp
    test_mutable_polygon.cpp
    test_mutable_priority_queue.cpp
    test_minimum_spanning_tree.cpp
    test_nozzle_volume_type.cpp
    test_obj.cpp
    test_obj_color_utils.cpp
    test_sl1.cpp
    test_zipper_archive_import.cpp
    test_step.cpp
    test_stl.cpp
    test_svg.cpp
    test_nsvg_utils.cpp
    test_triangle_selector.cpp
    test_meshboolean.cpp
    test_trianglemesh.cpp
    test_trianglemesh_slicer.cpp
    test_marchingsquares.cpp
    test_measure_contracts.cpp
    test_lay_on_face.cpp
    test_model.cpp
    test_project_task.cpp
    test_utils.cpp
    test_miniz_extension.cpp
    test_timeutils.cpp
    test_voronoi.cpp
    test_voxel_utils.cpp
    test_wipe_tower_estimate.cpp
    test_wipe_tower.cpp
    test_wipe_path.cpp
    test_optimizers.cpp
    test_ordering_strategies.cpp
    # test_png_io.cpp
    test_indexed_triangle_set.cpp
    test_connected_bodies.cpp
    test_texture_displacement.cpp
    test_texture_to_color_contracts.cpp
    test_instance_lock.cpp
    ../libnest2d/printer_parts.cpp
    )

# TriMesh.hpp's public includes are relative to the libslic3r include root.
set_property(SOURCE test_color_utils_contracts.cpp test_texture_to_color_contracts.cpp APPEND PROPERTY
    INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/src/libslic3r")

if (SLIC3R_CAD)
    target_sources(${_TEST_NAME}_tests PRIVATE
        test_caddocument.cpp
        test_sketchengine.cpp
        test_sketchconstraints.cpp
        test_sketchedit.cpp
        test_sketchprofile.cpp
        test_sketchimport.cpp
        test_sketchinference.cpp
        test_slvs_constraints.cpp)

    # Explicitly built sketch-engine contracts, without duplicate CTest registration.
    add_executable(libslic3r_sketch_engine_tests EXCLUDE_FROM_ALL test_sketchengine.cpp)
    target_link_libraries(libslic3r_sketch_engine_tests test_common libslic3r Catch2::Catch2WithMain)
    target_include_directories(libslic3r_sketch_engine_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
    set_property(TARGET libslic3r_sketch_engine_tests PROPERTY FOLDER "tests")
    orcaslicer_copy_test_dlls(libslic3r_sketch_engine_tests)
    slicer_enable_llvm_source_coverage(libslic3r_sketch_engine_tests)
endif ()

if (TARGET OpenVDB::openvdb)
    target_sources(${_TEST_NAME}_tests PRIVATE test_hollowing.cpp)
endif()

# libslic3r links Draco privately; test_drc.cpp encodes its own fixtures.
find_package(Draco REQUIRED)
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r draco::draco Catch2::Catch2WithMain)
target_include_directories(${_TEST_NAME}_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)

# Focused Preset bundle tests, without duplicate CTest registration.
# Keep the four-line NanoSVG implementation available to linked libslic3r objects.
add_executable(libslic3r_preset_bundle_tests EXCLUDE_FROM_ALL
    test_preset_bundle_loading.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_preset_bundle_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_preset_bundle_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_preset_bundle_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_preset_bundle_tests)
slicer_enable_llvm_source_coverage(libslic3r_preset_bundle_tests)

# Focused preset collection contracts without duplicate CTest registration.
# Reuse the standalone NanoSVG provider required by the linked libslic3r archive.
add_executable(libslic3r_preset_collection_tests EXCLUDE_FROM_ALL
    test_preset_diff.cpp
    test_printer_preset_collection.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_preset_collection_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_preset_collection_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_preset_collection_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_preset_collection_tests)
slicer_enable_llvm_source_coverage(libslic3r_preset_collection_tests)

# Explicitly built .3mf format tests, without duplicate CTest registration.
# Reuse the standalone NanoSVG implementation needed by linked libslic3r objects.
add_executable(libslic3r_3mf_tests EXCLUDE_FROM_ALL
    test_3mf.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_3mf_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_3mf_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_3mf_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_3mf_tests)
slicer_enable_llvm_source_coverage(libslic3r_3mf_tests)

# Explicitly built parser tests, without duplicate CTest registration.
add_executable(libslic3r_placeholder_parser_tests EXCLUDE_FROM_ALL test_placeholder_parser.cpp)
target_link_libraries(libslic3r_placeholder_parser_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_placeholder_parser_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_placeholder_parser_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_placeholder_parser_tests)
slicer_enable_llvm_source_coverage(libslic3r_placeholder_parser_tests)

# Explicitly built Model tests, without duplicate CTest registration.
# Reuse the standalone NanoSVG implementation needed by Model's import paths.
add_executable(libslic3r_model_tests EXCLUDE_FROM_ALL
    test_model.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_model_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_model_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_model_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_model_tests)
slicer_enable_llvm_source_coverage(libslic3r_model_tests)

# Focused ToolOrdering contracts, reusing the existing TU and NanoSVG implementation without CTest registration.
add_executable(libslic3r_toolordering_tests EXCLUDE_FROM_ALL
    test_toolordering_nozzle_group.cpp
    libslic3r_tests.cpp)
target_link_libraries(libslic3r_toolordering_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_toolordering_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_toolordering_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_toolordering_tests)
slicer_enable_llvm_source_coverage(libslic3r_toolordering_tests)

# Explicitly built Arachne contracts, without duplicate CTest registration.
add_executable(libslic3r_arachne_tests EXCLUDE_FROM_ALL test_arachne_walls.cpp)
target_link_libraries(libslic3r_arachne_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_arachne_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_arachne_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_arachne_tests)
slicer_enable_llvm_source_coverage(libslic3r_arachne_tests)

# Explicitly built texture-displacement contracts, without duplicate CTest registration.
# Model import paths need the existing standalone NanoSVG implementation.
add_executable(libslic3r_texture_displacement_tests EXCLUDE_FROM_ALL
    test_texture_displacement.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_texture_displacement_tests test_common libslic3r Catch2::Catch2WithMain)
set_property(TARGET libslic3r_texture_displacement_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_texture_displacement_tests)
slicer_enable_llvm_source_coverage(libslic3r_texture_displacement_tests)

# Explicitly built TextureToColor contracts, without duplicate CTest registration.
add_executable(libslic3r_texture_to_color_tests EXCLUDE_FROM_ALL test_texture_to_color_contracts.cpp)
target_link_libraries(libslic3r_texture_to_color_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_texture_to_color_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_texture_to_color_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_texture_to_color_tests)
slicer_enable_llvm_source_coverage(libslic3r_texture_to_color_tests)

# Explicitly built TextureBake stage-debug contracts, without duplicate CTest registration.
add_executable(libslic3r_texture_bake_debug_tests EXCLUDE_FROM_ALL test_texture_bake_debug.cpp)
target_link_libraries(libslic3r_texture_bake_debug_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_texture_bake_debug_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_texture_bake_debug_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_texture_bake_debug_tests)
slicer_enable_llvm_source_coverage(libslic3r_texture_bake_debug_tests)

# Focused ISO 8601 millisecond conversion contracts, without duplicate CTest registration.
add_executable(libslic3r_timeutils_tests EXCLUDE_FROM_ALL test_timeutils.cpp)
target_link_libraries(libslic3r_timeutils_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_timeutils_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_timeutils_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_timeutils_tests)
slicer_enable_llvm_source_coverage(libslic3r_timeutils_tests)

# Focused flush predictor contracts, without duplicate CTest registration.
add_executable(libslic3r_flush_volume_tests EXCLUDE_FROM_ALL test_flush_volume_contracts.cpp)
target_link_libraries(libslic3r_flush_volume_tests test_common libslic3r slicer_coverage_support Catch2::Catch2WithMain)
target_include_directories(libslic3r_flush_volume_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_flush_volume_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_flush_volume_tests)
slicer_enable_llvm_source_coverage(libslic3r_flush_volume_tests)

# Explicitly built PNG writer/scaling tests, without duplicate CTest registration.
add_executable(libslic3r_png_tests EXCLUDE_FROM_ALL test_png_read_write.cpp)
target_link_libraries(libslic3r_png_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_png_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_png_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_png_tests)
slicer_enable_llvm_source_coverage(libslic3r_png_tests)

# Focused SVG export contracts, without duplicate CTest registration.
# Keep the standalone NanoSVG implementation TU used by SVG import tests.
add_executable(libslic3r_svg_tests EXCLUDE_FROM_ALL
    test_svg.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_svg_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_svg_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_svg_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_svg_tests)
slicer_enable_llvm_source_coverage(libslic3r_svg_tests)

# Explicitly built orientation contracts, without duplicate CTest registration.
add_executable(libslic3r_orient_tests EXCLUDE_FROM_ALL
    test_orient.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_orient_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_orient_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_orient_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_orient_tests)
slicer_enable_llvm_source_coverage(libslic3r_orient_tests)

# Explicitly built ExPolygon contract tests, without duplicate CTest registration.
add_executable(libslic3r_expolygon_tests EXCLUDE_FROM_ALL
    test_expolygon_contracts.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_expolygon_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_expolygon_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_expolygon_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_expolygon_tests)
slicer_enable_llvm_source_coverage(libslic3r_expolygon_tests)

# Explicitly built EdgeGrid tests, without duplicate CTest registration.
add_executable(libslic3r_edgegrid_tests EXCLUDE_FROM_ALL
    test_edge_grid.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_edgegrid_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_edgegrid_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_edgegrid_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_edgegrid_tests)
slicer_enable_llvm_source_coverage(libslic3r_edgegrid_tests)

# Explicitly built utility contracts, without duplicate CTest registration.
add_executable(libslic3r_filament_group_utils_tests EXCLUDE_FROM_ALL
    test_filament_group_utils.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_filament_group_utils_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_filament_group_utils_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_filament_group_utils_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_filament_group_utils_tests)
slicer_enable_llvm_source_coverage(libslic3r_filament_group_utils_tests)

# Keep the shared polygon fixtures available to the focused geometry tests.
add_executable(libslic3r_geometry_tests EXCLUDE_FROM_ALL
    test_geometry.cpp
    test_polygon.cpp
    test_trianglemesh_slicer.cpp
    ../libnest2d/printer_parts.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_geometry_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_geometry_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_geometry_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_geometry_tests)
slicer_enable_llvm_source_coverage(libslic3r_geometry_tests)

# Explicitly built Voronoi tests, without duplicate CTest registration.
add_executable(libslic3r_voronoi_tests EXCLUDE_FROM_ALL
    test_voronoi.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_voronoi_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_voronoi_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_voronoi_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_voronoi_tests)
slicer_enable_llvm_source_coverage(libslic3r_voronoi_tests)

# Focused Config tests, without duplicate CTest registration.
# Reuse the standalone NanoSVG implementation TU needed by linked libslic3r objects.
add_executable(libslic3r_config_tests EXCLUDE_FROM_ALL
    test_config.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_config_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_config_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_config_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_config_tests)
slicer_enable_llvm_source_coverage(libslic3r_config_tests)

# Focused config-variant tests, without duplicate CTest registration.
# Use the standalone NanoSVG provider required by the linked libslic3r archive.
add_executable(libslic3r_config_variant_expansion_tests EXCLUDE_FROM_ALL
    test_config_variant_expansion.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_config_variant_expansion_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_config_variant_expansion_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_config_variant_expansion_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_config_variant_expansion_tests)
slicer_enable_llvm_source_coverage(libslic3r_config_variant_expansion_tests)

# Explicitly built by the inventory audit; it is not an extra Catch test.
add_executable(slicer_settings_inventory EXCLUDE_FROM_ALL settings_inventory_main.cpp)
target_link_libraries(slicer_settings_inventory libslic3r nlohmann_json)
target_include_directories(slicer_settings_inventory PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/deps_src)
set_property(TARGET slicer_settings_inventory PROPERTY FOLDER "tests")


# Explicitly built SurfaceCollection contracts, without duplicate CTest registration.
# The standalone NanoSVG implementation satisfies SVG symbols pulled by libslic3r.
add_executable(libslic3r_surface_collection_tests EXCLUDE_FROM_ALL
    test_surface_collection.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_surface_collection_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_surface_collection_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_surface_collection_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_surface_collection_tests)
slicer_enable_llvm_source_coverage(libslic3r_surface_collection_tests)

# Focused TriangleMesh contracts, without duplicate CTest registration.
# Keep the established FFF test helpers and printer-parts fixture object available.
add_executable(libslic3r_trianglemesh_tests EXCLUDE_FROM_ALL
    test_trianglemesh.cpp
    ../fff_print/fff_print_tests.cpp
    ../fff_print/test_helpers.cpp
    ../libnest2d/printer_parts.cpp)
target_link_libraries(libslic3r_trianglemesh_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_trianglemesh_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_trianglemesh_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_trianglemesh_tests)
slicer_enable_llvm_source_coverage(libslic3r_trianglemesh_tests)

# Focused TriangleMeshSlicer contour-repair contracts, without duplicate CTest registration.
# Reuse the standalone NanoSVG implementation needed by linked libslic3r objects.
add_executable(libslic3r_trianglemesh_slicer_tests EXCLUDE_FROM_ALL
    test_trianglemesh_slicer.cpp
    ../fff_print/fff_print_tests.cpp)
target_link_libraries(libslic3r_trianglemesh_slicer_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_trianglemesh_slicer_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_trianglemesh_slicer_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_trianglemesh_slicer_tests)
slicer_enable_llvm_source_coverage(libslic3r_trianglemesh_slicer_tests)

# Focused ZIP-path and archive-diagnostic contracts, without duplicate CTest registration.
# Reuse the accepted FFF helper/printer-parts link support for the full test TU.
add_executable(libslic3r_miniz_extension_tests EXCLUDE_FROM_ALL
    test_miniz_extension.cpp
    ../fff_print/fff_print_tests.cpp
    ../fff_print/test_helpers.cpp
    ../libnest2d/printer_parts.cpp)
target_link_libraries(libslic3r_miniz_extension_tests test_common libslic3r Catch2::Catch2WithMain)
target_include_directories(libslic3r_miniz_extension_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET libslic3r_miniz_extension_tests PROPERTY FOLDER "tests")
orcaslicer_copy_test_dlls(libslic3r_miniz_extension_tests)
slicer_enable_llvm_source_coverage(libslic3r_miniz_extension_tests)
