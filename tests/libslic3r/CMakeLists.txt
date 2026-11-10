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
    test_adaptive_pa.cpp
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
    test_cut_utils.cpp
    test_color.cpp
    test_color_decompose_recipe.cpp
    test_color_utils_contracts.cpp
    test_normal_utils_contracts.cpp
    test_config.cpp
    test_config_variant_expansion.cpp
    test_cut_surface_contracts.cpp
    test_parameter_utils.cpp
    test_locales_utils.cpp
    test_drc.cpp
    test_edge_grid.cpp
    test_gcodereader.cpp
    test_filament_group.cpp
    test_filament_group_utils.cpp
    test_tool_order_utils.cpp
    test_toolordering_nozzle_group.cpp
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
    test_gcode_class_contracts.cpp
    test_extrusion_entity.cpp
    test_extrusion_processor.cpp
    test_belt_transform.cpp
    test_intersection_points_contracts.cpp
    test_fill_corner_smoothing.cpp
    test_aabb_mesh.cpp
    test_filament_mixer.cpp
    test_flush_vol_predictor.cpp
    test_flush_volume_contracts.cpp
    test_fill_plane_path.cpp
    test_fill_tpms_adaptive.cpp
    test_fill_lateral_honeycomb.cpp
    test_flow.cpp
    test_fuzzy_skin.cpp
    test_small_area_infill_flow_compensator.cpp
    test_geometry.cpp
    test_multimaterial_segmentation.cpp
    test_placeholder_parser.cpp
    test_png_read_write.cpp
    test_point.cpp
    test_polygon.cpp
    test_polyline.cpp
    test_multipoint.cpp
    test_mutable_polygon.cpp
    test_mutable_priority_queue.cpp
    test_minimum_spanning_tree.cpp
    test_nozzle_volume_type.cpp
    test_obj.cpp
    test_obj_color_utils.cpp
    test_orient.cpp
    test_sl1.cpp
    test_slicing.cpp
    test_zipper_archive_import.cpp
    test_step.cpp
    test_stl.cpp
    test_nsvg_utils.cpp
    test_surface_collection.cpp
    test_svg.cpp
    test_svg_import.cpp
    test_short_edge_collapse.cpp
    test_triangle_selector.cpp
    test_measure_utils.cpp
    test_meshboolean.cpp
    test_trianglemesh.cpp
    test_trianglemesh_slicer.cpp
    test_marchingsquares.cpp
    test_texture_to_color_contracts.cpp
    test_texture_painting.cpp
    test_measure_contracts.cpp
    test_lay_on_face.cpp
    test_model.cpp
    test_project_task.cpp
    test_rotfinder.cpp
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
    test_texture_bake_regularize.cpp
    test_texture_bake_relocate.cpp
    test_texture_bake_debug.cpp
    test_texture_to_color_cluster.cpp
    test_texture_to_color_repair.cpp
    test_instance_lock.cpp
    ../libnest2d/printer_parts.cpp
    )

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
endif ()

if (TARGET OpenVDB::openvdb)
    target_sources(${_TEST_NAME}_tests PRIVATE test_hollowing.cpp)
endif()

# libslic3r links Draco privately; test_drc.cpp encodes its own fixtures.
find_package(Draco REQUIRED)
# FlushVolCalc obtains RGB2HSV from this production translation unit, which also
# contains wxColour helpers. Link only that unit and its base/core dependencies.
if (CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(wxWidgets_USE_UNICODE ON)
    set(wxWidgets_USE_STATIC ${SLIC3R_STATIC})
    set(wxWidgets_CONFIG_OPTIONS "--toolkit=gtk${SLIC3R_GTK}")
    find_package(wxWidgets 3.3 REQUIRED COMPONENTS base core)
else ()
    find_package(wxWidgets 3.3 CONFIG REQUIRED COMPONENTS base core)
endif ()
add_library(slic3r_test_color_support STATIC
    ${CMAKE_SOURCE_DIR}/src/slic3r/Utils/ColorSpaceConvert.cpp)
target_include_directories(slic3r_test_color_support SYSTEM PRIVATE ${wxWidgets_INCLUDE_DIRS})
target_compile_definitions(slic3r_test_color_support PRIVATE
    ${wxWidgets_DEFINITIONS} "$<$<CONFIG:Debug>:${wxWidgets_DEFINITIONS_DEBUG}>")
target_compile_options(slic3r_test_color_support PRIVATE ${wxWidgets_CXX_FLAGS})
# Preserve module-mode search/rpath usage on the final executable.
target_link_directories(slic3r_test_color_support INTERFACE ${wxWidgets_LIBRARY_DIRS})
if (SLIC3R_STATIC)
    # Match the canonical static wx dependency even in Debug.
    target_compile_definitions(slic3r_test_color_support PRIVATE wxDEBUG_LEVEL=0)
endif ()
target_link_libraries(slic3r_test_color_support PRIVATE Boost::headers ${wxWidgets_LIBRARIES})
target_link_libraries(${_TEST_NAME}_tests test_common libslic3r draco::draco Catch2::Catch2WithMain
    slic3r_test_color_support)
# The TextureToColor headers include "Point.hpp" without the libslic3r/ prefix.
set_property(SOURCE test_color_utils_contracts.cpp test_texture_to_color_cluster.cpp test_texture_to_color_contracts.cpp
                    test_texture_to_color_repair.cpp APPEND PROPERTY
    INCLUDE_DIRECTORIES "${CMAKE_SOURCE_DIR}/src/libslic3r")
target_include_directories(${_TEST_NAME}_tests PRIVATE ${CMAKE_SOURCE_DIR}/src)
set_property(TARGET ${_TEST_NAME}_tests PROPERTY FOLDER "tests")

orcaslicer_copy_test_dlls()

orcaslicer_discover_tests(${_TEST_NAME}_tests)

# Explicitly built by the inventory audit; it is not an extra Catch test.
add_executable(slicer_settings_inventory EXCLUDE_FROM_ALL settings_inventory_main.cpp)
target_link_libraries(slicer_settings_inventory libslic3r nlohmann_json)
target_include_directories(slicer_settings_inventory PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/deps_src)
set_property(TARGET slicer_settings_inventory PROPERTY FOLDER "tests")
