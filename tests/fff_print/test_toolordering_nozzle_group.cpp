#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <boost/filesystem/operations.hpp>
#include <initializer_list>
#include <vector>
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_utils.hpp"

using namespace Slic3r;
using namespace Slic3r::MultiNozzleUtils;

TEST_CASE("Sequential selector flag selects the public static or dynamic process path", "[Print][H2C][Dynamic]")
{
    // Keep the two-object H2C geometry and all selector inputs identical. The project flag is
    // the only varied value: false takes the static grouping path with no cached per-object plans;
    // true enables the cross-object selector path, publishes its stitched result, and caches the
    // exact plans later consumed by sequential G-code export.
    DynamicPrintConfig base_config = DynamicPrintConfig::full_print_config();
    base_config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4};
    base_config.option<ConfigOptionStrings>("extruder_nozzle_stats", true)->values = {"Standard#1", "Standard#1|High Flow#2"};
    base_config.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {etDirectDrive, etDirectDrive};
    base_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {nvtStandard, nvtStandard};
    base_config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {"Direct Drive Standard,Direct Drive High Flow",
                                                                                       "Direct Drive Standard,Direct Drive High Flow"};
    base_config.option<ConfigOptionInts>("print_extruder_id", true)->values = {1, 1, 2, 2};
    base_config.option<ConfigOptionStrings>("print_extruder_variant", true)->values = {"Direct Drive Standard", "Direct Drive High Flow",
                                                                                       "Direct Drive Standard", "Direct Drive High Flow"};
    base_config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75};
    base_config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00"};
    base_config.option<ConfigOptionInts>("filament_map", true)->values = {1, 2};
    base_config.option<ConfigOptionInts>("filament_volume_map", true)->values = {(int)nvtStandard, (int)nvtStandard};
    base_config.option<ConfigOptionEnum<FilamentMapMode>>("filament_map_mode", true)->value = FilamentMapMode::fmmAutoForFlush;
    base_config.option<ConfigOptionEnum<PrintSequence>>("print_sequence", true)->value = PrintSequence::ByObject;
    base_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values = std::vector<double>(8, 140.);
    base_config.option<ConfigOptionFloats>("flush_multiplier", true)->values = {1., 1.};

    Model model;
    ModelObject *object_a = model.add_object("cube_a", "", make_cube(8, 8, 1.2));
    ModelInstance *instance_a = object_a->add_instance();
    instance_a->set_offset(Vec3d(70., 100., 0.));
    ModelObject *object_b = model.add_object("cube_b", "", make_cube(8, 8, 1.2));
    object_b->config.set_key_value("extruder", new ConfigOptionInt(2));
    ModelInstance *instance_b = object_b->add_instance();
    instance_b->set_offset(Vec3d(150., 100., 0.));
    instance_a->arrange_order = 1;
    instance_b->arrange_order = 2;

    for (bool enabled : {false, true}) {
        DynamicPrintConfig config = base_config;
        config.set_key_value("enable_filament_dynamic_map", new ConfigOptionBool(enabled));
        Print print;
        print.apply(model, config);
        CAPTURE(enabled);
        CHECK(print.is_dynamic_group_reorder() == enabled);

        print.process();
        REQUIRE(print.is_step_done(psSlicingFinished));
        auto result = print.get_layered_nozzle_group_result();
        REQUIRE(result != nullptr);
        // Static sequential processing publishes a single-map result with no layer axis;
        // the selector path stitches the object plans onto a nonempty layer axis.
        if (enabled) {
            CHECK(result->get_layer_count() > 0);
            CHECK(print.sequential_dynamic_orderings().size() == 2);
        } else {
            CHECK(result->get_layer_count() == 0);
            CHECK(print.sequential_dynamic_orderings().empty());
            CHECK_FALSE(result->is_support_dynamic_nozzle_map());
        }
        REQUIRE(print.config().filament_map.values == result->get_extruder_map(false));

        // Exercise each mode through the public export path; the dynamic path must reuse the
        // per-object plans that were observed above, while the static path remains exportable.
        ScopedTemporaryFile gcode(".gcode");
        REQUIRE_NOTHROW(print.export_gcode(gcode.string(), nullptr, nullptr));
        REQUIRE(boost::filesystem::exists(gcode.path()));
    }
}
