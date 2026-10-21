#include "libslic3r/ExPolygon.hpp"
#include <functional>
#include <iterator>
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Circle.hpp"
#include "libslic3r/ArcFitter.hpp"
#include <nlohmann/json.hpp>
#include "libslic3r/BoundingBox.hpp"
#include <boost/filesystem/directory.hpp>
#include <ios>
#include "libslic3r/Slicing.hpp"
#include "libslic3r/Utils.hpp"
#include <initializer_list>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

#include <catch2/catch_test_macros.hpp>
#include <string>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include <cstddef>
#include "libslic3r/Surface.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <catch2/catch_all.hpp>

#include "libslic3r/Print.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/Support/TreeModelVolumes.hpp"
#include "libslic3r/Support/TreeSupportCommon.hpp"
#include "libslic3r/Support/BeltFloorContext.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Polyline.hpp"
#include <limits>
#include <cmath>
#include <map>
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Exception.hpp"
#include "libslic3r/LifecycleEvents.hpp"
#include "libslic3r/GCode/ThumbnailData.hpp"
#include <stdexcept>

#include "test_helpers.hpp"
#include "test_utils.hpp"

#include <algorithm>
#include <boost/algorithm/string/predicate.hpp>
#include <cstdlib>
#include <sstream>
#include <limits>
#include <fstream>
#include <iterator>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/SurfaceCollection.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

TEST_CASE("Timelapse g-code is emitted once per layer for Bambu and non-Bambu printers", "[Print][Regression]")
{
    struct PrinterCase {
        std::string name;
        std::string structure;
        bool        is_bbl;
    };
    const PrinterCase printer = GENERATE(from_range(std::vector<PrinterCase>{
        { "non-BBL undefined", "undefine", false },
        { "non-BBL CoreXY",    "corexy",   false },
        { "non-BBL i3",        "i3",       false },
        { "non-BBL H-Bot",     "hbot",     false },
        { "non-BBL Delta",     "delta",    false },
        { "Bambu CoreXY",      "corexy",   true },
        { "Bambu i3",          "i3",       true },
    }));
    INFO("printer: " << printer.name);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "initial_layer_print_height", 0.2 },
        { "layer_change_gcode",          ";TEST_LAYER_CHANGE" },
        { "layer_height",                0.2 },
        { "printer_structure",           printer.structure },
        { "spiral_mode",                 false },
        { "time_lapse_gcode",            "TIMELAPSE_TAKE_FRAME" },
    });
    Print print;
    print.is_BBL_printer() = printer.is_bbl;
    Model model;
    init_print({ cube(20) }, print, model, config);
    const std::string gcode = Slic3r::Test::gcode(print);

    const auto count = [&gcode](std::string_view token) {
        size_t occurrences = 0;
        size_t pos = 0;
        while ((pos = gcode.find(token, pos)) != std::string::npos) {
            ++occurrences;
            pos += token.size();
        }
        return occurrences;
    };

    const size_t layer_changes = count("\n;TEST_LAYER_CHANGE\n");
    REQUIRE(layer_changes > 0);
    CHECK(count("\nTIMELAPSE_TAKE_FRAME\n") == layer_changes);
}

SCENARIO("Changing the number of solid shell layers does not make all surfaces internal", "[Print]") {
    GIVEN("sliced 20mm cube and config with top_shell_layers = 2 and bottom_shell_layers = 1") {
        Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
		config.set_deserialize_strict({
            { "top_shell_layers",           2 },
            { "bottom_shell_layers",        1 },
            { "layer_height",               0.25 }, // get a known number of layers
            { "initial_layer_print_height", 0.25 }
			});
        Slic3r::Print print;
        Slic3r::Model model;
        Slic3r::Test::init_print({cube(20)}, print, model, config);
        // Precondition: Ensure that the model has 2 solid top layers (79, 78)
        // and one solid bottom layer (0).
		auto test_is_solid_infill = [&print](size_t obj_id, size_t layer_id) {
		    const Layer &layer = *(print.objects().at(obj_id)->get_layer((int)layer_id));
		    // iterate over all of the regions in the layer
		    for (const LayerRegion *region : layer.regions()) {
		        // for each region, iterate over the fill surfaces
		        for (const Surface &surface : region->fill_surfaces.surfaces)
		            CHECK(surface.is_solid());
		    }
		};
        print.process();
        test_is_solid_infill(0,  0); // should be solid
        test_is_solid_infill(0, 79); // should be solid
        test_is_solid_infill(0, 78); // should be solid
        WHEN("Model is re-sliced with top_shell_layers == 3") {
			config.set("top_shell_layers", 3);
			print.apply(model, config);
            print.process();
            THEN("Print object does not have 0 solid bottom layers.") {
                test_is_solid_infill(0, 0);
            }
            AND_THEN("Print object has 3 top solid layers") {
                test_is_solid_infill(0, 79);
                test_is_solid_infill(0, 78);
                test_is_solid_infill(0, 77);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Print::validate() warning collection
//
// validate() returns its warnings in a vector. The warning paths deliberately
// differ in how many entries they produce; these tests pin down each behaviour:
//   * independent checks    -> stack (one entry each)
//   * motion-ability        -> coalesce into one (mutually exclusive, gated)
//   * clumping detection    -> one independent warning
//   * layered clearance     -> many collisions concatenated into one entry
//   * null warnings pointer -> no-op, no crash, no blocking error
// ---------------------------------------------------------------------------
namespace {

// Build `n` 20mm cubes (spread apart, or stacked at the origin when `overlap`) into
// `model`/`print` and apply `config`, leaving the print ready to validate(). No slicing needed.
void build_cubes(Slic3r::Model& model, Slic3r::Print& print,
                 DynamicPrintConfig config, int n, bool overlap)
{
    config.set_key_value("layer_change_gcode", new ConfigOptionString("G92 E0\n")); // validate() relative-E reset

    for (int i = 0; i < n; ++i) {
        ModelObject* object = model.add_object();
        object->add_volume(cube(20));
        ModelInstance* inst = object->add_instance();
        inst->set_offset(Vec3d(overlap ? 0.0 : i * 60.0, 0.0, 0.0));
    }
    for (ModelObject* mo : model.objects) {
        mo->ensure_on_bed();
        print.auto_assign_extruders(mo);
    }
    print.apply(model, config);
}

// Build cubes and run validate(), collecting warnings; returns the blocking error.
StringObjectException validate_cubes(const DynamicPrintConfig& config,
                                     std::vector<StringObjectException>& warnings,
                                     int n = 1, bool overlap = false)
{
    Slic3r::Model model;
    Slic3r::Print print;
    build_cubes(model, print, config, n, overlap);
    return print.validate(&warnings);
}

size_t count_opt_key(const std::vector<StringObjectException>& warnings, const std::string& key)
{
    return std::count_if(warnings.begin(), warnings.end(),
        [&](const StringObjectException& w) { return w.opt_key == key; });
}

// Make `default_acceleration` exceed the machine's extruding-acceleration limit.
void trigger_acceleration_warning(DynamicPrintConfig& c)
{
    c.set_key_value("machine_max_acceleration_extruding", new ConfigOptionFloats{ 100. });
    c.set_key_value("default_acceleration", new ConfigOptionFloatsNullable{ 100000. });
}

// Make `default_jerk` exceed the machine's jerk limit (junction deviation off so
// the jerk check is not skipped).
void trigger_jerk_warning(DynamicPrintConfig& c)
{
    c.set_key_value("machine_max_junction_deviation", new ConfigOptionFloats{ 0. });
    c.set_key_value("machine_max_jerk_x", new ConfigOptionFloats{ 1. });
    c.set_key_value("machine_max_jerk_y", new ConfigOptionFloats{ 1. });
    c.set_key_value("default_jerk", new ConfigOptionFloatsNullable{ 9999. });
}

// Precise outer wall is ignored unless the wall sequence is inner-outer.
void trigger_precise_wall_warning(DynamicPrintConfig& c)
{
    c.set_key_value("precise_outer_wall", new ConfigOptionBool(true));
    c.set_key_value("wall_sequence", new ConfigOptionEnum<WallSequence>(WallSequence::OuterInner));
}

} // namespace

// ---------------------------------------------------------------------------
// {first_object_name} filename placeholder
// ---------------------------------------------------------------------------
namespace {

// Add a printable 20mm cube named `name` to `model`; returns it so the caller can tweak it.
ModelObject* add_named_cube(Model& model, const std::string& name)
{
    ModelObject* obj = model.add_object();
    obj->name = name;
    obj->add_volume(make_cube(20.0, 20.0, 20.0));
    obj->add_instance();
    obj->ensure_on_bed();
    return obj;
}

// Resolve `format` to an output file name for a print of `model`. `filename_base`, when set,
// is the saved-project name passed to output_filename().
std::string resolved_output_name(Model& model, const std::string& format, const std::string& filename_base = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filename_format", new ConfigOptionString(format));

    Print print;
    for (ModelObject* obj : model.objects)
        print.auto_assign_extruders(obj);
    print.apply(model, config);
    return print.output_filename(filename_base);
}

struct ScopedLifecycleHook
{
    explicit ScopedLifecycleHook(LifecycleHookFn hook) { set_lifecycle_hook_fn(std::move(hook)); }
    ~ScopedLifecycleHook() { set_lifecycle_hook_fn(nullptr); }
};

} // namespace

TEST_CASE("Slicing lifecycle events identify the model", "[Print][LifecycleEvents]")
{
    struct ObservedEvent {
        LifecycleEvent event;
        std::string id;
        std::string name;
    };
    std::vector<ObservedEvent> events;
    ScopedLifecycleHook hook([&](LifecycleEvent event, const LifecycleEventContext& ctx) {
        events.push_back({ event, ctx.id, ctx.name });
    });

    Print print;
    Model model;
    ModelInfo info;
    info.model_name = "Lifecycle test model";
    model.model_info = std::make_shared<ModelInfo>(std::move(info));
    init_print({cube(20)}, print, model);

    print.process();
    ScopedTemporaryFile temp(".gcode");
    print.export_gcode(temp.string(), nullptr, nullptr);
    GCodeProcessorResult result;
    print.export_gcode_from_previous_file(temp.string(), &result);

    const std::string expected_id = std::to_string(print.model().id().id);
    const std::vector<LifecycleEvent> expected_events = {
        LifecycleEvent::SliceStarted,
        LifecycleEvent::SliceGeometryFinished,
        LifecycleEvent::GCodeExportStarted,
        LifecycleEvent::GCodeExportFinished,
        LifecycleEvent::GCodeExportStarted,
        LifecycleEvent::GCodeExportFinished,
    };
    REQUIRE(events.size() == expected_events.size());
    for (size_t i = 0; i < expected_events.size(); ++i) {
        CHECK(events[i].event == expected_events[i]);
        CHECK(events[i].id == expected_id);
        CHECK(events[i].name == "Lifecycle test model");
    }
}

TEST_CASE("Slicing lifecycle event name is empty without model metadata", "[Print][LifecycleEvents]")
{
    std::string event_id;
    std::string event_name = "unset";
    ScopedLifecycleHook hook([&](LifecycleEvent event, const LifecycleEventContext& ctx) {
        if (event == LifecycleEvent::SliceStarted) {
            event_id = ctx.id;
            event_name = ctx.name;
        }
    });

    Print print;
    Model model;
    init_print({cube(20)}, print, model);
    print.process();

    CHECK(event_id == std::to_string(print.model().id().id));
    CHECK(event_name.empty());
}

TEST_CASE("Output filenames with numeric statistics fail before slicing finishes", "[Print][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filename_format", new ConfigOptionString("{int(total_weight*10) / 10.0}"));

    Print print;
    Model model;
    init_print({cube(20)}, print, model, config);

    CHECK_THROWS_AS(print.output_filename(), PlaceholderParserError);
}

TEST_CASE("Print: {first_object_name} names the first printable object on the plate", "[Print]")
{
    Model model;

    SECTION("uses the object's name") {
        add_named_cube(model, "WidgetPart");
        CHECK(resolved_output_name(model, "{first_object_name}") == "WidgetPart.gcode");
    }

    SECTION("picks the first when several objects are printable") {
        add_named_cube(model, "FirstPart");
        add_named_cube(model, "SecondPart");
        CHECK(resolved_output_name(model, "{first_object_name}") == "FirstPart.gcode");
    }

    SECTION("skips objects outside the print volume (e.g. on another plate)") {
        // First in model order, but not on the current plate, so is_printable() is false.
        add_named_cube(model, "OtherPlatePart")->instances.front()->print_volume_state = ModelInstancePVS_Fully_Outside;
        add_named_cube(model, "OnPlatePart");
        CHECK(resolved_output_name(model, "{first_object_name}") == "OnPlatePart.gcode");
    }

    SECTION("is empty when the object has no name") {
        add_named_cube(model, "");
        CHECK(resolved_output_name(model, "part_{first_object_name}") == "part_.gcode");
    }
}

TEST_CASE("Print: {first_object_name} is not replaced by the saved-project file name", "[Print]")
{
    // Passing a saved-project file name as the filename_base must not change {first_object_name}.
    Model model;
    add_named_cube(model, "WidgetPart");
    CHECK(resolved_output_name(model, "{first_object_name}", "SavedProject") == "WidgetPart.gcode");
}

TEST_CASE("Print statistics defer placeholders and finalize output filenames", "[Print][PrintStatisticsFilename]")
{
    PrintStatistics stats;
    stats.estimated_normal_print_time = "1h 2m 30s";
    stats.estimated_silent_print_time = "2h 59m 30s";
    stats.total_used_filament        = 1250.;
    stats.total_extruded_volume      = 275.;
    stats.total_cost                 = 4.5;
    stats.total_weight               = 6.25;
    stats.total_wipe_tower_cost      = 0.5;
    stats.total_wipe_tower_filament  = 12.5;
    stats.total_toolchanges          = 7;
    stats.initial_tool               = 3;
    stats.initial_no_support_tool    = 5;

    const DynamicConfig placeholders = PrintStatistics::placeholders();
    const auto* time_placeholder = dynamic_cast<const ConfigOptionString*>(placeholders.option("print_time"));
    const auto* filament_placeholder = dynamic_cast<const ConfigOptionString*>(placeholders.option("used_filament"));
    const auto* tool_placeholder = dynamic_cast<const ConfigOptionString*>(placeholders.option("initial_tool"));
    REQUIRE(time_placeholder != nullptr);
    REQUIRE(filament_placeholder != nullptr);
    REQUIRE(tool_placeholder != nullptr);
    CHECK(time_placeholder->value == "{print_time}");
    CHECK(filament_placeholder->value == "{used_filament}");
    CHECK(tool_placeholder->value == "{initial_tool}");

    const boost::filesystem::path input = boost::filesystem::path("reports") /
        "stats_{print_time}_{normal_print_time}_{silent_print_time}_{used_filament}_{extruded_volume}_{extruded_volume_total}_{total_cost}_{total_weight}_{extruded_weight_total}_{total_wipe_tower_cost}_{total_wipe_tower_filament}_{initial_tool}_{initial_extruder}_{initial_no_support_extruder}_{total_toolchanges}.gcode";
    const boost::filesystem::path expected = boost::filesystem::path("reports") /
        "stats_1h3m_1h3m_3h0m_1.25_275_275_4.5_6.25_6.25_0.5_12.5_3_3_5_7.gcode";

    CHECK(stats.finalize_output_path(input.string()) == expected.string());
}

TEST_CASE("Print statistics preserve filenames when placeholders cannot be resolved", "[Print][PrintStatisticsFilename]")
{
    PrintStatistics stats;
    const std::string filename = GENERATE(
        std::string("plain_name.gcode"),
        std::string("{missing_statistic}.gcode"),
        std::string("{total_weight.gcode"));
    const std::string input = (boost::filesystem::path("reports") / filename).string();
    CAPTURE(input);

    CHECK(stats.finalize_output_path(input) == input);
}

TEST_CASE("Print::validate stacks independent warnings", "[Print][validate]")
{
    // Two unrelated checks (region precise-wall + machine acceleration) must each
    // contribute their own entry.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    trigger_precise_wall_warning(config);
    trigger_acceleration_warning(config);

    std::vector<StringObjectException> warnings;
    StringObjectException err = validate_cubes(config, warnings);

    CHECK(err.string.empty());
    CHECK(warnings.size() >= 2);
    CHECK(count_opt_key(warnings, "precise_outer_wall") == 1);  // jump-to key is preserved
    for (const auto& w : warnings)
        CHECK(w.is_warning);                                   // every collected entry is a warning
}

TEST_CASE("Print::validate coalesces motion-ability warnings into one", "[Print][validate]")
{
    // The jerk/junction/acceleration checks are mutually exclusive (gated on a shared
    // key), so adding a second motion trigger must NOT add a second warning.
    DynamicPrintConfig accel_only = DynamicPrintConfig::full_print_config();
    trigger_acceleration_warning(accel_only);
    std::vector<StringObjectException> w_accel;
    CHECK(validate_cubes(accel_only, w_accel).string.empty());

    DynamicPrintConfig accel_and_jerk = DynamicPrintConfig::full_print_config();
    trigger_acceleration_warning(accel_and_jerk);
    trigger_jerk_warning(accel_and_jerk);
    std::vector<StringObjectException> w_both;
    CHECK(validate_cubes(accel_and_jerk, w_both).string.empty());

    CHECK(w_accel.size() >= 1);
    CHECK(w_both.size() == w_accel.size());  // the extra motion trigger collapses into the same warning
}

TEST_CASE("Print::validate reports the clumping-detection warning", "[Print][validate]")
{
    // A distinct single-shot path: clumping/wrapping detection without a prime tower warns
    // (and carries the enable_prime_tower jump-to key). enable_prime_tower must be off, as
    // the warning lives in the no-prime-tower branch.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("enable_prime_tower", new ConfigOptionBool(false));
    config.set_key_value("enable_wrapping_detection", new ConfigOptionBool(true));

    std::vector<StringObjectException> warnings;
    StringObjectException err = validate_cubes(config, warnings);

    CHECK(err.string.empty());
    CHECK(count_opt_key(warnings, "enable_prime_tower") == 1);
}

TEST_CASE("Print::validate concatenates layered-clearance collisions into one warning", "[Print][validate]")
{
    // In by-layer mode, layered_print_cleareance_valid folds every too-close pair into a
    // single warning entry (newline-joined), unlike the per-check stacking above. Isolate
    // that entry by type so unrelated default-config warnings don't affect the assertion.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();

    std::vector<StringObjectException> warnings;
    StringObjectException err = validate_cubes(config, warnings, /*n=*/3, /*overlap=*/true);

    CHECK(err.string.empty());
    auto is_layered = [](const StringObjectException& w) {
        return w.type == STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT; };
    REQUIRE(std::count_if(warnings.begin(), warnings.end(), is_layered) == 1);  // 3 objects, 2 collisions, 1 entry
    auto it = std::find_if(warnings.begin(), warnings.end(), is_layered);
    CHECK(it->string.find('\n') != std::string::npos);  // the collisions were concatenated
}

TEST_CASE("Print::validate tolerates a null warnings pointer", "[Print][validate]")
{
    // Callers may pass no warnings sink: a warning-producing config must not crash
    // and must still return without a blocking error.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    trigger_precise_wall_warning(config);
    trigger_acceleration_warning(config);

    Slic3r::Model model;
    Slic3r::Print print;
    build_cubes(model, print, config, /*n=*/1, /*overlap=*/false);

    StringObjectException err = print.validate();  // warnings == nullptr
    CHECK(err.string.empty());
}

TEST_CASE("Purge tower selection keeps ordinary printers on the classic path", "[Print][PurgeTower][Regression]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "belt_printer",            0 },
        { "enable_prime_tower",      1 },
        { "enable_belt_purge_tower", 1 }
    });
    config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(TimelapseType::tlSmooth));

    Model model;
    Print print;
    build_cubes(model, print, config, /*n=*/1, /*overlap=*/false);

    CHECK(print.has_wipe_tower());
    CHECK_FALSE(print.has_belt_purge_tower());
}

TEST_CASE("Belt purge planning requires its managed purge object", "[Print][PurgeTower][Regression]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "belt_printer",             1 },
        { "enable_belt_purge_tower", 1 }
    });

    Model model;
    Print print;
    build_cubes(model, print, config, /*n=*/1, /*overlap=*/false);
    CHECK_FALSE(print.has_belt_purge_tower());

    model.objects.front()->config.set_key_value("belt_purge_tower_object", new ConfigOptionBool(true));
    print.apply(model, config);
    CHECK(print.has_belt_purge_tower());
    CHECK_FALSE(print.has_wipe_tower());
}

// The GUI creates the purge tower object; a project sliced without one (the CLI) must say
// that its filament changes go unpurged.
TEST_CASE("Belt purge tower enabled without a tower object warns", "[Print][PurgeTower][belt]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "belt_printer",            1 },
        { "enable_belt_purge_tower", 1 },
        { "layer_change_gcode",      "G92 E0\n" }
    });
    auto purge_warnings = [](Print &print) {
        std::vector<StringObjectException> warnings;
        print.validate(&warnings);
        return std::count_if(warnings.begin(), warnings.end(), [](const StringObjectException &w) {
            return w.opt_key == "enable_belt_purge_tower";
        });
    };

    Model model;
    Print print;
    build_cubes(model, print, config, /*n=*/2, /*overlap=*/false);
    model.objects[1]->config.set_key_value("extruder", new ConfigOptionInt(2));
    print.apply(model, config);
    REQUIRE(print.extruders().size() > 1);
    CHECK(purge_warnings(print) == 1);

    model.objects.front()->config.set_key_value("belt_purge_tower_object", new ConfigOptionBool(true));
    print.apply(model, config);
    CHECK(purge_warnings(print) == 0);
}

TEST_CASE("Belt purge rejects multiple managed purge objects", "[Print][PurgeTower][Regression]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "belt_printer",             1 },
        { "enable_belt_purge_tower", 1 }
    });

    Model model;
    Print print;
    build_cubes(model, print, config, /*n=*/2, /*overlap=*/false);
    for (ModelObject *object : model.objects)
        object->config.set_key_value("belt_purge_tower_object", new ConfigOptionBool(true));
    print.apply(model, config);

    CHECK_FALSE(print.validate().string.empty());
}

TEST_CASE("A default slice emits perimeter, infill, and skirt", "[Print]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "z_hop",                      0 } // keep recorded Z at the printed height
    });
    CHECK(role_passes(gcode, "perimeter") > 0);
    CHECK(role_passes(gcode, "infill")    > 0);
    CHECK(role_passes(gcode, "skirt")     > 0);
    CHECK_THAT(max_z(gcode), Catch::Matchers::WithinAbs(20.0, 1e-4));
}

// The G-code carries a config-comment block describing the resolved settings. The
// per-region width lines are always present; the support and first-layer lines appear
// only when those features are configured.
TEST_CASE("G-code lists the resolved extrusion-width settings", "[Print][ConfigBlock]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "initial_layer_line_width", 0 },
        { "gcode_skip_config_block", false },
    });
    CHECK(gcode.find("; external perimeters extrusion width") != std::string::npos);
    CHECK(gcode.find("; perimeters extrusion width")          != std::string::npos);
    CHECK(gcode.find("; infill extrusion width")              != std::string::npos);
    CHECK(gcode.find("; solid infill extrusion width")        != std::string::npos);
    CHECK(gcode.find("; top infill extrusion width")          != std::string::npos);
    CHECK(gcode.find("; support material extrusion width")    == std::string::npos);
    CHECK(gcode.find("; first layer extrusion width")         == std::string::npos);
    CHECK(gcode.find("; layer_height")                        != std::string::npos);
    CHECK(gcode.find("; sparse_infill_density")               != std::string::npos);

    const std::string with_support = slice({ cube(20) }, {
        { "initial_layer_line_width", 0 }, { "enable_support", true }, { "raft_layers", 3 },
    });
    CHECK(with_support.find("; support material extrusion width") != std::string::npos);

    const std::string with_first_layer = slice({ cube(20) }, { { "initial_layer_line_width", "0.5" } });
    CHECK(with_first_layer.find("; first layer extrusion width") != std::string::npos);
}

// gcode_skip_config_block suppresses the resolved-settings block while leaving the
// header and executable blocks intact.
TEST_CASE("gcode_skip_config_block omits the resolved-settings comment block", "[Print][ConfigBlock]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "gcode_skip_config_block", true },
        { "gcode_comments",         true },
    });
    CHECK(gcode.find("; CONFIG_BLOCK_START")     == std::string::npos);
    CHECK(gcode.find("; CONFIG_BLOCK_END")       == std::string::npos);
    CHECK(gcode.find("; layer_height =")         == std::string::npos);
    CHECK(gcode.find("; fill_density =")         == std::string::npos);
    CHECK(gcode.find("; HEADER_BLOCK_START")     != std::string::npos);
    CHECK(gcode.find("; EXECUTABLE_BLOCK_START") != std::string::npos);
}

// Some firmwares only scan the last N lines of the file for "estimated printing time", so it
// must stay close to EOF regardless of the resolved-settings config block's size.
TEST_CASE("The estimated printing time comment stays near the end of the file", "[Print]")
{
    const std::string gcode = slice({ cube(20) }, {});
    const size_t config_block_end = gcode.find("; CONFIG_BLOCK_END");
    const size_t filament_stats   = gcode.find("; filament used [mm]");
    const size_t time_comment     = gcode.find("estimated printing time");
    REQUIRE(config_block_end != std::string::npos);
    REQUIRE(filament_stats != std::string::npos);
    REQUIRE(time_comment != std::string::npos);
    CHECK(filament_stats > config_block_end);
    CHECK(time_comment > filament_stats);

    const size_t line_start = gcode.rfind('\n', time_comment) + 1;
    const size_t trailing_lines = std::count(gcode.begin() + line_start, gcode.end(), '\n');
    CHECK(trailing_lines <= 5);
}

// Custom G-code templates substitute placeholders during export.
TEST_CASE("Custom G-code placeholders are substituted", "[Print]")
{
    // [current_extruder] in the start G-code.
    CHECK(slice({ cube(20) }, { { "machine_start_gcode", "; Extruder [current_extruder]" } })
              .find("; Extruder 0") != std::string::npos);

    // [layer_num] / [layer_z] in the end G-code (a 20mm cube at 0.1mm is 200 layers).
    const std::string end_gcode = slice({ cube(20) }, {
        { "machine_end_gcode",          "; Layer_num [layer_num]\n; Layer_z [layer_z]" },
        { "layer_height",               0.1 },
        { "initial_layer_print_height", 0.1 },
    });
    CHECK(end_gcode.find("; Layer_num 199") != std::string::npos);
    CHECK(end_gcode.find("; Layer_z 20")    != std::string::npos);

    // printing_by_object_gcode is emitted between sequentially printed objects.
    CHECK(slice_two_cubes_arranged({
                    { "print_sequence",           "by object" },
                    { "printing_by_object_gcode", "; between-object-gcode" },
                })
              .find("; between-object-gcode") != std::string::npos);

    // [layer_num] keeps counting across sequentially printed objects (199 then 399).
    const std::string per_layer = slice_two_cubes_arranged({
        { "print_sequence",             "by object" },
        { "layer_change_gcode",         ";Layer:[layer_num] ([layer_z] mm)" },
        { "layer_height",               0.1 },
        { "initial_layer_print_height", 0.1 },
    });
    CHECK(per_layer.find(";Layer:199 ") != std::string::npos);
    CHECK(per_layer.find(";Layer:399 ") != std::string::npos);
}

TEST_CASE("export_gcode writes G-code without a result pointer", "[Print][export_gcode]")
{
    Print print;
    Model model;
    Slic3r::Test::init_print({cube(20)}, print, model);
    print.process();

    SECTION("non-BBL printer") {}
    SECTION("BBL printer") { print.is_BBL_printer() = true; }

    ScopedTemporaryFile temp(".gcode");
    REQUIRE_NOTHROW(print.export_gcode(temp.string(), nullptr, nullptr));

    std::ifstream in(temp.string());
    const std::string gcode((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

    REQUIRE_FALSE(gcode.empty());
}

TEST_CASE("Exporting a sliced print again gives the same G-code", "[Print][export_gcode][Regression]")
{
    const int instances = GENERATE(1, 3);
    CAPTURE(instances);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    TestMesh           mesh   = TestMesh::ipadstand;
    SECTION("infill reversed by chaining") { config.set_deserialize_strict({{"sparse_infill_pattern", "gyroid"}}); }
    SECTION("support reversed by chaining") {
        mesh = TestMesh::overhang;
        config.set_deserialize_strict({{"enable_support", true}, {"support_interface_pattern", "concentric"}});
    }
    Print print;
    Model model;
    Slic3r::Test::init_print({Slic3r::Test::mesh(mesh)}, print, model, config, nullptr, true, instances);

    const auto export_without_timestamp = [&print]() {
        std::string gcode = Slic3r::Test::gcode(print);
        const size_t line = gcode.find("; generated by ");
        REQUIRE(line != std::string::npos);
        gcode.erase(line, gcode.find('\n', line) - line);
        return gcode;
    };
    const std::string first  = export_without_timestamp();
    const std::string second = export_without_timestamp();

    // Shows the first differing line on failure.
    const size_t diff       = std::mismatch(first.begin(), first.end(), second.begin(), second.end()).first - first.begin();
    const size_t line_start = diff == 0 ? 0 : first.rfind('\n', diff - 1) + 1;
    INFO("first export:  " << first.substr(line_start, first.find('\n', diff) - line_start));
    INFO("second export: " << second.substr(line_start, second.find('\n', diff) - line_start));
    CHECK(diff == first.size());
    CHECK(first.size() == second.size());
}

TEST_CASE("Sequential printing follows model order", "[Print]")
{
    // Two objects of different heights, taller one added first. Orca prints
    // sequential objects in model order, so the taller one is printed first.
    const std::string gcode = Slic3r::Test::slice({ cube(20), Slic3r::make_cube(20, 20, 10) }, {
        { "print_sequence",             "by object" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "z_hop",                      0 }
    });

    // The first object's height is the peak Z reached before Z drops back to the
    // first layer (the object change). With by-object printing only an object
    // change returns Z to the bottom.
    double first_object_peak_z = 0.0;
    double running_peak        = 0.0;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
        if (first_object_peak_z != 0.0 || !line.extruding(self)) return; // ignore travels (e.g. start-gcode Z lift)
        if (running_peak > 1.0 && self.z() < 1.0)
            first_object_peak_z = running_peak;
        else
            running_peak = std::max(running_peak, static_cast<double>(self.z()));
    });

    REQUIRE_THAT(first_object_peak_z, Catch::Matchers::WithinAbs(20.0, 0.3));
}

// A sequential (by-object) print must publish the print-level nozzle group result just
// like a by-layer print, so custom g-code can index the per-nozzle placeholder tables
// (e.g. nozzle_diameter_at_nozzle_id[]) instead of failing on an empty vector.
TEST_CASE("Sequential printing publishes the nozzle group result", "[Print][MultiNozzle]")
{
    SECTION("process() publishes the result") {
        Print print;
        Model model;
        place_two_cubes_apart(60.0, { { "print_sequence", "by object" } }, print, model);
        print.process();
        REQUIRE(print.get_layered_nozzle_group_result() != nullptr);
    }

    SECTION("start g-code can index the per-nozzle diameter table") {
        const std::string gcode = slice_two_cubes_arranged({
            { "print_sequence",      "by object" },
            { "machine_start_gcode", "{if nozzle_diameter_at_nozzle_id[0] > 0}; SEQ-ND-OK\n{endif}" },
        });
        CHECK(gcode.find("; SEQ-ND-OK") != std::string::npos);
    }
}

// A scarf joint starts one layer height below the layer and ramps up along the
// wall. On a tilted belt that start is a step backwards along the belt axis, into
// the previous layer's wall at the seam: 0.283 mm per 0.2 mm layer at 45 degrees.
// With an aligned seam the nozzle rams the same spot on every layer (field report
// from a BabyBelt Pro: the belt "jumped backwards" and knocked the part loose).
// Belt printers therefore never get a scarf, whatever the process preset says.
TEST_CASE("Belt printers never start a scarf seam below the layer", "[Print][belt][Seam]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        1 },
        { "wall_loops",                 2 },
        { "seam_position",              "back" },
        { "seam_slope_type",            "external" },
        { "seam_slope_inner_walls",     1 },
        { "seam_slope_start_height",    0 },
        // No z-hop: on a belt a lift is a move along the belt axis (0.4 mm / sin 45 = 0.57 mm)
        // and its return would read as a back-step. The shipped belt profiles print without one.
        { "z_hop",                      0 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    const std::string gcode = slice({ cube(20) }, config);
    REQUIRE(! gcode.empty());

    // The belt axis is machine Z. Within a layer it only drifts by the frame
    // coupling (well under 0.1 mm across a 20 mm cube); a scarf start is a full
    // layer pitch (0.283 mm) backwards.
    double last_z = std::numeric_limits<double>::lowest();
    double worst_backstep = 0.;
    GCodeReader parser;
    parser.parse_buffer(gcode, [&](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (! line.cmd_is("G1") || ! line.has_z())
            return;
        const double z = line.z();
        if (last_z != std::numeric_limits<double>::lowest())
            worst_backstep = std::max(worst_backstep, last_z - z);
        last_z = z;
    });
    CHECK(worst_backstep < 0.2);
}

// printable_height on a belt printer is the clearance under the gantry, so an object taller
// than that is refused whatever the machine-frame transform does to the emitted coordinates.
TEST_CASE("Belt printers refuse an object taller than the gantry clearance", "[Print][belt]")
{
    auto belt_config = [](double printable_height) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "belt_printer",               1 },
            { "belt_slice_rotation",        "x" },
            { "belt_slice_rotation_angle",  45 },
            { "gcode_remap_x",              "rev_x" },
            { "gcode_remap_y",              "pos_z" },
            { "gcode_remap_z",              "pos_y" },
            { "printable_height",           printable_height },
            { "skirt_loops",                0 },
            { "layer_change_gcode",         "G92 E0\n" },
        });
        return config;
    };

    SECTION("a 20 mm cube fits under 50 mm of clearance") {
        Print print;
        Model model;
        init_print({ cube(20) }, print, model, belt_config(50));
        CHECK(print.validate().string.empty());
    }
    SECTION("a 60 mm cube does not") {
        Print print;
        Model model;
        init_print({ cube(60) }, print, model, belt_config(50));
        CHECK(print.validate().string.find("height") != std::string::npos);
    }
}

// On a belt every tilted layer starts on the belt, so "the first layers" the fan stays off
// for are a band along the belt, not the first slicing layers. The generator marks where
// each extrusion segment enters and leaves that band and the cooling buffer keeps the fan
// off inside it, on every layer.
TEST_CASE("Belt printers keep the part fan off within the band above the belt", "[Print][belt][Cooling]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",                 1 },
        { "belt_slice_rotation",          "x" },
        { "belt_slice_rotation_angle",    45 },
        { "gcode_remap_x",                "rev_x" },
        { "gcode_remap_y",                "pos_z" },
        { "gcode_remap_z",                "pos_y" },
        { "layer_height",                 0.2 },
        { "initial_layer_print_height",   0.2 },
        { "skirt_loops",                  0 },
        { "z_hop",                        0 },
        // Three layers, 0.6 mm: the lowest wall of each tilted layer is centred about 0.3 mm
        // above the belt (half a line width in from the contact edge).
        { "close_fan_the_first_x_layers", 3 },
        { "full_fan_speed_layer",         0 },
        { "fan_min_speed",                100 },
        { "fan_max_speed",                100 },
        { "slow_down_layer_time",         1000 },
        { "fan_cooling_layer_time",       1001 },
        { "reduce_fan_stop_start_freq",   0 },
        { "machine_start_gcode",          "T[initial_tool]\n" },
        { "layer_change_gcode",           "G92 E0\n" },
    });
    const std::string gcode = slice({ cube(20) }, config);
    REQUIRE(! gcode.empty());

    // The markers are consumed by the cooling buffer and never reach the file.
    CHECK(gcode.find(";_BELT_BAND") == std::string::npos);

    // With this axis mapping machine Y is the height above the belt along the gantry. Walk
    // the moves with the fan state: extrusions that stay within 0.45 mm of the belt are well
    // inside the band and must print with the fan off; extrusions that stay 5 mm clear of it
    // must print with it on. The first three slicing layers have the fan off altogether.
    size_t in_band = 0, in_band_fan_on = 0, clear = 0, clear_fan_off = 0;
    int    layer   = -1;
    bool   fan_on  = false;
    double y       = 0.;
    std::istringstream lines(gcode);
    for (std::string line; std::getline(lines, line); ) {
        if (boost::starts_with(line, ";LAYER_CHANGE")) {
            ++ layer;
        } else if (boost::starts_with(line, "M107")) {
            fan_on = false;
        } else if (boost::starts_with(line, "M106")) {
            const size_t s = line.find('S');
            fan_on = s != std::string::npos && std::atof(line.c_str() + s + 1) > 0.;
        } else if (boost::starts_with(line, "G1 ")) {
            const size_t comment = line.find(';');
            const std::string cmd = line.substr(0, comment);
            const size_t ypos = cmd.find(" Y"), epos = cmd.find(" E");
            if (ypos == std::string::npos)
                continue;
            const double y_new     = std::atof(cmd.c_str() + ypos + 2);
            const bool   extruding = epos != std::string::npos && std::atof(cmd.c_str() + epos + 2) > 0.;
            if (extruding && layer >= 3) {
                if (std::max(y, y_new) < 0.45) {
                    ++ in_band;
                    in_band_fan_on += fan_on;
                } else if (std::min(y, y_new) > 5.) {
                    ++ clear;
                    clear_fan_off += ! fan_on;
                }
            }
            y = y_new;
        }
    }
    CHECK(in_band > 20);
    CHECK(in_band_fan_on == 0);
    CHECK(clear > 20);
    CHECK(clear_fan_off == 0);
}

// Organic supports under an overhang on a belt printer reach below the object's first layer,
// where the virtual belt raft layers sit at negative Z. The lowest of them used to get a
// negative height and abort slicing with a negative flow error.
TEST_CASE("Belt printers slice organic tree supports that reach the belt", "[Print][belt][Support]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "z_hop",                      0 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "organic" },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    std::string gcode;
    REQUIRE_NOTHROW(gcode = slice({ TestMesh::overhang }, config));
    CHECK(! gcode.empty());
}

TEST_CASE("Slicing errors are reported per object with the object's name", "[Print]")
{
    Print print;
    Model model;
    init_print({Slic3r::Test::cube(20.)}, print, model);
    // Lift the cube off the bed: its first layer is empty, which G-code export reports per object.
    ModelObject *object = model.objects.front();
    object->name = "floating cube";
    object->instances.front()->set_offset(object->instances.front()->get_offset() + Vec3d(0., 0., 2.));
    print.apply(model, DynamicPrintConfig::full_print_config());
    print.set_status_silent();

    ScopedTemporaryFile temp(".gcode");
    std::string message;
    try {
        print.process();
        print.export_gcode(temp.string(), nullptr, nullptr);
        FAIL("slicing did not report the empty first layer");
    } catch (const SlicingErrors &errors) {
        REQUIRE(errors.errors_.size() == 1);
        message = print.slicing_errors_message(errors);
    }
    CHECK(message.rfind("floating cube: ", 0) == 0);
    CHECK(message.find("empty first layer") != std::string::npos);
}

// ---------------------------------------------------------------------------
// Belt mode must be invisible when it is off, and must not leave traces behind.
// ---------------------------------------------------------------------------

// Everything the slicer decided, without the lines that legitimately differ between
// two exports of the same print: comments (the config block lists every key, the
// header carries the export time) and the thumbnail blocks.
static std::string gcode_body(const std::string &gcode)
{
    std::string      body;
    std::istringstream in(gcode);
    for (std::string line; std::getline(in, line); ) {
        line.erase(std::min(line.size(), line.find(';')));
        while (! line.empty() && line.back() == ' ')
            line.pop_back();
        if (! line.empty())
            body += line + '\n';
    }
    return body;
}

static DynamicPrintConfig belt_test_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "z_hop",                      0 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    return config;
}

TEST_CASE("Belt-only keys at non-default values leave non-belt G-code unchanged", "[Print][belt][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "z_hop",                      0 },
        { "brim_type",                  "outer_only" },
        { "brim_width",                 4 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "organic" },
        { "sparse_infill_pattern",      "adaptivecubic" },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    const std::string reference = gcode_body(slice({ TestMesh::overhang }, config));
    REQUIRE(! reference.empty());

    // Every belt key a profile can carry, at a value that would change a belt print.
    // belt_printer stays off, so none of them may reach the G-code: the axis remaps are
    // gated on belt mode, the rest is only read on belt printers. build_plate_tilt_x/y
    // is a feature of its own on a flat bed and is left alone here; "leading_edge_only"
    // prints as an outer brim by design.
    config.set_deserialize_strict({
        { "belt_printer",                0 },
        { "belt_printer_infinite_y",     0 },
        { "belt_slice_rotation",         "y" },
        { "belt_slice_rotation_angle",   30 },
        { "gcode_remap_x",               "rev_x" },
        { "gcode_remap_y",               "pos_z" },
        { "gcode_remap_z",               "pos_y" },
        { "belt_frame_tilt_decouple",    1 },
        { "belt_frame_tilt_angle",       30 },
        { "belt_support_floor_offset",   -5 },
        { "enable_belt_purge_tower",     1 },
        { "belt_purge_tower_width",      10 },
        { "leading_brim_length",         10 },
        { "extra_brim_width",            5 },
    });
    CHECK(gcode_body(slice({ TestMesh::overhang }, config)) == reference);
}

TEST_CASE("Switching a sliced project from belt to non-belt matches a fresh slice", "[Print][belt][Regression]")
{
    // The organic support layers and the adaptive infill octree are placed with the
    // belt global Z offset, and the mesh with the belt min-Z lift. Both are only
    // written while belt mode slices, so they used to survive a switch away from it.
    DynamicPrintConfig flat = DynamicPrintConfig::full_print_config();
    flat.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "z_hop",                      0 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "organic" },
        { "sparse_infill_pattern",      "adaptivecubic" },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    DynamicPrintConfig belt = belt_test_config();
    belt.set_deserialize_strict({
        { "enable_support",        1 },
        { "support_type",          "tree(auto)" },
        { "support_style",         "organic" },
        { "sparse_infill_pattern", "adaptivecubic" },
    });

    // Both prints are placed with the belt config, so only the slicing history differs.
    auto fresh_slice = [&](const DynamicPrintConfig &target) {
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, belt);
        print.apply(model, target);
        const std::string out = gcode(print);
        return gcode_body(out);
    };
    auto resliced = [&](const DynamicPrintConfig &target) {
        Print print;
        Model model;
        init_print({ TestMesh::overhang }, print, model, belt);
        REQUIRE(! gcode(print).empty());
        print.apply(model, target);
        const std::string out = gcode(print);
        return gcode_body(out);
    };
    SECTION("belt printer to a flat-bed printer") {
        CHECK(resliced(flat) == fresh_slice(flat));
    }
    SECTION("belt tilt axis set to None") {
        DynamicPrintConfig untilted = belt;
        untilted.set_deserialize_strict({ { "belt_slice_rotation", "none" } });
        CHECK(resliced(untilted) == fresh_slice(untilted));
    }
}

TEST_CASE("A support-only change on a belt purge print matches a fresh slice", "[Print][belt][PurgeTower][Regression]")
{
    // Snapping the purge prism onto the parts' layer grid shifts every object's layers by
    // up to half a layer. A support-only change reruns support generation without
    // reslicing, so the cached belt floor and the global Z offset have to carry the
    // snap too, or the supports land on the pre-snap grid.
    auto make_config = [](bool support) {
        DynamicPrintConfig config = multifilament_config(2, {
            { "belt_printer",               1 },
            { "belt_slice_rotation",        "x" },
            { "belt_slice_rotation_angle",  45 },
            { "gcode_remap_x",              "rev_x" },
            { "gcode_remap_y",              "pos_z" },
            { "gcode_remap_z",              "pos_y" },
            { "layer_height",               0.2 },
            { "initial_layer_print_height", 0.2 },
            { "skirt_loops",                0 },
            { "z_hop",                      0 },
            { "enable_belt_purge_tower",    1 },
            { "machine_start_gcode",        "T[initial_tool]\n" },
            { "layer_change_gcode",         "G92 E0\n" },
        });
        config.set_deserialize_strict({
            { "enable_support", support ? 1 : 0 },
            { "support_type",   "tree(auto)" },
            { "support_style",  "organic" },
        });
        return config;
    };
    const std::vector<std::vector<Slic3r::ConfigBase::SetDeserializeItem>> overrides {
        { { "extruder", 1 } }, { { "extruder", 2 } },
    };
    auto build = [&](Print &print, Model &model, const DynamicPrintConfig &config) {
        init_print(std::vector<TriangleMesh>{ mesh(TestMesh::overhang), cube(20) }, print, model, config, &overrides);
        model.objects.back()->config.set_key_value("belt_purge_tower_object", new ConfigOptionBool(true));
        print.apply(model, config);
        REQUIRE(print.has_belt_purge_tower());
    };

    std::string fresh;
    {
        Print print;
        Model model;
        build(print, model, make_config(true));
        fresh = gcode_body(gcode(print));
    }
    REQUIRE(! fresh.empty());

    Print print;
    Model model;
    build(print, model, make_config(false));
    REQUIRE(! gcode(print).empty());
    // Support only: posSlice stays valid, posSupportMaterial reruns.
    print.apply(model, make_config(true));
    CHECK(gcode_body(gcode(print)) == fresh);
}

TEST_CASE("Organic tree supports place a support blocker at its own height above a raft", "[Print][Support][Regression]")
{
    // TreeModelVolumes consumes the support blockers in the same index space as the
    // layer outlines, where object layer i sits at num_raft_layers + i, but
    // slice_support_blockers() returns them in object-layer space. Without the shift
    // every blocker lands num_raft_layers too low, so branches are kept out of the
    // wrong layers and may pass through the blocked ones.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "organic" },
        { "raft_layers",                3 },
    });
    Print print;
    Model model;
    init_print({ cube(20) }, print, model, config);
    // A blocker floating beside the cube, 8 mm to 12 mm above the bed, so a collision at
    // its centre can only come from the blocker itself (the part keeps its mesh
    // coordinates in object space, hence the offset relative to the part).
    ModelObject *object  = model.objects.front();
    ModelVolume *blocker = object->add_volume(TriangleMesh(its_make_cube(6., 6., 4.)));
    blocker->set_type(ModelVolumeType::SUPPORT_BLOCKER);
    const Vec3d part_offset = object->volumes.front()->get_offset();
    blocker->set_offset(Vec3d(part_offset.x() + 20., part_offset.y(), 10.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();

    const PrintObject &print_object = *print.objects().front();
    const std::vector<Vec2d> bed = { { 0., 0. }, { 200., 0. }, { 200., 200. }, { 0., 200. } };
    const BuildVolume build_volume{ bed, print.config().printable_height.value, {}, {} };
    TreeSupport3D::TreeModelVolumes volumes{ print_object, build_volume, scaled<coord_t>(1.), scaled<coord_t>(0.5), 0, {} };

    // The generator's raft layer count: the raft itself plus the gap layers up to the object.
    const size_t num_raft = TreeSupport3D::TreeSupportSettings(TreeSupport3D::TreeSupportMeshGroupSettings(print_object),
                                                               print_object.slicing_parameters()).raft_layers.size();
    REQUIRE(num_raft >= 3);
    // Object layers the blocker was sliced into (object-layer space, as the generator
    // receives them).
    const std::vector<Polygons> blockers = print_object.slice_support_blockers();
    size_t first = 0, last = 0;
    bool   found = false;
    for (size_t i = 0; i < blockers.size(); ++ i)
        if (! blockers[i].empty()) {
            if (! found) { first = i; found = true; }
            last = i;
        }
    REQUIRE(found);
    REQUIRE(last - first > num_raft);
    // The blocker's centre in the slicing frame (add_volume centred its mesh on its offset).
    const Vec3d centre3 = print_object.trafo_sliced() * blocker->get_offset();
    const Point centre  = Point::new_scale(centre3.x(), centre3.y());
    auto collides = [&](size_t tree_layer) {
        for (const Slic3r::Polygon &poly : volumes.getCollision(0, TreeSupport3D::LayerIndex(tree_layer), false))
            if (poly.contains(centre))
                return true;
        return false;
    };
    // In TreeModelVolumes' index space the blocker lives at num_raft + object layer.
    CHECK(collides(num_raft + first));
    CHECK(collides(num_raft + last));
    // The layers just below it, where an unshifted blocker would land, are free; the
    // layers just above the unshifted range, which the blocker does occupy, are not.
    CHECK_FALSE(collides(first));
    CHECK_FALSE(collides(first + num_raft - 1));
    CHECK(collides(last + 1));
    CHECK(collides(last + num_raft));
}

// organic_draw_branches() trims every branch slice against the collision volume (the
// part grown by the support XY distance), the bed and, on a belt, the belt plane before
// it becomes support, so a branch never runs into the part it supports.  Not a belt
// feature: this is the generator every printer uses.
TEST_CASE("Organic tree supports keep their distance from the part", "[Print][Support]")
{
    // A 20 mm cube carrying a 60 x 60 mm plate: a 20 mm wide ceiling all around the
    // cube, 16 mm above the bed, with the cube's four corners in the way of the branches
    // that drop from it.  The plate reaches into the cube so the two shells overlap
    // instead of sharing a face.
    indexed_triangle_set its   = its_make_cube(20., 20., 20.);
    indexed_triangle_set plate = its_make_cube(60., 60., 4.);
    its_translate(its, Vec3f(20.f, 20.f, 0.f));
    its_translate(plate, Vec3f(0.f, 0.f, 16.f));
    its_merge(its, plate);
    TriangleMesh mesh(std::move(its));

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "enable_support",             1 },
        { "support_type",               "tree(auto)" },
        { "support_style",              "organic" },
        { "support_threshold_angle",    30 },
    });
    Print print;
    Model model;
    init_print({ mesh }, print, model, config);
    // On the bed, not at its corner (the fixture leaves the object at the origin).
    model.objects.front()->instances.front()->set_offset(Vec3d(100., 100., 0.));
    print.apply(model, config);
    print.set_status_silent();
    print.process();

    const PrintObject &object = *print.objects().front();
    INFO("object layers " << object.layers().size() << ", support layers " << object.support_layers().size());
    REQUIRE(! object.support_layers().empty());
    // Support exists under the plate at all.
    size_t support_layers_with_fills = 0;
    for (const SupportLayer *layer : object.support_layers())
        if (! layer->support_fills.empty())
            ++ support_layers_with_fills;
    INFO("support layers with extrusions " << support_layers_with_fills);
    CHECK(support_layers_with_fills > 20);

    // Object layers by print_z, to look up the part's slice at a support layer's height.
    std::map<coord_t, const Layer *> object_layers;
    for (const Layer *layer : object.layers())
        object_layers[scaled<coord_t>(layer->print_z)] = layer;
    auto contains = [](const ExPolygons &expolys, const Point &pt) {
        for (const ExPolygon &ex : expolys)
            if (ex.contains(pt))
                return true;
        return false;
    };
    // No support extrusion may run closer to the part's slice than half a line width:
    // the generator keeps the support XY distance (0.35 mm by default) plus the line's
    // own half width away from it.
    const float min_gap = scaled<float>(0.2);
    size_t too_close = 0, points = 0, layers_checked = 0, layers_unmatched = 0;
    for (const SupportLayer *layer : object.support_layers()) {
        if (layer->support_fills.empty())
            continue;
        // The object layer whose slab spans this support layer's height.
        auto it = object_layers.lower_bound(scaled<coord_t>(layer->print_z - EPSILON));
        if (it == object_layers.end()) {
            ++ layers_unmatched;
            continue;
        }
        ++ layers_checked;
        const ExPolygons grown = offset_ex(it->second->lslices, min_gap);
        for (const ExtrusionEntity *entity : layer->support_fills.flatten().entities)
            for (const Slic3r::Polyline &pl : entity->as_polylines())
                for (size_t i = 0; i < pl.points.size(); ++ i) {
                    // The vertices and the midpoints of the segments between them.
                    ++ points;
                    if (contains(grown, pl.points[i]))
                        ++ too_close;
                    if (i + 1 < pl.points.size() && contains(grown, (pl.points[i] + pl.points[i + 1]) / 2))
                        ++ too_close;
                }
    }
    INFO("support layers checked " << layers_checked << " (unmatched " << layers_unmatched << "), support points " << points
         << ", within 0.2 mm of the part " << too_close);
    CHECK(layers_checked > 20);
    CHECK(layers_unmatched == 0);
    REQUIRE(points > 0);
    CHECK(too_close == 0);
}

// Two parts along the belt: the second part's slicing frame starts at the belt
// below its leading end, so its first layers are empty and interleave with the
// first part's printing layers. Those must not reach the G-code as layer changes
// that print nothing: the preview numbers its layers from the moves it sees, and
// a gap folded every later layer into the one before it.
TEST_CASE("Belt G-code has no layer that prints nothing", "[Print][belt][GCode][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "belt_printer",               1 },
        { "belt_slice_rotation",        "x" },
        { "belt_slice_rotation_angle",  45 },
        { "gcode_remap_x",              "rev_x" },
        { "gcode_remap_y",              "pos_z" },
        { "gcode_remap_z",              "pos_y" },
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "skirt_loops",                0 },
        { "z_hop",                      0 },
        { "brim_type",                  "outer_only" },
        { "brim_width",                 4 },
        { "machine_start_gcode",        "T[initial_tool]\n" },
        { "layer_change_gcode",         "G92 E0\n" },
    });
    // Both export paths drop the empty layers and count the layers the same way.
    SECTION("by layer")  { config.set_deserialize_strict({{ "print_sequence", "by layer" }}); }
    SECTION("by object") { config.set_deserialize_strict({{ "print_sequence", "by object" }}); }
    Print print;
    Model model;
    TriangleMesh cube_a(its_make_cube(20., 20., 20.));
    TriangleMesh cube_b(its_make_cube(20., 20., 20.));
    init_print({ cube_a, cube_b }, print, model, config);
    // 60 mm apart along the belt: the second cube's lead-in layers fall among the
    // first cube's layers.
    model.objects[0]->instances.front()->set_offset(Vec3d(50., 40., 0.));
    model.objects[1]->instances.front()->set_offset(Vec3d(50., 100., 0.));
    print.apply(model, config);
    print.set_status_silent();
    const std::string gc = gcode(print);
    REQUIRE(! gc.empty());

    size_t layers = 0, empty = 0, total_header = 0, total_count = 0;
    bool   extruded = true;   // before the first layer change
    auto close_layer = [&]() { if (! extruded) ++ empty; };
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gc, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string &raw = line.raw();
        if (raw.rfind(";LAYER_CHANGE", 0) == 0) {
            close_layer();
            ++ layers;
            extruded = false;
        } else if (raw.rfind("; total layer number: ", 0) == 0) {
            // Counted by the G-code processor from the layer changes it saw.
            total_header = size_t(std::atoi(raw.c_str() + 22));
        } else if (raw.rfind("; total layers count = ", 0) == 0) {
            // GCode::m_layer_count, counted up front from the objects' layers; it also
            // drives the M73 progress and the total_layer_count placeholder.
            total_count = size_t(std::atoi(raw.c_str() + 23));
        } else if (! extruded && line.extruding(self) && line.dist_XY(self) > EPSILON) {
            // Material laid down along a move: a wipe or an unretraction does not count.
            extruded = true;
        }
    });
    close_layer();
    INFO("layers " << layers << ", header " << total_header << ", count " << total_count
         << ", layers without extrusion " << empty);
    CHECK(layers > 150);          // both cubes, 141 layers each, overlapping along the belt
    CHECK(empty == 0);
    CHECK(total_header == layers);
    CHECK(total_count == layers);
}

namespace {

DynamicPrintConfig validation_contract_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "nozzle_diameter", "0.4" },
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "line_width", 0.45 },
        { "wall_loops", 2 },
        { "top_shell_layers", 2 },
        { "bottom_shell_layers", 2 },
        { "sparse_infill_density", "15%" },
        { "enable_support", false },
        { "enable_prime_tower", false },
        { "spiral_mode", false },
        { "belt_printer", false },
        { "printable_height", 100.0 },
        { "gcode_flavor", "marlin2" },
        { "use_relative_e_distances", true },
        { "before_layer_change_gcode", "" },
        { "layer_change_gcode", "G92 E0\n" },
    });
    return config;
}

void check_validation_option(const DynamicPrintConfig &config, const std::string &expected_key,
                             bool valid)
{
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    const auto error = print.validate();
    CAPTURE(expected_key, error.string, error.opt_key);
    if (valid) {
        CHECK(error.string.empty());
    } else {
        REQUIRE_FALSE(error.string.empty());
        CHECK(error.opt_key == expected_key);
        CHECK_FALSE(error.is_warning);
    }
}

} // namespace

TEST_CASE("Print validation bounds line widths by layer height and nozzle size", "[Print][ValidationContracts]")
{
    const std::string key = GENERATE("line_width", "support_line_width", "inner_wall_line_width",
        "outer_wall_line_width", "sparse_infill_line_width", "internal_solid_infill_line_width",
        "top_surface_line_width", "skin_infill_line_width", "skeleton_infill_line_width");
    struct WidthCase { double width; bool valid; };
    const auto part = GENERATE(WidthCase{0.0, true}, WidthCase{0.2, false},
                              WidthCase{0.45, true}, WidthCase{4.0, false});
    CAPTURE(key, part.width);
    auto config = validation_contract_config();
    // Support width is meaningful only while support or a raft is enabled.
    config.set_deserialize_strict({ { "enable_support", true }, { "support_type", "normal(auto)" } });
    config.set_key_value(key, new ConfigOptionFloatOrPercent(part.width, false));
    check_validation_option(config, key, part.valid);
}

TEST_CASE("Print validation resolves relative line widths against nozzle diameter", "[Print][ValidationContracts]")
{
    const double percent = GENERATE(50.0, 112.5, 1000.0);
    auto config = validation_contract_config();
    config.set_key_value("outer_wall_line_width", new ConfigOptionFloatOrPercent(percent, true));
    check_validation_option(config, "outer_wall_line_width", percent > 50.0 && percent < 1000.0);
}

TEST_CASE("Print validation rejects layer heights larger than the nozzle", "[Print][ValidationContracts]")
{
    const std::string key = GENERATE("layer_height", "initial_layer_print_height");
    const double height = GENERATE(0.2, 0.41);
    auto config = validation_contract_config();
    config.set_key_value(key, new ConfigOptionFloat(height));
    check_validation_option(config, key, height < 0.4);
}

TEST_CASE("Print validation permits thin bridge widths only with both thick bridge modes", "[Print][ValidationContracts]")
{
    const bool external_thick = GENERATE(false, true);
    const bool internal_thick = GENERATE(false, true);
    const double width = GENERATE(0.15, 0.3, 0.5);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "thick_bridges", external_thick },
        { "thick_internal_bridges", internal_thick },
        { "bridge_line_width", width },
    });
    const bool valid = width <= 0.4 && (width > 0.2 || (external_thick && internal_thick));
    check_validation_option(config, "bridge_line_width", valid);
}

TEST_CASE("Print validation identifies unsafe organic support dimensions", "[Print][ValidationContracts]")
{
    struct SupportCase { double tip; double branch; const char *error_key; };
    const auto part = GENERATE(SupportCase{0.2, 2.0, "tree_support_tip_diameter"},
        SupportCase{0.6, 0.7, "tree_support_branch_diameter_organic"},
        SupportCase{2.0, 1.2, "tree_support_branch_diameter_organic"}, SupportCase{0.6, 2.0, ""});
    CAPTURE(part.tip, part.branch);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "enable_support", true }, { "support_type", "tree(auto)" }, { "support_style", "organic" },
        { "support_line_width", 0.45 }, { "tree_support_tip_diameter", part.tip },
        { "tree_support_branch_diameter_organic", part.branch },
    });
    check_validation_option(config, part.error_key, std::string(part.error_key).empty());
}

TEST_CASE("Print validation reports a support pattern fallback without blocking slicing", "[Print][ValidationContracts]")
{
    const std::string pattern = GENERATE("hollow", "lightning");
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "enable_support", true }, { "support_type", "normal(auto)" },
        { "support_base_pattern", pattern },
    });
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    std::vector<StringObjectException> warnings;
    REQUIRE(print.validate(&warnings).string.empty());
    const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const auto &entry) {
        return entry.opt_key == "support_base_pattern";
    });
    REQUIRE(warning != warnings.end());
    CHECK(warning->is_warning);
    CHECK_FALSE(warning->string.empty());
}

TEST_CASE("Print validation checks reset commands in both layer hooks and extrusion modes", "[Print][ValidationContracts]")
{
    const std::string hook = GENERATE("before_layer_change_gcode", "layer_change_gcode");
    const bool relative = GENERATE(false, true);
    const std::string command = GENERATE("G92 E0", "  G92 E0.00 ; reset", "g92 E0", "G92 e0", "");
    CAPTURE(hook, relative, command);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "before_layer_change_gcode", "" }, { "layer_change_gcode", "" },
        { "use_relative_e_distances", relative },
    });
    config.set_key_value(hook, new ConfigOptionString(command));
    const bool exact = command == "G92 E0" || command == "  G92 E0.00 ; reset";
    const bool valid = relative ? exact : command.empty();
    const std::string error_key = relative && command.empty() ? "before_layer_change_gcode" : hook;
    check_validation_option(config, error_key, valid);
}

TEST_CASE("Print validation requires per-layer resets only for relative Marlin output", "[Print][ValidationContracts]")
{
    const std::string flavor = GENERATE("marlin", "marlin2", "klipper", "reprapfirmware");
    auto config = validation_contract_config();
    config.set_deserialize_strict({ { "gcode_flavor", flavor }, { "layer_change_gcode", "" } });
    const bool marlin = flavor == "marlin" || flavor == "marlin2";
    check_validation_option(config, "before_layer_change_gcode", !marlin);
}

TEST_CASE("Print validation enforces build height after vertical shrinkage compensation", "[Print][ValidationContracts]")
{
    const double max_height = GENERATE(9.0, 10.0, 20.0);
    const double compensation = GENERATE(100.0, 95.0);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "printable_height", max_height }, { "filament_shrinkage_compensation_z", std::to_string(compensation) },
    });
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    const auto error = print.validate();
    CAPTURE(max_height, compensation, error.string);
    const bool fits = 10.0 * 100.0 / compensation <= max_height;
    CHECK(error.string.empty() == fits);
    if (!fits) {
        CHECK(error.object == print.objects().front()->model_object());
        CHECK_FALSE(error.is_warning);
    }
}

TEST_CASE("Print validation checks filament temperature support for every bed family", "[Print][ValidationContracts]")
{
    struct BedCase { BedType type; const char *temperature_key; };
    const auto bed = GENERATE(BedCase{btPC, "cool_plate_temp"}, BedCase{btEP, "eng_plate_temp"},
        BedCase{btPEI, "hot_plate_temp"}, BedCase{btPTE, "textured_plate_temp"},
        BedCase{btPCT, "textured_cool_plate_temp"}, BedCase{btSuperTack, "supertack_plate_temp"});
    const bool multiple_beds = GENERATE(false, true);
    const int temperature = GENERATE(0, 60);
    auto config = validation_contract_config();
    config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(bed.type));
    config.set_key_value("support_multi_bed_types", new ConfigOptionBool(multiple_beds));
    config.set_key_value(bed.temperature_key, new ConfigOptionInts{temperature});
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    const auto error = print.validate();
    CAPTURE(bed.temperature_key, multiple_beds, temperature, error.string);
    if (multiple_beds && temperature == 0) {
        REQUIRE_FALSE(error.string.empty());
        CHECK(error.type == STRING_EXCEPT_FILAMENT_NOT_MATCH_BED_TYPE);
        REQUIRE(error.params.size() == 3);
        CHECK(error.params[2] == "1"); // The incompatible filament is identified to the caller.
    } else {
        CHECK(error.string.empty());
    }
}

TEST_CASE("Print validation identifies the jerk setting that exceeds machine limits", "[Print][ValidationContracts]")
{
    const std::string key = GENERATE("default_jerk", "outer_wall_jerk", "inner_wall_jerk",
        "infill_jerk", "top_surface_jerk", "initial_layer_jerk", "travel_jerk");
    const double jerk = GENERATE(5.0, 30.0);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "machine_max_junction_deviation", "0,0" },
        { "machine_max_jerk_x", "20,20" }, { "machine_max_jerk_y", "20,20" },
        { "default_jerk", "10" }, { "outer_wall_jerk", "10" }, { "inner_wall_jerk", "10" },
        { "infill_jerk", "10" }, { "top_surface_jerk", "10" },
        { "initial_layer_jerk", "10" }, { "travel_jerk", "10" },
    });
    config.set_key_value(key, new ConfigOptionFloats{jerk});
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    std::vector<StringObjectException> warnings;
    REQUIRE(print.validate(&warnings).string.empty());
    const auto count = std::count_if(warnings.begin(), warnings.end(), [&key](const auto &entry) {
        return entry.opt_key == key && entry.is_warning;
    });
    CAPTURE(key, jerk);
    CHECK(count == (jerk > 20.0 ? 1 : 0));
}

TEST_CASE("Print validation gates junction-deviation warnings on firmware support", "[Print][ValidationContracts]")
{
    const std::string flavor = GENERATE("marlin2", "klipper");
    const double deviation = GENERATE(0.01, 0.1);
    auto config = validation_contract_config();
    config.set_deserialize_strict({
        { "gcode_flavor", flavor }, { "machine_max_junction_deviation", "0.02,0.02" },
        { "default_junction_deviation", std::to_string(deviation) },
        { "default_acceleration", "0" },
        { "default_jerk", "0" }, { "outer_wall_jerk", "0" }, { "inner_wall_jerk", "0" },
    });
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    std::vector<StringObjectException> warnings;
    REQUIRE(print.validate(&warnings).string.empty());
    const auto count = std::count_if(warnings.begin(), warnings.end(), [](const auto &entry) {
        return entry.opt_key == "default_junction_deviation" && entry.is_warning;
    });
    CAPTURE(flavor, deviation);
    CHECK(count == (flavor == "marlin2" && deviation > 0.02 ? 1 : 0));
}

namespace {

struct TravelAccelerationCase {
    const char *flavor;
    double      travel_acceleration;
    double      maximum_travel_acceleration;
    bool        expects_warning;
};

struct TravelAccelerationGateCase {
    const char *flavor;
    double      maximum_travel_acceleration;
    double      default_acceleration;
    bool        expects_warning;
};

void check_travel_acceleration_validation(const char *flavor, double travel_acceleration,
                                          double maximum_travel_acceleration,
                                          double default_acceleration, bool expects_warning)
{
    auto config = validation_contract_config();
    const std::string travel = std::to_string(travel_acceleration) + "," + std::to_string(travel_acceleration);
    const std::string travel_limit = std::to_string(maximum_travel_acceleration) + "," +
                                      std::to_string(maximum_travel_acceleration);
    const std::string default_accel = std::to_string(default_acceleration);
    config.set_deserialize_strict({
        { "gcode_flavor", flavor },
        { "machine_max_acceleration_extruding", "1000,1000" },
        { "machine_max_acceleration_travel", travel_limit },
        { "machine_max_junction_deviation", "0,0" },
        { "default_junction_deviation", "0,0" },
        { "default_acceleration", default_accel },
        { "inner_wall_acceleration", "100,100" },
        { "outer_wall_acceleration", "100,100" },
        { "bridge_acceleration", "100,100" },
        { "initial_layer_acceleration", "100,100" },
        { "sparse_infill_acceleration", "100,100" },
        { "internal_solid_infill_acceleration", "100,100" },
        { "top_surface_acceleration", "100,100" },
        { "travel_acceleration", travel },
        { "default_jerk", "0,0" },
        { "outer_wall_jerk", "0,0" },
        { "inner_wall_jerk", "0,0" },
        { "infill_jerk", "0,0" },
        { "top_surface_jerk", "0,0" },
        { "initial_layer_jerk", "0,0" },
        { "travel_jerk", "0,0" },
    });

    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    std::vector<StringObjectException> warnings;
    const auto error = print.validate(&warnings);
    CAPTURE(flavor, travel_acceleration, maximum_travel_acceleration, default_acceleration,
            error.string, error.opt_key);
    REQUIRE(error.string.empty());
    CHECK_FALSE(error.is_warning);

    const auto warning_count = std::count_if(warnings.begin(), warnings.end(), [](const auto &entry) {
        return entry.opt_key == "travel_acceleration";
    });
    CHECK(warning_count == (expects_warning ? 1 : 0));
    if (expects_warning) {
        const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const auto &entry) {
            return entry.opt_key == "travel_acceleration";
        });
        REQUIRE(warning != warnings.end());
        CHECK(warning->is_warning);
        CHECK_FALSE(warning->string.empty());
    }
}

TEST_CASE("Print validation reports travel acceleration against supported firmware limits",
          "[Print][ValidationContracts][PrintValidationTravelAcceleration]")
{
    const auto part = GENERATE(
        TravelAccelerationCase{"marlin2", 400., 500., false},
        TravelAccelerationCase{"marlin2", 500., 500., false},
        TravelAccelerationCase{"marlin2", 600., 500., true},
        TravelAccelerationCase{"reprapfirmware", 600., 500., true},
        TravelAccelerationCase{"repetier", 600., 500., true});
    check_travel_acceleration_validation(part.flavor, part.travel_acceleration,
                                         part.maximum_travel_acceleration, 100., part.expects_warning);
}

TEST_CASE("Print validation gates travel acceleration warnings on firmware and limits",
          "[Print][ValidationContracts][PrintValidationTravelAcceleration]")
{
    const auto part = GENERATE(
        TravelAccelerationGateCase{"klipper", 500., 100., false},
        TravelAccelerationGateCase{"marlin2", 0., 100., false},
        TravelAccelerationGateCase{"marlin2", 500., 0., false});
    check_travel_acceleration_validation(part.flavor, 600., part.maximum_travel_acceleration,
                                         part.default_acceleration, part.expects_warning);
}

struct BeltValidationCase {
    int         raft_layers;
    DraftShield draft_shield;
    const char *expected_error;
};

TEST_CASE("Print validation rejects belt raft and draft shield combinations",
          "[Print][ValidationContracts][PrintValidationInteractions]")
{
    const auto part = GENERATE(
        BeltValidationCase{0, dsDisabled, ""},
        BeltValidationCase{1, dsDisabled, "Raft is not compatible with belt printer mode."},
        BeltValidationCase{0, dsEnabled, "Draft shield is not compatible with belt printer mode."});
    CAPTURE(part.raft_layers, int(part.draft_shield));

    auto config = validation_contract_config();
    config.set_key_value("belt_printer", new ConfigOptionBool(true));
    config.set_key_value("raft_layers", new ConfigOptionInt(part.raft_layers));
    config.set_key_value("draft_shield", new ConfigOptionEnum<DraftShield>(part.draft_shield));

    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    const StringObjectException error = print.validate();
    CAPTURE(error.string, error.opt_key);
    CHECK_FALSE(error.is_warning);
    if (std::string(part.expected_error).empty()) {
        CHECK(error.string.empty());
        CHECK(error.opt_key.empty());
    } else {
        REQUIRE(error.string == part.expected_error);
        CHECK(error.opt_key.empty());
    }
}

struct SequentialValidationCase {
    TimelapseType timelapse;
    bool          wrapping_detection;
    const char   *expected_error;
    const char   *expected_opt_key;
};

TEST_CASE("Print validation rejects sequential feature combinations",
          "[Print][ValidationContracts][PrintValidationInteractions]")
{
    const auto part = GENERATE(
        SequentialValidationCase{tlTraditional, false, "", ""},
        SequentialValidationCase{tlSmooth, false,
            "Smooth mode of timelapse is not supported when \"by object\" sequence is enabled.", ""},
        SequentialValidationCase{tlTraditional, true,
            "Clumping detection is not supported when \"by object\" sequence is enabled.",
            "enable_wrapping_detection"});
    CAPTURE(int(part.timelapse), part.wrapping_detection);

    auto config = validation_contract_config();
    config.set_deserialize_strict({ { "print_sequence", "by object" } });
    config.set_key_value("timelapse_type", new ConfigOptionEnum<TimelapseType>(part.timelapse));
    config.set_key_value("enable_wrapping_detection", new ConfigOptionBool(part.wrapping_detection));

    Print print;
    Model model;
    build_cubes(model, print, config, /*n=*/2, /*overlap=*/false);
    const StringObjectException error = print.validate();
    CAPTURE(error.string, error.opt_key);
    REQUIRE(error.string == part.expected_error);
    CHECK(error.opt_key == part.expected_opt_key);
    CHECK_FALSE(error.is_warning);
}

struct WipeTowerValidationCase {
    bool          ooze_prevention;
    bool          single_extruder_multi_material;
    const char   *expected_error;
};

void build_two_filament_wipe_print(const DynamicPrintConfig &config, Model &model, Print &print)
{
    // Assign both materials through the public per-object configuration.
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        {},
        { { "outer_wall_filament_id", 2 }, { "inner_wall_filament_id", 2 },
          { "sparse_infill_filament_id", 2 }, { "internal_solid_filament_id", 2 },
          { "top_surface_filament_id", 2 }, { "bottom_surface_filament_id", 2 } },
    };
    init_print(std::vector<TriangleMesh>{ cube(20), cube(20) }, print, model, config, &overrides);
    // The first apply normalizes the tower before the new regions expose both
    // materials. Reapply after the regions exist, as in the slicing fixtures.
    print.apply(model, config);
    REQUIRE(print.extruders(true).size() == 2);
    REQUIRE(print.has_wipe_tower());
}

TEST_CASE("Print validation enforces wipe tower extrusion addressing interactions",
          "[Print][ValidationContracts][PrintValidationInteractions]")
{
    const auto part = GENERATE(
        WipeTowerValidationCase{false, false, ""},
        WipeTowerValidationCase{false, true, ""},
        WipeTowerValidationCase{true, false, ""},
        WipeTowerValidationCase{true, true,
            "Ooze prevention is only supported with the wipe tower when 'single_extruder_multi_material' is off."});
    const bool relative_e = GENERATE(false, true);
    CAPTURE(part.ooze_prevention, part.single_extruder_multi_material, relative_e);

    const DynamicPrintConfig config = multifilament_config(2, {
        { "enable_prime_tower",                 true },
        { "belt_printer",                       false },
        { "spiral_mode",                        false },
        { "print_sequence",                     "by layer" },
        { "timelapse_type",                     "0" },
        { "enable_wrapping_detection",          false },
        { "wipe_tower_type",                    "type2" },
        { "wipe_tower_wall_type",               "rectangle" },
        { "printable_area",                     "0x0,200x0,200x200,0x200" },
        { "wipe_tower_x",                       50.0 },
        { "wipe_tower_y",                       50.0 },
        { "use_relative_e_distances",           relative_e },
        { "layer_change_gcode",                 "G92 E0\n" },
        { "ooze_prevention",                    part.ooze_prevention },
        { "single_extruder_multi_material",     part.single_extruder_multi_material },
    });
    Model model;
    Print print;
    build_two_filament_wipe_print(config, model, print);

    const StringObjectException error = print.validate();
    CAPTURE(error.string, error.opt_key);
    const std::string expected_error = relative_e ? part.expected_error :
        "The Wipe Tower is currently only supported with the relative extruder addressing (use_relative_e_distances=1).";
    REQUIRE(error.string == expected_error);
    CHECK(error.opt_key.empty());
    CHECK_FALSE(error.is_warning);
}

} // namespace

namespace print_workflow_contracts {
namespace {

DynamicPrintConfig pipeline_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"top_shell_layers", 2},
        {"bottom_shell_layers", 2},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"sparse_infill_density", "15%"},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"before_layer_change_gcode", ""},
        {"gcode_comments", true},
    });
    return config;
}

void require_valid(const Print &print)
{
    const StringObjectException error = print.validate();
    INFO(error.string);
    REQUIRE(error.string.empty());
}

void init_valid(std::initializer_list<TriangleMesh> meshes, Print &print, Model &model,
                const DynamicPrintConfig &config, bool arrange = true, size_t instances = 1)
{
    if (instances == 1 && arrange) {
        init_print(meshes, print, model, config);
    } else {
        init_print(std::vector<TriangleMesh>(meshes), print, model, config, nullptr, arrange, instances);
    }
    require_valid(print);
}

double layer_slice_area(const PrintObject &object, size_t index)
{
    double area = 0.0;
    for (const ExPolygon &slice : object.get_layer(static_cast<int>(index))->lslices)
        // ExPolygon::area() is in scaled-coordinate squared units.
        area += slice.area() * SCALING_FACTOR * SCALING_FACTOR;
    return area;
}

double polygon_area_mm2(const Polygons &polygons)
{
    return area(polygons) * SCALING_FACTOR * SCALING_FACTOR;
}

size_t perimeter_items(const PrintObject &object, size_t layer)
{
    size_t items = 0;
    for (const LayerRegion *region : object.get_layer(static_cast<int>(layer))->regions())
        items += region->perimeters.items_count();
    return items;
}

size_t support_polygon_count(const PrintObject &object)
{
    size_t count = 0;
    for (const SupportLayer *layer : object.support_layers())
        count += layer->support_islands.size();
    return count;
}

class ScopedResourcesDir
{
public:
    explicit ScopedResourcesDir(const boost::filesystem::path &path) : previous(resources_dir())
    {
        set_resources_dir(path.string());
    }
    ~ScopedResourcesDir() { set_resources_dir(previous); }
private:
    std::string previous;
};

} // namespace

TEST_CASE("An empty Print has no completed workflow and clear removes applied objects", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    CHECK(print.empty());
    CHECK_FALSE(print.finished());
    CHECK_FALSE(print.is_step_done(posSlice));
    CHECK_FALSE(print.is_step_done(psGCodeExport));
    REQUIRE_THAT(print.max_allowed_layer_height(), Catch::Matchers::WithinAbs(0.0, 1e-9));

    Model model;
    init_valid({cube(20.0)}, print, model, pipeline_config());
    CHECK_FALSE(print.empty());
    print.clear();
    CHECK(print.empty());
    CHECK(print.print_object_ids().empty());
    CHECK_FALSE(print.finished());
}

TEST_CASE("Processing and exporting advance the Print lifecycle in order", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(20.0)}, print, model, pipeline_config());
    CHECK_FALSE(print.is_step_done(posInfill));
    CHECK_FALSE(print.is_step_done(psGCodeExport));

    print.process();
    REQUIRE(print.is_step_done(posSlice));
    REQUIRE(print.is_step_done(posInfill));
    REQUIRE(print.is_step_done(psSkirtBrim));
    CHECK_FALSE(print.finished());

    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    CHECK(print.is_step_done(psGCodeExport));
    CHECK(print.finished());
}

TEST_CASE("A G-code-only config change preserves sliced object steps and invalidates export", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    Print print;
    Model model;
    init_valid({cube(20.0)}, print, model, config);
    print.process();
    const size_t layers_before = print.objects().front()->layer_count();
    gcode(print);
    REQUIRE(print.finished());

    config.set_deserialize_strict({{"gcode_comments", false}, {"default_acceleration", "1200"}});
    print.apply(model, config);
    CHECK(print.objects().front()->layer_count() == layers_before);
    CHECK_FALSE(print.is_step_done(psGCodeExport));
    print.process();
    CHECK(print.is_step_done(posInfill));
    CHECK_FALSE(gcode(print).empty());
}

TEST_CASE("A wall config change invalidates and recomputes perimeters", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    Print print;
    Model model;
    init_valid({cube(20.0)}, print, model, config);
    print.process();
    const PrintObject *object = print.objects().front();
    REQUIRE(object->layer_count() > 3);
    const size_t layers_before = object->layer_count();
    const size_t before = perimeter_items(*object, 2);

    config.set_deserialize_strict({{"wall_loops", 4}});
    print.apply(model, config);
    CHECK_FALSE(print.is_step_done(posPerimeters));
    CHECK_FALSE(print.is_step_done(posInfill));
    print.process();
    const size_t after = perimeter_items(*print.objects().front(), 2);
    CHECK(after > before);
    CHECK(print.objects().front()->layer_count() == layers_before);
}

TEST_CASE("Multiple overhang objects sort layers by their distinct bounding footprints", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    TriangleMesh large = mesh(TestMesh::overhang);
    TriangleMesh small = mesh(TestMesh::overhang);
    small.scale(0.7f);
    // Distinct overhang footprints make the descending-area order observable.
    init_valid({std::move(large), std::move(small)}, print, model, pipeline_config());
    REQUIRE(print.objects().size() == 2);
    REQUIRE(print.num_object_instances() == 2);
    const std::vector<ObjectID> ids = print.print_object_ids();
    REQUIRE(ids.size() == 2);
    CHECK(ids[0] != ids[1]);

    print.process();
    std::vector<LayerPtrs> layers;
    std::vector<BoundingBox> boxes;
    VecOfPoints shifts;
    const std::vector<size_t> order = print.layers_sorted_for_object(0.f, 100.f, layers, boxes, shifts);
    REQUIRE(order.size() == 2);
    REQUIRE(layers.size() == 2);
    REQUIRE(boxes.size() == 2);
    REQUIRE(shifts.size() == 2);
    CHECK(layers[0].size() > 0);
    CHECK(layers[1].size() > 0);
    REQUIRE(boxes[0].area() != boxes[1].area());
    const size_t larger = boxes[0].area() > boxes[1].area() ? 0 : 1;
    CHECK(order.front() == larger);
    CHECK(boxes[order.front()].area() > boxes[order.back()].area());
}

TEST_CASE("Multiple instances contribute to object counts and first-layer geometry", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(15.0)}, print, model, pipeline_config(), false, 2);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->instances().size() == 2);
    CHECK(print.num_object_instances() == 2);
    print.process();
    REQUIRE(print.objects().front()->layer_count() > 1);
    CHECK(print.first_layer_convex_hull().size() >= 4);
}

TEST_CASE("Object and support filament collections report the configured extruders", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = multifilament_config(2, {
        {"enable_support", true},
        {"support_filament", 2},
        {"support_interface_filament", 2},
        {"nozzle_diameter", "0.4,0.8"},
    });
    config.set_deserialize_strict({{"layer_change_gcode", "G92 E0\n"}, {"machine_start_gcode", ""}, {"machine_end_gcode", ""}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(cube(20.0));
    object->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(2));
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    require_valid(print);

    const std::vector<unsigned int> objects = print.object_extruders();
    const std::vector<unsigned int> supports = print.support_material_extruders();
    REQUIRE(objects.size() == 1);
    CHECK(objects.front() == 1);
    REQUIRE(supports.size() == 1);
    CHECK(supports.front() == 1);
    const std::vector<unsigned int> all = print.extruders();
    REQUIRE(all.size() == 1);
    CHECK(all.front() == 1);
    REQUIRE_THAT(print.max_allowed_layer_height(), Catch::Matchers::WithinAbs(0.8, 1e-6));
}

TEST_CASE("PrintObject layer lookup distinguishes exact, epsilon, below, and bottom queries", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(8.0)}, print, model, pipeline_config());
    print.process();
    PrintObject &object = *print.get_object(0);
    REQUIRE(object.layer_count() > 3);
    const Layer *middle = object.get_layer(2);
    REQUIRE(middle != nullptr);
    CHECK(object.get_layer_at_printz(middle->print_z) == middle);
    CHECK(object.get_layer_at_printz(middle->print_z + 0.01, 0.02) == middle);
    CHECK(object.get_layer_at_printz(middle->print_z + 0.05, 0.01) == nullptr);
    CHECK(object.get_layer_idx_get_printz(middle->print_z, 0.001) == 3);
    CHECK(object.get_first_layer_bellow_printz(middle->print_z + 0.05, 0.01) == middle);
    CHECK(object.get_layer_at_bottomz(middle->bottom_z(), 0.001) == middle);
    CHECK(object.get_layer_at_bottomz(middle->bottom_z() + 0.05, 0.001) == nullptr);
}

TEST_CASE("Layer range and instance shift queries preserve selected geometry", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(12.0)}, print, model, pipeline_config(), false, 2);
    print.process();
    PrintObject &object = *print.get_object(0);
    REQUIRE(object.layer_count() > 4);
    std::vector<LayerPtrs> selected;
    std::vector<BoundingBox> boxes;
    const float start_z = object.get_layer(1)->print_z;
    const float end_z = object.get_layer(3)->print_z + 0.005;
    INFO("start=" << start_z << " end=" << end_z << " z1=" << object.get_layer(1)->print_z << " z2=" << object.get_layer(2)->print_z << " z3=" << object.get_layer(3)->print_z);
    object.get_certain_layers(start_z, end_z, selected, boxes);
    REQUIRE(selected.size() == 1);
    REQUIRE(selected.front().size() >= 2);
    INFO("selected_count=" << selected.front().size() << " first=" << selected.front().front()->print_z << " last=" << selected.front().back()->print_z);
    for (const Layer *layer : selected.front()) {
        CHECK(layer->print_z >= start_z - EPSILON);
        CHECK(layer->print_z <= end_z + EPSILON);
    }
    REQUIRE(boxes.size() == 1);
    const Points first_shifts = object.get_instances_shift_without_plate_offset();
    REQUIRE(first_shifts.size() == 2);

    print.set_plate_origin(Vec3d(3.0, 4.0, 0.0));
    const Points shifted = object.get_instances_shift_without_plate_offset();
    REQUIRE(shifted.size() == first_shifts.size());
    for (size_t i = 0; i < shifted.size(); ++i) {
        CHECK(shifted[i].x() == first_shifts[i].x() - scaled(3.0));
        CHECK(shifted[i].y() == first_shifts[i].y() - scaled(4.0));
    }
}

TEST_CASE("The first-layer bbox reports a 20 mm cube's area, height, and identity", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    ModelObject *object = model.add_object();
    object->name = "bbox-cube";
    object->add_volume(cube(20.0));
    object->add_instance();
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, pipeline_config());
    require_valid(print);
    print.process();
    float area = 0.f;
    float height = 0.f;
    std::string name;
    const BoundingBox bbox = print.get_object(0)->get_first_layer_bbox(area, height, name);
    REQUIRE(bbox.defined);
    CHECK_THAT(static_cast<double>(area) * SCALING_FACTOR * SCALING_FACTOR,
               Catch::Matchers::WithinAbs(400.0, 1e-3));
    CHECK_THAT(height, Catch::Matchers::WithinAbs(0.2f, 1e-5f));
    CHECK(name == "bbox-cube");
    CHECK_THAT(unscaled<double>(bbox.size().x()), Catch::Matchers::WithinAbs(20.0, 1e-5));
    CHECK_THAT(unscaled<double>(bbox.size().y()), Catch::Matchers::WithinAbs(20.0, 1e-5));
}

TEST_CASE("Support blocker and enforcer volumes slice into their own polygons", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    config.set_deserialize_strict({{"enable_support", true}, {"support_type", "normal"}});
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(cube(20.0));
    object->add_volume(make_cube(5.0, 5.0, 5.0), ModelVolumeType::SUPPORT_BLOCKER);
    object->add_volume(make_cube(4.0, 4.0, 4.0), ModelVolumeType::SUPPORT_ENFORCER);
    object->volumes[1]->set_offset(Vec3d(7.0, 7.0, 0.0));
    object->volumes[2]->set_offset(Vec3d(-7.0, -7.0, 0.0));
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    require_valid(print);
    print.process();
    const PrintObject &po = *print.objects().front();
    const std::vector<Polygons> blockers = po.slice_support_blockers();
    const std::vector<Polygons> enforcers = po.slice_support_enforcers();
    REQUIRE(blockers.size() == po.layer_count());
    REQUIRE(enforcers.size() == po.layer_count());
    std::vector<size_t> blocker_layers;
    std::vector<size_t> enforcer_layers;
    for (size_t i = 0; i < blockers.size(); ++i) {
        if (!blockers[i].empty())
            blocker_layers.push_back(i);
        if (!enforcers[i].empty())
            enforcer_layers.push_back(i);
    }
    REQUIRE_FALSE(blocker_layers.empty());
    REQUIRE_FALSE(enforcer_layers.empty());
    const BoundingBoxf3 blocker_bbox = object->volumes[1]->mesh().transformed_bounding_box(po.trafo_sliced() * object->volumes[1]->get_matrix());
    const BoundingBoxf3 enforcer_bbox = object->volumes[2]->mesh().transformed_bounding_box(po.trafo_sliced() * object->volumes[2]->get_matrix());
    INFO("blocker bbox z=" << blocker_bbox.min.z() << ".." << blocker_bbox.max.z()
         << " enforcer bbox z=" << enforcer_bbox.min.z() << ".." << enforcer_bbox.max.z()
         << " blocker last index=" << blocker_layers.back() << " enforcer last index=" << enforcer_layers.back());
    CHECK(blocker_layers.front() == 0);
    CHECK(enforcer_layers.front() == 0);
    CHECK_THAT(po.get_layer(static_cast<int>(blocker_layers.front()))->print_z,
               Catch::Matchers::WithinAbs(0.2, 1e-5));
    const double blocker_top_z = std::max(0.0, blocker_bbox.max.z());
    const double enforcer_top_z = std::max(0.0, enforcer_bbox.max.z());
    CHECK(po.get_layer(static_cast<int>(blocker_layers.back()))->print_z <= blocker_top_z + 0.2f + EPSILON);
    CHECK(po.get_layer(static_cast<int>(enforcer_layers.back()))->print_z <= enforcer_top_z + 0.2f + EPSILON);
    REQUIRE(blocker_layers.back() + 1 < po.layer_count());
    CHECK(po.get_layer(static_cast<int>(blocker_layers.back() + 1))->print_z > blocker_top_z - EPSILON);
    REQUIRE(enforcer_layers.back() + 1 < po.layer_count());
    CHECK(po.get_layer(static_cast<int>(enforcer_layers.back() + 1))->print_z > enforcer_top_z - EPSILON);
    for (size_t i : blocker_layers)
        CHECK_THAT(polygon_area_mm2(blockers[i]), Catch::Matchers::WithinAbs(25.0, 1e-3));
    for (size_t i : enforcer_layers)
        CHECK_THAT(polygon_area_mm2(enforcers[i]), Catch::Matchers::WithinAbs(16.0, 1e-3));
}

TEST_CASE("Single-volume slicing preserves a hole in every occupied layer", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({mesh(TestMesh::cube_with_hole)}, print, model, pipeline_config());
    print.process();
    const PrintObject &object = *print.objects().front();
    REQUIRE_FALSE(object.model_object()->volumes.empty());
    const std::vector<ExPolygons> slices = object.slice_single_volume_regions(object.model_object()->volumes.front());
    REQUIRE(slices.size() == object.layer_count());
    size_t occupied = 0;
    for (const ExPolygons &layer : slices) {
        for (const ExPolygon &region : layer)
            CHECK_FALSE(region.holes.empty());
        occupied += !layer.empty();
    }
    CHECK(occupied > 0);
}

TEST_CASE("Support preview generates support layers without requiring full G-code export", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    config.set_deserialize_strict({
        {"enable_support", true},
        {"support_type", "normal(auto)"},
        {"support_style", "default"},
        {"support_threshold_angle", 30.0},
        {"support_on_build_plate_only", false},
    });
    Print print;
    Model model;
    init_valid({mesh(TestMesh::overhang)}, print, model, config);
    PrintObject *object = print.get_object(0);
    REQUIRE(object != nullptr);
    object->generate_support_preview();
    REQUIRE(object->layer_count() > 0);
    CHECK(object->support_layer_count() > 0);
    CHECK(support_polygon_count(*object) > 0);
}

TEST_CASE("Slicing parameters reflect layer height, object height, and shrinkage", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(cube(20.0));
    object->add_instance();
    const SlicingParameters normal = PrintObject::slicing_parameters(config, *object, 20.f, Vec3d(1., 1., 1.));
    REQUIRE(normal.valid);
    CHECK_THAT(normal.layer_height, Catch::Matchers::WithinAbs(0.2, 1e-5));
    CHECK_THAT(normal.object_print_z_height(), Catch::Matchers::WithinAbs(20.0, 1e-4));

    config.set_deserialize_strict({{"layer_height", 0.4}, {"initial_layer_print_height", 0.3}});
    const SlicingParameters changed = PrintObject::slicing_parameters(config, *object, 20.f, Vec3d(1., 1., 1.1));
    REQUIRE(changed.valid);
    CHECK_THAT(changed.layer_height, Catch::Matchers::WithinAbs(0.4, 1e-5));
    CHECK_THAT(changed.first_print_layer_height, Catch::Matchers::WithinAbs(0.3, 1e-5));
    CHECK_THAT(changed.object_print_z_height(), Catch::Matchers::WithinAbs(22.0, 1e-4));
}

TEST_CASE("A negative volume removes only its footprint from the first layer", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    Print positive;
    Model positive_model;
    init_valid({cube(20.0)}, positive, positive_model, config);
    positive.process();
    const double positive_area = layer_slice_area(*positive.objects().front(), 0);

    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(cube(20.0));
    object->add_volume(make_cube(6.0, 6.0, 20.0), ModelVolumeType::NEGATIVE_VOLUME);
    // Place the 6 x 6 negative volume in the interior of the 20 x 20 base.
    object->volumes[1]->set_offset(Vec3d(7., 7., 0.));
    object->add_instance();
    object->ensure_on_bed();
    Print cut;
    cut.auto_assign_extruders(object);
    cut.apply(model, config);
    require_valid(cut);
    cut.process();
    const double cut_area = layer_slice_area(*cut.objects().front(), 0);
    CHECK_THAT(positive_area, Catch::Matchers::WithinAbs(400.0, 1e-3));
    CHECK_THAT(cut_area, Catch::Matchers::WithinAbs(364.0, 1e-3));
    CHECK_THAT(positive_area - cut_area, Catch::Matchers::WithinAbs(36.0, 1e-3));
}

TEST_CASE("Overlapping model parts assign modifiers and subtract negative volumes", "[Print][VolumeRegionAssignment]")
{
    DynamicPrintConfig config = pipeline_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"brim_type", "no_brim"},
        {"slice_closing_radius", 0.0},
        {"xy_hole_compensation", 0.0},
        {"xy_contour_compensation", 0.0},
    });

    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *part_a = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
    // The slicer orders model volumes by ObjectID, so establish the intended parent/overlap order explicitly.
    ModelVolume *modifier = object->add_volume(make_cube(10., 10., 4.), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_offset(Vec3d(15., 5., 0.));
    modifier->config.set("wall_loops", 4);
    ModelVolume *part_b = object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
    part_b->set_offset(Vec3d(10., 10., 0.));
    ModelVolume *negative = object->add_volume(make_cube(8., 8., 4.), ModelVolumeType::NEGATIVE_VOLUME, false);
    negative->set_offset(Vec3d(16., 16., 0.));
    REQUIRE(part_a->is_model_part());
    REQUIRE(modifier->is_modifier());
    REQUIRE(part_b->is_model_part());
    REQUIRE(negative->is_negative_volume());
    REQUIRE(part_a->id() < modifier->id());
    REQUIRE(modifier->id() < part_b->id());
    REQUIRE(part_b->id() < negative->id());

    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    require_valid(print);
    PrintObject *print_object = print.get_object(0);
    REQUIRE(print_object != nullptr);
    print_object->slice();
    REQUIRE(print_object->layer_count() > 4);
    REQUIRE(print_object->num_printing_regions() == 2);

    size_t layer_idx = print_object->layer_count() / 2;
    REQUIRE(layer_idx < print_object->layer_count());
    const Layer *layer = print_object->get_layer(static_cast<int>(layer_idx));
    REQUIRE(layer != nullptr);
    REQUIRE(layer->print_z > 0.0f);
    REQUIRE(layer->print_z < 4.0f);
    auto region_area_mm2 = [](const LayerRegion &region) {
        double area = 0.;
        for (const Surface &surface : region.slices.surfaces)
            area += std::abs(static_cast<double>(surface.expolygon.area())) * SCALING_FACTOR * SCALING_FACTOR;
        return area;
    };

    double default_area = 0.;
    double modifier_area = 0.;
    for (size_t region_id = 0; region_id < print_object->num_printing_regions(); ++region_id) {
        const double area = region_area_mm2(*layer->get_region(static_cast<int>(region_id)));
        if (print_object->printing_region(region_id).config().wall_loops.value == 4)
            modifier_area += area;
        else
            default_area += area;
    }

    // A=[0,20]^2, B=[10,30]^2 => union 700. N=[16,24]^2 removes 64.
    // M=[15,25]x[5,15] is clipped to A, then to the later B overlap, leaving 25.
    CHECK_THAT(modifier_area, Catch::Matchers::WithinAbs(25., 0.05));
    CHECK_THAT(default_area, Catch::Matchers::WithinAbs(611., 0.05));
    CHECK_THAT(default_area + modifier_area, Catch::Matchers::WithinAbs(636., 0.05));
}

TEST_CASE("Same-type support volumes merge overlap and preserve separate islands", "[Print][VolumeRegionAssignment]")
{
    DynamicPrintConfig config = pipeline_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"slice_closing_radius", 0.0},
        {"xy_hole_compensation", 0.0},
        {"xy_contour_compensation", 0.0},
    });
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 4.), ModelVolumeType::MODEL_PART, false);
    object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::SUPPORT_BLOCKER, false)->set_offset(Vec3d(4., 4., 0.));
    object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::SUPPORT_BLOCKER, false)->set_offset(Vec3d(6., 4., 0.));
    object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::SUPPORT_BLOCKER, false)->set_offset(Vec3d(15., 4., 0.));
    object->add_instance();
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    require_valid(print);
    PrintObject *print_object = print.get_object(0);
    REQUIRE(print_object != nullptr);
    print_object->slice();

    size_t layer_idx = print_object->layer_count();
    for (size_t i = 0; i < print_object->layer_count(); ++i)
        if (print_object->get_layer(static_cast<int>(i))->print_z > 0.3 &&
            print_object->get_layer(static_cast<int>(i))->print_z < 3.5) {
            layer_idx = i;
            break;
        }
    REQUIRE(layer_idx < print_object->layer_count());
    const std::vector<Polygons> blockers = print_object->slice_support_blockers();
    REQUIRE(blockers.size() == print_object->layer_count());
    REQUIRE(blockers[layer_idx].size() == 2);
    CHECK_THAT(polygon_area_mm2(blockers[layer_idx]), Catch::Matchers::WithinAbs(40., 0.05));
}

TEST_CASE("A layer-range override creates an additional print region and changes wall planning", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(cube(20.0));
    object->add_instance();
    ModelConfig range;
    range.set_key_value("wall_loops", new ConfigOptionInt(4));
    range.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    object->layer_config_ranges[{1.0, 3.0}] = range;
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    require_valid(print);
    print.process();
    const PrintObject &po = *print.objects().front();
    REQUIRE(po.layer_count() > 10);
    CHECK(po.num_printing_regions() >= 2);
    auto first_layer_matching = [&po](auto predicate) -> size_t {
        for (size_t i = 0; i < po.layer_count(); ++i)
            if (predicate(po.get_layer(static_cast<int>(i))->print_z))
                return i;
        return po.layer_count();
    };
    const size_t below = first_layer_matching([](float z) { return z < 1.0f - EPSILON; });
    const size_t inside = first_layer_matching([](float z) { return z > 1.0f + EPSILON && z < 3.0f - EPSILON; });
    const size_t above = first_layer_matching([](float z) { return z > 3.0f + EPSILON; });
    REQUIRE(below < po.layer_count());
    REQUIRE(inside < po.layer_count());
    REQUIRE(above < po.layer_count());
    CHECK(perimeter_items(po, below) == 2);
    CHECK(perimeter_items(po, inside) == 4);
    CHECK(perimeter_items(po, above) == 2);
}

TEST_CASE("Cache export persists layer geometry and rejects a missing directory", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(16.0)}, print, model, pipeline_config());
    print.process();
    const PrintObject &before_object = *print.objects().front();
    REQUIRE(before_object.layer_count() > 2);
    const size_t layer_count = before_object.layer_count();
    const size_t first_perimeters = perimeter_items(before_object, 0);

    ScopedTemporaryDir cache("orca-print-cache");
    REQUIRE(print.export_cached_data(cache.string(), true) == 0);
    std::ifstream cache_file;
    for (const auto &entry : boost::filesystem::directory_iterator(cache.string())) {
        cache_file.open(entry.path().string(), std::ios::binary);
        if (cache_file.good())
            break;
        cache_file.clear();
    }
    REQUIRE(cache_file.good());
    const nlohmann::json payload = nlohmann::json::parse(cache_file);
    CHECK(payload.at("name").get<std::string>() == before_object.model_object()->name);
    const auto &layers = payload.at("layers");
    REQUIRE(layers.size() == layer_count);
    for (size_t i = 0; i < layers.size(); ++i) {
        CHECK_THAT(layers.at(i).at("height").get<double>(), Catch::Matchers::WithinAbs(0.2, 1e-6));
        CHECK_THAT(layers.at(i).at("print_z").get<double>(), Catch::Matchers::WithinAbs(0.2 * (i + 1), 1e-5));
    }
    CHECK(print.load_cached_data(cache.string() + "/missing") == CLI_IMPORT_CACHE_NOT_FOUND);
    CHECK(before_object.layer_count() == layer_count);
    CHECK(layer_slice_area(before_object, 0) > 0.0);
    CHECK(first_perimeters > 0);
}

TEST_CASE("Filament temperature classification and mixed compatibility are symmetric", "[Print][PrintPipelineWorkflows]")
{
    CHECK(Print::get_filament_temp_type("PLA") == FilamentTempType::LowTemp);
    CHECK(Print::get_filament_temp_type("ABS") == FilamentTempType::HighTemp);
    CHECK(Print::get_filament_temp_type("not-a-material") == FilamentTempType::Undefine);
    CHECK(Print::is_filaments_compatible({FilamentTempType::LowTemp, FilamentTempType::LowTemp}));
    CHECK_FALSE(Print::is_filaments_compatible({FilamentTempType::LowTemp, FilamentTempType::HighTemp}));
    CHECK_FALSE(Print::is_filaments_compatible({FilamentTempType::HighTemp, FilamentTempType::LowTemp}));
    CHECK(Print::get_compatible_filament_type({FilamentTempType::HighTemp}) == FilamentTempType::HighTemp);
    CHECK(Print::get_compatible_filament_type({FilamentTempType::LowTemp, FilamentTempType::HighTemp}) == FilamentTempType::HighLowCompatible);
    ScopedResourcesDir resources(boost::filesystem::path(PROFILES_DIR).parent_path());
    const std::vector<std::string> incompatible = Print::get_incompatible_filaments_by_nozzle(0.4f);
    CHECK_FALSE(incompatible.empty());
    const std::vector<std::string> high_flow_incompatible =
        Print::get_incompatible_filaments_by_nozzle(0.4f, NozzleVolumeType::nvtHighFlow);
    REQUIRE_FALSE(high_flow_incompatible.empty());
}

TEST_CASE("Print-space translation follows the configured plate origin", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    print.set_plate_origin(Vec3d(12.5, -7.25, 0.0));
    const Vec2d point(15.0, -3.25);
    const Vec2d translated = print.translate_to_print_space(point);
    CHECK_THAT(translated.x(), Catch::Matchers::WithinAbs(2.5, 1e-9));
    CHECK_THAT(translated.y(), Catch::Matchers::WithinAbs(4.0, 1e-9));
    const Point scaled_point = Point::new_scale(15.0, -3.25);
    const Vec2d translated_scaled = print.translate_to_print_space(scaled_point);
    CHECK_THAT(translated_scaled.x(), Catch::Matchers::WithinAbs(2.5, 1e-9));
    CHECK_THAT(translated_scaled.y(), Catch::Matchers::WithinAbs(4.0, 1e-9));
}

TEST_CASE("Print validates sequential collisions and reports the colliding footprint", "[Print][PrintPipelineWorkflows]")
{
    DynamicPrintConfig config = pipeline_config();
    config.set_deserialize_strict({{"print_sequence", "by object"}, {"extruder_clearance_radius", 10.0}});
    Print print;
    Model model;
    TriangleMesh left = cube(20.0);
    TriangleMesh right = cube(20.0);
    right.translate(18.0, 0.0, 0.0);
    init_print(std::vector<TriangleMesh>{std::move(left), std::move(right)}, print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE_FALSE(validation.string.empty());
    CHECK(validation.type == STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT);
    Polygons collision;
    const StringObjectException error = Print::sequential_print_clearance_valid(print, &collision);
    REQUIRE_FALSE(error.string.empty());
    CHECK_FALSE(collision.empty());
}

TEST_CASE("Print g-code export leaves a readable file result and can reload moves", "[Print][PrintPipelineWorkflows]")
{
    Print print;
    Model model;
    init_valid({cube(10.0)}, print, model, pipeline_config());
    print.process();
    ScopedTemporaryFile file(".gcode");
    GCodeProcessorResult result;
    const std::string path = print.export_gcode(file.string(), &result, nullptr);
    REQUIRE(path == file.string());
    std::ifstream stream(path, std::ios::binary);
    const std::string data((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(data.empty());
    CHECK(result.moves.size() > 0);
    const size_t old_moves = result.moves.size();
    print.reload_gcode_moves(&result);
    CHECK(result.moves.size() == old_moves);
}

namespace {
using Json = nlohmann::json;

std::vector<ExtrusionPath> cached_paths(const PrintObject &object)
{
    std::vector<ExtrusionPath> result;
    for (const Layer *layer : object.layers())
        for (const LayerRegion *region : layer->regions()) {
            for_each_extrusion_path(region->perimeters, [&](const ExtrusionPath &p) { result.push_back(p); });
            for_each_extrusion_path(region->fills, [&](const ExtrusionPath &p) { result.push_back(p); });
        }
    return result;
}

struct CachedPathSnapshot {
    std::string geometry_key;
    ExtrusionRole role;
    double mm3_per_mm;
    float width;
    float height;
};

std::vector<CachedPathSnapshot> canonical_cached_paths(const PrintObject &object)
{
    std::vector<CachedPathSnapshot> result;
    const std::vector<ExtrusionPath> paths = cached_paths(object);
    result.reserve(paths.size());
    for (const ExtrusionPath &path : paths) {
        std::ostringstream key;
        key << static_cast<int>(path.role()) << ':';
        for (const Point3 &point : path.polyline.points)
            key << point.x() << ',' << point.y() << ',' << point.z() << ';';
        result.push_back({key.str(), path.role(), path.mm3_per_mm, path.width, path.height});
    }
    // Cache JSON traversal and regenerated extrusion collections are implementation details;
    // compare a multiset-like sorted snapshot rather than traversal order.
    std::sort(result.begin(), result.end(), [](const CachedPathSnapshot &lhs, const CachedPathSnapshot &rhs) {
        return lhs.geometry_key < rhs.geometry_key;
    });
    return result;
}

void rewrite_cache(const std::string &directory, const std::function<void(Json &)> &edit)
{
    size_t edited = 0;
    for (const auto &entry : boost::filesystem::directory_iterator(directory)) {
        std::ifstream input(entry.path().string());
        Json document = Json::parse(input);
        input.close();
        edit(document);
        std::ofstream output(entry.path().string());
        output << document.dump();
        REQUIRE(output.good());
        ++edited;
    }
    REQUIRE(edited == 1);
}

void visit_cached_extrusions(Json &node, const std::function<void(Json &)> &edit)
{
    if (node.is_object() && node.contains("polyline"))
        edit(node);
    if (node.is_structured())
        for (auto &child : node)
            visit_cached_extrusions(child, edit);
}

void inject_arc(Print &print)
{
    ExtrusionPath path(erPerimeter, 0.08, 0.4, 0.2);
    path.polyline.points = {Point3::new_scale(1., 0., 0.1),
                            Point3::new_scale(0.6, 0.8, 0.2),
                            Point3::new_scale(0., 1., 0.3)};
    path.z_contoured = true;
    ArcSegment arc(Point(0, 0), scaled(1.), Point(scaled(1.), 0), Point(0, scaled(1.)), ArcDirection::Arc_Dir_CCW);
    REQUIRE(arc.is_arc);
    path.polyline.fitting_result.push_back({0, 2, EMovePathType::Arc_move_ccw, arc});
    LayerRegion *region = print.get_object(0)->get_layer(0)->get_region(0);
    region->perimeters.clear();
    region->perimeters.entities.push_back(new ExtrusionPath(path));
}
}

TEST_CASE("Cached planar layers preserve extrusion coordinates and material flow", "[Print][PrintCacheRoundtrip]")
{
    Print print;
    Model model;
    init_valid({cube(4.)}, print, model, pipeline_config());
    print.process();
    const auto before = cached_paths(*print.objects().front());
    REQUIRE_FALSE(before.empty());
    ScopedTemporaryDir cache("orca-cache-planar");
    REQUIRE(print.export_cached_data(cache.string(), true) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    REQUIRE(print.objects().front()->layer_count() == 20);
    const auto after = cached_paths(*print.objects().front());
    REQUIRE(after.size() == before.size());
    for (size_t i = 0; i < before.size(); ++i) {
        CHECK(after[i].polyline.points == before[i].polyline.points);
        CHECK(after[i].role() == before[i].role());
        CHECK_THAT(after[i].mm3_per_mm, Catch::Matchers::WithinAbs(before[i].mm3_per_mm, 1e-12));
        CHECK_THAT(after[i].width, Catch::Matchers::WithinAbs(before[i].width, 1e-6));
        CHECK_THAT(after[i].height, Catch::Matchers::WithinAbs(0.2, 1e-6));
    }
}

TEST_CASE("Print process selects cached paths and completes cached object steps", "[Print][PrintCacheProcess]")
{
    constexpr double calibrated_flow = 0.12345;
    const bool use_cache = GENERATE(false, true);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"enable_support", false},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"gcode_comments", true},
        {"use_relative_e_distances", true},
        {"before_layer_change_gcode", "G92 E0\n"},
    });

    Print source;
    Model model;
    init_valid({make_cube(8.0, 8.0, 1.2)}, source, model, config);
    REQUIRE(source.objects().size() == 1);
    const PrintObject *source_object = source.objects().front();
    REQUIRE(source_object->layer_count() == 0);
    CHECK_FALSE(source_object->is_step_done(posSlice));
    REQUIRE(source_object->instances().size() == 1);
    const ModelInstance *source_instance = source_object->instances().front().model_instance;
    REQUIRE(source_instance != nullptr);
    const auto source_object_id = source_object->model_object()->id().id;
    const auto source_instance_id = source_instance->id().id;
    const auto source_loaded_id = source_instance->loaded_id;

    source.process(nullptr, false);
    REQUIRE(source.objects().front()->layer_count() == 6);
    const std::vector<CachedPathSnapshot> original = canonical_cached_paths(*source.objects().front());
    REQUIRE_FALSE(original.empty());
    REQUIRE(std::any_of(original.begin(), original.end(), [](const CachedPathSnapshot &path) {
        return std::isfinite(path.mm3_per_mm) && std::abs(path.mm3_per_mm - calibrated_flow) > 1e-6;
    }));

    ScopedTemporaryDir cache("orca-cache-process-select");
    REQUIRE(source.export_cached_data(cache.string(), true) == 0);
    size_t flow_edits = 0;
    rewrite_cache(cache.string(), [&](Json &document) {
        visit_cached_extrusions(document, [&](Json &extrusion) {
            REQUIRE(extrusion.at("mm3_per_mm").is_number());
            extrusion.at("mm3_per_mm") = calibrated_flow;
            ++flow_edits;
        });
    });
    REQUIRE(flow_edits > 0);

    Print fresh;
    fresh.apply(model, config);
    require_valid(fresh);
    fresh.set_status_silent();
    REQUIRE(fresh.objects().size() == 1);
    const PrintObject *fresh_object = fresh.objects().front();
    REQUIRE(fresh_object->layer_count() == 0);
    const PrintObjectStep cache_steps[] = {
        posSlice, posPerimeters, posPrepareInfill, posInfill, posIroning, posContouring,
        posSupportMaterial, posDetectOverhangsForLift, posSimplifyPath, posSimplifyInfill,
        posSimplifySupportPath,
    };
    for (const PrintObjectStep step : cache_steps)
        CHECK_FALSE(fresh_object->is_step_done(step));
    CHECK(fresh_object->model_object()->id().id == source_object_id);
    REQUIRE(fresh_object->instances().size() == 1);
    REQUIRE(fresh_object->instances().front().model_instance != nullptr);
    CHECK(fresh_object->instances().front().model_instance->id().id == source_instance_id);
    CHECK(fresh_object->instances().front().model_instance->loaded_id == source_loaded_id);

    REQUIRE(fresh.load_cached_data(cache.string()) == 0);
    REQUIRE(fresh_object->layer_count() == 6);
    const std::vector<CachedPathSnapshot> loaded = canonical_cached_paths(*fresh_object);
    REQUIRE(loaded.size() == original.size());
    for (const CachedPathSnapshot &path : loaded) {
        CHECK(std::isfinite(path.mm3_per_mm));
        CHECK_THAT(path.mm3_per_mm, Catch::Matchers::WithinAbs(calibrated_flow, 1e-12));
    }

    fresh.process(nullptr, use_cache);
    const std::vector<CachedPathSnapshot> processed = canonical_cached_paths(*fresh.objects().front());
    REQUIRE(processed.size() == original.size());
    for (size_t index = 0; index < original.size(); ++index) {
        INFO("path index=" << index << " geometry=" << original[index].geometry_key);
        CHECK(processed[index].geometry_key == original[index].geometry_key);
        CHECK(processed[index].role == original[index].role);
        CHECK_THAT(processed[index].width, Catch::Matchers::WithinAbs(original[index].width, 1e-6));
        CHECK_THAT(processed[index].height, Catch::Matchers::WithinAbs(original[index].height, 1e-6));
        const double expected_flow = use_cache ? calibrated_flow : original[index].mm3_per_mm;
        CHECK(std::isfinite(processed[index].mm3_per_mm));
        CHECK_THAT(processed[index].mm3_per_mm, Catch::Matchers::WithinAbs(expected_flow, use_cache ? 1e-12 : 1e-10));
    }
    for (const PrintObjectStep step : cache_steps)
        CHECK(fresh.objects().front()->is_step_done(step));
}

TEST_CASE("Cached non-planar arcs preserve Z and fitting data", "[Print][PrintCacheRoundtrip]")
{
    Print print;
    Model model;
    init_valid({cube(4.)}, print, model, pipeline_config());
    print.process();
    inject_arc(print);
    const auto before_paths = cached_paths(*print.objects().front());
    REQUIRE_FALSE(before_paths.empty());
    const ExtrusionPath before = before_paths.front();
    ScopedTemporaryDir cache("orca-cache-arc");
    REQUIRE(print.export_cached_data(cache.string(), true) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    const auto after = cached_paths(*print.objects().front());
    REQUIRE_FALSE(after.empty());
    CHECK(after.front().polyline.points == before.polyline.points);
    CHECK(after.front().z_contoured);
    const auto &fits = after.front().polyline.fitting_result;
    REQUIRE(fits.size() == 1);
    CHECK(fits.front().start_point_index == 0);
    CHECK(fits.front().end_point_index == 2);
    CHECK(fits.front().path_type == EMovePathType::Arc_move_ccw);
    CHECK(fits.front().arc_data.is_arc);
    CHECK(fits.front().arc_data.center == Point(0, 0));
    CHECK_THAT(fits.front().arc_data.radius, Catch::Matchers::WithinAbs(scaled(1.), 1e-6));
    CHECK_THAT(fits.front().arc_data.angle_radians, Catch::Matchers::WithinAbs(PI / 2., 1e-9));
}

TEST_CASE("Legacy cached polylines without Z restore planar coordinates and arcs", "[Print][PrintCacheRoundtrip]")
{
    Print print;
    Model model;
    init_valid({cube(4.)}, print, model, pipeline_config());
    print.process();
    inject_arc(print);
    ScopedTemporaryDir cache("orca-cache-legacy");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    rewrite_cache(cache.string(), [](Json &document) {
        visit_cached_extrusions(document, [](Json &extrusion) {
            REQUIRE(extrusion.at("polyline").is_object());
            extrusion.at("polyline").erase("z");
            extrusion.erase("z_contoured");
        });
    });
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    const auto paths = cached_paths(*print.objects().front());
    REQUIRE_FALSE(paths.empty());
    for (const auto &path : paths) {
        CHECK_FALSE(path.z_contoured);
        for (const auto &point : path.polyline.points)
            CHECK(point.z() == 0);
    }
    REQUIRE(paths.front().polyline.fitting_result.size() == 1);
    CHECK(paths.front().polyline.fitting_result.front().path_type == EMovePathType::Arc_move_ccw);
    CHECK(paths.front().polyline.points.front().x() == scaled(1.));
    CHECK(paths.front().polyline.points.back().y() == scaled(1.));
}

TEST_CASE("Cached Z vectors must match the number of polyline points", "[Print][PrintCacheRoundtrip]")
{
    const bool array = GENERATE(false, true);
    Print print;
    Model model;
    init_valid({cube(4.)}, print, model, pipeline_config());
    print.process();
    ScopedTemporaryDir cache("orca-cache-invalid-z");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    rewrite_cache(cache.string(), [array](Json &document) {
        visit_cached_extrusions(document, [array](Json &extrusion) {
            REQUIRE(extrusion.at("polyline").is_object());
            extrusion.at("polyline")["z"] = array ? Json::array() : Json("invalid");
        });
    });
    CHECK(print.load_cached_data(cache.string()) == CLI_IMPORT_CACHE_LOAD_FAILED);
}

TEST_CASE("Reloading cached first-layer groups replaces prior metadata", "[Print][PrintCacheRoundtrip]")
{
    Print print;
    Model model;
    init_valid({cube(4.)}, print, model, pipeline_config());
    print.process();
    ScopedTemporaryDir cache("orca-cache-first-layer-groups");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    const size_t expected_groups = print.get_object(0)->firstLayerObjGroups().size() + 1;
    print.get_object(0)->firstLayerObjGroupsMod().clear();
    rewrite_cache(cache.string(), [](Json &document) {
        REQUIRE(document.contains("first_layer_groups"));
        REQUIRE(document.at("first_layer_groups").is_array());
        Json group = Json::object();
        group["group_id"] = 991;
        group["volume_ids"] = Json::array({0});
        group["slices"] = Json::array();
        document["first_layer_groups"].push_back(std::move(group));
    });
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    REQUIRE(print.get_object(0)->firstLayerObjGroups().size() == expected_groups);
    const auto &last_group = print.get_object(0)->firstLayerObjGroups().back();
    CHECK(last_group.groupId == 991);
    REQUIRE(last_group.volume_ids.size() == 1);
    CHECK(last_group.volume_ids.front() == model.objects.front()->volumes.front()->id());
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    CHECK(print.get_object(0)->firstLayerObjGroups().size() == expected_groups);
}


TEST_CASE("Per-extruder printable regions expose shared and excluded bed geometry", "[Print][PrintSharedPrintableArea]")
{
    const auto rectangle = [](double min_x, double min_y, double max_x, double max_y) {
        return Vec2ds{ Vec2d(min_x, min_y), Vec2d(max_x, min_y),
                       Vec2d(max_x, max_y), Vec2d(min_x, max_y) };
    };
    const auto configure_areas = [&](size_t nozzle_count, const std::vector<Vec2ds> &areas) {
        DynamicPrintConfig config = multifilament_config(static_cast<unsigned int>(nozzle_count));
        config.set_deserialize_strict({
            { "use_relative_e_distances", true },
            { "layer_change_gcode", "G92 E0\n" },
        });
        if (nozzle_count > 1)
            config.set_key_value("nozzle_diameter", new ConfigOptionFloats({ 0.4, 0.4 }));
        config.set_key_value("printable_area", new ConfigOptionPoints({
            Vec2d(0., 0.), Vec2d(100., 0.), Vec2d(100., 100.), Vec2d(0., 100.)
        }));
        config.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups(areas));
        return config;
    };
    const auto require_bounds_and_area = [&](const Polygons &polygons,
                                             double min_x, double min_y, double max_x, double max_y,
                                             double expected_area) {
        REQUIRE_FALSE(polygons.empty());
        const BoundingBox bbox = get_extents(polygons);
        REQUIRE(bbox.defined);
        CHECK_THAT(unscaled<double>(bbox.min.x()), Catch::Matchers::WithinAbs(min_x, 1e-5));
        CHECK_THAT(unscaled<double>(bbox.min.y()), Catch::Matchers::WithinAbs(min_y, 1e-5));
        CHECK_THAT(unscaled<double>(bbox.max.x()), Catch::Matchers::WithinAbs(max_x, 1e-5));
        CHECK_THAT(unscaled<double>(bbox.max.y()), Catch::Matchers::WithinAbs(max_y, 1e-5));
        CHECK_THAT(polygon_area_mm2(polygons), Catch::Matchers::WithinAbs(expected_area, 1e-3));
    };

    SECTION("two nozzles intersect printable regions and expose their complements") {
        const std::vector<Vec2ds> areas = { rectangle(0., 0., 80., 100.), rectangle(20., 0., 100., 100.) };
        DynamicPrintConfig config = configure_areas(2, areas);
        Print print;
        Model model;
        init_valid({ cube(5.) }, print, model, config);

        const std::vector<Polygons> printable = print.get_extruder_printable_polygons();
        REQUIRE(printable.size() == 2);
        require_bounds_and_area(printable[0], 0., 0., 80., 100., 8000.);
        require_bounds_and_area(printable[1], 20., 0., 100., 100., 8000.);

        const std::vector<Polygons> unprintable = print.get_extruder_unprintable_polygons();
        REQUIRE(unprintable.size() == 2);
        require_bounds_and_area(unprintable[0], 80., 0., 100., 100., 2000.);
        require_bounds_and_area(unprintable[1], 0., 0., 20., 100., 2000.);

        require_bounds_and_area(print.get_extruder_shared_printable_polygon(), 20., 0., 80., 100., 6000.);
    }

    SECTION("single nozzle and degenerate areas fall back to the full printable bed") {
        {
            DynamicPrintConfig config = configure_areas(1, { rectangle(20., 0., 80., 100.) });
            Print print;
            Model model;
            init_valid({ cube(5.) }, print, model, config);
            require_bounds_and_area(print.get_extruder_shared_printable_polygon(), 0., 0., 100., 100., 10000.);
        }
        {
            DynamicPrintConfig config = configure_areas(2, { Vec2ds{ Vec2d(0., 0.), Vec2d(20., 0.) } });
            Print print;
            Model model;
            init_valid({ cube(5.) }, print, model, config);
            require_bounds_and_area(print.get_extruder_shared_printable_polygon(), 0., 0., 100., 100., 10000.);
        }
    }
}


namespace {

ExPolygon svg_rectangle_mm(double min_x, double min_y, double max_x, double max_y)
{
    return ExPolygon({
        Point::new_scale(min_x, min_y), Point::new_scale(max_x, min_y),
        Point::new_scale(max_x, max_y), Point::new_scale(min_x, max_y)
    });
}

ExPolygon svg_rectangle_with_hole_mm(double min_x, double min_y, double max_x, double max_y,
                                     double hole_min_x, double hole_min_y,
                                     double hole_max_x, double hole_max_y)
{
    ExPolygon shape = svg_rectangle_mm(min_x, min_y, max_x, max_y);
    Polygon hole;
    hole.points = { Point::new_scale(hole_min_x, hole_min_y), Point::new_scale(hole_min_x, hole_max_y),
                    Point::new_scale(hole_max_x, hole_max_y), Point::new_scale(hole_max_x, hole_min_y) };
    shape.holes.emplace_back(std::move(hole));
    return shape;
}

std::string svg_filled_path(const std::string &svg, const std::string &color)
{
    const std::string style = "style=\"fill: " + color + ";";
    const size_t style_pos = svg.find(style);
    if (style_pos == std::string::npos)
        return {};
    const size_t path_start = svg.rfind("<path d=\"", style_pos);
    const size_t path_end = svg.find("/>", style_pos);
    if (path_start == std::string::npos || path_end == std::string::npos)
        return {};
    return svg.substr(path_start, path_end + 2 - path_start);
}

struct SvgPathBounds
{
    bool defined = false;
    double min_x = 0.;
    double min_y = 0.;
    double max_x = 0.;
    double max_y = 0.;
};

SvgPathBounds svg_path_bounds(const std::string &path)
{
    SvgPathBounds bounds;
    const size_t d_start = path.find("d=\"");
    if (d_start == std::string::npos)
        return bounds;
    const size_t d_end = path.find('\"', d_start + 3);
    if (d_end == std::string::npos)
        return bounds;

    std::istringstream input(path.substr(d_start + 3, d_end - (d_start + 3)));
    std::string token;
    input >> token; // M
    std::vector<double> coordinates;
    while (input >> token) {
        char *end = nullptr;
        const double value = std::strtod(token.c_str(), &end);
        if (end != token.c_str())
            coordinates.emplace_back(value);
    }
    if (coordinates.size() < 6 || coordinates.size() % 2 != 0)
        return bounds;

    bounds.min_x = bounds.max_x = coordinates[0];
    bounds.min_y = bounds.max_y = coordinates[1];
    for (size_t i = 2; i < coordinates.size(); i += 2) {
        bounds.min_x = std::min(bounds.min_x, coordinates[i]);
        bounds.max_x = std::max(bounds.max_x, coordinates[i]);
        bounds.min_y = std::min(bounds.min_y, coordinates[i + 1]);
        bounds.max_y = std::max(bounds.max_y, coordinates[i + 1]);
    }
    bounds.defined = true;
    return bounds;
}

std::string read_svg_file(const ScopedTemporaryFile &file)
{
    std::ifstream input(file.string(), std::ios::binary);
    if (!input)
        return {};
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

} // namespace

TEST_CASE("Layer and region SVG exports reflect their public surface collections", "[Print][PrintPipelineWorkflows][LayerSvg]")
{
    Print print;
    Model model;
    init_valid({ cube(4.) }, print, model, pipeline_config());
    print.process();

    PrintObject *object = print.get_object(0);
    REQUIRE(object != nullptr);
    Layer *layer = object->get_layer(0);
    REQUIRE(layer != nullptr);
    REQUIRE(layer->region_count() == 1);
    LayerRegion *region = layer->get_region(0);

    // The two public collections deliberately occupy disjoint footprints and carry
    // different surface roles, so an exporter cannot pass by drawing the wrong data.
    const ExPolygon slice = svg_rectangle_mm(0., 0., 2., 2.);
    const ExPolygon fill = svg_rectangle_with_hole_mm(100., 0., 104., 4., 101., 1., 103., 3.);
    region->slices.set(ExPolygons{ slice }, stTop);
    region->fill_surfaces.set(ExPolygons{ fill }, stInternal);

    ScopedTemporaryFile slices_svg(".svg");
    layer->export_region_slices_to_svg(slices_svg.string().c_str());
    const std::string slices_text = read_svg_file(slices_svg);
    REQUIRE_FALSE(slices_text.empty());
    const std::string top_path = svg_filled_path(slices_text, "rgb(255,0,0)");
    REQUIRE_FALSE(top_path.empty());
    CHECK(top_path.find("fill-opacity=\"0.500000\"") != std::string::npos);
    const SvgPathBounds top_bounds = svg_path_bounds(top_path);
    REQUIRE(top_bounds.defined);
    CHECK_THAT(top_bounds.max_x - top_bounds.min_x, Catch::Matchers::WithinAbs(20., 1e-4));
    CHECK_THAT(top_bounds.max_y - top_bounds.min_y, Catch::Matchers::WithinAbs(20., 1e-4));

    const ExPolygon fill_boundary = svg_rectangle_mm(0.5, 0.5, 1.5, 1.5);
    region->fill_surfaces.set(ExPolygons{ fill_boundary }, stInternal);
    ScopedTemporaryFile region_slices_svg(".svg");
    region->export_region_slices_to_svg(region_slices_svg.string().c_str());
    const std::string region_slices_text = read_svg_file(region_slices_svg);
    REQUIRE_FALSE(region_slices_text.empty());
    const std::string region_top_path = svg_filled_path(region_slices_text, "rgb(255,0,0)");
    REQUIRE_FALSE(region_top_path.empty());
    const SvgPathBounds region_top_bounds = svg_path_bounds(region_top_path);
    REQUIRE(region_top_bounds.defined);
    CHECK_THAT(region_top_bounds.max_x - region_top_bounds.min_x, Catch::Matchers::WithinAbs(20., 1e-4));
    CHECK(region_slices_text.find("style=\"stroke: rgb(255,255,128);") != std::string::npos);

    // Move the fill surface to a remote, wider footprint before testing the Layer
    // fill exporter so collection routing and the SVG viewport origin are distinct.
    region->fill_surfaces.set(ExPolygons{ fill }, stInternal);
    ScopedTemporaryFile layer_fill_svg(".svg");
    layer->export_region_fill_surfaces_to_svg(layer_fill_svg.string().c_str());
    const std::string layer_fill_text = read_svg_file(layer_fill_svg);
    REQUIRE_FALSE(layer_fill_text.empty());
    const std::string layer_fill_path = svg_filled_path(layer_fill_text, "rgb(255,255,128)");
    // This is the defining collection-routing contract: Layer fill export must use fill_surfaces,
    // not the disjoint top slice collection.
    CHECK_FALSE(layer_fill_path.empty());
    if (!layer_fill_path.empty()) {
        CHECK(layer_fill_path.find("fill-opacity=\"0.500000\"") != std::string::npos);
        const SvgPathBounds fill_bounds = svg_path_bounds(layer_fill_path);
        REQUIRE(fill_bounds.defined);
        CHECK_THAT(fill_bounds.max_x - fill_bounds.min_x, Catch::Matchers::WithinAbs(40., 1e-4));
        CHECK_THAT(fill_bounds.max_y - fill_bounds.min_y, Catch::Matchers::WithinAbs(40., 1e-4));
        const size_t svg_width_pos = layer_fill_text.find("width=\"");
        REQUIRE(svg_width_pos != std::string::npos);
        char *svg_width_end = nullptr;
        const double svg_width = std::strtod(layer_fill_text.c_str() + svg_width_pos + 7, &svg_width_end);
        REQUIRE(svg_width_end != layer_fill_text.c_str() + svg_width_pos + 7);
        CHECK(fill_bounds.min_x >= 0.);
        CHECK(fill_bounds.max_x <= svg_width); // the fill must stay inside its SVG viewport
        CHECK(layer_fill_path.find("M ") != layer_fill_path.rfind("M ")); // outer contour and hole
    }
    CHECK(svg_filled_path(layer_fill_text, "rgb(255,0,0)").empty());

    ScopedTemporaryFile region_fill_svg(".svg");
    region->export_region_fill_surfaces_to_svg(region_fill_svg.string().c_str());
    const std::string region_fill_text = read_svg_file(region_fill_svg);
    REQUIRE_FALSE(region_fill_text.empty());
    const std::string region_fill_path = svg_filled_path(region_fill_text, "rgb(255,255,128)");
    REQUIRE_FALSE(region_fill_path.empty());
    const SvgPathBounds region_bounds = svg_path_bounds(region_fill_path);
    REQUIRE(region_bounds.defined);
    CHECK_THAT(region_bounds.max_x - region_bounds.min_x, Catch::Matchers::WithinAbs(40., 1e-4));
    CHECK_THAT(region_bounds.max_y - region_bounds.min_y, Catch::Matchers::WithinAbs(40., 1e-4));
    CHECK(region_fill_text.find("style=\"fill: none; stroke: black;") != std::string::npos);
    CHECK(region_fill_text.find("style=\"fill: none; stroke: blue;") != std::string::npos);
}

} // namespace print_workflow_contracts

namespace support_cache_contracts {
using Json = nlohmann::json;

namespace {
struct SupportState {
    size_t id = 0;
    size_t interface_id = 0;
    double z = 0.;
    double height = 0.;
    double island_area = 0.;
    ExPolygons islands;
    int type = 0;
    bool no_sort = false;
    std::vector<ExtrusionPath> paths;
};

std::vector<SupportState> capture_support(const PrintObject &object)
{
    std::vector<SupportState> result;
    for (const SupportLayer *layer : object.support_layers()) {
        SupportState state;
        state.id = layer->id();
        state.interface_id = layer->interface_id();
        state.z = layer->print_z;
        state.height = layer->height;
        state.type = int(layer->support_type);
        state.islands = layer->support_islands;
        state.no_sort = layer->support_fills.no_sort;
        for (const ExPolygon &island : layer->support_islands)
            state.island_area += island.area() * SCALING_FACTOR * SCALING_FACTOR;
        for_each_extrusion_path(layer->support_fills,
                                [&state](const ExtrusionPath &path) { state.paths.push_back(path); });
        result.push_back(std::move(state));
    }
    return result;
}

DynamicPrintConfig support_config(const char *support_type, const char *support_style = "default", int raft_layers = 0)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "wall_loops", 2 },
        { "top_shell_layers", 2 },
        { "bottom_shell_layers", 2 },
        { "skirt_loops", 0 },
        { "brim_type", "no_brim" },
        { "raft_layers", raft_layers },
        { "sparse_infill_density", "15%" },
        { "machine_start_gcode", "" },
        { "machine_end_gcode", "" },
        { "layer_change_gcode", "G92 E0\n" },
        { "before_layer_change_gcode", "" },
        { "enable_support", true },
        { "support_type", support_type },
        { "support_style", support_style },
        { "support_on_build_plate_only", false },
        { "support_interface_layers", 2 },
    });
    return config;
}

void process_support(Print &print, Model &model, const TriangleMesh &mesh, const DynamicPrintConfig &config)
{
    init_print({ mesh }, print, model, config);
    const StringObjectException validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
}

void compare_support(const std::vector<SupportState> &before, const std::vector<SupportState> &after)
{
    REQUIRE(after.size() == before.size());
    for (size_t i = 0; i < before.size(); ++i) {
        INFO("support layer=" << i);
        CHECK(after[i].id == before[i].id);
        CHECK(after[i].interface_id == before[i].interface_id);
        CHECK(after[i].type == before[i].type);
        CHECK(after[i].no_sort == before[i].no_sort);
        CHECK_THAT(after[i].z, Catch::Matchers::WithinAbs(before[i].z, 1e-6));
        CHECK_THAT(after[i].height, Catch::Matchers::WithinAbs(before[i].height, 1e-6));
        CHECK_THAT(after[i].island_area, Catch::Matchers::WithinAbs(before[i].island_area, 1e-6));
        REQUIRE(after[i].islands.size() == before[i].islands.size());
        for (size_t island = 0; island < before[i].islands.size(); ++island) {
            CHECK(after[i].islands[island].contour.points == before[i].islands[island].contour.points);
            REQUIRE(after[i].islands[island].holes.size() == before[i].islands[island].holes.size());
            for (size_t hole = 0; hole < before[i].islands[island].holes.size(); ++hole)
                CHECK(after[i].islands[island].holes[hole].points == before[i].islands[island].holes[hole].points);
        }
        REQUIRE(after[i].paths.size() == before[i].paths.size());
        for (size_t j = 0; j < before[i].paths.size(); ++j) {
            const ExtrusionPath &old_path = before[i].paths[j];
            const ExtrusionPath &new_path = after[i].paths[j];
            CHECK(new_path.role() == old_path.role());
            CHECK(new_path.polyline.points == old_path.polyline.points);
            CHECK_THAT(new_path.mm3_per_mm, Catch::Matchers::WithinAbs(old_path.mm3_per_mm, 1e-12));
            CHECK_THAT(new_path.width, Catch::Matchers::WithinAbs(old_path.width, 1e-6));
            CHECK_THAT(new_path.height, Catch::Matchers::WithinAbs(old_path.height, 1e-6));
            CHECK(new_path.z_contoured == old_path.z_contoured);
        }
    }
}

void compare_links(const PrintObject &object, size_t count)
{
    REQUIRE(object.support_layers().size() == count);
    for (size_t i = 0; i < count; ++i) {
        const SupportLayer *layer = object.support_layers()[i];
        const SupportLayer *lower = i == 0 ? nullptr : object.support_layers()[i - 1];
        const SupportLayer *upper = i + 1 == count ? nullptr : object.support_layers()[i + 1];
        CHECK(layer->lower_layer == lower);
        CHECK(layer->upper_layer == upper);
    }
}

void rewrite_cache(const std::string &directory, const std::function<void(Json &)> &edit)
{
    size_t edited = 0;
    for (const auto &entry : boost::filesystem::directory_iterator(directory)) {
        if (!boost::filesystem::is_regular_file(entry.path()))
            continue;
        std::ifstream input(entry.path().string());
        Json document = Json::parse(input);
        input.close();
        edit(document);
        std::ofstream output(entry.path().string());
        output << document.dump();
        REQUIRE(output.good());
        ++edited;
    }
    REQUIRE(edited == 1);
}

size_t support_path_count(const std::vector<SupportState> &states)
{
    size_t count = 0;
    for (const SupportState &state : states)
        count += state.paths.size();
    return count;
}

void expect_roundtrip(const TriangleMesh &mesh, const DynamicPrintConfig &config, bool pretty)
{
    Print print;
    Model model;
    process_support(print, model, mesh, config);
    const auto before = capture_support(*print.objects().front());
    REQUIRE_FALSE(before.empty());
    REQUIRE(support_path_count(before) > 0);
    const size_t before_count = before.size();
    ScopedTemporaryDir cache("orca-support-cache-contract");
    REQUIRE(print.export_cached_data(cache.string(), pretty) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    const auto after = capture_support(*print.objects().front());
    compare_support(before, after);
    compare_links(*print.objects().front(), before_count);
}
}

TEST_CASE("Raft support cache preserves linked layers and compact JSON", "[Print][SupportCache]")
{
    expect_roundtrip(cube(6.), support_config("normal(auto)", "default", 3), false);
}

TEST_CASE("Pretty raft support cache preserves the same geometry", "[Print][SupportCache]")
{
    expect_roundtrip(cube(6.), support_config("normal(auto)", "default", 3), true);
}

TEST_CASE("Normal support cache preserves islands and interface paths", "[Print][SupportCache]")
{
    expect_roundtrip(Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"), false);
}

TEST_CASE("Normal snug support cache preserves nested fill paths", "[Print][SupportCache]")
{
    expect_roundtrip(Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)", "snug"), false);
}

TEST_CASE("Organic tree support cache preserves tree layer metadata", "[Print][SupportCache]")
{
    expect_roundtrip(Slic3r::Test::mesh(TestMesh::overhang), support_config("tree(auto)", "organic"), false);
}

TEST_CASE("Slim tree support cache preserves tree geometry", "[Print][SupportCache]")
{
    expect_roundtrip(Slic3r::Test::mesh(TestMesh::overhang), support_config("tree(auto)", "tree_slim"), false);
}

TEST_CASE("Strong tree support cache preserves branch extrusion paths", "[Print][SupportCache]")
{
    expect_roundtrip(Slic3r::Test::mesh(TestMesh::overhang), support_config("tree(auto)", "tree_strong"), false);
}

TEST_CASE("Support no-sort metadata survives cache loading", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    auto before = capture_support(*print.objects().front());
    REQUIRE_FALSE(before.empty());
    for (SupportLayer *layer : print.objects().front()->support_layers())
        layer->support_fills.no_sort = true;
    before = capture_support(*print.objects().front());
    ScopedTemporaryDir cache("orca-support-cache-nosort");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    const auto after = capture_support(*print.objects().front());
    compare_support(before, after);
    for (const SupportState &state : after)
        CHECK(state.no_sort);
}

TEST_CASE("Repeated support cache loads replace support layers and links", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    const auto before = capture_support(*print.objects().front());
    REQUIRE_FALSE(before.empty());
    ScopedTemporaryDir cache("orca-support-cache-repeat");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    REQUIRE(print.load_cached_data(cache.string()) == 0);
    const auto after = capture_support(*print.objects().front());
    compare_support(before, after);
    compare_links(*print.objects().front(), before.size());
}

TEST_CASE("Missing support cache directory leaves the processed support state intact", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    const auto before = capture_support(*print.objects().front());
    REQUIRE_FALSE(before.empty());
    ScopedTemporaryDir cache("orca-support-cache-missing");
    CHECK(print.load_cached_data(cache.string() + "/missing") == CLI_IMPORT_CACHE_NOT_FOUND);
    const auto after = capture_support(*print.objects().front());
    compare_support(before, after);
}

TEST_CASE("Corrupt support type metadata is rejected", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    ScopedTemporaryDir cache("orca-support-cache-bad-type");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    rewrite_cache(cache.string(), [](Json &document) {
        REQUIRE_FALSE(document.at("support_layers").empty());
        document.at("support_layers").at(0)["support_type"] = "invalid-support-type";
    });
    CHECK(print.load_cached_data(cache.string()) == CLI_IMPORT_CACHE_LOAD_FAILED);
}

TEST_CASE("Corrupt support extrusion payload is rejected", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    ScopedTemporaryDir cache("orca-support-cache-bad-fill");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    rewrite_cache(cache.string(), [](Json &document) {
        REQUIRE_FALSE(document.at("support_layers").empty());
        auto &entities = document.at("support_layers").at(0).at("support_fills").at("entities");
        REQUIRE_FALSE(entities.empty());
        entities.at(0)["entity_type"] = "not-an-extrusion-entity";
    });
    CHECK(print.load_cached_data(cache.string()) == CLI_IMPORT_CACHE_LOAD_FAILED);
}

TEST_CASE("Corrupt support island payload is rejected", "[Print][SupportCache]")
{
    Print print;
    Model model;
    process_support(print, model, Slic3r::Test::mesh(TestMesh::overhang), support_config("normal(auto)"));
    ScopedTemporaryDir cache("orca-support-cache-bad-island");
    REQUIRE(print.export_cached_data(cache.string(), false) == 0);
    rewrite_cache(cache.string(), [](Json &document) {
        REQUIRE_FALSE(document.at("support_layers").empty());
        document.at("support_layers").at(0).at("support_islands") = 17;
    });
    CHECK(print.load_cached_data(cache.string()) == CLI_IMPORT_CACHE_LOAD_FAILED);
}

} // namespace support_cache_contracts

// A part with an overhang on its LEADING side (the end that prints first) needs
// supports below the object's own lowest slicing layer: the belt under that overhang
// is reached before the object's first contact with it, so the support layers sit at
// a lower slicing Z than any object layer. A generator that stops at the object's
// first layer, or at global Z = 0, leaves those supports floating above the belt.
TEST_CASE("Compacted prime tower clearance aggregates pre-slice offenders", "[Print][WipeTower][CompactedTower]")
{
    const int rod_band_offenders = GENERATE(0, 1, 2);
    const bool check_enabled     = GENERATE(false, true);
    INFO("rod-band offenders: " << rod_band_offenders);
    INFO("no sparse clearance check: " << check_enabled);

    DynamicPrintConfig config = multifilament_config(2, {
        { "enable_prime_tower",                 true  },
        { "wipe_tower_no_sparse_layers",        check_enabled },
        { "wipe_tower_sparse_layers_combination", false },
        { "timelapse_type",                     0     },
        { "enable_wrapping_detection",           false },
        { "belt_printer",                       false },
        { "spiral_mode",                        false },
        { "z_hop",                              0.0   },
        { "print_sequence",                     "by layer" },
        { "printable_area",                     "0x0,500x0,500x500,0x500" },
        { "wipe_tower_wall_type",               "rectangle" },
        { "prime_tower_width",                  35.0  },
        { "prime_volume",                       45.0  },
        { "prime_tower_brim_width",             0.0   },
        { "wipe_tower_x",                       0.0   },
        { "wipe_tower_y",                       0.0   },
        { "wipe_tower_rotation_angle",          0.0   },
        { "extruder_clearance_dist_to_rod",     40.0  },
        { "extruder_clearance_height_to_rod",   25.0  },
        { "extruder_clearance_height_to_lid",   120.0 },
        { "extruder_clearance_radius",          40.0  },
        { "nozzle_height",                      5.0   },
        { "layer_height",                       0.2   },
        { "initial_layer_print_height",         0.2   },
    });

    Model model;
    ObjectID first_object_id;
    // The 30 mm parts exceed the 25 mm rod limit but fit below the 120 mm lid.
    // Their X positions clear the 35 mm tower and 40 mm head radius; only Y
    // determines whether each part enters the rod's band across the bed.
    for (int i = 0; i < 2; ++i) {
        ModelObject *object = model.add_object();
        if (i == 0)
            first_object_id = object->id();
        object->add_volume(make_cube(10.0, 10.0, 30.0));
        object->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(i + 1));
        ModelInstance *instance = object->add_instance();
        const bool in_rod_band = i < rod_band_offenders;
        instance->set_offset(Vec3d(200.0 + i * 100.0, in_rod_band ? 0.0 : 300.0, 0.0));
        object->ensure_on_bed();
    }

    Print print;
    print.apply(model, config);
    REQUIRE(print.objects().size() == 2);
    REQUIRE(print.extruders().size() == 2);
    REQUIRE(print.has_wipe_tower());
    REQUIRE(print.wipe_tower_data(2).width > 0.0f);
    REQUIRE(print.wipe_tower_data(2).depth > 0.0f);
    const ModelInstance *first_instance = nullptr;
    for (const PrintObject *object : print.objects()) {
        REQUIRE(unscaled<double>(object->max_z()) > 25.0);
        REQUIRE(unscaled<double>(object->max_z()) < 120.0);
        REQUIRE(object->model_object()->instances.size() == 1);
        if (object->model_object()->id() == first_object_id)
            first_instance = object->model_object()->instances.front();
    }
    REQUIRE(first_instance != nullptr);

    Polygons collision_polygons;
    std::vector<std::pair<Polygon, float>> height_polygons;
    const StringObjectException error = Print::compacted_wipe_tower_clearance_valid(
        print, &collision_polygons, &height_polygons);
    const int expected_offenders = check_enabled ? rod_band_offenders : 0;

    if (expected_offenders == 0) {
        CHECK(error.string.empty());
        CHECK(error.object == nullptr);
        CHECK(collision_polygons.empty());
        CHECK(height_polygons.empty());
        return;
    }

    REQUIRE_FALSE(error.string.empty());
    REQUIRE(height_polygons.size() == size_t(expected_offenders));
    CHECK(collision_polygons.size() > height_polygons.size());
    if (expected_offenders == 1)
        CHECK(error.object == first_instance);
    else
        CHECK(error.object == nullptr);

    for (const auto &entry : height_polygons) {
        CHECK_THAT(entry.second, Catch::Matchers::WithinAbs(25.0, 1e-6));
        REQUIRE(entry.first.points.size() >= 3);
        CHECK(std::abs(entry.first.area()) > 0);
    }
    for (const Polygon &polygon : collision_polygons) {
        REQUIRE(polygon.points.size() >= 3);
        CHECK(std::abs(polygon.area()) > 0);
    }
}

TEST_CASE("Belt supports reach the belt under a leading overhang", "[Print][belt][Support][Regression]")
{
    // default resolves to organic for tree support; tree_hybrid is the classic tree.
    const char *support_type  = GENERATE("normal(auto)", "tree(auto)");
    const char *support_style = GENERATE("default", "organic", "tree_hybrid");
    if (std::string(support_type) == "normal(auto)" && std::string(support_style) != "default")
        return;   // organic and tree_hybrid are tree styles
    DYNAMIC_SECTION(support_type << " / " << support_style) {
        // A 20 mm cube with a 2 mm thick fin that leaves its top edge and reaches
        // 20 mm toward -Y, the end of the part that prints first, climbing at 45 deg
        // as it goes (from z = 18 at the cube to z = 38 at the tip).  With the layers
        // leaning toward -Y at 45 deg the fin's underside is parallel to the layers:
        // a ceiling 20 x 28 mm in one layer, with nothing but air between it and the
        // belt, which lies up to 41 mm (of slicing Z) below the object's own lowest
        // point.  Support has to span all of it.
        indexed_triangle_set its = its_make_cube(20., 20., 20.);
        indexed_triangle_set fin = its_make_cube(20., 20., 2.);
        Transform3d shear = Transform3d::Identity();
        shear.matrix() << 1., 0., 0.,   0.,
                          0., 1., 0., -20.,
                          0., -1., 1., 38.,
                          0., 0., 0.,   1.;
        its_transform(fin, shear);
        its_merge(its, fin);
        TriangleMesh mesh(std::move(its));

        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "belt_printer",               1 },
            { "belt_slice_rotation",        "x" },
            { "belt_slice_rotation_angle",  45 },
            { "gcode_remap_x",              "rev_x" },
            { "gcode_remap_y",              "pos_z" },
            { "gcode_remap_z",              "pos_y" },
            { "layer_height",               0.2 },
            { "initial_layer_print_height", 0.2 },
            { "skirt_loops",                0 },
            { "z_hop",                      0 },
            { "enable_support",             1 },
            { "support_type",               support_type },
            { "support_style",              support_style },
            { "support_threshold_angle",    30 },
            { "machine_start_gcode",        "T[initial_tool]\n" },
            { "layer_change_gcode",         "G92 E0\n" },
        });
        Print print;
        Model model;
        init_print({ mesh }, print, model, config);
        // On the bed, not at its corner: organic tree support clips its branches to
        // the bed outline, and the fixture leaves the object at the origin.
        model.objects.front()->instances.front()->set_offset(Vec3d(100., 100., 0.));
        print.apply(model, config);
        print.set_status_silent();
        print.process();

        const PrintObject &object = *print.objects().front();
        REQUIRE(! object.layers().empty());
        // The whole part is sliced: the layers lean at 45 deg, so the part spans
        // (y + z) / sqrt(2) of slicing Z, and every layer in that span has geometry.
        {
            double lo = std::numeric_limits<double>::max(), hi = std::numeric_limits<double>::lowest();
            for (const stl_vertex &v : mesh.its.vertices) {
                lo = std::min<double>(lo, v.y() + v.z());
                hi = std::max<double>(hi, v.y() + v.z());
            }
            const double span = (hi - lo) / std::sqrt(2.);
            size_t nonempty = 0;
            for (const Layer *layer : object.layers())
                if (! layer->lslices.empty())
                    ++ nonempty;
            INFO("non-empty object layers " << nonempty << ", slicing span " << span << " mm");
            CHECK(double(nonempty) * 0.2 > span - 0.6);
        }
        BeltFloorContext floor;
        REQUIRE(floor.init(object.slicing_parameters(), print.config()));

        // The lowest support layer that prints anything, and the belt floor beneath it.
        const SupportLayer *lowest = nullptr;
        for (const SupportLayer *layer : object.support_layers())
            if (! layer->support_fills.empty() && (lowest == nullptr || layer->print_z < lowest->print_z))
                lowest = layer;
        REQUIRE(lowest != nullptr);
        double floor_under_lowest = std::numeric_limits<double>::max();
        for (const ExtrusionEntity *entity : lowest->support_fills.flatten().entities)
            for (const Slic3r::Polyline &pl : entity->as_polylines())
                for (const Point &pt : pl.points)
                    floor_under_lowest = std::min(floor_under_lowest, floor.floor_print_z(pt));
        // The object's lowest geometry.  The slicing frame starts at the lowest
        // belt-floor point under the footprint, so the layers below the leading
        // tip of the overhang are empty.
        double first_object_z = std::numeric_limits<double>::max();
        for (const Layer *layer : object.layers())
            if (! layer->lslices.empty()) { first_object_z = layer->print_z; break; }
        REQUIRE(first_object_z < std::numeric_limits<double>::max());
        INFO("lowest support z " << lowest->print_z << ", floor under it " << floor_under_lowest
             << ", first object layer " << first_object_z);
        // Well below the object's own lowest layer (the belt under the tip of the fin
        // is ~41 mm of slicing Z below the cube's leading edge, which rests on it)...
        CHECK(lowest->print_z < first_object_z - 5.);
        // ...and resting on the belt: within a few layers of the floor beneath its own lines.
        CHECK(lowest->print_z - floor_under_lowest < 4. * 0.2 + EPSILON);
        CHECK(lowest->print_z - floor_under_lowest > -0.2 - EPSILON);
    }
}

TEST_CASE("Print compares filament temperatures with explicit and material ranges",
          "[Print][PrintMultiFilamentValidation]")
{
    struct CompatibilityCase {
        std::vector<std::string> filament_types;
        std::vector<int>         temperatures;
        std::vector<int>         range_lows;
        std::vector<int>         range_highs;
        FilamentCompatibilityType expected;
    };
    const auto part = GENERATE(from_range(std::vector<CompatibilityCase>{
        { { "PLA", "PETG" }, { 225, 250 }, { 190, 220 }, { 240, 260 }, FilamentCompatibilityType::HighLowMixed },
        { { "PLA", "PETG" }, { 225, 250 }, { 190, 220 }, {   0, 260 }, FilamentCompatibilityType::HighLowMixed },
        { { "PLA", "PETG" }, { 225, 225 }, {   0, 220 }, { 240, 260 }, FilamentCompatibilityType::Compatible },
        { { "PLA", "PETG" }, { 225, 250 }, { 240, 220 }, { 240, 260 }, FilamentCompatibilityType::InvalidTemperatureRange },
    }));

    CHECK(Print::check_multi_filaments_compatibility(
              part.filament_types, part.temperatures, part.range_lows, part.range_highs) == part.expected);
}

TEST_CASE("Print validation includes support filament temperature compatibility",
          "[Print][PrintMultiFilamentValidation]")
{
    struct SupportCase {
        int  support_filament;
        bool allow_mixed_temperatures;
        bool warning;
    };
    const auto part = GENERATE(
        SupportCase{ 1, false, false },
        SupportCase{ 2, false, false },
        SupportCase{ 2, true,  true });
    CAPTURE(part.support_filament, part.allow_mixed_temperatures);

    DynamicPrintConfig config = multifilament_config(2, {
        { "enable_support",                 true },
        { "support_type",                   "normal(auto)" },
        { "support_style",                  "default" },
        { "support_on_build_plate_only",    false },
        { "print_sequence",                 "by object" },
        { "layer_height",                   0.2 },
        { "initial_layer_print_height",     0.2 },
        { "skirt_loops",                    0 },
        { "brim_type",                      "no_brim" },
        { "nozzle_temperature",             "225,250" },
        { "nozzle_temperature_range_low",   "190,220" },
        { "nozzle_temperature_range_high",  "240,260" },
        { "timelapse_type",                 "0" },
        { "enable_wrapping_detection",      false },
        { "use_relative_e_distances",       true },
        { "layer_change_gcode",             "G92 E0\n" },
    });
    config.set_key_value("filament_type", new ConfigOptionStrings({ "PLA", "PETG" }));

    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{{
        { "extruder",                  1 },
        { "support_filament",          part.support_filament },
        { "support_interface_filament", 1 },
    }};
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{ mesh(TestMesh::overhang) }, print, model, config, &overrides);

    // These checks prove the public configuration reached the validation path: one nozzle,
    // by-object sequencing, support enabled, and a support-bearing object using the requested
    // support extruder.  A set_deserialize_strict call alone is not this proof.
    REQUIRE(print.config().nozzle_diameter.size() == 1);
    REQUIRE(print.config().print_sequence == PrintSequence::ByObject);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->config().enable_support);
    REQUIRE(print.objects().front()->has_support_material());
    REQUIRE(print.objects().front()->config().support_filament == part.support_filament);
    REQUIRE(print.objects().front()->config().support_interface_filament == 1);
    REQUIRE(print.object_extruders().size() == 1);
    CHECK(print.object_extruders().front() == 0);
    REQUIRE(print.extruders().size() == (part.support_filament == 1 ? 1u : 2u));

    print.set_check_multi_filaments_compatibility(!part.allow_mixed_temperatures);
    std::vector<StringObjectException> warnings;
    const StringObjectException validation = print.validate(&warnings);
    CAPTURE(validation.string, validation.is_warning);
    if (part.support_filament == 1) {
        CHECK(validation.string.empty());
        CHECK_FALSE(validation.is_warning);
        CHECK(std::none_of(warnings.begin(), warnings.end(), [](const StringObjectException &entry) {
            return entry.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP;
        }));
        return;
    }

    if (part.allow_mixed_temperatures) {
        CHECK(validation.string.empty());
        const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const StringObjectException &entry) {
            return entry.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP;
        });
        REQUIRE(warning != warnings.end());
        REQUIRE(warning->string.rfind("Selected nozzle temperatures are incompatible.", 0) == 0);
        CHECK(warning->is_warning == part.warning);
    } else {
        REQUIRE_FALSE(validation.string.empty());
        REQUIRE(validation.string.rfind("Selected nozzle temperatures are incompatible.", 0) == 0);
        CHECK(validation.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP);
        CHECK_FALSE(validation.is_warning);
    }
}

TEST_CASE("Print validation enforces mixed filament temperature policy by sequence",
          "[Print][PrintMultiFilamentValidation]")
{
    struct SequenceCase {
        const char *range_lows;
        const char *range_highs;
        const char *temperatures;
        bool        allow_mixed_temperatures;
        bool        invalid_range;
    };
    const auto part = GENERATE(
        SequenceCase{ "190,220", "240,260", "225,250", false, false },
        SequenceCase{ "190,220", "240,260", "225,250", true,  false },
        SequenceCase{ "240,220", "230,260", "225,250", false, true  });
    CAPTURE(part.allow_mixed_temperatures, part.invalid_range);

    DynamicPrintConfig config = multifilament_config(2, {
        { "print_sequence",                "by layer" },
        { "enable_support",                false },
        { "layer_height",                  0.2 },
        { "initial_layer_print_height",    0.2 },
        { "skirt_loops",                   0 },
        { "brim_type",                     "no_brim" },
        { "nozzle_temperature",            part.temperatures },
        { "nozzle_temperature_range_low",  part.range_lows },
        { "nozzle_temperature_range_high", part.range_highs },
        { "use_relative_e_distances",      true },
        { "layer_change_gcode",            "G92 E0\n" },
    });
    config.set_key_value("filament_type", new ConfigOptionStrings({ "PLA", "PETG" }));
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        { { "extruder", 1 } }, { { "extruder", 2 } },
    };

    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{ cube(8.0), cube(8.0) }, print, model, config, &overrides);
    REQUIRE(print.config().nozzle_diameter.size() == 1);
    REQUIRE(print.config().print_sequence == PrintSequence::ByLayer);
    REQUIRE(print.objects().size() == 2);
    for (const PrintObject *object : print.objects()) {
        REQUIRE(object != nullptr);
        REQUIRE_FALSE(object->config().enable_support);
    }
    REQUIRE(print.object_extruders().size() == 2);
    REQUIRE(print.extruders().size() == 2);

    print.set_check_multi_filaments_compatibility(!part.allow_mixed_temperatures);
    std::vector<StringObjectException> warnings;
    const StringObjectException validation = print.validate(&warnings);
    CAPTURE(validation.string, validation.is_warning);
    if (part.invalid_range) {
        REQUIRE_FALSE(validation.string.empty());
        REQUIRE(validation.string.rfind("Invalid recommended nozzle temperature range.", 0) == 0);
        CHECK(validation.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP);
        CHECK_FALSE(validation.is_warning);
        return;
    }

    if (part.allow_mixed_temperatures) {
        CHECK(validation.string.empty());
        const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const StringObjectException &entry) {
            return entry.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP;
        });
        REQUIRE(warning != warnings.end());
        REQUIRE(warning->string.rfind("Selected nozzle temperatures are incompatible.", 0) == 0);
        CHECK(warning->is_warning);
    } else {
        REQUIRE_FALSE(validation.string.empty());
        REQUIRE(validation.string.rfind("Selected nozzle temperatures are incompatible.", 0) == 0);
        CHECK(validation.type == STRING_EXCEPT_FILAMENTS_DIFFERENT_TEMP);
        CHECK_FALSE(validation.is_warning);
    }
}

TEST_CASE("Failed G-code exports report their outcome and remain retryable",
          "[Print][LifecycleEvents][GCodeExportFailures]")
{
    enum class Failure { TemporaryPathIsDirectory, ThumbnailCallback, DestinationCollision };
    const Failure failure = GENERATE(Failure::TemporaryPathIsDirectory,
                                     Failure::ThumbnailCallback,
                                     Failure::DestinationCollision);
    CAPTURE(static_cast<int>(failure));

    ScopedTemporaryDir directory("orca-export-failure");
    const boost::filesystem::path output = directory.path() / "output.gcode";
    const boost::filesystem::path temporary(output.string() + ".tmp");
    const boost::filesystem::path sentinel = output / "existing.txt";

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"enable_support", false},
        {"enable_prime_tower", false},
        {"thumbnails", "32x32/PNG"},
    });
    Print print;
    Model model;
    ModelInfo info;
    info.model_name = "Export failure fixture";
    model.model_info = std::make_shared<ModelInfo>(std::move(info));
    init_print({TriangleMesh(make_cube(5., 5., 0.4))}, print, model, config);
    REQUIRE_FALSE(print.is_BBL_printer());
    REQUIRE(print.validate().string.empty());
    print.process();

    struct Event {
        LifecycleEvent kind;
        LifecycleEvtCode code;
        std::string id;
        std::string name;
        std::string message;
    };
    std::vector<Event> events;
    bool inject_failure = true;
    ScopedLifecycleHook hook([&](LifecycleEvent event, const LifecycleEventContext &context) {
        if (event != LifecycleEvent::GCodeExportStarted && event != LifecycleEvent::GCodeExportFinished)
            return;
        events.push_back({event, context.code, context.id, context.name, context.msg});
        // output_filepath() has already resolved a file name. Simulate a destination
        // appearing while the exporter works, without relying on a racing thread.
        if (event == LifecycleEvent::GCodeExportStarted && inject_failure &&
            failure == Failure::DestinationCollision) {
            boost::filesystem::create_directory(output);
            std::ofstream stream(sentinel.string());
            stream << "preserve existing destination\n";
        }
    });
    size_t thumbnail_calls = 0;
    const ThumbnailsGeneratorCallback thumbnails = [&](const ThumbnailsParams &) -> ThumbnailsList {
        ++thumbnail_calls;
        if (inject_failure && failure == Failure::ThumbnailCallback)
            throw std::runtime_error("thumbnail callback failed");
        return {};
    };
    if (failure == Failure::TemporaryPathIsDirectory)
        REQUIRE(boost::filesystem::create_directory(temporary));

    REQUIRE_THROWS_AS(print.export_gcode(output.string(), nullptr, thumbnails), std::runtime_error);
    REQUIRE(events.size() == 2);
    CHECK(events[0].kind == LifecycleEvent::GCodeExportStarted);
    CHECK(events[0].code == LifecycleEvtCode::Ok);
    CHECK(events[1].kind == LifecycleEvent::GCodeExportFinished);
    CHECK(events[1].code == LifecycleEvtCode::Error);
    CHECK_FALSE(print.is_step_done(psGCodeExport));
    for (const Event &event : events) {
        CHECK(event.id == std::to_string(print.model().id().id));
        CHECK(event.name == "Export failure fixture");
        CHECK(event.message.find(output.string()) != std::string::npos);
    }

    if (failure == Failure::ThumbnailCallback) {
        REQUIRE(thumbnail_calls == 1);
        CHECK_FALSE(boost::filesystem::exists(temporary));
        CHECK_FALSE(boost::filesystem::exists(output));
    } else if (failure == Failure::DestinationCollision) {
        REQUIRE(thumbnail_calls == 1);
        REQUIRE(boost::filesystem::is_regular_file(temporary));
        REQUIRE(boost::filesystem::is_directory(output));
        std::ifstream stream(sentinel.string());
        std::string preserved;
        std::getline(stream, preserved);
        CHECK(preserved == "preserve existing destination");
        boost::filesystem::remove_all(output);
    } else {
        CHECK(thumbnail_calls == 0);
        REQUIRE(boost::filesystem::is_directory(temporary));
        boost::filesystem::remove_all(temporary);
    }

    // A handled export error must not mark the print complete or poison the next export.
    inject_failure = false;
    REQUIRE_NOTHROW(print.export_gcode(output.string(), nullptr, thumbnails));
    REQUIRE(events.size() == 4);
    CHECK(events[2].kind == LifecycleEvent::GCodeExportStarted);
    CHECK(events[3].kind == LifecycleEvent::GCodeExportFinished);
    CHECK(events[3].code == LifecycleEvtCode::Ok);
    CHECK(print.is_step_done(psGCodeExport));
    CHECK(boost::filesystem::is_regular_file(output));
    CHECK_FALSE(boost::filesystem::exists(temporary));

    std::ifstream stream(output.string());
    const std::string text((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    size_t deposited_moves = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(text, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > EPSILON)
            ++deposited_moves;
    });
    CHECK(deposited_moves > 0);
}
namespace print_object_sharing_contracts {

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

enum class ShareDifference {
    MeshPointer,
    VolumeTransform,
    VolumeExtruder,
    LayerProfile,
    ObjectExtruder,
    LayerConfigRange,
    MaterialConfig,
};

DynamicPrintConfig sharing_config(ShareDifference difference)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height",                 0.2},
        {"initial_layer_print_height",   0.2},
        {"wall_loops",                   2},
        {"top_shell_layers",             1},
        {"bottom_shell_layers",          1},
        {"sparse_infill_density",        "15%"},
        {"skirt_loops",                  0},
        {"brim_type",                    "no_brim"},
        {"raft_layers",                  0},
        {"machine_start_gcode",          ""},
        {"machine_end_gcode",            ""},
        {"layer_change_gcode",           "G92 E0\n"},
        {"before_layer_change_gcode",    ""},
        {"print_sequence",               "by layer"},
    });
    if (difference == ShareDifference::VolumeExtruder || difference == ShareDifference::ObjectExtruder)
        config = Test::multifilament_config(2, {
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            {"wall_loops", 2},
            {"top_shell_layers", 1},
            {"bottom_shell_layers", 1},
            {"sparse_infill_density", "15%"},
            {"skirt_loops", 0},
            {"brim_type", "no_brim"},
            {"raft_layers", 0},
            {"machine_start_gcode", ""},
            {"machine_end_gcode", ""},
            {"layer_change_gcode", "G92 E0\n"},
            {"print_sequence", "by layer"},
        });
    return config;
}

struct ShareProbeResult {
    bool                shared = false;
    double              first_area = 0.;
    double              second_area = 0.;
    std::vector<double> first_print_z;
    std::vector<double> second_print_z;
    std::vector<size_t> first_perimeters;
    std::vector<size_t> second_perimeters;
};

double first_layer_area(const PrintObject &object)
{
    REQUIRE(object.layer_count() > 0);
    const Layer *layer = object.get_layer(0);
    REQUIRE(layer != nullptr);
    double result = 0.;
    for (const ExPolygon &slice : layer->lslices)
        result += std::abs(static_cast<double>(slice.area())) * SCALING_FACTOR * SCALING_FACTOR;
    return result;
}

std::vector<double> layer_print_zs(const PrintObject &object)
{
    REQUIRE(object.layer_count() > 0);
    std::vector<double> result;
    result.reserve(object.layer_count());
    for (int index = 0; index < object.layer_count(); ++index) {
        const Layer *layer = object.get_layer(index);
        REQUIRE(layer != nullptr);
        result.emplace_back(layer->print_z);
    }
    return result;
}

std::vector<size_t> layer_perimeter_counts(const PrintObject &object)
{
    std::vector<size_t> result;
    result.reserve(object.layer_count());
    for (size_t layer = 0; layer < object.layer_count(); ++layer)
        result.emplace_back(print_workflow_contracts::perimeter_items(object, layer));
    return result;
}

size_t layer_index_matching(const std::vector<double> &print_zs, const std::function<bool(double)> &matches)
{
    const auto iter = std::find_if(print_zs.begin(), print_zs.end(), matches);
    REQUIRE(iter != print_zs.end());
    return static_cast<size_t>(std::distance(print_zs.begin(), iter));
}

ShareProbeResult run_nonsharing_probe(ShareDifference difference)
{
    DynamicPrintConfig config = sharing_config(difference);
    Model model;
    Print print;

    ModelObject *first = model.add_object();
    REQUIRE(first != nullptr);
    first->add_volume(make_cube(8., 8., 1.2));
    REQUIRE(first->volumes.size() == 1);
    first->add_instance();
    REQUIRE_FALSE(first->instances.empty());
    first->config.set_key_value("extruder", new ConfigOptionInt(1));

    ModelObject *second = model.add_object(*first);
    REQUIRE(second != nullptr);
    REQUIRE(second->volumes.size() == 1);
    REQUIRE_FALSE(second->instances.empty());
    for (ModelInstance *instance : second->instances) {
        REQUIRE(instance != nullptr);
        instance->set_offset(instance->get_offset() + Vec3d(28., 0., 0.));
    }

    if (difference == ShareDifference::MeshPointer)
        second->volumes.front()->set_mesh(make_cube(10., 8., 1.2));
    else if (difference == ShareDifference::VolumeTransform)
        second->volumes.front()->set_scaling_factor(Vec3d(1.25, 1., 1.));
    else if (difference == ShareDifference::VolumeExtruder) {
        first->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(1));
        second->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(2));
    } else if (difference == ShareDifference::LayerProfile) {
        second->layer_height_profile.set(std::vector<coordf_t>{0., 0.2, 0.2, 0.25, 1.2, 0.25});
    } else if (difference == ShareDifference::ObjectExtruder) {
        first->config.set_key_value("extruder", new ConfigOptionInt(1));
        second->config.set_key_value("extruder", new ConfigOptionInt(2));
    }

    if (difference == ShareDifference::LayerConfigRange || difference == ShareDifference::MaterialConfig)
        second->volumes.front()->set_mesh(first->volumes.front()->mesh());

    if (difference == ShareDifference::LayerConfigRange) {
        ModelConfig range;
        range.set_key_value("layer_height", new ConfigOptionFloat(0.2));
        range.set_key_value("wall_loops", new ConfigOptionInt(4));
        second->layer_config_ranges[{0.4, 0.8}] = range;
        REQUIRE(first->layer_config_ranges.empty());
        REQUIRE(second->layer_config_ranges.size() == 1);
    } else if (difference == ShareDifference::MaterialConfig) {
        ModelMaterial *two_wall_material = model.add_material("two-wall-loops");
        ModelMaterial *four_wall_material = model.add_material("four-wall-loops");
        REQUIRE(two_wall_material != nullptr);
        REQUIRE(four_wall_material != nullptr);
        two_wall_material->config.set_key_value("wall_loops", new ConfigOptionInt(2));
        four_wall_material->config.set_key_value("wall_loops", new ConfigOptionInt(4));
        first->volumes.front()->set_material_id("two-wall-loops");
        second->volumes.front()->set_material_id("four-wall-loops");
        REQUIRE(first->volumes.front()->material_id() != second->volumes.front()->material_id());
        REQUIRE(first->volumes.front()->material()->config.opt_int("wall_loops") == 2);
        REQUIRE(second->volumes.front()->material()->config.opt_int("wall_loops") == 4);
    }

    if (difference == ShareDifference::LayerConfigRange || difference == ShareDifference::MaterialConfig) {
        REQUIRE(first->volumes.front()->mesh_ptr() != second->volumes.front()->mesh_ptr());
        const auto &first_vertices = first->volumes.front()->mesh().its.vertices;
        const auto &second_vertices = second->volumes.front()->mesh().its.vertices;
        REQUIRE(first_vertices.size() == second_vertices.size());
        for (size_t vertex = 0; vertex < first_vertices.size(); ++vertex)
            for (size_t axis = 0; axis < 3; ++axis)
                CHECK_THAT(first_vertices[vertex][axis], Catch::Matchers::WithinAbs(second_vertices[vertex][axis], 0.));
        REQUIRE(first->volumes.front()->mesh().its.indices == second->volumes.front()->mesh().its.indices);
        const auto &first_properties = first->volumes.front()->mesh().its.properties;
        const auto &second_properties = second->volumes.front()->mesh().its.properties;
        REQUIRE(first_properties.size() == second_properties.size());
        for (size_t face = 0; face < first_properties.size(); ++face) {
            REQUIRE(first_properties[face].type == second_properties[face].type);
            CHECK_THAT(first_properties[face].area, Catch::Matchers::WithinAbs(second_properties[face].area, 1e-9));
        }
        REQUIRE(first->config.get() == second->config.get());
        REQUIRE(first->volumes.front()->config.get() == second->volumes.front()->config.get());
        REQUIRE(first->volumes.front()->get_transformation() == second->volumes.front()->get_transformation());
        if (difference == ShareDifference::LayerConfigRange) {
            REQUIRE(first->layer_height_profile.empty());
            REQUIRE(second->layer_height_profile.empty());
        }
    }

    for (ModelObject *object : model.objects) {
        object->ensure_on_bed();
        print.auto_assign_extruders(object);
    }
    print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO("validation=" << validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    print.process();

    REQUIRE(print.objects().size() == 2);
    const PrintObject *first_print_object = print.get_object(0);
    const PrintObject *second_print_object = print.get_object(1);
    REQUIRE(first_print_object != nullptr);
    REQUIRE(second_print_object != nullptr);
    if (difference == ShareDifference::LayerConfigRange || difference == ShareDifference::MaterialConfig)
        REQUIRE(first_print_object->trafo().matrix().isApprox(second_print_object->trafo().matrix(), 0.));
    return {
        second_print_object->get_shared_object() != nullptr,
        first_layer_area(*first_print_object),
        first_layer_area(*second_print_object),
        layer_print_zs(*first_print_object),
        layer_print_zs(*second_print_object),
        layer_perimeter_counts(*first_print_object),
        layer_perimeter_counts(*second_print_object),
    };
}

} // namespace

TEST_CASE("Distinct model meshes keep public Print objects independently sliced",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::MeshPointer);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(80., 0.05));
}

TEST_CASE("A volume transform difference prevents public Print layer sharing",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::VolumeTransform);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(80., 0.05));
}

TEST_CASE("A volume extruder difference preserves independent public Print layers",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::VolumeExtruder);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(64., 0.05));
}

TEST_CASE("A custom layer profile prevents public Print layer sharing",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::LayerProfile);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    REQUIRE(result.first_print_z.size() != result.second_print_z.size());
    CHECK_FALSE(result.first_print_z == result.second_print_z);
}

TEST_CASE("An object extruder difference prevents public Print layer sharing",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::ObjectExtruder);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(64., 0.05));
}


TEST_CASE("Equal separately allocated meshes preserve per-object layer-range walls",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::LayerConfigRange);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(64., 0.05));
    REQUIRE(result.first_print_z.size() == result.second_print_z.size());
    for (size_t layer = 0; layer < result.first_print_z.size(); ++layer)
        CHECK_THAT(result.first_print_z[layer], Catch::Matchers::WithinAbs(result.second_print_z[layer], 1e-6));
    REQUIRE(result.first_perimeters.size() == result.first_print_z.size());
    REQUIRE(result.second_perimeters.size() == result.second_print_z.size());

    const size_t below = layer_index_matching(result.first_print_z, [](double z) { return z < 0.4 - 1e-4; });
    const size_t inside = layer_index_matching(result.first_print_z, [](double z) { return z > 0.4 + 1e-4 && z < 0.8 - 1e-4; });
    const size_t above = layer_index_matching(result.first_print_z, [](double z) { return z > 0.8 + 1e-4; });
    CHECK(result.first_perimeters[below] == 2);
    CHECK(result.second_perimeters[below] == 2);
    CHECK(result.first_perimeters[inside] == 2);
    CHECK(result.second_perimeters[inside] == 4);
    CHECK(result.first_perimeters[above] == 2);
    CHECK(result.second_perimeters[above] == 2);
}

TEST_CASE("Equal separately allocated meshes preserve material-specific wall paths",
          "[Print][PrintObjectSharing]")
{
    const ShareProbeResult result = run_nonsharing_probe(ShareDifference::MaterialConfig);
    CHECK_FALSE(result.shared);
    CHECK_THAT(result.first_area, Catch::Matchers::WithinAbs(64., 0.05));
    CHECK_THAT(result.second_area, Catch::Matchers::WithinAbs(64., 0.05));
    REQUIRE(result.first_print_z.size() == result.second_print_z.size());
    for (size_t layer = 0; layer < result.first_print_z.size(); ++layer)
        CHECK_THAT(result.first_print_z[layer], Catch::Matchers::WithinAbs(result.second_print_z[layer], 1e-6));
    REQUIRE(result.first_perimeters.size() == result.first_print_z.size());
    REQUIRE(result.second_perimeters.size() == result.second_print_z.size());

    const size_t first_layer = layer_index_matching(result.first_print_z, [](double z) { return z > 0.2 - 1e-4; });
    const size_t middle_layer = layer_index_matching(result.first_print_z, [](double z) { return z > 0.6 - 1e-4; });
    const size_t upper_layer = layer_index_matching(result.first_print_z, [](double z) { return z > 1.0 - 1e-4; });
    for (size_t layer : {first_layer, middle_layer, upper_layer}) {
        CHECK(result.first_perimeters[layer] == 2);
        CHECK(result.second_perimeters[layer] == 4);
    }
}

TEST_CASE("A missing previous G-code file reports a failed public export lifecycle",
          "[Print][PreviousGCodeWorkflow]")
{
    Print print;
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0}, {"brim_type", "no_brim"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
    });
    init_print({make_cube(8., 8., 1.2)}, print, model, config);

    ScopedTemporaryDir directory("orca-previous-gcode-missing");
    const std::string missing = (directory.path() / "not-there.gcode").string();
    std::vector<LifecycleEvent> events;
    std::vector<LifecycleEvtCode> codes;
    std::vector<std::string> messages;
    ScopedLifecycleHook hook([&](LifecycleEvent event, const LifecycleEventContext &context) {
        if (event == LifecycleEvent::GCodeExportStarted || event == LifecycleEvent::GCodeExportFinished) {
            events.push_back(event);
            codes.push_back(context.code);
            messages.push_back(context.msg);
        }
    });

    GCodeProcessorResult result;
    const std::vector<unsigned int> filament_sequence{4, 2};
    const std::vector<unsigned int> nozzle_sequence{1, 0};
    result.filament_change_sequence = filament_sequence;
    result.nozzle_change_sequence = nozzle_sequence;
    REQUIRE_THROWS_AS(print.export_gcode_from_previous_file(missing, &result), Slic3r::RuntimeError);
    REQUIRE(events.size() == 2);
    CHECK(events[0] == LifecycleEvent::GCodeExportStarted);
    CHECK(events[1] == LifecycleEvent::GCodeExportFinished);
    CHECK(codes[0] == LifecycleEvtCode::Ok);
    CHECK(codes[1] == LifecycleEvtCode::Error);
    CHECK(messages[0] == missing);
    CHECK(messages[1].find(missing) != std::string::npos);
    CHECK(result.filament_change_sequence == filament_sequence);
    CHECK(result.nozzle_change_sequence == nozzle_sequence);
}

TEST_CASE("Reprocessing previous G-code preserves caller tool sequences and public moves",
          "[Print][PreviousGCodeWorkflow]")
{
    Print print;
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0}, {"brim_type", "no_brim"},
        {"machine_start_gcode", ""}, {"machine_end_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
    });
    init_print({make_cube(8., 8., 1.2)}, print, model, config);
    print.process();
    ScopedTemporaryFile file(".gcode");
    REQUIRE(print.export_gcode(file.string(), nullptr, nullptr) == file.string());

    GCodeProcessorResult result;
    const std::vector<unsigned int> filament_sequence{7, 3, 7};
    const std::vector<unsigned int> nozzle_sequence{2, 1};
    result.filament_change_sequence = filament_sequence;
    result.nozzle_change_sequence = nozzle_sequence;
    std::vector<LifecycleEvtCode> finished_codes;
    ScopedLifecycleHook hook([&](LifecycleEvent event, const LifecycleEventContext &context) {
        if (event == LifecycleEvent::GCodeExportFinished)
            finished_codes.push_back(context.code);
    });

    REQUIRE_NOTHROW(print.export_gcode_from_previous_file(file.string(), &result));
    REQUIRE(result.moves.size() > 0);
    CHECK(result.filament_change_sequence == filament_sequence);
    CHECK(result.nozzle_change_sequence == nozzle_sequence);
    REQUIRE(finished_codes.size() == 1);
    CHECK(finished_codes.front() == LifecycleEvtCode::Ok);
}

} // namespace print_object_sharing_contracts

namespace print_validation_followup_contracts {

struct TreePatternValidationCase {
    const char *support_style;
    int         wall_count;
    const char *base_pattern;
    size_t      expected_pattern_warnings;
    bool        expect_two_wall_warning;
    bool        expect_lightning_warning;
};

TEST_CASE("Organic support validation reports wall and base-pattern conflicts",
          "[Print][ValidationContracts][TreeSupportValidation]")
{
    const auto part = GENERATE(
        TreePatternValidationCase{"organic", 1, "rectilinear", 0, false, false},
        TreePatternValidationCase{"organic", 2, "rectilinear", 1, true, false},
        TreePatternValidationCase{"default", 2, "rectilinear", 1, true, false},
        TreePatternValidationCase{"organic", 2, "lightning", 2, true, true});
    CAPTURE(part.support_style, part.wall_count, part.base_pattern);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"sparse_infill_density", "15%"},
        {"enable_support", true},
        {"support_type", "tree(auto)"},
        {"support_style", part.support_style},
        {"support_base_pattern", part.base_pattern},
        {"support_line_width", 0.45},
        {"tree_support_wall_count", part.wall_count},
        {"tree_support_tip_diameter", 0.6},
        {"tree_support_branch_diameter_organic", 2.0},
        {"enable_prime_tower", false},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });

    Print print;
    Model model;
    init_print({mesh(TestMesh::overhang)}, print, model, config);
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->has_support_material());

    std::vector<StringObjectException> warnings;
    const StringObjectException validation = print.validate(&warnings);
    REQUIRE(validation.string.empty());

    std::vector<const StringObjectException *> pattern_warnings;
    for (const StringObjectException &warning : warnings)
        if (warning.opt_key == "support_base_pattern")
            pattern_warnings.push_back(&warning);
    REQUIRE(pattern_warnings.size() == part.expected_pattern_warnings);

    bool saw_two_wall_warning = false;
    bool saw_lightning_warning = false;
    for (const StringObjectException *warning : pattern_warnings) {
        saw_two_wall_warning = saw_two_wall_warning ||
            warning->string.find("two walls are supported only with the Hollow/Default base pattern") != std::string::npos;
        saw_lightning_warning = saw_lightning_warning ||
            warning->string.find("Lightning base pattern is not supported") != std::string::npos;
        CHECK(warning->is_warning);
    }
    CHECK(saw_two_wall_warning == part.expect_two_wall_warning);
    CHECK(saw_lightning_warning == part.expect_lightning_warning);
}

namespace {

const std::vector<coordf_t> kLayerProfileA{0., 0.2, 0.6, 0.2, 1.2, 0.2};
const std::vector<coordf_t> kLayerProfileB{0., 0.2, 0.6, 0.25, 1.2, 0.2};
// Stay clear of rounding at the exact floating-point epsilon boundary.
const std::vector<coordf_t> kLayerProfileInside{0., 0.2, 0.6, 0.2 + 0.4 * EPSILON, 1.2, 0.2};
const std::vector<coordf_t> kLayerProfileOutside{0., 0.2, 0.6, 0.2 + 0.6 * EPSILON, 1.2, 0.2};

DynamicPrintConfig custom_layer_validation_config(bool precise_z_height)
{
    DynamicPrintConfig config = multifilament_config(2, {
        {"enable_prime_tower", true},
        {"print_sequence", "by layer"},
        // Keep the generated tower wholly inside a finite, non-belt bed so the
        // profile-validation rows reach the layer-profile contract.
        {"belt_printer", false},
        {"printable_area", "0x0,200x0,200x200,0x200"},
        {"wipe_tower_wall_type", "rectangle"},
        {"prime_tower_width", 35.0},
        {"prime_tower_brim_width", 0.0},
        {"wipe_tower_x", 50.0},
        {"wipe_tower_y", 50.0},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"sparse_infill_density", "15%"},
        {"precise_z_height", precise_z_height},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });
    return config;
}

std::vector<double> public_layer_zs(const PrintObject &object)
{
    std::vector<double> zs;
    zs.reserve(object.layer_count());
    for (int i = 0; i < object.layer_count(); ++i) {
        const Layer *layer = object.get_layer(i);
        REQUIRE(layer != nullptr);
        REQUIRE(std::isfinite(layer->print_z));
        zs.push_back(layer->print_z);
    }
    return zs;
}

namespace print_validate_followup_detail {

DynamicPrintConfig low_jerk_config(const char *selected_key, double value, const char *machine_max_junction_deviation)
{
    DynamicPrintConfig config = validation_contract_config();
    config.set_deserialize_strict({
        { "machine_max_junction_deviation", machine_max_junction_deviation },
        { "default_junction_deviation", "0" },
        { "machine_max_jerk_x", "100,100" },
        { "machine_max_jerk_y", "100,100" },
        { "default_jerk", "0" },
        { "outer_wall_jerk", "0" },
        { "inner_wall_jerk", "0" },
        { "default_acceleration", "0" },
    });
    config.set_key_value(selected_key, new ConfigOptionFloatsNullable{value});
    return config;
}

DynamicPrintConfig adaptive_pa_validation_config(const std::string &adaptive,
                                                 const std::string &enable,
                                                 const std::string &model0,
                                                 const std::string &model1,
                                                 const std::string &model2)
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "gcode_flavor",                "marlin2" },
        { "layer_height",                0.2 },
        { "initial_layer_print_height",  0.2 },
        { "enable_support",              false },
        { "spiral_mode",                 false },
        { "belt_printer",                false },
        // One physical extruder exposes both variants. The persisted selector result below
        // migrates filament 1 between these two nozzles, while filament 2 remains Standard.
        { "extruder_nozzle_stats",       "Standard#1|High Flow#1" },
        { "extruder_variant_list",       "Direct Drive Standard,Direct Drive High Flow" },
        { "filament_extruder_variant",   "Direct Drive Standard;Direct Drive High Flow;Direct Drive Standard" },
        { "filament_self_index",         "1,1,2" },
        { "filament_map",                "1,1" },
        { "filament_volume_map",         "0,0" },
        { "sparse_infill_filament_id",   1 },
        { "internal_solid_filament_id",  1 },
        { "top_surface_filament_id",     1 },
        { "bottom_surface_filament_id",  1 },
        { "outer_wall_filament_id",      1 },
        { "inner_wall_filament_id",      1 },
        { "adaptive_pressure_advance",   adaptive },
        { "enable_pressure_advance",     enable },
        { "enable_prime_tower",          false },
        { "skirt_loops",                 0 },
        { "brim_type",                   "no_brim" },
        { "layer_change_gcode",          "G92 E0\n" },
    });
    config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {nvtHybrid};
    config.option<ConfigOptionStrings>("adaptive_pressure_advance_model")->values = {
        model0,
        model1,
        model2,
    };
    return config;
}

void apply_persisted_adaptive_pa_group(Print &print, const Model &model, const DynamicPrintConfig &config)
{
    using MultiNozzleUtils::LayeredNozzleGroupResult;
    using MultiNozzleUtils::NozzleInfo;
    const std::vector<NozzleInfo> nozzles{
        NozzleInfo{"0.4", nvtStandard, 0, 0},
        NozzleInfo{"0.4", nvtHighFlow, 0, 1},
    };
    auto group = LayeredNozzleGroupResult::create(
        std::vector<std::vector<int>>{{0, 0}, {1, 0}}, nozzles, {0}, std::vector<std::vector<unsigned int>>{{0}, {0}});
    REQUIRE(group.has_value());
    REQUIRE(group->is_support_dynamic_nozzle_map());
    print.set_nozzle_group_result(std::make_shared<LayeredNozzleGroupResult>(*group));
    print.apply(model, config);

    const PrintConfig &expanded = print.config();
    const auto *self_index = expanded.option<ConfigOptionInts>("filament_self_index");
    const auto *variants = expanded.option<ConfigOptionStrings>("filament_extruder_variant");
    const auto *models = expanded.option<ConfigOptionStrings>("adaptive_pressure_advance_model");
    const auto *adaptive = expanded.option<ConfigOptionBools>("adaptive_pressure_advance");
    const auto *enable = expanded.option<ConfigOptionBools>("enable_pressure_advance");
    REQUIRE(self_index != nullptr);
    REQUIRE(variants != nullptr);
    REQUIRE(models != nullptr);
    REQUIRE(adaptive != nullptr);
    REQUIRE(enable != nullptr);
    REQUIRE(self_index->values == std::vector<int>{1, 1, 2});
    REQUIRE(variants->values == std::vector<std::string>{"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard"});
    REQUIRE(models->values == config.option<ConfigOptionStrings>("adaptive_pressure_advance_model")->values);
    REQUIRE(adaptive->values == config.option<ConfigOptionBools>("adaptive_pressure_advance")->values);
    REQUIRE(enable->values == config.option<ConfigOptionBools>("enable_pressure_advance")->values);
    REQUIRE(print.extruders() == std::vector<unsigned int>{0});
}

size_t warning_count_for(const std::vector<StringObjectException> &warnings, const std::string &key)
{
    return std::count_if(warnings.begin(), warnings.end(), [&key](const auto &entry) {
        return entry.opt_key == key && entry.is_warning;
    });
}

size_t low_jerk_warning_count(const std::vector<StringObjectException> &warnings)
{
    return std::count_if(warnings.begin(), warnings.end(), [](const auto &entry) {
        return entry.is_warning && (entry.opt_key == "default_jerk" ||
                                    entry.opt_key == "outer_wall_jerk" ||
                                    entry.opt_key == "inner_wall_jerk");
    });
}

} // namespace print_validate_followup_detail

TEST_CASE("Print validation dispatches low-jerk warnings and honors the Marlin junction gate", "[Print][ValidationContracts][PrintValidationMotion]")
{
    struct LowJerkCase {
        const char *key;
        double value;
        const char *machine_max_junction_deviation;
        bool expected_warning;
        const char *expected_key;
    };
    const auto part = GENERATE(
        LowJerkCase{ "default_jerk", 0.0, "0,0", false, "" },
        LowJerkCase{ "default_jerk", 1.0, "0,0", true,  "default_jerk" },
        LowJerkCase{ "default_jerk", 2.0, "0,0", false, "" },
        LowJerkCase{ "outer_wall_jerk", 1.0, "0,0", true,  "outer_wall_jerk" },
        LowJerkCase{ "inner_wall_jerk", 1.0, "0,0", true,  "inner_wall_jerk" },
        LowJerkCase{ "default_jerk", 1.0, "0.02,0.02", false, "" });
    CAPTURE(part.key, part.value, part.machine_max_junction_deviation, part.expected_warning, part.expected_key);

    DynamicPrintConfig config = print_validate_followup_detail::low_jerk_config(
        part.key, part.value, part.machine_max_junction_deviation);
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);

    std::vector<StringObjectException> warnings;
    const auto error = print.validate(&warnings);
    REQUIRE(error.string.empty());
    CHECK(print_validate_followup_detail::low_jerk_warning_count(warnings) ==
          (part.expected_warning ? 1u : 0u));
    if (!part.expected_warning)
        return;

    const auto warning = std::find_if(warnings.begin(), warnings.end(), [&part](const auto &entry) {
        return entry.opt_key == part.expected_key && entry.is_warning;
    });
    REQUIRE(warning != warnings.end());
    CHECK(warning->object == nullptr);
    CHECK_FALSE(warning->string.empty());
}

TEST_CASE("Print validation reports an invalid adaptive pressure advance model for a used column", "[Print][ValidationContracts][PrintValidationPressureAdvance]")
{
    const std::string invalid_model = "0.04,10";
    const std::string valid_model = "0.04,10,1000\n0.05,20,2000";
    DynamicPrintConfig config = print_validate_followup_detail::adaptive_pa_validation_config(
        "1,1,1", "1,1,1", invalid_model, valid_model, valid_model);
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    print_validate_followup_detail::apply_persisted_adaptive_pa_group(print, model, config);

    std::vector<StringObjectException> warnings;
    const auto error = print.validate(&warnings);
    REQUIRE(error.string.empty());
    REQUIRE(print_validate_followup_detail::warning_count_for(warnings, "adaptive_pressure_advance_model") == 1);
    const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const auto &entry) {
        return entry.opt_key == "adaptive_pressure_advance_model" && entry.is_warning;
    });
    REQUIRE(warning != warnings.end());
    CHECK(warning->object == nullptr);
    CHECK_FALSE(warning->string.empty());
}

TEST_CASE("Print validation gates adaptive pressure advance by controls and used filament mapping", "[Print][ValidationContracts][PrintValidationPressureAdvance]")
{
    struct AdaptiveCase {
        const char *adaptive;
        const char *enable;
        const char *model0;
        const char *model1;
        const char *model2;
        bool expected_warning;
    };
    const auto part = GENERATE(
        AdaptiveCase{ "0,1,1", "1,1,1", "0.04,10", "0.04,10,1000", "0.04,10,1000", false },
        AdaptiveCase{ "1,1,1", "0,1,1", "0.04,10", "0.04,10,1000", "0.04,10,1000", false },
        AdaptiveCase{ "1,1,1", "1,1,1", "0.04,10,1000", "0.04,10,1000", "0.04,10", false },
        AdaptiveCase{ "1,1,1", "1,1,1", "0.04,10,1000", "0.04,10", "0.04,10,1000", true });
    CAPTURE(part.adaptive, part.enable, part.model0, part.model1, part.model2, part.expected_warning);
    DynamicPrintConfig config = print_validate_followup_detail::adaptive_pa_validation_config(
        part.adaptive, part.enable, part.model0, part.model1, part.model2);
    Print print;
    Model model;
    init_print({ cube(10.0) }, print, model, config);
    print_validate_followup_detail::apply_persisted_adaptive_pa_group(print, model, config);

    std::vector<StringObjectException> warnings;
    const auto error = print.validate(&warnings);
    REQUIRE(error.string.empty());
    CHECK(print_validate_followup_detail::warning_count_for(warnings, "adaptive_pressure_advance_model") ==
          (part.expected_warning ? 1 : 0));
}

} // namespace

TEST_CASE("Print validation compares custom layer profiles across prime-tower objects",
          "[Print][ValidationContracts][CustomLayerProfiles]")
{
    enum class ProfileRelation { Equal, Different, InsideTolerance, OutsideTolerance };
    const ProfileRelation relation = GENERATE(ProfileRelation::Equal, ProfileRelation::Different, ProfileRelation::InsideTolerance, ProfileRelation::OutsideTolerance);
    const bool precise_z_height = GENERATE(false, true);
    CAPTURE(static_cast<int>(relation), precise_z_height);

    const DynamicPrintConfig config = custom_layer_validation_config(precise_z_height);
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        {{"extruder", 1}},
        {{"extruder", 2}},
    };
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{make_cube(8., 8., 1.2), make_cube(8., 8., 1.2)},
               print, model, config, &overrides);
    REQUIRE(model.objects.size() == 2);
    model.objects[0]->layer_height_profile.set(kLayerProfileA);
    model.objects[1]->layer_height_profile.set(
        relation == ProfileRelation::Equal ? kLayerProfileA :
        relation == ProfileRelation::InsideTolerance ? kLayerProfileInside :
        relation == ProfileRelation::OutsideTolerance ? kLayerProfileOutside : kLayerProfileB);
    print.apply(model, config);

    REQUIRE(print.extruders(true).size() == 2);
    std::vector<StringObjectException> warnings;
    const StringObjectException validation = print.validate(&warnings);
    CAPTURE(validation.string, validation.opt_key);
    const size_t precise_warnings = std::count_if(warnings.begin(), warnings.end(), [](const StringObjectException &warning) {
        return warning.opt_key == "precise_z_height";
    });
    CHECK(precise_warnings == (precise_z_height ? 1u : 0u));
    if (relation == ProfileRelation::Different || relation == ProfileRelation::OutsideTolerance) {
        REQUIRE(validation.string == "The prime tower is only supported if all objects have the same variable layer height.");
        CHECK(validation.opt_key.empty());
        CHECK_FALSE(validation.is_warning);
        return;
    }
    if (relation == ProfileRelation::InsideTolerance) {
        REQUIRE(validation.string.empty());
        return;
    }

    REQUIRE(validation.string.empty());
    print.set_status_silent();
    print.process();
    REQUIRE(print.objects().size() >= 2);
    const PrintObject *first = print.get_object(0);
    const PrintObject *second = print.get_object(1);
    REQUIRE(first != nullptr);
    REQUIRE(second != nullptr);
    const std::vector<double> first_zs = public_layer_zs(*first);
    const std::vector<double> second_zs = public_layer_zs(*second);
    REQUIRE_FALSE(first_zs.empty());
    REQUIRE(first_zs.size() == second_zs.size());
    CHECK(first_zs.back() > 1.0);
    for (size_t i = 0; i < first_zs.size(); ++i)
        CHECK_THAT(first_zs[i], Catch::Matchers::WithinAbs(second_zs[i], 1e-6));
}

} // namespace print_validation_followup_contracts

TEST_CASE("Print validation distinguishes a last-layer overrun from a fitting height boundary",
          "[Print][ValidationContracts][PrintValidationPublicBoundary]")
{
    struct HeightCase {
        double printable_height;
        bool   expects_last_layer_error;
    };
    const HeightCase part = GENERATE(HeightCase{9.9, true}, HeightCase{10.0, false});
    CAPTURE(part.printable_height, part.expects_last_layer_error);

    DynamicPrintConfig config = validation_contract_config();
    config.set_key_value("printable_height", new ConfigOptionFloat(part.printable_height));
    Print print;
    Model model;
    init_print({cube(10.0)}, print, model, config);
    REQUIRE(print.objects().size() == 1);

    const StringObjectException error = print.validate();
    if (!part.expects_last_layer_error) {
        CHECK(error.string.empty());
        CHECK(error.opt_key.empty());
        return;
    }

    REQUIRE_FALSE(error.string.empty());
    CHECK(error.string.find("its last layer exceeds the maximum build volume height.") != std::string::npos);
    CHECK(error.object == print.objects().front()->model_object());
    CHECK(error.opt_key.empty());
    CHECK_FALSE(error.is_warning);
}

TEST_CASE("Print validation rejects spiral vase with two by-layer copies at the copy-count boundary",
          "[Print][ValidationContracts][PrintValidationPublicBoundary]")
{
    const size_t copies = GENERATE(size_t(1), size_t(2));
    CAPTURE(copies);

    DynamicPrintConfig config = validation_contract_config();
    config.set_deserialize_strict({
        {"spiral_mode", true},
        {"print_sequence", "by layer"},
    });
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{cube(10.0)}, print, model, config, nullptr, true, copies);

    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->instances().size() == copies);
    const StringObjectException error = print.validate();
    if (copies == 1) {
        CHECK(error.string.empty());
        CHECK(error.opt_key.empty());
    } else {
        CHECK(error.string == "Please select \"By object\" print sequence to print multiple objects in spiral vase mode.");
        CHECK(error.opt_key == "spiral_mode");
        CHECK(error.object == nullptr);
        CHECK_FALSE(error.is_warning);
    }
}

TEST_CASE("Print validation requires prime-tower objects to share their layer-height contract",
          "[Print][ValidationContracts][PrintValidationPublicBoundary]")
{
    const bool change_second_layer_height = GENERATE(false, true);
    CAPTURE(change_second_layer_height);

    const DynamicPrintConfig config = multifilament_config(2, {
        {"enable_prime_tower", true},
        {"belt_printer", false},
        {"spiral_mode", false},
        {"print_sequence", "by layer"},
        {"timelapse_type", "0"},
        {"enable_wrapping_detection", false},
        {"wipe_tower_type", "type2"},
        {"wipe_tower_wall_type", "rectangle"},
        {"printable_area", "0x0,200x0,200x200,0x200"},
        {"wipe_tower_x", 50.0},
        {"wipe_tower_y", 50.0},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });
    Model model;
    Print print;
    build_two_filament_wipe_print(config, model, print);
    REQUIRE(print.objects().size() == 2);

    if (change_second_layer_height)
        model.objects[1]->config.set_key_value("layer_height", new ConfigOptionFloat(0.3));
    print.apply(model, config);
    REQUIRE(print.has_wipe_tower());

    const StringObjectException error = print.validate();
    if (!change_second_layer_height) {
        CHECK(error.string.empty());
        CHECK(error.opt_key.empty());
        return;
    }

    CHECK(error.string == "A prime tower requires that all objects have the same layer height.");
    CHECK(error.opt_key == "initial_layer_print_height");
    CHECK(error.object == print.objects()[1]);
    CHECK_FALSE(error.is_warning);
}

TEST_CASE("Print validation warns about unused support enforcers but ignores blockers and enabled support",
          "[Print][ValidationContracts][PrintValidationPublicBoundary]")
{
    struct EnforcerCase {
        ModelVolumeType volume_type;
        bool            support_enabled;
        bool            expects_warning;
    };
    const EnforcerCase part = GENERATE(
        EnforcerCase{ModelVolumeType::SUPPORT_ENFORCER, false, true},
        EnforcerCase{ModelVolumeType::SUPPORT_ENFORCER, true, false},
        EnforcerCase{ModelVolumeType::SUPPORT_BLOCKER, false, false});
    CAPTURE(static_cast<int>(part.volume_type), part.support_enabled, part.expects_warning);

    DynamicPrintConfig config = validation_contract_config();
    config.set_deserialize_strict({
        {"enable_support", part.support_enabled},
        {"support_type", "normal(auto)"},
    });
    Print print;
    Model model;
    init_print({cube(10.0)}, print, model, config);
    ModelVolume *modifier = model.objects.front()->add_volume(cube(4.0), part.volume_type);
    REQUIRE(modifier != nullptr);
    print.apply(model, config);

    REQUIRE(print.objects().size() == 1);
    CHECK(print.objects().front()->has_support() == part.support_enabled);
    std::vector<StringObjectException> warnings;
    const StringObjectException error = print.validate(&warnings);
    REQUIRE(error.string.empty());

    const auto warning = std::find_if(warnings.begin(), warnings.end(), [](const StringObjectException &entry) {
        return entry.string.find("Support enforcers are used but support is not enabled.") != std::string::npos;
    });
    if (!part.expects_warning) {
        CHECK(warning == warnings.end());
        return;
    }

    REQUIRE(warning != warnings.end());
    CHECK(warning->opt_key.empty());
    CHECK(warning->is_warning);
    CHECK(warning->object == print.objects().front());
}

TEST_CASE("Print validation reports belt-only brim settings with stable warning keys",
          "[Print][ValidationContracts][PrintValidationPublicBoundary]")
{
    struct BrimCase {
        const char *name;
        bool        belt_printer;
        const char *brim_type;
        double      belt_angle;
        double      brim_width;
        double      leading_brim_length;
        double      brim_object_gap;
        int         object_count;
        const char *expected_key;
        const char *expected_message;
        bool        expected_model_object;
    };
    const BrimCase part = GENERATE(
        BrimCase{"flat belt with outer brim", true, "outer_only", 0.0, 5.0, 0.0, 0.0, 1,
                 "brim_type", "A brim is only generated when the belt is tilted", true},
        BrimCase{"steep belt with outer brim", true, "outer_only", 90.0, 5.0, 0.0, 0.0, 1,
                 "brim_width", "The belt is too steep for a brim", true},
        BrimCase{"tilted belt with automatic brim", true, "auto_brim", 45.0, 5.0, 0.0, 0.0, 1,
                 "brim_type", "Belt printers support outer and inner brim only", true},
        BrimCase{"leading brim separated from the object", true, "outer_only", 45.0, 5.0, 10.0, 2.0, 1,
                 "brim_object_gap", "Brim-object gap separates the leading brim", true},
        BrimCase{"multiple belt objects with leading brim", true, "outer_only", 45.0, 0.0, 10.0, 0.0, 2,
                 "leading_brim_length", "Arrange does not reserve that space", false},
        BrimCase{"leading-edge-only brim on a fixed bed", false, "leading_edge_only", 0.0, 0.0, 0.0, 0.0, 1,
                 "brim_type", "applies to belt printers", true},
        BrimCase{"ordinary brim on a properly tilted belt", true, "outer_only", 45.0, 5.0, 0.0, 0.0, 1,
                 "", "", false});
    CAPTURE(part.name, part.belt_printer, part.brim_type, part.belt_angle,
            part.brim_width, part.leading_brim_length, part.brim_object_gap, part.object_count);

    DynamicPrintConfig config = validation_contract_config();
    config.set_deserialize_strict({
        {"belt_printer", part.belt_printer},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", part.belt_angle},
        {"brim_type", part.brim_type},
        {"brim_width", part.brim_width},
        {"leading_brim_length", part.leading_brim_length},
        {"brim_object_gap", part.brim_object_gap},
    });

    Print print;
    Model model;
    build_cubes(model, print, config, part.object_count, false);
    REQUIRE(print.objects().size() == size_t(part.object_count));
    std::vector<StringObjectException> warnings;
    const StringObjectException error = print.validate(&warnings);
    REQUIRE(error.string.empty());

    if (part.expected_key[0] == '\0') {
        CHECK(std::none_of(warnings.begin(), warnings.end(), [](const StringObjectException &warning) {
            return warning.opt_key == "brim_type" || warning.opt_key == "brim_width" ||
                   warning.opt_key == "brim_object_gap" || warning.opt_key == "leading_brim_length";
        }));
        return;
    }

    const auto warning = std::find_if(warnings.begin(), warnings.end(), [&part](const StringObjectException &entry) {
        return entry.opt_key == part.expected_key;
    });
    REQUIRE(warning != warnings.end());
    CHECK(warning->is_warning);
    CHECK(warning->string.find(part.expected_message) != std::string::npos);
    CHECK((warning->object != nullptr) == part.expected_model_object);
    if (part.expected_model_object)
        CHECK(warning->object == print.objects().front()->model_object());
}
