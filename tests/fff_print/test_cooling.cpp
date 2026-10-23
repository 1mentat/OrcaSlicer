#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstddef>
#include <iterator>
#include <cmath>
#include <string_view>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/CoolingBuffer.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

// The fan is held off for the first close_fan_the_first_x_layers layers, so an explicit
// fan-off command is emitted.
TEST_CASE("Fan is held off for the initial layers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, {
        { "slow_down_for_layer_cooling", true },
        { "close_fan_the_first_x_layers", 5 },
    });
    CHECK(gcode.find("M106 S0") != std::string::npos);
}

// The cooling pass resolves and strips its internal speed placeholders; none leak into
// the final G-code.
TEST_CASE("Cooling consumes its internal speed markers", "[Cooling]")
{
    const std::string gcode = slice({ cube(20) }, { { "layer_height", 0.2 } });
    CHECK(gcode.find(";_EXTRUDE_SET_SPEED") == std::string::npos);
}

TEST_CASE("Overhang fan transitions do not depend on overhang speed", "[Cooling][Regression]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "bridge_speed",                   2.0 },
        { "enable_arc_fitting",             false },
        { "enable_overhang_bridge_fan",     true },
        { "enable_overhang_speed",          false },
        { "initial_layer_print_height",     0.3 },
        { "inner_wall_speed",              30.0 },
        { "layer_height",                   0.3 },
        { "outer_wall_speed",              30.0 },
        { "overhang_1_4_speed",            "30" },
        { "overhang_2_4_speed",            "29" },
        { "overhang_3_4_speed",             "6" },
        { "overhang_4_4_speed",             "3" },
        { "slow_down_for_layer_cooling",    false },
        { "slowdown_for_curled_perimeters", false },
    });
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    config.set_key_value("layer_change_gcode", new ConfigOptionString{";TEST_LAYER_Z=[layer_z]"});

    const auto fan_commands = [](const std::string &gcode) {
        std::vector<std::pair<std::string, std::string>> commands;
        std::istringstream input(gcode);
        std::string layer;
        std::string line;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER_Z=", 0) == 0)
                layer = line;
            else if (!layer.empty() && (line.rfind("M106", 0) == 0 || line.rfind("M107", 0) == 0))
                commands.emplace_back(layer, line);
        }
        return commands;
    };
    const auto feedrates = [](const std::string &gcode) {
        std::vector<std::string> values;
        std::istringstream input(gcode);
        std::string word;
        while (input >> word)
            if (!word.empty() && word.front() == 'F')
                values.push_back(word);
        return values;
    };

    constexpr double sphere_radius = 50.0; // 100 mm diameter.
    const std::string without_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);
    config.set_deserialize_strict({{"enable_overhang_speed", true}});
    const std::string with_speed_gcode = slice({make_sphere(sphere_radius, PI / 24.0)}, config);

    const auto without_speed_fan = fan_commands(without_speed_gcode);
    const auto with_speed_fan = fan_commands(with_speed_gcode);
    const auto without_speed_feedrates = feedrates(without_speed_gcode);
    const auto with_speed_feedrates = feedrates(with_speed_gcode);

    REQUIRE_FALSE(without_speed_fan.empty());
    REQUIRE(std::any_of(without_speed_fan.begin(), without_speed_fan.end(),
                        [](const auto &command) { return command.second.find("S255") != std::string::npos; }));
    REQUIRE(with_speed_feedrates != without_speed_feedrates);
    CHECK(with_speed_fan == without_speed_fan);
}

namespace cooling_slowdown_contracts {

struct FeedEvidence {
    std::vector<double> feeds;
    double path_length = 0.0;
    double filament_length = 0.0;
    double final_x = 0.0;
    double final_y = 0.0;
    bool has_final_coordinates = false;
    bool finite = true;
};

DynamicPrintConfig slowdown_config(const bool enabled, const double layer_time, const double min_speed)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"minimum_sparse_infill_area", 0.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"enable_arc_fitting", false},
        {"gcode_comments", true},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"slow_down_for_layer_cooling", enabled},
        {"slow_down_layer_time", layer_time},
        {"slow_down_min_speed", min_speed},
        {"slow_down_layers", 0},
        {"fan_cooling_layer_time", 1000.0},
        {"fan_min_speed", 0},
        {"fan_max_speed", 0},
        {"close_fan_the_first_x_layers", 0},
        {"full_fan_speed_layer", 0},
        {"dont_slow_down_outer_wall", false},
        {"slowdown_for_curled_perimeters", false},
        {"part_cooling_fan_min_pwm", 0},
        {"initial_layer_speed", 60.0},
        {"outer_wall_speed", 60.0},
        {"inner_wall_speed", 60.0},
        {"sparse_infill_speed", 60.0},
        {"top_surface_speed", 60.0},
        {"travel_speed", 120.0},
        {"retraction_length", 0.8},
        {"retraction_minimum_travel", 0.0},
    });
    return config;
}

FeedEvidence inspect_feed_evidence(const std::string &gcode, const DynamicPrintConfig &config)
{
    FeedEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&evidence](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if ((line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) &&
            (line.has_x() || line.has_y())) {
            evidence.final_x = line.new_X(self);
            evidence.final_y = line.new_Y(self);
            evidence.has_final_coordinates = true;
            evidence.finite = evidence.finite && std::isfinite(evidence.final_x) && std::isfinite(evidence.final_y);
        }
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        const double feed = line.has_f() ? line.f() : self.f();
        const double path = line.dist_XY(self);
        const double filament = line.dist_E(self);
        evidence.feeds.push_back(feed);
        evidence.path_length += path;
        evidence.filament_length += filament;
        evidence.finite = evidence.finite && std::isfinite(feed) && feed > 0.0 && std::isfinite(path) && std::isfinite(filament);
    });
    return evidence;
}

void require_same_deposition(const FeedEvidence &expected, const FeedEvidence &actual)
{
    REQUIRE_FALSE(expected.feeds.empty());
    REQUIRE_FALSE(actual.feeds.empty());
    REQUIRE(expected.finite);
    REQUIRE(actual.finite);
    REQUIRE(expected.has_final_coordinates);
    REQUIRE(actual.has_final_coordinates);
    CHECK(actual.feeds.size() == expected.feeds.size());
    CHECK_THAT(actual.path_length, Catch::Matchers::WithinRel(expected.path_length, 1e-6));
    CHECK_THAT(actual.filament_length, Catch::Matchers::WithinAbs(expected.filament_length, 1e-3));
    CHECK_THAT(actual.final_x, Catch::Matchers::WithinAbs(expected.final_x, 1e-3));
    CHECK_THAT(actual.final_y, Catch::Matchers::WithinAbs(expected.final_y, 1e-3));
}

double minimum_feed(const FeedEvidence &evidence)
{
    return *std::min_element(evidence.feeds.begin(), evidence.feeds.end());
}

} // namespace cooling_slowdown_contracts

TEST_CASE("Layer cooling settings slow extrusion without changing deposited geometry",
          "[Cooling][SlowDownSettings]")
{
    using namespace cooling_slowdown_contracts;

    const DynamicPrintConfig disabled_config = slowdown_config(false, 1000.0, 5.0);
    const DynamicPrintConfig no_time_config = slowdown_config(true, 0.0, 5.0);
    const DynamicPrintConfig slow_five_config = slowdown_config(true, 1000.0, 5.0);
    const DynamicPrintConfig slow_one_config = slowdown_config(true, 1000.0, 1.0);
    const DynamicPrintConfig slow_zero_config = slowdown_config(true, 1000.0, 0.0);

    const FeedEvidence disabled = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, disabled_config), disabled_config);
    const FeedEvidence no_time = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, no_time_config), no_time_config);
    const FeedEvidence slow_five = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, slow_five_config), slow_five_config);
    const FeedEvidence slow_one = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, slow_one_config), slow_one_config);
    const FeedEvidence slow_zero = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, slow_zero_config), slow_zero_config);

    require_same_deposition(disabled, no_time);
    require_same_deposition(disabled, slow_five);
    require_same_deposition(disabled, slow_one);
    require_same_deposition(disabled, slow_zero);

    // The boolean gate is isolated by disabled versus the otherwise identical 1000 s case.
    REQUIRE(minimum_feed(slow_five) + 1.0 < minimum_feed(disabled));
    // With the gate enabled, a zero threshold retains the disabled control's feed rate.
    CHECK_THAT(minimum_feed(no_time), Catch::Matchers::WithinAbs(minimum_feed(disabled), 1e-3));
    REQUIRE(minimum_feed(slow_five) + 1.0 < minimum_feed(no_time));
    // With the gate and threshold held fixed, the configured minimum feed remains observable.
    REQUIRE(minimum_feed(slow_one) + 1.0 < minimum_feed(slow_five));
    CHECK_THAT(minimum_feed(slow_five), Catch::Matchers::WithinAbs(5.0 * 60.0, 0.1));
    CHECK_THAT(minimum_feed(slow_one), Catch::Matchers::WithinAbs(1.0 * 60.0, 0.1));

    // The registered lower bound is zero. At the same 1000 s target, emitted
    // deposition remains finite and positive while falling below 1 mm/s.
    REQUIRE(minimum_feed(slow_zero) > 0.0);
    CHECK(minimum_feed(slow_zero) < 1.0 * 60.0);
}

namespace cooling_settings_followup {

struct Evidence
{
    std::vector<double> outer_feeds;
    std::vector<double> inner_feeds;
    std::vector<int> part_fan_states;
    double xy_path = 0.0;
    double filament = 0.0;
    bool finite = true;
    bool fan_seen = false;
    size_t deposition_moves = 0;
    size_t unknown_role_moves = 0;
};

DynamicPrintConfig config_for(const bool dont_slow_outer_wall,
                              const bool reduce_fan_stop_start_freq,
                              const bool slowdown)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"minimum_sparse_infill_area", 0.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"enable_arc_fitting", false},
        {"gcode_comments", true},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"slow_down_for_layer_cooling", slowdown},
        {"slow_down_layer_time", slowdown ? 1000.0 : 0.0},
        {"slow_down_min_speed", 5.0},
        {"slow_down_layers", 0},
        {"slowdown_for_curled_perimeters", false},
        {"dont_slow_down_outer_wall", dont_slow_outer_wall},
        {"outer_wall_speed", 60.0},
        {"inner_wall_speed", 60.0},
        {"initial_layer_speed", 60.0},
        {"initial_layer_infill_speed", 60.0},
        {"sparse_infill_speed", 60.0},
        {"travel_speed", 120.0},
        {"small_perimeter_speed", "0"},
        {"small_perimeter_threshold", 0.0},
        {"fan_cooling_layer_time", 0.0},
        {"fan_min_speed", slowdown ? 0 : 30},
        {"fan_max_speed", slowdown ? 0 : 30},
        {"close_fan_the_first_x_layers", 0},
        {"full_fan_speed_layer", 0},
        {"initial_layer_fan_speed", -1},
        {"reduce_fan_stop_start_freq", reduce_fan_stop_start_freq},
        {"part_cooling_fan_min_pwm", 0},
    });
    return config;
}

Evidence inspect(const std::string &gcode, const DynamicPrintConfig &config)
{
    Evidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    std::string active_role;
    int part_fan = -1;
    reader.parse_buffer(gcode, [&evidence, &active_role, &part_fan](GCodeReader &self,
                                                                    const GCodeReader::GCodeLine &line) {
        const std::string comment(line.comment());
        if (comment.rfind("TYPE:", 0) == 0)
            active_role = comment.substr(5);
        else if (comment.rfind("FEATURE:", 0) == 0)
            active_role = comment.substr(8);

        if (line.cmd_is("M107") && !line.has('P')) {
            part_fan = 0;
            evidence.fan_seen = true;
        } else if (line.cmd_is("M106") && !line.has('P')) {
            float pwm = 0.0f;
            REQUIRE(line.has_value('S', pwm));
            REQUIRE(std::isfinite(pwm));
            REQUIRE(pwm >= 0.0f);
            REQUIRE(pwm <= 255.0f);
            part_fan = static_cast<int>(std::lround(pwm));
            evidence.fan_seen = true;
        }

        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        ++evidence.deposition_moves;
        const double xy = line.dist_XY(self);
        const double filament = line.dist_E(self);
        const double feed = line.has_f() ? line.f() : self.f();
        evidence.finite = evidence.finite && std::isfinite(xy) && std::isfinite(filament) &&
                          std::isfinite(feed) && xy > 0.0 && filament > 0.0 && feed > 0.0;
        if (!evidence.finite)
            return;
        evidence.xy_path += xy;
        evidence.filament += filament;
        if (active_role == "Outer wall")
            evidence.outer_feeds.push_back(feed);
        else if (active_role == "Inner wall")
            evidence.inner_feeds.push_back(feed);
        else
            ++evidence.unknown_role_moves;
        if (part_fan >= 0)
            evidence.part_fan_states.push_back(part_fan);
    });
    return evidence;
}

void require_same_material(const Evidence &reference, const Evidence &observed)
{
    REQUIRE(reference.finite);
    REQUIRE(observed.finite);
    REQUIRE(reference.deposition_moves > 0);
    REQUIRE(observed.deposition_moves > 0);
    REQUIRE(reference.unknown_role_moves == 0);
    REQUIRE(observed.unknown_role_moves == 0);
    REQUIRE(reference.part_fan_states.size() == reference.deposition_moves);
    REQUIRE(observed.part_fan_states.size() == observed.deposition_moves);
    REQUIRE(reference.outer_feeds.size() + reference.inner_feeds.size() == reference.deposition_moves);
    REQUIRE(observed.outer_feeds.size() + observed.inner_feeds.size() == observed.deposition_moves);
    REQUIRE_FALSE(reference.outer_feeds.empty());
    REQUIRE_FALSE(reference.inner_feeds.empty());
    REQUIRE_FALSE(observed.outer_feeds.empty());
    REQUIRE_FALSE(observed.inner_feeds.empty());
    CHECK_THAT(observed.xy_path, Catch::Matchers::WithinRel(reference.xy_path, 1e-6));
    CHECK_THAT(observed.filament, Catch::Matchers::WithinAbs(reference.filament, 1e-3));
    CHECK(observed.outer_feeds.size() == reference.outer_feeds.size());
    CHECK(observed.inner_feeds.size() == reference.inner_feeds.size());
}

double minimum(const std::vector<double> &values)
{
    return *std::min_element(values.begin(), values.end());
}

} // namespace cooling_settings_followup

TEST_CASE("Outer wall slowdown can be disabled without changing deposited material",
          "[Cooling][CoolingSettings][CoolingDontOuterWall]")
{
    using namespace cooling_settings_followup;
    const DynamicPrintConfig slowdown_config = config_for(false, false, true);
    const DynamicPrintConfig preserved_config = config_for(true, false, true);
    const Evidence slowdown = inspect(slice({make_cube(12.0, 12.0, 1.2)}, slowdown_config), slowdown_config);
    const Evidence preserved = inspect(slice({make_cube(12.0, 12.0, 1.2)}, preserved_config), preserved_config);

    require_same_material(slowdown, preserved);
    REQUIRE(slowdown.part_fan_states.size() > 0);
    REQUIRE(preserved.part_fan_states.size() > 0);
    CHECK(std::all_of(slowdown.part_fan_states.begin(), slowdown.part_fan_states.end(), [](int pwm) { return pwm == 0; }));
    CHECK(std::all_of(preserved.part_fan_states.begin(), preserved.part_fan_states.end(), [](int pwm) { return pwm == 0; }));

    const double slowed_outer = minimum(slowdown.outer_feeds);
    const double preserved_outer = minimum(preserved.outer_feeds);
    REQUIRE(slowed_outer > 0.0);
    REQUIRE(preserved_outer > slowed_outer + 300.0);
    CHECK(preserved_outer > slowed_outer * 2.0);

    // Inner walls remain subject to the same cooling slowdown in both rows.
    CHECK_THAT(minimum(preserved.inner_feeds), Catch::Matchers::WithinAbs(minimum(slowdown.inner_feeds), 0.25));
}

TEST_CASE("Keeping the cooling fan running preserves a positive fan during deposition",
          "[Cooling][CoolingSettings][CoolingFanMinHold]")
{
    using namespace cooling_settings_followup;
    const DynamicPrintConfig stop_config = config_for(false, false, false);
    const DynamicPrintConfig keep_config = config_for(false, true, false);
    const Evidence stop = inspect(slice({make_cube(8.0, 8.0, 0.8)}, stop_config), stop_config);
    const Evidence keep = inspect(slice({make_cube(8.0, 8.0, 0.8)}, keep_config), keep_config);

    require_same_material(stop, keep);
    REQUIRE(stop.fan_seen);
    REQUIRE(keep.fan_seen);
    REQUIRE_FALSE(stop.part_fan_states.empty());
    REQUIRE_FALSE(keep.part_fan_states.empty());
    CHECK(std::all_of(stop.part_fan_states.begin(), stop.part_fan_states.end(), [](int pwm) { return pwm == 0; }));
    CHECK(std::all_of(keep.part_fan_states.begin(), keep.part_fan_states.end(), [](int pwm) { return pwm > 0; }));
    CHECK_THAT(minimum(keep.inner_feeds), Catch::Matchers::WithinAbs(minimum(stop.inner_feeds), 0.25));
}


namespace cooling_role_fan_settings_contracts {

struct RolePaths {
    size_t internal_bridge = 0;
    size_t support_interface = 0;
    size_t ironing = 0;
};

TriangleMesh support_fixture()
{
    // Same real support-producing base/stem/cap shape used by the public G-code support test.
    TriangleMesh base = make_cube(40., 40., 2.);
    TriangleMesh stem = make_cube(8., 8., 12.);
    stem.translate(Vec3f(16.f, 16.f, 1.f));
    TriangleMesh cap = make_cube(40., 40., 2.);
    cap.translate(Vec3f(0.f, 0.f, 12.f));
    base.merge(stem);
    base.merge(cap);
    return base;
}

TriangleMesh internal_bridge_fixture()
{
    // Same stepped solid whose public bridge test observes erInternalBridgeInfill paths.
    TriangleMesh mesh = make_cube(30., 24., 3.);
    TriangleMesh tower = make_cube(14., 10., 1.);
    tower.translate(Vec3f(8.f, 7.f, 3.f));
    mesh.merge(tower);
    return mesh;
}

TriangleMesh role_fixture()
{
    TriangleMesh mesh = support_fixture();
    TriangleMesh bridge = internal_bridge_fixture();
    // Two separated islands in one object retain all fan roles without introducing
    // unrelated cross-object collision checks into every fan-settings export.
    bridge.translate(Vec3f(60.f, 0.f, 0.f));
    mesh.merge(bridge);
    return mesh;
}

DynamicPrintConfig config_for_roles(const int internal_bridge_speed,
                                   const int support_interface_speed,
                                   const int ironing_speed,
                                   const int minimum_pwm)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"bridge_no_support", false},
        {"bridge_speed", 21.0},
        {"close_fan_the_first_x_layers", 0},
        {"enable_arc_fitting", false},
        {"enable_overhang_bridge_fan", true},
        {"enable_support", true},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"fan_cooling_layer_time", 0.0},
        {"fan_kickstart", 0.0},
        {"fan_speedup_time", 0.0},
        {"fan_speedup_overhangs", false},
        {"full_fan_speed_layer", 0},
        {"initial_layer_fan_speed", -1},
        {"initial_layer_print_height", 0.3},
        {"ironing_flow", "10%"},
        {"ironing_inset", 0.0},
        {"ironing_pattern", "rectilinear"},
        {"ironing_spacing", 0.2},
        {"ironing_speed", 20.0},
        {"ironing_type", "top"},
        {"layer_height", 0.3},
        {"minimum_sparse_infill_area", 0.0},
        {"part_cooling_fan_min_pwm", minimum_pwm},
        {"reduce_fan_stop_start_freq", true},
        // Fan-role checks do not depend on visibility-based seam placement.
        {"seam_position", "back"},
        {"slow_down_for_layer_cooling", false},
        {"slowdown_for_curled_perimeters", false},
        {"sparse_infill_density", "15%"},
        {"support_base_pattern", "rectilinear"},
        {"support_interface_bottom_layers", 2},
        {"support_interface_pattern", "rectilinear"},
        {"support_interface_top_layers", 2},
        {"support_on_build_plate_only", false},
        {"support_type", "normal(auto)"},
        {"thick_internal_bridges", false},
        {"top_shell_layers", 3},
        {"wall_loops", 1},
    });
    // Fan option arrays are per filament. Zero is a deliberate off control; the three
    // role speeds are independently observed in the emitted part-fan commands.
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{0.0});
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{0.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{0});
    config.set_key_value("internal_bridge_fan_speed", new ConfigOptionInts{internal_bridge_speed});
    config.set_key_value("support_material_interface_fan_speed", new ConfigOptionInts{support_interface_speed});
    config.set_key_value("ironing_fan_speed", new ConfigOptionInts{ironing_speed});
    return config;
}

RolePaths count_role_paths(const Print &print)
{
    RolePaths paths;
    const auto count = [&paths](const ExtrusionPath &path) {
        if (path.role() == erInternalBridgeInfill)
            ++paths.internal_bridge;
        else if (path.role() == erSupportMaterialInterface)
            ++paths.support_interface;
        else if (path.role() == erIroning)
            ++paths.ironing;
    };
    for (const PrintObject *object : print.objects()) {
        for (const Layer *layer : object->layers()) {
            for (const LayerRegion *region : layer->regions()) {
                for_each_extrusion_path(region->perimeters, count);
                for_each_extrusion_path(region->fills, count);
            }
        }
        for (const SupportLayer *layer : object->support_layers())
            for_each_extrusion_path(layer->support_fills, count);
    }
    return paths;
}

struct RoleExport {
    std::string gcode;
    RolePaths paths;
};

RoleExport export_role_fixture(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    Test::init_print(std::vector<TriangleMesh>{role_fixture()},
                     print, model, config, nullptr, true);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.set_status_silent();
    // Test::gcode calls Print::process once before export. Export first so this helper
    // captures the processed paths without doing a redundant second process pass.
    std::string output = Test::gcode(print);
    const RolePaths paths = count_role_paths(print);
    // Assert that all three user-visible feature roles exist in the sliced geometry before
    // interpreting any emitted fan PWM as evidence for a role-specific setting.
    REQUIRE(paths.internal_bridge > 0);
    REQUIRE(paths.support_interface > 0);
    REQUIRE(paths.ironing > 0);
    return {std::move(output), paths};
}

template <class ConfigType>
std::vector<int> part_fan_commands(const std::string &gcode, const ConfigType &config)
{
    std::vector<int> commands;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&commands](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M107") && !line.has_p()) {
            commands.emplace_back(0);
            return;
        }
        if (!line.cmd_is("M106") || line.has_p())
            return;
        float pwm = 0.0f;
        REQUIRE(line.has_value('S', pwm));
        REQUIRE(std::isfinite(pwm));
        REQUIRE(pwm >= 0.0f);
        REQUIRE(pwm <= 255.0f);
        CHECK_THAT(pwm, Catch::Matchers::WithinAbs(std::round(pwm), 1e-6f));
        commands.emplace_back(static_cast<int>(std::lround(pwm)));
    });
    return commands;
}

void require_role_pwm_then_off(const std::vector<int> &commands, const int pwm)
{
    const auto target = std::find(commands.begin(), commands.end(), pwm);
    REQUIRE(target != commands.end());
    const auto after = std::next(target);
    REQUIRE(after != commands.end());
    CHECK(*after == 0);
}

} // namespace cooling_role_fan_settings_contracts

TEST_CASE("Cooling role fan settings follow their printed path roles",
          "[Cooling][CoolingSettings][RoleFanSettings]")
{
    using namespace cooling_role_fan_settings_contracts;
    const DynamicPrintConfig disabled_config = config_for_roles(-1, -1, -1, 0);
    const DynamicPrintConfig active_config = config_for_roles(40, 60, 80, 0);
    const DynamicPrintConfig floored_config = config_for_roles(40, 60, 80, 50);

    const RoleExport disabled = export_role_fixture(disabled_config);
    const RoleExport active = export_role_fixture(active_config);
    const RoleExport floored = export_role_fixture(floored_config);
    const std::vector<int> disabled_fan = part_fan_commands(disabled.gcode, disabled_config);
    const std::vector<int> active_fan = part_fan_commands(active.gcode, active_config);
    const std::vector<int> floored_fan = part_fan_commands(floored.gcode, floored_config);

    REQUIRE_FALSE(active_fan.empty());
    REQUIRE_FALSE(floored_fan.empty());
    CHECK(std::none_of(disabled_fan.begin(), disabled_fan.end(), [](int pwm) { return pwm > 0; }));
    // 40/60/80 percent role targets map to S102/S153/S204. Each target must be
    // followed by the configured off state after its role marker ends.
    require_role_pwm_then_off(active_fan, 102);
    require_role_pwm_then_off(active_fan, 153);
    require_role_pwm_then_off(active_fan, 204);
    // A 50 percent floor clamps only the 40 percent internal-bridge request; zero
    // remains off, while the 60 and 80 percent roles remain above the floor.
    require_role_pwm_then_off(floored_fan, 127);
    require_role_pwm_then_off(floored_fan, 153);
    require_role_pwm_then_off(floored_fan, 204);
    CHECK(std::find(floored_fan.begin(), floored_fan.end(), 102) == floored_fan.end());

    CHECK(active.paths.internal_bridge == disabled.paths.internal_bridge);
    CHECK(active.paths.support_interface == disabled.paths.support_interface);
    CHECK(active.paths.ironing == disabled.paths.ironing);
    CHECK(floored.paths.internal_bridge == active.paths.internal_bridge);
    CHECK(floored.paths.support_interface == active.paths.support_interface);
    CHECK(floored.paths.ironing == active.paths.ironing);

    const auto disabled_material = cooling_slowdown_contracts::inspect_feed_evidence(disabled.gcode, disabled_config);
    const auto active_material = cooling_slowdown_contracts::inspect_feed_evidence(active.gcode, active_config);
    const auto floored_material = cooling_slowdown_contracts::inspect_feed_evidence(floored.gcode, floored_config);
    REQUIRE(disabled_material.finite);
    REQUIRE(active_material.finite);
    REQUIRE(floored_material.finite);
    REQUIRE_FALSE(disabled_material.feeds.empty());
    CHECK_THAT(active_material.path_length, Catch::Matchers::WithinRel(disabled_material.path_length, 1e-6));
    CHECK_THAT(active_material.filament_length, Catch::Matchers::WithinAbs(disabled_material.filament_length, 1e-3));
    CHECK_THAT(floored_material.path_length, Catch::Matchers::WithinRel(active_material.path_length, 1e-6));
    CHECK_THAT(floored_material.filament_length, Catch::Matchers::WithinAbs(active_material.filament_length, 1e-3));
}

namespace cooling_role_fan_boundary_contracts {

using namespace cooling_role_fan_settings_contracts;

enum class RoleChannel { other, internal_bridge, support_interface, ironing };

struct RoleFanSamples {
    std::vector<int> internal_bridge;
    std::vector<int> support_interface;
    std::vector<int> ironing;
};

struct RoleFixture {
    Print print;
    Model model;
    bool initialized = false;

    RoleExport export_config(const DynamicPrintConfig &config)
    {
        if (!initialized) {
            Test::init_print(std::vector<TriangleMesh>{role_fixture()},
                             print, model, config, nullptr, true);
            initialized = true;
        } else {
            print.apply(model, config);
        }
        const auto validation = print.validate();
        INFO(validation.string);
        REQUIRE(validation.string.empty());

        // This helper performs exactly one Print::process call. These settings only
        // affect cooling/G-code output, so subsequent calls reuse the same sliced model.
        std::string output = Test::gcode(print);
        const RolePaths paths = count_role_paths(print);
        REQUIRE(paths.internal_bridge > 0);
        REQUIRE(paths.support_interface > 0);
        REQUIRE(paths.ironing > 0);
        return {std::move(output), paths};
    }
};

DynamicPrintConfig config_for_boundaries(const int internal_bridge_speed,
                                         const int support_interface_speed,
                                         const int ironing_speed,
                                         const int minimum_pwm)
{
    DynamicPrintConfig config = config_for_roles(internal_bridge_speed, support_interface_speed,
                                                 ironing_speed, minimum_pwm);
    // A positive base and overhang speed make the -1 sentinels observably distinct from 0.
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{30});
    return config;
}

RoleFanSamples role_fan_samples(const std::string &gcode, const DynamicPrintConfig &config)
{
    RoleFanSamples samples;
    RoleChannel active_role = RoleChannel::other;
    int part_fan_pwm = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&samples, &active_role, &part_fan_pwm](GCodeReader &self,
                                                                       const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("TYPE:") != std::string_view::npos ||
            comment.find("FEATURE:") != std::string_view::npos) {
            std::string normalized(comment);
            std::transform(normalized.begin(), normalized.end(), normalized.begin(), [](unsigned char ch) {
                return static_cast<char>(std::tolower(ch));
            });
            active_role = RoleChannel::other;
            if (normalized.find("internal bridge") != std::string::npos)
                active_role = RoleChannel::internal_bridge;
            else if (normalized.find("support interface") != std::string::npos ||
                     normalized.find("support material interface") != std::string::npos)
                active_role = RoleChannel::support_interface;
            else if (normalized.find("ironing") != std::string::npos)
                active_role = RoleChannel::ironing;
        }

        if (line.cmd_is("M107") && !line.has_p()) {
            part_fan_pwm = 0;
        } else if (line.cmd_is("M106") && !line.has_p()) {
            float pwm = 0.0f;
            REQUIRE(line.has_value('S', pwm));
            REQUIRE(std::isfinite(pwm));
            REQUIRE(pwm >= 0.0f);
            REQUIRE(pwm <= 255.0f);
            part_fan_pwm = static_cast<int>(std::lround(pwm));
            CHECK_THAT(pwm, Catch::Matchers::WithinAbs(static_cast<float>(part_fan_pwm), 1e-6f));
        }

        // Observe the effective state at the toolpath, rather than associating a fan
        // command with whichever role comment happened to precede it. This includes
        // values carried forward when CoolingBuffer suppresses a redundant M106 and
        // keeps role-end resets from being mistaken for that role's extrusion speed.
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        if (active_role == RoleChannel::internal_bridge)
            samples.internal_bridge.push_back(part_fan_pwm);
        else if (active_role == RoleChannel::support_interface)
            samples.support_interface.push_back(part_fan_pwm);
        else if (active_role == RoleChannel::ironing)
            samples.ironing.push_back(part_fan_pwm);
    });
    return samples;
}

int expected_pwm(const int speed, const int floor)
{
    const int effective = speed == 0 ? 0 : std::max(speed, floor);
    switch (effective) {
    case 0:   return 0;
    case 1:   return 2;
    case 20:  return 51;
    case 30:  return 76;
    case 99:  return 252;
    case 100: return 255;
    default:  FAIL("the fan boundary table only uses independently enumerated PWM points");
    }
    return -1;
}

void require_pwm_samples(const std::vector<int> &samples, const int expected)
{
    INFO("Expected every positive-XY role extrusion to use part-fan PWM S" << expected);
    REQUIRE_FALSE(samples.empty());
    CHECK(std::all_of(samples.begin(), samples.end(), [expected](const int pwm) { return pwm == expected; }));
}

void require_same_paths(const RolePaths &expected, const RolePaths &actual)
{
    CHECK(actual.internal_bridge == expected.internal_bridge);
    CHECK(actual.support_interface == expected.support_interface);
    CHECK(actual.ironing == expected.ironing);
}

void require_same_role_samples(const RoleFanSamples &expected, const RoleFanSamples &actual)
{
    CHECK(actual.internal_bridge == expected.internal_bridge);
    CHECK(actual.support_interface == expected.support_interface);
    CHECK(actual.ironing == expected.ironing);
}

} // namespace cooling_role_fan_boundary_contracts

TEST_CASE("Cooling role fan arrays cover their endpoints through cached and fresh prints",
          "[Cooling][CoolingSettings][RoleFanBoundaries]")
{
    using namespace cooling_role_fan_boundary_contracts;

    struct Row {
        int internal_bridge;
        int support_interface;
        int ironing;
        int minimum_pwm;
    };
    constexpr std::array<Row, 6> active_rows{{
        {0,   1,  99,   0},
        {1,  99, 100,   1},
        {99, 100,  0,   0},
        {100,  0,  1,   0},
        {0,   1,  99,  99},
        {0,   1,  99, 100},
    }};

    RoleFixture incremental;
    const DynamicPrintConfig defaults = config_for_boundaries(-1, -1, -1, 0);
    const RoleExport baseline = incremental.export_config(defaults);
    const RoleFanSamples baseline_roles = role_fan_samples(baseline.gcode, defaults);
    const std::vector<int> baseline_all_fans = part_fan_commands(baseline.gcode, defaults);
    REQUIRE(std::find(baseline_all_fans.begin(), baseline_all_fans.end(), 51) != baseline_all_fans.end());
    // Internal-bridge -1 falls back to the positive overhang speed. The other -1 roles
    // leave the ordinary 20% fan command in effect instead of forcing an explicit off.
    require_pwm_samples(baseline_roles.internal_bridge, 76);
    require_pwm_samples(baseline_roles.support_interface, 51);
    require_pwm_samples(baseline_roles.ironing, 51);

    const auto baseline_material = cooling_slowdown_contracts::inspect_feed_evidence(baseline.gcode, defaults);
    REQUIRE(baseline_material.finite);
    REQUIRE_FALSE(baseline_material.feeds.empty());

    RoleFanSamples fresh_control_roles;
    DynamicPrintConfig fresh_control_config = config_for_boundaries(1, 99, 100, 1);
    for (size_t i = 0; i < active_rows.size(); ++i) {
        const Row &row = active_rows[i];
        CAPTURE(i, row.internal_bridge, row.support_interface, row.ironing, row.minimum_pwm);
        const DynamicPrintConfig config = config_for_boundaries(row.internal_bridge, row.support_interface,
                                                                row.ironing, row.minimum_pwm);
        const RoleExport current = incremental.export_config(config);
        const RoleFanSamples observed = role_fan_samples(current.gcode, config);
        require_same_paths(baseline.paths, current.paths);
        const auto material = cooling_slowdown_contracts::inspect_feed_evidence(current.gcode, config);
        cooling_slowdown_contracts::require_same_deposition(baseline_material, material);

        require_pwm_samples(observed.internal_bridge, expected_pwm(row.internal_bridge, row.minimum_pwm));
        require_pwm_samples(observed.support_interface, expected_pwm(row.support_interface, row.minimum_pwm));
        require_pwm_samples(observed.ironing, expected_pwm(row.ironing, row.minimum_pwm));
        if (i == 1) {
            fresh_control_roles = observed;
        }
    }

    // The endpoint row is also generated from a new Print; this checks that apply()'s
    // cached G-code-only invalidation agrees with a clean process for the same settings.
    RoleFixture fresh;
    const RoleExport fresh_control = fresh.export_config(fresh_control_config);
    require_same_paths(baseline.paths, fresh_control.paths);
    require_same_role_samples(fresh_control_roles,
                              role_fan_samples(fresh_control.gcode, fresh_control_config));
    const auto fresh_material = cooling_slowdown_contracts::inspect_feed_evidence(fresh_control.gcode,
                                                                                   fresh_control_config);
    cooling_slowdown_contracts::require_same_deposition(baseline_material, fresh_material);

    const RoleExport restored = incremental.export_config(defaults);
    require_same_paths(baseline.paths, restored.paths);
    require_same_role_samples(baseline_roles, role_fan_samples(restored.gcode, defaults));
    const auto restored_material = cooling_slowdown_contracts::inspect_feed_evidence(restored.gcode, defaults);
    cooling_slowdown_contracts::require_same_deposition(baseline_material, restored_material);
}

TEST_CASE("Minimum part cooling fan PWM covers input endpoints and their nearest integer neighbors",
          "[Cooling][CoolingSettings][FanPwmBoundaryValues]")
{
    GCodeWriter writer;
    writer.config.gcode_flavor.value = gcfMarlinFirmware;
    struct Case { unsigned int floor; unsigned int request; int expected_pwm; };
    constexpr std::array<Case, 19> cases{{
        {0,   0,   0}, {0,   1,   2}, {0,  99, 252}, {0, 100, 255},
        {1,   0,   0}, {1,   1,   2}, {1,   2,   5},
        {99,  0,   0}, {99,  1, 252}, {99, 98, 252}, {99, 99, 252}, {99, 100, 255},
        {100, 0,   0}, {100, 1, 255}, {100, 99, 255}, {100, 100, 255},
        {40, 39, 102}, {40, 40, 102}, {40, 41, 104},
    }};
    for (const Case &test : cases) {
        CAPTURE(test.floor, test.request, test.expected_pwm);
        writer.config.part_cooling_fan_min_pwm.value = static_cast<int>(test.floor);
        const std::vector<int> observed = cooling_role_fan_settings_contracts::part_fan_commands(
            writer.set_fan(test.request), writer.config);
        REQUIRE(observed.size() == 1);
        CHECK(observed.front() == test.expected_pwm);
    }
}


namespace fan_speedup_overhangs_physical_contracts {

// A compact public stepped mesh: a 1.2 mm base, an overlapping 6x6 mm stem,
// and an 18x18x1.2 mm cap beginning at Z=3.6 mm. With 0.3 mm layers, the stem gives
// the ordinary fan target several complete layers to flush before the cap's
// wider unsupported perimeter requests the overhang target.
TriangleMesh stepped_overhang_fixture()
{
    TriangleMesh fixture = make_cube(18.0, 18.0, 1.2);
    TriangleMesh stem = make_cube(6.0, 6.0, 3.6);
    stem.translate(Vec3f(6.0f, 6.0f, 0.6f));
    TriangleMesh cap = make_cube(18.0, 18.0, 1.2);
    cap.translate(Vec3f(0.0f, 0.0f, 3.6f));
    fixture.merge(stem);
    fixture.merge(cap);
    return fixture;
}

struct FanOnEvent {
    double path_distance = 0.0;
    double elapsed_seconds = 0.0;
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    int pwm = 0;
    bool overhang = false;
    bool rise = false;
};

struct MotionEvidence {
    std::vector<FanOnEvent> fan_on;
    double path_distance = 0.0;
    double elapsed_seconds = 0.0;
    double deposited_xy_mm = 0.0;
    double deposited_filament_mm = 0.0;
    double full_fan_extrusion_xy_mm = 0.0;
    bool finite = true;
};

DynamicPrintConfig config_for(const bool speedup_overhangs)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"bridge_speed",                   2.0},
        {"enable_arc_fitting",             false},
        {"enable_overhang_bridge_fan",     true},
        {"enable_overhang_speed",          false},
        {"fan_cooling_layer_time",         0.0},
        {"fan_kickstart",                  0.0},
        {"reduce_fan_stop_start_freq",     true},
        {"fan_speedup_overhangs",          speedup_overhangs},
        {"fan_speedup_time",               0.5},
        {"gcode_comments",                 true},
        {"gcode_flavor",                   "marlin2"},
        {"initial_layer_print_height",     0.3},
        {"layer_change_gcode",             ";TEST_LAYER_Z=[layer_z]"},
        {"layer_height",                   0.3},
        {"slow_down_for_layer_cooling",    false},
        {"slowdown_for_curled_perimeters", false},
        {"close_fan_the_first_x_layers",   1},
        {"full_fan_speed_layer",           0},
        {"use_relative_e_distances",       true},
        {"wall_loops",                     1},
        {"top_shell_layers",               1},
        {"bottom_shell_layers",            0},
        {"top_surface_speed",              30.0},
        {"ironing_type",                   "top"},
        {"ironing_pattern",                "rectilinear"},
        {"ironing_flow",                   "10%"},
        {"ironing_speed",                  20.0},
        {"ironing_spacing",                0.2},
        {"ironing_inset",                  0.0},
        {"sparse_infill_density",          "0%"},
        {"minimum_sparse_infill_area",     0.0},
        {"enable_support",                 false},
    });
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4});
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("ironing_fan_speed", new ConfigOptionInts{40});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    return config;
}

MotionEvidence inspect_motion_and_fans(const std::string &gcode, const DynamicPrintConfig &config)
{
    MotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int previous_fan_pwm = 0;
    int part_fan_pwm = 0;
    reader.parse_buffer(gcode, [&evidence, &previous_fan_pwm, &part_fan_pwm](GCodeReader &self,
                                                   const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M107") && !line.has_p())
            part_fan_pwm = 0;
        if (line.cmd_is("M106") && !line.has_p()) {
            float pwm = 0.0f;
            REQUIRE(line.has_value('S', pwm));
            REQUIRE(std::isfinite(pwm));
            REQUIRE(pwm >= 0.0f);
            REQUIRE(pwm <= 255.0f);
            const int fan_pwm = static_cast<int>(std::lround(pwm));
            const bool rising = fan_pwm > previous_fan_pwm;
            previous_fan_pwm = fan_pwm;
            part_fan_pwm = fan_pwm;
            if (fan_pwm > 0) {
                // FanMover can relocate this command across buffered motion; classify by
                // pinned PWM targets rather than output TYPE comments. Ordinary fan is
                // 20% (S51), ironing is 40% (S102), and overhang is 100% (S255). Track rises explicitly
                // so an S51 fall from S255 cannot masquerade as the ordinary rise.
                const bool overhang = fan_pwm == 255;
                evidence.fan_on.push_back({evidence.path_distance, evidence.elapsed_seconds,
                                           self.x(), self.y(), self.z(), fan_pwm, overhang, rising});
            }
        }

        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;
        const double dx = line.dist_X(self);
        const double dy = line.dist_Y(self);
        const double dz = line.dist_Z(self);
        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (line.extruding(self)) {
            const double xy = line.dist_XY(self);
            evidence.deposited_xy_mm += xy;
            evidence.deposited_filament_mm += line.dist_E(self);
            if (part_fan_pwm == 255)
                evidence.full_fan_extrusion_xy_mm += xy;
        }
        if (!(distance > 0.0))
            return;
        evidence.path_distance += distance;
        const double feed = line.has_f() ? line.f() : self.f();
        if (std::isfinite(feed) && feed > 0.0)
            evidence.elapsed_seconds += distance * 60.0 / feed;
        evidence.finite = evidence.finite && std::isfinite(evidence.path_distance) &&
                          std::isfinite(evidence.elapsed_seconds) &&
                          std::isfinite(evidence.deposited_xy_mm) &&
                          std::isfinite(evidence.deposited_filament_mm) &&
                          std::isfinite(evidence.full_fan_extrusion_xy_mm);
    });
    return evidence;
}

const FanOnEvent &first_rising_fan_target(const MotionEvidence &evidence, const int target_pwm)
{
    std::ostringstream diagnostic;
    for (const FanOnEvent &event : evidence.fan_on)
        diagnostic << " pwm=" << event.pwm << " rise=" << event.rise
                   << " distance=" << event.path_distance << " time=" << event.elapsed_seconds;
    INFO("rising target=" << target_pwm << "; fan events:" << diagnostic.str());
    const auto found = std::find_if(evidence.fan_on.begin(), evidence.fan_on.end(),
        [target_pwm](const FanOnEvent &event) { return event.pwm == target_pwm && event.rise; });
    REQUIRE(found != evidence.fan_on.end());
    REQUIRE(found->path_distance > 0.0);
    return *found;
}

std::size_t count_overhang_fan_events(const MotionEvidence &evidence)
{
    return static_cast<size_t>(std::count_if(evidence.fan_on.begin(), evidence.fan_on.end(),
        [](const FanOnEvent &event) { return event.overhang; }));
}

} // namespace fan_speedup_overhangs_physical_contracts

TEST_CASE("Fan speedup overhang setting changes physical fan scheduling without changing deposition",
          "[Cooling][FanSpeedupOverhangs]")
{
    using namespace fan_speedup_overhangs_physical_contracts;
    const DynamicPrintConfig ordinary_config = config_for(false);
    const DynamicPrintConfig overhang_config = config_for(true);
    // Exposed top surfaces emit ironing after ordinary motion within a layer.
    // The wider cap also retains the independently pinned overhang fan target.
    const TriangleMesh fixture = stepped_overhang_fixture();

    const std::string ordinary_gcode = slice({fixture}, ordinary_config);
    const std::string overhang_gcode = slice({fixture}, overhang_config);
    const MotionEvidence ordinary = inspect_motion_and_fans(ordinary_gcode, ordinary_config);
    const MotionEvidence overhang = inspect_motion_and_fans(overhang_gcode, overhang_config);

    REQUIRE(ordinary.finite);
    REQUIRE(overhang.finite);
    REQUIRE(ordinary.path_distance > 0.0);
    REQUIRE(overhang.path_distance > 0.0);
    REQUIRE(ordinary.elapsed_seconds > 0.0);
    REQUIRE(overhang.elapsed_seconds > 0.0);
    REQUIRE_FALSE(ordinary.fan_on.empty());
    REQUIRE_FALSE(overhang.fan_on.empty());
    REQUIRE(count_overhang_fan_events(ordinary) > 0);
    REQUIRE(count_overhang_fan_events(overhang) > 0);
    REQUIRE(std::any_of(ordinary.fan_on.begin(), ordinary.fan_on.end(),
                        [](const FanOnEvent &event) { return !event.overhang && event.rise && event.path_distance > 0.0; }));
    // fan_speedup_overhangs intentionally leaves ordinary rises immediate;
    // the ordinary control still requires a rise after positive motion above.
    REQUIRE(std::any_of(overhang.fan_on.begin(), overhang.fan_on.end(),
                        [](const FanOnEvent &event) { return !event.overhang && event.rise; }));
    REQUIRE(std::any_of(ordinary.fan_on.begin(), ordinary.fan_on.end(),
                        [](const FanOnEvent &event) { return event.pwm == 255; }));
    REQUIRE(std::any_of(overhang.fan_on.begin(), overhang.fan_on.end(),
                        [](const FanOnEvent &event) { return event.pwm == 255; }));

    // Layer boundaries flush FanMover, so their initial S51 target has no earlier
    // buffered motion to advance into. The later ironing rise to 40% (S102) does.
    // Restricting speedup to overhangs leaves this non-overhang rise at its original
    // physical position; unrestricted speedup must emit it earlier in the motion.
    constexpr int ironing_fan_pwm = 102;
    const FanOnEvent &advanced = first_rising_fan_target(ordinary, ironing_fan_pwm);
    const FanOnEvent &unadvanced = first_rising_fan_target(overhang, ironing_fan_pwm);
    const double distance_advance = unadvanced.path_distance - advanced.path_distance;
    const double time_advance = unadvanced.elapsed_seconds - advanced.elapsed_seconds;
    INFO("ironing fan advance: distance=" << distance_advance << ", time=" << time_advance);
    REQUIRE(distance_advance > 0.05);
    REQUIRE(time_advance > 0.01);

    const cooling_slowdown_contracts::FeedEvidence ordinary_material =
        cooling_slowdown_contracts::inspect_feed_evidence(ordinary_gcode, ordinary_config);
    const cooling_slowdown_contracts::FeedEvidence overhang_material =
        cooling_slowdown_contracts::inspect_feed_evidence(overhang_gcode, overhang_config);
    REQUIRE_FALSE(ordinary_material.feeds.empty());
    REQUIRE_FALSE(overhang_material.feeds.empty());
    REQUIRE(ordinary_material.finite);
    REQUIRE(overhang_material.finite);
    // FanMover may split an extrusion to place a command within it. The physical
    // path and deposited filament must be conserved regardless of segment count.
    CHECK_THAT(overhang_material.path_length,
               Catch::Matchers::WithinRel(ordinary_material.path_length, 1e-6));
    CHECK_THAT(overhang_material.filament_length,
               Catch::Matchers::WithinAbs(ordinary_material.filament_length, 1e-3));
}

namespace overhang_fan_threshold_settings_contracts {

using fan_speedup_overhangs_physical_contracts::FanOnEvent;
using fan_speedup_overhangs_physical_contracts::MotionEvidence;

TriangleMesh partially_supported_cap_fixture()
{
    // The stem supports ten of the cap's sixteen millimeters from one side. This gives the
    // threshold endpoints both supported and unsupported perimeter spans.
    TriangleMesh fixture = make_cube(16.0, 16.0, 1.2);
    TriangleMesh stem = make_cube(10.0, 16.0, 3.6);
    stem.translate(Vec3f(0.0f, 0.0f, 0.6f));
    TriangleMesh cap = make_cube(16.0, 16.0, 1.2);
    cap.translate(Vec3f(0.0f, 0.0f, 3.6f));
    fixture.merge(stem);
    fixture.merge(cap);
    return fixture;
}

MotionEvidence export_case(const TriangleMesh &fixture, const bool enabled,
                           const int speed, const std::string &threshold)
{
    DynamicPrintConfig config = fan_speedup_overhangs_physical_contracts::config_for(false);
    config.set_deserialize_strict({
        {"detect_overhang_wall", false},
        {"enable_overhang_bridge_fan", enabled},
        {"fan_kickstart", 0.0},
        {"fan_speedup_time", 0.0},
        {"ironing_type", "no ironing"},
        {"reduce_fan_stop_start_freq", false},
        {"seam_position", "back"},
        {"top_shell_layers", 0},
    });
    // config_for() installs this enum through an integer-only typed override. Remove
    // that override so strict string parsing recreates the option from the registry,
    // including its enum-name map.
    REQUIRE(config.erase("overhang_fan_threshold"));
    config.set_deserialize_strict("overhang_fan_threshold", threshold);
    const auto *threshold_option = config.option<ConfigOptionEnumsGeneric>("overhang_fan_threshold");
    REQUIRE(threshold_option != nullptr);
    REQUIRE(threshold_option->keys_map != nullptr);
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{speed});
    const std::string gcode = slice({fixture}, config);
    return fan_speedup_overhangs_physical_contracts::inspect_motion_and_fans(gcode, config);
}

} // namespace overhang_fan_threshold_settings_contracts

TEST_CASE("Overhang fan threshold strings bound full-fan extrusion monotonically",
          "[Cooling][OverhangFanSettings]")
{
    using namespace overhang_fan_threshold_settings_contracts;
    constexpr std::array<const char *, 6> thresholds{{"0%", "10%", "25%", "50%", "75%", "95%"}};
    const TriangleMesh fixture = partially_supported_cap_fixture();
    std::array<MotionEvidence, thresholds.size()> rows{};
    for (size_t i = 0; i < thresholds.size(); ++i) {
        CAPTURE(thresholds[i]);
        rows[i] = export_case(fixture, true, 100, thresholds[i]);
        REQUIRE(rows[i].finite);
        REQUIRE(rows[i].path_distance > 0.0);
        REQUIRE(rows[i].elapsed_seconds > 0.0);
        if (i > 0) {
            CHECK_THAT(rows[i].path_distance, Catch::Matchers::WithinAbs(rows[0].path_distance, 0.05));
            CHECK(rows[i].full_fan_extrusion_xy_mm <= rows[i - 1].full_fan_extrusion_xy_mm + 0.05);
        }
    }
    REQUIRE(rows.front().deposited_xy_mm > 0.0);
    REQUIRE(rows.front().deposited_filament_mm > 0.0);
    const auto check_same_deposition = [&rows](const MotionEvidence &observed) {
        CHECK_THAT(observed.deposited_xy_mm,
                   Catch::Matchers::WithinRel(rows.front().deposited_xy_mm, 1e-6));
        CHECK_THAT(observed.deposited_filament_mm,
                   Catch::Matchers::WithinAbs(rows.front().deposited_filament_mm, 1e-3));
    };
    for (size_t i = 1; i < rows.size(); ++i)
        check_same_deposition(rows[i]);

    // The endpoints express the public contract: 0% permits external-perimeter cooling,
    // while 95% limits it to nearly unsupported spans. Intermediate bins may coincide.
    CHECK(rows.front().full_fan_extrusion_xy_mm > rows.back().full_fan_extrusion_xy_mm + 0.1);
    // 10% and 95% exercise the overlap-dependent threshold independently of the 0% special branch.
    CHECK(rows[1].full_fan_extrusion_xy_mm > rows.back().full_fan_extrusion_xy_mm + 0.1);

    const MotionEvidence disabled = export_case(fixture, false, 100, "0%");
    const MotionEvidence zero_speed = export_case(fixture, true, 0, "0%");
    for (const MotionEvidence &control : {disabled, zero_speed}) {
        REQUIRE(control.finite);
        CHECK(control.path_distance > 0.0);
        CHECK_THAT(control.full_fan_extrusion_xy_mm, Catch::Matchers::WithinAbs(0.0, 0.05));
        check_same_deposition(control);
    }

    const MotionEvidence half_speed = export_case(fixture, true, 50, "0%");
    REQUIRE(half_speed.finite);
    CHECK(std::any_of(half_speed.fan_on.begin(), half_speed.fan_on.end(),
                      [](const FanOnEvent &event) { return event.pwm >= 126 && event.pwm <= 129; }));
    check_same_deposition(half_speed);
}


TEST_CASE("Cooling fan speed endpoints follow the selected layer-time branch",
          "[Cooling][CoolingSettings][FanSpeedEndpoints]")
{
    using namespace cooling_settings_followup;

    const DynamicPrintConfig reference_config = config_for(false, true, false);
    const Evidence reference = inspect(slice({make_cube(8.0, 8.0, 0.8)}, reference_config), reference_config);
    REQUIRE(reference.fan_seen);
    REQUIRE_FALSE(reference.outer_feeds.empty());
    REQUIRE_FALSE(reference.inner_feeds.empty());

    const bool minimum_branch = GENERATE(true, false);
    const int endpoint = GENERATE(0, 40, 100);
    const int configured_minimum = minimum_branch ? endpoint : 0;
    const int configured_maximum = minimum_branch ? 100 : endpoint;
    const double threshold = minimum_branch ? 0.0 : 1000.0;
    REQUIRE(configured_minimum <= configured_maximum);
    CAPTURE(minimum_branch, endpoint, configured_minimum, configured_maximum, threshold);

    // Keep minimum-hold enabled in every row. Equal legal thresholds avoid interpolation:
    // zero selects minimum speed, while 1000 seconds selects maximum for this short-layer fixture.
    DynamicPrintConfig config = config_for(false, true, false);
    config.set_deserialize_strict({
        {"fan_min_speed", configured_minimum},
        {"fan_max_speed", configured_maximum},
        {"slow_down_layer_time", threshold},
        {"fan_cooling_layer_time", threshold},
    });
    const Evidence observed = inspect(slice({make_cube(8.0, 8.0, 0.8)}, config), config);

    require_same_material(reference, observed);
    REQUIRE(observed.fan_seen);
    REQUIRE_FALSE(observed.part_fan_states.empty());
    const int expected_pwm = static_cast<int>(std::lround(endpoint * 255.0 / 100.0));
    CHECK(std::all_of(observed.part_fan_states.begin(), observed.part_fan_states.end(),
                      [expected_pwm](int pwm) { return pwm == expected_pwm; }));

    REQUIRE(observed.outer_feeds.size() == reference.outer_feeds.size());
    REQUIRE(observed.inner_feeds.size() == reference.inner_feeds.size());
    for (size_t i = 0; i < observed.outer_feeds.size(); ++i)
        CHECK_THAT(observed.outer_feeds[i], Catch::Matchers::WithinAbs(reference.outer_feeds[i], 0.25));
    for (size_t i = 0; i < observed.inner_feeds.size(); ++i)
        CHECK_THAT(observed.inner_feeds[i], Catch::Matchers::WithinAbs(reference.inner_feeds[i], 0.25));
}

TEST_CASE("Cooling layer-time boundary selects minimum or interpolated fan speed",
          "[Cooling][CoolingSettings][FanCoolingLayerTimeBoundaries]")
{
    using namespace cooling_settings_followup;

    DynamicPrintConfig reference_config = config_for(false, true, false);
    reference_config.set_deserialize_strict({
        {"fan_min_speed", 40},
        {"fan_max_speed", 100},
        {"slow_down_layer_time", 0.0},
        {"fan_cooling_layer_time", 0.0},
    });
    const Evidence reference = inspect(slice({make_cube(8.0, 8.0, 0.8)}, reference_config), reference_config);
    REQUIRE(reference.fan_seen);
    REQUIRE_FALSE(reference.part_fan_states.empty());
    CHECK(std::all_of(reference.part_fan_states.begin(), reference.part_fan_states.end(),
                      [](int pwm) { return pwm == 102; }));

    const double fan_cooling_layer_time = GENERATE(0.0, 1000.0);
    CAPTURE(fan_cooling_layer_time);
    DynamicPrintConfig config = config_for(false, true, false);
    config.set_deserialize_strict({
        {"fan_min_speed", 40},
        {"fan_max_speed", 100},
        {"slow_down_layer_time", 0.0},
        {"fan_cooling_layer_time", fan_cooling_layer_time},
    });
    const Evidence observed = inspect(slice({make_cube(8.0, 8.0, 0.8)}, config), config);

    require_same_material(reference, observed);
    REQUIRE(observed.fan_seen);
    REQUIRE_FALSE(observed.part_fan_states.empty());
    if (fan_cooling_layer_time == 0.0) {
        CHECK(std::all_of(observed.part_fan_states.begin(), observed.part_fan_states.end(),
                          [](int pwm) { return pwm == 102; }));
    } else {
        CHECK(std::all_of(observed.part_fan_states.begin(), observed.part_fan_states.end(),
                          [](int pwm) { return pwm > 102 && pwm <= 255; }));
    }

    REQUIRE(observed.outer_feeds.size() == reference.outer_feeds.size());
    REQUIRE(observed.inner_feeds.size() == reference.inner_feeds.size());
    for (size_t i = 0; i < observed.outer_feeds.size(); ++i)
        CHECK_THAT(observed.outer_feeds[i], Catch::Matchers::WithinAbs(reference.outer_feeds[i], 0.25));
    for (size_t i = 0; i < observed.inner_feeds.size(); ++i)
        CHECK_THAT(observed.inner_feeds[i], Catch::Matchers::WithinAbs(reference.inner_feeds[i], 0.25));
}


namespace cooling_fan_layer_contracts {

DynamicPrintConfig layer_config(const int close_layers, const int full_layer, const int first_layer_speed)
{
    DynamicPrintConfig config = cooling_settings_followup::config_for(false, true, false);
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{100.0});
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{100.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{0});
    config.set_key_value("close_fan_the_first_x_layers", new ConfigOptionInts{close_layers});
    config.set_key_value("full_fan_speed_layer", new ConfigOptionInts{full_layer});
    config.set_key_value("initial_layer_fan_speed", new ConfigOptionInts{first_layer_speed});
    return config;
}

std::array<std::vector<int>, 4> physical_layer_pwm(const std::string &gcode,
                                                    const DynamicPrintConfig &config)
{
    std::array<std::vector<int>, 4> samples;
    GCodeReader reader;
    reader.apply_config(config);
    int part_fan = -1;
    reader.parse_buffer(gcode, [&samples, &part_fan](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M107") && !line.has('P'))
            part_fan = 0;
        else if (line.cmd_is("M106") && !line.has('P')) {
            float pwm = 0.0f;
            REQUIRE(line.has_value('S', pwm));
            REQUIRE(std::isfinite(pwm));
            REQUIRE(pwm >= 0.0f);
            REQUIRE(pwm <= 255.0f);
            part_fan = static_cast<int>(std::lround(pwm));
        }
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        const int layer = static_cast<int>(std::lround(line.new_Z(self) / 0.2)) - 1;
        REQUIRE(layer >= 0);
        REQUIRE(layer < static_cast<int>(samples.size()));
        REQUIRE(part_fan >= 0);
        samples[static_cast<size_t>(layer)].push_back(part_fan);
    });
    return samples;
}

void require_layer_pwm(const std::array<std::vector<int>, 4> &observed,
                       const std::array<int, 4> &expected_pwm)
{
    for (size_t layer = 0; layer < observed.size(); ++layer) {
        INFO("physical deposition layer " << layer);
        REQUIRE_FALSE(observed[layer].empty());
        const int pwm = expected_pwm[layer];
        CHECK(std::all_of(observed[layer].begin(), observed[layer].end(),
                          [pwm](const int value) { return value == pwm; }));
    }
}

void require_public_filter_material(const cooling_slowdown_contracts::FeedEvidence &evidence,
                                    const size_t expected_deposits)
{
    REQUIRE(evidence.finite);
    REQUIRE(evidence.feeds.size() == expected_deposits);
    CHECK_THAT(evidence.path_length,
               Catch::Matchers::WithinAbs(std::sqrt(2.0) + static_cast<double>(expected_deposits - 1), 1e-3));
    CHECK_THAT(evidence.filament_length,
               Catch::Matchers::WithinAbs(0.1 * static_cast<double>(expected_deposits), 1e-3));
    for (const double feed : evidence.feeds)
        CHECK_THAT(feed, Catch::Matchers::WithinAbs(3600.0, 1e-3));
}

std::string run_public_filter(const DynamicPrintConfig &config, const size_t last_layer)
{
    PrintConfig print_config;
    print_config.apply(config, true);
    GCode generator;
    generator.apply_print_config(print_config);
    generator.writer().set_extruders({0});
    CoolingBuffer buffer(generator);

    std::string output;
    for (size_t layer_id = 0; layer_id <= last_layer; ++layer_id) {
        // Each public filter input is a small valid layer with positive XY/E deposition.
        const std::string input = "G1 F3600 ;_EXTRUDE_SET_SPEED\nG1 X" + std::to_string((layer_id % 2) + 1) +
                                  " Y1 E0.1\n;_EXTRUDE_END\n";
        output += buffer.process_layer(std::string(input), layer_id, true);
    }
    return output;
}

} // namespace cooling_fan_layer_contracts

TEST_CASE("Cooling layer gates and first-layer override control physical P0 output",
          "[Cooling][CoolingSettings][FanLayerPhysical]")
{
    using namespace cooling_fan_layer_contracts;
    struct Row {
        const char *name;
        int close_layers;
        int full_layer;
        int first_layer_speed;
        std::array<int, 4> expected_pwm;
    };
    // Independently pinned Marlin PWM oracle: 25/50/75/100% -> S63/127/191/255;
    // the anchored 33/67/83% ramp points -> S84/171/212.
    constexpr std::array<Row, 8> rows{{
        {"ramp without override", 0, 4, -1, {63, 127, 191, 255}},
        {"zero override anchors ramp", 0, 4, 0, {0, 84, 171, 255}},
        {"representative override anchors ramp", 0, 4, 50, {127, 171, 212, 255}},
        {"maximum override", 0, 4, 100, {255, 255, 255, 255}},
        {"one closed layer", 1, 4, -1, {0, 84, 171, 255}},
        {"override gated by closed layer", 1, 4, 100, {0, 84, 171, 255}},
        {"full layer below close gate", 2, 1, -1, {0, 0, 255, 255}},
        {"zero full-speed layer disables ramp", 0, 0, -1, {255, 255, 255, 255}},
    }};

    const DynamicPrintConfig baseline_config = layer_config(0, 4, -1);
    const auto baseline_material = cooling_slowdown_contracts::inspect_feed_evidence(
        slice({make_cube(8.0, 8.0, 0.8)}, baseline_config), baseline_config);
    REQUIRE(baseline_material.finite);
    REQUIRE_FALSE(baseline_material.feeds.empty());

    for (const Row &row : rows) {
        CAPTURE(row.name, row.close_layers, row.full_layer, row.first_layer_speed);
        const DynamicPrintConfig config = layer_config(row.close_layers, row.full_layer, row.first_layer_speed);
        const std::string gcode = slice({make_cube(8.0, 8.0, 0.8)}, config);
        const auto material = cooling_slowdown_contracts::inspect_feed_evidence(gcode, config);
        cooling_slowdown_contracts::require_same_deposition(baseline_material, material);
        REQUIRE(material.feeds.size() == baseline_material.feeds.size());
        for (size_t i = 0; i < material.feeds.size(); ++i)
            CHECK_THAT(material.feeds[i], Catch::Matchers::WithinAbs(baseline_material.feeds[i], 0.25));
        require_layer_pwm(physical_layer_pwm(gcode, config), row.expected_pwm);
    }
}

TEST_CASE("Cooling layer settings honor their upper registered endpoints in the public filter",
          "[Cooling][CoolingSettings][FanLayerBoundaries]")
{
    using namespace cooling_fan_layer_contracts;
    // The public filter accepts one generated layer at a time; these bounded inputs exercise
    // layer 1000 without slicing a 1000-layer mesh. Every input has positive XY/E deposition.
    const DynamicPrintConfig close_config = layer_config(1000, 0, -1);
    const std::string close_gcode = run_public_filter(close_config, 1000);
    const auto close = cooling_settings_followup::inspect(close_gcode, close_config);
    REQUIRE(close.deposition_moves == 1001);
    REQUIRE(close.part_fan_states.size() == close.deposition_moves);
    require_public_filter_material(cooling_slowdown_contracts::inspect_feed_evidence(close_gcode, close_config),
                                   close.deposition_moves);
    for (size_t layer = 0; layer < 1000; ++layer)
        CHECK(close.part_fan_states[layer] == 0);
    CHECK(close.part_fan_states[1000] == 255);

    const DynamicPrintConfig full_config = layer_config(0, 1000, -1);
    const std::string full_gcode = run_public_filter(full_config, 999);
    const auto full = cooling_settings_followup::inspect(full_gcode, full_config);
    REQUIRE(full.deposition_moves == 1000);
    REQUIRE(full.part_fan_states.size() == full.deposition_moves);
    require_public_filter_material(cooling_slowdown_contracts::inspect_feed_evidence(full_gcode, full_config),
                                   full.deposition_moves);
    CHECK(full.part_fan_states.front() == 0);
    CHECK(full.part_fan_states[499] == 127);
    CHECK(full.part_fan_states.back() == 255);
}
