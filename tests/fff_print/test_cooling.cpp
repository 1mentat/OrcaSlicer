#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/GCodeReader.hpp"
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
        { "cooling",                      true },
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
        {"fan_cooling_layer_time", 1001.0},
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
    CHECK(actual.feeds.size() == expected.feeds.size());
    CHECK_THAT(actual.path_length, Catch::Matchers::WithinRel(expected.path_length, 1e-6));
    CHECK_THAT(actual.filament_length, Catch::Matchers::WithinAbs(expected.filament_length, 1e-3));
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

    const FeedEvidence disabled = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, disabled_config), disabled_config);
    const FeedEvidence no_time = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, no_time_config), no_time_config);
    const FeedEvidence slow_five = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, slow_five_config), slow_five_config);
    const FeedEvidence slow_one = inspect_feed_evidence(slice({make_cube(8.0, 8.0, 1.2)}, slow_one_config), slow_one_config);

    require_same_deposition(disabled, no_time);
    require_same_deposition(disabled, slow_five);
    require_same_deposition(disabled, slow_one);

    // The boolean gate is isolated by disabled versus the otherwise identical 1000 s case.
    REQUIRE(minimum_feed(slow_five) + 1.0 < minimum_feed(disabled));
    // With the gate enabled, a zero threshold retains the disabled control's feed rate.
    CHECK_THAT(minimum_feed(no_time), Catch::Matchers::WithinAbs(minimum_feed(disabled), 1e-3));
    REQUIRE(minimum_feed(slow_five) + 1.0 < minimum_feed(no_time));
    // With the gate and threshold held fixed, the configured minimum feed remains observable.
    REQUIRE(minimum_feed(slow_one) + 1.0 < minimum_feed(slow_five));
    CHECK_THAT(minimum_feed(slow_five), Catch::Matchers::WithinAbs(5.0 * 60.0, 0.1));
    CHECK_THAT(minimum_feed(slow_one), Catch::Matchers::WithinAbs(1.0 * 60.0, 0.1));
}
