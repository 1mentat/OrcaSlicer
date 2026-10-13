#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

// The fan is held off for the first close_fan_the_first_x_layers layers. Every layer of a 20 mm cube
// prints faster than fan_cooling_layer_time, and fan_min_speed equals fan_max_speed, so the part fan
// runs at 100% on every layer it is allowed to run on, with no ramp.
TEST_CASE("Fan is held off for the initial layers", "[Cooling]")
{
    constexpr double layer_height = 0.2;
    // Z of the layer during which the part fan is first switched on, 0 for the first layer.
    const auto first_fan_on_z = [](int close_fan_layers) {
        const std::string gcode = slice({ cube(20) }, {
            { "layer_height",                 layer_height },
            { "initial_layer_print_height",   layer_height },
            { "close_fan_the_first_x_layers", close_fan_layers },
            { "full_fan_speed_layer",         0 },
            { "initial_layer_fan_speed",      -1 },
            { "reduce_fan_stop_start_freq",   false },
            { "fan_min_speed",                100 },
            { "fan_max_speed",                100 },
            { "slow_down_layer_time",         1 },
            { "fan_cooling_layer_time",       1000 },
            { "layer_change_gcode",           ";TEST_LAYER_Z=[layer_z]" },
        });
        std::istringstream input(gcode);
        std::string line;
        double z = 0.;
        while (std::getline(input, line)) {
            if (line.rfind(";TEST_LAYER_Z=", 0) == 0)
                z = std::stod(line.substr(14));
            else if (line.rfind("M106 S", 0) == 0 && std::stoi(line.substr(6)) > 0)
                return z;
        }
        return -1.;
    };

    const double without_gate = first_fan_on_z(0);
    const double with_gate    = first_fan_on_z(5);
    // The fan is used at all, and from the first layers when nothing holds it off.
    REQUIRE(without_gate >= 0.);
    CHECK(without_gate <= 2 * layer_height + EPSILON);
    // Held off for layers 1 to 5. The command that switches it on for layer 6 may be emitted
    // just before that layer's change, at the end of layer 5.
    REQUIRE(with_gate >= 0.);
    CHECK(with_gate >= 5 * layer_height - EPSILON);
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
