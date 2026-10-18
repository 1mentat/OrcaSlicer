#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "test_helpers.hpp"

#include <algorithm>
#include <cstddef>
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
        {"top_shell_layers",               0},
        {"bottom_shell_layers",            0},
        {"sparse_infill_density",          "0%"},
        {"minimum_sparse_infill_area",     0.0},
        {"enable_support",                 false},
    });
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4});
    config.set_key_value("fan_max_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("fan_min_speed", new ConfigOptionFloats{20.0});
    config.set_key_value("overhang_fan_speed", new ConfigOptionInts{100});
    config.set_key_value("overhang_fan_threshold", new ConfigOptionEnumsGeneric{Overhang_threshold_2_4});
    return config;
}

MotionEvidence inspect_motion_and_fans(const std::string &gcode, const DynamicPrintConfig &config)
{
    MotionEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int previous_fan_pwm = 0;
    reader.parse_buffer(gcode, [&evidence, &previous_fan_pwm](GCodeReader &self,
                                                   const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M106") && !line.has_p()) {
            float pwm = 0.0f;
            REQUIRE(line.has_value('S', pwm));
            REQUIRE(std::isfinite(pwm));
            REQUIRE(pwm >= 0.0f);
            REQUIRE(pwm <= 255.0f);
            const int fan_pwm = static_cast<int>(std::lround(pwm));
            const bool rising = fan_pwm > previous_fan_pwm;
            previous_fan_pwm = fan_pwm;
            if (fan_pwm > 0) {
                // FanMover can relocate this command across buffered motion; classify by
                // pinned PWM targets rather than output TYPE comments. Ordinary fan is
                // 20% (S51), while overhang fan is 100% (S255). Track rises explicitly
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
        if (!(distance > 0.0))
            return;
        evidence.path_distance += distance;
        const double feed = line.has_f() ? line.f() : self.f();
        if (std::isfinite(feed) && feed > 0.0)
            evidence.elapsed_seconds += distance * 60.0 / feed;
        evidence.finite = evidence.finite && std::isfinite(evidence.path_distance) &&
                          std::isfinite(evidence.elapsed_seconds);
    });
    return evidence;
}

double first_ordinary_fan_distance(const MotionEvidence &evidence)
{
    bool found = false;
    double first = 0.0;
    for (const FanOnEvent &event : evidence.fan_on) {
        if (!event.overhang && event.pwm == 51 && event.rise) {
            first = event.path_distance;
            found = true;
            break;
        }
    }
    REQUIRE(found);
    return first;
}

double first_ordinary_fan_time(const MotionEvidence &evidence)
{
    bool found = false;
    double first = 0.0;
    for (const FanOnEvent &event : evidence.fan_on) {
        if (!event.overhang && event.pwm == 51 && event.rise) {
            first = event.elapsed_seconds;
            found = true;
            break;
        }
    }
    REQUIRE(found);
    return first;
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
    // The ordinary stem precedes the wider cap in one public mesh, so the
    // queued S51 rise is flushed before the cap's S255 overhang request.
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

    // The setting controls where ordinary fan rises are emitted in physical motion;
    // compare the accumulated motion location/time instead of serialized line positions.
    const double ordinary_distance_delta = std::abs(first_ordinary_fan_distance(ordinary) -
                                                    first_ordinary_fan_distance(overhang));
    const double ordinary_time_delta = std::abs(first_ordinary_fan_time(ordinary) -
                                                first_ordinary_fan_time(overhang));
    const bool distance_changed = ordinary_distance_delta > 0.05;
    const bool time_changed = ordinary_time_delta > 0.01;
    INFO("ordinary fan scheduling deltas: distance=" << ordinary_distance_delta
         << ", time=" << ordinary_time_delta);
    if (!distance_changed)
        REQUIRE(time_changed);

    const cooling_slowdown_contracts::FeedEvidence ordinary_material =
        cooling_slowdown_contracts::inspect_feed_evidence(ordinary_gcode, ordinary_config);
    const cooling_slowdown_contracts::FeedEvidence overhang_material =
        cooling_slowdown_contracts::inspect_feed_evidence(overhang_gcode, overhang_config);
    cooling_slowdown_contracts::require_same_deposition(ordinary_material, overhang_material);
}
