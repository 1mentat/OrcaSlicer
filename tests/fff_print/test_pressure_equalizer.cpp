#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers.hpp>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/PressureEqualizer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"
#include "test_helpers.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace Slic3r {
namespace {

struct PressureObservation {
    std::size_t extrusion_moves = 0;
    std::size_t speed_markers = 0;
    double total_extrusion = 0.0;
    double path_length = 0.0;
    double max_abs_y = 0.0;
    double final_x = 0.0;
    std::vector<float> high_flow_feedrates;
};

std::string pressure_fixture(bool relative_e, Slic3r::ExtrusionRole role)
{
    const char *e_values = relative_e ? "E1\nG1 F600 X20 Y0 E1\nG1 F1200 X30 Y0 E2\nG1 F1200 X40 Y0 E2\n"
                                      : "E1\nG1 F600 X20 Y0 E2\nG1 F1200 X30 Y0 E4\nG1 F1200 X40 Y0 E6\n";
    return ";_EXTRUSION_ROLE:" + std::to_string(int(role)) +
           "\nG1 F600 ;_EXTRUDE_SET_SPEED\nG1 F600 X10 Y0 " + e_values +
           ";_EXTRUDE_END\n";
}

std::string equalize_pressure(const std::string &input, bool relative_e, bool external_only,
                              const std::vector<double> &filament_diameters = {1.75},
                              double max_rate_slope = 1.0, double max_segment_length = 2.0)
{
    Slic3r::GCodeConfig config;
    config.filament_diameter.values = filament_diameters;
    config.use_relative_e_distances.value = relative_e;
    config.max_volumetric_extrusion_rate_slope.value = max_rate_slope;
    config.max_volumetric_extrusion_rate_slope_segment_length.value = max_segment_length;
    config.extrusion_rate_smoothing_external_perimeter_only.value = external_only;

    Slic3r::PressureEqualizer equalizer(config);
    Slic3r::LayerResult first{input, 0, false, false, false};
    REQUIRE(equalizer.process_layer(std::move(first)).nop_layer_result);
    const Slic3r::LayerResult flushed = equalizer.process_layer(Slic3r::LayerResult::make_nop_layer_result());
    return flushed.gcode;
}

PressureObservation observe_pressure(const std::string &gcode, const Slic3r::GCodeConfig &config)
{
    Slic3r::GCodeReader reader;
    reader.apply_config(config);
    PressureObservation observation;
    reader.parse_buffer(gcode, [&observation](Slic3r::GCodeReader &state, const Slic3r::GCodeReader::GCodeLine &line) {
        if (line.raw().find(";_EXTRUDE_SET_SPEED") != std::string::npos)
            ++observation.speed_markers;
        if (!line.cmd_is("G1") || !line.extruding(state))
            return;
        ++observation.extrusion_moves;
        observation.total_extrusion += line.dist_E(state);
        observation.path_length += line.dist_XY(state);
        observation.max_abs_y = std::max(observation.max_abs_y, std::abs(double(line.new_Y(state))));
        if (state.x() >= 20.f)
            observation.high_flow_feedrates.push_back(line.new_F(state));
        observation.final_x = line.new_X(state);
    });
    return observation;
}

void check_conserved_path(const PressureObservation &observation)
{
    CHECK(observation.extrusion_moves > 0);
    CHECK_THAT(observation.total_extrusion, Catch::Matchers::WithinAbs(6.0, 0.001));
    CHECK_THAT(observation.path_length, Catch::Matchers::WithinAbs(40.0, 0.001));
    CHECK_THAT(observation.max_abs_y, Catch::Matchers::WithinAbs(0.0, 0.001));
    CHECK_THAT(observation.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
}

struct PrintMotionObservation
{
    std::vector<std::array<double, 6>> moves;
    std::vector<double> extrusion_feedrates;
    std::size_t extrusion_moves = 0;
    std::size_t outer_wall_moves = 0;
    std::size_t inner_wall_moves = 0;
    double total_extrusion = 0.0;
    double path_length = 0.0;
    double max_outer_wall_feedrate = 0.0;
    double max_inner_wall_feedrate = 0.0;
};

Slic3r::DynamicPrintConfig pressure_print_config(double rate_slope, double segment_length)
{
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"max_volumetric_extrusion_rate_slope", rate_slope},
        {"max_volumetric_extrusion_rate_slope_segment_length", segment_length},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 2},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"outer_wall_speed", 20.0},
        {"inner_wall_speed", 80.0},
        {"initial_layer_speed", 20.0},
        {"initial_layer_infill_speed", 20.0},
        {"filament_max_volumetric_speed", "200"},
        {"filament_adaptive_volumetric_speed", false},
        {"small_perimeter_threshold", 0.0},
        {"gcode_comments", true},
        {"enable_arc_fitting", false},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0.0},
        {"fan_cooling_layer_time", 0.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
    });
    return config;
}

PrintMotionObservation observe_print_motion(const std::string &gcode,
                                           const Slic3r::DynamicPrintConfig &config)
{
    enum class WallRole { Other, Outer, Inner };
    PrintMotionObservation observation;
    WallRole active_wall = WallRole::Other;
    Slic3r::GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&observation, &active_wall](Slic3r::GCodeReader &state,
                                                            const Slic3r::GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind("TYPE:", 0) == 0) {
            active_wall = comment == "TYPE:Outer wall" ? WallRole::Outer :
                          comment == "TYPE:Inner wall" ? WallRole::Inner : WallRole::Other;
        }
        if (!line.cmd_is("G0") && !line.cmd_is("G1"))
            return;
        if (!line.has(Slic3r::X) && !line.has(Slic3r::Y) && !line.has(Slic3r::Z) &&
            !line.has(Slic3r::E) && !line.has(Slic3r::F))
            return;

        const bool deposits_xy = line.extruding(state) && line.dist_XY(state) > EPSILON;
        observation.moves.push_back({line.cmd_is("G0") ? 0.0 : 1.0, line.new_X(state), line.new_Y(state),
                                     line.new_Z(state), line.dist_E(state), line.new_F(state)});
        if (!deposits_xy)
            return;

        ++observation.extrusion_moves;
        observation.extrusion_feedrates.push_back(line.new_F(state));
        observation.total_extrusion += line.dist_E(state);
        observation.path_length += line.dist_XY(state);
        if (active_wall == WallRole::Outer) {
            ++observation.outer_wall_moves;
            observation.max_outer_wall_feedrate = std::max(observation.max_outer_wall_feedrate,
                                                           double(line.new_F(state)));
        } else if (active_wall == WallRole::Inner) {
            ++observation.inner_wall_moves;
            observation.max_inner_wall_feedrate = std::max(observation.max_inner_wall_feedrate,
                                                           double(line.new_F(state)));
        }
    });
    return observation;
}

bool same_print_motion_stream(const std::vector<std::array<double, 6>> &left,
                              const std::vector<std::array<double, 6>> &right)
{
    if (left.size() != right.size())
        return false;
    const auto same_value = [](double a, double b) { return std::abs(a - b) <= 0.001; };
    for (std::size_t i = 0; i < left.size(); ++i) {
        for (std::size_t axis = 0; axis < left[i].size(); ++axis)
            if (!same_value(left[i][axis], right[i][axis]))
                return false;
    }
    return true;
}

bool extrusion_feedrates_differ(const PrintMotionObservation &left, const PrintMotionObservation &right)
{
    if (left.extrusion_feedrates.size() != right.extrusion_feedrates.size())
        return false;
    for (std::size_t i = 0; i < left.extrusion_feedrates.size(); ++i)
        if (std::abs(left.extrusion_feedrates[i] - right.extrusion_feedrates[i]) > 0.01)
            return true;
    return false;
}

} // namespace

TEST_CASE("Tagged external flow transitions preserve path geometry while smoothing feed rate", "[PressureEqualizer]")
{
    const bool relative_e = GENERATE(false, true);
    const std::string input = pressure_fixture(relative_e, Slic3r::erExternalPerimeter);

    Slic3r::GCodeConfig config;
    config.use_relative_e_distances.value = relative_e;
    const PressureObservation external = observe_pressure(
        equalize_pressure(input, relative_e, true), config);
    const PressureObservation internal_control = observe_pressure(
        equalize_pressure(pressure_fixture(relative_e, Slic3r::erPerimeter), relative_e, true), config);

    REQUIRE(external.extrusion_moves > 4);
    REQUIRE(external.speed_markers > 1);
    REQUIRE_THAT(external.total_extrusion, Catch::Matchers::WithinAbs(6.0, 0.001));
    REQUIRE_THAT(external.path_length, Catch::Matchers::WithinAbs(40.0, 0.001));
    REQUIRE_THAT(external.max_abs_y, Catch::Matchers::WithinAbs(0.0, 0.001));
    REQUIRE_THAT(external.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
    REQUIRE_FALSE(external.high_flow_feedrates.empty());
    for (float feedrate : external.high_flow_feedrates) {
        CHECK(feedrate > 0.f);
        CHECK(feedrate <= 1200.f);
    }
    // At X20 the filament per millimeter doubles: continuous volumetric flow
    // can require a speed below the preceding F600, not between F600 and F1200.
    CHECK(std::any_of(external.high_flow_feedrates.begin(), external.high_flow_feedrates.end(), [](float feedrate) {
        return feedrate < 1200.f;
    }));

    CHECK(internal_control.extrusion_moves == 4);
    CHECK(internal_control.speed_markers == 1);
    CHECK_THAT(internal_control.total_extrusion, Catch::Matchers::WithinAbs(6.0, 0.001));
    CHECK_THAT(internal_control.path_length, Catch::Matchers::WithinAbs(40.0, 0.001));
    CHECK_THAT(internal_control.max_abs_y, Catch::Matchers::WithinAbs(0.0, 0.001));
    CHECK_THAT(internal_control.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
    REQUIRE(internal_control.high_flow_feedrates.size() == 2);
    for (float feedrate : internal_control.high_flow_feedrates)
        CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(1200.f, 0.001f));
}

TEST_CASE("Pressure smoothing honors the external-only setting for tagged roles", "[PressureEqualizer][ExternalOnlyBoolean]")
{
    const bool relative_e = GENERATE(false, true);
    const bool external_only = GENERATE(false, true);
    CAPTURE(relative_e, external_only);

    Slic3r::GCodeConfig config;
    config.use_relative_e_distances.value = relative_e;
    const PressureObservation external = observe_pressure(
        equalize_pressure(pressure_fixture(relative_e, Slic3r::erExternalPerimeter), relative_e, external_only), config);
    const PressureObservation internal = observe_pressure(
        equalize_pressure(pressure_fixture(relative_e, Slic3r::erPerimeter), relative_e, external_only), config);

    const auto require_conserved_output = [](const PressureObservation &observation) {
        REQUIRE(observation.extrusion_moves > 0);
        CHECK_THAT(observation.total_extrusion, Catch::Matchers::WithinAbs(6.0, 0.001));
        CHECK_THAT(observation.path_length, Catch::Matchers::WithinAbs(40.0, 0.001));
        CHECK_THAT(observation.max_abs_y, Catch::Matchers::WithinAbs(0.0, 0.001));
        CHECK_THAT(observation.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
        REQUIRE_FALSE(observation.high_flow_feedrates.empty());
        for (const float feedrate : observation.high_flow_feedrates) {
            CHECK(feedrate > 0.f);
            CHECK(feedrate <= 1200.f);
        }
    };
    const auto require_smoothed_transition = [](const PressureObservation &observation) {
        REQUIRE(observation.extrusion_moves > 4);
        REQUIRE(observation.speed_markers > 1);
        CHECK(std::any_of(observation.high_flow_feedrates.begin(), observation.high_flow_feedrates.end(),
                          [](const float feedrate) { return feedrate < 1200.f; }));
    };

    require_conserved_output(external);
    require_conserved_output(internal);
    // External perimeters remain eligible under both settings. The boolean changes
    // only whether the same physical flow transition is also smoothed on an inner perimeter.
    require_smoothed_transition(external);
    if (external_only) {
        CHECK(internal.extrusion_moves == 4);
        CHECK(internal.speed_markers == 1);
        REQUIRE(internal.high_flow_feedrates.size() == 2);
        for (const float feedrate : internal.high_flow_feedrates)
            CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(1200.f, 0.001f));
    } else {
        require_smoothed_transition(internal);
    }
}

TEST_CASE("Positive volumetric-rate slopes allow progressively faster flow transitions", "[PressureEqualizer][VolumetricRateSlope]")
{
    const bool relative_e = GENERATE(false, true);
    const std::string input = pressure_fixture(relative_e, Slic3r::erExternalPerimeter);
    Slic3r::GCodeConfig reader_config;
    reader_config.use_relative_e_distances.value = relative_e;

    const PressureObservation gentle = observe_pressure(
        equalize_pressure(input, relative_e, false, {1.75}, 1.0, 2.0), reader_config);
    const PressureObservation medium = observe_pressure(
        equalize_pressure(input, relative_e, false, {1.75}, 10.0, 2.0), reader_config);
    const PressureObservation steep = observe_pressure(
        equalize_pressure(input, relative_e, false, {1.75}, 100.0, 2.0), reader_config);

    REQUIRE_FALSE(gentle.high_flow_feedrates.empty());
    REQUIRE_FALSE(medium.high_flow_feedrates.empty());
    REQUIRE_FALSE(steep.high_flow_feedrates.empty());
    const auto minimum_high_flow_feedrate = [](const PressureObservation &observation) {
        return *std::min_element(observation.high_flow_feedrates.begin(), observation.high_flow_feedrates.end());
    };
    const float gentle_min = minimum_high_flow_feedrate(gentle);
    const float medium_min = minimum_high_flow_feedrate(medium);
    const float steep_min = minimum_high_flow_feedrate(steep);
    CAPTURE(relative_e, gentle_min, medium_min, steep_min);

    CHECK(gentle_min < medium_min);
    CHECK(medium_min < steep_min);
    CHECK(steep_min <= 1200.f);

    check_conserved_path(gentle);
    check_conserved_path(medium);
    check_conserved_path(steep);
}

TEST_CASE("Volumetric-rate segment limits change ramp partitioning without changing the path", "[PressureEqualizer][VolumetricRateSlopeSegmentLength]")
{
    const bool relative_e = GENERATE(false, true);
    const std::string input = pressure_fixture(relative_e, Slic3r::erExternalPerimeter);
    Slic3r::GCodeConfig reader_config;
    reader_config.use_relative_e_distances.value = relative_e;

    // These are the registered 0.5 mm minimum and 5 mm maximum. Keep the slope
    // fixed so only the ramp partition length changes.
    const PressureObservation fine = observe_pressure(
        equalize_pressure(input, relative_e, false, {1.75}, 10.0, 0.5), reader_config);
    const PressureObservation coarse = observe_pressure(
        equalize_pressure(input, relative_e, false, {1.75}, 10.0, 5.0), reader_config);
    CAPTURE(relative_e, fine.extrusion_moves, coarse.extrusion_moves);

    CHECK(fine.extrusion_moves > coarse.extrusion_moves);
    check_conserved_path(fine);
    check_conserved_path(coarse);
}

TEST_CASE("Zero slope ignores segment length while positive smoothing changes exported ramps", "[PressureEqualizer][VolumetricRateSlope][VolumetricRateSlopeSegmentLength]")
{
    const auto export_print = [](double slope, double segment_length) {
        const Slic3r::DynamicPrintConfig config = pressure_print_config(slope, segment_length);
        const std::string gcode = Slic3r::Test::slice({Slic3r::Test::cube(4.0)}, config);
        return observe_print_motion(gcode, config);
    };

    // With smoothing disabled, segment length must not affect the public Print
    // export. With smoothing enabled, the same two bounds must affect the ramp.
    const PrintMotionObservation zero_fine = export_print(0.0, 0.5);
    const PrintMotionObservation zero_coarse = export_print(0.0, 5.0);
    const PrintMotionObservation enabled_fine = export_print(1.0, 0.5);
    const PrintMotionObservation enabled_coarse = export_print(1.0, 5.0);
    CAPTURE(zero_fine.moves.size(), zero_coarse.moves.size(), enabled_fine.extrusion_moves,
            enabled_coarse.extrusion_moves);

    CHECK(same_print_motion_stream(zero_fine.moves, zero_coarse.moves));
    REQUIRE(zero_fine.outer_wall_moves > 0);
    REQUIRE(zero_fine.inner_wall_moves > 0);
    CHECK(zero_fine.max_inner_wall_feedrate > zero_fine.max_outer_wall_feedrate);

    const bool segmentation_changed = enabled_fine.extrusion_moves != enabled_coarse.extrusion_moves;
    const bool feed_response_changed = extrusion_feedrates_differ(enabled_fine, enabled_coarse);
    const bool positive_setting_changes_response = segmentation_changed || feed_response_changed;
    CHECK(positive_setting_changes_response);

    const auto check_nonempty_deposition = [](const PrintMotionObservation &observation) {
        CHECK(observation.extrusion_moves > 0);
        CHECK(observation.total_extrusion > 0.0);
        CHECK(observation.path_length > 0.0);
    };
    check_nonempty_deposition(zero_fine);
    check_nonempty_deposition(zero_coarse);
    check_nonempty_deposition(enabled_fine);
    check_nonempty_deposition(enabled_coarse);
    for (const PrintMotionObservation *observation : {&zero_coarse, &enabled_fine, &enabled_coarse}) {
        CHECK_THAT(observation->total_extrusion,
                   Catch::Matchers::WithinAbs(zero_fine.total_extrusion, 0.05));
        CHECK_THAT(observation->path_length, Catch::Matchers::WithinAbs(zero_fine.path_length, 0.05));
    }
}

namespace {

struct PressureStateMove {
    double start_x = 0.0;
    double start_y = 0.0;
    double end_x = 0.0;
    double end_y = 0.0;
    double delta_e = 0.0;
    double feed = 0.0;
};

struct PressureStateObservation {
    std::size_t g92_lines = 0;
    double post_reset_distance = 0.0;
    double post_reset_extrusion = 0.0;
    double first_post_reset_start_x = 0.0;
    double last_post_reset_end_x = 0.0;
    bool saw_post_reset_extrusion = false;
    std::vector<PressureStateMove> moves;
};

PressureStateObservation observe_g92_state(const std::string &gcode)
{
    Slic3r::GCodeConfig config;
    config.use_relative_e_distances.value = false;
    Slic3r::GCodeReader reader;
    reader.apply_config(config);
    PressureStateObservation observation;
    bool after_g92 = false;
    reader.parse_buffer(gcode, [&observation, &after_g92](Slic3r::GCodeReader &state, const Slic3r::GCodeReader::GCodeLine &line) {
        if (line.cmd_is("G92")) {
            ++observation.g92_lines;
            after_g92 = true;
            return;
        }
        if (!after_g92 || !line.cmd_is("G1") || !line.extruding(state))
            return;

        const double start_x = state.x();
        const double start_y = state.y();
        const double end_x = line.new_X(state);
        const double end_y = line.new_Y(state);
        const PressureStateMove move{
            start_x, start_y, end_x, end_y,
            line.dist_E(state), line.new_F(state)
        };
        observation.moves.push_back(move);
        observation.post_reset_distance += line.dist_XY(state);
        observation.post_reset_extrusion += line.dist_E(state);
        if (!observation.saw_post_reset_extrusion) {
            observation.first_post_reset_start_x = start_x;
            observation.saw_post_reset_extrusion = true;
        }
        observation.last_post_reset_end_x = end_x;
    });
    return observation;
}

struct PressureToolObservation {
    std::size_t tool_changes = 0;
    std::size_t tool0_extrusion_moves = 0;
    std::size_t tool1_extrusion_moves = 0;
    double tool0_extrusion = 0.0;
    double tool1_extrusion = 0.0;
    double final_x = 0.0;
};

PressureToolObservation observe_pressure_tools(const std::string &gcode)
{
    Slic3r::GCodeConfig config;
    config.use_relative_e_distances.value = false;
    Slic3r::GCodeReader reader;
    reader.apply_config(config);
    PressureToolObservation observation;
    int active_tool = 0;
    reader.parse_buffer(gcode, [&observation, &active_tool](Slic3r::GCodeReader &state, const Slic3r::GCodeReader::GCodeLine &line) {
        const std::string command(line.cmd());
        if (command.size() > 1 && command[0] == 'T') {
            const int new_tool = command[1] - '0';
            if (new_tool != active_tool)
                ++observation.tool_changes;
            active_tool = new_tool;
            return;
        }
        if (!line.cmd_is("G1") || !line.extruding(state))
            return;
        if (active_tool == 0) {
            ++observation.tool0_extrusion_moves;
            observation.tool0_extrusion += line.dist_E(state);
        } else if (active_tool == 1) {
            ++observation.tool1_extrusion_moves;
            observation.tool1_extrusion += line.dist_E(state);
        }
        observation.final_x = line.new_X(state);
    });
    return observation;
}

} // namespace

TEST_CASE("G92 resets the logical extrusion state before a new pressure block", "[PressureEqualizer][PressureEqualizerStateTransitions]")
{
    const std::string zero_origin_input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nG1 F600 X10 Y0 E1\nG92 E0\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X20 Y0 E1\nG1 F600 X30 Y0 E2\nG1 F630 X40 Y0 E3\nG1 F630 X50 Y0 E4\n"
        ";_EXTRUDE_END\n";
    const std::string shifted_origin_input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nG1 F600 X10 Y0 E1\nG92 E5\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X20 Y0 E6\nG1 F600 X30 Y0 E7\nG1 F630 X40 Y0 E8\nG1 F630 X50 Y0 E9\n"
        ";_EXTRUDE_END\n";

    const std::string zero_origin_output = equalize_pressure(zero_origin_input, false, false, {1.75});
    const std::string shifted_origin_output = equalize_pressure(shifted_origin_input, false, false, {1.75});
    const PressureStateObservation zero_origin = observe_g92_state(zero_origin_output);
    const PressureStateObservation shifted_origin = observe_g92_state(shifted_origin_output);

    const auto check_physical_contract = [](const PressureStateObservation &observation) {
        REQUIRE(observation.g92_lines == 1);
        REQUIRE(observation.saw_post_reset_extrusion);
        REQUIRE_FALSE(observation.moves.empty());
        CHECK_THAT(observation.post_reset_extrusion, Catch::Matchers::WithinAbs(4.0, 0.001));
        CHECK_THAT(observation.post_reset_distance, Catch::Matchers::WithinAbs(40.0, 0.001));
        CHECK_THAT(observation.first_post_reset_start_x, Catch::Matchers::WithinAbs(10.0, 0.001));
        CHECK_THAT(observation.last_post_reset_end_x, Catch::Matchers::WithinAbs(50.0, 0.001));
    };
    check_physical_contract(zero_origin);
    check_physical_contract(shifted_origin);

    // Changing only the logical E origin must not change physical pressure
    // smoothing: compare every emitted XY endpoint, E delta, and feed value.
    INFO("zero-origin emitted block:\n" << zero_origin_output);
    INFO("shifted-origin emitted block:\n" << shifted_origin_output);
    REQUIRE(zero_origin.moves.size() == shifted_origin.moves.size());
    for (std::size_t i = 0; i < zero_origin.moves.size(); ++i) {
        const PressureStateMove &lhs = zero_origin.moves[i];
        const PressureStateMove &rhs = shifted_origin.moves[i];
        CHECK_THAT(lhs.start_x, Catch::Matchers::WithinAbs(rhs.start_x, 0.001));
        CHECK_THAT(lhs.start_y, Catch::Matchers::WithinAbs(rhs.start_y, 0.001));
        CHECK_THAT(lhs.end_x, Catch::Matchers::WithinAbs(rhs.end_x, 0.001));
        CHECK_THAT(lhs.end_y, Catch::Matchers::WithinAbs(rhs.end_y, 0.001));
        CHECK_THAT(lhs.delta_e, Catch::Matchers::WithinAbs(rhs.delta_e, 0.001));
        CHECK_THAT(lhs.feed, Catch::Matchers::WithinAbs(rhs.feed, 0.001));
    }
}

// Regression reproducer for the classifier defect: G92 is a logical reset, not a physical extrusion.
// The current implementation classifies the E5 form as extruding because its stored logical X/Y
// endpoints differ and its E value increases; the E0 form does not. Keep this expected baseline
// failure separate from the passing E-only state-transition contract above.
TEST_CASE("G92 combined coordinate reset is not physical extrusion", "[PressureEqualizer][PressureEqualizerCoordinateResetDefect][KnownBaselineDefect][!shouldfail]")
{
    const std::string zero_origin_input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nG1 F600 X10 Y0 E1\nG92 X0 Y0 E0\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X10 Y0 E1\nG1 F600 X20 Y0 E2\nG1 F630 X30 Y0 E3\nG1 F630 X40 Y0 E4\n"
        ";_EXTRUDE_END\n";
    const std::string shifted_origin_input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nG1 F600 X10 Y0 E1\nG92 X0 Y0 E5\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X10 Y0 E6\nG1 F600 X20 Y0 E7\nG1 F630 X30 Y0 E8\nG1 F630 X40 Y0 E9\n"
        ";_EXTRUDE_END\n";

    const std::string zero_origin_output = equalize_pressure(zero_origin_input, false, false, {1.75});
    const std::string shifted_origin_output = equalize_pressure(shifted_origin_input, false, false, {1.75});
    const PressureStateObservation zero_origin = observe_g92_state(zero_origin_output);
    const PressureStateObservation shifted_origin = observe_g92_state(shifted_origin_output);

    const auto check_physical_contract = [](const PressureStateObservation &observation) {
        REQUIRE(observation.g92_lines == 1);
        REQUIRE(observation.saw_post_reset_extrusion);
        REQUIRE_FALSE(observation.moves.empty());
        CHECK_THAT(observation.post_reset_extrusion, Catch::Matchers::WithinAbs(4.0, 0.001));
        CHECK_THAT(observation.post_reset_distance, Catch::Matchers::WithinAbs(40.0, 0.001));
        CHECK_THAT(observation.first_post_reset_start_x, Catch::Matchers::WithinAbs(0.0, 0.001));
        CHECK_THAT(observation.last_post_reset_end_x, Catch::Matchers::WithinAbs(40.0, 0.001));
    };
    check_physical_contract(zero_origin);
    check_physical_contract(shifted_origin);

    // Changing only the logical E origin must not change physical pressure
    // smoothing: compare every emitted XY endpoint, E delta, and feed value.
    INFO("zero-origin emitted block:\n" << zero_origin_output);
    INFO("shifted-origin emitted block:\n" << shifted_origin_output);
    REQUIRE(zero_origin.moves.size() == shifted_origin.moves.size());
    for (std::size_t i = 0; i < zero_origin.moves.size(); ++i) {
        const PressureStateMove &lhs = zero_origin.moves[i];
        const PressureStateMove &rhs = shifted_origin.moves[i];
        CHECK_THAT(lhs.start_x, Catch::Matchers::WithinAbs(rhs.start_x, 0.001));
        CHECK_THAT(lhs.start_y, Catch::Matchers::WithinAbs(rhs.start_y, 0.001));
        CHECK_THAT(lhs.end_x, Catch::Matchers::WithinAbs(rhs.end_x, 0.001));
        CHECK_THAT(lhs.end_y, Catch::Matchers::WithinAbs(rhs.end_y, 0.001));
        CHECK_THAT(lhs.delta_e, Catch::Matchers::WithinAbs(rhs.delta_e, 0.001));
        CHECK_THAT(lhs.feed, Catch::Matchers::WithinAbs(rhs.feed, 0.001));
    }
}

TEST_CASE("Tool changes select the filament cross section used by pressure smoothing", "[PressureEqualizer][PressureEqualizerStateTransitions]")
{
    const std::string input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nT0\nG1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X10 Y0 E1\nG1 F600 X20 Y0 E2\nG1 F630 X30 Y0 E3\nG1 F630 X40 Y0 E4\n"
        ";_EXTRUDE_END\nG0 X50 Y0 F1200\nT1\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X60 Y0 E5\nG1 F600 X70 Y0 E6\nG1 F630 X80 Y0 E7\nG1 F630 X90 Y0 E8\n"
        ";_EXTRUDE_END\n";

    const PressureToolObservation observation = observe_pressure_tools(equalize_pressure(input, false, false, {1.75, 3.5}));
    REQUIRE(observation.tool_changes == 1);
    CHECK(observation.tool0_extrusion_moves == 4);
    CHECK(observation.tool1_extrusion_moves > observation.tool0_extrusion_moves);
    CHECK_THAT(observation.tool0_extrusion, Catch::Matchers::WithinAbs(4.0, 0.001));
    CHECK_THAT(observation.tool1_extrusion, Catch::Matchers::WithinAbs(4.0, 0.001));
    CHECK_THAT(observation.final_x, Catch::Matchers::WithinAbs(90.0, 0.001));
}

} // namespace Slic3r
