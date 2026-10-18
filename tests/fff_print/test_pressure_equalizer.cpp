#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCode/PressureEqualizer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace Slic3r {
namespace {

struct PressureObservation {
    std::size_t extrusion_moves = 0;
    std::size_t speed_markers = 0;
    double total_extrusion = 0.0;
    double final_x = 0.0;
    std::vector<float> feedrates;
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
                              const std::vector<double> &filament_diameters = {1.75})
{
    Slic3r::GCodeConfig config;
    config.filament_diameter.values = filament_diameters;
    config.use_relative_e_distances.value = relative_e;
    config.max_volumetric_extrusion_rate_slope.value = 1.0;
    config.max_volumetric_extrusion_rate_slope_segment_length.value = 2.0;
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
        if (line.cmd_is("G1") && line.has_f())
            observation.feedrates.push_back(line.f());
        if (!line.cmd_is("G1") || !line.extruding(state))
            return;
        ++observation.extrusion_moves;
        observation.total_extrusion += line.dist_E(state);
        observation.final_x = line.new_X(state);
    });
    return observation;
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
    REQUIRE_THAT(external.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
    CHECK(std::any_of(external.feedrates.begin(), external.feedrates.end(), [](float feedrate) {
        return feedrate > 600.f && feedrate < 1200.f;
    }));

    CHECK(internal_control.extrusion_moves == 4);
    CHECK(internal_control.speed_markers == 1);
    CHECK_THAT(internal_control.total_extrusion, Catch::Matchers::WithinAbs(6.0, 0.001));
    CHECK_THAT(internal_control.final_x, Catch::Matchers::WithinAbs(40.0, 0.001));
}

namespace {

struct PressureStateObservation {
    std::size_t g92_lines = 0;
    std::size_t post_reset_extrusion_moves = 0;
    double post_reset_distance = 0.0;
    double post_reset_extrusion = 0.0;
    double first_post_reset_start_x = 0.0;
    double first_post_reset_end_x = 0.0;
    bool saw_post_reset_extrusion = false;
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
        const double end_x = line.new_X(state);
        if (!observation.saw_post_reset_extrusion) {
            observation.first_post_reset_start_x = start_x;
            observation.first_post_reset_end_x = end_x;
            observation.saw_post_reset_extrusion = true;
        }
        ++observation.post_reset_extrusion_moves;
        observation.post_reset_distance += line.dist_XY(state);
        observation.post_reset_extrusion += line.dist_E(state);
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
    const std::string input =
        ";_EXTRUSION_ROLE:" + std::to_string(int(Slic3r::erExternalPerimeter)) +
        "\nG1 F600 X10 Y0 E1\nG92 X0 Y0 E5\n"
        "G1 F600 ;_EXTRUDE_SET_SPEED\n"
        "G1 F600 X10 Y0 E6\nG1 F600 X20 Y0 E7\nG1 F630 X30 Y0 E8\nG1 F630 X40 Y0 E9\n"
        ";_EXTRUDE_END\n";

    const PressureStateObservation observation = observe_g92_state(equalize_pressure(input, false, false, {1.75}));
    REQUIRE(observation.g92_lines == 1);
    REQUIRE(observation.saw_post_reset_extrusion);
    CHECK(observation.post_reset_extrusion_moves == 4);
    CHECK_THAT(observation.post_reset_extrusion, Catch::Matchers::WithinAbs(4.0, 0.001));
    CHECK_THAT(observation.post_reset_distance, Catch::Matchers::WithinAbs(40.0, 0.001));
    CHECK_THAT(observation.first_post_reset_start_x, Catch::Matchers::WithinAbs(0.0, 0.001));
    CHECK_THAT(observation.first_post_reset_end_x, Catch::Matchers::WithinAbs(10.0, 0.001));
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
