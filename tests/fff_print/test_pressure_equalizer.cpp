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

std::string equalize_pressure(const std::string &input, bool relative_e, bool external_only)
{
    Slic3r::GCodeConfig config;
    config.filament_diameter.values = {1.75};
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

} // namespace Slic3r
