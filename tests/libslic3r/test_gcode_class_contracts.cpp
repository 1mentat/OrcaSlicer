#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "gcode_class_contract_helpers.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode.hpp"
#include "libslic3r/GCodeWriter.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace gcode_broad_contracts {
using namespace Slic3r;

TEST_CASE("GCode coordinate conversion round-trips the configured origin and nozzle offset", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(10.5, -4.25));
    const Point point      = Point::new_scale(12.345, 6.789);
    const Vec2d emitted    = gcode.point_to_gcode(point);
    const Point round_trip = gcode.gcode_to_point(emitted);
    CHECK_THAT(unscale<double>(round_trip.x()), Catch::Matchers::WithinAbs(12.345, 1e-6));
    CHECK_THAT(unscale<double>(round_trip.y()), Catch::Matchers::WithinAbs(6.789, 1e-6));
    CHECK_THAT(emitted.x(), Catch::Matchers::WithinAbs(21.595, 1e-6));
    CHECK_THAT(emitted.y(), Catch::Matchers::WithinAbs(5.039, 1e-6));
}

TEST_CASE("GCode quantized coordinates stay within formatter precision", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(-2.0, 3.0));
    const Point point     = Point::new_scale(4.123456, -8.765432);
    const Vec2d raw       = gcode.point_to_gcode(point);
    const Vec2d quantized = gcode.point_to_gcode_quantized(point);
    CHECK_THAT(quantized.x(), Catch::Matchers::WithinAbs(raw.x(), 0.001));
    CHECK_THAT(quantized.y(), Catch::Matchers::WithinAbs(raw.y(), 0.001));
    CHECK(std::isfinite(quantized.x()));
}

TEST_CASE("Changing GCode origin preserves the model-space last position delta", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_origin(Vec2d(2.0, 3.0));
    const Point before = Point::new_scale(15.0, 16.0);
    gcode.set_last_pos(before);
    gcode.set_origin(Vec2d(7.0, -1.0));
    const Point after = gcode.last_pos();
    CHECK_THAT(unscale<double>(after.x() - before.x()), Catch::Matchers::WithinAbs(-5.0, 1e-6));
    CHECK_THAT(unscale<double>(after.y() - before.y()), Catch::Matchers::WithinAbs(4.0, 1e-6));
}

TEST_CASE("GCode layer interpolation handles initial, gradual, and stepped bands", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    gcode.set_layer_count(5);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.2);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.4);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f, 5.f), Catch::Matchers::WithinAbs(10.f, 1e-6));
    gcode.change_layer(0.6);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(15.f, 1e-6));
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f, 5.f), Catch::Matchers::WithinAbs(15.f, 1e-6));
    gcode.change_layer(0.8);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(17.5f, 1e-6));
    gcode.change_layer(1.0);
    CHECK_THAT(gcode.interpolate_value_across_layers(10.f, 20.f), Catch::Matchers::WithinAbs(20.f, 1e-6));
}

TEST_CASE("GCode extrusion-role parser names preserve every printable role", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    const std::vector<std::pair<ExtrusionRole, std::string>> roles = {
        {erExternalPerimeter, "ExternalPerimeter"},
        {erPerimeter, "Perimeter"},
        {erOverhangPerimeter, "OverhangPerimeter"},
        {erInternalInfill, "InternalInfill"},
        {erSolidInfill, "SolidInfill"},
        {erTopSolidInfill, "TopSolidInfill"},
        {erBottomSurface, "BottomSurface"},
        {erBridgeInfill, "BridgeInfill"},
        {erInternalBridgeInfill, "BridgeInfill"},
        {erGapFill, "GapFill"},
        {erIroning, "Ironing"},
        {erSkirt, "Skirt"},
        {erBrim, "Brim"},
        {erSupportMaterial, "SupportMaterial"},
        {erSupportMaterialInterface, "SupportMaterialInterface"},
        {erSupportTransition, "SupportTransition"},
        {erWipeTower, "WipeTower"},
    };
    for (const auto& [role, expected] : roles) {
        INFO("role " << expected);
        CHECK(gcode.extrusion_role_to_string_for_parser(role) == expected);
    }
    CHECK(gcode.extrusion_role_to_string_for_parser(erNone) == "Mixed");
    CHECK(gcode.extrusion_role_to_string_for_parser(erCustom) == "Mixed");
}

TEST_CASE("GCode direct extruder index is stable before Print binding", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    CHECK(gcode.cur_extruder_index() == 0);
    gcode.set_extruders({0, 1});
    gcode.writer().set_extruder(1);
    CHECK(gcode.cur_extruder_index() == 0);
}

TEST_CASE("GCode volumetric polynomial evaluates six coefficients and rejects malformed models", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    init_direct_gcode(gcode);
    CHECK_THAT(gcode.calc_max_volumetric_speed(0.2, 0.5, "1 2 3 4 5 6"), Catch::Matchers::WithinAbs(10.14, 1e-9));
    CHECK_THAT(gcode.calc_max_volumetric_speed(0.2, 0.5, "0 0 0 0 0 1"), Catch::Matchers::WithinAbs(1.0, 1e-9));
    CHECK(gcode.calc_max_volumetric_speed(0.2, 0.5, "1 2 3") >= std::numeric_limits<double>::max() / 2.0);
    CHECK(gcode.calc_max_volumetric_speed(0.2, 0.5, "0 0 0 0 0 0") >= std::numeric_limits<double>::max() / 2.0);
}

TEST_CASE("GCode lift policy maps every configured Z-hop mode", "[GCode][GCodeBroad]")
{
    TestGCode gcode;
    CHECK(gcode.to_lift_type(ZHopType::zhtNormal) == LiftType::NormalLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtSlope) == LiftType::SlopeLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtSpiral) == LiftType::SpiralLift);
    CHECK(gcode.to_lift_type(ZHopType::zhtAuto) == LiftType::NormalLift);
}

} // namespace gcode_broad_contracts
