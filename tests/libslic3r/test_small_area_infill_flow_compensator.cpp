#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Exception.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/SmallAreaInfillFlowCompensator.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

TEST_CASE("Small-area flow compensation scales only its supported extrusion roles", "[SmallAreaInfillFlowCompensator]")
{
    GCodeConfig config;
    config.small_area_infill_flow_compensation_model.values = {"0,0.5", "1,0.75", "2,1.0"};
    SmallAreaInfillFlowCompensator compensator(config);

    // Knots make the expected interpolation independent of PCHIP's interior
    // shape.  The explicit zero and upper-bound cases cover the two bypasses.
    CHECK_THAT(compensator.modify_flow(1.0, 8.0, erSolidInfill), WithinAbs(6.0, 1e-9));
    CHECK_THAT(compensator.modify_flow(1.0, 8.0, erTopSolidInfill), WithinAbs(6.0, 1e-9));
    CHECK_THAT(compensator.modify_flow(1.0, 8.0, erBottomSurface), WithinAbs(6.0, 1e-9));
    CHECK_THAT(compensator.modify_flow(1.0, 8.0, erPerimeter), WithinAbs(8.0, 1e-9));
    CHECK_THAT(compensator.modify_flow(0.0, 8.0, erSolidInfill), WithinAbs(8.0, 1e-9));
    CHECK_THAT(compensator.modify_flow(3.0, 8.0, erSolidInfill), WithinAbs(8.0, 1e-9));
}

TEST_CASE("Small-area compensation preserves flow at and beyond the model endpoint", "[SmallAreaInfillFlowCompensator]")
{
    GCodeConfig config;
    config.small_area_infill_flow_compensation_model.values = {"0,0.5", "1,1.0"};
    SmallAreaInfillFlowCompensator compensator(config);

    CHECK_THAT(compensator.modify_flow(1.0, 3.25, erSolidInfill), WithinAbs(3.25, 1e-9));
    CHECK_THAT(compensator.modify_flow(100.0, 3.25, erBottomSurface), WithinAbs(3.25, 1e-9));
}

TEST_CASE("Small-area compensation rejects malformed and non-monotonic models", "[SmallAreaInfillFlowCompensator]")
{
    SECTION("the first length is zero")
    {
        GCodeConfig config;
        config.small_area_infill_flow_compensation_model.values = {"0.1,0.5", "1,1.0"};
        CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(config), InvalidArgument);
    }
    SECTION("lengths increase strictly")
    {
        GCodeConfig config;
        config.small_area_infill_flow_compensation_model.values = {"0,0.5", "1,0.7", "1,1.0"};
        CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(config), InvalidArgument);
    }
    SECTION("factors increase strictly")
    {
        GCodeConfig config;
        config.small_area_infill_flow_compensation_model.values = {"0,0.8", "1,0.7", "2,1.0"};
        CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(config), InvalidArgument);
    }
    SECTION("the final factor is identity")
    {
        GCodeConfig config;
        config.small_area_infill_flow_compensation_model.values = {"0,0.5", "1,0.9"};
        CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(config), InvalidArgument);
    }
    SECTION("numeric fields are parseable")
    {
        GCodeConfig config;
        config.small_area_infill_flow_compensation_model.values = {"0,not-a-number"};
        CHECK_THROWS_AS(SmallAreaInfillFlowCompensator(config), InvalidArgument);
    }
}
