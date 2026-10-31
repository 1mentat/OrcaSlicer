#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Circle.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include <cmath>
#include <algorithm>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
Point p(double x, double y)
{
    return Point::new_scale(x, y);
}

Point3 p3(double x, double y, double z)
{
    return Point3::new_scale(x, y, z);
}

void require_point(const Point &actual, double x, double y)
{
    CHECK_THAT(unscaled(actual.x()), WithinAbs(x, 0.002));
    CHECK_THAT(unscaled(actual.y()), WithinAbs(y, 0.002));
}
} // namespace

TEST_CASE("Arc slice containment follows direction and wraps through zero", "[Circle][CircleArcPublic]")
{
    const Point center = p(0., 0.);
    const double radius = scaled(5.);

    const ArcSegment ccw(center, radius, p(5., 0.), p(0., 5.), ArcDirection::Arc_Dir_CCW);
    const Points3 ccw_points{p3(5., 0., 1.), p3(4., 3., 8.), p3(0., 5., 15.)};
    CHECK(ArcSegment::are_points_within_slice(ccw, ccw_points));
    const Points3 ccw_outside{p3(5., 0., 1.), p3(-5., 0., 8.), p3(0., 5., 15.)};
    CHECK_FALSE(ArcSegment::are_points_within_slice(ccw, ccw_outside));

    const ArcSegment cw(center, radius, p(0., 5.), p(5., 0.), ArcDirection::Arc_Dir_CW);
    const Points3 cw_points{p3(0., 5., 1.), p3(4., 3., 8.), p3(5., 0., 15.)};
    CHECK(ArcSegment::are_points_within_slice(cw, cw_points));
    const Points3 cw_outside{p3(0., 5., 1.), p3(-5., 0., 8.), p3(5., 0., 15.)};
    CHECK_FALSE(ArcSegment::are_points_within_slice(cw, cw_outside));

    // These are exact radius-65 points; the first and last straddle the zero-angle ray.
    const Point wrap_start = p(56., -33.);
    const Point wrap_end = p(56., 33.);
    const double wrap_radius = std::hypot(double(wrap_start.x()), double(wrap_start.y()));
    const ArcSegment wrap_ccw(center, wrap_radius, wrap_start, wrap_end, ArcDirection::Arc_Dir_CCW);
    const Points3 wrap_ccw_points{p3(56., -33., 2.), p3(63., 16., 11.), p3(56., 33., 20.)};
    CHECK(ArcSegment::are_points_within_slice(wrap_ccw, wrap_ccw_points));

    const ArcSegment wrap_cw(center, wrap_radius, wrap_end, wrap_start, ArcDirection::Arc_Dir_CW);
    const Points3 wrap_cw_points{p3(56., 33., 2.), p3(63., 16., 11.), p3(56., -33., 20.)};
    CHECK(ArcSegment::are_points_within_slice(wrap_cw, wrap_cw_points));
    const Points3 wrap_ccw_outside{p3(56., -33., 2.), p3(-65., 0., 11.), p3(56., 33., 20.)};
    CHECK_FALSE(ArcSegment::are_points_within_slice(wrap_ccw, wrap_ccw_outside));
    const Points3 wrap_cw_outside{p3(56., 33., 2.), p3(-65., 0., 11.), p3(56., -33., 20.)};
    CHECK_FALSE(ArcSegment::are_points_within_slice(wrap_cw, wrap_cw_outside));
}

TEST_CASE("Arc slice containment accepts several interior points and rejects one outside before the last", "[Circle][CircleArcPublic]")
{
    const ArcSegment ccw(p(0., 0.), scaled(5.), p(5., 0.), p(0., 5.), ArcDirection::Arc_Dir_CCW);
    const Points3 inside{p3(5., 0., 0.), p3(4.8, 1.4, 1.), p3(4., 3., 2.), p3(3., 4., 3.), p3(0., 5., 4.)};
    CHECK(ArcSegment::are_points_within_slice(ccw, inside));
    const Points3 outside_before_last{p3(5., 0., 0.), p3(4., 3., 1.), p3(-5., 0., 2.), p3(3., 4., 3.), p3(0., 5., 4.)};
    CHECK_FALSE(ArcSegment::are_points_within_slice(ccw, outside_before_last));
}

// Known bug: are_points_within_slice only checks the last two points, so an earlier point outside the slice is accepted; fixed by the follow-up fix PR
TEST_CASE("Arc slice containment rejects an early point outside the slice", "[Circle][CircleArcPublic][!shouldfail]")
{
    const ArcSegment ccw(p(0., 0.), scaled(5.), p(5., 0.), p(0., 5.), ArcDirection::Arc_Dir_CCW);
    // (-5, 0) lies on the circle but outside the CCW quarter arc from (5, 0) to (0, 5).
    CHECK_FALSE(ArcSegment::are_points_within_slice(ccw, Points3{p3(5., 0., 0.), p3(-5., 0., 1.), p3(4., 3., 2.), p3(3., 4., 3.), p3(0., 5., 4.)}));
    CHECK_FALSE(ArcSegment::are_points_within_slice(ccw, Points{p(5., 0.), p(-5., 0.), p(4., 3.), p(3., 4.), p(0., 5.)}));
}

TEST_CASE("Points3 arc fitting recovers a known XY circle", "[Circle][CircleArcPublic]")
{
    const bool clockwise = GENERATE(false, true);
    // These are exact radius-13 points around (20, 30); Z is intentionally varied because fitting is XY-only.
    Points3 points{p3(33., 30., 2.), p3(32., 35., 7.), p3(20., 43., 13.)};
    if (clockwise)
        std::reverse(points.begin(), points.end());
    ArcSegment fitted;
    const double expected_length = scaled(13.) * PI / 2.;
    REQUIRE(ArcSegment::try_create_arc(points, fitted, expected_length, scaled(14.), scaled(3.), 0.05));

    REQUIRE(fitted.is_valid());
    require_point(fitted.center, 20., 30.);
    CHECK_THAT(unscaled(fitted.radius), WithinAbs(13., 0.002));
    CHECK(fitted.direction == (clockwise ? ArcDirection::Arc_Dir_CW : ArcDirection::Arc_Dir_CCW));
    require_point(fitted.start_point, clockwise ? 20. : 33., clockwise ? 43. : 30.);
    require_point(fitted.end_point, clockwise ? 33. : 20., clockwise ? 30. : 43.);
    CHECK_THAT(fitted.angle_radians, WithinAbs((clockwise ? -1. : 1.) * PI / 2., 1e-9));
    CHECK_THAT(unscaled(fitted.length), WithinAbs(13. * PI / 2., 0.01));
}

TEST_CASE("Arc fitting rejects collinear, over-radius, and impossible-length inputs", "[Circle][CircleArcPublic]")
{
    const Points3 valid_points{p3(33., 30., 2.), p3(32., 35., 7.), p3(20., 43., 13.)};
    const double expected_length = scaled(13.) * PI / 2.;
    ArcSegment target;

    const Points3 collinear{p3(0., 0., 1.), p3(5., 0., 5.), p3(10., 0., 9.)};
    CHECK_FALSE(ArcSegment::try_create_arc(collinear, target, scaled(10.)));
    CHECK_FALSE(ArcSegment::try_create_arc(valid_points, target, expected_length, scaled(12.), scaled(3.)));
    CHECK_FALSE(ArcSegment::try_create_arc(valid_points, target, scaled(100.), scaled(14.), scaled(3.)));
}
