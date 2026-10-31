#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BridgeDetector.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <tuple>

using namespace Slic3r;

namespace {

ExPolygon box(double x0, double y0, double x1, double y1)
{
    return ExPolygon(Polygon::new_scale({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}));
}

} // namespace

TEST_CASE("Bridge direction follows the long unsupported edge", "[BridgeDetector]")
{
    const Lines floating_edges{
        Line(Point::new_scale(0., 0.), Point::new_scale(30., 0.)),
    };

    const auto [direction, unsupported_length] = detect_bridging_direction(floating_edges, {});

    CHECK_THAT(direction.norm(), Catch::Matchers::WithinAbs(1., 1e-12));
    CHECK(std::abs(direction.x()) > 0.99);
    CHECK(std::abs(direction.y()) < 0.01);
    CHECK_THAT(unsupported_length, Catch::Matchers::WithinAbs(0., 1e-9));
}

TEST_CASE("Fully anchored bridge direction remains finite", "[BridgeDetector]")
{
    const Polygons region{
        Polygon::new_scale({{0., 0.}, {30., 0.}, {30., 10.}, {0., 10.}}),
    };

    const auto [direction, unsupported_length] = detect_bridging_direction(Lines{}, region);

    CHECK(direction.array().isFinite().all());
    CHECK_THAT(direction.norm(), Catch::Matchers::WithinAbs(1., 1e-12));
    CHECK_THAT(unsupported_length, Catch::Matchers::WithinAbs(0., 1e-9));
}

TEST_CASE("Bridge detector finds coverage between two lower anchors", "[BridgeDetector]")
{
    const ExPolygon bridge = box(0., 0., 40., 20.);
    const ExPolygons lower{
        box(-5., 0., 6., 20.),
        box(34., 0., 45., 20.),
    };
    BridgeDetector detector(bridge, lower, scale_(1.));

    REQUIRE(detector.detect_angle());
    CHECK(detector.angle >= 0.);
    CHECK(detector.angle < PI);
    CHECK_THAT(std::abs(std::cos(detector.angle)), Catch::Matchers::WithinAbs(1., 0.02));

    const Polygons precise = detector.coverage(-1., true);
    const Polygons coarse = detector.coverage(-1., false);
    REQUIRE_FALSE(precise.empty());
    REQUIRE_FALSE(coarse.empty());
    CHECK(precise.front().area() > 0);
    CHECK(coarse.front().area() > 0);

    const Polylines unsupported = detector.unsupported_edges(PI / 2.);
    REQUIRE_FALSE(unsupported.empty());
    for (const Polyline &edge : unsupported) {
        CHECK(edge.length() > 0);
        for (const Line &line : edge.lines())
            CHECK_FALSE(Geometry::directions_parallel(line.direction(), PI / 2.));
    }
}

TEST_CASE("Bridge detector rejects a bridge with no lower anchors", "[BridgeDetector]")
{
    const ExPolygons lower;
    BridgeDetector detector(box(0., 0., 20., 20.), lower, scale_(1.));

    CHECK_FALSE(detector.detect_angle());
    CHECK(detector.coverage().empty());
    CHECK(detector.unsupported_edges().empty());
}

TEST_CASE("Bridge detector requires two anchored sides for coverage", "[BridgeDetector]")
{
    const ExPolygons lower{box(0., 0., 6., 20.)};
    BridgeDetector detector(box(0., 0., 40., 20.), lower, scale_(1.));

    CHECK_FALSE(detector.detect_angle());
    CHECK(detector.coverage().empty());
    CHECK(detector.coverage(0., true).empty());
}
