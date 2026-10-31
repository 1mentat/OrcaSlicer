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

#include <algorithm>
#include <cmath>
#include <tuple>
#include <vector>

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

TEST_CASE("Fully anchored bridge spans the short side of the region", "[BridgeDetector]")
{
    const Polygons region{
        Polygon::new_scale({{0., 0.}, {30., 0.}, {30., 10.}, {0., 10.}}),
    };

    const auto [direction, unsupported_length] = detect_bridging_direction(Lines{}, region);

    CHECK(direction.array().isFinite().all());
    CHECK_THAT(direction.norm(), Catch::Matchers::WithinAbs(1., 1e-12));
    // With no floating edge the minor principal axis is used, which is the 10 mm side.
    CHECK(std::abs(direction.y()) > 0.99);
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
    // The coverage must span the unsupported gap, not just sit on one anchor.
    const Point gap_center = Point::new_scale(20., 10.);
    CHECK(contains(precise, gap_center));
    CHECK(contains(coarse, gap_center));

    const Polylines unsupported = detector.unsupported_edges(PI / 2.);
    // The anchors, grown by the 1 mm spacing, support x <= 7 and x >= 33. Bridging across Y leaves
    // the top and bottom edges unsupported over that gap; the vertical edges sit on the anchors.
    REQUIRE(unsupported.size() == 2);
    std::vector<coord_t> edge_ys;
    for (const Polyline &edge : unsupported) {
        REQUIRE(edge.size() == 2);
        CHECK(edge.first_point().y() == edge.last_point().y());
        edge_ys.push_back(edge.first_point().y());
        const coord_t x0 = edge.first_point().x();
        const coord_t x1 = edge.last_point().x();
        CHECK_THAT(unscale_(std::min(x0, x1)), Catch::Matchers::WithinAbs(7., 0.01));
        CHECK_THAT(unscale_(std::max(x0, x1)), Catch::Matchers::WithinAbs(33., 0.01));
    }
    std::sort(edge_ys.begin(), edge_ys.end());
    CHECK(edge_ys == std::vector<coord_t>{0, coord_t(scale_(20.))});

    // Along the detected direction the only unsupported edges run parallel to the bridge.
    CHECK(detector.unsupported_edges().empty());
}

TEST_CASE("Bridge detector rejects a bridge with no lower anchors", "[BridgeDetector]")
{
    const ExPolygons lower;
    BridgeDetector detector(box(0., 0., 20., 20.), lower, scale_(1.));

    CHECK_FALSE(detector.detect_angle());
    CHECK(detector.coverage().empty());
    CHECK(detector.unsupported_edges().empty());
    CHECK(detector.coverage(0.).empty());
    // With an explicit direction, every edge not parallel to it is unsupported: the two vertical sides.
    const Polylines unsupported = detector.unsupported_edges(0.);
    REQUIRE(unsupported.size() == 2);
    for (const Polyline &edge : unsupported)
        CHECK(edge.first_point().x() == edge.last_point().x());
}

TEST_CASE("Bridge detector ignores a lower island inside the bridge outline", "[BridgeDetector]")
{
    // The island touches the bridge but never crosses its grown outline, so it yields no anchoring
    // edge and the detector rejects the bridge even though the island overlaps it.
    const ExPolygons lower{box(0., 0., 6., 20.)};
    BridgeDetector detector(box(0., 0., 40., 20.), lower, scale_(1.));

    CHECK_FALSE(detector.detect_angle());
    CHECK(detector.coverage().empty());
    CHECK(detector.coverage(0., true).empty());
}
