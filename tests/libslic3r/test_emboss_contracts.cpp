#include <initializer_list>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cstddef>
#include <utility>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Emboss.hpp"

using namespace Slic3r;

namespace emboss_extra {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

Polygon scaled_polygon(std::initializer_list<std::pair<double, double>> points)
{
    Polygon result;
    for (const auto &[x, y] : points)
        result.points.push_back(Point::new_scale(x, y));
    return result;
}

ExPolygon square(double x, double y, double side)
{
    return ExPolygon(scaled_polygon({{x, y}, {x + side, y}, {x + side, y + side}, {x, y + side}}));
}

double area_of(const ExPolygons &shape)
{
    double area = 0.;
    for (const ExPolygon &expolygon : shape)
        area += expolygon.area();
    return area;
}

} // namespace

TEST_CASE("Healing removes self intersections while retaining filled area", "[Emboss][EmbossHealing]")
{
    // A bow tie: two 20 x 10 triangles meeting at its centre. Left self-crossing, its
    // signed area is 0, so the full 200 mm^2 shows the crossing was resolved.
    const Polygons bowtie{scaled_polygon({{0., 0.}, {20., 20.}, {0., 20.}, {20., 0.}})};
    const HealedExPolygons healed = Emboss::heal_polygons(bowtie, true, 10);
    REQUIRE_FALSE(healed.expolygons.empty());
    CHECK(healed.is_healed);
    CHECK_THAT(area_of(healed.expolygons), WithinRel(200. / (SCALING_FACTOR * SCALING_FACTOR), 1e-2));

    // A repeated neighbour point is dropped rather than cut out as a duplicate.
    ExPolygons duplicate{ExPolygon(scaled_polygon({{0., 0.}, {20., 0.}, {20., 20.}, {20., 20.}, {0., 20.}}))};
    CHECK(Emboss::heal_expolygons(duplicate, 10));
    REQUIRE(duplicate.size() == 1);
    CHECK(duplicate.front().contour.size() == 4);
    CHECK_THAT(area_of(duplicate), WithinRel(400. / (SCALING_FACTOR * SCALING_FACTOR), 1e-9));
}

static ExPolygons close_point_shape()
{
    // The triangle's tip at (50, 2) lies 2 mm above the square's bottom edge.
    return {
        ExPolygon(scaled_polygon({{0., 0.}, {100., 0.}, {100., 100.}, {0., 100.}})),
        ExPolygon(scaled_polygon({{50., 2.}, {55., 10.}, {45., 10.}})),
    };
}

// Known bug: divide_segments_for_close_point passes the radius to AABBTreeLines::all_lines_in_radius, which expects
// it squared, so the search radius is sqrt(distance); fixed by the follow-up fix PR.
TEST_CASE("Close-point division honors a Euclidean radius boundary", "[Emboss][EmbossHealing][!shouldfail]")
{
    const double unit = 1. / SCALING_FACTOR;
    const size_t original_points = close_point_shape().front().contour.size();

    ExPolygons below = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(below, 1.9 * unit));
    CHECK(below.front().contour.size() == original_points);

    ExPolygons above = close_point_shape();
    REQUIRE(Emboss::divide_segments_for_close_point(above, 2.1 * unit));
    CHECK(above.front().contour.size() == original_points + 1);
    CHECK(std::count(above.front().contour.points.begin(), above.front().contour.points.end(),
                     Point::new_scale(50., 2.)) == 1);

    ExPolygons zero = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(zero, 0.));
    CHECK(zero.front().contour.size() == original_points);

    ExPolygons negative = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(negative, -unit));
    CHECK(negative.front().contour.size() == original_points);

    ExPolygons empty;
    CHECK_FALSE(Emboss::divide_segments_for_close_point(empty, 2.0 * unit));
}

TEST_CASE("Union-with-delta merges overlapping glyph shapes and caches the healed result", "[Emboss][EmbossHealing]")
{
    EmbossShape shape;
    shape.shapes_with_ids = {
        {10u, {square(0., 0., 20.)}, true},
        {11u, {square(10., 0., 20.)}, true},
    };
    const ExPolygons first = Slic3r::union_with_delta(shape, 1.f, 10);
    // Two 20 x 20 squares overlap by a 10 x 20 rectangle.
    REQUIRE(first.size() == 1);
    CHECK_THAT(area_of(first), WithinRel(600. / (SCALING_FACTOR * SCALING_FACTOR), 1e-3));
    CHECK(shape.final_shape.is_healed);

    // The cached final shape is returned even after the source shapes change.
    shape.shapes_with_ids.clear();
    const ExPolygons cached = Slic3r::union_with_delta(shape, 100.f, 0);
    REQUIRE(cached.size() == first.size());
    CHECK_THAT(area_of(cached), WithinAbs(area_of(first), 1e-6));
}

} // namespace emboss_extra

namespace emboss_spike_followups {

using Catch::Matchers::WithinAbs;

namespace {

Point emboss_point(coord_t x, coord_t y)
{
    // Emboss healing operates on raw glyph-grid coordinates.  Point(x, y)
    // deliberately avoids Point::new_scale, which would rescale these units.
    return Point(x, y);
}

ExPolygon broad_triangle()
{
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(-10000, 0),
                             emboss_point(0, -10000)});
}

ExPolygon sharp_duplicate_island()
{
    // The fixed duplicate-spike half-bevel is 500 raw units.  Both legs are
    // shorter than that and their angle is below the five-pixel threshold.
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(300, 0),
                             emboss_point(450, 20)});
}

ExPolygon blunt_duplicate_island()
{
    // The same shared origin and short legs, but a 45 degree corner, provide
    // a duplicate-point control that must not be classified as a spike.
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(300, 0),
                             emboss_point(300, 300)});
}

void check_positive_oriented_contours(const ExPolygons &shape)
{
    for (const ExPolygon &expolygon : shape) {
        REQUIRE(expolygon.contour.points.size() >= 3);
        CHECK(expolygon.contour.area() > 0.);
        CHECK(expolygon.holes.empty());
    }
}

} // namespace

TEST_CASE("Healing removes a sharp duplicate-point island and preserves the broad contour", "[Emboss][EmbossHealing]")
{
    const ExPolygon broad = broad_triangle();
    const double broad_area = broad.area();
    ExPolygons shape{sharp_duplicate_island(), broad};

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    REQUIRE(shape.size() == 1);
    check_positive_oriented_contours(shape);
    CHECK_THAT(shape.front().area(), WithinAbs(broad_area, 4.));
}

TEST_CASE("Healing retains a non-spike duplicate-point island as a separate contour", "[Emboss][EmbossHealing]")
{
    const ExPolygon broad = broad_triangle();
    const double broad_area = broad.area();
    const double blunt_area = blunt_duplicate_island().area();
    ExPolygons shape{blunt_duplicate_island(), broad};

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    REQUIRE(shape.size() == 2);
    check_positive_oriented_contours(shape);

    // Only the 3 x 3 cut around the shared point is lost from each part.
    std::sort(shape.begin(), shape.end(), [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    CHECK_THAT(shape.back().area(), WithinAbs(broad_area, 4.));
    CHECK_THAT(shape.front().area(), WithinAbs(blunt_area, 4.));
}

TEST_CASE("Healing leaves a shape without duplicate points unchanged", "[Emboss][EmbossHealing]")
{
    ExPolygons shape{broad_triangle()};
    const ExPolygons original = shape;

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    CHECK(shape == original);
}

} // namespace emboss_spike_followups
