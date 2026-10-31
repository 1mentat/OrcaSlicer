#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/EdgeGrid.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
Polygon rectangle(coord_t x0, coord_t y0, coord_t x1, coord_t y1)
{
    return Polygon(Points{Point(x0, y0), Point(x1, y0), Point(x1, y1), Point(x0, y1)});
}

double absolute_area(const Polygons &polygons)
{
    double area = 0.;
    for (const Polygon &polygon : polygons)
        area += std::abs(polygon.area());
    return area;
}

} // namespace

TEST_CASE("EdgeGrid reports signed edge distances and closest points", "[EdgeGrid]")
{
    const Polygon outer = rectangle(0, 0, 100, 100);
    EdgeGrid::Grid grid;
    const Polygons contours{outer};
    grid.create(contours, 10);

    REQUIRE(grid.contours().size() == 1);
    REQUIRE(grid.resolution() == 10);
    REQUIRE(grid.bbox().min == Point(-16, -16));
    REQUIRE(grid.bbox().max == Point(116, 116));

    coordf_t distance = 0.;
    bool on_segment = false;
    REQUIRE(grid.signed_distance_edges(Point(50, 50), 100, distance, &on_segment));
    CHECK_THAT(distance, WithinAbs(-50., 1e-6));
    CHECK(on_segment);

    const auto closest = grid.closest_point_signed_distance(Point(50, 50), 100);
    REQUIRE(closest.valid());
    CHECK(closest.contour_idx == 0);
    CHECK(closest.start_point_idx < 4);
    CHECK_THAT(closest.t, WithinAbs(.5, 1e-6));
    CHECK_THAT(closest.distance, WithinAbs(-50., 1e-6));
}

TEST_CASE("EdgeGrid rejects a point outside its search radius", "[EdgeGrid]")
{
    EdgeGrid::Grid grid;
    const Polygons contours{rectangle(0, 0, 100, 100)};
    grid.create(contours, 10);

    coordf_t distance = 0.;
    CHECK_FALSE(grid.signed_distance_edges(Point(50, 50), 50, distance));
    CHECK_FALSE(grid.closest_point_signed_distance(Point(50, 50), 50).valid());

    REQUIRE(grid.signed_distance_edges(Point(50, 50), 51, distance));
    CHECK_THAT(distance, WithinAbs(-50., 1e-6));
}

TEST_CASE("EdgeGrid SDF preserves the inside/outside sign away from edges", "[EdgeGrid]")
{
    EdgeGrid::Grid grid;
    const Polygons contours{rectangle(0, 0, 100, 100)};
    grid.create(contours, 10);
    grid.calculate_sdf();

    coordf_t distance = 0.;
    REQUIRE(grid.signed_distance(Point(50, 50), 1, distance));
    CHECK(distance < 0.);
    REQUIRE(grid.signed_distance(Point(50, 10), 1, distance));
    CHECK(distance < 0.);
    CHECK(grid.signed_distance_bilinear(Point(50, 50)) < 0.);
    CHECK(grid.signed_distance_bilinear(Point(110, 50)) > 0.);
    CHECK_THAT(grid.signed_distance_bilinear(Point(50, 10)), WithinAbs(distance, 1e-4));
}

TEST_CASE("EdgeGrid keeps open and closed input contours distinct", "[EdgeGrid]")
{
    const Points open{Point(0, 0), Point(100, 0), Point(100, 100)};
    EdgeGrid::Grid grid;
    const std::vector<Points> open_contours{open};
    grid.create(open_contours, 10, true);
    REQUIRE(grid.contours().size() == 1);
    CHECK(grid.contours().front().open());
    CHECK(grid.contours().front().num_segments() == 2);

    const Points explicitly_closed{Point(0, 0), Point(100, 0), Point(100, 100), Point(0, 0)};
    const std::vector<Points> closed_contours{explicitly_closed};
    grid.create(closed_contours, 10, true);
    REQUIRE(grid.contours().size() == 1);
    CHECK(grid.contours().front().closed());
    CHECK(grid.contours().front().num_segments() == 3);
}

TEST_CASE("EdgeGrid visits every raster cell crossed by a line", "[EdgeGrid]")
{
    EdgeGrid::Grid grid;
    const Polygons contours{rectangle(0, 0, 100, 100)};
    grid.create(contours, 10);

    std::vector<std::pair<size_t, size_t>> visited;
    auto visitor = [&visited](size_t row, size_t col) {
        visited.emplace_back(row, col);
        return true;
    };
    grid.visit_cells_intersecting_line(Point(1, 1), Point(99, 99), visitor);

    REQUIRE(visited.size() >= 2);
    CHECK(visited.front() == std::pair<size_t, size_t>(1, 1));
    CHECK(visited.back() == std::pair<size_t, size_t>(11, 11));
    CHECK(std::adjacent_find(visited.begin(), visited.end()) == visited.end());
    for (size_t cell = 1; cell <= 11; ++cell)
        CHECK(std::find(visited.begin(), visited.end(), std::make_pair(cell, cell)) != visited.end());

    visited.clear();
    grid.visit_cells_intersecting_line(Point(1, 1), Point(99, 1), visitor);
    REQUIRE(visited.size() == 11);
    for (size_t column = 1; column <= 11; ++column)
        CHECK(visited[column - 1] == std::make_pair(size_t(1), column));

    size_t calls = 0;
    auto stop_after_one = [&calls](size_t, size_t) {
        ++calls;
        return false;
    };
    grid.visit_cells_intersecting_line(Point(1, 1), Point(99, 99), stop_after_one);
    CHECK(calls == 1);
}

TEST_CASE("EdgeGrid detects crossing edges without flagging a simple contour", "[EdgeGrid]")
{
    EdgeGrid::Grid square;
    const Polygons square_contours{rectangle(0, 0, 100, 100)};
    square.create(square_contours, 10);
    CHECK_FALSE(square.has_intersecting_edges());
    CHECK(square.intersecting_edges().empty());

    EdgeGrid::Grid bowtie;
    const Polygons crossing_contours{
        Polygon(Points{Point(0, 0), Point(100, 100), Point(0, 100)}),
        Polygon(Points{Point(0, 100), Point(100, 0), Point(100, 100)})};
    bowtie.create(crossing_contours, 10);
    CHECK(bowtie.has_intersecting_edges());
    CHECK_FALSE(bowtie.intersecting_edges().empty());
}

TEST_CASE("EdgeGrid simplifies an ExPolygon with optional hole filling", "[EdgeGrid]")
{
    ExPolygon expoly(rectangle(0, 0, 100, 100));
    expoly.holes.emplace_back(rectangle(45, 45, 65, 65));
    expoly.holes.back().reverse();
    EdgeGrid::Grid grid;
    grid.create(expoly, 10);
    grid.calculate_sdf();

    const Polygons keep_hole = grid.contours_simplified(0, false);
    const Polygons fill_hole = grid.contours_simplified(0, true);
    REQUIRE_FALSE(keep_hole.empty());
    REQUIRE_FALSE(fill_hole.empty());
    INFO("keep_count=" << keep_hole.size() << " fill_count=" << fill_hole.size());
    CHECK(absolute_area(fill_hole) > 0.);
    CHECK(absolute_area(keep_hole) > 0.);
    CHECK(keep_hole.size() == 2);
    CHECK(fill_hole.size() == 1);
}

TEST_CASE("EdgeGrid handles mixed polygons and polylines", "[EdgeGrid]")
{
    const Polygons polygons{rectangle(0, 0, 100, 100)};
    const Polylines polylines{Polyline{Point(10, 10), Point(90, 90)}};
    EdgeGrid::Grid grid;
    grid.create(polygons, polylines, 10);

    REQUIRE(grid.contours().size() == 2);
    CHECK(grid.contours()[0].open());
    CHECK(grid.contours()[1].closed());
    CHECK(grid.contours()[0].num_segments() == 1);
    CHECK(grid.contours()[1].num_segments() == 4);
}

TEST_CASE("EdgeGrid reports convex vertex distance and segment status", "[EdgeGrid]")
{
    const Polygon outer = rectangle(0, 0, 100, 100);
    const Polygons contours{outer};
    EdgeGrid::Grid grid;
    grid.create(contours, 10);

    const Point near_vertex(-3, -3);
    const double expected = std::sqrt(18.0);
    coordf_t edge_distance = 0.;
    bool on_segment = true;
    REQUIRE(grid.signed_distance_edges(near_vertex, 20, edge_distance, &on_segment));
    CHECK_THAT(edge_distance, WithinAbs(expected, 1e-6));
    CHECK_FALSE(on_segment);

    const auto closest = grid.closest_point_signed_distance(near_vertex, 20);
    REQUIRE(closest.valid());
    REQUIRE(closest.contour_idx < grid.contours().size());
    const auto &contour = grid.contours()[closest.contour_idx];
    REQUIRE(closest.start_point_idx < contour.num_segments());
    CHECK(closest.t >= 0.);
    CHECK(closest.t <= 1.);
    const Line segment = contour.get_segment(closest.start_point_idx);
    const Vec2d nearest_point = segment.a.cast<double>() * (1. - closest.t) +
                                segment.b.cast<double>() * closest.t;
    CHECK_THAT(nearest_point(0), WithinAbs(0., 1e-6));
    CHECK_THAT(nearest_point(1), WithinAbs(0., 1e-6));
    CHECK_THAT(closest.distance, WithinAbs(expected, 1e-6));
}

TEST_CASE("EdgeGrid extrapolates positive SDF values beyond each grid boundary", "[EdgeGrid]")
{
    const Polygon outer = rectangle(0, 0, 100, 100);
    const Polygons contours{outer};
    EdgeGrid::Grid grid;
    grid.create(contours, 10);
    grid.calculate_sdf();

    const BoundingBox &box = grid.bbox();
    const coord_t width = grid.resolution() * coord_t(grid.cols());
    const coord_t height = grid.resolution() * coord_t(grid.rows());
    const coord_t center_x = box.min(0) + width / 2;
    const coord_t center_y = box.min(1) + height / 2;

    const float left_boundary = grid.signed_distance_bilinear(Point(box.min(0), center_y));
    const float left_outside = grid.signed_distance_bilinear(Point(box.min(0) - 10, center_y));
    CHECK_THAT(left_outside - left_boundary, WithinAbs(10., 1e-5));

    const float right_boundary = grid.signed_distance_bilinear(Point(box.min(0) + width - 1, center_y));
    const float right_outside = grid.signed_distance_bilinear(Point(box.min(0) + width + 9, center_y));
    CHECK_THAT(right_outside - right_boundary, WithinAbs(10., 1e-5));

    const float bottom_boundary = grid.signed_distance_bilinear(Point(center_x, box.min(1)));
    const float bottom_outside = grid.signed_distance_bilinear(Point(center_x, box.min(1) - 10));
    CHECK_THAT(bottom_outside - bottom_boundary, WithinAbs(10., 1e-5));

    const float top_boundary = grid.signed_distance_bilinear(Point(center_x, box.min(1) + height - 1));
    const float top_outside = grid.signed_distance_bilinear(Point(center_x, box.min(1) + height + 9));
    CHECK_THAT(top_outside - top_boundary, WithinAbs(10., 1e-5));
}

TEST_CASE("EdgeGrid ignores empty and degenerate contours across create overloads", "[EdgeGrid]")
{
    const Polygon outer = rectangle(0, 0, 100, 100);
    const Polygon empty;

    EdgeGrid::Grid grid;
    const Polygons polygons{empty, outer};
    grid.create(polygons, 10);
    REQUIRE(grid.contours().size() == 1);
    CHECK(grid.contours().front().num_segments() == 4);

    const std::vector<const Polygon *> polygon_ptrs{&empty, &outer};
    grid.create(polygon_ptrs, 10);
    REQUIRE(grid.contours().size() == 1);
    CHECK(grid.contours().front().num_segments() == 4);

    const std::vector<Points> points{{}, outer.points, {Point(1, 1)}};
    grid.create(points, 10);
    REQUIRE(grid.contours().size() == 1);
    CHECK(grid.contours().front().num_segments() == 4);
}

TEST_CASE("EdgeGrid simplifies a concave contour into one nonempty region", "[EdgeGrid]")
{
    const Polygon l_shape(Points{Point(0, 0), Point(60, 0), Point(60, 20),
                                 Point(20, 20), Point(20, 60), Point(0, 60)});
    const Polygons contours{l_shape};
    EdgeGrid::Grid grid;
    grid.create(contours, 10);
    grid.calculate_sdf();

    const Polygons simplified = grid.contours_simplified(1, false);
    REQUIRE(simplified.size() == 1);
    REQUIRE(simplified.front().is_valid());
    CHECK(absolute_area(simplified) > 1000.);
    CHECK(simplified.front().contains(Point(10, 10)));
    CHECK(simplified.front().contains(Point(50, 10)));
    CHECK(simplified.front().contains(Point(10, 50)));
    CHECK_FALSE(simplified.front().contains(Point(50, 50)));
}
