#include <algorithm>
#include <cstddef>
#include <vector>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/libslic3r.h"

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <libslic3r/Polyline.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_utils.hpp"

using namespace Slic3r;

namespace {

ExPolygon rectangle_with_two_holes()
{
    // The contour is CCW and both holes are CW, as required by ExPolygon.
    ExPolygon result(
        Polygon { { 0, 0 }, { 1000, 0 }, { 1000, 1000 }, { 0, 1000 } },
        Polygon { { 150, 350 }, { 150, 650 }, { 450, 650 }, { 450, 350 } });
    result.holes.emplace_back(Polygon { { 550, 350 }, { 550, 650 }, { 850, 650 }, { 850, 350 } });
    return result;
}
} // namespace

TEST_CASE("ExPolygon area and containment honor its holes", "[ExPolygon]")
{
    ExPolygon expoly = rectangle_with_two_holes();

    REQUIRE(expoly.is_valid());
    REQUIRE(expoly.num_contours() == 3);
    REQUIRE_THAT(expoly.area(), Catch::Matchers::WithinAbs(820000.0, 1e-6));
    REQUIRE(expoly.contains(Point { 50, 50 }));
    REQUIRE_FALSE(expoly.contains(Point { 300, 500 }));
    REQUIRE_FALSE(expoly.contains(Point { 700, 500 }));

    const Polylines loops = to_polylines(expoly);
    REQUIRE(loops.size() == expoly.num_contours());
    for (size_t i = 0; i < loops.size(); ++i) {
        REQUIRE(loops[i].points.size() == expoly.contour_or_hole(i).points.size() + 1);
        CHECK(loops[i].points.front() == loops[i].points.back());
    }
    CHECK(to_lines(expoly).size() == count_points(expoly));
}

TEST_CASE("ExPolygon transforms preserve winding and scale area", "[ExPolygon]")
{
    ExPolygon expoly = rectangle_with_two_holes();
    expoly.translate(Point { 2000, -500 });
    expoly.scale(2.0, 3.0);

    REQUIRE(expoly.is_valid());
    REQUIRE_THAT(expoly.area(), Catch::Matchers::WithinAbs(4920000.0, 1e-6));
    CHECK(expoly.contour.is_counter_clockwise());
    CHECK(std::all_of(expoly.holes.begin(), expoly.holes.end(),
                      [](const Polygon &hole) { return !hole.is_counter_clockwise(); }));
}

TEST_CASE("Mesh slicing returns separate hollow islands as ExPolygons", "[ExPolygon][TriangleMeshSlicer]")
{
    const TriangleMesh hollow_cube = load_model("cube_with_hole.obj");
    const auto hollow_slices = hollow_cube.slice({ 5.0 });
    REQUIRE(hollow_slices.size() == 1);
    REQUIRE(hollow_slices.front().size() == 1);
    REQUIRE(hollow_slices.front().front().is_valid());
    REQUIRE(hollow_slices.front().front().holes.size() == 1);
    CHECK(hollow_slices.front().front().contains(Point::new_scale(2.0, 2.0)));
    CHECK_FALSE(hollow_slices.front().front().contains(Point::new_scale(10.0, 10.0)));

    const TriangleMesh two_hollow_squares = load_model("two_hollow_squares.obj");
    const auto two_slices = two_hollow_squares.slice({ 1.4 });
    REQUIRE(two_slices.size() == 1);
    REQUIRE(two_slices.front().size() == 2);
    for (const ExPolygon &slice : two_slices.front()) {
        CHECK(slice.is_valid());
        CHECK(slice.holes.size() == 1);
    }
}


namespace {

ExPolygon rectangle(coord_t x, coord_t y, coord_t width, coord_t height)
{
    return ExPolygon(Points { Point { x, y }, Point { x + width, y },
                              Point { x + width, y + height }, Point { x, y + height } });
}

ExPolygon rectangle_with_hole()
{
    ExPolygon result = rectangle(0, 0, 100, 100);
    result.holes.emplace_back(Polygon { { 35, 35 }, { 35, 65 }, { 65, 65 }, { 65, 35 } });
    return result;
}

} // namespace

TEST_CASE("ExPolygon requires every polyline to stay inside the filled area", "[ExPolygon][ExPolygonGap]")
{
    const ExPolygon expoly = rectangle_with_hole();
    const Polylines inside { Polyline { { 10, 10 }, { 25, 10 } },
                             Polyline { { 75, 75 }, { 90, 75 } } };
    CHECK(expoly.contains(inside));

    const Polylines through_hole { Polyline { { 20, 50 }, { 80, 50 } } };
    CHECK_FALSE(expoly.contains(through_hole));

    const Polylines crossing_outer_boundary { Polyline { { -10, 10 }, { 10, 10 } } };
    CHECK_FALSE(expoly.contains(crossing_outer_boundary));
}

TEST_CASE("ExPolygon matching accepts cyclic ring starts but checks hole topology", "[ExPolygon][ExPolygonGap]")
{
    const ExPolygon original(
        Polygon { { 0, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 } },
        Polygon { { 20, 20 }, { 20, 40 }, { 40, 40 }, { 40, 20 } });
    const ExPolygon cyclically_rotated(
        Polygon { { 100, 0 }, { 100, 100 }, { 0, 100 }, { 0, 0 } },
        Polygon { { 40, 40 }, { 40, 20 }, { 20, 20 }, { 20, 40 } });
    REQUIRE(expolygons_match(original, cyclically_rotated));

    const ExPolygon without_hole(original.contour);
    CHECK_FALSE(expolygons_match(original, without_hole));

    const ExPolygon changed_hole(
        original.contour, Polygon { { 60, 20 }, { 60, 40 }, { 80, 40 }, { 80, 20 } });
    CHECK_FALSE(expolygons_match(original, changed_hole));

    const ExPolygon changed_outer(
        Polygon { { -10, 0 }, { 100, 0 }, { 100, 100 }, { -10, 100 } },
        original.holes.front());
    CHECK_FALSE(expolygons_match(original, changed_outer));
}

TEST_CASE("ExPolygon collection overlap checks later candidates", "[ExPolygon][ExPolygonGap]")
{
    const ExPolygon query = rectangle(0, 0, 20, 20);
    const ExPolygons empty;
    CHECK_FALSE(overlaps(empty, query));

    const ExPolygons disjoint { rectangle(40, 40, 10, 10) };
    CHECK_FALSE(overlaps(disjoint, query));

    const ExPolygons hit_after_miss { rectangle(40, 40, 10, 10), rectangle(15, 15, 10, 10) };
    CHECK(overlaps(hit_after_miss, query));
}

TEST_CASE("ExPolygon duplicate checks include contour holes and separate components", "[ExPolygon][ExPolygonGap]")
{
    const ExPolygon unique_ring_points = rectangle_with_hole();
    CHECK_FALSE(has_duplicate_points(unique_ring_points));

    ExPolygon repeated_across_rings = rectangle_with_hole();
    repeated_across_rings.holes.front().points.front() = repeated_across_rings.contour.points.front();
    CHECK(has_duplicate_points(repeated_across_rings));

    const ExPolygons separated { rectangle(0, 0, 10, 10), rectangle(30, 0, 10, 10) };
    CHECK_FALSE(has_duplicate_points(separated));

    const ExPolygons distinct_holes { rectangle_with_hole(), rectangle(-20, -20, 140, 140) };
    CHECK_FALSE(has_duplicate_points(distinct_holes));

    ExPolygon first_hole_owner = rectangle(0, 0, 100, 100);
    first_hole_owner.holes.emplace_back(Polygon { { 35, 35 }, { 35, 65 }, { 65, 65 }, { 65, 35 } });
    ExPolygon second_hole_owner = rectangle(-20, -20, 140, 140);
    second_hole_owner.holes.emplace_back(first_hole_owner.holes.front());
    const ExPolygons repeated_hole_points { first_hole_owner, second_hole_owner };
    CHECK(has_duplicate_points(repeated_hole_points));

    const ExPolygons shared_boundary { rectangle(0, 0, 10, 10), rectangle(10, 0, 10, 10) };
    CHECK(has_duplicate_points(shared_boundary));
}

TEST_CASE("ExPolygon collection bounds preserve each region and rotation", "[ExPolygon][ExPolygonGap]")
{
    const ExPolygon first = rectangle(0, 0, 40, 20);
    const BoundingBox rotated = get_extents_rotated(first, 1.57079632679489661923);
    CHECK(rotated.min.x() == -20);
    CHECK(rotated.max.x() == 0);
    CHECK(rotated.min.y() == 0);
    CHECK(rotated.max.y() == 40);

    const std::vector<BoundingBox> boxes = get_extents_vector(
        ExPolygons { first, rectangle(100, -10, 15, 30) });
    REQUIRE(boxes.size() == 2);
    CHECK(boxes[0].min == Point(0, 0));
    CHECK(boxes[0].max == Point(40, 20));
    CHECK(boxes[1].min == Point(100, -10));
    CHECK(boxes[1].max == Point(115, 20));
}


TEST_CASE("ExPolygon collection keeps the region with the largest outer contour", "[ExPolygon][ExPolygonGap]")
{
    ExPolygons empty;
    keep_largest_contour_only(empty);
    CHECK(empty.empty());

    const ExPolygon singleton_before = rectangle_with_hole();
    ExPolygons singleton { singleton_before };
    keep_largest_contour_only(singleton);
    REQUIRE(singleton.size() == 1);
    CHECK(singleton.front().contour.points == singleton_before.contour.points);
    REQUIRE(singleton.front().holes.size() == 1);
    CHECK(singleton.front().holes.front().points == singleton_before.holes.front().points);

    const ExPolygon small_first = rectangle(-200, 0, 30, 30);
    ExPolygon largest_outer = rectangle(0, 0, 100, 100);
    largest_outer.holes.emplace_back(Polygon { { 10, 10 }, { 10, 90 }, { 90, 90 }, { 90, 10 } });
    const ExPolygon medium_solid = rectangle(200, 0, 90, 90);
    REQUIRE(largest_outer.is_valid());
    REQUIRE(medium_solid.is_valid());
    REQUIRE_THAT(largest_outer.area(), Catch::Matchers::WithinAbs(3600.0, 1e-6));
    REQUIRE_THAT(medium_solid.area(), Catch::Matchers::WithinAbs(8100.0, 1e-6));

    const Points expected_outer = largest_outer.contour.points;
    const Points expected_hole = largest_outer.holes.front().points;
    ExPolygons candidates { small_first, largest_outer, medium_solid };
    keep_largest_contour_only(candidates);

    REQUIRE(candidates.size() == 1);
    CHECK(candidates.front().contour.points == expected_outer);
    REQUIRE(candidates.front().holes.size() == 1);
    CHECK(candidates.front().holes.front().points == expected_hole);
    CHECK(candidates.front().is_valid());
    CHECK_THAT(candidates.front().area(), Catch::Matchers::WithinAbs(3600.0, 1e-6));
}
