#include <algorithm>
#include <cstddef>

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
