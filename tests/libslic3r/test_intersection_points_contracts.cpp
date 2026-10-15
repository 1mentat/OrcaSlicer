#include <catch2/catch_all.hpp>

#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/IntersectionPoints.hpp"

using namespace Slic3r;

TEST_CASE("IntersectionPoints reports a genuine non-adjacent crossing", "[IntersectionPoints]")
{
    // The detector intentionally ignores neighboring segments because polygon
    // vertices are shared by design. Keep the crossing at indices 0 and 2 so
    // this also exercises the indexed AABB query rather than endpoint filtering.
    const Lines lines {
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 200, 200 }, Point { 300, 200 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
        Line { Point { 300, 300 }, Point { 400, 300 } },
    };

    const IntersectionsLines intersections = get_intersections(lines);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 0);
    CHECK(intersections.front().line_index2 == 2);
    CHECK(intersections.front().intersection.isApprox(Vec2d { 50., 50. }));
}

TEST_CASE("IntersectionPoints ignores polygon vertices shared by neighboring edges", "[IntersectionPoints]")
{
    const Polygon square {
        { 0, 0 }, { 100, 0 }, { 100, 100 }, { 0, 100 }
    };

    CHECK(get_intersections(square).empty());
}

TEST_CASE("IntersectionPoints checks every contour in an ExPolygon", "[IntersectionPoints]")
{
    const ExPolygon square_with_hole {
        Polygon { { 0, 0 }, { 200, 0 }, { 200, 200 }, { 0, 200 } },
        Polygon { { 50, 50 }, { 50, 150 }, { 150, 150 }, { 150, 50 } }
    };

    REQUIRE(square_with_hole.is_valid());
    CHECK(get_intersections(square_with_hole).empty());
}
