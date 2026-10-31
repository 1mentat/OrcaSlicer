#include <catch2/catch_test_macros.hpp>

#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/IntersectionPoints.hpp>
#include <libslic3r/Line.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/Polygon.hpp>

using namespace Slic3r;

TEST_CASE("IntersectionPoints reports one genuine crossing among independent lines", "[IntersectionPoints]")
{
    // Keep the crossing at indices 0 and 2 so the reverse traversal exercises the
    // lower-index filter, while the middle line remains disjoint.
    const Lines lines {
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 200, 200 }, Point { 300, 200 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
    };

    const IntersectionsLines intersections = get_intersections(lines);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 0);
    CHECK(intersections.front().line_index2 == 2);
    CHECK(intersections.front().intersection.isApprox(Vec2d { 50., 50. }));
}

TEST_CASE("IntersectionPoints remains unique when the lines are reordered", "[IntersectionPoints]")
{
    const Lines reordered {
        Line { Point { 200, 200 }, Point { 300, 200 } },
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
    };

    const IntersectionsLines intersections = get_intersections(reordered);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 1);
    CHECK(intersections.front().line_index2 == 2);
    CHECK(intersections.front().intersection.isApprox(Vec2d { 50., 50. }));
}

TEST_CASE("IntersectionPoints finds a crossing in a two-line input", "[IntersectionPoints]")
{
    const Lines lines {
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
    };

    const IntersectionsLines intersections = get_intersections(lines);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 0);
    CHECK(intersections.front().line_index2 == 1);
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

// Known bug: compute_intersections' neighbour filter computes node_index - 1 in size_t, which wraps at 0; fixed by the follow-up fix PR
TEST_CASE("IntersectionPoints reports a crossing with line 0 in ascending index order", "[IntersectionPoints][!shouldfail]")
{
    const Lines lines {
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
        Line { Point { 200, 200 }, Point { 300, 200 } },
        Line { Point { 400, 400 }, Point { 500, 400 } },
    };

    const IntersectionsLines intersections = get_intersections(lines);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 0);
    CHECK(intersections.front().line_index2 == 1);
    CHECK(intersections.front().intersection.isApprox(Vec2d { 50., 50. }));
}

// Known bug: compute_intersections' neighbour filter computes node_index - 1 in size_t, which wraps at 0; fixed by the follow-up fix PR
TEST_CASE("IntersectionPoints reports a crossing with line 0 only once", "[IntersectionPoints][!shouldfail]")
{
    const Lines lines {
        Line { Point { 0, 0 }, Point { 100, 100 } },
        Line { Point { 200, 200 }, Point { 300, 200 } },
        Line { Point { 400, 400 }, Point { 500, 400 } },
        Line { Point { 0, 100 }, Point { 100, 0 } },
        Line { Point { 600, 600 }, Point { 700, 600 } },
    };

    const IntersectionsLines intersections = get_intersections(lines);

    REQUIRE(intersections.size() == 1);
    CHECK(intersections.front().line_index1 == 0);
    CHECK(intersections.front().line_index2 == 3);
    CHECK(intersections.front().intersection.isApprox(Vec2d { 50., 50. }));
}
