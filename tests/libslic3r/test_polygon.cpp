#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cstddef>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Polyline.hpp"

using namespace Slic3r;

SCENARIO("Converted Perl tests", "[Polygon]") {
    GIVEN("ccw_square") {
        Polygon ccw_square{ { 100, 100 }, { 200, 100 }, { 200, 200 }, { 100, 200 } };
        Polygon cw_square(ccw_square);
        cw_square.reverse();

        THEN("ccw_square is valid") {
            REQUIRE(ccw_square.is_valid());
        }
        THEN("cw_square is valid") {
            REQUIRE(cw_square.is_valid());
        }
        THEN("ccw_square.area") {
            REQUIRE(ccw_square.area() == 100 * 100);
        }
        THEN("cw_square.area") {
            REQUIRE(cw_square.area() == - 100 * 100);
        }
        THEN("ccw_square.centroid") {
            REQUIRE(ccw_square.centroid() == Point { 150, 150 });
        }
        THEN("cw_square.centroid") {
            REQUIRE(cw_square.centroid() == Point { 150, 150 });
        }
        THEN("ccw_square.contains_point(150, 150)") {
            REQUIRE(ccw_square.contains({ 150, 150 }));
        }
        THEN("cw_square.contains_point(150, 150)") {
            REQUIRE(cw_square.contains({ 150, 150 }));
        }
        THEN("conversion to lines") {
            REQUIRE(ccw_square.lines() == Lines{
                { { 100, 100 }, { 200, 100 } },
                { { 200, 100 }, { 200, 200 } },
                { { 200, 200 }, { 100, 200 } },
                { { 100, 200 }, { 100, 100 } } });
        }
        THEN("split_at_first_point") {
            REQUIRE(ccw_square.split_at_first_point() == Polyline { ccw_square[0], ccw_square[1], ccw_square[2], ccw_square[3], ccw_square[0] });
        }
        THEN("split_at_index(2)") {
            REQUIRE(ccw_square.split_at_index(2) == Polyline { ccw_square[2], ccw_square[3], ccw_square[0], ccw_square[1], ccw_square[2] });
        }
        THEN("split_at_vertex(ccw_square[2])") {
            REQUIRE(ccw_square.split_at_vertex(ccw_square[2]) == Polyline { ccw_square[2], ccw_square[3], ccw_square[0], ccw_square[1], ccw_square[2] });
        }
        THEN("is_counter_clockwise") {
            REQUIRE(ccw_square.is_counter_clockwise());
        }
        THEN("! is_counter_clockwise") {
            REQUIRE(! cw_square.is_counter_clockwise());
        }
        THEN("make_counter_clockwise") {
            cw_square.make_counter_clockwise();
            REQUIRE(cw_square.is_counter_clockwise());
        }
        THEN("make_counter_clockwise^2") {
            cw_square.make_counter_clockwise();
            cw_square.make_counter_clockwise();
            REQUIRE(cw_square.is_counter_clockwise());
        }
        THEN("first_point") {
            REQUIRE(&ccw_square.first_point() == &ccw_square.points.front());
        }
    }
    GIVEN("Triangulating hexagon") {
        Polygon hexagon{ { 100, 0 } };
        for (size_t i = 1; i < 6; ++ i) {
            Point p = hexagon.points.front();
            p.rotate(PI / 3 * i);
            hexagon.points.emplace_back(p);
        }
        Polygons triangles;
        hexagon.triangulate_convex(&triangles);
        THEN("right number of triangles") {
            REQUIRE(triangles.size() == 4);
        }
        THEN("all triangles are ccw") {
            auto it = std::find_if(triangles.begin(), triangles.end(), [](const Polygon &tri) { return tri.is_clockwise(); });
            REQUIRE(it == triangles.end());
        }
    }
    GIVEN("General triangle") {
        Polygon polygon { { 50000000, 100000000 }, { 300000000, 102000000 }, { 50000000, 104000000 } };
        Line    line { { 175992032, 102000000 }, { 47983964, 102000000 } };
        Point   intersection;
        bool    has_intersection = polygon.intersection(line, &intersection);
        THEN("Intersection with line") {
            REQUIRE(has_intersection);
            REQUIRE(intersection == Point { 50000000, 102000000 });
        }
    }
}

TEST_CASE("Centroid of Trapezoid must be inside", "[Polygon][Utils]")
{
    Slic3r::Polygon trapezoid {
        { 4702134, 1124765853 },
        { -4702134, 1124765853 },
        { -9404268, 1049531706 },
        { 9404268, 1049531706 },
    };
    Point centroid = trapezoid.centroid();
    CHECK(trapezoid.contains(centroid));
}

// This test currently only covers remove_collinear_points.
// All remaining tests are to be ported from xs/t/06_polygon.t

Slic3r::Points collinear_circle({
    Slic3r::Point::new_scale(0, 0), // 3 collinear points at beginning
    Slic3r::Point::new_scale(10, 0),
    Slic3r::Point::new_scale(20, 0),
    Slic3r::Point::new_scale(30, 10),
    Slic3r::Point::new_scale(40, 20), // 2 collinear points
    Slic3r::Point::new_scale(40, 30),
    Slic3r::Point::new_scale(30, 40), // 3 collinear points
    Slic3r::Point::new_scale(20, 40),
    Slic3r::Point::new_scale(10, 40),
    Slic3r::Point::new_scale(-10, 20),
    Slic3r::Point::new_scale(-20, 10),
    Slic3r::Point::new_scale(-20, 0), // 3 collinear points at end
    Slic3r::Point::new_scale(-10, 0),
    Slic3r::Point::new_scale(-5, 0)
});

SCENARIO("Remove collinear points from Polygon", "[Polygon]") {
    GIVEN("Polygon with collinear points"){
        Slic3r::Polygon p(collinear_circle);
        WHEN("collinear points are removed") {
            remove_collinear(p);
            THEN("Leading collinear points are removed") {
                REQUIRE(p.points.front() == Slic3r::Point::new_scale(20, 0));
            }
            THEN("Trailing collinear points are removed") {
                REQUIRE(p.points.back() == Slic3r::Point::new_scale(-20, 0));
            }
            THEN("Number of remaining points is correct") {
                REQUIRE(p.points.size() == 7);
            }
        }
    }
}

namespace math_gap_contracts {

Polygon concave_notch()
{
    return Polygon{{0, 0}, {100, 0}, {100, 20}, {20, 20},
                   {20, 80}, {100, 80}, {100, 100}, {0, 100}};
}

TEST_CASE("Polygon first intersection returns the closest crossing from the line start", "[Polygon][PolygonGap][MathGap]")
{
    const Polygon polygon = concave_notch();
    Point from_right;
    REQUIRE(polygon.first_intersection(Line(Point(50, 50), Point(-10, 50)), &from_right));
    CHECK(from_right == Point(20, 50));

    Point from_left;
    REQUIRE(polygon.first_intersection(Line(Point(-10, 50), Point(50, 50)), &from_left));
    CHECK(from_left == Point(0, 50));
}

TEST_CASE("Polygon first intersection preserves output on a miss", "[Polygon][PolygonGap][MathGap]")
{
    const Polygon polygon = concave_notch();
    const Point sentinel(7, 9);
    Point intersection = sentinel;

    CHECK_FALSE(polygon.first_intersection(Line(Point(30, 50), Point(90, 50)), &intersection));
    CHECK(intersection == sentinel);
}

TEST_CASE("Polygon intersections append every crossing", "[Polygon][PolygonGap][MathGap]")
{
    const Polygon polygon = concave_notch();
    const Line line(Point(50, 50), Point(-10, 50));
    const Point sentinel(-7, -9);
    Points intersections{sentinel};

    REQUIRE(polygon.intersections(line, &intersections));
    REQUIRE(intersections.size() == 3);
    CHECK(intersections.front() == sentinel);
    CHECK(std::find(intersections.begin(), intersections.end(), Point(0, 50)) != intersections.end());
    CHECK(std::find(intersections.begin(), intersections.end(), Point(20, 50)) != intersections.end());
}

TEST_CASE("Polygon intersection queries handle short inputs and misses", "[Polygon][PolygonGap][MathGap]")
{
    const Polygon polygon = concave_notch();
    const Line miss(Point(30, 50), Point(90, 50));
    const Point sentinel(7, 9);
    Point first = sentinel;
    Points all{sentinel};

    CHECK_FALSE(polygon.first_intersection(miss, &first));
    CHECK(first == sentinel);
    CHECK_FALSE(polygon.intersections(miss, &all));
    CHECK(all == Points{sentinel});

    const Polygon empty;
    const Polygon one_point{{0, 0}};
    CHECK_FALSE(empty.first_intersection(miss, &first));
    CHECK_FALSE(one_point.first_intersection(miss, &first));
    CHECK_FALSE(empty.intersections(miss, &all));
    CHECK_FALSE(one_point.intersections(miss, &all));
    CHECK(first == sentinel);
    CHECK(all == Points{sentinel});
}

TEST_CASE("Polygon overlap includes containment without boundary crossings", "[Polygon][PolygonGap][MathGap]")
{
    const Polygon outer{{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    const Polygon inner{{20, 20}, {30, 20}, {30, 30}, {20, 30}};
    const Polygon disjoint{{120, 120}, {130, 120}, {130, 130}, {120, 130}};
    const Polygon empty;

    CHECK(inner.overlaps(Polygons{outer}));
    CHECK_FALSE(outer.overlaps(Polygons{disjoint}));
    CHECK_FALSE(empty.overlaps(Polygons{inner}));
}

} // namespace math_gap_contracts
