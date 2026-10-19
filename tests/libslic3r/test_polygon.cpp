#include <algorithm>
#include <catch2/catch_all.hpp>
#include <cstddef>
#include <vector>
#include "libslic3r/libslic3r.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Geometry>
#include "libslic3r/BoundingBox.hpp"
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


namespace polygon_family_gap_contracts {

using Catch::Matchers::WithinAbs;

TEST_CASE("Polygon perimeter lengths and densification include the closing edge", "[Polygon][PolygonFamilyGap]")
{
    const Polygon source{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const std::vector<float> original_lengths = source.parameter_by_length();
    const std::vector<float> expected_original_lengths{0.f, 10.f, 20.f, 30.f, 40.f};
    REQUIRE(original_lengths.size() == expected_original_lengths.size());
    for (size_t i = 0; i < original_lengths.size(); ++i)
        CHECK_THAT(original_lengths[i], WithinAbs(expected_original_lengths[i], 1e-5));
    CHECK_THAT(source.length(), WithinAbs(40.0, 1e-9));

    const Points expected_points{{0, 0}, {6, 0}, {10, 0}, {10, 6},
                                 {10, 10}, {4, 10}, {0, 10}, {0, 4}};
    const std::vector<float> expected_dense_lengths{0.f, 6.f, 10.f, 16.f, 20.f, 26.f, 30.f, 36.f, 40.f};

    Polygon computed = source;
    computed.densify(6.f);
    CHECK(computed.points == expected_points);
    const std::vector<float> computed_lengths = computed.parameter_by_length();
    REQUIRE(computed_lengths.size() == expected_dense_lengths.size());
    for (size_t i = 0; i < computed_lengths.size(); ++i)
        CHECK_THAT(computed_lengths[i], WithinAbs(expected_dense_lengths[i], 1e-5));

    Polygon supplied = source;
    std::vector<float> supplied_lengths = original_lengths;
    supplied.densify(6.f, &supplied_lengths);
    CHECK(supplied.points == expected_points);
    REQUIRE(supplied_lengths.size() == expected_dense_lengths.size());
    for (size_t i = 0; i < supplied_lengths.size(); ++i)
        CHECK_THAT(supplied_lengths[i], WithinAbs(expected_dense_lengths[i], 1e-5));

    Polygon at_threshold = source;
    at_threshold.densify(10.f);
    CHECK(at_threshold.points == source.points);
}

TEST_CASE("Polygon affine transforms map each planar vertex", "[Polygon][PolygonFamilyGap]")
{
    const Polygon source{{0, 0}, {4, 0}, {4, 3}, {0, 3}};
    Transform3d affine = Transform3d::Identity();
    affine.linear()(0, 0) = 2.;
    affine.linear()(0, 1) = -1.;
    affine.linear()(1, 0) = 1.;
    affine.linear()(1, 1) = 1.;
    affine.translation() = Vec3d(20., -5., 3.);

    const Polygon transformed = source.transform(affine);
    const Points expected{{20, -5}, {28, -1}, {25, 2}, {17, -2}};
    CHECK(transformed.points == expected);
    CHECK((source.points == Points{{0, 0}, {4, 0}, {4, 3}, {0, 3}}));
    CHECK(Polygon{}.transform(affine).points.empty());
}

TEST_CASE("Polygon collection bounds combine rotated and per-region extents", "[Polygon][PolygonFamilyGap]")
{
    const Polygons regions{
        Polygon{{0, 0}, {10, 0}, {10, 4}, {0, 4}},
        Polygon{{20, -3}, {24, -3}, {24, 5}, {20, 5}}
    };

    const BoundingBox merged = get_extents(regions);
    CHECK(merged.min == Point(0, -3));
    CHECK(merged.max == Point(24, 5));

    const std::vector<BoundingBox> individual = get_extents_vector(regions);
    REQUIRE(individual.size() == 2);
    CHECK(individual[0].min == Point(0, 0));
    CHECK(individual[0].max == Point(10, 4));
    CHECK(individual[1].min == Point(20, -3));
    CHECK(individual[1].max == Point(24, 5));

    const BoundingBox rotated = get_extents_rotated(regions, 1.57079632679489661923);
    CHECK(rotated.defined);
    CHECK(rotated.min == Point(-5, 0));
    CHECK(rotated.max == Point(3, 24));
    CHECK_FALSE(get_extents_rotated(Polygons{}, 1.57079632679489661923).defined);
    CHECK(get_extents_vector(Polygons{}).empty());
}

TEST_CASE("Polygon collections detect a vertex shared by separate contours", "[Polygon][PolygonFamilyGap]")
{
    const Polygon left{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Polygon adjacent{{10, 0}, {20, 0}, {20, 10}, {10, 10}};
    const Polygon separated{{30, 0}, {40, 0}, {40, 10}, {30, 10}};

    CHECK(has_duplicate_points(Polygons{left, adjacent}));
    CHECK_FALSE(has_duplicate_points(Polygons{left, separated}));
    CHECK_FALSE(has_duplicate_points(Polygons{}));
}

TEST_CASE("Polygon collection cleanup removes adjacent and closing duplicates", "[Polygon][PolygonFamilyGap]")
{
    Polygon repeated{{0, 0}, {10, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}};
    const Points clean_points{{0, 0}, {10, 0}, {10, 10}, {0, 10}};

    CHECK(remove_same_neighbor(repeated));
    CHECK(repeated.points == clean_points);
    CHECK_FALSE(remove_same_neighbor(repeated));
    Polygon empty;
    CHECK_FALSE(remove_same_neighbor(empty));

    Polygons collection{Polygon{{0, 0}, {10, 0}, {10, 0}, {0, 10}},
                        Polygon{{1, 1}, {2, 2}},
                        Polygon{{20, 0}, {30, 0}, {30, 10}, {20, 10}}};
    CHECK(remove_same_neighbor(collection));
    REQUIRE(collection.size() == 2);
    CHECK((collection[0].points == Points{{0, 0}, {10, 0}, {0, 10}}));
    CHECK((collection[1].points == Points{{20, 0}, {30, 0}, {30, 10}, {20, 10}}));
    CHECK_FALSE(remove_same_neighbor(collection));
}

TEST_CASE("Polygon cleanup removes a backtracking stick without changing area", "[Polygon][PolygonFamilyGap]")
{
    Polygon stick{{0, 0}, {10, 0}, {5, 0}, {5, 10}, {0, 10}};
    CHECK(remove_sticks(stick));
    CHECK((stick.points == Points{{0, 0}, {5, 0}, {5, 10}, {0, 10}}));
    CHECK_THAT(stick.area(), WithinAbs(50.0, 1e-9));

    Polygon rectangle{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    CHECK_FALSE(remove_sticks(rectangle));
    CHECK((rectangle.points == Points{{0, 0}, {10, 0}, {10, 10}, {0, 10}}));

    Polygons collection{Polygon{{0, 0}, {10, 0}, {5, 0}, {5, 10}, {0, 10}},
                        Polygon{{1, 1}, {2, 2}}};
    CHECK(remove_sticks(collection));
    REQUIRE(collection.size() == 1);
    CHECK((collection.front().points == Points{{0, 0}, {5, 0}, {5, 10}, {0, 10}}));
}

TEST_CASE("Polygon collections remove degenerate and sub-threshold contours", "[Polygon][PolygonFamilyGap]")
{
    const Polygon degenerate{{0, 0}, {4, 0}};
    const Polygon below_threshold{{20, 0}, {24, 0}, {24, 4}, {20, 4}};
    const Polygon at_threshold{{40, 0}, {45, 0}, {45, 5}, {40, 5}};
    const Polygon above_threshold{{60, 0}, {70, 0}, {70, 10}, {60, 10}};
    Polygons regions{degenerate, below_threshold, at_threshold, above_threshold};

    CHECK(remove_degenerate(regions));
    REQUIRE(regions.size() == 3);
    CHECK(remove_small(regions, 25.0));
    REQUIRE(regions.size() == 2);
    CHECK(regions[0].points == at_threshold.points);
    CHECK(regions[1].points == above_threshold.points);
    CHECK_FALSE(remove_small(regions, 25.0));
}

TEST_CASE("Polygon collection collinear removal preserves each contour", "[Polygon][PolygonFamilyGap]")
{
    Polygons contours{
        Polygon{Point::new_scale(0, 0), Point::new_scale(5, 0), Point::new_scale(10, 0),
                Point::new_scale(10, 10), Point::new_scale(0, 10), Point::new_scale(0, 5)},
        Polygon{Point::new_scale(20, 0), Point::new_scale(30, 0), Point::new_scale(30, 5),
                Point::new_scale(30, 10), Point::new_scale(20, 10)}
    };

    remove_collinear(contours);
    REQUIRE(contours.size() == 2);
    CHECK((contours[0].points == Points{Point::new_scale(0, 0), Point::new_scale(10, 0),
                                       Point::new_scale(10, 10), Point::new_scale(0, 10)}));
    CHECK((contours[1].points == Points{Point::new_scale(20, 0), Point::new_scale(30, 0),
                                       Point::new_scale(30, 10), Point::new_scale(20, 10)}));
    const double expected_scaled_area = 100.0 / (SCALING_FACTOR * SCALING_FACTOR);
    CHECK_THAT(contours[0].area(), WithinAbs(expected_scaled_area, 0.5));
    CHECK_THAT(contours[1].area(), WithinAbs(expected_scaled_area, 0.5));
}

TEST_CASE("Polygon matching accepts cyclic starts and rejects changed vertices", "[Polygon][PolygonFamilyGap]")
{
    const Polygon original{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    const Polygon cyclic_start{{10, 10}, {0, 10}, {0, 0}, {10, 0}};
    const Polygon changed{{10, 10}, {0, 10}, {-1, 0}, {10, 0}};

    CHECK(polygons_match(original, cyclic_start));
    CHECK_FALSE(polygons_match(original, changed));
}

TEST_CASE("Polygon collection overlap finds a later intersecting region", "[Polygon][PolygonFamilyGap]")
{
    const Polygons candidates{
        Polygon{{20, 20}, {30, 20}, {30, 30}, {20, 30}},
        Polygon{{40, 40}, {50, 40}, {50, 50}, {40, 50}},
        Polygon{{5, 5}, {15, 5}, {15, 15}, {5, 15}}
    };
    const Polygons target{Polygon{{0, 0}, {10, 0}, {10, 10}, {0, 10}}};

    CHECK(overlaps(candidates, target));
    CHECK_FALSE(overlaps(Polygons{candidates.front()}, target));
    CHECK_FALSE(overlaps(Polygons{}, target));
}

} // namespace polygon_family_gap_contracts
