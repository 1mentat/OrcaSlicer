#include <algorithm>
#include <cmath>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Geometry>
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Geometry/Circle.hpp"
#include "libslic3r/Geometry/ConvexHull.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ShortestPath.hpp"

//#include <random>
//#include "libnest2d/tools/benchmark.h"
#include "libslic3r/SVG.hpp"

#include "../libnest2d/printer_parts.hpp"

#include "libslic3r/libslic3r.h"
#include <cstddef>
#include <cstdlib>
#include <math.h>
#include <functional>
#include <string>
#include <unordered_set>
#include <vector>
#include "libnest2d/backends/libslic3r/geometries.hpp"

using namespace Slic3r;

TEST_CASE("Line::parallel_to", "[Geometry]"){
    Line l{ { 100000, 0 }, { 0, 0 } };
    Line l2{ { 200000, 0 }, { 0, 0 } };
    REQUIRE(l.parallel_to(l));
    REQUIRE(l.parallel_to(l2));

    Line l3(l2);
    l3.rotate(0.9 * EPSILON, { 0, 0 });
    REQUIRE(l.parallel_to(l3));

    Line l4(l2);
    l4.rotate(1.1 * EPSILON, { 0, 0 });
    REQUIRE_FALSE(l.parallel_to(l4));

    // The angle epsilon is so low that vectors shorter than 100um rotated by epsilon radians are not rotated at all.
    Line l5{ { 20000, 0 }, { 0, 0 } };
    l5.rotate(1.1 * EPSILON, { 0, 0 });
    REQUIRE(l.parallel_to(l5));

    l.rotate(1., { 0, 0 });
    Point offset{ 342876, 97636249 };
    l.translate(offset);
    l3.rotate(1., { 0, 0 });
    l3.translate(offset);
    l4.rotate(1., { 0, 0 });
    l4.translate(offset);
    REQUIRE(l.parallel_to(l3));
    REQUIRE_FALSE(l.parallel_to(l4));
}

TEST_CASE("Line::perpendicular_to", "[Geometry]") {
    Line l{ { 100000, 0 }, { 0, 0 } };
    Line l2{ { 0, 200000 }, { 0, 0 } };
    REQUIRE_FALSE(l.perpendicular_to(l));
    REQUIRE(l.perpendicular_to(l2));

    Line l3(l2);
    l3.rotate(0.9 * EPSILON, { 0, 0 });
    REQUIRE(l.perpendicular_to(l3));

    Line l4(l2);
    l4.rotate(1.1 * EPSILON, { 0, 0 });
    REQUIRE_FALSE(l.perpendicular_to(l4));

    // The angle epsilon is so low that vectors shorter than 100um rotated by epsilon radians are not rotated at all.
    Line l5{ { 0, 20000 }, { 0, 0 } };
    l5.rotate(1.1 * EPSILON, { 0, 0 });
    REQUIRE(l.perpendicular_to(l5));

    l.rotate(1., { 0, 0 });
    Point offset{ 342876, 97636249 };
    l.translate(offset);
    l3.rotate(1., { 0, 0 });
    l3.translate(offset);
    l4.rotate(1., { 0, 0 });
    l4.translate(offset);
    REQUIRE(l.perpendicular_to(l3));
    REQUIRE_FALSE(l.perpendicular_to(l4));
}

TEST_CASE("Polygon::contains works properly", "[Geometry]"){
   // this test was failing on Windows (GH #1950)
    Slic3r::Polygon polygon(Points({
        Point(207802834,-57084522),
        Point(196528149,-37556190),
        Point(173626821,-25420928),
        Point(171285751,-21366123),
        Point(118673592,-21366123),
        Point(116332562,-25420928),
        Point(93431208,-37556191),
        Point(82156517,-57084523),
        Point(129714478,-84542120),
        Point(160244873,-84542120)
    }));
    Point point(95706562, -57294774);
    REQUIRE(polygon.contains(point));
}

SCENARIO("Intersections of line segments", "[Geometry]"){
    GIVEN("Integer coordinates"){
        Line line1(Point(5,15),Point(30,15));
        Line line2(Point(10,20), Point(10,10));
        THEN("The intersection is valid"){
            Point point;
            line1.intersection(line2,&point);
            REQUIRE(Point(10,15) == point);
        }
    }

    GIVEN("Scaled coordinates"){
        Line line1(Point(73.6310778185108 / 0.00001, 371.74239268924 / 0.00001), Point(73.6310778185108 / 0.00001, 501.74239268924 / 0.00001));
        Line line2(Point(75/0.00001, 437.9853/0.00001), Point(62.7484/0.00001, 440.4223/0.00001));
        THEN("There is still an intersection"){
            Point point;
            REQUIRE(line1.intersection(line2,&point));
        }
    }
}

SCENARIO("polygon_is_convex works") {
    GIVEN("A square of dimension 10") {
        WHEN("Polygon is convex clockwise") {
            Polygon cw_square  { { {0, 0}, {0,10}, {10,10}, {10,0} } };
            THEN("it is not convex") {
                REQUIRE_FALSE(polygon_is_convex(cw_square));
            }
        }
        WHEN("Polygon is convex counter-clockwise") {
            Polygon ccw_square { { {0, 0}, {10,0}, {10,10}, {0,10} } };
            THEN("it is convex") {
                REQUIRE(polygon_is_convex(ccw_square));
            }
        } 
    }
    GIVEN("A concave polygon") {
        Polygon concave = { {0,0}, {10,0}, {10,10}, {0,10}, {0,6}, {4,6}, {4,4}, {0,4} };
        THEN("It is not convex") {
            REQUIRE_FALSE(polygon_is_convex(concave));
        }
    }
}

TEST_CASE("Creating a polyline generates the obvious lines", "[Geometry]"){
    Slic3r::Polyline polyline;
    polyline.points = Points({Point(0, 0), Point(10, 0), Point(20, 0)});
    REQUIRE(polyline.lines().at(0).a == Point(0,0));
    REQUIRE(polyline.lines().at(0).b == Point(10,0));
    REQUIRE(polyline.lines().at(1).a == Point(10,0));
    REQUIRE(polyline.lines().at(1).b == Point(20,0));
}

TEST_CASE("Splitting a Polygon generates a polyline correctly", "[Geometry]"){
    Slic3r::Polygon polygon(Points({Point(0, 0), Point(10, 0), Point(5, 5)}));
    Slic3r::Polyline split = polygon.split_at_index(1);
    REQUIRE(split.points[0]==Point(10,0));
    REQUIRE(split.points[1]==Point(5,5));
    REQUIRE(split.points[2]==Point(0,0));
    REQUIRE(split.points[3]==Point(10,0));
}


TEST_CASE("Bounding boxes are scaled appropriately", "[Geometry]"){
    BoundingBox bb(Points({Point(0, 1), Point(10, 2), Point(20, 2)}));
    bb.scale(2);
    REQUIRE(bb.min == Point(0,2));
    REQUIRE(bb.max == Point(40,4));
}


TEST_CASE("Offsetting a line generates a polygon correctly", "[Geometry]"){
	Slic3r::Polyline tmp = { Point(10,10), Point(20,10) };
    Slic3r::Polygon area = offset(tmp,5).at(0);
    REQUIRE(area.area() == Slic3r::Polygon(Points({Point(10,5),Point(20,5),Point(20,15),Point(10,15)})).area());
}

SCENARIO("Circle Fit, TaubinFit with Newton's method", "[Geometry]") {
    GIVEN("A vector of Vec2ds arranged in a half-circle with approximately the same distance R from some point") {
        Vec2d expected_center(-6, 0);
        Vec2ds sample {Vec2d(6.0, 0), Vec2d(5.1961524, 3), Vec2d(3 ,5.1961524), Vec2d(0, 6.0), Vec2d(3, 5.1961524), Vec2d(-5.1961524, 3), Vec2d(-6.0, 0)};
        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Vec2d& a) { return a + expected_center;});

        WHEN("Circle fit is called on the entire array") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of -6,0 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
    }
    GIVEN("A vector of Vec2ds arranged in a half-circle with approximately the same distance R from some point") {
        Vec2d expected_center(-3, 9);
        Vec2ds sample {Vec2d(6.0, 0), Vec2d(5.1961524, 3), Vec2d(3 ,5.1961524), 
                        Vec2d(0, 6.0), 
                        Vec2d(3, 5.1961524), Vec2d(-5.1961524, 3), Vec2d(-6.0, 0)};

        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Vec2d& a) { return a + expected_center;});


        WHEN("Circle fit is called on the entire array") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Vec2d result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
    }
    GIVEN("A vector of Points arranged in a half-circle with approximately the same distance R from some point") {
        Point expected_center { Point::new_scale(-3, 9)};
        Points sample {Point::new_scale(6.0, 0), Point::new_scale(5.1961524, 3), Point::new_scale(3 ,5.1961524), 
                        Point::new_scale(0, 6.0), 
                        Point::new_scale(3, 5.1961524), Point::new_scale(-5.1961524, 3), Point::new_scale(-6.0, 0)};

        std::transform(sample.begin(), sample.end(), sample.begin(), [expected_center] (const Point& a) { return a + expected_center;});


        WHEN("Circle fit is called on the entire array") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the first four points") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin(), sample.cbegin()+4);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
        WHEN("Circle fit is called on the middle four points") {
            Point result_center(0,0);
            result_center = Geometry::circle_center_taubin_newton(sample.cbegin()+2, sample.cbegin()+6);
            THEN("A center point of scaled 3,9 is returned.") {
                REQUIRE(is_approx(result_center, expected_center));
            }
        }
    }
}

TEST_CASE("Circle fit APIs reject underdetermined samples and recover an exact circle", "[Geometry][CircleFit]")
{
    const Vec2ds underdetermined{Vec2d(9., -3.), Vec2d(4., 2.)};
    CHECK(Geometry::circle_taubin_newton(underdetermined).invalid());
    CHECK(Geometry::circle_ransac(underdetermined).invalid());

    // Four axis points provide an exact, non-vacuous fit oracle for center and radius.
    const Vec2ds exact_circle{Vec2d(9., -3.), Vec2d(4., 2.), Vec2d(-1., -3.), Vec2d(4., -8.)};
    const Geometry::Circled taubin = Geometry::circle_taubin_newton(exact_circle);
    REQUIRE(taubin.valid());
    CHECK_THAT(taubin.center.x(), Catch::Matchers::WithinAbs(4., 1e-8));
    CHECK_THAT(taubin.center.y(), Catch::Matchers::WithinAbs(-3., 1e-8));
    CHECK_THAT(taubin.radius, Catch::Matchers::WithinAbs(5., 1e-8));

    double ransac_error = -1.;
    const Geometry::Circled ransac = Geometry::circle_ransac(exact_circle, 16, &ransac_error);
    REQUIRE(ransac.valid());
    CHECK_THAT(ransac.center.x(), Catch::Matchers::WithinAbs(4., 1e-8));
    CHECK_THAT(ransac.center.y(), Catch::Matchers::WithinAbs(-3., 1e-8));
    CHECK_THAT(ransac.radius, Catch::Matchers::WithinAbs(5., 1e-8));
    CHECK_THAT(ransac_error, Catch::Matchers::WithinAbs(0., 1e-8));
}

TEST_CASE("smallest_enclosing_circle_welzl", "[Geometry]") {
    // Some random points in plane.
    Points pts { 
        { 89243, 4359 }, { 763465, 59687 }, { 3245, 734987 }, { 2459867, 987634 }, { 759866, 67843982 }, { 9754687, 9834658 }, { 87235089, 743984373 }, 
        { 65874456, 2987546 }, { 98234524, 657654873 }, { 786243598, 287934765 }, { 824356, 734265 }, { 82576449, 7864534 }, { 7826345, 3984765 }
    };

    const auto c = Slic3r::Geometry::smallest_enclosing_circle_welzl(pts);
    // The radius returned is inflated by SCALED_EPSILON, thus all points should be inside.
    bool all_inside = std::all_of(pts.begin(), pts.end(), [c](const Point &pt){ return c.contains(pt.cast<double>()); });
    auto c2(c);
    c2.radius -= SCALED_EPSILON * 2.1;
    auto num_on_boundary = std::count_if(pts.begin(), pts.end(), [c2](const Point& pt) { return ! c2.contains(pt.cast<double>(), SCALED_EPSILON); });

    REQUIRE(all_inside);
    REQUIRE(num_on_boundary == 3);
}

SCENARIO("Path chaining", "[Geometry]") {
	GIVEN("A path") {
		Points points = { Point(26,26),Point(52,26),Point(0,26),Point(26,52),Point(26,0),Point(0,52),Point(52,52),Point(52,0) };
		THEN("Chained with no diagonals (thus 26 units long)") {
			std::vector<size_t> indices = chain_points(points);
			for (size_t i = 0; i + 1 < indices.size(); ++ i) {
				double dist = (points.at(indices.at(i)).cast<double>() - points.at(indices.at(i+1)).cast<double>()).norm();
				REQUIRE(std::abs(dist-26) <= EPSILON);
			}
		}
	}
	GIVEN("Gyroid infill end points") {
		Polylines polylines = {
			{ {28122608, 3221037}, {27919139, 56036027} },
			{ {33642863, 3400772}, {30875220, 56450360} },
			{ {34579315, 3599827}, {35049758, 55971572} },
			{ {26483070, 3374004}, {23971830, 55763598} },
			{ {38931405, 4678879}, {38740053, 55077714} },
			{ {20311895, 5015778}, {20079051, 54551952} },
			{ {16463068, 6773342}, {18823514, 53992958} },
			{ {44433771, 7424951}, {42629462, 53346059} },
			{ {15697614, 7329492}, {15350896, 52089991} },
			{ {48085792, 10147132}, {46435427, 50792118} },
			{ {48828819, 10972330}, {49126582, 48368374} },
			{ {9654526, 12656711}, {10264020, 47691584} },
			{ {5726905, 18648632}, {8070762, 45082416} },
			{ {54818187, 39579970}, {52974912, 43271272} }, 
			{ {4464342, 37371742}, {5027890, 39106220} },
			{ {54139746, 18417661}, {55177987, 38472580} }, 
			{ {56527590, 32058461}, {56316456, 34067185} },
			{ {3303988, 29215290}, {3569863, 32985633} },
			{ {56255666, 25025857}, {56478310, 27144087} }, 
			{ {4300034, 22805361}, {3667946, 25752601} },
			{ {8266122, 14250611}, {6244813, 17751595} },
			{ {12177955, 9886741}, {10703348, 11491900} } 
		};
		Polylines chained = chain_polylines(polylines);
		THEN("Chained taking the shortest path") {
			double connection_length = 0.;
			for (size_t i = 1; i < chained.size(); ++i) {
				const Polyline &pl1 = chained[i - 1];
				const Polyline &pl2 = chained[i];
				connection_length += (pl2.first_point() - pl1.last_point()).cast<double>().norm();
			}
			REQUIRE(connection_length < 85206000.);
		}
	}
	GIVEN("Loop pieces") {
		Point a { 2185796, 19058485 };
		Point b { 3957902, 18149382 };
		Point c { 2912841, 18790564 };
		Point d { 2831848, 18832390 };
		Point e { 3179601, 18627769 };
		Point f { 3137952, 18653370 };
		Polylines polylines = { { a, b },
								{ c, d },
								{ e, f },
								{ d, a },
								{ f, c },
								{ b, e } };
		Polylines chained = chain_polylines(polylines, &a);
		THEN("Connected without a gap") {
			for (size_t i = 0; i < chained.size(); ++i) {
				const Polyline &pl1 = (i == 0) ? chained.back() : chained[i - 1];
				const Polyline &pl2 = chained[i];
				REQUIRE(pl1.points.back() == pl2.points.front());
			}
		}
	}
}

SCENARIO("Line distances", "[Geometry]"){
    GIVEN("A line"){
        Line line(Point(0, 0), Point(20, 0));
        THEN("Points on the line segment have 0 distance"){
            REQUIRE(line.distance_to(Point(0, 0))  == 0);
            REQUIRE(line.distance_to(Point(20, 0)) == 0);
            REQUIRE(line.distance_to(Point(10, 0)) == 0);
        
        }
        THEN("Points off the line have the appropriate distance"){
            REQUIRE(line.distance_to(Point(10, 10)) == 10);
            REQUIRE(line.distance_to(Point(50, 0)) == 30);
        }
    }
}

SCENARIO("Polygon convex/concave detection", "[Geometry]"){
    GIVEN(("A Square with dimension 100")){
        auto square = Slic3r::Polygon /*new_scale*/(Points({
            Point(100,100),
            Point(200,100),
            Point(200,200),
            Point(100,200)}));

		WHEN("Angle threshold is not set") {
			THEN("It has 4 convex points counterclockwise"){
				auto cave_pts = square.concave_points();
				auto vex_pts = square.convex_points();
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 0);
				REQUIRE(vex_pts.size() == 4);
			}
			THEN("It has 4 concave points clockwise"){
				square.make_clockwise();
				auto cave_pts = square.concave_points();
				auto vex_pts = square.convex_points();
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 4);
				REQUIRE(vex_pts.size() == 0);
			}
		}
		WHEN("Angle threshold is greater than right angle") {
			double angle_threshold = M_PI*4/3;
			THEN("It has no convex points counterclockwise"){
				auto cave_pts = square.concave_points(angle_threshold);
				auto vex_pts = square.convex_points(angle_threshold);
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 0);
				REQUIRE(vex_pts.size() == 0);
			}
			THEN("It has no concave points clockwise"){
				square.make_clockwise();
				auto cave_pts = square.concave_points(angle_threshold);
				auto vex_pts = square.convex_points(angle_threshold);
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 0);
				REQUIRE(vex_pts.size() == 0);
			}
		}
		WHEN("Angle threshold is less than right angle") {
			double angle_threshold = M_PI/3;
			THEN("It has 4 convex points counterclockwise"){
				auto cave_pts = square.concave_points(angle_threshold);
				auto vex_pts = square.convex_points(angle_threshold);
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 0);
				REQUIRE(vex_pts.size() == 4);
			}
			THEN("It has 4 concave points clockwise"){
				square.make_clockwise();
				auto cave_pts = square.concave_points(angle_threshold);
				auto vex_pts = square.convex_points(angle_threshold);
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 4);
				REQUIRE(vex_pts.size() == 0);
			}
		}
		WHEN("Angle threshold is equal to right angle") {
			double angle_threshold = M_PI/2;
			THEN("It has no convex points counterclockwise"){
				auto cave_pts = square.concave_points(angle_threshold);
				auto vex_pts = square.convex_points(angle_threshold);
				CAPTURE(cave_pts);
				CAPTURE(vex_pts);
				REQUIRE(cave_pts.size() == 0);
				REQUIRE(vex_pts.size() == 0);
			}
		}
    }
    GIVEN("A Square with an extra colinearvertex"){
        auto square = Slic3r::Polygon /*new_scale*/(Points({
            Point(150,100),
            Point(200,100),
            Point(200,200),
            Point(100,200),
            Point(100,100)}));
        THEN("It has 4 convex points counterclockwise"){
            REQUIRE(square.concave_points().size() == 0);
            REQUIRE(square.convex_points().size() == 4);
        }
    }
    GIVEN("A Square with an extra collinear vertex in different order"){
        auto square = Slic3r::Polygon /*new_scale*/(Points({
            Point(200,200),
            Point(100,200),
            Point(100,100),
            Point(150,100),
            Point(200,100)}));
        THEN("It has 4 convex points counterclockwise"){
            REQUIRE(square.concave_points().size() == 0);
            REQUIRE(square.convex_points().size() == 4);
        }
    }

    GIVEN("A triangle"){
        auto triangle = Slic3r::Polygon(Points({
            Point(16000170,26257364),
            Point(714223,461012),
            Point(31286371,461008)
        }));
        THEN("it has three convex vertices"){
            REQUIRE(triangle.concave_points().size() == 0);
            REQUIRE(triangle.convex_points().size() == 3);
        }
    }

    GIVEN("A triangle with an extra collinear point"){
        auto triangle = Slic3r::Polygon(Points({
            Point(16000170,26257364),
            Point(714223,461012),
            Point(20000000,461012),
            Point(31286371,461012)
        }));
        THEN("it has three convex vertices"){
            REQUIRE(triangle.concave_points().size() == 0);
            REQUIRE(triangle.convex_points().size() == 3);
        }
    }
}

TEST_CASE("Triangle Simplification does not result in less than 3 points", "[Geometry]"){
    auto triangle = Slic3r::Polygon(Points({
        Point(16000170,26257364), Point(714223,461012), Point(31286371,461008)
    }));
    REQUIRE(triangle.simplify(250000).at(0).points.size() == 3);
}

SCENARIO("Ported from xs/t/14_geometry.t", "[Geometry]"){
    GIVEN(("square")){
    	Slic3r::Points points { { 100, 100 }, {100, 200 }, { 200, 200 }, { 200, 100 }, { 150, 150 } };
		Slic3r::Polygon hull = Slic3r::Geometry::convex_hull(points);
		SECTION("convex hull returns the correct number of points") { REQUIRE(hull.points.size() == 4); }
    }
	SECTION("arrange returns expected number of positions") {
		Pointfs positions;
		Slic3r::Geometry::arrange(4, Vec2d(20, 20), 5, nullptr, positions);
    	REQUIRE(positions.size() == 4);
    }
	SECTION("directions_parallel") {
    	REQUIRE(Slic3r::Geometry::directions_parallel(0, 0, 0)); 
    	REQUIRE(Slic3r::Geometry::directions_parallel(0, M_PI, 0)); 
    	REQUIRE(Slic3r::Geometry::directions_parallel(0, 0, M_PI / 180));
    	REQUIRE(Slic3r::Geometry::directions_parallel(0, M_PI, M_PI / 180));
    	REQUIRE_FALSE(Slic3r::Geometry::directions_parallel(M_PI /2, M_PI, 0));
    	REQUIRE_FALSE(Slic3r::Geometry::directions_parallel(M_PI /2, PI, M_PI /180));
    }
}

TEST_CASE("Convex polygon intersection on two disjoint squares", "[Geometry][Rotcalip]") {
    Polygon A{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    A.scale(1. / SCALING_FACTOR);

    Polygon B = A;
    B.translate(20 / SCALING_FACTOR, 0);

    bool is_inters = Geometry::convex_polygons_intersect(A, B);

    REQUIRE(is_inters == false);
}

TEST_CASE("Convex polygon intersection on two intersecting squares", "[Geometry][Rotcalip]") {
    Polygon A{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    A.scale(1. / SCALING_FACTOR);

    Polygon B = A;
    B.translate(5 / SCALING_FACTOR, 5 / SCALING_FACTOR);

    bool is_inters = Geometry::convex_polygons_intersect(A, B);

    REQUIRE(is_inters == true);
}

TEST_CASE("Convex polygon intersection on two squares touching one edge", "[Geometry][Rotcalip]") {
    Polygon A{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    A.scale(1. / SCALING_FACTOR);

    Polygon B = A;
    B.translate(10 / SCALING_FACTOR, 0);

    bool is_inters = Geometry::convex_polygons_intersect(A, B);

    REQUIRE(is_inters == false);
}

TEST_CASE("Convex polygon intersection on two squares touching one vertex", "[Geometry][Rotcalip]") {
    Polygon A{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    A.scale(1. / SCALING_FACTOR);

    Polygon B = A;
    B.translate(10 / SCALING_FACTOR, 10 / SCALING_FACTOR);

    bool is_inters = Geometry::convex_polygons_intersect(A, B);

    REQUIRE(is_inters == false);
}

TEST_CASE("Convex polygon intersection on two overlapping squares", "[Geometry][Rotcalip]") {
    Polygon A{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    A.scale(1. / SCALING_FACTOR);

    Polygon B = A;

    bool is_inters = Geometry::convex_polygons_intersect(A, B);

    REQUIRE(is_inters == true);
}

//// Only for benchmarking
//static Polygon gen_convex_poly(std::mt19937_64 &rg, size_t point_cnt)
//{
//    std::uniform_int_distribution<coord_t> dist(0, 100);

//    Polygon out;
//    out.points.reserve(point_cnt);

//    coord_t tr = dist(rg) * 2 / SCALING_FACTOR;

//    for (size_t i = 0; i < point_cnt; ++i)
//        out.points.emplace_back(tr + dist(rg) / SCALING_FACTOR,
//                                tr + dist(rg) / SCALING_FACTOR);

//    return Geometry::convex_hull(out.points);
//}
//TEST_CASE("Convex polygon intersection test on random polygons", "[Geometry]") {
//    constexpr size_t TEST_CNT = 1000;
//    constexpr size_t POINT_CNT = 1000;

//    auto seed = std::random_device{}();
////    unsigned long seed = 2525634386;
//    std::mt19937_64 rg{seed};
//    Benchmark bench;

//    auto tests = reserve_vector<std::pair<Polygon, Polygon>>(TEST_CNT);
//    auto results = reserve_vector<bool>(TEST_CNT);
//    auto expects = reserve_vector<bool>(TEST_CNT);

//    for (size_t i = 0; i < TEST_CNT; ++i) {
//        tests.emplace_back(gen_convex_poly(rg, POINT_CNT), gen_convex_poly(rg, POINT_CNT));
//    }

//    bench.start();
//    for (const auto &test : tests)
//        results.emplace_back(Geometry::convex_polygons_intersect(test.first, test.second));
//    bench.stop();

//    std::cout << "Test time: " << bench.getElapsedSec() << std::endl;

//    bench.start();
//    for (const auto &test : tests)
//        expects.emplace_back(!intersection(test.first, test.second).empty());
//    bench.stop();

//    std::cout << "Clipper time: " << bench.getElapsedSec() << std::endl;

//    REQUIRE(results.size() == expects.size());

//    auto seedstr = std::to_string(seed);
//    for (size_t i = 0; i < results.size(); ++i) {
//        // std::cout << expects[i] << " ";

//        if (results[i] != expects[i]) {
//            SVG svg{std::string("fail_seed") + seedstr + "_" + std::to_string(i) + ".svg"};
//            svg.draw(tests[i].first, "blue");
//            svg.draw(tests[i].second, "green");
//            svg.Close();

//            // std::cout << std::endl;
//        }
//        REQUIRE(results[i] == expects[i]);
//    }
//    std::cout << std::endl;

//}

struct Pair
{
    size_t first, second;
    bool operator==(const Pair &b) const { return first == b.first && second == b.second; }
};

template<> struct std::hash<Pair> {
    size_t operator()(const Pair &c) const
    {
        return c.first * PRINTER_PART_POLYGONS.size() + c.second;
    }
};

TEST_CASE("Convex polygon intersection test prusa polygons", "[Geometry][Rotcalip]") {

    // Overlap of the same polygon should always be an intersection
    for (size_t i = 0; i < PRINTER_PART_POLYGONS.size(); ++i) {
        Polygon P = PRINTER_PART_POLYGONS[i];
        P = Geometry::convex_hull(P.points);
        bool res = Geometry::convex_polygons_intersect(P, P);
        if (!res) {
            SVG svg{std::string("fail_self") + std::to_string(i) + ".svg"};
            svg.draw(P, "green");
            svg.Close();
        }
        REQUIRE(res == true);
    }

    std::unordered_set<Pair> combos;
    for (size_t i = 0; i < PRINTER_PART_POLYGONS.size(); ++i) {
        for (size_t j = 0; j < PRINTER_PART_POLYGONS.size(); ++j) {
            if (i != j) {
                size_t a = std::min(i, j), b = std::max(i, j);
                combos.insert(Pair{a, b});
            }
        }
    }

    // All disjoint
    for (const auto &combo : combos) {
        Polygon A = PRINTER_PART_POLYGONS[combo.first], B = PRINTER_PART_POLYGONS[combo.second];
        A = Geometry::convex_hull(A.points);
        B = Geometry::convex_hull(B.points);

        auto bba = A.bounding_box();
        auto bbb = B.bounding_box();

        A.translate(-bba.center());
        B.translate(-bbb.center());

        B.translate(bba.size() + bbb.size());

        bool res = Geometry::convex_polygons_intersect(A, B);
        bool ref = !intersection(A, B).empty();

        if (res != ref) {
            SVG svg{std::string("fail") + std::to_string(combo.first) + "_" + std::to_string(combo.second) + ".svg"};
            svg.draw(A, "blue");
            svg.draw(B, "green");
            svg.Close();
        }

        REQUIRE(res == ref);
    }

    // All intersecting
    for (const auto &combo : combos) {
        Polygon A = PRINTER_PART_POLYGONS[combo.first], B = PRINTER_PART_POLYGONS[combo.second];
        A = Geometry::convex_hull(A.points);
        B = Geometry::convex_hull(B.points);

        auto bba = A.bounding_box();
        auto bbb = B.bounding_box();

        A.translate(-bba.center());
        B.translate(-bbb.center());

        bool res = Geometry::convex_polygons_intersect(A, B);
        bool ref = !intersection(A, B).empty();

        if (res != ref) {
            SVG svg{std::string("fail") + std::to_string(combo.first) + "_" + std::to_string(combo.second) + ".svg"};
            svg.draw(A, "blue");
            svg.draw(B, "green");
            svg.Close();
        }

        REQUIRE(res == ref);
    }
}


namespace {

using Catch::Matchers::WithinAbs;

void check_linear_and_offset(const Transform3d &actual, const Matrix3d &expected_linear, const Vec3d &expected_offset,
                             double tolerance = 1e-12)
{
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 3; ++column)
            CHECK_THAT(actual.linear()(row, column), WithinAbs(expected_linear(row, column), tolerance));
        CHECK_THAT(actual.translation()[row], WithinAbs(expected_offset[row], tolerance));
    }
}

// Quarter turn about Z followed by a scale of (4, 3, 2) and the given mirror, offset by (5, -6, 7).
Geometry::Transformation scaled_quarter_turn(const Vec3d &mirror)
{
    return Geometry::Transformation(
        Geometry::assemble_transform(Vec3d(5., -6., 7.), Vec3d(0., 0., PI / 2.), Vec3d(4., 3., 2.), mirror));
}

Matrix3d quarter_turn_times(const Vec3d &signed_scale)
{
    return Eigen::AngleAxisd(PI / 2., Vec3d::UnitZ()).toRotationMatrix() * signed_scale.asDiagonal();
}

} // namespace

TEST_CASE("Transformation SVD recognizes axis-aligned anisotropic scale", "[Geometry][Transformation][SVD]")
{
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = Vec3d(2., 1., 3.).asDiagonal();

    const Geometry::TransformationSVD svd(matrix);
    CHECK(svd.anisotropic_scale);
    CHECK(svd.rotation_90_degrees);
    CHECK_FALSE(svd.skew);
    CHECK_FALSE(svd.mirror);

    Transform3d uniform_matrix = Transform3d::Identity();
    uniform_matrix.linear() = 2. * Matrix3d::Identity();
    const Geometry::TransformationSVD uniform_svd(uniform_matrix);
    CHECK_FALSE(uniform_svd.anisotropic_scale);
    CHECK_FALSE(uniform_svd.skew);
}

TEST_CASE("Transformation SVD distinguishes rotated anisotropic skew", "[Geometry][Transformation][SVD]")
{
    const Matrix3d axis_scale = Vec3d(2., 1., 1.).asDiagonal();
    const Matrix3d rotation = Eigen::AngleAxisd(PI / 4., Vec3d::UnitZ()).toRotationMatrix();
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = axis_scale * rotation;

    const Geometry::TransformationSVD svd(matrix);
    CHECK(svd.anisotropic_scale);
    CHECK_FALSE(svd.rotation_90_degrees);
    CHECK(svd.skew);
}

TEST_CASE("Transformation vector mirror setter treats zero as unmirrored and uses only input signs",
          "[Geometry][Transformation][Mirror]")
{
    const Vec3d offset(5., -6., 7.);
    Geometry::Transformation transform;
    transform.set_scaling_factor(Vec3d(2., 3., 4.));
    transform.set_offset(offset);
    transform.set_mirror(Vec3d(-1., 1., 1.));
    check_linear_and_offset(transform.get_matrix(), Vec3d(-2., 3., 4.).asDiagonal(), offset);

    // Zero un-mirrors the already mirrored X axis; 2 keeps Y; -0.5 mirrors Z.
    transform.set_mirror(Vec3d(0., 2., -0.5));
    check_linear_and_offset(transform.get_matrix(), Vec3d(2., 3., -4.).asDiagonal(), offset);
    CHECK(transform.get_scaling_factor().isApprox(Vec3d(2., 3., 4.), 1e-12));
}

TEST_CASE("Transformation axis mirror setter treats zero as unmirrored and uses only the input sign",
          "[Geometry][Transformation][Mirror]")
{
    Geometry::Transformation transform;
    transform.set_scaling_factor(Vec3d(2., 3., 4.));

    transform.set_mirror(X, -2.);
    check_linear_and_offset(transform.get_matrix(), Vec3d(-2., 3., 4.).asDiagonal(), Vec3d::Zero());
    transform.set_mirror(X, -0.5); // Already mirrored: a negative input keeps the reflection.
    check_linear_and_offset(transform.get_matrix(), Vec3d(-2., 3., 4.).asDiagonal(), Vec3d::Zero());
    transform.set_mirror(X, 0.);
    check_linear_and_offset(transform.get_matrix(), Vec3d(2., 3., 4.).asDiagonal(), Vec3d::Zero());
    transform.set_mirror(X, 0.5);
    check_linear_and_offset(transform.get_matrix(), Vec3d(2., 3., 4.).asDiagonal(), Vec3d::Zero());
}

TEST_CASE("Orthogonal transform axes remain free of skew after rotation and reflection",
          "[Geometry][Transformation][Skew]")
{
    const int mirror_axis = GENERATE(-1, 0, 1, 2);
    const double angle = GENERATE(0., PI / 7.);
    const Matrix3d rotation = Eigen::AngleAxisd(angle, Vec3d::UnitZ()).toRotationMatrix();
    Transform3d matrix = Transform3d::Identity();
    Vec3d scale(2., 3., 4.);
    if (mirror_axis >= 0)
        scale[mirror_axis] = -scale[mirror_axis];
    matrix.linear() = rotation * scale.asDiagonal();
    matrix.translation() = Vec3d(5., -6., 7.);

    // Independent geometric witness: nonuniform scaling and reflection keep the
    // transformed coordinate axes perpendicular.
    for (int a = 0; a < 3; ++a)
        for (int b = a + 1; b < 3; ++b)
            REQUIRE_THAT(matrix.linear().col(a).dot(matrix.linear().col(b)), Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK_FALSE(Geometry::Transformation(matrix).has_skew());
}


TEST_CASE("Sheared transform axes report skew with either determinant sign",
          "[Geometry][Transformation][Skew]")
{
    const int mirror_axis = GENERATE(-1, 0, 1, 2);
    const int first_axis = GENERATE(0, 1, 2);
    const int second_axis = (first_axis + 1) % 3;
    Transform3d matrix = Transform3d::Identity();
    Vec3d scale(2., 3., 4.);
    if (mirror_axis >= 0)
        scale[mirror_axis] = -scale[mirror_axis];
    matrix.linear() = scale.asDiagonal();
    matrix.linear()(first_axis, second_axis) = 0.75;
    matrix.translation() = Vec3d(-3., 8., 2.);

    // A shear makes these two transformed axes nonorthogonal, regardless of a
    // reflected axis or a translation of the object.
    REQUIRE(std::abs(matrix.linear().col(first_axis).dot(matrix.linear().col(second_axis))) > 1.);
    CHECK(Geometry::Transformation(matrix).has_skew());
}


TEST_CASE("Transform constructors preserve declared affine coordinate mapping",
          "[Geometry][Transformation][Construct]")
{
    const Transform3d translation = Geometry::translation_transform(Vec3d(3., -5., 7.));
    const Transform3d rotation = Geometry::rotation_transform(Vec3d(0., 0., PI / 2.));
    const Transform3d scaling = Geometry::scale_transform(Vec3d(2., 3., 4.));
    const Transform3d mirror = Geometry::scale_transform(Vec3d(-1., 1., 1.));

    Transform3d assembled_by_reference;
    Geometry::assemble_transform(assembled_by_reference, translation, rotation, scaling, mirror);
    const Transform3d assembled_by_value = Geometry::assemble_transform(translation, rotation, scaling, mirror);

    // Apply mirror, scale, quarter-turn, then translation by hand to an asymmetric point.
    const Vec3d sample(1., 2., 3.);
    const Vec3d expected(-3., -7., 19.);
    CHECK((assembled_by_reference * sample).isApprox(expected, 1e-12));
    CHECK((assembled_by_value * sample).isApprox(expected, 1e-12));

    Transform3d scalar_scale_by_reference;
    Geometry::scale_transform(scalar_scale_by_reference, 2.);
    const Transform3d scalar_scale_by_value = Geometry::scale_transform(2.);
    const Vec3d scale_sample(1., -2., 3.);
    const Vec3d scaled_expected(2., -4., 6.);
    CHECK((scalar_scale_by_reference * scale_sample).isApprox(scaled_expected, 1e-12));
    CHECK((scalar_scale_by_value * scale_sample).isApprox(scaled_expected, 1e-12));

    const Vec3d origin(7., -3., 5.);
    const Geometry::Transformation frame = Geometry::generate_transform(
        Vec3d(0., 4., 0.), Vec3d(-2., 0., 0.), Vec3d(0., 0., 3.), origin);
    const Vec3d local_point(2., 3., 4.);
    const Vec3d frame_expected(4., -1., 9.);
    CHECK((frame.get_matrix() * local_point).isApprox(frame_expected, 1e-12));
}

TEST_CASE("Rotating a transformation about a fixed point updates its origin and axes",
          "[Geometry][Transformation][Rotation]")
{
    Geometry::Transformation input(Geometry::scale_transform(Vec3d(2., 3., 4.)));
    input.set_offset(Vec3d(2., 3., 4.));

    const Vec3d pivot(10., 20., 30.);
    const Geometry::Transformation rotated = Geometry::mat_around_a_point_rotate(
        input, pivot, Vec3d::UnitZ(), float(PI / 2.));

    // A positive quarter-turn maps (x,y,z) to (-y,x,z). Applying that to the
    // scaled basis gives this explicit matrix; the translated origin is rotated
    // around the independently specified pivot. The angle is passed as a float,
    // hence the looser tolerance.
    Matrix3d expected_linear;
    expected_linear << 0., -3., 0., 2., 0., 0., 0., 0., 4.;
    check_linear_and_offset(rotated.get_matrix(), expected_linear, Vec3d(27., 12., 4.), 1e-5);
    const Vec3d transformed_x = rotated.get_matrix() * Vec3d::UnitX();
    CHECK(transformed_x.isApprox(Vec3d(27., 14., 4.), 1e-5));
}

TEST_CASE("Transformation quaternion readback recovers the Euler angles", "[Geometry][Transformation][Rotation]")
{
    const Vec3d euler(0.3, -0.2, 0.4);
    const Geometry::Transformation oriented(Geometry::rotation_transform(euler));
    const Vec3d recovered = oriented.get_rotation_by_quaternion();
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(recovered[axis], WithinAbs(euler[axis], 1e-12));
}

TEST_CASE("Transformation signed-scale accessors recompose the matrix with one reflection",
          "[Geometry][Transformation][Mirror][Scale]")
{
    // Mirror the axis with the largest scale. A matrix alone cannot tell "mirror X" from
    // "mirror Z plus a half turn about Y", and the decomposition reports the reflection on
    // the smallest-scale axis, so only decomposition-independent properties are asserted.
    const Matrix3d rotation = Eigen::AngleAxisd(0.3, Vec3d::UnitZ()).toRotationMatrix();
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = rotation * Vec3d(-4., 3., 2.).asDiagonal();
    const Geometry::Transformation transform(matrix);

    CHECK(transform.get_scaling_factor().isApprox(Vec3d(4., 3., 2.), 1e-12));
    const Matrix3d scale = transform.get_scaling_factor_matrix().linear();
    CHECK(scale.isApprox(Matrix3d(Vec3d(4., 3., 2.).asDiagonal()), 1e-12));

    const Matrix3d mirror = transform.get_mirror_matrix().linear();
    CHECK(mirror.isDiagonal(1e-12));
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(std::abs(mirror(axis, axis)), WithinAbs(1., 1e-12));
    CHECK_THAT(mirror.determinant(), WithinAbs(-1., 1e-12));

    const Matrix3d recomposed = transform.get_rotation_matrix().linear() * mirror * scale;
    CHECK(recomposed.isApprox(matrix.linear(), 1e-12));
}

TEST_CASE("Transformation axis setters change only their own component", "[Geometry][Transformation][Scale]")
{
    const Vec3d offset(5., -6., 7.);

    Geometry::Transformation positive = scaled_quarter_turn(Vec3d::Ones());
    positive.set_scaling_factor(Y, 5.);
    check_linear_and_offset(positive.get_matrix(), quarter_turn_times(Vec3d(4., 5., 2.)), offset);

    // The reflection of X survives a scale change of another axis, whichever axis the
    // decomposition attributes it to.
    Geometry::Transformation mirrored = scaled_quarter_turn(Vec3d(-1., 1., 1.));
    mirrored.set_scaling_factor(Y, 5.);
    check_linear_and_offset(mirrored.get_matrix(), quarter_turn_times(Vec3d(-4., 5., 2.)), offset);
    mirrored.set_scaling_factor(X, 6.);
    check_linear_and_offset(mirrored.get_matrix(), quarter_turn_times(Vec3d(-6., 5., 2.)), offset);

    // An angle a hair below zero wraps to a full turn, which must be treated as no rotation.
    Geometry::Transformation no_full_turn;
    no_full_turn.set_offset(offset);
    no_full_turn.set_rotation(Z, -1e-10);
    check_linear_and_offset(no_full_turn.get_matrix(), Matrix3d::Identity(), offset);
}

// Known bug: set_scaling_factor(Axis) overwrites the signed scale entry that carries the reflection with a positive value, un-mirroring the transform; fixed by the follow-up fix PR
TEST_CASE("Transformation axis scale setter keeps the reflection of the axis it changes",
          "[Geometry][Transformation][Scale][Mirror][!shouldfail]")
{
    Geometry::Transformation mirrored = scaled_quarter_turn(Vec3d(1., 1., -1.));
    mirrored.set_scaling_factor(Z, 5.);
    check_linear_and_offset(mirrored.get_matrix(), quarter_turn_times(Vec3d(4., 3., -5.)), Vec3d(5., -6., 7.));
}

// Known bug: set_scaling_factor(Vec3d) rebuilds the matrix from the rotation and the new positive scale, dropping any reflection; fixed by the follow-up fix PR
TEST_CASE("Transformation vector scale setter keeps an existing reflection",
          "[Geometry][Transformation][Scale][Mirror][!shouldfail]")
{
    Geometry::Transformation mirrored = scaled_quarter_turn(Vec3d(1., 1., -1.));
    mirrored.set_scaling_factor(Vec3d(4., 3., 5.));
    check_linear_and_offset(mirrored.get_matrix(), quarter_turn_times(Vec3d(4., 3., -5.)), Vec3d(5., -6., 7.));
}

TEST_CASE("Transformation rotation reset keeps scale and offset", "[Geometry][Transformation][Reset]")
{
    const Matrix3d rotation = Eigen::AngleAxisd(0.37, Vec3d::UnitZ()).toRotationMatrix();
    const Vec3d scale(4., 3., 2.);
    const Vec3d offset(-8., 6., 11.);
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = rotation * scale.asDiagonal();
    matrix.translation() = offset;

    Geometry::Transformation rotation_cleared(matrix);
    rotation_cleared.reset_rotation();
    check_linear_and_offset(rotation_cleared.get_matrix(), scale.asDiagonal(), offset);

    Geometry::Transformation cleared(matrix);
    cleared.reset();
    CHECK(cleared.get_matrix().isApprox(Transform3d::Identity(), 1e-12));
}

TEST_CASE("Transformation skew reset removes shear while keeping offset and volume",
          "[Geometry][Transformation][Reset][Skew]")
{
    const Vec3d offset(-3., 8., 2.);
    Transform3d matrix = Transform3d::Identity();
    matrix.linear() = Vec3d(2., 3., 4.).asDiagonal();
    matrix.linear()(0, 1) = 0.75;
    matrix.translation() = offset;
    Geometry::Transformation transform(matrix);
    REQUIRE(transform.has_skew());

    transform.reset_skew();
    CHECK_FALSE(transform.has_skew());
    CHECK(transform.get_offset().isApprox(offset, 1e-12));
    // The shear is upper triangular, so the volume scale is the product of the diagonal.
    CHECK_THAT(transform.get_matrix().linear().determinant(), WithinAbs(24., 1e-9));
}

TEST_CASE("Bounding boxes rotate around an explicit center", "[Geometry][BoundingBox]")
{
    const BoundingBox source(Point(10, 20), Point(14, 22));
    const BoundingBox rotated = source.rotated(PI / 2., Point(7, 17));

    REQUIRE(rotated.defined);
    CHECK(rotated.min == Point(2, 20));
    CHECK(rotated.max == Point(4, 24));
}

TEST_CASE("3D bounding boxes merge point clouds and expose xy footprints in both units",
          "[Geometry][BoundingBox]")
{
    const Pointf3s points{
        Vec3d(4., -7., 2.),
        Vec3d(10., 3., 15.),
        Vec3d(-2., 8., 9.)
    };
    BoundingBoxf3 bounds;
    bounds.merge(points);

    REQUIRE(bounds.defined);
    CHECK(bounds.min.isApprox(Vec3d(-2., -7., 2.), 1e-12));
    CHECK(bounds.max.isApprox(Vec3d(10., 8., 15.), 1e-12));
    CHECK(bounds.size().isApprox(Vec3d(12., 15., 13.), 1e-12));
    CHECK_THAT(bounds.max_size(), Catch::Matchers::WithinAbs(15., 1e-12));

    const Vec3d before_empty_merge_min = bounds.min;
    const Vec3d before_empty_merge_max = bounds.max;
    bounds.merge(BoundingBoxf3{});
    CHECK(bounds.defined);
    CHECK(bounds.min.isApprox(before_empty_merge_min, 1e-12));
    CHECK(bounds.max.isApprox(before_empty_merge_max, 1e-12));

    const Points raw_expected{
        Point(-2, -7), Point(10, -7), Point(10, 8), Point(-2, 8)
    };
    CHECK(bounds.polygon(false).points == raw_expected);

    const Points scaled_expected{
        Point::new_scale(-2., -7.), Point::new_scale(10., -7.),
        Point::new_scale(10., 8.), Point::new_scale(-2., 8.)
    };
    CHECK(bounds.polygon(true).points == scaled_expected);
}
