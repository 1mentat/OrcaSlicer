#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

TEST_CASE("Scaled point conversions retain all physical coordinates", "[Point][PointContracts]")
{
    const Vec3d physical(1.25, -2.5, 4.0);
    const Vec3crd expected(1250000, -2500000, 4000000);
    CHECK(Point3::new_scale(physical) == expected);
    CHECK(Point3::new_scale(Vec3f(1.25f, -2.5f, 4.0f)) == expected);
    CHECK(Point3::new_scale(1.25, -2.5, 4.0) == expected);
    CHECK_THAT((unscale(expected) - physical).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((unscale(Vec3d(expected.cast<double>())) - physical).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((unscale(expected.x(), expected.y(), expected.z()) - physical).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((unscale(Vec2d(1250000.0, -2500000.0)) - Vec2d(1.25, -2.5)).norm(), WithinAbs(0.0, 1e-12));
}

TEST_CASE("Point arithmetic retains height while projecting appends only XY", "[Point][PointContracts]")
{
    const Point3 original(8, -12, 20);
    Point3 changed = original;
    changed -= Point3(2, 4, 6);
    CHECK(changed == Point3(6, -16, 14));
    changed *= -0.5;
    CHECK(changed == Point3(-3, 8, -7));
    CHECK(original.operator*(2.0) == Point3(16, -24, 40));
    CHECK(Slic3r::operator*(original, 2.0) == Point3(16, -24, 40));
    CHECK(original == Point3(8, -12, 20));
    CHECK_THAT(original.distance_to(Point3(11, -8, 32)), WithinAbs(13.0, 1e-12));

    Points projected{Point(-1, -2)};
    append_points(projected, Points3{original, Point3(3, 5, -99)});
    CHECK(projected == Points{Point(-1, -2), Point(8, -12), Point(3, 5)});
    append_points(projected, Points3{});
    CHECK(projected.size() == 3);
}

TEST_CASE("Point rotations preserve height and rotate around the chosen center", "[Point][PointContracts]")
{
    const double quarter_turn = std::acos(-1.0) / 2.0;
    const Point3 original(7, 3, 11);
    CHECK(original.rotated(quarter_turn) == Point3(-3, 7, 11));
    CHECK(original.rotated(0.0, 1.0) == Point3(-3, 7, 11));
    CHECK(original.rotate_90_degree_ccw() == Point3(-3, 7, 11));
    CHECK(original.rotated(quarter_turn, Point3(2, 1, 100)) == Point3(0, 6, 11));
    CHECK(original == Point3(7, 3, 11));
    const Point flat(7, 3);
    CHECK(flat.rotated(quarter_turn, Point(2, 1)) == Point(0, 6));
    CHECK(flat.rotate_90_degree_ccw() == Point(-3, 7));
    CHECK(flat == Point(7, 3));
}

TEST_CASE("Point comparisons distinguish every axis from any axis", "[Point][PointContracts]")
{
    Point point(3, 9);
    CHECK(point.both_comp(Point(5, 10), "<"));
    CHECK_FALSE(point.both_comp(Point(5, 8), "<"));
    CHECK(point.any_comp(Point(5, 8), "<"));
    CHECK(point.any_comp(Point(5, 8), ">"));
    CHECK_FALSE(point.any_comp(Point(3, 9), "<"));
    CHECK_FALSE(point.any_comp(Point(3, 9), ">"));
    CHECK_FALSE(point.both_comp(Point(0, 0), "unsupported"));
    CHECK_FALSE(point.any_comp(Point(0, 0), "unsupported"));
}

TEST_CASE("Duplicate point queries distinguish open paths from closed rings", "[Point][PointContracts]")
{
    const Points ring{Point(0, 0), Point(4, 0), Point(0, 3), Point(0, 0)};
    const Points original = ring;
    CHECK(has_duplicate_points(ring));
    CHECK(ring == original);
    CHECK_FALSE(has_duplicate_successive_points(ring));
    CHECK(has_duplicate_successive_points_closed(ring));
    CHECK(has_duplicate_successive_points(Points{Point(1, 2), Point(1, 2)}));
    CHECK_FALSE(has_duplicate_successive_points_closed(Points{Point(1, 2)}));
    CHECK_FALSE(has_duplicate_successive_points_closed(Points{}));
    CHECK_FALSE(has_duplicate_points(Points{Point(0, 0), Point(2, 3)}));
}

TEST_CASE("Float point proximity excludes the tolerance boundary on either axis", "[Point][PointContracts]")
{
    const Vec2f origin(3.0f, -2.0f);
    CHECK(is_approx(origin, Vec2f(3.5f, -2.5f), 1.0f));
    CHECK_FALSE(is_approx(origin, Vec2f(4.0f, -2.0f), 1.0f));
    CHECK_FALSE(is_approx(origin, Vec2f(3.0f, -3.0f), 1.0f));
    CHECK(is_approx(Vec2f(3.5f, -2.5f), origin, 1.0f));
}

TEST_CASE("Transform bases ignore translation and identify odd reflections", "[Point][PointContracts]")
{
    Transform3d transform = Transform3d::Identity();
    transform.linear() << 0.0, -3.0, 0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 4.0;
    transform.translation() = Vec3d(30.0, -20.0, 10.0);
    const Vec3d x(0.0, 2.0, 0.0), y(-3.0, 0.0, 0.0), z(0.0, 0.0, 4.0);
    CHECK_THAT((get_x_base(transform) - x).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((get_y_base(transform) - y).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((get_z_base(transform) - z).norm(), WithinAbs(0.0, 1e-12));
    const Transform3d::LinearPart linear = transform.linear();
    CHECK_THAT((get_x_base(linear) - x).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((get_y_base(linear) - y).norm(), WithinAbs(0.0, 1e-12));
    CHECK_THAT((get_z_base(linear) - z).norm(), WithinAbs(0.0, 1e-12));
    CHECK_FALSE(has_reflection(transform));
    transform.linear().col(0) *= -1.0;
    CHECK(has_reflection(transform));
    transform.linear().col(1) *= -1.0;
    CHECK_FALSE(has_reflection(transform));
}

TEST_CASE("Float point vectors preserve order and input while applying an affine transform", "[Point][PointVectorTransformContracts]")
{
    Transform3f affine = Transform3f::Identity();
    affine.linear() << 0.0f, -2.0f, 0.0f,
                       3.0f,  0.0f, 0.0f,
                       0.0f,  0.0f, 0.5f;
    affine.translation() = Vec3f(10.0f, -4.0f, 2.0f);

    const std::vector<Vec3f> empty_points;
    CHECK(Slic3r::transform(empty_points, affine).empty());

    const std::vector<Vec3f> points{
        Vec3f(1.0f, 2.0f, 4.0f),
        Vec3f(-3.0f, 0.0f, 2.0f),
        Vec3f(0.0f, -1.0f, -2.0f),
    };
    const std::vector<Vec3f> original_points = points;
    const std::vector<Vec3f> transformed = Slic3r::transform(points, affine);
    const std::vector<Vec3f> expected{
        Vec3f(6.0f, -1.0f, 4.0f),
        Vec3f(10.0f, -13.0f, 3.0f),
        Vec3f(12.0f, -4.0f, 1.0f),
    };

    REQUIRE(transformed.size() == expected.size());
    REQUIRE(points.size() == original_points.size());
    for (std::size_t index = 0; index < expected.size(); ++index) {
        CHECK_THAT((transformed[index] - expected[index]).norm(), WithinAbs(0.0, 1e-5));
        CHECK_THAT((points[index] - original_points[index]).norm(), WithinAbs(0.0, 1e-6));
    }
}

TEST_CASE("Exact orientation predicates preserve signs for collinear and near-collinear vectors", "[Point][Int128Predicates]")
{
    CHECK(int128::cross(Point(3, 0), Point(0, 2)) == 1);
    CHECK(int128::cross(Point(-3, 0), Point(0, 2)) == -1);
    CHECK(int128::cross(Point(3, 2), Point(6, 4)) == 0);

    // The exact determinant is +1 although both rounded products are equal.
    const std::int64_t magnitude = std::int64_t{1} << 40;
    const Point near_collinear_v(magnitude, magnitude - 1);
    const Point near_collinear_w(magnitude + 1, magnitude);
    const Point origin(0, 0);
    CHECK(int128::cross(near_collinear_v, near_collinear_w) == 1);
    CHECK(int128::cross(near_collinear_w, near_collinear_v) == -1);
    CHECK(int128::orient(origin, near_collinear_v, near_collinear_w) == 1);
    CHECK(int128::orient(origin, near_collinear_w, near_collinear_v) == -1);

    const Point collinear_w(2 * magnitude, 2 * (magnitude - 1));
    CHECK(int128::cross(near_collinear_v, collinear_w) == 0);
    CHECK(int128::orient(origin, near_collinear_v, collinear_w) == 0);

    // Large orthogonal components put the rounded determinant well outside the error bound.
    const std::int64_t separated = std::int64_t{1} << 50;
    const Point separated_x(separated, 0);
    const Point separated_y(0, separated);
    CHECK(int128::cross(separated_x, separated_y) == 1);
    CHECK(int128::cross(separated_y, separated_x) == -1);

    // Large absolute coordinates are safe here: the predicate subtracts them to small deltas.
    const std::int64_t translation = std::int64_t{1} << 60;
    const Point translated_origin(translation, -translation);
    const Point translated_v(translation + magnitude, -translation + magnitude - 1);
    const Point translated_w(translation + magnitude + 1, -translation + magnitude);
    CHECK(int128::orient(translated_origin, translated_v, translated_w) == 1);
    CHECK(int128::orient(translated_origin, translated_w, translated_v) == -1);
}
