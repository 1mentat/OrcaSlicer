#include <limits>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/MeasureUtils.hpp"
#include <initializer_list>
#include <cstdint>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/catch_test_macros.hpp"
#include "libslic3r/Measure.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <libslic3r/Point.hpp>
#include <libslic3r/libslic3r.h>
#include <cstddef>
#include <memory>
#include <optional>
#include <set>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Measure;
using Catch::Matchers::WithinAbs;

namespace {

void check_vec(const Vec3d &actual, const Vec3d &expected, double tolerance = 1e-8)
{
    CHECK_THAT((actual - expected).norm(), WithinAbs(0.0, tolerance));
}

SurfaceFeature edge(const Vec3d &from, const Vec3d &to, std::optional<Vec3d> extra = std::nullopt)
{
    return SurfaceFeature(SurfaceFeatureType::Edge, from, to, extra);
}

SurfaceFeature circle(const Vec3d &center, const Vec3d &normal, double radius)
{
    return SurfaceFeature(SurfaceFeatureType::Circle, center, normal, std::nullopt, radius);
}

SurfaceFeature plane(int index, const Vec3d &normal, const Vec3d &origin,
                     std::shared_ptr<std::vector<SurfaceFeature>> world_features = nullptr)
{
    SurfaceFeature result(SurfaceFeatureType::Plane, normal, origin, std::nullopt,
                          static_cast<double>(index) + 0.0001);
    result.world_plane_features = std::move(world_features);
    return result;
}

void check_distance(const std::optional<DistAndPoints> &distance, double expected)
{
    REQUIRE(distance.has_value());
    CHECK_THAT(distance->dist, WithinAbs(expected, 1e-8));
}

} // namespace

TEST_CASE("Measure projection and feature transforms preserve analytic geometry", "[Measure][MeasureContracts]")
{
    const Vec3d origin(1.0, 2.0, 1.0);
    const Vec3d normal(0.0, 0.0, 2.0);
    Vec3d projected;
    REQUIRE(get_point_projection_to_plane(Vec3d(2.0, 3.0, 5.0), origin, normal, projected));
    check_vec(projected, Vec3d(2.0, 3.0, 1.0));

    const Vec3d in_plane = get_one_point_in_plane(origin, Vec3d(1.0, 0.0, 0.0));
    CHECK_THAT((in_plane - origin).dot(Vec3d(1.0, 0.0, 0.0)), WithinAbs(0.0, 1e-8));

    const Vec3d displacement(4.0, -2.0, 1.0);
    SurfaceFeature point(Vec3d(1.0, 2.0, 3.0));
    point.translate(displacement);
    check_vec(point.get_point(), Vec3d(5.0, 0.0, 4.0));

    const Vec3d edge_extra(0.0, 1.0, 0.0);
    SurfaceFeature translated_edge = edge(Vec3d::Zero(), Vec3d(1.0, 0.0, 0.0), edge_extra);
    translated_edge.translate(displacement);
    const auto [edge_start, edge_end] = translated_edge.get_edge();
    check_vec(edge_start, displacement);
    check_vec(edge_end, Vec3d(5.0, -2.0, 1.0));
    REQUIRE(translated_edge.get_extra_point().has_value());
    check_vec(*translated_edge.get_extra_point(), Vec3d(4.0, -1.0, 1.0));

    SurfaceFeature translated_plane = plane(3, Vec3d::UnitZ(), Vec3d(1.0, 2.0, 3.0));
    translated_plane.translate(displacement);
    const auto [plane_index, plane_normal, plane_origin] = translated_plane.get_plane();
    CHECK(plane_index == 3);
    check_vec(plane_normal, Vec3d::UnitZ());
    check_vec(plane_origin, Vec3d(5.0, 0.0, 4.0));

    SurfaceFeature translated_circle = circle(Vec3d(1.0, 2.0, 3.0), Vec3d::UnitZ(), 2.5);
    translated_circle.translate(displacement);
    const auto [circle_center, circle_radius, circle_normal] = translated_circle.get_circle();
    check_vec(circle_center, Vec3d(5.0, 0.0, 4.0));
    check_vec(circle_normal, Vec3d::UnitZ());
    CHECK_THAT(circle_radius, WithinAbs(2.5, 1e-8));

    SurfaceFeature clone(Vec3d::Zero());
    clone.clone(translated_edge);
    CHECK(clone == translated_edge);
    CHECK(edge(Vec3d(1.0, 0.0, 0.0), Vec3d::Zero()) == edge(Vec3d::Zero(), Vec3d(1.0, 0.0, 0.0)));

    Transform3d rigid = Transform3d::Identity();
    rigid.linear() = Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitZ()).toRotationMatrix();
    rigid.translation() = displacement;
    SurfaceFeature transformed_point(Vec3d(1.0, 2.0, 3.0));
    transformed_point.translate(rigid);
    check_vec(transformed_point.get_point(), rigid * Vec3d(1.0, 2.0, 3.0));
    SurfaceFeature transformed_plane_feature = plane(4, Vec3d::UnitZ(), Vec3d(1.0, 2.0, 3.0));
    transformed_plane_feature.translate(rigid);
    const auto [transformed_plane_index, transformed_plane_normal, transformed_plane_origin] = transformed_plane_feature.get_plane();
    CHECK(transformed_plane_index == 4);
    check_vec(transformed_plane_normal, Vec3d::UnitZ());
    check_vec(transformed_plane_origin, rigid * Vec3d(1.0, 2.0, 3.0));
    SurfaceFeature transformed_circle_feature = circle(Vec3d(1.0, 2.0, 3.0), Vec3d::UnitZ(), 2.5);
    transformed_circle_feature.translate(rigid);
    const auto [transformed_center, transformed_radius, transformed_normal] = transformed_circle_feature.get_circle();
    check_vec(transformed_center, rigid * Vec3d(1.0, 2.0, 3.0));
    check_vec(transformed_normal, Vec3d::UnitZ());
    CHECK_THAT(transformed_radius, WithinAbs(2.5, 1e-8));
}

TEST_CASE("Measure feature distances and angles match analytic witnesses", "[Measure][MeasureContracts]")
{
    const SurfaceFeature p0(Vec3d::Zero());
    const SurfaceFeature p1(Vec3d(3.0, 4.0, 0.0));
    const MeasurementResult point_distance = get_measurement(p0, p1);
    check_distance(point_distance.distance_strict, 5.0);
    REQUIRE(point_distance.distance_xyz.has_value());
    check_vec(*point_distance.distance_xyz, Vec3d(3.0, 4.0, 0.0));
    const MeasurementResult reversed_points = get_measurement(p1, p0);
    check_distance(reversed_points.distance_strict, 5.0);
    REQUIRE(reversed_points.distance_xyz.has_value());
    check_vec(*reversed_points.distance_xyz, Vec3d(-3.0, -4.0, 0.0));

    const SurfaceFeature x_edge = edge(Vec3d::Zero(), Vec3d(4.0, 0.0, 0.0));
    const MeasurementResult point_edge = get_measurement(SurfaceFeature(Vec3d(2.0, 3.0, 0.0)), x_edge);
    check_distance(point_edge.distance_infinite, 3.0);
    check_distance(point_edge.distance_strict, 3.0);
    const MeasurementResult outside_edge = get_measurement(SurfaceFeature(Vec3d(10.0, 0.0, 0.0)), x_edge);
    check_distance(outside_edge.distance_infinite, 0.0);
    check_distance(outside_edge.distance_strict, 6.0);

    const SurfaceFeature unit_circle = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const MeasurementResult point_circle = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 0.0)), unit_circle);
    check_distance(point_circle.distance_strict, 3.0);
    REQUIRE(point_circle.distance_strict.has_value());
    check_vec(point_circle.distance_strict->to, Vec3d(2.0, 0.0, 0.0));

    const SurfaceFeature y_edge = edge(Vec3d(2.0, 3.0, 0.0), Vec3d(2.0, 7.0, 0.0));
    const MeasurementResult edge_edge = get_measurement(x_edge, y_edge);
    check_distance(edge_edge.distance_infinite, 3.0);
    REQUIRE(edge_edge.angle.has_value());
    CHECK_THAT(edge_edge.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    CHECK(edge_edge.angle->coplanar);

    const SurfaceFeature offset_edge = edge(Vec3d(-5.0, 3.0, 0.0), Vec3d(5.0, 3.0, 0.0));
    const MeasurementResult edge_circle = get_measurement(offset_edge, unit_circle);
    check_distance(edge_circle.distance_infinite, 1.0);

    const SurfaceFeature second_circle = circle(Vec3d(5.0, 0.0, 0.0), Vec3d::UnitZ(), 2.0);
    const MeasurementResult circle_distance = get_measurement(unit_circle, second_circle);
    check_distance(circle_distance.distance_infinite, 1.0);
    const MeasurementResult circle_centers = get_measurement(unit_circle, second_circle, true);
    check_distance(circle_centers.distance_strict, 5.0);

    const SurfaceFeature z_plane = plane(0, Vec3d::UnitZ(), Vec3d::Zero());
    const SurfaceFeature raised_plane = plane(1, Vec3d::UnitZ(), Vec3d(0.0, 0.0, 3.0));
    const MeasurementResult parallel_planes = get_measurement(z_plane, raised_plane);
    check_distance(parallel_planes.distance_infinite, 3.0);
    // Keep both plane origins away from their intersection line; the angle
    // witness construction needs a nonzero in-plane direction on each face.
    const MeasurementResult perpendicular_planes = get_measurement(
        plane(2, Vec3d::UnitZ(), Vec3d(1.0, 0.0, 0.0)),
        plane(3, Vec3d::UnitX(), Vec3d(0.0, 0.0, 3.0)));
    REQUIRE(perpendicular_planes.angle.has_value());
    CHECK_THAT(perpendicular_planes.angle->angle, WithinAbs(0.5 * PI, 1e-8));

    // Non-parallel edge/plane and circle/plane branches consult this pointer.
    // Supplying it explicitly is part of the public call precondition.
    const auto plane_features = std::make_shared<std::vector<SurfaceFeature>>();
    const SurfaceFeature bounded_plane = plane(3, Vec3d::UnitZ(), Vec3d::Zero(), plane_features);
    const MeasurementResult vertical_edge = get_measurement(edge(Vec3d(1.0, 1.0, -1.0), Vec3d(1.0, 1.0, 1.0)), bounded_plane);
    REQUIRE(vertical_edge.angle.has_value());
    CHECK_THAT(vertical_edge.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    const MeasurementResult elevated_circle = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0), bounded_plane);
    check_distance(elevated_circle.distance_infinite, 5.0);
}

TEST_CASE("Measure assembly and coordinate-distance contracts are explicit", "[Measure][MeasureContracts]")
{
    const SurfaceFeature point_a(Vec3d::Zero());
    const SurfaceFeature point_b(Vec3d(1.0, 2.0, 3.0));
    const SurfaceFeature circle_a = circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0);
    const SurfaceFeature circle_b = circle(Vec3d(1.0, 2.0, 3.0), Vec3d::UnitZ(), 1.0);
    CHECK(can_set_xyz_distance(point_a, point_b));
    CHECK(can_set_xyz_distance(circle_a, circle_b));
    CHECK_FALSE(can_set_xyz_distance(point_a, edge(Vec3d::Zero(), Vec3d::UnitX())));
    CHECK_FALSE(can_set_xyz_distance(circle_a, plane(0, Vec3d::UnitZ(), Vec3d::Zero())));

    const SurfaceFeature z0 = plane(0, Vec3d::UnitZ(), Vec3d::Zero());
    const SurfaceFeature z3 = plane(1, Vec3d::UnitZ(), Vec3d(0.0, 0.0, 3.0));
    const AssemblyAction parallel = get_assembly_action(z0, z3);
    CHECK_FALSE(parallel.can_set_to_parallel);
    CHECK(parallel.has_parallel_distance);
    CHECK(parallel.can_set_to_center_coincidence);
    CHECK(parallel.can_around_center_of_faces);
    CHECK_THAT(parallel.parallel_distance, WithinAbs(3.0, 1e-6));
    const AssemblyAction below = get_assembly_action(z0, plane(2, Vec3d::UnitZ(), Vec3d(0.0, 0.0, -3.0)));
    CHECK_THAT(below.parallel_distance, WithinAbs(-3.0, 1e-6));

    const AssemblyAction perpendicular = get_assembly_action(z0, plane(3, Vec3d::UnitX(), Vec3d::Zero()));
    CHECK(perpendicular.can_set_to_parallel);
    CHECK_FALSE(perpendicular.has_parallel_distance);
    CHECK_THAT(perpendicular.angle_radian, WithinAbs(0.5 * PI, 1e-6));
}

TEST_CASE("Measure handles perpendicular planes sharing their intersection origin", "[Measure][MeasureContracts]")
{
    // Both origins lie on the common intersection line.  This is a valid
    // plane pair and must not depend on selecting an offset witness point.
    const MeasurementResult result = get_measurement(
        plane(5, Vec3d::UnitZ(), Vec3d::Zero()),
        plane(6, Vec3d::UnitX(), Vec3d::Zero()));
    REQUIRE(result.angle.has_value());
    REQUIRE(std::isfinite(result.angle->angle));
    CHECK_THAT(result.angle->angle, WithinAbs(0.5 * PI, 1e-8));
}

TEST_CASE("Measuring groups cube facets into planar features and honors world transforms", "[Measure][MeasureContracts]")
{
    const indexed_triangle_set cube_mesh = its_make_cube(20.0, 20.0, 20.0);
    Measuring measuring(cube_mesh);
    REQUIRE(measuring.get_its().indices.size() == 12);
    REQUIRE(measuring.get_num_of_planes() == 6);

    std::set<int> all_faces;
    for (int plane_id = 0; plane_id < measuring.get_num_of_planes(); ++plane_id) {
        const auto &facets = measuring.get_plane_triangle_indices(plane_id);
        REQUIRE(facets.size() == 2);
        all_faces.insert(facets.begin(), facets.end());

        const auto &features = measuring.get_plane_features(static_cast<unsigned int>(plane_id));
        REQUIRE(features.size() == 5);
        REQUIRE(features.back().get_type() == SurfaceFeatureType::Plane);
        const auto [index, normal, origin] = features.back().get_plane();
        CHECK(index == plane_id);
        CHECK_THAT(normal.norm(), WithinAbs(1.0, 1e-8));
        int dominant_axis = 0;
        normal.cwiseAbs().maxCoeff(&dominant_axis);
        CHECK_THAT(std::abs(std::abs(normal[dominant_axis]) - 1.0), WithinAbs(0.0, 1e-8));
        CHECK_THAT(std::min(std::abs(origin[dominant_axis]), std::abs(origin[dominant_axis] - 20.0)),
                   WithinAbs(0.0, 1e-8));
        size_t edges = 0;
        for (const SurfaceFeature &feature : features)
            if (feature.get_type() == SurfaceFeatureType::Edge)
                ++edges;
        CHECK(edges == 4);
    }
    CHECK(all_faces.size() == 12);

    const auto plane_pick = measuring.get_feature(0, Vec3d(10.0, 10.0, 0.0), Transform3d::Identity(), true);
    REQUIRE(plane_pick.has_value());
    REQUIRE(plane_pick->get_type() == SurfaceFeatureType::Plane);
    const auto [plane_index, plane_normal, plane_origin] = plane_pick->get_plane();
    CHECK(plane_index == 0);
    CHECK_THAT(std::abs(plane_normal.z()), WithinAbs(1.0, 1e-8));
    check_vec(plane_origin, Vec3d(10.0, 10.0, 0.0));

    Transform3d world = Transform3d::Identity();
    world.translation() = Vec3d(3.0, 4.0, 5.0);
    const auto transformed_plane = measuring.get_feature(0, Vec3d(10.0, 10.0, 0.0), world, true);
    REQUIRE(transformed_plane.has_value());
    const auto [transformed_index, transformed_normal, transformed_origin] = transformed_plane->get_plane();
    CHECK(transformed_index == 0);
    check_vec(transformed_normal, plane_normal);
    check_vec(transformed_origin, Vec3d(13.0, 14.0, 5.0));

    const auto edge_pick = measuring.get_feature(0, Vec3d(10.0, 0.0, 0.0), Transform3d::Identity(), false);
    REQUIRE(edge_pick.has_value());
    CHECK(edge_pick->get_type() == SurfaceFeatureType::Edge);
    CHECK(measuring.get_feature(cube_mesh.indices.size(), Vec3d::Zero(), Transform3d::Identity(), false) == std::nullopt);
}

namespace measure_utils_contracts {
using Catch::Matchers::WithinRel;
namespace {
void check_poly(const Polynomial1 &p, std::initializer_list<double> expected)
{
    REQUIRE(p.GetDegree() + 1 == expected.size());
    std::size_t i = 0;
    for (double coefficient : expected)
        CHECK_THAT(p[static_cast<uint32_t>(i++)], WithinAbs(coefficient, 1e-12));
}

double eval_reference(std::initializer_list<double> coefficients, double x)
{
    double result = 0.0;
    double power = 1.0;
    for (double coefficient : coefficients) {
        result += coefficient * power;
        power *= x;
    }
    return result;
}

} // namespace

TEST_CASE("MeasureUtils polynomials retain coefficients and evaluate independently", "[Measure][MeasureUtilsExtended]")
{
    Polynomial1 p{2.0, -3.0, 0.5};
    CHECK(p.GetDegree() == 2);
    for (double x : {-2.0, -0.25, 0.0, 1.5, 4.0})
        CHECK_THAT(p(x), WithinRel(eval_reference({2.0, -3.0, 0.5}, x), 1e-12));

    Polynomial1 reduced{4.0, 0.0, 0.0};
    CHECK(reduced.GetDegree() == 0);
    CHECK_THAT(reduced(2.5), WithinAbs(4.0, 1e-12));
    Polynomial1 zero_leading{0.0, 0.0, 0.0};
    CHECK(zero_leading.GetDegree() == 0);
    CHECK_THAT(zero_leading(11.0), WithinAbs(0.0, 1e-12));
}

TEST_CASE("MeasureUtils polynomial arithmetic matches convolution and pointwise sums", "[Measure][MeasureUtilsExtended]")
{
    const Polynomial1 p{1.0, 2.0, 3.0};
    const Polynomial1 q{4.0, -1.0, 0.0, 2.0};
    const Polynomial1 sum = q + p;
    const Polynomial1 difference = q - p;
    const Polynomial1 product = p * q;
    check_poly(sum, {5.0, 1.0, 3.0, 2.0});
    check_poly(difference, {3.0, -3.0, -3.0, 2.0});
    check_poly(product, {4.0, 7.0, 10.0, -1.0, 4.0, 6.0});
    for (double x : {-1.0, 0.5, 2.0})
        CHECK_THAT(product(x), WithinRel(p(x) * q(x), 1e-12));

    const Polynomial1 scaled = 2.5 * p;
    check_poly(scaled, {2.5, 5.0, 7.5});
}

TEST_CASE("MeasureUtils mutable polynomial degree and coefficient reset are explicit", "[Measure][MeasureUtilsExtended]")
{
    Polynomial1 p(4);
    CHECK(p.GetDegree() == 4);
    p.SetCoefficients(3.0);
    CHECK(p.GetDegree() == 4);
    CHECK_THAT(p(2.0), WithinAbs(3.0 * 31.0, 1e-12));
    p[4] = 0.0;
    p.EliminateLeadingZeros();
    CHECK(p.GetDegree() == 3);
    p[3] = 0.0;
    p.EliminateLeadingZeros();
    CHECK(p.GetDegree() == 2);
}

TEST_CASE("MeasureUtils root finder handles linear, constant, and zero polynomials", "[Measure][MeasureUtilsExtended]")
{
    const double linear[] = {-6.0, 2.0};
    double roots[4] = {};
    CHECK(RootsPolynomial::Find(1, linear, 64, roots) == 1);
    CHECK_THAT(roots[0], WithinAbs(3.0, 1e-12));

    const double constant[] = {7.0};
    CHECK(RootsPolynomial::Find(0, constant, 64, roots) == 0);
    const double zero[] = {0.0, 0.0, 0.0};
    CHECK(RootsPolynomial::Find(2, zero, 64, roots) == 1);
    CHECK_THAT(roots[0], WithinAbs(0.0, 1e-12));
    CHECK(RootsPolynomial::Find(-1, linear, 64, roots) == 0);
    CHECK(RootsPolynomial::Find(1, nullptr, 64, roots) == 0);
}

TEST_CASE("MeasureUtils root finder returns all simple quadratic roots", "[Measure][MeasureUtilsExtended]")
{
    const double coefficients[] = {6.0, -5.0, 1.0};
    double roots[4] = {};
    const int count = RootsPolynomial::Find(2, coefficients, 128, roots);
    REQUIRE(count == 2);
    std::sort(roots, roots + count);
    CHECK_THAT(roots[0], WithinAbs(2.0, 1e-8));
    CHECK_THAT(roots[1], WithinAbs(3.0, 1e-8));
    CHECK_THAT(coefficients[0] + coefficients[1] * roots[0] + roots[0] * roots[0], WithinAbs(0.0, 1e-7));
    CHECK_THAT(coefficients[0] + coefficients[1] * roots[1] + roots[1] * roots[1], WithinAbs(0.0, 1e-7));
}

TEST_CASE("MeasureUtils root finder handles repeated and leading-zero equations", "[Measure][MeasureUtilsExtended]")
{
    const double repeated[] = {-4.0, 4.0, -1.0};
    double roots[4] = {};
    const int repeated_count = RootsPolynomial::Find(2, repeated, 128, roots);
    REQUIRE(repeated_count >= 1);
    REQUIRE(repeated_count <= 2);
    for (int i = 0; i < repeated_count; ++i)
        CHECK_THAT(roots[i], WithinAbs(2.0, 1e-7));

    const double leading_zero[] = {-2.0, 1.0, 0.0};
    const int count = RootsPolynomial::Find(2, leading_zero, 128, roots);
    REQUIRE(count == 1);
    CHECK_THAT(roots[0], WithinAbs(2.0, 1e-8));
}

TEST_CASE("MeasureUtils interval root finder honors endpoint and bracket contracts", "[Measure][MeasureUtilsExtended]")
{
    const double coefficients[] = {-2.0, 1.0};
    double root = -1.0;
    CHECK(RootsPolynomial::Find(1, coefficients, 2.0, 4.0, 64, root));
    CHECK_THAT(root, WithinAbs(2.0, 1e-12));
    CHECK_FALSE(RootsPolynomial::Find(1, coefficients, 3.0, 4.0, 64, root));
    CHECK_FALSE(RootsPolynomial::Find(1, coefficients, 4.0, 3.0, 64, root));

    const double quadratic[] = {-6.0, 0.0, 1.0};
    CHECK(RootsPolynomial::Find(2, quadratic, 0.0, 3.0, 64, root));
    CHECK_THAT(root, WithinAbs(std::sqrt(6.0), 1e-8));
}

TEST_CASE("MeasureUtils bisection converges on a narrow root interval", "[Measure][MeasureUtilsExtended]")
{
    const double coefficients[] = {-0.125, 0.0, 1.0};
    double root = 0.0;
    REQUIRE(RootsPolynomial::Find(2, coefficients, 0.0, 1.0, 128, root));
    CHECK_THAT(root, WithinAbs(0.3535533905932738, 1e-8));
}

TEST_CASE("MeasureUtils orthogonal vectors satisfy dot and length invariants", "[Measure][MeasureUtilsExtended]")
{
    for (const Vec3d &v : {Vec3d(5.0, 1.0, 2.0), Vec3d(1.0, 5.0, 2.0), Vec3d(1.0, 2.0, 5.0), Vec3d(-3.0, 4.0, -2.0)}) {
        const Vec3d u = get_orthogonal(v, false);
        const Vec3d unit = get_orthogonal(v, true);
        CHECK_THAT(u.dot(v), WithinAbs(0.0, 1e-12));
        CHECK_THAT(unit.dot(v), WithinAbs(0.0, 1e-12));
        CHECK_THAT(unit.norm(), WithinAbs(1.0, 1e-12));
        int max_index = 0;
        if (std::abs(v[1]) > std::abs(v[max_index])) max_index = 1;
        if (std::abs(v[2]) > std::abs(v[max_index])) max_index = 2;
        const int next_index = (max_index + 1) % 3;
        CHECK_THAT(u.norm(), WithinAbs(std::sqrt(v[max_index] * v[max_index] +
                                                  v[next_index] * v[next_index]), 1e-12));
    }
}

TEST_CASE("MeasureUtils orthogonal choice is deterministic at tied maxima", "[Measure][MeasureUtilsExtended]")
{
    const Vec3d v(3.0, 3.0, 1.0);
    const Vec3d u = get_orthogonal(v, false);
    CHECK_THAT(u[0], WithinAbs(3.0, 1e-12));
    CHECK_THAT(u[1], WithinAbs(-3.0, 1e-12));
    CHECK_THAT(u[2], WithinAbs(0.0, 1e-12));
}


} // namespace measure_utils_contracts

namespace measure_geometry_extended {

using namespace Slic3r;
using namespace Slic3r::Measure;
using Catch::Matchers::WithinAbs;

namespace {

void check_vec(const Vec3d &actual, const Vec3d &expected, double tolerance = 1e-7)
{
    CHECK_THAT((actual - expected).norm(), WithinAbs(0.0, tolerance));
}

SurfaceFeature edge(const Vec3d &from, const Vec3d &to)
{
    return SurfaceFeature(SurfaceFeatureType::Edge, from, to);
}

SurfaceFeature circle(const Vec3d &center, const Vec3d &normal, double radius)
{
    return SurfaceFeature(SurfaceFeatureType::Circle, center, normal, std::nullopt, radius);
}

SurfaceFeature plane(int index, const Vec3d &normal, const Vec3d &origin,
                     std::shared_ptr<std::vector<SurfaceFeature>> boundaries = nullptr)
{
    SurfaceFeature result(SurfaceFeatureType::Plane, normal, origin, std::nullopt,
                          static_cast<double>(index) + 0.0001);
    result.world_plane_features = std::move(boundaries);
    return result;
}

void require_distance(const std::optional<DistAndPoints> &distance, double expected,
                      double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    CHECK_THAT(distance->dist, WithinAbs(expected, tolerance));
    CHECK_THAT((distance->to - distance->from).norm(), WithinAbs(distance->dist, tolerance));
}

void require_point_circle_witness(const std::optional<DistAndPoints> &distance,
                                 const SurfaceFeature &circle_feature,
                                 const Vec3d &query_point,
                                 double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    const auto [center, radius, normal] = circle_feature.get_circle();
    const Vec3d unit_normal = normal.normalized();
    CHECK_THAT((distance->from - query_point).norm(), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->to - center).dot(unit_normal), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->to - center).norm(), WithinAbs(radius, tolerance));
    CHECK_THAT((distance->to - distance->from).norm(), WithinAbs(distance->dist, tolerance));
}

void require_reversed_point_circle_witness(const std::optional<DistAndPoints> &distance,
                                           const SurfaceFeature &circle_feature,
                                           const Vec3d &query_point,
                                           double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    const auto [center, radius, normal] = circle_feature.get_circle();
    const Vec3d unit_normal = normal.normalized();
    CHECK_THAT((distance->to - query_point).norm(), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->from - center).dot(unit_normal), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->from - center).norm(), WithinAbs(radius, tolerance));
    CHECK_THAT((distance->to - distance->from).norm(), WithinAbs(distance->dist, tolerance));
}

void require_circle_pair_witness(const std::optional<DistAndPoints> &distance,
                                 const SurfaceFeature &from_circle,
                                 const SurfaceFeature &to_circle,
                                 double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    const auto [from_center, from_radius, from_normal] = from_circle.get_circle();
    const auto [to_center, to_radius, to_normal] = to_circle.get_circle();
    const Vec3d from_unit_normal = from_normal.normalized();
    const Vec3d to_unit_normal = to_normal.normalized();
    CHECK_THAT((distance->from - from_center).dot(from_unit_normal), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->from - from_center).norm(), WithinAbs(from_radius, tolerance));
    CHECK_THAT((distance->to - to_center).dot(to_unit_normal), WithinAbs(0.0, tolerance));
    CHECK_THAT((distance->to - to_center).norm(), WithinAbs(to_radius, tolerance));
    CHECK_THAT((distance->to - distance->from).norm(), WithinAbs(distance->dist, tolerance));
}


} // namespace

TEST_CASE("Geometry projection covers oblique normals and alternate basis", "[Measure][MeasureGeometryExtended]")
{
    const Vec3d origin(2.0, -3.0, 4.0);
    const Vec3d normal(1.0, 2.0, 3.0);
    const Vec3d point(5.0, 7.0, -1.0);
    Vec3d projected;
    REQUIRE(get_point_projection_to_plane(point, origin, normal, projected));
    CHECK_THAT((projected - origin).dot(normal), WithinAbs(0.0, 1e-8));
    CHECK_THAT((point - projected).cross(normal).norm(), WithinAbs(0.0, 1e-8));

    const Vec3d x_aligned = get_one_point_in_plane(origin, Vec3d::UnitX());
    CHECK_THAT((x_aligned - origin).dot(Vec3d::UnitX()), WithinAbs(0.0, 1e-8));
    CHECK_THAT((x_aligned - origin).norm(), WithinAbs(1.0, 1e-8));
    const Vec3d oblique = get_one_point_in_plane(origin, normal);
    CHECK_THAT((oblique - origin).dot(normal), WithinAbs(0.0, 1e-8));
}

TEST_CASE("Affine feature transforms preserve representation policy", "[Measure][MeasureGeometryExtended]")
{
    Transform3d affine = Transform3d::Identity();
    affine.linear() << 2.0, 0.0, 0.0,
                       0.0, 3.0, 0.0,
                       0.0, 0.0, 1.5;
    affine.translation() = Vec3d(-4.0, 7.0, 2.0);

    SurfaceFeature point(Vec3d(1.0, 2.0, 3.0));
    point.translate(affine);
    check_vec(point.get_point(), affine * Vec3d(1.0, 2.0, 3.0));

    SurfaceFeature segment = edge(Vec3d(1.0, 2.0, 3.0), Vec3d(-2.0, 1.0, 0.0));
    segment.translate(affine);
    const auto [start, end] = segment.get_edge();
    check_vec(start, affine * Vec3d(1.0, 2.0, 3.0));
    check_vec(end, affine * Vec3d(-2.0, 1.0, 0.0));

    SurfaceFeature z_plane = plane(4, Vec3d::UnitZ(), Vec3d(1.0, 2.0, 3.0));
    z_plane.translate(affine);
    const auto [plane_index, plane_normal, plane_origin] = z_plane.get_plane();
    CHECK(plane_index == 4);
    check_vec(plane_origin, affine * Vec3d(1.0, 2.0, 3.0));
    check_vec(plane_normal, Vec3d::UnitZ());

    SurfaceFeature xy_circle = circle(Vec3d(1.0, 2.0, 3.0), Vec3d::UnitZ(), 2.0);
    xy_circle.translate(affine);
    const auto [circle_center, circle_radius, circle_normal] = xy_circle.get_circle();
    check_vec(circle_center, affine * Vec3d(1.0, 2.0, 3.0));
    // Circle features retain the implementation's radius representation under
    // anisotropic transforms; this is a policy check, not an exact transformed
    // Euclidean-circle claim.
    CHECK_THAT(circle_radius, WithinAbs(4.0, 1e-7));
    check_vec(circle_normal, Vec3d::UnitZ());
}

TEST_CASE("Point circle contracts distinguish radial, axial, axis, and center measurements", "[Measure][MeasureGeometryExtended]")
{
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0);
    const MeasurementResult radial = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 0.0)), ring);
    require_distance(radial.distance_strict, 2.0);
    require_point_circle_witness(radial.distance_strict, ring, Vec3d(5.0, 0.0, 0.0));
    check_vec(radial.distance_strict->to, Vec3d(3.0, 0.0, 0.0));

    const MeasurementResult axial = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 4.0)), ring);
    require_distance(axial.distance_strict, std::sqrt(20.0));
    require_point_circle_witness(axial.distance_strict, ring, Vec3d(5.0, 0.0, 4.0));

    const MeasurementResult axis_positive = get_measurement(SurfaceFeature(Vec3d(0.0, 0.0, 4.0)), ring);
    require_distance(axis_positive.distance_strict, 5.0);
    require_point_circle_witness(axis_positive.distance_strict, ring, Vec3d(0.0, 0.0, 4.0));
    const MeasurementResult axis_negative = get_measurement(SurfaceFeature(Vec3d(0.0, 0.0, -4.0)), ring);
    require_distance(axis_negative.distance_strict, 5.0);
    require_point_circle_witness(axis_negative.distance_strict, ring, Vec3d(0.0, 0.0, -4.0));

    const Vec3d axis_query(0.0, 0.0, 4.0);
    const MeasurementResult axis_reversed = get_measurement(ring, SurfaceFeature(axis_query));
    require_distance(axis_reversed.distance_strict, 5.0);
    require_reversed_point_circle_witness(axis_reversed.distance_strict, ring, axis_query);

    const MeasurementResult center_mode = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 4.0)), ring, true);
    require_distance(center_mode.distance_strict, std::sqrt(41.0));
    check_vec(center_mode.distance_strict->to, Vec3d::Zero());

    const MeasurementResult axis_center_mode = get_measurement(SurfaceFeature(Vec3d(0.0, 0.0, -4.0)), ring, true);
    require_distance(axis_center_mode.distance_strict, 4.0);
    check_vec(axis_center_mode.distance_strict->to, Vec3d::Zero());
    const Vec3d center_query(0.0, 0.0, -4.0);
    const MeasurementResult center_mode_reversed = get_measurement(ring, SurfaceFeature(center_query), true);
    require_distance(center_mode_reversed.distance_strict, 4.0);
    check_vec(center_mode_reversed.distance_strict->from, Vec3d::Zero());
    check_vec(center_mode_reversed.distance_strict->to, center_query);
}

TEST_CASE("Pair matrix covers point, edge, circle, and plane boundary branches", "[Measure][MeasureGeometryExtended]")
{
    const SurfaceFeature z_plane = plane(0, Vec3d::UnitZ(), Vec3d::Zero(),
                                         std::make_shared<std::vector<SurfaceFeature>>());
    const MeasurementResult point_plane = get_measurement(SurfaceFeature(Vec3d(1.0, 2.0, 4.0)), z_plane);
    require_distance(point_plane.distance_infinite, 4.0);
    check_vec(point_plane.distance_infinite->to, Vec3d(1.0, 2.0, 0.0));

    const SurfaceFeature parallel_a = edge(Vec3d::Zero(), Vec3d(4.0, 0.0, 0.0));
    const SurfaceFeature parallel_b = edge(Vec3d(0.0, 2.0, 3.0), Vec3d(4.0, 2.0, 3.0));
    const MeasurementResult parallel_edges = get_measurement(parallel_a, parallel_b);
    require_distance(parallel_edges.distance_infinite, std::sqrt(13.0));
    REQUIRE(parallel_edges.angle.has_value());
    CHECK_THAT(parallel_edges.angle->angle, WithinAbs(0.0, 1e-8));

    const SurfaceFeature skew_a = edge(Vec3d::Zero(), Vec3d(4.0, 0.0, 0.0));
    const SurfaceFeature skew_b = edge(Vec3d(2.0, 0.0, 2.0), Vec3d(2.0, 4.0, 2.0));
    const MeasurementResult skew_edges = get_measurement(skew_a, skew_b);
    require_distance(skew_edges.distance_infinite, 2.0);
    REQUIRE(skew_edges.angle.has_value());
    CHECK_THAT(skew_edges.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    CHECK_FALSE(skew_edges.angle->coplanar);

    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const SurfaceFeature secant = edge(Vec3d(-5.0, 2.0, 0.0), Vec3d(5.0, 2.0, 0.0));
    const MeasurementResult edge_ring = get_measurement(secant, ring);
    require_distance(edge_ring.distance_infinite, 0.0);
    const MeasurementResult edge_ring_center_mode = get_measurement(secant, ring, true);
    REQUIRE(edge_ring_center_mode.distance_infinite.has_value());
    CHECK_THAT(edge_ring_center_mode.distance_infinite->dist, WithinAbs(2.0, 1e-8));

    const SurfaceFeature in_plane = edge(Vec3d(-2.0, 1.0, 0.0), Vec3d(3.0, 1.0, 0.0));
    const MeasurementResult in_plane_result = get_measurement(in_plane, z_plane);
    REQUIRE(in_plane_result.angle.has_value());
    CHECK_THAT(in_plane_result.angle->angle, WithinAbs(0.0, 1e-8));

    const auto boundary = std::make_shared<std::vector<SurfaceFeature>>();
    boundary->emplace_back(edge(Vec3d(-10.0, 0.0, 0.0), Vec3d(10.0, 0.0, 0.0)));
    const SurfaceFeature bounded_plane = plane(1, Vec3d::UnitZ(), Vec3d::Zero(), boundary);
    const MeasurementResult bounded_ring = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0), bounded_plane);
    REQUIRE(bounded_ring.distance_infinite.has_value());
    CHECK(std::isfinite(bounded_ring.distance_infinite->dist));

    const MeasurementResult coplanar_ring = get_measurement(ring, z_plane);
    require_distance(coplanar_ring.distance_strict, 0.0);
}

TEST_CASE("Parallel circles return valid closest witnesses across geometric regimes", "[Measure][MeasureGeometryExtended]")
{
    const auto check_pair = [](const SurfaceFeature &a, const SurfaceFeature &b, double expected) {
        const MeasurementResult result = get_measurement(a, b);
        require_distance(result.distance_infinite, expected);
        require_circle_pair_witness(result.distance_infinite, a, b);
        // get_measurement canonically orders feature types, then restores the
        // caller's endpoint order. This also checks symmetry of the witness.
        const MeasurementResult reversed = get_measurement(b, a);
        require_distance(reversed.distance_infinite, expected);
        require_circle_pair_witness(reversed.distance_infinite, b, a);
    };

    const SurfaceFeature radius2 = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const SurfaceFeature radius1 = circle(Vec3d(5.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0);
    check_pair(radius2, radius1, 2.0); // externally disjoint
    check_pair(radius2, circle(Vec3d(5.0, 0.0, 3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0));
    check_pair(radius2, circle(Vec3d(5.0, 0.0, -3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0));
    check_pair(radius2, circle(Vec3d(3.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0), 0.0); // external tangent

    check_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0),
               circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0), 1.0); // nested
    check_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0),
               circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 3.0), 1.0); // reverse nested
    check_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0),
               circle(Vec3d(0.0, 0.0, 2.0), Vec3d::UnitZ(), 1.0), std::sqrt(8.0)); // concentric axial
    check_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0),
               circle(Vec3d(0.0, 0.0, -2.0), Vec3d::UnitZ(), 2.0), 2.0); // equal concentric axial
    check_pair(radius2, circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 2.0), 0.0); // coplanar intersection
    check_pair(radius2, circle(Vec3d(1.0, 0.0, 2.0), Vec3d::UnitZ(), 2.0), 2.0); // axial intersection projection
}

TEST_CASE("Parallel circle witnesses survive rigid transforms", "[Measure][MeasureGeometryExtended]")
{
    SurfaceFeature first = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    SurfaceFeature second = circle(Vec3d(5.0, 0.0, 3.0), Vec3d::UnitZ(), 1.0);
    Transform3d rigid = Eigen::Translation3d(7.0, -4.0, 2.0) *
                        Eigen::AngleAxisd(0.4, Vec3d(1.0, 2.0, 3.0).normalized());
    first.translate(rigid);
    second.translate(rigid);
    const MeasurementResult result = get_measurement(first, second);
    require_distance(result.distance_infinite, std::sqrt(13.0));
    require_circle_pair_witness(result.distance_infinite, first, second);
}

TEST_CASE("Nonparallel circle solver returns independent geometric witnesses", "[Measure][MeasureGeometryExtended]")
{
    const SurfaceFeature horizontal = circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0);
    const SurfaceFeature perpendicular = circle(Vec3d::Zero(), Vec3d::UnitY(), 1.0);
    const MeasurementResult intersecting = get_measurement(horizontal, perpendicular);
    require_distance(intersecting.distance_infinite, 0.0, 1e-6);
    CHECK_THAT(intersecting.distance_infinite->from.norm(), WithinAbs(1.0, 1e-6));
    CHECK_THAT(intersecting.distance_infinite->to.norm(), WithinAbs(1.0, 1e-6));
    CHECK_THAT(intersecting.distance_infinite->from.z(), WithinAbs(0.0, 1e-6));
    CHECK_THAT(intersecting.distance_infinite->to.y(), WithinAbs(0.0, 1e-6));

    const SurfaceFeature offset = circle(Vec3d(0.0, 0.0, 3.0), Vec3d::UnitY(), 1.0);
    const MeasurementResult separated = get_measurement(horizontal, offset);
    REQUIRE(separated.distance_infinite.has_value());
    CHECK_THAT(separated.distance_infinite->dist, WithinAbs(std::sqrt(11.0 - 2.0 * std::sqrt(10.0)), 1e-6));
    CHECK_THAT((separated.distance_infinite->to - separated.distance_infinite->from).norm(),
               WithinAbs(separated.distance_infinite->dist, 1e-6));
    CHECK_THAT(separated.distance_infinite->from.norm(), WithinAbs(1.0, 1e-6));
    CHECK_THAT(separated.distance_infinite->from.z(), WithinAbs(0.0, 1e-6));
    CHECK_THAT((separated.distance_infinite->to - Vec3d(0.0, 0.0, 3.0)).norm(), WithinAbs(1.0, 1e-6));
    CHECK_THAT((separated.distance_infinite->to - Vec3d(0.0, 0.0, 3.0)).dot(Vec3d::UnitY()),
               WithinAbs(0.0, 1e-6));
}

TEST_CASE("Reversed edge directions preserve angle and endpoint witnesses", "[Measure][MeasureGeometryExtended]")
{
    const SurfaceFeature first = edge(Vec3d(4.0, 0.0, 0.0), Vec3d::Zero());
    const SurfaceFeature second = edge(Vec3d(2.0, 3.0, 2.0), Vec3d(2.0, -1.0, 2.0));
    const MeasurementResult result = get_measurement(first, second);
    REQUIRE(result.distance_infinite.has_value());
    CHECK(std::isfinite(result.distance_infinite->dist));
    REQUIRE(result.angle.has_value());
    CHECK_THAT(result.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    CHECK(std::isfinite(result.angle->radius));
}

TEST_CASE("Measuring feature selection returns endpoint points and circular probes", "[Measure][MeasureGeometryExtended]")
{
    const indexed_triangle_set cube_mesh = its_make_cube(20.0, 20.0, 20.0);
    Measuring cube(cube_mesh);
    const auto endpoint = cube.get_feature(0, Vec3d(0.0, 0.0, 0.0), Transform3d::Identity(), false);
    REQUIRE(endpoint.has_value());
    CHECK(endpoint->get_type() == SurfaceFeatureType::Point);

    const indexed_triangle_set cylinder_mesh = its_make_cylinder(5.0, 10.0, PI / 16.0);
    Measuring cylinder(cylinder_mesh);
    int circle_plane = -1;
    for (int id = 0; id < cylinder.get_num_of_planes(); ++id) {
        const auto &features = cylinder.get_plane_features(static_cast<unsigned int>(id));
        for (size_t i = 0; i + 1 < features.size(); ++i)
            if (features[i].get_type() == SurfaceFeatureType::Circle)
                circle_plane = id;
    }
    REQUIRE(circle_plane >= 0);
    const auto &faces = cylinder.get_plane_triangle_indices(circle_plane);
    REQUIRE_FALSE(faces.empty());
    const auto circular_pick = cylinder.get_feature(faces.front(), Vec3d(0.0, 5.0, 10.0),
                                                    Transform3d::Identity(), false);
    REQUIRE(circular_pick.has_value());
    CHECK(circular_pick->get_type() == SurfaceFeatureType::Circle);
}

TEST_CASE("Edge and plane branches return finite witnesses for boundary consumers", "[Measure][MeasureGeometryExtended]")
{
    const auto boundary = std::make_shared<std::vector<SurfaceFeature>>();
    boundary->emplace_back(edge(Vec3d(2.0, -10.0, 0.0), Vec3d(2.0, 10.0, 0.0)));
    boundary->emplace_back(edge(Vec3d(-10.0, 3.0, 0.0), Vec3d(10.0, 3.0, 0.0)));
    const SurfaceFeature floor = plane(7, Vec3d::UnitZ(), Vec3d::Zero(), boundary);

    const SurfaceFeature diagonal = edge(Vec3d(0.0, 0.0, 3.0), Vec3d(4.0, 0.0, 5.0));
    const MeasurementResult diagonal_floor = get_measurement(diagonal, floor);
    REQUIRE(diagonal_floor.angle.has_value());
    CHECK(std::isfinite(diagonal_floor.angle->angle));
    REQUIRE(diagonal_floor.distance_infinite.has_value());
    CHECK_THAT(diagonal_floor.distance_infinite->dist, WithinAbs(std::sqrt(13.0), 1e-7));
    CHECK_THAT((diagonal_floor.distance_infinite->to - diagonal_floor.distance_infinite->from).norm(),
               WithinAbs(diagonal_floor.distance_infinite->dist, 1e-7));

    const SurfaceFeature vertical = edge(Vec3d(2.0, 3.0, 2.0), Vec3d(2.0, 3.0, 6.0));
    const MeasurementResult vertical_floor = get_measurement(vertical, floor);
    require_distance(vertical_floor.distance_infinite, 2.0);
    REQUIRE(vertical_floor.angle.has_value());
    CHECK_THAT(vertical_floor.angle->angle, WithinAbs(0.5 * PI, 1e-8));

    const SurfaceFeature raised_ring = circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0);
    const auto no_boundaries = std::make_shared<std::vector<SurfaceFeature>>();
    const MeasurementResult ring_floor = get_measurement(
        raised_ring, plane(8, Vec3d::UnitZ(), Vec3d::Zero(), no_boundaries));
    require_distance(ring_floor.distance_infinite, 5.0);
}

TEST_CASE("Cylinder extraction identifies circular end faces and stable plane ownership", "[Measure][MeasureGeometryExtended]")
{
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(5.0, 10.0, PI / 16.0);
    Measuring measuring(cylinder_mesh);
    REQUIRE(measuring.get_num_of_planes() > 20);

    std::set<int> all_faces;
    size_t circular_features = 0;
    size_t plane_features = 0;
    int circle_plane = -1;
    for (int plane_id = 0; plane_id < measuring.get_num_of_planes(); ++plane_id) {
        const auto &facets = measuring.get_plane_triangle_indices(plane_id);
        REQUIRE_FALSE(facets.empty());
        all_faces.insert(facets.begin(), facets.end());
        const auto &features = measuring.get_plane_features(static_cast<unsigned int>(plane_id));
        REQUIRE_FALSE(features.empty());
        REQUIRE(features.back().get_type() == SurfaceFeatureType::Plane);
        ++plane_features;
        for (size_t i = 0; i + 1 < features.size(); ++i) {
            if (features[i].get_type() == SurfaceFeatureType::Circle) {
                const auto [center, radius, normal] = features[i].get_circle();
                CHECK_THAT(center.x(), WithinAbs(0.0, 0.15));
                CHECK_THAT(center.y(), WithinAbs(0.0, 0.15));
                CHECK_THAT(radius, WithinAbs(5.0, 0.15));
                CHECK_THAT(std::abs(normal.z()), WithinAbs(1.0, 1e-7));
                ++circular_features;
                circle_plane = plane_id;
            }
        }
    }
    CHECK(all_faces.size() == cylinder_mesh.indices.size());
    CHECK(plane_features == static_cast<size_t>(measuring.get_num_of_planes()));
    CHECK(circular_features == 2);
    REQUIRE(circle_plane >= 0);

    const auto &circle_plane_faces = measuring.get_plane_triangle_indices(circle_plane);
    const auto feature = measuring.get_feature(circle_plane_faces.front(), Vec3d(5.0, 0.0, 10.0),
                                               Transform3d::Identity(), false);
    REQUIRE(feature.has_value());
    const bool circle_or_plane = feature->get_type() == SurfaceFeatureType::Circle ||
                                 feature->get_type() == SurfaceFeatureType::Plane;
    CHECK(circle_or_plane);
}

TEST_CASE("Cylinder feature selection honors endpoint and world transforms", "[Measure][MeasureGeometryExtended]")
{
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(4.0, 6.0, PI / 12.0);
    Measuring measuring(cylinder_mesh);
    const Transform3d world = Eigen::Translation3d(11.0, -3.0, 2.0) *
                              Eigen::AngleAxisd(0.25, Vec3d::UnitY());
    const auto &facets = measuring.get_plane_triangle_indices(0);
    REQUIRE_FALSE(facets.empty());
    const Vec3d local_point = Vec3d(0.0, 0.0, 0.0);
    const auto picked_plane = measuring.get_feature(facets.front(), local_point, world, true);
    REQUIRE(picked_plane.has_value());
    REQUIRE(picked_plane->get_type() == SurfaceFeatureType::Plane);
    const auto [index, normal, origin] = picked_plane->get_plane();
    CHECK(index == 0);
    CHECK_THAT(normal.norm(), WithinAbs(1.0, 1e-7));
    CHECK_THAT((origin - world * local_point).norm(), WithinAbs(0.0, 1e-7));
}

namespace skew_oracle {

struct CircleInput {
    Vec3d center;
    Vec3d normal;
    double radius;

    SurfaceFeature feature() const
    {
        return SurfaceFeature(SurfaceFeatureType::Circle, center, normal, std::nullopt, radius);
    }
};

double sampled_distance(const CircleInput &a, const CircleInput &b)
{
    const Vec3d u = b.normal.unitOrthogonal();
    const Vec3d v = b.normal.cross(u);
    const auto square_distance = [&](double angle) {
        const Vec3d candidate = b.center + b.radius * (u * std::cos(angle) + v * std::sin(angle));
        const Vec3d delta = candidate - a.center;
        const double height = delta.dot(a.normal);
        const double radial = (delta - height * a.normal).norm() - a.radius;
        return height * height + radial * radial;
    };
    constexpr int samples = 512;
    constexpr double tau = 6.2831853071795864769;
    constexpr double step = tau / samples;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
        const double angle = i * step;
        const double current = square_distance(angle);
        best = std::min(best, current);
        if (current > square_distance(angle - step) || current > square_distance(angle + step))
            continue;
        double lo = angle - step;
        double hi = angle + step;
        for (int iteration = 0; iteration < 80; ++iteration) {
            const double left = lo + (hi - lo) / 3.;
            const double right = hi - (hi - lo) / 3.;
            if (square_distance(left) < square_distance(right))
                hi = right;
            else
                lo = left;
        }
        best = std::min(best, square_distance((lo + hi) / 2.));
    }
    return std::sqrt(best);
}

void check_witness(const Vec3d &point, const CircleInput &circle_input)
{
    CHECK_THAT((point - circle_input.center).dot(circle_input.normal), WithinAbs(0., 1e-6));
    CHECK_THAT((point - circle_input.center).norm(), WithinAbs(circle_input.radius, 1e-6));
}

void check_pair(const CircleInput &a, const CircleInput &b)
{
    const auto result = get_measurement(a.feature(), b.feature());
    REQUIRE(result.distance_infinite.has_value());
    const auto &distance = *result.distance_infinite;
    CHECK(std::isfinite(distance.dist));
    CHECK_THAT(distance.dist, WithinAbs(sampled_distance(a, b), 1e-6));
    CHECK_THAT((distance.to - distance.from).norm(), WithinAbs(distance.dist, 1e-6));
    check_witness(distance.from, a);
    check_witness(distance.to, b);
}

} // namespace skew_oracle

TEST_CASE("Skew circle solver agrees with independent angular minimization", "[Measure][MeasureGeometryExtended]")
{
    const skew_oracle::CircleInput cases[] = {
        {Vec3d(.2, -.4, .6), Vec3d::UnitZ(), 1.3},
        {Vec3d(2.4, .7, .3), Vec3d(std::sin(.56), 0., std::cos(.56)), 1.01},
        {Vec3d(.82, 1.27, 1.37), Vec3d(std::sin(1.08), 0., std::cos(1.08)), 1.51},
    };
    for (size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        skew_oracle::check_pair(cases[i], cases[(i + 1) % 3]);
        skew_oracle::check_pair(cases[(i + 1) % 3], cases[i]);
    }
}

TEST_CASE("Parallel circle analytic sweep preserves axial and radial gaps", "[Measure][MeasureGeometryExtended]")
{
    struct Case {
        double radius0;
        double radius1;
        double projected_separation;
        double axial_separation;
        bool opposed_normal;
    };
    const Case cases[] = {
        {.5, 1.0, 0.0, 1.25, false}, // concentric nested
        {3.0, 1.0, .25, -2.0, false}, // nested
        {3.0, 1.0, 2.0, 0.0, false}, // internal tangent
        {3.0, 1.0, 2.5, .75, false}, // projected intersection
        {3.0, 1.0, 4.0, -1.5, false}, // external tangent
        {3.0, 1.0, 5.0, 2.0, false}, // disjoint
        {.5, 3.0, 1.5, -.5, false}, // reverse nested
        {.5, .5, 0.0, -1.0, false}, // equal concentric
        {.5, 1.0, 1.2, .3, true}, // opposed normals
        {.5, .5, 1e-8, 0.0, false}, // near-concentric equal radii
        {.5, 1.0, 1e-6, 1.25, false}, // near-concentric nested with axial offset
        {1.0, .5, 1e-4, -.3, true}, // near-concentric opposed normals
    };
    const Transform3d rigid = Eigen::Translation3d(11.0, -7.0, 4.0) *
                               Eigen::AngleAxisd(.37, Vec3d(1.0, 2.0, 3.0).normalized());
    for (const Case &test : cases) {
        CAPTURE(test.radius0, test.radius1, test.projected_separation, test.axial_separation, test.opposed_normal);
        const Vec3d normal0 = Vec3d::UnitZ();
        const Vec3d normal1 = test.opposed_normal ? Vec3d(0.0, 0.0, -1.0) : Vec3d::UnitZ();
        SurfaceFeature first = circle(Vec3d(0.0, 0.0, 0.0), normal0, test.radius0);
        SurfaceFeature second = circle(Vec3d(test.projected_separation, 0.0, test.axial_separation),
                                       normal1, test.radius1);
        first.translate(rigid);
        second.translate(rigid);
        const double radial_gap = std::max({test.projected_separation - test.radius0 - test.radius1,
                                             std::abs(test.radius0 - test.radius1) - test.projected_separation,
                                             0.0});
        const double expected = std::hypot(radial_gap, std::abs(test.axial_separation));
        const MeasurementResult result = get_measurement(first, second);
        require_distance(result.distance_infinite, expected, 1e-6);
        require_circle_pair_witness(result.distance_infinite, first, second, 1e-6);
    }
}

} // namespace measure_geometry_extended

namespace measure_skew {

using namespace Slic3r;
using namespace Slic3r::Measure;
using Catch::Matchers::WithinAbs;

namespace {
constexpr double tau = 6.2831853071795864769;
struct CircleInput {
    Vec3d center;
    Vec3d normal;
    double radius;
    SurfaceFeature feature() const
    {
        return SurfaceFeature(SurfaceFeatureType::Circle, center, normal, std::nullopt, radius);
    }
};

// Independent numerical oracle: minimize point-to-circle distance over the
// second circle's angle. This uses neither production polynomial coefficients
// nor its degree-eight root solver.
double sampled_distance(const CircleInput &a, const CircleInput &b)
{
    const Vec3d u = b.normal.unitOrthogonal();
    const Vec3d v = b.normal.cross(u);
    const auto square_distance = [&](double angle) {
        const Vec3d delta = b.center + b.radius * (u * std::cos(angle) + v * std::sin(angle)) - a.center;
        const double height = delta.dot(a.normal);
        const double radial = (delta - height * a.normal).norm() - a.radius;
        return height * height + radial * radial;
    };
    constexpr int samples = 512;
    constexpr double step = tau / samples;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
        const double angle = i * step;
        const double current = square_distance(angle);
        best = std::min(best, current);
        if (current > square_distance(angle - step) || current > square_distance(angle + step))
            continue;
        double lo = angle - step;
        double hi = angle + step;
        // Refine each sampled local minimum, rather than assuming one minimum.
        for (int iteration = 0; iteration < 80; ++iteration) {
            const double left = lo + (hi - lo) / 3.;
            const double right = hi - (hi - lo) / 3.;
            if (square_distance(left) < square_distance(right))
                hi = right;
            else
                lo = left;
        }
        best = std::min(best, square_distance((lo + hi) / 2.));
    }
    return std::sqrt(best);
}

void check_witness(const Vec3d &point, const CircleInput &circle)
{
    CHECK_THAT((point - circle.center).dot(circle.normal), WithinAbs(0., 1e-6));
    CHECK_THAT((point - circle.center).norm(), WithinAbs(circle.radius, 1e-6));
}

void check_pair(const CircleInput &a, const CircleInput &b)
{
    const auto result = get_measurement(a.feature(), b.feature());
    REQUIRE(result.distance_infinite.has_value());
    const auto &distance = *result.distance_infinite;
    CHECK(std::isfinite(distance.dist));
    CHECK_THAT(distance.dist, WithinAbs(sampled_distance(a, b), 1e-6));
    CHECK_THAT((distance.to - distance.from).norm(), WithinAbs(distance.dist, 1e-6));
    check_witness(distance.from, a);
    check_witness(distance.to, b);
}
}

TEST_CASE("Skew circle distances agree with angular minimization and geometric witnesses", "[Measure][MeasureSkew]")
{
    const int index = GENERATE(0, 1, 2, 3, 4, 5, 6, 7);
    const double angle = .17 + .13 * index;
    const CircleInput a{Vec3d(.2, -.4, .6), Vec3d::UnitZ(), 1.3 + .1 * index};
    const CircleInput b{Vec3d(2.4 - .23 * index, .7 + .19 * index, .3 + .11 * index),
                        Vec3d(std::sin(angle), 0., std::cos(angle)), .8 + .07 * index};
    CAPTURE(index);
    check_pair(a, b);
    check_pair(b, a);
}

TEST_CASE("Perpendicular circles retain analytic separation and concentric intersection", "[Measure][MeasureSkew]")
{
    const double separation = GENERATE(0., 1., 4.);
    const CircleInput a{Vec3d::Zero(), Vec3d::UnitZ(), 1.};
    const CircleInput b{Vec3d(separation, 0., 0.), Vec3d::UnitY(), 1.};
    check_pair(a, b);
    check_pair(b, a);
}

TEST_CASE("Skew circle distances scale under a common rigid placement", "[Measure][MeasureSkew]")
{
    const double scale = GENERATE(.2, 1., 5.);
    const Vec3d normal = Vec3d(1., 2., 3.).normalized();
    const CircleInput a{Vec3d(0., 0., 0.), Vec3d::UnitZ(), 2.};
    const CircleInput b{Vec3d(3., 2., 1.), normal, 1.};
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(.71, Vec3d(2., 3., 1.).normalized()).toRotationMatrix();
    const Vec3d translation(17., -9., 4.);
    const CircleInput moved_a{translation + rotation * (a.center * scale), rotation * a.normal, a.radius * scale};
    const CircleInput moved_b{translation + rotation * (b.center * scale), rotation * b.normal, b.radius * scale};
    check_pair(moved_a, moved_b);
    const auto original = get_measurement(a.feature(), b.feature());
    const auto moved = get_measurement(moved_a.feature(), moved_b.feature());
    REQUIRE(original.distance_infinite.has_value());
    REQUIRE(moved.distance_infinite.has_value());
    CHECK_THAT(moved.distance_infinite->dist, WithinAbs(scale * original.distance_infinite->dist, 1e-6));
}

} // namespace measure_skew
