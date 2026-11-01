#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Measure.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
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

// A distance whose witness points are as far apart as the reported distance.
void check_witnessed_distance(const std::optional<DistAndPoints> &distance, double expected, double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    CHECK_THAT(distance->dist, WithinAbs(expected, tolerance));
    CHECK_THAT((distance->to - distance->from).norm(), WithinAbs(distance->dist, tolerance));
}

// A witness point that lies on the circle.
void check_on_circle(const Vec3d &point, const SurfaceFeature &circle_feature, double tolerance = 1e-7)
{
    const auto [center, radius, normal] = circle_feature.get_circle();
    CHECK_THAT((point - center).dot(normal.normalized()), WithinAbs(0.0, tolerance));
    CHECK_THAT((point - center).norm(), WithinAbs(radius, tolerance));
}

// A witness pair of a circle-circle distance: one point on each circle.
void check_circle_pair_witness(const std::optional<DistAndPoints> &distance, const SurfaceFeature &from_circle,
                               const SurfaceFeature &to_circle, double tolerance = 1e-7)
{
    REQUIRE(distance.has_value());
    check_on_circle(distance->from, from_circle, tolerance);
    check_on_circle(distance->to, to_circle, tolerance);
}

// Independent oracle for the circle-circle distance: minimize the exact point-to-circle distance from
// b's points to a over b's angle. It uses neither the production polynomial nor its root finder.
double sampled_circle_distance(const SurfaceFeature &a, const SurfaceFeature &b)
{
    const auto [a_center, a_radius, a_normal_raw] = a.get_circle();
    const auto [b_center, b_radius, b_normal_raw] = b.get_circle();
    const Vec3d a_normal = a_normal_raw.normalized();
    const Vec3d u = b_normal_raw.unitOrthogonal();
    const Vec3d v = b_normal_raw.normalized().cross(u);
    const auto square_distance = [&](double angle) {
        const Vec3d delta = b_center + b_radius * (u * std::cos(angle) + v * std::sin(angle)) - a_center;
        const double height = delta.dot(a_normal);
        const double radial = (delta - height * a_normal).norm() - a_radius;
        return height * height + radial * radial;
    };
    constexpr int samples = 512;
    constexpr double step = 2.0 * PI / samples;
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < samples; ++i) {
        const double angle = i * step;
        const double current = square_distance(angle);
        best = std::min(best, current);
        if (current > square_distance(angle - step) || current > square_distance(angle + step))
            continue;
        // Refine every sampled local minimum by ternary search.
        double lo = angle - step;
        double hi = angle + step;
        for (int iteration = 0; iteration < 80; ++iteration) {
            const double left = lo + (hi - lo) / 3.0;
            const double right = hi - (hi - lo) / 3.0;
            if (square_distance(left) < square_distance(right))
                hi = right;
            else
                lo = left;
        }
        best = std::min(best, square_distance(0.5 * (lo + hi)));
    }
    return std::sqrt(best);
}

// The 1e-6 tolerance of the solver checks is loose on purpose: measured errors are at most 2e-12 on
// arm64, and roundoff differences between platforms (e.g. FMA contraction on x86) can move a double
// root of the degree-8 polynomial by about sqrt(machine epsilon), 1e-8, which stays well inside it.
void check_against_oracle(const SurfaceFeature &a, const SurfaceFeature &b)
{
    const MeasurementResult result = get_measurement(a, b);
    check_witnessed_distance(result.distance_infinite, sampled_circle_distance(a, b), 1e-6);
    check_circle_pair_witness(result.distance_infinite, a, b, 1e-6);
}

// Checks a parallel-circle distance in both argument orders: swapping the circles swaps the witnesses.
void check_parallel_pair(const SurfaceFeature &a, const SurfaceFeature &b, double expected)
{
    const MeasurementResult result = get_measurement(a, b);
    check_witnessed_distance(result.distance_infinite, expected);
    check_circle_pair_witness(result.distance_infinite, a, b);
    const MeasurementResult reversed = get_measurement(b, a);
    check_witnessed_distance(reversed.distance_infinite, expected);
    check_circle_pair_witness(reversed.distance_infinite, b, a);
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
    // The rotation turns X into Y, so the normals below must change for the checks to pass.
    SurfaceFeature transformed_edge = edge(Vec3d::Zero(), Vec3d(1.0, 0.0, 0.0), edge_extra);
    transformed_edge.translate(rigid);
    const auto [rigid_edge_start, rigid_edge_end] = transformed_edge.get_edge();
    check_vec(rigid_edge_start, rigid * Vec3d::Zero());
    check_vec(rigid_edge_end, rigid * Vec3d(1.0, 0.0, 0.0));
    REQUIRE(transformed_edge.get_extra_point().has_value());
    check_vec(*transformed_edge.get_extra_point(), rigid * edge_extra);
    SurfaceFeature transformed_plane_feature = plane(4, Vec3d::UnitX(), Vec3d(1.0, 2.0, 3.0));
    transformed_plane_feature.translate(rigid);
    const auto [transformed_plane_index, transformed_plane_normal, transformed_plane_origin] = transformed_plane_feature.get_plane();
    CHECK(transformed_plane_index == 4);
    check_vec(transformed_plane_normal, Vec3d::UnitY());
    check_vec(transformed_plane_origin, rigid * Vec3d(1.0, 2.0, 3.0));
    SurfaceFeature transformed_circle_feature = circle(Vec3d(1.0, 2.0, 3.0), Vec3d::UnitX(), 2.5);
    transformed_circle_feature.translate(rigid);
    const auto [transformed_center, transformed_radius, transformed_normal] = transformed_circle_feature.get_circle();
    check_vec(transformed_center, rigid * Vec3d(1.0, 2.0, 3.0));
    check_vec(transformed_normal, Vec3d::UnitY());
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

    // Edge/plane (edge not perpendicular to the normal) and non-coplanar circle/plane pairs measure
    // to the boundary edges in world_plane_features: here a 2 x 2 square around the origin.
    const auto plane_features = std::make_shared<std::vector<SurfaceFeature>>(std::vector<SurfaceFeature>{
        edge(Vec3d(-1.0, -1.0, 0.0), Vec3d(1.0, -1.0, 0.0)), edge(Vec3d(1.0, -1.0, 0.0), Vec3d(1.0, 1.0, 0.0)),
        edge(Vec3d(1.0, 1.0, 0.0), Vec3d(-1.0, 1.0, 0.0)), edge(Vec3d(-1.0, 1.0, 0.0), Vec3d(-1.0, -1.0, 0.0))});
    const SurfaceFeature bounded_plane = plane(3, Vec3d::UnitZ(), Vec3d::Zero(), plane_features);
    const MeasurementResult vertical_edge = get_measurement(edge(Vec3d(1.0, 1.0, -1.0), Vec3d(1.0, 1.0, 1.0)), bounded_plane);
    REQUIRE(vertical_edge.angle.has_value());
    CHECK_THAT(vertical_edge.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    // The edge passes through the corner (1, 1, 0) of the face.
    check_distance(vertical_edge.distance_infinite, 0.0);
    const MeasurementResult elevated_circle = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0), bounded_plane);
    // The r = 2 circle lies outside the face, so the closest face point is a corner, not the centre below.
    check_distance(elevated_circle.distance_infinite, std::sqrt(sqr(2.0 - std::sqrt(2.0)) + 25.0));
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

TEST_CASE("Plane angle is the dihedral angle when both origins lie on the intersection line", "[Measure][MeasureContracts]")
{
    // The planes meet at 45 degrees along the Y axis, and both origins sit on that line, so
    // there is no offset witness point to orient the angle: either 45 or 135 degrees is valid.
    const MeasurementResult result = get_measurement(
        plane(5, Vec3d::UnitZ(), Vec3d::Zero()),
        plane(6, Vec3d(1.0, 0.0, 1.0).normalized(), Vec3d::Zero()));
    REQUIRE(result.angle.has_value());
    REQUIRE(std::isfinite(result.angle->angle));
    CHECK_THAT(std::min(result.angle->angle, PI - result.angle->angle), WithinAbs(0.25 * PI, 1e-8));
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
    world.linear() = Eigen::AngleAxisd(0.5 * PI, Vec3d::UnitX()).toRotationMatrix();
    world.translation() = Vec3d(3.0, 4.0, 5.0);
    const auto transformed_plane = measuring.get_feature(0, Vec3d(10.0, 10.0, 0.0), world, true);
    REQUIRE(transformed_plane.has_value());
    const auto [transformed_index, transformed_normal, transformed_origin] = transformed_plane->get_plane();
    CHECK(transformed_index == 0);
    check_vec(transformed_normal, world.linear() * plane_normal);
    check_vec(transformed_origin, world * Vec3d(10.0, 10.0, 0.0));

    const auto edge_pick = measuring.get_feature(0, Vec3d(10.0, 0.0, 0.0), Transform3d::Identity(), false);
    REQUIRE(edge_pick.has_value());
    CHECK(edge_pick->get_type() == SurfaceFeatureType::Edge);
    CHECK(measuring.get_feature(cube_mesh.indices.size(), Vec3d::Zero(), Transform3d::Identity(), false) == std::nullopt);
}

TEST_CASE("Plane projection handles oblique normals and the alternate in-plane basis", "[Measure][MeasureContracts]")
{
    const Vec3d origin(2.0, -3.0, 4.0);
    const Vec3d normal(1.0, 2.0, 3.0);
    const Vec3d point(5.0, 7.0, -1.0);
    Vec3d projected;
    REQUIRE(get_point_projection_to_plane(point, origin, normal, projected));
    CHECK_THAT((projected - origin).dot(normal), WithinAbs(0.0, 1e-8));
    CHECK_THAT((point - projected).cross(normal).norm(), WithinAbs(0.0, 1e-8));

    // A normal along X cannot use the default X probe direction, so the point comes from Y.
    const Vec3d x_aligned = get_one_point_in_plane(origin, Vec3d::UnitX());
    check_vec(x_aligned, origin + Vec3d::UnitY());
    const Vec3d oblique = get_one_point_in_plane(origin, normal);
    CHECK_THAT((oblique - origin).dot(normal), WithinAbs(0.0, 1e-8));
    CHECK((oblique - origin).norm() > 0.5);
}

TEST_CASE("Point circle distances cover radial, axial and center-mode measurements", "[Measure][MeasureContracts]")
{
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0);
    const MeasurementResult radial = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 0.0)), ring);
    check_witnessed_distance(radial.distance_strict, 2.0);
    check_vec(radial.distance_strict->from, Vec3d(5.0, 0.0, 0.0));
    check_vec(radial.distance_strict->to, Vec3d(3.0, 0.0, 0.0));

    // Off the axis and off the plane: radial gap 2, axial gap 4.
    const MeasurementResult axial = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 4.0)), ring);
    check_witnessed_distance(axial.distance_strict, std::sqrt(20.0));
    check_vec(axial.distance_strict->from, Vec3d(5.0, 0.0, 4.0));
    check_vec(axial.distance_strict->to, Vec3d(3.0, 0.0, 0.0));

    // With the circle first, the witnesses come back in the caller's order.
    const MeasurementResult reversed = get_measurement(ring, SurfaceFeature(Vec3d(5.0, 0.0, 4.0)));
    check_witnessed_distance(reversed.distance_strict, std::sqrt(20.0));
    check_vec(reversed.distance_strict->from, Vec3d(3.0, 0.0, 0.0));
    check_vec(reversed.distance_strict->to, Vec3d(5.0, 0.0, 4.0));

    // Center mode measures to the circle center.
    const MeasurementResult center_mode = get_measurement(SurfaceFeature(Vec3d(5.0, 0.0, 4.0)), ring, true);
    check_witnessed_distance(center_mode.distance_strict, std::sqrt(41.0));
    check_vec(center_mode.distance_strict->from, Vec3d(5.0, 0.0, 4.0));
    check_vec(center_mode.distance_strict->to, Vec3d::Zero());
}

TEST_CASE("Point on a circle's axis measures to the circle and to its center", "[Measure][MeasureContracts]")
{
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0);
    // Every point of the circle is sqrt(3^2 + 4^2) away from a point 4 above or below its center.
    for (const Vec3d &axis_query : {Vec3d(0.0, 0.0, 4.0), Vec3d(0.0, 0.0, -4.0)}) {
        const MeasurementResult on_axis = get_measurement(SurfaceFeature(axis_query), ring);
        check_witnessed_distance(on_axis.distance_strict, 5.0);
        check_vec(on_axis.distance_strict->from, axis_query);
        check_on_circle(on_axis.distance_strict->to, ring);
    }
    const MeasurementResult axis_reversed = get_measurement(ring, SurfaceFeature(Vec3d(0.0, 0.0, 4.0)));
    check_witnessed_distance(axis_reversed.distance_strict, 5.0);
    check_on_circle(axis_reversed.distance_strict->from, ring);
    check_vec(axis_reversed.distance_strict->to, Vec3d(0.0, 0.0, 4.0));

    const MeasurementResult center_mode = get_measurement(SurfaceFeature(Vec3d(0.0, 0.0, -4.0)), ring, true);
    check_witnessed_distance(center_mode.distance_strict, 4.0);
    check_vec(center_mode.distance_strict->from, Vec3d(0.0, 0.0, -4.0));
    check_vec(center_mode.distance_strict->to, Vec3d::Zero());
}

TEST_CASE("Pair matrix covers point, edge, circle, and plane branches", "[Measure][MeasureContracts]")
{
    const SurfaceFeature z_plane = plane(0, Vec3d::UnitZ(), Vec3d::Zero(), std::make_shared<std::vector<SurfaceFeature>>());
    const MeasurementResult point_plane = get_measurement(SurfaceFeature(Vec3d(1.0, 2.0, 4.0)), z_plane);
    check_witnessed_distance(point_plane.distance_infinite, 4.0);
    check_vec(point_plane.distance_infinite->to, Vec3d(1.0, 2.0, 0.0));

    const MeasurementResult parallel_edges = get_measurement(edge(Vec3d::Zero(), Vec3d(4.0, 0.0, 0.0)),
                                                             edge(Vec3d(0.0, 2.0, 3.0), Vec3d(4.0, 2.0, 3.0)));
    check_witnessed_distance(parallel_edges.distance_infinite, std::sqrt(13.0));
    REQUIRE(parallel_edges.angle.has_value());
    CHECK_THAT(parallel_edges.angle->angle, WithinAbs(0.0, 1e-8));

    // The closest points are an endpoint of the second edge and its foot on the first.
    const MeasurementResult skew_edges = get_measurement(edge(Vec3d::Zero(), Vec3d(4.0, 0.0, 0.0)),
                                                         edge(Vec3d(2.0, 0.0, 2.0), Vec3d(2.0, 4.0, 2.0)));
    check_witnessed_distance(skew_edges.distance_infinite, 2.0);
    REQUIRE(skew_edges.angle.has_value());
    CHECK_THAT(skew_edges.angle->angle, WithinAbs(0.5 * PI, 1e-8));
    CHECK_FALSE(skew_edges.angle->coplanar);

    // A tangent edge touches the circle at (0, 2, 0); center mode measures from there to the center.
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const SurfaceFeature tangent = edge(Vec3d(-5.0, 2.0, 0.0), Vec3d(5.0, 2.0, 0.0));
    const MeasurementResult edge_ring = get_measurement(tangent, ring);
    check_witnessed_distance(edge_ring.distance_infinite, 0.0);
    check_vec(edge_ring.distance_infinite->from, Vec3d(0.0, 2.0, 0.0));
    const MeasurementResult edge_ring_center_mode = get_measurement(tangent, ring, true);
    check_witnessed_distance(edge_ring_center_mode.distance_infinite, 2.0);
    check_vec(edge_ring_center_mode.distance_infinite->to, Vec3d::Zero());

    const MeasurementResult in_plane = get_measurement(edge(Vec3d(-2.0, 1.0, 0.0), Vec3d(3.0, 1.0, 0.0)), z_plane);
    REQUIRE(in_plane.angle.has_value());
    CHECK_THAT(in_plane.angle->angle, WithinAbs(0.0, 1e-8));
    check_witnessed_distance(in_plane.distance_infinite, 0.0);

    const MeasurementResult coplanar_ring = get_measurement(ring, z_plane);
    check_witnessed_distance(coplanar_ring.distance_strict, 0.0);
}

// Known bug: edge-circle distance only measures from the edge endpoints and from the foot of the circle
// center on the edge, so it misses where a secant crosses the circle away from those points; needs a fix PR
TEST_CASE("Edge distance to a circle is zero where a secant crosses it", "[Measure][MeasureContracts][!shouldfail]")
{
    // y = 1 cuts the radius-2 circle at x = +-sqrt(3); the center's foot (0, 1, 0) is 1 inside the circle.
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const MeasurementResult result = get_measurement(edge(Vec3d(-5.0, 1.0, 0.0), Vec3d(5.0, 1.0, 0.0)), ring);
    check_witnessed_distance(result.distance_infinite, 0.0);
    check_on_circle(result.distance_infinite->from, ring);
}

TEST_CASE("Edge angles measure the rays pointing away from the intersection", "[Measure][MeasureContracts]")
{
    // Both edges lie on rays from the origin at 45 degrees; the second pair is given end to start.
    for (const bool reversed : {false, true}) {
        const SurfaceFeature first = reversed ? edge(Vec3d(4.0, 0.0, 0.0), Vec3d(1.0, 0.0, 0.0))
                                              : edge(Vec3d(1.0, 0.0, 0.0), Vec3d(4.0, 0.0, 0.0));
        const SurfaceFeature second = reversed ? edge(Vec3d(3.0, 3.0, 0.0), Vec3d(1.0, 1.0, 0.0))
                                               : edge(Vec3d(1.0, 1.0, 0.0), Vec3d(3.0, 3.0, 0.0));
        const MeasurementResult result = get_measurement(first, second);
        REQUIRE(result.angle.has_value());
        CHECK_THAT(result.angle->angle, WithinAbs(0.25 * PI, 1e-8));
        check_vec(result.angle->center, Vec3d::Zero());
        // The returned edges start at the end nearer the intersection.
        check_vec(result.angle->e1.first, Vec3d(1.0, 0.0, 0.0));
        check_vec(result.angle->e1.second, Vec3d(4.0, 0.0, 0.0));
        check_vec(result.angle->e2.first, Vec3d(1.0, 1.0, 0.0));
        check_vec(result.angle->e2.second, Vec3d(3.0, 3.0, 0.0));
        CHECK(result.angle->coplanar);
    }
}

TEST_CASE("Edge distance finds interior closest points of skew segments", "[Measure][MeasureContracts]")
{
    // The segments cross over each other 2 apart at x = 2, away from all four endpoints.
    const MeasurementResult result = get_measurement(edge(Vec3d(4.0, 0.0, 0.0), Vec3d::Zero()),
                                                     edge(Vec3d(2.0, 3.0, 2.0), Vec3d(2.0, -1.0, 2.0)));
    check_witnessed_distance(result.distance_infinite, 2.0);
}

TEST_CASE("Edge distance finds the crossing of nearly parallel segments", "[Measure][MeasureContracts]")
{
    // About 0.01 rad apart, the segments cross at (100, 0, 0), far from all four endpoints.
    const MeasurementResult result = get_measurement(edge(Vec3d::Zero(), Vec3d(200.0, 0.0, 0.0)),
                                                     edge(Vec3d(0.0, -1.0, 0.0), Vec3d(200.0, 1.0, 0.0)));
    check_witnessed_distance(result.distance_infinite, 0.0);
    check_vec(result.distance_infinite->from, Vec3d(100.0, 0.0, 0.0), 1e-7);
    check_vec(result.distance_infinite->to, Vec3d(100.0, 0.0, 0.0), 1e-7);
}

TEST_CASE("Measuring feature selection returns endpoint points and circular probes", "[Measure][MeasureContracts]")
{
    const indexed_triangle_set cube_mesh = its_make_cube(20.0, 20.0, 20.0);
    Measuring cube(cube_mesh);
    // A pick at an edge endpoint returns that vertex as a point feature.
    const auto endpoint = cube.get_feature(0, Vec3d::Zero(), Transform3d::Identity(), false);
    REQUIRE(endpoint.has_value());
    REQUIRE(endpoint->get_type() == SurfaceFeatureType::Point);
    check_vec(endpoint->get_point(), Vec3d::Zero());

    // A pick on the rim of a cylinder cap returns the cap circle.
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(5.0, 10.0, PI / 16.0);
    Measuring cylinder(cylinder_mesh);
    int top_cap = -1;
    for (int id = 0; id < cylinder.get_num_of_planes(); ++id) {
        const auto [index, normal, origin] = cylinder.get_plane_features(static_cast<unsigned int>(id)).back().get_plane();
        if (normal.z() > 0.5)
            top_cap = id;
    }
    REQUIRE(top_cap >= 0);
    const auto rim_pick = cylinder.get_feature(cylinder.get_plane_triangle_indices(top_cap).front(), Vec3d(0.0, 5.0, 10.0),
                                               Transform3d::Identity(), false);
    REQUIRE(rim_pick.has_value());
    REQUIRE(rim_pick->get_type() == SurfaceFeatureType::Circle);
    const auto [center, radius, normal] = rim_pick->get_circle();
    check_vec(center, Vec3d(0.0, 0.0, 10.0), 1e-6);
    CHECK_THAT(radius, WithinAbs(5.0, 1e-6));
    CHECK_THAT(normal.z(), WithinAbs(1.0, 1e-7));
}

TEST_CASE("Edge and circle distances to a bounded plane use its boundary edges", "[Measure][MeasureContracts]")
{
    // A 2 x 2 face around the origin of the floor.
    const auto boundary = std::make_shared<std::vector<SurfaceFeature>>(std::vector<SurfaceFeature>{
        edge(Vec3d(-1.0, -1.0, 0.0), Vec3d(1.0, -1.0, 0.0)), edge(Vec3d(1.0, -1.0, 0.0), Vec3d(1.0, 1.0, 0.0)),
        edge(Vec3d(1.0, 1.0, 0.0), Vec3d(-1.0, 1.0, 0.0)), edge(Vec3d(-1.0, 1.0, 0.0), Vec3d(-1.0, -1.0, 0.0))});
    const SurfaceFeature floor = plane(7, Vec3d::UnitZ(), Vec3d::Zero(), boundary);

    // An edge rising at 45 degrees beside the face: both ends project outside it, and the closest pair is
    // its lower end (3, 0, 1) and the boundary point (1, 0, 0), sqrt(2^2 + 1^2) apart.
    const MeasurementResult diagonal_floor = get_measurement(edge(Vec3d(3.0, 0.0, 1.0), Vec3d(5.0, 0.0, 3.0)), floor);
    REQUIRE(diagonal_floor.angle.has_value());
    // The plane-side witness ray starts at the plane origin, which lies across the intersection (2, 0, 0)
    // from this edge, so the branch reports the supplement here; either is the 45 degree line-plane angle.
    CHECK_THAT(std::min(diagonal_floor.angle->angle, PI - diagonal_floor.angle->angle), WithinAbs(0.25 * PI, 1e-8));
    check_witnessed_distance(diagonal_floor.distance_infinite, std::sqrt(5.0));
    check_vec(diagonal_floor.distance_infinite->from, Vec3d(3.0, 0.0, 1.0));
    check_vec(diagonal_floor.distance_infinite->to, Vec3d(1.0, 0.0, 0.0));

    // A circle standing in the plane x = 4 beside the face, so every circle point projects outside it: the
    // closest pair is (1, 0, 0) and the lowest circle point (4, 0, 3), sqrt(3^2 + 3^2) apart.
    const SurfaceFeature standing = circle(Vec3d(4.0, 0.0, 5.0), Vec3d::UnitX(), 2.0);
    const MeasurementResult standing_floor = get_measurement(standing, floor);
    check_witnessed_distance(standing_floor.distance_infinite, std::sqrt(18.0));
    // The witness order is not checked: this branch returns them as (face point, circle point).
    const bool from_is_face = (standing_floor.distance_infinite->from - Vec3d(1.0, 0.0, 0.0)).norm() < 1e-7;
    check_vec(from_is_face ? standing_floor.distance_infinite->from : standing_floor.distance_infinite->to, Vec3d(1.0, 0.0, 0.0));
    check_on_circle(from_is_face ? standing_floor.distance_infinite->to : standing_floor.distance_infinite->from, standing);

    // Without boundary edges, a parallel circle 5 above the plane is 5 away. The witnesses are checked in
    // the tagged test below.
    const SurfaceFeature unbounded = plane(8, Vec3d::UnitZ(), Vec3d::Zero(), std::make_shared<std::vector<SurfaceFeature>>());
    const MeasurementResult ring_floor = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0), unbounded);
    REQUIRE(ring_floor.distance_infinite.has_value());
    CHECK_THAT(ring_floor.distance_infinite->dist, WithinAbs(5.0, 1e-7));
}

TEST_CASE("Circle distance to a boundary-less plane it does not lie in has a witness on the circle", "[Measure][MeasureContracts]")
{
    const SurfaceFeature ring = circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitZ(), 2.0);
    const SurfaceFeature unbounded = plane(8, Vec3d::UnitZ(), Vec3d::Zero(), std::make_shared<std::vector<SurfaceFeature>>());
    const MeasurementResult ring_floor = get_measurement(ring, unbounded);
    check_witnessed_distance(ring_floor.distance_infinite, 5.0);
    check_on_circle(ring_floor.distance_infinite->from, ring);
    // The plane witness is the circle witness dropped onto z = 0.
    check_vec(ring_floor.distance_infinite->to, Vec3d(ring_floor.distance_infinite->from.x(), ring_floor.distance_infinite->from.y(), 0.0));

    // Tilted 45 degrees about Y, the circle's lowest point (sqrt(2), 0, 5 - sqrt(2)) is nearest the plane.
    const Vec3d tilt = Vec3d(1.0, 0.0, 1.0).normalized();
    const SurfaceFeature tilted = circle(Vec3d(0.0, 0.0, 5.0), tilt, 2.0);
    const MeasurementResult tilted_floor = get_measurement(tilted, unbounded);
    check_witnessed_distance(tilted_floor.distance_infinite, 5.0 - std::sqrt(2.0));
    check_vec(tilted_floor.distance_infinite->from, Vec3d(std::sqrt(2.0), 0.0, 5.0 - std::sqrt(2.0)));
    // Center mode measures from the center instead.
    const MeasurementResult tilted_center_mode = get_measurement(tilted, unbounded, true);
    check_witnessed_distance(tilted_center_mode.distance_infinite, 5.0);
    check_vec(tilted_center_mode.distance_infinite->from, Vec3d(0.0, 0.0, 5.0));
    check_vec(tilted_center_mode.distance_infinite->to, Vec3d::Zero());

    // Lowered to z = 1, the tilted circle cuts the plane: distance 0 at a circle point on z = 0.
    const SurfaceFeature cutting = circle(Vec3d(0.0, 0.0, 1.0), tilt, 2.0);
    const MeasurementResult cutting_floor = get_measurement(cutting, unbounded);
    check_witnessed_distance(cutting_floor.distance_infinite, 0.0);
    check_on_circle(cutting_floor.distance_infinite->from, cutting);
    CHECK_THAT(cutting_floor.distance_infinite->from.z(), WithinAbs(0.0, 1e-7));

    // Centered on the plane, the tilted circle crosses it 90 degrees from its lowest direction.
    const SurfaceFeature centered = circle(Vec3d::Zero(), tilt, 2.0);
    const MeasurementResult centered_floor = get_measurement(centered, unbounded);
    check_witnessed_distance(centered_floor.distance_infinite, 0.0);
    check_on_circle(centered_floor.distance_infinite->from, centered);
    CHECK_THAT(centered_floor.distance_infinite->from.z(), WithinAbs(0.0, 1e-7));

    // Raised to z = sqrt(2), the tilted circle touches the plane at its lowest point (sqrt(2), 0, 0).
    const MeasurementResult tangent_floor = get_measurement(circle(Vec3d(0.0, 0.0, std::sqrt(2.0)), tilt, 2.0), unbounded);
    check_witnessed_distance(tangent_floor.distance_infinite, 0.0);
    check_vec(tangent_floor.distance_infinite->from, Vec3d(std::sqrt(2.0), 0.0, 0.0));

    // Standing upright 5 above the plane, the circle's lowest point (0, 0, 3) is nearest.
    const MeasurementResult upright_floor = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitX(), 2.0), unbounded);
    check_witnessed_distance(upright_floor.distance_infinite, 3.0);
    check_vec(upright_floor.distance_infinite->from, Vec3d(0.0, 0.0, 3.0));
}

// Known bug: for a circle lying in the plane the circle-plane branch returns the circle center and the plane
// origin as witnesses of distance 0, so the first is not on the circle and they can be far apart; needs a fix PR
TEST_CASE("Circle lying in a plane has witnesses at a common circle point", "[Measure][MeasureContracts][!shouldfail]")
{
    const SurfaceFeature ring = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    // The plane's origin is away from the circle; it has no boundary edges.
    const SurfaceFeature floor = plane(8, Vec3d::UnitZ(), Vec3d(5.0, 5.0, 0.0), std::make_shared<std::vector<SurfaceFeature>>());
    const MeasurementResult result = get_measurement(ring, floor);
    check_witnessed_distance(result.distance_strict, 0.0);
    check_on_circle(result.distance_strict->from, ring);
}

// Known bug: the edge-plane and circle-plane branches measure only to the face's boundary edges, so a
// feature above the face interior gets the distance to the boundary instead; needs a fix PR
TEST_CASE("Edge and circle distances to a bounded plane reach the face interior", "[Measure][MeasureContracts][!shouldfail]")
{
    const auto boundary = std::make_shared<std::vector<SurfaceFeature>>(std::vector<SurfaceFeature>{
        edge(Vec3d(-1.0, -1.0, 0.0), Vec3d(1.0, -1.0, 0.0)), edge(Vec3d(1.0, -1.0, 0.0), Vec3d(1.0, 1.0, 0.0)),
        edge(Vec3d(1.0, 1.0, 0.0), Vec3d(-1.0, 1.0, 0.0)), edge(Vec3d(-1.0, 1.0, 0.0), Vec3d(-1.0, -1.0, 0.0))});
    const SurfaceFeature floor = plane(7, Vec3d::UnitZ(), Vec3d::Zero(), boundary);

    // The lower end (0, 0, 3) of a rising edge lies right above the face center, 3 away.
    const MeasurementResult diagonal_floor = get_measurement(edge(Vec3d(0.0, 0.0, 3.0), Vec3d(4.0, 0.0, 5.0)), floor);
    check_witnessed_distance(diagonal_floor.distance_infinite, 3.0);

    // The lowest point (0, 0, 3) of a circle standing in x = 0 lies right above the face center, 3 away.
    const MeasurementResult standing_floor = get_measurement(circle(Vec3d(0.0, 0.0, 5.0), Vec3d::UnitX(), 2.0), floor);
    check_witnessed_distance(standing_floor.distance_infinite, 3.0);
}

TEST_CASE("Cylinder extraction finds two circular caps and gives each facet one plane", "[Measure][MeasureContracts]")
{
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(5.0, 10.0, PI / 16.0);
    Measuring measuring(cylinder_mesh);
    // Two caps plus one plane per side facet.
    REQUIRE(measuring.get_num_of_planes() > 20);

    std::map<int, std::set<int>> owners;
    size_t circular_features = 0;
    for (int plane_id = 0; plane_id < measuring.get_num_of_planes(); ++plane_id) {
        const auto &facets = measuring.get_plane_triangle_indices(plane_id);
        REQUIRE_FALSE(facets.empty());
        for (const int facet : facets)
            owners[facet].insert(plane_id);
        const auto &features = measuring.get_plane_features(static_cast<unsigned int>(plane_id));
        REQUIRE_FALSE(features.empty());
        REQUIRE(features.back().get_type() == SurfaceFeatureType::Plane);
        for (size_t i = 0; i + 1 < features.size(); ++i) {
            if (features[i].get_type() == SurfaceFeatureType::Circle) {
                const auto [center, radius, normal] = features[i].get_circle();
                CHECK_THAT(center.x(), WithinAbs(0.0, 1e-6));
                CHECK_THAT(center.y(), WithinAbs(0.0, 1e-6));
                CHECK_THAT(radius, WithinAbs(5.0, 1e-6));
                CHECK_THAT(std::abs(normal.z()), WithinAbs(1.0, 1e-7));
                ++circular_features;
            }
        }
    }
    // Every facet belongs to exactly one plane.
    CHECK(owners.size() == cylinder_mesh.indices.size());
    for (const auto &[facet, planes] : owners) {
        CAPTURE(facet);
        CHECK(planes.size() == 1);
    }
    CHECK(circular_features == 2);
}

TEST_CASE("Cylinder feature selection honors world transforms", "[Measure][MeasureContracts]")
{
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(4.0, 6.0, PI / 12.0);
    Measuring measuring(cylinder_mesh);
    // Use a side plane, whose normal is horizontal, rather than a cap.
    int side_plane = -1;
    for (int id = 0; id < measuring.get_num_of_planes() && side_plane < 0; ++id) {
        const auto [index, normal, origin] = measuring.get_plane_features(static_cast<unsigned int>(id)).back().get_plane();
        if (std::abs(normal.z()) < 0.1)
            side_plane = id;
    }
    REQUIRE(side_plane >= 0);
    const int facet = measuring.get_plane_triangle_indices(side_plane).front();
    const Vec3d pick = cylinder_mesh.vertices[cylinder_mesh.indices[facet][0]].cast<double>();
    const auto local_plane = measuring.get_feature(facet, pick, Transform3d::Identity(), true);
    REQUIRE(local_plane.has_value());
    const auto [local_index, local_normal, local_origin] = local_plane->get_plane();
    CHECK(local_index == side_plane);

    const Transform3d world = Eigen::Translation3d(11.0, -3.0, 2.0) * Eigen::AngleAxisd(0.25, Vec3d::UnitY()) *
                              Eigen::AngleAxisd(0.6, Vec3d::UnitX());
    const auto picked_plane = measuring.get_feature(facet, pick, world, true);
    REQUIRE(picked_plane.has_value());
    REQUIRE(picked_plane->get_type() == SurfaceFeatureType::Plane);
    const auto [index, normal, origin] = picked_plane->get_plane();
    CHECK(index == local_index);
    check_vec(normal, world.linear() * local_normal);
    check_vec(origin, world * local_origin);
}

TEST_CASE("Measuring lists each facet of a plane once", "[Measure][MeasureContracts]")
{
    // The cap fans of a cylinder put the seed's neighbours next to each other.
    const indexed_triangle_set cylinder_mesh = its_make_cylinder(5.0, 10.0, PI / 16.0);
    Measuring measuring(cylinder_mesh);
    size_t listed = 0;
    for (int plane_id = 0; plane_id < measuring.get_num_of_planes(); ++plane_id) {
        const auto &facets = measuring.get_plane_triangle_indices(plane_id);
        CAPTURE(plane_id);
        CHECK(std::set<int>(facets.begin(), facets.end()).size() == facets.size());
        listed += facets.size();
    }
    CHECK(listed == cylinder_mesh.indices.size());
}

TEST_CASE("Coplanar circles report disjoint, tangent and nested gaps", "[Measure][MeasureContracts]")
{
    const SurfaceFeature radius2 = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    check_parallel_pair(radius2, circle(Vec3d(5.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0), 2.0); // externally disjoint
    check_parallel_pair(radius2, circle(Vec3d(3.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0), 0.0); // external tangent
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0), circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 1.0),
                        1.0); // nested
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0), circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 3.0),
                        1.0); // reverse nested
}

TEST_CASE("Intersecting coplanar circles report zero distance at a common point", "[Measure][MeasureContracts]")
{
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0), circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 2.0), 0.0);
}

TEST_CASE("Parallel circles in different planes include the axial gap", "[Measure][MeasureContracts]")
{
    const SurfaceFeature radius2 = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    check_parallel_pair(radius2, circle(Vec3d(5.0, 0.0, 3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0));  // disjoint, above
    check_parallel_pair(radius2, circle(Vec3d(5.0, 0.0, -3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0)); // disjoint, below
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0), circle(Vec3d(0.0, 0.0, 2.0), Vec3d::UnitZ(), 1.0),
                        std::sqrt(8.0)); // concentric
    check_parallel_pair(radius2, circle(Vec3d(0.0, 0.0, -2.0), Vec3d::UnitZ(), 2.0), 2.0); // equal concentric
    check_parallel_pair(radius2, circle(Vec3d(1.0, 0.0, 2.0), Vec3d::UnitZ(), 2.0), 2.0);  // projections intersect
}

TEST_CASE("Parallel circle analytic sweep preserves axial and radial gaps", "[Measure][MeasureContracts]")
{
    struct Case {
        double radius0;
        double radius1;
        double projected_separation;
        double axial_separation;
        bool opposed_normal;
    };
    const Case cases[] = {
        {0.5, 1.0, 0.0, 1.25, false},   // concentric nested
        {3.0, 1.0, 0.25, -2.0, false},  // nested
        {3.0, 1.0, 2.0, 0.0, false},    // internal tangent
        {3.0, 1.0, 2.5, 0.75, false},   // projected intersection
        {3.0, 1.0, 4.0, -1.5, false},   // external tangent
        {3.0, 1.0, 5.0, 2.0, false},    // disjoint
        {0.5, 3.0, 1.5, -0.5, false},   // reverse nested
        {0.5, 0.5, 0.0, -1.0, false},   // equal concentric
        {0.5, 1.0, 1.2, 0.3, true},     // opposed normals
        {0.5, 0.5, 1e-8, 0.0, false},   // near-concentric equal radii
        {0.5, 1.0, 1e-6, 1.25, false},  // near-concentric nested with axial offset
        {1.0, 0.5, 1e-4, -0.3, true},   // near-concentric opposed normals
    };
    // A generic rigid placement, so that no coordinate axis lines up with the circle axes.
    const Transform3d rigid = Eigen::Translation3d(11.0, -7.0, 4.0) * Eigen::AngleAxisd(0.37, Vec3d(1.0, 2.0, 3.0).normalized());
    for (const Case &test : cases) {
        CAPTURE(test.radius0, test.radius1, test.projected_separation, test.axial_separation, test.opposed_normal);
        SurfaceFeature first = circle(Vec3d::Zero(), Vec3d::UnitZ(), test.radius0);
        SurfaceFeature second = circle(Vec3d(test.projected_separation, 0.0, test.axial_separation),
                                       test.opposed_normal ? Vec3d(0.0, 0.0, -1.0) : Vec3d::UnitZ(), test.radius1);
        first.translate(rigid);
        second.translate(rigid);
        // In-plane gap between the projected circles, combined with the axial gap.
        const double radial_gap = std::max({test.projected_separation - test.radius0 - test.radius1,
                                            std::abs(test.radius0 - test.radius1) - test.projected_separation, 0.0});
        const double expected = std::hypot(radial_gap, test.axial_separation);
        const MeasurementResult result = get_measurement(first, second);
        check_witnessed_distance(result.distance_infinite, expected, 1e-6);
        check_circle_pair_witness(result.distance_infinite, first, second, 1e-6);
    }
}

TEST_CASE("Perpendicular circles have analytic separation and intersection witnesses", "[Measure][MeasureContracts]")
{
    // Two unit circles about the same center in perpendicular planes meet at (+-1, 0, 0).
    const SurfaceFeature horizontal = circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0);
    const SurfaceFeature vertical = circle(Vec3d::Zero(), Vec3d::UnitY(), 1.0);
    const MeasurementResult intersecting = get_measurement(horizontal, vertical);
    check_witnessed_distance(intersecting.distance_infinite, 0.0, 1e-6);
    check_circle_pair_witness(intersecting.distance_infinite, horizontal, vertical, 1e-6);
    CHECK_THAT(std::abs(intersecting.distance_infinite->from.x()), WithinAbs(1.0, 1e-6));

    // Lifted by 3, the closest pair is (+-1, 0, 0) and the lifted-circle point towards it,
    // (0, 0, 3) + (+-1, 0, -3) / sqrt(10): sqrt(10) - 1 = sqrt(11 - 2 sqrt(10)) apart.
    const SurfaceFeature lifted = circle(Vec3d(0.0, 0.0, 3.0), Vec3d::UnitY(), 1.0);
    const MeasurementResult separated = get_measurement(horizontal, lifted);
    check_witnessed_distance(separated.distance_infinite, std::sqrt(11.0 - 2.0 * std::sqrt(10.0)), 1e-6);
    check_circle_pair_witness(separated.distance_infinite, horizontal, lifted, 1e-6);
}

TEST_CASE("Skew circle distances agree with angular minimization", "[Measure][MeasureContracts]")
{
    // Three fixed skew placements, each measured in both orders.
    const SurfaceFeature fixed[] = {
        circle(Vec3d(0.2, -0.4, 0.6), Vec3d::UnitZ(), 1.3),
        circle(Vec3d(2.4, 0.7, 0.3), Vec3d(std::sin(0.56), 0.0, std::cos(0.56)), 1.01),
        circle(Vec3d(0.82, 1.27, 1.37), Vec3d(std::sin(1.08), 0.0, std::cos(1.08)), 1.51),
    };
    for (size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        check_against_oracle(fixed[i], fixed[(i + 1) % 3]);
        check_against_oracle(fixed[(i + 1) % 3], fixed[i]);
    }

    // A sweep of tilts, offsets and radii.
    for (int index = 0; index < 8; ++index) {
        CAPTURE(index);
        const double angle = 0.17 + 0.13 * index;
        const SurfaceFeature a = circle(Vec3d(0.2, -0.4, 0.6), Vec3d::UnitZ(), 1.3 + 0.1 * index);
        const SurfaceFeature b = circle(Vec3d(2.4 - 0.23 * index, 0.7 + 0.19 * index, 0.3 + 0.11 * index),
                                        Vec3d(std::sin(angle), 0.0, std::cos(angle)), 0.8 + 0.07 * index);
        check_against_oracle(a, b);
        check_against_oracle(b, a);
    }

    // Perpendicular circles pulled apart along X.
    for (const double separation : {1.0, 4.0}) {
        CAPTURE(separation);
        const SurfaceFeature a = circle(Vec3d::Zero(), Vec3d::UnitZ(), 1.0);
        const SurfaceFeature b = circle(Vec3d(separation, 0.0, 0.0), Vec3d::UnitY(), 1.0);
        check_against_oracle(a, b);
        check_against_oracle(b, a);
    }
}

TEST_CASE("Skew circle distances scale under a common rigid placement", "[Measure][MeasureContracts]")
{
    const Vec3d normal = Vec3d(1.0, 2.0, 3.0).normalized();
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(0.71, Vec3d(2.0, 3.0, 1.0).normalized()).toRotationMatrix();
    const Vec3d translation(17.0, -9.0, 4.0);
    const SurfaceFeature a = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    const SurfaceFeature b = circle(Vec3d(3.0, 2.0, 1.0), normal, 1.0);
    check_against_oracle(a, b);
    const MeasurementResult original = get_measurement(a, b);
    REQUIRE(original.distance_infinite.has_value());
    for (const double scale : {0.2, 5.0}) {
        CAPTURE(scale);
        const SurfaceFeature moved_a = circle(translation, rotation * Vec3d::UnitZ(), 2.0 * scale);
        const SurfaceFeature moved_b = circle(translation + rotation * (Vec3d(3.0, 2.0, 1.0) * scale), rotation * normal, scale);
        check_against_oracle(moved_a, moved_b);
        const MeasurementResult moved = get_measurement(moved_a, moved_b);
        REQUIRE(moved.distance_infinite.has_value());
        CHECK_THAT(moved.distance_infinite->dist, WithinAbs(scale * original.distance_infinite->dist, 1e-6));
    }
}
