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

// Known bug: when the projected circles intersect, the parallel branch returns the two centers as
// witnesses and their distance; fixed by the follow-up fix PR
TEST_CASE("Intersecting coplanar circles report zero distance at a common point", "[Measure][MeasureContracts][!shouldfail]")
{
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0), circle(Vec3d(1.0, 0.0, 0.0), Vec3d::UnitZ(), 2.0), 0.0);
}

// Known bug: for parallel circles in different planes the distance ignores the axial gap; fixed by the
// follow-up fix PR
TEST_CASE("Parallel circles in different planes include the axial gap", "[Measure][MeasureContracts][!shouldfail]")
{
    const SurfaceFeature radius2 = circle(Vec3d::Zero(), Vec3d::UnitZ(), 2.0);
    check_parallel_pair(radius2, circle(Vec3d(5.0, 0.0, 3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0));  // disjoint, above
    check_parallel_pair(radius2, circle(Vec3d(5.0, 0.0, -3.0), Vec3d::UnitZ(), 1.0), std::sqrt(13.0)); // disjoint, below
    check_parallel_pair(circle(Vec3d::Zero(), Vec3d::UnitZ(), 3.0), circle(Vec3d(0.0, 0.0, 2.0), Vec3d::UnitZ(), 1.0),
                        std::sqrt(8.0)); // concentric
    check_parallel_pair(radius2, circle(Vec3d(0.0, 0.0, -2.0), Vec3d::UnitZ(), 2.0), 2.0); // equal concentric
    check_parallel_pair(radius2, circle(Vec3d(1.0, 0.0, 2.0), Vec3d::UnitZ(), 2.0), 2.0);  // projections intersect
}

// Known bug: as above, parallel circles in different planes lose the axial gap; fixed by the follow-up fix PR
TEST_CASE("Parallel circle analytic sweep preserves axial and radial gaps", "[Measure][MeasureContracts][!shouldfail]")
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
