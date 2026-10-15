#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/catch_test_macros.hpp"
#include "libslic3r/Measure.hpp"
#include "libslic3r/TriangleMesh.hpp"

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
