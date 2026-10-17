#include <algorithm>
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include <tbb/global_control.h>
#include <vector>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

// The slab slicer collects each slab's intersection lines from a parallel loop over the facets.
// Its loops, and therefore the projected polygons, are derived from the order of those lines, so
// the order has to be canonical or the same mesh projects to different polygons run to run.
// The single-threaded projection is the reference; every multi-threaded run must reproduce it
// exactly, vertex order included.
TEST_CASE("Slab slicing projects the same polygons whatever the thread schedule", "[TriangleMeshSlicer][Parallel]")
{
    // A dense sphere, tilted so no facet is axis aligned: thousands of upward and downward
    // facing facets spread over every slab.
    indexed_triangle_set mesh = its_make_sphere(10., 0.05);
    Transform3d trafo = Transform3d::Identity();
    trafo.rotate(Eigen::AngleAxisd(0.37, Vec3d(0.3, 0.5, 1.).normalized()));
    trafo.translate(Vec3d(1., 2., 0.));
    std::vector<float> zs;
    for (float z = -9.7f; z < 9.7f; z += 0.2f)
        zs.emplace_back(z);

    auto project = [&mesh, &trafo, &zs]() {
        std::vector<Polygons> top, bottom;
        slice_mesh_slabs(mesh, zs, trafo, &top, &bottom, nullptr, []{});
        return std::make_pair(std::move(top), std::move(bottom));
    };

    std::pair<std::vector<Polygons>, std::vector<Polygons>> reference;
    {
        tbb::global_control single_thread(tbb::global_control::max_allowed_parallelism, 1);
        reference = project();
    }
    REQUIRE(reference.first.size() == zs.size());
    REQUIRE(std::any_of(reference.first.begin(), reference.first.end(), [](const Polygons &p) { return !p.empty(); }));
    REQUIRE(std::any_of(reference.second.begin(), reference.second.end(), [](const Polygons &p) { return !p.empty(); }));

    for (int run = 0; run < 3; ++run) {
        DYNAMIC_SECTION("multi-threaded run " << run)
        {
            auto parallel = project();
            CHECK(parallel.first == reference.first);
            CHECK(parallel.second == reference.second);
        }
    }
}


namespace {

indexed_triangle_set vertical_panel_mesh()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 0.f), Vec3f(10.f, 0.f, 0.f),
        Vec3f(10.f, 0.f, 10.f), Vec3f(0.f, 0.f, 10.f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(0, 2, 3) };
    return mesh;
}

indexed_triangle_set coplanar_up_and_down_mesh()
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 5.f), Vec3f(10.f, 0.f, 5.f), Vec3f(0.f, 10.f, 5.f),
        Vec3f(20.f, 0.f, 5.f), Vec3f(20.f, 10.f, 5.f), Vec3f(30.f, 0.f, 5.f)
    };
    // The first triangle has +Z winding; the second has -Z winding.
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(3, 4, 5) };
    return mesh;
}

void require_xy_bounds(const Polygon &polygon, double min_x, double max_x, double min_y, double max_y)
{
    REQUIRE_FALSE(polygon.empty());
    const auto bounds = polygon.bounding_box();
    CHECK_THAT(unscaled<double>(bounds.min.x()), Catch::Matchers::WithinAbs(min_x, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.max.x()), Catch::Matchers::WithinAbs(max_x, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.min.y()), Catch::Matchers::WithinAbs(min_y, 1e-4));
    CHECK_THAT(unscaled<double>(bounds.max.y()), Catch::Matchers::WithinAbs(max_y, 1e-4));
}

} // namespace

TEST_CASE("Slab slicing reports vertical face centers and normals", "[TriangleMeshSlicer][VerticalPoints][TriangleMeshSlicerContracts]")
{
    std::vector<std::pair<Vec3f, Vec3f>> vertical_points;
    slice_mesh_slabs(vertical_panel_mesh(), { 0.f, 5.f, 10.f }, Transform3d::Identity(),
                     nullptr, nullptr, &vertical_points, []{});

    REQUIRE(vertical_points.size() == 2);
    std::sort(vertical_points.begin(), vertical_points.end(), [](const auto &left, const auto &right) {
        return left.first.x() < right.first.x();
    });
    CHECK_THAT(double(vertical_points[0].first.x()) * SCALING_FACTOR, Catch::Matchers::WithinAbs(10.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[0].first.z()), Catch::Matchers::WithinAbs(20.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[1].first.x()) * SCALING_FACTOR, Catch::Matchers::WithinAbs(20.0 / 3.0, 1e-5));
    CHECK_THAT(double(vertical_points[1].first.z()), Catch::Matchers::WithinAbs(10.0 / 3.0, 1e-5));
    for (const auto &[center, normal] : vertical_points) {
        CHECK_THAT(center.y(), Catch::Matchers::WithinAbs(0.f, 1e-5));
        CHECK_THAT(normal.norm(), Catch::Matchers::WithinAbs(1.f, 1e-5));
        CHECK_THAT(std::abs(normal.x()), Catch::Matchers::WithinAbs(0.f, 1e-5));
        CHECK_THAT(std::abs(normal.y()), Catch::Matchers::WithinAbs(1.f, 1e-5));
        CHECK_THAT(std::abs(normal.z()), Catch::Matchers::WithinAbs(0.f, 1e-5));
    }
}

TEST_CASE("Slab slicing assigns coplanar faces to their documented surface", "[TriangleMeshSlicer][Slab][TriangleMeshSlicerContracts]")
{
    const auto mesh = coplanar_up_and_down_mesh();
    const std::vector<float> zs { 5.f, 10.f };

    std::vector<Polygons> top;
    slice_mesh_slabs(mesh, zs, Transform3d::Identity(), &top, nullptr, nullptr, []{});
    REQUIRE(top.size() == zs.size());
    REQUIRE(top[0].size() == 1);
    CHECK(top[1].empty());
    require_xy_bounds(top[0].front(), 0., 10., 0., 10.);

    std::vector<Polygons> bottom;
    slice_mesh_slabs(mesh, zs, Transform3d::Identity(), nullptr, &bottom, nullptr, []{});
    REQUIRE(bottom.size() == zs.size());
    CHECK(bottom[0].empty());
    REQUIRE(bottom[1].size() == 1);
    require_xy_bounds(bottom[1].front(), 20., 30., 0., 10.);
}

TEST_CASE("Mirrored slab transforms preserve surface roles", "[TriangleMeshSlicer][Slab][Transform][TriangleMeshSlicerContracts]")
{
    Transform3d mirror = Transform3d::Identity();
    mirror.prescale(Vec3d(-1., 1., 1.));

    std::vector<Polygons> top, bottom;
    slice_mesh_slabs(coplanar_up_and_down_mesh(), { 5.f, 10.f }, mirror,
                     &top, &bottom, nullptr, []{});
    REQUIRE(top.size() == 2);
    REQUIRE(bottom.size() == 2);
    REQUIRE(top[0].size() == 1);
    CHECK(bottom[0].empty());
    CHECK(top[1].empty());
    REQUIRE(bottom[1].size() == 1);
    require_xy_bounds(top[0].front(), -10., 0., 0., 10.);
    require_xy_bounds(bottom[1].front(), -30., -20., 0., 10.);
}

TEST_CASE("Mesh projection exposes each requested surface independently", "[TriangleMeshSlicer][Projection][TriangleMeshSlicerContracts]")
{
    const auto cube = its_make_cube(10., 20., 30.);
    Polygons top;
    project_mesh(cube, Transform3d::Identity(), &top, nullptr, []{});
    REQUIRE(top.size() == 1);
    CHECK_THAT(std::abs(double(top.front().area())) * SCALING_FACTOR * SCALING_FACTOR, Catch::Matchers::WithinAbs(200.0, 1e-4));
    CHECK_THAT(unscaled<double>(top.front().bounding_box().size().x()), Catch::Matchers::WithinAbs(10., 1e-4));
    CHECK_THAT(unscaled<double>(top.front().bounding_box().size().y()), Catch::Matchers::WithinAbs(20., 1e-4));

    Polygons bottom;
    project_mesh(cube, Transform3d::Identity(), nullptr, &bottom, []{});
    REQUIRE(bottom.size() == 1);
    CHECK_THAT(std::abs(double(bottom.front().area())) * SCALING_FACTOR * SCALING_FACTOR, Catch::Matchers::WithinAbs(200.0, 1e-4));
    CHECK_THAT(unscaled<double>(bottom.front().bounding_box().size().x()), Catch::Matchers::WithinAbs(10., 1e-4));
    CHECK_THAT(unscaled<double>(bottom.front().bounding_box().size().y()), Catch::Matchers::WithinAbs(20., 1e-4));
}
