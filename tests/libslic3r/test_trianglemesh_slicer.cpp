#include <algorithm>
#include <cmath>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include <tbb/global_control.h>
#include <vector>
#include <utility>

#include <catch2/catch_test_macros.hpp>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleMeshSlicer.hpp"

using namespace Slic3r;

// The slab slicer collects each slab's intersection lines from a parallel loop over the facets.
// Its loops, and therefore the projected polygons, are derived from the order of those lines, so
// the order has to be canonical or the same mesh projects to different polygons run to run.
// The single-threaded projection is the reference; every multi-threaded run must reproduce it
// exactly, vertex order included.
TEST_CASE("Slab slicing projects the same polygons whatever the thread schedule", "[TriangleMeshSlicer]")
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

namespace cut_mesh_plane_contracts {

TriangleMesh box_with_middle_edge_ring()
{
    std::vector<Vec3f> vertices;
    vertices.reserve(12);
    for (int layer = 0; layer < 3; ++layer) {
        const float z = 10.f * float(layer);
        vertices.emplace_back(0.f, 0.f, z);
        vertices.emplace_back(20.f, 0.f, z);
        vertices.emplace_back(20.f, 20.f, z);
        vertices.emplace_back(0.f, 20.f, z);
    }

    std::vector<Vec3i32> facets { { 0, 2, 1 }, { 0, 3, 2 }, { 8, 9, 10 }, { 8, 10, 11 } };
    for (int layer = 0; layer < 2; ++layer) {
        const int lower = 4 * layer;
        const int upper = lower + 4;
        for (int edge = 0; edge < 4; ++edge) {
            const int next = (edge + 1) % 4;
            facets.emplace_back(lower + edge, lower + next, upper + next);
            facets.emplace_back(lower + edge, upper + next, upper + edge);
        }
    }
    return TriangleMesh(vertices, facets);
}

} // namespace cut_mesh_plane_contracts

TEST_CASE("The cut keeps both halves watertight when the plane follows an existing mesh edge",
          "[TriangleMeshSlicer][TriangleMeshCutMesh]")
{
    const TriangleMesh mesh = cut_mesh_plane_contracts::box_with_middle_edge_ring();
    indexed_triangle_set upper;
    indexed_triangle_set lower;

    cut_mesh(mesh.its, 10.f, &upper, &lower);

    CHECK(its_num_open_edges(upper) == 0);
    CHECK(its_num_open_edges(lower) == 0);
    CHECK_THAT(std::abs(its_volume(upper)), Catch::Matchers::WithinAbs(4000.f, 1e-2f));
    CHECK_THAT(std::abs(its_volume(lower)), Catch::Matchers::WithinAbs(4000.f, 1e-2f));
}

TEST_CASE("The cut caps a nested inward-facing cavity without filling it",
          "[TriangleMeshSlicer][TriangleMeshCutMesh]")
{
    indexed_triangle_set mesh = its_make_cube(20., 14., 8.);
    indexed_triangle_set cavity = its_make_cube(4., 6., 4.);
    for (Vec3f &vertex : cavity.vertices)
        vertex += Vec3f(8.f, 4.f, 2.f);
    for (Vec3i32 &face : cavity.indices)
        std::swap(face[0], face[1]);
    its_merge(mesh, cavity);
    REQUIRE(its_num_open_edges(mesh) == 0);
    REQUIRE_THAT(std::abs(its_volume(mesh)), Catch::Matchers::WithinAbs(2144.f, 1e-2f));

    indexed_triangle_set upper;
    indexed_triangle_set lower;
    cut_mesh(mesh, 4.f, &upper, &lower);

    // Each half contains half of the 20 x 14 x 8 shell.  The reversed 4 x 6 x 4
    // inner box removes 4 x 6 x 2 from each half, so both capped volumes are 1072.
    CHECK(its_num_open_edges(upper) == 0);
    CHECK(its_num_open_edges(lower) == 0);
    CHECK_THAT(std::abs(its_volume(upper)), Catch::Matchers::WithinAbs(1072.f, 1e-2f));
    CHECK_THAT(std::abs(its_volume(lower)), Catch::Matchers::WithinAbs(1072.f, 1e-2f));
}

TEST_CASE("The cut preserves a plane vertex when adjacent facets cross the plane",
          "[TriangleMeshSlicer][TriangleMeshCutMesh]")
{
    const TriangleMesh tetrahedron(
        std::vector<Vec3f> { { 0.f, 0.f, 10.f }, { 0.f, 10.f, 0.f }, { 10.f, 0.f, 20.f }, { 0.f, 0.f, 20.f } },
        std::vector<Vec3i32> { { 0, 1, 2 }, { 0, 2, 3 }, { 0, 3, 1 }, { 1, 3, 2 } });
    indexed_triangle_set upper;
    indexed_triangle_set lower;

    cut_mesh(tetrahedron.its, 10.f, &upper, &lower);

    CHECK(its_num_open_edges(upper) == 0);
    CHECK(its_num_open_edges(lower) == 0);
    CHECK_THAT(std::abs(its_volume(upper)), Catch::Matchers::WithinAbs(125.f, 1e-2f));
    CHECK_THAT(std::abs(its_volume(lower)), Catch::Matchers::WithinAbs(125.f / 3.f, 1e-2f));

    indexed_triangle_set upper_only;
    cut_mesh(tetrahedron.its, 10.f, &upper_only, nullptr);
    CHECK(its_num_open_edges(upper_only) == 0);
    CHECK_THAT(std::abs(its_volume(upper_only)), Catch::Matchers::WithinAbs(125.f, 1e-2f));

    indexed_triangle_set lower_only;
    cut_mesh(tetrahedron.its, 10.f, nullptr, &lower_only);
    CHECK(its_num_open_edges(lower_only) == 0);
    CHECK_THAT(std::abs(its_volume(lower_only)), Catch::Matchers::WithinAbs(125.f / 3.f, 1e-2f));
}

