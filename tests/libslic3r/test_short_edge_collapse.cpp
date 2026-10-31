#include <algorithm>
#include <cstddef>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include <Eigen/Core>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/ShortEdgeCollapse.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

bool all_indices_valid(const indexed_triangle_set& its)
{
    return std::all_of(its.indices.begin(), its.indices.end(), [&its](const Vec3i32& face) {
        return std::all_of(face.begin(), face.end(), [&its](int i) { return i >= 0 && size_t(i) < its.vertices.size(); });
    });
}

} // namespace

TEST_CASE("Short-edge collapse preserves geometry when the face budget is already met", "[ShortEdgeCollapse]")
{
    auto mesh = its_make_cube(2., 2., 2.);
    const size_t original_faces = mesh.indices.size();
    REQUIRE(original_faces == 12);
    REQUIRE(its_num_open_edges(mesh) == 0);
    const BoundingBoxf3 source_bounds = bounding_box(mesh);

    its_short_edge_collpase(mesh, original_faces);

    CHECK(mesh.indices.size() == original_faces);
    REQUIRE(all_indices_valid(mesh));
    CHECK_THAT(its_volume(mesh), WithinAbs(8., 1e-5));
    const BoundingBoxf3 result_bounds = bounding_box(mesh);
    for (int axis = 0; axis < 3; ++axis) {
        CHECK_THAT(result_bounds.min[axis], WithinAbs(source_bounds.min[axis], 1e-6));
        CHECK_THAT(result_bounds.max[axis], WithinAbs(source_bounds.max[axis], 1e-6));
    }
}

TEST_CASE("Short-edge collapse decimates a closed sphere to its face budget without orphan vertices", "[ShortEdgeCollapse]")
{
    const indexed_triangle_set source = its_make_sphere(5., PI / 12.);
    REQUIRE(source.indices.size() > 8);
    REQUIRE(its_num_open_edges(source) == 0);
    const float source_volume = its_volume(source);
    const size_t target_faces = source.indices.size() * 3 / 4;

    indexed_triangle_set result = source;
    its_short_edge_collpase(result, target_faces);

    CHECK(result.indices.size() <= target_faces);
    CHECK(result.vertices.size() < source.vertices.size());
    REQUIRE(all_indices_valid(result));
    // Only vertices still referenced by a face are kept.
    CHECK(its_compactify_vertices(result) == 0);
    // Collapsing short edges of a fine sphere keeps it a solid of nearly the same volume.
    CHECK(its_volume(result) > 0.9f * source_volume);
    CHECK(its_volume(result) <= source_volume);
}

TEST_CASE("Short-edge collapse can reduce an open indexed surface to its requested face budget", "[ShortEdgeCollapse]")
{
    indexed_triangle_set mesh;
    mesh.vertices = {
        Vec3f(0.f, 0.f, 0.f), Vec3f(1.f, 0.f, 0.f), Vec3f(0.f, 1.f, 0.f),
        Vec3f(4.f, 0.f, 0.f), Vec3f(5.f, 0.f, 0.f), Vec3f(4.f, 1.f, 0.f)
    };
    mesh.indices = { Vec3i32(0, 1, 2), Vec3i32(3, 4, 5) };
    REQUIRE(its_num_open_edges(mesh) == 6);

    its_short_edge_collpase(mesh, 1);

    REQUIRE(mesh.indices.size() == 1);
    REQUIRE(mesh.vertices.size() == 3);
    REQUIRE(all_indices_valid(mesh));
    CHECK(its_num_open_edges(mesh) == 3);
    const Vec3i32& face = mesh.indices.front();
    const Vec3f normal = (mesh.vertices[face[1]] - mesh.vertices[face[0]]).cross(
        mesh.vertices[face[2]] - mesh.vertices[face[0]]);
    CHECK(normal.squaredNorm() > 0.f);
}
