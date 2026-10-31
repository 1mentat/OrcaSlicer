#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/MeshBoolean.hpp>
#include <cstddef>
#include <vector>

using namespace Slic3r;

TEST_CASE("CGAL and TriangleMesh conversions", "[MeshBoolean]") {
    TriangleMesh sphere = make_sphere(1.);
    
    auto cgalmesh_ptr = MeshBoolean::cgal::triangle_mesh_to_cgal(sphere);
    
    REQUIRE(cgalmesh_ptr);
    REQUIRE(! MeshBoolean::cgal::does_self_intersect(*cgalmesh_ptr));
    
    TriangleMesh M = MeshBoolean::cgal::cgal_to_triangle_mesh(*cgalmesh_ptr);
    
    REQUIRE(M.its.vertices.size() == sphere.its.vertices.size());
    REQUIRE(M.its.indices.size() == sphere.its.indices.size());
    
    REQUIRE(M.volume() == Catch::Approx(sphere.volume()));
    
    REQUIRE(! MeshBoolean::cgal::does_self_intersect(M));
}

TEST_CASE("mcut difference handles source splits between cuts", "[MeshBoolean]") {
    TriangleMesh body = make_cube(30., 10., 10.);

    TriangleMesh tool;

    // First cut splits the source into two disconnected components.
    TriangleMesh slab = make_cube(2., 12., 20.);
    slab.translate(Vec3f(14.f, -1.f, -5.f));
    its_merge(tool.its, slab.its);

    // These cuts must still be applied after the source has been split.
    TriangleMesh left_hole = make_cube(4., 4., 20.);
    left_hole.translate(Vec3f(3.f, 3.f, -5.f));
    its_merge(tool.its, left_hole.its);

    TriangleMesh right_hole = make_cube(4., 4., 20.);
    right_hole.translate(Vec3f(21.f, 3.f, -5.f));
    its_merge(tool.its, right_hole.its);

    std::vector<TriangleMesh> result;
    MeshBoolean::mcut::make_boolean(body, tool, result, "A_NOT_B");

    REQUIRE(result.size() == 1);

    const std::vector<indexed_triangle_set> components =
        its_split(result.front().its);

    REQUIRE(components.size() == 2);

    // 3000 - 200 - 160 - 160 = 2480.
    REQUIRE_THAT(
        result.front().volume(),
        Catch::Matchers::WithinRel(2480., 1e-3)
    );
}

namespace meshboolean_contracts {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

TriangleMesh shifted_cube(float x, float y = 0.f, float z = 0.f, float side = 10.f)
{
    TriangleMesh mesh = make_cube(side, side, side);
    mesh.translate(Vec3f(x, y, z));
    return mesh;
}

float mesh_volume(const TriangleMesh &mesh)
{
    TriangleMesh copy = mesh;
    return copy.volume();
}

void check_valid_indices(const indexed_triangle_set &mesh)
{
    REQUIRE_FALSE(mesh.vertices.empty());
    REQUIRE_FALSE(mesh.indices.empty());
    for (const Vec3i32 &face : mesh.indices) {
        REQUIRE(face[0] >= 0);
        REQUIRE(face[1] >= 0);
        REQUIRE(face[2] >= 0);
        REQUIRE(static_cast<size_t>(face[0]) < mesh.vertices.size());
        REQUIRE(static_cast<size_t>(face[1]) < mesh.vertices.size());
        REQUIRE(static_cast<size_t>(face[2]) < mesh.vertices.size());
    }
}

} // namespace

TEST_CASE("MCUT conversion round trips cuboid topology and empty state", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh cube = shifted_cube(7.f, 1.f, -2.f);
    const auto converted = MeshBoolean::mcut::triangle_mesh_to_mcut(cube.its);
    REQUIRE(converted != nullptr);
    CHECK_FALSE(MeshBoolean::mcut::empty(*converted));
    const TriangleMesh round_trip = MeshBoolean::mcut::mcut_to_triangle_mesh(*converted);
    CHECK(round_trip.its.vertices.size() == cube.its.vertices.size());
    CHECK(round_trip.its.indices.size() == cube.its.indices.size());
    CHECK_THAT(mesh_volume(round_trip), WithinRel(mesh_volume(cube), 1e-5f));

    const auto empty = MeshBoolean::mcut::triangle_mesh_to_mcut(indexed_triangle_set{});
    REQUIRE(empty != nullptr);
    CHECK(MeshBoolean::mcut::empty(*empty));
}

TEST_CASE("MCUT union combines disjoint solids and preserves component volume", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh first = shifted_cube(0.f);
    const TriangleMesh second = shifted_cube(12.f);
    std::vector<TriangleMesh> result;
    MeshBoolean::mcut::make_boolean(first, second, result, "UNION");
    REQUIRE_FALSE(result.empty());
    float volume = 0.f;
    for (const TriangleMesh &mesh : result) {
        check_valid_indices(mesh.its);
        volume += mesh_volume(mesh);
    }
    CHECK_THAT(volume, WithinRel(2000.f, 1e-3f));
}

TEST_CASE("MCUT union accepts an empty source as the identity element", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh tool = shifted_cube(4.f, 5.f, 6.f);
    std::vector<TriangleMesh> result;
    MeshBoolean::mcut::make_boolean(TriangleMesh{}, tool, result, "UNION");
    REQUIRE(result.size() == 1);
    check_valid_indices(result.front().its);
    CHECK_THAT(mesh_volume(result.front()), WithinRel(1000.f, 1e-3f));
}

TEST_CASE("MCUT intersection returns the shared volume of overlapping solids", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh first = shifted_cube(0.f);
    const TriangleMesh second = shifted_cube(5.f);
    std::vector<TriangleMesh> result;
    MeshBoolean::mcut::make_boolean(first, second, result, "INTERSECTION");
    REQUIRE_FALSE(result.empty());
    float volume = 0.f;
    for (const TriangleMesh &mesh : result) {
        check_valid_indices(mesh.its);
        volume += mesh_volume(mesh);
    }
    CHECK_THAT(volume, WithinRel(500.f, 1e-2f));
}

TEST_CASE("MCUT difference preserves analytic hole volume for a centered tool", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh body = shifted_cube(0.f);
    const TriangleMesh tool = shifted_cube(3.f, 3.f, -1.f, 4.f);
    std::vector<TriangleMesh> result;
    MeshBoolean::mcut::make_boolean(body, tool, result, "A_NOT_B");
    REQUIRE_FALSE(result.empty());
    float volume = 0.f;
    for (const TriangleMesh &mesh : result)
        volume += mesh_volume(mesh);
    CHECK_THAT(volume, WithinAbs(952.f, 5.f));
}

} // namespace meshboolean_contracts
