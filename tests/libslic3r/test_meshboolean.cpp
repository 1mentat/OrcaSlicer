#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/MeshBoolean.hpp>
#include <cstddef>
#include <utility>
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

indexed_triangle_set open_square()
{
    indexed_triangle_set mesh;
    mesh.vertices = {Vec3f(0.f, 0.f, 0.f), Vec3f(10.f, 0.f, 0.f), Vec3f(10.f, 10.f, 0.f), Vec3f(0.f, 10.f, 0.f)};
    mesh.indices = {Vec3i32(0, 1, 2), Vec3i32(0, 2, 3)};
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

TEST_CASE("CGAL booleans preserve analytic cuboid volumes", "[MeshBoolean][MeshBooleanContracts]")
{
    const TriangleMesh original = shifted_cube(0.f);
    const TriangleMesh tool = shifted_cube(5.f);

    TriangleMesh difference = original;
    MeshBoolean::cgal::minus(difference, tool);
    CHECK_THAT(mesh_volume(difference), WithinRel(500.f, 1e-3f));

    TriangleMesh intersection = original;
    MeshBoolean::cgal::intersect(intersection, tool);
    CHECK_THAT(mesh_volume(intersection), WithinRel(500.f, 1e-3f));

    TriangleMesh union_mesh = original;
    MeshBoolean::cgal::plus(union_mesh, tool);
    CHECK_THAT(mesh_volume(union_mesh), WithinRel(1500.f, 1e-3f));
    CHECK_FALSE(MeshBoolean::cgal::does_self_intersect(union_mesh));
}

TEST_CASE("Indexed CGAL booleans retain valid topology and volumes", "[MeshBoolean][MeshBooleanContracts]")
{
    const indexed_triangle_set original = shifted_cube(0.f).its;
    const indexed_triangle_set tool = shifted_cube(5.f).its;

    indexed_triangle_set difference = original;
    MeshBoolean::cgal::minus(difference, tool);
    check_valid_indices(difference);
    CHECK_THAT(mesh_volume(TriangleMesh(difference)), WithinRel(500.f, 1e-3f));

    indexed_triangle_set intersection = original;
    MeshBoolean::cgal::intersect(intersection, tool);
    check_valid_indices(intersection);
    CHECK_THAT(mesh_volume(TriangleMesh(intersection)), WithinRel(500.f, 1e-3f));

    indexed_triangle_set union_mesh = original;
    MeshBoolean::cgal::plus(union_mesh, tool);
    check_valid_indices(union_mesh);
    CHECK_THAT(mesh_volume(TriangleMesh(union_mesh)), WithinRel(1500.f, 1e-3f));
}

TEST_CASE("CGAL conversion identifies empty, closed, and cloned meshes", "[MeshBoolean][MeshBooleanContracts]")
{
    const auto empty = MeshBoolean::cgal::triangle_mesh_to_cgal(indexed_triangle_set{});
    REQUIRE(empty != nullptr);
    CHECK(MeshBoolean::cgal::empty(*empty));

    const TriangleMesh cube = shifted_cube(3.f, -2.f, 4.f);
    const auto converted = MeshBoolean::cgal::triangle_mesh_to_cgal(cube);
    REQUIRE(converted != nullptr);
    CHECK_FALSE(MeshBoolean::cgal::empty(*converted));
    CHECK(MeshBoolean::cgal::does_bound_a_volume(*converted));
    const auto open_patch = MeshBoolean::cgal::triangle_mesh_to_cgal(open_square());
    REQUIRE(open_patch != nullptr);
    CHECK_FALSE(MeshBoolean::cgal::does_bound_a_volume(*open_patch));

    const auto cloned = MeshBoolean::cgal::clone(*converted);
    REQUIRE(cloned != nullptr);
    const TriangleMesh round_trip = MeshBoolean::cgal::cgal_to_triangle_mesh(*cloned);
    CHECK_THAT(mesh_volume(round_trip), WithinRel(mesh_volume(cube), 1e-4f));
    CHECK(round_trip.its.indices.size() == cube.its.indices.size());
}

TEST_CASE("CGAL merge joins disjoint closed components without changing total volume", "[MeshBoolean][MeshBooleanContracts]")
{
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(shifted_cube(0.f));
    meshes.emplace_back(shifted_cube(12.f));
    const TriangleMesh merged = MeshBoolean::cgal::merge(std::move(meshes));
    check_valid_indices(merged.its);
    CHECK_THAT(mesh_volume(merged), WithinRel(2000.f, 1e-3f));
    CHECK_THAT(merged.bounding_box().min.x(), WithinAbs(0., 1e-5));
    CHECK_THAT(merged.bounding_box().max.x(), WithinAbs(22., 1e-5));
}

} // namespace meshboolean_contracts
