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

TEST_CASE("CGAL remeshing keeps a constrained cuboid footprint and valid faces", "[MeshBoolean][MeshBooleanContracts]")
{
    const indexed_triangle_set source = shifted_cube(2.f, -3.f, 4.f).its;
    const indexed_triangle_set remeshed = MeshBoolean::cgal::remesh_isotropic(source, 2.5, 1, 40., 1);
    REQUIRE(remeshed.indices.size() > source.indices.size());
    check_valid_indices(remeshed);
    const TriangleMesh result(remeshed);
    CHECK_THAT(result.bounding_box().min.x(), WithinAbs(2., 1e-4));
    CHECK_THAT(result.bounding_box().max.x(), WithinAbs(12., 1e-4));
    CHECK_THAT(result.bounding_box().min.y(), WithinAbs(-3., 1e-4));
    CHECK_THAT(result.bounding_box().max.y(), WithinAbs(7., 1e-4));
    CHECK_THAT(result.bounding_box().min.z(), WithinAbs(4., 1e-4));
    CHECK_THAT(result.bounding_box().max.z(), WithinAbs(14., 1e-4));
    CHECK_THAT(mesh_volume(result), WithinRel(1000.f, 5e-2f));
}

TEST_CASE("CGAL remeshing returns invalid requests unchanged", "[MeshBoolean][MeshBooleanContracts]")
{
    const indexed_triangle_set source = shifted_cube(0.f).its;
    const indexed_triangle_set zero_length = MeshBoolean::cgal::remesh_isotropic(source, 0., 2);
    const indexed_triangle_set empty = MeshBoolean::cgal::remesh_isotropic(indexed_triangle_set{}, 2.);
    CHECK(zero_length.vertices == source.vertices);
    CHECK(zero_length.indices == source.indices);
    CHECK(empty.vertices.empty());
    CHECK(empty.indices.empty());
}

} // namespace meshboolean_contracts
