#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <Eigen/Core>
#include <libslic3r/Point.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/MeshBoolean.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
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

TEST_CASE("CGAL remeshing caps every edge at 4/3 of the target and keeps a constrained cuboid footprint", "[MeshBoolean][MeshBooleanContracts]")
{
    // The prepass only splits the protected (sharp) cube edges; the 14.1 mm face diagonals are
    // shortened only by the isotropic remeshing pass itself, which splits anything above 4/3 target.
    const float target = 2.5f;
    const indexed_triangle_set source = shifted_cube(2.f, -3.f, 4.f).its;
    const indexed_triangle_set remeshed = MeshBoolean::cgal::remesh_isotropic(source, target, 1, 40., 1);
    REQUIRE(remeshed.indices.size() > source.indices.size());
    check_valid_indices(remeshed);
    CHECK(its_num_open_edges(remeshed) == 0);
    float max_edge = 0.f;
    for (const Vec3i32 &face : remeshed.indices)
        for (int i = 0; i < 3; ++i)
            max_edge = std::max(max_edge, (remeshed.vertices[face[i]] - remeshed.vertices[face[(i + 1) % 3]]).norm());
    CHECK(max_edge <= 4.f / 3.f * target);
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

TEST_CASE("CGAL LSCM parameterization maps a disk and rejects disconnected patches", "[MeshBoolean][MeshBooleanContracts]")
{
    const indexed_triangle_set square = open_square();
    const auto uv = MeshBoolean::cgal::parameterize_lscm(square);
    REQUIRE(uv.has_value());
    REQUIRE(uv->size() == square.vertices.size());
    for (const Vec2f &point : *uv) {
        CHECK(std::isfinite(point.x()));
        CHECK(std::isfinite(point.y()));
    }
    const Vec2f e0 = (*uv)[1] - (*uv)[0];
    const Vec2f e1 = (*uv)[2] - (*uv)[1];
    const Vec2f e2 = (*uv)[3] - (*uv)[2];
    const Vec2f e3 = (*uv)[0] - (*uv)[3];
    const Vec2f d0 = (*uv)[2] - (*uv)[0];
    const Vec2f d1 = (*uv)[3] - (*uv)[1];
    const float uv_area = std::abs(e0.x() * e1.y() - e0.y() * e1.x()) * 0.5f +
                          std::abs(e2.x() * e3.y() - e2.y() * e3.x()) * 0.5f;
    CHECK(uv_area > 1e-6f);
    CHECK_THAT(e0.norm(), WithinRel(e1.norm(), 1e-3f));
    CHECK_THAT(e0.dot(e1), WithinAbs(0.f, 1e-5f));
    CHECK_THAT(e0.norm(), WithinRel(e2.norm(), 1e-3f));
    CHECK_THAT(e1.norm(), WithinRel(e3.norm(), 1e-3f));
    CHECK_THAT(d0.norm(), WithinRel(d1.norm(), 1e-3f));

    indexed_triangle_set disconnected = square;
    const int offset = static_cast<int>(disconnected.vertices.size());
    disconnected.vertices.insert(disconnected.vertices.end(), square.vertices.begin(), square.vertices.end());
    for (Vec3i32 face : square.indices) {
        face.array() += offset;
        disconnected.indices.push_back(face);
    }
    CHECK_FALSE(MeshBoolean::cgal::parameterize_lscm(disconnected).has_value());
}

TEST_CASE("CGAL segmentation splits a dumbbell into closed ball and rod parts", "[MeshBoolean][MeshBooleanContracts]")
{
    // Two spheres joined by a thin rod, so the SDF segmentation has regions of clearly different
    // thickness to separate. Each part comes back hole-filled, so the parts' volumes add up.
    TriangleMesh dumbbell = make_sphere(5.);
    TriangleMesh far_ball = make_sphere(5.);
    far_ball.translate(Vec3f(0.f, 0.f, 20.f));
    MeshBoolean::cgal::plus(dumbbell, far_ball);
    MeshBoolean::cgal::plus(dumbbell, make_cylinder(1., 20.));
    const float dumbbell_volume = mesh_volume(dumbbell);
    const float ball_volume = mesh_volume(make_sphere(5.));

    const std::vector<TriangleMesh> pieces = MeshBoolean::cgal::segment(dumbbell, 0.5, 3);
    REQUIRE(pieces.size() >= 3);
    std::vector<float> volumes;
    for (const TriangleMesh &piece : pieces) {
        check_valid_indices(piece.its);
        CHECK(its_num_open_edges(piece.its) == 0);
        volumes.push_back(mesh_volume(piece));
    }
    std::sort(volumes.begin(), volumes.end(), [](float a, float b) { return a > b; });
    // The two thick parts are the balls; the rod between them is a separate, much thinner part.
    CHECK_THAT(volumes[0], WithinRel(ball_volume, 1e-2f));
    CHECK_THAT(volumes[1], WithinRel(ball_volume, 1e-2f));
    for (size_t i = 2; i < volumes.size(); ++i) {
        CHECK(volumes[i] > 0.f);
        CHECK(volumes[i] < 0.1f * ball_volume);
    }
    float total_volume = 0.f;
    for (const float volume : volumes)
        total_volume += volume;
    CHECK_THAT(total_volume, WithinRel(dumbbell_volume, 1e-2f));
}

TEST_CASE("CGAL repair closes a cube with one missing triangle", "[MeshBoolean][MeshBooleanContracts]")
{
    // its_volume() measures heights from the first vertex, so drop a triangle on a face that does
    // not contain that vertex; otherwise the hole would not change the volume. TriangleMesh::volume()
    // is not used because it returns the cached volume of the closed cube.
    TriangleMesh open = shifted_cube(0.f);
    const Vec3f ref = open.its.vertices.front();
    const auto dropped = std::find_if(open.its.indices.begin(), open.its.indices.end(), [&](const Vec3i32 &f) {
        const Vec3f &a = open.its.vertices[f[0]];
        const Vec3f n = (open.its.vertices[f[1]] - a).cross(open.its.vertices[f[2]] - a).normalized();
        return std::abs(n.dot(a - ref)) > 1.f;
    });
    REQUIRE(dropped != open.its.indices.end());
    open.its.indices.erase(dropped);
    REQUIRE(its_volume(open.its) < 999.f);
    REQUIRE(its_num_open_edges(open.its) > 0);
    REQUIRE_FALSE(MeshBoolean::cgal::does_bound_a_volume(*MeshBoolean::cgal::triangle_mesh_to_cgal(open)));
    RepairedMeshErrors errors;
    std::string error;
    REQUIRE(MeshBoolean::cgal::repair(open, &errors, &error));
    CHECK(error.empty());
    CHECK(its_num_open_edges(open.its) == 0);
    CHECK_THAT(its_volume(open.its), WithinRel(1000.f, 1e-3f));
    CHECK_FALSE(MeshBoolean::cgal::does_self_intersect(open));
    const auto repaired_cgal = MeshBoolean::cgal::triangle_mesh_to_cgal(open);
    REQUIRE(repaired_cgal != nullptr);
    CHECK(MeshBoolean::cgal::does_bound_a_volume(*repaired_cgal));
}

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

TEST_CASE("Eigen conversion round trips and self-union merges overlapping cuboids", "[MeshBoolean][MeshBooleanContracts]")
{
    // Two overlapping cubes in one mesh: the raw signed volume counts the 500 mm^3 overlap twice.
    TriangleMesh source = shifted_cube(0.f);
    TriangleMesh second = shifted_cube(5.f);
    its_merge(source.its, second.its);
    const MeshBoolean::EigenMesh eigen = MeshBoolean::triangle_mesh_to_eigen(source);
    REQUIRE(eigen.first.rows() == static_cast<Eigen::Index>(source.its.vertices.size()));
    REQUIRE(eigen.second.rows() == static_cast<Eigen::Index>(source.its.indices.size()));
    const TriangleMesh round_trip = MeshBoolean::eigen_to_triangle_mesh(eigen);
    CHECK_THAT(mesh_volume(round_trip), WithinRel(2000.f, 1e-5f));

    MeshBoolean::EigenMesh union_input = eigen;
    MeshBoolean::self_union(union_input);
    const TriangleMesh union_mesh = MeshBoolean::eigen_to_triangle_mesh(union_input);
    CHECK_THAT(mesh_volume(union_mesh), WithinRel(1500.f, 1e-3f));
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
