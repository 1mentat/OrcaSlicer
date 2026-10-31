#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "test_utils.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

bool all_indices_valid(const indexed_triangle_set& its)
{
    return std::all_of(its.indices.begin(), its.indices.end(), [&its](const Vec3i32& face) {
        return std::all_of(face.begin(), face.end(), [&its](int i) { return i >= 0 && size_t(i) < its.vertices.size(); });
    });
}

void parse_obj(const std::string& text, std::vector<Vec3f>& vertices, std::vector<Vec3i32>& faces)
{
    std::istringstream input(text);
    std::string kind;
    while (input >> kind) {
        if (kind == "v") {
            float x = 0.f;
            float y = 0.f;
            float z = 0.f;
            input >> x >> y >> z;
            vertices.emplace_back(x, y, z);
        } else if (kind == "f") {
            int a = 0;
            int b = 0;
            int c = 0;
            input >> a >> b >> c;
            faces.emplace_back(a - 1, b - 1, c - 1);
        }
        std::string line;
        std::getline(input, line);
    }
}

} // namespace

TEST_CASE("Indexed mesh edge identifiers pair every cube edge", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    const auto ids                  = its_face_edge_ids(cube);
    REQUIRE(ids.size() == cube.indices.size());
    std::unordered_map<int, int> occurrences;
    for (const Vec3i32& face : ids)
        for (int id : face) {
            REQUIRE(id >= 0);
            ++occurrences[id];
        }
    REQUIRE(occurrences.size() == 18);
    for (int id = 0; id < 18; ++id)
        REQUIRE(occurrences.count(id) == 1);
    for (const auto& [id, count] : occurrences) {
        (void) id;
        CHECK(count == 2);
    }
    bool callback_called    = false;
    const auto callback_ids = its_face_edge_ids(cube, [&callback_called] { callback_called = true; });
    REQUIRE(callback_called);
    REQUIRE(callback_ids == ids);
}

TEST_CASE("Masked and supplied-neighbor edge identifiers preserve unbound edges", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 2., 2.);
    std::vector<bool> mask(cube.indices.size(), false);
    mask[0]           = true;
    mask[1]           = true;
    const auto masked = its_face_edge_ids(cube, mask);
    REQUIRE(masked.size() == cube.indices.size());
    int selected_zero_edges = 0;
    int selected_one_edges  = 0;
    for (int edge = 0; edge < 3; ++edge) {
        if (masked[0][edge] >= 0)
            ++selected_zero_edges;
        if (masked[1][edge] >= 0)
            ++selected_one_edges;
        CHECK(masked[0][edge] >= 0);
        CHECK(masked[1][edge] >= 0);
    }
    CHECK(selected_zero_edges == 3);
    CHECK(selected_one_edges == 3);
    for (size_t face = 2; face < masked.size(); ++face)
        for (int edge = 0; edge < 3; ++edge)
            CHECK(masked[face][edge] == -1);
    int shared_id      = -1;
    int shared_matches = 0;
    for (int edge0 = 0; edge0 < 3; ++edge0)
        for (int edge1 = 0; edge1 < 3; ++edge1)
            if (masked[0][edge0] == masked[1][edge1]) {
                shared_id = masked[0][edge0];
                ++shared_matches;
            }
    CHECK(shared_matches == 1);
    REQUIRE(shared_id >= 0);
    auto neighbors      = its_face_neighbors(cube);
    int num_edges       = 0;
    const auto supplied = its_face_edge_ids(cube, neighbors, true, &num_edges);
    REQUIRE(supplied.size() == cube.indices.size());
    REQUIRE(num_edges == 18);
    for (const Vec3i32& face : supplied)
        for (int id : face)
            REQUIRE(id >= 0);
}

TEST_CASE("Disconnected indexed meshes report patches and preserve additive volume", "[TriangleMesh][TriangleMeshContracts]")
{
    auto left  = its_make_cube(2., 2., 2.);
    auto right = its_make_cube(2., 2., 2.);
    its_transform(left, identity3f().translate(Vec3f(-3.f, 0.f, 0.f)));
    its_transform(right, identity3f().translate(Vec3f(3.f, 0.f, 0.f)));
    const float expected = its_volume(left) + its_volume(right);
    its_merge(left, right);
    REQUIRE(its_number_of_patches(left) == 2);
    REQUIRE(its_is_splittable(left));
    CHECK_THAT(its_volume(left), WithinAbs(expected, 1e-4f));
    const auto parts = its_split(left);
    REQUIRE(parts.size() == 2);
    CHECK_THAT(its_volume(parts[0]) + its_volume(parts[1]), WithinAbs(expected, 1e-4f));
}

TEST_CASE("Open and closed indexed meshes expose their boundary edges", "[TriangleMesh][TriangleMeshContracts]")
{
    auto closed = its_make_cube(2., 2., 2.);
    REQUIRE(its_num_open_edges(closed) == 0);
    closed.indices.pop_back();
    REQUIRE(its_num_open_edges(closed) > 0);
    REQUIRE(its_number_of_patches(closed) == 1);
    const auto neighbors = its_face_neighbors_par(closed);
    REQUIRE(neighbors.size() == closed.indices.size());
    REQUIRE(its_num_open_edges(neighbors) == its_num_open_edges(closed));
}

TEST_CASE("Indexed mesh cleanup removes duplicate and unused vertices without invalidating faces", "[TriangleMesh][TriangleMeshContracts]")
{
    indexed_triangle_set mesh = its_make_cube(1., 1., 1.);
    mesh.vertices.push_back(mesh.vertices.front());
    const size_t with_unused = mesh.vertices.size();
    mesh.indices.emplace_back(0, 0, 0);
    const int removed_faces = its_remove_degenerate_faces(mesh);
    REQUIRE(removed_faces == 1);
    const int removed_vertices = its_merge_vertices(mesh);
    REQUIRE(removed_vertices >= 1);
    REQUIRE(all_indices_valid(mesh));
    its_shrink_to_fit(mesh);
    REQUIRE(its_compactify_vertices(mesh) == 0);
    REQUIRE(mesh.vertices.size() < with_unused);
    CHECK(its_num_open_edges(mesh) == 0);
}

TEST_CASE("Vertex face incidence counts every cube corner consistently", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    VertexFaceIndex index(cube);
    size_t total = 0;
    for (size_t i = 0; i < cube.vertices.size(); ++i) {
        REQUIRE(index.count(i) > 0);
        total += index.count(i);
        CHECK(std::distance(index.begin(i), index.end(i)) == static_cast<std::ptrdiff_t>(index.count(i)));
    }
    REQUIRE(total == cube.indices.size() * 3);
}

TEST_CASE("Reversing all facets changes signed volume but not geometry", "[TriangleMesh][TriangleMeshContracts]")
{
    indexed_triangle_set mesh  = its_make_cube(2., 3., 4.);
    const BoundingBoxf3 before = bounding_box(mesh);
    const float volume         = its_volume(mesh);
    REQUIRE(volume > 0.f);
    its_flip_triangles(mesh);
    CHECK_THAT(its_volume(mesh), WithinAbs(-volume, 1e-5f));
    REQUIRE(bounding_box(mesh).min.isApprox(before.min, 1e-6));
    REQUIRE(bounding_box(mesh).max.isApprox(before.max, 1e-6));
}

TEST_CASE("Average edge length follows the cube's explicit triangle edge sum", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    const double expected_sum = 2. * 2. * (2. + 3. + std::sqrt(13.)) + 2. * 2. * (2. + 4. + std::sqrt(20.)) + 2. * 2. * (3. + 4. + 5.);
    CHECK_THAT(its_average_edge_length(cube), WithinAbs(float(expected_sum / 36.), 1e-5f));
    CHECK_THAT(its_average_edge_length(indexed_triangle_set{}), WithinAbs(0.f, 1e-7f));
}

TEST_CASE("Projected-above helpers include clipped triangle intersections", "[TriangleMesh][TriangleMeshContracts]")
{
    // Only the apex lies above z = 5, so the footprint above that plane comes
    // from the edges clipped there: the right triangle (0, 0), (5, 0), (0, 5).
    const indexed_triangle_set tetrahedron{{{0, 2, 1}, {0, 1, 3}, {1, 2, 3}, {0, 3, 2}},
                                           {{0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, {0.f, 10.f, 0.f}, {0.f, 0.f, 10.f}}};
    Points points_matrix;
    Points points_transform;
    its_collect_mesh_projection_points_above(tetrahedron, Matrix3f::Identity(), 5.f, points_matrix);
    its_collect_mesh_projection_points_above(tetrahedron, Transform3f::Identity(), 5.f, points_transform);
    REQUIRE(points_matrix.size() == points_transform.size());
    REQUIRE(points_matrix.size() > 3);
    const Polygon matrix_hull    = its_convex_hull_2d_above(tetrahedron, Matrix3f::Identity(), 5.f);
    const Polygon transform_hull = its_convex_hull_2d_above(tetrahedron, Transform3f::Identity(), 5.f);
    REQUIRE(matrix_hull.size() == 3);
    REQUIRE(transform_hull.size() == matrix_hull.size());
    CHECK_THAT(matrix_hull.area(), WithinAbs(12.5 / (SCALING_FACTOR * SCALING_FACTOR), 1e-4));
    CHECK_THAT(transform_hull.area(), WithinAbs(matrix_hull.area(), 1e-4));
}

TEST_CASE("Convex hull construction encloses a translated tetrahedron", "[TriangleMesh][TriangleMeshContracts]")
{
    const std::vector<Vec3f> points = {{4.f, 5.f, 6.f},  {14.f, 5.f, 6.f}, {4.f, 15.f, 6.f},
                                       {4.f, 5.f, 16.f}, {8.f, 8.f, 8.f},  {6.f, 7.f, 9.f}};
    const indexed_triangle_set hull = its_convex_hull(points);
    REQUIRE(hull.vertices.size() == 4);
    REQUIRE(hull.indices.size() == 4);
    CHECK_THAT(its_volume(hull), WithinAbs(1000.f / 6.f, 1e-3f));
    const TriangleMesh hull_mesh(hull);
    CHECK(hull_mesh.stats().open_edges == 0);
}

TEST_CASE("Empty indexed mesh mass properties and normals have neutral values", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set empty;
    const MassProperties properties = its_mass_properties(empty);
    CHECK_THAT(properties.mass, WithinAbs(0., 1e-12));
    CHECK_THAT(properties.volume, WithinAbs(0., 1e-12));
    REQUIRE(properties.center.isApprox(Vec3d::Zero(), 1e-12));
    REQUIRE(properties.spread.isApprox(Matrix3d::Zero(), 1e-12));
    REQUIRE(its_face_normals(empty).empty());
}

TEST_CASE("Indexed mesh OBJ subset writers preserve requested vertices and faces", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 2., 2.);
    ScopedTemporaryFile one(".obj");
    ScopedTemporaryFile many(".obj");
    REQUIRE(its_store_triangle(cube, one.string().c_str(), 0));
    REQUIRE(its_store_triangles(cube, many.string().c_str(), {0, 3, 7}));
    std::ifstream one_stream(one.string());
    std::ifstream many_stream(many.string());
    REQUIRE(one_stream.good());
    REQUIRE(many_stream.good());
    const std::string one_text((std::istreambuf_iterator<char>(one_stream)), {});
    const std::string many_text((std::istreambuf_iterator<char>(many_stream)), {});
    std::vector<Vec3f> one_vertices;
    std::vector<Vec3i32> one_faces;
    parse_obj(one_text, one_vertices, one_faces);
    REQUIRE(one_vertices.size() == 3);
    REQUIRE(one_faces.size() == 1);
    const Vec3i32 source_face = cube.indices[0];
    for (int corner = 0; corner < 3; ++corner) {
        CHECK(one_faces[0][corner] == corner);
        const Vec3f& expected = cube.vertices[source_face[corner]];
        CHECK_THAT(one_vertices[corner].x(), WithinAbs(expected.x(), 1e-5));
        CHECK_THAT(one_vertices[corner].y(), WithinAbs(expected.y(), 1e-5));
        CHECK_THAT(one_vertices[corner].z(), WithinAbs(expected.z(), 1e-5));
    }
    std::vector<Vec3f> many_vertices;
    std::vector<Vec3i32> many_faces;
    parse_obj(many_text, many_vertices, many_faces);
    const std::vector<size_t> requested = {0, 3, 7};
    std::vector<int> source_to_output(cube.vertices.size(), -1);
    std::vector<Vec3f> expected_vertices;
    for (size_t triangle : requested)
        for (int corner = 0; corner < 3; ++corner) {
            const int source_index = cube.indices[triangle][corner];
            if (source_to_output[source_index] == -1) {
                source_to_output[source_index] = int(expected_vertices.size());
                expected_vertices.push_back(cube.vertices[source_index]);
            }
        }
    REQUIRE(many_vertices.size() == expected_vertices.size());
    REQUIRE(many_faces.size() == requested.size());
    for (size_t vertex = 0; vertex < expected_vertices.size(); ++vertex) {
        CHECK_THAT(many_vertices[vertex].x(), WithinAbs(expected_vertices[vertex].x(), 1e-5));
        CHECK_THAT(many_vertices[vertex].y(), WithinAbs(expected_vertices[vertex].y(), 1e-5));
        CHECK_THAT(many_vertices[vertex].z(), WithinAbs(expected_vertices[vertex].z(), 1e-5));
    }
    for (size_t face = 0; face < requested.size(); ++face)
        for (int corner = 0; corner < 3; ++corner)
            CHECK(many_faces[face][corner] == source_to_output[cube.indices[requested[face]][corner]]);
    REQUIRE_FALSE(its_store_triangle(cube, one.string().c_str(), cube.indices.size()));
    REQUIRE_FALSE(its_store_triangles(cube, many.string().c_str(), {cube.indices.size()}));
    std::ifstream one_after_stream(one.string());
    REQUIRE(one_after_stream.good());
    const std::string one_after((std::istreambuf_iterator<char>(one_after_stream)), {});
    CHECK(one_after == one_text);
}

TEST_CASE("Face normals remain unit length with two facets on every signed cube axis", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 2., 2.);
    const auto normals              = its_face_normals(cube);
    REQUIRE(normals.size() == cube.indices.size());
    std::array<int, 6> signed_axis_counts = {0, 0, 0, 0, 0, 0};
    for (const Vec3f& normal : normals) {
        CHECK_THAT(normal.norm(), WithinAbs(1.f, 1e-5f));
        int axis = -1;
        if (normal.x() > .99f)
            axis = 0;
        else if (normal.x() < -.99f)
            axis = 1;
        else if (normal.y() > .99f)
            axis = 2;
        else if (normal.y() < -.99f)
            axis = 3;
        else if (normal.z() > .99f)
            axis = 4;
        else if (normal.z() < -.99f)
            axis = 5;
        REQUIRE(axis >= 0);
        ++signed_axis_counts[size_t(axis)];
    }
    for (int count : signed_axis_counts)
        CHECK(count == 2);
}
