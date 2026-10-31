#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Geometry.hpp"
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

void require_finite_box(const BoundingBoxf3& box)
{
    REQUIRE(box.min.array().isFinite().all());
    REQUIRE(box.max.array().isFinite().all());
    REQUIRE(box.max.x() >= box.min.x());
    REQUIRE(box.max.y() >= box.min.y());
    REQUIRE(box.max.z() >= box.min.z());
}

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

// A face index paired with the index of one of its edges; edge e runs from corner e to corner e + 1.
using FaceEdge = std::pair<size_t, int>;

std::map<int, std::vector<FaceEdge>> edge_id_occurrences(const std::vector<Vec3i32>& ids)
{
    std::map<int, std::vector<FaceEdge>> occurrences;
    for (size_t face = 0; face < ids.size(); ++face)
        for (int edge = 0; edge < 3; ++edge)
            if (ids[face][edge] >= 0)
                occurrences[ids[face][edge]].emplace_back(face, edge);
    return occurrences;
}

// True when the two face edges belong to different faces and traverse the same two vertices in opposite directions.
bool is_reverse_edge_pair(const indexed_triangle_set& its, const FaceEdge& a, const FaceEdge& b)
{
    if (a.first == b.first)
        return false;
    const Vec3i32& fa = its.indices[a.first];
    const Vec3i32& fb = its.indices[b.first];
    return fa[a.second] == fb[(b.second + 1) % 3] && fa[(a.second + 1) % 3] == fb[b.second];
}

} // namespace

TEST_CASE("TriangleMesh keeps topology and statistics for a translated solid", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh = make_cube(2., 3., 4.);
    REQUIRE(mesh.stats().number_of_facets == 12);
    REQUIRE(mesh.stats().number_of_parts == 1);
    REQUIRE(mesh.stats().open_edges == 0);
    REQUIRE_FALSE(mesh.stats().repaired());
    CHECK_THAT(mesh.volume(), WithinAbs(24.f, 1e-5f));
    CHECK_THAT(mesh.size().x(), WithinAbs(2., 1e-6));
    CHECK_THAT(mesh.size().y(), WithinAbs(3., 1e-6));
    CHECK_THAT(mesh.size().z(), WithinAbs(4., 1e-6));
    mesh.translate(Vec3f(10.f, -2.f, 7.f));
    REQUIRE(mesh.bounding_box().min.isApprox(Vec3d(10., -2., 7.), 1e-6));
    REQUIRE(mesh.bounding_box().max.isApprox(Vec3d(12., 1., 11.), 1e-6));
    CHECK_THAT(mesh.volume(), WithinAbs(24.f, 1e-5f));
}

TEST_CASE("TriangleMesh affine and mirror operations preserve their stated orientation semantics", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh scaled = make_cube(2., 3., 4.);
    scaled.scale(Vec3f(2.f, 3.f, 4.f));
    CHECK_THAT(scaled.volume(), WithinAbs(576.f, 1e-3f));
    CHECK_THAT(scaled.size().x(), WithinAbs(4., 1e-5));
    CHECK_THAT(scaled.size().y(), WithinAbs(9., 1e-5));
    CHECK_THAT(scaled.size().z(), WithinAbs(16., 1e-5));

    TriangleMesh reflected = make_cube(2., 3., 4.);
    Transform3d reflection = Transform3d::Identity();
    reflection(0, 0)       = -1.;
    reflected.transform(reflection, false);
    CHECK_THAT(reflected.volume(), WithinAbs(-24.f, 1e-4f));
    CHECK_THAT(its_volume(reflected.its), WithinAbs(-24.f, 1e-4f));
    TriangleMesh fixed = make_cube(2., 3., 4.);
    fixed.transform(reflection, true);
    CHECK_THAT(fixed.volume(), WithinAbs(24.f, 1e-4f));
    // The fix-up must reorient the facets themselves, not just the cached volume.
    CHECK_THAT(its_volume(fixed.its), WithinAbs(24.f, 1e-4f));
    reflected.mirror_y();
    CHECK_THAT(its_volume(reflected.its), WithinAbs(-24.f, 1e-4f));
    REQUIRE(reflected.bounding_box().min.y() < 0.);
}

TEST_CASE("TriangleMesh direct mirror and facet flip update orientation and bounds", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mirrored = make_cube(2., 3., 4.);
    mirrored.mirror_x();
    REQUIRE(mirrored.bounding_box().min.x() < 0.);
    CHECK_THAT(mirrored.bounding_box().max.x(), WithinAbs(0., 1e-6));
    CHECK_THAT(its_volume(mirrored.its), WithinAbs(24.f, 1e-4f));

    TriangleMesh flipped = make_cube(2., 3., 4.);
    const float original = flipped.volume();
    flipped.flip_triangles();
    CHECK_THAT(flipped.volume(), WithinAbs(-original, 1e-5f));
    CHECK_THAT(its_volume(flipped.its), WithinAbs(-original, 1e-5f));
}

TEST_CASE("TriangleMesh merging combines component statistics and geometry", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh first  = make_cube(2., 2., 2.);
    TriangleMesh second = make_cube(2., 2., 2.);
    second.translate(3.f, 0.f, 0.f);
    first.merge(second);
    REQUIRE(first.stats().number_of_parts == 2);
    REQUIRE(first.stats().number_of_facets == 24);
    CHECK_THAT(first.volume(), WithinAbs(16.f, 1e-4f));
    CHECK_THAT(first.bounding_box().max.x(), WithinAbs(5., 1e-5));
    // The geometry itself holds both cubes, not just the merged statistics.
    REQUIRE(first.its.vertices.size() == 16);
    REQUIRE(first.its.indices.size() == 24);
    CHECK_THAT(its_volume(first.its), WithinAbs(16.f, 1e-4f));
    const BoundingBoxf3 merged = bounding_box(first.its);
    CHECK_THAT(merged.min.x(), WithinAbs(0., 1e-5));
    CHECK_THAT(merged.max.x(), WithinAbs(5., 1e-5));
    CHECK(its_number_of_patches(first.its) == 2);
}

TEST_CASE("TriangleMesh rotates around a 2D center in mesh coordinates", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh = make_cube(20., 10., 2.);
    // TriangleMesh coordinates are millimetres; this overload consumes the
    // center in the same unscaled coordinate system as its vertices.
    Point center(10, 5);
    // This legacy overload is expressed in degrees, matching ADMesh's
    // its_rotate_z routine (the Axis overload below takes radians).
    mesh.rotate(90., &center);
    // Check the vertices independently of the cached-bounds regression below.
    const BoundingBoxf3 box = bounding_box(mesh.its);
    CHECK_THAT(box.min.x(), WithinAbs(5., 1e-4));
    CHECK_THAT(box.max.x(), WithinAbs(15., 1e-4));
    CHECK_THAT(box.min.y(), WithinAbs(-5., 1e-4));
    CHECK_THAT(box.max.y(), WithinAbs(15., 1e-4));
    CHECK_THAT(mesh.volume(), WithinAbs(400.f, 1e-3f));
}

TEST_CASE("TriangleMesh transformed bounding box clips triangles at a world plane", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh           = make_cube(20., 10., 20.);
    const Transform3d translate = Geometry::translation_transform(Vec3d(2., -3., 4.));
    const BoundingBoxf3 full    = mesh.transformed_bounding_box(translate);
    REQUIRE(full.min.isApprox(Vec3d(2., -3., 4.), 1e-6));
    REQUIRE(full.max.isApprox(Vec3d(22., 7., 24.), 1e-6));
    const BoundingBoxf3 upper = mesh.transformed_bounding_box(translate, 10.);
    CHECK_THAT(upper.min.x(), WithinAbs(2., 1e-6));
    CHECK_THAT(upper.min.y(), WithinAbs(-3., 1e-6));
    CHECK_THAT(upper.min.z(), WithinAbs(10., 1e-6));
    CHECK_THAT(upper.max.x(), WithinAbs(22., 1e-6));
    CHECK_THAT(upper.max.y(), WithinAbs(7., 1e-6));
    CHECK_THAT(upper.max.z(), WithinAbs(24., 1e-6));
}

TEST_CASE("TriangleMesh axis rotation uses radians and refreshes bounds", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh = make_cube(20., 10., 2.);
    mesh.rotate(float(PI / 2.), Z);
    const BoundingBoxf3 box = mesh.bounding_box();
    CHECK_THAT(box.size().x(), WithinAbs(10., 1e-4));
    CHECK_THAT(box.size().y(), WithinAbs(20., 1e-4));
    CHECK_THAT(mesh.volume(), WithinAbs(400.f, 1e-3f));
}

TEST_CASE("TriangleMesh projections expose the solid footprint", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh           = make_cube(20., 10., 5.);
    const ExPolygons projection = mesh.horizontal_projection();
    REQUIRE(projection.size() == 1);
    CHECK_THAT(projection.front().area(), WithinAbs(200. / (SCALING_FACTOR * SCALING_FACTOR), 1e-4));
    const Polygon hull = mesh.convex_hull();
    REQUIRE(hull.size() == 4);
    CHECK_THAT(hull.area(), WithinAbs(200. / (SCALING_FACTOR * SCALING_FACTOR), 1e-4));
}

TEST_CASE("Indexed mesh edge identifiers pair every cube edge", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set cube = its_make_cube(2., 3., 4.);
    const auto ids                  = its_face_edge_ids(cube);
    REQUIRE(ids.size() == cube.indices.size());
    for (const Vec3i32& face : ids)
        for (int id : face)
            REQUIRE(id >= 0);
    const auto occurrences = edge_id_occurrences(ids);
    REQUIRE(occurrences.size() == 18);
    int expected_id = 0;
    for (const auto& [id, uses] : occurrences) {
        CHECK(id == expected_id++);
        REQUIRE(uses.size() == 2);
        CHECK(is_reverse_edge_pair(cube, uses[0], uses[1]));
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
    for (int edge = 0; edge < 3; ++edge) {
        CHECK(masked[0][edge] >= 0);
        CHECK(masked[1][edge] >= 0);
    }
    for (size_t face = 2; face < masked.size(); ++face)
        for (int edge = 0; edge < 3; ++edge)
            CHECK(masked[face][edge] == -1);
    // The two masked faces share exactly one edge, and it gets one ID on both sides.
    const auto masked_occurrences = edge_id_occurrences(masked);
    REQUIRE(masked_occurrences.size() == 5);
    int shared_ids = 0;
    for (const auto& [id, uses] : masked_occurrences) {
        (void) id;
        if (uses.size() == 2) {
            ++shared_ids;
            CHECK(is_reverse_edge_pair(cube, uses[0], uses[1]));
        }
    }
    CHECK(shared_ids == 1);

    // Removing one face opens three boundary edges whose supplied neighbor is -1.
    indexed_triangle_set open = its_make_cube(2., 2., 2.);
    open.indices.pop_back();
    auto neighbors = its_face_neighbors(open);
    int num_edges  = 0;
    const auto bound_only = its_face_edge_ids(open, neighbors, false, &num_edges);
    REQUIRE(bound_only.size() == open.indices.size());
    CHECK(num_edges == 15);
    std::vector<FaceEdge> unbound;
    for (size_t face = 0; face < bound_only.size(); ++face)
        for (int edge = 0; edge < 3; ++edge)
            if (bound_only[face][edge] == -1) {
                CHECK(neighbors[face][edge] == -1);
                unbound.emplace_back(face, edge);
            }
    REQUIRE(unbound.size() == 3);
    for (const auto& [id, uses] : edge_id_occurrences(bound_only)) {
        (void) id;
        REQUIRE(uses.size() == 2);
        CHECK(is_reverse_edge_pair(open, uses[0], uses[1]));
    }

    const auto assigned = its_face_edge_ids(open, neighbors, true, &num_edges);
    CHECK(num_edges == 18);
    std::vector<FaceEdge> single_use;
    for (const auto& [id, uses] : edge_id_occurrences(assigned)) {
        CHECK(id < num_edges);
        if (uses.size() == 1)
            single_use.push_back(uses.front());
        else
            CHECK(is_reverse_edge_pair(open, uses[0], uses[1]));
    }
    std::sort(single_use.begin(), single_use.end());
    CHECK(single_use == unbound);
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
    REQUIRE(its_num_open_edges(closed) == 3);
    REQUIRE(its_number_of_patches(closed) == 1);
    const auto neighbors = its_face_neighbors_par(closed);
    REQUIRE(neighbors.size() == closed.indices.size());
    REQUIRE(its_num_open_edges(neighbors) == its_num_open_edges(closed));
}

TEST_CASE("Indexed mesh cleanup removes duplicate and unused vertices without invalidating faces", "[TriangleMesh][TriangleMeshContracts]")
{
    indexed_triangle_set mesh  = its_make_cube(1., 1., 1.);
    const size_t cube_vertices = mesh.vertices.size();
    // Re-point one corner of one face at a copy of vertex 0, which opens the surface along that corner.
    const int duplicate = int(mesh.vertices.size());
    mesh.vertices.push_back(mesh.vertices[0]);
    size_t remapped_face   = 0;
    int    remapped_corner = -1;
    for (size_t face = 0; face < mesh.indices.size() && remapped_corner < 0; ++face)
        for (int corner = 0; corner < 3; ++corner)
            if (mesh.indices[face][corner] == 0) {
                mesh.indices[face][corner] = duplicate;
                remapped_face              = face;
                remapped_corner            = corner;
                break;
            }
    REQUIRE(remapped_corner >= 0);
    REQUIRE(its_num_open_edges(mesh) > 0);
    // A distinct vertex that no face references.
    mesh.vertices.emplace_back(5.f, 5.f, 5.f);
    mesh.indices.emplace_back(0, 0, 1);

    REQUIRE(its_remove_degenerate_faces(mesh) == 1);
    REQUIRE(its_merge_vertices(mesh) == 1);
    CHECK(mesh.indices[remapped_face][remapped_corner] == 0);
    REQUIRE(all_indices_valid(mesh));
    CHECK(its_num_open_edges(mesh) == 0);
    REQUIRE(mesh.vertices.size() == cube_vertices + 1);

    REQUIRE(its_compactify_vertices(mesh) == 1);
    REQUIRE(mesh.vertices.size() == cube_vertices);
    REQUIRE(all_indices_valid(mesh));
    CHECK(its_num_open_edges(mesh) == 0);
    CHECK_THAT(its_volume(mesh), WithinAbs(1.f, 1e-5f));
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
    // The hull vertices are exactly the four translated corners; the interior points are dropped.
    for (size_t corner = 0; corner < 4; ++corner) {
        const Vec3f& expected = points[corner];
        CHECK(std::count_if(hull.vertices.begin(), hull.vertices.end(),
                            [&expected](const Vec3f& vertex) { return (vertex - expected).norm() < 1e-5f; }) == 1);
    }
    const TriangleMesh hull_mesh(hull);
    CHECK(hull_mesh.stats().open_edges == 0);
}

TEST_CASE("TriangleMesh split normalizes a reversed component", "[TriangleMesh][TriangleMeshContracts]")
{
    indexed_triangle_set reversed = its_make_cube(2., 2., 2.);
    its_flip_triangles(reversed);
    REQUIRE(its_volume(reversed) < 0.f);
    TriangleMesh mesh(std::move(reversed));
    auto parts = mesh.split();
    REQUIRE(parts.size() == 1);
    CHECK_THAT(parts.front().volume(), WithinAbs(8.f, 1e-4f));
    // The facets themselves must be reoriented, not just the cached volume.
    CHECK_THAT(its_volume(parts.front().its), WithinAbs(8.f, 1e-4f));
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

TEST_CASE("TriangleMesh STL writers round trip ASCII and binary geometry", "[TriangleMesh][TriangleMeshContracts]")
{
    const TriangleMesh source = make_cube(3., 4., 5.);
    ScopedTemporaryFile ascii(".stl");
    ScopedTemporaryFile binary(".stl");
    REQUIRE(source.write_ascii(ascii.string().c_str()));
    REQUIRE(source.write_binary(binary.string().c_str()));
    TriangleMesh ascii_mesh;
    TriangleMesh binary_mesh;
    REQUIRE(ascii_mesh.ReadSTLFile(ascii.string().c_str()));
    REQUIRE(binary_mesh.ReadSTLFile(binary.string().c_str()));
    const float source_volume = its_volume(source.its);
    CHECK_THAT(ascii_mesh.volume(), WithinAbs(source_volume, 0.01f));
    CHECK_THAT(binary_mesh.volume(), WithinAbs(source_volume, 0.01f));
    REQUIRE(ascii_mesh.stats().open_edges == 0);
    REQUIRE(binary_mesh.stats().open_edges == 0);
    CHECK_THAT(ascii_mesh.bounding_box().size().x(), WithinAbs(3., 1e-4));
    CHECK_THAT(binary_mesh.bounding_box().size().z(), WithinAbs(5., 1e-4));
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

TEST_CASE("TriangleMesh transformed bounding box applies rotation before translation", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh           = make_cube(2., 4., 6.);
    const Transform3d transform = Geometry::translation_transform(Vec3d(-10., 20., 30.)) *
                                  Geometry::rotation_transform(Vec3d(0., 0., PI / 2.));
    const BoundingBoxf3 transformed = mesh.transformed_bounding_box(transform);
    require_finite_box(transformed);
    CHECK_THAT(transformed.size().x(), WithinAbs(4., 1e-6));
    CHECK_THAT(transformed.size().y(), WithinAbs(2., 1e-6));
    CHECK_THAT(transformed.size().z(), WithinAbs(6., 1e-6));
    CHECK_THAT(transformed.min.x(), WithinAbs(-14., 1e-6));
    CHECK_THAT(transformed.max.y(), WithinAbs(22., 1e-6));
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

TEST_CASE("Empty TriangleMesh operations keep the mesh empty", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh;
    REQUIRE(mesh.empty());
    CHECK_THAT(mesh.volume(), WithinAbs(0.f, 1e-7f));
    REQUIRE(mesh.split().empty());
    REQUIRE(mesh.horizontal_projection().empty());
    REQUIRE(mesh.convex_hull().empty());
    REQUIRE(mesh.convex_hull_3d().empty());
    mesh.align_to_origin();
    mesh.translate(1.f, 2.f, 3.f);
    mesh.rotate_z(float(PI / 2.));
    mesh.scale(2.f);
    REQUIRE(mesh.empty());
    CHECK_THAT(mesh.volume(), WithinAbs(0.f, 1e-7f));
}

// Known bug: TriangleMesh::rotate(double, Point*) leaves the cached bounds stale; fixed by the follow-up fix PR
TEST_CASE("Legacy center rotation keeps mesh bounds and size synchronized",
          "[TriangleMesh][TriangleMeshContracts][TriangleMeshRotateCache][Regression][!shouldfail]")
{
    TriangleMesh mesh = make_cube(20., 10., 2.);
    Point center(10, 5);

    mesh.rotate(90., &center);

    const BoundingBoxf3 actual = bounding_box(mesh.its);
    const BoundingBoxf3 cached = mesh.bounding_box();
    CHECK_THAT(cached.min.x(), WithinAbs(actual.min.x(), 1e-4));
    CHECK_THAT(cached.min.y(), WithinAbs(actual.min.y(), 1e-4));
    CHECK_THAT(cached.max.x(), WithinAbs(actual.max.x(), 1e-4));
    CHECK_THAT(cached.max.y(), WithinAbs(actual.max.y(), 1e-4));
    CHECK_THAT(mesh.size().x(), WithinAbs(actual.size().x(), 1e-4));
    CHECK_THAT(mesh.size().y(), WithinAbs(actual.size().y(), 1e-4));
}
