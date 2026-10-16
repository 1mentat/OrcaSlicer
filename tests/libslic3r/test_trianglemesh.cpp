#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <functional>
#include <iterator>
#include <map>
#include <numeric>
#include <set>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
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
using Catch::Matchers::WithinRel;

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

bool every_edge_has_two_incidents(const indexed_triangle_set& mesh)
{
    std::map<std::pair<int, int>, int> edge_counts;
    for (const Vec3i32& face : mesh.indices)
        for (int corner = 0; corner < 3; ++corner) {
            int first  = face[corner];
            int second = face[(corner + 1) % 3];
            if (first > second)
                std::swap(first, second);
            ++edge_counts[{first, second}];
        }
    if (edge_counts.empty())
        return false;
    for (const auto& [edge, count] : edge_counts) {
        (void) edge;
        if (count != 2)
            return false;
    }
    return true;
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
    const size_t before = mesh.memsize();
    REQUIRE(before >= sizeof(TriangleMesh));
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
    TriangleMesh fixed = make_cube(2., 3., 4.);
    fixed.transform(reflection, true);
    CHECK_THAT(fixed.volume(), WithinAbs(24.f, 1e-4f));
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
}

TEST_CASE("TriangleMesh rotates around a 2D center in mesh coordinates", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh = make_cube(20., 10., 2.);
    // TriangleMesh coordinates are millimetres; this overload consumes the
    // center in the same unscaled coordinate system as its vertices.
    Point center(10, 5);
    // This legacy overload is expressed in degrees, matching ADMesh's
    // its_rotate_z routine (the Axis overload above takes radians).
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

TEST_CASE("Primitive generators preserve analytic bounds and positive solids", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set frustum = its_make_frustum(6., 8., PI / 12.);
    const BoundingBoxf3 frustum_box    = bounding_box(frustum);
    CHECK_THAT(frustum_box.size().z(), WithinAbs(8., 1e-5));
    CHECK_THAT(its_volume(frustum), WithinAbs(float((7. / 12.) * PI * 36. * 8.), 15.f));
    const indexed_triangle_set torus = its_make_torus(8., 2., PI / 8.);
    const BoundingBoxf3 torus_box    = bounding_box(torus);
    CHECK_THAT(torus_box.size().x(), WithinAbs(20., 0.2));
    CHECK_THAT(torus_box.size().y(), WithinAbs(20., 0.2));
    CHECK_THAT(torus_box.size().z(), WithinAbs(4., 0.2));
    CHECK(its_volume(torus) > 0.f);
}

TEST_CASE("Specialized primitive generators produce closed positive solids with analytic bounds", "[TriangleMesh][TriangleMeshContracts]")
{
    const indexed_triangle_set dowel = its_make_frustum_dowel(3., 8., 16);
    REQUIRE_FALSE(dowel.vertices.empty());
    REQUIRE_FALSE(dowel.indices.empty());
    REQUIRE(all_indices_valid(dowel));
    REQUIRE(every_edge_has_two_incidents(dowel));
    CHECK(its_volume(dowel) > 0.f);
    const BoundingBoxf3 dowel_box = bounding_box(dowel);
    CHECK_THAT(dowel_box.min.x(), WithinAbs(-3., 1e-4));
    CHECK_THAT(dowel_box.max.x(), WithinAbs(3., 1e-4));
    CHECK_THAT(dowel_box.min.y(), WithinAbs(-3., 1e-4));
    CHECK_THAT(dowel_box.max.y(), WithinAbs(3., 1e-4));
    CHECK_THAT(dowel_box.min.z(), WithinAbs(-8., 1e-4));
    CHECK_THAT(dowel_box.max.z(), WithinAbs(8., 1e-4));

    const indexed_triangle_set pyramid = its_make_pyramid(6.f, 9.f);
    REQUIRE(all_indices_valid(pyramid));
    REQUIRE(every_edge_has_two_incidents(pyramid));
    CHECK_THAT(its_volume(pyramid), WithinAbs(108.f, 1e-4));
    const BoundingBoxf3 pyramid_box = bounding_box(pyramid);
    CHECK_THAT(pyramid_box.min.x(), WithinAbs(-3., 1e-4));
    CHECK_THAT(pyramid_box.max.x(), WithinAbs(3., 1e-4));
    CHECK_THAT(pyramid_box.min.y(), WithinAbs(-3., 1e-4));
    CHECK_THAT(pyramid_box.max.y(), WithinAbs(3., 1e-4));
    CHECK_THAT(pyramid_box.min.z(), WithinAbs(0., 1e-4));
    CHECK_THAT(pyramid_box.max.z(), WithinAbs(9., 1e-4));

    const indexed_triangle_set snap = its_make_snap(4., 8.);
    REQUIRE(all_indices_valid(snap));
    REQUIRE(every_edge_has_two_incidents(snap));
    CHECK(its_volume(snap) > 0.f);
    const BoundingBoxf3 snap_box = bounding_box(snap);
    CHECK_THAT(snap_box.min.z(), WithinAbs(0., 1e-4));
    CHECK_THAT(snap_box.max.z(), WithinAbs(8., 1e-4));
    CHECK_THAT(snap_box.min.x(), WithinAbs(-snap_box.max.x(), 1e-4));
    CHECK_THAT(snap_box.min.y(), WithinAbs(-snap_box.max.y(), 1e-4));
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
    const indexed_triangle_set cube = its_make_cube(10., 8., 10.);
    Points points_matrix;
    Points points_transform;
    its_collect_mesh_projection_points_above(cube, Matrix3f::Identity(), 5.f, points_matrix);
    its_collect_mesh_projection_points_above(cube, Transform3f::Identity(), 5.f, points_transform);
    REQUIRE(points_matrix.size() == points_transform.size());
    REQUIRE(points_matrix.size() > 4);
    const Polygon matrix_hull    = its_convex_hull_2d_above(cube, Matrix3f::Identity(), 5.f);
    const Polygon transform_hull = its_convex_hull_2d_above(cube, Transform3f::Identity(), 5.f);
    REQUIRE(matrix_hull.size() >= 4);
    REQUIRE(transform_hull.size() == matrix_hull.size());
    CHECK_THAT(matrix_hull.area(), WithinAbs(80. / (SCALING_FACTOR * SCALING_FACTOR), 1e-4));
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

TEST_CASE("TriangleMesh split normalizes a reversed component", "[TriangleMesh][TriangleMeshContracts]")
{
    indexed_triangle_set reversed = its_make_cube(2., 2., 2.);
    its_flip_triangles(reversed);
    REQUIRE(its_volume(reversed) < 0.f);
    TriangleMesh mesh(std::move(reversed));
    auto parts = mesh.split();
    REQUIRE(parts.size() == 1);
    CHECK(parts.front().volume() > 0.f);
    CHECK_THAT(parts.front().volume(), WithinAbs(8.f, 1e-4f));
}

TEST_CASE("TriangleMesh empty mass properties and normals have neutral values", "[TriangleMesh][TriangleMeshContracts]")
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

TEST_CASE("TriangleMesh keeps transformed bounding-box geometry independent of cached stats", "[TriangleMesh][TriangleMeshContracts]")
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

TEST_CASE("Empty TriangleMesh operations remain empty and finite", "[TriangleMesh][TriangleMeshContracts]")
{
    TriangleMesh mesh;
    REQUIRE(mesh.empty());
    CHECK_THAT(mesh.volume(), WithinAbs(0.f, 1e-7f));
    REQUIRE(mesh.split().empty());
    REQUIRE(mesh.horizontal_projection().empty());
    REQUIRE(mesh.convex_hull().empty());
    REQUIRE(mesh.convex_hull_3d().empty());
    REQUIRE(mesh.memsize() > 0);
    mesh.align_to_origin();
    mesh.translate(0.f, 0.f, 0.f);
    mesh.rotate_z(0.f);
    REQUIRE(mesh.empty());
}

TEST_CASE("Legacy center rotation keeps mesh bounds and size synchronized",
          "[TriangleMesh][TriangleMeshContracts][TriangleMeshRotateCache][Regression]")
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

namespace pyramid_winding_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

TEST_CASE("Pyramid facets all face away from an interior point", "[TriangleMesh][PyramidWinding]")
{
    const auto mesh = its_make_pyramid(6.f, 9.f);
    const Vec3f interior(0.f, 0.f, 2.25f);
    REQUIRE(mesh.indices.size() == 6);
    for (const auto& face : mesh.indices) {
        for (int index : face) {
            REQUIRE(index >= 0);
            REQUIRE(static_cast<std::size_t>(index) < mesh.vertices.size());
        }
        const Vec3f a = mesh.vertices[face[0]], b = mesh.vertices[face[1]], c = mesh.vertices[face[2]];
        const Vec3f normal = (b - a).cross(c - a);
        CHECK(normal.dot((a + b + c) / 3.f - interior) > 0.f);
    }
}

TEST_CASE("Pyramid topology pairs every directed edge with its reverse", "[TriangleMesh][PyramidWinding]")
{
    const auto mesh = its_make_pyramid(6.f, 9.f);
    CHECK(its_num_open_edges(mesh) == 0);
    const auto adjacent = its_face_neighbors(mesh);
    REQUIRE(adjacent.size() == 6);
    for (const auto& neighbors : adjacent)
        for (int face : neighbors)
            CHECK(face >= 0);
}

TEST_CASE("Pyramid signed volume is invariant under translation", "[TriangleMesh][PyramidWinding]")
{
    const Vec3f displacement = GENERATE(Vec3f(0.f, 0.f, 7.f), Vec3f(0.f, 0.f, -7.f), Vec3f(11.f, -13.f, 5.f));
    auto mesh                = its_make_pyramid(6.f, 9.f);
    // One third of the 6 by 6 mm base times the 9 mm height.
    CHECK_THAT(its_volume(mesh), WithinAbs(108., 1e-4));
    for (auto& vertex : mesh.vertices)
        vertex += displacement;
    CHECK_THAT(its_volume(mesh), WithinAbs(108., 1e-3));
}

} // namespace pyramid_winding_contracts
