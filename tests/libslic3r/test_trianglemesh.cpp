#include <cstddef>
#include <utility>

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
