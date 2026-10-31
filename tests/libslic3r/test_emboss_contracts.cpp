#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <admesh/stl.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Emboss.hpp"

using namespace Slic3r;

namespace {

double projected_triangle_area(const Vec3f &a, const Vec3f &b, const Vec3f &c)
{
    const double ab_x = static_cast<double>(b.x()) - a.x();
    const double ab_y = static_cast<double>(b.y()) - a.y();
    const double ac_x = static_cast<double>(c.x()) - a.x();
    const double ac_y = static_cast<double>(c.y()) - a.y();
    return std::abs(ab_x * ac_y - ab_y * ac_x) * 0.5;
}

struct MeshSummary {
    double front_area = 0.;
    double back_area = 0.;
    size_t side_faces = 0;
};

// Sums the XY area of the triangles lying on the front (z = 0) and back (z = back_z) planes; every other
// triangle is a side wall.
MeshSummary inspect_projected_shell(const indexed_triangle_set &mesh, double back_z)
{
    MeshSummary summary;
    for (const stl_triangle_vertex_indices &triangle : mesh.indices) {
        const Vec3f &a = mesh.vertices[triangle[0]];
        const Vec3f &b = mesh.vertices[triangle[1]];
        const Vec3f &c = mesh.vertices[triangle[2]];
        const auto on_plane = [&](double z) {
            return std::abs(a.z() - z) < 1e-5 && std::abs(b.z() - z) < 1e-5 && std::abs(c.z() - z) < 1e-5;
        };
        if (on_plane(0.))
            summary.front_area += projected_triangle_area(a, b, c);
        else if (on_plane(back_z))
            summary.back_area += projected_triangle_area(a, b, c);
        else
            ++summary.side_faces;
    }
    return summary;
}

ExPolygon rectangle_with_hole()
{
    return ExPolygon(
        Polygon {{0, 0}, {100, 0}, {100, 100}, {0, 100}},
        Polygon {{25, 25}, {25, 75}, {75, 75}, {75, 25}});
}

// The mesh holds each unique 2d point twice: first all front vertices, then the matching back vertices.
void check_surface_vertices(const indexed_triangle_set &mesh, double back_z, size_t expected_unique_points)
{
    REQUIRE(mesh.vertices.size() == expected_unique_points * 2);
    for (size_t i = 0; i < expected_unique_points; ++i) {
        CHECK_THAT(mesh.vertices[i].z(), Catch::Matchers::WithinAbs(0., 1e-5));
        CHECK_THAT(mesh.vertices[i + expected_unique_points].z(),
                   Catch::Matchers::WithinAbs(back_z, 1e-5));
        CHECK_THAT(mesh.vertices[i].x(),
                   Catch::Matchers::WithinAbs(mesh.vertices[i + expected_unique_points].x(), 1e-5));
        CHECK_THAT(mesh.vertices[i].y(),
                   Catch::Matchers::WithinAbs(mesh.vertices[i + expected_unique_points].y(), 1e-5));
    }
}

void same_vector(const Vec3d &actual, const Vec3d &expected)
{
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(actual[axis], Catch::Matchers::WithinAbs(expected[axis], 1e-6));
}

} // namespace

TEST_CASE("Emboss polygons2model preserves hole area and closes the projected shell", "[Emboss]")
{
    const ExPolygons shape{rectangle_with_hole()};
    constexpr double depth = 7.;
    const indexed_triangle_set mesh = Emboss::polygons2model(shape, Emboss::ProjectZ(depth));

    check_surface_vertices(mesh, depth, 8);
    REQUIRE(its_num_open_edges(mesh) == 0);
    const MeshSummary summary = inspect_projected_shell(mesh, depth);
    const double expected_area = shape.front().area();
    CHECK_THAT(summary.front_area, Catch::Matchers::WithinAbs(expected_area, 1e-3));
    CHECK_THAT(summary.back_area, Catch::Matchers::WithinAbs(expected_area, 1e-3));
    CHECK(summary.side_faces == 16); // Two triangles per wall, four walls each for the contour and its hole.
}

TEST_CASE("Emboss polygons2model handles shared polygon points without duplicate vertices", "[Emboss]")
{
    // The two components touch at one corner.  That shared point intentionally
    // selects polygons2model_duplicit without relying on triangle ordering.
    const ExPolygons shape{
        ExPolygon(Polygon {{0, 0}, {100, 0}, {100, 100}, {0, 100}}),
        ExPolygon(Polygon {{100, 100}, {200, 100}, {200, 200}, {100, 200}}),
    };
    constexpr double depth = 5.;
    const indexed_triangle_set mesh = Emboss::polygons2model(shape, Emboss::ProjectZ(depth));

    check_surface_vertices(mesh, depth, 7);
    const MeshSummary summary = inspect_projected_shell(mesh, depth);
    CHECK_THAT(summary.front_area, Catch::Matchers::WithinAbs(20000., 1e-3));
    CHECK_THAT(summary.back_area, Catch::Matchers::WithinAbs(20000., 1e-3));
    CHECK(summary.side_faces == 16);
}

TEST_CASE("Emboss places text onto a surface point facing along its normal", "[Emboss]")
{
    // The text's Z axis follows the surface normal and its Y axis the suggested up direction.
    struct Case { Vec3d normal; Vec3d up; };
    const std::array<Case, 3> cases{{{Vec3d::UnitZ(), Vec3d::UnitY()},
                                     {-Vec3d::UnitZ(), Vec3d::UnitY()},
                                     {Vec3d::UnitX(), Vec3d::UnitZ()}}};
    const Vec3d position(12., -4., 9.);
    for (const Case &c : cases) {
        const Transform3d transform = Emboss::create_transformation_onto_surface(position, c.normal);
        same_vector(transform.translation(), position);
        same_vector((transform.linear() * Vec3d::UnitZ()).normalized(), c.normal);
        same_vector((transform.linear() * Vec3d::UnitY()).normalized(), c.up);
    }
}

TEST_CASE("Emboss Z and orthogonal projections round-trip a 2d point", "[Emboss]")
{
    const Point point{30, 40};
    Emboss::ProjectZ project_z(11.);
    const auto front_back = project_z.create_front_back(point);
    same_vector(front_back.first, Vec3d(30., 40., 0.));
    same_vector(front_back.second, Vec3d(30., 40., 11.));
    const std::optional<Vec2d> unprojected = project_z.unproject(front_back.second);
    REQUIRE(unprojected.has_value());
    CHECK_THAT((*unprojected - point.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));

    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(5., 6., 7.));
    Emboss::OrthoProject ortho(matrix, Vec3d(0., 0., 3.));
    const auto ortho_front_back = ortho.create_front_back(point);
    same_vector(ortho_front_back.first, Vec3d(35., 46., 7.));
    same_vector(ortho_front_back.second, Vec3d(35., 46., 10.));
    double depth = -1.;
    const std::optional<Vec2d> from_back = ortho.unproject(ortho_front_back.second, &depth);
    REQUIRE(from_back.has_value());
    CHECK_THAT((*from_back - point.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(depth, Catch::Matchers::WithinAbs(3., 1e-9));
}

TEST_CASE("Projection wrappers preserve round-trip coordinates and scale the depth", "[Emboss]")
{
    const Point p = Point::new_scale(3., 5.);

    SECTION("ProjectScale scales the projected points and the depth") {
        auto core = std::make_unique<Emboss::OrthoProject>(Transform3d::Identity(), Vec3d(0., 0., 4.));
        Emboss::ProjectScale scaled(std::move(core), 2.);
        const auto front_back = scaled.create_front_back(p);
        same_vector(front_back.second, Vec3d(2. * p.x(), 2. * p.y(), 8.));
        double depth = 0.;
        const auto back = scaled.unproject(front_back.second, &depth);
        REQUIRE(back.has_value());
        CHECK_THAT((*back - p.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));
        CHECK_THAT(depth, Catch::Matchers::WithinAbs(8., 1e-9));
    }
    SECTION("ProjectTransform moves the projected points and scales the depth by its Z scale") {
        Transform3d transform = Transform3d::Identity();
        transform.translate(Vec3d(10., -4., 3.));
        transform.scale(Vec3d(1., 1., 3.));
        auto core = std::make_unique<Emboss::OrthoProject>(Transform3d::Identity(), Vec3d(0., 0., 2.));
        Emboss::ProjectTransform transformed(std::move(core), transform);
        const auto front_back = transformed.create_front_back(p);
        same_vector(front_back.second, Vec3d(p.x() + 10., p.y() - 4., 3. + 3. * 2.));
        double depth = 0.;
        const auto back = transformed.unproject(front_back.second, &depth);
        REQUIRE(back.has_value());
        CHECK_THAT((*back - p.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT(depth, Catch::Matchers::WithinAbs(6., 1e-9));
    }
}

TEST_CASE("Orthogonal 3d projection moves a point by its direction", "[Emboss]")
{
    const Emboss::OrthoProject3d projection(Vec3d(1., 2., 3.));
    same_vector(projection.project(Vec3d(4., -1., 8.)), Vec3d(5., 1., 11.));
}

TEST_CASE("Suggested-up and calculated-up distinguish identity and rotated frames", "[Emboss]")
{
    // A normal tilted 37 degrees from Z: the suggested up lies in the surface and points upward.
    const Vec3d normal = Vec3d(0.6, 0.0, 0.8);
    const Vec3d suggested = Emboss::suggest_up(normal);
    same_vector(suggested, Vec3d(-0.8, 0., 0.6));

    CHECK_FALSE(Emboss::calc_up(Transform3d::Identity()).has_value());
    Transform3d rotated = Transform3d::Identity();
    rotated.rotate(Eigen::AngleAxisd(PI / 2., Vec3d::UnitZ()));
    const auto calculated = Emboss::calc_up(rotated);
    REQUIRE(calculated.has_value());
    CHECK_THAT(std::abs(*calculated), Catch::Matchers::WithinAbs(PI / 2., 1e-6));
}

TEST_CASE("An absent emboss transform preserves the existing placement", "[Emboss]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(0.7, Vec3d::UnitX()));
    const Transform3d before = transform;
    Emboss::apply_transformation(std::nullopt, std::nullopt, transform);
    CHECK_THAT((transform.matrix() - before.matrix()).norm(), Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("Emboss rotation acts around the local text normal", "[Emboss]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    Emboss::apply_transformation(float(PI / 2), std::nullopt, transform);
    same_vector(transform * Vec3d::UnitX(), Vec3d(3, 5, 5));
    same_vector(transform * Vec3d::UnitY(), Vec3d(2, 4, 5));
    same_vector(transform * Vec3d::Zero(), Vec3d(3, 4, 5));
}

TEST_CASE("Emboss distance follows the transformed normal", "[Emboss]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(PI / 2, Vec3d::UnitY()));
    Emboss::apply_transformation(std::nullopt, -2.f, transform);
    same_vector(transform * Vec3d::Zero(), Vec3d(1, 4, 5));
    same_vector(transform.linear() * Vec3d::UnitZ(), Vec3d::UnitX());
}

TEST_CASE("Combined emboss rotation and distance retain both placement effects", "[Emboss]")
{
    Transform3d transform = Transform3d::Identity();
    Emboss::apply_transformation(float(PI / 2), 3.f, transform);
    same_vector(transform * Vec3d::UnitX(), Vec3d(0, 1, 3));
    same_vector(transform * Vec3d::UnitY(), Vec3d(-1, 0, 3));
}
