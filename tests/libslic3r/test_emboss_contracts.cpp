#include <initializer_list>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <Eigen/Geometry>
#include <admesh/stl.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Emboss.hpp"

using namespace Slic3r;

namespace emboss_extra {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

Polygon scaled_polygon(std::initializer_list<std::pair<double, double>> points)
{
    Polygon result;
    for (const auto &[x, y] : points)
        result.points.push_back(Point::new_scale(x, y));
    return result;
}

ExPolygon square(double x, double y, double side)
{
    return ExPolygon(scaled_polygon({{x, y}, {x + side, y}, {x + side, y + side}, {x, y + side}}));
}

double area_of(const ExPolygons &shape)
{
    double area = 0.;
    for (const ExPolygon &expolygon : shape)
        area += expolygon.area();
    return area;
}

} // namespace

TEST_CASE("Healing removes self intersections while retaining filled area", "[Emboss][EmbossHealing]")
{
    // A bow tie: two 20 x 10 triangles meeting at its centre. Left self-crossing, its
    // signed area is 0, so the full 200 mm^2 shows the crossing was resolved.
    const Polygons bowtie{scaled_polygon({{0., 0.}, {20., 20.}, {0., 20.}, {20., 0.}})};
    const HealedExPolygons healed = Emboss::heal_polygons(bowtie, true, 10);
    REQUIRE_FALSE(healed.expolygons.empty());
    CHECK(healed.is_healed);
    CHECK_THAT(area_of(healed.expolygons), WithinRel(200. / (SCALING_FACTOR * SCALING_FACTOR), 1e-2));

    // A repeated neighbour point is dropped rather than cut out as a duplicate.
    ExPolygons duplicate{ExPolygon(scaled_polygon({{0., 0.}, {20., 0.}, {20., 20.}, {20., 20.}, {0., 20.}}))};
    CHECK(Emboss::heal_expolygons(duplicate, 10));
    REQUIRE(duplicate.size() == 1);
    CHECK(duplicate.front().contour.size() == 4);
    CHECK_THAT(area_of(duplicate), WithinRel(400. / (SCALING_FACTOR * SCALING_FACTOR), 1e-9));
}

static ExPolygons close_point_shape()
{
    // The triangle's tip at (50, 2) lies 2 mm above the square's bottom edge.
    return {
        ExPolygon(scaled_polygon({{0., 0.}, {100., 0.}, {100., 100.}, {0., 100.}})),
        ExPolygon(scaled_polygon({{50., 2.}, {55., 10.}, {45., 10.}})),
    };
}

TEST_CASE("Close-point division honors a Euclidean radius boundary", "[Emboss][EmbossHealing]")
{
    const double unit = 1. / SCALING_FACTOR;
    const size_t original_points = close_point_shape().front().contour.size();

    ExPolygons below = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(below, 1.9 * unit));
    CHECK(below.front().contour.size() == original_points);

    ExPolygons above = close_point_shape();
    REQUIRE(Emboss::divide_segments_for_close_point(above, 2.1 * unit));
    CHECK(above.front().contour.size() == original_points + 1);
    CHECK(std::count(above.front().contour.points.begin(), above.front().contour.points.end(),
                     Point::new_scale(50., 2.)) == 1);

    ExPolygons zero = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(zero, 0.));
    CHECK(zero.front().contour.size() == original_points);

    ExPolygons negative = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(negative, -unit));
    CHECK(negative.front().contour.size() == original_points);

    ExPolygons empty;
    CHECK_FALSE(Emboss::divide_segments_for_close_point(empty, 2.0 * unit));
}

TEST_CASE("Union-with-delta merges overlapping glyph shapes and caches the healed result", "[Emboss][EmbossHealing]")
{
    EmbossShape shape;
    shape.shapes_with_ids = {
        {10u, {square(0., 0., 20.)}, true},
        {11u, {square(10., 0., 20.)}, true},
    };
    const ExPolygons first = Slic3r::union_with_delta(shape, 1.f, 10);
    // Two 20 x 20 squares overlap by a 10 x 20 rectangle.
    REQUIRE(first.size() == 1);
    CHECK_THAT(area_of(first), WithinRel(600. / (SCALING_FACTOR * SCALING_FACTOR), 1e-3));
    CHECK(shape.final_shape.is_healed);

    // The cached final shape is returned even after the source shapes change.
    shape.shapes_with_ids.clear();
    const ExPolygons cached = Slic3r::union_with_delta(shape, 100.f, 0);
    REQUIRE(cached.size() == first.size());
    CHECK_THAT(area_of(cached), WithinAbs(area_of(first), 1e-6));
}

} // namespace emboss_extra

namespace emboss_spike_followups {

using Catch::Matchers::WithinAbs;

namespace {

Point emboss_point(coord_t x, coord_t y)
{
    // Emboss healing operates on raw glyph-grid coordinates.  Point(x, y)
    // deliberately avoids Point::new_scale, which would rescale these units.
    return Point(x, y);
}

ExPolygon broad_triangle()
{
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(-10000, 0),
                             emboss_point(0, -10000)});
}

ExPolygon sharp_duplicate_island()
{
    // The fixed duplicate-spike half-bevel is 500 raw units.  Both legs are
    // shorter than that and their angle is below the five-pixel threshold.
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(300, 0),
                             emboss_point(450, 20)});
}

ExPolygon blunt_duplicate_island()
{
    // The same shared origin and short legs, but a 45 degree corner, provide
    // a duplicate-point control that must not be classified as a spike.
    return ExPolygon(Polygon{emboss_point(0, 0),
                             emboss_point(300, 0),
                             emboss_point(300, 300)});
}

void check_positive_oriented_contours(const ExPolygons &shape)
{
    for (const ExPolygon &expolygon : shape) {
        REQUIRE(expolygon.contour.points.size() >= 3);
        CHECK(expolygon.contour.area() > 0.);
        CHECK(expolygon.holes.empty());
    }
}

} // namespace

TEST_CASE("Healing removes a sharp duplicate-point island and preserves the broad contour", "[Emboss][EmbossHealing]")
{
    const ExPolygon broad = broad_triangle();
    const double broad_area = broad.area();
    ExPolygons shape{sharp_duplicate_island(), broad};

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    REQUIRE(shape.size() == 1);
    check_positive_oriented_contours(shape);
    CHECK_THAT(shape.front().area(), WithinAbs(broad_area, 4.));
}

TEST_CASE("Healing retains a non-spike duplicate-point island as a separate contour", "[Emboss][EmbossHealing]")
{
    const ExPolygon broad = broad_triangle();
    const double broad_area = broad.area();
    const double blunt_area = blunt_duplicate_island().area();
    ExPolygons shape{blunt_duplicate_island(), broad};

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    REQUIRE(shape.size() == 2);
    check_positive_oriented_contours(shape);

    // Only the 3 x 3 cut around the shared point is lost from each part.
    std::sort(shape.begin(), shape.end(), [](const ExPolygon &a, const ExPolygon &b) { return a.area() < b.area(); });
    CHECK_THAT(shape.back().area(), WithinAbs(broad_area, 4.));
    CHECK_THAT(shape.front().area(), WithinAbs(blunt_area, 4.));
}

TEST_CASE("Healing leaves a shape without duplicate points unchanged", "[Emboss][EmbossHealing]")
{
    ExPolygons shape{broad_triangle()};
    const ExPolygons original = shape;

    REQUIRE(Emboss::heal_expolygons(shape, 10));
    CHECK(shape == original);
}

} // namespace emboss_spike_followups

namespace emboss_layout_contracts {

using Catch::Matchers::WithinAbs;

namespace {
Point xy(double x, double y) { return Point(scaled(x), scaled(y)); }
// A 20 mm square around the origin, counter-clockwise.
Polygon square() { return Polygon{Points{xy(-10, -10), xy(10, -10), xy(10, 10), xy(-10, 10)}}; }
// A text line along the square's bottom edge, starting at its midpoint.
Emboss::TextLine baseline() { return Emboss::TextLine{square(), PolygonPoint{0, xy(0, -10)}, 0.f}; }
// A 2-unit glyph box (in font units) centred at x = center.
BoundingBox glyph_box(int center) { return BoundingBox(Point(center - 1, -1), Point(center + 1, 1)); }
} // namespace

TEST_CASE("Emboss normals point outward along each side of a rectangle", "[Emboss][EmbossLayout]")
{
    const Polygon polygon = square();
    const PolygonPoints locations{{0, xy(0, -10)}, {1, xy(10, 0)}, {2, xy(0, 10)}, {3, xy(-10, 0)}};
    const auto angles = Emboss::calculate_angles(int32_t(scaled(1.)), locations, polygon);
    REQUIRE(angles.size() == 4);
    const std::vector<Vec2d> expected{{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    for (size_t i = 0; i < angles.size(); ++i) {
        CHECK_THAT(std::cos(angles[i]), WithinAbs(expected[i].x(), 1e-8));
        CHECK_THAT(std::sin(angles[i]), WithinAbs(expected[i].y(), 1e-8));
    }
}

TEST_CASE("Emboss normals bisect a right-angle corner", "[Emboss][EmbossLayout]")
{
    const double angle = Emboss::calculate_angle(int32_t(scaled(2.)), PolygonPoint{1, xy(10, -10)}, square());
    CHECK_THAT(std::cos(angle), WithinAbs(std::sqrt(0.5), 1e-8));
    CHECK_THAT(std::sin(angle), WithinAbs(-std::sqrt(0.5), 1e-8));
}

TEST_CASE("Emboss normal calculation handles crossing into either adjacent edge", "[Emboss][EmbossLayout]")
{
    // An 8 mm chord from 5 mm off the bottom edge's midpoint: one sample crosses the near corner and meets
    // the side edge where the 8 mm circle cuts it, sqrt(64 - 25) = sqrt(39) mm above the corner; the other
    // stays on the bottom edge 8 mm away. The normal is perpendicular to the line between the two samples,
    // so it tilts away from straight down (the bottom-edge normal and the clamp-to-corner result).
    const double rise = std::sqrt(39.);
    const double expected_tilt = std::atan2(-13., rise);  // -64.34 degrees
    SECTION("the forward sample crosses the right corner") {
        const double angle = Emboss::calculate_angle(int32_t(scaled(8.)), PolygonPoint{0, xy(5, -10)}, square());
        CHECK_THAT(angle, WithinAbs(expected_tilt, 1e-6));
    }
    SECTION("the reverse sample crosses the left corner") {
        const double angle = Emboss::calculate_angle(int32_t(scaled(8.)), PolygonPoint{0, xy(-5, -10)}, square());
        CHECK_THAT(angle, WithinAbs(-PI - expected_tilt, 1e-6)); // -115.66 degrees, the mirror image
    }
}

TEST_CASE("An empty list of emboss samples produces no normal angles", "[Emboss][EmbossLayout]")
{
    CHECK(Emboss::calculate_angles(int32_t(scaled(1.)), PolygonPoints{}, square()).empty());
    CHECK(Emboss::sample_slice(baseline(), BoundingBoxes{}, 1.).empty());
}

TEST_CASE("Emboss glyph centers sample both sides of the baseline origin", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(-4), glyph_box(0), glyph_box(4)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i) {
        CHECK(samples[i].index == 0);
        CHECK(samples[i].point == xy(double(i) * 4. - 4., -10));
    }
}

TEST_CASE("Emboss glyph samples scale font coordinates into millimeters", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(2), glyph_box(4), glyph_box(6)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 0.5);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i)
        CHECK(samples[i].point == xy(double(i) + 1., -10));
}

TEST_CASE("Emboss sampling supports a line containing only left-aligned glyphs", "[Emboss][EmbossLayout]")
{
    const BoundingBoxes boxes{glyph_box(-6), glyph_box(-4), glyph_box(-2)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    for (size_t i = 0; i < samples.size(); ++i)
        CHECK(samples[i].point == xy(double(i) * 2. - 6., -10));
}

TEST_CASE("Whitespace glyphs retain an existing baseline cursor", "[Emboss][EmbossLayout]")
{
    // A whitespace glyph has no box; it takes the previous glyph's position.
    const BoundingBoxes boxes{glyph_box(2), BoundingBox(), glyph_box(6)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    CHECK(samples[0].point == xy(2, -10));
    CHECK(samples[1].point == samples[0].point);
    CHECK(samples[2].point == xy(6, -10));
}

TEST_CASE("Emboss glyph sampling crosses a corner at the requested chord distance", "[Emboss][EmbossLayout]")
{
    // A 13 mm chord from (0,-10) reaches a side edge sqrt(13^2 - 10^2) = sqrt(69) mm above its base.
    SECTION("a glyph right of the origin lands on the right edge") {
        const auto samples = Emboss::sample_slice(baseline(), BoundingBoxes{glyph_box(13)}, 1.);
        REQUIRE(samples.size() == 1);
        CHECK(samples[0].index == 1);
        CHECK(samples[0].point.x() == scaled(10.));
        CHECK_THAT(unscale<double>(samples[0].point.y()), WithinAbs(-10. + std::sqrt(69.), 2e-6));
    }
    SECTION("a glyph left of the origin lands on the left edge") {
        const auto samples = Emboss::sample_slice(baseline(), BoundingBoxes{glyph_box(-13)}, 1.);
        REQUIRE(samples.size() == 1);
        CHECK(samples[0].index == 3);
        CHECK(samples[0].point.x() == scaled(-10.));
        CHECK_THAT(unscale<double>(samples[0].point.y()), WithinAbs(-10. + std::sqrt(69.), 2e-6));
    }
}

} // namespace emboss_layout_contracts

namespace emboss_projection_contracts {

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

// A frame at (3, 4, 5) turned 90 degrees about world Y: local X is world -Z, local Y world Y, local Z world X.
Transform3d tilted_frame()
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(PI / 2, Vec3d::UnitY()));
    return transform;
}

} // namespace

TEST_CASE("Emboss polygons2model preserves hole area and closes the projected shell", "[Emboss][EmbossProjection]")
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

TEST_CASE("Emboss polygons2model handles shared polygon points without duplicate vertices", "[Emboss][EmbossProjection]")
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

TEST_CASE("Emboss places text onto a surface point facing along its normal", "[Emboss][EmbossProjection]")
{
    // The text's Z axis follows the surface normal, its Y axis the suggested up direction, and its X axis
    // completes a right-handed frame.
    struct Case { Vec3d normal; Vec3d x; Vec3d up; };
    // Mirrored normals (+X and -X, tilted toward +X and -X) need opposite turns about the normal.
    const std::array<Case, 5> cases{{{-Vec3d::UnitZ(), -Vec3d::UnitX(), Vec3d::UnitY()},
                                     {Vec3d::UnitX(), Vec3d::UnitY(), Vec3d::UnitZ()},
                                     {-Vec3d::UnitX(), -Vec3d::UnitY(), Vec3d::UnitZ()},
                                     {Vec3d(0.6, 0., 0.8), Vec3d::UnitY(), Vec3d(-0.8, 0., 0.6)},
                                     {Vec3d(-0.6, 0., 0.8), -Vec3d::UnitY(), Vec3d(0.8, 0., 0.6)}}};
    const Vec3d position(12., -4., 9.);
    for (const Case &c : cases) {
        const Transform3d transform = Emboss::create_transformation_onto_surface(position, c.normal);
        same_vector(transform.translation(), position);
        same_vector(transform.linear() * Vec3d::UnitX(), c.x);
        same_vector(transform.linear() * Vec3d::UnitY(), c.up);
        same_vector(transform.linear() * Vec3d::UnitZ(), c.normal);
    }
}

TEST_CASE("Emboss Z and orthogonal projections round-trip a 2d point", "[Emboss][EmbossProjection]")
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

    // With a rotated matrix the direction is given in world space and unprojection undoes the rotation.
    Transform3d rotated = Transform3d::Identity();
    rotated.translate(Vec3d(5., 6., 7.));
    rotated.rotate(Eigen::AngleAxisd(PI / 2., Vec3d::UnitX()));
    Emboss::OrthoProject rotated_ortho(rotated, rotated.linear() * Vec3d(0., 0., 3.));
    const auto rotated_front_back = rotated_ortho.create_front_back(point);
    same_vector(rotated_front_back.first, Vec3d(35., 6., 47.));
    same_vector(rotated_front_back.second, Vec3d(35., 3., 47.));
    depth = -1.;
    const std::optional<Vec2d> rotated_back = rotated_ortho.unproject(rotated_front_back.second, &depth);
    REQUIRE(rotated_back.has_value());
    CHECK_THAT((*rotated_back - point.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(depth, Catch::Matchers::WithinAbs(3., 1e-9));
}

TEST_CASE("Projection wrappers preserve round-trip coordinates and scale the depth", "[Emboss][EmbossProjection]")
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
        // Tilted 30 degrees about X, so the Z scale is the length of the transformed Z axis (3), not its
        // Z component (3 cos 30).
        Transform3d transform = Transform3d::Identity();
        transform.translate(Vec3d(10., -4., 3.));
        transform.rotate(Eigen::AngleAxisd(PI / 6., Vec3d::UnitX()));
        transform.scale(Vec3d(1., 1., 3.));
        auto core = std::make_unique<Emboss::OrthoProject>(Transform3d::Identity(), Vec3d(0., 0., 2.));
        Emboss::ProjectTransform transformed(std::move(core), transform);
        const auto front_back = transformed.create_front_back(p);
        same_vector(front_back.second, Vec3d(p.x() + 10.,
                                             p.y() * std::cos(PI / 6.) - 6. * std::sin(PI / 6.) - 4.,
                                             p.y() * std::sin(PI / 6.) + 6. * std::cos(PI / 6.) + 3.));
        double depth = 0.;
        const auto back = transformed.unproject(front_back.second, &depth);
        REQUIRE(back.has_value());
        CHECK_THAT((*back - p.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT(depth, Catch::Matchers::WithinAbs(6., 1e-9));
    }
}

TEST_CASE("Orthogonal 3d projection moves a point by its direction", "[Emboss][EmbossProjection]")
{
    const Emboss::OrthoProject3d projection(Vec3d(1., 2., 3.));
    same_vector(projection.project(Vec3d(4., -1., 8.)), Vec3d(5., 1., 11.));
}

TEST_CASE("Suggested-up and calculated-up distinguish identity and rotated frames", "[Emboss][EmbossProjection]")
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
    // A frame turned +90 degrees about Z (its Y axis now points along -X) reports +pi/2.
    CHECK_THAT(*calculated, Catch::Matchers::WithinAbs(PI / 2., 1e-6));
}

TEST_CASE("An absent emboss transform preserves the existing placement", "[Emboss][EmbossProjection]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(0.7, Vec3d::UnitX()));
    const Transform3d before = transform;
    Emboss::apply_transformation(std::nullopt, std::nullopt, transform);
    CHECK_THAT((transform.matrix() - before.matrix()).norm(), Catch::Matchers::WithinAbs(0., 1e-12));
}

TEST_CASE("Emboss rotation acts around the local text normal", "[Emboss][EmbossProjection]")
{
    // In the tilted frame the text normal (local Z) is world +X, so a rotation about world Z would differ.
    Transform3d transform = tilted_frame();
    Emboss::apply_transformation(float(PI / 2), std::nullopt, transform);
    same_vector(transform.translation(), Vec3d(3, 4, 5));
    same_vector(transform.linear() * Vec3d::UnitX(), Vec3d::UnitY());
    same_vector(transform.linear() * Vec3d::UnitY(), Vec3d::UnitZ());
    same_vector(transform.linear() * Vec3d::UnitZ(), Vec3d::UnitX());
}

TEST_CASE("Emboss distance follows the transformed normal", "[Emboss][EmbossProjection]")
{
    Transform3d transform = tilted_frame();
    Emboss::apply_transformation(std::nullopt, -2.f, transform);
    same_vector(transform * Vec3d::Zero(), Vec3d(1, 4, 5));
    same_vector(transform.linear() * Vec3d::UnitZ(), Vec3d::UnitX());
}

TEST_CASE("Combined emboss rotation and distance retain both placement effects", "[Emboss][EmbossProjection]")
{
    // A rotation about the normal and a move along it commute, so only the frame (local, not world) is
    // observable: the text turns about world +X and moves 3 mm along it.
    Transform3d transform = tilted_frame();
    Emboss::apply_transformation(float(PI / 2), 3.f, transform);
    same_vector(transform.translation(), Vec3d(6, 4, 5));
    same_vector(transform.linear() * Vec3d::UnitX(), Vec3d::UnitY());
    same_vector(transform.linear() * Vec3d::UnitY(), Vec3d::UnitZ());
    same_vector(transform.linear() * Vec3d::UnitZ(), Vec3d::UnitX());
}

} // namespace emboss_projection_contracts
