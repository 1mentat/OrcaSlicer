#include <ios>
#include <initializer_list>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/EmbossShape.hpp"
#include <iterator>
#include <fstream>
#include <cstdint>
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TextConfiguration.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <libslic3r/Point.hpp>
#include <libslic3r/EmbossShape.hpp>
#include <libslic3r/ExPolygon.hpp>
#include <libslic3r/TextConfiguration.hpp>
#include <libslic3r/BoundingBox.hpp>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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

std::filesystem::path repo_font_path()
{
    return std::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() /
           "resources/fonts/Sarabun-Medium.ttf";
}

double glyph_area(const ExPolygonsWithIds &shapes)
{
    double result = 0.;
    for (const ExPolygonsWithId &shape : shapes)
        for (const ExPolygon &expolygon : shape.expoly)
            result += expolygon.area();
    return result;
}

struct MeshSummary {
    double front_area = 0.;
    double back_area = 0.;
    size_t side_faces = 0;
};

MeshSummary inspect_projected_shell(const indexed_triangle_set &mesh, double back_z)
{
    MeshSummary summary;
    for (const stl_triangle_vertex_indices &triangle : mesh.indices) {
        REQUIRE(triangle[0] < mesh.vertices.size());
        REQUIRE(triangle[1] < mesh.vertices.size());
        REQUIRE(triangle[2] < mesh.vertices.size());

        const Vec3f &a = mesh.vertices[triangle[0]];
        const Vec3f &b = mesh.vertices[triangle[1]];
        const Vec3f &c = mesh.vertices[triangle[2]];
        REQUIRE((b - a).cross(c - a).squaredNorm() > 1e-8f);

        const bool front = std::abs(a.z()) < 1e-5f &&
                           std::abs(b.z()) < 1e-5f &&
                           std::abs(c.z()) < 1e-5f;
        const bool back = std::abs(static_cast<double>(a.z()) - back_z) < 1e-5 &&
                          std::abs(static_cast<double>(b.z()) - back_z) < 1e-5 &&
                          std::abs(static_cast<double>(c.z()) - back_z) < 1e-5;
        if (front)
            summary.front_area += projected_triangle_area(a, b, c);
        else if (back)
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

} // namespace

TEST_CASE("Emboss polygons2model preserves hole area and closes the projected shell", "[Emboss]")
{
    const ExPolygons shape{rectangle_with_hole()};
    constexpr double depth = 7.;
    const indexed_triangle_set mesh = Emboss::polygons2model(shape, Emboss::ProjectZ(depth));

    check_surface_vertices(mesh, depth, 8);
    const MeshSummary summary = inspect_projected_shell(mesh, depth);
    const double expected_area = shape.front().area();
    CHECK_THAT(summary.front_area, Catch::Matchers::WithinAbs(expected_area, 1e-3));
    CHECK_THAT(summary.back_area, Catch::Matchers::WithinAbs(expected_area, 1e-3));
    CHECK(summary.side_faces >= 16); // Four walls for both the contour and its hole.
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
    CHECK(summary.side_faces >= 16);
}

TEST_CASE("Emboss projections preserve surface orientation and round trips", "[Emboss]")
{
    const std::array<Vec3d, 3> normals{Vec3d::UnitZ(), -Vec3d::UnitZ(), Vec3d::UnitX()};
    const Vec3d position(12., -4., 9.);
    for (const Vec3d &normal : normals) {
        const Transform3d transform = Emboss::create_transformation_onto_surface(position, normal);
        const Vec3d transformed_normal = (transform.linear() * Vec3d::UnitZ()).normalized();
        CHECK(transform.translation().isApprox(position, 1e-9));
        CHECK_THAT(transformed_normal.dot(normal), Catch::Matchers::WithinAbs(1., 1e-9));
    }

    const Point point{30, 40};
    Emboss::ProjectZ project_z(11.);
    const auto front_back = project_z.create_front_back(point);
    CHECK_THAT(front_back.first.z(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(front_back.second.z(), Catch::Matchers::WithinAbs(11., 1e-9));
    double ignored_depth = 0.;
    const std::optional<Vec2d> unprojected = project_z.unproject(front_back.second, &ignored_depth);
    REQUIRE(unprojected.has_value());
    CHECK_THAT((*unprojected - point.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));

    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(5., 6., 7.));
    Emboss::OrthoProject ortho(matrix, Vec3d(0., 0., 3.));
    const auto ortho_front_back = ortho.create_front_back(point);
    const std::optional<Vec2d> ortho_unprojected = ortho.unproject(ortho_front_back.first, &ignored_depth);
    REQUIRE(ortho_unprojected.has_value());
    CHECK_THAT((*ortho_unprojected - point.cast<double>()).norm(), Catch::Matchers::WithinAbs(0., 1e-9));
}

TEST_CASE("Emboss font contracts cover real glyph caching and style variance", "[Emboss]")
{
    const std::filesystem::path font_path = repo_font_path();
    REQUIRE(std::filesystem::exists(font_path));
    std::string font_path_string = font_path.string();
    std::unique_ptr<Emboss::FontFile> font_file = Emboss::create_font_file(font_path_string.c_str());
    REQUIRE(font_file != nullptr);
    REQUIRE_FALSE(font_file->infos.empty());

    FontProp regular(10.f);
    const int line_height = Emboss::get_line_height(*font_file, regular);
    REQUIRE(line_height > 0);

    FontProp large(20.f);
    const double regular_scale = Emboss::get_text_shape_scale(regular, *font_file);
    const double large_scale = Emboss::get_text_shape_scale(large, *font_file);
    CHECK_THAT(large_scale / regular_scale, Catch::Matchers::WithinAbs(2., 1e-12));

    bool has_unknown = false;
    CHECK(Emboss::create_range_text("BBAA", *font_file, 0, &has_unknown) == "AB");
    CHECK_FALSE(has_unknown);
    const std::string unknown_text("A\x01", 2);
    has_unknown = false;
    CHECK(Emboss::create_range_text(unknown_text, *font_file, 0, &has_unknown) == "A");
    CHECK(has_unknown);

    Emboss::FontFileWithCache cached(std::move(font_file));
    const std::wstring text = L"A B\nC";
    const ExPolygonsWithIds first = Emboss::text2vshapes(cached, text, regular);
    REQUIRE(first.size() == text.size());
    CHECK(first[3].id == static_cast<unsigned>(L'\n'));
    CHECK(first[3].expoly.empty());
    CHECK(Emboss::get_count_lines(text) == 2);
    CHECK(Emboss::get_count_lines(std::string("A B\nC")) == 2);
    CHECK(Emboss::get_count_lines(first) == 2);
    size_t nonempty_glyphs = 0;
    for (const ExPolygonsWithId &shape : first)
        nonempty_glyphs += !shape.expoly.empty();
    CHECK(nonempty_glyphs == 3); // A, B, and C; space and newline have no outline.
    const size_t cache_size = cached.cache->size();
    REQUIRE(cache_size >= 3);
    const double first_area = glyph_area(first);

    const ExPolygonsWithIds repeated = Emboss::text2vshapes(cached, text, regular);
    CHECK(cached.cache->size() == cache_size);
    CHECK_THAT(glyph_area(repeated), Catch::Matchers::WithinAbs(first_area, 1e-9));
    const HealedExPolygons healed = Emboss::text2shapes(cached, "A B\nC", regular);
    REQUIRE_FALSE(healed.expolygons.empty());
    CHECK(healed.is_healed);

    auto left_font = Emboss::create_font_file(font_path_string.c_str());
    auto right_font = Emboss::create_font_file(font_path_string.c_str());
    REQUIRE(left_font != nullptr);
    REQUIRE(right_font != nullptr);
    Emboss::FontFileWithCache left_cache(std::move(left_font));
    Emboss::FontFileWithCache right_cache(std::move(right_font));
    FontProp left(10.f);
    left.align = {FontProp::HorizontalAlign::left, FontProp::VerticalAlign::center};
    FontProp right = left;
    right.align.first = FontProp::HorizontalAlign::right;
    const BoundingBox left_bounds = get_extents(Emboss::text2vshapes(left_cache, L"AB", left));
    const BoundingBox right_bounds = get_extents(Emboss::text2vshapes(right_cache, L"AB", right));
    REQUIRE(left_bounds.defined);
    REQUIRE(right_bounds.defined);
    CHECK(left_bounds.max.x() > right_bounds.max.x());
    CHECK_THAT(right_bounds.max.x(), Catch::Matchers::WithinAbs(0., 1.));

    auto plain_font = Emboss::create_font_file(font_path_string.c_str());
    auto bold_font = Emboss::create_font_file(font_path_string.c_str());
    REQUIRE(plain_font != nullptr);
    REQUIRE(bold_font != nullptr);
    Emboss::FontFileWithCache plain_cache(std::move(plain_font));
    Emboss::FontFileWithCache bold_cache(std::move(bold_font));
    FontProp bold(10.f);
    bold.boldness = 0.2f;
    const double plain_area = glyph_area(Emboss::text2vshapes(plain_cache, L"A", regular));
    const double bold_area = glyph_area(Emboss::text2vshapes(bold_cache, L"A", bold));
    CHECK(bold_area > plain_area);
}

namespace emboss_layout_contracts {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
Point xy(double x, double y) { return Point(scaled(x), scaled(y)); }
Polygon square() { return Polygon{Points{xy(-10, -10), xy(10, -10), xy(10, 10), xy(-10, 10)}}; }
Emboss::TextLine baseline() { return Emboss::TextLine{square(), PolygonPoint{0, xy(0, -10)}, 0.f}; }
BoundingBox glyph_box(int center) { return BoundingBox(Point(center - 1, -1), Point(center + 1, 1)); }
void same_vector(const Vec3d& actual, const Vec3d& expected)
{
    for (int axis = 0; axis < 3; ++axis)
        CHECK_THAT(actual[axis], WithinAbs(expected[axis], 1e-6));
}
auto font()
{
    const auto path = repo_font_path();
    auto result     = Emboss::create_font_file(path.string().c_str());
    REQUIRE(result != nullptr);
    return result;
}
} // namespace

TEST_CASE("An absent emboss transform preserves the existing placement", "[Emboss][EmbossLayout]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(0.7, Vec3d::UnitX()));
    const Transform3d before = transform;
    Emboss::apply_transformation(std::nullopt, std::nullopt, transform);
    CHECK_THAT((transform.matrix() - before.matrix()).norm(), WithinAbs(0., 1e-12));
}

TEST_CASE("Emboss rotation acts around the local text normal", "[Emboss][EmbossLayout]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    Emboss::apply_transformation(float(PI / 2), std::nullopt, transform);
    same_vector(transform * Vec3d::UnitX(), Vec3d(3, 5, 5));
    same_vector(transform * Vec3d::UnitY(), Vec3d(2, 4, 5));
    same_vector(transform * Vec3d::Zero(), Vec3d(3, 4, 5));
}

TEST_CASE("Emboss distance follows the transformed normal", "[Emboss][EmbossLayout]")
{
    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(3, 4, 5));
    transform.rotate(Eigen::AngleAxisd(PI / 2, Vec3d::UnitY()));
    Emboss::apply_transformation(std::nullopt, -2.f, transform);
    same_vector(transform * Vec3d::Zero(), Vec3d(1, 4, 5));
    same_vector(transform.linear() * Vec3d::UnitZ(), Vec3d::UnitX());
}

TEST_CASE("Combined emboss rotation and distance retain both placement effects", "[Emboss][EmbossLayout]")
{
    Transform3d transform = Transform3d::Identity();
    Emboss::apply_transformation(float(PI / 2), 3.f, transform);
    same_vector(transform * Vec3d::UnitX(), Vec3d(0, 1, 3));
    same_vector(transform * Vec3d::UnitY(), Vec3d(-1, 0, 3));
}

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
    const Polygon polygon = square();
    const double angle    = Emboss::calculate_angle(int32_t(scaled(2.)), PolygonPoint{1, xy(10, -10)}, polygon);
    CHECK_THAT(std::cos(angle), WithinAbs(std::sqrt(0.5), 1e-8));
    CHECK_THAT(std::sin(angle), WithinAbs(-std::sqrt(0.5), 1e-8));
}

TEST_CASE("Emboss normal calculation handles crossing into both adjacent edges", "[Emboss][EmbossLayout]")
{
    const Polygon polygon = square();
    const double angle    = Emboss::calculate_angle(int32_t(scaled(12.)), PolygonPoint{0, xy(0, -10)}, polygon);
    CHECK_THAT(std::cos(angle), WithinAbs(0., 1e-8));
    CHECK_THAT(std::sin(angle), WithinAbs(-1., 1e-8));
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
    const BoundingBoxes boxes{glyph_box(2), BoundingBox(), glyph_box(6)};
    const auto samples = Emboss::sample_slice(baseline(), boxes, 1.);
    REQUIRE(samples.size() == 3);
    CHECK(samples[0].point == xy(2, -10));
    CHECK(samples[1].point == samples[0].point);
    CHECK(samples[2].point == xy(6, -10));
}

TEST_CASE("Emboss glyph sampling crosses a corner at the requested chord distance", "[Emboss][EmbossLayout]")
{
    const auto samples = Emboss::sample_slice(baseline(), BoundingBoxes{glyph_box(13)}, 1.);
    REQUIRE(samples.size() == 1);
    CHECK(samples[0].index == 1);
    CHECK(samples[0].point.x() == scaled(10.));
    // A 13 mm chord from (0,-10) reaches the x=10 edge at sqrt(69) above its base.
    CHECK_THAT(unscale<double>(samples[0].point.y()), WithinAbs(-10. + std::sqrt(69.), 2e-6));
}

TEST_CASE("Emboss vertical alignment offsets respond independently to text height and line count", "[Emboss][EmbossLayout]")
{
    auto face = font();
    FontProp prop(10.f);
    const auto offset = [&](FontProp::VerticalAlign alignment, unsigned count) {
        return Emboss::get_align_y_offset_in_mm(alignment, count, *face, prop);
    };
    const double bottom_one = offset(FontProp::VerticalAlign::bottom, 1);
    const double bottom_two = offset(FontProp::VerticalAlign::bottom, 2);
    CHECK_THAT(bottom_one, WithinAbs(0., 1e-8));
    REQUIRE(bottom_two > 0.);
    CHECK_THAT(offset(FontProp::VerticalAlign::bottom, 3), WithinAbs(2. * bottom_two, 1e-5));
    const double top_one = offset(FontProp::VerticalAlign::top, 1);
    REQUIRE(top_one < 0.);
    CHECK_THAT(offset(FontProp::VerticalAlign::top, 3), WithinAbs(top_one, 1e-5));
    const double center_one = offset(FontProp::VerticalAlign::center, 1);
    CHECK_THAT(offset(FontProp::VerticalAlign::center, 3) - center_one, WithinAbs(bottom_two, 1e-5));
    FontProp doubled(20.f);
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 2, *face, doubled), WithinAbs(2. * bottom_two, 1e-5));
}

TEST_CASE("Emboss vertical offsets convert supplied font metrics into exact millimeters", "[Emboss][EmbossLayout]")
{
    auto face = font();
    // Layout consumes cached metrics. Keep a real owned font buffer and supply
    // simple independent metrics: 1000 units/em, 800 ascent, 1200 line advance.
    face->infos = {{800, -200, 200, 1000}};
    FontProp prop(10.f);
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 3, *face, prop), WithinAbs(-8., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 3, *face, prop), WithinAbs(24., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::center, 3, *face, prop), WithinAbs(12. - 8. / 3., 1e-5));
}

TEST_CASE("Extra emboss line gap changes only multi-line baseline spacing", "[Emboss][EmbossLayout]")
{
    auto face   = font();
    face->infos = {{800, -200, 200, 1000}};
    FontProp prop(10.f);
    prop.line_gap = 300;
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 1, *face, prop), WithinAbs(0., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 3, *face, prop), WithinAbs(30., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 3, *face, prop), WithinAbs(-8., 1e-5));
}

TEST_CASE("Emboss layout selects metrics from the requested font collection entry", "[Emboss][EmbossLayout]")
{
    auto face   = font();
    face->infos = {{800, -200, 200, 1000}, {1200, -400, 100, 2000}};
    FontProp prop(20.f);
    prop.collection_number = 1;
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 1, *face, prop), WithinAbs(-12., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 2, *face, prop), WithinAbs(17., 1e-5));
}

} // namespace emboss_layout_contracts

namespace emboss_extra {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::filesystem::path repo_font_path()
{
    return std::filesystem::path(TEST_DATA_DIR).parent_path().parent_path() /
           "resources/fonts/Sarabun-Medium.ttf";
}

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

TEST_CASE("Healing removes self intersections while retaining filled area", "[Emboss][EmbossExtra]")
{
    const Polygons bowtie{scaled_polygon({{0., 0.}, {20., 20.}, {0., 20.}, {20., 0.}})};
    const HealedExPolygons healed = Emboss::heal_polygons(bowtie, true, 10);
    REQUIRE_FALSE(healed.expolygons.empty());
    CHECK(healed.is_healed);
    CHECK_THAT(area_of(healed.expolygons), WithinRel(200. / (SCALING_FACTOR * SCALING_FACTOR), 1e-2));

    ExPolygons duplicate{ExPolygon(scaled_polygon({{0., 0.}, {20., 0.}, {20., 20.}, {20., 20.}, {0., 20.}}))};
    CHECK(Emboss::heal_expolygons(duplicate, 10));
    REQUIRE_FALSE(duplicate.empty());
    CHECK_THAT(area_of(duplicate), WithinRel(400. / (SCALING_FACTOR * SCALING_FACTOR), 1e-2));
}

static ExPolygons close_point_shape()
{
    return {
        ExPolygon(scaled_polygon({{0., 0.}, {100., 0.}, {100., 100.}, {0., 100.}})),
        ExPolygon(scaled_polygon({{50., 2.}, {55., 10.}, {45., 10.}})),
    };
}

TEST_CASE("Close-point division honors a Euclidean radius boundary", "[Emboss][EmbossExtra]")
{
    const double unit = 1. / SCALING_FACTOR;
    const size_t original_points = close_point_shape().front().contour.size();

    ExPolygons below = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(below, 1.9 * unit));
    CHECK(below.front().contour.size() == original_points);

    // AABBTreeLines uses a strict squared-distance comparison, so the exact
    // boundary is intentionally excluded; a radius above the boundary includes the nearby point.
    ExPolygons exact = close_point_shape();
    CHECK_FALSE(Emboss::divide_segments_for_close_point(exact, 2.0 * unit));
    CHECK(exact.front().contour.size() == original_points);

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
    for (const Point &point : above.front().contour.points) {
        CHECK(std::isfinite(static_cast<double>(point.x())));
        CHECK(std::isfinite(static_cast<double>(point.y())));
    }
}

TEST_CASE("Font loading supports raw data, glyph extraction, and invalid collection selection", "[Emboss][EmbossExtra]")
{
    const std::filesystem::path path = repo_font_path();
    REQUIRE(std::filesystem::exists(path));
    std::ifstream input(path, std::ios::binary);
    REQUIRE(input.good());
    std::vector<unsigned char> data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    REQUIRE_FALSE(data.empty());
    auto font = Emboss::create_font_file(std::make_unique<std::vector<unsigned char>>(data));
    REQUIRE(font != nullptr);
    REQUIRE_FALSE(font->infos.empty());

    const auto glyph = Emboss::letter2glyph(*font, 0, 'A', 0.1f);
    REQUIRE(glyph.has_value());
    REQUIRE_FALSE(glyph->shape.empty());
    CHECK(glyph->advance_width > 0);
    CHECK_FALSE(Emboss::letter2glyph(*font, static_cast<unsigned>(font->infos.size()), 'A', 0.1f).has_value());
    CHECK_FALSE(Emboss::is_italic(*font, static_cast<unsigned>(font->infos.size())));
}

TEST_CASE("Glyph cancellation stops a long text conversion at its cancellation checkpoint", "[Emboss][EmbossExtra]")
{
    auto font = Emboss::create_font_file(repo_font_path().string().c_str());
    REQUIRE(font != nullptr);
    Emboss::FontFileWithCache cached(std::move(font));
    FontProp prop(10.f);
    size_t checks = 0;
    const std::wstring text(20, L'A');
    const ExPolygonsWithIds cancelled = Emboss::text2vshapes(cached, text, prop, [&checks]() {
        ++checks;
        return true;
    });
    CHECK(checks > 0);
    CHECK(cancelled.empty());
}

TEST_CASE("Union-with-delta merges overlapping glyph shapes and caches the healed result", "[Emboss][EmbossExtra]")
{
    EmbossShape shape;
    shape.shapes_with_ids = {
        {10u, {square(0., 0., 20.)}, true},
        {11u, {square(10., 0., 20.)}, true},
    };
    const ExPolygons first = Slic3r::union_with_delta(shape, 1.f, 10);
    REQUIRE_FALSE(first.empty());
    // Two 20 x 20 squares overlap by a 10 x 20 rectangle.
    CHECK_THAT(area_of(first), WithinRel(600. / (SCALING_FACTOR * SCALING_FACTOR), 1e-3));
    CHECK(shape.final_shape.is_healed);
    const ExPolygons cached = Slic3r::union_with_delta(shape, 100.f, 0);
    CHECK(cached.size() == first.size());
    CHECK_THAT(area_of(cached), WithinAbs(area_of(first), 1e-6));
}

TEST_CASE("Shape translation, centering, and extents are mutually consistent", "[Emboss][EmbossExtra]")
{
    ExPolygonsWithIds shapes{
        {1u, {square(0., 0., 10.)}, true},
        {2u, {square(20., 10., 5.)}, true},
    };
    const BoundingBox before = Slic3r::get_extents(shapes);
    REQUIRE(before.defined);
    CHECK_THAT(before.min.x(), WithinAbs(0., 1e-6));
    CHECK_THAT(before.max.y(), WithinAbs(15. / SCALING_FACTOR, 1e-6));
    Slic3r::translate(shapes, Point::new_scale(5., -3.));
    const BoundingBox moved = Slic3r::get_extents(shapes);
    CHECK_THAT(moved.min.x(), WithinAbs(5. / SCALING_FACTOR, 1e-6));
    Slic3r::center(shapes);
    const BoundingBox centered = Slic3r::get_extents(shapes);
    CHECK_THAT(centered.center().x(), WithinAbs(0., 1e-6));
    CHECK_THAT(centered.center().y(), WithinAbs(0., 1e-6));
}

TEST_CASE("Suggested-up and calculated-up distinguish identity and rotated frames", "[Emboss][EmbossExtra]")
{
    const Vec3d normal = Vec3d(0.6, 0.0, 0.8).normalized();
    const Vec3d suggested = Emboss::suggest_up(normal);
    CHECK_THAT(suggested.norm(), WithinAbs(1., 1e-9));
    CHECK_THAT(suggested.dot(normal), WithinAbs(0., 1e-9));

    const Transform3d identity = Transform3d::Identity();
    CHECK_FALSE(Emboss::calc_up(identity).has_value());
    Transform3d rotated = Transform3d::Identity();
    rotated.rotate(Eigen::AngleAxisd(PI / 2., Vec3d::UnitZ()));
    const auto calculated = Emboss::calc_up(rotated);
    REQUIRE(calculated.has_value());
    CHECK(std::isfinite(*calculated));
    CHECK_THAT(std::abs(*calculated), WithinAbs(PI / 2., 1e-6));
}

TEST_CASE("Projection wrappers preserve round-trip coordinates and depth", "[Emboss][EmbossExtra]")
{
    auto scale_core = std::make_unique<Emboss::OrthoProject>(Transform3d::Identity(), Vec3d(0., 0., 4.));
    Emboss::ProjectScale scaled(std::move(scale_core), 2.);
    const Point p = Point::new_scale(3., 5.);
    const auto front_back = scaled.create_front_back(p);
    CHECK_THAT(front_back.second.z(), WithinAbs(8., 1e-9));
    double depth = 0.;
    const auto back = scaled.unproject(front_back.second, &depth);
    REQUIRE(back.has_value());
    CHECK_THAT((*back - p.cast<double>()).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(depth, WithinAbs(8., 1e-9));

    Transform3d transform = Transform3d::Identity();
    transform.translate(Vec3d(10., -4., 3.));
    auto transform_core = std::make_unique<Emboss::ProjectZ>(2.);
    Emboss::ProjectTransform transformed(std::move(transform_core), transform);
    const auto transformed_fb = transformed.create_front_back(p);
    double transformed_depth = 0.;
    const auto transformed_back = transformed.unproject(transformed_fb.second, &transformed_depth);
    REQUIRE(transformed_back.has_value());
    CHECK_THAT((*transformed_back - p.cast<double>()).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(transformed_depth, WithinAbs(0., 1e-9));
}

TEST_CASE("Orthogonal direction projection is reversible through the 3d adapter", "[Emboss][EmbossExtra]")
{
    const Emboss::OrthoProject3d projection(Vec3d(1., 2., 3.));
    const Vec3d point(4., -1., 8.);
    const Vec3d projected = projection.project(point);
    CHECK_THAT((projected - point).norm(), WithinAbs(std::sqrt(14.), 1e-9));

    Transform3d matrix = Transform3d::Identity();
    matrix.translate(Vec3d(2., 3., 4.));
    Emboss::OrthoProject ortho(matrix, Vec3d(0., 0., 5.));
    double depth = 0.;
    const auto front_back = ortho.create_front_back(Point::new_scale(2., 3.));
    const auto original = ortho.unproject(front_back.first, &depth);
    REQUIRE(original.has_value());
    CHECK_THAT((*original - Point::new_scale(2., 3.).cast<double>()).norm(), WithinAbs(0., 1e-9));
    CHECK_THAT(depth, WithinAbs(0., 1e-9));
}

} // namespace emboss_extra
