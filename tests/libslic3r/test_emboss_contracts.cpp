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
