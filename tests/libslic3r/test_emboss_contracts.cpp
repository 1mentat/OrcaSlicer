#include <ios>
#include <iterator>
#include <fstream>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/TextConfiguration.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Emboss.hpp"

using namespace Slic3r;

namespace emboss_font_contracts {

using Catch::Matchers::WithinAbs;

namespace {

// resources/fonts sits beside resources/profiles (PROFILES_DIR).
std::string font_path()
{
    return (std::filesystem::path(std::string(PROFILES_DIR)).parent_path() / "fonts" / "Sarabun-Medium.ttf").string();
}

std::unique_ptr<Emboss::FontFile> load_font()
{
    std::unique_ptr<Emboss::FontFile> font = Emboss::create_font_file(font_path().c_str());
    REQUIRE(font != nullptr);
    REQUIRE_FALSE(font->infos.empty());
    return font;
}

double glyph_area(const ExPolygonsWithIds &shapes)
{
    double result = 0.;
    for (const ExPolygonsWithId &shape : shapes)
        for (const ExPolygon &expolygon : shape.expoly)
            result += expolygon.area();
    return result;
}

ExPolygon square(double x, double y, double side)
{
    return ExPolygon(Polygon{Point::new_scale(x, y), Point::new_scale(x + side, y),
                             Point::new_scale(x + side, y + side), Point::new_scale(x, y + side)});
}

} // namespace

TEST_CASE("Emboss loads a font from a file or raw data and extracts glyphs", "[Emboss][EmbossFont]")
{
    std::ifstream input(font_path(), std::ios::binary);
    REQUIRE(input.good());
    std::vector<unsigned char> data((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    auto font = Emboss::create_font_file(std::make_unique<std::vector<unsigned char>>(data));
    REQUIRE(font != nullptr);
    REQUIRE_FALSE(font->infos.empty());

    const auto glyph = Emboss::letter2glyph(*font, 0, 'A', 0.1f);
    REQUIRE(glyph.has_value());
    CHECK_FALSE(glyph->shape.empty());
    CHECK(glyph->advance_width > 0);

    // A collection index past the end is rejected instead of read.
    const auto missing = static_cast<unsigned>(font->infos.size());
    CHECK_FALSE(Emboss::letter2glyph(*font, missing, 'A', 0.1f).has_value());
    CHECK_FALSE(Emboss::is_italic(*font, missing));
}

TEST_CASE("Emboss text shape scale is proportional to the font size", "[Emboss][EmbossFont]")
{
    const auto font = load_font();
    CHECK(Emboss::get_line_height(*font, FontProp(10.f)) > 0);
    const double regular_scale = Emboss::get_text_shape_scale(FontProp(10.f), *font);
    const double large_scale = Emboss::get_text_shape_scale(FontProp(20.f), *font);
    CHECK_THAT(large_scale / regular_scale, WithinAbs(2., 1e-12));
}

TEST_CASE("Emboss range text lists each known glyph once and flags unknown ones", "[Emboss][EmbossFont]")
{
    const auto font = load_font();
    bool has_unknown = true;
    CHECK(Emboss::create_range_text("BBAA", *font, 0, &has_unknown) == "AB");
    CHECK_FALSE(has_unknown);
    has_unknown = false;
    CHECK(Emboss::create_range_text(std::string("A\x01", 2), *font, 0, &has_unknown) == "A");
    CHECK(has_unknown);
}

TEST_CASE("Emboss text conversion yields one shape per character and caches glyphs", "[Emboss][EmbossFont]")
{
    Emboss::FontFileWithCache cached(load_font());
    const FontProp prop(10.f);
    const std::wstring text = L"A B\nC";
    const ExPolygonsWithIds first = Emboss::text2vshapes(cached, text, prop);
    REQUIRE(first.size() == text.size());
    CHECK(first[3].id == static_cast<unsigned>(L'\n'));
    size_t outlined = 0;
    for (const ExPolygonsWithId &shape : first)
        outlined += !shape.expoly.empty();
    CHECK(outlined == 3); // A, B and C; the space and the newline have no outline.
    CHECK(Emboss::get_count_lines(text) == 2);
    CHECK(Emboss::get_count_lines(std::string("A B\nC")) == 2);
    CHECK(Emboss::get_count_lines(first) == 2);

    const size_t cache_size = cached.cache->size();
    REQUIRE(cache_size >= 3);
    const ExPolygonsWithIds repeated = Emboss::text2vshapes(cached, text, prop);
    CHECK(cached.cache->size() == cache_size);
    CHECK_THAT(glyph_area(repeated), WithinAbs(glyph_area(first), 1e-9));

    const HealedExPolygons healed = Emboss::text2shapes(cached, "A B\nC", prop);
    CHECK_FALSE(healed.expolygons.empty());
    CHECK(healed.is_healed);
}

TEST_CASE("Emboss right alignment ends the text at the origin", "[Emboss][EmbossFont]")
{
    Emboss::FontFileWithCache left_cache(load_font());
    Emboss::FontFileWithCache right_cache(load_font());
    FontProp left(10.f);
    left.align = {FontProp::HorizontalAlign::left, FontProp::VerticalAlign::center};
    FontProp right = left;
    right.align.first = FontProp::HorizontalAlign::right;
    const BoundingBox left_bounds = get_extents(Emboss::text2vshapes(left_cache, L"AB", left));
    const BoundingBox right_bounds = get_extents(Emboss::text2vshapes(right_cache, L"AB", right));
    REQUIRE(left_bounds.defined);
    REQUIRE(right_bounds.defined);
    CHECK(left_bounds.max.x() > 0);
    CHECK(right_bounds.max.x() == 0);
    CHECK(right_bounds.size() == left_bounds.size());
}

TEST_CASE("Emboss boldness grows the glyph area", "[Emboss][EmbossFont]")
{
    Emboss::FontFileWithCache plain_cache(load_font());
    Emboss::FontFileWithCache bold_cache(load_font());
    FontProp bold(10.f);
    bold.boldness = 0.2f;
    const double plain_area = glyph_area(Emboss::text2vshapes(plain_cache, L"A", FontProp(10.f)));
    const double bold_area = glyph_area(Emboss::text2vshapes(bold_cache, L"A", bold));
    CHECK(bold_area > plain_area);
}

TEST_CASE("Glyph cancellation stops a long text conversion at its cancellation checkpoint", "[Emboss][EmbossFont]")
{
    Emboss::FontFileWithCache cached(load_font());
    size_t checks = 0;
    const ExPolygonsWithIds cancelled = Emboss::text2vshapes(cached, std::wstring(20, L'A'), FontProp(10.f), [&checks]() {
        ++checks;
        return true;
    });
    CHECK(checks > 0);
    CHECK(cancelled.empty());
}

TEST_CASE("Shape translation, centering, and extents are mutually consistent", "[Emboss][EmbossFont]")
{
    ExPolygonsWithIds shapes{
        {1u, {square(0., 0., 10.)}, true},
        {2u, {square(20., 10., 5.)}, true},
    };
    const BoundingBox before = Slic3r::get_extents(shapes);
    REQUIRE(before.defined);
    CHECK(before.min == Point::new_scale(0., 0.));
    CHECK(before.max == Point::new_scale(25., 15.));
    Slic3r::translate(shapes, Point::new_scale(5., -3.));
    const BoundingBox moved = Slic3r::get_extents(shapes);
    CHECK(moved.min == Point::new_scale(5., -3.));
    CHECK(moved.max == Point::new_scale(30., 12.));
    Slic3r::center(shapes);
    const BoundingBox centered = Slic3r::get_extents(shapes);
    CHECK(centered.center() == Point(0, 0));
    CHECK(centered.size() == before.size());
}

TEST_CASE("Emboss vertical alignment offsets respond independently to text height and line count", "[Emboss][EmbossFont]")
{
    const auto face = load_font();
    const FontProp prop(10.f);
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
    const FontProp doubled(20.f);
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 2, *face, doubled), WithinAbs(2. * bottom_two, 1e-5));
}

TEST_CASE("Emboss vertical offsets convert supplied font metrics into exact millimeters", "[Emboss][EmbossFont]")
{
    const auto face = load_font();
    // Layout consumes cached metrics. Keep a real owned font buffer and supply
    // simple independent metrics: 1000 units/em, 800 ascent, 1200 line advance.
    face->infos = {{800, -200, 200, 1000}};
    const FontProp prop(10.f);
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 3, *face, prop), WithinAbs(-8., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 3, *face, prop), WithinAbs(24., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::center, 3, *face, prop), WithinAbs(12. - 8. / 3., 1e-5));
}

TEST_CASE("Extra emboss line gap changes only multi-line baseline spacing", "[Emboss][EmbossFont]")
{
    const auto face = load_font();
    face->infos = {{800, -200, 200, 1000}};
    FontProp prop(10.f);
    prop.line_gap = 300;
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 1, *face, prop), WithinAbs(0., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 3, *face, prop), WithinAbs(30., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 3, *face, prop), WithinAbs(-8., 1e-5));
}

TEST_CASE("Emboss layout selects metrics from the requested font collection entry", "[Emboss][EmbossFont]")
{
    const auto face = load_font();
    face->infos = {{800, -200, 200, 1000}, {1200, -400, 100, 2000}};
    FontProp prop(20.f);
    prop.collection_number = 1;
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::top, 1, *face, prop), WithinAbs(-12., 1e-5));
    CHECK_THAT(Emboss::get_align_y_offset_in_mm(FontProp::VerticalAlign::bottom, 2, *face, prop), WithinAbs(17., 1e-5));
}

} // namespace emboss_font_contracts
