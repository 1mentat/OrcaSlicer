#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/NSVGUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/libslic3r.h"
#include "nanosvg/nanosvg.h"
#include "test_utils.hpp"

#include <cmath>
#include <cstddef>
#include <fstream>
#include <ios>
#include <memory>
#include <string>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

double area_scale()
{
    return SCALING_FACTOR * SCALING_FACTOR;
}

std::string svg_document(const std::string &body, const std::string &attributes = "")
{
    return "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"20mm\" height=\"20mm\" viewBox=\"0 0 20 20\" "
           + attributes + ">" + body + "</svg>";
}

NSVGimage_ptr parse_svg(const std::string &body, const std::string &attributes = "")
{
    NSVGimage_ptr image = nsvgParse(svg_document(body, attributes));
    REQUIRE(image != nullptr);
    return image;
}

NSVGLineParams line_params(double tolerance = 0.01)
{
    return NSVGLineParams(tolerance);
}

double polygons_area_mm2(const Polygons &polygons)
{
    double area = 0.;
    for (const Polygon &polygon : polygons)
        area += std::abs(polygon.area());
    return area * area_scale();
}

double expolygons_area_mm2(const ExPolygons &expolygons)
{
    double area = 0.;
    for (const ExPolygon &expolygon : expolygons)
        area += expolygon.area();
    return area * area_scale();
}

size_t polygon_point_count(const Polygons &polygons)
{
    size_t result = 0;
    for (const Polygon &polygon : polygons)
        result += polygon.points.size();
    return result;
}

size_t expolygon_point_count(const ExPolygons &expolygons)
{
    size_t result = 0;
    for (const ExPolygon &expolygon : expolygons) {
        result += expolygon.contour.points.size();
        for (const Polygon &hole : expolygon.holes)
            result += hole.points.size();
    }
    return result;
}

bool same_expolygon_points(const ExPolygons &left, const ExPolygons &right)
{
    if (left.size() != right.size())
        return false;
    for (size_t i = 0; i < left.size(); ++i) {
        if (left[i].contour.points != right[i].contour.points || left[i].holes.size() != right[i].holes.size())
            return false;
        for (size_t j = 0; j < left[i].holes.size(); ++j)
            if (left[i].holes[j].points != right[i].holes[j].points)
                return false;
    }
    return true;
}

BoundingBox polygon_extents(const Polygons &polygons)
{
    BoundingBox result;
    for (const Polygon &polygon : polygons)
        result.merge(get_extents(polygon));
    return result;
}

BoundingBox expolygon_extents(const ExPolygons &expolygons)
{
    BoundingBox result;
    for (const ExPolygon &expolygon : expolygons)
        result.merge(get_extents(expolygon));
    return result;
}

double extent_width_mm(const BoundingBox &bbox)
{
    return unscale<double>(bbox.max.x() - bbox.min.x());
}

double extent_height_mm(const BoundingBox &bbox)
{
    return unscale<double>(bbox.max.y() - bbox.min.y());
}

void write_file(const ScopedTemporaryFile &file, const std::string &text)
{
    std::ofstream stream(file.string(), std::ios::binary);
    REQUIRE(stream.good());
    stream << text;
    REQUIRE(stream.good());
}

ExPolygonsWithIds shape_ids(const std::string &body, const NSVGLineParams &params = line_params())
{
    NSVGimage_ptr image = parse_svg(body);
    return create_shape_with_ids(*image, params);
}

ExPolygonsWithId only_shape(ExPolygonsWithIds shapes)
{
    REQUIRE(shapes.size() == 1);
    return shapes.front();
}

BoundingBox shape_group_extents(const ExPolygonsWithIds &shapes)
{
    BoundingBox result;
    for (const ExPolygonsWithId &shape : shapes)
        result.merge(expolygon_extents(shape.expoly));
    return result;
}

} // namespace

TEST_CASE("NSVG parses image dimensions and counts linked shapes", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg(
        "<path fill=\"red\" d=\"M0 0h4v5H0z\"/>"
        "<path fill=\"blue\" d=\"M8 0h3v2H8z\"/>");
    const double px_per_mm = 96. / 25.4;
    CHECK_THAT(image->width, WithinAbs(20. * px_per_mm, 0.01));
    CHECK_THAT(image->height, WithinAbs(20. * px_per_mm, 0.01));
    CHECK(get_shapes_count(*image) == 2);
}

TEST_CASE("NSVG converts a filled rectangle to an analytic polygon", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg("<path fill=\"red\" d=\"M0 0L10 0L10 20L0 20z\"/>");
    const Polygons polygons = to_polygons(*image, line_params());
    REQUIRE(polygons.size() == 1);
    CHECK_THAT(polygons_area_mm2(polygons), WithinAbs(200., 0.01));
    const BoundingBox bbox = polygon_extents(polygons);
    CHECK_THAT(unscale<double>(bbox.min.x()), WithinAbs(0., 0.001));
    CHECK_THAT(unscale<double>(bbox.max.x()), WithinAbs(10., 0.001));
    CHECK_THAT(unscale<double>(bbox.min.y()), WithinAbs(-20., 0.001));
    CHECK_THAT(unscale<double>(bbox.max.y()), WithinAbs(0., 0.001));
}

TEST_CASE("NSVG preserves polygon area when the Y orientation is changed", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg("<path fill=\"red\" d=\"M1 2L9 2L9 7L1 7z\"/>");
    NSVGLineParams upward = line_params();
    upward.is_y_negative = false;
    const Polygons positive = to_polygons(*image, upward);
    const Polygons negative = to_polygons(*image, line_params());
    REQUIRE(positive.size() == 1);
    REQUIRE(negative.size() == 1);
    CHECK_THAT(polygons_area_mm2(positive), WithinAbs(40., 0.01));
    CHECK_THAT(polygons_area_mm2(negative), WithinAbs(40., 0.01));
    CHECK_THAT(unscale<double>(polygon_extents(positive).min.y()), WithinAbs(2., 0.001));
    CHECK_THAT(unscale<double>(polygon_extents(negative).max.y()), WithinAbs(-2., 0.001));
}

TEST_CASE("NSVG returns centered fill and stroke groups with stable IDs", "[NSVGUtils]")
{
    const ExPolygonsWithIds shapes = shape_ids(
        "<path fill=\"red\" stroke=\"blue\" stroke-width=\"2\" d=\"M0 0h10v10H0z\"/>");
    REQUIRE(shapes.size() == 2);
    CHECK(shapes[0].id == 0u);
    CHECK(shapes[1].id == 1u);
    CHECK_THAT(expolygons_area_mm2(shapes[0].expoly), WithinAbs(100., 0.01));
    CHECK(expolygons_area_mm2(shapes[1].expoly) > 0.);
    const BoundingBox all = shape_group_extents(shapes);
    CHECK_THAT(unscale<double>(all.center().x()), WithinAbs(0., 0.001));
    CHECK_THAT(unscale<double>(all.center().y()), WithinAbs(0., 0.001));
}

TEST_CASE("NSVG even-odd fill creates a hole with conserved area", "[NSVGUtils]")
{
    const ExPolygonsWithIds shapes = shape_ids(
        "<path fill=\"red\" fill-rule=\"evenodd\" d=\"M0 0h20v20H0z M5 5h10v10H5z\"/>");
    const ExPolygonsWithId shape = only_shape(shapes);
    REQUIRE(shape.expoly.size() == 1);
    REQUIRE(shape.expoly.front().holes.size() == 1);
    CHECK_THAT(expolygons_area_mm2(shape.expoly), WithinAbs(300., 0.05));
    CHECK(shape.is_healed);
}

TEST_CASE("NSVG assigns even IDs to independent filled shapes", "[NSVGUtils]")
{
    const ExPolygonsWithIds shapes = shape_ids(
        "<path fill=\"red\" d=\"M0 0h4v5H0z\"/>"
        "<path fill=\"blue\" d=\"M8 0h3v2H8z\"/>");
    REQUIRE(shapes.size() == 2);
    CHECK(shapes[0].id == 0u);
    CHECK(shapes[1].id == 2u);
    CHECK_THAT(expolygons_area_mm2(shapes[0].expoly), WithinAbs(20., 0.01));
    CHECK_THAT(expolygons_area_mm2(shapes[1].expoly), WithinAbs(6., 0.01));
}

TEST_CASE("NSVG skips invisible and paintless shapes without changing source IDs", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg(
        "<path fill=\"red\" display=\"none\" d=\"M0 0h2v2H0z\"/>"
        "<path fill=\"none\" stroke=\"none\" d=\"M3 0h2\"/>"
        "<path fill=\"blue\" d=\"M6 0h2v2H6z\"/>");
    const ExPolygonsWithIds shapes = create_shape_with_ids(*image, line_params());
    REQUIRE(shapes.size() == 1);
    CHECK(shapes.front().id == 4u);
    CHECK_THAT(expolygons_area_mm2(shapes.front().expoly), WithinAbs(4., 0.01));
}

TEST_CASE("NSVG bounds include every subpath of every shape in source coordinates", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg(
        "<path fill=\"red\" d=\"M2 3L10 3L10 7z\"/>"
        // The blue shape has two subpaths: one sets the minimum, the other the maximum, so
        // both extremes are found only when bounds() follows path->next within a shape.
        "<path fill=\"blue\" d=\"M-4 -2L-1 -2L-1 1z M12 15h2v1h-2z\"/>");
    Vec2f minimum(100.f, 100.f);
    Vec2f maximum(-100.f, -100.f);
    bounds(*image, minimum, maximum);
    CHECK_THAT(minimum.x(), WithinAbs(-4., 0.001));
    CHECK_THAT(minimum.y(), WithinAbs(-2., 0.001));
    CHECK_THAT(maximum.x(), WithinAbs(14., 0.001));
    CHECK_THAT(maximum.y(), WithinAbs(16., 0.001));
}

TEST_CASE("NSVG file and memory parsing produce equivalent geometry", "[NSVGUtils]")
{
    const std::string text = svg_document("<path fill=\"red\" d=\"M1 2h7v5H1z\"/>");
    ScopedTemporaryFile file(".svg");
    write_file(file, text);
    NSVGimage_ptr memory = nsvgParse(text);
    NSVGimage_ptr disk = nsvgParseFromFile(file.string());
    REQUIRE(memory != nullptr);
    REQUIRE(disk != nullptr);
    CHECK_THAT(disk->width, WithinAbs(memory->width, 0.001));
    CHECK_THAT(disk->height, WithinAbs(memory->height, 0.001));
    CHECK(get_shapes_count(*disk) == get_shapes_count(*memory));
    CHECK_THAT(polygons_area_mm2(to_polygons(*disk, line_params())), WithinAbs(
        polygons_area_mm2(to_polygons(*memory, line_params())), 0.01));
    const std::unique_ptr<std::string> loaded = read_from_disk(file.string());
    REQUIRE(loaded != nullptr);
    CHECK(*loaded == text);
}

TEST_CASE("NSVG init_image lazily loads and then caches an SVG image", "[NSVGUtils]")
{
    ScopedTemporaryFile file(".svg");
    const std::string text = svg_document("<path fill=\"red\" d=\"M0 0h3v4H0z\"/>");
    write_file(file, text);
    EmbossShape::SvgFile from_path;
    from_path.path = file.string();
    NSVGimage *first = init_image(from_path);
    REQUIRE(first != nullptr);
    REQUIRE(from_path.file_data != nullptr);
    CHECK(from_path.image.get() == first);
    CHECK(init_image(from_path) == first);
    EmbossShape::SvgFile from_memory;
    from_memory.file_data = std::make_shared<std::string>(text);
    NSVGimage *memory = init_image(from_memory);
    REQUIRE(memory != nullptr);
    CHECK(from_memory.path.empty());
    CHECK(get_shapes_count(*memory) == 1);
}

TEST_CASE("NSVG rejects empty malformed and missing input", "[NSVGUtils]")
{
    NSVGimage_ptr empty = nsvgParse("");
    NSVGimage_ptr malformed = nsvgParse("<svg xmlns=\"http://www.w3.org/2000/svg\"><path");
    REQUIRE(empty != nullptr);
    REQUIRE(malformed != nullptr);
    CHECK(get_shapes_count(*empty) == 0);
    CHECK(get_shapes_count(*malformed) == 0);
    EmbossShape::SvgFile missing;
    missing.path = "/path/that/does/not/exist.svg";
    CHECK(init_image(missing) == nullptr);
    CHECK(missing.file_data == nullptr);
}

TEST_CASE("NSVG stroke caps produce the documented endpoint extents", "[NSVGUtils]")
{
    const auto stroke = [](const std::string &cap) {
        return shape_ids("<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-linecap=\"" + cap
                         + "\" d=\"M2 10L12 10\"/>");
    };
    const ExPolygonsWithId butt = only_shape(stroke("butt"));
    const ExPolygonsWithId round = only_shape(stroke("round"));
    const ExPolygonsWithId square = only_shape(stroke("square"));
    CHECK_THAT(extent_width_mm(expolygon_extents(butt.expoly)), WithinAbs(10., 0.02));
    CHECK_THAT(extent_width_mm(expolygon_extents(round.expoly)), WithinAbs(12., 0.02));
    CHECK_THAT(extent_width_mm(expolygon_extents(square.expoly)), WithinAbs(12., 0.02));
    CHECK(expolygons_area_mm2(round.expoly) > expolygons_area_mm2(butt.expoly));
    CHECK(expolygons_area_mm2(square.expoly) > expolygons_area_mm2(butt.expoly));
    // Square caps add a 1x2 mm rectangle per end (24 mm²), round caps a half disc (20 + pi mm²).
    CHECK(expolygons_area_mm2(square.expoly) > expolygons_area_mm2(round.expoly));
}

TEST_CASE("NSVG miter joins cover more corner than squared bevel joins and round joins add arc points", "[NSVGUtils]")
{
    const auto stroke = [](const std::string &join) {
        return only_shape(shape_ids("<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-linejoin=\""
                                    + join + "\" d=\"M3 12L10 12L10 5\"/>"));
    };
    const ExPolygonsWithId bevel = stroke("bevel");
    const ExPolygonsWithId miter = stroke("miter");
    const ExPolygonsWithId round = stroke("round");
    REQUIRE_FALSE(bevel.expoly.empty());
    REQUIRE_FALSE(miter.expoly.empty());
    REQUIRE_FALSE(round.expoly.empty());
    CHECK(expolygons_area_mm2(miter.expoly) > expolygons_area_mm2(bevel.expoly));
    CHECK(expolygon_point_count(round.expoly) > expolygon_point_count(bevel.expoly));
}

TEST_CASE("NSVG dashed strokes split material and honor dash offsets", "[NSVGUtils]")
{
    const ExPolygonsWithId solid = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" d=\"M0 10L20 10\"/>"));
    const ExPolygonsWithId dashed = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-dasharray=\"4 2\" d=\"M0 10L20 10\"/>"));
    const ExPolygonsWithId shifted = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-dasharray=\"4 2\" stroke-dashoffset=\"2\" d=\"M0 10L20 10\"/>"));
    REQUIRE(dashed.expoly.size() > 1);
    CHECK(expolygons_area_mm2(dashed.expoly) < expolygons_area_mm2(solid.expoly));
    // Visible dash lengths are 4+4+4+2 mm and 2+4+4+4 mm respectively.
    CHECK_THAT(expolygons_area_mm2(dashed.expoly), WithinAbs(28., 0.02));
    CHECK_THAT(expolygons_area_mm2(shifted.expoly), WithinAbs(28., 0.02));
    CHECK_FALSE(same_expolygon_points(shifted.expoly, dashed.expoly));
}

TEST_CASE("NSVG odd dash arrays repeat their complete alternating cycle", "[NSVGUtils]")
{
    const ExPolygonsWithId offset_eight = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-dasharray=\"4 2 1\" "
        "stroke-dashoffset=\"8\" d=\"M0 10L20 10\"/>"));
    const ExPolygonsWithId offset_one = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-dasharray=\"4 2 1\" "
        "stroke-dashoffset=\"1\" d=\"M0 10L20 10\"/>"));
    // An odd three-entry array repeats after two sums, hence 14 mm.
    const ExPolygonsWithId offset_twenty_two = only_shape(shape_ids(
        "<path fill=\"none\" stroke=\"black\" stroke-width=\"2\" stroke-dasharray=\"4 2 1\" "
        "stroke-dashoffset=\"22\" d=\"M0 10L20 10\"/>"));
    REQUIRE(same_expolygon_points(offset_eight.expoly, offset_twenty_two.expoly));
    REQUIRE_FALSE(offset_eight.expoly.empty());
    REQUIRE_FALSE(offset_one.expoly.empty());
    CHECK_FALSE(same_expolygon_points(offset_eight.expoly, offset_one.expoly));
}

TEST_CASE("NSVG curve tessellation refines curved geometry at smaller tolerance", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg("<path fill=\"red\" d=\"M0 0C0 12 12 12 12 0z\"/>");
    NSVGLineParams coarse = line_params(2.e14);
    NSVGLineParams fine = line_params(2.e4);
    const Polygons coarse_polygons = to_polygons(*image, coarse);
    const Polygons fine_polygons = to_polygons(*image, fine);
    REQUIRE_FALSE(coarse_polygons.empty());
    REQUIRE_FALSE(fine_polygons.empty());
    INFO("coarse area=" << polygons_area_mm2(coarse_polygons) << " fine area=" << polygons_area_mm2(fine_polygons)
         << " coarse points=" << polygon_point_count(coarse_polygons) << " fine points=" << polygon_point_count(fine_polygons));
    CHECK(polygon_point_count(fine_polygons) > polygon_point_count(coarse_polygons));
    // Cubic control points give x(t)=36t²−24t³ and y(t)=36t(1−t).
    // Integrating y(t) x'(t) over [0,1] gives 2592/30 mm².
    const double analytic_cubic_area = 86.4;
    CHECK_THAT(polygons_area_mm2(fine_polygons), WithinAbs(analytic_cubic_area, 0.01));
    CHECK(polygons_area_mm2(fine_polygons) > polygons_area_mm2(coarse_polygons));
}

TEST_CASE("NSVG closes an open filled path before calculating area", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg("<path fill=\"red\" d=\"M0 0L10 0L10 10\"/>");
    const Polygons polygons = to_polygons(*image, line_params());
    REQUIRE(polygons.size() == 1);
    CHECK_THAT(polygons_area_mm2(polygons), WithinAbs(50., 0.01));
    const ExPolygonsWithIds shapes = create_shape_with_ids(*image, line_params());
    REQUIRE(shapes.size() == 1);
    CHECK_THAT(expolygons_area_mm2(shapes.front().expoly), WithinAbs(50., 0.05));
}

TEST_CASE("NSVG custom scale changes coordinates but preserves topology", "[NSVGUtils]")
{
    NSVGimage_ptr image = parse_svg("<path fill=\"red\" d=\"M0 0h4v3H0z\"/>");
    NSVGLineParams params = line_params();
    params.scale = 0.5 / SCALING_FACTOR;
    const Polygons polygons = to_polygons(*image, params);
    REQUIRE(polygons.size() == 1);
    CHECK_THAT(polygons_area_mm2(polygons), WithinAbs(3., 0.01));
    CHECK_THAT(extent_width_mm(polygon_extents(polygons)), WithinAbs(2., 0.01));
    CHECK_THAT(extent_height_mm(polygon_extents(polygons)), WithinAbs(1.5, 0.01));
}

TEST_CASE("NSVG centering makes translated inputs geometrically equivalent", "[NSVGUtils]")
{
    const ExPolygonsWithId origin = only_shape(shape_ids("<path fill=\"red\" d=\"M0 0h5v3H0z\"/>"));
    const ExPolygonsWithId translated = only_shape(shape_ids("<path fill=\"red\" d=\"M7 11h5v3H7z\"/>"));
    CHECK_THAT(expolygons_area_mm2(origin.expoly), WithinAbs(expolygons_area_mm2(translated.expoly), 0.01));
    const BoundingBox origin_box = expolygon_extents(origin.expoly);
    const BoundingBox translated_box = expolygon_extents(translated.expoly);
    CHECK_THAT(extent_width_mm(origin_box), WithinAbs(extent_width_mm(translated_box), 0.01));
    CHECK_THAT(extent_height_mm(origin_box), WithinAbs(extent_height_mm(translated_box), 0.01));
    CHECK_THAT(unscale<double>(origin_box.center().x()), WithinAbs(0., 0.001));
    CHECK_THAT(unscale<double>(translated_box.center().x()), WithinAbs(0., 0.001));
}
