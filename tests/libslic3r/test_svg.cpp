#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/SVG.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/libslic3r.h"
#include "test_utils.hpp"

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>

#include <algorithm>
#include <cstddef>
#include <ios>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
namespace pt = boost::property_tree;

namespace {

ExPolygon square(double x0, double y0, double x1, double y1)
{
    ExPolygon result;
    result.contour.points = {
        Point(scaled(x0), scaled(y0)), Point(scaled(x1), scaled(y0)),
        Point(scaled(x1), scaled(y1)), Point(scaled(x0), scaled(y1))};
    return result;
}

ExPolygon square_with_hole()
{
    ExPolygon result = square(2., 2., 18., 14.);
    Polygon hole;
    hole.points = {
        Point(scaled(7.), scaled(6.)), Point(scaled(7.), scaled(10.)),
        Point(scaled(13.), scaled(10.)), Point(scaled(13.), scaled(6.))};
    if (!hole.is_clockwise())
        hole.reverse();
    result.holes.push_back(std::move(hole));
    return result;
}

// Parses the written file as XML, so the assertions read elements and attributes
// rather than the writer's byte layout. Throws if the output is not well formed.
pt::ptree read_svg(const ScopedTemporaryFile &file)
{
    boost::nowide::ifstream in(file.string(), std::ios::binary);
    REQUIRE(in.good());
    pt::ptree tree;
    pt::read_xml(in, tree);
    return tree.get_child("svg");
}

// Direct children of <svg> with the given tag, in document order.
std::vector<pt::ptree> elements(const pt::ptree &svg, const std::string &tag)
{
    std::vector<pt::ptree> out;
    for (const auto &child : svg)
        if (child.first == tag)
            out.push_back(child.second);
    return out;
}

std::string attr(const pt::ptree &element, const std::string &name)
{
    return element.get<std::string>("<xmlattr>." + name, "");
}

std::string text(const pt::ptree &element) { return boost::algorithm::trim_copy(element.get_value<std::string>()); }

// Declarations of the style attribute, e.g. {"fill": "red", "stroke-width": "2.000000"}.
std::map<std::string, std::string> style(const pt::ptree &element)
{
    std::map<std::string, std::string> out;
    std::istringstream in(attr(element, "style"));
    for (std::string declaration; std::getline(in, declaration, ';');) {
        const size_t colon = declaration.find(':');
        if (colon != std::string::npos)
            out[boost::algorithm::trim_copy(declaration.substr(0, colon))] =
                boost::algorithm::trim_copy(declaration.substr(colon + 1));
    }
    return out;
}

double style_number(const pt::ptree &element, const std::string &name) { return std::stod(style(element).at(name)); }

bool is_filled(const pt::ptree &path) { return style(path).at("fill") != "none"; }

// Coordinates of a path "d" attribute or a polygon "points" attribute, with the
// M / z commands and the comma separators dropped.
std::vector<double> coordinates(std::string data)
{
    std::replace(data.begin(), data.end(), ',', ' ');
    std::istringstream in(data);
    std::vector<double> out;
    for (std::string token; in >> token;)
        if (token != "M" && token != "z")
            out.push_back(std::stod(token));
    return out;
}

size_t subpath_count(const std::string &d)
{
    std::istringstream in(d);
    size_t count = 0;
    for (std::string token; in >> token;)
        count += token == "M";
    return count;
}

void check_x_range(const std::string &d, double min_x, double max_x)
{
    const std::vector<double> c = coordinates(d);
    REQUIRE(c.size() >= 2);
    double lo = c[0];
    double hi = c[0];
    for (size_t i = 0; i < c.size(); i += 2) {
        lo = std::min(lo, c[i]);
        hi = std::max(hi, c[i]);
    }
    CHECK_THAT(lo, WithinAbs(min_x, 1e-4));
    CHECK_THAT(hi, WithinAbs(max_x, 1e-4));
}

void check_coordinates(const std::string &data, const std::vector<double> &expected)
{
    const std::vector<double> actual = coordinates(data);
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(actual[i], WithinAbs(expected[i], 1e-4));
}

} // namespace

TEST_CASE("SVG exports an expolygon hole with independent outline colors", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    const ExPolygons shapes{square_with_hole()};
    SVG::export_expolygons(output.string(), get_extents(shapes), shapes, "outer", "hole", scaled(.2));

    const std::vector<pt::ptree> paths = elements(read_svg(output), "path");
    REQUIRE(paths.size() == 3);
    // The fill is a single path holding both the contour and the hole.
    CHECK(is_filled(paths[0]));
    CHECK(subpath_count(attr(paths[0], "d")) == 2);

    // The export box (2,2)-(18,14) gets a 1 mm margin, so x maps to (x - 1) * 10 SVG units.
    CHECK_FALSE(is_filled(paths[1]));
    CHECK(style(paths[1]).at("stroke") == "outer");
    check_x_range(attr(paths[1], "d"), 10., 170.);
    CHECK_FALSE(is_filled(paths[2]));
    CHECK(style(paths[2]).at("stroke") == "hole");
    check_x_range(attr(paths[2], "d"), 60., 120.);
}

TEST_CASE("SVG attributes preserve opacity legends and point markers", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    const ExPolygons shapes{square(0., 0., 10., 8.)};
    const SVG::ExPolygonAttributes attributes(
        "fixture legend", "crimson", "green", "blue", scaled(.1), .35f, "yellow", scaled(.2));
    SVG::export_expolygons(output.string(), {{shapes, attributes}});

    const pt::ptree svg = read_svg(output);
    const std::vector<pt::ptree> paths = elements(svg, "path");
    REQUIRE(paths.size() == 2);
    CHECK(style(paths[0]).at("fill") == "crimson");
    CHECK_THAT(std::stod(attr(paths[0], "fill-opacity")), WithinAbs(.35, 1e-6));
    CHECK_FALSE(is_filled(paths[1]));
    CHECK(style(paths[1]).at("stroke") == "green");

    // One marker per contour point with the requested 0.2 mm radius, then the legend swatch.
    size_t markers = 0;
    size_t swatches = 0;
    for (const pt::ptree &circle : elements(svg, "circle")) {
        if (style(circle)["fill"] == "yellow") {
            ++markers;
            CHECK_THAT(std::stod(attr(circle, "r")), WithinAbs(2., 1e-6));
        } else if (attr(circle, "fill") == "crimson")
            ++swatches;
    }
    CHECK(markers == shapes.front().contour.size());
    CHECK(swatches == 1);
    const std::vector<pt::ptree> texts = elements(svg, "text");
    REQUIRE(texts.size() == 1);
    CHECK(text(texts.front()) == "fixture legend");
}

TEST_CASE("SVG skips an empty attribute export without creating a file", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    SVG::export_expolygons(output.string(), std::vector<std::pair<ExPolygons, SVG::ExPolygonAttributes>>{});
    CHECK_FALSE(boost::filesystem::exists(output.path()));
}

TEST_CASE("SVG emits grid lines only when the requested step fits", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    const BoundingBox box(Point(0, 0), Point(scaled(10.), scaled(10.)));
    {
        SVG svg(output.string(), box, scaled(1.), false);
        svg.draw_grid(box, "grid", scaled(.1), scaled(5.));
        svg.Close();
    }
    // Rows and columns at 0, 5 and 10 mm.
    const std::vector<pt::ptree> lines = elements(read_svg(output), "line");
    CHECK(lines.size() == 6);
    for (const pt::ptree &line : lines)
        CHECK(style(line).at("stroke") == "grid");

    ScopedTemporaryFile too_small(".svg");
    const BoundingBox narrow(Point(0, 0), Point(scaled(2.), scaled(10.)));
    {
        SVG svg(too_small.string(), narrow, scaled(1.), false);
        svg.draw_grid(narrow, "grid", scaled(.1), scaled(5.));
        svg.Close();
    }
    CHECK(elements(read_svg(too_small), "line").empty());
}

TEST_CASE("SVG paints counter-clockwise polygons with their fill and clockwise ones white", "[SVG]")
{
    Polygon ccw = square(1., 1., 8., 8.).contour;
    if (!ccw.is_counter_clockwise())
        ccw.reverse();
    Polygon cw = square(10., 10., 18., 18.).contour;
    if (!cw.is_clockwise())
        cw.reverse();

    ScopedTemporaryFile output(".svg");
    {
        SVG svg(output.string(), BoundingBox(Point(0, 0), Point(scaled(20.), scaled(20.))), scaled(1.), false);
        svg.draw(Polygons{ccw, cw}, "red");
        svg.Close();
    }
    // A clockwise polygon is painted white, as a hole, instead of the requested fill.
    const std::vector<pt::ptree> paths = elements(read_svg(output), "path");
    REQUIRE(paths.size() == 2);
    CHECK(style(paths[0]).at("fill") == "red");
    CHECK(style(paths[1]).at("fill") == "white");
}

TEST_CASE("SVG writes line arrows, text, legends and comments", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    {
        SVG svg(output.string(), BoundingBox(Point(0, 0), Point(scaled(20.), scaled(20.))), scaled(1.), false);
        svg.arrows = true;
        svg.draw(Line(Point(scaled(2.), scaled(2.)), Point(scaled(6.), scaled(2.))), "orange", scaled(.1));
        svg.draw_text(Point(scaled(2.), scaled(4.)), "text", "purple", 3);
        svg.draw_legend(Point(scaled(2.), scaled(6.)), "legend", "teal");
        svg.add_comment("metadata");
        svg.Close();
    }
    const pt::ptree svg = read_svg(output);

    const std::vector<pt::ptree> lines = elements(svg, "line");
    REQUIRE(lines.size() == 1);
    CHECK(style(lines.front()).at("stroke") == "orange");
    CHECK(attr(lines.front(), "marker-end") == "url(#endArrow)");

    // draw_text scales the font size by 10; the legend is a swatch plus a black label.
    const std::vector<pt::ptree> texts = elements(svg, "text");
    REQUIRE(texts.size() == 2);
    CHECK(text(texts[0]) == "text");
    CHECK(attr(texts[0], "fill") == "purple");
    CHECK(attr(texts[0], "font-size") == "30px");
    CHECK(text(texts[1]) == "legend");
    const std::vector<pt::ptree> circles = elements(svg, "circle");
    REQUIRE(circles.size() == 1);
    CHECK(attr(circles.front(), "fill") == "teal");

    const std::vector<pt::ptree> comments = elements(svg, "<xmlcomment>");
    REQUIRE(comments.size() == 1);
    CHECK(text(comments.front()) == "metadata");
}

TEST_CASE("SVG creates nested output paths and reports an unopenable destination", "[SVG]")
{
    ScopedTemporaryDir directory("orca-svg-open");
    const boost::filesystem::path nested_output = directory.path() / "nested" / "output.svg";
    {
        SVG svg(nested_output.string());
        REQUIRE(svg.is_opened());
        svg.add_comment("created nested parent");
        svg.Close();
    }
    boost::nowide::ifstream nested(nested_output.string(), std::ios::binary);
    REQUIRE(nested.good());
    pt::ptree nested_tree;
    pt::read_xml(nested, nested_tree);
    CHECK(text(nested_tree.get_child("svg.<xmlcomment>")) == "created nested parent");

    // A regular file where the parent directory should be cannot be opened through.
    ScopedTemporaryFile blocker(".file");
    {
        boost::nowide::ofstream file(blocker.string(), std::ios::binary);
        REQUIRE(file.good());
        file << "not a directory";
    }
    SVG unopenable((blocker.path() / "child.svg").string());
    CHECK_FALSE(unopenable.is_opened());

    const BoundingBox box(Point(0, 0), Point(scaled(1.), scaled(1.)));
    SVG unopenable_with_bbox((blocker.path() / "boxed-child.svg").string(), box, 0, false);
    CHECK_FALSE(unopenable_with_bbox.is_opened());
}

TEST_CASE("SVG surface value and pointer collections preserve filled and outline paths", "[SVG]")
{
    const Surface top(stTop, square(1., 1., 8., 8.));
    const Surface holed(stInternal, square_with_hole());
    const Surfaces surfaces{top, holed};
    const SurfacesPtr surface_refs{&surfaces[0], &surfaces[1]};

    const BoundingBox box(Point(0, 0), Point(scaled(20.), scaled(20.)));
    ScopedTemporaryFile output(".svg");
    {
        SVG svg(output.string(), box, 0, false);
        svg.draw(top, "single-fill", .4f);
        svg.draw_outline(top, "single-rim", "single-hole", scaled(.2));
        svg.draw(surfaces, "collection-fill", .6f);
        svg.draw_outline(surfaces, "collection-rim", "collection-hole", scaled(.2));
        svg.draw(surface_refs, "pointer-fill", .8f);
        svg.draw_outline(surface_refs, "pointer-rim", "pointer-hole", scaled(.2));
        svg.Close();
    }

    // Each expolygon fill is one path; each outline is one path per contour and hole.
    std::map<std::string, size_t> fills;
    std::map<std::string, size_t> outlines;
    for (const pt::ptree &path : elements(read_svg(output), "path")) {
        if (is_filled(path))
            ++fills[style(path).at("fill")];
        else
            ++outlines[style(path).at("stroke")];
    }
    const std::map<std::string, size_t> expected_fills{
        {"single-fill", 1}, {"collection-fill", 2}, {"pointer-fill", 2}};
    const std::map<std::string, size_t> expected_outlines{
        {"single-rim", 1}, {"collection-rim", 2}, {"collection-hole", 1}, {"pointer-rim", 2}, {"pointer-hole", 1}};
    CHECK(fills == expected_fills);
    CHECK(outlines == expected_outlines);
}

TEST_CASE("SVG primitive collections retain tapered widths and scaled coordinates", "[SVG]")
{
    // Zero margin and no Y flip: SVG coordinates are millimetres * 10.
    const BoundingBox box(Point(0, 0), Point(scaled(20.), scaled(20.)));
    ScopedTemporaryFile output(".svg");
    {
        SVG svg(output.string(), box, 0, false);
        svg.draw(Lines{Line(Point(scaled(0.), scaled(1.)), Point(scaled(2.), scaled(1.)))}, "line");
        svg.draw(Polylines{Polyline{Point(scaled(1.), scaled(4.)), Point(scaled(3.), scaled(4.))}}, "polyline", scaled(.1));

        const ThickLine thick_line(Point(scaled(1.), scaled(2.)), Point(scaled(5.), scaled(2.)),
                                   scaled(1.), scaled(3.));
        svg.draw(ThickLines{thick_line}, "direct-fill", "direct-stroke", scaled(.1));

        ThickPolyline thick_polyline;
        thick_polyline.points = {Point(scaled(1.), scaled(2.)), Point(scaled(5.), scaled(2.))};
        thick_polyline.width = {static_cast<coordf_t>(scaled(1.)), static_cast<coordf_t>(scaled(3.))};
        const ThickPolylines thick_polylines{thick_polyline};
        svg.draw(thick_polylines, "thin-stroke", scaled(.1));
        svg.draw(thick_polylines, "taper-fill", "taper-stroke", scaled(.1));

        svg.draw(Points{Point(scaled(2.), scaled(8.)), Point(scaled(3.), scaled(8.))}, "marker", 0);

        const Points scaled_path{Point(scaled(2.), scaled(3.)), Point(scaled(4.), scaled(3.)), Point(scaled(4.), scaled(5.))};
        svg.draw(scaled_path, .5, "scaled-path", scaled(.2));
        VecOfPoints scaled_groups;
        scaled_groups.push_back(scaled_path);
        scaled_groups.push_back(Points{Point(scaled(6.), scaled(3.)), Point(scaled(8.), scaled(3.))});
        svg.draw(scaled_groups, .5, "scaled-group", scaled(.2));
        svg.Close();
    }

    const pt::ptree svg = read_svg(output);
    CHECK(elements(svg, "line").size() == 1);
    CHECK(elements(svg, "circle").size() == 2);

    // Both thick-line paths draw the same trapezoid: 1 mm wide at x = 1, 3 mm wide at x = 5.
    const std::vector<pt::ptree> polygons = elements(svg, "polygon");
    REQUIRE(polygons.size() == 2);
    CHECK(style(polygons[0]).at("fill") == "direct-fill");
    CHECK(style(polygons[1]).at("fill") == "taper-fill");
    for (const pt::ptree &polygon : polygons)
        check_coordinates(attr(polygon, "points"), {10., 15., 50., 5., 50., 35., 10., 25.});

    const std::vector<pt::ptree> paths = elements(svg, "path");
    REQUIRE(paths.size() == 5);
    CHECK(style(paths[0]).at("stroke") == "polyline");
    // Without a fill colour a thick polyline is drawn as its untapered centreline.
    CHECK(style(paths[1]).at("stroke") == "thin-stroke");
    check_coordinates(attr(paths[1], "d"), {10., 20., 50., 20.});
    // The scaled overloads multiply the input coordinates by 0.5 before mapping them.
    const std::vector<double> expected_scaled{10., 15., 20., 15., 20., 25.};
    CHECK(style(paths[2]).at("stroke") == "scaled-path");
    check_coordinates(attr(paths[2], "d"), expected_scaled);
    CHECK(style(paths[3]).at("stroke") == "scaled-group");
    check_coordinates(attr(paths[3], "d"), expected_scaled);
    CHECK(style(paths[4]).at("stroke") == "scaled-group");
    check_coordinates(attr(paths[4], "d"), {30., 15., 40., 15.});
}

TEST_CASE("SVG attribute export applies contour fallbacks and wraps legends", "[SVG]")
{
    const ExPolygons with_hole{square_with_hole()};
    const ExPolygons plain{square(20., 2., 28., 10.)};

    // No contour or hole colour: both outlines fall back to the fill colour.
    SVG::ExPolygonAttributes fill_fallback("fill fallback", "magenta", .35f);
    fill_fallback.outline_width = scaled(.2);
    // Empty hole colour: the hole outline falls back to the contour colour.
    SVG::ExPolygonAttributes hole_fallback("hole fallback", "orange", "teal", "", scaled(.2), .45f);
    // No outline width: no outline is drawn.
    SVG::ExPolygonAttributes third_legend("third legend", "gold", .55f);
    SVG::ExPolygonAttributes fourth_legend("fourth legend", "navy", .65f);
    const std::vector<std::pair<ExPolygons, SVG::ExPolygonAttributes>> layers{
        {with_hole, fill_fallback},
        {with_hole, hole_fallback},
        {plain, third_legend},
        {plain, fourth_legend},
    };

    ScopedTemporaryFile output(".svg");
    SVG::export_expolygons(output.string(), layers);
    const pt::ptree svg = read_svg(output);

    std::map<std::string, size_t> outlines;
    for (const pt::ptree &path : elements(svg, "path"))
        if (!is_filled(path))
            ++outlines[style(path).at("stroke")];
    const std::map<std::string, size_t> expected_outlines{{"magenta", 2}, {"teal", 2}};
    CHECK(outlines == expected_outlines);

    const std::vector<pt::ptree> texts = elements(svg, "text");
    REQUIRE(texts.size() == 4);
    CHECK(text(texts[0]) == "fill fallback");
    CHECK(text(texts[1]) == "hole fallback");
    CHECK(text(texts[2]) == "third legend");
    CHECK(text(texts[3]) == "fourth legend");

    const std::vector<pt::ptree> circles = elements(svg, "circle");
    REQUIRE(circles.size() == 4);
    std::vector<std::pair<double, double>> centers;
    for (const pt::ptree &circle : circles)
        centers.emplace_back(std::stod(attr(circle, "cx")), std::stod(attr(circle, "cy")));
    // Three columns 20 mm (200 SVG units) apart on the first row.
    for (size_t i = 1; i < 3; ++i)
        CHECK_THAT(centers[i].second, WithinAbs(centers[0].second, 1e-4));
    CHECK_THAT(centers[1].first - centers[0].first, WithinAbs(200., 1e-4));
    CHECK_THAT(centers[2].first - centers[1].first, WithinAbs(200., 1e-4));
    // The fourth legend wraps to the first column of a row 1.3 mm (13 units) further;
    // the flipped Y axis makes its screen coordinate smaller.
    CHECK_THAT(centers[3].first, WithinAbs(centers[0].first, 1e-4));
    CHECK_THAT(centers[3].second - centers[0].second, WithinAbs(-13., 1e-4));
}

TEST_CASE("SVG filled paths suppress outlines while open paths retain visible stroke widths", "[SVG]")
{
    ScopedTemporaryFile output(".svg");
    const Polygon polygon({Point(scaled(1.), scaled(1.)), Point(scaled(5.), scaled(1.)),
                           Point(scaled(5.), scaled(5.)), Point(scaled(1.), scaled(5.))});
    {
        const BoundingBox bounds(Point(0, 0), Point(scaled(10.), scaled(10.)));
        SVG svg(output.string(), bounds, scaled(1.), false);
        svg.draw(Polygons{polygon}, "red");
        svg.draw_outline(polygon, "blue", scaled(.3));
        svg.draw(Polyline({Point(scaled(2.), scaled(2.)), Point(scaled(6.), scaled(2.))}), "green");
        svg.Close();
    }

    const std::vector<pt::ptree> paths = elements(read_svg(output), "path");
    REQUIRE(paths.size() == 3);
    CHECK(style(paths[0]).at("fill") == "red");
    CHECK_THAT(style_number(paths[0], "stroke-width"), WithinAbs(0., 1e-6));
    // An explicit 0.3 mm outline is 3 SVG units; a polyline without a width gets the 2 unit default.
    CHECK_FALSE(is_filled(paths[1]));
    CHECK(style(paths[1]).at("stroke") == "blue");
    CHECK_THAT(style_number(paths[1], "stroke-width"), WithinAbs(3., 1e-6));
    CHECK_FALSE(is_filled(paths[2]));
    CHECK(style(paths[2]).at("stroke") == "green");
    CHECK_THAT(style_number(paths[2], "stroke-width"), WithinAbs(2., 1e-6));
}
