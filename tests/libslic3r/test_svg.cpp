#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Line.hpp"
#include <ios>
#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/SVG.hpp"
#include "test_utils.hpp"

#include <boost/filesystem/operations.hpp>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

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

std::string read_text(const std::string &path)
{
    std::ifstream in(path, std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

size_t count_text(const std::string &text, const std::string &needle)
{
    size_t count = 0;
    for (size_t pos = text.find(needle); pos != std::string::npos; pos = text.find(needle, pos + needle.size()))
        ++count;
    return count;
}

} // namespace

TEST_CASE("SVG exports an expolygon hole with independent outline colors", "[SVG][SLASVGFormatExtended]")
{
    ScopedTemporaryFile output(".svg");
    const ExPolygons shapes{square_with_hole()};
    SVG::export_expolygons(output.string(), get_extents(shapes), shapes, "outer", "hole", scaled(.2));

    const std::string text = read_text(output.string());
    CHECK(text.find("<?xml version=\"1.0\"") != std::string::npos);
    CHECK(text.find("fill-type: evenodd") != std::string::npos);
    CHECK(text.find("stroke: outer") != std::string::npos);
    CHECK(text.find("stroke: hole") != std::string::npos);
    CHECK(count_text(text, "<path ") >= 3);
}

TEST_CASE("SVG attributes preserve opacity legends and point markers", "[SVG][SLASVGFormatExtended]")
{
    ScopedTemporaryFile output(".svg");
    const ExPolygons shapes{square(0., 0., 10., 8.)};
    const SVG::ExPolygonAttributes attributes(
        "fixture legend", "crimson", "green", "blue", scaled(.1), .35f, "yellow", scaled(.2));
    SVG::export_expolygons(output.string(), {{shapes, attributes}});

    const std::string text = read_text(output.string());
    CHECK(text.find("fill: crimson") != std::string::npos);
    CHECK(text.find("stroke: green") != std::string::npos);
    CHECK(text.find("fixture legend") != std::string::npos);
    CHECK(text.find("fill-opacity=\"0.350000\"") != std::string::npos);
    CHECK(text.find("fill: yellow") != std::string::npos);
    CHECK(count_text(text, "<circle ") >= 5);
}

TEST_CASE("SVG skips an empty attribute export without creating a file", "[SVG][SLASVGFormatExtended]")
{
    ScopedTemporaryFile output(".svg");
    SVG::export_expolygons(output.string(), std::vector<std::pair<ExPolygons, SVG::ExPolygonAttributes>>{});
    CHECK_FALSE(boost::filesystem::exists(output.path()));
}

TEST_CASE("SVG emits grid lines only when the requested step fits", "[SVG][SLASVGFormatExtended]")
{
    ScopedTemporaryFile output(".svg");
    const BoundingBox box(Point(0, 0), Point(scaled(10.), scaled(10.)));
    {
        SVG svg(output.string(), box, scaled(1.), false);
        svg.draw_grid(box, "grid", scaled(.1), scaled(5.));
        svg.Close();
    }
    const std::string text = read_text(output.string());
    CHECK(count_text(text, "<line ") == 6);

    ScopedTemporaryFile too_small(".svg");
    const BoundingBox tiny(Point(0, 0), Point(scaled(2.), scaled(10.)));
    {
        SVG svg(too_small.string(), tiny, scaled(1.), false);
        svg.draw_grid(tiny, "grid", scaled(.1), scaled(5.));
        svg.Close();
    }
    CHECK(count_text(read_text(too_small.string()), "<line ") == 0);
}

TEST_CASE("SVG handles polygon winding and primitive annotations", "[SVG][SLASVGFormatExtended]")
{
    const BoundingBox box(Point(0, 0), Point(scaled(20.), scaled(20.)));
    Polygon ccw = square(1., 1., 8., 8.).contour;
    if (!ccw.is_counter_clockwise())
        ccw.reverse();
    Polygon cw = ccw;
    cw.reverse();

    ScopedTemporaryFile ccw_output(".svg");
    {
        SVG svg(ccw_output.string(), box, scaled(1.), false);
        svg.arrows = true;
        svg.draw(Polygons{ccw}, "red");
        svg.draw(Line(Point(scaled(2.), scaled(2.)), Point(scaled(6.), scaled(2.))), "orange", scaled(.1));
        svg.draw_text(Point(scaled(2.), scaled(4.)), "text", "purple", 3);
        svg.draw_legend(Point(scaled(2.), scaled(6.)), "legend", "teal");
        svg.add_comment("metadata");
        svg.Close();
    }

    const std::string ccw_text = read_text(ccw_output.string());
    CHECK(ccw_text.find("fill: red") != std::string::npos);
    CHECK(ccw_text.find("fill: white") == std::string::npos);
    CHECK(ccw_text.find("marker-end=\"url(#endArrow)\"") != std::string::npos);
    CHECK(ccw_text.find("font-size=\"30px\"") != std::string::npos);
    CHECK(ccw_text.find("legend") != std::string::npos);
    CHECK(ccw_text.find("<!-- metadata -->") != std::string::npos);

    ScopedTemporaryFile cw_output(".svg");
    {
        SVG svg(cw_output.string(), box, scaled(1.), false);
        svg.draw(Polygons{cw}, "red");
        svg.Close();
    }
    const std::string cw_text = read_text(cw_output.string());
    CHECK(cw_text.find("fill: white") != std::string::npos);
    CHECK(cw_text.find("fill: red") == std::string::npos);
}
