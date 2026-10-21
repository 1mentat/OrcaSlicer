#include "libslic3r/Polyline.hpp"
#include <regex>
#include "libslic3r/Format/svg.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <cmath>
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
#include "libslic3r/Surface.hpp"
#include "test_utils.hpp"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <iterator>
#include <sstream>
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

TEST_CASE("SVG creates nested output paths and reports an unopenable destination", "[SVG][SVGExportContracts]")
{
    ScopedTemporaryDir directory("orca-svg-open");
    const std::string nested_output = (directory.path() / "nested" / "output.svg").string();
    {
        SVG svg(nested_output);
        REQUIRE(svg.is_opened());
        svg.add_comment("created nested parent");
        svg.Close();
    }
    CHECK(boost::filesystem::exists(directory.path() / "nested"));
    CHECK(read_text(nested_output).find("<!-- created nested parent -->") != std::string::npos);

    ScopedTemporaryFile blocker(".file");
    {
        std::ofstream file(blocker.string(), std::ios::binary);
        REQUIRE(file.good());
        file << "not a directory";
    }
    SVG unopenable((blocker.path() / "child.svg").string());
    CHECK_FALSE(unopenable.is_opened());

    const BoundingBox box(Point(0, 0), Point(scaled(1.), scaled(1.)));
    SVG unopenable_with_bbox((blocker.path() / "boxed-child.svg").string(), box, 0, false);
    CHECK_FALSE(unopenable_with_bbox.is_opened());
}

TEST_CASE("SVG surface value and pointer collections preserve filled and outline paths", "[SVG][SVGExportContracts]")
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

    const std::string text = read_text(output.string());
    // Each expolygon fill is one even-odd path; each outline is one path per contour and hole.
    CHECK(count_text(text, "<path ") == 12);
    CHECK(count_text(text, "fill: single-fill;") == 1);
    CHECK(count_text(text, "fill: none;") == 7);
    CHECK(count_text(text, "fill: collection-fill;") == 2);
    CHECK(count_text(text, "fill: pointer-fill;") == 2);
    CHECK(count_text(text, "fill: none; stroke: collection-rim;") == 2);
    CHECK(count_text(text, "fill: none; stroke: collection-hole;") == 1);
    CHECK(count_text(text, "fill: none; stroke: pointer-rim;") == 2);
    CHECK(count_text(text, "fill: none; stroke: pointer-hole;") == 1);
    CHECK(count_text(text, "fill: none; stroke: single-rim;") == 1);
}

TEST_CASE("SVG primitive collections retain tapered widths and scaled coordinates", "[SVG][SVGExportContracts]")
{
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

        const Points markers{Point(scaled(2.), scaled(8.)), Point(scaled(3.), scaled(8.))};
        svg.draw(markers, "marker", 0);

        const Points scaled_path{Point(scaled(2.), scaled(3.)), Point(scaled(4.), scaled(3.)), Point(scaled(4.), scaled(5.))};
        svg.draw(scaled_path, .5, "scaled-path", scaled(.2));
        VecOfPoints scaled_groups;
        scaled_groups.push_back(scaled_path);
        scaled_groups.push_back(Points{Point(scaled(6.), scaled(3.)), Point(scaled(8.), scaled(3.))});
        svg.draw(scaled_groups, .5, "scaled-group", scaled(.2));
        svg.Close();
    }

    const std::string text = read_text(output.string());
    CHECK(count_text(text, "<line ") == 1);
    CHECK(count_text(text, "<circle cx=") == 2);
    CHECK(count_text(text, "<polygon points=\"10.000000,15.000000 50.000000,5.000000 50.000000,35.000000 10.000000,25.000000\"") == 2);
    CHECK(count_text(text, "<path d=\"") == 5);
    const std::regex scaled_path_regex(
        R"svg(<path d="([^"]*)" style="fill: none; stroke: (scaled-path|scaled-group);[^"]*"[^>]*>)svg");
    std::vector<std::vector<double>> single_scaled_paths;
    std::vector<std::vector<double>> scaled_groups;
    for (std::sregex_iterator it(text.begin(), text.end(), scaled_path_regex), end; it != end; ++it) {
        std::istringstream path_data((*it)[1].str());
        std::string command;
        path_data >> command;
        REQUIRE(command == "M");

        std::vector<double> coordinates;
        double coordinate;
        while (path_data >> coordinate)
            coordinates.push_back(coordinate);
        path_data.clear();
        path_data >> command;
        CHECK(command == "z");

        if ((*it)[2].str() == "scaled-path")
            single_scaled_paths.push_back(std::move(coordinates));
        else
            scaled_groups.push_back(std::move(coordinates));
    }
    REQUIRE(single_scaled_paths.size() == 1);
    REQUIRE(scaled_groups.size() == 2);
    const std::vector<double> expected_single{10., 15., 20., 15., 20., 25.};
    const std::vector<double> expected_second_group{30., 15., 40., 15.};
    CHECK(single_scaled_paths.front() == expected_single);
    CHECK(scaled_groups.front() == expected_single);
    CHECK(scaled_groups.back() == expected_second_group);
}

TEST_CASE("SVG attribute export applies contour fallbacks and wraps legends", "[SVG][SVGExportContracts]")
{
    const ExPolygons with_hole{square_with_hole()};
    const ExPolygons plain{square(20., 2., 28., 10.)};

    SVG::ExPolygonAttributes fill_fallback("fill fallback", "magenta", .35f);
    fill_fallback.outline_width = scaled(.2);

    SVG::ExPolygonAttributes hole_fallback("hole fallback", "orange", "teal", "", scaled(.2), .45f);
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

    const std::string text = read_text(output.string());
    CHECK(count_text(text, "stroke: magenta;") == 2);
    CHECK(count_text(text, "stroke: teal;") == 2);
    CHECK(count_text(text, "stroke: orange;") == 0);
    CHECK(text.find("fill fallback") != std::string::npos);
    CHECK(text.find("hole fallback") != std::string::npos);
    CHECK(text.find("third legend") != std::string::npos);
    CHECK(text.find("fourth legend") != std::string::npos);
    CHECK(count_text(text, "<circle cx=") == 4);
    CHECK(count_text(text, "<text x=") == 4);

    const std::regex legend_circle(R"svg(<circle cx="([0-9.]+)" cy="([0-9.]+)" r="10" fill="[^"]*"/>)svg");
    std::vector<std::pair<double, double>> legend_centers;
    for (std::sregex_iterator it(text.begin(), text.end(), legend_circle), end; it != end; ++it)
        legend_centers.emplace_back(std::stod((*it)[1].str()), std::stod((*it)[2].str()));
    REQUIRE(legend_centers.size() == 4);
    for (size_t i = 1; i < 3; ++i)
        CHECK_THAT(legend_centers[i].second, WithinAbs(legend_centers[0].second, 1e-4));
    CHECK_THAT(legend_centers[1].first - legend_centers[0].first, WithinAbs(200., 1e-4));
    CHECK_THAT(legend_centers[2].first - legend_centers[1].first, WithinAbs(200., 1e-4));
    CHECK_THAT(legend_centers[3].first, WithinAbs(legend_centers[0].first, 1e-4));
    // The columns are 20 mm apart (200 SVG units); a 1.3 mm row step is 13 units.
    // SVG's flipped Y axis makes the next row's screen coordinate smaller.
    CHECK_THAT(legend_centers[3].second - legend_centers[0].second, WithinAbs(-13., 1e-4));
}

namespace svg_import_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double PX_TO_MM = 25.4 / 96.0;

bool write_svg(const ScopedTemporaryFile &file, const std::string &body)
{
    std::ofstream out(file.string(), std::ios::binary);
    out << body;
    return out.good();
}

const TriangleMesh &single_mesh(const Model &model)
{
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    return model.objects.front()->volumes.front()->mesh();
}

void check_box(const TriangleMesh &mesh, double x, double y, double z, double expected_volume)
{
    TriangleMesh volume_probe(mesh);
    const Vec3d size = mesh.size();
    CHECK_THAT(size.x(), WithinAbs(x, 0.15));
    CHECK_THAT(size.y(), WithinAbs(y, 0.15));
    CHECK_THAT(size.z(), WithinAbs(z, 0.15));
    CHECK(std::isfinite(volume_probe.volume()));
    CHECK(volume_probe.volume() > 0.f);
    CHECK_THAT(double(volume_probe.volume()),
               WithinAbs(expected_volume, expected_volume * 0.05 + 0.2));
}

} // namespace

TEST_CASE("SVG importer reports missing and malformed files", "[SvgFormat]")
{
    ScopedTemporaryFile missing(".svg");
    Model model;
    std::string message;
    CHECK_FALSE(load_svg(missing.string().c_str(), &model, message));
    CHECK(message.find("could not open") != std::string::npos);

    ScopedTemporaryFile malformed(".svg");
    REQUIRE(write_svg(malformed, "<svg xmlns=\"http://www.w3.org/2000/svg\"><path"));
    model.clear_objects();
    message.clear();
    CHECK_FALSE(load_svg(malformed.string().c_str(), &model, message));
    CHECK(message.find("could not parse") != std::string::npos);
}

TEST_CASE("SVG importer creates a physical rectangle", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"20mm\">"
        "<path fill=\"#112233\" d=\"M0 0 L10 0 L10 20 L0 20 Z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    const TriangleMesh &mesh = single_mesh(model);
    check_box(mesh, 10. * PX_TO_MM, 20. * PX_TO_MM, 10., 10. * 20. * PX_TO_MM * PX_TO_MM * 10.);
    CHECK(model.objects.front()->volumes.front()->name == "part_1");
}

TEST_CASE("SVG importer preserves separate filled components", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"red\" d=\"M0 0 L4 0 L4 5 L0 5 Z\"/>"
        "<path fill=\"blue\" d=\"M8 0 L14 0 L14 3 L8 3 Z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    CHECK(model.objects.front()->volumes[0]->name == "part_1");
    CHECK(model.objects.front()->volumes[1]->name == "part_2");
    check_box(model.objects.front()->volumes[0]->mesh(), 4. * PX_TO_MM, 5. * PX_TO_MM, 10.,
              4. * 5. * PX_TO_MM * PX_TO_MM * 10.);
    check_box(model.objects.front()->volumes[1]->mesh(), 6. * PX_TO_MM, 3. * PX_TO_MM, 10.,
              6. * 3. * PX_TO_MM * PX_TO_MM * 10.);
}

TEST_CASE("SVG importer turns an open stroke into a solid", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"none\" stroke=\"#000000\" stroke-width=\"2\" "
        "d=\"M0 0 L10 0\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    const TriangleMesh &mesh = single_mesh(model);
    const Vec3d size = mesh.size();
    CHECK_THAT(size.x(), WithinAbs(11. * PX_TO_MM, 0.15));
    CHECK_THAT(size.y(), WithinAbs(1. * PX_TO_MM, 0.15));
    CHECK_THAT(size.z(), WithinAbs(10., 0.15));
}

TEST_CASE("SVG importer evaluates cubic path geometry", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#40a0ff\" d=\"M0 0 C0 10 10 10 10 0 Z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    const TriangleMesh &mesh = single_mesh(model);
    const Vec3d size = mesh.size();
    CHECK_THAT(size.x(), WithinAbs(10. * PX_TO_MM, 0.15));
    CHECK_THAT(size.y(), WithinAbs(7.5 * PX_TO_MM, 0.15));
    CHECK_THAT(size.z(), WithinAbs(10., 0.15));
    TriangleMesh volume_probe(mesh);
    CHECK_THAT(double(volume_probe.volume()), WithinAbs(60. * PX_TO_MM * PX_TO_MM * 10., 2.0));
}

TEST_CASE("SVG importer keeps an inner contour as a hole", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#bbbbbb\" d=\"M0 0 L20 0 L20 20 L0 20 Z "
        "M5 5 L5 15 L15 15 L15 5 Z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    const TriangleMesh &mesh = single_mesh(model);
    check_box(mesh, 20. * PX_TO_MM, 20. * PX_TO_MM, 10.,
              (20. * 20. - 10. * 10.) * PX_TO_MM * PX_TO_MM * 10.);
    TriangleMesh volume_probe(mesh);
    const double bounding_volume = mesh.size().x() * mesh.size().y() * mesh.size().z();
    CHECK(volume_probe.volume() < 0.85 * bounding_volume);
    CHECK(volume_probe.volume() > 0.65 * bounding_volume);
}

TEST_CASE("SVG importer accepts gradient-backed shapes", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\"><defs>"
        "<linearGradient id=\"g\"><stop offset=\"0\" stop-color=\"red\"/>"
        "<stop offset=\"1\" stop-color=\"blue\"/></linearGradient></defs>"
        "<path fill=\"url(#g)\" d=\"M0 0 L6 0 L6 7 L0 7\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_box(single_mesh(model), 6. * PX_TO_MM, 7. * PX_TO_MM, 10.,
              6. * 7. * PX_TO_MM * PX_TO_MM * 10.);
}

TEST_CASE("SVG importer handles a self-intersecting contour", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#ffffff\" d=\"M0 0 L10 10 L0 10 L10 0 Z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->volumes.size() == 1);
    const TriangleMesh &mesh = model.objects.front()->volumes.front()->mesh();
    CHECK(mesh.facets_count() > 0);
    TriangleMesh volume_probe(mesh);
    CHECK(std::isfinite(volume_probe.volume()));
}

TEST_CASE("SVG importer rejects an SVG with no drawable shapes", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file, "<svg xmlns=\"http://www.w3.org/2000/svg\"></svg>"));
    Model model;
    std::string message;
    CHECK_FALSE(load_svg(file.string().c_str(), &model, message));
    CHECK(message.find("could not parse") != std::string::npos);
}

TEST_CASE("SVG importer accepts a path with a relative command mix", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#00cc88\" d=\"m2 3 h8 v4 h-8 z\"/></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_box(single_mesh(model), 8. * PX_TO_MM, 4. * PX_TO_MM, 10.,
              8. * 4. * PX_TO_MM * PX_TO_MM * 10.);
}

TEST_CASE("SVG importer keeps distinct element names for styled paths", "[SvgFormat]")
{
    ScopedTemporaryFile file(".svg");
    REQUIRE(write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<g><path fill=\"red\" d=\"M0 0 L2 0 L2 2 L0 2 Z\"/></g>"
        "<g><path fill=\"blue\" d=\"M4 0 L7 0 L7 1 L4 1 Z\"/></g></svg>"));
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    CHECK(model.objects.front()->volumes[0]->name != model.objects.front()->volumes[1]->name);
    CHECK(model.objects.front()->volumes[0]->mesh().facets_count() > 0);
    CHECK(model.objects.front()->volumes[1]->mesh().facets_count() > 0);
}

} // namespace svg_import_contracts

namespace svg_stroke_contracts {
using namespace Slic3r;

TEST_CASE("SVG filled paths suppress outlines while open paths retain visible stroke widths", "[SVG][SVGStrokeWidth]")
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
    std::ifstream input(output.string());
    REQUIRE(input.good());
    const std::string text{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
    const std::regex path_style(R"svg(<path\b[^>]*style="([^"]*)")svg");
    const std::regex stroke_width(R"svg((?:^|;)\s*stroke-width:\s*([0-9.]+))svg");
    std::vector<std::string> styles;
    for (std::sregex_iterator it(text.begin(), text.end(), path_style), end; it != end; ++it)
        styles.push_back((*it)[1].str());
    REQUIRE(styles.size() == 3);
    const std::vector<double> expected_widths{0., 3., 2.}; // SVG coordinates use 10 units per mm.
    for (size_t i = 0; i < styles.size(); ++i) {
        std::smatch match;
        REQUIRE(std::regex_search(styles[i], match, stroke_width));
        CHECK_THAT(std::stod(match[1].str()), Catch::Matchers::WithinAbs(expected_widths[i], 1e-6));
    }
    CHECK(styles[0].find("fill: red") != std::string::npos);
    CHECK(styles[1].find("fill: none") != std::string::npos);
    CHECK(styles[1].find("stroke: blue") != std::string::npos);
    CHECK(styles[2].find("fill: none") != std::string::npos);
    CHECK(styles[2].find("stroke: green") != std::string::npos);
}

} // namespace svg_stroke_contracts
