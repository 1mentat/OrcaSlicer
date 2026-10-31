#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Format/svg.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_utils.hpp"

#include <boost/nowide/fstream.hpp>

#include <cmath>
#include <ios>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// The importer reads SVG user units as CSS pixels at 96 DPI and extrudes every shape 10 mm.
constexpr double PX_TO_MM = 25.4 / 96.0;
constexpr double HEIGHT   = 10.;

void write_svg(const ScopedTemporaryFile &file, const std::string &body)
{
    boost::nowide::ofstream out(file.string(), std::ios::binary);
    out << body;
    REQUIRE(out.good());
}

const TriangleMesh &single_mesh(const Model &model)
{
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    return model.objects.front()->volumes.front()->mesh();
}

// Checks the extents and the volume of a prism over a w x h pixel profile of the given area.
// Straight-edged profiles import exactly; curves need the looser tolerances.
void check_prism(const TriangleMesh &mesh, double w_px, double h_px, double area_px,
                 double size_tolerance_mm = 0.01, double relative_volume_tolerance = 0.005)
{
    const Vec3d size = mesh.size();
    CHECK_THAT(size.x(), WithinAbs(w_px * PX_TO_MM, size_tolerance_mm));
    CHECK_THAT(size.y(), WithinAbs(h_px * PX_TO_MM, size_tolerance_mm));
    CHECK_THAT(size.z(), WithinAbs(HEIGHT, 0.01));
    const double expected_volume = area_px * PX_TO_MM * PX_TO_MM * HEIGHT;
    const double volume          = TriangleMesh(mesh).volume(); // volume() is not const
    CHECK_THAT(volume, WithinRel(expected_volume, relative_volume_tolerance));
}

} // namespace

TEST_CASE("SVG importer reports missing and malformed files", "[SVGImport]")
{
    ScopedTemporaryFile missing(".svg");
    Model model;
    std::string message;
    CHECK_FALSE(load_svg(missing.string().c_str(), &model, message));
    CHECK(message.find("could not open") != std::string::npos);

    ScopedTemporaryFile malformed(".svg");
    write_svg(malformed, "<svg xmlns=\"http://www.w3.org/2000/svg\"><path");
    model.clear_objects();
    message.clear();
    CHECK_FALSE(load_svg(malformed.string().c_str(), &model, message));
    CHECK(message.find("could not parse") != std::string::npos);
}

TEST_CASE("SVG importer extrudes a filled rectangle from pixel units", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#112233\" d=\"M0 0 L10 0 L10 20 L0 20 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 10., 20., 10. * 20.);
    CHECK(model.objects.front()->volumes.front()->name == "part_1");
}

TEST_CASE("SVG importer preserves separate filled components", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"red\" d=\"M0 0 L4 0 L4 5 L0 5 Z\"/>"
        "<path fill=\"blue\" d=\"M8 0 L14 0 L14 3 L8 3 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    REQUIRE(model.objects.size() == 1);
    const ModelVolumePtrs &volumes = model.objects.front()->volumes;
    REQUIRE(volumes.size() == 2);
    CHECK(volumes[0]->name == "part_1");
    CHECK(volumes[1]->name == "part_2");
    check_prism(volumes[0]->mesh(), 4., 5., 4. * 5.);
    check_prism(volumes[1]->mesh(), 6., 3., 6. * 3.);
}

TEST_CASE("SVG importer turns an open stroke into a solid", "[SVGImport]")
{
    // Explicit width and height keep nanosvg's view scale at 1. Without them a zero-height
    // drawing gets a zero vertical scale, which halves the stroke width.
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"20\" height=\"20\">"
        "<path fill=\"none\" stroke=\"#000000\" stroke-width=\"2\" d=\"M2 5 L12 5\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    // A 2 px wide band whose square caps extend 1 px past each end.
    check_prism(single_mesh(model), 12., 2., 12. * 2.);
}

TEST_CASE("SVG importer evaluates cubic path geometry", "[SVGImport]")
{
    // The curve peaks at 3/4 of its control height and encloses 60 px^2 with its chord,
    // where the control polygon would give 10 px and 100 px^2. The importer samples each
    // curve at 10 points, so the sampled profile is slightly smaller than the true curve.
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#40a0ff\" d=\"M0 0 C0 10 10 10 10 0 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 10., 7.5, 60., 0.05, 0.03);
}

TEST_CASE("SVG importer keeps an inner contour as a hole", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#bbbbbb\" d=\"M0 0 L20 0 L20 20 L0 20 Z M5 5 L5 15 L15 15 L15 5 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 20., 20., 20. * 20. - 10. * 10.);
}

TEST_CASE("SVG importer accepts gradient-backed shapes", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\"><defs>"
        "<linearGradient id=\"g\"><stop offset=\"0\" stop-color=\"red\"/>"
        "<stop offset=\"1\" stop-color=\"blue\"/></linearGradient></defs>"
        "<path fill=\"url(#g)\" d=\"M0 0 L6 0 L6 7 L0 7 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 6., 7., 6. * 7.);
}

// Known bug: gradient shapes close an open path by moving the end of the last sampled edge onto
// the start point instead of adding a closing edge, cutting off a sliver; fixed by the follow-up fix PR.
TEST_CASE("SVG importer closes an open gradient-filled path along its implicit closing edge", "[SVGImport][!shouldfail]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\"><defs>"
        "<linearGradient id=\"g\"><stop offset=\"0\" stop-color=\"red\"/>"
        "<stop offset=\"1\" stop-color=\"blue\"/></linearGradient></defs>"
        "<path fill=\"url(#g)\" d=\"M0 0 L6 0 L6 7 L0 7\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 6., 7., 6. * 7.);
}

TEST_CASE("SVG importer handles a self-intersecting contour", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#ffffff\" d=\"M0 0 L10 10 L0 10 L10 0 Z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    const TriangleMesh &mesh = single_mesh(model);
    CHECK(mesh.facets_count() > 0);
    CHECK(std::isfinite(TriangleMesh(mesh).volume()));
}

TEST_CASE("SVG importer rejects an SVG with no drawable shapes", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file, "<svg xmlns=\"http://www.w3.org/2000/svg\"></svg>");
    Model model;
    std::string message;
    CHECK_FALSE(load_svg(file.string().c_str(), &model, message));
    CHECK(message.find("could not parse") != std::string::npos);
}

TEST_CASE("SVG importer accepts a path with a relative command mix", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<path fill=\"#00cc88\" d=\"m2 3 h8 v4 h-8 z\"/></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    check_prism(single_mesh(model), 8., 4., 8. * 4.);
}

TEST_CASE("SVG importer keeps distinct element names for styled paths", "[SVGImport]")
{
    ScopedTemporaryFile file(".svg");
    write_svg(file,
        "<svg xmlns=\"http://www.w3.org/2000/svg\">"
        "<g><path fill=\"red\" d=\"M0 0 L2 0 L2 2 L0 2 Z\"/></g>"
        "<g><path fill=\"blue\" d=\"M4 0 L7 0 L7 1 L4 1 Z\"/></g></svg>");
    Model model;
    std::string message;
    REQUIRE(load_svg(file.string().c_str(), &model, message));
    REQUIRE(model.objects.size() == 1);
    const ModelVolumePtrs &volumes = model.objects.front()->volumes;
    REQUIRE(volumes.size() == 2);
    CHECK(volumes[0]->name != volumes[1]->name);
    CHECK(volumes[0]->mesh().facets_count() > 0);
    CHECK(volumes[1]->mesh().facets_count() > 0);
}
