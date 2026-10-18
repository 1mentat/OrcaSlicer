#include <Font_FontAspect.hxx>
#include <Font_FontMgr.hxx>
#include <Font_SystemFont.hxx>
#include <Standard_Handle.hxx>
#include <TCollection_AsciiString.hxx>

#include <Eigen/Geometry>
#include <admesh/stl.h>

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Shape/TextShape.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/BoundingBox.hpp"

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

std::filesystem::path bundled_font_path()
{
    return std::filesystem::path(std::string(TEST_DATA_DIR)).parent_path().parent_path() /
           "resources/fonts/Sarabun-Medium.ttf";
}

Handle(Font_SystemFont) register_bundled_font()
{
    const std::filesystem::path path = bundled_font_path();
    REQUIRE(std::filesystem::is_regular_file(path));

    const Handle(Font_FontMgr) manager = Font_FontMgr::GetInstance();
    const Handle(Font_SystemFont) checked = manager->CheckFont(path.u8string().c_str());
    REQUIRE(!checked.IsNull());
    const TCollection_AsciiString font_path = checked->FontPath(Font_FontAspect_Regular);
    REQUIRE_FALSE(font_path.IsEmpty());

    // A test-owned family leaves any host-installed Sarabun family untouched.
    const Handle(Font_SystemFont) font = new Font_SystemFont("OrcaSlicer TextShape Test");
    font->SetFontPath(Font_FontAspect_Regular, font_path,
                      checked->FontFaceId(Font_FontAspect_Regular));
    REQUIRE(manager->RegisterFont(font, true));
    const Handle(Font_SystemFont) registered = manager->GetFont(font->FontName());
    REQUIRE(!registered.IsNull());
    REQUIRE(registered->FontPath(Font_FontAspect_Regular).IsEqual(font_path));
    return registered;
}

struct TextMeshMetrics {
    double width = 0.;
    double height = 0.;
    double depth = 0.;
};

TextMeshMetrics inspect_text_mesh(const TextResult &result)
{
    const TriangleMesh &mesh = result.text_mesh;
    REQUIRE_FALSE(mesh.empty());
    REQUIRE_FALSE(mesh.its.vertices.empty());
    REQUIRE_FALSE(mesh.its.indices.empty());

    for (const Vec3f &vertex : mesh.its.vertices) {
        REQUIRE(std::isfinite(static_cast<double>(vertex.x())));
        REQUIRE(std::isfinite(static_cast<double>(vertex.y())));
        REQUIRE(std::isfinite(static_cast<double>(vertex.z())));
    }
    for (const stl_triangle_vertex_indices &triangle : mesh.its.indices) {
        REQUIRE(triangle[0] >= 0);
        REQUIRE(static_cast<size_t>(triangle[0]) < mesh.its.vertices.size());
        REQUIRE(triangle[1] >= 0);
        REQUIRE(static_cast<size_t>(triangle[1]) < mesh.its.vertices.size());
        REQUIRE(triangle[2] >= 0);
        REQUIRE(static_cast<size_t>(triangle[2]) < mesh.its.vertices.size());
        const Vec3f &a = mesh.its.vertices[triangle[0]];
        const Vec3f &b = mesh.its.vertices[triangle[1]];
        const Vec3f &c = mesh.its.vertices[triangle[2]];
        REQUIRE((b - a).cross(c - a).squaredNorm() > 1e-12f);
    }

    REQUIRE(its_num_open_edges(mesh.its) == 0);
    const BoundingBoxf3 bounds = mesh.bounding_box();
    REQUIRE(bounds.defined);
    TextMeshMetrics metrics;
    metrics.width = bounds.max.x() - bounds.min.x();
    metrics.height = bounds.max.y() - bounds.min.y();
    metrics.depth = bounds.max.z() - bounds.min.z();
    REQUIRE(std::isfinite(metrics.width));
    REQUIRE(std::isfinite(metrics.height));
    REQUIRE(std::isfinite(metrics.depth));
    REQUIRE(metrics.width > 0.);
    REQUIRE(metrics.height > 0.);
    REQUIRE(metrics.depth > 0.);
    return metrics;
}

} // namespace

TEST_CASE("OCCT text shape exposes a closed finite mesh with height and thickness scaling", "[TextShape]")
{
    const Handle(Font_SystemFont) font = register_bundled_font();
    const char *font_name = font->FontName().ToCString();

    TextResult base{};
    load_text_shape("Ag", font_name, 10.f, 0.6f, false, false, base);
    const TextMeshMetrics base_metrics = inspect_text_mesh(base);
    REQUIRE(std::isfinite(base.text_width));
    REQUIRE(base.text_width > 0.);
    CHECK_THAT(base_metrics.depth, WithinAbs(0.6, 2e-3));

    TextResult doubled{};
    load_text_shape("Ag", font_name, 20.f, 0.6f, false, false, doubled);
    const TextMeshMetrics doubled_metrics = inspect_text_mesh(doubled);
    REQUIRE(std::isfinite(doubled.text_width));
    REQUIRE(doubled.text_width > 0.);
    CHECK_THAT(doubled.text_width / base.text_width, WithinRel(2., 2e-3));
    CHECK_THAT(doubled_metrics.width / base_metrics.width, WithinRel(2., 2e-2));
    CHECK_THAT(doubled_metrics.height / base_metrics.height, WithinRel(2., 2e-2));
    CHECK_THAT(doubled_metrics.depth, WithinAbs(0.6, 2e-3));

    TextResult thicker{};
    load_text_shape("Ag", font_name, 10.f, 1.8f, false, false, thicker);
    const TextMeshMetrics thicker_metrics = inspect_text_mesh(thicker);
    REQUIRE(std::isfinite(thicker.text_width));
    CHECK_THAT(thicker.text_width, WithinRel(base.text_width, 2e-3));
    CHECK_THAT(thicker_metrics.width, WithinRel(base_metrics.width, 2e-2));
    CHECK_THAT(thicker_metrics.height, WithinRel(base_metrics.height, 2e-2));
    CHECK_THAT(thicker_metrics.depth, WithinAbs(1.8, 2e-3));
}

TEST_CASE("OCCT text shape preserves the caller result for non-positive thickness", "[TextShape]")
{
    const TriangleMesh sentinel = make_cube(1., 1., 1.);

    const float thickness = GENERATE(0.f, -0.5f);
    TextResult result{sentinel, 37.};
    load_text_shape("A", "unused-font", 10.f, thickness, false, false, result);

    CHECK_THAT(result.text_width, WithinAbs(37., 1e-12));
    REQUIRE(result.text_mesh.its.vertices.size() == sentinel.its.vertices.size());
    REQUIRE(result.text_mesh.its.indices.size() == sentinel.its.indices.size());
    for (size_t i = 0; i < sentinel.its.vertices.size(); ++i) {
        CHECK_THAT(result.text_mesh.its.vertices[i].x(), WithinAbs(sentinel.its.vertices[i].x(), 0.));
        CHECK_THAT(result.text_mesh.its.vertices[i].y(), WithinAbs(sentinel.its.vertices[i].y(), 0.));
        CHECK_THAT(result.text_mesh.its.vertices[i].z(), WithinAbs(sentinel.its.vertices[i].z(), 0.));
    }
    for (size_t i = 0; i < sentinel.its.indices.size(); ++i)
        CHECK(result.text_mesh.its.indices[i] == sentinel.its.indices[i]);

}
