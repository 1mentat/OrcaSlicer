#include <Font_FontAspect.hxx>
#include <Font_FontMgr.hxx>
#include <Font_SystemFont.hxx>
#include <Standard_Handle.hxx>
#include <TCollection_AsciiString.hxx>

#include <admesh/stl.h>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Shape/TextShape.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cstddef>
#include <filesystem>
#include <string>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

// Registers the bundled Sarabun font under a test-owned family name, so the
// tests do not depend on the host's installed fonts and leave any host-installed
// Sarabun family untouched. resources/fonts sits beside resources/profiles
// (PROFILES_DIR), so the path is taken from PROFILES_DIR's parent.
Handle(Font_SystemFont) register_bundled_font()
{
    const std::filesystem::path path = std::filesystem::path(std::string(PROFILES_DIR)).parent_path() / "fonts" /
                                       "Sarabun-Medium.ttf";
    REQUIRE(std::filesystem::is_regular_file(path));

    const Handle(Font_FontMgr) manager = Font_FontMgr::GetInstance();
    const Handle(Font_SystemFont) checked = manager->CheckFont(path.u8string().c_str());
    REQUIRE(!checked.IsNull());
    const TCollection_AsciiString font_path = checked->FontPath(Font_FontAspect_Regular);
    REQUIRE_FALSE(font_path.IsEmpty());

    const Handle(Font_SystemFont) font = new Font_SystemFont("OrcaSlicer TextShape Test");
    font->SetFontPath(Font_FontAspect_Regular, font_path, checked->FontFaceId(Font_FontAspect_Regular));
    REQUIRE(manager->RegisterFont(font, true));
    const Handle(Font_SystemFont) registered = manager->GetFont(font->FontName());
    REQUIRE(!registered.IsNull());
    return registered;
}

struct TextMeshSize {
    double width = 0.;
    double height = 0.;
    double depth = 0.;
};

// Requires a closed mesh without degenerate triangles and returns its extents.
TextMeshSize closed_mesh_size(const TriangleMesh &mesh)
{
    REQUIRE_FALSE(mesh.its.indices.empty());
    size_t degenerate = 0;
    for (const stl_triangle_vertex_indices &triangle : mesh.its.indices) {
        const Vec3f &a = mesh.its.vertices[triangle[0]];
        const Vec3f &b = mesh.its.vertices[triangle[1]];
        const Vec3f &c = mesh.its.vertices[triangle[2]];
        degenerate += (b - a).cross(c - a).squaredNorm() <= 1e-12f;
    }
    CHECK(degenerate == 0);
    REQUIRE(its_num_open_edges(mesh.its) == 0);

    const BoundingBoxf3 bounds = mesh.bounding_box();
    REQUIRE(bounds.defined);
    return {bounds.max.x() - bounds.min.x(), bounds.max.y() - bounds.min.y(), bounds.max.z() - bounds.min.z()};
}

} // namespace

TEST_CASE("OCCT text shape exposes a closed mesh that scales with height and thickness", "[TextShape]")
{
    const Handle(Font_SystemFont) font = register_bundled_font();
    const char *font_name = font->FontName().ToCString();

    TextResult base{};
    load_text_shape("Ag", font_name, 10.f, 0.6f, false, false, base);
    const TextMeshSize base_size = closed_mesh_size(base.text_mesh);
    REQUIRE(base.text_width > 0.);
    CHECK_THAT(base_size.depth, WithinAbs(0.6, 2e-3));

    SECTION("doubling the text height doubles the outline but keeps the thickness") {
        TextResult doubled{};
        load_text_shape("Ag", font_name, 20.f, 0.6f, false, false, doubled);
        const TextMeshSize doubled_size = closed_mesh_size(doubled.text_mesh);
        CHECK_THAT(doubled.text_width / base.text_width, WithinRel(2., 1e-2));
        CHECK_THAT(doubled_size.width / base_size.width, WithinRel(2., 2e-2));
        CHECK_THAT(doubled_size.height / base_size.height, WithinRel(2., 2e-2));
        CHECK_THAT(doubled_size.depth, WithinAbs(0.6, 2e-3));
    }
    SECTION("tripling the thickness only deepens the mesh") {
        TextResult thicker{};
        load_text_shape("Ag", font_name, 10.f, 1.8f, false, false, thicker);
        const TextMeshSize thicker_size = closed_mesh_size(thicker.text_mesh);
        CHECK_THAT(thicker.text_width, WithinRel(base.text_width, 1e-2));
        CHECK_THAT(thicker_size.width, WithinRel(base_size.width, 2e-2));
        CHECK_THAT(thicker_size.height, WithinRel(base_size.height, 2e-2));
        CHECK_THAT(thicker_size.depth, WithinAbs(1.8, 2e-3));
    }
}

TEST_CASE("OCCT text shape leaves the caller's result untouched for non-positive thickness", "[TextShape]")
{
    // A real font, so only the thickness guard keeps the result from being overwritten.
    const Handle(Font_SystemFont) font = register_bundled_font();
    const TriangleMesh sentinel = make_cube(1., 1., 1.);

    const float thickness = GENERATE(0.f, -0.5f);
    TextResult result{sentinel, 37.};
    load_text_shape("A", font->FontName().ToCString(), 10.f, thickness, false, false, result);

    CHECK(result.text_width == 37.);
    CHECK(result.text_mesh.its.vertices == sentinel.its.vertices);
    CHECK(result.text_mesh.its.indices == sentinel.its.indices);
}
