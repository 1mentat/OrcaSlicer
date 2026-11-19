#include <cmath>
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
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <fstream>
#include <ios>
#include <map>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <vector>
#if defined(__linux__)
#include <stdlib.h>
#endif

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
    REQUIRE(registered->FontPath(Font_FontAspect_Regular).IsEqual(font_path));
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
    REQUIRE_FALSE(mesh.its.vertices.empty());
    for (const Vec3f &vertex : mesh.its.vertices) {
        REQUIRE(std::isfinite(static_cast<double>(vertex.x())));
        REQUIRE(std::isfinite(static_cast<double>(vertex.y())));
        REQUIRE(std::isfinite(static_cast<double>(vertex.z())));
    }
    size_t degenerate = 0;
    for (const stl_triangle_vertex_indices &triangle : mesh.its.indices) {
        for (int corner = 0; corner < 3; ++corner) {
            REQUIRE(triangle[corner] >= 0);
            REQUIRE(static_cast<size_t>(triangle[corner]) < mesh.its.vertices.size());
        }
        const Vec3f &a = mesh.its.vertices[triangle[0]];
        const Vec3f &b = mesh.its.vertices[triangle[1]];
        const Vec3f &c = mesh.its.vertices[triangle[2]];
        degenerate += (b - a).cross(c - a).squaredNorm() <= 1e-12f;
    }
    CHECK(degenerate == 0);
    REQUIRE_FALSE(mesh.empty());
    REQUIRE(its_num_open_edges(mesh.its) == 0);

    const BoundingBoxf3 bounds = mesh.bounding_box();
    REQUIRE(bounds.defined);
    const TextMeshSize size{bounds.max.x() - bounds.min.x(), bounds.max.y() - bounds.min.y(), bounds.max.z() - bounds.min.z()};
    REQUIRE(std::isfinite(size.width));
    REQUIRE(std::isfinite(size.height));
    REQUIRE(std::isfinite(size.depth));
    REQUIRE(size.width > 0.);
    REQUIRE(size.height > 0.);
    REQUIRE(size.depth > 0.);
    return size;
}

} // namespace

TEST_CASE("OCCT text shape exposes a closed mesh that scales with height and thickness", "[TextShape]")
{
    const Handle(Font_SystemFont) font = register_bundled_font();
    const char *font_name = font->FontName().ToCString();

    TextResult base{};
    load_text_shape("Ag", font_name, 10.f, 0.6f, false, false, base);
    const TextMeshSize base_size = closed_mesh_size(base.text_mesh);
    REQUIRE(std::isfinite(base.text_width));
    REQUIRE(base.text_width > 0.);
    CHECK_THAT(base_size.depth, WithinAbs(0.6, 2e-3));

    SECTION("doubling the text height doubles the outline but keeps the thickness") {
        TextResult doubled{};
        load_text_shape("Ag", font_name, 20.f, 0.6f, false, false, doubled);
        const TextMeshSize doubled_size = closed_mesh_size(doubled.text_mesh);
        REQUIRE(std::isfinite(doubled.text_width));
        REQUIRE(doubled.text_width > 0.);
        CHECK_THAT(doubled.text_width / base.text_width, WithinRel(2., 1e-2));
        CHECK_THAT(doubled_size.width / base_size.width, WithinRel(2., 2e-2));
        CHECK_THAT(doubled_size.height / base_size.height, WithinRel(2., 2e-2));
        CHECK_THAT(doubled_size.depth, WithinAbs(0.6, 2e-3));
    }
    SECTION("tripling the thickness only deepens the mesh") {
        TextResult thicker{};
        load_text_shape("Ag", font_name, 10.f, 1.8f, false, false, thicker);
        const TextMeshSize thicker_size = closed_mesh_size(thicker.text_mesh);
        REQUIRE(std::isfinite(thicker.text_width));
        REQUIRE(thicker.text_width > 0.);
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

TEST_CASE("OCCT text shape exposes a closed finite mesh with height and thickness scaling", "[TextShape]")
{
    const Handle(Font_SystemFont) font = register_bundled_font();
    const char *font_name = font->FontName().ToCString();

    TextResult base{};
    load_text_shape("Ag", font_name, 10.f, 0.6f, false, false, base);
    const TextMeshSize base_metrics = closed_mesh_size(base.text_mesh);
    REQUIRE(std::isfinite(base.text_width));
    REQUIRE(base.text_width > 0.);
    CHECK_THAT(base_metrics.depth, WithinAbs(0.6, 2e-3));

    TextResult doubled{};
    load_text_shape("Ag", font_name, 20.f, 0.6f, false, false, doubled);
    const TextMeshSize doubled_metrics = closed_mesh_size(doubled.text_mesh);
    REQUIRE(std::isfinite(doubled.text_width));
    REQUIRE(doubled.text_width > 0.);
    CHECK_THAT(doubled.text_width / base.text_width, WithinRel(2., 2e-3));
    CHECK_THAT(doubled_metrics.width / base_metrics.width, WithinRel(2., 2e-2));
    CHECK_THAT(doubled_metrics.height / base_metrics.height, WithinRel(2., 2e-2));
    CHECK_THAT(doubled_metrics.depth, WithinAbs(0.6, 2e-3));

    TextResult thicker{};
    load_text_shape("Ag", font_name, 10.f, 1.8f, false, false, thicker);
    const TextMeshSize thicker_metrics = closed_mesh_size(thicker.text_mesh);
    REQUIRE(std::isfinite(thicker.text_width));
    CHECK_THAT(thicker.text_width, WithinRel(base.text_width, 2e-3));
    CHECK_THAT(thicker_metrics.width, WithinRel(base_metrics.width, 2e-2));
    CHECK_THAT(thicker_metrics.height, WithinRel(base_metrics.height, 2e-2));
    CHECK_THAT(thicker_metrics.depth, WithinAbs(1.8, 2e-3));
}

#if defined(__linux__)
namespace {
std::filesystem::path bundled_font_path()
{
    return std::filesystem::path(std::string(TEST_DATA_DIR)).parent_path().parent_path() /
           "resources/fonts/Sarabun-Medium.ttf";
}

class ScopedTemporaryDirectory
{
public:
    ScopedTemporaryDirectory()
    {
        const std::string pattern =
            (std::filesystem::temp_directory_path() / "orca-textshape-fontconfig-XXXXXX").string();
        std::vector<char> writable_pattern(pattern.begin(), pattern.end());
        writable_pattern.push_back('\0');
        char *created = ::mkdtemp(writable_pattern.data());
        if (created == nullptr)
            throw std::system_error(errno, std::generic_category(), "mkdtemp");
        m_path = created;
    }

    ~ScopedTemporaryDirectory()
    {
        std::error_code error;
        std::filesystem::remove_all(m_path, error);
    }

    const std::filesystem::path &path() const { return m_path; }

private:
    std::filesystem::path m_path;
};

std::string xml_escape(const std::string &value)
{
    std::string escaped;
    for (const char character : value) {
        switch (character) {
        case '&': escaped += "&amp;"; break;
        case '<': escaped += "&lt;"; break;
        case '>': escaped += "&gt;"; break;
        case '\"': escaped += "&quot;"; break;
        case '\'': escaped += "&apos;"; break;
        default: escaped += character; break;
        }
    }
    return escaped;
}

class ScopedFontconfigSandbox
{
public:
    ScopedFontconfigSandbox()
    {
        if (const char *configured_file = std::getenv("FONTCONFIG_FILE"))
            m_original_config_file = configured_file;

        m_font_directory = m_temporary_directory.path() / "fonts";
        std::filesystem::create_directory(m_font_directory);
        for (const char *font_name : {"NanumGothic-Regular.ttf",
                                      "NanumGothic-Bold.ttf",
                                      "Sarabun-Medium.ttf"}) {
            const std::filesystem::path source = bundled_font_path().parent_path() / font_name;
            std::filesystem::copy_file(source, m_font_directory / font_name);
        }
        const std::filesystem::path cache_directory = m_temporary_directory.path() / "cache";
        std::filesystem::create_directory(cache_directory);
        m_config_file = m_temporary_directory.path() / "fonts.conf";
        std::ofstream config(m_config_file, std::ios::binary);
        config << "<?xml version=\"1.0\"?>\n"
                  "<!DOCTYPE fontconfig SYSTEM \"urn:fontconfig:fonts.dtd\">\n"
                  "<fontconfig>\n"
               << "  <dir>" << xml_escape(m_font_directory.string()) << "</dir>\n"
               << "  <cachedir>" << xml_escape(cache_directory.string()) << "</cachedir>\n"
                  "</fontconfig>\n";
        config.close();
        if (!config)
            throw std::runtime_error("could not write isolated Fontconfig configuration");

        if (::setenv("FONTCONFIG_FILE", m_config_file.c_str(), 1) != 0)
            throw std::system_error(errno, std::generic_category(), "setenv(FONTCONFIG_FILE)");
        m_environment_overridden = true;
    }

    ~ScopedFontconfigSandbox()
    {
        if (!m_environment_overridden)
            return;

        const int restored = m_original_config_file
            ? ::setenv("FONTCONFIG_FILE", m_original_config_file->c_str(), 1)
            : ::unsetenv("FONTCONFIG_FILE");
        if (restored == 0 && m_manager_may_be_modified) {
            try {
                (void)init_occt_fonts();
            } catch (...) {
            }
        }
    }

    std::vector<std::string> initialize_fonts()
    {
        m_manager_may_be_modified = true;
        return init_occt_fonts();
    }

private:
    ScopedTemporaryDirectory m_temporary_directory;
    std::filesystem::path m_font_directory;
    std::filesystem::path m_config_file;
    std::optional<std::string> m_original_config_file;
    bool m_environment_overridden = false;
    bool m_manager_may_be_modified = false;
};
} // namespace

TEST_CASE("OCCT font enumeration uses configured bundled fonts and filters style families",
          "[TextShape]")
{
    ScopedFontconfigSandbox fontconfig;
    const std::vector<std::string> names = fontconfig.initialize_fonts();
    const std::map<std::string, std::string> fonts = get_occt_fonts_maps();

    REQUIRE(names.size() == fonts.size());
    auto name = names.cbegin();
    for (const auto &font : fonts) {
        CHECK(*name == font.first);
        ++name;
    }

    const auto noto_font = fonts.find("NanumGothic");
    REQUIRE(noto_font != fonts.end());
    CHECK(fonts.find("Sarabun Medium") == fonts.end());
    std::string extension = std::filesystem::path(noto_font->second).extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(),
                   [](unsigned char character) { return static_cast<char>(std::tolower(character)); });
    CHECK((extension == ".ttf" || extension == ".otf" || extension == ".ttc"));
}
#endif // defined(__linux__)
