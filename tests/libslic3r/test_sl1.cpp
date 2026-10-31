#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <miniz.h>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Format/SL1.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/SLA/RasterBase.hpp"
#include "libslic3r/SLAPrint.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Zipper.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/miniz_extension.hpp"

#include "test_utils.hpp"

#include <boost/nowide/fstream.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ios>
#include <map>
#include <stdexcept>
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

SLAPrinterConfig raster_config()
{
    SLAPrinterConfig cfg;
    cfg.display_width.value = 32.;
    cfg.display_height.value = 24.;
    cfg.display_pixels_x.value = 32;
    cfg.display_pixels_y.value = 24;
    cfg.display_orientation.value = sladoLandscape;
    cfg.display_mirror_x.value = false;
    cfg.display_mirror_y.value = false;
    cfg.gamma_correction.value = 0.;
    return cfg;
}

std::string profile_ini(bool portrait = false, bool mirror_x = false, bool mirror_y = false)
{
    return "display_width = 32\n"
           "display_height = 24\n"
           "display_pixels_x = 32\n"
           "display_pixels_y = 24\n"
           "display_orientation = " + std::string(portrait ? "portrait" : "landscape") + "\n"
           "display_mirror_x = " + std::string(mirror_x ? "1" : "0") + "\n"
           "display_mirror_y = " + std::string(mirror_y ? "1" : "0") + "\n"
           "layer_height = 0.1\n"
           "initial_layer_height = 0.1\n";
}

std::string config_ini()
{
    return "jobDir = Fixture\nlayerHeight = 0.1\n";
}

sla::EncodedRaster make_layer_png(const ExPolygon &shape)
{
    sla::RasterBase::Trafo trafo{sla::RasterBase::roLandscape, sla::RasterBase::NoMirror};
    auto raster = sla::create_raster_grayscale_aa({32, 24}, {1., 1.}, 0., trafo);
    raster->draw(shape);
    return raster->encode(sla::PNGRasterEncoder{});
}

sla::EncodedRaster make_layer_png(bool large = true)
{
    return make_layer_png(large ? square(4., 3., 27., 20.) : square(10., 8., 20., 16.));
}

struct RawEntry {
    std::string name;
    std::vector<uint8_t> bytes;
};

RawEntry raw_entry(const std::string &name, const sla::EncodedRaster &raster)
{
    const auto *begin = static_cast<const uint8_t *>(raster.data());
    return {name, std::vector<uint8_t>(begin, begin + raster.size())};
}

void write_archive(const std::string &path,
                   const std::string &config,
                   const std::string &profile,
                   const std::vector<RawEntry> &entries)
{
    Zipper zipper(path, Zipper::NO_COMPRESSION);
    if (!config.empty()) {
        zipper.add_entry("config.ini");
        zipper << config;
    }
    if (!profile.empty()) {
        zipper.add_entry("prusaslicer.ini");
        zipper << profile;
    }
    for (const RawEntry &entry : entries)
        zipper.add_entry(entry.name, entry.bytes.data(), entry.bytes.size());
    zipper.finalize();
}

std::vector<std::pair<std::string, size_t>> archive_entries(const std::string &path)
{
    MZ_Archive archive;
    if (!open_zip_reader(&archive.arch, path))
        throw std::runtime_error("unable to open generated SLA archive");

    std::vector<std::pair<std::string, size_t>> result;
    const mz_uint count = mz_zip_reader_get_num_files(&archive.arch);
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat stat;
        if (mz_zip_reader_file_stat(&archive.arch, i, &stat))
            result.emplace_back(stat.m_filename, size_t(stat.m_uncomp_size));
    }
    close_zip_reader(&archive.arch);
    return result;
}

void require_mesh(const indexed_triangle_set &mesh)
{
    REQUIRE_FALSE(mesh.vertices.empty());
    REQUIRE_FALSE(mesh.indices.empty());
    CHECK(std::abs(its_volume(mesh)) > 0.f);
    for (const auto &face : mesh.indices) {
        REQUIRE(face[0] < mesh.vertices.size());
        REQUIRE(face[1] < mesh.vertices.size());
        REQUIRE(face[2] < mesh.vertices.size());
    }
    for (const Vec3f &vertex : mesh.vertices) {
        CHECK(std::isfinite(vertex.x()));
        CHECK(std::isfinite(vertex.y()));
        CHECK(std::isfinite(vertex.z()));
    }
}

long long quantized(float value)
{
    return std::llround(static_cast<double>(value) * 100000.);
}

std::vector<std::array<long long, 9>> canonical_geometry(const indexed_triangle_set &mesh)
{
    std::vector<std::array<long long, 9>> result;
    result.reserve(mesh.indices.size());
    for (const auto &face : mesh.indices) {
        std::array<std::array<long long, 3>, 3> vertices;
        for (size_t i = 0; i < 3; ++i) {
            const Vec3f &vertex = mesh.vertices[face[i]];
            vertices[i] = {quantized(vertex.x()), quantized(vertex.y()), quantized(vertex.z())};
        }
        std::sort(vertices.begin(), vertices.end());
        std::array<long long, 9> triangle{};
        for (size_t i = 0; i < 3; ++i)
            for (size_t j = 0; j < 3; ++j)
                triangle[i * 3 + j] = vertices[i][j];
        result.push_back(triangle);
    }
    std::sort(result.begin(), result.end());
    return result;
}

std::array<float, 4> xy_bounds(const indexed_triangle_set &mesh)
{
    REQUIRE_FALSE(mesh.vertices.empty());
    std::array<float, 4> bounds{mesh.vertices.front().x(), mesh.vertices.front().x(),
                                mesh.vertices.front().y(), mesh.vertices.front().y()};
    for (const Vec3f &vertex : mesh.vertices) {
        bounds[0] = std::min(bounds[0], vertex.x());
        bounds[1] = std::max(bounds[1], vertex.x());
        bounds[2] = std::min(bounds[2], vertex.y());
        bounds[3] = std::max(bounds[3], vertex.y());
    }
    return bounds;
}

std::vector<std::array<float, 4>> layer_bounds(const indexed_triangle_set &mesh)
{
    std::map<long long, std::array<float, 4>> grouped;
    for (const Vec3f &vertex : mesh.vertices) {
        const long long z = quantized(vertex.z());
        auto [it, inserted] = grouped.emplace(z, std::array<float, 4>{vertex.x(), vertex.x(), vertex.y(), vertex.y()});
        if (!inserted) {
            it->second[0] = std::min(it->second[0], vertex.x());
            it->second[1] = std::max(it->second[1], vertex.x());
            it->second[2] = std::min(it->second[2], vertex.y());
            it->second[3] = std::max(it->second[3], vertex.y());
        }
    }
    std::vector<std::array<float, 4>> result;
    result.reserve(grouped.size());
    for (const auto &[z, bounds] : grouped)
        result.push_back(bounds);
    return result;
}

} // namespace

TEST_CASE("SL1 archive rasterizes and exports named layers", "[SL1]")
{
    SLAPrinterConfig cfg = raster_config();
    SL1Archive archive(cfg);
    const ExPolygon shape = square(4., 3., 27., 20.);
    std::atomic<int> drawn{0};
    archive.draw_layers(2, [&](sla::RasterBase &raster, size_t) {
        raster.draw(shape);
        ++drawn;
    }, []() { return false; });
    REQUIRE(drawn == 2);

    SLAPrint print;
    ScopedTemporaryFile output(".sl1");
    archive.export_print(output.string(), print, "fixture");
    const auto entries = archive_entries(output.string());
    // config.ini plus two layers. prusaslicer.ini is absent: the unconfigured SLAPrint has an
    // empty full config, and Zipper drops entries with no data.
    REQUIRE(entries.size() == 3);
    CHECK(std::find_if(entries.begin(), entries.end(), [](const auto &e) { return e.first == "config.ini"; }) != entries.end());
    const auto layer0 = std::find_if(entries.begin(), entries.end(), [](const auto &e) { return e.first == "fixture00000.png"; });
    const auto layer1 = std::find_if(entries.begin(), entries.end(), [](const auto &e) { return e.first == "fixture00001.png"; });
    REQUIRE(layer0 != entries.end());
    REQUIRE(layer1 != entries.end());
    CHECK(layer0->second > 0);
    CHECK(layer1->second > 0);
}

TEST_CASE("SL1 archive configuration changes invalidate encoded layers", "[SL1]")
{
    SLAPrinterConfig cfg = raster_config();
    SL1Archive archive(cfg);
    archive.draw_layers(1, [](sla::RasterBase &raster, size_t) { raster.draw(square(2., 2., 8., 8.)); }, []() { return false; });

    SLAPrinterConfig changed = cfg;
    changed.display_width.value = 33.;
    archive.apply(changed);

    SLAPrint print;
    ScopedTemporaryFile output(".sl1");
    archive.export_print(output.string(), print, "invalidated");
    const auto entries = archive_entries(output.string());
    CHECK(std::none_of(entries.begin(), entries.end(), [](const auto &e) { return e.first.find(".png") != std::string::npos; }));
    CHECK(std::count_if(entries.begin(), entries.end(), [](const auto &e) { return e.first == "config.ini"; }) == 1);
}

TEST_CASE("SL1 import reads profile and builds a mesh from raster layers", "[SL1]")
{
    const sla::EncodedRaster first = make_layer_png(true);
    const sla::EncodedRaster second = make_layer_png(true);
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), profile_ini(),
                  {raw_entry("layer00000.png", first), raw_entry("layer00001.png", second)});

    indexed_triangle_set mesh;
    DynamicPrintConfig profile;
    const auto substitutions = import_sla_archive(archive.string(), {1, 1}, mesh, profile);
    REQUIRE(substitutions.empty());
    require_mesh(mesh);
    REQUIRE(profile.has("display_width"));
    CHECK_THAT(profile.option<ConfigOptionFloat>("display_width")->value, WithinAbs(32., 1e-6));
    CHECK_THAT(profile.option<ConfigOptionFloat>("layer_height")->value, WithinAbs(.1, 1e-6));
}

TEST_CASE("SL1 import clamps a zero marching-squares window", "[SL1]")
{
    // Diagonal edges make the contour depend on the window size; an axis-aligned square does not.
    ExPolygon diamond;
    diamond.contour.points = {Point(scaled(16.), scaled(2.)), Point(scaled(29.), scaled(12.)),
                              Point(scaled(16.), scaled(22.)), Point(scaled(3.), scaled(12.))};
    const sla::EncodedRaster layer = make_layer_png(diamond);
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), profile_ini(),
                  {raw_entry("layer00000.png", layer), raw_entry("layer00001.png", layer)});

    indexed_triangle_set zero_mesh, unit_mesh;
    DynamicPrintConfig zero_profile, unit_profile;
    import_sla_archive(archive.string(), {0, 0}, zero_mesh, zero_profile);
    import_sla_archive(archive.string(), {1, 1}, unit_mesh, unit_profile);
    require_mesh(zero_mesh);
    // Unclamped, marching squares would substitute its own default window for zero.
    CHECK(canonical_geometry(zero_mesh) == canonical_geometry(unit_mesh));
}

TEST_CASE("SL1 import honors progress cancellation without overwriting output", "[SL1]")
{
    const sla::EncodedRaster layer = make_layer_png(true);
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), profile_ini(),
                  {raw_entry("layer00000.png", layer), raw_entry("layer00001.png", layer), raw_entry("layer00002.png", layer)});

    indexed_triangle_set mesh = its_make_cube(2., 2., 2.);
    const float original_volume = its_volume(mesh);
    const auto original_geometry = canonical_geometry(mesh);
    DynamicPrintConfig profile;
    std::atomic<int> callbacks{0};
    import_sla_archive(archive.string(), {1, 1}, mesh, profile, [&](int) {
        ++callbacks;
        return false;
    });
    CHECK(callbacks.load() > 0);
    CHECK_THAT(its_volume(mesh), WithinAbs(original_volume, 1e-5));
    CHECK(canonical_geometry(mesh) == original_geometry);
}

TEST_CASE("SL1 import recovers layer height when profile is absent", "[SL1]")
{
    const sla::EncodedRaster first = make_layer_png(true);
    const sla::EncodedRaster second = make_layer_png(true);
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), {},
                  {raw_entry("layer00000.png", first), raw_entry("layer00001.png", second)});

    DynamicPrintConfig fallback;
    std::string fallback_profile = profile_ini();
    fallback_profile.replace(fallback_profile.find("layer_height = 0.1"), 18, "layer_height = 0.2");
    fallback_profile.replace(fallback_profile.find("initial_layer_height = 0.1"), 26, "initial_layer_height = 0.2");
    fallback.load_from_ini_string(fallback_profile, ForwardCompatibilitySubstitutionRule::Enable);
    indexed_triangle_set mesh;
    import_sla_archive(archive.string(), {1, 1}, mesh, fallback);
    require_mesh(mesh);
    CHECK(fallback.empty());
    float max_z = 0.f;
    for (const Vec3f &vertex : mesh.vertices)
        max_z = std::max(max_z, vertex.z());
    CHECK_THAT(max_z, WithinAbs(.2f, .05f));
}

TEST_CASE("SL1 import sorts case-insensitive image names and excludes thumbnails", "[SL1]")
{
    const sla::EncodedRaster small = make_layer_png(false);
    const sla::EncodedRaster large = make_layer_png(true);
    ScopedTemporaryFile unsorted(".sl1"), sorted(".sl1");
    const std::vector<RawEntry> unsorted_layers{
        raw_entry("Z_LAYER.PNG", large), raw_entry("a_layer.png", small), raw_entry("thumbnail.PNG", large)};
    const std::vector<RawEntry> sorted_layers{
        raw_entry("a_layer.png", small), raw_entry("Z_LAYER.PNG", large)};
    write_archive(unsorted.string(), config_ini(), profile_ini(), unsorted_layers);
    write_archive(sorted.string(), config_ini(), profile_ini(), sorted_layers);

    indexed_triangle_set unsorted_mesh, sorted_mesh;
    DynamicPrintConfig profile_a, profile_b;
    import_sla_archive(unsorted.string(), {1, 1}, unsorted_mesh, profile_a);
    import_sla_archive(sorted.string(), {1, 1}, sorted_mesh, profile_b);
    require_mesh(unsorted_mesh);
    require_mesh(sorted_mesh);
    CHECK_THAT(std::abs(its_volume(unsorted_mesh)), WithinAbs(std::abs(its_volume(sorted_mesh)), .05));
    // The archive entry order differs, but case-insensitive filename sorting must make the
    // decoded layer sequence and therefore the full triangle geometry identical.
    CHECK(canonical_geometry(unsorted_mesh) == canonical_geometry(sorted_mesh));
    const auto layers = layer_bounds(unsorted_mesh);
    REQUIRE(layers.size() >= 2);
    const float first_width = layers.front()[1] - layers.front()[0];
    const float last_width = layers.back()[1] - layers.back()[0];
    const float first_height = layers.front()[3] - layers.front()[2];
    const float last_height = layers.back()[3] - layers.back()[2];
    // The small layer is "a_layer.png" and the large one "Z_LAYER.PNG": a case-sensitive sort
    // would put 'Z' (0x5A) before 'a' (0x61). This semantic assertion makes the filename
    // ordering observable instead of comparing two archives that could both be decoded in
    // the same wrong order.
    CHECK(first_width < last_width);
    CHECK(first_height < last_height);
}

TEST_CASE("SL1 import returns an empty mesh for undecodable raster data", "[SL1]")
{
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), profile_ini(),
                  {{"layer00000.png", {0, 1, 2, 3}}, {"layer00001.png", {4, 5, 6, 7}}});

    indexed_triangle_set mesh;
    DynamicPrintConfig profile;
    import_sla_archive(archive.string(), {1, 1}, mesh, profile);
    CHECK(mesh.empty());
}

TEST_CASE("SL1 import rejects a profile without raster dimensions", "[SL1]")
{
    const sla::EncodedRaster layer = make_layer_png(true);
    ScopedTemporaryFile archive(".sl1");
    write_archive(archive.string(), config_ini(), "layer_height = 0.1\ninitial_layer_height = 0.1\n",
                  {raw_entry("layer00000.png", layer), raw_entry("layer00001.png", layer)});

    indexed_triangle_set mesh;
    DynamicPrintConfig profile;
    CHECK_THROWS_AS(import_sla_archive(archive.string(), {1, 1}, mesh, profile), MissingProfileError);
}

TEST_CASE("SL1 import rejects malformed INI and corrupt zip inputs", "[SL1]")
{
    const sla::EncodedRaster layer = make_layer_png(true);
    ScopedTemporaryFile malformed_ini(".sl1");
    write_archive(malformed_ini.string(), config_ini(), "[broken\n",
                  {raw_entry("layer00000.png", layer), raw_entry("layer00001.png", layer)});
    DynamicPrintConfig profile;
    CHECK_THROWS(import_sla_archive(malformed_ini.string(), profile));

    ScopedTemporaryFile corrupt_zip(".sl1");
    {
        boost::nowide::ofstream out(corrupt_zip.string(), std::ios::binary);
        out << "not a zip archive";
    }
    CHECK_THROWS(import_sla_archive(corrupt_zip.string(), profile));
}

TEST_CASE("SL1 import applies portrait and mirror raster transforms", "[SL1]")
{
    // The large fixture square is centred on the 32x24 px raster, so mirroring it is a no-op.
    // Use an off-centre, non-square shape so mirror and portrait transforms are observable.
    const sla::EncodedRaster first = make_layer_png(square(2., 3., 20., 16.));
    const sla::EncodedRaster second = make_layer_png(square(2., 3., 20., 16.));
    ScopedTemporaryFile landscape(".sl1"), portrait(".sl1"), mirror_x(".sl1"), mirror_y(".sl1");
    const std::vector<RawEntry> layers{raw_entry("layer00000.png", first), raw_entry("layer00001.png", second)};
    write_archive(landscape.string(), config_ini(), profile_ini(false, false, false), layers);
    write_archive(portrait.string(), config_ini(), profile_ini(true, false, false), layers);
    write_archive(mirror_x.string(), config_ini(), profile_ini(false, true, false), layers);
    write_archive(mirror_y.string(), config_ini(), profile_ini(false, false, true), layers);

    indexed_triangle_set normal_mesh, portrait_mesh, mirror_x_mesh, mirror_y_mesh;
    DynamicPrintConfig normal_profile, portrait_profile, mirror_x_profile, mirror_y_profile;
    import_sla_archive(landscape.string(), {1, 1}, normal_mesh, normal_profile);
    import_sla_archive(portrait.string(), {1, 1}, portrait_mesh, portrait_profile);
    import_sla_archive(mirror_x.string(), {1, 1}, mirror_x_mesh, mirror_x_profile);
    import_sla_archive(mirror_y.string(), {1, 1}, mirror_y_mesh, mirror_y_profile);
    require_mesh(normal_mesh);
    require_mesh(portrait_mesh);
    require_mesh(mirror_x_mesh);
    require_mesh(mirror_y_mesh);

    const auto normal_bounds = xy_bounds(normal_mesh);
    const auto portrait_bounds = xy_bounds(portrait_mesh);
    const auto mirror_x_bounds = xy_bounds(mirror_x_mesh);
    const auto mirror_y_bounds = xy_bounds(mirror_y_mesh);
    CHECK_THAT(normal_bounds[0] + mirror_x_bounds[1], WithinAbs(32.f, .25f));
    CHECK_THAT(normal_bounds[1] + mirror_x_bounds[0], WithinAbs(32.f, .25f));
    CHECK_THAT(normal_bounds[2] + mirror_y_bounds[3], WithinAbs(24.f, .25f));
    CHECK_THAT(normal_bounds[3] + mirror_y_bounds[2], WithinAbs(24.f, .25f));
    CHECK_THAT(normal_bounds[1] - normal_bounds[0], WithinAbs(portrait_bounds[3] - portrait_bounds[2], .25f));
    CHECK_THAT(normal_bounds[3] - normal_bounds[2], WithinAbs(portrait_bounds[1] - portrait_bounds[0], .25f));
}
