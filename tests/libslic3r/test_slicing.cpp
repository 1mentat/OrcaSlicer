#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "libslic3r/Slicing.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

using namespace Slic3r;

namespace {

constexpr unsigned char guard_byte = 0xa5;

struct TextureRun {
    int                         rows = 0;
    int                         cols = 0;
    bool                        lod = false;
    int                         cells = 0;
    int                         requested_cells = 0;
    int                         logical_capacity = 0;
    int                         lod_cells = 0;
    std::size_t                 promised_bytes = 0;
    std::vector<unsigned char>  data;
};

SlicingParameters parameters(double height, double min_height, double layer_height = 0.2)
{
    SlicingParameters out;
    out.layer_height = layer_height;
    out.min_layer_height = min_height;
    out.max_layer_height = std::max(layer_height, 0.4);
    out.object_print_z_min = 0.;
    out.object_print_z_max = height;
    return out;
}

std::vector<coordf_t> complete_layers(double height, double step)
{
    std::vector<coordf_t> out;
    for (double lo = 0.; lo < height; ) {
        const double hi = std::min(height, lo + step);
        out.push_back(lo);
        out.push_back(hi);
        if (hi >= height)
            break;
        lo = hi;
    }
    return out;
}

TextureRun run_texture(int rows, int cols, bool lod, double height, double min_height,
                       double layer_height = 0.2, double profile_step = 0.2)
{
    std::vector<coordf_t> layers = complete_layers(height, profile_step);
    if (!layers.empty())
        layers.back() = height;
    TextureRun out;
    out.rows = rows;
    out.cols = cols;
    out.lod = lod;
    const int base_capacity = (cols - 1) * rows;
    const int lod_rows = rows / 2;
    const int lod_cols = cols / 2;
    out.lod_cells = (lod_cols - 1) * lod_rows;
    out.logical_capacity = lod ? std::min(base_capacity, 2 * out.lod_cells) : base_capacity;
    out.requested_cells = static_cast<int>(std::ceil(16. * height / min_height));
    out.promised_bytes = 4 * (static_cast<std::size_t>(rows) * cols +
                             (lod ? static_cast<std::size_t>(rows / 2) * (cols / 2) : 0));
    // Keep an oversized canary so regressions in mip dimensions fail assertions
    // without corrupting the test process. Exact allocations are sanitizer-tested.
    out.data.assign(out.promised_bytes + static_cast<std::size_t>(rows) * cols * 8 + 32, guard_byte);
    out.cells = generate_layer_height_texture(parameters(height, min_height, layer_height), layers,
                                               out.data.data(), rows, cols, lod);
    return out;
}

TextureRun run_profile(int rows, int cols, bool lod, double height, double min_height,
                       const std::vector<coordf_t> &layers, double layer_height = 0.2)
{
    TextureRun out;
    out.rows = rows;
    out.cols = cols;
    out.lod = lod;
    const int base_capacity = (cols - 1) * rows;
    const int lod_rows = rows / 2;
    const int lod_cols = cols / 2;
    out.lod_cells = (lod_cols - 1) * lod_rows;
    out.logical_capacity = lod ? std::min(base_capacity, 2 * out.lod_cells) : base_capacity;
    out.requested_cells = static_cast<int>(std::ceil(16. * height / min_height));
    out.promised_bytes = 4 * (static_cast<std::size_t>(rows) * cols +
                             (lod ? static_cast<std::size_t>(rows / 2) * (cols / 2) : 0));
    out.data.assign(out.promised_bytes + static_cast<std::size_t>(rows) * cols * 8 + 32, guard_byte);
    out.cells = generate_layer_height_texture(parameters(height, min_height, layer_height), layers,
                                               out.data.data(), rows, cols, lod);
    return out;
}

bool outside_guard_is_unchanged(const TextureRun &run)
{
    return std::all_of(run.data.begin() + run.promised_bytes, run.data.end(),
                       [](unsigned char value) { return value == guard_byte; });
}

bool slot_has_alpha(const std::vector<unsigned char> &data, std::size_t rgba_slot)
{
    return data[rgba_slot * 4 + 3] == 255;
}

bool alpha_slots_are_valid(const std::vector<unsigned char> &data, std::size_t byte_count)
{
    for (std::size_t i = 3; i < byte_count; i += 4)
        if (data[i] != 255 && data[i] != guard_byte)
            return false;
    return true;
}

void check_full_base_cells(const TextureRun &run)
{
    REQUIRE(run.cells == std::min(run.logical_capacity, run.requested_cells));
    REQUIRE(run.cells >= 2);
    REQUIRE(outside_guard_is_unchanged(run));
    REQUIRE(alpha_slots_are_valid(run.data, static_cast<std::size_t>(run.rows) * run.cols * 4));

    const int base_rows_used = (run.cells - 1) / (run.cols - 1) + 1;
    REQUIRE(base_rows_used <= (run.lod ? run.rows / 2 : run.rows));
    for (int cell = 0; cell < run.cells; ++cell) {
        const int row = cell / (run.cols - 1);
        const int col = cell % (run.cols - 1);
        REQUIRE(slot_has_alpha(run.data, static_cast<std::size_t>(row * run.cols + col)));
    }
}

void check_lod_cells(const TextureRun &run)
{
    REQUIRE(run.lod);
    const int lod_count = run.cells / 2;
    REQUIRE(lod_count >= 1);
    for (int cell = 0; cell < lod_count; ++cell) {
        const int cols1 = run.cols / 2;
        const int row = cell / (cols1 - 1);
        const int col = cell % (cols1 - 1);
        const std::size_t slot = static_cast<std::size_t>(run.rows) * run.cols + row * cols1 + col;
        REQUIRE(slot_has_alpha(run.data, slot));
    }
    REQUIRE((lod_count - 1) / (run.cols / 2 - 1) + 1 <= run.rows / 2);
}

} // namespace

TEST_CASE("Layer texture base-only output uses the complete RGBA packing capacity", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(8, 8, false, 10., 0.01);
    check_full_base_cells(run);
    REQUIRE(run.cells == 56);
}

TEST_CASE("Layer texture LOD reserves a bounded half-resolution row domain", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(8, 8, true, 10., 0.01);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == 24); // 2 * ((8 / 2 - 1) * (8 / 2)).
}

TEST_CASE("Layer texture LOD remains inside the packed mipmap at a larger square size", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(32, 32, true, 100., 0.01);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == 2 * (32 / 2 - 1) * (32 / 2));
}

TEST_CASE("Layer texture LOD fits both packed levels for a non-square texture", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(8, 16, true, 20., 0.01);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == 2 * (16 / 2 - 1) * (8 / 2));
}

TEST_CASE("Layer texture odd dimensions use the rounded-down mip allocation", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(7, 9, true, 20., 0.01);
    REQUIRE(run.promised_bytes == 4 * (7 * 9 + 3 * 4));
    REQUIRE(run.cells == 18);
    check_full_base_cells(run);
    check_lod_cells(run);
}

TEST_CASE("Layer texture saturates an overflowing cell request before converting to int", "[Slicing][LayerTextureContracts]")
{
    const SlicingParameters params = parameters(1000., std::numeric_limits<double>::min());
    std::vector<unsigned char> data(8 * 8 * 5, guard_byte);
    REQUIRE(generate_layer_height_texture(params, {}, data.data(), 8, 8, true) == 24);
    REQUIRE(std::all_of(data.begin(), data.end(), [](unsigned char value) { return value == guard_byte; }));
}

TEST_CASE("Layer texture LOD maps a partial layer interval to an independent cell prefix", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_profile(16, 16, true, 2., 0.2, {0., 0.2});
    REQUIRE(run.cells == run.logical_capacity);
    const int expected_base_cells = 12; // floor(.2 * (112 - 1) / 2) + 1.
    for (int cell = 0; cell < expected_base_cells; ++cell) {
        const int row = cell / (run.cols - 1);
        const int col = cell % (run.cols - 1);
        REQUIRE(slot_has_alpha(run.data, static_cast<std::size_t>(row * run.cols + col)));
    }
    const int cols1 = run.cols / 2;
    const int expected_lod_cells = 6; // floor(.2 * (56 - 1) / 2) + 1.
    for (int cell = 0; cell < expected_lod_cells; ++cell) {
        const int row = cell / (cols1 - 1);
        const int col = cell % (cols1 - 1);
        const std::size_t slot = static_cast<std::size_t>(run.rows) * run.cols + row * cols1 + col;
        REQUIRE(slot_has_alpha(run.data, slot));
    }
    const std::size_t first_unexpected_lod = static_cast<std::size_t>(run.rows) * run.cols * 4 + expected_lod_cells * 4;
    REQUIRE(run.data[first_unexpected_lod + 3] == guard_byte);
}

TEST_CASE("Layer texture exact LOD capacity is accepted without touching the guard", "[Slicing][LayerTextureContracts]")
{
    // 16 * 0.6 / 0.4 == 24, exactly 2 * ((8 / 2 - 1) * (8 / 2)).
    const TextureRun run = run_texture(8, 8, true, 0.6, 0.4, 0.8, 0.6);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == run.logical_capacity);
}

TEST_CASE("Layer texture request above LOD capacity clamps both levels consistently", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(16, 16, true, 100., 0.001);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == run.logical_capacity);
}

TEST_CASE("Layer texture without LOD keeps the full base row range", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(16, 16, false, 100., 0.001);
    check_full_base_cells(run);
    REQUIRE(run.cells == run.logical_capacity);
    REQUIRE((run.cells - 1) / (run.cols - 1) + 1 == run.rows);
}

TEST_CASE("Layer texture does not write a second level when LOD is disabled", "[Slicing][LayerTextureContracts]")
{
    constexpr int rows = 16;
    constexpr int cols = 16;
    const std::vector<coordf_t> layers = complete_layers(2., 0.2);
    // Use the real GUI allocation even when this invocation requests base-only output.
    std::vector<unsigned char> data(static_cast<std::size_t>(rows) * cols * 5, guard_byte);
    const int cells = generate_layer_height_texture(parameters(2., 0.2), layers,
                                                     data.data(), rows, cols, false);
    REQUIRE(cells == std::min((cols - 1) * rows, 160));
    const std::size_t base_bytes = static_cast<std::size_t>(rows) * cols * 4;
    REQUIRE(std::all_of(data.begin() + base_bytes, data.end(),
                        [](unsigned char value) { return value == guard_byte; }));
}

TEST_CASE("Layer texture keeps a short but valid profile finite and addressable", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(16, 16, true, 0.05, 0.2, 0.05, 0.05);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(outside_guard_is_unchanged(run));
}

TEST_CASE("Layer texture raises a sub-cell request to a safe interpolation domain", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(8, 8, true, 0.01, 0.4, 0.01, 0.01);
    REQUIRE(run.requested_cells == 1);
    REQUIRE(run.cells == 4);
    REQUIRE(outside_guard_is_unchanged(run));
    check_lod_cells(run);
}

TEST_CASE("Layer texture rejects an empty Z domain without dividing by zero", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(16, 16, true, 0., 0.2, 0.2, 0.2);
    REQUIRE(run.cells == 0);
    REQUIRE(outside_guard_is_unchanged(run));
}

TEST_CASE("Layer texture high-resolution profiles stay within the GUI allocation", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_texture(1024, 1024, true, 1000., 0.4, 0.4, 0.4);
    check_full_base_cells(run);
    check_lod_cells(run);
    REQUIRE(run.cells == 40000);
}

TEST_CASE("Layer texture reports the base cell count independently of LOD storage", "[Slicing][LayerTextureContracts]")
{
    const TextureRun base = run_texture(32, 32, false, 20., 0.01);
    const TextureRun lod = run_texture(32, 32, true, 20., 0.01);
    REQUIRE(base.cells == std::min((32 - 1) * 32, base.requested_cells));
    REQUIRE(lod.cells == std::min(2 * (32 / 2 - 1) * (32 / 2), lod.requested_cells));
    REQUIRE(base.cells > lod.cells);
}


TEST_CASE("Layer texture high-resolution LOD trigger stays inside the 1024 squared canary allocation", "[Slicing][LayerTextureContracts]")
{
    // The small minimum height drives the old implementation past the half-resolution
    // mip allocation while the backing store retains a local canary after the GUI bytes.
    const TextureRun run = run_texture(1024, 1024, true, 1000., 0.01, 0.4, 0.4);
    REQUIRE(run.cells == 2 * (1024 / 2 - 1) * (1024 / 2));
    REQUIRE(slot_has_alpha(run.data, 0));
    REQUIRE(slot_has_alpha(run.data, static_cast<std::size_t>(run.rows) * run.cols));
    REQUIRE(outside_guard_is_unchanged(run));
}

TEST_CASE("Layer texture encodes the documented palette and row-edge duplication", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_profile(2, 4, false, 0.2, 0.2, {0., 0.2});
    REQUIRE(run.cells == 6);
    const auto rgba = [&](std::size_t slot) {
        return std::array<unsigned char, 4>{run.data[slot * 4], run.data[slot * 4 + 1],
                                             run.data[slot * 4 + 2], run.data[slot * 4 + 3]};
    };
    // A 0.2 mm layer at z=0 has the fixed palette midpoint scaled by cos(0.35*pi).
    REQUIRE(rgba(0) == std::array<unsigned char, 4>{107, 107, 107, 255});
    // Cell 3 starts the second packed row; its value is duplicated into row 0's edge cell.
    REQUIRE(rgba(4) == std::array<unsigned char, 4>{230, 230, 229, 255});
    REQUIRE(rgba(3) == rgba(4));
}


TEST_CASE("Layer texture rejects invalid dimensions and scalar domains before arithmetic", "[Slicing][LayerTextureContracts]")
{
    const std::vector<coordf_t> layers = {0., 0.2};
    SlicingParameters params = parameters(0.2, 0.2);
    std::vector<unsigned char> data(8 * 8 * 5, guard_byte);
    REQUIRE(generate_layer_height_texture(params, layers, nullptr, 8, 8, true) == 0);
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), 0, 8, true) == 0);
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), 8, 0, true) == 0);
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), 8, -1, true) == 0);
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), std::numeric_limits<int>::max(), 2, true) == 0);
    params.min_layer_height = 0.;
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), 8, 8, true) == 0);
    params.min_layer_height = 0.2;
    params.object_print_z_max = 0.;
    REQUIRE(generate_layer_height_texture(params, layers, data.data(), 8, 8, true) == 0);
}


TEST_CASE("Layer texture mip samples preserve the palette midpoint without layer intensity", "[Slicing][LayerTextureContracts]")
{
    const TextureRun run = run_profile(8, 8, true, 0.2, 0.2, {0., 0.2});
    REQUIRE(run.cells == 16);
    // Nominal layer height is halfway between palette entries (217,241,235)
    // and (254,230,235); mip samples omit the base texture's cosine intensity.
    const std::size_t first_mip_byte = 8 * 8 * 4;
    CHECK(run.data[first_mip_byte] == 236);
    CHECK(run.data[first_mip_byte + 1] == 236);
    CHECK(run.data[first_mip_byte + 2] == 235);
    CHECK(run.data[first_mip_byte + 3] == 255);
    CHECK(outside_guard_is_unchanged(run));
}
