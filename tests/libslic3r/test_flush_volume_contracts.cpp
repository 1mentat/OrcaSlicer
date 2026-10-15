#include <array>
#include <cstddef>

#include <catch2/catch_all.hpp>

#include "catch2/catch_test_macros.hpp"
#include "libslic3r/FlushVolCalc.hpp"

using namespace Slic3r;

namespace {

// Dataset 1 deliberately selects the analytic RGB/HSV path in calc_flush_vol_rgb().
// Keeping these tests on that entry point avoids depending on the optional predictor
// data files while still exercising the production color conversion and distance code.
constexpr int analytic_dataset = 1;
constexpr int no_predictor_dataset = -1;

struct Rgb {
    unsigned char r;
    unsigned char g;
    unsigned char b;
};

} // namespace

TEST_CASE("Flush volume identity has the analytic minimum", "[FlushVolCalculator]")
{
    FlushVolCalculator calculator(0, g_max_flush_volume, analytic_dataset);
    const std::array<Rgb, 9> colors {{
        {0, 0, 0},
        {255, 255, 255},
        {255, 0, 0},
        {0, 255, 0},
        {0, 0, 255},
        {255, 255, 0},
        {255, 0, 255},
        {0, 255, 255},
        {128, 128, 128},
    }};

    for (const Rgb color : colors) {
        const int flush = calculator.calc_flush_vol_rgb(color.r, color.g, color.b,
                                                        color.r, color.g, color.b);
        CHECK(flush == 60);
    }
}

TEST_CASE("Flush volume covers representative RGB sectors", "[FlushVolCalculator]")
{
    FlushVolCalculator calculator(0, g_max_flush_volume, analytic_dataset);

    // These pairs cross each non-gray primary sector. A non-floor result proves that
    // hue/saturation distance participates in the calculation rather than only luminance.
    const std::array<std::array<Rgb, 2>, 3> sector_pairs {{
        {{{255, 0, 0}, {0, 255, 0}}},
        {{{0, 255, 0}, {0, 0, 255}}},
        {{{0, 0, 255}, {255, 0, 0}}},
    }};

    for (const auto &pair : sector_pairs) {
        const int flush = calculator.calc_flush_vol_rgb(
            pair[0].r, pair[0].g, pair[0].b,
            pair[1].r, pair[1].g, pair[1].b);
        CHECK(flush > 60);
    }
}

TEST_CASE("Flush volume is directional for a luminance change", "[FlushVolCalculator]")
{
    FlushVolCalculator calculator(0, g_max_flush_volume, analytic_dataset);

    const int dark_to_light = calculator.calc_flush_vol_rgb(0, 0, 0, 255, 255, 255);
    const int light_to_dark = calculator.calc_flush_vol_rgb(255, 255, 255, 0, 0, 0);

    CHECK(dark_to_light > light_to_dark);
    CHECK(light_to_dark >= 60);
}

TEST_CASE("Flush volume treats transparent colors as white and applies limits", "[FlushVolCalculator]")
{
    // An invalid dataset id is intentional: it makes calc_flush_vol() use the same
    // analytic fallback without reading a predictor resource file.
    FlushVolCalculator calculator(37, 10000, no_predictor_dataset);

    const int opaque_white_source = calculator.calc_flush_vol(
        255, 255, 255, 255, 255, 0, 0, 0);
    const int transparent_source = calculator.calc_flush_vol(
        0, 0, 0, 0, 255, 0, 0, 0);
    const int opaque_white_destination = calculator.calc_flush_vol(
        255, 0, 0, 0, 255, 255, 255, 255);
    const int transparent_destination = calculator.calc_flush_vol(
        255, 0, 0, 0, 0, 0, 0, 0);

    CHECK(transparent_source == opaque_white_source);
    CHECK(transparent_destination == opaque_white_destination);

    FlushVolCalculator without_minimum(0, 10000, no_predictor_dataset);
    CHECK(opaque_white_source == without_minimum.calc_flush_vol(
        255, 255, 255, 255, 255, 0, 0, 0) + 37);

    FlushVolCalculator capped(0, 100, no_predictor_dataset);
    const int uncapped = without_minimum.calc_flush_vol(
        255, 255, 255, 255, 255, 0, 0, 0);
    CHECK(uncapped > 100);
    CHECK(capped.calc_flush_vol(255, 255, 255, 255, 255, 0, 0, 0) == 100);
}
