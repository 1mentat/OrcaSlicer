#include <catch2/catch_test_macros.hpp>

#include "libslic3r/FlushVolCalc.hpp"

#include <array>

using namespace Slic3r;

namespace {

// Dataset 1 makes calc_flush_vol_rgb use its analytic RGB/HSV path directly.
constexpr int analytic_dataset = 1;

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

