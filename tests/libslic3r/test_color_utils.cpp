#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/TextureToColor/ColorUtils.hpp"

using namespace Slic3r::tex2color::color_utils;
using Catch::Matchers::WithinAbs;

// Expected values come from the reference formulas in Sharma, Wu and Dalal (2005),
// "The CIEDE2000 Color-Difference Formula: Implementation Notes", applied to the
// sRGB -> XYZ -> Lab (D65) conversion used by ColorUtils.
TEST_CASE("CIEDE2000 matches the published formula", "[ColorUtils]")
{
    SECTION("the T term uses the 63 degree offset") {
        const RGB green{0, 255, 0};
        const RGB magenta{255, 0, 255};
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000(green, magenta), WithinAbs(111.41433059755204, 1e-6));
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000(magenta, green), WithinAbs(111.41433059755204, 1e-6));
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000_srgb01({0., 1., 0.}, {1., 0., 1.}), WithinAbs(111.41433059755204, 1e-6));
    }

    SECTION("the mean hue wraps down when the hue sum exceeds 360 degrees") {
        const RGB violet{96, 32, 255};
        const RGB brown{160, 96, 64};
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000(violet, brown), WithinAbs(45.73921346241746, 1e-6));
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000(brown, violet), WithinAbs(45.73921346241746, 1e-6));
    }

    SECTION("black against white is unaffected by hue") {
        CHECK_THAT(calc_rgb_color_difference_by_ciede2000({0, 0, 0}, {255, 255, 255}), WithinAbs(100.00000386666865, 1e-6));
    }
}
