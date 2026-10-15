#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Color.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <vector>
#include <utility>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

void require_rgb(const ColorRGB &actual, float r, float g, float b)
{
    CHECK_THAT(actual.r(), WithinAbs(r, 1e-6));
    CHECK_THAT(actual.g(), WithinAbs(g, 1e-6));
    CHECK_THAT(actual.b(), WithinAbs(b, 1e-6));
}

void require_rgba(const ColorRGBA &actual, float r, float g, float b, float a)
{
    CHECK_THAT(actual.r(), WithinAbs(r, 1e-6));
    CHECK_THAT(actual.g(), WithinAbs(g, 1e-6));
    CHECK_THAT(actual.b(), WithinAbs(b, 1e-6));
    CHECK_THAT(actual.a(), WithinAbs(a, 1e-6));
}

} // namespace

TEST_CASE("ColorRGB float construction clamps each channel", "[Color]")
{
    require_rgb(ColorRGB(-0.25f, 0.25f, 1.25f), 0.0f, 0.25f, 1.0f);
}

TEST_CASE("ColorRGBA float construction clamps alpha independently", "[Color]")
{
    require_rgba(ColorRGBA(0.1f, -1.0f, 0.75f, 2.0f), 0.1f, 0.0f, 0.75f, 1.0f);
}

TEST_CASE("8-bit color construction preserves byte endpoints", "[Color]")
{
    require_rgb(ColorRGB(static_cast<unsigned char>(0), static_cast<unsigned char>(128), static_cast<unsigned char>(255)), 0.0f, 128.0f / 255.0f, 1.0f);
    require_rgba(ColorRGBA(static_cast<unsigned char>(255), static_cast<unsigned char>(64), static_cast<unsigned char>(0), static_cast<unsigned char>(127)), 1.0f, 64.0f / 255.0f, 0.0f, 127.0f / 255.0f);
}

TEST_CASE("Color channel setters clamp indexed assignments", "[Color]")
{
    ColorRGB rgb;
    rgb.r(-1.0f);
    rgb.g(0.4f);
    rgb.b(4.0f);
    rgb.set(1, 2.0f);
    require_rgb(rgb, 0.0f, 1.0f, 1.0f);

    ColorRGBA rgba;
    rgba.a(-0.1f);
    rgba.set(0, 0.3f);
    require_rgba(rgba, 0.3f, 1.0f, 1.0f, 0.0f);
}

TEST_CASE("RGB addition saturates without wrapping", "[Color]")
{
    const ColorRGB result = ColorRGB(0.7f, 0.2f, 0.0f) + ColorRGB(0.6f, 0.5f, 0.4f);
    require_rgb(result, 1.0f, 0.7f, 0.4f);
}

TEST_CASE("RGB scalar multiplication clamps and supports both operand orders", "[Color]")
{
    const ColorRGB color(0.2f, 0.5f, 0.8f);
    require_rgb(color * 3.0f, 0.6f, 1.0f, 1.0f);
    require_rgb(0.5f * color, 0.1f, 0.25f, 0.4f);
}

TEST_CASE("RGBA multiplication preserves alpha while scaling RGB", "[Color]")
{
    const ColorRGBA result = ColorRGBA(0.2f, 0.5f, 0.8f, 0.35f) * 2.0f;
    require_rgba(result, 0.4f, 1.0f, 1.0f, 0.35f);
}

TEST_CASE("Color ordering is lexicographic over RGB channels", "[Color]")
{
    CHECK(ColorRGB(0.1f, 0.9f, 0.9f) < ColorRGB(0.2f, 0.0f, 0.0f));
    CHECK(ColorRGB(0.2f, 0.0f, 0.0f) > ColorRGB(0.1f, 0.9f, 0.9f));
    CHECK_FALSE(ColorRGB(0.2f, 0.0f, 0.0f) < ColorRGB(0.2f, 0.0f, 0.0f));
    CHECK_FALSE(ColorRGBA(0.2f, 0.0f, 0.0f, 0.1f) < ColorRGBA(0.2f, 0.0f, 0.0f, 0.9f));
    CHECK_FALSE(ColorRGBA(0.2f, 0.0f, 0.0f, 0.1f) > ColorRGBA(0.2f, 0.0f, 0.0f, 0.9f));
}

TEST_CASE("RGB interpolation preserves endpoints and weighted channel values", "[Color]")
{
    const ColorRGB red = ColorRGB::RED();
    const ColorRGB blue = ColorRGB::BLUE();
    require_rgb(lerp(red, blue, 0.0f), 1.0f, 0.0f, 0.0f);
    require_rgb(lerp(red, blue, 1.0f), 0.0f, 0.0f, 1.0f);
    require_rgb(lerp(red, blue, 0.25f), 0.75f, 0.0f, 0.25f);
}

TEST_CASE("RGBA interpolation blends opaque RGB channels", "[Color]")
{
    const ColorRGBA first(1.0f, 0.0f, 0.0f, 1.0f);
    const ColorRGBA second(0.0f, 0.0f, 1.0f, 1.0f);
    require_rgba(lerp(first, second, 0.5f), 0.5f, 0.0f, 0.5f, 1.0f);
}

TEST_CASE("Complementary colors invert RGB and retain RGBA alpha", "[Color]")
{
    require_rgb(complementary(ColorRGB(0.2f, 0.4f, 0.8f)), 0.8f, 0.6f, 0.2f);
    require_rgba(complementary(ColorRGBA(0.2f, 0.4f, 0.8f, 0.3f)), 0.8f, 0.6f, 0.2f, 0.3f);
}

TEST_CASE("Saturation factor zero produces value-preserving gray", "[Color]")
{
    const ColorRGB gray = saturate(ColorRGB(0.2f, 0.6f, 0.4f), 0.0f);
    require_rgb(gray, 0.6f, 0.6f, 0.6f);
}

TEST_CASE("Saturation factor above one is bounded and alpha is retained", "[Color]")
{
    const ColorRGB color(0.2f, 0.6f, 0.4f);
    const ColorRGB saturated = saturate(color, 2.0f);
    require_rgb(saturated, 0.0f, 0.6f, 0.3f);

    const ColorRGBA rgba = saturate(ColorRGBA(0.2f, 0.6f, 0.4f, 0.27f), 0.0f);
    CHECK_THAT(rgba.a(), WithinAbs(0.27f, 1e-6));
    CHECK_THAT(rgba.r(), WithinAbs(rgba.g(), 1e-6));
    CHECK_THAT(rgba.g(), WithinAbs(rgba.b(), 1e-6));
}

TEST_CASE("Opposite color keeps the documented hue offset and bounds", "[Color]")
{
    const ColorRGB result = opposite(ColorRGB::RED());
    // At hue 65 degrees, green is the maximum and blue the minimum.
    // Random saturation/value cancel in this channel ratio.
    REQUIRE(result.g() > result.b());
    CHECK_THAT((result.r() - result.b()) / (result.g() - result.b()), WithinAbs(55.0f / 60.0f, 1e-5));
    CHECK(result.r() >= 0.0f);
    CHECK(result.r() <= 1.0f);
    CHECK(result.g() >= 0.0f);
    CHECK(result.g() <= 1.0f);
    CHECK(result.b() >= 0.0f);
    CHECK(result.b() <= 1.0f);
}

TEST_CASE("Opposite pair chooses the longer hue arc midpoint with its offset", "[Color]")
{
    const ColorRGB result = opposite(ColorRGB::RED(), ColorRGB::GREEN());
    // Red/green place the longer-arc midpoint at 240 degrees, plus 5 degrees.
    REQUIRE(result.b() > result.g());
    CHECK_THAT((result.r() - result.g()) / (result.b() - result.g()), WithinAbs(5.0f / 60.0f, 1e-5));
    CHECK(result.r() >= 0.0f);
    CHECK(result.g() <= 1.0f);
    CHECK(result.b() >= 0.0f);
}

TEST_CASE("Color shape validation distinguishes RGB and RGBA lengths", "[Color]")
{
    CHECK(can_decode_color("#123456"));
    CHECK(can_decode_color("#12345678"));
    CHECK_FALSE(can_decode_color("123456"));
    CHECK_FALSE(can_decode_color("#12345"));
    CHECK_FALSE(can_decode_color("#123456789"));
}

TEST_CASE("RGB hexadecimal decoding accepts upper and lower case digits", "[Color]")
{
    ColorRGB upper;
    ColorRGB lower;
    REQUIRE(decode_color("#A0400F", upper));
    REQUIRE(decode_color("#a0400f", lower));
    require_rgb(upper, 160.0f / 255.0f, 64.0f / 255.0f, 15.0f / 255.0f);
    CHECK(upper == lower);
}

TEST_CASE("RGBA hexadecimal decoding preserves explicit alpha", "[Color]")
{
    ColorRGBA color;
    REQUIRE(decode_color("#10203080", color));
    require_rgba(color, 16.0f / 255.0f, 32.0f / 255.0f, 48.0f / 255.0f, 128.0f / 255.0f);
}

TEST_CASE("Structural decode failure preserves RGB and resets RGBA", "[Color]")
{
    ColorRGB rgb(0.2f, 0.3f, 0.4f);
    CHECK_FALSE(decode_color("red", rgb));
    require_rgb(rgb, 0.2f, 0.3f, 0.4f);

    ColorRGBA rgba(0.2f, 0.3f, 0.4f, 0.5f);
    CHECK_FALSE(decode_color("#12345", rgba));
    require_rgba(rgba, 0.0f, 0.0f, 0.0f, 1.0f);
}

TEST_CASE("RGB vector decoding is all-successful and preserves order", "[Color]")
{
    const std::vector<std::string> input{"#FF0000", "#008000", "#0000FF"};
    std::vector<ColorRGB> output;
    REQUIRE(decode_colors(input, output));
    REQUIRE(output.size() == input.size());
    CHECK(output[0] == ColorRGB::RED());
    CHECK_THAT(output[1].g(), WithinAbs(128.0f / 255.0f, 1e-6));
    CHECK(output[2] == ColorRGB::BLUE());
}

TEST_CASE("RGBA vector decoding reports the first invalid entry without reordering", "[Color]")
{
    const std::vector<std::string> input{"#11223344", "invalid", "#AABBCCDD"};
    std::vector<ColorRGBA> output;
    CHECK_FALSE(decode_colors(input, output));
    REQUIRE(output.size() == input.size());
    require_rgba(output[0], 17.0f / 255.0f, 34.0f / 255.0f, 51.0f / 255.0f, 68.0f / 255.0f);
    require_rgba(output[1], 0.0f, 0.0f, 0.0f, 1.0f);
    require_rgba(output[2], 0.0f, 0.0f, 0.0f, 1.0f);
}

TEST_CASE("RGB encoding uses uppercase bytes and exact padding", "[Color]")
{
    CHECK(encode_color(ColorRGB(1.0f, 64.0f / 255.0f, 0.0f)) == "#FF4000");
    CHECK(encode_color(ColorRGB(0.0f, 0.0f, 0.0f)) == "#000000");
}

TEST_CASE("RGBA encoding intentionally serializes only RGB", "[Color]")
{
    CHECK(encode_color(ColorRGBA(1.0f, 64.0f / 255.0f, 0.0f, 0.1f)) == "#FF4000");
    CHECK(encode_color(ColorRGBA(1.0f, 64.0f / 255.0f, 0.0f, 0.9f)) == "#FF4000");
}

TEST_CASE("RGB and RGBA conversions preserve channels and define alpha", "[Color]")
{
    const ColorRGB rgb(0.1f, 0.2f, 0.3f);
    const ColorRGBA opaque = to_rgba(rgb);
    const ColorRGBA translucent = to_rgba(rgb, 0.4f);
    require_rgba(opaque, 0.1f, 0.2f, 0.3f, 1.0f);
    require_rgba(translucent, 0.1f, 0.2f, 0.3f, 0.4f);
    require_rgb(to_rgb(translucent), 0.1f, 0.2f, 0.3f);
}

TEST_CASE("Picking IDs round-trip the three packed byte channels", "[Color]")
{
    for (const auto bytes : {std::array<unsigned char, 3>{0, 0, 0},
                             std::array<unsigned char, 3>{1, 2, 3},
                             std::array<unsigned char, 3>{255, 128, 17}}) {
        const unsigned int id = picking_encode(bytes[0], bytes[1], bytes[2]);
        const ColorRGBA decoded = picking_decode(id);
        CHECK(decoded.r_uchar() == bytes[0]);
        CHECK(decoded.g_uchar() == bytes[1]);
        CHECK(decoded.b_uchar() == bytes[2]);
    }
}

TEST_CASE("Picking checksum has stable known vectors", "[Color]")
{
    CHECK(picking_checksum_alpha_channel(0, 0, 0) == 85);
    CHECK(picking_checksum_alpha_channel(1, 2, 3) == 48);
    CHECK(picking_checksum_alpha_channel(255, 0, 0) == 176);
    CHECK(picking_checksum_alpha_channel(12, 34, 56) == 39);
}

TEST_CASE("Color equality uses the documented component tolerance", "[Color]")
{
    const RGBA base{0.2f, 0.4f, 0.6f, 0.8f};
    const RGBA near{0.205f, 0.395f, 0.605f, 0.795f};
    const RGBA far{0.3f, 0.4f, 0.6f, 0.8f};
    CHECK(color_is_equal(base, near));
    CHECK_FALSE(color_is_equal(base, far));
}

TEST_CASE("Named color constants expose their documented channels", "[Color]")
{
    require_rgb(ColorRGB::BLACK(), 0.0f, 0.0f, 0.0f);
    require_rgb(ColorRGB::WHITE(), 1.0f, 1.0f, 1.0f);
    require_rgb(ColorRGB::ORCA(), 0.0f, 150.0f / 255.0f, 136.0f / 255.0f);
    require_rgba(ColorRGBA::RED(), 1.0f, 0.0f, 0.0f, 1.0f);
}

TEST_CASE("Saturation identity preserves each hue sector including gray", "[Color]")
{
    const auto color = GENERATE(ColorRGB::RED(), ColorRGB::YELLOW(), ColorRGB::GREEN(),
                                ColorRGB::CYAN(), ColorRGB::BLUE(), ColorRGB::MAGENTA(),
                                ColorRGB::BLACK(), ColorRGB::GRAY());
    const ColorRGB result = saturate(color, 1.0f);
    require_rgb(result, color.r(), color.g(), color.b());
}

TEST_CASE("Color ordering resolves green and blue ties independently of alpha", "[Color]")
{
    const auto pair = GENERATE(std::make_pair(ColorRGB(0.2f, 0.1f, 0.8f), ColorRGB(0.2f, 0.3f, 0.0f)),
                               std::make_pair(ColorRGB(0.2f, 0.3f, 0.1f), ColorRGB(0.2f, 0.3f, 0.8f)));
    CHECK(pair.first < pair.second);
    CHECK(pair.second > pair.first);
    CHECK_FALSE(pair.second < pair.first);
    CHECK_FALSE(pair.first > pair.second);
    CHECK(to_rgba(pair.first, 1.0f) < to_rgba(pair.second, 0.0f));
    CHECK(to_rgba(pair.second, 0.0f) > to_rgba(pair.first, 1.0f));
}

TEST_CASE("Empty color lists replace prior output with an empty successful result", "[Color]")
{
    std::vector<ColorRGB> rgb{ColorRGB::RED()};
    std::vector<ColorRGBA> rgba{ColorRGBA::RED()};
    REQUIRE(decode_colors({}, rgb));
    REQUIRE(decode_colors({}, rgba));
    CHECK(rgb.empty());
    CHECK(rgba.empty());
}
