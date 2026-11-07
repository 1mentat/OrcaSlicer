#include "libslic3r/TexturePainting.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

using namespace Slic3r;

namespace {

using Color = std::array<std::size_t, 3>;

} // namespace

TEST_CASE("Texture painting decodes raw RGBA pixels to contiguous BGR", "[TexturePainting]")
{
    const TextureImage image { 2, 1, 4, { 255, 0, 0, 11, 0, 255, 0, 22 } };
    std::vector<unsigned char> pixels;
    int width = 0;
    int height = 0;
    REQUIRE(decode_texture_to_pixels(image, pixels, width, height));
    CHECK(width == 2);
    CHECK(height == 1);
    CHECK(pixels == std::vector<unsigned char> { 0, 0, 255, 0, 255, 0 });

    TextureImage invalid = image;
    invalid.channels = 2;
    CHECK_FALSE(decode_texture_to_pixels(invalid, pixels, width, height));
}

TEST_CASE("Texture painting chooses the nearest filament for every cluster", "[TexturePainting]")
{
    const std::vector<Color> clusters { { 0, 0, 0 }, { 255, 255, 255 } };
    const std::vector<std::array<float, 4>> filaments {
        { 0.f, 0.f, 0.f, 1.f }, { 1.f, 1.f, 1.f, 1.f },
    };
    const auto matches = match_clusters_to_filaments(clusters, filaments, { "black", "white" });
    REQUIRE(matches.size() == 2);
    CHECK(matches[0].filament_index == 0);
    CHECK(matches[1].filament_index == 1);
    CHECK_THAT(matches[0].delta_e, Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK_THAT(matches[1].delta_e, Catch::Matchers::WithinAbs(0., 1e-12));
}
