#include "libslic3r/FlushVolPredictor.hpp"
#include "libslic3r/Utils.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <filesystem>
#include <string>

using namespace Slic3r;

namespace {

// Points resources_dir() at the repository's resources/ for the bundled flush datasets and
// restores the previous value afterwards.
struct ScopedRepositoryResources
{
    std::string previous;

    ScopedRepositoryResources() : previous(resources_dir())
    {
        const std::filesystem::path data_dir(TEST_DATA_DIR);
        set_resources_dir((data_dir.parent_path().parent_path() / "resources").string());
    }

    ~ScopedRepositoryResources() { set_resources_dir(previous); }
};

} // namespace

TEST_CASE("Flush predictor RGB to Lab conversion handles dark neutral colors", "[FlushVolPredictor]")
{
    // The grayscale L* reference follows the standard sRGB low-light linearization
    // and CIELAB transfer; equal RGB channels should remain neutral.
    const auto dark_gray = FlushPredict::RGB2LAB({10, 10, 10});
    CHECK_THAT(dark_gray.l, Catch::Matchers::WithinAbs(2.742, 0.01));
    CHECK_THAT(dark_gray.a, Catch::Matchers::WithinAbs(0.0, 0.02));
    CHECK_THAT(dark_gray.b, Catch::Matchers::WithinAbs(0.0, 0.02));

    const auto white = FlushPredict::RGB2LAB({255, 255, 255});
    CHECK_THAT(white.l, Catch::Matchers::WithinAbs(100.0, 0.01));

    const auto red = FlushPredict::RGB2LAB({255, 0, 0});
    // Standard sRGB/D65 red reference, with tolerance for the implementation's
    // rounded RGB-to-XYZ matrix coefficients.
    CHECK_THAT(red.l, Catch::Matchers::WithinAbs(53.2408, 0.02));
    CHECK_THAT(red.a, Catch::Matchers::WithinAbs(80.0925, 0.02));
    CHECK_THAT(red.b, Catch::Matchers::WithinAbs(67.2032, 0.02));

    CHECK(FlushPredict::is_similar_color({10, 10, 10}, {11, 11, 11}, 1.0f));
    CHECK_FALSE(FlushPredict::is_similar_color({0, 0, 0}, {255, 255, 255}, 5.0f));
}

TEST_CASE("Flush predictor CIEDE2000 matches Sharma supplemental pairs", "[FlushVolPredictor]")
{
    // Sharma, Wu, and Dalal, Color Research & Application 30(1), Table I,
    // pairs 1 and 17 (CIEDE2000 supplementary test data; DOI 10.1002/col.20070).
    const FlushPredict::LABColor pair1_first{50.0, 2.6772, -79.7751};
    const FlushPredict::LABColor pair1_second{50.0, 0.0, -82.7485};
    CHECK_THAT(FlushPredict::calc_color_distance(pair1_first, pair1_second),
               Catch::Matchers::WithinAbs(2.0425, 0.0002));

    // Pair 17 crosses the hue-zero boundary and exercises the wrapped mean-hue
    // branch (|h1-h2| > 180 degrees, with h1+h2 < 360 degrees).
    const FlushPredict::LABColor pair17_first{50.0, 2.5000, 0.0000};
    const FlushPredict::LABColor pair17_second{73.0, 25.0000, -18.0000};
    const float forward = FlushPredict::calc_color_distance(pair17_first, pair17_second);
    const float reverse = FlushPredict::calc_color_distance(pair17_second, pair17_first);
    CHECK_THAT(forward, Catch::Matchers::WithinAbs(27.1492, 0.0002));
    CHECK_THAT(reverse, Catch::Matchers::WithinAbs(27.1492, 0.0002));
    CHECK_THAT(FlushPredict::calc_color_distance(
                   FlushPredict::LABColor{0.0, 0.0, 0.0},
                   FlushPredict::LABColor{100.0, 0.0, 0.0}),
               Catch::Matchers::WithinAbs(100.0, 0.0001));
}

TEST_CASE("Flush predictor reads directional bundled datasets", "[FlushVolPredictor]")
{
    ScopedRepositoryResources resources;
    using FlushPredict::RGBColor;

    const RGBColor black{0, 0, 0};
    const RGBColor near_black{1, 1, 1};
    const RGBColor gray{84, 84, 84};
    const RGBColor red{193, 46, 31};
    const RGBColor white{255, 255, 255};

    GenericFlushPredictor standard(0);
    CHECK(standard.get_min_flush_volume() == 26);
    float flush = -1.0f;
    REQUIRE(standard.predict(black, gray, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(236.0f, 0.001f));

    REQUIRE(standard.predict(near_black, gray, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(236.0f, 0.001f));

    flush = 123.0f;
    CHECK_FALSE(standard.predict(black, red, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(123.0f, 0.001f));
    REQUIRE(standard.predict(red, black, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(26.0f, 0.001f));

    GenericFlushPredictor dual_standard(1);
    CHECK(dual_standard.get_min_flush_volume() == 60);
    REQUIRE(dual_standard.predict(black, gray, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(450.0f, 0.001f));
    REQUIRE(dual_standard.predict(black, white, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(900.0f, 0.001f));

    GenericFlushPredictor dual_highflow(2);
    CHECK(dual_highflow.get_min_flush_volume() == 90);
    REQUIRE(dual_highflow.predict(black, gray, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(270.0f, 0.001f));
    REQUIRE(dual_highflow.predict(black, white, flush));
    CHECK_THAT(flush, Catch::Matchers::WithinAbs(900.0f, 0.001f));
}
