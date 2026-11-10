#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/AABBTreeLines.hpp"
#include "libslic3r/GCode/ExtrusionProcessor.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

using namespace Slic3r;

TEST_CASE("A nonempty public point estimate honors its maximum segment length", "[ExtrusionProcessor][PointEstimatorMaxLength]")
{
    const AABBTreeLines::LinesDistancer<Linef> previous_layer(std::vector<Linef>{
        {{0., 0.}, {12., 0.}}, {{12., 0.}, {12., -10.}},
        {{12., -10.}, {0., -10.}}, {{0., -10.}, {0., 0.}}});
    const Points wall{Point::new_scale(0., 5.), Point::new_scale(12., 5.)};
    const std::vector<ExtendedPoint<2>> disabled =
        estimate_points_properties<true, true, true, true>(wall, previous_layer, 0.4f, -1.f);
    const std::vector<ExtendedPoint<2>> limited =
        estimate_points_properties<true, true, true, true>(wall, previous_layer, 0.4f, 4.0f);

    REQUIRE(disabled.size() == 2);
    REQUIRE(limited.size() == 4);
    CHECK_THAT(disabled.front().position.x(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(disabled.back().position.x(), Catch::Matchers::WithinAbs(12., 1e-9));
    CHECK_THAT(limited.front().position.x(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(limited.back().position.x(), Catch::Matchers::WithinAbs(12., 1e-9));

    double disabled_length = 0.;
    double limited_length = 0.;
    for (size_t idx = 0; idx < disabled.size(); ++idx)
        CHECK_THAT(disabled[idx].position.y(), Catch::Matchers::WithinAbs(5., 1e-9));
    for (size_t idx = 0; idx < limited.size(); ++idx)
        CHECK_THAT(limited[idx].position.y(), Catch::Matchers::WithinAbs(5., 1e-9));
    for (size_t idx = 1; idx < disabled.size(); ++idx)
        disabled_length += (disabled[idx].position - disabled[idx - 1].position).norm();
    for (size_t idx = 1; idx < limited.size(); ++idx) {
        const double length = (limited[idx].position - limited[idx - 1].position).norm();
        limited_length += length;
        CHECK(std::isfinite(length));
        CHECK(length <= 4.0 + 1e-6);
    }
    CHECK_THAT(disabled_length, Catch::Matchers::WithinAbs(12., 1e-9));
    CHECK_THAT(limited_length, Catch::Matchers::WithinAbs(12., 1e-9));
}
