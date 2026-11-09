#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/libslic3r.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <vector>

using namespace Slic3r;

namespace layer_profile_edit_contracts {

SlicingParameters edit_parameters()
{
    SlicingParameters params;
    params.valid = true;
    params.layer_height = 0.2;
    params.min_layer_height = 0.1;
    params.max_layer_height = 0.4;
    params.first_print_layer_height = 0.2;
    params.first_object_layer_height = 0.2;
    params.object_print_z_max = 4.;
    params.object_print_z_uncompensated_max = 4.;
    params.object_shrinkage_compensation_z = 1.;
    return params;
}

std::vector<coordf_t> flat_profile(double height)
{
    return {0., 0.2, 0.2, 0.2, 1., height, 2., height, 3., height, 4., height};
}

double height_at_sample(const std::vector<coordf_t> &profile, double z)
{
    for (std::size_t i = 0; i + 1 < profile.size(); i += 2)
        if (std::abs(profile[i] - z) < 1e-8)
            return profile[i + 1];
    FAIL("The requested profile sample is missing");
    return 0.;
}

void check_same_samples(const std::vector<coordf_t> &actual, const std::vector<coordf_t> &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (std::size_t i = 0; i < actual.size(); ++i)
        REQUIRE_THAT(actual[i], Catch::Matchers::WithinAbs(expected[i], 1e-12));
}

void check_edited_profile(const std::vector<coordf_t> &profile)
{
    REQUIRE(profile.size() >= 4);
    REQUIRE(profile.size() % 2 == 0);
    REQUIRE_THAT(profile.front(), Catch::Matchers::WithinAbs(0., 1e-9));
    REQUIRE_THAT(profile[profile.size() - 2], Catch::Matchers::WithinAbs(4., 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 0.), Catch::Matchers::WithinAbs(0.2, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 0.2), Catch::Matchers::WithinAbs(0.2, 1e-9));
    for (std::size_t i = 0; i < profile.size(); i += 2) {
        REQUIRE(std::isfinite(profile[i]));
        REQUIRE(std::isfinite(profile[i + 1]));
        REQUIRE(profile[i + 1] >= 0.1 - 1e-9);
        REQUIRE(profile[i + 1] <= 0.4 + 1e-9);
        if (i > 0)
            REQUIRE(profile[i] > profile[i - 2]);
    }
}

} // namespace layer_profile_edit_contracts

TEST_CASE("Layer profile increase and decrease stop at the configured thickness bounds",
          "[Slicing][LayerProfileEditing]")
{
    using namespace layer_profile_edit_contracts;
    const bool decrease = GENERATE(false, true);
    const auto action = decrease ? LAYER_HEIGHT_EDIT_ACTION_DECREASE : LAYER_HEIGHT_EDIT_ACTION_INCREASE;
    const SlicingParameters params = edit_parameters();
    Model model;
    const ModelObject &object = *model.add_object();
    auto profile = flat_profile(params.layer_height);
    adjust_layer_height_profile(object, params, profile, 2., 0.5, 1., action);
    check_edited_profile(profile);
    REQUIRE_THAT(height_at_sample(profile, 2.),
                 Catch::Matchers::WithinAbs(decrease ? params.min_layer_height : params.max_layer_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 1.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 3.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    const auto bounded = profile;
    adjust_layer_height_profile(object, params, profile, 2., 0.5, 1., action);
    check_same_samples(profile, bounded);
}

TEST_CASE("Layer profile reduction approaches the nominal thickness from both directions",
          "[Slicing][LayerProfileEditing]")
{
    using namespace layer_profile_edit_contracts;
    const double initial_height = GENERATE(0.12, 0.35);
    const SlicingParameters params = edit_parameters();
    Model model;
    const ModelObject &object = *model.add_object();
    auto profile = flat_profile(initial_height);
    adjust_layer_height_profile(object, params, profile, 2., 0.5, 1., LAYER_HEIGHT_EDIT_ACTION_REDUCE);
    check_edited_profile(profile);
    REQUIRE_THAT(height_at_sample(profile, 2.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 1.), Catch::Matchers::WithinAbs(initial_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 3.), Catch::Matchers::WithinAbs(initial_height, 1e-9));
    for (std::size_t i = 0; i < profile.size(); i += 2) {
        if (profile[i] >= 1.5 && profile[i] <= 2.5) {
            REQUIRE(profile[i + 1] >= std::min(initial_height, params.layer_height) - 1e-9);
            REQUIRE(profile[i + 1] <= std::max(initial_height, params.layer_height) + 1e-9);
        }
    }
    const auto reduced = profile;
    adjust_layer_height_profile(object, params, profile, 2., 0.5, 1., LAYER_HEIGHT_EDIT_ACTION_REDUCE);
    check_same_samples(profile, reduced);
}

TEST_CASE("Layer profile smoothing lowers a local peak without changing distant samples",
          "[Slicing][LayerProfileEditing]")
{
    using namespace layer_profile_edit_contracts;
    const SlicingParameters params = edit_parameters();
    Model model;
    const ModelObject &object = *model.add_object();
    auto profile = flat_profile(params.layer_height);
    profile[7] = params.max_layer_height; // Peak at z=2, between nominal samples at z=1 and z=3.
    adjust_layer_height_profile(object, params, profile, 2., 0.1, 1., LAYER_HEIGHT_EDIT_ACTION_SMOOTH);
    check_edited_profile(profile);
    REQUIRE(height_at_sample(profile, 2.) > params.layer_height);
    REQUIRE(height_at_sample(profile, 2.) < params.max_layer_height - 1e-4);
    REQUIRE_THAT(height_at_sample(profile, 1.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 3.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 4.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
    auto flat = flat_profile(params.layer_height);
    const auto unchanged = flat;
    adjust_layer_height_profile(object, params, flat, 2., 0.1, 1., LAYER_HEIGHT_EDIT_ACTION_SMOOTH);
    check_same_samples(flat, unchanged);
}

TEST_CASE("Layer profile edits respect the fixed first layer and retain the object top",
          "[Slicing][LayerProfileEditing]")
{
    using namespace layer_profile_edit_contracts;
    const SlicingParameters params = edit_parameters();
    Model model;
    const ModelObject &object = *model.add_object();
    for (const double z : {0., 0.1, 4.1}) {
        auto profile = flat_profile(params.layer_height);
        const auto unchanged = profile;
        adjust_layer_height_profile(object, params, profile, z, 0.1, 1., LAYER_HEIGHT_EDIT_ACTION_INCREASE);
        check_same_samples(profile, unchanged);
    }
    auto profile = flat_profile(params.layer_height);
    adjust_layer_height_profile(object, params, profile, 4., 0.1, 1., LAYER_HEIGHT_EDIT_ACTION_INCREASE);
    check_edited_profile(profile);
    REQUIRE_THAT(height_at_sample(profile, 4.), Catch::Matchers::WithinAbs(0.3, 1e-9));
    REQUIRE_THAT(height_at_sample(profile, 3.), Catch::Matchers::WithinAbs(params.layer_height, 1e-9));
}
