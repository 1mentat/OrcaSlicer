#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <libslic3r/libslic3r.h>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "test_helpers.hpp"
#include "libslic3r/Slicing.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

SlicingParameters slicing_parameters(coordf_t object_height = 2.)
{
    SlicingParameters params;
    params.valid = true;
    params.layer_height = 0.2;
    params.min_layer_height = 0.1;
    params.max_layer_height = 0.4;
    params.first_print_layer_height = 0.2;
    params.first_object_layer_height = 0.2;
    params.object_print_z_min = 0.;
    params.object_print_z_max = object_height;
    params.object_print_z_uncompensated_max = object_height;
    params.object_shrinkage_compensation_z = 1.;
    return params;
}

ModelConfig layer_height_override(coordf_t height)
{
    DynamicPrintConfig config;
    config.set_key_value("layer_height", new ConfigOptionFloat(height));
    ModelConfig result;
    result.assign_config(std::move(config));
    return result;
}

bool has_profile_point(const std::vector<coordf_t> &profile, coordf_t z, coordf_t height)
{
    for (size_t i = 0; i + 1 < profile.size(); i += 2)
        if (std::abs(profile[i] - z) < EPSILON && std::abs(profile[i + 1] - height) < EPSILON)
            return true;
    return false;
}

void check_profile_shape(const std::vector<coordf_t> &profile, const SlicingParameters &params)
{
    REQUIRE(profile.size() >= 4);
    REQUIRE(profile.size() % 2 == 0);
    CHECK_THAT(profile.front(), Catch::Matchers::WithinAbs(0., EPSILON));
    CHECK_THAT(profile[profile.size() - 2],
               Catch::Matchers::WithinAbs(params.object_print_z_uncompensated_height(), EPSILON));
    for (size_t i = 0; i < profile.size(); i += 2) {
        CHECK(profile[i] <= params.object_print_z_uncompensated_height() + EPSILON);
        if (i > 0)
            CHECK(profile[i] >= profile[i - 2] - EPSILON);
        CHECK(profile[i + 1] >= params.min_layer_height - EPSILON);
        CHECK(profile[i + 1] <= params.max_layer_height + EPSILON);
    }
}

void check_layer_pairs(const std::vector<coordf_t> &layers, const SlicingParameters &params)
{
    REQUIRE(layers.size() >= 2);
    REQUIRE(layers.size() % 2 == 0);
    CHECK_THAT(layers.front(), Catch::Matchers::WithinAbs(0., EPSILON));
    CHECK_THAT(layers.back(),
               Catch::Matchers::WithinAbs(params.object_print_z_height(), EPSILON));
    for (size_t i = 0; i < layers.size(); i += 2) {
        CHECK(layers[i + 1] > layers[i]);
        if (i > 0)
            CHECK_THAT(layers[i], Catch::Matchers::WithinAbs(layers[i - 1], EPSILON));
        const coordf_t height = layers[i + 1] - layers[i];
        CHECK(height >= params.min_layer_height - EPSILON);
        CHECK(height <= params.max_layer_height + EPSILON);
    }
}

} // namespace

TEST_CASE("Layer-height settings derive safe nozzle bounds", "[LayerHeightVariance]")
{
    const DynamicPrintConfig config = multifilament_config(2, {
        { "nozzle_diameter",   "0.4,0.6" },
        { "min_layer_height",  "0.1,0.35" },
        { "max_layer_height",  "0,0.3" },
    });

    CHECK_THAT(Slicing::min_layer_height_from_nozzle(config, 1),
               Catch::Matchers::WithinAbs(0.1, 1e-9));
    CHECK_THAT(Slicing::min_layer_height_from_nozzle(config, 2),
               Catch::Matchers::WithinAbs(0.35, 1e-9));
    CHECK_THAT(Slicing::max_layer_height_from_nozzle(config, 1),
               Catch::Matchers::WithinAbs(0.3, 1e-9)); // 3/4 of a 0.4 mm nozzle.
    CHECK_THAT(Slicing::max_layer_height_from_nozzle(config, 2),
               Catch::Matchers::WithinAbs(0.35, 1e-9)); // Never below the configured minimum.
}

TEST_CASE("Layer-height ranges remain contiguous and generate bounded object layers", "[LayerHeightVariance]")
{
    const SlicingParameters params = slicing_parameters();
    t_layer_config_ranges ranges;
    ranges[{0.4, 1.0}] = layer_height_override(0.3);
    ranges[{0.8, 1.4}] = layer_height_override(0.1);

    const std::vector<coordf_t> profile = layer_height_profile_from_ranges(params, ranges);
    check_profile_shape(profile, params);
    CHECK(has_profile_point(profile, 0., 0.2));
    CHECK(has_profile_point(profile, 0.4, 0.3));
    CHECK(has_profile_point(profile, 1.0, 0.1)); // The overlapping range starts after the first one.
    CHECK(has_profile_point(profile, 1.4, 0.1));
    CHECK(has_profile_point(profile, 2.0, 0.2));

    const std::vector<coordf_t> layers = generate_object_layers(params, profile, true);
    check_layer_pairs(layers, params);
    CHECK_FALSE(check_object_layers_fixed(params, profile));

    const std::vector<coordf_t> fixed_profile{0., 0.2, 2., 0.2};
    CHECK(check_object_layers_fixed(params, fixed_profile));
    check_layer_pairs(generate_object_layers(params, fixed_profile, true), params);
}

TEST_CASE("Adaptive and smoothed profiles preserve configured layer bounds", "[LayerHeightVariance]")
{
    SlicingParameters params = slicing_parameters(4.);
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10., 10., 4.));
    object->add_instance();

    const std::vector<double> adaptive = layer_height_profile_adaptive(params, *object, 0.5f);
    REQUIRE(adaptive.size() >= 4);
    REQUIRE(adaptive.size() % 2 == 0);
    CHECK_THAT(adaptive.front(), Catch::Matchers::WithinAbs(0., EPSILON));
    CHECK_THAT(adaptive[adaptive.size() - 2], Catch::Matchers::WithinAbs(4., EPSILON));
    for (size_t i = 0; i < adaptive.size(); i += 2) {
        CHECK(adaptive[i] >= 0. - EPSILON);
        if (i > 0)
            CHECK(adaptive[i] >= adaptive[i - 2] - EPSILON);
        CHECK(adaptive[i + 1] >= params.min_layer_height - EPSILON);
        CHECK(adaptive[i + 1] <= params.max_layer_height + EPSILON);
    }

    const std::vector<double> profile{
        0., 0.2, 0.2, 0.2, 0.4, 0.4, 0.6, 0.1, 0.8, 0.4, 1.0, 0.1,
        1.2, 0.4, 1.4, 0.1, 1.6, 0.4, 1.8, 0.1, 2.0, 0.2,
    };
    const std::vector<double> smoothed = smooth_height_profile(
        profile, slicing_parameters(), HeightProfileSmoothingParams(1, true));
    REQUIRE(smoothed.size() == profile.size());
    for (size_t i = 0; i < smoothed.size(); i += 2) {
        CHECK_THAT(smoothed[i], Catch::Matchers::WithinAbs(profile[i], 1e-9));
        CHECK(smoothed[i + 1] >= 0.1 - EPSILON);
        CHECK(smoothed[i + 1] <= 0.4 + EPSILON);
        if (i >= 4)
            CHECK(smoothed[i + 1] <= profile[i + 1] + EPSILON);
    }
    bool interior_height_changed = false;
    for (size_t i = 5; i < smoothed.size(); i += 2)
        interior_height_changed |= std::abs(smoothed[i] - profile[i]) > 1e-6;
    CHECK(interior_height_changed);
}

TEST_CASE("Layer-height editing changes free bands but respects ranges", "[LayerHeightVariance]")
{
    const SlicingParameters params = slicing_parameters();
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10., 10., 2.));
    object->add_instance();

    const std::vector<coordf_t> initial{0., 0.2, 2., 0.2};
    std::vector<coordf_t> increased = initial;
    adjust_layer_height_profile(*object, params, increased, 1.0, 0.1, 0.6,
                                LAYER_HEIGHT_EDIT_ACTION_INCREASE);
    check_profile_shape(increased, params);
    CHECK(increased.size() > initial.size());
    bool increased_height_found = false;
    for (size_t i = 1; i < increased.size(); i += 2)
        increased_height_found |= increased[i] > 0.2 + EPSILON;
    CHECK(increased_height_found);

    object->layer_config_ranges[{0.8, 1.2}] = layer_height_override(0.3);
    std::vector<coordf_t> protected_profile = initial;
    adjust_layer_height_profile(*object, params, protected_profile, 1.0, 0.1, 0.6,
                                LAYER_HEIGHT_EDIT_ACTION_INCREASE);
    REQUIRE(protected_profile.size() == initial.size());
    for (size_t i = 0; i < initial.size(); ++i)
        CHECK_THAT(protected_profile[i], Catch::Matchers::WithinAbs(initial[i], EPSILON));
}
