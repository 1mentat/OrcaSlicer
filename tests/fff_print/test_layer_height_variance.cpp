#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <libslic3r/libslic3r.h>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
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

TEST_CASE("Zero adaptive minimum reaches real sliced layers through PrintObject parameters",
          "[LayerHeightVariance]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", "0.4"},
        {"min_layer_height", "0"},
        {"max_layer_height", "0.3"},
        {"precise_z_height", false},
        {"use_relative_e_distances", 1},
        {"layer_change_gcode", "G92 E0\n"},
    });

    Model model;
    ModelObject *object = model.add_object();
    TriangleMesh sloped_pyramid = mesh(TestMesh::pyramid);
    // Shallow facets request less than the nozzle minimum under highest adaptive quality.
    sloped_pyramid.scale(Vec3f(1.f, 1.f, 0.05f));
    object->add_volume(std::move(sloped_pyramid));
    object->add_instance();

    const SlicingParameters params = PrintObject::slicing_parameters(
        config, *object, 0.f, Vec3d(1., 1., 1.));
    REQUIRE(params.valid);
    CHECK_THAT(params.min_layer_height, Catch::Matchers::WithinAbs(0.07, 1e-9));

    const std::vector<double> profile = layer_height_profile_adaptive(params, *object, 0.f);
    check_profile_shape(profile, params);
    bool profile_uses_minimum = false;
    for (size_t i = 1; i < profile.size(); i += 2)
        profile_uses_minimum |= std::abs(profile[i] - params.min_layer_height) < EPSILON;
    REQUIRE(profile_uses_minimum);

    // The normal Print slice path consumes the same profile and parameter object.
    object->layer_height_profile.set(profile);
    object->ensure_on_bed();
    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, config);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE_FALSE(print.objects().empty());
    const auto layers = print.objects().front()->layers();
    REQUIRE(layers.size() > 2);
    bool emitted_minimum_layer = false;
    for (const Layer *layer : layers) {
        REQUIRE(std::isfinite(layer->height));
        CHECK(layer->height >= params.min_layer_height - EPSILON);
        CHECK(layer->height <= params.max_layer_height + EPSILON);
        emitted_minimum_layer |= std::abs(layer->height - 0.07) < EPSILON;
    }
    CHECK(emitted_minimum_layer);
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


namespace layer_geometry_settings_contracts {
namespace {

struct LayerGeometryEvidence {
    double first_extrusion_z = std::numeric_limits<double>::quiet_NaN();
    double max_extrusion_z = -std::numeric_limits<double>::infinity();
    double top_length = 0.0;
    double bottom_length = 0.0;
    double solid_length = 0.0;
    double sparse_length = 0.0;
    std::vector<double> extrusion_layers;
    std::size_t motion_moves = 0;
    std::size_t extrusion_moves = 0;
    bool finite = true;
    std::size_t layer_changes = 0;
    std::vector<double> emitted_heights;
};

DynamicPrintConfig layer_geometry_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 2},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"top_surface_density", "100%"},
        {"bottom_surface_density", "100%"},
        {"sparse_infill_density", "15%"},
        {"minimum_sparse_infill_area", 0.0},
        {"ensure_vertical_shell_thickness", "ensure_all"},
        {"top_surface_pattern", "rectilinear"},
        {"bottom_surface_pattern", "rectilinear"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"enable_support", false},
        {"gcode_comments", true},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });
    return config;
}

TriangleMesh layer_geometry_fixture()
{
    TriangleMesh shape = cube(10.0);
    shape.scale(Vec3f(1.0f, 1.0f, 0.12f));
    return shape;
}

void add_distinct_layer(std::vector<double> &layers, double z)
{
    for (const double old_z : layers)
        if (std::abs(old_z - z) < 1e-5)
            return;
    layers.push_back(z);
}

LayerGeometryEvidence export_evidence(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({layer_geometry_fixture()}, print, model, config);
    const auto validation = print.validate();
    INFO("validation.string: " << validation.string);
    INFO("validation.opt_key: " << validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    REQUIRE(output.size() <= 4u * 1024u * 1024u);

    LayerGeometryEvidence evidence;
    std::istringstream raw_lines(output);
    std::string raw;
    bool awaiting_layer_height = false;
    while (std::getline(raw_lines, raw)) {
        if (raw == ";LAYER_CHANGE") {
            ++evidence.layer_changes;
            awaiting_layer_height = true;
            continue;
        }
        constexpr std::string_view height_prefix = ";HEIGHT:";
        if (!awaiting_layer_height || raw.rfind(height_prefix.data(), 0) != 0)
            continue;
        // The first HEIGHT after a layer marker describes the layer. Later
        // HEIGHT tags may describe a bridge or another extrusion role.
        awaiting_layer_height = false;
        try {
            std::size_t parsed = 0;
            const double height = std::stod(raw.substr(height_prefix.size()), &parsed);
            if (!std::isfinite(height) || parsed != raw.size() - height_prefix.size())
                evidence.finite = false;
            else
                evidence.emitted_heights.push_back(height);
        } catch (...) {
            evidence.finite = false;
        }
    }

    enum class Role { Other, Top, Bottom, Solid, Sparse };
    Role active_role = Role::Other;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&evidence, &active_role](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("Top surface") != std::string_view::npos ||
            comment.find("Top solid infill") != std::string_view::npos)
            active_role = Role::Top;
        else if (comment.find("Bottom surface") != std::string_view::npos)
            active_role = Role::Bottom;
        else if (comment.find("Internal solid infill") != std::string_view::npos)
            active_role = Role::Solid;
        else if (comment.find("Sparse infill") != std::string_view::npos)
            active_role = Role::Sparse;
        else if (comment.find("TYPE:") != std::string_view::npos ||
                 comment.find("FEATURE:") != std::string_view::npos)
            active_role = Role::Other;

        const bool motion = line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3");
        if (!motion)
            return;
        ++evidence.motion_moves;
        const double z = line.new_Z(self);
        if (!std::isfinite(z)) {
            evidence.finite = false;
            return;
        }
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        ++evidence.extrusion_moves;
        if (!std::isfinite(line.dist_XY(self))) {
            evidence.finite = false;
            return;
        }
        if (!std::isfinite(evidence.first_extrusion_z))
            evidence.first_extrusion_z = z;
        evidence.max_extrusion_z = std::max(evidence.max_extrusion_z, z);
        add_distinct_layer(evidence.extrusion_layers, z);
        const double length = line.dist_XY(self);
        if (active_role == Role::Top)
            evidence.top_length += length;
        else if (active_role == Role::Bottom)
            evidence.bottom_length += length;
        else if (active_role == Role::Solid)
            evidence.solid_length += length;
        else if (active_role == Role::Sparse)
            evidence.sparse_length += length;
    });
    return evidence;
}

void require_valid_evidence(const LayerGeometryEvidence &evidence)
{
    REQUIRE(evidence.finite);
    REQUIRE(std::isfinite(evidence.first_extrusion_z));
    REQUIRE(std::isfinite(evidence.max_extrusion_z));
    REQUIRE(evidence.motion_moves > 0);
    REQUIRE(evidence.extrusion_moves > 0);
    REQUIRE_FALSE(evidence.extrusion_layers.empty());
}

} // namespace

TEST_CASE("Initial layer print height reaches the first extrusion and preserves object height",
          "[LayerHeightVariance][LayerGeometrySettings][SettingsVariance]")
{
    DynamicPrintConfig thin_config = layer_geometry_config();
    DynamicPrintConfig thick_config = thin_config;
    thick_config.set_deserialize_strict("initial_layer_print_height", "0.4");

    const LayerGeometryEvidence thin = export_evidence(thin_config);
    const LayerGeometryEvidence thick = export_evidence(thick_config);
    require_valid_evidence(thin);
    require_valid_evidence(thick);

    CHECK_THAT(thin.first_extrusion_z, Catch::Matchers::WithinAbs(0.2, 1e-4));
    CHECK_THAT(thick.first_extrusion_z, Catch::Matchers::WithinAbs(0.4, 1e-4));
    CHECK_THAT(thin.max_extrusion_z, Catch::Matchers::WithinAbs(1.2, 1e-3));
    CHECK_THAT(thick.max_extrusion_z, Catch::Matchers::WithinAbs(1.2, 1e-3));
    CHECK(thick.extrusion_layers.size() < thin.extrusion_layers.size());
}

TEST_CASE("Shell thickness propagates solid material beyond the configured shell count",
          "[LayerHeightVariance][LayerGeometrySettings][SettingsVariance]")
{
    const std::string key = GENERATE("top_shell_thickness", "bottom_shell_thickness");
    DYNAMIC_SECTION("shell thickness " << key) {
        DynamicPrintConfig thin_config = layer_geometry_config();
        DynamicPrintConfig thick_config = thin_config;
        thick_config.set_deserialize_strict(key, "0.6");

        const LayerGeometryEvidence thin = export_evidence(thin_config);
        const LayerGeometryEvidence thick = export_evidence(thick_config);
        require_valid_evidence(thin);
        require_valid_evidence(thick);

        REQUIRE(thin.top_length > 0.0);
        REQUIRE(thin.bottom_length > 0.0);
        REQUIRE(thin.sparse_length > 0.0);
        REQUIRE(thick.top_length > 0.0);
        REQUIRE(thick.bottom_length > 0.0);
        CHECK(thick.solid_length > thin.solid_length);
        CHECK(thick.extrusion_layers.size() == thin.extrusion_layers.size());
        CHECK_THAT(thick.first_extrusion_z, Catch::Matchers::WithinAbs(thin.first_extrusion_z, 1e-4));
        CHECK_THAT(thick.max_extrusion_z, Catch::Matchers::WithinAbs(thin.max_extrusion_z, 1e-3));
    }
}

TEST_CASE("Top and bottom surface densities change their generated surface material",
          "[LayerHeightVariance][LayerGeometrySettings][SettingsVariance]")
{
    const DynamicPrintConfig dense_config = layer_geometry_config();
    const LayerGeometryEvidence full = export_evidence(dense_config);
    require_valid_evidence(full);
    REQUIRE(full.top_length > 0.0);
    REQUIRE(full.bottom_length > 0.0);

    const std::vector<std::pair<std::string, std::string>> cases{
        {"top_surface_density", "0%"},
        {"top_surface_density", "40%"},
        {"bottom_surface_density", "10%"},
        {"bottom_surface_density", "40%"}};
    for (const auto &[key, sparse_value] : cases) {
        INFO("density setting: " << key);
        INFO("density value: " << sparse_value);
        DynamicPrintConfig sparse_config = layer_geometry_config();
        sparse_config.set_deserialize_strict(key, sparse_value);
        const LayerGeometryEvidence sparse = export_evidence(sparse_config);
        require_valid_evidence(sparse);

        if (key == "top_surface_density") {
            if (sparse_value == "0%")
                CHECK_THAT(sparse.top_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            else
                REQUIRE(sparse.top_length > 0.0);
            CHECK(full.top_length > sparse.top_length);
            CHECK_THAT(sparse.bottom_length, Catch::Matchers::WithinRel(full.bottom_length, 1e-6));
        } else {
            REQUIRE(sparse.top_length > 0.0);
            REQUIRE(sparse.bottom_length > 0.0);
            CHECK(full.bottom_length > sparse.bottom_length);
            CHECK_THAT(sparse.top_length, Catch::Matchers::WithinRel(full.top_length, 1e-6));
        }
    }
}

double layer_height_output_tolerance(double expected)
{
    // XYZF_EXPORT_DIGITS is three; layer comments are derived from the same float Z state.
    constexpr double gcode_quantization_mm = 0.001;
    return 0.5 * gcode_quantization_mm +
           8.0 * std::numeric_limits<float>::epsilon() * std::max(1.0, std::abs(expected));
}

TEST_CASE("Configured layer heights reach exact full-print layers and HEIGHT tags",
          "[LayerHeightPrintPipeline][LayerHeightVariance][LayerGeometrySettings][SettingsVariance]")
{
    constexpr double fixture_height = 1.2;
    constexpr double nozzle_diameter = 0.4;
    constexpr double minimum_layer_height = 0.1;
    constexpr double nominal_layer_height = 0.2;
    constexpr double maximum_layer_height = 0.3;

    for (const double layer_height : {minimum_layer_height, nominal_layer_height, maximum_layer_height}) {
        INFO("layer height: " << layer_height);
        DynamicPrintConfig config = layer_geometry_config();
        config.set_deserialize_strict({
            {"layer_height", layer_height},
            {"initial_layer_print_height", layer_height},
            {"nozzle_diameter", nozzle_diameter},
            {"min_layer_height", "0.1"},
            {"max_layer_height", "0.3"},
        });

        const LayerGeometryEvidence evidence = export_evidence(config);
        require_valid_evidence(evidence);
        const std::size_t expected_layers = static_cast<std::size_t>(std::llround(fixture_height / layer_height));
        const double height_tolerance = layer_height_output_tolerance(layer_height);
        const double top_z_tolerance = layer_height_output_tolerance(fixture_height);

        REQUIRE_THAT(expected_layers * layer_height,
                     Catch::Matchers::WithinAbs(fixture_height,
                         4.0 * std::numeric_limits<double>::epsilon() * fixture_height));
        REQUIRE(evidence.layer_changes == expected_layers);
        REQUIRE(evidence.emitted_heights.size() == expected_layers);
        REQUIRE(evidence.extrusion_layers.size() == expected_layers);
        REQUIRE(evidence.top_length > 0.0);
        REQUIRE(evidence.bottom_length > 0.0);
        REQUIRE(evidence.sparse_length > 0.0);
        CHECK_THAT(evidence.first_extrusion_z, Catch::Matchers::WithinAbs(layer_height, height_tolerance));
        CHECK_THAT(evidence.max_extrusion_z, Catch::Matchers::WithinAbs(fixture_height, top_z_tolerance));
        for (std::size_t layer = 0; layer < expected_layers; ++layer) {
            const double expected_z = (layer + 1) * layer_height;
            CHECK_THAT(evidence.extrusion_layers[layer],
                       Catch::Matchers::WithinAbs(expected_z, layer_height_output_tolerance(expected_z)));
        }
        for (const double emitted_height : evidence.emitted_heights)
            CHECK_THAT(emitted_height, Catch::Matchers::WithinAbs(layer_height, height_tolerance));
    }
}

} // namespace layer_geometry_settings_contracts


TEST_CASE("Adaptive Print layers reach the configured or nozzle-derived maximum",
          "[LayerHeightVariance]")
{
    struct MaximumCase {
        const char *nozzle_diameter;
        const char *max_layer_height;
        double expected_maximum;
    };
    const MaximumCase cases[] = {
        { "0.4", "0",    0.30 }, // Zero uses three quarters of a 0.4 mm nozzle.
        { "0.6", "0",    0.45 }, // The same sentinel follows the selected nozzle diameter.
        { "0.4", "0.25", 0.25 }, // An explicit maximum overrides the nozzle-derived default.
    };

    for (const MaximumCase &test : cases) {
        INFO("nozzle=" << test.nozzle_diameter << " max_layer_height=" << test.max_layer_height);
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            { "layer_height",              0.2 },
            { "initial_layer_print_height", 0.2 },
            { "nozzle_diameter",            test.nozzle_diameter },
            { "min_layer_height",           "0.1" },
            { "max_layer_height",           test.max_layer_height },
            { "precise_z_height",           false },
            { "use_relative_e_distances",   true },
            { "layer_change_gcode",         "G92 E0\n" },
        });

        Model model;
        ModelObject *object = model.add_object();
        object->add_volume(make_cube(10., 10., 4.));
        object->add_instance();

        const SlicingParameters params = PrintObject::slicing_parameters(
            config, *object, 0.f, Vec3d(1., 1., 1.));
        REQUIRE(params.valid);
        CHECK_THAT(params.max_layer_height,
                   Catch::Matchers::WithinAbs(test.expected_maximum, 1e-9));

        // A flat-sided cube has vertical facets that allow the coarse quality path to
        // reach the configured cap; the profile then feeds the normal Print slice path.
        const std::vector<double> profile = layer_height_profile_adaptive(params, *object, 1.f);
        check_profile_shape(profile, params);
        bool profile_reaches_maximum = false;
        for (size_t i = 1; i < profile.size(); i += 2)
            profile_reaches_maximum |= std::abs(profile[i] - test.expected_maximum) < EPSILON;
        CHECK(profile_reaches_maximum);

        object->layer_height_profile.set(profile);
        object->ensure_on_bed();
        Print print;
        print.auto_assign_extruders(object);
        print.apply(model, config);
        const auto validation = print.validate();
        INFO(validation.string);
        REQUIRE(validation.string.empty());
        print.process();
        REQUIRE_FALSE(print.objects().empty());
        const auto layers = print.objects().front()->layers();
        REQUIRE(layers.size() > 2);
        double actual_maximum = 0.;
        for (const Layer *layer : layers) {
            REQUIRE(std::isfinite(layer->height));
            CHECK(layer->height >= params.min_layer_height - EPSILON);
            CHECK(layer->height <= params.max_layer_height + EPSILON);
            actual_maximum = std::max(actual_maximum, layer->height);
        }
        CHECK_THAT(actual_maximum,
                   Catch::Matchers::WithinAbs(test.expected_maximum, EPSILON));
    }
}
