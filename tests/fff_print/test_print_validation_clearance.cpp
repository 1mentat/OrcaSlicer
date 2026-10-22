#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_helpers.hpp"

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct SequentialHeightCase {
    const char *name;
    double first_height;
    double second_height;
    double third_height;
    size_t object_count;
    double later_y;
    double rod_height;
    double lid_height;
    size_t expected_tall_objects;
    double expected_clearance;
};

void make_sequential_height_print(const SequentialHeightCase &part, Print &print, Model &model,
                                  DynamicPrintConfig &config)
{
    config.set_deserialize_strict({
        {"print_sequence", "by object"},
        {"extruder_clearance_radius", 6.0},
        {"extruder_clearance_height_to_rod", part.rod_height},
        {"extruder_clearance_height_to_lid", part.lid_height},
        {"printable_height", 100.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });

    const double heights[] = {part.first_height, part.second_height, part.third_height};
    std::vector<TriangleMesh> meshes;
    meshes.reserve(part.object_count);
    for (size_t i = 0; i < part.object_count; ++i)
        meshes.emplace_back(make_cube(20.0, 20.0, heights[i]));
    init_print(std::move(meshes), print, model, config, nullptr, false);

    for (size_t i = 0; i < model.objects.size(); ++i) {
        model.objects[i]->name = i == 0 ? "tall-first" : i == 1 ? "tall-second" : "short-last";
        model.objects[i]->instances.front()->set_offset(
            Vec3d(60.0 * i, i == 0 ? 0.0 : part.later_y, 0.0));
    }
    print.apply(model, config);
}

void set_rectangle(DynamicPrintConfig &config, const std::string &key,
                   double min_x, double min_y, double max_x, double max_y)
{
    config.set_key_value(key, new ConfigOptionPoints{
        Vec2d(min_x, min_y), Vec2d(max_x, min_y), Vec2d(max_x, max_y), Vec2d(min_x, max_y)
    });
}

void make_clearance_cubes(const DynamicPrintConfig &config, Print &print, Model &model,
                          size_t count, bool second_filament = false)
{
    std::vector<TriangleMesh> meshes;
    meshes.reserve(count);
    for (size_t i = 0; i < count; ++i)
        meshes.emplace_back(make_cube(10.0, 10.0, 10.0));

    std::vector<std::vector<ConfigBase::SetDeserializeItem>> per_object_overrides;
    if (second_filament && count > 1) {
        per_object_overrides.resize(count);
        per_object_overrides[1].emplace_back("extruder", 2);
    }
    init_print(std::move(meshes), print, model, config,
               per_object_overrides.empty() ? nullptr : &per_object_overrides, false);
    for (size_t i = 0; i < model.objects.size(); ++i) {
        model.objects[i]->name = i == 0 ? "clearance-first" : "clearance-second";
        model.objects[i]->instances.front()->set_offset(Vec3d(60.0 * i, 0.0, 0.0));
    }
    print.apply(model, config);
}

size_t count_warning_key(const std::vector<StringObjectException> &warnings, const std::string &key)
{
    size_t count = 0;
    for (const StringObjectException &warning : warnings)
        if (warning.opt_key == key)
            ++count;
    return count;
}

size_t count_warning_text(const std::vector<StringObjectException> &warnings, const std::string &text)
{
    size_t count = 0;
    for (const StringObjectException &warning : warnings)
        if (warning.string.find(text) != std::string::npos)
            ++count;
    return count;
}

} // namespace

TEST_CASE("Print validation applies rod and lid height limits to sequential order",
          "[Print][ValidationContracts][PrintValidationSequentialHeight]")
{
    const auto part = GENERATE(
        SequentialHeightCase{"overlapping Y bands use rod limit", 30.0, 8.0, 0.0, 2, 0.0, 25.0, 120.0, 1, 25.0},
        SequentialHeightCase{"rod equality is safe", 25.0, 8.0, 0.0, 2, 0.0, 25.0, 120.0, 0, 0.0},
        SequentialHeightCase{"separate Y bands use lid limit", 41.0, 8.0, 0.0, 2, 40.0, 25.0, 40.0, 1, 40.0},
        SequentialHeightCase{"lid equality is safe", 40.0, 8.0, 0.0, 2, 40.0, 25.0, 40.0, 0, 0.0},
        SequentialHeightCase{"multiple tall objects are aggregated", 30.0, 30.0, 8.0, 3, 0.0, 25.0, 120.0, 2, 25.0});
    CAPTURE(part.name, part.first_height, part.second_height, part.later_y,
            part.rod_height, part.lid_height);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Print print;
    Model model;
    make_sequential_height_print(part, print, model, config);
    REQUIRE(print.objects().size() == part.object_count);

    Polygons collision_polygons;
    std::vector<std::pair<Polygon, float>> height_polygons;
    const StringObjectException error = print.validate(nullptr, &collision_polygons, &height_polygons);
    CAPTURE(error.string, static_cast<int>(error.type), error.opt_key);
    CHECK_FALSE(error.is_warning);
    CHECK(error.type == (part.expected_tall_objects == 0 ? STRING_EXCEPT_NOT_DEFINED :
                         STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT));
    CHECK(collision_polygons.empty());
    CHECK(height_polygons.size() == part.expected_tall_objects);

    if (part.expected_tall_objects == 0) {
        CHECK(error.string.empty());
        CHECK(error.object == nullptr);
        return;
    }

    REQUIRE_FALSE(error.string.empty());
    CHECK(error.string.find("is too tall, and collisions will be caused.") != std::string::npos);
    if (part.expected_tall_objects == 1) {
        CHECK(error.object == print.objects().front()->model_object());
        CHECK(error.string.find("tall-first") != std::string::npos);
    } else {
        CHECK(error.object == nullptr);
        CHECK(error.string.find("tall-first") != std::string::npos);
        CHECK(error.string.find("tall-second") != std::string::npos);
        CHECK(error.string.find('\n') != std::string::npos);
    }
    for (const auto &entry : height_polygons) {
        CHECK_THAT(entry.second, Catch::Matchers::WithinAbs(part.expected_clearance, 1e-6));
        REQUIRE(entry.first.points.size() >= 3);
        CHECK(std::abs(entry.first.area()) > 0.0);
    }
}

TEST_CASE("Print validation rejects a belt brim combined with spiral vase",
          "[Print][ValidationContracts][PrintValidationBeltBrimSpiral]")
{
    const bool brim_enabled = GENERATE(false, true);
    const bool spiral_mode = GENERATE(false, true);
    CAPTURE(brim_enabled, spiral_mode);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"belt_printer", true},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", 45.0},
        {"brim_type", brim_enabled ? "outer_only" : "no_brim"},
        {"brim_width", 5.0},
        {"spiral_mode", spiral_mode},
        {"print_sequence", "by object"},
        {"wall_loops", 1},
        {"top_shell_layers", 0},
        {"bottom_shell_layers", 0},
        {"sparse_infill_density", "0%"},
        {"enable_support", false},
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"skirt_loops", 0},
        {"enable_prime_tower", false},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });

    Print print;
    Model model;
    init_print({cube(10.0)}, print, model, config);
    const StringObjectException error = print.validate();
    CAPTURE(error.string, error.opt_key);
    if (brim_enabled && spiral_mode) {
        CHECK(error.string.find("Brim is not compatible with spiral vase mode") != std::string::npos);
        CHECK(error.string.find("belt printer") != std::string::npos);
        CHECK(error.opt_key.empty());
        CHECK(error.object == nullptr);
        CHECK_FALSE(error.is_warning);
    } else {
        CHECK(error.string.empty());
        CHECK(error.opt_key.empty());
    }
}

TEST_CASE("By-object validation rejects objects inside the bed keep-out area",
          "[Print][ValidationContracts][PrintValidationPreflight]")
{
    const bool overlaps_keepout = GENERATE(false, true);
    CAPTURE(overlaps_keepout);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"print_sequence", "by object"},
        {"extruder_clearance_radius", 6.0},
        {"extruder_clearance_height_to_rod", 120.0},
        {"extruder_clearance_height_to_lid", 120.0},
        {"printable_height", 100.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });
    const double area_min = overlaps_keepout ? 1.0 : 200.0;
    set_rectangle(config, "bed_exclude_area", area_min, area_min, area_min + 8.0, area_min + 8.0);

    Print print;
    Model model;
    make_clearance_cubes(config, print, model, 2);
    REQUIRE(print.objects().size() == 2);

    const StringObjectException error = print.validate();
    if (!overlaps_keepout) {
        CHECK(error.string.empty());
        CHECK(error.type == STRING_EXCEPT_NOT_DEFINED);
        CHECK(error.object == nullptr);
        CHECK(error.opt_key.empty());
        return;
    }

    CHECK_FALSE(error.is_warning);
    CHECK(error.type == STRING_EXCEPT_OBJECT_COLLISION_IN_SEQ_PRINT);
    CHECK(error.string.find("clearance-first") != std::string::npos);
    CHECK(error.string.find("is too close to exclusion area") != std::string::npos);
    CHECK(error.object == print.model().objects.front()->instances.front());
    CHECK(error.opt_key.empty());
}

TEST_CASE("Layered validation honors bed and clumping keep-out areas",
          "[Print][ValidationContracts][PrintValidationPreflight]")
{
    struct KeepoutCase {
        const char *name;
        bool bed_keepout;
        bool wrapping_enabled;
        bool blocked;
    };
    const KeepoutCase part = GENERATE(
        KeepoutCase{"bed keep-out blocks the object", true, false, true},
        KeepoutCase{"enabled clumping keep-out blocks the object", false, true, true},
        KeepoutCase{"disabled clumping keep-out leaves the object clear", false, false, false});
    CAPTURE(part.name, part.bed_keepout, part.wrapping_enabled);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"print_sequence", "by layer"},
        {"enable_wrapping_detection", part.wrapping_enabled},
        {"layer_change_gcode", "G92 E0\n"},
    });
    const double area_min = part.bed_keepout ? 1.0 : 200.0;
    set_rectangle(config, "bed_exclude_area", area_min, area_min, area_min + 8.0, area_min + 8.0);
    set_rectangle(config, "wrapping_exclude_area", 1.0, 1.0, 9.0, 9.0);

    Print print;
    Model model;
    make_clearance_cubes(config, print, model, 1);
    REQUIRE(print.objects().size() == 1);

    const StringObjectException error = print.validate();
    if (!part.blocked) {
        CHECK(error.string.empty());
        CHECK(error.type == STRING_EXCEPT_NOT_DEFINED);
        CHECK(error.object == nullptr);
        CHECK(error.opt_key.empty());
        return;
    }

    CHECK_FALSE(error.is_warning);
    CHECK(error.type == STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT);
    CHECK(error.object == print.model().objects.front()->instances.front());
    CHECK(error.opt_key.empty());
    if (part.bed_keepout)
        CHECK(error.string.find("is too close to exclusion area") != std::string::npos);
    else
        CHECK(error.string.find("is too close to clumping detection area") != std::string::npos);
}

TEST_CASE("Estimated prime tower validation distinguishes safe, model, bed, and clumping footprints",
          "[Print][ValidationContracts][PrintValidationPreflight]")
{
    enum class TowerCase { Safe, ModelOverlap, BedKeepout, WrappingKeepout };
    const TowerCase part = GENERATE(TowerCase::Safe, TowerCase::ModelOverlap,
                                    TowerCase::BedKeepout, TowerCase::WrappingKeepout);
    CAPTURE(static_cast<int>(part));

    const bool wrapping_enabled = part == TowerCase::WrappingKeepout;
    const double tower_x = part == TowerCase::ModelOverlap ? 0.0 : 40.0;
    const double tower_y = part == TowerCase::ModelOverlap ? 0.0 : 40.0;
    DynamicPrintConfig config = multifilament_config(2, {
        {"print_sequence", "by layer"},
        {"enable_prime_tower", true},
        {"enable_wrapping_detection", wrapping_enabled},
        {"wipe_tower_x", tower_x},
        {"wipe_tower_y", tower_y},
        {"wipe_tower_rotation_angle", 0.0},
        {"prime_tower_brim_width", 0.0},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });

    Print print;
    Model model;
    make_clearance_cubes(config, print, model, 2, true);
    REQUIRE(print.config().enable_prime_tower.value);
    REQUIRE(print.config().filament_colour.values.size() == 2);
    REQUIRE(print.extruders().size() == 2);
    REQUIRE(print.has_wipe_tower());

    const WipeTowerData &estimate = print.wipe_tower_data(print.extruders(true).size());
    REQUIRE(estimate.width > 0.0f);
    REQUIRE(estimate.depth > 0.0f);
    const double margin = estimate.brim_width + 2.0;
    if (part == TowerCase::BedKeepout)
        set_rectangle(config, "bed_exclude_area", tower_x - margin, tower_y - margin,
                      tower_x + estimate.width + margin, tower_y + estimate.depth + margin);
    else if (part == TowerCase::WrappingKeepout)
        set_rectangle(config, "wrapping_exclude_area", tower_x - margin, tower_y - margin,
                      tower_x + estimate.width + margin, tower_y + estimate.depth + margin);
    if (part == TowerCase::BedKeepout || part == TowerCase::WrappingKeepout)
        print.apply(model, config);

    std::vector<StringObjectException> warnings;
    const StringObjectException error = print.validate(&warnings);
    const std::string model_warning = "Prime Tower is too close to others, and collisions may be caused.";
    const size_t model_warning_count = count_warning_text(warnings, model_warning);

    if (part == TowerCase::Safe) {
        CHECK(error.string.empty());
        CHECK(error.type == STRING_EXCEPT_NOT_DEFINED);
        CHECK(error.object == nullptr);
        CHECK(model_warning_count == 0);
    } else if (part == TowerCase::ModelOverlap) {
        CHECK(error.string.empty());
        CHECK(error.type == STRING_EXCEPT_NOT_DEFINED);
        REQUIRE(model_warning_count == 1);
        for (const StringObjectException &warning : warnings) {
            if (warning.string.find(model_warning) == std::string::npos)
                continue;
            CHECK(warning.is_warning);
            CHECK(warning.opt_key.empty());
            CHECK(warning.object == nullptr);
        }
    } else {
        CHECK_FALSE(error.is_warning);
        CHECK(error.type == STRING_EXCEPT_OBJECT_COLLISION_IN_LAYER_PRINT);
        CHECK(error.object == nullptr);
        CHECK(error.opt_key.empty());
        CHECK(model_warning_count == 0);
        if (part == TowerCase::BedKeepout)
            CHECK(error.string.find("Prime Tower is too close to an exclusion area") != std::string::npos);
        else
            CHECK(error.string.find("Prime Tower is too close to clumping detection area") != std::string::npos);
    }
}

TEST_CASE("Belt purge tower validation distinguishes By-object, By-layer, and disabled settings",
          "[Print][ValidationContracts][PrintValidationPreflight]")
{
    const bool by_object = GENERATE(false, true);
    const bool enabled = GENERATE(false, true);
    CAPTURE(by_object, enabled);

    DynamicPrintConfig config = multifilament_config(2, {
        {"belt_printer", true},
        {"belt_slice_rotation", "x"},
        {"belt_slice_rotation_angle", 45.0},
        {"print_sequence", by_object ? "by object" : "by layer"},
        {"enable_belt_purge_tower", enabled},
        {"enable_prime_tower", false},
        {"extruder_clearance_radius", 6.0},
        {"extruder_clearance_height_to_rod", 120.0},
        {"extruder_clearance_height_to_lid", 120.0},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
    });

    Print print;
    Model model;
    make_clearance_cubes(config, print, model, 2, true);
    REQUIRE(print.extruders().size() == 2);

    std::vector<StringObjectException> warnings;
    const StringObjectException error = print.validate(&warnings);
    CHECK(error.string.empty());
    CHECK(error.type == STRING_EXCEPT_NOT_DEFINED);
    CHECK(error.object == nullptr);
    CHECK(error.opt_key.empty());

    const size_t purge_warnings = count_warning_key(warnings, "enable_belt_purge_tower");
    if (!enabled) {
        CHECK(purge_warnings == 0);
        return;
    }

    REQUIRE(purge_warnings == 1);
    for (const StringObjectException &warning : warnings) {
        if (warning.opt_key != "enable_belt_purge_tower")
            continue;
        CHECK(warning.is_warning);
        CHECK(warning.object == nullptr);
        if (by_object)
            CHECK(warning.string.find("not generated in \"By object\" print sequence") != std::string::npos);
        else
            CHECK(warning.string.find("project has no purge tower object") != std::string::npos);
    }
}
