#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Model.hpp"
#include "libslic3r/Polygon.hpp"
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
