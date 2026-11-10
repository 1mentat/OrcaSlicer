#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include "test_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

constexpr double kFirstLayerZ = 0.2;
constexpr double kFootprintX = 12.0;
constexpr double kFootprintY = 8.0;

struct FirstLayerExtents {
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    size_t xy_extrusions = 0;
};

struct BedMeshOutput {
    std::string gcode;
    FirstLayerExtents first_layer;
    Vec2d min = Vec2d::Zero();
    Vec2d max = Vec2d::Zero();
    Vec2d probe_count = Vec2d::Zero();
    std::string algorithm;
};

struct MeshSettings {
    double margin;
    Vec2d allowed_min;
    Vec2d allowed_max;
    Vec2d probe_distance;
    std::string flavor;
};

std::string machine_start_placeholders()
{
    return "M117 ABM_MIN={adaptive_bed_mesh_min[0]},{adaptive_bed_mesh_min[1]}\n"
           "M117 ABM_MAX={adaptive_bed_mesh_max[0]},{adaptive_bed_mesh_max[1]}\n"
           "M117 ABM_POINTS={bed_mesh_probe_count[0]},{bed_mesh_probe_count[1]}\n"
           "M117 ABM_ALGO={bed_mesh_algo}\n";
}

DynamicPrintConfig bed_mesh_config(const MeshSettings &settings)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "wall_loops", 1 },
        { "top_shell_layers", 0 },
        { "bottom_shell_layers", 1 },
        { "top_shell_thickness", 0.0 },
        { "bottom_shell_thickness", 0.0 },
        { "sparse_infill_density", "0%" },
        { "line_width", 0.5 },
        { "initial_layer_line_width", 0.5 },
        { "outer_wall_line_width", 0.5 },
        { "skirt_loops", 0 },
        // Print::has_skirt() gates the first-layer skirt hull on skirt_height.
        { "skirt_height", 0 },
        { "brim_type", "no_brim" },
        { "raft_layers", 0 },
        { "gcode_flavor", settings.flavor },
        { "machine_start_gcode", machine_start_placeholders() },
        { "machine_end_gcode", "" },
        { "file_start_gcode", "" },
        { "layer_change_gcode", "G92 E0\n" },
        { "use_relative_e_distances", true },
        { "elefant_foot_compensation", 0.0 },
        { "xy_contour_compensation", 0.0 },
        { "xy_hole_compensation", 0.0 },
        { "enable_arc_fitting", false },
        { "enable_support", false },
        { "z_hop", 0.0 },
    });
    config.set_key_value("adaptive_bed_mesh_margin", new ConfigOptionFloat(settings.margin));
    config.set_key_value("bed_mesh_min", new ConfigOptionPoint(settings.allowed_min));
    config.set_key_value("bed_mesh_max", new ConfigOptionPoint(settings.allowed_max));
    config.set_key_value("bed_mesh_probe_distance", new ConfigOptionPoint(settings.probe_distance));
    return config;
}

void include_xy(FirstLayerExtents &extents, double x, double y)
{
    extents.min_x = std::min(extents.min_x, x);
    extents.min_y = std::min(extents.min_y, y);
    extents.max_x = std::max(extents.max_x, x);
    extents.max_y = std::max(extents.max_y, y);
}

FirstLayerExtents emitted_first_layer_extents(const std::string &gcode, const DynamicPrintConfig &config)
{
    FirstLayerExtents extents;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&extents](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;
        if (std::abs(line.new_Z(self) - kFirstLayerZ) > 1e-3)
            return;
        include_xy(extents, self.x(), self.y());
        include_xy(extents, line.new_X(self), line.new_Y(self));
        ++extents.xy_extrusions;
    });
    return extents;
}

std::string tagged_value(const std::string &gcode, const std::string &name)
{
    const std::string prefix = "M117 " + name + "=";
    const size_t start = gcode.find(prefix);
    REQUIRE(start != std::string::npos);
    const size_t value_start = start + prefix.size();
    const size_t line_end = gcode.find_first_of("\r\n", value_start);
    REQUIRE(line_end != std::string::npos);
    return gcode.substr(value_start, line_end - value_start);
}

Vec2d tagged_pair(const std::string &gcode, const std::string &name)
{
    std::istringstream input(tagged_value(gcode, name));
    double x = 0.0;
    double y = 0.0;
    char comma = 0;
    REQUIRE(static_cast<bool>(input >> x >> comma >> y));
    CHECK(comma == ',');
    return Vec2d(x, y);
}

int expected_probe_count(double span, double distance, bool klipper)
{
    const double effective_distance = std::max(1.0, distance);
    int count = std::max(3, static_cast<int>(std::ceil(span / effective_distance)) + 1);
    if (klipper)
        count = std::max(count, 4);
    return count;
}

BedMeshOutput export_box(const MeshSettings &settings)
{
    DynamicPrintConfig config = bed_mesh_config(settings);
    Model model;
    Print print;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(make_cube(kFootprintX, kFootprintY, kFirstLayerZ));
    Test::init_print(std::move(meshes), print, model, config, nullptr, false, 1);
    const auto validation = print.validate();
    INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());

    BedMeshOutput output;
    output.gcode = Test::gcode(print);
    REQUIRE_FALSE(output.gcode.empty());
    output.first_layer = emitted_first_layer_extents(output.gcode, config);
    REQUIRE(output.first_layer.xy_extrusions > 0);
    REQUIRE(std::isfinite(output.first_layer.min_x));
    REQUIRE(std::isfinite(output.first_layer.min_y));
    REQUIRE(std::isfinite(output.first_layer.max_x));
    REQUIRE(std::isfinite(output.first_layer.max_y));

    output.min = tagged_pair(output.gcode, "ABM_MIN");
    output.max = tagged_pair(output.gcode, "ABM_MAX");
    output.probe_count = tagged_pair(output.gcode, "ABM_POINTS");
    output.algorithm = tagged_value(output.gcode, "ABM_ALGO");
    return output;
}

void require_box_footprint(const FirstLayerExtents &extents)
{
    REQUIRE(extents.xy_extrusions > 0);
    // The unslotted 12 x 8 mm input cube is placed without XY rearrangement.
    // Its emitted perimeter is inset from that analytic mesh envelope.
    CHECK(extents.min_x >= -1e-3);
    CHECK(extents.min_y >= -1e-3);
    CHECK(extents.max_x <= kFootprintX + 1e-3);
    CHECK(extents.max_y <= kFootprintY + 1e-3);
    CHECK(extents.max_x - extents.min_x > 10.0);
    CHECK(extents.max_y - extents.min_y > 6.0);
}

void require_mesh_placeholders(const BedMeshOutput &output, const MeshSettings &settings)
{
    // GCode.cpp derives the public adaptive-bed-mesh bounds from the first-layer island hull,
    // not the emitted extrusion centerline. The known, unarranged cube defines that hull envelope.
    const Vec2d island_min(0.0, 0.0);
    const Vec2d island_max(kFootprintX, kFootprintY);
    const Vec2d expected_min(
        std::max(settings.allowed_min.x(), island_min.x() - settings.margin),
        std::max(settings.allowed_min.y(), island_min.y() - settings.margin));
    const Vec2d expected_max(
        std::min(settings.allowed_max.x(), island_max.x() + settings.margin),
        std::min(settings.allowed_max.y(), island_max.y() + settings.margin));

    CHECK_THAT(output.min.x(), WithinAbs(expected_min.x(), 0.02));
    CHECK_THAT(output.min.y(), WithinAbs(expected_min.y(), 0.02));
    CHECK_THAT(output.max.x(), WithinAbs(expected_max.x(), 0.02));
    CHECK_THAT(output.max.y(), WithinAbs(expected_max.y(), 0.02));

    const bool klipper = settings.flavor == "klipper";
    const int count_x = expected_probe_count(expected_max.x() - expected_min.x(), settings.probe_distance.x(), klipper);
    const int count_y = expected_probe_count(expected_max.y() - expected_min.y(), settings.probe_distance.y(), klipper);
    CHECK_THAT(output.probe_count.x(), WithinAbs(count_x, 0.01));
    CHECK_THAT(output.probe_count.y(), WithinAbs(count_y, 0.01));
    // Each axis is clamped to at least three points, so the product is at least nine;
    // the public result therefore remains bicubic on these valid configurations.
    CHECK(output.algorithm == "bicubic");
}

void require_same_first_layer(const FirstLayerExtents &a, const FirstLayerExtents &b)
{
    CHECK(a.xy_extrusions > 0);
    CHECK(b.xy_extrusions > 0);
    CHECK_THAT(a.min_x, WithinAbs(b.min_x, 0.02));
    CHECK_THAT(a.min_y, WithinAbs(b.min_y, 0.02));
    CHECK_THAT(a.max_x, WithinAbs(b.max_x, 0.02));
    CHECK_THAT(a.max_y, WithinAbs(b.max_y, 0.02));
}

} // namespace

TEST_CASE("Adaptive bed mesh margins follow the analytic first-layer island envelope", "[GCode][AdaptiveBedMesh]")
{
    const Vec2d wide_min(-50.0, -50.0);
    const Vec2d wide_max(50.0, 50.0);
    const Vec2d spacing(5.0, 4.0);
    const MeshSettings unexpanded{0.0, wide_min, wide_max, spacing, "marlin2"};
    const MeshSettings expanded{2.0, wide_min, wide_max, spacing, "marlin2"};

    const BedMeshOutput base = export_box(unexpanded);
    const BedMeshOutput margin = export_box(expanded);
    require_box_footprint(base.first_layer);
    require_box_footprint(margin.first_layer);
    require_same_first_layer(base.first_layer, margin.first_layer);
    require_mesh_placeholders(base, unexpanded);
    require_mesh_placeholders(margin, expanded);

    CHECK_THAT(base.min.x(), WithinAbs(0.0, 0.02));
    CHECK_THAT(base.min.y(), WithinAbs(0.0, 0.02));
    CHECK_THAT(base.max.x(), WithinAbs(kFootprintX, 0.02));
    CHECK_THAT(base.max.y(), WithinAbs(kFootprintY, 0.02));
    CHECK_THAT(margin.min.x(), WithinAbs(-2.0, 0.02));
    CHECK_THAT(margin.min.y(), WithinAbs(-2.0, 0.02));
    CHECK_THAT(margin.max.x(), WithinAbs(kFootprintX + 2.0, 0.02));
    CHECK_THAT(margin.max.y(), WithinAbs(kFootprintY + 2.0, 0.02));
}

TEST_CASE("Adaptive bed mesh limits clip the analytic island envelope and zero probe spacing uses the minimum", "[GCode][AdaptiveBedMesh]")
{
    const MeshSettings clipped{100.0, Vec2d(1.0, 2.0), Vec2d(10.0, 6.0), Vec2d(0.0, 0.0), "marlin2"};
    const BedMeshOutput output = export_box(clipped);
    require_box_footprint(output.first_layer);
    REQUIRE(output.first_layer.min_x < clipped.allowed_min.x());
    REQUIRE(output.first_layer.min_y < clipped.allowed_min.y());
    REQUIRE(output.first_layer.max_x > clipped.allowed_max.x());
    REQUIRE(output.first_layer.max_y > clipped.allowed_max.y());
    require_mesh_placeholders(output, clipped);

    CHECK_THAT(output.min.x(), WithinAbs(1.0, 0.02));
    CHECK_THAT(output.min.y(), WithinAbs(2.0, 0.02));
    CHECK_THAT(output.max.x(), WithinAbs(10.0, 0.02));
    CHECK_THAT(output.max.y(), WithinAbs(6.0, 0.02));
    CHECK_THAT(output.probe_count.x(), WithinAbs(10.0, 0.01));
    CHECK_THAT(output.probe_count.y(), WithinAbs(5.0, 0.01));
}

TEST_CASE("Klipper adaptive bed mesh retains the four-point axis minimum", "[GCode][AdaptiveBedMesh]")
{
    const Vec2d wide_min(-50.0, -50.0);
    const Vec2d wide_max(50.0, 50.0);
    const Vec2d broad_spacing(100.0, 100.0);
    const MeshSettings marlin{0.0, wide_min, wide_max, broad_spacing, "marlin2"};
    const MeshSettings klipper{0.0, wide_min, wide_max, broad_spacing, "klipper"};

    const BedMeshOutput marlin_output = export_box(marlin);
    const BedMeshOutput klipper_output = export_box(klipper);
    require_box_footprint(marlin_output.first_layer);
    require_box_footprint(klipper_output.first_layer);
    require_same_first_layer(marlin_output.first_layer, klipper_output.first_layer);
    require_mesh_placeholders(marlin_output, marlin);
    require_mesh_placeholders(klipper_output, klipper);

    CHECK_THAT(marlin_output.probe_count.x(), WithinAbs(3.0, 0.01));
    CHECK_THAT(marlin_output.probe_count.y(), WithinAbs(3.0, 0.01));
    CHECK_THAT(klipper_output.probe_count.x(), WithinAbs(4.0, 0.01));
    CHECK_THAT(klipper_output.probe_count.y(), WithinAbs(4.0, 0.01));
    CHECK(marlin_output.algorithm == "bicubic");
    CHECK(klipper_output.algorithm == "bicubic");
}
