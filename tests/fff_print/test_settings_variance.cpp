#include <array>
#include <tuple>
#include <catch2/generators/catch_generators_range.hpp>
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include <functional>
#include <limits>
#include <map>
#include <numeric>
#include "libslic3r/Geometry.hpp"
#include <iterator>
#include <algorithm>
#include <set>
#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/catch_message.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Surface.hpp>
#include <libslic3r/libslic3r.h>
#include <libslic3r/Config.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// Keep these fixtures small enough for a full enum sweep while making every
// role under test appear on at least one layer.
DynamicPrintConfig variance_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height",  0.2 },
        { "wall_loops",                 2 },
        { "top_shell_layers",           2 },
        { "bottom_shell_layers",        2 },
        { "top_shell_thickness",        0.0 },
        { "bottom_shell_thickness",     0.0 },
        { "sparse_infill_density",      "15%" },
        { "top_surface_density",        "100%" },
        { "bottom_surface_density",     "100%" },
        { "gcode_comments",             true },
        // Keep the emitted file self-contained when the test uses absolute E.
        { "layer_change_gcode",         "G92 E0\n" },
    });
    return config;
}

double collection_length(const ExtrusionEntityCollection &collection)
{
    double length = 0.0;
    for_each_extrusion_path(collection, [&length](const ExtrusionPath &path) {
        length += path.length();
    });
    return length;
}

bool finite_bounded_gcode(const std::string &output, const GCodeConfig &config,
                          size_t &parsed_motion_moves, size_t &parsed_extrusion_moves);

struct SliceEvidence {
    size_t layers = 0;
    size_t perimeter_items = 0;
    size_t perimeter_layers = 0;
    size_t top_layers = 0;
    size_t bottom_layers = 0;
    size_t top_nominal_shell_layers = 0;
    size_t bottom_nominal_shell_layers = 0;
    size_t top_bridge_supported_layers = 0;
    size_t bottom_bridge_supported_layers = 0;
    double perimeter_length = 0.0;
    double sparse_length = 0.0;
    double solid_length = 0.0;
    double top_length = 0.0;
    double bottom_length = 0.0;
    double top_shell_length = 0.0;
    double bottom_shell_length = 0.0;
    double top_solid_surface_area = 0.0;
    double bottom_solid_surface_area = 0.0;
    size_t parsed_motion_moves = 0;
    size_t parsed_extrusion_moves = 0;
    bool gcode_valid = false;
    std::string gcode;
};

SliceEvidence slice_fixture(DynamicPrintConfig config, TriangleMesh shape = cube(20.0))
{
    SliceEvidence evidence;
    Print print;
    Model model;
    // Keep the useful 20 mm XY footprint, but give the fixture a physically
    // valid 4 mm Z range for the default 0.4 mm nozzle and 0.2 mm layers.
    shape.scale(Vec3f(1.0f, 1.0f, 0.2f));
    init_print({ shape }, print, model, config);
    REQUIRE(print.validate().string.empty());
    REQUIRE_NOTHROW(print.process());
    REQUIRE_FALSE(print.objects().empty());

    const PrintObject &object = *print.objects().front();
    evidence.layers = object.layers().size();
    std::vector<double> shell_length_by_layer;
    shell_length_by_layer.reserve(evidence.layers);
    std::vector<double> solid_surface_area_by_layer;
    solid_surface_area_by_layer.reserve(evidence.layers);
    std::vector<double> nominal_shell_area_by_layer;
    nominal_shell_area_by_layer.reserve(evidence.layers);
    std::vector<double> bridge_surface_area_by_layer;
    bridge_surface_area_by_layer.reserve(evidence.layers);
    for (const Layer *layer : object.layers()) {
        double layer_shell_length = 0.0;
        double layer_solid_surface_area = 0.0;
        double layer_nominal_shell_area = 0.0;
        double layer_bridge_surface_area = 0.0;
        size_t layer_perimeter_items = 0;
        for (const LayerRegion *region : layer->regions()) {
            evidence.perimeter_length += collection_length(region->perimeters);
            layer_perimeter_items += region->perimeters.items_count();
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                if (surface.is_solid()) {
                    layer_solid_surface_area += surface.expolygon.area();
                    // stTop/stBottom plus stInternalSolid are the surfaces
                    // requested by the shell setting. Bridge surfaces are
                    // tracked separately because bridge_over_infill() can
                    // convert one of those solid layers to stInternalBridge.
                    if (surface.surface_type == stTop || surface.surface_type == stBottom ||
                        surface.surface_type == stInternalSolid)
                        layer_nominal_shell_area += surface.expolygon.area();
                }
                if (surface.surface_type == stBottomBridge || surface.surface_type == stInternalBridge)
                    layer_bridge_surface_area += surface.expolygon.area();
            }
            for_each_extrusion_path(region->fills, [&evidence, &layer_shell_length](const ExtrusionPath &path) {
                switch (path.role()) {
                case erInternalInfill:  evidence.sparse_length += path.length(); break;
                case erSolidInfill:
                    evidence.solid_length += path.length();
                    layer_shell_length += path.length();
                    break;
                case erTopSolidInfill:
                    evidence.top_length += path.length();
                    layer_shell_length += path.length();
                    break;
                case erBottomSurface:
                    evidence.bottom_length += path.length();
                    layer_shell_length += path.length();
                    break;
                case erInternalBridgeInfill:
                    // Orca can route an additional solid skin over a bridge
                    // surface through this role.  It is still shell coverage
                    // for the extremal-layer oracle.
                    layer_shell_length += path.length();
                    break;
                default:
                    break;
                }
            });
        }
        evidence.perimeter_items += layer_perimeter_items;
        if (layer_perimeter_items > 0)
            ++evidence.perimeter_layers;
        shell_length_by_layer.push_back(layer_shell_length);
        solid_surface_area_by_layer.push_back(layer_solid_surface_area);
        nominal_shell_area_by_layer.push_back(layer_nominal_shell_area);
        bridge_surface_area_by_layer.push_back(layer_bridge_surface_area);
    }

    // The outermost surface has the top/bottom role, while preceding shell
    // layers use erSolidInfill or (for bridge-backed skins) the internal
    // bridge role. Count contiguous coverage from each extremity and retain
    // the nominal shell and bridge-supported counts separately. The pipeline
    // first propagates the configured shell count in discover_horizontal_shells()
    // and then bridge_over_infill() can reclassify the first propagated solid
    // layer over sparse infill as stInternalBridge. That layer is physically
    // solid coverage, but it is not a nominal shell layer.
    evidence.bottom_layers = 0;
    while (evidence.bottom_layers < solid_surface_area_by_layer.size() &&
           solid_surface_area_by_layer[evidence.bottom_layers] > 0.0) {
        evidence.bottom_solid_surface_area += solid_surface_area_by_layer[evidence.bottom_layers];
        evidence.bottom_shell_length += shell_length_by_layer[evidence.bottom_layers];
        if (nominal_shell_area_by_layer[evidence.bottom_layers] > 0.0)
            ++evidence.bottom_nominal_shell_layers;
        if (bridge_surface_area_by_layer[evidence.bottom_layers] > 0.0)
            ++evidence.bottom_bridge_supported_layers;
        ++evidence.bottom_layers;
    }
    evidence.top_layers = 0;
    while (evidence.top_layers < solid_surface_area_by_layer.size() &&
           solid_surface_area_by_layer[solid_surface_area_by_layer.size() - 1 - evidence.top_layers] > 0.0) {
        const size_t index = solid_surface_area_by_layer.size() - 1 - evidence.top_layers;
        evidence.top_solid_surface_area += solid_surface_area_by_layer[index];
        evidence.top_shell_length += shell_length_by_layer[index];
        if (nominal_shell_area_by_layer[index] > 0.0)
            ++evidence.top_nominal_shell_layers;
        if (bridge_surface_area_by_layer[index] > 0.0)
            ++evidence.top_bridge_supported_layers;
        ++evidence.top_layers;
    }

    REQUIRE_NOTHROW(evidence.gcode = gcode(print));
    evidence.gcode_valid = finite_bounded_gcode(evidence.gcode, print.config(), evidence.parsed_motion_moves,
                                                evidence.parsed_extrusion_moves);
    return evidence;
}

bool is_motion_command(const GCodeReader::GCodeLine &line)
{
    return line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3") ||
           line.cmd_is("G92");
}

bool raw_motion_axes_are_finite(const GCodeReader::GCodeLine &line)
{
    // GCodeReader silently skips malformed axis words, so inspect the raw
    // motion words first instead of allowing a skipped value to look safe.
    const std::string &raw = line.raw();
    const size_t comment = raw.find(';');
    const size_t end = comment == std::string::npos ? raw.size() : comment;
    auto whitespace = [](const char c) { return c == ' ' || c == '\t'; };
    size_t pos = 0;
    while (pos < end && whitespace(raw[pos]))
        ++pos;
    while (pos < end && !whitespace(raw[pos]))
        ++pos; // command word

    while (pos < end) {
        while (pos < end && whitespace(raw[pos]))
            ++pos;
        const size_t word_begin = pos;
        while (pos < end && !whitespace(raw[pos]))
            ++pos;
        if (word_begin == pos)
            continue;

        const char axis = raw[word_begin];
        const bool is_axis = axis == 'X' || axis == 'Y' || axis == 'Z' || axis == 'E' || axis == 'F' ||
                             axis == 'I' || axis == 'J';
        if (!is_axis)
            continue;
        if (word_begin + 1 == pos)
            return false;

        const std::string value_text = raw.substr(word_begin + 1, pos - word_begin - 1);
        char *value_end = nullptr;
        const double value = std::strtod(value_text.c_str(), &value_end);
        if (value_end != value_text.c_str() + value_text.size() || !std::isfinite(value))
            return false;
        const double bound = axis == 'E' || axis == 'F' ? 100000.0 : 1000.0;
        if (std::abs(value) > bound)
            return false;
    }
    return true;
}

bool finite_bounded_gcode(const std::string &output, const GCodeConfig &config,
                          size_t &parsed_motion_moves, size_t &parsed_extrusion_moves)
{
    bool valid = true;
    GCodeReader reader;
    // Relative-E mode is part of the active print config; applying it here is
    // required for line.extruding() and dist_E() to describe the emitted file.
    reader.apply_config(config);
    reader.parse_buffer(output, [&valid, &parsed_motion_moves, &parsed_extrusion_moves](GCodeReader &self,
                                                                                          const GCodeReader::GCodeLine &line) {
        if (!is_motion_command(line))
            return;
        if (!raw_motion_axes_are_finite(line))
            valid = false;
        const bool moved = line.dist_XY(self) > EPSILON || std::abs(line.dist_Z(self)) > EPSILON;
        if (moved)
            ++parsed_motion_moves;
        if (line.extruding(self) && line.dist_XY(self) > EPSILON)
            ++parsed_extrusion_moves;
    });
    return valid;
}

std::optional<std::pair<double, double>> first_outer_wall_start(const std::string &output)
{
    std::optional<std::pair<double, double>> start;
    bool outer_wall = false;
    GCodeReader reader;
    reader.parse_buffer(output, [&start, &outer_wall](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("Outer wall") != std::string_view::npos)
            outer_wall = true;
        else if (comment.find("FEATURE:") != std::string_view::npos ||
                 comment.find("TYPE:") != std::string_view::npos)
            outer_wall = false;
        if (start.has_value() || !outer_wall || !line.extruding(self))
            return;
        // GCodeReader invokes the callback before applying the line, so the
        // reader coordinates are the actual start of this extrusion.
        start = std::make_pair(static_cast<double>(self.x()), static_cast<double>(self.y()));
    });
    return start;
}

void require_valid_slice(const SliceEvidence &evidence)
{
    REQUIRE(evidence.layers > 0);
    REQUIRE(evidence.perimeter_length > 0.0);
    REQUIRE_FALSE(evidence.gcode.empty());
    REQUIRE(std::isfinite(max_z(evidence.gcode)));
    REQUIRE(evidence.gcode_valid);
    REQUIRE(evidence.parsed_motion_moves > 0);
    REQUIRE(evidence.parsed_extrusion_moves > 0);
}

const ConfigOptionDef &enum_definition(const char *key)
{
    const ConfigOptionDef *definition = print_config_def.get(key);
    REQUIRE(definition != nullptr);
    REQUIRE_FALSE(definition->enum_values.empty());
    return *definition;
}

} // namespace

TEST_CASE("G-code contract rejects malformed motion words", "[SettingsVariance]")
{
    const GCodeConfig config;
    size_t motion_moves = 0;
    size_t extrusion_moves = 0;
    CHECK(finite_bounded_gcode("G1 X1 Y0 E1\n", config, motion_moves, extrusion_moves));
    CHECK(motion_moves > 0);
    CHECK(extrusion_moves > 0);

    motion_moves = 0;
    extrusion_moves = 0;
    CHECK_FALSE(finite_bounded_gcode("G1 Xnan Y0 E1\n", config, motion_moves, extrusion_moves));
    CHECK_FALSE(finite_bounded_gcode("G1 X1e999 Y0 E1\n", config, motion_moves, extrusion_moves));
    CHECK_FALSE(finite_bounded_gcode("G1 Xoops Y0 E1\n", config, motion_moves, extrusion_moves));
}

TEST_CASE("Every registered sparse infill pattern is accepted and emits sparse role geometry", "[SettingsVariance]")
{
    const ConfigOptionDef &definition = enum_definition("sparse_infill_pattern");
    for (const std::string &pattern : definition.enum_values) {
        CAPTURE(pattern);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("sparse_infill_pattern", pattern));
        REQUIRE(config.option("sparse_infill_pattern")->serialize() == pattern);

        const SliceEvidence evidence = slice_fixture(std::move(config), mesh(TestMesh::slopy_cube));
        require_valid_slice(evidence);
        // This sweep proves the schema token reaches a sparse-role output. The
        // bounded pairwise case below provides the selected routing distinctness
        // oracle; this loop does not claim every pattern has a unique length.
        REQUIRE(evidence.sparse_length > 0.0);
    }
}

TEST_CASE("Every registered top and bottom surface pattern produces its role", "[SettingsVariance]")
{
    const ConfigOptionDef &top_definition = enum_definition("top_surface_pattern");
    const ConfigOptionDef &bottom_definition = enum_definition("bottom_surface_pattern");
    SliceEvidence top_monotonic;
    SliceEvidence top_concentric;
    SliceEvidence bottom_monotonic;
    SliceEvidence bottom_concentric;

    for (const std::string &pattern : top_definition.enum_values) {
        CAPTURE(pattern);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("top_surface_pattern", pattern));
        REQUIRE(config.option("top_surface_pattern")->serialize() == pattern);
        const SliceEvidence evidence = slice_fixture(std::move(config));
        require_valid_slice(evidence);
        REQUIRE(evidence.top_length > 0.0);
        if (pattern == "monotonic")
            top_monotonic = evidence;
        if (pattern == "concentric")
            top_concentric = evidence;
    }

    for (const std::string &pattern : bottom_definition.enum_values) {
        CAPTURE(pattern);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("bottom_surface_pattern", pattern));
        REQUIRE(config.option("bottom_surface_pattern")->serialize() == pattern);
        const SliceEvidence evidence = slice_fixture(std::move(config));
        require_valid_slice(evidence);
        REQUIRE(evidence.bottom_length > 0.0);
        if (pattern == "monotonic")
            bottom_monotonic = evidence;
        if (pattern == "concentric")
            bottom_concentric = evidence;
    }

    CHECK(std::abs(top_monotonic.top_length - top_concentric.top_length) > 0.01);
    CHECK(std::abs(bottom_monotonic.bottom_length - bottom_concentric.bottom_length) > 0.01);
}

TEST_CASE("Wall, shell, and seam modes preserve slicing contracts", "[SettingsVariance]")
{
    SliceEvidence classic_evidence;
    SliceEvidence arachne_evidence;
    for (const std::string &wall_generator : { "classic", "arachne" }) {
        CAPTURE(wall_generator);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("wall_generator", wall_generator));
        REQUIRE(config.option("wall_generator")->serialize() == wall_generator);
        const SliceEvidence evidence = slice_fixture(std::move(config), mesh(TestMesh::step));
        require_valid_slice(evidence);
        if (wall_generator == "classic")
            classic_evidence = evidence;
        else
            arachne_evidence = evidence;
    }
    // The ledged fixture exercises the variable-width decision, so this is a
    // semantic check that the selected generator reached the pipeline.
    CHECK(std::abs(classic_evidence.perimeter_length - arachne_evidence.perimeter_length) > 0.01);

    const ConfigOptionDef &shell_definition = enum_definition("ensure_vertical_shell_thickness");
    SliceEvidence no_extra_shells;
    SliceEvidence all_shells;
    for (const std::string &shell_mode : shell_definition.enum_values) {
        CAPTURE(shell_mode);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("ensure_vertical_shell_thickness", shell_mode));
        REQUIRE(config.option("ensure_vertical_shell_thickness")->serialize() == shell_mode);
        const SliceEvidence evidence = slice_fixture(std::move(config), mesh(TestMesh::slopy_cube));
        require_valid_slice(evidence);
        if (shell_mode == "none")
            no_extra_shells = evidence;
        if (shell_mode == "ensure_all")
            all_shells = evidence;
    }
    CHECK(std::abs(no_extra_shells.solid_length - all_shells.solid_length) > 0.01);

    const ConfigOptionDef &seam_definition = enum_definition("seam_position");
    SliceEvidence nearest_seam;
    SliceEvidence random_seam;
    for (const std::string &seam_position : seam_definition.enum_values) {
        CAPTURE(seam_position);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("seam_position", seam_position));
        REQUIRE(config.option("seam_position")->serialize() == seam_position);
        const SliceEvidence evidence = slice_fixture(std::move(config));
        require_valid_slice(evidence);
        if (seam_position == "nearest")
            nearest_seam = evidence;
        if (seam_position == "random")
            random_seam = evidence;
    }
    const std::optional<std::pair<double, double>> nearest_start = first_outer_wall_start(nearest_seam.gcode);
    const std::optional<std::pair<double, double>> random_start = first_outer_wall_start(random_seam.gcode);
    REQUIRE(nearest_start.has_value());
    REQUIRE(random_start.has_value());
    const double seam_distance = std::hypot(nearest_start->first - random_start->first,
                                            nearest_start->second - random_start->second);
    CHECK(seam_distance > 0.01);
}

TEST_CASE("Meaningful numeric boundaries retain a valid print contract", "[SettingsVariance]")
{
    for (const std::string &density : { "0%", "15%", "100%" }) {
        CAPTURE(density);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict("sparse_infill_density", density));
        REQUIRE(config.option("sparse_infill_density")->serialize() == density);
        const SliceEvidence evidence = slice_fixture(std::move(config));
        require_valid_slice(evidence);
        if (density == "0%")
            CHECK_THAT(evidence.sparse_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
        else if (density == "100%")
            CHECK(evidence.solid_length > 0.0);
        else
            CHECK(evidence.sparse_length > 0.0);
    }

    SliceEvidence one_wall;
    SliceEvidence four_walls;
    for (const int wall_loops : { 0, 1, 4 }) {
        CAPTURE(wall_loops);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict({
            { "wall_loops",         wall_loops },
            { "top_shell_layers",   0 },
            { "bottom_shell_layers", 0 },
        }));
        REQUIRE(config.option("wall_loops")->serialize() == std::to_string(wall_loops));
        const SliceEvidence evidence = slice_fixture(std::move(config));
        REQUIRE(evidence.layers > 0);
        REQUIRE_FALSE(evidence.gcode.empty());
        REQUIRE(evidence.gcode_valid);
        REQUIRE(evidence.parsed_motion_moves > 0);
        REQUIRE(evidence.parsed_extrusion_moves > 0);
        if (wall_loops == 0)
        {
            CHECK_THAT(evidence.perimeter_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK(evidence.perimeter_items == 0);
            CHECK(evidence.perimeter_layers == 0);
        }
        else {
            CHECK(evidence.perimeter_length > 0.0);
            CHECK(evidence.perimeter_items > 0);
            CHECK(evidence.perimeter_layers > 0);
        }
        if (wall_loops == 1)
            one_wall = evidence;
        if (wall_loops == 4)
            four_walls = evidence;
    }
    CHECK(four_walls.perimeter_items > one_wall.perimeter_items);
    CHECK(four_walls.perimeter_length > one_wall.perimeter_length);

    SliceEvidence one_shell;
    SliceEvidence two_shells;
    for (const int shell_layers : { 0, 1, 2 }) {
        CAPTURE(shell_layers);
        DynamicPrintConfig config = variance_config();
        REQUIRE_NOTHROW(config.set_deserialize_strict({
            { "top_shell_layers",    shell_layers },
            { "bottom_shell_layers", shell_layers },
        }));
        REQUIRE(config.option("top_shell_layers")->serialize() == std::to_string(shell_layers));
        REQUIRE(config.option("bottom_shell_layers")->serialize() == std::to_string(shell_layers));
        const SliceEvidence evidence = slice_fixture(std::move(config));
        REQUIRE(evidence.layers > 0);
        REQUIRE_FALSE(evidence.gcode.empty());
        REQUIRE(evidence.gcode_valid);
        REQUIRE(evidence.parsed_motion_moves > 0);
        REQUIRE(evidence.parsed_extrusion_moves > 0);
        if (shell_layers == 0) {
            CHECK_THAT(evidence.top_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK_THAT(evidence.bottom_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK_THAT(evidence.top_shell_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK_THAT(evidence.bottom_shell_length, Catch::Matchers::WithinAbs(0.0, 1e-6));
            CHECK(evidence.top_layers == 0);
            CHECK(evidence.bottom_layers == 0);
            CHECK(evidence.top_nominal_shell_layers == 0);
            CHECK(evidence.bottom_nominal_shell_layers == 0);
            CHECK(evidence.top_bridge_supported_layers == 0);
            CHECK(evidence.bottom_bridge_supported_layers == 0);
        } else {
            CHECK(evidence.top_length > 0.0);
            CHECK(evidence.bottom_length > 0.0);
            CHECK(evidence.top_shell_length > 0.0);
            CHECK(evidence.bottom_shell_length > 0.0);
            CHECK(evidence.top_nominal_shell_layers == static_cast<size_t>(shell_layers));
            CHECK(evidence.bottom_nominal_shell_layers == static_cast<size_t>(shell_layers));
            CHECK(evidence.top_layers == evidence.top_nominal_shell_layers + evidence.top_bridge_supported_layers);
            CHECK(evidence.bottom_layers == evidence.bottom_nominal_shell_layers + evidence.bottom_bridge_supported_layers);
        }
        if (shell_layers == 1)
            one_shell = evidence;
        if (shell_layers == 2)
            two_shells = evidence;
    }
    CHECK(two_shells.top_layers > one_shell.top_layers);
    CHECK(two_shells.bottom_layers > one_shell.bottom_layers);
    // The emitted path length may remain equal when the fill pattern uses a
    // shared solid path representation.  The fill-surface areas are the
    // independent shell-depth oracle and must grow with the configured count.
    CHECK(two_shells.top_solid_surface_area > one_shell.top_solid_surface_area);
    CHECK(two_shells.bottom_solid_surface_area > one_shell.bottom_solid_surface_area);
}

TEST_CASE("Wall generator and sparse infill remain valid across a bounded pairwise matrix", "[SettingsVariance]")
{
    for (const std::string &wall_generator : { "classic", "arachne" }) {
        const std::string first_pattern = "rectilinear";
        const std::string second_pattern = "gyroid";
        SliceEvidence first_evidence;
        SliceEvidence second_evidence;
        for (const std::string &pattern : { first_pattern, second_pattern }) {
            CAPTURE(wall_generator, pattern);
            DynamicPrintConfig config = variance_config();
            REQUIRE_NOTHROW(config.set_deserialize_strict({
                { "wall_generator",        wall_generator },
                { "sparse_infill_pattern", pattern },
            }));
            REQUIRE(config.option("wall_generator")->serialize() == wall_generator);
            REQUIRE(config.option("sparse_infill_pattern")->serialize() == pattern);
            const SliceEvidence evidence = slice_fixture(std::move(config), mesh(TestMesh::step));
            require_valid_slice(evidence);
            REQUIRE(evidence.sparse_length > 0.0);
            if (pattern == first_pattern)
                first_evidence = evidence;
            else
                second_evidence = evidence;
        }
        CHECK(std::abs(first_evidence.sparse_length - second_evidence.sparse_length) > 0.01);
    }
}

namespace seam_distribution_contract {
using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct OuterWallGeometry {
    struct Start { double x; double y; double z; };
    std::vector<Start> starts;
    double min_x = 0.0;
    double max_x = 0.0;
    double min_y = 0.0;
    double max_y = 0.0;
};

DynamicPrintConfig seam_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height", 0.2 },
        { "initial_layer_print_height", 0.2 },
        { "wall_loops", 2 },
        { "top_shell_layers", 2 },
        { "bottom_shell_layers", 2 },
        { "top_shell_thickness", 0.0 },
        { "bottom_shell_thickness", 0.0 },
        { "sparse_infill_density", "15%" },
        { "top_surface_density", "100%" },
        { "bottom_surface_density", "100%" },
        { "gcode_comments", true },
        { "layer_change_gcode", "G92 E0\n" },
        { "seam_position", "random" },
    });
    return config;
}

std::string slice_without_arrangement(TriangleMesh shape)
{
    shape.scale(Vec3f(1.5f, 1.0f, 0.2f));
    Print print;
    Model model;
    DynamicPrintConfig config = seam_config();
    init_print(std::vector<TriangleMesh>{ std::move(shape) }, print, model, config, nullptr, false);
    REQUIRE(print.validate().string.empty());
    print.process();
    return gcode(print);
}

OuterWallGeometry outer_wall_geometry(const std::string &output)
{
    OuterWallGeometry result;
    bool outer_wall = false;
    bool bounds_initialized = false;
    GCodeReader reader;
    reader.parse_buffer(output, [&result, &outer_wall, &bounds_initialized](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("Outer wall") != std::string_view::npos)
            outer_wall = true;
        else if (comment.find("FEATURE:") != std::string_view::npos ||
                 comment.find("TYPE:") != std::string_view::npos)
            outer_wall = false;
        if (!outer_wall || !line.extruding(self) || line.dist_XY(self) <= EPSILON)
            return;

        const double x0 = self.x();
        const double y0 = self.y();
        const double x1 = line.has_x() ? line.x() : self.x();
        const double y1 = line.has_y() ? line.y() : self.y();
        if (result.starts.empty() || std::abs(result.starts.back().z - self.z()) > 1e-4)
            result.starts.push_back({ x0, y0, self.z() });
        if (!bounds_initialized) {
            result.min_x = std::min(x0, x1);
            result.max_x = std::max(x0, x1);
            result.min_y = std::min(y0, y1);
            result.max_y = std::max(y0, y1);
            bounds_initialized = true;
        } else {
            result.min_x = std::min({ result.min_x, x0, x1 });
            result.max_x = std::max({ result.max_x, x0, x1 });
            result.min_y = std::min({ result.min_y, y0, y1 });
            result.max_y = std::max({ result.max_y, y0, y1 });
        }
    });
    return result;
}

int nearest_side(const OuterWallGeometry &geometry, const OuterWallGeometry::Start &start)
{
    const double distances[] = {
        std::abs(start.x - geometry.max_x), std::abs(start.x - geometry.min_x),
        std::abs(start.y - geometry.max_y), std::abs(start.y - geometry.min_y)
    };
    return static_cast<int>(std::min_element(distances, distances + 4) - distances);
}

} // namespace

TEST_CASE("Random seams visit multiple sides of emitted outer walls", "[SettingsVariance][SeamDistribution]")
{
    // Seed the process PRNG so the same geometry produces a reproducible distribution.
    std::srand(0x5EEDu);
    std::set<int> selected_sides;
    for (const Vec3f &offset : {
             Vec3f(0.0f, 0.0f, 0.0f), Vec3f(17.0f, 13.0f, 0.0f),
             Vec3f(-23.0f, 19.0f, 0.0f), Vec3f(31.0f, -15.0f, 0.0f) }) {
        TriangleMesh shape = cube(20.0);
        shape.translate(offset);
        const OuterWallGeometry geometry = outer_wall_geometry(slice_without_arrangement(std::move(shape)));
        REQUIRE(geometry.starts.size() > 1);
        REQUIRE(geometry.max_x > geometry.min_x);
        REQUIRE(geometry.max_y > geometry.min_y);
        for (const OuterWallGeometry::Start &start : geometry.starts)
            selected_sides.insert(nearest_side(geometry, start));
    }
    CHECK(selected_sides.size() >= 2);
}

} // namespace seam_distribution_contract

namespace wall_bridge_contracts {

DynamicPrintConfig wall_infill_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height",                0.2},
        {"initial_layer_print_height",  0.2},
        {"wall_generator",              "classic"},
        {"wall_loops",                  2},
        {"top_shell_layers",            0},
        {"bottom_shell_layers",         0},
        {"top_shell_thickness",         0.0},
        {"bottom_shell_thickness",      0.0},
        {"sparse_infill_density",       "20%"},
        {"sparse_infill_pattern",       "rectilinear"},
        {"skirt_loops",                 0},
        {"brim_type",                   "no_brim"},
        {"raft_layers",                 0},
        {"slow_down_for_layer_cooling", false},
        {"filament_max_volumetric_speed", "200"},
        {"gcode_comments",              true},
        {"layer_change_gcode",           "G92 E0\n"},
        {"machine_start_gcode",          ""},
        {"machine_end_gcode",            ""},
    });
    return config;
}

TriangleMesh thin_cube()
{
    TriangleMesh mesh = cube(20.0);
    mesh.scale(Vec3f(1.f, 1.f, 0.2f)); // 4 mm, twenty 0.2 mm layers.
    return mesh;
}

void process(Print &print, Model &model, const DynamicPrintConfig &config)
{
    print.apply(model, config);
    REQUIRE(print.validate().string.empty());
    REQUIRE_NOTHROW(print.process());
    REQUIRE_FALSE(print.objects().empty());
    REQUIRE(print.objects().front()->layer_count() > 2);
}

struct WallSnapshot {
    std::vector<std::vector<ExtrusionRole>> layer_roles;
    std::vector<size_t> perimeter_items;
    std::vector<size_t> external_perimeter_indices;
};

WallSnapshot walls(const Print &print)
{
    WallSnapshot result;
    const PrintObject &object = *print.objects().front();
    result.layer_roles.reserve(object.layer_count());
    result.perimeter_items.reserve(object.layer_count());
    result.external_perimeter_indices.reserve(object.layer_count());
    for (const Layer *layer : object.layers()) {
        std::vector<ExtrusionRole> roles;
        size_t items = 0;
        for (const LayerRegion *region : layer->regions()) {
            items += region->perimeters.items_count();
            for_each_extrusion_path(region->perimeters, [&roles](const ExtrusionPath &path) {
                roles.push_back(path.role());
            });
        }
        result.layer_roles.push_back(std::move(roles));
        result.perimeter_items.push_back(items);
        const auto external = std::find(result.layer_roles.back().begin(),
                                        result.layer_roles.back().end(), erExternalPerimeter);
        result.external_perimeter_indices.push_back(external == result.layer_roles.back().end() ?
            result.layer_roles.back().size() :
            static_cast<size_t>(std::distance(result.layer_roles.back().begin(), external)));
    }
    return result;
}

struct InfillSnapshot {
    std::vector<size_t> sparse_path_count;
    std::vector<size_t> sparse_surface_count;
    std::vector<double> sparse_path_heights;
    size_t void_surface_count = 0;
};

InfillSnapshot infill(const Print &print)
{
    InfillSnapshot result;
    const PrintObject &object = *print.objects().front();
    result.sparse_path_count.reserve(object.layer_count());
    result.sparse_surface_count.reserve(object.layer_count());
    for (const Layer *layer : object.layers()) {
        size_t paths = 0;
        size_t surfaces = 0;
        for (const LayerRegion *region : layer->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                if (surface.surface_type == stInternal)
                    ++surfaces;
                if (surface.surface_type == stInternalVoid)
                    ++result.void_surface_count;
            }
            for_each_extrusion_path(region->fills, [&paths, &result](const ExtrusionPath &path) {
                if (path.role() == erInternalInfill) {
                    ++paths;
                    result.sparse_path_heights.push_back(path.height);
                }
            });
        }
        result.sparse_path_count.push_back(paths);
        result.sparse_surface_count.push_back(surfaces);
    }
    return result;
}

TriangleMesh internal_bridge_step()
{
    TriangleMesh mesh = make_cube(30, 24, 3);
    TriangleMesh tower = make_cube(14, 10, 1);
    tower.translate(8, 7, 3);
    mesh.merge(tower);
    return mesh;
}

TriangleMesh external_bridge_fixture()
{
    TriangleMesh mesh = Slic3r::Test::mesh(TestMesh::bridge);
    // Keep the existing bridge mesh's XY span while making a short, valid
    // multi-layer object, as the FFF boolean-setting fixtures do.
    mesh.scale(Vec3f(1.f, 1.f, 0.2f));
    return mesh;
}

DynamicPrintConfig bridge_config()
{
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({
        {"sparse_infill_density",  "15%"},
        {"top_shell_layers",        3},
        {"bottom_shell_layers",     2},
        {"top_shell_thickness",     0.0},
        {"bottom_shell_thickness",  0.0},
        {"internal_bridge_angle",   0.0},
        {"thick_internal_bridges",   false},
    });
    return config;
}

struct BridgeSnapshot {
    size_t surfaces = 0;
    size_t paths = 0;
    double volume = 0.0;
    std::vector<double> angles;
};

BridgeSnapshot bridges(const Print &print)
{
    BridgeSnapshot result;
    const PrintObject &object = *print.objects().front();
    for (const Layer *layer : object.layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                if (surface.surface_type != stInternalBridge)
                    continue;
                ++result.surfaces;
                result.angles.push_back(surface.bridge_angle);
            }
            for_each_extrusion_path(region->fills, [&result](const ExtrusionPath &path) {
                if (path.role() == erInternalBridgeInfill) {
                    ++result.paths;
                    result.volume += path.total_volume();
                }
            });
        }
    }
    return result;
}

std::vector<std::vector<double>> bridge_angles_by_layer(const Print &print)
{
    std::vector<std::vector<double>> result;
    const PrintObject &object = *print.objects().front();
    result.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        std::vector<double> angles;
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->fill_surfaces.surfaces)
                if (surface.surface_type == stInternalBridge)
                    angles.push_back(surface.bridge_angle);
        result.push_back(std::move(angles));
    }
    return result;
}

struct ExternalBridgeSnapshot {
    size_t paths = 0;
    double volume = 0.0;
};

ExternalBridgeSnapshot external_bridges(const Print &print)
{
    ExternalBridgeSnapshot result;
    for (const Layer *layer : print.objects().front()->layers())
        for (const LayerRegion *region : layer->regions())
            for_each_extrusion_path(region->fills, [&result](const ExtrusionPath &path) {
                if (path.role() == erBridgeInfill) {
                    ++result.paths;
                    result.volume += path.total_volume();
                }
            });
    return result;
}

TEST_CASE("Wall sequence variants preserve fresh/incremental perimeters and ordering", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    const std::string sequence = GENERATE("inner wall/outer wall", "outer wall/inner wall", "inner-outer-inner wall");
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({{"wall_loops", 3}, {"wall_sequence", "inner wall/outer wall"}});

    Print print;
    Model model;
    init_print({thin_cube()}, print, model, config, nullptr, false);
    process(print, model, config);
    const WallSnapshot baseline = walls(print);

    config.set_deserialize_strict({{"wall_sequence", sequence}});
    process(print, model, config);
    const WallSnapshot incremental = walls(print);

    Print fresh;
    Model fresh_model;
    init_print({thin_cube()}, fresh, fresh_model, config, nullptr, false);
    process(fresh, fresh_model, config);
    const WallSnapshot expected = walls(fresh);

    REQUIRE(incremental.layer_roles == expected.layer_roles);
    REQUIRE(incremental.perimeter_items == expected.perimeter_items);
    REQUIRE_FALSE(incremental.layer_roles.empty());
    for (const auto &roles : incremental.layer_roles) {
        REQUIRE(roles.size() >= 3);
        CHECK(std::count(roles.begin(), roles.end(), erExternalPerimeter) == 1);
        CHECK(std::count(roles.begin(), roles.end(), erPerimeter) >= 2);
    }
    const size_t expected_external_index = sequence == "outer wall/inner wall" ? 0 :
        (sequence == "inner-outer-inner wall" ? 1 : 2);
    REQUIRE(incremental.external_perimeter_indices.size() > 2);
    for (size_t layer = 1; layer + 1 < incremental.layer_roles.size(); ++layer) {
        REQUIRE(incremental.layer_roles[layer].size() == 3);
        CHECK(incremental.external_perimeter_indices[layer] == expected_external_index);
    }
    if (sequence == "inner wall/outer wall")
        CHECK(incremental.layer_roles == baseline.layer_roles);
    else
        CHECK(incremental.layer_roles != baseline.layer_roles);
}

TEST_CASE("Alternate extra wall adds only alternating infill-supported layers", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    const bool alternate = GENERATE(false, true);
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({{"alternate_extra_wall", alternate}});

    Print print;
    Model model;
    init_print({thin_cube()}, print, model, config, nullptr, false);
    process(print, model, config);
    const WallSnapshot actual = walls(print);

    REQUIRE(actual.perimeter_items.size() >= 4);
    for (size_t layer = 0; layer < actual.perimeter_items.size(); ++layer) {
        const size_t expected = alternate && (layer % 2 == 1) ? 3 : 2;
        CHECK(actual.perimeter_items[layer] == expected);
    }
}

TEST_CASE("Infill combination turns lower layers into void and bounds sparse path heights", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    const double max_height = GENERATE(0.4, 0.6);
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({
        {"wall_loops", 1},
        {"infill_combination", true},
        {"infill_combination_max_layer_height", max_height},
        {"nozzle_diameter", "0.8"},
    });

    Print print;
    Model model;
    init_print({thin_cube()}, print, model, config, nullptr, false);
    process(print, model, config);
    const InfillSnapshot actual = infill(print);

    size_t sparse_layers = 0;
    for (size_t i = 0; i < actual.sparse_path_count.size(); ++i) {
        if (actual.sparse_path_count[i] > 0)
            ++sparse_layers;
    }
    REQUIRE(sparse_layers > 0);
    REQUIRE(actual.void_surface_count > 0);
    REQUIRE_FALSE(actual.sparse_path_heights.empty());
    for (double height : actual.sparse_path_heights) {
        REQUIRE(height > 0.0);
        CHECK(height <= max_height + 1e-6);
    }

    DynamicPrintConfig baseline_config = config;
    baseline_config.set_deserialize_strict({{"infill_combination", false}});
    Print baseline_print;
    Model baseline_model;
    init_print({thin_cube()}, baseline_print, baseline_model, baseline_config, nullptr, false);
    process(baseline_print, baseline_model, baseline_config);
    const InfillSnapshot baseline = infill(baseline_print);
    size_t baseline_sparse_layers = 0;
    for (size_t paths : baseline.sparse_path_count)
        baseline_sparse_layers += paths > 0;
    CHECK(sparse_layers < baseline_sparse_layers);
}

TEST_CASE("Infill combination height cap bounds and changes sparse path heights", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](double max_height) {
        DynamicPrintConfig config = wall_infill_config();
        config.set_deserialize_strict({
            {"wall_loops", 1},
            {"infill_combination", true},
            {"infill_combination_max_layer_height", max_height},
            {"nozzle_diameter", "0.8"},
        });
        Print print;
        Model model;
        init_print({thin_cube()}, print, model, config, nullptr, false);
        process(print, model, config);
        return infill(print);
    };
    const InfillSnapshot low = run(0.2);
    const InfillSnapshot high = run(0.6);
    CAPTURE(low.sparse_surface_count, high.sparse_surface_count,
            low.void_surface_count, high.void_surface_count,
            low.sparse_path_heights, high.sparse_path_heights);
    REQUIRE_FALSE(low.sparse_path_heights.empty());
    REQUIRE_FALSE(high.sparse_path_heights.empty());
    for (double height : low.sparse_path_heights) {
        REQUIRE(height > 0.0);
        CHECK(height <= 0.2 + 1e-6);
    }
    for (double height : high.sparse_path_heights) {
        REQUIRE(height > 0.0);
        CHECK(height <= 0.6 + 1e-6);
    }
    CHECK(std::any_of(high.sparse_path_heights.begin(), high.sparse_path_heights.end(),
                      [](double height) { return height > 0.2 + 1e-6; }));
    CHECK(low.sparse_surface_count != high.sparse_surface_count);
    CHECK(low.void_surface_count != high.void_surface_count);
}

TEST_CASE("Changing infill combination invalidates the prepared sparse surfaces", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({
        {"wall_loops", 1},
        {"infill_combination", false},
        {"infill_combination_max_layer_height", 0.4},
    });
    Print print;
    Model model;
    init_print({thin_cube()}, print, model, config, nullptr, false);
    process(print, model, config);
    config.set_deserialize_strict({{"infill_combination", true}});
    process(print, model, config);
    const InfillSnapshot incremental = infill(print);

    Print fresh;
    Model fresh_model;
    init_print({thin_cube()}, fresh, fresh_model, config, nullptr, false);
    process(fresh, fresh_model, config);
    const InfillSnapshot expected = infill(fresh);
    REQUIRE(incremental.sparse_path_count == expected.sparse_path_count);
    REQUIRE(incremental.sparse_surface_count == expected.sparse_surface_count);
    REQUIRE(incremental.void_surface_count == expected.void_surface_count);
}

TEST_CASE("Thick internal bridges change deposited bridge flow", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](bool thick) {
        DynamicPrintConfig config = bridge_config();
        config.set_deserialize_strict({{"thick_internal_bridges", thick}});
        Print print;
        Model model;
        init_print({internal_bridge_step()}, print, model, config, nullptr, false);
        process(print, model, config);
        return bridges(print);
    };
    const BridgeSnapshot regular = run(false);
    const BridgeSnapshot thick = run(true);
    REQUIRE(regular.surfaces > 0);
    REQUIRE(thick.surfaces > 0);
    REQUIRE(regular.paths > 0);
    REQUIRE(thick.paths > 0);
    REQUIRE(regular.volume > 0.0);
    REQUIRE(thick.volume > 0.0);
    CHECK(thick.volume > regular.volume);
}

TEST_CASE("Internal bridge angle overrides automatic direction", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    const double angle = GENERATE(30.0, 120.0);
    DynamicPrintConfig config = bridge_config();
    config.set_deserialize_strict({{"internal_bridge_angle", angle}});
    Print print;
    Model model;
    init_print({internal_bridge_step()}, print, model, config, nullptr, false);
    process(print, model, config);
    const BridgeSnapshot actual = bridges(print);
    REQUIRE(actual.surfaces > 0);
    REQUIRE(actual.paths > 0);
    REQUIRE_FALSE(actual.angles.empty());
    const double expected = std::fmod(Geometry::deg2rad(angle), PI);
    for (double observed : actual.angles)
        CHECK_THAT(observed, Catch::Matchers::WithinAbs(expected, 1e-3));
}

TEST_CASE("Relative internal bridge angle adds to automatic direction", "[SettingsVariance][SettingsWallBridgeVariance][RelativeBridgeAngle]")
{
    struct Run {
        BridgeSnapshot snapshot;
        std::vector<std::vector<double>> angles_by_layer;
    };
    auto run = [](double angle, bool relative) {
        DynamicPrintConfig config = bridge_config();
        config.set_deserialize_strict({
            {"internal_bridge_angle", angle},
            {"relative_bridge_angle", relative},
        });
        Print print;
        Model model;
        init_print({internal_bridge_step()}, print, model, config, nullptr, false);
        process(print, model, config);
        return Run{bridges(print), bridge_angles_by_layer(print)};
    };

    const Run automatic = run(0., false);
    const Run absolute  = run(30., false);
    const Run relative  = run(30., true);
    REQUIRE(automatic.snapshot.surfaces > 0);
    REQUIRE(automatic.snapshot.paths > 0);
    REQUIRE(absolute.snapshot.paths > 0);
    REQUIRE(relative.snapshot.paths > 0);
    REQUIRE_FALSE(automatic.snapshot.angles.empty());
    REQUIRE_FALSE(absolute.snapshot.angles.empty());
    REQUIRE_FALSE(relative.snapshot.angles.empty());
    REQUIRE(absolute.snapshot.surfaces == automatic.snapshot.surfaces);
    REQUIRE(relative.snapshot.surfaces == automatic.snapshot.surfaces);
    REQUIRE(absolute.angles_by_layer.size() == automatic.angles_by_layer.size());
    REQUIRE(relative.angles_by_layer.size() == automatic.angles_by_layer.size());

    const double offset = Geometry::deg2rad(30.);
    const auto modulo_pi = [](double angle) {
        double result = std::fmod(angle, PI);
        if (result < 0.)
            result += PI;
        return result;
    };
    const auto angle_delta = [&modulo_pi](double lhs, double rhs) {
        const double delta = std::abs(modulo_pi(lhs) - modulo_pi(rhs));
        return std::min(delta, PI - delta);
    };
    const auto sorted_angles = [&modulo_pi](const std::vector<double> &source) {
        std::vector<double> result;
        result.reserve(source.size());
        for (double angle : source)
            result.push_back(modulo_pi(angle));
        std::sort(result.begin(), result.end());
        return result;
    };
    const auto sorted_offset_angles = [&modulo_pi, offset](const std::vector<double> &source) {
        std::vector<double> result;
        result.reserve(source.size());
        for (double angle : source)
            result.push_back(modulo_pi(angle + offset));
        std::sort(result.begin(), result.end());
        return result;
    };
    bool distinct_modes = false;
    for (size_t layer = 0; layer < automatic.angles_by_layer.size(); ++layer) {
        const std::vector<double> base = sorted_angles(automatic.angles_by_layer[layer]);
        const std::vector<double> abs   = sorted_angles(absolute.angles_by_layer[layer]);
        const std::vector<double> rel   = sorted_angles(relative.angles_by_layer[layer]);
        const std::vector<double> expected_rel = sorted_offset_angles(automatic.angles_by_layer[layer]);
        CAPTURE(layer);
        REQUIRE(abs.size() == base.size());
        REQUIRE(rel.size() == base.size());
        REQUIRE(expected_rel.size() == base.size());
        for (size_t i = 0; i < base.size(); ++i) {
            CAPTURE(i, base[i], abs[i], rel[i], expected_rel[i]);
            CHECK(angle_delta(abs[i], offset) < 1e-3);
            CHECK(angle_delta(rel[i], expected_rel[i]) < 1e-3);
            distinct_modes |= angle_delta(rel[i], abs[i]) > 0.01;
        }
    }
    // The fixture's automatic bridge direction is intentionally non-zero, so
    // absolute and relative modes exercise different semantics.
    REQUIRE(distinct_modes);
}

TEST_CASE("Overhang detection partitions perimeter roles", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    const bool detect = GENERATE(false, true);
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({{"detect_overhang_wall", detect}});
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{mesh(TestMesh::overhang)}, print, model, config, nullptr, false);
    process(print, model, config);
    size_t overhang_paths = 0;
    size_t perimeter_paths = 0;
    for (const Layer *layer : print.objects().front()->layers())
        for (const LayerRegion *region : layer->regions())
            for_each_extrusion_path(region->perimeters, [&](const ExtrusionPath &path) {
                if (path.role() == erOverhangPerimeter)
                    ++overhang_paths;
                if (is_perimeter(path.role()))
                    ++perimeter_paths;
            });
    REQUIRE(perimeter_paths > 0);
    if (detect)
        CHECK(overhang_paths > 0);
    else
        CHECK(overhang_paths == 0);
}

TEST_CASE("Bridge density changes external bridge deposition", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](double density) {
        DynamicPrintConfig config = wall_infill_config();
        config.set_deserialize_strict({
            {"bridge_density", density},
            {"thick_bridges", false},
            {"bridge_flow", 0.75},
            {"bridge_no_support", true},
            {"support_remove_small_overhang", false},
            {"top_shell_layers", 2},
            {"bottom_shell_layers", 2},
        });
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{external_bridge_fixture()}, print, model, config, nullptr, false);
        process(print, model, config);
        return external_bridges(print);
    };
    const ExternalBridgeSnapshot sparse = run(50.0);
    const ExternalBridgeSnapshot full = run(100.0);
    REQUIRE(sparse.paths > 0);
    REQUIRE(full.paths > 0);
    REQUIRE(sparse.volume > 0.0);
    REQUIRE(full.volume > 0.0);
    CHECK(full.volume > sparse.volume);
}

TEST_CASE("Bridge flow changes external bridge deposition", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](double flow) {
        DynamicPrintConfig config = wall_infill_config();
        config.set_deserialize_strict({
            {"bridge_density", 100.0},
            {"bridge_flow", flow},
            {"bridge_no_support", true},
            {"support_remove_small_overhang", false},
            {"top_shell_layers", 2},
            {"bottom_shell_layers", 2},
        });
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{external_bridge_fixture()}, print, model, config, nullptr, false);
        process(print, model, config);
        return external_bridges(print);
    };
    const ExternalBridgeSnapshot low = run(0.5);
    const ExternalBridgeSnapshot nominal = run(1.0);
    REQUIRE(low.paths > 0);
    REQUIRE(nominal.paths > 0);
    REQUIRE(low.volume > 0.0);
    REQUIRE(nominal.volume > 0.0);
    // Flow changes width while bridge spacing compensates, so total deposited
    // volume is expected to change but is not monotonic in this fixture.
    CHECK(std::abs(low.volume - nominal.volume) > 0.001);
}

TEST_CASE("Internal bridge density changes deposited bridge volume", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](double density) {
        DynamicPrintConfig config = bridge_config();
        config.set_deserialize_strict({{"internal_bridge_density", density}});
        Print print;
        Model model;
        init_print({internal_bridge_step()}, print, model, config, nullptr, false);
        process(print, model, config);
        return bridges(print);
    };
    const BridgeSnapshot sparse = run(25.0);
    const BridgeSnapshot full = run(100.0);
    REQUIRE(sparse.paths > 0);
    REQUIRE(full.paths > 0);
    REQUIRE(sparse.volume > 0.0);
    REQUIRE(full.volume > 0.0);
    CHECK(full.volume > sparse.volume);
}

TEST_CASE("Bridge line width changes external bridge deposition", "[SettingsVariance][SettingsWallBridgeVariance]")
{
    auto run = [](double width) {
        DynamicPrintConfig config = wall_infill_config();
        config.set_deserialize_strict({
            {"bridge_density", 100.0},
            {"bridge_line_width", width},
            {"bridge_flow", 0.75},
            {"bridge_no_support", true},
            {"support_remove_small_overhang", false},
            {"top_shell_layers", 2},
            {"bottom_shell_layers", 2},
        });
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{external_bridge_fixture()}, print, model, config, nullptr, false);
        process(print, model, config);
        return external_bridges(print);
    };
    const ExternalBridgeSnapshot narrow = run(0.25);
    const ExternalBridgeSnapshot wide = run(0.35);
    REQUIRE(narrow.paths > 0);
    REQUIRE(wide.paths > 0);
    REQUIRE(narrow.volume > 0.0);
    REQUIRE(wide.volume > 0.0);
    CHECK(std::abs(narrow.volume - wide.volume) > 0.001);
}

namespace sparse_infill_density_incremental_contracts {

namespace {

struct ExternalPerimeterGeometry
{
    size_t path_count = 0;
    double total_length = 0.0;
    double min_x = std::numeric_limits<double>::max();
    double max_x = std::numeric_limits<double>::lowest();
    double min_y = std::numeric_limits<double>::max();
    double max_y = std::numeric_limits<double>::lowest();
};

struct SparseDensitySnapshot
{
    WallSnapshot walls;
    InfillSnapshot infill;
    std::vector<double> sparse_length_by_layer;
    std::vector<ExternalPerimeterGeometry> external_by_layer;
};

TriangleMesh sparse_density_cube()
{
    TriangleMesh mesh = cube(12.0);
    mesh.scale(Vec3f(1.0f, 1.0f, 0.8f / 12.0f));
    return mesh;
}

DynamicPrintConfig sparse_density_config(const char *density)
{
    DynamicPrintConfig config = wall_infill_config();
    config.set_deserialize_strict({
        { "alternate_extra_wall", true },
        { "sparse_infill_density", density },
        { "sparse_infill_pattern", "rectilinear" },
        { "wall_sequence", "inner wall/outer wall" },
        { "minimum_sparse_infill_area", 0.0 },
        { "nozzle_diameter", "0.4" },
        { "outer_wall_line_width", 0.4 },
        { "inner_wall_line_width", 0.4 },
        { "sparse_infill_line_width", 0.4 },
        { "infill_anchor", 0 },
        { "infill_anchor_max", 0 },
    });
    return config;
}

void add_external_geometry(ExternalPerimeterGeometry &geometry,
                           const ExtrusionPath &path)
{
    ++geometry.path_count;
    geometry.total_length += unscale<double>(path.length());
    REQUIRE(path.polyline.points.size() >= 2);
    for (const Point3 &point : path.polyline.points) {
        const double x = unscale<double>(point.x());
        const double y = unscale<double>(point.y());
        REQUIRE(std::isfinite(x));
        REQUIRE(std::isfinite(y));
        geometry.min_x = std::min(geometry.min_x, x);
        geometry.max_x = std::max(geometry.max_x, x);
        geometry.min_y = std::min(geometry.min_y, y);
        geometry.max_y = std::max(geometry.max_y, y);
    }
}

void require_external_geometry_equal(const ExternalPerimeterGeometry &actual,
                                     const ExternalPerimeterGeometry &expected)
{
    REQUIRE(actual.path_count == expected.path_count);
    REQUIRE(actual.path_count == 1);
    CHECK_THAT(actual.total_length, Catch::Matchers::WithinAbs(expected.total_length, 1e-6));
    CHECK_THAT(actual.min_x, Catch::Matchers::WithinAbs(expected.min_x, 1e-6));
    CHECK_THAT(actual.max_x, Catch::Matchers::WithinAbs(expected.max_x, 1e-6));
    CHECK_THAT(actual.min_y, Catch::Matchers::WithinAbs(expected.min_y, 1e-6));
    CHECK_THAT(actual.max_y, Catch::Matchers::WithinAbs(expected.max_y, 1e-6));
}

SparseDensitySnapshot sparse_density_snapshot(const Print &print)
{
    SparseDensitySnapshot result{walls(print), infill(print), {}, {}};
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() == 4);
    result.sparse_length_by_layer.reserve(object.layers().size());
    result.external_by_layer.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        double sparse_length = 0.0;
        ExternalPerimeterGeometry external;
        for (const LayerRegion *region : layer->regions()) {
            for_each_extrusion_path(region->fills, [&sparse_length](const ExtrusionPath &path) {
                if (path.role() == erInternalInfill)
                    sparse_length += unscale<double>(path.length());
            });
            for_each_extrusion_path(region->perimeters, [&external](const ExtrusionPath &path) {
                if (path.role() == erExternalPerimeter)
                    add_external_geometry(external, path);
            });
        }
        REQUIRE(external.path_count == 1);
        REQUIRE(std::isfinite(external.total_length));
        result.sparse_length_by_layer.push_back(sparse_length);
        result.external_by_layer.push_back(external);
    }
    return result;
}

SparseDensitySnapshot fresh_sparse_density_snapshot(const char *density)
{
    DynamicPrintConfig config = sparse_density_config(density);
    Print print;
    Model model;
    init_print({sparse_density_cube()}, print, model, config, nullptr, false);
    process(print, model, config);
    return sparse_density_snapshot(print);
}

void require_same_sparse_density_output(const SparseDensitySnapshot &actual,
                                        const SparseDensitySnapshot &expected)
{
    REQUIRE(actual.walls.layer_roles == expected.walls.layer_roles);
    REQUIRE(actual.walls.perimeter_items == expected.walls.perimeter_items);
    REQUIRE(actual.walls.external_perimeter_indices == expected.walls.external_perimeter_indices);
    REQUIRE(actual.infill.sparse_path_count == expected.infill.sparse_path_count);
    REQUIRE(actual.infill.sparse_surface_count == expected.infill.sparse_surface_count);
    REQUIRE(actual.infill.void_surface_count == expected.infill.void_surface_count);
    REQUIRE(actual.infill.sparse_path_heights.size() == expected.infill.sparse_path_heights.size());
    for (size_t index = 0; index < expected.infill.sparse_path_heights.size(); ++index)
        CHECK_THAT(actual.infill.sparse_path_heights[index],
                   Catch::Matchers::WithinAbs(expected.infill.sparse_path_heights[index], 1e-6));
    REQUIRE(actual.sparse_length_by_layer.size() == expected.sparse_length_by_layer.size());
    REQUIRE(actual.external_by_layer.size() == expected.external_by_layer.size());
    for (size_t layer = 0; layer < expected.sparse_length_by_layer.size(); ++layer) {
        CHECK_THAT(actual.sparse_length_by_layer[layer],
                   Catch::Matchers::WithinAbs(expected.sparse_length_by_layer[layer], 1e-6));
        require_external_geometry_equal(actual.external_by_layer[layer], expected.external_by_layer[layer]);
    }
}

double total_sparse_length(const SparseDensitySnapshot &snapshot)
{
    return std::accumulate(snapshot.sparse_length_by_layer.begin(),
                           snapshot.sparse_length_by_layer.end(), 0.0);
}

void require_zero_boundary_perimeter_delta(const SparseDensitySnapshot &normal,
                                           const SparseDensitySnapshot &zero)
{
    REQUIRE(normal.walls.perimeter_items.size() == zero.walls.perimeter_items.size());
    REQUIRE(normal.walls.layer_roles.size() == zero.walls.layer_roles.size());
    REQUIRE(normal.external_by_layer.size() == zero.external_by_layer.size());
    bool saw_odd_change = false;
    for (size_t layer = 0; layer < normal.walls.perimeter_items.size(); ++layer) {
        REQUIRE(std::count(normal.walls.layer_roles[layer].begin(),
                           normal.walls.layer_roles[layer].end(), erExternalPerimeter) == 1);
        REQUIRE(std::count(zero.walls.layer_roles[layer].begin(),
                           zero.walls.layer_roles[layer].end(), erExternalPerimeter) == 1);
        // Compare the external geometry by aggregate bounds/length.  The
        // extra inner wall can change flattened role ordering, so its index
        // is intentionally not used as a cross-density oracle.
        require_external_geometry_equal(normal.external_by_layer[layer], zero.external_by_layer[layer]);
        if (layer % 2 == 1) {
            CHECK(normal.walls.perimeter_items[layer] > zero.walls.perimeter_items[layer]);
            saw_odd_change = saw_odd_change ||
                             normal.walls.perimeter_items[layer] != zero.walls.perimeter_items[layer];
        } else {
            CHECK(normal.walls.perimeter_items[layer] == zero.walls.perimeter_items[layer]);
            CHECK(normal.walls.layer_roles[layer] == zero.walls.layer_roles[layer]);
        }
    }
    REQUIRE(saw_odd_change);
}

} // namespace

TEST_CASE("Sparse density transitions preserve fresh perimeters and infill",
          "[SettingsVariance][SettingsReslice][SparseInfillDensity]")
{
    const DynamicPrintConfig density20 = sparse_density_config("20%");
    const DynamicPrintConfig density0 = sparse_density_config("0%");
    const DynamicPrintConfig density40 = sparse_density_config("40%");

    // One mutable Print exercises 20->0->20->40. Three independent fresh
    // snapshots are captured once and reused as immutable transition oracles.
    Print incremental_print;
    Model incremental_model;
    init_print({sparse_density_cube()}, incremental_print, incremental_model, density20, nullptr, false);
    process(incremental_print, incremental_model, density20);
    const SparseDensitySnapshot baseline20 = sparse_density_snapshot(incremental_print);

    const SparseDensitySnapshot fresh0 = fresh_sparse_density_snapshot("0%");
    const SparseDensitySnapshot fresh20 = fresh_sparse_density_snapshot("20%");
    const SparseDensitySnapshot fresh40 = fresh_sparse_density_snapshot("40%");
    require_same_sparse_density_output(baseline20, fresh20);
    REQUIRE(total_sparse_length(baseline20) > 1e-6);
    REQUIRE(std::all_of(fresh0.infill.sparse_path_count.begin(),
                        fresh0.infill.sparse_path_count.end(),
                        [](size_t count) { return count == 0; }));
    CHECK_THAT(total_sparse_length(fresh0), Catch::Matchers::WithinAbs(0.0, 1e-6));
    REQUIRE(total_sparse_length(fresh40) > 1e-6);

    process(incremental_print, incremental_model, density0);
    const SparseDensitySnapshot incremental0 = sparse_density_snapshot(incremental_print);
    require_same_sparse_density_output(incremental0, fresh0);
    require_zero_boundary_perimeter_delta(fresh20, fresh0);

    process(incremental_print, incremental_model, density20);
    const SparseDensitySnapshot incremental20 = sparse_density_snapshot(incremental_print);
    require_same_sparse_density_output(incremental20, fresh20);
    require_zero_boundary_perimeter_delta(incremental20, incremental0);

    process(incremental_print, incremental_model, density40);
    const SparseDensitySnapshot incremental40 = sparse_density_snapshot(incremental_print);
    require_same_sparse_density_output(incremental40, fresh40);
    REQUIRE(incremental40.walls.layer_roles == baseline20.walls.layer_roles);
    REQUIRE(incremental40.walls.perimeter_items == baseline20.walls.perimeter_items);
    REQUIRE(std::abs(total_sparse_length(incremental40) - total_sparse_length(baseline20)) > 1e-3);
}

} // namespace sparse_infill_density_incremental_contracts

} // namespace wall_bridge_contracts

namespace settings_family_contracts {
namespace {

DynamicPrintConfig family_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_loops", 2},
        {"top_shell_layers", 2},
        {"bottom_shell_layers", 2},
        {"top_shell_thickness", 0},
        {"bottom_shell_thickness", 0},
        {"sparse_infill_density", "20%"},
        {"gcode_comments", true},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layer_time", 0},
        {"slow_down_min_speed", 1},
        {"fan_cooling_layer_time", 0},
        {"filament_max_volumetric_speed", "200"},
        {"filament_adaptive_volumetric_speed", false},
        {"machine_max_acceleration_extruding", "20000"},
        {"machine_max_acceleration_retracting", "20000"},
        {"machine_max_acceleration_travel", "20000"},
        {"machine_max_jerk_x", "20000"},
        {"machine_max_jerk_y", "20000"},
        {"machine_max_jerk_z", "20000"},
        {"machine_max_jerk_e", "20000"},
    });
    return config;
}

std::string export_mesh(const TriangleMesh& mesh, const DynamicPrintConfig& config)
{
    Model local_model;
    Print local_print;
    init_print(std::vector<TriangleMesh>{mesh}, local_print, local_model, config, nullptr, false);
    const auto validation = local_print.validate();
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(local_print.process());
    REQUIRE_FALSE(local_print.objects().empty());
    std::string output = gcode(local_print);
    REQUIRE_FALSE(output.empty());
    return output;
}

struct SupportMotion
{
    std::vector<double> base_feeds;
    std::vector<double> interface_feeds;
};

SupportMotion support_motion(const std::string& output, const DynamicPrintConfig& config)
{
    SupportMotion result;
    GCodeReader reader;
    reader.apply_config(config);
    std::string support_role;
    reader.parse_buffer(output, [&result, &support_role](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos) {
            if (comment.find("Support interface") != std::string_view::npos ||
                comment.find("support material interface") != std::string_view::npos)
                support_role = "interface";
            else if (comment.find("TYPE:Support") != std::string_view::npos || comment.find("support material") != std::string_view::npos)
                support_role = "base";
            else
                support_role.clear();
        }
        if (!line.extruding(self))
            return;
        if (support_role == "interface")
            result.interface_feeds.push_back(line.new_F(self));
        else if (support_role == "base")
            result.base_feeds.push_back(line.new_F(self));
    });
    return result;
}

TriangleMesh internal_bridge_fixture()
{
    TriangleMesh mesh  = make_cube(30, 24, 3);
    TriangleMesh tower = make_cube(14, 10, 1);
    tower.translate(8, 7, 3);
    mesh.merge(tower);
    return mesh;
}

struct InternalBridgeMotion
{
    double e     = 0.;
    double xy    = 0.;
    size_t moves = 0;
};

InternalBridgeMotion internal_bridge_motion(const std::string& output, const DynamicPrintConfig& config)
{
    InternalBridgeMotion result;
    GCodeReader reader;
    reader.apply_config(config);
    bool internal_bridge = false;
    reader.parse_buffer(output, [&result, &internal_bridge](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            internal_bridge = comment.find("Internal Bridge") != std::string_view::npos ||
                              comment.find("Internal bridge") != std::string_view::npos;
        if (!internal_bridge || !line.extruding(self))
            return;
        result.e += line.dist_E(self);
        result.xy += line.dist_XY(self);
        ++result.moves;
    });
    return result;
}

struct IroningMotion
{
    size_t moves = 0;
    double e     = 0.;
    double xy    = 0.;
    std::set<int> feeds;
};

IroningMotion ironing_motion(const std::string& output, const DynamicPrintConfig& config)
{
    IroningMotion result;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&result](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        if (!line.extruding(self) || line.comment().find("ironing") == std::string_view::npos)
            return;
        result.e += line.dist_E(self);
        result.xy += line.dist_XY(self);
        result.feeds.insert(int(std::lround(line.new_F(self))));
        ++result.moves;
    });
    return result;
}

struct IroningShape
{
    size_t paths  = 0;
    size_t closed = 0;
    double length = 0.;
};

IroningShape ironing_shape(const DynamicPrintConfig& config)
{
    Model model;
    Print print;
    init_print({cube(20.)}, print, model, config, nullptr, false);
    const auto validation = print.validate();
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());
    IroningShape result;
    for (const Layer* layer : print.objects().front()->layers())
        for (const LayerRegion* region : layer->regions())
            for_each_extrusion_path(region->fills, [&result](const ExtrusionPath& path) {
                if (path.role() != erIroning)
                    return;
                ++result.paths;
                result.length += unscale<double>(path.length());
                if (path.first_point3() == path.last_point3())
                    ++result.closed;
            });
    return result;
}

DynamicPrintConfig ironing_config()
{
    DynamicPrintConfig config = family_config();
    config.set_deserialize_strict({
        {"top_shell_layers", 2},
        {"bottom_shell_layers", 1},
        {"top_surface_pattern", "monotonic"},
        {"ironing_type", "top"},
        {"ironing_pattern", "rectilinear"},
        {"ironing_flow", "10%"},
        {"ironing_speed", 20},
        {"ironing_spacing", 0.2},
        {"ironing_inset", 0},
    });
    return config;
}

constexpr double caged_layer_height     = 0.2;
constexpr double caged_wall_width       = 0.42;
constexpr double caged_outer_wall_speed = 200.;
constexpr double caged_slope_face_sum   = 15.878796;
constexpr double caged_slope_z_min      = 5.711731;
constexpr double caged_slope_z_max      = 15.878796;
constexpr double caged_span_z_min       = caged_slope_z_min + caged_layer_height;
constexpr double caged_slope_wall_sum   = caged_slope_face_sum + 0.5 * caged_layer_height + 0.5 * caged_wall_width;
constexpr double caged_slope_span       = 34.914257 - 5.0859995;
constexpr double caged_wall_tolerance   = 0.18;

TriangleMesh caged_overhang_mesh()
{
    return TriangleMesh(
        {
            {5.0859987f, 10.167065f, 5.711731f},
            {34.914257f, 10.167065f, 5.711731f},
            {34.914257f, 0.f, 15.878796f},
            {5.0859995f, 0.f, 15.878796f},
            {0.f, 0.f, 0.f},
            {0.f, 0.f, 20.f},
            {0.f, 20.f, 20.f},
            {0.f, 20.f, 0.f},
            {40.f, 20.f, 20.f},
            {40.f, 20.f, 0.f},
            {40.f, 0.f, 20.f},
            {40.f, 0.f, 0.f},
            {34.914257f, 0.f, 0.f},
            {5.0859995f, 0.f, 0.f},
            {34.914257f, 10.167065f, 0.f},
            {5.0859995f, 10.167065f, 0.f},
        },
        {
            {0, 1, 2},  {0, 2, 3},   {4, 5, 6},  {4, 6, 7},   {7, 6, 8},   {7, 8, 9},   {9, 8, 10}, {9, 10, 11}, {12, 11, 10}, {5, 4, 13},
            {5, 13, 3}, {2, 12, 10}, {5, 3, 2},  {10, 5, 2},  {9, 11, 12}, {9, 12, 14}, {13, 4, 7}, {9, 14, 15}, {15, 13, 7},  {7, 9, 15},
            {8, 6, 5},  {8, 5, 10},  {14, 1, 0}, {14, 0, 15}, {2, 1, 14},  {2, 14, 12}, {15, 0, 3}, {15, 3, 13},
        });
}

TriangleMesh severe_caged_overhang_mesh()
{
    // A shallower face complements the first caged slope for speed-gate checks.
    return TriangleMesh(
        {
            {5.0859987f, 10.f, 5.f},
            {34.914257f, 10.f, 5.f},
            {34.914257f, 0.f, 10.f},
            {5.0859995f, 0.f, 10.f},
            {0.f, 0.f, 0.f},
            {0.f, 0.f, 20.f},
            {0.f, 20.f, 20.f},
            {0.f, 20.f, 0.f},
            {40.f, 20.f, 20.f},
            {40.f, 20.f, 0.f},
            {40.f, 0.f, 20.f},
            {40.f, 0.f, 0.f},
            {34.914257f, 0.f, 0.f},
            {5.0859995f, 0.f, 0.f},
            {34.914257f, 10.f, 0.f},
            {5.0859995f, 10.f, 0.f},
        },
        {
            {0, 1, 2},  {0, 2, 3},   {4, 5, 6},  {4, 6, 7},   {7, 6, 8},   {7, 8, 9},   {9, 8, 10}, {9, 10, 11}, {12, 11, 10}, {5, 4, 13},
            {5, 13, 3}, {2, 12, 10}, {5, 3, 2},  {10, 5, 2},  {9, 11, 12}, {9, 12, 14}, {13, 4, 7}, {9, 14, 15}, {15, 13, 7},  {7, 9, 15},
            {8, 6, 5},  {8, 5, 10},  {14, 1, 0}, {14, 0, 15}, {2, 1, 14},  {2, 14, 12}, {15, 0, 3}, {15, 3, 13},
        });
}

TriangleMesh adaptive_pa_overhang_mesh()
{
    // Keep both shells disjoint so the exporter must encounter two independent
    // overhang-role islands.  The different slope angles exercise different
    // overhang speed sections, rather than duplicating one sample verbatim.
    TriangleMesh mesh = caged_overhang_mesh();
    TriangleMesh severe = severe_caged_overhang_mesh();
    severe.translate(45.f, 0.f, 0.f);
    mesh.merge(severe);
    return mesh;
}

DynamicPrintConfig overhang_config()
{
    DynamicPrintConfig config = family_config();
    config.set_deserialize_strict({
        {"initial_layer_print_height", caged_layer_height},
        {"layer_height", caged_layer_height},
        {"line_width", caged_wall_width},
        {"outer_wall_line_width", caged_wall_width},
        {"inner_wall_line_width", 0.45},
        {"wall_loops", 2},
        {"wall_generator", "classic"},
        {"wall_sequence", "inner wall/outer wall"},
        {"sparse_infill_density", "15%"},
        {"detect_overhang_wall", true},
        {"enable_overhang_speed", true},
        // Keep this fixture about the overhang-speed gate.  The default bridge-fan
        // path is an independent fallback in GCode.cpp and would otherwise make
        // disabling enable_overhang_speed look like a no-op.
        {"enable_overhang_bridge_fan", false},
        {"slowdown_for_curled_perimeters", true},
        {"zaa_enabled", false},
        {"outer_wall_speed", caged_outer_wall_speed},
        {"inner_wall_speed", 300},
        {"overhang_1_4_speed", 180},
        {"overhang_2_4_speed", 140},
        {"overhang_3_4_speed", 100},
        {"overhang_4_4_speed", 60},
        {"bridge_speed", 50},
        {"slow_down_for_layer_cooling", false},
        {"slow_down_layers", 0},
    });
    return config;
}

std::vector<double> caged_slope_feeds(const std::string& output, const DynamicPrintConfig& config)
{
    std::vector<double> feeds;
    GCodeReader reader;
    reader.apply_config(config);
    bool outer_wall = false;
    reader.parse_buffer(output, [&feeds, &outer_wall](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            outer_wall = comment.find("Outer wall") != std::string_view::npos ||
                         comment.find("External perimeter") != std::string_view::npos;
        if (!outer_wall || !line.extruding(self))
            return;
        const double z = line.new_Z(self);
        if (z <= caged_span_z_min || z >= caged_slope_z_max || line.dist_XY(self) <= 1.0)
            return;
        if (std::abs(self.y() + z - caged_slope_wall_sum) < caged_wall_tolerance &&
            std::abs(line.new_Y(self) + z - caged_slope_wall_sum) < caged_wall_tolerance)
            feeds.push_back(line.new_F(self));
    });
    return feeds;
}

std::vector<double> severe_caged_slope_feeds(const std::string& output, const DynamicPrintConfig& config)
{
    std::vector<double> feeds;
    GCodeReader reader;
    reader.apply_config(config);
    bool outer_wall = false;
    reader.parse_buffer(output, [&feeds, &outer_wall](GCodeReader& self, const GCodeReader::GCodeLine& line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            outer_wall = comment.find("Outer wall") != std::string_view::npos ||
                         comment.find("External perimeter") != std::string_view::npos;
        if (!outer_wall || !line.extruding(self))
            return;
        const double z = line.new_Z(self);
        if (z <= 5.2 || z >= 10.1 || line.dist_XY(self) <= 1.)
            return;
        if (std::abs(self.y() + 2. * z - 20.5) < 0.55 && std::abs(line.new_Y(self) + 2. * z - 20.5) < 0.55)
            feeds.push_back(line.new_F(self));
    });
    return feeds;
}

struct AnchorEvidence
{
    size_t paths                  = 0;
    double length                 = 0.;
    size_t endpoint_contacts      = 0;
    size_t endpoint_samples       = 0;
    double endpoint_distance_sum  = 0.;
    double endpoint_distance_max  = 0.;
    double spacing                = 0.;
};

double point_line_distance(const Point3& point, const Point3& a, const Point3& b)
{
    const double px = point.x(), py = point.y();
    const double ax = a.x(), ay = a.y();
    const double bx = b.x(), by = b.y();
    const double dx = bx - ax, dy = by - ay;
    const double denom = dx * dx + dy * dy;
    const double t     = denom > 0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / denom, 0., 1.) : 0.;
    return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

AnchorEvidence anchor_evidence(const DynamicPrintConfig& config)
{
    Model model;
    Print print;
    init_print({mesh(TestMesh::cube_with_concave_hole)}, print, model, config, nullptr, false);
    const auto validation = print.validate();
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());
    AnchorEvidence result;
    for (const Layer* layer : print.objects().front()->layers()) {
        Polylines perimeter_lines;
        for (const LayerRegion* region : layer->regions())
            region->perimeters.collect_polylines(perimeter_lines);
        for (const LayerRegion* region : layer->regions())
            for_each_extrusion_path(region->fills, [&result, &perimeter_lines, layer, region](const ExtrusionPath& path) {
                if (path.role() != erInternalInfill)
                    return;
                if (layer->id() > 0 && result.spacing <= 0.)
                    result.spacing = region->flow(frInfill).spacing();
                ++result.paths;
                result.length += unscale<double>(path.length());
                for (const Point3& endpoint : {path.first_point3(), path.last_point3()}) {
                    double nearest = std::numeric_limits<double>::max();
                    for (const Polyline& line : perimeter_lines)
                        for (size_t i = 1; i < line.points.size(); ++i)
                            nearest = std::min(nearest,
                                               point_line_distance(endpoint,
                                                                   Point3(line.points[i - 1].x(), line.points[i - 1].y(), endpoint.z()),
                                                                   Point3(line.points[i].x(), line.points[i].y(), endpoint.z())));
                    if (nearest != std::numeric_limits<double>::max()) {
                        const double nearest_mm = unscale<double>(nearest);
                        ++result.endpoint_samples;
                        result.endpoint_distance_sum += nearest_mm;
                        result.endpoint_distance_max = std::max(result.endpoint_distance_max, nearest_mm);
                        if (nearest <= scale_(0.5))
                            ++result.endpoint_contacts;
                    }
                }
            });
    }
    return result;
}

} // namespace

TEST_CASE("Support base and interface speeds reach their labeled support roles", "[SettingsFamilyVariance][SupportRoleMotion]")
{
    DynamicPrintConfig config = family_config();
    config.set_deserialize_strict({
        {"enable_support", true},
        {"support_on_build_plate_only", false},
        {"support_interface_top_layers", 3},
        {"support_interface_bottom_layers", 0},
        {"support_speed", 18},
        {"support_interface_speed", 31},
    });
    const SupportMotion motion = support_motion(export_mesh(mesh(TestMesh::overhang), config), config);
    REQUIRE_FALSE(motion.base_feeds.empty());
    REQUIRE_FALSE(motion.interface_feeds.empty());
    CHECK(std::all_of(motion.base_feeds.begin(), motion.base_feeds.end(), [](double f) { return std::isfinite(f) && f > 0.; }));
    CHECK(std::all_of(motion.interface_feeds.begin(), motion.interface_feeds.end(), [](double f) { return std::isfinite(f) && f > 0.; }));
    CHECK(std::find_if(motion.base_feeds.begin(), motion.base_feeds.end(), [](double f) { return std::abs(f - 1080.) < 1e-3; }) !=
          motion.base_feeds.end());
    CHECK(std::find_if(motion.interface_feeds.begin(), motion.interface_feeds.end(), [](double f) { return std::abs(f - 1860.) < 1e-3; }) !=
          motion.interface_feeds.end());
    CHECK(std::none_of(motion.base_feeds.begin(), motion.base_feeds.end(), [](double f) { return std::abs(f - 1860.) < 1e-3; }));
    CHECK(std::none_of(motion.interface_feeds.begin(), motion.interface_feeds.end(), [](double f) { return std::abs(f - 1080.) < 1e-3; }));
}

TEST_CASE("Internal bridge flow scales extrusion at fixed path length", "[SettingsFamilyVariance][InternalBridgeFlow]")
{
    DynamicPrintConfig base = family_config();
    base.set_deserialize_strict({
        {"sparse_infill_density", "15%"},
        {"internal_bridge_flow", "1"},
        {"top_shell_layers", 3},
        {"bottom_shell_layers", 2},
        {"thick_internal_bridges", false},
    });
    const InternalBridgeMotion nominal = internal_bridge_motion(export_mesh(internal_bridge_fixture(), base), base);
    REQUIRE(nominal.moves > 0);
    REQUIRE(nominal.xy > 0.);

    DynamicPrintConfig changed = base;
    changed.set_deserialize_strict("internal_bridge_flow", "1.35");
    const InternalBridgeMotion scaled = internal_bridge_motion(export_mesh(internal_bridge_fixture(), changed), changed);
    REQUIRE(scaled.moves == nominal.moves);
    REQUIRE_THAT(scaled.xy, Catch::Matchers::WithinRel(nominal.xy, 1e-9));
    REQUIRE_THAT(scaled.e / nominal.e, Catch::Matchers::WithinRel(1.35, 0.03));
}

TEST_CASE("Overhang speed gate and two configured bands affect the caged slope", "[SettingsFamilyVariance][OverhangSpeedBands]")
{
    DynamicPrintConfig enabled                 = overhang_config();
    const std::vector<double> reference        = caged_slope_feeds(export_mesh(caged_overhang_mesh(), enabled), enabled);
    const std::vector<double> severe_reference = severe_caged_slope_feeds(export_mesh(severe_caged_overhang_mesh(), enabled), enabled);
    REQUIRE_FALSE(reference.empty());
    REQUIRE_FALSE(severe_reference.empty());
    REQUIRE(std::all_of(reference.begin(), reference.end(), [](double f) { return std::isfinite(f) && f > 0.; }));
    REQUIRE(*std::min_element(reference.begin(), reference.end()) < caged_outer_wall_speed * 60.);

    DynamicPrintConfig disabled = enabled;
    disabled.set_deserialize_strict("enable_overhang_speed", "0");
    const std::vector<double> ungated = caged_slope_feeds(export_mesh(caged_overhang_mesh(), disabled), disabled);
    REQUIRE_FALSE(ungated.empty());
    CHECK(std::all_of(ungated.begin(), ungated.end(), [](double f) { return std::abs(f - caged_outer_wall_speed * 60.) < 1e-3; }));

    // This second disabled case exercises the bridge-fan fallback's explicit
    // restoration branch.  The primary gate fixture disables that fallback so
    // the speed estimator itself remains independently observable.
    DynamicPrintConfig fan_disabled = enabled;
    fan_disabled.set_deserialize_strict({
        {"enable_overhang_bridge_fan", true},
        {"enable_overhang_speed", false},
    });
    const std::vector<double> fan_ungated = caged_slope_feeds(export_mesh(caged_overhang_mesh(), fan_disabled), fan_disabled);
    REQUIRE_FALSE(fan_ungated.empty());
    CHECK(std::all_of(fan_ungated.begin(), fan_ungated.end(), [](double f) { return std::abs(f - caged_outer_wall_speed * 60.) < 1e-3; }));

    // The two fixtures cover the first and second bands.  The steeper slope is
    // still useful for gate coverage, but it does not reliably produce a
    // segment in the third or fourth overlap interval, so those settings are
    // deliberately left outside this pipeline claim.
    for (const char* key : {"overhang_1_4_speed", "overhang_2_4_speed"}) {
        DynamicPrintConfig changed = enabled;
        changed.set_deserialize_strict(std::string(key), "25");
        const std::vector<double> feeds        = caged_slope_feeds(export_mesh(caged_overhang_mesh(), changed), changed);
        const std::vector<double> severe_feeds = severe_caged_slope_feeds(export_mesh(severe_caged_overhang_mesh(), changed), changed);
        REQUIRE_FALSE(feeds.empty());
        REQUIRE_FALSE(severe_feeds.empty());
        const bool normal_diff = feeds.size() != reference.size() ||
                                 std::inner_product(feeds.begin(), feeds.begin() + std::min(feeds.size(), reference.size()),
                                                    reference.begin(), 0., std::plus<>(),
                                                    [](double a, double b) { return std::abs(a - b) > 1e-3 ? 1. : 0.; }) > 0.;
        const bool severe_diff = severe_feeds.size() != severe_reference.size() ||
                                 std::inner_product(severe_feeds.begin(),
                                                    severe_feeds.begin() + std::min(severe_feeds.size(), severe_reference.size()),
                                                    severe_reference.begin(), 0., std::plus<>(),
                                                    [](double a, double b) { return std::abs(a - b) > 1e-3 ? 1. : 0.; }) > 0.;
        const bool emitted_feeds_changed = normal_diff || severe_diff;
        CHECK(emitted_feeds_changed);
        CHECK(std::any_of(feeds.begin(), feeds.end(), [](double f) { return f < caged_outer_wall_speed * 60. - 1e-3; }));
    }
}

TEST_CASE("Ironing settings control path topology extrusion and speed", "[SettingsFamilyVariance][IroningMotion]")
{
    DynamicPrintConfig rectilinear  = ironing_config();
    const IroningShape rect_shape   = ironing_shape(rectilinear);
    const IroningMotion rect_motion = ironing_motion(export_mesh(cube(20.), rectilinear), rectilinear);
    REQUIRE(rect_shape.paths > 0);
    REQUIRE(rect_motion.moves > 0);
    CHECK(rect_motion.feeds == std::set<int>({1200}));

    DynamicPrintConfig no_ironing = rectilinear;
    no_ironing.set_deserialize_strict("ironing_type", "no ironing");
    const IroningMotion absent = ironing_motion(export_mesh(cube(20.), no_ironing), no_ironing);
    CHECK(absent.moves == 0);

    DynamicPrintConfig concentric = rectilinear;
    concentric.set_deserialize_strict("ironing_pattern", "concentric");
    const IroningShape concentric_shape = ironing_shape(concentric);
    REQUIRE(concentric_shape.paths > 0);
    const bool topology_changed = concentric_shape.closed > rect_shape.closed || concentric_shape.paths != rect_shape.paths;
    CHECK(topology_changed);

    DynamicPrintConfig flow = rectilinear;
    flow.set_deserialize_strict("ironing_flow", "20%");
    const IroningMotion doubled = ironing_motion(export_mesh(cube(20.), flow), flow);
    REQUIRE(doubled.moves > 0);
    CHECK_THAT(doubled.e / rect_motion.e, Catch::Matchers::WithinRel(2., 0.08));
    CHECK_THAT(doubled.xy, Catch::Matchers::WithinRel(rect_motion.xy, 0.02));

    DynamicPrintConfig speed = rectilinear;
    speed.set_deserialize_strict("ironing_speed", "35");
    const IroningMotion faster = ironing_motion(export_mesh(cube(20.), speed), speed);
    REQUIRE(faster.moves > 0);
    CHECK(faster.feeds == std::set<int>({2100}));
}

TEST_CASE("Filament ironing overrides select the active top-surface filament", "[SettingsFamilyVariance][IroningMotion]")
{
    DynamicPrintConfig config  = multifilament_config(2, {
                                                            {"layer_height", 0.2},
                                                            {"initial_layer_print_height", 0.2},
                                                            {"wall_loops", 2},
                                                            {"top_shell_layers", 2},
                                                            {"bottom_shell_layers", 1},
                                                            {"gcode_comments", true},
                                                            {"machine_start_gcode", ""},
                                                            {"machine_end_gcode", ""},
                                                            {"layer_change_gcode", "G92 E0\n"},
                                                            {"skirt_loops", 0},
                                                            {"brim_type", "no_brim"},
                                                            {"raft_layers", 0},
                                                            {"slow_down_for_layer_cooling", false},
                                                            {"filament_max_volumetric_speed", "200"},
                                                            {"filament_adaptive_volumetric_speed", false},
                                                            {"top_surface_pattern", "monotonic"},
                                                            {"ironing_type", "top"},
                                                            {"ironing_pattern", "rectilinear"},
                                                            {"ironing_flow", "10%"},
                                                            {"ironing_speed", 20},
                                                            {"top_surface_filament_id", 1},
                                                            {"filament_ironing_flow", "25%,10%"},
                                                            {"filament_ironing_speed", "35,20"},
                                                            {"ironing_spacing", 0.2},
                                                            {"ironing_inset", 0},
                                                        });
    const IroningMotion motion = ironing_motion(export_mesh(cube(20.), config), config);
    REQUIRE(motion.moves > 0);
    CHECK(motion.feeds == std::set<int>({2100}));
    CHECK(motion.e > 0.);

    DynamicPrintConfig flow_baseline = config;
    flow_baseline.set_deserialize_strict("filament_ironing_flow", "10%,10%");
    const IroningMotion baseline = ironing_motion(export_mesh(cube(20.), flow_baseline), flow_baseline);
    REQUIRE(baseline.moves == motion.moves);
    CHECK_THAT(motion.e / baseline.e, Catch::Matchers::WithinRel(2.5, 0.08));
}

TEST_CASE("Infill anchor maximum enables connected sparse geometry", "[SettingsFamilyVariance][InfillAnchor]")
{
    DynamicPrintConfig no_anchor = family_config();
    no_anchor.set_deserialize_strict({
        {"sparse_infill_pattern", "rectilinear"},
        {"sparse_infill_density", "35%"},
        {"infill_anchor", 0},
        {"infill_anchor_max", 0},
    });
    const AnchorEvidence disabled = anchor_evidence(no_anchor);
    REQUIRE(disabled.paths > 0);

    DynamicPrintConfig anchored = no_anchor;
    // Use a nonzero requested anchor together with a large maximum.  The
    // bounded output contract below is intentionally attributed to the maximum
    // (the separate anchor-length mutation remains an audit survivor).
    anchored.set_deserialize_strict({{"infill_anchor", 4}, {"infill_anchor_max", 1000}});
    const AnchorEvidence enabled = anchor_evidence(anchored);
    REQUIRE(enabled.paths > 0);
    CHECK(enabled.length > disabled.length + 0.1);
    CHECK(enabled.endpoint_contacts < disabled.endpoint_contacts);
}

// Keep short connectors out of the principal per-layer ironing direction.
std::vector<int> ironing_layer_directions(const std::string &output, const DynamicPrintConfig &config)
{
    std::map<size_t, std::pair<double, int>> longest;
    size_t layer = 0;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&longest, &layer](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos)
            ++layer;
        if (line.comment().find("ironing") == std::string_view::npos)
            return;
        if (!line.extruding(self))
            return;
        const double dx = line.dist_X(self);
        const double dy = line.dist_Y(self);
        const double length = std::hypot(dx, dy);
        if (!std::isfinite(length) || length <= 1.0)
            return;
        double degrees = std::fmod(Geometry::rad2deg(std::atan2(dy, dx)), 180.0);
        if (degrees < 0.0)
            degrees += 180.0;
        const int direction = int(std::lround(degrees)) % 180;
        auto &best = longest[layer];
        if (length > best.first)
            best = {length, direction};
    });
    std::vector<int> result;
    result.reserve(longest.size());
    for (const auto &[layer_id, direction] : longest)
        result.push_back(direction.second);
    return result;
}

TEST_CASE("Fixed ironing angle controls inherited solid direction and layer alternation",
          "[SettingsFamilyVariance][IroningAngleFixed]")
{
    auto make_config = [](bool fixed) {
        DynamicPrintConfig config = family_config();
        config.set_deserialize_strict({
            {"wall_loops", 1}, {"top_shell_layers", 1}, {"bottom_shell_layers", 1},
            {"sparse_infill_density", "100%"}, {"solid_infill_direction", 45.0},
            {"solid_infill_rotate_template", ""}, {"top_layer_direction", -1.0},
            {"ironing_type", "solid"}, {"ironing_pattern", "rectilinear"},
            // Fixed mode uses the configured absolute direction.
            {"ironing_angle", fixed ? 90.0 : 0.0}, {"ironing_angle_fixed", fixed},
            {"ironing_flow", "10%"}, {"ironing_spacing", 0.2}, {"ironing_inset", 0.0},
        });
        return config;
    };
    const DynamicPrintConfig relative_config = make_config(false);
    const DynamicPrintConfig fixed_config = make_config(true);
    REQUIRE(relative_config.opt_string("solid_infill_rotate_template").empty());
    REQUIRE(fixed_config.opt_string("solid_infill_rotate_template").empty());

    const std::string relative_output = export_mesh(cube(4.), relative_config);
    const std::string fixed_output = export_mesh(cube(4.), fixed_config);
    const IroningMotion relative_motion = ironing_motion(relative_output, relative_config);
    const IroningMotion fixed_motion = ironing_motion(fixed_output, fixed_config);
    const std::vector<int> relative_layers = ironing_layer_directions(relative_output, relative_config);
    const std::vector<int> fixed_layers = ironing_layer_directions(fixed_output, fixed_config);
    REQUIRE(relative_motion.moves > 4);
    REQUIRE(fixed_motion.moves > 4);
    REQUIRE(relative_motion.e > 0.0);
    REQUIRE(fixed_motion.e > 0.0);
    REQUIRE(relative_layers.size() > 2);
    REQUIRE(fixed_layers.size() == relative_layers.size());

    const auto near = [](int actual, int expected) {
        int delta = std::abs(actual - expected);
        if (delta > 90)
            delta = 180 - delta;
        return delta <= 5;
    };
    bool relative_has_45 = false;
    bool relative_has_135 = false;
    for (const int direction : relative_layers) {
        CAPTURE(direction);
        const bool at_45 = near(direction, 45);
        const bool at_135 = near(direction, 135);
        const bool expected_direction = at_45 || at_135;
        CHECK(expected_direction);
        relative_has_45 = relative_has_45 || at_45;
        relative_has_135 = relative_has_135 || at_135;
    }
    for (const int direction : fixed_layers) {
        CAPTURE(direction);
        CHECK(near(direction, 90));
    }
    CHECK(relative_has_45);
    CHECK(relative_has_135);
}


struct AdaptivePASample
{
    double mm3_per_mm = 0.;
    double feed = 0.;
    double actual_pa = std::numeric_limits<double>::quiet_NaN();
};

DynamicPrintConfig adaptive_pa_output_config(bool enabled)
{
    DynamicPrintConfig config = overhang_config();
    config.set_deserialize_strict({
        {"gcode_flavor", "klipper"},
        {"gcode_comments", true},
        {"enable_arc_fitting", false},
        {"enable_pressure_advance", enabled},
        {"adaptive_pressure_advance", enabled},
        {"adaptive_pressure_advance_overhangs", enabled},
        {"adaptive_pressure_advance_bridges", 0.0},
        // Keep distinct overhang bands separated enough for the strict PA-spread
        // oracle: at this fixture's ~0.0754 mm3/mm, 50 mm/s is ~0.0059 PA.
        {"overhang_1_4_speed", 190},
        {"overhang_2_4_speed", 140},
        {"overhang_3_4_speed", 90},
        {"overhang_4_4_speed", 20},
        {"pressure_advance", 0.03},
        {"wipe", false},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"enable_support", false},
        {"enable_prime_tower", false},
    });
    // Equal acceleration rows make the two-knot flow model a literal linear
    // function while still producing materially different emitted PA values.
    config.option<ConfigOptionStrings>("adaptive_pressure_advance_model")->values = {
        "0.020,1,1000\n0.800,500,1000\n0.020,1,100000\n0.800,500,100000"
    };
    return config;
}

std::vector<AdaptivePASample> adaptive_pa_eligible_samples(const std::string &output, const DynamicPrintConfig &config)
{
    std::vector<AdaptivePASample> samples;
    GCodeReader reader;
    reader.apply_config(config);
    bool eligible = false;
    bool first_original_line_seen = false;
    double mm3_per_mm = 0.;
    double eligible_feed = 0.;
    double active_pa = std::numeric_limits<double>::quiet_NaN();
    reader.parse_buffer(output, [&samples, &eligible, &first_original_line_seen, &mm3_per_mm, &eligible_feed, &active_pa](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        const size_t flow_marker = comment.find("MM3MM:");
        const size_t accel_marker = comment.find(" ACCEL:", flow_marker);
        const size_t overhang_marker = comment.find(" OV:", accel_marker);
        const bool pa_event = comment.find("PA_CHANGE:") != std::string_view::npos &&
                              flow_marker != std::string_view::npos &&
                              accel_marker != std::string_view::npos &&
                              overhang_marker != std::string_view::npos;
        if (pa_event) {
            // A new event before the previous eligible event reaches extrusion
            // ends that candidate; only the explicitly eligible subset counts.
            eligible = std::stoi(std::string(comment.substr(overhang_marker + 4))) == 1;
            first_original_line_seen = false;
            mm3_per_mm = std::stod(std::string(comment.substr(flow_marker + 6, accel_marker - flow_marker - 6)));
            eligible_feed = 0.;
        }

        if (line.cmd_is("SET_PRESSURE_ADVANCE")) {
            const std::string raw = line.raw();
            const size_t pa_marker = raw.find("ADVANCE=");
            REQUIRE(pa_marker != std::string::npos);
            const size_t pa_end = raw.find(';', pa_marker);
            active_pa = std::stod(raw.substr(pa_marker + 8, pa_end == std::string::npos ? std::string::npos : pa_end - pa_marker - 8));
            return;
        }

        const std::string raw = line.raw();
        const bool generated_comment = raw.rfind("; APA", 0) == 0;
        if (!eligible || generated_comment || comment.find("PA_CHANGE:") != std::string_view::npos)
            return;

        if (!first_original_line_seen) {
            // The production lookahead sets current and next feed to this value
            // when it is the first original line after the PA tag.
            first_original_line_seen = true;
            if (raw.rfind("G1 F", 0) != 0) {
                eligible = false;
                return;
            }
            eligible_feed = line.f() / 60.;
        }

        if (line.cmd_is("G1") && line.extruding(self) && line.dist_XY(self) > EPSILON) {
            samples.push_back({mm3_per_mm, eligible_feed, active_pa});
            eligible = false;
        }
    });
    return samples;
}

double expected_two_knot_pa(const AdaptivePASample &sample)
{
    const double flow = std::clamp(sample.mm3_per_mm * sample.feed, 1., 500.);
    const double unrounded = 0.020 + (0.800 - 0.020) * (flow - 1.) / (500. - 1.);
    return std::round(unrounded * 1000.) / 1000.;
}

TEST_CASE("Adaptive pressure advance emits calibrated values on overhangs",
          "[SettingsFamilyVariance][AdaptivePressureAdvanceRamps]")
{
    const DynamicPrintConfig enabled = adaptive_pa_output_config(true);
    const std::string enabled_gcode = export_mesh(adaptive_pa_overhang_mesh(), enabled);
    const std::vector<AdaptivePASample> samples = adaptive_pa_eligible_samples(enabled_gcode, enabled);
    REQUIRE(samples.size() > 1);

    double minimum_pa = std::numeric_limits<double>::max();
    double maximum_pa = std::numeric_limits<double>::lowest();
    for (const AdaptivePASample &sample : samples) {
        CAPTURE(sample.mm3_per_mm, sample.feed, sample.actual_pa);
        REQUIRE(std::isfinite(sample.mm3_per_mm));
        REQUIRE(std::isfinite(sample.feed));
        REQUIRE(std::isfinite(sample.actual_pa));
        REQUIRE(sample.mm3_per_mm > 0.);
        REQUIRE(sample.feed > 0.);
        REQUIRE(sample.actual_pa > 0.);
        CHECK_THAT(sample.actual_pa, Catch::Matchers::WithinAbs(expected_two_knot_pa(sample), 0.0011));
        minimum_pa = std::min(minimum_pa, sample.actual_pa);
        maximum_pa = std::max(maximum_pa, sample.actual_pa);
    }
    CAPTURE(minimum_pa, maximum_pa);
    CHECK(maximum_pa - minimum_pa > 0.005);

    const DynamicPrintConfig disabled = adaptive_pa_output_config(false);
    const std::string disabled_gcode = export_mesh(adaptive_pa_overhang_mesh(), disabled);
    CHECK(adaptive_pa_eligible_samples(disabled_gcode, disabled).empty());
    CHECK(disabled_gcode.find("SET_PRESSURE_ADVANCE ADVANCE=") == std::string::npos);
}

} // namespace settings_family_contracts

namespace reslice_settings_contracts {
using Catch::Matchers::WithinAbs;

namespace {
struct Totals {
    size_t paths = 0;
    double length = 0.;
    double volume = 0.;
};
struct LayerSnapshot {
    std::map<ExtrusionRole, Totals> totals;
    std::vector<std::array<long long, 5>> path_order;
};
using Snapshot = std::vector<LayerSnapshot>;

DynamicPrintConfig config()
{
    DynamicPrintConfig c = DynamicPrintConfig::full_print_config();
    c.set_deserialize_strict({
        {"layer_height", 0.2}, {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4}, {"wall_loops", 2}, {"wall_generator", "classic"},
        {"top_shell_layers", 3}, {"bottom_shell_layers", 3},
        {"top_shell_thickness", 0}, {"bottom_shell_thickness", 0},
        {"sparse_infill_density", "20%"}, {"elefant_foot_compensation", 0},
        // Center-based top and bottom patterns make fill-order variants
        // observable while retaining a small, deterministic stepped fixture.
        {"top_surface_pattern", "concentric"}, {"bottom_surface_pattern", "concentric"},
        {"internal_solid_infill_pattern", "monotonic"},
        {"layer_change_gcode", "G92 E0"},
        {"bridge_no_support", false}, {"make_overhang_printable", false},
    });
    return c;
}

TriangleMesh fixture()
{
    // A stepped solid includes exposed shelves, top/bottom shells, and internal
    // sparse regions in only twenty layers.
    TriangleMesh a = make_cube(12., 12., 2.);
    TriangleMesh b = make_cube(6., 8., 2.);
    b.translate(3., 2., 2.);
    a.merge(b);
    return a;
}

Snapshot snapshot(Print &print, const std::optional<size_t> expected_layer_count = size_t(20))
{
    const auto error = print.validate();
    INFO(error.string);
    REQUIRE(error.string.empty());
    print.set_status_silent();
    print.process();
    REQUIRE(print.objects().size() == 1);
    if (expected_layer_count)
        REQUIRE(print.objects().front()->layer_count() == *expected_layer_count);
    Snapshot result;
    for (const Layer *layer : print.objects().front()->layers()) {
        LayerSnapshot layer_snapshot;
        auto &totals = layer_snapshot.totals;
        for (const LayerRegion *region : layer->regions()) {
            const auto collect = [&](const ExtrusionPath &path) {
                const double length = unscale<double>(path.length());
                CHECK(std::isfinite(length));
                CHECK(length >= 0.);
                CHECK(std::isfinite(path.mm3_per_mm));
                CHECK(path.mm3_per_mm > 0.);
                auto &sum = totals[path.role()];
                ++sum.paths;
                sum.length += length;
                sum.volume += length * path.mm3_per_mm;
                const Point first = path.first_point();
                const Point last = path.last_point();
                layer_snapshot.path_order.push_back({
                    static_cast<long long>(path.role()), first.x(), first.y(), last.x(), last.y()});
            };
            for_each_extrusion_path(region->perimeters, collect);
            for_each_extrusion_path(region->fills, collect);
        }
        REQUIRE_FALSE(totals.empty());
        result.push_back(std::move(layer_snapshot));
    }
    return result;
}

void same(const Snapshot &a, const Snapshot &b)
{
    REQUIRE(a.size() == b.size());
    for (size_t layer = 0; layer < a.size(); ++layer) {
        CAPTURE(layer);
        REQUIRE(a[layer].totals.size() == b[layer].totals.size());
        for (const auto &[role, expected] : a[layer].totals) {
            CAPTURE(role);
            REQUIRE(b[layer].totals.count(role) == 1);
            const auto &actual = b[layer].totals.at(role);
            CHECK(actual.paths == expected.paths);
            CHECK_THAT(actual.length, WithinAbs(expected.length, 1e-5));
            CHECK_THAT(actual.volume, WithinAbs(expected.volume, 1e-5));
        }
        CHECK(a[layer].path_order == b[layer].path_order);
    }
}

bool different(const Snapshot &a, const Snapshot &b)
{
    if (a.size() != b.size()) return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (a[i].totals.size() != b[i].totals.size()) return true;
        for (const auto &[role, p] : a[i].totals) {
            const auto it = b[i].totals.find(role);
            if (it == b[i].totals.end()) return true;
            const auto &q = it->second;
            if (p.paths != q.paths || std::abs(p.length - q.length) > 1e-5 ||
                std::abs(p.volume - q.volume) > 1e-5) return true;
        }
        if (a[i].path_order != b[i].path_order) return true;
    }
    return false;
}

std::pair<size_t, double> path_totals(const Snapshot &snapshot, const bool solid_only = false)
{
    size_t paths = 0;
    double length = 0.;
    for (const LayerSnapshot &layer : snapshot)
        for (const auto &[role, totals] : layer.totals)
            if (!solid_only || role == erSolidInfill) {
                paths += totals.paths;
                length += totals.length;
            }
    return {paths, length};
}

size_t role_path_count(const Snapshot &snapshot, const ExtrusionRole role)
{
    size_t paths = 0;
    for (const LayerSnapshot &layer : snapshot)
        if (const auto it = layer.totals.find(role); it != layer.totals.end())
            paths += it->second.paths;
    return paths;
}

void check_variant_on(const TriangleMesh &mesh, std::initializer_list<ConfigBase::SetDeserializeItem> fixture_options,
                      const std::string &key, const std::string &before, const std::string &after,
                      bool expect_effect = true,
                      const std::optional<size_t> expected_layer_count = size_t(20))
{
    CAPTURE(key, before, after);
    DynamicPrintConfig base = config();
    base.set_deserialize_strict(fixture_options);
    REQUIRE(base.option(key) != nullptr);
    base.set_deserialize_strict(key, before);
    DynamicPrintConfig changed = base;
    changed.set_deserialize_strict(key, after);
    REQUIRE_FALSE(*base.option(key) == *changed.option(key));

    Print reused;
    Model model;
    init_print({mesh}, reused, model, base, nullptr, false);
    const Snapshot original = snapshot(reused, expected_layer_count);
    reused.apply(model, changed);
    const Snapshot incremental = snapshot(reused, expected_layer_count);
    Print fresh;
    Model fresh_model;
    init_print({mesh}, fresh, fresh_model, changed, nullptr, false);
    const Snapshot expected = snapshot(fresh, expected_layer_count);
    if (expect_effect)
        REQUIRE(different(original, expected));
    else
        CHECK_FALSE(different(original, expected));
    if (key == "detect_narrow_internal_solid_infill")
        REQUIRE(path_totals(expected, true).first > path_totals(original, true).first);
    if (key == "only_one_wall_top")
        REQUIRE(path_totals(expected).first < path_totals(original).first);
    if (key == "precise_outer_wall")
        REQUIRE(path_totals(expected).second < path_totals(original).second);
    if (key == "infill_complete_top" && expected_layer_count && *expected_layer_count == 50)
        REQUIRE(path_totals(expected).first > path_totals(original).first);
    if (key == "gap_fill_target" && after == "topbottom")
        REQUIRE(role_path_count(expected, erGapFill) < role_path_count(original, erGapFill));
    same(expected, incremental);
    reused.apply(model, base);
    same(original, snapshot(reused, expected_layer_count));

}

void check_variant(const std::string &key, const std::string &before, const std::string &after,
                   bool expect_effect = true)
{
    check_variant_on(fixture(), {}, key, before, after, expect_effect);
}

TriangleMesh overlap_fixture()
{
    TriangleMesh base = make_cube(12., 12., 4.);
    TriangleMesh overlap = make_cube(6., 8., 2.);
    overlap.translate(3., 2., 1.);
    base.merge(overlap);
    return base;
}

TriangleMesh overhang_fixture()
{
    TriangleMesh base = make_cube(12., 12., 2.);
    TriangleMesh overhang = make_cube(5., 5., 2.);
    overhang.translate(9., 3., 2.);
    base.merge(overhang);
    return base;
}

TriangleMesh step_expansion_fixture()
{
    TriangleMesh base = make_cube(60., 60., 5.);
    TriangleMesh upper = make_cube(54., 54., 5.);
    upper.translate(3.f, 3.f, 5.f);
    base.merge(upper);
    return base;
}

TriangleMesh nonuniform_height_fixture()
{
    TriangleMesh mesh = make_cube(12., 12., 4.);
    mesh.scale(Vec3f(1.f, 1.f, 1.005f));
    return mesh;
}

std::vector<bool> contour_winding(Print &print)
{
    std::vector<bool> winding;
    REQUIRE(print.objects().size() == 1);
    for (const Layer *layer : print.objects().front()->layers())
        for (const LayerRegion *region : layer->regions())
            for (const ExtrusionEntity *entity : region->perimeters.flatten().entities)
                if (entity->is_loop()) {
                    const auto *loop = static_cast<const ExtrusionLoop *>(entity);
                    winding.push_back(loop->polygon().is_counter_clockwise());
                }
    return winding;
}
}

namespace interface_shells_followup {

constexpr size_t expected_region_count = 2;

DynamicPrintConfig config(bool interface_shells)
{
    DynamicPrintConfig result = multifilament_config(2, {
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_generator", "classic"},
        {"wall_loops", 2},
        {"top_shell_layers", 2},
        {"bottom_shell_layers", 2},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"sparse_infill_density", "0%"},
        {"top_surface_density", "100%"},
        {"bottom_surface_density", "100%"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"single_extruder_multi_material", false},
        {"spiral_mode", false},
        {"interface_shells", interface_shells},
        {"layer_change_gcode", "G92 E0\n"},
        {"gcode_comments", true},
        {"outer_wall_line_width", 0.42},
        {"inner_wall_line_width", 0.45},
    });
    return result;
}

// The lower material ends at z=1.0 while the upper material starts at z=0.6
// and continues above it.  Their known 8x8 material interface is hidden only
// when regions are treated as one collective body.  The strongest-delta
// search intentionally finds the actual transition layer after slicing,
// rather than assuming which layer owns the overlap precedence.  A single
// ModelObject with two MODEL_PART volumes gives Print::apply the same public
// multi-region path used by production.
struct ProcessedFixture {
    Model model;
    Print print;

    explicit ProcessedFixture(bool interface_shells)
    {
        ModelObject *object = model.add_object();
        auto *lower = object->add_volume(make_cube(12., 12., 1.0), ModelVolumeType::MODEL_PART, false);
        TriangleMesh upper_mesh = make_cube(8., 8., 1.2);
        upper_mesh.translate(2.f, 2.f, 0.6f);
        auto *upper = object->add_volume(std::move(upper_mesh), ModelVolumeType::MODEL_PART, false);
        lower->config.set("extruder", 1);
        upper->config.set("extruder", 2);
        object->add_instance();
        object->ensure_on_bed();

        DynamicPrintConfig cfg = config(interface_shells);
        print.auto_assign_extruders(object);
        print.apply(model, cfg);
        const auto validation = print.validate();
        INFO("interface_shells=" << interface_shells << " validation=" << validation.string);
        REQUIRE(validation.string.empty());
        print.process();
        REQUIRE(print.objects().size() == 1);
        REQUIRE(print.objects().front()->num_printing_regions() == expected_region_count);
    }
};

double area_mm2(double scaled_area)
{
    const double one_mm = double(scale_(1.));
    return scaled_area / (one_mm * one_mm);
}

double area_for(const LayerRegion &region, SurfaceType type)
{
    double area = 0.;
    for (const Surface &surface : region.slices.surfaces)
        if (surface.surface_type == type)
            area += surface.expolygon.area();
    return area_mm2(area);
}

double all_surface_area(const LayerRegion &region)
{
    double area = 0.;
    for (const Surface &surface : region.slices.surfaces)
        area += surface.expolygon.area();
    return area_mm2(area);
}

struct TopSurfaceObservation {
    double z = 0.;
    size_t region = 0;
    double top_disabled = 0.;
    double top_enabled = 0.;
    double internal_disabled = 0.;
    double internal_enabled = 0.;
};

TopSurfaceObservation strongest_interface_surface_change(const PrintObject &disabled,
                                                          const PrintObject &enabled)
{
    TopSurfaceObservation best;
    REQUIRE_FALSE(disabled.layers().empty());
    REQUIRE_FALSE(enabled.layers().empty());
    for (const Layer *enabled_layer : enabled.layers()) {
        const Layer *disabled_layer = nullptr;
        for (const Layer *candidate : disabled.layers()) {
            if (std::abs(double(candidate->print_z) - double(enabled_layer->print_z)) < 1e-5) {
                disabled_layer = candidate;
                break;
            }
        }
        REQUIRE(disabled_layer != nullptr);
        REQUIRE(enabled_layer->regions().size() == expected_region_count);
        REQUIRE(disabled_layer->regions().size() == expected_region_count);
        for (size_t region_idx = 0; region_idx < enabled_layer->regions().size(); ++region_idx) {
            const LayerRegion &disabled_region = *disabled_layer->get_region(static_cast<int>(region_idx));
            const LayerRegion &enabled_region = *enabled_layer->get_region(static_cast<int>(region_idx));
            const double disabled_top = area_for(disabled_region, stTop);
            const double enabled_top = area_for(enabled_region, stTop);
            const double gain = enabled_top - disabled_top;
            if (gain > best.top_enabled - best.top_disabled) {
                best.z = double(enabled_layer->print_z);
                best.region = region_idx;
                best.top_disabled = disabled_top;
                best.top_enabled = enabled_top;
                best.internal_disabled = area_for(disabled_region, stInternal);
                best.internal_enabled = area_for(enabled_region, stInternal);
            }
        }
    }
    return best;
}

} // namespace interface_shells_followup

TEST_CASE("Interface shells preserve a material's hidden top surface", "[InterfaceShells][SettingsVariance]")
{
    interface_shells_followup::ProcessedFixture disabled(false);
    interface_shells_followup::ProcessedFixture enabled(true);
    const PrintObject &disabled_object = *disabled.print.objects().front();
    const PrintObject &enabled_object = *enabled.print.objects().front();
    REQUIRE_FALSE(disabled_object.layers().empty());
    REQUIRE_FALSE(enabled_object.layers().empty());
    REQUIRE(disabled_object.layers().size() == enabled_object.layers().size());
    REQUIRE(disabled_object.layers().front()->regions().size() == interface_shells_followup::expected_region_count);
    REQUIRE(enabled_object.layers().front()->regions().size() == interface_shells_followup::expected_region_count);

    const auto changed = interface_shells_followup::strongest_interface_surface_change(disabled_object, enabled_object);
    INFO("strongest interface surface change at z=" << changed.z << " region=" << changed.region
         << " disabled top=" << changed.top_disabled << " enabled top=" << changed.top_enabled
         << " disabled internal=" << changed.internal_disabled << " enabled internal=" << changed.internal_enabled);

    // The known 8x8 material overlap is hidden from the lower material when
    // interface shells are disabled.  A safety offset can trim its boundary,
    // so the oracle intentionally uses a conservative area lower bound.
    CHECK(changed.top_enabled - changed.top_disabled > 20.0);
    CHECK(changed.internal_disabled - changed.internal_enabled > 20.0);
    CHECK_THAT(changed.internal_disabled - changed.internal_enabled,
               Catch::Matchers::WithinAbs(changed.top_enabled - changed.top_disabled, 2.0));

    // The partition must conserve each region's sliced footprint.  This also
    // prevents a mutation from passing by deleting the overlap instead of
    // reclassifying it as a top surface.
    for (const Layer *enabled_layer : enabled_object.layers()) {
        const Layer *disabled_layer = nullptr;
        for (const Layer *candidate : disabled_object.layers()) {
            if (std::abs(double(candidate->print_z) - double(enabled_layer->print_z)) < 1e-5) {
                disabled_layer = candidate;
                break;
            }
        }
        REQUIRE(disabled_layer != nullptr);
        REQUIRE(enabled_layer->regions().size() == interface_shells_followup::expected_region_count);
        REQUIRE(disabled_layer->regions().size() == interface_shells_followup::expected_region_count);
        for (size_t region_idx = 0; region_idx < enabled_layer->regions().size(); ++region_idx) {
            const double disabled_area = interface_shells_followup::all_surface_area(
                *disabled_layer->get_region(static_cast<int>(region_idx)));
            const double enabled_area = interface_shells_followup::all_surface_area(
                *enabled_layer->get_region(static_cast<int>(region_idx)));
            CHECK_THAT(enabled_area, Catch::Matchers::WithinAbs(disabled_area, 0.05));
        }
    }

    // The upper material's final exterior layer is outside the lower volume;
    // its top-surface area is therefore an unaffected control for this toggle.
    const Layer *disabled_last = disabled_object.layers().back();
    const Layer *enabled_last = enabled_object.layers().back();
    REQUIRE_THAT(double(enabled_last->print_z), Catch::Matchers::WithinAbs(double(disabled_last->print_z), 1e-5));
    REQUIRE(disabled_last->regions().size() == interface_shells_followup::expected_region_count);
    REQUIRE(enabled_last->regions().size() == interface_shells_followup::expected_region_count);
    double disabled_last_top = 0.;
    double enabled_last_top = 0.;
    for (const LayerRegion *region : disabled_last->regions())
        disabled_last_top += interface_shells_followup::area_for(*region, stTop);
    for (const LayerRegion *region : enabled_last->regions())
        enabled_last_top += interface_shells_followup::area_for(*region, stTop);
    REQUIRE(disabled_last_top > 20.0);
    REQUIRE(enabled_last_top > 20.0);
    CHECK_THAT(enabled_last_top, Catch::Matchers::WithinAbs(disabled_last_top, 0.05));
}

TEST_CASE("Solid infill pattern changes match fresh slicing in both directions", "[SettingsVariance][SettingsReslice]")
{
    const auto pattern = GENERATE("monotonicline", "rectilinear", "alignedrectilinear", "concentric",
                                  "spiralinset", "hilbertcurve", "archimedeanchords", "octagramspiral");
    check_variant("internal_solid_infill_pattern", "monotonic", pattern);
}

TEST_CASE("Shell and perimeter enum changes match fresh slicing in both directions", "[SettingsVariance][SettingsReslice]")
{
    struct Row { const char *key; const char *before; const char *after; bool expect_effect; };
    const std::vector<Row> rows {
        {"top_surface_fill_order", "default", "outward", true},
        {"top_surface_fill_order", "default", "inward", true},
        // The stepped solid has no bottom internal interface and therefore
        // exercises the setting without making the result change.
        {"bottom_surface_fill_order", "default", "outward", false},
        {"bottom_surface_fill_order", "default", "inward", true},
        // Expansion is only observable when only-one-wall top frees an area;
        // this fixture deliberately records that activation precondition.
        {"top_surface_expansion_direction", "inward_and_outward", "inward", false},
        {"top_surface_expansion_direction", "inward_and_outward", "outward", false},
        {"gap_fill_target", "everywhere", "topbottom", true},
        {"gap_fill_target", "everywhere", "nowhere", true},
        {"wall_direction", "ccw", "cw", false},
        {"slicing_mode", "regular", "even_odd", false},
        {"slicing_mode", "regular", "close_holes", false},
        {"dont_filter_internal_bridges", "disabled", "limited", true},
        {"dont_filter_internal_bridges", "disabled", "nofilter", true},
        // No bridge surface is present in this non-support fixture.
        {"enable_extra_bridge_layer", "disabled", "external_bridge_only", false},
        {"enable_extra_bridge_layer", "disabled", "internal_bridge_only", true},
        {"enable_extra_bridge_layer", "disabled", "apply_to_all", true},
    };
    const size_t index = GENERATE_COPY(Catch::Generators::range(size_t(0), rows.size()));
    check_variant(rows[index].key, rows[index].before, rows[index].after, rows[index].expect_effect);
}

TEST_CASE("Geometry boolean changes match fresh slicing in both directions", "[SettingsVariance][SettingsReslice]")
{
    struct Row { const char *key; bool expect_effect; };
    const std::vector<Row> rows {
        {"bridge_no_support", false},
        {"detect_narrow_internal_solid_infill", true},
        {"infill_complete_top", false},
        {"interface_shells", false},
        {"make_overhang_printable", false},
        {"precise_outer_wall", true},
        {"precise_z_height", false},
        {"staggered_inner_seams", false},
        {"only_one_wall_top", true},
        {"only_one_wall_first_layer", true},
    };
    const size_t index = GENERATE_COPY(Catch::Generators::range(size_t(0), rows.size()));
    // The false rows are explicit negative controls: their activation
    // preconditions are absent from this fixture, while fresh/incremental
    // equivalence and restoration remain required.
    check_variant(rows[index].key, "0", "1", rows[index].expect_effect);
}

TEST_CASE("Bottom fill order reverses the bottom surface ordering", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant("bottom_surface_fill_order", "inward", "outward");
}

TEST_CASE("Top expansion direction changes a configured top expansion", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(step_expansion_fixture(),
                     {{"wall_generator", "arachne"}, {"wall_loops", 3}, {"top_shell_layers", 3},
                      {"bottom_shell_layers", 3}, {"top_surface_density", "100%"},
                     {"min_width_top_surface", 0.0}, {"only_one_wall_top", true},
                      {"top_surface_expansion", 2.0}},
                     "top_surface_expansion_direction", "inward", "outward", true, 50);
}

TEST_CASE("Honeycomb complete-top setting changes the internal solid fill", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(make_cube(20., 20., 10.),
                     {{"sparse_infill_pattern", "3dhoneycomb"}, {"sparse_infill_density", "20%"},
                      {"top_shell_layers", 0}, {"bottom_shell_layers", 0}},
                     "infill_complete_top", "0", "1", true, 50);
}

TEST_CASE("Overlapping volumes select a slicing mode with different parity", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(overlap_fixture(), {}, "slicing_mode", "regular", "even_odd");
}

TEST_CASE("External overhang setting changes printable overhang treatment", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(overhang_fixture(), {{"make_overhang_printable_angle", 20.0}},
                     "make_overhang_printable", "0", "1");
}

TEST_CASE("Extra bridge layer mode changes overhang processing", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(overhang_fixture(), {{"bridge_no_support", true}},
                     "enable_extra_bridge_layer", "disabled", "external_bridge_only");
}

TEST_CASE("Precise Z height changes a nonuniform-height mesh", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    check_variant_on(nonuniform_height_fixture(), {}, "precise_z_height", "0", "1", true, std::nullopt);
}

TEST_CASE("Wall direction reverses every generated perimeter winding", "[SettingsVariance][SettingsReslice][SettingsResliceActivation]")
{
    DynamicPrintConfig ccw = config();
    ccw.set_deserialize_strict({{"wall_direction", "ccw"}, {"wall_loops", 2}});
    DynamicPrintConfig cw = ccw;
    cw.set_deserialize_strict("wall_direction", "cw");
    Print first;
    Model first_model;
    init_print({fixture()}, first, first_model, ccw, nullptr, false);
    first.process();
    const auto first_winding = contour_winding(first);
    Print second;
    Model second_model;
    init_print({fixture()}, second, second_model, cw, nullptr, false);
    second.process();
    const auto second_winding = contour_winding(second);
    REQUIRE_FALSE(first_winding.empty());
    REQUIRE(first_winding.size() == second_winding.size());
    CHECK(std::all_of(first_winding.begin(), first_winding.end(), [](bool value) { return value; }));
    CHECK(std::all_of(second_winding.begin(), second_winding.end(), [](bool value) { return !value; }));
    bool all_reversed = true;
    for (size_t index = 0; index < first_winding.size(); ++index)
        all_reversed = all_reversed && first_winding[index] != second_winding[index];
    CHECK(all_reversed);
}


namespace seam_stagger_followup {

struct InnerSeamLayerEvidence {
    std::vector<std::array<double, 3>> starts;
    double xy = 0.0;
    double filament = 0.0;
    size_t positive_xy_segments = 0;
};

struct InnerSeamGcodeEvidence {
    std::vector<InnerSeamLayerEvidence> layers;
    bool finite = true;
};

InnerSeamGcodeEvidence parse_inner_seams(const std::string &output,
                                         const DynamicPrintConfig &config)
{
    InnerSeamGcodeEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    int current_layer = -1;
    bool inner_role = false;
    bool awaiting_loop_start = false;
    reader.parse_buffer(output, [&evidence, &current_layer, &inner_role, &awaiting_loop_start](GCodeReader &self,
                                                                                                const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.rfind("LAYER_CHANGE", 0) == 0) {
            ++current_layer;
            evidence.layers.emplace_back();
            awaiting_loop_start = true;
        }

        const bool enters_inner_role =
            comment.rfind("TYPE:Inner wall", 0) == 0 ||
            comment.rfind("FEATURE:Inner wall", 0) == 0 ||
            comment.rfind("TYPE:Internal perimeter", 0) == 0 ||
            comment.rfind("FEATURE:Internal perimeter", 0) == 0;
        if (enters_inner_role) {
            if (!inner_role)
                awaiting_loop_start = true;
            inner_role = true;
        } else if (comment.rfind("TYPE:", 0) == 0 || comment.rfind("FEATURE:", 0) == 0) {
            inner_role = false;
            awaiting_loop_start = false;
        }

        if (current_layer < 0 || current_layer >= static_cast<int>(evidence.layers.size()))
            return;

        const double xy = line.dist_XY(self);
        if (inner_role && line.retracting(self))
            awaiting_loop_start = true;
        if (inner_role && (line.travel() || line.cmd_is("G0")) && xy > EPSILON)
            awaiting_loop_start = true;

        if (!inner_role || !line.extruding(self) || xy <= EPSILON)
            return;

        const double de = line.dist_E(self);
        if (!std::isfinite(xy) || !std::isfinite(de) ||
            !std::isfinite(static_cast<double>(self.x())) ||
            !std::isfinite(static_cast<double>(self.y())) ||
            !std::isfinite(static_cast<double>(self.z()))) {
            evidence.finite = false;
            return;
        }

        InnerSeamLayerEvidence &layer = evidence.layers[static_cast<size_t>(current_layer)];
        if (awaiting_loop_start) {
            layer.starts.push_back({static_cast<double>(self.x()),
                                    static_cast<double>(self.y()),
                                    static_cast<double>(self.z())});
            awaiting_loop_start = false;
        }
        layer.xy += xy;
        layer.filament += de;
        ++layer.positive_xy_segments;
    });
    return evidence;
}

size_t inner_loop_count(const InnerSeamGcodeEvidence &evidence)
{
    size_t count = 0;
    for (const InnerSeamLayerEvidence &layer : evidence.layers)
        count += layer.starts.size();
    return count;
}

} // namespace seam_stagger_followup

TEST_CASE("Staggered inner seams move internal perimeter starts while preserving geometry",
          "[SettingsVariance][StaggeredInnerSeams]")
{
    const auto run = [](const bool staggered) {
        DynamicPrintConfig config = reslice_settings_contracts::config();
        config.set_deserialize_strict({
            {"wall_loops", 3},
            {"wall_sequence", "inner wall/outer wall"},
            {"top_shell_layers", 0},
            {"bottom_shell_layers", 0},
            {"top_shell_thickness", 0.0},
            {"bottom_shell_thickness", 0.0},
            {"sparse_infill_density", "0%"},
            {"seam_position", "back"},
            {"seam_slope_type", "none"},
            {"seam_gap", 0.0},
            {"resolution", 0.001},
            {"wall_maximum_resolution", 0.005},
            {"enable_arc_fitting", false},
            {"use_relative_e_distances", true},
            {"gcode_comments", true},
            {"staggered_inner_seams", staggered},
        });
        Print print;
        Model model;
        init_print({reslice_settings_contracts::fixture()}, print, model, config, nullptr, false);
        REQUIRE(print.validate().string.empty());
        REQUIRE_NOTHROW(print.process());
        const std::string output = gcode(print);
        REQUIRE_FALSE(output.empty());

        size_t parsed_motion_moves = 0;
        size_t parsed_extrusion_moves = 0;
        REQUIRE(finite_bounded_gcode(output, print.config(), parsed_motion_moves, parsed_extrusion_moves));
        REQUIRE(parsed_motion_moves > 0);
        REQUIRE(parsed_extrusion_moves > 0);

        const auto evidence = seam_stagger_followup::parse_inner_seams(output, config);
        REQUIRE(evidence.finite);
        REQUIRE_FALSE(evidence.layers.empty());
        return evidence;
    };

    const auto disabled = run(false);
    const auto enabled = run(true);
    REQUIRE(disabled.layers.size() == enabled.layers.size());

    const size_t disabled_count = seam_stagger_followup::inner_loop_count(disabled);
    const size_t enabled_count = seam_stagger_followup::inner_loop_count(enabled);
    REQUIRE(disabled_count > 0);
    REQUIRE(enabled_count == disabled_count);

    size_t shifted_starts = 0;
    for (size_t layer = 0; layer < disabled.layers.size(); ++layer) {
        const auto &disabled_layer = disabled.layers[layer];
        const auto &enabled_layer = enabled.layers[layer];
        REQUIRE_FALSE(disabled_layer.starts.empty());
        REQUIRE_FALSE(enabled_layer.starts.empty());
        REQUIRE(disabled_layer.starts.size() == enabled_layer.starts.size());
        CHECK_THAT(enabled_layer.starts.front()[2], WithinAbs(disabled_layer.starts.front()[2], 0.002));

        // GCodeWriter emits XYZ with three decimals and E with five. Bound the
        // conservation check by the number of positive XY segments; raw segment
        // count is deliberately not a seam oracle.
        const size_t max_segments = std::max<size_t>(
            1, std::max(disabled_layer.positive_xy_segments, enabled_layer.positive_xy_segments));
        const double xy_tolerance = 2.0 * std::sqrt(2.0) * 0.001 * static_cast<double>(max_segments) + 1e-6;
        const double filament_tolerance = 2.0 * 0.00001 * static_cast<double>(max_segments) + 1e-6;
        CHECK_THAT(enabled_layer.xy, WithinAbs(disabled_layer.xy, xy_tolerance));
        CHECK_THAT(enabled_layer.filament, WithinAbs(disabled_layer.filament, filament_tolerance));

        for (size_t index = 0; index < disabled_layer.starts.size(); ++index) {
            const auto &a = disabled_layer.starts[index];
            const auto &b = enabled_layer.starts[index];
            CHECK_THAT(b[2], WithinAbs(a[2], 0.002));
            const double shift = std::hypot(a[0] - b[0], a[1] - b[1]);
            if (shift > 0.01)
                ++shifted_starts;
        }
    }
    REQUIRE(shifted_starts > 0);
}

} // namespace reslice_settings_contracts
namespace ironing_enum_selection_followup {

struct LayerObservation {
    double z = 0.0;
    double top_surface_area = 0.0;
    double interior_fill_area = 0.0;
    size_t ironing_paths = 0;
    double ironing_length = 0.0;
};

struct IroningEnumObservation {
    std::vector<LayerObservation> layers;
    double lower_top_area = 0.0;
    double upper_top_area = 0.0;
    double interior_fill_area = 0.0;
    size_t interior_fill_layers = 0;
    size_t interior_ironing_layers = 0;
    size_t lower_ironing_paths = 0;
    size_t upper_ironing_paths = 0;
    size_t interior_ironing_paths = 0;
    double lower_ironing_length = 0.0;
    double upper_ironing_length = 0.0;
    double interior_ironing_length = 0.0;
};

TriangleMesh stepped_ironing_fixture()
{
    TriangleMesh lower = make_cube(12., 12., 2.);
    TriangleMesh upper = make_cube(6., 8., 2.);
    upper.translate(3., 2., 2.);
    lower.merge(upper);
    return lower;
}

IroningEnumObservation run_ironing_type(const char *type)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"nozzle_diameter", 0.4},
        {"wall_generator", "classic"},
        {"wall_loops", 2},
        {"top_shell_layers", 1},
        {"bottom_shell_layers", 1},
        {"top_shell_thickness", 0.0},
        {"bottom_shell_thickness", 0.0},
        {"sparse_infill_density", "100%"},
        {"top_surface_density", "100%"},
        {"bottom_surface_density", "100%"},
        {"ironing_type", type},
        {"ironing_pattern", "rectilinear"},
        {"ironing_flow", "10%"},
        {"ironing_speed", 20.0},
        {"ironing_spacing", 0.2},
        {"ironing_inset", 0.0},
        {"spiral_mode", false},
        {"use_relative_e_distances", true},
        {"layer_change_gcode", "G92 E0\n"},
        {"skirt_loops", 0},
        {"brim_type", "no_brim"},
        {"raft_layers", 0},
    });

    Print print;
    Model model;
    init_print({stepped_ironing_fixture()}, print, model, config, nullptr, false);
    const auto validation = print.validate();
    INFO("ironing_type=" << type << " validation=" << validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());
    REQUIRE(print.objects().size() == 1);
    REQUIRE(print.objects().front()->layer_count() == 20);

    IroningEnumObservation result;
    const PrintObject &object = *print.objects().front();
    result.layers.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        LayerObservation observed;
        observed.z = layer->print_z;
        for (const LayerRegion *region : layer->regions()) {
            for (const Surface &surface : region->fill_surfaces.surfaces) {
                const double area = static_cast<double>(surface.expolygon.area());
                if (surface.surface_type == stTop)
                    observed.top_surface_area += area;
                if (surface.surface_type == stInternal || surface.surface_type == stInternalSolid)
                    observed.interior_fill_area += area;
            }
            for_each_extrusion_path(region->fills, [&observed](const ExtrusionPath &path) {
                if (path.role() != erIroning)
                    return;
                const double length = unscale<double>(path.length());
                REQUIRE(std::isfinite(length));
                REQUIRE(length > 0.0);
                REQUIRE(std::isfinite(path.mm3_per_mm));
                REQUIRE(path.mm3_per_mm > 0.0);
                ++observed.ironing_paths;
                observed.ironing_length += length;
            });
        }
        result.layers.push_back(observed);
        if (std::abs(observed.z - 2.0) < 0.03) {
            result.lower_top_area += observed.top_surface_area;
            result.lower_ironing_paths += observed.ironing_paths;
            result.lower_ironing_length += observed.ironing_length;
        }
        if (std::abs(observed.z - 4.0) < 0.03) {
            result.upper_top_area += observed.top_surface_area;
            result.upper_ironing_paths += observed.ironing_paths;
            result.upper_ironing_length += observed.ironing_length;
        }
        if (observed.z > 0.4 && observed.z < 1.8) {
            if (observed.interior_fill_area > 0.0)
                ++result.interior_fill_layers;
            if (observed.ironing_paths > 0)
                ++result.interior_ironing_layers;
            result.interior_fill_area += observed.interior_fill_area;
            result.interior_ironing_paths += observed.ironing_paths;
            result.interior_ironing_length += observed.ironing_length;
        }
    }

    // The mesh has an exposed lower terrace at z=2 and an upper top at z=4;
    // 100% sparse density gives the intervening layers independent solid area.
    REQUIRE(result.lower_top_area > 0.0);
    REQUIRE(result.upper_top_area > 0.0);
    REQUIRE(result.interior_fill_area > 0.0);
    return result;
}

} // namespace ironing_enum_selection_followup

TEST_CASE("Ironing type selects terrace, topmost, and all-solid public paths",
          "[SettingsVariance][IroningTypeSelection]")
{
    using ironing_enum_selection_followup::run_ironing_type;

    const auto no_ironing = run_ironing_type("no ironing");
    const auto top         = run_ironing_type("top");
    const auto topmost     = run_ironing_type("topmost");
    const auto solid       = run_ironing_type("solid");

    REQUIRE(std::all_of(no_ironing.layers.begin(), no_ironing.layers.end(),
                        [](const ironing_enum_selection_followup::LayerObservation &layer) {
                            return layer.ironing_paths == 0;
                        }));

    // The stepped fixture has two independent top surfaces.  Top irons the
    // lower exposed terrace and the upper top; topmost restricts ironing to
    // the highest layer while retaining the same upper geometry route.
    REQUIRE(top.lower_ironing_paths > 0);
    REQUIRE(top.upper_ironing_paths > 0);
    REQUIRE(topmost.lower_ironing_paths == 0);
    REQUIRE(topmost.upper_ironing_paths > 0);
    CHECK_THAT(top.upper_ironing_length, Catch::Matchers::WithinRel(topmost.upper_ironing_length, 0.05));
    CHECK(top.lower_ironing_length > 0.0);
    REQUIRE(top.interior_ironing_layers == 0);
    REQUIRE(topmost.interior_ironing_layers == 0);

    // AllSolid is independently selected by the sparse 100% interior layers,
    // so it must produce ironing inside the object as well as on its exposed
    // step surfaces.
    REQUIRE(solid.lower_ironing_paths > 0);
    REQUIRE(solid.upper_ironing_paths > 0);
    REQUIRE(solid.interior_fill_layers > 0);
    REQUIRE(solid.interior_ironing_layers == solid.interior_fill_layers);
    for (const auto &layer : solid.layers) {
        if (layer.z > 0.4 && layer.z < 1.8) {
            CAPTURE(layer.z);
            REQUIRE(layer.interior_fill_area > 0.0);
            REQUIRE(layer.ironing_paths > 0);
            REQUIRE(layer.ironing_length > 0.0);
        }
    }
    REQUIRE(solid.interior_ironing_paths > 0);
    REQUIRE(solid.interior_ironing_length > 0.0);
}
TEST_CASE("Infill anchor maximum preserves zero-boundary and clamp equivalence", "[SettingsFamilyVariance][SettingsVariance][InfillAnchorVariance]")
{
    auto evidence_for = [](const std::string &anchor, const std::string &maximum) {
        DynamicPrintConfig config = family_config();
        config.set_deserialize_strict({
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            // Grid with multiline=1 enters FillRectilinear::fill_surface_by_multilines,
            // which reaches connect_infill and consumes both anchor controls.
            {"sparse_infill_pattern", "grid"},
            {"fill_multiline", 1},
            {"sparse_infill_density", "35%"},
            {"sparse_infill_line_width", 0.4},
            {"initial_layer_line_width", 0.4},
            {"infill_anchor", anchor},
            {"infill_anchor_max", maximum},
        });
        return anchor_evidence(config);
    };

    // PrintConfig.cpp documents maximum=0 as the legacy connection mode,
    // equivalent to the explicit 0/0 no-anchor boundary.
    const AnchorEvidence legacy = evidence_for("1000", "0");
    const AnchorEvidence zero    = evidence_for("0", "0");
    REQUIRE(legacy.paths > 0);
    REQUIRE(zero.paths > 0);
    CHECK(legacy.paths == zero.paths);
    CHECK(legacy.endpoint_contacts == zero.endpoint_contacts);
    CHECK(legacy.endpoint_samples == zero.endpoint_samples);
    CHECK_THAT(legacy.length, Catch::Matchers::WithinAbs(zero.length, 1e-6));
    CHECK_THAT(legacy.endpoint_distance_sum, Catch::Matchers::WithinAbs(zero.endpoint_distance_sum, 1e-6));
    CHECK_THAT(legacy.endpoint_distance_max, Catch::Matchers::WithinAbs(zero.endpoint_distance_max, 1e-6));

    // The production clamp makes a requested 1.2 mm anchor with a 0.6 mm
    // maximum observably identical to an explicitly requested 0.6/0.6 pair.
    const AnchorEvidence requested_above_max = evidence_for("1.2", "0.6");
    const AnchorEvidence explicit_max        = evidence_for("0.6", "0.6");
    REQUIRE(requested_above_max.paths > 0);
    REQUIRE(explicit_max.paths > 0);
    CHECK(requested_above_max.paths == explicit_max.paths);
    CHECK(requested_above_max.endpoint_contacts == explicit_max.endpoint_contacts);
    CHECK(requested_above_max.endpoint_samples == explicit_max.endpoint_samples);
    CHECK_THAT(requested_above_max.length, Catch::Matchers::WithinAbs(explicit_max.length, 1e-6));
    CHECK_THAT(requested_above_max.endpoint_distance_sum, Catch::Matchers::WithinAbs(explicit_max.endpoint_distance_sum, 1e-6));
    CHECK_THAT(requested_above_max.endpoint_distance_max, Catch::Matchers::WithinAbs(explicit_max.endpoint_distance_max, 1e-6));

    // Keep each setting's positive response independently visible: the
    // anchor changes while max=0.6 is fixed, and max changes while anchor=0.6 is
    // fixed.  These controls prevent an insensitive fixture from earning
    // percentage/clamp coverage merely through equal-output rows.
    const AnchorEvidence no_anchor_limited = evidence_for("0", "0.6");
    const AnchorEvidence anchor_limited    = evidence_for("0.6", "0.6");
    REQUIRE(no_anchor_limited.paths > 0);
    REQUIRE(anchor_limited.paths > 0);
    CHECK(std::abs(anchor_limited.length - no_anchor_limited.length) > 0.05);

    const AnchorEvidence max_zero = evidence_for("0.6", "0");
    const AnchorEvidence max_small = anchor_limited;
    REQUIRE(max_zero.paths > 0);
    REQUIRE(max_small.paths > 0);
    CHECK(std::abs(max_small.length - max_zero.length) > 0.05);
}

TEST_CASE("Infill anchor percentages match equivalent absolute spacing", "[SettingsFamilyVariance][SettingsVariance][InfillAnchorVariance]")
{
    constexpr double anchor_mm  = 0.3;
    constexpr double maximum_mm = 0.6;

    auto config_for = [](const std::string &anchor, const std::string &maximum) {
        DynamicPrintConfig config = family_config();
        config.set_deserialize_strict({
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            // Grid with multiline=1 enters FillRectilinear::fill_surface_by_multilines,
            // which reaches connect_infill and consumes both anchor controls.
            {"sparse_infill_pattern", "grid"},
            {"fill_multiline", 1},
            {"sparse_infill_density", "35%"},
            {"sparse_infill_line_width", 0.4},
            {"initial_layer_line_width", 0.4},
            {"infill_anchor", anchor},
            {"infill_anchor_max", maximum},
        });
        return config;
    };

    DynamicPrintConfig absolute = config_for("0.3", "0.6");
    const AnchorEvidence absolute_evidence = anchor_evidence(absolute);
    REQUIRE(absolute_evidence.paths > 0);
    REQUIRE(std::isfinite(absolute_evidence.spacing));
    REQUIRE(absolute_evidence.spacing > 0.);

    const std::string anchor_percent = std::to_string(100. * anchor_mm / absolute_evidence.spacing) + "%";
    const std::string maximum_percent = std::to_string(100. * maximum_mm / absolute_evidence.spacing) + "%";

    auto check_equivalent = [&](const AnchorEvidence &candidate) {
        REQUIRE(candidate.paths > 0);
        CHECK(candidate.paths == absolute_evidence.paths);
        CHECK(candidate.endpoint_contacts == absolute_evidence.endpoint_contacts);
        CHECK(candidate.endpoint_samples == absolute_evidence.endpoint_samples);
        CHECK_THAT(candidate.length, Catch::Matchers::WithinAbs(absolute_evidence.length, 0.02));
        CHECK_THAT(candidate.endpoint_distance_sum, Catch::Matchers::WithinAbs(absolute_evidence.endpoint_distance_sum, 0.02));
        CHECK_THAT(candidate.endpoint_distance_max, Catch::Matchers::WithinAbs(absolute_evidence.endpoint_distance_max, 0.005));
    };

    // Isolate each conversion and then exercise both conversions together.
    check_equivalent(anchor_evidence(config_for(anchor_percent, "0.6")));
    check_equivalent(anchor_evidence(config_for("0.3", maximum_percent)));
    check_equivalent(anchor_evidence(config_for(anchor_percent, maximum_percent)));
}
