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

} // namespace wall_bridge_contracts
