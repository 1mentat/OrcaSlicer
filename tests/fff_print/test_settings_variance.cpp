#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
    double perimeter_length = 0.0;
    double sparse_length = 0.0;
    double solid_length = 0.0;
    double top_length = 0.0;
    double bottom_length = 0.0;
    double top_shell_length = 0.0;
    double bottom_shell_length = 0.0;
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
    for (const Layer *layer : object.layers()) {
        double layer_shell_length = 0.0;
        size_t layer_perimeter_items = 0;
        for (const LayerRegion *region : layer->regions()) {
            evidence.perimeter_length += collection_length(region->perimeters);
            layer_perimeter_items += region->perimeters.items_count();
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
                default:
                    break;
                }
            });
        }
        evidence.perimeter_items += layer_perimeter_items;
        if (layer_perimeter_items > 0)
            ++evidence.perimeter_layers;
        shell_length_by_layer.push_back(layer_shell_length);
    }

    // The outermost surface has the top/bottom role, while preceding shell
    // layers use erSolidInfill. Count contiguous shell coverage from each
    // extremity so changing shell_layers cannot be hidden by role selection.
    evidence.bottom_layers = 0;
    while (evidence.bottom_layers < shell_length_by_layer.size() &&
           shell_length_by_layer[evidence.bottom_layers] > 0.0) {
        evidence.bottom_shell_length += shell_length_by_layer[evidence.bottom_layers];
        ++evidence.bottom_layers;
    }
    evidence.top_layers = 0;
    while (evidence.top_layers < shell_length_by_layer.size() &&
           shell_length_by_layer[shell_length_by_layer.size() - 1 - evidence.top_layers] > 0.0) {
        evidence.top_shell_length += shell_length_by_layer[shell_length_by_layer.size() - 1 - evidence.top_layers];
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
        } else {
            CHECK(evidence.top_length > 0.0);
            CHECK(evidence.bottom_length > 0.0);
            CHECK(evidence.top_shell_length > 0.0);
            CHECK(evidence.bottom_shell_length > 0.0);
            CHECK(evidence.top_layers == static_cast<size_t>(shell_layers));
            CHECK(evidence.bottom_layers == static_cast<size_t>(shell_layers));
        }
        if (shell_layers == 1)
            one_shell = evidence;
        if (shell_layers == 2)
            two_shells = evidence;
    }
    CHECK(two_shells.top_layers > one_shell.top_layers);
    CHECK(two_shells.bottom_layers > one_shell.bottom_layers);
    CHECK(two_shells.top_shell_length > one_shell.top_shell_length);
    CHECK(two_shells.bottom_shell_length > one_shell.bottom_shell_length);
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
