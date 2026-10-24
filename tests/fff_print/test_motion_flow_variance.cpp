#include "catch2/catch_test_macros.hpp"
#include "catch2/generators/catch_generators.hpp"
#include "catch2/catch_message.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "test_helpers.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

enum class Signal {
    outer_feedrate,
    inner_feedrate,
    sparse_feedrate,
    top_feedrate,
    travel_feedrate,
    acceleration_commands,
    outer_geometry,
    inner_geometry,
    sparse_geometry,
    all_filament,
};

struct MatrixCase {
    const char *key;
    ConfigOptionType type;
    Signal signal;
    std::vector<double> values;
};

struct RoleEvidence {
    // Geometry is measured from PrintObject extrusion paths. G-code path
    // lengths are intentionally kept separate from it: quantization and
    // writer filtering make them related, but they are different oracles.
    double geometry_path_length = 0.0;
    double gcode_path_length = 0.0;
    double role_motion_path_length = 0.0;
    size_t role_motion_moves = 0;
    double filament_length = 0.0;
    // Maximum of |previous E| and |current E| for positive moves. This is
    // needed because GCodeReader::dist_E() subtracts two parsed float E values.
    double max_positive_e_pair = 0.0;
    double feedrate = 0.0;
    size_t moves = 0;
    // Layer-split material evidence keeps first-layer flow distinct from a
    // role-specific ratio. A role can legitimately occur only on one side of
    // the split (bottom on the first layers, top on later layers).
    double first_layer_filament_length = 0.0;
    double later_layer_filament_length = 0.0;
    size_t first_layer_moves = 0;
    size_t later_layer_moves = 0;
};

struct Evidence {
    size_t layers = 0;
    size_t perimeter_items = 0;
    double perimeter_length = 0.0;
    std::array<RoleEvidence, 8> roles{};
    double travel_feedrate = 0.0;
    size_t motion_moves = 0;
    size_t extrusion_moves = 0;
    double gcode_path_length = 0.0;
    double gcode_filament_length = 0.0;
    // Maximum of |previous E| and |current E| for every positive move.
    double max_positive_e_pair = 0.0;
    std::vector<std::string> acceleration_commands;
    std::set<std::string> acceleration_states;
    size_t role_marker_count = 0;
    std::vector<std::string> role_marker_samples;
};

enum RoleIndex : size_t {
    outer = 0,
    inner = 1,
    sparse = 2,
    top = 3,
    internal_solid = 4,
    bottom = 5,
    gap_fill = 6,
    overhang = 7,
};

DynamicPrintConfig matrix_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",              0.2 },
        { "initial_layer_print_height", 0.2 },
        { "wall_loops",                2 },
        { "top_shell_layers",          2 },
        { "bottom_shell_layers",       2 },
        { "top_shell_thickness",       0.0 },
        { "bottom_shell_thickness",    0.0 },
        { "sparse_infill_density",     "20%" },
        { "gcode_comments",            true },
        { "layer_change_gcode",        "G92 E0\n" },
        { "skirt_loops",               0 },
        { "brim_type",                 "no_brim" },
        { "raft_layers",               0 },
        // Role speed, width, and flow oracles do not depend on seam visibility.
        { "seam_position",             "back" },
        // Keep requested feature speeds from being hidden by cooling or the
        // default filament volumetric cap. These are registered FFF options,
        // so the fixture records the actual companion gates it depends on.
        { "slow_down_for_layer_cooling", false },
        { "slow_down_layer_time",       0.0 },
        { "slow_down_min_speed",        1.0 },
        { "fan_cooling_layer_time",     0.0 },
        { "filament_max_volumetric_speed", "200" },
        { "filament_adaptive_volumetric_speed", false },
        { "default_acceleration",      "1200" },
        { "initial_layer_acceleration", "300" },
        { "machine_max_acceleration_extruding", "20000,20000" },
        { "machine_max_acceleration_retracting", "20000,20000" },
        { "machine_max_acceleration_travel",    "20000,20000" },
        { "emit_machine_limits_to_gcode", true },
    });
    return config;
}

TriangleMesh matrix_fixture()
{
    TriangleMesh shape = cube(20.0);
    shape.scale(Vec3f(1.0f, 1.0f, 0.2f));
    return shape;
}

const ConfigOptionDef &require_registry_case(const MatrixCase &test_case, double value)
{
    const ConfigOptionDef *definition = print_config_def.get(test_case.key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == test_case.type);
    // All selected values are absolute values. Percent-valued alternatives are
    // intentionally deferred until a separate ratio partition is registered.
    REQUIRE(definition->is_value_valid(value));
    return *definition;
}

void set_registry_value(DynamicPrintConfig &config, const MatrixCase &test_case, double value)
{
    require_registry_case(test_case, value);
    switch (test_case.type) {
    case coFloats:
        config.set_key_value(test_case.key, new ConfigOptionFloats{ value });
        break;
    case coFloatsOrPercents:
        config.set_key_value(test_case.key, new ConfigOptionFloatsOrPercents{{ FloatOrPercent(value, false) }});
        break;
    case coFloat:
        config.set_key_value(test_case.key, new ConfigOptionFloat(value));
        break;
    case coFloatOrPercent:
        config.set_key_value(test_case.key, new ConfigOptionFloatOrPercent(value, false));
        break;
    case coPercent:
        config.set_key_value(test_case.key, new ConfigOptionPercent(value));
        break;
    default:
        FAIL("Matrix case has an unsupported ConfigOptionType");
    }
    REQUIRE(config.option(test_case.key) != nullptr);
    REQUIRE_FALSE(config.option(test_case.key)->serialize().empty());
}

double collection_length(const ExtrusionEntityCollection &collection)
{
    double length = 0.0;
    for_each_extrusion_path(collection, [&length](const ExtrusionPath &path) { length += path.length(); });
    return length;
}

Evidence evidence_for(const Print &print, const std::string &output)
{
    Evidence evidence;
    REQUIRE_FALSE(print.objects().empty());
    const PrintObject &object = *print.objects().front();
    evidence.layers = object.layers().size();

    for (const Layer *layer : object.layers()) {
        for (const LayerRegion *region : layer->regions()) {
            evidence.perimeter_length += collection_length(region->perimeters);
            evidence.perimeter_items += region->perimeters.items_count();
            for_each_extrusion_path(region->perimeters, [&evidence](const ExtrusionPath &path) {
                if (path.role() == erExternalPerimeter)
                    evidence.roles[outer].geometry_path_length += path.length();
                else if (path.role() == erPerimeter)
                    evidence.roles[inner].geometry_path_length += path.length();
                else if (path.role() == erOverhangPerimeter)
                    evidence.roles[overhang].geometry_path_length += path.length();
            });
            for_each_extrusion_path(region->fills, [&evidence](const ExtrusionPath &path) {
                switch (path.role()) {
                case erExternalPerimeter:
                    evidence.roles[outer].geometry_path_length += path.length();
                    break;
                case erPerimeter:
                    evidence.roles[inner].geometry_path_length += path.length();
                    break;
                case erInternalInfill:
                    evidence.roles[sparse].geometry_path_length += path.length();
                    break;
                case erOverhangPerimeter:
                    evidence.roles[overhang].geometry_path_length += path.length();
                    break;
                case erTopSolidInfill:
                    evidence.roles[top].geometry_path_length += path.length();
                    break;
                case erSolidInfill:
                    evidence.roles[internal_solid].geometry_path_length += path.length();
                    break;
                case erBottomSurface:
                    evidence.roles[bottom].geometry_path_length += path.length();
                    break;
                default:
                    break;
                }
            });
            for_each_extrusion_path(region->thin_fills, [&evidence](const ExtrusionPath &path) {
                if (path.role() == erGapFill)
                    evidence.roles[gap_fill].geometry_path_length += path.length();
            });
        }
    }

    GCodeReader reader;
    reader.apply_config(print.config());
    std::optional<size_t> active_role;
    int layer_index = -1;
    reader.parse_buffer(output, [&evidence, &active_role, &layer_index](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("M204") || line.cmd_is("M201") || line.cmd_is("M205")) {
            evidence.acceleration_commands.emplace_back(line.raw());
            evidence.acceleration_states.emplace(line.raw());
        }

        // A role marker is emitted before the moves in a role block. Depending
        // on printer compatibility, it is either `;TYPE:...` or
        // `; FEATURE: ...`; subsequent G1 lines generally carry no role text.
        // Match the role text itself because the compatible marker includes a
        // leading space and the exporter can select either tag spelling.
        const std::string_view comment = line.comment();
        if (comment.find("LAYER_CHANGE") != std::string_view::npos)
            ++layer_index;
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos) {
            ++evidence.role_marker_count;
            if (evidence.role_marker_samples.size() < 8)
                evidence.role_marker_samples.emplace_back(comment);
        }
        if (comment.find("Overhang wall") != std::string_view::npos)
            active_role = overhang;
        else if (comment.find("Outer wall") != std::string_view::npos)
            active_role = outer;
        else if (comment.find("Inner wall") != std::string_view::npos)
            active_role = inner;
        else if (comment.find("Sparse infill") != std::string_view::npos)
            active_role = sparse;
        else if (comment.find("Top surface") != std::string_view::npos)
            active_role = top;
        else if (comment.find("Internal solid infill") != std::string_view::npos)
            active_role = internal_solid;
        else if (comment.find("Bottom surface") != std::string_view::npos)
            active_role = bottom;
        else if (comment.find("Gap infill") != std::string_view::npos)
            active_role = gap_fill;
        else if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            active_role.reset();
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")))
            return;

        const double xy = line.dist_XY(self);
        // Keep the production predicate and measure its rounded E delta only
        // after the predicate succeeds. This avoids duplicating its semantics.
        const bool extrusion = line.extruding(self) && xy > EPSILON;
        const float delta_e_f = extrusion ? line.dist_E(self) : 0.f;
        double max_pair_abs_e = 0.0;
        if (extrusion) {
            const double previous_e = static_cast<double>(self.e());
            const double current_e = static_cast<double>(line.e());
            max_pair_abs_e = std::max(std::abs(previous_e), std::abs(current_e));
        }
        if (xy > EPSILON || std::abs(line.dist_Z(self)) > EPSILON)
            ++evidence.motion_moves;
        if (extrusion)
            ++evidence.extrusion_moves;
        if (extrusion) {
            evidence.max_positive_e_pair = std::max(evidence.max_positive_e_pair, max_pair_abs_e);
            evidence.gcode_path_length += xy;
            evidence.gcode_filament_length += static_cast<double>(delta_e_f);
        }

        if (active_role.has_value() && xy > EPSILON) {
            RoleEvidence &role_evidence = evidence.roles[*active_role];
            role_evidence.role_motion_path_length += xy;
            ++role_evidence.role_motion_moves;
        }
        if (active_role.has_value() && extrusion) {
            RoleEvidence &role_evidence = evidence.roles[*active_role];
            role_evidence.max_positive_e_pair = std::max(role_evidence.max_positive_e_pair, max_pair_abs_e);
            role_evidence.gcode_path_length += xy;
            role_evidence.filament_length += static_cast<double>(delta_e_f);
            // Most G-code moves inherit F from an earlier move. GCodeLine::f()
            // is zero when F is absent, while new_F() reports the effective
            // feedrate before this callback updates the reader coordinates.
            role_evidence.feedrate = std::max(role_evidence.feedrate, static_cast<double>(line.new_F(self)));
            ++role_evidence.moves;
            if (layer_index == 0) {
                role_evidence.first_layer_filament_length += static_cast<double>(delta_e_f);
                ++role_evidence.first_layer_moves;
            } else if (layer_index > 0) {
                role_evidence.later_layer_filament_length += static_cast<double>(delta_e_f);
                ++role_evidence.later_layer_moves;
            }
        } else if (!extrusion && xy > EPSILON)
            evidence.travel_feedrate = std::max(evidence.travel_feedrate, static_cast<double>(line.f()));
    });
    return evidence;
}

std::string evidence_summary(const Evidence &evidence)
{
    std::ostringstream summary;
    summary << "markers=" << evidence.role_marker_count << " samples=";
    for (const std::string &sample : evidence.role_marker_samples)
        summary << '[' << sample << ']';
    for (size_t i = 0; i < evidence.roles.size(); ++i) {
        const RoleEvidence &role = evidence.roles[i];
        summary << " role" << i << "{moves=" << role.moves
                << ",feed=" << role.feedrate
                << ",gcode=" << role.gcode_path_length
                << ",filament=" << role.filament_length
                << ",max_pair_e=" << role.max_positive_e_pair << '}';
    }
    summary << " totals{gcode=" << evidence.gcode_path_length
            << ",filament=" << evidence.gcode_filament_length
            << ",max_pair_e=" << evidence.max_positive_e_pair
            << ",accel_states=" << evidence.acceleration_states.size() << '}';
    return summary.str();
}

void require_valid(const Evidence &evidence)
{
    REQUIRE(evidence.layers > 0);
    REQUIRE(evidence.perimeter_length > 0.0);
    REQUIRE(evidence.motion_moves > 0);
    REQUIRE(evidence.extrusion_moves > 0);
    REQUIRE(std::isfinite(evidence.max_positive_e_pair));
    for (const RoleEvidence &role : evidence.roles) {
        REQUIRE(std::isfinite(role.geometry_path_length));
        REQUIRE(std::isfinite(role.gcode_path_length));
        REQUIRE(std::isfinite(role.filament_length));
        REQUIRE(std::isfinite(role.max_positive_e_pair));
    }
}

Evidence fresh_evidence(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    TriangleMesh shape = matrix_fixture();
    init_print({ shape }, print, model, config);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    Evidence evidence = evidence_for(print, output);
    require_valid(evidence);
    return evidence;
}

Evidence incremental_evidence(const DynamicPrintConfig &baseline, const DynamicPrintConfig &changed,
                              PrintBase::ApplyStatus &apply_status)
{
    Print print;
    Model model;
    TriangleMesh shape = matrix_fixture();
    init_print({ shape }, print, model, baseline);
    const StringObjectException baseline_validation = print.validate();
    INFO("baseline validation.string: " + baseline_validation.string);
    INFO("baseline validation.opt_key: " + baseline_validation.opt_key);
    REQUIRE(baseline_validation.string.empty());
    REQUIRE_FALSE(gcode(print).empty());

    apply_status = print.apply(model, changed);
    REQUIRE(apply_status != PrintBase::APPLY_STATUS_UNCHANGED);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    Evidence evidence = evidence_for(print, output);
    require_valid(evidence);
    return evidence;
}

void compare_fresh_and_incremental(const Evidence &fresh, const Evidence &incremental)
{
    REQUIRE(fresh.layers == incremental.layers);
    REQUIRE(fresh.perimeter_items == incremental.perimeter_items);
    CHECK_THAT(incremental.perimeter_length, Catch::Matchers::WithinRel(fresh.perimeter_length, 1e-7));
    // Segment counts are not semantic: changing line width can split or join
    // paths in untracked roles while preserving deposited path length.
    CHECK_THAT(incremental.gcode_path_length, Catch::Matchers::WithinRel(fresh.gcode_path_length, 1e-6));
    CHECK_THAT(incremental.gcode_filament_length, Catch::Matchers::WithinRel(fresh.gcode_filament_length, 1e-6));
    CHECK_THAT(incremental.max_positive_e_pair, Catch::Matchers::WithinAbs(fresh.max_positive_e_pair, 1e-5));
    for (size_t i = 0; i < fresh.roles.size(); ++i) {
        CHECK_THAT(incremental.roles[i].geometry_path_length, Catch::Matchers::WithinRel(fresh.roles[i].geometry_path_length, 1e-7));
        CHECK_THAT(incremental.roles[i].gcode_path_length, Catch::Matchers::WithinRel(fresh.roles[i].gcode_path_length, 1e-6));
        CHECK_THAT(incremental.roles[i].filament_length, Catch::Matchers::WithinRel(fresh.roles[i].filament_length, 1e-6));
        CHECK_THAT(incremental.roles[i].max_positive_e_pair, Catch::Matchers::WithinAbs(fresh.roles[i].max_positive_e_pair, 1e-5));
        CHECK_THAT(incremental.roles[i].feedrate, Catch::Matchers::WithinAbs(fresh.roles[i].feedrate, 1e-4));
        CHECK(incremental.roles[i].moves == fresh.roles[i].moves);
    }
    CHECK_THAT(incremental.travel_feedrate, Catch::Matchers::WithinAbs(fresh.travel_feedrate, 1e-4));
    // Path segmentation can repeat the same M204 state. Compare the stable
    // semantic state set rather than incidental repeated command ordering.
    CHECK(incremental.acceleration_states == fresh.acceleration_states);
}

bool changed_signal(const MatrixCase &test_case, const Evidence &baseline, const Evidence &changed)
{
    constexpr double feedrate_tolerance = 1e-3;
    constexpr double geometry_tolerance = 1e-5;
    switch (test_case.signal) {
    case Signal::outer_feedrate:
        return std::abs(changed.roles[outer].feedrate - baseline.roles[outer].feedrate) > feedrate_tolerance;
    case Signal::inner_feedrate:
        return std::abs(changed.roles[inner].feedrate - baseline.roles[inner].feedrate) > feedrate_tolerance;
    case Signal::sparse_feedrate:
        return std::abs(changed.roles[sparse].feedrate - baseline.roles[sparse].feedrate) > feedrate_tolerance;
    case Signal::top_feedrate:
        return std::abs(changed.roles[top].feedrate - baseline.roles[top].feedrate) > feedrate_tolerance;
    case Signal::travel_feedrate:
        return std::abs(changed.travel_feedrate - baseline.travel_feedrate) > feedrate_tolerance;
    case Signal::acceleration_commands:
        return changed.acceleration_commands != baseline.acceleration_commands;
    case Signal::outer_geometry:
        return std::abs(changed.roles[outer].geometry_path_length - baseline.roles[outer].geometry_path_length) > geometry_tolerance ||
               std::abs(changed.roles[outer].filament_length - baseline.roles[outer].filament_length) > geometry_tolerance;
    case Signal::inner_geometry:
        return std::abs(changed.roles[inner].geometry_path_length - baseline.roles[inner].geometry_path_length) > geometry_tolerance ||
               std::abs(changed.roles[inner].filament_length - baseline.roles[inner].filament_length) > geometry_tolerance;
    case Signal::sparse_geometry:
        return std::abs(changed.roles[sparse].geometry_path_length - baseline.roles[sparse].geometry_path_length) > geometry_tolerance ||
               std::abs(changed.roles[sparse].filament_length - baseline.roles[sparse].filament_length) > geometry_tolerance;
    case Signal::all_filament:
        for (size_t i = 0; i < changed.roles.size(); ++i)
            if (std::abs(changed.roles[i].filament_length - baseline.roles[i].filament_length) > geometry_tolerance)
                return true;
        return false;
    }
    return false;
}

std::vector<MatrixCase> first_motion_flow_cases()
{
    return {
        { "outer_wall_speed",       coFloats,          Signal::outer_feedrate, { 30.0, 90.0 } },
        { "inner_wall_speed",       coFloats,          Signal::inner_feedrate, { 35.0, 110.0 } },
        { "sparse_infill_speed",    coFloats,          Signal::sparse_feedrate, { 50.0, 150.0 } },
        { "top_surface_speed",      coFloats,          Signal::top_feedrate, { 40.0, 120.0 } },
        { "travel_speed",           coFloats,          Signal::travel_feedrate, { 70.0, 180.0 } },
        { "outer_wall_acceleration", coFloats,         Signal::acceleration_commands, { 250.0, 1000.0 } },
        { "sparse_infill_acceleration", coFloatsOrPercents, Signal::acceleration_commands, { 500.0, 12000.0 } },
        { "outer_wall_line_width",  coFloatOrPercent,  Signal::outer_geometry, { 0.42, 0.60 } },
        { "inner_wall_line_width",  coFloatOrPercent,  Signal::inner_geometry, { 0.42, 0.60 } },
        { "sparse_infill_line_width", coFloatOrPercent, Signal::sparse_geometry, { 0.55, 0.70 } },
        { "print_flow_ratio",       coFloat,           Signal::all_filament, { 0.95, 1.05 } },
        { "infill_wall_overlap",    coPercent,         Signal::sparse_geometry, { 5.0, 35.0 } },
    };
}

} // namespace

TEST_CASE("Motion and flow settings preserve fresh and incremental equivalence", "[MotionFlowVariance]")
{
    const DynamicPrintConfig baseline = matrix_config();
    const Evidence baseline_evidence = fresh_evidence(baseline);
    for (const MatrixCase &test_case : first_motion_flow_cases()) {
        CAPTURE(test_case.key);

        for (const double value : test_case.values) {
            CAPTURE(value);
            DynamicPrintConfig changed = baseline;
            set_registry_value(changed, test_case, value);
            REQUIRE(changed.option(test_case.key)->serialize() != baseline.option(test_case.key)->serialize());

            const Evidence fresh = fresh_evidence(changed);
            std::optional<Evidence> fresh_repeat;
            if (test_case.signal == Signal::outer_geometry ||
                test_case.signal == Signal::inner_geometry ||
                test_case.signal == Signal::sparse_geometry) {
                fresh_repeat = fresh_evidence(changed);
            }
            PrintBase::ApplyStatus apply_status = PrintBase::APPLY_STATUS_UNCHANGED;
            const Evidence incremental = incremental_evidence(baseline, changed, apply_status);
            INFO("baseline evidence: " + evidence_summary(baseline_evidence));
            INFO("fresh evidence: " + evidence_summary(fresh));
            INFO("incremental evidence: " + evidence_summary(incremental));
            if (fresh_repeat.has_value()) {
                INFO("fresh repeat evidence: " + evidence_summary(*fresh_repeat));
                compare_fresh_and_incremental(fresh, *fresh_repeat);
            }
            CHECK(apply_status == PrintBase::APPLY_STATUS_INVALIDATED);
            compare_fresh_and_incremental(fresh, incremental);
            CHECK(changed_signal(test_case, baseline_evidence, fresh));
        }
    }
}

namespace {

DynamicPrintConfig filament_flow_config()
{
    DynamicPrintConfig config = matrix_config();
    config.set_deserialize_strict({
        // Keep the existing cube fixture (20 x 20 x 4 mm) bounded at 0.2 mm
        // layers, with one explicitly sized filament column.
        { "nozzle_diameter",                 "0.4" },
        { "filament_diameter",               "1.75" },
        { "filament_flow_ratio",             "1.0" },
        { "filament_max_volumetric_speed",   "8.0" },
        { "filament_adaptive_volumetric_speed", false },
        { "volumetric_speed_coefficients",   "" },
        { "outer_wall_speed",                "120" },
        { "inner_wall_speed",                "120" },
        { "sparse_infill_speed",             "120" },
        { "top_surface_speed",               "120" },
        { "use_relative_e_distances",        false },
        // Absolute-E mode rejects a G92 E0 layer hook.  The shared matrix
        // fixture keeps that hook for relative-E tests, so clear it here
        // rather than weakening Print::validate() or changing the oracle.
        { "layer_change_gcode",              "" },
    });
    return config;
}

void require_same_geometry(const Evidence &expected, const Evidence &observed)
{
    REQUIRE(observed.layers == expected.layers);
    REQUIRE(observed.perimeter_items == expected.perimeter_items);
    CHECK_THAT(observed.perimeter_length, Catch::Matchers::WithinRel(expected.perimeter_length, 1e-7));
    CHECK(observed.motion_moves == expected.motion_moves);
    CHECK(observed.extrusion_moves == expected.extrusion_moves);
    CHECK_THAT(observed.gcode_path_length, Catch::Matchers::WithinRel(expected.gcode_path_length, 1e-6));
    for (size_t i = 0; i < expected.roles.size(); ++i) {
        CAPTURE(i);
        CHECK_THAT(observed.roles[i].geometry_path_length,
                   Catch::Matchers::WithinRel(expected.roles[i].geometry_path_length, 1e-7));
        CHECK_THAT(observed.roles[i].gcode_path_length,
                   Catch::Matchers::WithinRel(expected.roles[i].gcode_path_length, 1e-6));
    }
}

void require_same_material(const Evidence &expected, const Evidence &observed)
{
    CHECK_THAT(observed.gcode_filament_length,
               Catch::Matchers::WithinRel(expected.gcode_filament_length, 1e-6));
    for (size_t i = 0; i < expected.roles.size(); ++i) {
        CAPTURE(i);
        CHECK_THAT(observed.roles[i].filament_length,
                   Catch::Matchers::WithinRel(expected.roles[i].filament_length, 1e-6));
    }
}

double minimum_outer_flow_mm3_per_mm(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    TriangleMesh shape = matrix_fixture();
    init_print({ shape }, print, model, config);
    REQUIRE(print.validate().string.empty());
    REQUIRE_FALSE(gcode(print).empty());
    REQUIRE_FALSE(print.objects().empty());

    double minimum = std::numeric_limits<double>::max();
    for (const Layer *layer : print.objects().front()->layers()) {
        for (const LayerRegion *region : layer->regions()) {
            for_each_extrusion_path(region->perimeters, [&minimum](const ExtrusionPath &path) {
                if (path.role() == erExternalPerimeter && path.mm3_per_mm > 0.0)
                    minimum = std::min(minimum, path.mm3_per_mm);
            });
            for_each_extrusion_path(region->fills, [&minimum](const ExtrusionPath &path) {
                if (path.role() == erExternalPerimeter && path.mm3_per_mm > 0.0)
                    minimum = std::min(minimum, path.mm3_per_mm);
            });
        }
    }
    REQUIRE(std::isfinite(minimum));
    REQUIRE(minimum < std::numeric_limits<double>::max());
    return minimum;
}

double outer_feedrate_cap_bound(double cap_mm3_per_s, double filament_flow_ratio,
                                double minimum_flow_mm3_per_mm)
{
    constexpr double requested_speed_mm_per_s = 120.0;
    constexpr double feedrate_per_speed = 60.0;
    const double capped_speed = cap_mm3_per_s / (minimum_flow_mm3_per_mm * filament_flow_ratio);
    return feedrate_per_speed * std::min(requested_speed_mm_per_s, capped_speed);
}

void require_incremental_equivalence(const DynamicPrintConfig &baseline,
                                      const DynamicPrintConfig &changed,
                                      const Evidence &fresh)
{
    PrintBase::ApplyStatus apply_status = PrintBase::APPLY_STATUS_UNCHANGED;
    const Evidence incremental = incremental_evidence(baseline, changed, apply_status);
    REQUIRE(apply_status == PrintBase::APPLY_STATUS_INVALIDATED);
    compare_fresh_and_incremental(fresh, incremental);
}


constexpr double kEExportQuantum = 1e-5; // GCodeFormatter::E_EXPORT_DIGITS == 5.

// For an absolute-E positive move, two decimal coordinates are rounded to
// five digits, each is parsed into float, and dist_E() subtracts those floats
// with a float result. The q term covers two half-quantum decimal errors. The
// 4*epsilon*M term conservatively covers both float conversions and the float
// subtraction, where M is max(|previous E|, |current E|).
double e_pair_error_bound(double max_pair_abs_e)
{
    return kEExportQuantum + 4.0 * std::numeric_limits<float>::epsilon() * max_pair_abs_e;
}

double ratio_material_tolerance(size_t baseline_moves, double baseline_max_pair_e,
                                size_t observed_moves, double observed_max_pair_e,
                                double ratio, double baseline_total, double observed_total)
{
    const double baseline_bound = static_cast<double>(baseline_moves) * e_pair_error_bound(baseline_max_pair_e);
    const double observed_bound = static_cast<double>(observed_moves) * e_pair_error_bound(observed_max_pair_e);
    const double expected = baseline_total * ratio;
    const double arithmetic_guard = 64.0 * std::numeric_limits<double>::epsilon()
                                  * std::max({1.0, std::abs(expected), std::abs(observed_total)});
    return observed_bound + std::abs(ratio) * baseline_bound + arithmetic_guard;
}

void require_ratio_material(const Evidence &baseline, const Evidence &observed, double ratio)
{
    const double expected_total = baseline.gcode_filament_length * ratio;
    const double total_tolerance = ratio_material_tolerance(
        baseline.extrusion_moves, baseline.max_positive_e_pair,
        observed.extrusion_moves, observed.max_positive_e_pair,
        ratio, baseline.gcode_filament_length, observed.gcode_filament_length);
    REQUIRE(std::isfinite(total_tolerance));
    REQUIRE(total_tolerance >= 0.0);
    const double total_signal = std::abs(expected_total - baseline.gcode_filament_length);
    REQUIRE(total_signal > 0.0);
    // A 0.75/1.25 ratio has a 25% signal; this prevents the analytic bound
    // from swallowing the defining flow-ratio behavior.
    CHECK(total_tolerance < total_signal);
    CHECK_THAT(observed.gcode_filament_length,
               Catch::Matchers::WithinAbs(expected_total, total_tolerance));

    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        const double expected = baseline.roles[i].filament_length * ratio;
        const double tolerance = ratio_material_tolerance(
            baseline.roles[i].moves, baseline.roles[i].max_positive_e_pair,
            observed.roles[i].moves, observed.roles[i].max_positive_e_pair,
            ratio, baseline.roles[i].filament_length, observed.roles[i].filament_length);
        REQUIRE(std::isfinite(tolerance));
        REQUIRE(tolerance >= 0.0);
        CHECK_THAT(observed.roles[i].filament_length,
                   Catch::Matchers::WithinAbs(expected, tolerance));
        if (baseline.roles[i].filament_length > 0.0) {
            const double signal = std::abs(expected - baseline.roles[i].filament_length);
            CHECK(signal > 0.0);
            CHECK(tolerance < signal);
        }
    }
}

} // namespace

TEST_CASE("Filament flow ratio changes capped feedrate without changing deposited geometry",
          "[MotionFlowVariance][FilamentFlowVariance]")
{
    const DynamicPrintConfig baseline = filament_flow_config();
    const Evidence baseline_evidence = fresh_evidence(baseline);
    const double minimum_outer_flow = minimum_outer_flow_mm3_per_mm(baseline);
    REQUIRE(baseline_evidence.roles[outer].moves > 0);
    REQUIRE(baseline_evidence.roles[outer].feedrate > 0.0);

    const MatrixCase flow_case{
        "filament_flow_ratio", coFloats, Signal::outer_feedrate, { 0.75, 1.25 }
    };
    for (const double value : flow_case.values) {
        CAPTURE(value);
        DynamicPrintConfig changed = baseline;
        set_registry_value(changed, flow_case, value);
        REQUIRE(changed.option(flow_case.key)->serialize() != baseline.option(flow_case.key)->serialize());

        const Evidence fresh = fresh_evidence(changed);
        INFO("baseline evidence: " + evidence_summary(baseline_evidence));
        INFO("fresh evidence: " + evidence_summary(fresh));
        require_same_geometry(baseline_evidence, fresh);
        require_ratio_material(baseline_evidence, fresh, value);
        require_incremental_equivalence(baseline, changed, fresh);
        const double cap_bound = outer_feedrate_cap_bound(8.0, value, minimum_outer_flow);
        CHECK(fresh.roles[outer].feedrate <= cap_bound + 1e-3);

        if (value < 1.0)
            CHECK(fresh.roles[outer].feedrate > baseline_evidence.roles[outer].feedrate);
        else
            CHECK(fresh.roles[outer].feedrate < baseline_evidence.roles[outer].feedrate);
    }
}

TEST_CASE("Filament volumetric cap changes feedrate without changing deposited geometry",
          "[MotionFlowVariance][FilamentMaxVolumetricSpeed]")
{
    const DynamicPrintConfig baseline = filament_flow_config();
    const Evidence baseline_evidence = fresh_evidence(baseline);
    const double minimum_outer_flow = minimum_outer_flow_mm3_per_mm(baseline);
    REQUIRE(baseline_evidence.roles[outer].moves > 0);
    REQUIRE(baseline_evidence.roles[outer].feedrate > 0.0);

    const MatrixCase cap_case{
        "filament_max_volumetric_speed", coFloats, Signal::outer_feedrate, { 1.5, 20.0 }
    };
    for (const double value : cap_case.values) {
        CAPTURE(value);
        DynamicPrintConfig changed = baseline;
        set_registry_value(changed, cap_case, value);
        REQUIRE(changed.option(cap_case.key)->serialize() != baseline.option(cap_case.key)->serialize());

        const Evidence fresh = fresh_evidence(changed);
        INFO("baseline evidence: " + evidence_summary(baseline_evidence));
        INFO("fresh evidence: " + evidence_summary(fresh));
        require_same_geometry(baseline_evidence, fresh);
        require_same_material(baseline_evidence, fresh);
        require_incremental_equivalence(baseline, changed, fresh);
        const double cap_bound = outer_feedrate_cap_bound(value, 1.0, minimum_outer_flow);
        CHECK(fresh.roles[outer].feedrate <= cap_bound + 1e-3);

        if (value < 8.0)
            CHECK(fresh.roles[outer].feedrate < baseline_evidence.roles[outer].feedrate);
        else
            CHECK(fresh.roles[outer].feedrate > baseline_evidence.roles[outer].feedrate);
    }
}


namespace {

DynamicPrintConfig role_flow_config()
{
    DynamicPrintConfig config = matrix_config();
    config.set_deserialize_strict({
        { "wall_generator",              "classic" },
        { "outer_wall_line_width",       0.42 },
        { "inner_wall_line_width",       0.45 },
        { "detect_thin_wall",            false },
        { "set_other_flow_ratios",       true },
        { "first_layer_flow_ratio",      1.0 },
        { "top_solid_infill_flow_ratio",  "1.0" },
        { "bottom_solid_infill_flow_ratio", 1.0 },
        { "internal_solid_infill_flow_ratio", 1.0 },
        { "gap_fill_flow_ratio",         1.0 },
        { "use_relative_e_distances",    true },
        { "minimum_sparse_infill_area",  0.0 },
        { "filter_out_gap_fill",        0.0 },
        { "extra_solid_infills",         "3" },
        { "sparse_infill_density",       "20%" },
        { "wall_loops",                  2 },
        { "top_shell_layers",            2 },
        { "bottom_shell_layers",         2 },
        { "layer_height",                0.2 },
        { "initial_layer_print_height",  0.2 },
        { "gcode_comments",              true },
    });
    // Pin positive gap speed instead of relying on profile-sized nullable defaults.
    config.set_key_value("gap_infill_speed", new ConfigOptionFloatsNullable({ 30.0 }));
    return config;
}

void init_role_flow_fixture(Print &print, Model &model, const DynamicPrintConfig &config)
{
    // Reuse init_print for the primary object so evidence_for() sees every
    // volume through its single front object. The separate 1mm-wall cavity is
    // based on test_perimeters.cpp:square_tube and supplies a real erGapFill
    // region without removing the solid island's sparse-fill witness.
    init_print({ make_cube(10.0, 10.0, 1.2) }, print, model, config);
    ModelObject *object = model.objects.front();
    TriangleMesh outer = make_cube(20.0, 20.0, 1.2);
    outer.translate(14.0f, 0.0f, 0.0f);
    object->add_volume(std::move(outer), ModelVolumeType::MODEL_PART, false);
    TriangleMesh cavity = make_cube(18.0, 18.0, 1.2);
    cavity.translate(15.0f, 1.0f, 0.0f);
    object->add_volume(std::move(cavity), ModelVolumeType::NEGATIVE_VOLUME, false);
    print.auto_assign_extruders(object);
    print.apply(model, config);
    print.set_status_silent();
}

void require_bounded_role_output(const std::string &output)
{
    constexpr size_t max_output_bytes = 16u * 1024u * 1024u;
    constexpr size_t max_output_lines = 250000u;
    REQUIRE(output.size() <= max_output_bytes);
    const size_t output_lines = static_cast<size_t>(std::count(output.begin(), output.end(), '\n'));
    REQUIRE(output_lines <= max_output_lines);
}

Evidence fresh_role_flow_evidence(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_role_flow_fixture(print, model, config);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    require_bounded_role_output(output);
    Evidence evidence = evidence_for(print, output);
    require_valid(evidence);
    REQUIRE(evidence.roles[gap_fill].geometry_path_length > 0.);
    return evidence;
}

void require_same_material_section(size_t expected_moves, double expected_length, double expected_max_pair_e,
                                   size_t observed_moves, double observed_length, double observed_max_pair_e)
{
    CHECK(observed_moves == expected_moves);
    if (expected_moves == 0) {
        CHECK_THAT(observed_length, Catch::Matchers::WithinAbs(0.0, 1e-9));
        return;
    }
    REQUIRE(expected_length > 0.0);
    const double tolerance = ratio_material_tolerance(
        expected_moves, expected_max_pair_e, observed_moves, observed_max_pair_e,
        1.0, expected_length, observed_length);
    REQUIRE(std::isfinite(tolerance));
    REQUIRE(tolerance >= 0.0);
    CHECK_THAT(observed_length, Catch::Matchers::WithinAbs(expected_length, tolerance));
}

void require_scaled_material_section(size_t expected_moves, double expected_length, double expected_max_pair_e,
                                     size_t observed_moves, double observed_length, double observed_max_pair_e,
                                     double ratio)
{
    REQUIRE(expected_moves > 0);
    REQUIRE(observed_moves == expected_moves);
    REQUIRE(expected_length > 0.0);
    const double tolerance = ratio_material_tolerance(
        expected_moves, expected_max_pair_e, observed_moves, observed_max_pair_e,
        ratio, expected_length, observed_length);
    REQUIRE(std::isfinite(tolerance));
    REQUIRE(tolerance >= 0.0);
    const double expected_length_after_ratio = expected_length * ratio;
    const double signal = std::abs(expected_length_after_ratio - expected_length);
    REQUIRE(signal > 0.0);
    CHECK(tolerance < signal);
    CHECK_THAT(observed_length,
               Catch::Matchers::WithinAbs(expected_length_after_ratio, tolerance));
}

void require_same_role_material(const RoleEvidence &expected, const RoleEvidence &observed)
{
    require_same_material_section(expected.moves, expected.filament_length, expected.max_positive_e_pair,
                                   observed.moves, observed.filament_length, observed.max_positive_e_pair);
    require_same_material_section(expected.first_layer_moves, expected.first_layer_filament_length,
                                   expected.max_positive_e_pair, observed.first_layer_moves,
                                   observed.first_layer_filament_length, observed.max_positive_e_pair);
    require_same_material_section(expected.later_layer_moves, expected.later_layer_filament_length,
                                   expected.max_positive_e_pair, observed.later_layer_moves,
                                   observed.later_layer_filament_length, observed.max_positive_e_pair);
}

void require_scaled_role_material(const RoleEvidence &expected, const RoleEvidence &observed, double ratio)
{
    REQUIRE(expected.moves > 0);
    REQUIRE(observed.moves == expected.moves);
    if (expected.first_layer_moves > 0)
        require_scaled_material_section(expected.first_layer_moves, expected.first_layer_filament_length,
                                        expected.max_positive_e_pair, observed.first_layer_moves,
                                        observed.first_layer_filament_length, observed.max_positive_e_pair, ratio);
    else
        require_same_material_section(expected.first_layer_moves, expected.first_layer_filament_length,
                                      expected.max_positive_e_pair, observed.first_layer_moves,
                                      observed.first_layer_filament_length, observed.max_positive_e_pair);
    if (expected.later_layer_moves > 0)
        require_scaled_material_section(expected.later_layer_moves, expected.later_layer_filament_length,
                                        expected.max_positive_e_pair, observed.later_layer_moves,
                                        observed.later_layer_filament_length, observed.max_positive_e_pair, ratio);
    else
        require_same_material_section(expected.later_layer_moves, expected.later_layer_filament_length,
                                      expected.max_positive_e_pair, observed.later_layer_moves,
                                      observed.later_layer_filament_length, observed.max_positive_e_pair);
    require_scaled_material_section(expected.moves, expected.filament_length, expected.max_positive_e_pair,
                                    observed.moves, observed.filament_length, observed.max_positive_e_pair, ratio);
}

void require_first_layer_ratio(const Evidence &baseline, const Evidence &changed, double ratio)
{
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        const RoleEvidence &before = baseline.roles[i];
        const RoleEvidence &after = changed.roles[i];
        if (before.first_layer_moves > 0)
            require_scaled_material_section(before.first_layer_moves, before.first_layer_filament_length,
                                            before.max_positive_e_pair, after.first_layer_moves,
                                            after.first_layer_filament_length, after.max_positive_e_pair, ratio);
        else
            require_same_material_section(before.first_layer_moves, before.first_layer_filament_length,
                                          before.max_positive_e_pair, after.first_layer_moves,
                                          after.first_layer_filament_length, after.max_positive_e_pair);
        require_same_material_section(before.later_layer_moves, before.later_layer_filament_length,
                                      before.max_positive_e_pair, after.later_layer_moves,
                                      after.later_layer_filament_length, after.max_positive_e_pair);
    }
}

void require_role_ratio(const Evidence &baseline, const Evidence &changed, size_t target_role, double ratio)
{
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        const RoleEvidence &before = baseline.roles[i];
        const RoleEvidence &after = changed.roles[i];
        if (i == target_role)
            require_scaled_role_material(before, after, ratio);
        else
            require_same_role_material(before, after);
    }
}

struct RoleFlowCase {
    MatrixCase matrix;
    size_t target_role;
    bool first_layer_only;
};

std::array<RoleFlowCase, 5> role_flow_cases()
{
    return {{
        { { "first_layer_flow_ratio",          coFloat,  Signal::all_filament, { 0.8, 1.2 } }, outer, true },
        { { "top_solid_infill_flow_ratio",     coFloats, Signal::all_filament, { 0.8, 1.2 } }, top, false },
        { { "bottom_solid_infill_flow_ratio",  coFloat,  Signal::all_filament, { 0.8, 1.2 } }, bottom, false },
        { { "internal_solid_infill_flow_ratio",coFloat,  Signal::all_filament, { 0.8, 1.2 } }, internal_solid, false },
        { { "gap_fill_flow_ratio",             coFloat,  Signal::all_filament, { 0.8, 1.2 } }, gap_fill, false },
    }};
}

struct NamedRoleFactors {
    double outer;
    double inner;
    double sparse;
    double top;
    double internal_solid;
    double bottom;
    double gap_fill;
};

void require_role_flow_registry()
{
    const ConfigOptionDef *gate = print_config_def.get("set_other_flow_ratios");
    REQUIRE(gate != nullptr);
    REQUIRE(gate->type == coBool);
    for (const RoleFlowCase &flow_case : role_flow_cases())
        require_registry_case(flow_case.matrix, 1.0);
}

void require_changed_role_flow_options(const DynamicPrintConfig &baseline,
                                        const DynamicPrintConfig &changed)
{
    const auto require_changed = [&baseline, &changed](const char *key) {
        REQUIRE(baseline.option(key) != nullptr);
        REQUIRE(changed.option(key) != nullptr);
        REQUIRE(changed.option(key)->serialize() != baseline.option(key)->serialize());
    };
    require_changed("set_other_flow_ratios");
    for (const RoleFlowCase &flow_case : role_flow_cases())
        require_changed(flow_case.matrix.key);
}

void require_role_section_factor(const RoleEvidence &expected, const RoleEvidence &observed,
                                 double ratio, bool first_layer, const char *role_name)
{
    INFO("role: " + std::string(role_name));
    const size_t expected_moves = first_layer ? expected.first_layer_moves : expected.later_layer_moves;
    const double expected_length = first_layer ? expected.first_layer_filament_length
                                               : expected.later_layer_filament_length;
    const size_t observed_moves = first_layer ? observed.first_layer_moves : observed.later_layer_moves;
    const double observed_length = first_layer ? observed.first_layer_filament_length
                                               : observed.later_layer_filament_length;
    if (expected_moves == 0 || std::abs(ratio - 1.0) <= 1e-12) {
        require_same_material_section(expected_moves, expected_length, expected.max_positive_e_pair,
                                      observed_moves, observed_length, observed.max_positive_e_pair);
    } else {
        require_scaled_material_section(expected_moves, expected_length, expected.max_positive_e_pair,
                                        observed_moves, observed_length, observed.max_positive_e_pair, ratio);
    }
}

void require_named_role_factors(const Evidence &baseline, const Evidence &changed,
                                const NamedRoleFactors &first_layer, const NamedRoleFactors &later_layer)
{
    const auto require_role = [&baseline, &changed](
                                  size_t role, double first_ratio, double later_ratio, const char *name) {
        const RoleEvidence &expected = baseline.roles[role];
        const RoleEvidence &observed = changed.roles[role];
        REQUIRE(observed.moves == expected.moves);
        require_role_section_factor(expected, observed, first_ratio, true, name);
        require_role_section_factor(expected, observed, later_ratio, false, name);
    };

    // Spell out the role mapping so a future enum insertion cannot silently
    // move a factor to a different semantic role.
    require_role(outer, first_layer.outer, later_layer.outer, "outer");
    require_role(inner, first_layer.inner, later_layer.inner, "inner");
    require_role(sparse, first_layer.sparse, later_layer.sparse, "sparse");
    require_role(top, first_layer.top, later_layer.top, "top");
    require_role(internal_solid, first_layer.internal_solid, later_layer.internal_solid, "internal_solid");
    require_role(bottom, first_layer.bottom, later_layer.bottom, "bottom");
    require_role(gap_fill, first_layer.gap_fill, later_layer.gap_fill, "gap_fill");
}

} // namespace

TEST_CASE("Role flow ratios scale declared deposited material by role and layer",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    const DynamicPrintConfig baseline_config = role_flow_config();
    const Evidence baseline = fresh_role_flow_evidence(baseline_config);
    for (const size_t role : std::array<size_t, 7>{
             outer, inner, sparse, top, internal_solid, bottom, gap_fill }) {
        CAPTURE(role);
        REQUIRE(baseline.roles[role].moves > 0);
        REQUIRE(baseline.roles[role].filament_length > 0.0);
    }
    REQUIRE(baseline.roles[outer].first_layer_moves > 0);
    REQUIRE(baseline.roles[outer].later_layer_moves > 0);
    REQUIRE(baseline.roles[bottom].first_layer_moves > 0);
    REQUIRE(baseline.roles[top].later_layer_moves > 0);

    for (const RoleFlowCase &flow_case : role_flow_cases()) {
        CAPTURE(flow_case.matrix.key);
        for (const double value : flow_case.matrix.values) {
            CAPTURE(value);
            DynamicPrintConfig changed_config = baseline_config;
            set_registry_value(changed_config, flow_case.matrix, value);
            REQUIRE(changed_config.option(flow_case.matrix.key) != nullptr);
            REQUIRE(changed_config.option(flow_case.matrix.key)->serialize()
                    != baseline_config.option(flow_case.matrix.key)->serialize());
            const Evidence changed = fresh_role_flow_evidence(changed_config);
            INFO("baseline evidence: " + evidence_summary(baseline));
            INFO("changed evidence: " + evidence_summary(changed));
            require_same_geometry(baseline, changed);
            if (flow_case.first_layer_only)
                require_first_layer_ratio(baseline, changed, value);
            else
                require_role_ratio(baseline, changed, flow_case.target_role, value);
        }
    }
}

TEST_CASE("Role flow gate leaves top and bottom ratios active while disabling other ratios",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_role_flow_registry();
    const DynamicPrintConfig baseline_config = role_flow_config();
    const Evidence baseline = fresh_role_flow_evidence(baseline_config);
    for (const size_t role : std::array<size_t, 7>{
             outer, inner, sparse, top, internal_solid, bottom, gap_fill }) {
        CAPTURE(role);
        REQUIRE(baseline.roles[role].moves > 0);
        REQUIRE(baseline.roles[role].filament_length > 0.0);
    }
    REQUIRE(baseline.roles[bottom].first_layer_moves > 0);
    // Later shell layers on this uniform solid have the internal-solid role.
    REQUIRE(baseline.roles[internal_solid].later_layer_moves > 0);
    REQUIRE(baseline.roles[top].later_layer_moves > 0);
    REQUIRE(baseline.roles[outer].first_layer_moves > 0);
    REQUIRE(baseline.roles[outer].later_layer_moves > 0);

    DynamicPrintConfig changed_config = baseline_config;
    changed_config.set_deserialize_strict({
        { "set_other_flow_ratios",          false },
        { "first_layer_flow_ratio",         0.7 },
        { "top_solid_infill_flow_ratio",    "1.2" },
        { "bottom_solid_infill_flow_ratio", 1.2 },
        { "internal_solid_infill_flow_ratio", 0.7 },
        { "gap_fill_flow_ratio",            0.7 },
    });
    require_changed_role_flow_options(baseline_config, changed_config);
    const Evidence changed = fresh_role_flow_evidence(changed_config);
    INFO("baseline evidence: " + evidence_summary(baseline));
    INFO("changed evidence: " + evidence_summary(changed));
    require_same_geometry(baseline, changed);
    const NamedRoleFactors expected{ 1.0, 1.0, 1.0, 1.2, 1.0, 1.2, 1.0 };
    require_named_role_factors(baseline, changed, expected, expected);
}

TEST_CASE("Role flow gate compounds first-layer and role ratios",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_role_flow_registry();
    const DynamicPrintConfig baseline_config = role_flow_config();
    const Evidence baseline = fresh_role_flow_evidence(baseline_config);
    for (const size_t role : std::array<size_t, 7>{
             outer, inner, sparse, top, internal_solid, bottom, gap_fill }) {
        CAPTURE(role);
        REQUIRE(baseline.roles[role].moves > 0);
        REQUIRE(baseline.roles[role].filament_length > 0.0);
    }
    REQUIRE(baseline.roles[bottom].first_layer_moves > 0);
    // Later shell layers on this uniform solid have the internal-solid role.
    REQUIRE(baseline.roles[internal_solid].later_layer_moves > 0);
    REQUIRE(baseline.roles[top].later_layer_moves > 0);
    REQUIRE(baseline.roles[outer].first_layer_moves > 0);
    REQUIRE(baseline.roles[outer].later_layer_moves > 0);

    DynamicPrintConfig changed_config = baseline_config;
    changed_config.set_deserialize_strict({
        { "set_other_flow_ratios",          true },
        { "first_layer_flow_ratio",         0.8 },
        { "top_solid_infill_flow_ratio",    "1.1" },
        { "bottom_solid_infill_flow_ratio", 1.2 },
        { "internal_solid_infill_flow_ratio", 0.7 },
        { "gap_fill_flow_ratio",            1.4 },
    });
    // The gate remains true in both configs; the five ratio serializations
    // must still differ so this case cannot silently become a baseline rerun.
    for (const RoleFlowCase &flow_case : role_flow_cases()) {
        REQUIRE(changed_config.option(flow_case.matrix.key) != nullptr);
        REQUIRE(changed_config.option(flow_case.matrix.key)->serialize()
                != baseline_config.option(flow_case.matrix.key)->serialize());
    }
    const Evidence changed = fresh_role_flow_evidence(changed_config);
    INFO("baseline evidence: " + evidence_summary(baseline));
    INFO("changed evidence: " + evidence_summary(changed));
    require_same_geometry(baseline, changed);
    const NamedRoleFactors first_layer{ 0.8, 0.8, 0.8, 0.88, 0.56, 0.96, 1.12 };
    const NamedRoleFactors later_layer{ 1.0, 1.0, 1.0, 1.1, 0.7, 1.2, 1.4 };
    require_named_role_factors(baseline, changed, first_layer, later_layer);
}


namespace {

struct ObjectSpeedTransition {
    Evidence nil_initial;
    Evidence explicit_value;
    Evidence nil_final;
};

DynamicPrintConfig speed_fixture_settings(DynamicPrintConfig config)
{
    // Keep both single- and multi-filament fixtures on the same explicit,
    // deterministic pipeline settings. In particular, do not rely on the
    // multifilament helper's full-config defaults for cooling or acceleration.
    config.set_deserialize_strict({
        { "layer_height",                       0.2 },
        { "initial_layer_print_height",         0.2 },
        { "wall_loops",                           2 },
        { "top_shell_layers",                     2 },
        { "bottom_shell_layers",                  2 },
        { "top_shell_thickness",                  0.0 },
        { "bottom_shell_thickness",               0.0 },
        { "sparse_infill_density",              "20%" },
        { "gcode_comments",                      true },
        { "layer_change_gcode",         "G92 E0\n" },
        { "skirt_loops",                           0 },
        { "brim_type",                     "no_brim" },
        { "raft_layers",                          0 },
        { "slow_down_for_layer_cooling",       false },
        { "slow_down_layer_time",                0.0 },
        { "slow_down_min_speed",                 1.0 },
        { "fan_cooling_layer_time",              0.0 },
        { "filament_max_volumetric_speed",     200.0 },
        { "filament_adaptive_volumetric_speed", false },
        { "default_acceleration",             "1200" },
        { "initial_layer_acceleration",        "300" },
        { "machine_max_acceleration_extruding", "20000,20000" },
        { "machine_max_acceleration_retracting", "20000,20000" },
        { "machine_max_acceleration_travel",    "20000,20000" },
        { "emit_machine_limits_to_gcode",       true },
        { "initial_layer_speed",                 5.0 },
        { "initial_layer_infill_speed",          7.0 },
        { "enable_overhang_speed",              false },
        { "slowdown_for_curled_perimeters",     false },
        { "small_perimeter_threshold",            0.0 },
        { "small_support_perimeter_threshold",    0.0 },
        { "single_extruder_multi_material",     false },
        { "enable_prime_tower",                 false },
    });
    return config;
}

DynamicPrintConfig object_speed_config()
{
    DynamicPrintConfig config = speed_fixture_settings(matrix_config());
    config.set_deserialize_strict({
        { "outer_wall_speed",    60.0 },
        { "inner_wall_speed",    70.0 },
        { "sparse_infill_speed", 80.0 },
        { "top_surface_speed",   90.0 },
    });
    config.set_key_value("outer_wall_speed",    new ConfigOptionFloatsNullable({ 60.0 }));
    config.set_key_value("inner_wall_speed",    new ConfigOptionFloatsNullable({ 70.0 }));
    config.set_key_value("sparse_infill_speed", new ConfigOptionFloatsNullable({ 80.0 }));
    config.set_key_value("top_surface_speed",   new ConfigOptionFloatsNullable({ 90.0 }));
    return config;
}

DynamicPrintConfig variant_object_speed_config()
{
    DynamicPrintConfig config = speed_fixture_settings(multifilament_config(2));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({ 0.4, 0.4 }));
    config.set_key_value("extruder_printable_height", new ConfigOptionFloats({ 0.0, 0.0 }));
    config.set_key_value("extruder_type", new ConfigOptionEnumsGeneric({ etDirectDrive, etDirectDrive }));
    config.set_key_value("nozzle_volume_type", new ConfigOptionEnumsGeneric({ nvtStandard, nvtHighFlow }));
    config.set_key_value("extruder_variant_list", new ConfigOptionStrings({ "Direct Drive Standard", "Direct Drive High Flow" }));
    config.set_key_value("printer_extruder_id", new ConfigOptionInts({ 1, 2 }));
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings({ "Direct Drive Standard", "Direct Drive High Flow" }));
    config.set_key_value("print_extruder_id", new ConfigOptionInts({ 1, 2 }));
    config.set_key_value("print_extruder_variant", new ConfigOptionStrings({ "Direct Drive Standard", "Direct Drive High Flow" }));
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings({ "Direct Drive Standard", "Direct Drive High Flow" }));
    config.set_key_value("filament_self_index", new ConfigOptionInts({ 1, 2 }));
    config.set_key_value("filament_map", new ConfigOptionInts({ 1, 2 }));
    config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
    config.set_key_value("outer_wall_filament_id", new ConfigOptionInt(2));
    config.set_key_value("inner_wall_filament_id", new ConfigOptionInt(2));
    config.set_key_value("sparse_infill_filament_id", new ConfigOptionInt(2));
    config.set_key_value("top_surface_filament_id", new ConfigOptionInt(2));
    config.set_key_value("bottom_surface_filament_id", new ConfigOptionInt(2));
    config.set_key_value("internal_solid_filament_id", new ConfigOptionInt(2));
    config.set_key_value("outer_wall_speed",    new ConfigOptionFloatsNullable({ 60.0, 90.0 }));
    config.set_key_value("inner_wall_speed",    new ConfigOptionFloatsNullable({ 70.0, 100.0 }));
    config.set_key_value("sparse_infill_speed", new ConfigOptionFloatsNullable({ 80.0, 110.0 }));
    config.set_key_value("top_surface_speed",   new ConfigOptionFloatsNullable({ 90.0, 120.0 }));
    return config;
}

Evidence apply_object_speed_state(Print &print, Model &model, ModelObject &object,
                                  const DynamicPrintConfig &config, const char *key,
                                  const ConfigOptionFloatsNullable &override_value)
{
    // set_key_value replaces an existing model option and returns false in
    // that case; the public behavior under test is the subsequent Print::apply.
    object.config.set_key_value(key, new ConfigOptionFloatsNullable(override_value));
    REQUIRE(object.config.option(key) != nullptr);
    (void) print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());
    Evidence evidence = evidence_for(print, output);
    require_valid(evidence);
    return evidence;
}

ObjectSpeedTransition apply_object_speed_nil_explicit_nil(const DynamicPrintConfig &config, const char *key,
                                                          double explicit_value)
{
    Print print;
    Model model;
    init_print({ matrix_fixture() }, print, model, config);
    REQUIRE_FALSE(model.objects.empty());
    ModelObject *object = model.objects.front();
    REQUIRE(object != nullptr);

    ObjectSpeedTransition transition;
    transition.nil_initial = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ ConfigOptionFloatsNullable::nil_value() }));
    transition.explicit_value = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ explicit_value }));
    transition.nil_final = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ ConfigOptionFloatsNullable::nil_value() }));
    return transition;
}

} // namespace

TEST_CASE("Nullable object speed overrides restore inherited G-code feeds",
          "[MotionFlowVariance][ObjectSpeedNullable]")
{
    struct SpeedCase { const char *key; size_t role; double explicit_value; };
    const std::array<SpeedCase, 4> cases{{
        { "outer_wall_speed", outer, 25.0 },
        { "inner_wall_speed", inner, 35.0 },
        { "sparse_infill_speed", sparse, 45.0 },
        { "top_surface_speed", top, 55.0 },
    }};
    const std::array<DynamicPrintConfig, 2> fixtures{{
        object_speed_config(), variant_object_speed_config()
    }};

    for (size_t fixture_index = 0; fixture_index < fixtures.size(); ++fixture_index) {
        const DynamicPrintConfig &parent_config = fixtures[fixture_index];
        const bool uses_variants = fixture_index == 1;
        const Evidence parent = fresh_evidence(parent_config);
        for (const SpeedCase &speed_case : cases) {
            CAPTURE(fixture_index, speed_case.key);
            REQUIRE(parent.roles[speed_case.role].later_layer_moves > 0);
            REQUIRE(parent.roles[speed_case.role].later_layer_filament_length > 0.0);
            REQUIRE(parent.roles[speed_case.role].feedrate > 0.0);

            DynamicPrintConfig explicit_global = parent_config;
            if (uses_variants)
                explicit_global.set_key_value(speed_case.key,
                    new ConfigOptionFloatsNullable({ speed_case.explicit_value, speed_case.explicit_value }));
            else
                explicit_global.set_key_value(speed_case.key,
                    new ConfigOptionFloatsNullable({ speed_case.explicit_value }));
            const Evidence expected_explicit = fresh_evidence(explicit_global);
            REQUIRE(std::abs(expected_explicit.roles[speed_case.role].feedrate
                             - parent.roles[speed_case.role].feedrate) > 1e-3);

            const ObjectSpeedTransition transition = apply_object_speed_nil_explicit_nil(
                parent_config, speed_case.key, speed_case.explicit_value);
            compare_fresh_and_incremental(parent, transition.nil_initial);
            compare_fresh_and_incremental(expected_explicit, transition.explicit_value);
            compare_fresh_and_incremental(parent, transition.nil_final);
        }
    }
}

namespace {

void set_nullable_role_speed(DynamicPrintConfig &config, const char *key, double value)
{
    const ConfigOptionDef *definition = print_config_def.get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == coFloats);
    REQUIRE(definition->nullable);
    REQUIRE(definition->is_value_valid(value));
    config.set_key_value(key, new ConfigOptionFloatsNullable({ value }));
}

DynamicPrintConfig role_speed_contract_config()
{
    DynamicPrintConfig config = role_flow_config();
    // Keep all target-role G-code speeds observable: first-layer infill uses
    // this speed before the role-specific value takes effect on later layers.
    set_nullable_role_speed(config, "initial_layer_infill_speed", 1.0);
    set_nullable_role_speed(config, "sparse_infill_speed", 91.0);
    set_nullable_role_speed(config, "internal_solid_infill_speed", 62.0);
    set_nullable_role_speed(config, "gap_infill_speed", 46.0);
    return config;
}

ObjectSpeedTransition apply_role_flow_object_speed_nil_explicit_nil(
    const DynamicPrintConfig &config, const char *key, double explicit_value)
{
    Print print;
    Model model;
    init_role_flow_fixture(print, model, config);
    REQUIRE_FALSE(model.objects.empty());
    ModelObject *object = model.objects.front();
    REQUIRE(object != nullptr);

    ObjectSpeedTransition transition;
    transition.nil_initial = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ ConfigOptionFloatsNullable::nil_value() }));
    transition.explicit_value = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ explicit_value }));
    transition.nil_final = apply_object_speed_state(
        print, model, *object, config, key,
        ConfigOptionFloatsNullable({ ConfigOptionFloatsNullable::nil_value() }));
    return transition;
}

void require_role_speed_output(const Evidence &evidence, size_t role, double configured_mm_per_second)
{
    const RoleEvidence &observed = evidence.roles[role];
    REQUIRE(observed.moves > 0);
    REQUIRE(observed.later_layer_moves > 0);
    REQUIRE(observed.geometry_path_length > 0.0);
    REQUIRE(observed.gcode_path_length > 0.0);
    REQUIRE(observed.filament_length > 0.0);
    // G-code F is expressed in mm/min; GCode::extrude() converts mm/s by 60.
    CHECK_THAT(observed.feedrate,
               Catch::Matchers::WithinAbs(configured_mm_per_second * 60.0, 1.0));
}

void verify_nullable_role_speed_contract(
    const char *key, size_t role,
    const DynamicPrintConfig &base_config = role_speed_contract_config())
{
    constexpr double representative_speed = 52.0;
    constexpr double explicit_object_speed = 24.0;

    const ConfigOptionDef *definition = print_config_def.get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == coFloats);
    REQUIRE(definition->nullable);
    const double minimum = definition->min;
    REQUIRE(std::isfinite(minimum));
    REQUIRE(definition->is_value_valid(minimum));

    DynamicPrintConfig boundary_config = base_config;
    set_nullable_role_speed(boundary_config, key, minimum);
    const Evidence boundary = fresh_role_flow_evidence(boundary_config);
    INFO("registered minimum output: " + evidence_summary(boundary));
    require_role_speed_output(boundary, role, minimum);

    DynamicPrintConfig parent_config = base_config;
    set_nullable_role_speed(parent_config, key, representative_speed);
    const Evidence parent = fresh_role_flow_evidence(parent_config);
    INFO("inherited output: " + evidence_summary(parent));
    require_role_speed_output(parent, role, representative_speed);
    require_same_geometry(parent, boundary);
    require_same_material(parent, boundary);

    DynamicPrintConfig explicit_global_config = parent_config;
    set_nullable_role_speed(explicit_global_config, key, explicit_object_speed);
    const Evidence explicit_global = fresh_role_flow_evidence(explicit_global_config);
    require_role_speed_output(explicit_global, role, explicit_object_speed);

    const ObjectSpeedTransition transition = apply_role_flow_object_speed_nil_explicit_nil(
        parent_config, key, explicit_object_speed);
    compare_fresh_and_incremental(parent, transition.nil_initial);
    compare_fresh_and_incremental(explicit_global, transition.explicit_value);
    compare_fresh_and_incremental(parent, transition.nil_final);
    require_role_speed_output(transition.nil_initial, role, representative_speed);
    require_role_speed_output(transition.explicit_value, role, explicit_object_speed);
    require_role_speed_output(transition.nil_final, role, representative_speed);
}

} // namespace

TEST_CASE("Internal solid infill speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][InternalSolidInfillSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("internal_solid_infill_speed", internal_solid);
}

TEST_CASE("Gap infill speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][GapInfillSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("gap_infill_speed", gap_fill);
}

namespace {

DynamicPrintConfig role_speed_endpoint_config()
{
    DynamicPrintConfig config = role_speed_contract_config();
    // Pin first-layer feeds to the tested minimum so they cannot hide the
    // target role's later-layer feed; a zero threshold disables the separate
    // small-perimeter speed branch for external walls.
    set_nullable_role_speed(config, "initial_layer_speed", 1.0);
    set_nullable_role_speed(config, "initial_layer_infill_speed", 1.0);
    set_nullable_role_speed(config, "small_perimeter_threshold", 0.0);
    // Hold companion roles at distinct registered rates while one setting is
    // swept. This also makes a role-to-role speed routing error observable.
    set_nullable_role_speed(config, "outer_wall_speed", 67.0);
    set_nullable_role_speed(config, "inner_wall_speed", 77.0);
    set_nullable_role_speed(config, "sparse_infill_speed", 87.0);
    set_nullable_role_speed(config, "top_surface_speed", 97.0);
    return config;
}

} // namespace

TEST_CASE("Outer wall speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][OuterWallSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("outer_wall_speed", outer, role_speed_endpoint_config());
}

TEST_CASE("Inner wall speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][InnerWallSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("inner_wall_speed", inner, role_speed_endpoint_config());
}

TEST_CASE("Sparse infill speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][SparseInfillSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("sparse_infill_speed", sparse, role_speed_endpoint_config());
}

TEST_CASE("Top surface speed honors its minimum and nullable object inheritance",
          "[MotionFlowVariance][TopSurfaceSpeed][SettingsVariance]")
{
    verify_nullable_role_speed_contract("top_surface_speed", top, role_speed_endpoint_config());
}

namespace {

struct WidthMeasurement {
    size_t paths = 0;
    double path_length = 0.0;
    double length_weighted_width = 0.0;
    double material_volume_mm3 = 0.0;
    double minimum_width = std::numeric_limits<double>::max();
    double maximum_width = 0.0;
};

struct WidthRoleEvidence {
    WidthMeasurement all;
    WidthMeasurement first_layer;
    WidthMeasurement later_layers;
};

struct WidthPipelineEvidence {
    std::array<WidthRoleEvidence, 5> roles{};
    Evidence emitted;
};

std::optional<size_t> width_role_index(ExtrusionRole role)
{
    switch (role) {
    case erExternalPerimeter: return outer;
    case erPerimeter: return inner;
    case erInternalInfill: return sparse;
    case erTopSolidInfill: return top;
    case erSolidInfill: return internal_solid;
    default: return std::nullopt;
    }
}

void add_width_path(WidthMeasurement &measurement, const ExtrusionPath &path)
{
    const double length = path.length();
    const double width = static_cast<double>(path.width);
    ++measurement.paths;
    measurement.path_length += length;
    measurement.length_weighted_width += length * width;
    measurement.material_volume_mm3 += path.total_volume();
    measurement.minimum_width = std::min(measurement.minimum_width, width);
    measurement.maximum_width = std::max(measurement.maximum_width, width);
}

WidthPipelineEvidence fresh_width_pipeline_evidence(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ matrix_fixture() }, print, model, config);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());

    const std::string output = gcode(print);
    REQUIRE_FALSE(output.empty());

    WidthPipelineEvidence evidence;
    evidence.emitted = evidence_for(print, output);
    require_valid(evidence.emitted);
    REQUIRE_FALSE(print.objects().empty());
    const PrintObject &object = *print.objects().front();
    REQUIRE_FALSE(object.layers().empty());
    const Layer *first_layer = object.layers().front();

    const auto record_collection = [&evidence](const ExtrusionEntityCollection &collection,
                                                bool is_first_layer) {
        for_each_extrusion_path(collection, [&evidence, is_first_layer](const ExtrusionPath &path) {
            const std::optional<size_t> index = width_role_index(path.role());
            if (!index)
                return;
            WidthRoleEvidence &role = evidence.roles[*index];
            add_width_path(role.all, path);
            add_width_path(is_first_layer ? role.first_layer : role.later_layers, path);
        });
    };

    for (const Layer *layer : object.layers()) {
        const bool is_first_layer = layer == first_layer;
        for (const LayerRegion *region : layer->regions()) {
            record_collection(region->perimeters, is_first_layer);
            record_collection(region->fills, is_first_layer);
            record_collection(region->thin_fills, is_first_layer);
        }
    }
    return evidence;
}

constexpr double width_pipeline_minimum_layer_height = 0.2;

DynamicPrintConfig width_pipeline_config(const char *setting, int width_mode, double nozzle_diameter)
{
    DynamicPrintConfig config = matrix_config();
    config.set_deserialize_strict({
        { "wall_generator",                   "classic" },
        { "nozzle_diameter",                  std::to_string(nozzle_diameter) },
        { "filament_diameter",                "1.75" },
        { "filament_flow_ratio",              "1.0" },
        { "print_flow_ratio",                 1.0 },
        { "filament_max_volumetric_speed",    "200" },
        { "filament_adaptive_volumetric_speed", false },
        { "set_other_flow_ratios",            true },
        { "first_layer_flow_ratio",           1.0 },
        { "outer_wall_flow_ratio",            1.0 },
        { "inner_wall_flow_ratio",            1.0 },
        { "sparse_infill_flow_ratio",         1.0 },
        { "internal_solid_infill_flow_ratio", 1.0 },
        { "top_solid_infill_flow_ratio",      "1.0" },
        { "bottom_solid_infill_flow_ratio",   1.0 },
        { "gap_fill_flow_ratio",              1.0 },
        { "outer_wall_speed",                 "120" },
        { "inner_wall_speed",                 "120" },
        { "sparse_infill_speed",              "120" },
        { "internal_solid_infill_speed",      "120" },
        { "top_surface_speed",                "120" },
        { "wall_loops",                       2 },
        { "top_shell_layers",                 2 },
        { "bottom_shell_layers",              2 },
        { "extra_solid_infills",              3 },
        { "minimum_sparse_infill_area",       0.0 },
        { "sparse_infill_density",            "20%" },
        { "detect_thin_wall",                 false },
        // Keep this nominal-width cohort on the regular full-line infill path.
        // Narrow-area concentric fill and adaptive patterns have separate width contracts.
        { "detect_narrow_internal_solid_infill", false },
        { "top_surface_pattern",              "monotonicline" },
        { "internal_solid_infill_pattern",    "monotonic" },
        { "layer_height",                     width_pipeline_minimum_layer_height },
        { "initial_layer_print_height",       0.3 },
        { "use_relative_e_distances",         true },
        { "gcode_comments",                   true },
    });

    // Start with every width on the public fallback path. Each case changes
    // exactly one canonical setting so 0, absolute, and nozzle-relative input
    // remain independently observable.
    constexpr std::array<const char *, 7> width_keys{{
        "line_width", "initial_layer_line_width", "outer_wall_line_width",
        "inner_wall_line_width", "sparse_infill_line_width",
        "internal_solid_infill_line_width", "top_surface_line_width"
    }};
    for (const char *key : width_keys)
        config.set_key_value(key, new ConfigOptionFloatOrPercent(0.0, false));

    const ConfigOptionFloatOrPercent selected = width_mode == 0
        ? ConfigOptionFloatOrPercent(0.0, false)
        : (width_mode == 1 ? ConfigOptionFloatOrPercent(0.50, false)
                           : ConfigOptionFloatOrPercent(125.0, true));
    config.set_key_value(setting, selected.clone());
    return config;
}

double expected_auto_width(size_t role, double nozzle_diameter)
{
    // The documented default is nozzle diameter for top solid infill and
    // 112.5% of nozzle diameter for perimeter and ordinary solid/infill roles.
    return role == top ? nozzle_diameter : 1.125 * nozzle_diameter;
}

double expected_selected_width(int width_mode, size_t role, double nozzle_diameter)
{
    if (width_mode == 0)
        return expected_auto_width(role, nozzle_diameter);
    if (width_mode == 1)
        return 0.50;
    return 1.25 * nozzle_diameter;
}

void require_width(const WidthMeasurement &measurement, double expected,
                   bool full_solid_spacing_may_adjust = false)
{
    REQUIRE(measurement.paths > 0);
    REQUIRE(measurement.path_length > 0.0);
    CHECK(std::isfinite(measurement.minimum_width));
    CHECK(std::isfinite(measurement.maximum_width));
    if (! full_solid_spacing_may_adjust) {
        CHECK_THAT(measurement.minimum_width, Catch::Matchers::WithinAbs(expected, 1e-4));
        CHECK_THAT(measurement.maximum_width, Catch::Matchers::WithinAbs(expected, 1e-4));
        CHECK_THAT(measurement.length_weighted_width / measurement.path_length,
                   Catch::Matchers::WithinAbs(expected, 1e-4));
        return;
    }

    // Full-density rectilinear fill may increase centerline spacing by at most 20%.
    // For rounded-rectangle flow, requested spacing is width - height*(1-pi/4),
    // so the largest possible width increment is 0.2 times that spacing.
    // The fixture's first layer is 0.3 mm, all later layers are 0.2 mm, and no
    // layer is thinner than this conservative height used to maximize the cap.
    constexpr double pi = 3.14159265358979323846;
    constexpr double minimum_fixture_layer_height = width_pipeline_minimum_layer_height;
    constexpr double maximum_spacing_increase = 0.2;
    const double nominal_spacing = expected - minimum_fixture_layer_height * (1.0 - pi / 4.0);
    REQUIRE(nominal_spacing > 0.0);
    const double maximum_width = expected + maximum_spacing_increase * nominal_spacing;
    // Spacing is quantized to internal coordinate units before being converted
    // back to float path widths. Four coordinate steps plus four float ulps are
    // a conservative envelope for those conversions, not a relaxed print-width tolerance.
    const double rounding_mm = 4.0 * SCALING_FACTOR
                             + 4.0 * std::numeric_limits<float>::epsilon()
                               * std::max(1.0, maximum_width);
    const double weighted_width = measurement.length_weighted_width / measurement.path_length;
    INFO("solid fill nominal width: " << expected << ", spacing cap width: " << maximum_width
         << ", conversion envelope: " << rounding_mm);
    CHECK(measurement.minimum_width >= expected - rounding_mm);
    CHECK(measurement.maximum_width <= maximum_width + rounding_mm);
    CHECK(weighted_width >= expected - rounding_mm);
    CHECK(weighted_width <= maximum_width + rounding_mm);
}

void require_role_material_reconciles(const WidthPipelineEvidence &evidence, size_t role,
                                      double filament_diameter)
{
    const WidthMeasurement &geometry = evidence.roles[role].all;
    const RoleEvidence &gcode_role = evidence.emitted.roles[role];
    REQUIRE(geometry.paths > 0);
    REQUIRE(geometry.material_volume_mm3 > 0.0);
    REQUIRE(gcode_role.moves > 0);
    REQUIRE(gcode_role.filament_length > 0.0);

    constexpr double pi = 3.14159265358979323846;
    const double filament_area = pi * filament_diameter * filament_diameter / 4.0;
    const double emitted_volume = gcode_role.filament_length * filament_area;
    const double e_rounding_bound = static_cast<double>(gcode_role.moves)
                                  * e_pair_error_bound(gcode_role.max_positive_e_pair) * filament_area;
    const double tolerance = e_rounding_bound
                           + 0.001 * std::max(1.0, geometry.material_volume_mm3);
    REQUIRE(tolerance < 0.01 * geometry.material_volume_mm3);
    CHECK_THAT(emitted_volume,
               Catch::Matchers::WithinAbs(geometry.material_volume_mm3, tolerance));
}

void require_role_pipeline(const WidthPipelineEvidence &evidence, size_t role,
                           double expected_width, double filament_diameter)
{
    const bool full_solid_spacing_may_adjust = role == top || role == internal_solid;
    const WidthRoleEvidence &geometry = evidence.roles[role];
    require_width(geometry.all, expected_width, full_solid_spacing_may_adjust);
    if (geometry.first_layer.paths > 0)
        require_width(geometry.first_layer, expected_width, full_solid_spacing_may_adjust);
    if (geometry.later_layers.paths > 0)
        require_width(geometry.later_layers, expected_width, full_solid_spacing_may_adjust);
    require_role_material_reconciles(evidence, role, filament_diameter);
}

void check_width_pipeline(const char *setting, int width_mode, double nozzle_diameter,
                          std::optional<size_t> target_role, bool initial_layer_contract,
                          bool global_fallback_contract)
{
    CAPTURE(setting, width_mode, nozzle_diameter);
    constexpr double filament_diameter = 1.75;
    const DynamicPrintConfig config = width_pipeline_config(setting, width_mode, nozzle_diameter);
    REQUIRE(config.option(setting) != nullptr);
    const auto *narrow_solid = config.opt<ConfigOptionBool>("detect_narrow_internal_solid_infill");
    REQUIRE(narrow_solid != nullptr);
    CHECK_FALSE(narrow_solid->value);
    const ConfigOption *top_pattern = config.option("top_surface_pattern");
    REQUIRE(top_pattern != nullptr);
    CHECK(top_pattern->getInt() == int(ipMonotonicLine));
    const ConfigOption *internal_solid_pattern = config.option("internal_solid_infill_pattern");
    REQUIRE(internal_solid_pattern != nullptr);
    CHECK(internal_solid_pattern->getInt() == int(ipMonotonic));
    const WidthPipelineEvidence evidence = fresh_width_pipeline_evidence(config);
    INFO("emitted evidence: " + evidence_summary(evidence.emitted));

    if (initial_layer_contract) {
        REQUIRE(target_role.has_value());
        const size_t role = *target_role;
        const WidthRoleEvidence &geometry = evidence.roles[role];
        REQUIRE(geometry.first_layer.paths > 0);
        REQUIRE(geometry.later_layers.paths > 0);
        const double first_expected = expected_selected_width(width_mode, role, nozzle_diameter);
        const double later_expected = expected_auto_width(role, nozzle_diameter);
        require_width(geometry.first_layer, first_expected);
        require_width(geometry.later_layers, later_expected);
        REQUIRE(evidence.emitted.roles[role].first_layer_moves > 0);
        REQUIRE(evidence.emitted.roles[role].later_layer_moves > 0);
        require_role_material_reconciles(evidence, role, filament_diameter);
        return;
    }

    if (global_fallback_contract) {
        for (size_t role = outer; role <= internal_solid; ++role) {
            CAPTURE(role);
            require_role_pipeline(evidence, role,
                                 expected_selected_width(width_mode, role, nozzle_diameter),
                                 filament_diameter);
        }
        return;
    }

    REQUIRE(target_role.has_value());
    const size_t role = *target_role;
    require_role_pipeline(evidence, role,
                         expected_selected_width(width_mode, role, nozzle_diameter),
                         filament_diameter);
}

} // namespace

TEST_CASE("Initial-layer line width controls first-layer extrusion paths",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("initial_layer_line_width", width_mode, nozzle_diameter,
                         outer, true, false);
}

TEST_CASE("Default line width reaches paths when role widths are automatic",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("line_width", width_mode, nozzle_diameter,
                         std::nullopt, false, true);
}

TEST_CASE("Outer-wall line width controls external-perimeter paths",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("outer_wall_line_width", width_mode, nozzle_diameter,
                         outer, false, false);
}

TEST_CASE("Inner-wall line width controls inner-perimeter paths",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("inner_wall_line_width", width_mode, nozzle_diameter,
                         inner, false, false);
}

TEST_CASE("Internal-solid infill line width controls solid-infill paths",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("internal_solid_infill_line_width", width_mode, nozzle_diameter,
                         internal_solid, false, false);
}

TEST_CASE("Top-surface line width controls top-infill paths",
          "[MotionFlowVariance][WidthPipeline][SettingsVariance]")
{
    const double nozzle_diameter = GENERATE(0.4, 0.6);
    const int width_mode = GENERATE(0, 1, 2);
    check_width_pipeline("top_surface_line_width", width_mode, nozzle_diameter,
                         top, false, false);
}

namespace {

DynamicPrintConfig numeric_role_flow_config()
{
    DynamicPrintConfig config = role_flow_config();
    config.set_deserialize_strict({
        { "nozzle_diameter",                  "0.4" },
        { "filament_diameter",                "1.75" },
        { "filament_flow_ratio",              "1.0" },
        { "print_flow_ratio",                 1.0 },
        { "filament_max_volumetric_speed",    "200" },
        { "filament_adaptive_volumetric_speed", false },
        { "set_other_flow_ratios",            true },
        { "first_layer_flow_ratio",           1.0 },
        { "outer_wall_flow_ratio",            1.0 },
        { "inner_wall_flow_ratio",            1.0 },
        { "overhang_flow_ratio",              1.0 },
        { "sparse_infill_flow_ratio",         1.0 },
        { "detect_overhang_wall",             true },
        { "enable_support",                   false },
    });
    return config;
}

Evidence fresh_overhang_role_flow_evidence(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{ mesh(TestMesh::overhang) }, print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO("validation.string: " + validation.string);
    INFO("validation.opt_key: " + validation.opt_key);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    require_bounded_role_output(output);
    Evidence evidence = evidence_for(print, output);
    require_valid(evidence);
    REQUIRE(evidence.roles[overhang].geometry_path_length > 0.0);
    REQUIRE(evidence.roles[overhang].role_motion_path_length > 0.0);
    return evidence;
}

Evidence numeric_role_flow_evidence(const DynamicPrintConfig &config, bool use_overhang_fixture)
{
    if (use_overhang_fixture)
        return fresh_overhang_role_flow_evidence(config);
    return fresh_role_flow_evidence(config);
}

void require_flow_ratio_geometry(const Evidence &expected, const Evidence &observed)
{
    REQUIRE(observed.layers == expected.layers);
    REQUIRE(observed.perimeter_items == expected.perimeter_items);
    CHECK_THAT(observed.perimeter_length, Catch::Matchers::WithinRel(expected.perimeter_length, 1e-7));
    CHECK(observed.motion_moves == expected.motion_moves);
    for (size_t i = 0; i < expected.roles.size(); ++i) {
        CAPTURE(i);
        CHECK_THAT(observed.roles[i].geometry_path_length,
                   Catch::Matchers::WithinRel(expected.roles[i].geometry_path_length, 1e-7));
        // This includes non-extruding XY moves. At the legal zero-flow endpoint,
        // the role path must still be emitted while its deposited E becomes zero.
        CHECK(observed.roles[i].role_motion_moves == expected.roles[i].role_motion_moves);
        CHECK_THAT(observed.roles[i].role_motion_path_length,
                   Catch::Matchers::WithinRel(expected.roles[i].role_motion_path_length, 1e-6));
    }
}

void require_numeric_role_flow_contract(const char *key, size_t target_role, bool use_overhang_fixture)
{
    const ConfigOptionDef *definition = print_config_def.get(key);
    REQUIRE(definition != nullptr);
    REQUIRE(definition->type == coFloat);
    CHECK_THAT(definition->min, Catch::Matchers::WithinAbs(0.0, 1e-12));
    CHECK_THAT(definition->max, Catch::Matchers::WithinAbs(2.0, 1e-12));
    const double minimum = definition->min;
    const double representative = 0.75;
    const double maximum = definition->max;
    REQUIRE(definition->is_value_valid(minimum));
    REQUIRE(definition->is_value_valid(representative));
    REQUIRE(definition->is_value_valid(maximum));

    const DynamicPrintConfig baseline_config = numeric_role_flow_config();
    const Evidence baseline = numeric_role_flow_evidence(baseline_config, use_overhang_fixture);
    INFO("baseline evidence: " + evidence_summary(baseline));
    REQUIRE(baseline.roles[target_role].geometry_path_length > 0.0);
    REQUIRE(baseline.roles[target_role].role_motion_path_length > 0.0);
    REQUIRE(baseline.roles[target_role].moves > 0);
    REQUIRE(baseline.roles[target_role].filament_length > 0.0);
    bool has_positive_non_target_deposition = false;
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        if (i != target_role && baseline.roles[i].moves > 0 && baseline.roles[i].filament_length > 0.0)
            has_positive_non_target_deposition = true;
    }
    REQUIRE(has_positive_non_target_deposition);

    for (const double ratio : { minimum, representative, maximum }) {
        CAPTURE(key, ratio);
        DynamicPrintConfig changed_config = baseline_config;
        changed_config.set_deserialize_strict(key, std::to_string(ratio));
        REQUIRE(changed_config.option(key) != nullptr);
        REQUIRE(changed_config.option(key)->serialize() != baseline_config.option(key)->serialize());
        REQUIRE(changed_config.option("set_other_flow_ratios")->serialize()
                == baseline_config.option("set_other_flow_ratios")->serialize());

        const Evidence changed = numeric_role_flow_evidence(changed_config, use_overhang_fixture);
        INFO("changed evidence: " + evidence_summary(changed));
        require_flow_ratio_geometry(baseline, changed);
        // A role-specific flow multiplier changes E-per-XY, not the other
        // deposition roles or the XY geometry of the selected role.
        for (size_t i = 0; i < baseline.roles.size(); ++i) {
            CAPTURE(i);
            if (i == target_role)
                continue;
            require_same_role_material(baseline.roles[i], changed.roles[i]);
        }
        const RoleEvidence &before = baseline.roles[target_role];
        const RoleEvidence &after = changed.roles[target_role];
        if (ratio == 0.0) {
            CHECK(after.moves == 0);
            CHECK_THAT(after.filament_length, Catch::Matchers::WithinAbs(0.0, 1e-9));
            CHECK_THAT(after.gcode_path_length, Catch::Matchers::WithinAbs(0.0, 1e-9));
        } else {
            require_scaled_role_material(before, after, ratio);
        }
    }
}

} // namespace

TEST_CASE("Outer wall flow ratio scales only external-wall material",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_numeric_role_flow_contract("outer_wall_flow_ratio", outer, false);
}

TEST_CASE("Inner wall flow ratio scales only inner-wall material",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_numeric_role_flow_contract("inner_wall_flow_ratio", inner, false);
}

TEST_CASE("Sparse infill flow ratio scales only sparse-infill material",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_numeric_role_flow_contract("sparse_infill_flow_ratio", sparse, false);
}

TEST_CASE("Overhang flow ratio scales only detected overhang-wall material",
          "[MotionFlowVariance][RoleFlowVariance][SettingsVariance]")
{
    require_numeric_role_flow_contract("overhang_flow_ratio", overhang, true);
}
