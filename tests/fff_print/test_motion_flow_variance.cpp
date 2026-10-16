#include "catch2/catch_test_macros.hpp"
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
    std::array<RoleEvidence, 7> roles{};
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
        if (comment.find("Outer wall") != std::string_view::npos)
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
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        REQUIRE(baseline.roles[i].moves > 0);
        REQUIRE(baseline.roles[i].filament_length > 0.0);
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
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        REQUIRE(baseline.roles[i].moves > 0);
        REQUIRE(baseline.roles[i].filament_length > 0.0);
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
    for (size_t i = 0; i < baseline.roles.size(); ++i) {
        CAPTURE(i);
        REQUIRE(baseline.roles[i].moves > 0);
        REQUIRE(baseline.roles[i].filament_length > 0.0);
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
