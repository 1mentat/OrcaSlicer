#include <algorithm>
#include <cmath>
#include <map>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_all.hpp>
#include "libslic3r/PrintConfig.hpp"
#include <string>
#include <cstddef>
#include <utility>
#include "libslic3r/Model.hpp"
#include "libslic3r/ObjectID.hpp"
#include <vector>
#include "libslic3r/Config.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintBase.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/Print.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

// Two physical filaments plus one mixed slot (config index 2, 1-based id 3) blending them 60/40.
// The mixed arrays are parallel to filament_colour and must be sized to the filament count.
// Note ConfigOptionBools deserializes on ',' while ConfigOptionStrings uses ';'.
DynamicPrintConfig mixed_config(bool sublayer_on, const char *ratios = "0.6,0.4")
{
    DynamicPrintConfig config = multifilament_config(3);
    config.set_deserialize_strict({
        {"filament_is_mixed",               "0,0,1"},
        {"filament_mixed_components",       ";;1,2"},
        {"filament_mixed_sublayer_ratios",  std::string(";;") + ratios},
        {"filament_mixed_gradient",         "0,0,0"},
        {"filament_mixed_gradient_range",   ";;"},
        {"filament_mixed_gradient_curve",   ";;"},
        {"filament_mixed_gradient_per_part","0,0,0"},
        {"enable_mixed_color_sublayer",     sublayer_on ? "1" : "0"},
        // Assign every region role to the mixed slot so it actually participates in slicing.
        {"outer_wall_filament_id",          "3"},
        {"inner_wall_filament_id",          "3"},
        {"sparse_infill_filament_id",       "3"},
        {"internal_solid_filament_id",      "3"},
        {"top_surface_filament_id",         "3"},
        {"bottom_surface_filament_id",      "3"},
    });
    return config;
}

// Total sub-layer groups and per-layer mixed-filament resolutions across the whole tool ordering.
void count_mixed(ToolOrdering &to, size_t &groups, size_t &resolutions)
{
    groups = resolutions = 0;
    for (const LayerTools &lt : to.layer_tools()) {
        groups      += lt.mixed_sub_layer_groups.size();
        resolutions += lt.mixed_filament_resolution.size();
    }
}

} // namespace

TEST_CASE("Mixed filament gradients record independent object and volume runs", "[MixedFilament][MixedGradient]")
{
    const bool per_part = GENERATE(false, true);
    constexpr double layer_height = 0.2;
    constexpr double short_height = 1.2;
    constexpr double tall_height  = 1.6;
    const size_t short_run = static_cast<size_t>(std::llround(short_height / layer_height)) - 1;
    const size_t tall_run  = static_cast<size_t>(std::llround(tall_height / layer_height)) - 1;

    DynamicPrintConfig config = mixed_config(true);
    config.set_deserialize_strict({
        {"layer_height",                    "0.2"},
        {"initial_layer_print_height",      "0.2"},
        {"wall_loops",                     "1"},
        {"top_shell_layers",               "0"},
        {"bottom_shell_layers",             "0"},
        {"sparse_infill_density",          "0%"},
        {"skirt_loops",                    "0"},
        {"brim_type",                      "no_brim"},
        {"layer_change_gcode",             "G92 E0\n"},
        {"filament_mixed_gradient",         "0,0,1"},
        {"filament_mixed_gradient_range",   ";;0.8,0.2"},
        {"filament_mixed_gradient_per_part", per_part ? "0,0,1" : "0,0,0"},
    });

    Model model;
    Print print;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->name = "mixed-gradient.stl";
    ModelVolume *short_volume = object->add_volume(make_cube(8., 8., short_height), ModelVolumeType::MODEL_PART, false);
    TriangleMesh tall_mesh = make_cube(8., 8., tall_height);
    tall_mesh.translate(10.f, 0.f, 0.f);
    ModelVolume *tall_volume = object->add_volume(std::move(tall_mesh), ModelVolumeType::MODEL_PART, false);
    REQUIRE(short_volume != nullptr);
    REQUIRE(tall_volume != nullptr);
    short_volume->config.set("extruder", 3);
    tall_volume->config.set("extruder", 3);
    object->add_instance();
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);

    const StringObjectException validation = print.validate();
    INFO("validation error: " << validation.string);
    REQUIRE(validation.string.empty());
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject *print_object = print.objects().front();
    REQUIRE(print_object != nullptr);
    const ToolOrdering &ordering_const = print.tool_ordering();
    ToolOrdering &ordering = const_cast<ToolOrdering &>(ordering_const);
    REQUIRE(ordering.layer_tools().size() == tall_run + 1);

    std::vector<const LayerTools::MixedSubLayerGroup *> groups;
    for (const LayerTools &layer_tools : ordering.layer_tools())
        for (const LayerTools::MixedSubLayerGroup &group : layer_tools.mixed_sub_layer_groups)
            if (group.mixed_slot_0based == 2)
                groups.push_back(&group);

    // Both physical components are sorted in the public metadata. The gradient range is
    // independently selected here; each sampled layer uses its half-step in the 7-layer run.
    REQUIRE(groups.size() == tall_run);
    std::map<ObjectID, std::vector<LayerTools::MixedSubLayerGroup::VolumeGradient>> volume_gradients;
    for (size_t i = 0; i < groups.size(); ++i) {
        const auto &group = *groups[i];
        REQUIRE(group.is_gradient);
        CHECK(group.components_0based == std::vector<unsigned int>{0, 1});
        CHECK(group.gradient_first_sorted_idx == 0);
        REQUIRE(group.sub_heights.size() == 2);
        CHECK_THAT(group.layer_height, Catch::Matchers::WithinAbs(layer_height, 1e-7));
        CHECK_THAT(group.sub_heights[0] + group.sub_heights[1], Catch::Matchers::WithinAbs(layer_height, 1e-7));

        const double t = (2. * static_cast<double>(i) + 1.) / (2. * static_cast<double>(tall_run));
        const double first_ratio = 0.8 + (0.2 - 0.8) * t;
        CHECK_THAT(group.sub_heights[0], Catch::Matchers::WithinAbs(first_ratio * layer_height, 1e-6));
        CHECK_THAT(group.sub_heights[1], Catch::Matchers::WithinAbs((1. - first_ratio) * layer_height, 1e-6));

        REQUIRE(group.per_object_gradient.size() == 1);
        const auto object_it = group.per_object_gradient.find(print_object);
        REQUIRE(object_it != group.per_object_gradient.end());
        CHECK(object_it->second.total_layers == tall_run);
        CHECK(object_it->second.current_idx == i);
        CHECK_THAT(object_it->second.gradient_start, Catch::Matchers::WithinAbs(0.8, 1e-7));
        CHECK_THAT(object_it->second.gradient_end, Catch::Matchers::WithinAbs(0.2, 1e-7));

        for (const auto &[key, gradient] : group.per_volume_gradient) {
            REQUIRE(key.obj == print_object);
            volume_gradients[key.volume_id].push_back(gradient);
            CHECK_THAT(gradient.gradient_start, Catch::Matchers::WithinAbs(0.8, 1e-7));
            CHECK_THAT(gradient.gradient_end, Catch::Matchers::WithinAbs(0.2, 1e-7));
        }
    }
    if (!per_part) {
        // With per-part gradient disabled, both model-part regions remain one object run and
        // no volume-tagged metadata is published.
        CHECK(volume_gradients.empty());
    } else {
        // The two model parts have intentionally different heights. Their independent runs start
        // after the bed layer and therefore have 5 and 7 entries, respectively.
        REQUIRE(volume_gradients.size() == 2);
        const auto check_volume_run = [&](ObjectID volume_id, size_t expected_layers) {
            const auto it = volume_gradients.find(volume_id);
            REQUIRE(it != volume_gradients.end());
            REQUIRE(it->second.size() == expected_layers);
            for (size_t i = 0; i < it->second.size(); ++i) {
                CHECK(it->second[i].total_layers == expected_layers);
                CHECK(it->second[i].current_idx == i);
                CHECK_THAT(it->second[i].gradient_start, Catch::Matchers::WithinAbs(0.8, 1e-7));
                CHECK_THAT(it->second[i].gradient_end, Catch::Matchers::WithinAbs(0.2, 1e-7));
            }
        };
        check_volume_run(short_volume->id(), short_run);
        check_volume_run(tall_volume->id(), tall_run);
    }
}

namespace mixed_gradient_gcode_contracts {

enum class Mode { LinearPerObject, CurvedPerVolume, DisabledControl };

struct PositiveExtrusion {
    int    tool;
    double x;
    double z;
    double e;
    double xy;
};

struct XBounds {
    double min_x;
    double max_x;
};

struct Bucket {
    double e = 0.;
    double xy = 0.;
    size_t lines = 0;
};

std::vector<PositiveExtrusion> read_positive_extrusions(const std::string &gcode,
                                                        const GCodeConfig &config)
{
    GCodeReader reader;
    reader.apply_config(config);
    std::vector<PositiveExtrusion> result;
    int current_tool = -1;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("T0")) {
            current_tool = 0;
            return;
        }
        if (line.cmd_is("T1")) {
            current_tool = 1;
            return;
        }
        if (current_tool < 0 || !line.extruding(self) || (!line.has_x() && !line.has_y()))
            return;
        const double xy = line.dist_XY(self);
        const double e  = line.dist_E(self);
        const double x  = line.new_X(self);
        const double z  = line.new_Z(self);
        if (!std::isfinite(x) || !std::isfinite(z) || !std::isfinite(e) || !std::isfinite(xy) || xy < 0.01)
            return;
        result.push_back({current_tool, x, z, e, xy});
    });
    return result;
}

Bucket bucket_at(const std::vector<PositiveExtrusion> &samples, const XBounds &bounds,
                 double target_z, int tool)
{
    Bucket result;
    constexpr double x_margin = 0.30;
    constexpr double z_tolerance = 0.002;
    for (const PositiveExtrusion &sample : samples) {
        if (sample.tool != tool)
            continue;
        if (sample.x < bounds.min_x - x_margin || sample.x > bounds.max_x + x_margin)
            continue;
        if (std::abs(sample.z - target_z) > z_tolerance)
            continue;
        result.e += sample.e;
        result.xy += sample.xy;
        ++result.lines;
    }
    return result;
}

double expected_ratio(bool curved, size_t total_layers, size_t current_idx)
{
    REQUIRE(current_idx < total_layers);
    const double t = (2. * static_cast<double>(current_idx) + 1.) /
                     (2. * static_cast<double>(total_layers));
    if (!curved)
        return 0.8 + (0.2 - 0.8) * t;

    // The fixture sets both tangents to zero at all three anchors. Each half
    // is therefore a smoothstep between its explicitly configured ratios.
    const double u = t <= 0.5 ? 2. * t : 2. * t - 1.;
    const double smoothstep = u * u * (3. - 2. * u);
    return t <= 0.5 ? 0.8 + 0.1 * smoothstep : 0.9 - 0.7 * smoothstep;
}

void check_volume_run(const std::vector<PositiveExtrusion> &samples, const PrintObject &print_object,
                      const XBounds &bounds, size_t present_layers, size_t gradient_layers, bool curved)
{
    REQUIRE(print_object.layers().size() >= present_layers + 1);
    constexpr double layer_height = 0.2;
    for (size_t i = 0; i < present_layers; ++i) {
        const Layer *layer = print_object.layers()[i + 1];
        REQUIRE(layer != nullptr);
        const double nominal_z = layer->print_z;
        const double ratio = expected_ratio(curved, gradient_layers, i);
        const double lower_z = nominal_z - layer_height + ratio * layer_height;
        const Bucket first = bucket_at(samples, bounds, lower_z, 0);
        const Bucket second = bucket_at(samples, bounds, nominal_z, 1);
        INFO("run=" << i << " nominal_z=" << nominal_z << " lower_z=" << lower_z);
        REQUIRE(first.lines > 0);
        REQUIRE(second.lines > 0);
        CHECK(first.xy > 0.1);
        CHECK(second.xy > 0.1);
        const double total_e = first.e + second.e;
        REQUIRE(total_e > 0.);
        CHECK_THAT(first.e / total_e, Catch::Matchers::WithinAbs(ratio, 0.04));
        CHECK_THAT(second.e / total_e, Catch::Matchers::WithinAbs(1. - ratio, 0.04));
    }
}

} // namespace mixed_gradient_gcode_contracts

TEST_CASE("Mixed filament gradient scales emitted volume flow", "[MixedFilament][MixedGradientGCode]")
{
    using namespace mixed_gradient_gcode_contracts;
    const Mode mode = GENERATE(Mode::LinearPerObject, Mode::CurvedPerVolume, Mode::DisabledControl);
    const bool enabled = mode != Mode::DisabledControl;
    const bool per_part = mode == Mode::CurvedPerVolume;
    const bool curved = mode == Mode::CurvedPerVolume;

    DynamicPrintConfig config = mixed_config(enabled);
    config.set_deserialize_strict({
        {"layer_height",                    "0.2"},
        {"initial_layer_print_height",      "0.2"},
        {"wall_loops",                     "1"},
        {"top_shell_layers",               "0"},
        {"bottom_shell_layers",            "0"},
        {"sparse_infill_density",          "0%"},
        {"skirt_loops",                    "0"},
        {"brim_type",                      "no_brim"},
        {"layer_change_gcode",             "G92 E0\n"},
        {"filament_flow_ratio",            "1,1,1"},
        {"filament_mixed_gradient",         enabled ? "0,0,1" : "0,0,0"},
        {"filament_mixed_gradient_range",   ";;0.8,0.2"},
        {"filament_mixed_gradient_curve",   curved ? ";;0,0.8,0,0|0.5,0.9,0,0|1,0.2,0,0" : ";;"},
        {"filament_mixed_gradient_per_part", per_part ? "0,0,1" : "0,0,0"},
    });

    Model model;
    Print print;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *short_volume = object->add_volume(make_cube(8., 8., 1.2), ModelVolumeType::MODEL_PART, false);
    TriangleMesh tall_mesh = make_cube(8., 8., 1.6);
    tall_mesh.translate(10.f, 0.f, 0.f);
    ModelVolume *tall_volume = object->add_volume(std::move(tall_mesh), ModelVolumeType::MODEL_PART, false);
    REQUIRE(short_volume != nullptr);
    REQUIRE(tall_volume != nullptr);
    short_volume->config.set("extruder", 3);
    tall_volume->config.set("extruder", 3);
    object->add_instance();
    object->ensure_on_bed();
    print.auto_assign_extruders(object);
    print.apply(model, config);
    const StringObjectException validation = print.validate();
    INFO("validation error: " << validation.string);
    REQUIRE(validation.string.empty());

    const std::string output = Slic3r::Test::gcode(print);
    REQUIRE(!output.empty());
    REQUIRE(print.objects().size() == 1);
    const PrintObject *print_object = print.objects().front();
    REQUIRE(print_object != nullptr);
    REQUIRE(print_object->instances().size() == 1);
    REQUIRE(print_object->layers().size() == 8);

    const auto short_bbox = short_volume->mesh().bounding_box();
    const auto tall_bbox = tall_volume->mesh().bounding_box();
    REQUIRE(short_bbox.defined);
    REQUIRE(tall_bbox.defined);
    const double instance_shift_x = unscale<double>(print_object->instances().front().shift.x());
    const double center_offset_x = unscale<double>(print_object->center_offset().x());
    const XBounds short_bounds{short_bbox.min.x() + instance_shift_x - center_offset_x,
                               short_bbox.max.x() + instance_shift_x - center_offset_x};
    const XBounds tall_bounds{tall_bbox.min.x() + instance_shift_x - center_offset_x,
                              tall_bbox.max.x() + instance_shift_x - center_offset_x};
    REQUIRE(tall_bounds.min_x - short_bounds.max_x > 1.0);

    const auto samples = read_positive_extrusions(output, print.config());
    REQUIRE(!samples.empty());
    constexpr size_t short_run = 5;
    constexpr size_t tall_run = 7;
    if (!enabled) {
        size_t fractional_events = 0;
        size_t nominal_events = 0;
        for (const PositiveExtrusion &sample : samples) {
            const bool in_short = sample.x >= short_bounds.min_x - 0.30 && sample.x <= short_bounds.max_x + 0.30;
            const bool in_tall = sample.x >= tall_bounds.min_x - 0.30 && sample.x <= tall_bounds.max_x + 0.30;
            if (!in_short && !in_tall)
                continue;
            for (size_t i = 0; i < tall_run; ++i) {
                const Layer *layer = print_object->layers()[i + 1];
                REQUIRE(layer != nullptr);
                const double nominal_z = layer->print_z;
                const double fractional_z = nominal_z - 0.2 + expected_ratio(false, tall_run, i) * 0.2;
                if (std::abs(sample.z - fractional_z) <= 0.002)
                    ++fractional_events;
                if (std::abs(sample.z - nominal_z) <= 0.002)
                    ++nominal_events;
            }
        }
        CHECK(fractional_events == 0);
        CHECK(nominal_events > 0);
    } else {
        check_volume_run(samples, *print_object, short_bounds, short_run, per_part ? short_run : tall_run, curved);
        check_volume_run(samples, *print_object, tall_bounds, tall_run, tall_run, curved);
        if (per_part) {
            // The short volume has no layers in the upper two tall-volume groups. A per-volume
            // region filter must not accidentally emit that geometry at those sub-Z values.
            size_t upper_short_events = 0;
            for (const PositiveExtrusion &sample : samples) {
                if (sample.x < short_bounds.min_x - 0.30 || sample.x > short_bounds.max_x + 0.30)
                    continue;
                if (sample.z > print_object->layers()[short_run]->print_z + 0.02)
                    ++upper_short_events;
            }
            CHECK(upper_short_events == 0);
        }
    }
}

TEST_CASE("enable_mixed_color_sublayer reaches the Print config", "[MixedFilament]")
{
    Print print;
    Model model;
    init_print({cube(20)}, print, model, mixed_config(true));

    // The option lives in PrintConfig; if it did not survive Print::apply the slicer would
    // silently fall back to the whole-layer path.
    CHECK(print.config().enable_mixed_color_sublayer.value == true);
    REQUIRE(print.config().filament_is_mixed.values.size() == 3);
    CHECK(print.config().filament_is_mixed.values[2] == true);
    REQUIRE(print.config().filament_mixed_components.values.size() == 3);
    CHECK(print.config().filament_mixed_components.values[2] == "1,2");
}

TEST_CASE("Mixed filament splits layers into sub-layers when the option is on", "[MixedFilament]")
{
    Print print;
    Model model;
    init_print({cube(20)}, print, model, mixed_config(true));
    print.process();

    ToolOrdering &to = const_cast<ToolOrdering &>(print.tool_ordering());
    REQUIRE(!to.layer_tools().empty());

    size_t groups = 0, resolutions = 0;
    count_mixed(to, groups, resolutions);

    INFO("layers=" << to.layer_tools().size() << " groups=" << groups);
    CHECK(groups > 0);
}

TEST_CASE("Mixed filament alternates whole layers when the option is off", "[MixedFilament]")
{
    Print print;
    Model model;
    init_print({cube(20)}, print, model, mixed_config(false));
    print.process();

    ToolOrdering &to = const_cast<ToolOrdering &>(print.tool_ordering());
    REQUIRE(!to.layer_tools().empty());

    size_t groups = 0, resolutions = 0;
    count_mixed(to, groups, resolutions);

    // With splitting off the slot is realized by the deficit round-robin scheduler instead:
    // no sub-layer groups, but a per-layer resolution to one physical component.
    INFO("layers=" << to.layer_tools().size() << " resolutions=" << resolutions);
    CHECK(groups == 0);
    CHECK(resolutions > 0);
}

TEST_CASE("Sub-layer splitting emits the scaled sub-heights into G-code", "[MixedFilament]")
{
    // layer_height 0.2 split 60/40 gives sub-layers of 0.12 and 0.08. The emitter reports the
    // sub-height (not the nominal layer height) in the HEIGHT tag and scales flow to match.
    DynamicPrintConfig config = mixed_config(true);
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"}});

    Print print;
    Model model;
    init_print({cube(20)}, print, model, config);
    print.process();
    const std::string gc = Slic3r::Test::gcode(print);

    REQUIRE(!gc.empty());
    INFO("gcode bytes=" << gc.size());
    CHECK(gc.find(";HEIGHT:0.12") != std::string::npos);
    CHECK(gc.find(";HEIGHT:0.08") != std::string::npos);
}

TEST_CASE("Whole-layer mixing emits only the nominal layer height", "[MixedFilament]")
{
    DynamicPrintConfig config = mixed_config(false);
    config.set_deserialize_strict({{"layer_height", "0.2"}, {"initial_layer_print_height", "0.2"}});

    Print print;
    Model model;
    init_print({cube(20)}, print, model, config);
    print.process();
    const std::string gc = Slic3r::Test::gcode(print);

    REQUIRE(!gc.empty());
    // No sub-layer split, so the 60/40 sub-heights must never appear.
    CHECK(gc.find(";HEIGHT:0.12") == std::string::npos);
    CHECK(gc.find(";HEIGHT:0.08") == std::string::npos);
}

TEST_CASE("By-object prints without mixed filaments keep their used-filament set", "[MixedFilament]")
{
    // With no mixed slot the by-object bookkeeping stays plain: object 2 prints with filament 2,
    // so both filaments are used and no mixed filament is reported.
    DynamicPrintConfig config = multifilament_config(2, {{"print_sequence", "by object"}});
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ {}, { {"extruder", "2"} } };

    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{cube(20), cube(20)}, print, model, config, &overrides);
    REQUIRE(print.objects().size() == 2);
    print.process();

    CHECK(print.get_slice_used_filaments(false) == std::vector<unsigned int>{0, 1});
    CHECK(print.get_slice_used_filaments(true) == std::vector<unsigned int>{0, 1});
    CHECK(print.get_slice_used_mixed_filaments().empty());
}

TEST_CASE("By-layer prints record a mixed slot's components and the slot itself", "[MixedFilament]")
{
    // Control for the by-object case below: the by-layer path publishes the physical
    // components (0-based 0 and 1) as used filaments and the mixed slot (config index 2) as
    // a used mixed filament. By-object prints must report exactly the same.
    Print print;
    Model model;
    init_print({cube(20)}, print, model, mixed_config(false));
    print.process();

    CHECK(print.get_slice_used_filaments(false) == std::vector<unsigned int>{0, 1});
    CHECK(print.get_slice_used_mixed_filaments() == std::vector<unsigned int>{2});
}

TEST_CASE("By-object prints expand a mixed slot to its components in the slice bookkeeping", "[MixedFilament]")
{
    // Sequential prints build their filament lists from unsorted per-object orderings, which
    // still carry the virtual slot (config index 2). The slice-used sets and the published
    // grouping result must see the physical components 0 and 1 instead, and the slot itself
    // must still be reported as a used mixed filament — exactly what the by-layer path yields.
    DynamicPrintConfig config = mixed_config(false);
    config.set_deserialize_strict({{"print_sequence", "by object"}});

    Print print;
    Model model;
    init_print({cube(20), cube(20)}, print, model, config);
    REQUIRE(print.objects().size() == 2);
    print.process();

    const std::vector<unsigned int> components{0, 1};
    CHECK(print.get_slice_used_filaments(false) == components);
    CHECK(print.get_slice_used_filaments(true) == components);
    CHECK(print.get_slice_used_mixed_filaments() == std::vector<unsigned int>{2});

    auto group_result = print.get_layered_nozzle_group_result();
    REQUIRE(group_result != nullptr);
    CHECK(group_result->get_used_filaments() == components);
}

TEST_CASE("By-object G-code lists a mixed slot's components in the filament header", "[MixedFilament]")
{
    DynamicPrintConfig config = mixed_config(false);
    config.set_deserialize_strict({{"print_sequence", "by object"}});

    Print print;
    Model model;
    init_print({cube(20), cube(20)}, print, model, config);
    const std::string gc = Slic3r::Test::gcode(print);

    REQUIRE(!gc.empty());
    // The header names the filaments that must be loaded (components 1 and 2, 1-based),
    // never the virtual slot 3.
    CHECK(gc.find("; filament: 1,2\n") != std::string::npos);
    CHECK(gc.find("; filament: 3") == std::string::npos);
}

TEST_CASE("Print::validate rejects a mixed filament as the wipe tower filament", "[MixedFilament]")
{
    // The validate backstop refuses a mixed (virtual) slot as the wipe tower filament; the GUI hides
    // the slot from that option. Two cubes on physical filaments 1 and 2 make the tower real, and the
    // region roles mixed_config() points at the slot are reset so only the tower uses it.
    DynamicPrintConfig config = mixed_config(false);
    config.set_deserialize_strict({
        {"enable_prime_tower",         "1"},
        {"wipe_tower_x",               "50"}, // inside the 200x200 test bed
        {"wipe_tower_y",               "50"}, // (the default y, 220, is not)
        {"layer_change_gcode",         "G92 E0\n"}, // validate() relative-E reset, as in test_print.cpp's build_cubes
        {"outer_wall_filament_id",     "0"},
        {"inner_wall_filament_id",     "0"},
        {"sparse_infill_filament_id",  "0"},
        {"internal_solid_filament_id", "0"},
        {"top_surface_filament_id",    "0"},
        {"bottom_surface_filament_id", "0"},
    });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{ { {"extruder", "1"} }, { {"extruder", "2"} } };

    SECTION("a physical wipe tower filament validates") {
        config.set_deserialize_strict({{"wipe_tower_filament", "2"}});
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{cube(20), cube(20)}, print, model, config, &overrides);
        REQUIRE(print.has_wipe_tower());
        const StringObjectException err = print.validate();
        INFO(err.string);
        CHECK(err.string.empty());
    }

    SECTION("the mixed slot is refused") {
        config.set_deserialize_strict({{"wipe_tower_filament", "3"}});
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{cube(20), cube(20)}, print, model, config, &overrides);
        REQUIRE(print.has_wipe_tower());
        const StringObjectException err = print.validate();
        CHECK_FALSE(err.string.empty());
        CHECK(err.opt_key == "wipe_tower_filament");
    }
}

TEST_CASE("Print::validate warns when a gradient mixed filament is used without sublayer mixing", "[MixedFilament]")
{
    // A gradient mixed filament only renders its gradient with the process option enabled; without
    // it ToolOrdering prints one whole component per layer and the gradient is dropped silently,
    // so validate() warns whenever the slot actually takes part in the print. The layer-change
    // reset avoids an unrelated relative-extrusion warning, as in the wipe tower test above.
    DynamicPrintConfig config = mixed_config(false);
    config.set_deserialize_strict({
        {"filament_mixed_gradient", "0,0,1"},
        {"layer_change_gcode",       "G92 E0\n"},
    });

    auto count_opt = [](Print &print, const char *opt_key) {
        std::vector<StringObjectException> warnings;
        print.validate(&warnings);
        return std::count_if(warnings.begin(), warnings.end(),
                             [&](const StringObjectException &w) { return w.opt_key == opt_key; });
    };

    SECTION("gradient slot used, sublayer mixing off") {
        Print print;
        Model model;
        init_print({cube(20)}, print, model, config);
        std::vector<StringObjectException> warnings;
        const StringObjectException err = print.validate(&warnings);
        CHECK(err.string.empty());
        const auto it = std::find_if(warnings.begin(), warnings.end(), [](const StringObjectException &w) {
            return w.opt_key == "enable_mixed_color_sublayer";
        });
        REQUIRE(it != warnings.end());
        CHECK(it->is_warning);
        CHECK(std::count_if(warnings.begin(), warnings.end(), [](const StringObjectException &w) {
                  return w.opt_key == "enable_mixed_color_sublayer";
              }) == 1);
    }

    SECTION("sublayer mixing on") {
        config.set_deserialize_strict({{"enable_mixed_color_sublayer", "1"}});
        Print print;
        Model model;
        init_print({cube(20)}, print, model, config);
        CHECK(count_opt(print, "enable_mixed_color_sublayer") == 0);
    }

    SECTION("gradient flag off") {
        config.set_deserialize_strict({{"filament_mixed_gradient", "0,0,0"}});
        Print print;
        Model model;
        init_print({cube(20)}, print, model, config);
        CHECK(count_opt(print, "enable_mixed_color_sublayer") == 0);
    }

    SECTION("mixed slot not used") {
        config.set_deserialize_strict({
            {"outer_wall_filament_id",     "0"},
            {"inner_wall_filament_id",     "0"},
            {"sparse_infill_filament_id",  "0"},
            {"internal_solid_filament_id", "0"},
            {"top_surface_filament_id",    "0"},
            {"bottom_surface_filament_id", "0"},
        });
        Print print;
        Model model;
        const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{{{ "extruder", "1" }}};
        init_print(std::vector<TriangleMesh>{cube(20)}, print, model, config, &overrides);
        CHECK(count_opt(print, "enable_mixed_color_sublayer") == 0);
    }
}
