#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
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
                                                        const GCodeConfig &config,
                                                        int initial_tool)
{
    GCodeReader reader;
    reader.apply_config(config);
    std::vector<PositiveExtrusion> result;
    int current_tool = initial_tool;
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.cmd_is("T0")) {
            current_tool = 0;
            return;
        }
        if (line.cmd_is("T1")) {
            current_tool = 1;
            return;
        }
        // BBL multi-extruder output selects a filament with M1020 S<filament_id> H<nozzle_id>;
        // S is the material/component identity used by this fixture, while H is physical-nozzle
        // context and must not be used to classify the deposited material.
        if (line.cmd_is("M1020")) {
            std::istringstream command(line.raw());
            std::string opcode, token;
            int selected_tool = -1;
            command >> opcode;
            while (command >> token) {
                if (!token.empty() && token.front() == ';')
                    break;
                if (token.size() <= 1 || token.front() != 'S')
                    continue;
                std::istringstream id_stream(token.substr(1));
                char trailing = '\0';
                int id = -1;
                if ((id_stream >> id) && id >= 0 && !(id_stream >> trailing))
                    selected_tool = id;
            }
            current_tool = selected_tool;
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

    const unsigned int initial_tool = print.tool_ordering().first_extruder();
    REQUIRE(initial_tool != static_cast<unsigned int>(-1));
    const auto samples = read_positive_extrusions(output, print.config(), static_cast<int>(initial_tool));
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

namespace mixed_bbl_label_contracts {

using namespace mixed_gradient_gcode_contracts;

struct LabelProtocolEvidence {
    bool   valid = true;
    size_t bbl_commands = 0;
    size_t bbl_payloads = 0;
    size_t bbl_end_commands = 0;
    size_t marlin_starts = 0;
    size_t marlin_ends = 0;
    size_t bbl_pairs = 0;
    size_t bbl_end_pairs = 0;
    bool   bbl_ids_match = false;
    bool   bbl_payloads_bit0 = true;
    bool   marlin_ids_match = false;
};

bool parse_nonnegative_id(const std::string &text, long long &id)
{
    std::istringstream input(text);
    char trailing = '\0';
    if (!(input >> id) || id < 0 || (input >> trailing))
        return false;
    return true;
}

bool parse_bbl_bit0_payload(const std::string &text)
{
    std::istringstream command(text);
    std::string opcode;
    std::string payload;
    std::string trailing;
    if (!(command >> opcode >> payload) || opcode != "M624" || payload != "AQAAAAAAAAA=")
        return false;
    return !(command >> trailing);
}

LabelProtocolEvidence parse_label_protocol(const std::string &gcode)
{
    LabelProtocolEvidence result;
    std::vector<std::string> lines;
    std::istringstream input(gcode);
    std::string line;
    while (std::getline(input, line))
        lines.push_back(line);

    constexpr const char *bbl_start_prefix = "; start printing object, unique label id: ";
    constexpr const char *bbl_end_prefix   = "; stop printing object, unique label id: ";
    bool bbl_open = false;
    long long bbl_open_id = -1;
    bool marlin_open = false;
    std::vector<long long> marlin_start_ids;

    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string &current = lines[i];
        if (current.rfind(bbl_start_prefix, 0) == 0) {
            long long id = -1;
            if (bbl_open || !parse_nonnegative_id(current.substr(std::char_traits<char>::length(bbl_start_prefix)), id)) {
                result.valid = false;
            } else {
                bbl_open = true;
                bbl_open_id = id;
                ++result.bbl_pairs;
            }
            const bool valid_payload = i + 1 < lines.size() && parse_bbl_bit0_payload(lines[i + 1]);
            if (!valid_payload) {
                result.valid = false;
                result.bbl_payloads_bit0 = false;
            } else {
                ++result.bbl_commands;
                ++result.bbl_payloads;
                ++i;
            }
            continue;
        }
        if (current.rfind(bbl_end_prefix, 0) == 0) {
            long long id = -1;
            if (!bbl_open || !parse_nonnegative_id(current.substr(std::char_traits<char>::length(bbl_end_prefix)), id) || id != bbl_open_id)
                result.valid = false;
            else {
                bbl_open = false;
                ++result.bbl_end_pairs;
            }
            if (i + 1 >= lines.size() || lines[i + 1] != "M625") {
                result.valid = false;
            } else {
                ++result.bbl_end_commands;
                ++i;
            }
            continue;
        }
        if (current.rfind("M624 ", 0) == 0 || current == "M625") {
            // A protocol command without the exact adjacent object marker is not a valid boundary.
            result.valid = false;
            continue;
        }
        if (current.rfind("M486 S", 0) == 0) {
            const std::string payload = current.substr(6);
            if (payload == "-1") {
                ++result.marlin_ends;
                if (!marlin_open)
                    result.valid = false;
                else
                    marlin_open = false;
            } else {
                long long id = -1;
                if (!parse_nonnegative_id(payload, id) || marlin_open) {
                    result.valid = false;
                } else {
                    marlin_open = true;
                    marlin_start_ids.push_back(id);
                    ++result.marlin_starts;
                }
            }
        }
    }

    if (bbl_open || marlin_open || result.bbl_pairs != result.bbl_end_pairs || result.marlin_starts != result.marlin_ends)
        result.valid = false;
    if (result.bbl_payloads != result.bbl_commands || !result.bbl_payloads_bit0)
        result.valid = false;
    result.bbl_ids_match = result.bbl_pairs == result.bbl_end_pairs;
    result.marlin_ids_match = result.marlin_starts == result.marlin_ends;
    // M486 S-1 carries no object id; the ordered state machine above pairs it with the preceding
    // start. This one-instance fixture must nevertheless keep one public instance id throughout.
    for (size_t i = 1; result.marlin_ids_match && i < marlin_start_ids.size(); ++i)
        result.marlin_ids_match = marlin_start_ids[i] == marlin_start_ids.front();
    return result;
}

struct MixedLabelExport {
    LabelProtocolEvidence labels;
    size_t                 mixed_groups = 0;
    size_t                 mixed_extra_boundaries = 0;
    double                 positive_e = 0.;
    double                 positive_xy = 0.;
    std::vector<double>    layer_xy;
};

double positive_xy_at_z(const std::vector<PositiveExtrusion> &samples, double target_z)
{
    double result = 0.;
    for (const PositiveExtrusion &sample : samples)
        if (std::abs(sample.z - target_z) <= 0.002)
            result += sample.xy;
    return result;
}

MixedLabelExport export_mixed_labels(bool bbl, bool split_sublayers)
{
    DynamicPrintConfig config = mixed_config(split_sublayers);
    config.set_deserialize_strict({
        {"gcode_flavor",                 "marlin2"},
        {"exclude_object",               true},
        {"gcode_label_objects",          false},
        {"layer_height",                 "0.2"},
        {"initial_layer_print_height",   "0.2"},
        {"wall_loops",                   "1"},
        {"top_shell_layers",             "0"},
        {"bottom_shell_layers",          "0"},
        {"sparse_infill_density",        "0%"},
        {"skirt_loops",                  "0"},
        {"brim_type",                    "no_brim"},
        {"layer_change_gcode",           "G92 E0\n"},
        {"filament_flow_ratio",          "1,1,1"},
        {"filament_mixed_gradient",      split_sublayers ? "0,0,1" : "0,0,0"},
        {"filament_mixed_gradient_range",";;0.8,0.2"},
        {"filament_mixed_gradient_per_part", "0,0,0"},
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
    print.is_BBL_printer() = bbl;

    // The label encoder's bit position is derived from the public print instance inventory.
    // Keep this fixture to one object instance so bit 0 has an unambiguous meaning in M624.
    REQUIRE(print.num_object_instances() == 1);

    const StringObjectException validation = print.validate();
    INFO("validation error: " << validation.string);
    REQUIRE(validation.string.empty());
    const std::string output = Slic3r::Test::gcode(print);
    REQUIRE_FALSE(output.empty());
    REQUIRE(print.objects().size() == 1);
    const PrintObject *print_object = print.objects().front();
    REQUIRE(print_object != nullptr);
    REQUIRE(print_object->instances().size() == 1);
    REQUIRE(print_object->instances().front().model_instance != nullptr);
    REQUIRE(print_object->layers().size() == 8);

    MixedLabelExport result;
    result.labels = parse_label_protocol(output);
    REQUIRE(result.labels.valid);
    const unsigned int initial_tool = print.tool_ordering().first_extruder();
    REQUIRE(initial_tool != static_cast<unsigned int>(-1));
    const auto samples = read_positive_extrusions(output, print.config(), static_cast<int>(initial_tool));
    REQUIRE_FALSE(samples.empty());
    for (const PositiveExtrusion &sample : samples) {
        REQUIRE(std::isfinite(sample.e));
        REQUIRE(std::isfinite(sample.xy));
        REQUIRE(sample.e > 0.);
        REQUIRE(sample.xy > 0.01);
        result.positive_e += sample.e;
        result.positive_xy += sample.xy;
    }
    bool all_motion_finite = true;
    size_t motion_lines = 0;
    GCodeReader motion_reader;
    motion_reader.apply_config(print.config());
    motion_reader.parse_buffer(output, [&all_motion_finite, &motion_lines](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1")))
            return;
        if (!line.has_x() && !line.has_y() && !line.has_z() && !line.has_e() && !line.has_f())
            return;
        ++motion_lines;
        all_motion_finite = all_motion_finite && std::isfinite(line.new_X(self)) && std::isfinite(line.new_Y(self))
            && std::isfinite(line.new_Z(self)) && std::isfinite(line.new_E(self)) && std::isfinite(line.new_F(self));
    });
    REQUIRE(all_motion_finite);
    REQUIRE(motion_lines > 0);
    REQUIRE(result.positive_e > 0.);
    REQUIRE(result.positive_xy > 0.);

    const ToolOrdering &ordering = print.tool_ordering();
    const size_t ordered_layer_count = static_cast<size_t>(ordering.end() - ordering.begin());
    for (const LayerTools &layer_tools : ordering) {
        for (const LayerTools::MixedSubLayerGroup &group : layer_tools.mixed_sub_layer_groups) {
            REQUIRE(group.mixed_slot_0based == 2);
            REQUIRE(group.components_0based == std::vector<unsigned int>{0, 1});
            REQUIRE(group.sub_heights.size() == 2);
            CHECK_THAT(group.sub_heights[0] + group.sub_heights[1], Catch::Matchers::WithinAbs(group.layer_height, 1e-7));
            ++result.mixed_groups;
            result.mixed_extra_boundaries += group.components_0based.size() - 1;
        }
    }
    result.layer_xy.resize(print_object->layers().size(), 0.);
    for (size_t layer_idx = 0; layer_idx < print_object->layers().size(); ++layer_idx) {
        const Layer *layer = print_object->layers()[layer_idx];
        REQUIRE(layer != nullptr);
        if (!split_sublayers) {
            result.layer_xy[layer_idx] = positive_xy_at_z(samples, layer->print_z);
            continue;
        }
        if (layer_idx == 0) {
            result.layer_xy[layer_idx] = positive_xy_at_z(samples, layer->print_z);
            continue;
        }
        REQUIRE(layer_idx < ordered_layer_count);
        const LayerTools::MixedSubLayerGroup *group = nullptr;
        for (const LayerTools::MixedSubLayerGroup &candidate : ordering.begin()[layer_idx].mixed_sub_layer_groups)
            if (candidate.mixed_slot_0based == 2) {
                REQUIRE(group == nullptr);
                group = &candidate;
            }
        REQUIRE(group != nullptr);
        const double lower_z = layer->print_z - group->layer_height + group->sub_heights.front();
        result.layer_xy[layer_idx] = positive_xy_at_z(samples, lower_z) + positive_xy_at_z(samples, layer->print_z);
    }
    if (split_sublayers) {
        REQUIRE(result.mixed_groups == 7);
        REQUIRE(result.mixed_extra_boundaries == 7);
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
        check_volume_run(samples, *print_object, short_bounds, 5, result.mixed_groups, false);
        check_volume_run(samples, *print_object, tall_bounds, result.mixed_groups, result.mixed_groups, false);
    } else {
        CHECK(result.mixed_groups == 0);
        CHECK(result.mixed_extra_boundaries == 0);
    }
    return result;
}

} // namespace mixed_bbl_label_contracts

TEST_CASE("BBL mixed sublayers pair object exclusion labels with their boundaries",
          "[GCode][MixedFilament][MixedBBLLabels]")
{
    using namespace mixed_bbl_label_contracts;

    // Every row constructs and processes its own model. The exporter stores pending object labels
    // in GCodeWriter, so toggling BBL or sublayer mode on a processed Print would not be an A/B test.
    const MixedLabelExport bbl = export_mixed_labels(true, true);
    const MixedLabelExport marlin_split = export_mixed_labels(false, true);
    const MixedLabelExport marlin_whole = export_mixed_labels(false, false);

    INFO("BBL pairs=" << bbl.labels.bbl_pairs
         << " Marlin split=" << marlin_split.labels.marlin_starts
         << " Marlin whole=" << marlin_whole.labels.marlin_starts
         << " mixed extra=" << bbl.mixed_extra_boundaries);
    REQUIRE(bbl.labels.bbl_pairs > 0);
    REQUIRE(bbl.labels.bbl_pairs == bbl.labels.bbl_end_pairs);
    REQUIRE(bbl.labels.bbl_ids_match);
    REQUIRE(bbl.labels.bbl_payloads == bbl.labels.bbl_pairs);
    REQUIRE(bbl.labels.bbl_payloads_bit0);
    CHECK(bbl.labels.bbl_commands == bbl.labels.bbl_pairs);
    CHECK(bbl.labels.bbl_end_commands == bbl.labels.bbl_pairs);
    CHECK(bbl.labels.marlin_starts == 0);
    CHECK(bbl.labels.marlin_ends == 0);

    REQUIRE(marlin_split.labels.marlin_starts > 0);
    REQUIRE(marlin_split.labels.marlin_starts == marlin_split.labels.marlin_ends);
    REQUIRE(marlin_split.labels.marlin_ids_match);
    CHECK(marlin_split.labels.bbl_commands == 0);
    CHECK(marlin_split.labels.bbl_end_commands == 0);

    REQUIRE(marlin_whole.labels.marlin_starts > 0);
    REQUIRE(marlin_whole.labels.marlin_starts == marlin_whole.labels.marlin_ends);
    REQUIRE(marlin_whole.labels.marlin_ids_match);
    CHECK(marlin_whole.labels.bbl_commands == 0);
    CHECK(marlin_whole.labels.bbl_end_commands == 0);

    // The same fresh mixed print uses one ordinary boundary per layer in whole-layer mode. The
    // public ToolOrdering metadata independently predicts one extra boundary per two-component
    // mixed sublayer; this difference isolates mixed-sublayer labels from header/ordinary markers.
    CHECK(bbl.labels.bbl_pairs == marlin_split.labels.marlin_starts);
    CHECK(marlin_split.labels.marlin_starts == marlin_whole.labels.marlin_starts + bbl.mixed_extra_boundaries);

    REQUIRE(bbl.positive_e > 0.);
    REQUIRE(bbl.positive_xy > 0.);
    REQUIRE(marlin_split.positive_e > 0.);
    REQUIRE(marlin_split.positive_xy > 0.);
    REQUIRE(marlin_whole.positive_e > 0.);
    REQUIRE(marlin_whole.positive_xy > 0.);
    CHECK_THAT(bbl.positive_e, Catch::Matchers::WithinAbs(marlin_split.positive_e, 1e-6));
    CHECK_THAT(bbl.positive_xy, Catch::Matchers::WithinAbs(marlin_split.positive_xy, 1e-6));
    CHECK_THAT(bbl.positive_e, Catch::Matchers::WithinRel(marlin_whole.positive_e, 1e-3));
    REQUIRE(bbl.layer_xy.size() == marlin_split.layer_xy.size());
    REQUIRE(marlin_whole.layer_xy.size() == bbl.layer_xy.size());
    for (size_t layer_idx = 0; layer_idx < bbl.layer_xy.size(); ++layer_idx) {
        CHECK_THAT(bbl.layer_xy[layer_idx], Catch::Matchers::WithinAbs(marlin_split.layer_xy[layer_idx], 1e-6));
        const double expected_xy = layer_idx == 0 ? marlin_whole.layer_xy[layer_idx] : 2. * marlin_whole.layer_xy[layer_idx];
        CHECK_THAT(bbl.layer_xy[layer_idx], Catch::Matchers::WithinRel(expected_xy, 1e-3));
    }
}
