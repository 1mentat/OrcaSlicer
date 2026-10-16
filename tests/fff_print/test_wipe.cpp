#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include <cstddef>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/calib.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include <map>
#include <set>
#include <tuple>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Layer.hpp"

#include "test_helpers.hpp"
#include "libslic3r/Polyline.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

DynamicPrintConfig wipe_config(const char *wall_generator, bool wipe_inward,
                               const char *wipe_inward_distance = "50%",
                               const char *seam_gap = "10%", bool wipe_on_loops = false,
                               const char *wall_loops = "2",
                               const char *wall_sequence = "inner wall/outer wall",
                               bool alternate_extra_wall = false,
                               const char *sparse_infill_density = "0%",
                               const char *seam_position = "aligned")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "nozzle_diameter",             "0.4" },
        { "layer_height",                "0.2" },
        { "initial_layer_print_height",  "0.2" },
        { "line_width",                  "0.45" },
        { "outer_wall_line_width",       "0" }, // Orca: Auto must use the actual path width.
        { "wall_loops",                  wall_loops },
        { "wall_generator",              wall_generator },
        { "wall_sequence",               wall_sequence },
        { "top_shell_layers",            "0" },
        { "bottom_shell_layers",         "0" },
        { "sparse_infill_density",       sparse_infill_density },
        { "seam_position",               seam_position },
        { "seam_gap",                    seam_gap },
        { "wipe",                        "1" },
        { "wipe_distance",               "2" },
        { "retraction_length",           "0.8" },
        { "retract_when_changing_layer", "1" },
        { "wipe_inward",                 wipe_inward ? "1" : "0" },
        { "wipe_inward_distance",        wipe_inward_distance },
        { "wipe_on_loops",               wipe_on_loops ? "1" : "0" },
        { "alternate_extra_wall",        alternate_extra_wall ? "1" : "0" },
        { "gcode_comments",              "1" },
        { "machine_start_gcode",         "" },
        { "machine_end_gcode",           "" },
    });
    return config;
}

struct WipeTrajectory {
    Vec2d start;
    double z;
    std::vector<Vec2d> destinations;
};

std::vector<WipeTrajectory> wipe_trajectories(const std::string &gcode)
{
    const std::string &start_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const std::string &end_tag   = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    std::vector<WipeTrajectory> trajectories;
    bool in_wipe = false;

    GCodeReader parser;
    parser.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find(start_tag) != std::string_view::npos) {
            in_wipe = true;
            trajectories.push_back({Vec2d(self.x(), self.y()), self.z(), {}});
            return;
        }
        if (comment.find(end_tag) != std::string_view::npos) {
            in_wipe = false;
            return;
        }
        if (in_wipe && line.dist_XY(self) > EPSILON)
            trajectories.back().destinations.emplace_back(line.new_X(self), line.new_Y(self));
    });
    return trajectories;
}

std::vector<Vec2d> wipe_destinations(const std::string &gcode)
{
    std::vector<Vec2d> destinations;
    for (const WipeTrajectory &trajectory : wipe_trajectories(gcode))
        destinations.insert(destinations.end(), trajectory.destinations.begin(), trajectory.destinations.end());
    return destinations;
}

bool trajectories_differ(const std::vector<Vec2d> &lhs, const std::vector<Vec2d> &rhs)
{
    if (lhs.size() != rhs.size())
        return true;
    for (size_t i = 0; i < lhs.size(); ++i)
        if ((lhs[i] - rhs[i]).norm() > 0.01)
            return true;
    return false;
}

double trajectory_length(const WipeTrajectory &trajectory)
{
    double length = 0.;
    Vec2d previous = trajectory.start;
    for (const Vec2d &destination : trajectory.destinations) {
        length += (destination - previous).norm();
        previous = destination;
    }
    return length;
}

} // namespace

TEST_CASE("Wipe retraction preserves fractional speed with inward wipe disabled", "[Wipe][Regression]")
{
    const char *retraction_speed = GENERATE("25.25", "25.5", "25.75");
    const char *relative_e = GENERATE("0", "1");
    INFO("retraction speed: " << retraction_speed);
    INFO("relative E: " << relative_e);
    DynamicPrintConfig config = wipe_config("classic", false);
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", relative_e},
        {"retraction_speed", retraction_speed},
        {"retraction_length", "0.8"},
        {"retract_before_wipe", "0%"},
        {"retract_after_wipe", "0%"},
        {"role_based_wipe_speed", "0"},
        {"wipe_speed", "100"},
        {"wipe_distance", "2"},
    });
    const std::string output = slice({make_cube(10., 10., 1.)}, config);
    const auto &start_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const auto &end_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    double before_wipe = 0.;
    double during_wipe = 0.;
    bool in_wipe = false;
    bool complete = false;
    GCodeReader parser;
    parser.apply_config(config);
    parser.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (complete)
            return;
        if (line.comment().find(start_tag) != std::string_view::npos) {
            in_wipe = true;
        } else if (in_wipe && line.comment().find(end_tag) != std::string_view::npos) {
            complete = true;
        } else if (line.retracting(self)) {
            (in_wipe ? during_wipe : before_wipe) -= line.dist_E(self);
        } else if (line.extruding(self)) {
            before_wipe = 0.;
        }
    });

    REQUIRE(complete);
    // At 100 mm/s, the 2 mm wipe lasts 0.02 seconds. The remaining part of
    // the configured 0.8 mm retraction must be emitted before that wipe.
    const double expected_during = std::stod(retraction_speed) * 2. / 100.;
    CHECK_THAT(during_wipe, Catch::Matchers::WithinAbs(expected_during, 0.00005));
    CHECK_THAT(before_wipe, Catch::Matchers::WithinAbs(0.8 - expected_during, 0.00005));
}

TEST_CASE("Inward wipe respects the minimum travel for retraction and Z hop", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    const char *relative_e = GENERATE("0", "1");
    const char *reduce_crossing_wall = GENERATE("0", "1");
    const char *minimum_travel = GENERATE("5", "0");
    CAPTURE(wall_generator, relative_e, reduce_crossing_wall, minimum_travel);
    DynamicPrintConfig config = wipe_config(
        wall_generator, true, "50%", "10%", false, "3", "inner-outer-inner wall");
    config.set_deserialize_strict({
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", relative_e},
        {"reduce_crossing_wall", reduce_crossing_wall},
        {"retraction_minimum_travel", minimum_travel},
        {"retract_when_changing_layer", "0"},
        {"use_firmware_retraction", "0"},
        {"retract_before_wipe", "0%"},
        {"retract_after_wipe", "0%"},
        {"retraction_speed", "25.5"},
        {"role_based_wipe_speed", "0"},
        {"wipe_speed", "100"},
        {"z_hop", "0.4"},
        {"retract_lift_above", "0"},
        {"retract_lift_below", "0"},
    });
    config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric{zhtNormal});
    config.set_key_value("retract_lift_enforce", new ConfigOptionEnumsGeneric{rletAllSurfaces});
    const std::string output = slice({make_cube(10., 10., 1.)}, config);
    const auto &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    const auto &start_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const auto &end_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    ExtrusionRole role = erNone;
    bool after_outer_wall = false;
    bool in_wipe = false;
    size_t transitions = 0;
    size_t same_layer_transitions = 0;
    size_t inward_wipes = 0;
    double retraction = 0.;
    double lift = 0.;
    double outer_z = 0.;
    GCodeReader parser;
    parser.apply_config(config);
    parser.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find(role_tag) == 0)
            role = ExtrusionEntity::string_to_role(line.comment().substr(role_tag.size()));
        if (line.comment().find(start_tag) == 0) {
            in_wipe = true;
            if (after_outer_wall)
                ++inward_wipes;
        } else if (line.comment().find(end_tag) == 0) {
            in_wipe = false;
        }
        if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
            if (role == erExternalPerimeter) {
                after_outer_wall = true;
                retraction = lift = 0.;
                outer_z = line.new_Z(self);
            } else if (after_outer_wall) {
                REQUIRE(role == erPerimeter);
                ++transitions;
                const double layer_rise = std::max(0., double(self.z()) - outer_z);
                if (layer_rise < EPSILON)
                    ++same_layer_transitions;
                // A 5 mm threshold suppresses retraction across a few wall widths.
                // A zero threshold still permits the ordinary retract and lift.
                const bool retract = std::stod(minimum_travel) == 0.;
                CHECK_THAT(retraction, Catch::Matchers::WithinAbs(retract ? 0.8 : 0., 0.00005));
                // Exclude an ordinary layer change from the accumulated upward motion.
                CHECK_THAT(lift - layer_rise, Catch::Matchers::WithinAbs(retract ? 0.4 : 0., 0.001));
                after_outer_wall = false;
            }
        } else if (after_outer_wall) {
            if (line.retracting(self))
                retraction -= line.dist_E(self);
            lift += std::max(0., double(line.dist_Z(self)));
            if (in_wipe)
                CHECK_THAT(line.dist_E(self), Catch::Matchers::WithinAbs(0., 0.00005));
        }
    });
    // The 1 mm cube has five 0.2 mm layers: every outer wall must still wipe.
    REQUIRE(transitions == 5);
    REQUIRE(same_layer_transitions >= 4);
    REQUIRE(inward_wipes == transitions);
}

TEST_CASE("Changing inward wipe settings preserves the sliced geometry", "[Wipe][Regression]")
{
    const char *key = GENERATE("wipe_inward", "wipe_inward_distance");
    DynamicPrintConfig config = wipe_config("classic", false);
    Print print;
    Model model;
    init_print({make_cube(10., 10., 1.)}, print, model, config);
    gcode(print);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.is_step_done(posPerimeters));
    REQUIRE(object.is_step_done(posInfill));
    REQUIRE(print.is_step_done(psWipeTower));
    REQUIRE(print.is_step_done(psGCodeExport));

    DynamicPrintConfig changed = config;
    changed.set_deserialize_strict({{key, std::string(key) == "wipe_inward" ? "1" : "75%"}});
    print.apply(model, changed);

    CHECK(print.objects().front()->is_step_done(posPerimeters));
    CHECK(print.objects().front()->is_step_done(posInfill));
    CHECK(print.is_step_done(psWipeTower));
    CHECK_FALSE(print.is_step_done(psGCodeExport));
}

TEST_CASE("Retraction and pressure advance calibration suppress inward wipe overrides", "[Wipe][Regression]")
{
    const auto mode = GENERATE(CalibMode::Calib_None, CalibMode::Calib_PA_Tower,
                              CalibMode::Calib_Auto_PA_Line, CalibMode::Calib_Retraction_tower,
                              CalibMode::Calib_Flow_Rate);
    const char *wall_generator = GENERATE("classic", "arachne");
    const bool per_object = GENERATE(false, true);
    INFO("calibration mode: " << int(mode) << ", wall generator: " << wall_generator
         << ", per-object override: " << per_object);

    const auto trajectories = [&](bool inward) {
        DynamicPrintConfig config = wipe_config(wall_generator, inward && !per_object);
        const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
            {{"wipe_inward", inward ? "1" : "0"}}
        };
        Print print;
        Model model;
        init_print({make_cube(10., 10., 1.)}, print, model, config, per_object ? &overrides : nullptr);
        Calib_Params params;
        params.mode = mode;
        params.start = 0.2;
        params.end = 0.4;
        params.step = 0.1;
        print.set_calib_params(params);
        return wipe_destinations(gcode(print));
    };

    const auto regular = trajectories(false);
    const auto inward = trajectories(true);
    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(inward.empty());
    // Other calibration modes and ordinary prints must still honor the option.
    const bool should_differ = mode == CalibMode::Calib_None || mode == CalibMode::Calib_Flow_Rate;
    CHECK(trajectories_differ(regular, inward) == should_differ);
}

TEST_CASE("Inactive inward wipe settings preserve the exported trajectory", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    const bool disable_wiping = GENERATE(false, true);
    DynamicPrintConfig regular = wipe_config(wall_generator, false);
    DynamicPrintConfig inward = wipe_config(wall_generator, true, disable_wiping ? "50%" : "0");
    if (disable_wiping) {
        regular.set_deserialize_strict({{"wipe", "0"}});
        inward.set_deserialize_strict({{"wipe", "0"}});
    }
    const auto regular_paths = wipe_destinations(slice({make_cube(10., 10., 1.)}, regular));
    const auto inward_paths = wipe_destinations(slice({make_cube(10., 10., 1.)}, inward));
    if (!disable_wiping)
        REQUIRE_FALSE(regular_paths.empty());
    CHECK_FALSE(trajectories_differ(regular_paths, inward_paths));
}

TEST_CASE("Inward wipe changes the exported trajectory when outer wall width is Auto", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, false)));
    const std::vector<Vec2d> inward = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, true)));

    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(inward.empty());
    REQUIRE(trajectories_differ(regular, inward));
}

TEST_CASE("Inward wipe recognizes an external wall starting on an overhang", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    const bool inward = GENERATE(false, true);
    CAPTURE(wall_generator, inward);
    const auto config = wipe_config(wall_generator, inward, "50%", "0%", false,
                                    "3", "inner-outer-inner wall", false, "0%", "back");
    Print print;
    Model model;
    init_print({make_cube(10., 10., 1.)}, print, model, config);
    print.process();
    size_t mixed_loops = 0;
    const auto mark_overhangs = [&](auto &&self, ExtrusionEntity *entity) -> void {
        if (auto *collection = dynamic_cast<ExtrusionEntityCollection *>(entity)) {
            for (ExtrusionEntity *child : collection->entities)
                self(self, child);
        } else if (auto *loop = dynamic_cast<ExtrusionLoop *>(entity); loop && is_external_perimeter(loop->role())) {
            // Keep the printed geometry intact and give the back seam overhang
            // roles. The front edge remains an ordinary external-wall segment.
            ExtrusionPaths paths;
            bool has_overhang = false;
            bool has_external = false;
            for (const ExtrusionPath &source : loop->paths) {
                for (size_t i = 1; i < source.polyline.points.size(); ++i) {
                    ExtrusionPath path = source;
                    path.polyline.points = {source.polyline.points[i - 1], source.polyline.points[i]};
                    const bool overhang = path.polyline.points.front().y() > 0 || path.polyline.points.back().y() > 0;
                    path.set_extrusion_role(overhang ? erOverhangPerimeter : erExternalPerimeter);
                    has_overhang |= overhang;
                    has_external |= !overhang;
                    paths.push_back(std::move(path));
                }
            }
            REQUIRE(has_overhang);
            REQUIRE(has_external);
            loop->paths = std::move(paths);
            ++mixed_loops;
        }
    };
    for (const PrintObject *object : print.objects())
        for (Layer *layer : object->layers())
            for (LayerRegion *region : layer->regions())
                mark_overhangs(mark_overhangs, &region->perimeters);
    REQUIRE(mixed_loops > 0);

    bool has_inward_wipe = false;
    for (const WipeTrajectory &trajectory : wipe_trajectories(gcode(print))) {
        if (trajectory.destinations.empty())
            continue;
        const Vec2d move = trajectory.destinations.front() - trajectory.start;
        if (trajectory.start.x() > 4. && trajectory.start.y() > 4. && move.x() < -0.05 && move.y() < -0.05)
            has_inward_wipe = true;
    }
    CHECK(has_inward_wipe == inward);
}

TEST_CASE("Inward wipe keeps its offset when seam gap is zero", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, false, "50%", "0%")));
    const std::vector<Vec2d> inward = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, true, "50%", "0%")));

    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(inward.empty());
    REQUIRE(trajectories_differ(regular, inward));
}

TEST_CASE("Inward wipe is retained across layers with a back seam", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const DynamicPrintConfig inward_config = wipe_config(
        wall_generator, true, "50%", "0%", false, "3", "inner-outer-inner wall", false, "0%", "back");
    const std::vector<WipeTrajectory> inward = wipe_trajectories(slice({make_cube(27., 27., 1.)}, inward_config));

    REQUIRE_FALSE(inward.empty());
    std::map<double, bool> inward_wipe_by_layer;
    for (const WipeTrajectory &trajectory : inward) {
        bool &has_inward_wipe = inward_wipe_by_layer[trajectory.z];
        if (trajectory.destinations.empty())
            continue;
        const Vec2d first_move = trajectory.destinations.front() - trajectory.start;
        // Orca: a back seam lands on the cube's positive-X/positive-Y corner.
        // Its inward wipe must move diagonally away from both external faces.
        has_inward_wipe = has_inward_wipe ||
            (trajectory.start.x() > 13. && trajectory.start.y() > 13. &&
             first_move.x() < -0.05 && first_move.y() < -0.05);
    }
    REQUIRE(inward_wipe_by_layer.size() == 5);
    for (const auto &[z, has_inward_wipe] : inward_wipe_by_layer) {
        INFO("layer Z: " << z);
        REQUIRE(has_inward_wipe);
    }
}

TEST_CASE("Literal inward wipe distance is clamped to the outer wall width", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, false)));
    const std::vector<Vec2d> full_width = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, true, "100%")));
    const std::vector<Vec2d> oversized = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, true, "2")));

    REQUIRE_FALSE(full_width.empty());
    REQUIRE(trajectories_differ(regular, full_width));
    REQUIRE(oversized.size() == full_width.size());
    for (size_t i = 0; i < full_width.size(); ++i)
        REQUIRE_THAT((oversized[i] - full_width[i]).norm(), Catch::Matchers::WithinAbs(0., 0.01));
}

TEST_CASE("Inward wipe is not applied without an adjacent wall", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, false, "50%", "10%", false, "1")));
    const std::vector<Vec2d> inward = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(wall_generator, true, "50%", "10%", false, "1")));

    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(trajectories_differ(regular, inward));
}

TEST_CASE("Inward wipe uses an alternate extra wall when the configured wall count is one", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const DynamicPrintConfig regular_config = wipe_config(
        wall_generator, false, "50%", "10%", false, "1", "inner wall/outer wall", true, "15%");
    const DynamicPrintConfig inward_config = wipe_config(
        wall_generator, true, "50%", "10%", false, "1", "inner wall/outer wall", true, "15%");
    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, regular_config));
    const std::vector<Vec2d> inward = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, inward_config));

    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(inward.empty());
    REQUIRE(trajectories_differ(regular, inward));
}

TEST_CASE("Inward wipe is not applied before the adjacent wall is printed", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    INFO("wall generator: " << wall_generator);

    const std::vector<Vec2d> regular = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(
            wall_generator, false, "50%", "10%", false, "2", "outer wall/inner wall")));
    const std::vector<Vec2d> inward = wipe_destinations(
        slice({make_cube(10., 10., 1.)}, wipe_config(
            wall_generator, true, "50%", "10%", false, "2", "outer wall/inner wall")));

    REQUIRE_FALSE(regular.empty());
    REQUIRE_FALSE(trajectories_differ(regular, inward));
}

TEST_CASE("Wipe on loops preserves the corner move with inward wipe disabled", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    const char *nozzle_diameter = GENERATE("0.4", "0.8");
    const char *comments = GENERATE("0", "1");
    CAPTURE(comments);
    INFO("wall generator: " << wall_generator << ", nozzle diameter: " << nozzle_diameter);
    // A closed square gives a 90-degree material-side corner at the seam.
    DynamicPrintConfig config = wipe_config(wall_generator, false, "50%", "0", true);
    config.set_deserialize_strict({{"nozzle_diameter", nozzle_diameter}, {"seam_position", "nearest"},
                                   {"gcode_comments", comments}});
    const std::string output = slice({make_cube(10., 10., 1.)}, config);
    const auto &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    const auto &wipe_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    ExtrusionRole role = erNone;
    std::vector<Vec2d> loop;
    bool after_extrusion = false;
    size_t moves = 0;
    GCodeReader parser;
    parser.apply_config(config);
    parser.parse_buffer(output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.comment().find(role_tag) == 0) {
            role = ExtrusionEntity::string_to_role(line.comment().substr(role_tag.size()));
            loop.clear();
            after_extrusion = false;
        }
        if (line.comment().find(wipe_tag) == 0)
            after_extrusion = false;
        if (role != erExternalPerimeter)
            return;
        if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
            if (loop.empty())
                loop.emplace_back(self.x(), self.y());
            loop.emplace_back(line.new_X(self), line.new_Y(self));
            after_extrusion = true;
            return;
        }
        // The loop move is the first non-extruding XY move after the external
        // wall and before the reserved wipe marker, regardless of comment text.
        if (!after_extrusion || line.dist_XY(self) <= EPSILON)
            return;
        after_extrusion = false;

        ++moves;
        INFO("layer Z: " << self.z());
        REQUIRE(loop.size() >= 4);
        const Vec2d seam = loop.front();
        REQUIRE_THAT((loop.back() - seam).norm(), Catch::Matchers::WithinAbs(0., 0.003));
        const Vec2d outgoing = (loop[1] - seam).normalized();
        const Vec2d into_corner = (loop[loop.size() - 2] - seam).normalized();
        REQUIRE_THAT(outgoing.dot(into_corner), Catch::Matchers::WithinAbs(0., 0.01));
        const Vec2d move = Vec2d(line.new_X(self), line.new_Y(self)) - seam;
        // The legacy corner move is 20% of the nozzle diameter, turned 30 degrees
        // from the outgoing edge into the square. Check both components independently.
        const double distance = 0.2 * std::stod(nozzle_diameter);
        CHECK_THAT(move.dot(outgoing), Catch::Matchers::WithinAbs(distance * std::sqrt(3.) / 2., 0.003));
        CHECK_THAT(move.dot(into_corner), Catch::Matchers::WithinAbs(distance / 2., 0.003));
    });
    REQUIRE(moves == 5);
}

TEST_CASE("Inward wipe remains valid after wipe on loops moves the nozzle", "[Wipe][Regression]")
{
    const char *wall_generator = GENERATE("classic", "arachne");
    const char *comments = GENERATE("0", "1");
    CAPTURE(comments);
    INFO("wall generator: " << wall_generator);

    DynamicPrintConfig config = wipe_config(wall_generator, false, "50%", "10%", true);
    config.set_deserialize_strict({{"gcode_comments", comments}});
    const std::string loop_move = slice({make_cube(10., 10., 1.)}, config);
    config.set_deserialize_strict({{"wipe_inward", "1"}});
    const std::string combined = slice({make_cube(10., 10., 1.)}, config);
    config.set_deserialize_strict({{"wipe_on_loops", "0"}});
    const std::string inward_only = slice({make_cube(10., 10., 1.)}, config);

    for (const std::string *output : {&loop_move, &combined}) {
        INFO("wipe_inward: " << (output == &combined));
        std::map<double, std::vector<Vec2d>> loop_moves_by_layer;
        const auto &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
        const auto &wipe_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
        ExtrusionRole role = erNone;
        bool after_extrusion = false;
        GCodeReader parser;
        parser.apply_config(config);
        parser.parse_buffer(*output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
            if (line.comment().find(role_tag) == 0) {
                role = ExtrusionEntity::string_to_role(line.comment().substr(role_tag.size()));
                after_extrusion = false;
            }
            if (line.comment().find(wipe_tag) == 0)
                after_extrusion = false;
            if (role != erExternalPerimeter || line.dist_XY(self) <= EPSILON)
                return;
            if (line.extruding(self)) {
                after_extrusion = true;
            } else if (after_extrusion) {
                loop_moves_by_layer[line.new_Z(self)].emplace_back(line.new_X(self), line.new_Y(self));
                after_extrusion = false;
            }
        });

        // The 1 mm cube at 0.2 mm layer height has one external loop on each of five layers.
        const auto trajectories = wipe_trajectories(*output);
        REQUIRE(loop_moves_by_layer.size() == 5);
        for (size_t layer = 1; layer <= 5; ++layer) {
            const double z = layer * 0.2;
            const auto moves = std::find_if(loop_moves_by_layer.begin(), loop_moves_by_layer.end(),
                [z](const auto &entry) { return std::abs(entry.first - z) < 0.001; });
            REQUIRE(moves != loop_moves_by_layer.end());
            REQUIRE(moves->second.size() == 1);
            const auto wipe = std::find_if(trajectories.begin(), trajectories.end(), [&](const WipeTrajectory &trajectory) {
                return std::abs(trajectory.z - z) < 0.001 &&
                       (trajectory.start - moves->second.front()).norm() < 0.001;
            });
            REQUIRE(wipe != trajectories.end());
            // The configured 2 mm wipe must be measured from the inward move's
            // endpoint, including when wipe_inward is off (set_last_pos regression).
            CHECK_THAT(trajectory_length(*wipe), Catch::Matchers::WithinAbs(2., 0.003));
        }
    }

    const std::vector<WipeTrajectory> combined_trajectories = wipe_trajectories(combined);
    const std::vector<WipeTrajectory> inward_trajectories = wipe_trajectories(inward_only);
    REQUIRE_FALSE(combined_trajectories.empty());
    REQUIRE(combined_trajectories.size() == inward_trajectories.size());
    REQUIRE(trajectories_differ(wipe_destinations(combined), wipe_destinations(loop_move)));

    bool start_changed = false;
    for (size_t i = 0; i < combined_trajectories.size(); ++i) {
        start_changed = start_changed ||
            (combined_trajectories[i].start - inward_trajectories[i].start).norm() > 0.01;
        REQUIRE_THAT(trajectory_length(combined_trajectories[i]),
                     Catch::Matchers::WithinAbs(trajectory_length(inward_trajectories[i]), 0.01));
    }
    REQUIRE(start_changed);
}

namespace retraction_variance_contracts {

DynamicPrintConfig retraction_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", "0.2"},
        {"initial_layer_print_height", "0.2"},
        {"nozzle_diameter", "0.4"},
        {"gcode_flavor", "marlin"},
        {"use_firmware_retraction", "0"},
        {"use_relative_e_distances", "1"},
        {"wall_generator", "classic"},
        {"wall_loops", "2"},
        {"top_shell_layers", "1"},
        {"bottom_shell_layers", "1"},
        {"sparse_infill_density", "20%"},
        {"retraction_length", "0.8"},
        {"retraction_speed", "30"},
        {"deretraction_speed", "0"},
        {"retraction_minimum_travel", "0"},
        {"retract_when_changing_layer", "0"},
        {"wipe", "0"},
        {"wipe_distance", "2"},
        {"wipe_speed", "100"},
        {"role_based_wipe_speed", "0"},
        {"initial_layer_speed", "30"},
        {"outer_wall_speed", "60"},
        {"inner_wall_speed", "40"},
        {"sparse_infill_speed", "35"},
        {"top_surface_speed", "25"},
        {"filament_max_volumetric_speed", "1000"},
        {"slow_down_for_layer_cooling", "0"},
        {"retract_before_wipe", "0%"},
        {"retract_after_wipe", "0%"},
        {"z_hop", "0.4"},
        {"retract_lift_above", "0"},
        {"retract_lift_below", "0"},
        {"gcode_comments", "1"},
        {"layer_change_gcode", "G92 E0\n"},
        {"machine_start_gcode", ""},
        {"machine_end_gcode", ""},
    });
    config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric{zhtNormal});
    config.set_key_value("retract_lift_enforce", new ConfigOptionEnumsGeneric{rletAllSurfaces});
    return config;
}

TriangleMesh short_cube(double height = 1.0)
{
    TriangleMesh result = make_cube(10., 10., height);
    return result;
}

TriangleMesh separated_cubes()
{
    TriangleMesh first = make_cube(10., 10., 1.0);
    TriangleMesh second = make_cube(10., 10., 1.0);
    second.translate(25.f, 0.f, 0.f);
    first.merge(second);
    return first;
}

std::string export_mesh(TriangleMesh mesh, const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({std::move(mesh)}, print, model, config);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    return gcode(print);
}

struct RetractionStats {
    std::vector<double> retract_lengths;
    std::vector<double> retract_feedrates;
    std::vector<double> deretract_feedrates;
    std::set<double> lift_zs;
    size_t unretractions = 0;
};

RetractionStats retraction_stats(const std::string &output, const DynamicPrintConfig &config)
{
    RetractionStats stats;
    GCodeReader reader;
    reader.apply_config(config);
    bool has_printed = false;
    reader.parse_buffer(output, [&stats, &has_printed](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.extruding(self) && line.dist_XY(self) > EPSILON)
            has_printed = true;
        const std::string_view comment = line.comment();
        // Initial nozzle positioning precedes any printed surface and does
        // not test the policy for retracting between extrusion paths.
        if (has_printed && comment.find("lift Z") != std::string_view::npos)
            stats.lift_zs.insert(line.new_Z(self));
        if (line.retracting(self)) {
            stats.retract_lengths.push_back(-line.dist_E(self));
            if (line.has_f())
                stats.retract_feedrates.push_back(line.f());
            return;
        }
        if ((line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) &&
            line.has_e() && line.dist_E(self) > EPSILON && line.dist_XY(self) <= EPSILON) {
            ++stats.unretractions;
            if (line.has_f())
                stats.deretract_feedrates.push_back(line.f());
        }
    });
    return stats;
}

struct WipeStats {
    double before = 0.;
    double during = 0.;
    double after = 0.;
    double path = 0.;
    double feedrate = 0.;
    bool saw_wipe = false;
};

WipeStats first_wipe_stats(const std::string &output, const DynamicPrintConfig &config)
{
    WipeStats stats;
    const std::string &start = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const std::string &end = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    double pending_before = 0.;
    bool during = false;
    bool after = false;
    bool finished = false;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&stats, &pending_before, &during, &after, &finished, &start, &end](GCodeReader &self,
                                                                                                      const GCodeReader::GCodeLine &line) {
        if (finished)
            return;
        if (after) {
            if (line.comment().find(start) != std::string_view::npos) {
                finished = true;
                return;
            }
            if (line.retracting(self)) {
                stats.after += -line.dist_E(self);
                return;
            }
            if ((line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) &&
                (line.has_x() || line.has_y() || line.has_z() || line.has_e())) {
                // A non-retracting move means the first wipe's immediate
                // post-wipe window has ended; later retracts belong to later
                // wipe cycles and must not be attributed to this one.
                finished = true;
                return;
            }
        }
        if (!stats.saw_wipe && line.comment().find(start) != std::string_view::npos) {
            stats.saw_wipe = true;
            // The first wipe marker follows any explicitly emitted
            // retract-before-wipe move. Retain only a contiguous pending
            // retract so previous travel/retraction cycles cannot leak into
            // this first-wipe measurement.
            stats.before = pending_before;
            during = true;
            return;
        }
        if (during && line.comment().find(end) != std::string_view::npos) {
            during = false;
            after = true;
            return;
        }
        if (!stats.saw_wipe) {
            if (line.retracting(self)) {
                pending_before += -line.dist_E(self);
                return;
            }
            if (line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3"))
                pending_before = 0.;
            return;
        }
        if (during && line.has_f())
            stats.feedrate = line.f();
        if (during && line.dist_XY(self) > EPSILON) {
            stats.path += line.dist_XY(self);
        }
        if (line.retracting(self)) {
            const double amount = -line.dist_E(self);
            if (during)
                stats.during += amount;
        }
    });
    return stats;
}

struct WipeRoleRelation {
    double wipe_feedrate = 0.;
    double preceding_print_feedrate = 0.;
};

std::vector<WipeRoleRelation> wipe_role_relations(const std::string &output, const DynamicPrintConfig &config)
{
    std::vector<WipeRoleRelation> result;
    const std::string &start = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Start);
    const std::string &end = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_End);
    bool in_wipe = false;
    bool got_wipe_feedrate = false;
    double current_feedrate = 0.;
    double preceding_print_feedrate = 0.;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&result, &in_wipe, &got_wipe_feedrate, &current_feedrate,
                                 &preceding_print_feedrate, &start, &end](GCodeReader &self,
                                                                           const GCodeReader::GCodeLine &line) {
        if (line.comment().find(start) != std::string_view::npos) {
            in_wipe = true;
            got_wipe_feedrate = false;
            return;
        }
        if (line.comment().find(end) != std::string_view::npos) {
            in_wipe = false;
            return;
        }
        if (line.has_f())
            current_feedrate = line.f();
        if (!in_wipe && line.has_e() && line.dist_E(self) > EPSILON && line.dist_XY(self) > EPSILON)
            preceding_print_feedrate = current_feedrate;
        if (in_wipe && !got_wipe_feedrate && line.has_f()) {
            result.push_back({line.f(), preceding_print_feedrate});
            got_wipe_feedrate = true;
        }
    });
    return result;
}

std::vector<double> printing_feedrates(const std::string &output, const DynamicPrintConfig &config)
{
    std::vector<double> result;
    double current_feedrate = 0.;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&result, &current_feedrate](GCodeReader &self,
                                                               const GCodeReader::GCodeLine &line) {
        if (line.has_f())
            current_feedrate = line.f();
        if (line.extruding(self) && current_feedrate > 0.)
            result.push_back(current_feedrate);
    });
    return result;
}

struct LayerRetractionStats {
    size_t markers = 0;
    size_t retractions = 0;
};

LayerRetractionStats layer_retraction_stats(const std::string &output, const DynamicPrintConfig &config)
{
    LayerRetractionStats stats;
    bool in_new_layer = false;
    bool saw_extrusion = false;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&stats, &in_new_layer, &saw_extrusion](GCodeReader &self,
                                                                         const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos) {
            ++stats.markers;
            in_new_layer = true;
            saw_extrusion = false;
            return;
        }
        if (!in_new_layer)
            return;
        if (!saw_extrusion && line.retracting(self)) {
            ++stats.retractions;
            return;
        }
        if (line.has_e() && line.dist_E(self) > EPSILON && line.dist_XY(self) > EPSILON)
            saw_extrusion = true;
    });
    return stats;
}

TEST_CASE("Configured retraction lengths control every emitted retract", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const double length : {0.35, 1.25}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict("retraction_length", std::to_string(length));
        const RetractionStats stats = retraction_stats(export_mesh(separated_cubes(), config), config);
        REQUIRE_FALSE(stats.retract_lengths.empty());
        for (double actual : stats.retract_lengths)
            CHECK_THAT(actual, Catch::Matchers::WithinAbs(length, 0.0001));
    }
}

TEST_CASE("Retraction speed is carried by emitted retract moves", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const double speed : {13., 47.}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({{"retraction_speed", std::to_string(speed)},
                                       // Keep deretraction distinct so a writer
                                       // accidentally using the fallback path
                                       // cannot make this assertion vacuous.
                                       {"deretraction_speed", "7"}});
        const RetractionStats stats = retraction_stats(export_mesh(separated_cubes(), config), config);
        REQUIRE_FALSE(stats.retract_feedrates.empty());
        for (double feedrate : stats.retract_feedrates)
            CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(speed * 60., 0.01));
    }
}

TEST_CASE("Deretraction speed uses the configured value and zero falls back", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const std::pair<const char *, double> setting : {std::pair{"0", 30.}, {"13", 13.}, {"51", 51.}}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({{"retraction_speed", "30"}, {"deretraction_speed", setting.first}});
        const RetractionStats stats = retraction_stats(export_mesh(separated_cubes(), config), config);
        REQUIRE_FALSE(stats.deretract_feedrates.empty());
        for (double feedrate : stats.deretract_feedrates)
            CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(setting.second * 60., 0.01));
    }
}

TEST_CASE("Before and after wipe percentages change where retraction is emitted", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    const std::vector<std::tuple<const char *, const char *, double, double, double>> cases{
        {"0%", "0%", 0., 0.8, 0.},
        {"100%", "0%", 0.8, 0., 0.},
        {"0%", "100%", 0., 0., 0.8},
    };
    for (const auto &[before, after, expected_before, expected_during, expected_after] : cases) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({
            {"top_shell_layers", "0"}, {"bottom_shell_layers", "0"}, {"sparse_infill_density", "0%"},
            {"wipe", "1"}, {"wipe_distance", "2"}, {"wipe_speed", "10"},
            {"retraction_speed", "30"}, {"retract_before_wipe", before}, {"retract_after_wipe", after},
        });
        const WipeStats stats = first_wipe_stats(export_mesh(short_cube(), config), config);
        REQUIRE(stats.saw_wipe);
        CHECK_THAT(stats.before, Catch::Matchers::WithinAbs(expected_before, 0.001));
        CHECK_THAT(stats.during, Catch::Matchers::WithinAbs(expected_during, 0.001));
        CHECK_THAT(stats.after, Catch::Matchers::WithinAbs(expected_after, 0.001));
        // The post-wipe standard retract is intentionally accounted for by the
        // after bucket; wipe-time E does not update the writer's retracted state.
        CHECK_THAT(stats.before + stats.during + stats.after, Catch::Matchers::WithinAbs(0.8, 0.001));
    }
}

TEST_CASE("Wipe distance controls the retraction partition and path length", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const double distance : {0.75, 2.0}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({{"wipe", "1"}, {"wipe_distance", std::to_string(distance)},
                                       {"top_shell_layers", "0"}, {"bottom_shell_layers", "0"},
                                       {"sparse_infill_density", "0%"},
                                       {"retraction_speed", "30"}, {"wipe_speed", "100"},
                                       {"retract_before_wipe", "0%"}, {"retract_after_wipe", "0%"}});
        const std::string output = export_mesh(short_cube(), config);
        const WipeStats stats = first_wipe_stats(output, config);
        REQUIRE(stats.saw_wipe);
        CHECK_THAT(stats.path, Catch::Matchers::WithinAbs(distance, 0.01));
        const double expected_during = std::min(0.8, 30. * distance / 100.);
        CHECK_THAT(stats.during, Catch::Matchers::WithinAbs(expected_during, 0.001));
        CHECK_THAT(stats.before, Catch::Matchers::WithinAbs(0.8 - expected_during, 0.001));
        CHECK_THAT(stats.after, Catch::Matchers::WithinAbs(0., 0.001));
        CHECK_THAT(stats.feedrate, Catch::Matchers::WithinAbs(100. * 60., 0.01));
    }
}

TEST_CASE("Role based wipe speed selects role speed instead of wipe speed", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const bool role_based : {false, true}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({{"wipe", "1"}, {"wipe_distance", "1"}, {"wipe_speed", "20"},
                                       {"top_shell_layers", "0"}, {"bottom_shell_layers", "0"},
                                       {"sparse_infill_density", "0%"},
                                       {"role_based_wipe_speed", role_based ? "1" : "0"},
                                       {"outer_wall_speed", "60"}, {"retract_before_wipe", "0%"},
                                       {"retract_after_wipe", "0%"}});
        const std::string output = export_mesh(short_cube(), config);
        const std::vector<WipeRoleRelation> relations = wipe_role_relations(output, config);
        const std::vector<double> printing = printing_feedrates(output, config);
        REQUIRE_FALSE(relations.empty());
        REQUIRE_FALSE(printing.empty());
        std::set<int> distinct_printing;
        for (const double feedrate : printing)
            distinct_printing.insert(static_cast<int>(std::lround(feedrate)));
        REQUIRE(distinct_printing.size() >= 2);
        std::set<int> preceding_speeds;
        for (const WipeRoleRelation &relation : relations)
            preceding_speeds.insert(static_cast<int>(std::lround(relation.preceding_print_feedrate)));
        REQUIRE(preceding_speeds.size() >= 2);
        const double expected = 20. * 60.;
        if (role_based)
            for (const WipeRoleRelation &relation : relations) {
                CAPTURE(relation.wipe_feedrate, relation.preceding_print_feedrate);
                CHECK_THAT(relation.wipe_feedrate, Catch::Matchers::WithinAbs(
                    std::max(10. * 60., relation.preceding_print_feedrate), 0.1));
            }
        else
            for (const WipeRoleRelation &relation : relations) {
                CAPTURE(relation.wipe_feedrate);
                CHECK_THAT(relation.wipe_feedrate, Catch::Matchers::WithinAbs(expected, 0.1));
                CHECK(std::none_of(printing.begin(), printing.end(), [&relation](double feedrate) {
                    return std::abs(relation.wipe_feedrate - feedrate) < 0.1;
                }));
            }
    }
}

TEST_CASE("Layer-change retraction is independently gated", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    for (const bool enabled : {false, true}) {
        DynamicPrintConfig config = retraction_config();
        config.set_deserialize_strict({{"wipe", "0"}, {"retraction_minimum_travel", "100"},
                                       {"retract_when_changing_layer", enabled ? "1" : "0"}});
        const std::string output = export_mesh(short_cube(2.0), config);
        const LayerRetractionStats stats = layer_retraction_stats(output, config);
        CAPTURE(enabled, stats.markers, stats.retractions);
        REQUIRE(stats.markers == 10);
        CHECK(stats.retractions == (enabled ? stats.markers : 0));
    }
}

TEST_CASE("Z-hop lower and upper boundaries gate lift moves", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    DynamicPrintConfig lower_open = retraction_config();
    lower_open.set_deserialize_strict({{"retract_lift_above", "0"}, {"retract_lift_below", "0"}});
    const RetractionStats all_layers = retraction_stats(export_mesh(short_cube(), lower_open), lower_open);
    REQUIRE_FALSE(all_layers.lift_zs.empty());

    DynamicPrintConfig above_blocked = lower_open;
    above_blocked.set_deserialize_strict("retract_lift_above", "2");
    const RetractionStats blocked_above = retraction_stats(export_mesh(short_cube(), above_blocked), above_blocked);
    CHECK(blocked_above.lift_zs.empty());

    DynamicPrintConfig below_blocked = lower_open;
    below_blocked.set_deserialize_strict("retract_lift_below", "0.1");
    const RetractionStats blocked_below = retraction_stats(export_mesh(short_cube(), below_blocked), below_blocked);
    CHECK(blocked_below.lift_zs.empty());
}

TEST_CASE("Retract lift enforcement separates top and bottom surface policies", "[Wipe][SettingsRetractionVariance][Pipeline]")
{
    std::map<RetractLiftEnforceType, std::set<double>> lifts;
    for (const RetractLiftEnforceType policy : {rletAllSurfaces, rletTopOnly, rletBottomOnly, rletTopAndBottom}) {
        DynamicPrintConfig config = retraction_config();
        config.set_key_value("retract_lift_enforce", new ConfigOptionEnumsGeneric{policy});
        lifts.emplace(policy, retraction_stats(export_mesh(short_cube(1.0), config), config).lift_zs);
    }
    REQUIRE_FALSE(lifts[rletAllSurfaces].empty());
    REQUIRE_FALSE(lifts[rletTopOnly].empty());
    REQUIRE_FALSE(lifts[rletBottomOnly].empty());
    for (const auto &[policy, values] : lifts) {
        INFO(static_cast<int>(policy));
        for (const double z : values) {
            CHECK(z >= 0.6 - 0.001);
            CHECK(z <= 1.4 + 0.001);
        }
    }
    for (double z : lifts[rletTopOnly])
        CHECK_THAT(z, Catch::Matchers::WithinAbs(1.4, 0.001));
    for (double z : lifts[rletBottomOnly])
        CHECK_THAT(z, Catch::Matchers::WithinAbs(0.6, 0.001));
    REQUIRE(lifts[rletAllSurfaces].size() > lifts[rletTopOnly].size());
    REQUIRE(lifts[rletAllSurfaces].size() > lifts[rletBottomOnly].size());
    const auto includes = [](const std::set<double> &outer, const std::set<double> &inner) {
        return std::includes(outer.begin(), outer.end(), inner.begin(), inner.end());
    };
    CHECK(includes(lifts[rletAllSurfaces], lifts[rletTopOnly]));
    CHECK(includes(lifts[rletAllSurfaces], lifts[rletBottomOnly]));
    std::set<double> top_and_bottom_union = lifts[rletTopOnly];
    top_and_bottom_union.insert(lifts[rletBottomOnly].begin(), lifts[rletBottomOnly].end());
    CHECK(lifts[rletTopAndBottom] == top_and_bottom_union);
    CHECK(lifts[rletTopOnly] != lifts[rletBottomOnly]);
    CHECK(lifts[rletAllSurfaces].size() >= lifts[rletTopOnly].size());
    CHECK(lifts[rletAllSurfaces].size() >= lifts[rletBottomOnly].size());
    CHECK(lifts[rletTopAndBottom].size() >= lifts[rletTopOnly].size());
    CHECK(lifts[rletTopAndBottom].size() >= lifts[rletBottomOnly].size());
}

} // namespace retraction_variance_contracts
