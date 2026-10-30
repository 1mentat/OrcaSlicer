#include <initializer_list>
#include "libslic3r/Surface.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/ExPolygon.hpp"
#include <catch2/catch_all.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"
#include <cstddef>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/calib.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include <map>
#include <set>
#include <sstream>
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
            stats.retract_feedrates.push_back(line.new_F(self));
            return;
        }
        if ((line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) &&
            line.has_e() && line.dist_E(self) > EPSILON && line.dist_XY(self) <= EPSILON) {
            ++stats.unretractions;
            stats.deretract_feedrates.push_back(line.new_F(self));
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


void require_bounded_retraction_output(const std::string &output)
{
    REQUIRE(output.size() <= 4 * 1024 * 1024);
    REQUIRE(std::count(output.begin(), output.end(), '\n') <= 100000);
}

DynamicPrintConfig nullable_retraction_config()
{
    DynamicPrintConfig config = retraction_config();
    config.set_deserialize_strict({
        {"skirt_loops", "0"},
        {"brim_type", "no_brim"},
        {"brim_width", "0"},
        {"enable_arc_fitting", "0"},
        {"seam_position", "aligned"},
        {"retract_restart_extra", "0"},
    });

    auto *filament_map = config.option<ConfigOptionInts>("filament_map", true);
    REQUIRE(filament_map != nullptr);
    filament_map->values = {1};

    // Explicitly nil every generated filament override so a default value
    // cannot make an inheritance case pass accidentally.
    for (const std::string &key : filament_extruder_override_keys) {
        ConfigOption *option = config.option(key, true);
        REQUIRE(option != nullptr);
        REQUIRE(option->nullable());
        config.set_deserialize_strict(key, "nil");
        REQUIRE(option->is_nil());
    }
    return config;
}

DynamicPrintConfig filament_retraction_config(const char *filament_key, const char *filament_value,
                                              const char *machine_key = nullptr, const char *machine_value = nullptr)
{
    DynamicPrintConfig config = nullable_retraction_config();
    if (machine_key != nullptr)
        config.set_deserialize_strict(machine_key, machine_value);
    config.set_deserialize_strict(filament_key, filament_value);
    const ConfigOption *override_option = config.option(filament_key, true);
    REQUIRE(override_option != nullptr);
    if (std::string(filament_value) == "nil")
        REQUIRE(override_option->is_nil());
    else
        REQUIRE_FALSE(override_option->is_nil());
    return config;
}

std::string bounded_retraction_export(TriangleMesh mesh, const DynamicPrintConfig &config)
{
    std::string output = export_mesh(std::move(mesh), config);
    require_bounded_retraction_output(output);
    return output;
}

void require_layer_marker_count(const std::string &output, const DynamicPrintConfig &config, size_t expected)
{
    const LayerRetractionStats layers = layer_retraction_stats(output, config);
    REQUIRE(layers.markers == expected);
}

struct DepositionStats {
    size_t positive_xy_moves = 0;
    double xy_path = 0.;
    double filament = 0.;
    double xy_error_bound = 0.;
    double filament_error_bound = 0.;
    bool finite = true;
};

DepositionStats deposition_stats(const std::string &output, const DynamicPrintConfig &config)
{
    DepositionStats stats;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(output, [&stats](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const double old_x = self.x();
        const double old_y = self.y();
        const double old_z = self.z();
        const double old_e = self.e();
        const double new_x = line.new_X(self);
        const double new_y = line.new_Y(self);
        const double new_z = line.new_Z(self);
        const double new_e = line.new_E(self);
        // Reject malformed coordinates before the extrusion predicate can
        // hide a NaN or infinity in a non-extruding line.
        if (!std::isfinite(old_x) || !std::isfinite(old_y) || !std::isfinite(old_z) ||
            !std::isfinite(old_e) || !std::isfinite(new_x) || !std::isfinite(new_y) ||
            !std::isfinite(new_z) || !std::isfinite(new_e)) {
            stats.finite = false;
            return;
        }
        const double xy = line.dist_XY(self);
        const double e_delta = line.dist_E(self);
        if (!std::isfinite(xy) || !std::isfinite(e_delta)) {
            stats.finite = false;
            return;
        }
        if (!line.extruding(self) || xy <= EPSILON)
            return;
        ++stats.positive_xy_moves;
        stats.xy_path += xy;
        stats.filament += std::abs(e_delta);

        const double max_coordinate = std::max(std::max(std::abs(old_x), std::abs(old_y)),
            std::max(std::abs(new_x), std::abs(new_y)));
        const double float_epsilon = std::numeric_limits<float>::epsilon();
        // XYZF_EXPORT_DIGITS is 3: each endpoint contributes 0.5e-3, with
        // an additional conservative float-read term in each axis.
        stats.xy_error_bound += 2. * std::sqrt(2.) *
            (0.5e-3 + float_epsilon * max_coordinate);
        // E_EXPORT_DIGITS is 5: a delta has two endpoint quantization errors.
        const double max_e = std::max(std::abs(old_e), std::abs(static_cast<double>(line.e())));
        stats.filament_error_bound += 1e-5 + 4. * float_epsilon * max_e;
    });
    REQUIRE(stats.finite);
    return stats;
}

void require_same_deposition(const DepositionStats &baseline, const DepositionStats &candidate)
{
    REQUIRE(baseline.positive_xy_moves > 0);
    REQUIRE(candidate.positive_xy_moves > 0);
    CHECK(candidate.positive_xy_moves == baseline.positive_xy_moves);
    CHECK_THAT(candidate.xy_path, Catch::Matchers::WithinAbs(
        baseline.xy_path, baseline.xy_error_bound + candidate.xy_error_bound));
    CHECK_THAT(candidate.filament, Catch::Matchers::WithinAbs(
        baseline.filament, baseline.filament_error_bound + candidate.filament_error_bound));
}

std::set<double> lift_height_deltas(const std::string &output, const DynamicPrintConfig &config)
{
    std::set<double> heights;
    std::vector<double> pending_lift_zs;
    GCodeReader reader;
    reader.apply_config(config);
    const double initial_layer_height = config.opt_float("initial_layer_print_height");
    const double layer_height = config.opt_float("layer_height");
    REQUIRE(std::isfinite(initial_layer_height));
    REQUIRE(std::isfinite(layer_height));
    REQUIRE(initial_layer_height > EPSILON);
    REQUIRE(layer_height > EPSILON);
    bool has_printed = false;
    size_t layer_markers = 0;
    bool awaiting_layer_plane = false;
    reader.parse_buffer(output, [&heights, &pending_lift_zs, &has_printed, &layer_markers,
                                 &awaiting_layer_plane, initial_layer_height, layer_height](GCodeReader &self,
                                                                                             const GCodeReader::GCodeLine &line) {
        if (line.comment().find("LAYER_CHANGE") != std::string_view::npos) {
            ++layer_markers;
            awaiting_layer_plane = true;
        }
        const bool positive_extrusion = line.extruding(self) && line.dist_XY(self) > EPSILON;
        if (positive_extrusion) {
            const double destination_plane = line.new_Z(self);
            REQUIRE(std::isfinite(destination_plane));
            if (awaiting_layer_plane) {
                const double expected_plane = initial_layer_height +
                    static_cast<double>(layer_markers - 1) * layer_height;
                CHECK_THAT(destination_plane, Catch::Matchers::WithinAbs(expected_plane, 0.001));
                awaiting_layer_plane = false;
            }
            if (has_printed && !pending_lift_zs.empty()) {
                // A lift may include a layer advance, and its descent can be
                // folded into another travel. Measure above the next print plane.
                for (const double lifted_z : pending_lift_zs) {
                    CAPTURE(lifted_z, destination_plane, layer_markers);
                    REQUIRE(std::isfinite(lifted_z));
                    const double height = lifted_z - destination_plane;
                    REQUIRE(std::isfinite(height));
                    REQUIRE(height > EPSILON);
                    heights.insert(height);
                }
                pending_lift_zs.clear();
            }
            has_printed = true;
        }

        const double lift_delta = line.dist_Z(self);
        if (has_printed && line.comment().find("lift Z") != std::string_view::npos &&
            std::isfinite(lift_delta) && lift_delta > EPSILON) {
            const double lifted_z = line.new_Z(self);
            REQUIRE(std::isfinite(lifted_z));
            pending_lift_zs.push_back(lifted_z);
        }
    });
    return heights;
}

TEST_CASE("Nullable filament retraction length inherits or overrides the machine length", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const auto &[value, expected] : std::vector<std::pair<const char *, double>>{
             {"nil", 0.8}, {"0.35", 0.35}, {"1.25", 1.25}}) {
        DynamicPrintConfig config = filament_retraction_config("filament_retraction_length", value, "retraction_length", "0.8");
        const std::string output = bounded_retraction_export(separated_cubes(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        REQUIRE_FALSE(stats.retract_lengths.empty());
        for (const double actual : stats.retract_lengths)
            CHECK_THAT(actual, Catch::Matchers::WithinAbs(expected, 0.0001));
    }
}

TEST_CASE("Nullable filament retraction speed inherits or overrides the machine speed", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const auto &[value, expected] : std::vector<std::pair<const char *, double>>{
             {"nil", 30.}, {"13", 13.}, {"47", 47.}}) {
        DynamicPrintConfig config = filament_retraction_config("filament_retraction_speed", value, "retraction_speed", "30");
        config.set_deserialize_strict("deretraction_speed", "7");
        const std::string output = bounded_retraction_export(separated_cubes(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        REQUIRE_FALSE(stats.retract_feedrates.empty());
        for (const double feedrate : stats.retract_feedrates)
            CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(expected * 60., 0.01));
    }
}

TEST_CASE("Nullable filament deretraction speed preserves zero fallback and explicit values", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const auto &[value, expected] : std::vector<std::pair<const char *, double>>{
             {"nil", 7.}, {"0", 30.}, {"13", 13.}, {"51", 51.}}) {
        DynamicPrintConfig config = filament_retraction_config("filament_deretraction_speed", value, "deretraction_speed", "7");
        config.set_deserialize_strict("retraction_speed", "30");
        const std::string output = bounded_retraction_export(separated_cubes(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        REQUIRE_FALSE(stats.deretract_feedrates.empty());
        for (const double feedrate : stats.deretract_feedrates)
            CHECK_THAT(feedrate, Catch::Matchers::WithinAbs(expected * 60., 0.01));
    }
}

TEST_CASE("Retraction speed rounding and deretraction fallback reach emitted feeds",
          "[Wipe][SettingsRetractionVariance][RetractionSpeedRounding][Pipeline]")
{
    struct SpeedCase
    {
        const char *label;
        const char *retraction_speed;
        const char *deretraction_speed;
        const char *filament_retraction_speed;
        const char *filament_deretraction_speed;
        double expected_retract_feed;
        double expected_deretract_feed;
    };
    const std::vector<SpeedCase> cases{
        {"machine value below positive rounding threshold", "1.49", "0", "nil", "nil", 60., 60.},
        {"machine value at positive rounding threshold", "1.5", "0", "nil", "nil", 120., 120.},
        {"machine deretraction below positive rounding threshold", "30", "1.49", "nil", "nil", 1800., 60.},
        {"machine deretraction at positive rounding threshold", "30", "1.5", "nil", "nil", 1800., 120.},
        {"filament value below positive rounding threshold", "30", "7", "1.49", "0", 60., 60.},
        {"filament deretraction below positive rounding threshold", "30", "7", "nil", "1.49", 1800., 60.},
        {"filament deretraction at positive rounding threshold", "30", "7", "nil", "1.5", 1800., 120.},
        {"filament nil inherits both machine speeds", "30", "7", "nil", "nil", 1800., 420.},
        {"filament positive values at rounding threshold remain independent", "30", "7", "1.5", "1.5", 120., 120.},
    };

    DepositionStats baseline_deposition;
    size_t baseline_retract_count = 0;
    size_t baseline_deretract_count = 0;
    bool have_baseline = false;
    for (const SpeedCase &setting : cases) {
        CAPTURE(setting.label, setting.retraction_speed, setting.deretraction_speed,
                setting.filament_retraction_speed, setting.filament_deretraction_speed);
        DynamicPrintConfig config = filament_retraction_config(
            "filament_retraction_speed", setting.filament_retraction_speed,
            "retraction_speed", setting.retraction_speed);
        config.set_deserialize_strict({
            {"deretraction_speed", setting.deretraction_speed},
            {"filament_deretraction_speed", setting.filament_deretraction_speed},
        });
        const ConfigOption *filament_deretraction = config.option("filament_deretraction_speed", true);
        REQUIRE(filament_deretraction != nullptr);
        if (std::string(setting.filament_deretraction_speed) == "nil")
            REQUIRE(filament_deretraction->is_nil());
        else
            REQUIRE_FALSE(filament_deretraction->is_nil());

        const std::string output = bounded_retraction_export(separated_cubes(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        const RetractionStats stats = retraction_stats(output, config);
        REQUIRE_FALSE(stats.retract_feedrates.empty());
        REQUIRE_FALSE(stats.deretract_feedrates.empty());

        if (!have_baseline) {
            baseline_deposition = deposition;
            baseline_retract_count = stats.retract_feedrates.size();
            baseline_deretract_count = stats.deretract_feedrates.size();
            REQUIRE(baseline_deposition.positive_xy_moves > 0);
            have_baseline = true;
        } else {
            require_same_deposition(baseline_deposition, deposition);
            CHECK(stats.retract_feedrates.size() == baseline_retract_count);
            CHECK(stats.deretract_feedrates.size() == baseline_deretract_count);
        }

        for (const double feed : stats.retract_feedrates) {
            CHECK(feed > 0.);
            CHECK_THAT(feed, Catch::Matchers::WithinAbs(setting.expected_retract_feed, 0.01));
        }
        for (const double feed : stats.deretract_feedrates) {
            CHECK(feed > 0.);
            CHECK_THAT(feed, Catch::Matchers::WithinAbs(setting.expected_deretract_feed, 0.01));
        }
    }
}

TEST_CASE("Nullable filament layer retraction inherits and overrides both boolean states", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    struct LayerCase { bool machine; const char *filament; bool expected; };
    const std::vector<LayerCase> cases{
        {false, "nil", false}, {true, "nil", true}, {true, "0", false}, {false, "1", true}};
    DepositionStats baseline;
    bool have_baseline = false;
    for (const LayerCase &layer_case : cases) {
        DynamicPrintConfig config = filament_retraction_config(
            "filament_retract_when_changing_layer", layer_case.filament,
            "retract_when_changing_layer", layer_case.machine ? "1" : "0");
        config.set_deserialize_strict("retraction_minimum_travel", "100");
        const std::string output = bounded_retraction_export(short_cube(2.0), config);
        require_layer_marker_count(output, config, 10);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const LayerRetractionStats stats = layer_retraction_stats(output, config);
        REQUIRE(stats.markers == 10);
        CHECK(stats.retractions == (layer_case.expected ? stats.markers : 0));
    }
}

TEST_CASE("Nullable filament minimum travel inherits and suppresses long travel retractions", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    size_t baseline_retractions = 0;
    DepositionStats baseline;
    bool have_baseline = false;
    for (const char *value : {"nil", "0", "1000"}) {
        DynamicPrintConfig config = filament_retraction_config("filament_retraction_minimum_travel", value, "retraction_minimum_travel", "0");
        const std::string output = bounded_retraction_export(separated_cubes(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        if (std::string(value) == "1000") {
            CHECK(stats.retract_lengths.size() < baseline_retractions);
        } else if (std::string(value) == "nil") {
            REQUIRE_FALSE(stats.retract_lengths.empty());
            baseline_retractions = stats.retract_lengths.size();
        } else {
            CHECK(stats.retract_lengths.size() == baseline_retractions);
        }
    }
}

TEST_CASE("Nullable filament z hop inherits and overrides the normal lift height", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const auto &[value, expected_lift] : std::vector<std::pair<const char *, double>>{
             {"nil", 0.4}, {"0", 0.}, {"0.8", 0.8}}) {
        CAPTURE(value, expected_lift);
        DynamicPrintConfig config = filament_retraction_config("filament_z_hop", value, "z_hop", "0.4");
        config.set_deserialize_strict("z_offset", "0");
        const std::string output = bounded_retraction_export(short_cube(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const std::set<double> lift_heights = lift_height_deltas(output, config);
        if (expected_lift == 0.) {
            CHECK(lift_heights.empty());
            CHECK(retraction_stats(output, config).lift_zs.empty());
        } else {
            REQUIRE_FALSE(lift_heights.empty());
            for (const double height : lift_heights)
                CHECK_THAT(height, Catch::Matchers::WithinAbs(expected_lift, 0.001));
        }
    }
}

TEST_CASE("Nullable filament upper lift bound inherits and overrides the machine bound", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const char *value : {"nil", "0", "2"}) {
        DynamicPrintConfig config = filament_retraction_config("filament_retract_lift_above", value, "retract_lift_above", "0");
        const std::string output = bounded_retraction_export(short_cube(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        if (std::string(value) == "2")
            CHECK(stats.lift_zs.empty());
        else
            REQUIRE_FALSE(stats.lift_zs.empty());
    }
}

TEST_CASE("Nullable filament lower lift bound inherits and overrides the machine bound", "[Wipe][FilamentRetractionOverrides][SettingsRetractionVariance][Pipeline]")
{
    DepositionStats baseline;
    bool have_baseline = false;
    for (const char *value : {"nil", "0", "0.1"}) {
        DynamicPrintConfig config = filament_retraction_config("filament_retract_lift_below", value, "retract_lift_below", "0");
        const std::string output = bounded_retraction_export(short_cube(), config);
        require_layer_marker_count(output, config, 5);
        const DepositionStats deposition = deposition_stats(output, config);
        if (!have_baseline) {
            baseline = deposition;
            have_baseline = true;
        } else {
            require_same_deposition(baseline, deposition);
        }
        const RetractionStats stats = retraction_stats(output, config);
        if (std::string(value) == "0.1")
            CHECK(stats.lift_zs.empty());
        else
            REQUIRE_FALSE(stats.lift_zs.empty());
    }
}

TEST_CASE("Nullable filament overrides update applied retraction settings across inheritance changes",
          "[Wipe][FilamentRetractionOverrides][FilamentRetractionApplyTransition][SettingsRetractionVariance][Pipeline]")
{
    Print print;
    Model model;

    const DynamicPrintConfig explicit_first =
        filament_retraction_config("filament_retraction_length", "0.35", "retraction_length", "0.8");
    init_print({short_cube()}, print, model, explicit_first);

    REQUIRE(print.config().retraction_length.values.size() == 1);
    CHECK_THAT(print.config().retraction_length.values.front(), Catch::Matchers::WithinAbs(0.35, 1e-6));

    const DynamicPrintConfig inherited =
        filament_retraction_config("filament_retraction_length", "nil", "retraction_length", "0.8");
    const PrintBase::ApplyStatus to_inherited = print.apply(model, inherited);
    REQUIRE(to_inherited != PrintBase::APPLY_STATUS_UNCHANGED);
    REQUIRE(print.config().retraction_length.values.size() == 1);
    CHECK_THAT(print.config().retraction_length.values.front(), Catch::Matchers::WithinAbs(0.8, 1e-6));

    const DynamicPrintConfig explicit_second =
        filament_retraction_config("filament_retraction_length", "1.25", "retraction_length", "0.8");
    const PrintBase::ApplyStatus to_explicit = print.apply(model, explicit_second);
    REQUIRE(to_explicit != PrintBase::APPLY_STATUS_UNCHANGED);
    REQUIRE(print.config().retraction_length.values.size() == 1);
    CHECK_THAT(print.config().retraction_length.values.front(), Catch::Matchers::WithinAbs(1.25, 1e-6));
}


} // namespace retraction_variance_contracts

namespace reduce_crossing_wall_contracts {

struct Segment {
    size_t layer_index = 0;
    size_t emission_index = 0;
    double z = 0.;
    Vec2d start;
    Vec2d end;
};

struct TravelRoute {
    size_t layer_index = 0;
    size_t start_emission_index = 0;
    double z = 0.;
    std::vector<Vec2d> points;
    double direct_mm = 0.;
    double path_mm = 0.;
    bool had_retraction = false;
    bool direct_crosses_wall = false;
    bool direct_crosses_hole_wall = false;
    bool legs_clear = false;
};

struct MotionSummary {
    size_t extrusion_moves = 0;
    double xy_path_mm = 0.;
    double filament_mm = 0.;
};

struct ParsedOutput {
    std::vector<Segment> walls;
    std::vector<TravelRoute> routes;
    MotionSummary motion;
    size_t central_barrier_walls = 0;
    size_t non_barrier_walls = 0;
};

DynamicPrintConfig crossing_wall_config(bool enabled)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "nozzle_diameter",             "0.4" },
        { "layer_height",                "0.2" },
        { "initial_layer_print_height",  "0.2" },
        { "line_width",                  "0.45" },
        { "outer_wall_line_width",       "0" },
        { "inner_wall_line_width",       "0" },
        { "wall_generator",              "classic" },
        { "wall_loops",                  "2" },
        { "wall_sequence",               "inner wall/outer wall" },
        { "top_shell_layers",            "0" },
        { "bottom_shell_layers",         "0" },
        { "sparse_infill_density",       "0%" },
        { "skirt_loops",                 "0" },
        { "brim_type",                   "no_brim" },
        { "seam_position",               "aligned" },
        { "seam_gap",                    "0" },
        { "reduce_crossing_wall",        enabled ? "1" : "0" },
        { "max_travel_detour_distance",  "1000" },
        { "retraction_minimum_travel",   "0.5" },
        { "retraction_length",           "0.8" },
        { "retract_when_changing_layer", "0" },
        { "use_firmware_retraction",     "0" },
        { "wipe",                         "0" },
        { "z_hop",                        "0" },
        { "gcode_flavor",                 "marlin2" },
        { "use_relative_e_distances",     "0" },
        { "enable_arc_fitting",           "0" },
        { "gcode_comments",               "1" },
        { "layer_change_gcode",           "" },
        { "machine_start_gcode",          "" },
        { "machine_end_gcode",            "" },
        { "print_sequence",               "by layer" },
        { "print_order",                  "as_obj_list" },
        { "gcode_label_objects",          "1" },
        { "exclude_object",               "1" },
        { "raft_layers",                  "0" },
        { "enable_prime_tower",           "0" },
    });
    config.set_key_value("extruder_offset", new ConfigOptionPoints{Vec2d(0., 0.)});
    return config;
}

std::vector<TriangleMesh> crossing_wall_fixture()
{
    std::vector<TriangleMesh> meshes;
    meshes.reserve(3);

    // Keep explicit model order and coordinates: the stretched hollow barrier
    // is followed by endpoint cubes on either side for the paired witness.
    TriangleMesh barrier = Test::mesh(Test::TestMesh::cube_with_hole);
    barrier.scale(Vec3f(1.f, 2.f, 0.04f));
    barrier.translate(100.f, 60.f, 0.f);
    meshes.emplace_back(std::move(barrier));

    TriangleMesh left = make_cube(10., 10., 0.4);
    left.translate(70., 75., 0.);
    meshes.emplace_back(std::move(left));

    TriangleMesh right = make_cube(10., 10., 0.4);
    right.translate(140., 75., 0.);
    meshes.emplace_back(std::move(right));
    return meshes;
}

// The barrier's model-space outer bounds are x=100..120, y=60..100.
// A one-millimetre margin admits line-center quantization/width offsets while
// remaining far from the endpoint cubes at x=70..80 and x=140..150. This
// selects every emitted outer and hole wall of the known barrier, and no
// endpoint-object wall.
constexpr double kBarrierMinX = 100.;
constexpr double kBarrierMaxX = 120.;
constexpr double kBarrierMinY = 60.;
constexpr double kBarrierMaxY = 100.;
constexpr double kBarrierCoordinateMargin = 1.;

bool central_barrier_wall(const Segment &wall)
{
    const auto inside = [](const Vec2d &point) {
        return point.x() >= kBarrierMinX - kBarrierCoordinateMargin &&
               point.x() <= kBarrierMaxX + kBarrierCoordinateMargin &&
               point.y() >= kBarrierMinY - kBarrierCoordinateMargin &&
               point.y() <= kBarrierMaxY + kBarrierCoordinateMargin;
    };
    return inside(wall.start) && inside(wall.end);
}

double cross_2d(const Vec2d &a, const Vec2d &b)
{
    return a.x() * b.y() - a.y() * b.x();
}

// Endpoint and tangent policy: only strict interior/interior crossings count. A route may
// start or finish on a printed wall, and a tangent contact is not an obstacle crossing.
bool proper_crossing(const Vec2d &a, const Vec2d &b, const Vec2d &c, const Vec2d &d)
{
    constexpr double parameter_margin = 0.02;
    const Vec2d r = b - a;
    const Vec2d s = d - c;
    const double denominator = cross_2d(r, s);
    if (std::abs(denominator) <= 1e-9)
        return false;
    const Vec2d offset = c - a;
    const double t = cross_2d(offset, s) / denominator;
    const double u = cross_2d(offset, r) / denominator;
    return t > parameter_margin && t < 1. - parameter_margin &&
           u > parameter_margin && u < 1. - parameter_margin;
}

// Restrict the paired oracle to a route spanning the two explicit endpoint
// squares. Other same-endpoint routes are retained in diagnostics but are not
// evidence for the barrier-crossing contract.
bool spans_endpoint_squares(const TravelRoute &route)
{
    if (route.points.size() < 2)
        return false;
    const Vec2d &start = route.points.front();
    const Vec2d &end = route.points.back();
    const bool left_to_right = start.x() < 95. && end.x() > 125.;
    const bool right_to_left = end.x() < 95. && start.x() > 125.;
    return left_to_right || right_to_left;
}

// The stretched cube_with_hole has a model-space void x=105..115, y=70..90
// after the fixture transform. AvoidCrossingPerimeters' active external
// boundary is built from hole contours, but the emitted contour list also has
// deeper perimeter walls. This independent analytic predicate therefore tests
// the public void-avoidance contract rather than claiming every emitted wall
// is avoided. Shrink the open rectangle by only one hundredth of a millimetre
// for the writer/parser coordinate precision.
constexpr double kHoleMinX = 105.;
constexpr double kHoleMaxX = 115.;
constexpr double kHoleMinY = 70.;
constexpr double kHoleMaxY = 90.;
constexpr double kHoleInteriorTolerance = 0.01;
// A one-millimetre band around the void admits the emitted line-center offset
// while excluding the barrier's outer contour. Both endpoints of a segment
// must be in this band. The wall lookup also checks prior emission, layer, Z,
// and a strict interior crossing.
constexpr double kHoleWallBandMargin = 1.;

bool hole_adjacent_barrier_wall(const Segment &wall)
{
    const auto inside_expanded_hole = [](const Vec2d &point) {
        return point.x() >= kHoleMinX - kHoleWallBandMargin &&
               point.x() <= kHoleMaxX + kHoleWallBandMargin &&
               point.y() >= kHoleMinY - kHoleWallBandMargin &&
               point.y() <= kHoleMaxY + kHoleWallBandMargin;
    };
    return central_barrier_wall(wall) && inside_expanded_hole(wall.start) &&
           inside_expanded_hole(wall.end);
}

bool segment_enters_hole_void(const Vec2d &start, const Vec2d &end)
{
    const double min_x = kHoleMinX + kHoleInteriorTolerance;
    const double max_x = kHoleMaxX - kHoleInteriorTolerance;
    const double min_y = kHoleMinY + kHoleInteriorTolerance;
    const double max_y = kHoleMaxY - kHoleInteriorTolerance;
    double enter = 0.;
    double leave = 1.;
    const auto clip = [&](double p, double q) {
        if (std::abs(p) <= 1e-12)
            return q >= 0.;
        const double t = q / p;
        if (p < 0.) {
            if (t > leave)
                return false;
            if (t > enter)
                enter = t;
        } else {
            if (t < enter)
                return false;
            if (t < leave)
                leave = t;
        }
        return true;
    };
    const double dx = end.x() - start.x();
    const double dy = end.y() - start.y();
    // Liang-Barsky clipping against the open, tolerance-shrunk rectangle.
    if (!clip(-dx, start.x() - min_x) || !clip(dx, max_x - start.x()) ||
        !clip(-dy, start.y() - min_y) || !clip(dy, max_y - start.y()))
        return false;
    return leave > enter + 1e-9 && leave > 1e-9 && enter < 1. - 1e-9;
}

bool route_enters_hole_void(const TravelRoute &route)
{
    for (size_t i = 1; i < route.points.size(); ++i)
        if (segment_enters_hole_void(route.points[i - 1], route.points[i]))
            return true;
    return false;
}

bool wall_crosses_route(const TravelRoute &route, const std::vector<Segment> &walls,
                        bool direct_segment, bool hole_wall_only = false)
{
    if (route.points.size() < 2)
        return false;
    for (const Segment &wall : walls) {
        // Leaving an endpoint object can require crossing its own outer wall.
        // This comparison checks clearance from the previously printed barrier.
        if (!central_barrier_wall(wall))
            continue;
        if (hole_wall_only && !hole_adjacent_barrier_wall(wall))
            continue;
        if (wall.layer_index != route.layer_index ||
            wall.emission_index >= route.start_emission_index ||
            std::abs(wall.z - route.z) > 0.001)
            continue;
        if (direct_segment) {
            if (proper_crossing(route.points.front(), route.points.back(), wall.start, wall.end))
                return true;
        } else {
            for (size_t i = 1; i < route.points.size(); ++i)
                if (proper_crossing(route.points[i - 1], route.points[i], wall.start, wall.end))
                    return true;
        }
    }
    return false;
}

void update_role(const GCodeReader::GCodeLine &line, ExtrusionRole &role)
{
    const std::string_view comment = line.comment();
    const std::string &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    if (comment.rfind(role_tag, 0) == 0)
        role = ExtrusionEntity::string_to_role(comment.substr(role_tag.size()));
    else if (comment.rfind("TYPE:", 0) == 0)
        role = ExtrusionEntity::string_to_role(comment.substr(5));
}

ParsedOutput parse_output(const std::string &gcode, const DynamicPrintConfig &config)
{
    constexpr size_t max_gcode_bytes = 8u * 1024u * 1024u;
    REQUIRE(gcode.size() <= max_gcode_bytes);

    ParsedOutput output;
    ExtrusionRole active_role = erNone;
    bool route_open = false;
    bool pending_retraction = false;
    size_t layer_index = 0;
    size_t emission_index = 0;
    TravelRoute current_route;

    const auto finish_route = [&]() {
        if (!route_open)
            return;
        if (current_route.points.size() >= 2) {
            current_route.direct_mm = (current_route.points.back() - current_route.points.front()).norm();
            current_route.path_mm = 0.;
            for (size_t i = 1; i < current_route.points.size(); ++i)
                current_route.path_mm += (current_route.points[i] - current_route.points[i - 1]).norm();
            REQUIRE(std::isfinite(current_route.direct_mm));
            REQUIRE(std::isfinite(current_route.path_mm));
            if (current_route.direct_mm > 0.2)
                output.routes.push_back(std::move(current_route));
        }
        route_open = false;
    };

    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(gcode, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        update_role(line, active_role);

        // Layer markers terminate a route before advancing its explicit parsed-layer identity.
        // Other metadata/comments do not move the nozzle and must not split one travel route.
        std::string_view comment = line.comment();
        while (!comment.empty() && (comment.front() == ' ' || comment.front() == '\t'))
            comment.remove_prefix(1);
        const bool layer_marker = comment == GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change) ||
                                  comment == "LAYER_CHANGE" || comment == "CHANGE_LAYER";
        if (layer_marker) {
            finish_route();
            ++layer_index;
            return;
        }

        const bool motion = line.cmd_is("G0") || line.cmd_is("G1");
        if (!motion)
            return;
        const size_t current_emission_index = emission_index++;

        const double xy = line.dist_XY(self);
        const double dz = line.dist_Z(self);
        const double de = line.dist_E(self);
        const double new_x = line.new_X(self);
        const double new_y = line.new_Y(self);
        const double new_z = line.new_Z(self);
        REQUIRE(std::isfinite(self.x()));
        REQUIRE(std::isfinite(self.y()));
        REQUIRE(std::isfinite(self.z()));
        REQUIRE(std::isfinite(new_x));
        REQUIRE(std::isfinite(new_y));
        REQUIRE(std::isfinite(new_z));
        REQUIRE(std::isfinite(xy));
        REQUIRE(std::isfinite(dz));
        REQUIRE(std::isfinite(de));
        const bool pure_xy_travel = xy > EPSILON && std::abs(dz) <= EPSILON &&
                                    std::abs(de) <= EPSILON && !line.extruding(self);
        if (line.retracting(self)) {
            finish_route();
            pending_retraction = true;
            return;
        }
        if (pure_xy_travel) {
            const Vec2d destination{new_x, new_y};
            if (!route_open) {
                current_route = TravelRoute{};
                current_route.layer_index = layer_index;
                current_route.start_emission_index = current_emission_index;
                current_route.z = self.z();
                current_route.had_retraction = pending_retraction;
                current_route.points = { Vec2d{self.x(), self.y()}, destination };
                pending_retraction = false;
                route_open = true;
            } else {
                current_route.points.push_back(destination);
            }
            return;
        }

        finish_route();
        if (line.extruding(self) && xy > EPSILON && is_perimeter(active_role)) {
            const double z = line.has_z() ? new_z : self.z();
            REQUIRE(std::isfinite(z));
            output.walls.push_back({layer_index, current_emission_index, z, Vec2d{self.x(), self.y()}, Vec2d{new_x, new_y}});
        }
        if (line.extruding(self) && xy > EPSILON) {
            pending_retraction = false;
            ++output.motion.extrusion_moves;
            output.motion.xy_path_mm += xy;
            output.motion.filament_mm += std::abs(de);
        }
    });
    finish_route();
    REQUIRE(!output.walls.empty());
    for (const Segment &wall : output.walls) {
        if (central_barrier_wall(wall))
            ++output.central_barrier_walls;
        else
            ++output.non_barrier_walls;
    }
    REQUIRE(output.central_barrier_walls > 0);
    for (TravelRoute &route : output.routes) {
        route.direct_crosses_wall = wall_crosses_route(route, output.walls, true);
        route.direct_crosses_hole_wall = wall_crosses_route(route, output.walls, true, true);
        route.legs_clear = !wall_crosses_route(route, output.walls, false);
    }
    return output;
}

ParsedOutput export_fixture(const DynamicPrintConfig &config)
{
    Model model;
    Print print;
    init_print(crossing_wall_fixture(), print, model, config, nullptr, /*arrange=*/false);
    REQUIRE(print.validate().string.empty());
    const std::string output = Test::gcode(print);
    REQUIRE(!output.empty());
    return parse_output(output, config);
}

bool same_endpoints(const TravelRoute &lhs, const TravelRoute &rhs)
{
    return lhs.layer_index == rhs.layer_index &&
           std::abs(lhs.z - rhs.z) < 0.001 &&
           (lhs.points.front() - rhs.points.front()).norm() < 0.05 &&
           (lhs.points.back() - rhs.points.back()).norm() < 0.05;
}

std::string route_diagnostics(const TravelRoute &route, const std::vector<Segment> &walls)
{
    std::ostringstream text;
    text << "layer=" << route.layer_index << " z=" << route.z
         << " emission=" << route.start_emission_index
         << " retracted=" << route.had_retraction
         << " direct_mm=" << route.direct_mm << " path_mm=" << route.path_mm
         << " direct_crosses=" << route.direct_crosses_wall
         << " direct_hole_wall=" << route.direct_crosses_hole_wall
         << " legs_clear=" << route.legs_clear
         << " hole_void_crossing=" << route_enters_hole_void(route)
         << " points=" << route.points.size();
    for (size_t i = 0; i < std::min<size_t>(route.points.size(), 32); ++i)
        text << " [" << i << ":" << route.points[i].x() << "," << route.points[i].y() << "]";

    size_t intersections = 0;
    for (const Segment &wall : walls) {
        if (!central_barrier_wall(wall) || wall.layer_index != route.layer_index ||
            wall.emission_index >= route.start_emission_index || std::abs(wall.z - route.z) > 0.001)
            continue;
        for (size_t i = 1; i < route.points.size(); ++i) {
            if (!proper_crossing(route.points[i - 1], route.points[i], wall.start, wall.end))
                continue;
            if (intersections++ < 16)
                text << "\nleg " << i - 1 << "->" << i << " crosses wall emission=" << wall.emission_index
                     << " [" << wall.start.x() << "," << wall.start.y() << "]->["
                     << wall.end.x() << "," << wall.end.y() << "]";
        }
    }
    text << "\nbarrier_intersections=" << intersections;
    return text.str();
}

} // namespace reduce_crossing_wall_contracts

TEST_CASE("Inter-object travel detours around a printed hole", "[Wipe][ReduceCrossingWall][SettingsRetractionVariance][Pipeline]")
{
    using namespace reduce_crossing_wall_contracts;
    const ParsedOutput disabled = export_fixture(crossing_wall_config(false));
    const ParsedOutput enabled = export_fixture(crossing_wall_config(true));
    REQUIRE(!disabled.routes.empty());
    REQUIRE(!enabled.routes.empty());

    const auto count_routes = [](const std::vector<TravelRoute> &routes, bool retracted, bool crossing) {
        return std::count_if(routes.begin(), routes.end(), [&](const TravelRoute &route) {
            return (!retracted || route.had_retraction) && (!crossing || route.direct_crosses_wall);
        });
    };
    const size_t disabled_retracted = count_routes(disabled.routes, true, false);
    const size_t disabled_crossing = count_routes(disabled.routes, false, true);
    const size_t disabled_retracted_crossing = count_routes(disabled.routes, true, true);
    const size_t enabled_detoured = std::count_if(enabled.routes.begin(), enabled.routes.end(), [](const TravelRoute &route) {
        return route.path_mm > route.direct_mm + 0.05;
    });
    const size_t enabled_clear_detoured = std::count_if(enabled.routes.begin(), enabled.routes.end(), [](const TravelRoute &route) {
        return route.path_mm > route.direct_mm + 0.05 && route.legs_clear;
    });
    size_t same_endpoint_pairs = 0;
    size_t same_endpoint_pairs_with_disabled_crossing = 0;
    size_t same_endpoint_pairs_with_disabled_hole_wall = 0;
    size_t same_endpoint_pairs_with_disabled_hole_void = 0;
    // Retained only as a broad diagnostic: active external planning may still
    // cross outer contours, so this is not the enabled success oracle.
    size_t fully_eligible_pairs = 0;
    size_t hole_eligible_pairs = 0;
    std::ostringstream paired_routes;
    for (const TravelRoute &candidate : disabled.routes) {
        if (!candidate.had_retraction || !spans_endpoint_squares(candidate))
            continue;
        for (const TravelRoute &route : enabled.routes) {
            if (!same_endpoints(route, candidate) || !spans_endpoint_squares(route))
                continue;
            ++same_endpoint_pairs;
            if (same_endpoint_pairs <= 4)
                paired_routes << "\nPair " << same_endpoint_pairs << " disabled: "
                              << route_diagnostics(candidate, disabled.walls) << "\nenabled: "
                              << route_diagnostics(route, enabled.walls) << '\n';
            if (candidate.direct_crosses_wall)
                ++same_endpoint_pairs_with_disabled_crossing;
            if (candidate.direct_crosses_hole_wall)
                ++same_endpoint_pairs_with_disabled_hole_wall;
            if (candidate.direct_crosses_hole_wall && route_enters_hole_void(candidate))
                ++same_endpoint_pairs_with_disabled_hole_void;
            if (candidate.direct_crosses_wall && route.direct_crosses_wall &&
                route.path_mm > route.direct_mm + 0.05 && route.legs_clear)
                ++fully_eligible_pairs;
            if (candidate.direct_crosses_hole_wall && route_enters_hole_void(candidate) &&
                route.had_retraction && route.path_mm > route.direct_mm + 0.05 &&
                !route_enters_hole_void(route))
                ++hole_eligible_pairs;
        }
    }
    CAPTURE(disabled.routes.size(), enabled.routes.size(), disabled_retracted, disabled_crossing,
            disabled_retracted_crossing, enabled_detoured, enabled_clear_detoured, same_endpoint_pairs,
            same_endpoint_pairs_with_disabled_crossing, same_endpoint_pairs_with_disabled_hole_wall,
            same_endpoint_pairs_with_disabled_hole_void,
            fully_eligible_pairs, hole_eligible_pairs,
            disabled.central_barrier_walls, disabled.non_barrier_walls,
            enabled.central_barrier_walls, enabled.non_barrier_walls);
    INFO(paired_routes.str());

    const TravelRoute *direct = nullptr;
    const TravelRoute *detour = nullptr;
    // Search every eligible disabled route: the first crossing is not necessarily the
    // one whose enabled counterpart is detoured by AvoidCrossingPerimeters.
    for (const TravelRoute &candidate : disabled.routes) {
        if (!candidate.had_retraction || !candidate.direct_crosses_wall ||
            !candidate.direct_crosses_hole_wall ||
            !route_enters_hole_void(candidate) || !spans_endpoint_squares(candidate) ||
            candidate.path_mm > candidate.direct_mm + 0.05)
            continue;
        const auto enabled_match = std::find_if(enabled.routes.begin(), enabled.routes.end(),
                                                [&](const TravelRoute &route) {
            return same_endpoints(route, candidate) && spans_endpoint_squares(route) &&
                   route.had_retraction && route.path_mm > route.direct_mm + 0.05 &&
                   !route_enters_hole_void(route);
        });
        if (enabled_match != enabled.routes.end()) {
            direct = &candidate;
            detour = &*enabled_match;
            break;
        }
    }
    REQUIRE(direct != nullptr);
    REQUIRE(detour != nullptr);
    REQUIRE(route_enters_hole_void(*direct));
    CHECK_FALSE(route_enters_hole_void(*detour));
    CHECK(direct->direct_crosses_wall);
    CHECK(direct->direct_crosses_hole_wall);
    CHECK(direct->had_retraction);
    CHECK(detour->had_retraction);
    CHECK(direct->path_mm <= direct->direct_mm + 0.05);

    REQUIRE(disabled.motion.extrusion_moves > 0);
    CHECK(enabled.motion.extrusion_moves == disabled.motion.extrusion_moves);
    CHECK_THAT(enabled.motion.xy_path_mm, Catch::Matchers::WithinRel(disabled.motion.xy_path_mm, 1e-5));
    CHECK_THAT(enabled.motion.filament_mm, Catch::Matchers::WithinRel(disabled.motion.filament_mm, 1e-5));
}

namespace reduce_infill_retraction_contracts {
namespace {

constexpr std::size_t kMaxGCodeBytes = 4u * 1024u * 1024u;
constexpr std::size_t kMaxGCodeLines = 100000u;
constexpr double kCubeSize = 12.0;

struct FixtureExport {
    std::string output;
    // A copy of the public post-process internal surfaces, indexed by layer.
    // The production containment helper consumes the same LayerRegion slices.
    std::vector<std::vector<ExPolygon>> internal_surfaces;
    std::vector<double> layer_print_z;
};

struct TravelWitness {
    int layer = -1;
    ExtrusionRole source_role = erNone;
    ExtrusionRole destination_role = erNone;
    double start_x = 0.0;
    double start_y = 0.0;
    double end_x = 0.0;
    double end_y = 0.0;
    std::size_t retractions = 0;
};

struct ReducedRetractionEvidence {
    std::size_t positive_moves = 0;
    std::size_t sparse_positive_moves = 0;
    std::size_t perimeter_positive_moves = 0;
    std::size_t retractions = 0;
    double positive_filament = 0.0;
    double positive_xy_path = 0.0;
    double sparse_filament = 0.0;
    double sparse_xy_path = 0.0;
    double perimeter_filament = 0.0;
    double perimeter_xy_path = 0.0;
    std::size_t finite_layer_z_checks = 0;
    bool invalid_layer_z = false;
    bool layer_z_mismatch = false;
    std::vector<TravelWitness> contained_internal_travels;
    std::vector<TravelWitness> perimeter_controls;
};

struct PendingTravel {
    bool active = false;
    bool saw_xy = false;
    bool all_xy_inside = true;
    bool crossed_layer = false;
    int source_layer = -1;
    ExtrusionRole source_role = erNone;
    std::size_t retractions = 0;
    double start_x = 0.0;
    double start_y = 0.0;
    double end_x = 0.0;
    double end_y = 0.0;
};

bool perimeter_role(ExtrusionRole role)
{
    return role == erPerimeter || role == erExternalPerimeter || role == erOverhangPerimeter;
}

DynamicPrintConfig reduce_infill_retraction_config(bool enabled)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height",                    "0.2"},
        {"initial_layer_print_height",      "0.2"},
        {"nozzle_diameter",                 "0.4"},
        {"line_width",                      "0.45"},
        {"outer_wall_line_width",           "0.45"},
        {"inner_wall_line_width",           "0.45"},
        {"sparse_infill_line_width",        "0.45"},
        {"gcode_flavor",                    "marlin2"},
        {"use_relative_e_distances",        false},
        {"wall_generator",                  "classic"},
        {"print_sequence",                 "by layer"},
        {"raft_layers",                     "0"},
        {"skirt_loops",                     "0"},
        {"brim_type",                       "no_brim"},
        {"spiral_mode",                     false},
        {"belt_printer",                    false},
        {"enable_arc_fitting",              false},
        {"wall_loops",                      "1"},
        {"top_shell_layers",                "0"},
        {"bottom_shell_layers",             "0"},
        {"top_shell_thickness",             "0"},
        {"bottom_shell_thickness",          "0"},
        {"sparse_infill_density",           "20%"},
        {"sparse_infill_pattern",           "rectilinear"},
        // Separate sparse lines leave real internal travel for the retraction comparison.
        {"infill_anchor",                   "0"},
        {"infill_anchor_max",               "0"},
        {"reduce_infill_retraction",        enabled},
        {"reduce_crossing_wall",            false},
        {"retraction_length",               "0.8"},
        {"retraction_speed",               "30"},
        {"deretraction_speed",              "30"},
        {"retraction_minimum_travel",       "0"},
        {"retract_when_changing_layer",     false},
        {"wipe",                             false},
        {"wipe_distance",                   "0"},
        {"z_hop",                            "0"},
        {"retract_lift_above",              "0"},
        {"retract_lift_below",              "0"},
        {"retract_before_wipe",             "0%"},
        {"retract_after_wipe",              "0%"},
        {"slow_down_for_layer_cooling",     false},
        {"slow_down_layer_time",            "0"},
        {"fan_cooling_layer_time",          "0"},
        {"close_fan_the_first_x_layers",    0},
        {"full_fan_speed_layer",            0},
        {"fan_min_speed",                   "0"},
        {"fan_max_speed",                   "0"},
        {"filament_max_volumetric_speed",   "1000"},
        {"layer_change_gcode",              ""},
        {"gcode_comments",                  true},
        {"machine_start_gcode",             ""},
        {"machine_end_gcode",               ""},
    });
    config.set_key_value("extruder_offset", new ConfigOptionPoints{Vec2d(0., 0.)});
    config.set_key_value("z_hop_types", new ConfigOptionEnumsGeneric{zhtNormal});
    config.set_key_value("retract_lift_enforce", new ConfigOptionEnumsGeneric{rletAllSurfaces});
    return config;
}

FixtureExport export_reduce_fixture(const DynamicPrintConfig &config)
{
    // Model must outlive Print because Print owns PrintObjects that refer back
    // to the model during teardown.
    Model model;
    Print print;
    init_print(std::vector<TriangleMesh>{make_cube(kCubeSize, kCubeSize, 1.0)}, print, model, config, nullptr,
               /*arrange=*/false);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    REQUIRE_NOTHROW(print.process());

    FixtureExport fixture;
    REQUIRE(print.objects().size() == 1);
    const PrintObject *object = print.objects().front();
    REQUIRE(object->instances().size() == 1);
    const Point instance_shift = object->instances().front().shift;
    for (std::size_t layer_index = 0; layer_index < object->layers().size(); ++layer_index) {
        const Layer *layer = object->layers()[layer_index];
        REQUIRE(std::isfinite(double(layer->print_z)));
        fixture.layer_print_z.push_back(layer->print_z);
        fixture.internal_surfaces.emplace_back();
        for (const LayerRegion *region : layer->regions())
            for (const Surface &surface : region->get_slices().surfaces)
                if (surface.is_internal()) {
                    ExPolygon shifted_surface = surface.expolygon;
                    shifted_surface.translate(instance_shift);
                    fixture.internal_surfaces.back().push_back(std::move(shifted_surface));
                }
    }
    REQUIRE(fixture.internal_surfaces.size() > 0);
    const bool has_internal_surface = std::any_of(
        fixture.internal_surfaces.begin(), fixture.internal_surfaces.end(),
        [](const auto &surfaces) { return !surfaces.empty(); });
    REQUIRE(has_internal_surface);
    fixture.output = gcode(print);
    REQUIRE(fixture.output.size() <= kMaxGCodeBytes);
    return fixture;
}

bool segment_inside_internal_surface(const FixtureExport &fixture, int layer,
                                     double start_x, double start_y, double end_x, double end_y)
{
    if (layer < 0 || static_cast<std::size_t>(layer) >= fixture.internal_surfaces.size())
        return false;
    REQUIRE(std::isfinite(start_x));
    REQUIRE(std::isfinite(start_y));
    REQUIRE(std::isfinite(end_x));
    REQUIRE(std::isfinite(end_y));
    const Line segment(Point(scaled(start_x), scaled(start_y)), Point(scaled(end_x), scaled(end_y)));
    for (const ExPolygon &surface : fixture.internal_surfaces[static_cast<std::size_t>(layer)])
        if (surface.contains(segment))
            return true;
    return false;
}

ReducedRetractionEvidence reduced_retraction_evidence(const FixtureExport &fixture,
                                                       const DynamicPrintConfig &config)
{
    const std::string &gcode_output = fixture.output;
    REQUIRE(gcode_output.size() <= kMaxGCodeBytes);
    REQUIRE(static_cast<std::size_t>(std::count(gcode_output.begin(), gcode_output.end(), '\n')) <= kMaxGCodeLines);

    ReducedRetractionEvidence evidence;
    const std::string &role_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Role);
    GCodeReader reader;
    reader.apply_config(config);
    ExtrusionRole active_role = erNone;
    ExtrusionRole last_positive_role = erNone;
    int current_layer = -1;
    int last_positive_layer = -1;
    bool have_last_positive = false;
    PendingTravel pending;

    auto begin_pending = [&]() {
        if (pending.active || !have_last_positive)
            return;
        pending.active = true;
        pending.all_xy_inside = true;
        pending.source_role = last_positive_role;
        pending.source_layer = last_positive_layer;
    };
    auto classify_pending = [&]() {
        if (!pending.active || !pending.saw_xy)
            return;
        if (!pending.crossed_layer && pending.source_layer == current_layer) {
            if (pending.source_role == erInternalInfill && active_role == erInternalInfill && pending.all_xy_inside) {
                evidence.contained_internal_travels.push_back({
                    pending.source_layer, pending.source_role, active_role,
                    pending.start_x, pending.start_y, pending.end_x, pending.end_y, pending.retractions});
            } else if (pending.source_role == erExternalPerimeter && active_role == erInternalInfill) {
                // This is deliberately outside the internal-surface gate: its
                // source endpoint is on the exterior wall. The production
                // external-wall guard is the control being observed.
                evidence.perimeter_controls.push_back({
                    pending.source_layer, pending.source_role, active_role,
                    pending.start_x, pending.start_y, pending.end_x, pending.end_y, pending.retractions});
            }
        }
    };

    reader.parse_buffer(gcode_output, [&](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find(role_tag) == 0)
            active_role = ExtrusionEntity::string_to_role(comment.substr(role_tag.size()));
        if (comment.find("LAYER_CHANGE") != std::string_view::npos) {
            ++current_layer;
            if (pending.active)
                pending.crossed_layer = true;
        }

        if (line.retracting(self)) {
            ++evidence.retractions;
            begin_pending();
            if (pending.active)
                ++pending.retractions;
            return;
        }

        if (line.travel() && line.dist_XY(self) > EPSILON) {
            begin_pending();
            if (pending.active) {
                const double start_x = self.x();
                const double start_y = self.y();
                const double end_x = line.new_X(self);
                const double end_y = line.new_Y(self);
                if (!pending.saw_xy) {
                    pending.start_x = start_x;
                    pending.start_y = start_y;
                }
                pending.end_x = end_x;
                pending.end_y = end_y;
                pending.all_xy_inside = pending.all_xy_inside &&
                    segment_inside_internal_surface(fixture, pending.source_layer, start_x, start_y, end_x, end_y);
                pending.saw_xy = true;
            }
            return;
        }

        if (line.extruding(self) && line.dist_XY(self) > EPSILON) {
            REQUIRE(std::isfinite(line.dist_E(self)));
            REQUIRE(std::isfinite(line.dist_XY(self)));
            if (current_layer < 0 || static_cast<std::size_t>(current_layer) >= fixture.layer_print_z.size()) {
                evidence.invalid_layer_z = true;
            } else {
                const double expected_z = fixture.layer_print_z[static_cast<std::size_t>(current_layer)];
                const double observed_z = line.new_Z(self);
                if (!std::isfinite(expected_z) || !std::isfinite(observed_z))
                    evidence.invalid_layer_z = true;
                else {
                    ++evidence.finite_layer_z_checks;
                    if (std::abs(expected_z - observed_z) > 0.01)
                        evidence.layer_z_mismatch = true;
                }
            }
            classify_pending();
            pending = PendingTravel{};
            ++evidence.positive_moves;
            evidence.positive_filament += line.dist_E(self);
            evidence.positive_xy_path += line.dist_XY(self);
            if (active_role == erInternalInfill) {
                ++evidence.sparse_positive_moves;
                evidence.sparse_filament += line.dist_E(self);
                evidence.sparse_xy_path += line.dist_XY(self);
            } else if (perimeter_role(active_role)) {
                ++evidence.perimeter_positive_moves;
                evidence.perimeter_filament += line.dist_E(self);
                evidence.perimeter_xy_path += line.dist_XY(self);
            }
            last_positive_role = active_role;
            last_positive_layer = current_layer;
            have_last_positive = true;
        }
    });

    REQUIRE(evidence.sparse_positive_moves > 0);
    REQUIRE(evidence.finite_layer_z_checks > 0);
    CHECK_FALSE(evidence.invalid_layer_z);
    CHECK_FALSE(evidence.layer_z_mismatch);
    return evidence;
}

TEST_CASE("Reduce infill retraction skips contained sparse-infill travel", "[Wipe][ReduceInfillRetraction][Pipeline]")
{
    const DynamicPrintConfig disabled_config = reduce_infill_retraction_config(false);
    const DynamicPrintConfig enabled_config = reduce_infill_retraction_config(true);
    const FixtureExport disabled_fixture = export_reduce_fixture(disabled_config);
    const FixtureExport enabled_fixture = export_reduce_fixture(enabled_config);
    const ReducedRetractionEvidence disabled = reduced_retraction_evidence(disabled_fixture, disabled_config);
    const ReducedRetractionEvidence enabled = reduced_retraction_evidence(enabled_fixture, enabled_config);

    REQUIRE(disabled.positive_moves > 0);
    REQUIRE(enabled.positive_moves > 0);
    REQUIRE(disabled.contained_internal_travels.size() == enabled.contained_internal_travels.size());
    REQUIRE(disabled.contained_internal_travels.size() > 0);
    bool saw_changed_internal_event = false;
    for (std::size_t i = 0; i < disabled.contained_internal_travels.size(); ++i) {
        const TravelWitness &before = disabled.contained_internal_travels[i];
        const TravelWitness &after = enabled.contained_internal_travels[i];
        INFO("paired contained travel index " << i);
        CHECK(before.layer == after.layer);
        CHECK(before.source_role == after.source_role);
        CHECK(before.destination_role == after.destination_role);
        CHECK_THAT(before.start_x, Catch::Matchers::WithinAbs(after.start_x, 0.01));
        CHECK_THAT(before.start_y, Catch::Matchers::WithinAbs(after.start_y, 0.01));
        CHECK_THAT(before.end_x, Catch::Matchers::WithinAbs(after.end_x, 0.01));
        CHECK_THAT(before.end_y, Catch::Matchers::WithinAbs(after.end_y, 0.01));
        if (before.retractions > after.retractions)
            saw_changed_internal_event = true;
    }
    CHECK(saw_changed_internal_event);

    // A perimeter-to-internal transition remains a visible control: the
    // external-wall guard must still retract in both fresh exports.
    REQUIRE(disabled.perimeter_controls.size() == enabled.perimeter_controls.size());
    REQUIRE(disabled.perimeter_controls.size() > 0);
    for (std::size_t i = 0; i < disabled.perimeter_controls.size(); ++i) {
        const TravelWitness &before = disabled.perimeter_controls[i];
        const TravelWitness &after = enabled.perimeter_controls[i];
        INFO("paired perimeter control index " << i);
        CHECK(before.layer == after.layer);
        CHECK_THAT(before.start_x, Catch::Matchers::WithinAbs(after.start_x, 0.01));
        CHECK_THAT(before.start_y, Catch::Matchers::WithinAbs(after.start_y, 0.01));
        CHECK_THAT(before.end_x, Catch::Matchers::WithinAbs(after.end_x, 0.01));
        CHECK_THAT(before.end_y, Catch::Matchers::WithinAbs(after.end_y, 0.01));
        CHECK(before.retractions > 0);
        CHECK(after.retractions > 0);
        CHECK(before.retractions == after.retractions);
    }

    // The option changes retraction policy only; positive geometry and
    // deposited material remain invariant between the fresh public exports.
    CHECK(disabled.positive_moves == enabled.positive_moves);
    CHECK(disabled.sparse_positive_moves == enabled.sparse_positive_moves);
    CHECK(disabled.perimeter_positive_moves == enabled.perimeter_positive_moves);
    CHECK_THAT(disabled.positive_xy_path, Catch::Matchers::WithinAbs(enabled.positive_xy_path, 0.01));
    CHECK_THAT(disabled.positive_filament, Catch::Matchers::WithinAbs(enabled.positive_filament, 0.01));
    CHECK_THAT(disabled.sparse_xy_path, Catch::Matchers::WithinAbs(enabled.sparse_xy_path, 0.01));
    CHECK_THAT(disabled.sparse_filament, Catch::Matchers::WithinAbs(enabled.sparse_filament, 0.01));
    CHECK_THAT(disabled.perimeter_xy_path, Catch::Matchers::WithinAbs(enabled.perimeter_xy_path, 0.01));
    CHECK_THAT(disabled.perimeter_filament, Catch::Matchers::WithinAbs(enabled.perimeter_filament, 0.01));
}


namespace wipe_before_external_contracts {
namespace {

constexpr std::size_t kWipeBeforeExternalMaxOutputBytes = 8u * 1024u * 1024u;
constexpr std::size_t kWipeBeforeExternalMaxDepositionSegments = 250000;
// The production search uses the configured nozzle diameter as its line-radius
// bound. This fixture pins the nozzle to 0.4 mm; keep the oracle source-aligned
// rather than treating the upcoming outer wall as already deposited material.
constexpr double kWipeBeforeExternalMaterialPadding = 0.4;
// init_print(..., arrange=false) preserves this cube's [0,10] XY frame. The
// G-code writer prints XYZ with three decimal places, so 0.002 mm is a small
// conservative coordinate/norm allowance for the endpoint and midpoint checks.
constexpr double kWipeBeforeExternalCubeMin = 0.0;
constexpr double kWipeBeforeExternalCubeMax = 10.0;
constexpr double kWipeBeforeExternalGcodePrecision = 0.001;
constexpr double kWipeBeforeExternalCubePadding = 2.0 * kWipeBeforeExternalGcodePrecision;
constexpr double kWipeBeforeExternalMaxTravel =
    kWipeBeforeExternalMaterialPadding + 2.0 * kWipeBeforeExternalGcodePrecision;
constexpr std::string_view kWipeBeforeExternalMarker =
    "move inwards before retraction/seam";

struct DepositionSummary {
    struct Segment {
        Vec2d start;
        Vec2d end;
        double z = 0.;
        bool outer_wall = false;
    };

    std::size_t moves = 0;
    double xy = 0.;
    double e = 0.;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    bool finite = true;
    std::vector<Segment> segments;
};

struct PreExternalWipeMarker {
    Vec2d start;
    Vec2d end;
    double z = 0.;
    double xy = 0.;
    double e = 0.;
    std::size_t prior_segment_count = 0;
    bool finite = true;
};

struct WipeBeforeExternalEvidence {
    std::vector<PreExternalWipeMarker> markers;
    DepositionSummary deposition;
    std::size_t layer_markers = 0;
};

void record_deposition(DepositionSummary &summary, GCodeReader &reader,
                       const GCodeReader::GCodeLine &line, bool outer_wall)
{
    if (!line.extruding(reader) || line.dist_XY(reader) <= EPSILON ||
        line.dist_E(reader) <= EPSILON)
        return;

    const double start_x = reader.x();
    const double start_y = reader.y();
    const double end_x = line.new_X(reader);
    const double end_y = line.new_Y(reader);
    const double z = line.new_Z(reader);
    const double xy = line.dist_XY(reader);
    const double e = line.dist_E(reader);
    if (!std::isfinite(start_x) || !std::isfinite(start_y) ||
        !std::isfinite(end_x) || !std::isfinite(end_y) ||
        !std::isfinite(z) || !std::isfinite(xy) || !std::isfinite(e)) {
        summary.finite = false;
        return;
    }

    ++summary.moves;
    summary.xy += xy;
    summary.e += e;
    summary.min_x = std::min(summary.min_x, std::min(start_x, end_x));
    summary.max_x = std::max(summary.max_x, std::max(start_x, end_x));
    summary.min_y = std::min(summary.min_y, std::min(start_y, end_y));
    summary.max_y = std::max(summary.max_y, std::max(start_y, end_y));
    if (summary.segments.size() >= kWipeBeforeExternalMaxDepositionSegments) {
        summary.finite = false;
        return;
    }
    summary.segments.push_back({Vec2d(start_x, start_y), Vec2d(end_x, end_y), z, outer_wall});
}

WipeBeforeExternalEvidence parse_wipe_before_external(const std::string &gcode,
                                                       const DynamicPrintConfig &config)
{
    REQUIRE(gcode.size() <= kWipeBeforeExternalMaxOutputBytes);

    WipeBeforeExternalEvidence evidence;
    const std::string layer_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Layer_Change);
    GCodeReader reader;
    reader.apply_config(config);
    bool outer_wall = false;
    reader.parse_buffer(gcode, [&evidence, &layer_tag, &outer_wall](GCodeReader &self,
                                                        const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("FEATURE:") != std::string_view::npos || comment.find("TYPE:") != std::string_view::npos)
            outer_wall = comment.find("Outer wall") != std::string_view::npos ||
                         comment.find("External perimeter") != std::string_view::npos;
        std::string_view trimmed_comment = comment;
        while (!trimmed_comment.empty() &&
               (trimmed_comment.back() == '\n' || trimmed_comment.back() == '\r'))
            trimmed_comment.remove_suffix(1);
        if (trimmed_comment == layer_tag)
            ++evidence.layer_markers;

        if (comment.find(kWipeBeforeExternalMarker) != std::string_view::npos &&
            (line.cmd_is("G0") || line.cmd_is("G1"))) {
            PreExternalWipeMarker marker;
            marker.start = Vec2d(self.x(), self.y());
            marker.end = Vec2d(line.new_X(self), line.new_Y(self));
            marker.z = line.new_Z(self);
            marker.xy = line.dist_XY(self);
            marker.e = line.dist_E(self);
            marker.prior_segment_count = evidence.deposition.segments.size();
            marker.finite = std::isfinite(marker.start.x()) && std::isfinite(marker.start.y()) &&
                std::isfinite(marker.end.x()) && std::isfinite(marker.end.y()) &&
                std::isfinite(marker.z) && std::isfinite(marker.xy) && std::isfinite(marker.e);
            evidence.markers.push_back(marker);
        }

        record_deposition(evidence.deposition, self, line, outer_wall);
    });
    return evidence;
}

std::string export_wipe_before_external(const DynamicPrintConfig &config)
{
    Model model;
    Print print;
    init_print(std::vector<TriangleMesh>{make_cube(10., 10., 0.4)}, print, model, config,
               nullptr, false);
    const auto validation = print.validate();
    INFO(validation.string);
    REQUIRE(validation.string.empty());
    const std::string output = gcode(print);
    REQUIRE(output.size() <= kWipeBeforeExternalMaxOutputBytes);
    return output;
}

double point_to_segment(const Vec2d &point, const DepositionSummary::Segment &segment)
{
    const Vec2d delta = segment.end - segment.start;
    const double length_squared = delta.squaredNorm();
    if (length_squared <= 0.)
        return (point - segment.start).norm();
    const double projection = std::clamp((point - segment.start).dot(delta) / length_squared, 0., 1.);
    return (point - (segment.start + projection * delta)).norm();
}

bool point_near_previous_material(const DepositionSummary &summary, const Vec2d &point,
                                  const double z, const std::size_t prior_segment_count)
{
    if (summary.moves == 0 || !summary.finite || summary.segments.empty())
        return false;
    const std::size_t count = std::min(prior_segment_count, summary.segments.size());
    for (std::size_t i = 0; i < count; ++i) {
        const DepositionSummary::Segment &segment = summary.segments[i];
        if (std::abs(segment.z - z) > 1e-4)
            continue;
        if (point_to_segment(point, segment) <= kWipeBeforeExternalMaterialPadding)
            return true;
    }
    return false;
}

bool point_in_fixture_cube(const Vec2d &point)
{
    return std::isfinite(point.x()) && std::isfinite(point.y()) &&
        point.x() >= kWipeBeforeExternalCubeMin - kWipeBeforeExternalCubePadding &&
        point.x() <= kWipeBeforeExternalCubeMax + kWipeBeforeExternalCubePadding &&
        point.y() >= kWipeBeforeExternalCubeMin - kWipeBeforeExternalCubePadding &&
        point.y() <= kWipeBeforeExternalCubeMax + kWipeBeforeExternalCubePadding;
}

void require_same_deposition(const DepositionSummary &lhs, const DepositionSummary &rhs)
{
    REQUIRE(lhs.finite);
    REQUIRE(rhs.finite);
    REQUIRE(lhs.moves == rhs.moves);
    CHECK_THAT(lhs.xy, Catch::Matchers::WithinAbs(rhs.xy, 0.01));
    CHECK_THAT(lhs.e, Catch::Matchers::WithinAbs(rhs.e, 0.0001));
    CHECK_THAT(lhs.min_x, Catch::Matchers::WithinAbs(rhs.min_x, 0.01));
    CHECK_THAT(lhs.max_x, Catch::Matchers::WithinAbs(rhs.max_x, 0.01));
    CHECK_THAT(lhs.min_y, Catch::Matchers::WithinAbs(rhs.min_y, 0.01));
    CHECK_THAT(lhs.max_y, Catch::Matchers::WithinAbs(rhs.max_y, 0.01));
}

void require_positive_deposition(const DepositionSummary &summary)
{
    REQUIRE(summary.finite);
    REQUIRE(summary.moves > 0);
    REQUIRE(summary.xy > 0.);
    REQUIRE(summary.e > 0.);
}

DynamicPrintConfig wipe_before_external_config(bool enabled, const char *wall_loops,
                                                const char *inner_width = "0")
{
    DynamicPrintConfig config = wipe_config("classic", false, "50%", "0%", false,
                                            wall_loops, "inner wall/outer wall");
    config.set_deserialize_strict({
        {"wipe", "0"},
        {"wipe_distance", "0"},
        {"wipe_before_external_loop", enabled ? "1" : "0"},
        {"wall_loops", wall_loops},
        {"inner_wall_line_width", inner_width},
        {"outer_wall_line_width", "0.45"},
        {"line_width", "0.45"},
        // Keep the first-layer role widths active; a positive initial-layer
        // width would override the 1.2mm wide-inner negative control.
        {"initial_layer_line_width", "0"},
        {"top_shell_layers", "0"},
        {"bottom_shell_layers", "0"},
        {"sparse_infill_density", "0%"},
        {"retract_when_changing_layer", "0"},
        {"retraction_minimum_travel", "0"},
        {"retraction_length", "0.8"},
        {"retraction_speed", "30"},
        {"deretraction_speed", "30"},
        {"retract_before_wipe", "0%"},
        {"retract_after_wipe", "0%"},
        {"z_hop", "0"},
        {"gcode_flavor", "marlin2"},
        {"use_relative_e_distances", "0"},
        {"enable_arc_fitting", "0"},
        {"gcode_comments", "1"},
        {"layer_change_gcode", ""},
    });
    return config;
}

} // namespace

TEST_CASE("Wipe before external loop enters the outer seam from an interior point",
          "[Wipe][WipeBeforeExternalLoop][Pipeline]")
{
    const DynamicPrintConfig enabled_config = wipe_before_external_config(true, "2");
    const DynamicPrintConfig disabled_config = wipe_before_external_config(false, "2");
    const DynamicPrintConfig one_wall_config = wipe_before_external_config(true, "1");
    const DynamicPrintConfig wide_inner_config = wipe_before_external_config(true, "2", "1.2");

    const WipeBeforeExternalEvidence enabled =
        parse_wipe_before_external(export_wipe_before_external(enabled_config), enabled_config);
    const WipeBeforeExternalEvidence disabled =
        parse_wipe_before_external(export_wipe_before_external(disabled_config), disabled_config);
    const WipeBeforeExternalEvidence one_wall =
        parse_wipe_before_external(export_wipe_before_external(one_wall_config), one_wall_config);
    const WipeBeforeExternalEvidence wide_inner =
        parse_wipe_before_external(export_wipe_before_external(wide_inner_config), wide_inner_config);

    REQUIRE(enabled.layer_markers >= 2);
    CHECK(disabled.layer_markers == enabled.layer_markers);
    CHECK(one_wall.layer_markers == enabled.layer_markers);
    CHECK(wide_inner.layer_markers == enabled.layer_markers);
    REQUIRE(enabled.markers.size() == enabled.layer_markers);
    CHECK(disabled.markers.empty());
    CHECK(one_wall.markers.empty());
    CHECK(wide_inner.markers.empty());

    require_positive_deposition(enabled.deposition);
    require_positive_deposition(disabled.deposition);
    require_positive_deposition(one_wall.deposition);
    require_positive_deposition(wide_inner.deposition);
    require_same_deposition(enabled.deposition, disabled.deposition);

    for (const PreExternalWipeMarker &marker : enabled.markers) {
        CAPTURE(marker.start.x(), marker.start.y(), marker.end.x(), marker.end.y(), marker.z);
        REQUIRE(marker.finite);
        REQUIRE(marker.xy > 0.05);
        REQUIRE(marker.xy <= kWipeBeforeExternalMaxTravel);
        CHECK_THAT(marker.e, Catch::Matchers::WithinAbs(0., 0.00005));
        const Vec2d midpoint = (marker.start + marker.end) * 0.5;
        REQUIRE(point_in_fixture_cube(marker.start));
        REQUIRE(point_in_fixture_cube(midpoint));
        REQUIRE(point_in_fixture_cube(marker.end));
        REQUIRE(point_near_previous_material(enabled.deposition, marker.start, marker.z,
                                             marker.prior_segment_count));
        // The unretraction point touches the inner wall. The following move
        // approaches the still-unprinted outer wall, so its midpoint need not
        // remain within the inner wall's nozzle-radius neighborhood.
        const auto depth_inside_cube = [](const Vec2d &point) {
            return std::min({point.x() - kWipeBeforeExternalCubeMin,
                             point.y() - kWipeBeforeExternalCubeMin,
                             kWipeBeforeExternalCubeMax - point.x(),
                             kWipeBeforeExternalCubeMax - point.y()});
        };
        CHECK(depth_inside_cube(marker.start) > depth_inside_cube(marker.end) + 0.05);
        REQUIRE(marker.prior_segment_count < enabled.deposition.segments.size());
        const auto &outer_start = enabled.deposition.segments[marker.prior_segment_count];
        REQUIRE(outer_start.outer_wall);
        CHECK_THAT(outer_start.z, Catch::Matchers::WithinAbs(marker.z, kWipeBeforeExternalCubePadding));
        CHECK((outer_start.start - marker.end).norm() <= kWipeBeforeExternalCubePadding);
    }
}

} // namespace wipe_before_external_contracts

} // namespace
} // namespace reduce_infill_retraction_contracts
