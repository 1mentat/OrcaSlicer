#include "libslic3r/GCode/WipeTower2.hpp"
#include <limits>
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "fff_print/test_helpers.hpp"
#include <cmath>
#include <set>
#include <utility>
#include <algorithm>
#include <catch2/catch_all.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include <cstddef>
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators_range.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/GCode/WipeTower.hpp"
#include "libslic3r/PrintConfig.hpp"

#include "test_helpers.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

// Taken from the config enum map rather than hand-listed, so a flavor added to GCodeFlavor later
// is covered here without editing this file.
static std::vector<GCodeFlavor> non_klipper_flavors()
{
    std::vector<GCodeFlavor> flavors;
    for (const auto &[name, value] : ConfigOptionEnum<GCodeFlavor>::get_enum_values())
        if (GCodeFlavor(value) != gcfKlipper)
            flavors.push_back(GCodeFlavor(value));
    return flavors;
}

static std::string flavor_name(GCodeFlavor flavor)
{
    return ConfigOptionEnum<GCodeFlavor>::get_enum_names()[int(flavor)];
}

TEST_CASE("Klipper flushes the wipe tower planner queue with M400", "[WipeTower]")
{
    CHECK(std::string(flush_planner_queue_command(gcfKlipper)) == "M400\n");
}

TEST_CASE("Other flavors flush the wipe tower planner queue with a zero dwell", "[WipeTower]")
{
    const GCodeFlavor flavor = GENERATE(from_range(non_klipper_flavors()));
    INFO("gcode flavor: " << flavor_name(flavor));
    CHECK(std::string(flush_planner_queue_command(flavor)) == "G4 S0\n");
}

// 1.5s is exactly representable as a float, so neither form can drift when rounded.
TEST_CASE("Klipper waits in the wipe tower with a millisecond dwell", "[WipeTower]")
{
    CHECK(wait_command(gcfKlipper, 1.5f) == "G4 P1500\n");
}

TEST_CASE("Other flavors wait in the wipe tower with a seconds dwell", "[WipeTower]")
{
    const GCodeFlavor flavor = GENERATE(from_range(non_klipper_flavors()));
    INFO("gcode flavor: " << flavor_name(flavor));
    CHECK(wait_command(flavor, 1.5f) == "G4 S1.500\n");
}

// The prime tower is validated against the real printable outline, so the placement clamps have to
// agree with it wherever that outline is not a rectangle. A regular hexagon inscribed in a 200mm
// circle stands in for the shipped delta beds.
TEST_CASE("The wipe tower placement clamp follows a non-rectangular bed outline", "[WipeTower]")
{
    const coord_t margin = scaled<coord_t>(1.);
    auto square_at = [](double x, double y, double side) {
        return BoundingBox(Point::new_scale(x, y), Point::new_scale(x + side, y + side));
    };
    // Does the footprint, padded by pad, sit inside the outline once the returned move is applied?
    auto lands_inside = [](BoundingBox box, const Polygons &bed, const Vec2f &move, coord_t pad) {
        box.translate(Point::new_scale(move.x(), move.y()));
        return diff(Polygons{box.inflated(pad).polygon()}, bed).empty();
    };

    const Polygons hex_bed{make_circle_num_segments(scaled<double>(100.), 6)};
    const Polygons square_bed{Polygon::new_scale(Pointfs{{0., 0.}, {200., 0.}, {200., 200.}, {0., 200.}})};

    SECTION("a rectangular bed is left to the bounding box clamp") {
        const Vec2f move = WipeTower::move_box_inside_polygon(square_at(50., 50., 30.), square_bed, margin);
        CHECK_THAT(move.x(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT(move.y(), Catch::Matchers::WithinAbs(0., 1e-6));
    }

    // Dragging the tower off one edge may not pull it away from the other, or it would jump out from
    // under the cursor instead of sliding along the edge.
    SECTION("only the violated axis is clamped") {
        const Vec2f move = WipeTower::move_box_inside_polygon(square_at(185., 50., 30.), square_bed, margin);
        CHECK_THAT(move.x(), Catch::Matchers::WithinAbs(-16., 1e-6));
        CHECK_THAT(move.y(), Catch::Matchers::WithinAbs(0., 1e-6));
    }

    SECTION("a footprint already inside the outline is left alone") {
        const Vec2f move = WipeTower::move_box_inside_polygon(square_at(-15., -15., 30.), hex_bed, margin);
        CHECK_THAT(move.x(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT(move.y(), Catch::Matchers::WithinAbs(0., 1e-6));
    }

    SECTION("a footprint in the bounding box corner is pulled onto the bed") {
        const BoundingBox box = square_at(55., 50., 30.);
        REQUIRE_FALSE(lands_inside(box, hex_bed, Vec2f::Zero(), margin)); // in the bbox, off the hexagon
        CHECK(lands_inside(box, hex_bed, WipeTower::move_box_inside_polygon(box, hex_bed, margin), margin));
    }

    // An unresolved auto brim width reaches the drag clamp as a negative margin. Padding by it would
    // shrink the footprint and hand back a position the slice validation still rejects.
    SECTION("a negative margin still lands the footprint inside the outline") {
        const BoundingBox box = square_at(55., 50., 30.);
        const coord_t     brim = scaled<coord_t>(-0.5);
        CHECK(lands_inside(box, hex_bed, WipeTower::move_box_inside_polygon(box, hex_bed, brim), 0));
    }

    SECTION("a footprint too large for the bed is left alone") {
        const Vec2f move = WipeTower::move_box_inside_polygon(square_at(-200., -200., 400.), hex_bed, margin);
        CHECK_THAT(move.x(), Catch::Matchers::WithinAbs(0., 1e-6));
        CHECK_THAT(move.y(), Catch::Matchers::WithinAbs(0., 1e-6));
    }
}

// The cases above only exercise the helpers in isolation. The one below slices a real
// two-filament print, so it also covers the binding constraint of both changes: that the
// configured `gcode_flavor` reaches the wipe tower writer and lands in the exported G-code.

// The G-code inside each WIPE_TOWER_START/WIPE_TOWER_END pair, concatenated, so an M400 emitted
// outside the tower (e.g. GCodeProcessor's pre-heat injector) cannot create a false match.
static std::string wipe_tower_regions(const std::string &gcode)
{
    const std::string &start_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_Start);
    const std::string &end_tag   = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End);
    std::string regions;
    size_t pos = 0;
    while (true) {
        size_t start = gcode.find(start_tag, pos);
        if (start == std::string::npos)
            break;
        size_t end = gcode.find(end_tag, start);
        if (end == std::string::npos)
            break;
        regions.append(gcode, start, end - start);
        pos = end + 1;
    }
    return regions;
}

// A per-layer toolchange between the wall and infill filaments, same shape as
// test_multifilament.cpp's "Each feature prints with its assigned filament", so the wipe tower
// runs its toolchange path (and so `flush_planner_queue()`) on every layer.
static DynamicPrintConfig wipe_tower_toolchange_config(const std::string &gcode_flavor)
{
    return multifilament_config(2, {
        { "sparse_infill_filament_id",  1 },
        { "internal_solid_filament_id", 1 },
        { "top_surface_filament_id",    1 },
        { "bottom_surface_filament_id", 1 },
        { "outer_wall_filament_id",     2 },
        { "inner_wall_filament_id",     2 },
        { "enable_prime_tower",         true },
        { "wipe_tower_x",               50 }, // inside the 200x200 test bed
        { "wipe_tower_y",               50 }, // (the default y, 220, is not)
        { "layer_height",               0.3 },
        { "gcode_flavor",               gcode_flavor },
    });
}

// Slices a 10mm cube under `config`. Not plain Test::slice: a brand-new Print's first `apply()`
// counts one filament in use, and DynamicPrintConfig::normalize_fdm_2's single-filament rule then
// clears `enable_prime_tower`. A second apply, once init_print's regions have settled, sees both
// filaments and the tower survives.
static std::string slice_with_prime_tower(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ cube(10) }, print, model, config);
    print.apply(model, config);
    return gcode(print);
}

TEST_CASE("The wipe tower's toolchange planner flush follows the gcode flavor", "[WipeTower]")
{
    auto [flavor, expected, unexpected] = GENERATE(table<std::string, std::string, std::string>({
        { "klipper", "M400",  "G4 S0" },
        { "marlin",  "G4 S0", "M400"  } }));
    DYNAMIC_SECTION(flavor) {
        const std::string tower = wipe_tower_regions(slice_with_prime_tower(wipe_tower_toolchange_config(flavor)));
        REQUIRE_FALSE(tower.empty());
        CHECK_THAT(tower, Catch::Matchers::ContainsSubstring(expected));
        CHECK_THAT(tower, !Catch::Matchers::ContainsSubstring(unexpected));
    }
}

// What Print feeds the shared estimate. The libslic3r WipeTowerEstimate cases cannot see this:
// they call the estimator directly. The estimate counts the filaments the print really uses,
// so the two-filament shape gives the outer wall the second one.
static DynamicPrintConfig tower_estimate_config(const char *wall_type, unsigned int filaments = 2)
{
    // 100 mm3 per purge on a 50 mm wide tower: one purge is 100/(layer_height * 50) of depth.
    return multifilament_config(filaments, {
        { "outer_wall_filament_id",         filaments == 2 ? "2" : "1" },
        { "enable_prime_tower",             "1"       },
        { "wipe_tower_wall_type",           wall_type },
        { "prime_tower_width",              "50"      },
        { "prime_volume",                   "100"     },
        { "prime_tower_infill_gap",         "100%"    },
        { "prime_tower_brim_width",         "3"       },
        { "purge_in_prime_tower",           "0"       },
        { "single_extruder_multi_material", "0"       },
        { "timelapse_type",                 "0"       },
        { "layer_height",                   "0.2"     },
        { "enable_wrapping_detection",      "0"       },
        { "raft_layers",                    "0"       } });
}

TEST_CASE("The tower is sized for the thinnest layer any object on the plate is sliced at", "[WipeTower]")
{
    // The tower has to survive its thinnest layer, so an override finer than the preset drives
    // the estimate even on the second object. Two 20 mm cubes, the second at 0.1 mm.
    const DynamicPrintConfig config = tower_estimate_config("rectangle");
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides = {
        {}, { { "layer_height", "0.1" } } };

    Print print;
    Model model;
    init_print({ cube(20), cube(20) }, print, model, config, &overrides);

    // One purge at 0.1 mm: 100 / (0.1 * 50) = 20 mm, above the 20 mm-tall tower's stability
    // floor. At the preset's 0.2 mm it would be half that, so the two are easy to tell apart.
    const float floor_20mm = WipeTower::get_limit_depth_by_height(20.f);
    REQUIRE(floor_20mm < 10.f);
    CHECK_THAT(print.wipe_tower_data(2).depth, Catch::Matchers::WithinAbs(20., 1e-4));
}

TEST_CASE("Validation is given the tower's effective width, not the configured one", "[WipeTower]")
{
    // A rib wall squares the tower, so its width is its depth. Validation reads this rather
    // than re-deriving the rule from the wall type.
    Print print;
    Model model;

    SECTION("a rectangle wall keeps the configured width") {
        const DynamicPrintConfig config = tower_estimate_config("rectangle");
        init_print({ cube(20) }, print, model, config);
        const WipeTowerData &data = print.wipe_tower_data(2);
        CHECK_THAT(data.width, Catch::Matchers::WithinAbs(50., 1e-4));
        CHECK(data.depth < data.width);
    }

    SECTION("a rib wall reports the squared footprint") {
        const DynamicPrintConfig config = tower_estimate_config("rib");
        init_print({ cube(20) }, print, model, config);
        const WipeTowerData &data = print.wipe_tower_data(2);
        CHECK_THAT(data.width, Catch::Matchers::WithinAbs(data.depth, 1e-4));
        CHECK(data.width > 0.f);
    }
}

TEST_CASE("Generating the tower keeps its reported width current", "[WipeTower]")
{
    // width is handed out after the slice, so leaving it at the estimate reports a zero-width
    // tower to every post-generation consumer.
    const DynamicPrintConfig config = wipe_tower_toolchange_config("marlin");
    Print print;
    Model model;
    init_print({ cube(10) }, print, model, config);
    print.apply(model, config);
    REQUIRE(print.wipe_tower_data(2).width > 0.f);

    print.process();
    REQUIRE(print.is_step_done(psWipeTower));
    const WipeTowerData &data = print.wipe_tower_data();
    // A width the generator never wrote reads as zero. A rib wall squares the tower, so the
    // generated width is the body square: under the configured 50 mm, and inside the depth.
    CHECK(data.width > 0.f);
    CHECK(data.width < 50.f);
    CHECK(data.width <= data.depth + EPSILON);
}

TEST_CASE("A single-filament plate reserves a tower only when one is actually printed", "[WipeTower]")
{
    // The estimate has to answer this the way Print::apply does: reporting no tower for one
    // that is built collapses the validation hull to a point, and reporting one for a tower
    // that is not built takes that bed area away from the arranger and draws a preview box
    // over nothing.
    Print print;
    Model model;

    SECTION("no tool change and nothing else that prints one") {
        const DynamicPrintConfig config = tower_estimate_config("rib", 1);
        init_print({ cube(20) }, print, model, config);
        REQUIRE_FALSE(print.has_wipe_tower());
        CHECK_THAT(print.wipe_tower_data(1).depth, Catch::Matchers::WithinAbs(0., 1e-6));
    }

    // A raft puts the tower on every layer below the object, but only where there is a tower:
    // Print::apply runs normalize_fdm_2, which clears enable_prime_tower for a plate that
    // purges one filament and has neither smooth timelapse nor wrapping detection on.
    SECTION("a raft alone does not print one") {
        DynamicPrintConfig config = tower_estimate_config("rib", 1);
        config.set_deserialize_strict({ { "raft_layers", "3" } });
        init_print({ cube(20) }, print, model, config);
        REQUIRE_FALSE(print.config().enable_prime_tower.value);
        REQUIRE_FALSE(print.has_wipe_tower());
        CHECK_THAT(print.wipe_tower_data(1).depth, Catch::Matchers::WithinAbs(0., 1e-6));
    }

    SECTION("smooth timelapse prints one, and keeps enable_prime_tower on") {
        DynamicPrintConfig config = tower_estimate_config("rib", 1);
        config.set_deserialize_strict({ { "timelapse_type", "1" } });
        init_print({ cube(20) }, print, model, config);
        REQUIRE(print.has_wipe_tower());
        CHECK(print.wipe_tower_data(1).depth > 0.f);
    }
}

// Filament 2 on the top surface only, so every layer below it is a toolchange-free tower layer: the
// run "Combine sparse layers" folds. The two heights decide whether anything folds, so they are the
// caller's business.
static DynamicPrintConfig sparse_run_config(double layer_height, const char *max_layer_height, bool combine)
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "top_surface_filament_id",        2     },
        { "enable_prime_tower",             true  },
        { "wipe_tower_x",                   50    }, // inside the 200x200 test bed
        { "wipe_tower_y",                   50    },
        { "prime_tower_width",              35    },
        { "min_layer_height",               "0.08"},
        { "single_extruder_multi_material", true  },
        { "timelapse_type",                 "0"   },
        { "enable_wrapping_detection",      false },
        { "raft_layers",                    "0"   } });
    // A taller first layer would top the plan and hide what the run does, so slice at one height.
    config.set_deserialize_strict({ { "layer_height",               std::to_string(layer_height) },
                                    { "initial_layer_print_height", std::to_string(layer_height) },
                                    { "max_layer_height", max_layer_height },
                                    { "wipe_tower_sparse_layers_combination", combine ? "1" : "0" } });
    return config;
}

// What a sliced tower did with its sparse run.
struct SparseRunResult { size_t planned, sparse, folded; float tallest_printed, printed_height; std::string gcode; };

static SparseRunResult slice_sparse_run(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ cube(10) }, print, model, config);
    print.apply(model, config);
    print.process();
    REQUIRE(print.is_step_done(psWipeTower));

    SparseRunResult r{};
    for (const std::vector<WipeTower::ToolChangeResult> &layer : print.wipe_tower_data().tool_changes) {
        if (layer.empty())
            continue;
        ++r.planned;
        if (wipe_tower_layer_is_sparse(layer))
            ++r.sparse;
        if (wipe_tower_layer_is_combined_away(layer)) {
            ++r.folded;
        } else {
            r.tallest_printed = std::max(r.tallest_printed, layer.front().layer_height);
            r.printed_height += layer.front().layer_height;
        }
    }
    r.gcode = Slic3r::Test::gcode(print);
    return r;
}

// How often the G-code declares `height` in the tag this printer's processor reads. The dialect is a
// global the exporter sets from the printer, so this is only correct after a slice - the point below.
static size_t count_height_tags(const std::string &gcode, const char *height)
{
    const std::string tag = ";" + GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Height) + height + "\n";
    size_t n = 0;
    for (size_t p = gcode.find(tag); p != std::string::npos; p = gcode.find(tag, p + 1))
        ++n;
    return n;
}

TEST_CASE("Combining sparse layers folds a run into whole layers the nozzle can lay down", "[WipeTower]")
{
    // 0.1 mm layers under a 0.32 mm cap: three fit (0.3), a fourth does not, so a run prints once
    // every three layers at 0.3 mm.
    const SparseRunResult plain    = slice_sparse_run(sparse_run_config(0.1, "0.32", false));
    const SparseRunResult combined = slice_sparse_run(sparse_run_config(0.1, "0.32", true));

    REQUIRE(plain.planned == combined.planned); // the plan still has one layer per object layer
    REQUIRE(plain.sparse > 10);
    CHECK(plain.folded == 0);
    CHECK_THAT(plain.tallest_printed, Catch::Matchers::WithinAbs(0.1f, 1e-4f));

    CHECK(combined.folded > 0);
    CHECK_THAT(combined.tallest_printed, Catch::Matchers::WithinAbs(0.3f, 1e-4f));
    // Two of every three sparse layers fold away, leaving the toolchange layers untouched.
    CHECK(combined.folded <= plain.sparse);
    CHECK(combined.folded >= plain.sparse / 2);
    // What folds away comes back as height on the layer that prints the run: no gap, nothing twice.
    CHECK_THAT(combined.printed_height, Catch::Matchers::WithinAbs(plain.printed_height, 1e-3f));
}

TEST_CASE("A run too thin to reach the nozzle's layer height is left alone", "[WipeTower]")
{
    // Only whole layers merge, so two 0.2 mm layers (0.4) do not fit a 0.32 mm maximum and the tower
    // prints as if the option were off. This is the common 0.4 nozzle case; the tooltip says so.
    const SparseRunResult plain    = slice_sparse_run(sparse_run_config(0.2, "0.32", false));
    const SparseRunResult combined = slice_sparse_run(sparse_run_config(0.2, "0.32", true));

    REQUIRE(plain.sparse > 10);
    CHECK(combined.folded == 0);
    CHECK(combined.planned == plain.planned);
    CHECK_THAT(combined.tallest_printed, Catch::Matchers::WithinAbs(0.2f, 1e-4f));
}

TEST_CASE("A merged tower layer declares its own height to the G-code processor", "[WipeTower]")
{
    // Each writer declares a height in a hardcoded tag dialect while the processor reads only its
    // printer's, so one of them is always dropped. A merged layer is the first time that shows, as a
    // thick layer drawn and costed as a thin one. 0.2 mm layers under a 0.42 mm maximum merge in pairs.
    const SparseRunResult plain    = slice_sparse_run(sparse_run_config(0.2, "0.42", false));
    const SparseRunResult combined = slice_sparse_run(sparse_run_config(0.2, "0.42", true));

    REQUIRE(combined.folded > 0);
    CHECK_THAT(combined.tallest_printed, Catch::Matchers::WithinAbs(0.4f, 1e-4f));
    // Every layer that prints a merged run has to say so, and nothing may say so without the option.
    CHECK(count_height_tags(combined.gcode, "0.4") - count_height_tags(plain.gcode, "0.4") == combined.folded);
}

TEST_CASE("A tower printed without a tool change is still validated against the bed", "[WipeTower]")
{
    // Wrapping detection prints a tower on a plate that purges one filament. Neither the old
    // estimate (which read the wall type and smooth timelapse) nor the old containment gate (the
    // filament count or smooth timelapse) knew about it, so between them that tower was never
    // checked against the bed.
    Print print;
    Model model;
    DynamicPrintConfig config = tower_estimate_config("rectangle", 1);
    // Relative E without a per-layer G92 is rejected before the tower is ever looked at, and
    // has_wipe_tower() wants a real exclusion polygon before it honours wrapping detection.
    config.set_deserialize_strict({ { "enable_wrapping_detection", "1" },
                                    { "wrapping_exclude_area", "180x180,190x180,190x190,180x190" },
                                    { "wipe_tower_x", "500" }, { "wipe_tower_y", "500" },                                    { "use_relative_e_distances", "0" } });

    init_print({ cube(20) }, print, model, config);
    REQUIRE(print.extruders(true).size() == 1);
    REQUIRE(print.has_wipe_tower());
    CHECK(print.wipe_tower_data(1).depth > 0.f);
    CHECK_THAT(print.validate().string, Catch::Matchers::ContainsSubstring("printable area"));
}

namespace wipe_tower_pipeline_contracts {

namespace {

// Two roles deliberately use different filament slots.  A single physical
// nozzle is enough to exercise ToolOrdering's logical filament transitions and
// both purge-tower implementations, while keeping every case small and deterministic.
DynamicPrintConfig tower_config(double prime_volume = 80., bool semm = false, bool top_only = false)
{
    const int wall_filament = top_only ? 1 : 2;
    DynamicPrintConfig config = multifilament_config(2, {
        { "sparse_infill_filament_id",       1 },
        { "internal_solid_filament_id",      1 },
        { "top_surface_filament_id",         top_only ? 2 : 1 },
        { "bottom_surface_filament_id",      1 },
        { "outer_wall_filament_id",          wall_filament },
        { "inner_wall_filament_id",          wall_filament },
        { "enable_prime_tower",              true },
        { "wipe_tower_x",                    50 },
        { "wipe_tower_y",                    50 },
        { "prime_tower_width",               35 },
        { "wipe_tower_type",                 "type2" },
        { "layer_height",                    0.2 },
        { "skirt_loops",                     0 },
        { "brim_type",                       "no_brim" },
        { "single_extruder_multi_material",  semm },
        { "single_extruder_multi_material_priming", semm },
        { "use_relative_e_distances",        true },
        { "layer_change_gcode",             "G92 E0\n" },
        { "wipe_tower_sparse_layers_combination", false },
        { "prime_volume",                    prime_volume },
    });
    // Keep this explicit: init_print's first apply can normalize a one-filament
    // plate before the object regions have settled.  The helper below reapplies
    // the same complete configuration once both slots are known to be used.
    return config;
}

struct TowerMetrics {
    std::size_t planned_layers = 0;
    std::size_t active_blocks = 0;
    std::size_t sparse_layers = 0;
    std::size_t combined_layers = 0;
    std::size_t transition_events = 0;
    std::size_t same_tool_blocks = 0;
    std::size_t invalid_results = 0;
    std::size_t gcode_tool_commands = 0;
    std::size_t arc_commands = 0;
    double extrusion_length = 0.;
    double elapsed_time = 0.;
    std::size_t bad_print_z = 0;
    std::size_t bad_layer_height = 0;
    std::size_t bad_elapsed = 0;
    std::size_t bad_purge = 0;
    std::size_t bad_endpoint = 0;
    std::size_t bad_extrusion = 0;
    std::set<std::pair<int, int>> transition_pairs;
    std::set<int> gcode_tools;
    FilamentChangeStats stats;
    std::vector<unsigned int> all_extruders;
};

void collect_gcode_tools(const std::string &gcode, TowerMetrics &metrics)
{
    GCodeReader reader;
    reader.parse_buffer(gcode, [&metrics](GCodeReader &, const GCodeReader::GCodeLine &line) {
        const std::string command(line.cmd());
        if (command.size() < 2 || command.front() != 'T' || command[1] < '0' || command[1] > '9')
            return;
        metrics.gcode_tool_commands++;
        metrics.gcode_tools.insert(std::stoi(command.substr(1)));
    });
}

TowerMetrics process_tower(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    init_print({ cube(10) }, print, model, config);
    // See test_wipe_tower.cpp: the first apply may normalize the prime tower
    // away before the second filament is observed by the print regions.
    print.apply(model, config);
    const auto validation = print.validate();
    INFO("validation: " << validation.string);
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE(print.is_step_done(psWipeTower));

    TowerMetrics metrics;
    const WipeTowerData &tower = print.wipe_tower_data();
    for (const std::vector<WipeTower::ToolChangeResult> &layer : tower.tool_changes) {
        if (layer.empty())
            continue;
        ++metrics.planned_layers;
        if (wipe_tower_layer_is_sparse(layer))
            ++metrics.sparse_layers;
        if (wipe_tower_layer_is_combined_away(layer))
            ++metrics.combined_layers;

        for (const WipeTower::ToolChangeResult &change : layer) {
            if (change.combined_away)
                continue;
            ++metrics.active_blocks;
            if (change.initial_tool == change.new_tool)
                ++metrics.same_tool_blocks;
            else {
                ++metrics.transition_events;
                metrics.transition_pairs.emplace(change.initial_tool, change.new_tool);
            }

            for (std::size_t i = 1; i < change.extrusions.size(); ++i) {
                const WipeTower::Extrusion &extrusion = change.extrusions[i];
                if (extrusion.width <= 0.f)
                    continue;
                const Vec2f delta = extrusion.pos - change.extrusions[i - 1].pos;
                metrics.extrusion_length += std::hypot(double(delta.x()), double(delta.y()));
            }

            GCodeReader change_reader;
            change_reader.parse_buffer(change.gcode, [&metrics](GCodeReader &, const GCodeReader::GCodeLine &line) {
                if (line.cmd() == "G2" || line.cmd() == "G3")
                    ++metrics.arc_commands;
            });

            const auto finite = [](float value) { return std::isfinite(double(value)); };
            metrics.elapsed_time += change.elapsed_time;
            if (!finite(change.print_z)) { ++metrics.invalid_results; ++metrics.bad_print_z; }
            if (!finite(change.layer_height) || change.layer_height <= 0.f) {
                ++metrics.invalid_results; ++metrics.bad_layer_height;
            }
            if (!finite(change.elapsed_time)) { ++metrics.invalid_results; ++metrics.bad_elapsed; }
            if (!finite(change.purge_volume)) { ++metrics.invalid_results; ++metrics.bad_purge; }
            if (!finite(change.start_pos.x()) || !finite(change.start_pos.y()) ||
                !finite(change.end_pos.x()) || !finite(change.end_pos.y()) ||
                (change.is_tool_change && (!finite(change.tool_change_start_pos.x()) ||
                                           !finite(change.tool_change_start_pos.y()))))
            {
                ++metrics.invalid_results;
                ++metrics.bad_endpoint;
            }
            for (const WipeTower::Extrusion &extrusion : change.extrusions)
                if (!finite(extrusion.pos.x()) || !finite(extrusion.pos.y()) || !finite(extrusion.width))
                {
                    ++metrics.invalid_results;
                    ++metrics.bad_extrusion;
                }
        }
    }

    metrics.stats = const_cast<ToolOrdering &>(print.tool_ordering())
                        .get_filament_change_stats(ToolOrdering::SingleExt);
    metrics.all_extruders = print.tool_ordering().all_extruders();
    collect_gcode_tools(gcode(print), metrics);
    INFO("invalid categories print_z=" << metrics.bad_print_z
         << " layer_height=" << metrics.bad_layer_height
         << " elapsed=" << metrics.bad_elapsed
         << " purge=" << metrics.bad_purge
         << " endpoint=" << metrics.bad_endpoint
         << " extrusion=" << metrics.bad_extrusion);
    CHECK(metrics.invalid_results == 0);
    return metrics;
}

} // namespace

TEST_CASE("A real two-filament print emits coherent wipe-tower transitions", "[WipeTower][WipeTowerVariance]")
{
    const TowerMetrics metrics = process_tower(tower_config());

    REQUIRE(metrics.planned_layers > 0);
    REQUIRE(metrics.active_blocks > 0);
    REQUIRE(metrics.transition_events > 0);
    CHECK(metrics.transition_pairs.count({ 0, 1 }) + metrics.transition_pairs.count({ 1, 0 }) > 0);
    CHECK(metrics.extrusion_length > 0.);
    CHECK(metrics.elapsed_time > 0.);
    CHECK(metrics.gcode_tool_commands >= metrics.transition_events);
    CHECK(metrics.gcode_tools.count(0) > 0);
    CHECK(metrics.gcode_tools.count(1) > 0);
    CHECK(metrics.stats.filament_change_count > 0);
    CHECK(metrics.stats.flush_filament_change_count > 0);
    CHECK_FALSE(metrics.all_extruders.empty());
    CHECK(metrics.invalid_results == 0);
}

TEST_CASE("Increasing prime volume increases extrusion without changing tool transitions", "[WipeTower][WipeTowerVariance]")
{
    const TowerMetrics low  = process_tower(tower_config(35.));
    const TowerMetrics high = process_tower(tower_config(120.));

    REQUIRE(low.transition_events > 0);
    REQUIRE(high.transition_events > 0);
    CHECK(high.extrusion_length > low.extrusion_length);
    CHECK(high.transition_events == low.transition_events);
    CHECK(high.transition_pairs == low.transition_pairs);
    CHECK(high.gcode_tools == low.gcode_tools);
}

TEST_CASE("Sparse tower blocks retain same-tool layers beside real changes", "[WipeTower][WipeTowerVariance]")
{
    const TowerMetrics metrics = process_tower(tower_config(80., false, true));

    REQUIRE(metrics.planned_layers > 0);
    CHECK(metrics.sparse_layers > 0);
    CHECK(metrics.same_tool_blocks > 0);
    CHECK(metrics.transition_events > 0);
    CHECK(metrics.invalid_results == 0);
}

TEST_CASE("SEMM still records material transitions and extrusion geometry", "[WipeTower][WipeTowerVariance]")
{
    const TowerMetrics metrics = process_tower(tower_config(80., true));

    REQUIRE(metrics.planned_layers > 0);
    CHECK(metrics.transition_events > 0);
    CHECK(metrics.extrusion_length > 0.);
    CHECK(metrics.elapsed_time > 0.);
    CHECK(metrics.stats.filament_change_count > 0);
    INFO("invalid categories print_z=" << metrics.bad_print_z
         << " layer_height=" << metrics.bad_layer_height
         << " elapsed=" << metrics.bad_elapsed
         << " purge=" << metrics.bad_purge
         << " endpoint=" << metrics.bad_endpoint
         << " extrusion=" << metrics.bad_extrusion);
    CHECK(metrics.invalid_results == 0);
}

TEST_CASE("Type1 wipe towers retain finite positive timing for linear and arc motion", "[WipeTower][WipeTowerVariance][WipeTowerLegacyVariance]")
{
    DynamicPrintConfig config = tower_config();
    config.set_deserialize_strict({{"wipe_tower_type", "type1"}, {"enable_arc_fitting", true}});
    const TowerMetrics metrics = process_tower(config);
    REQUIRE(metrics.planned_layers > 0);
    REQUIRE(metrics.transition_events > 0);
    CHECK(metrics.elapsed_time > 0.);
    CHECK(metrics.arc_commands > 0);
    CHECK(metrics.invalid_results == 0);
}

} // namespace wipe_tower_pipeline_contracts


namespace extended_tower_contracts {

using namespace Slic3r;
using namespace Slic3r::Test;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

DynamicPrintConfig extraction_config(bool semm, bool purge)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", semm},
        {"purge_in_prime_tower", purge},
        {"flush_volumes_matrix", "0,10,20,0"},
        {"flush_multiplier", "2"},
        {"filament_minimal_purge_on_wipe_tower", "3,7"},
    });
    return config;
}

FullPrintConfig direct_tower_config(WipeTowerWallType wall_type = WipeTowerWallType::wtwRectangle)
{
    DynamicPrintConfig dynamic = multifilament_config(2, {
        {"single_extruder_multi_material", false},
        {"enable_prime_tower", true},
        {"prime_tower_width", 35},
        {"prime_tower_brim_width", 0},
        {"wipe_tower_x", 20},
        {"wipe_tower_y", 20},
        {"wipe_tower_wall_type", wall_type == WipeTowerWallType::wtwRib ? "rib" : "rectangle"},
        {"prime_tower_skip_points", false},
        {"wipe_tower_no_sparse_layers", false},
        {"wipe_tower_extra_flow", 100},
        {"wipe_tower_extra_spacing", 100},
        {"enable_arc_fitting", false},
    });
    FullPrintConfig full;
    full.apply(dynamic);
    return full;
}

struct PlannedLayer {
    float z;
    float height;
    unsigned int old_tool;
    unsigned int new_tool;
    float wipe_volume;
};

struct DirectRun {
    std::size_t planned_layers = 0;
    std::size_t active_layers = 0;
    int tool_changes = 0;
    float depth = 0.f;
    float height = 0.f;
    float width = 0.f;
    float brim = 0.f;
    std::vector<float> used_filament;
    std::vector<std::pair<float, std::vector<float>>> used_until_layer;
    std::vector<std::pair<float, float>> z_depth;
    std::vector<std::vector<WipeTower::ToolChangeResult>> result;
};

DirectRun run_direct_tower(const std::vector<PlannedLayer> &layers,
                           WipeTowerWallType wall_type = WipeTowerWallType::wtwRectangle)
{
    FullPrintConfig config = direct_tower_config(wall_type);
    const std::vector<std::vector<float>> wiping_matrix{{0.f, 280.f}, {280.f, 0.f}};
    WipeTower2 tower(config, static_cast<const PrintRegionConfig &>(config), 0, Vec3d::Zero(), wiping_matrix, 0);
    tower.set_extruder(0, config);
    tower.set_extruder(1, config);
    for (const PlannedLayer &layer : layers)
        tower.plan_toolchange(layer.z, layer.height, layer.old_tool, layer.new_tool, layer.wipe_volume);

    DirectRun run;
    tower.generate(run.result);
    for (const auto &layer : run.result) {
        ++run.planned_layers;
        if (!layer.empty())
            ++run.active_layers;
    }
    run.tool_changes = tower.get_number_of_toolchanges();
    run.depth = tower.get_depth();
    run.height = tower.get_wipe_tower_height();
    run.width = tower.width();
    run.brim = tower.get_brim_width();
    run.used_filament = tower.get_used_filament();
    run.used_until_layer = tower.get_used_filament_until_layer();
    run.z_depth = tower.get_z_and_depth_pairs();
    return run;
}

void require_finite_result(const DirectRun &run)
{
    REQUIRE(run.planned_layers > 0);
    REQUIRE(run.active_layers > 0);
    CHECK(std::isfinite(double(run.depth)));
    CHECK(std::isfinite(double(run.height)));
    CHECK(std::isfinite(double(run.width)));
    CHECK(std::isfinite(double(run.brim)));
    for (const auto &layer : run.result)
        for (const auto &tcr : layer) {
            CHECK(std::isfinite(double(tcr.print_z)));
            CHECK(std::isfinite(double(tcr.layer_height)));
            CHECK(std::isfinite(double(tcr.elapsed_time)));
            CHECK(std::isfinite(double(tcr.purge_volume)));
            CHECK(tcr.layer_height > 0.f);
            CHECK(std::isfinite(double(tcr.start_pos.x())));
            CHECK(std::isfinite(double(tcr.start_pos.y())));
            CHECK(std::isfinite(double(tcr.end_pos.x())));
            CHECK(std::isfinite(double(tcr.end_pos.y())));
        }
}

} // namespace

TEST_CASE("WipeTower2 extracts SEMM flush volumes and preserves minimum purge floors", "[WipeTower][WipeTowerExtended]")
{
    const DynamicPrintConfig config = extraction_config(true, true);
    const auto matrix = WipeTower2::extract_wipe_volumes(config);
    REQUIRE(matrix.size() == 2);
    REQUIRE(matrix[0].size() == 2);
    REQUIRE(matrix[1].size() == 2);
    CHECK_THAT(matrix[0][0], WithinAbs(3.f, 1e-6f));
    CHECK_THAT(matrix[0][1], WithinAbs(20.f, 1e-6f));
    CHECK_THAT(matrix[1][0], WithinAbs(40.f, 1e-6f));
    CHECK_THAT(matrix[1][1], WithinAbs(7.f, 1e-6f));
}

TEST_CASE("WipeTower2 uses minimum purge volumes unless both SEMM and matrix purging are enabled", "[WipeTower][WipeTowerExtended]")
{
    const auto [semm, purge] = GENERATE(std::pair<bool, bool>{true, false},
                                       std::pair<bool, bool>{false, true},
                                       std::pair<bool, bool>{false, false});
    const DynamicPrintConfig config = extraction_config(semm, purge);
    const auto matrix = WipeTower2::extract_wipe_volumes(config);
    REQUIRE(matrix.size() == 2);
    REQUIRE(matrix[0].size() == 2);
    REQUIRE(matrix[1].size() == 2);
    CHECK_THAT(matrix[0][0], WithinAbs(3.f, 1e-6f));
    CHECK_THAT(matrix[0][1], WithinAbs(7.f, 1e-6f));
    CHECK_THAT(matrix[1][0], WithinAbs(3.f, 1e-6f));
    CHECK_THAT(matrix[1][1], WithinAbs(7.f, 1e-6f));
}

TEST_CASE("WipeTower2 folds multiple nozzle flush blocks by the worst scaled pair", "[WipeTower][WipeTowerExtended]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", true},
        {"purge_in_prime_tower", true},
        {"nozzle_diameter", "0.4,0.8"},
        // Different nozzles determine each direction: max(51, 21*2) and
        // max(12, 22*2). Both folding and per-nozzle scaling are observable.
        {"flush_volumes_matrix", "0,51,12,0,0,21,22,0"},
        {"flush_multiplier", "1,2"},
        {"filament_minimal_purge_on_wipe_tower", "0,0"},
    });
    const auto matrix = WipeTower2::extract_wipe_volumes(config);
    REQUIRE(matrix.size() == 2);
    REQUIRE(matrix[0].size() == 2);
    REQUIRE(matrix[1].size() == 2);
    CHECK_THAT(matrix[0][1], WithinAbs(51.f, 1e-6f));
    CHECK_THAT(matrix[1][0], WithinAbs(44.f, 1e-6f));
}

TEST_CASE("WipeTower2 accepts an old single-nozzle matrix when nozzle count changed", "[WipeTower][WipeTowerExtended]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", true},
        {"purge_in_prime_tower", true},
        {"nozzle_diameter", "0.4,0.8"},
        {"flush_volumes_matrix", "0,11,12,0"},
        {"flush_multiplier", "1,2"},
        {"filament_minimal_purge_on_wipe_tower", "0,0"},
    });
    const auto matrix = WipeTower2::extract_wipe_volumes(config);
    REQUIRE(matrix.size() == 2);
    REQUIRE(matrix[0].size() == 2);
    REQUIRE(matrix[1].size() == 2);
    CHECK_THAT(matrix[0][1], WithinAbs(11.f, 1e-6f));
    CHECK_THAT(matrix[1][0], WithinAbs(12.f, 1e-6f));
}

TEST_CASE("WipeTower2 returns an empty flush estimate for an empty matrix", "[WipeTower][WipeTowerExtended]")
{
    DynamicPrintConfig config = extraction_config(true, true);
    config.option<ConfigOptionFloats>("flush_volumes_matrix")->values.clear();
    CHECK_THAT(WipeTower2::estimate_semm_flush_volume(config, 2), WithinAbs(0.f, 1e-6f));
}

TEST_CASE("WipeTower2 SEMM estimate averages row maxima and scales by filament count", "[WipeTower][WipeTowerExtended]")
{
    const DynamicPrintConfig config = extraction_config(true, true);
    // The configured flush multiplier scales the reverse pair to 40 mm3, so the
    // row maxima are 20 and 40: (20 + 40) / 2 * 0.6 = 18 mm3 per filament.
    CHECK_THAT(WipeTower2::estimate_semm_flush_volume(config, 1), WithinAbs(18.f, 1e-5f));
    CHECK_THAT(WipeTower2::estimate_semm_flush_volume(config, 2), WithinAbs(36.f, 1e-5f));
    CHECK_THAT(WipeTower2::estimate_semm_flush_volume(config, 0), WithinAbs(0.f, 1e-6f));
}

TEST_CASE("WipeTower2 gap walls are disabled for cone walls", "[WipeTower][WipeTowerExtended]")
{
    FullPrintConfig config;
    config.prime_tower_skip_points.value = true;
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwRectangle;
    CHECK(WipeTower2::use_gap_wall(config));
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwRib;
    CHECK(WipeTower2::use_gap_wall(config));
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwCone;
    CHECK_FALSE(WipeTower2::use_gap_wall(config));
    config.prime_tower_skip_points.value = false;
    CHECK_FALSE(WipeTower2::use_gap_wall(config));
}

TEST_CASE("WipeTower2 tower temperature wait is only enabled for multi-extruder prints", "[WipeTower][WipeTowerExtended]")
{
    FullPrintConfig config;
    config.wait_for_temp_on_wipe_tower.value = true;
    config.single_extruder_multi_material.value = false;
    CHECK(WipeTower2::wait_for_temp_enabled(config));
    config.single_extruder_multi_material.value = true;
    CHECK_FALSE(WipeTower2::wait_for_temp_enabled(config));
    config.wait_for_temp_on_wipe_tower.value = false;
    config.single_extruder_multi_material.value = false;
    CHECK_FALSE(WipeTower2::wait_for_temp_enabled(config));
}

TEST_CASE("WipeTower2 direct planning keeps layers ordered and records every transition", "[WipeTower][WipeTowerExtended]")
{
    const DirectRun run = run_direct_tower({
        {0.2f, 0.2f, 0, 1, 35.f},
        {0.4f, 0.2f, 1, 0, 20.f},
        {0.6f, 0.2f, 0, 1, 50.f},
    });
    require_finite_result(run);
    CHECK(run.planned_layers == 3);
    CHECK(run.active_layers == 3);
    CHECK(run.tool_changes == 3);
    CHECK_THAT(run.height, WithinAbs(0.6f, 1e-4f));
    REQUIRE(run.used_until_layer.size() >= 3);
    CHECK_THAT(run.used_until_layer[1].first, WithinAbs(0.2f, 1e-4f));
    CHECK_THAT(run.used_until_layer[2].first, WithinAbs(0.4f, 1e-4f));
    REQUIRE(run.z_depth.size() >= 2);
    CHECK(run.z_depth.front().second >= run.z_depth.back().second);
}

TEST_CASE("WipeTower2 reserves more depth for a larger requested purge", "[WipeTower][WipeTowerExtended]")
{
    const DirectRun small = run_direct_tower({{0.2f, 0.2f, 0, 1, 15.f}});
    const DirectRun large = run_direct_tower({{0.2f, 0.2f, 0, 1, 150.f}});
    require_finite_result(small);
    require_finite_result(large);
    CHECK(large.depth > small.depth);
}

TEST_CASE("WipeTower2 same-tool layers still produce a finite finishing layer", "[WipeTower][WipeTowerExtended]")
{
    const DirectRun run = run_direct_tower({
        {0.2f, 0.2f, 0, 0, 0.f},
        {0.4f, 0.2f, 0, 0, 0.f},
    });
    require_finite_result(run);
    CHECK(run.planned_layers == 2);
    CHECK(run.tool_changes == 0);
    REQUIRE(run.used_filament.size() >= 1);
    CHECK(run.used_filament.front() > 0.f);
}

TEST_CASE("WipeTower2 respects the configured width, layer height, and zero brim", "[WipeTower][WipeTowerExtended]")
{
    const DirectRun run = run_direct_tower({{0.2f, 0.2f, 0, 1, 60.f}});
    require_finite_result(run);
    CHECK_THAT(run.width, WithinAbs(35.f, 1e-4f));
    CHECK(run.depth > 0.f);
    CHECK_THAT(run.height, WithinAbs(0.2f, 1e-4f));
    CHECK_THAT(run.brim, WithinAbs(0.f, 1e-4f));
}

TEST_CASE("WipeTower2 used filament is cumulative by planned layer", "[WipeTower][WipeTowerExtended]")
{
    const DirectRun run = run_direct_tower({
        {0.2f, 0.2f, 0, 1, 35.f},
        {0.4f, 0.2f, 1, 0, 35.f},
    });
    require_finite_result(run);
    REQUIRE(run.used_until_layer.size() >= 3);
    REQUIRE(run.used_until_layer[0].second.size() >= 2);
    REQUIRE(run.used_until_layer[1].second.size() >= 2);
    REQUIRE(run.used_until_layer[2].second.size() >= 2);
    CHECK(run.used_until_layer[1].second[0] >= run.used_until_layer[0].second[0]);
    CHECK(run.used_until_layer[1].second[1] >= run.used_until_layer[0].second[1]);
    CHECK(run.used_until_layer[2].second[0] >= run.used_until_layer[1].second[0]);
    CHECK(run.used_until_layer[2].second[1] >= run.used_until_layer[1].second[1]);
    REQUIRE(run.used_filament.size() == 2);
    for (size_t tool = 0; tool < 2; ++tool) {
        CHECK_THAT(run.used_until_layer.front().second[tool], WithinAbs(0.f, 1e-6f));
        CHECK(run.used_filament[tool] > 0.f);
        CHECK_THAT(run.used_until_layer.back().second[tool], WithinAbs(run.used_filament[tool], 1e-5f));
    }
}

} // namespace extended_tower_contracts
