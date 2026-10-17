#include <array>
#include <sstream>
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include <initializer_list>
#include "libslic3r/MultiNozzleUtils.hpp"
#include <numeric>
#include <optional>
#include <tuple>
#include "libslic3r/GCode/WipeTower2.hpp"
#include <limits>
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/GCode/ToolOrdering.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Flow.hpp"
#include "fff_print/test_helpers.hpp"
#include <cmath>
#include <cctype>
#include <string_view>
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

using ObjectSegmentKey = std::array<long long, 9>;

struct SparsePipelineRun {
    std::size_t planned_layers = 0;
    std::size_t sparse_layers = 0;
    std::size_t transition_events = 0;
    std::size_t same_tool_blocks = 0;
    std::set<std::size_t> sparse_layer_indices;
    std::set<std::size_t> transition_layer_indices;
    std::size_t tower_extrusion_moves = 0;
    double tower_extrusion = 0.;
    std::string tower_gcode;
    std::vector<ObjectSegmentKey> object_segments;
};

static long long quantize_object_value(float value)
{
    return std::llround(double(value) * 10000.);
}

struct SparseGCodeEvidence {
    std::string tower_gcode;
    std::vector<ObjectSegmentKey> object_segments;
    std::size_t tower_extrusion_moves = 0;
    double tower_extrusion = 0.;
    bool invalid_motion = false;
    bool unknown_object_tool = false;
};

static bool is_wipe_tower_role_line(std::string_view raw)
{
    static const std::string type_tag = ";TYPE:" + ExtrusionEntity::role_to_string(erWipeTower);
    static const std::string feature_tag = "; FEATURE: " + ExtrusionEntity::role_to_string(erWipeTower);
    return raw.rfind(std::string_view(type_tag), 0) == 0 || raw.rfind(std::string_view(feature_tag), 0) == 0;
}

static bool is_any_role_line(std::string_view raw)
{
    return raw.rfind(";TYPE:", 0) == 0 || raw.rfind("; FEATURE:", 0) == 0;
}

// Parse the complete public stream once. WipeTower2's tool-change purge uses reserved markers,
// while finish_layer() uses the Prime tower role without those markers. Keeping both the modal
// role/marker state and the GCodeReader E/position state in this pass prevents finish-layer tower
// paths from leaking into the object oracle or being counted by a second parser with stale E state.
static SparseGCodeEvidence collect_sparse_gcode(const std::string &gcode,
                                                 const DynamicPrintConfig &config,
                                                 int initial_tool,
                                                 std::size_t known_tool_count)
{
    const std::string &start_tag = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_Start);
    const std::string &end_tag   = GCodeProcessor::reserved_tag(GCodeProcessor::ETags::Wipe_Tower_End);
    SparseGCodeEvidence evidence;
    GCodeReader reader;
    reader.apply_config(config);
    bool in_tower = false;
    int current_tool = initial_tool;
    reader.parse_buffer(gcode, [&evidence, &in_tower, &current_tool, &start_tag, &end_tag, known_tool_count](GCodeReader &self,
                                                                                                               const GCodeReader::GCodeLine &line) {
        const std::string_view command = line.cmd();
        if (command.size() >= 2 && command.front() == 'T' && std::isdigit(static_cast<unsigned char>(command[1]))) {
            try {
                current_tool = std::stoi(std::string(command.substr(1)));
            } catch (...) {
                current_tool = -1;
            }
        }
        const std::string raw = line.raw();
        const std::string_view raw_view(raw);
        if (is_wipe_tower_role_line(raw_view))
            in_tower = true;
        else if (is_any_role_line(raw_view))
            in_tower = false;
        if (raw_view.find(std::string_view(start_tag)) != std::string_view::npos)
            in_tower = true;
        const bool tower_line = in_tower;
        if (tower_line) {
            evidence.tower_gcode += raw;
            evidence.tower_gcode.push_back('\n');
        }
        if (raw_view.find(std::string_view(end_tag)) != std::string_view::npos)
            in_tower = false;

        // This fixture disables arc fitting; endpoint pairs are exact geometry only for lines.
        if (line.cmd_is("G2") || line.cmd_is("G3")) {
            evidence.invalid_motion = true;
            return;
        }
        if (!line.cmd_is("G1") || !line.has_e() || (!line.has_x() && !line.has_y()))
            return;
        const float x0 = self.x();
        const float y0 = self.y();
        const float z0 = self.z();
        const float x1 = line.new_X(self);
        const float y1 = line.new_Y(self);
        const float z1 = line.new_Z(self);
        const double dE = line.dist_E(self);
        if (!std::isfinite(double(x0)) || !std::isfinite(double(y0)) || !std::isfinite(double(z0)) ||
            !std::isfinite(double(x1)) || !std::isfinite(double(y1)) || !std::isfinite(double(z1)) ||
            !std::isfinite(dE)) {
            evidence.invalid_motion = true;
            return;
        }
        const double xy_length = std::hypot(double(x1 - x0), double(y1 - y0));
        if (dE <= 0. || xy_length <= 1e-6)
            return;
        if (tower_line) {
            ++evidence.tower_extrusion_moves;
            evidence.tower_extrusion += dE;
            return;
        }
        if (current_tool < 0 || static_cast<std::size_t>(current_tool) >= known_tool_count) {
            evidence.unknown_object_tool = true;
            return;
        }
        std::array<long long, 3> start = {quantize_object_value(x0), quantize_object_value(y0), quantize_object_value(z0)};
        std::array<long long, 3> end = {quantize_object_value(x1), quantize_object_value(y1), quantize_object_value(z1)};
        if (end < start)
            std::swap(start, end);
        evidence.object_segments.push_back({static_cast<long long>(current_tool), quantize_object_value(z1), start[0], start[1], start[2],
                                             end[0], end[1], end[2], quantize_object_value(float(dE))});
    });
    std::sort(evidence.object_segments.begin(), evidence.object_segments.end());
    return evidence;
}

// Bottom and top surfaces use filament 2. The middle layers use only filament 1, creating real
// lower and upper transitions with a sparse same-tool gap between them.
static DynamicPrintConfig sparse_pipeline_config(bool no_sparse_layers)
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "sparse_infill_filament_id",        1 },
        { "internal_solid_filament_id",       1 },
        { "top_surface_filament_id",          2 },
        { "bottom_surface_filament_id",       2 },
        { "outer_wall_filament_id",           1 },
        { "inner_wall_filament_id",           1 },
        { "enable_prime_tower",               true },
        { "wipe_tower_x",                     50 },
        { "wipe_tower_y",                     50 },
        { "prime_tower_width",                35 },
        { "wipe_tower_type",                  "type2" },
        { "prime_volume",                     80 },
        { "layer_height",                     0.2 },
        { "initial_layer_print_height",        0.2 },
        { "top_shell_layers",                 2 },
        { "bottom_shell_layers",              2 },
        { "top_shell_thickness",              0.0 },
        { "bottom_shell_thickness",           0.0 },
        { "top_surface_density",              "100%" },
        { "bottom_surface_density",           "100%" },
        { "sparse_infill_density",            "20%" },
        { "wall_loops",                       1 },
        { "skirt_loops",                      0 },
        { "brim_type",                        "no_brim" },
        { "raft_layers",                      0 },
        { "timelapse_type",                   "0" }, // registered tlTraditional
        { "enable_wrapping_detection",        false },
        { "wipe_tower_no_sparse_layers",      no_sparse_layers },
        { "wipe_tower_sparse_layers_combination", false },
        { "enable_arc_fitting",              false },
        { "use_relative_e_distances",         true },
        { "layer_change_gcode",               "G92 E0\n" },
    });
    return config;
}

static SparsePipelineRun run_sparse_pipeline(bool no_sparse_layers)
{
    const DynamicPrintConfig config = sparse_pipeline_config(no_sparse_layers);
    Print print;
    Model model;
    init_print({ make_cube(10., 10., 2.) }, print, model, config);
    // init_print's first normalization sees one role; reapply after regions know both filaments.
    print.apply(model, config);
    const auto validation = print.validate();
    INFO("validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE(print.is_step_done(psWipeTower));

    SparsePipelineRun result;
    std::size_t layer_index = 0;
    for (const auto &layer : print.wipe_tower_data().tool_changes) {
        if (layer.empty()) {
            ++layer_index;
            continue;
        }
        ++result.planned_layers;
        if (wipe_tower_layer_is_sparse(layer)) {
            ++result.sparse_layers;
            result.sparse_layer_indices.insert(layer_index);
        }
        for (const auto &change : layer) {
            if (change.combined_away)
                continue;
            if (change.initial_tool == change.new_tool)
                ++result.same_tool_blocks;
            else {
                ++result.transition_events;
                result.transition_layer_indices.insert(layer_index);
            }
        }
        ++layer_index;
    }
    const std::string output = gcode(print);
    const unsigned int first_extruder = print.tool_ordering().first_extruder();
    const int initial_tool = first_extruder == static_cast<unsigned int>(-1) ? -1 : static_cast<int>(first_extruder);
    const SparseGCodeEvidence evidence = collect_sparse_gcode(output, config, initial_tool, print.extruders().size());
    REQUIRE_FALSE(evidence.invalid_motion);
    REQUIRE_FALSE(evidence.unknown_object_tool);
    result.tower_gcode = evidence.tower_gcode;
    result.object_segments = evidence.object_segments;
    result.tower_extrusion_moves = evidence.tower_extrusion_moves;
    result.tower_extrusion = evidence.tower_extrusion;
    return result;
}

TEST_CASE("Skipping sparse wipe-tower layers preserves object output and reduces tower paths",
          "[WipeTower][WipeTowerSparsePipeline]")
{
    const SparsePipelineRun ordinary = run_sparse_pipeline(false);
    const SparsePipelineRun compacted = run_sparse_pipeline(true);

    REQUIRE(ordinary.planned_layers == compacted.planned_layers);
    REQUIRE(ordinary.sparse_layers == compacted.sparse_layers);
    REQUIRE(ordinary.sparse_layers > 0);
    REQUIRE(ordinary.transition_events > 0);
    REQUIRE(ordinary.transition_layer_indices.size() >= 2);
    REQUIRE(ordinary.same_tool_blocks > 0);
    const auto first_transition = *ordinary.transition_layer_indices.begin();
    const auto last_transition = *ordinary.transition_layer_indices.rbegin();
    CHECK(std::any_of(ordinary.sparse_layer_indices.begin(), ordinary.sparse_layer_indices.end(),
                      [first_transition, last_transition](std::size_t index) {
                          return first_transition < index && index < last_transition;
                      }));
    REQUIRE(ordinary.tower_extrusion_moves > 0);
    REQUIRE(compacted.tower_extrusion_moves > 0);
    CHECK(compacted.tower_extrusion_moves < ordinary.tower_extrusion_moves);
    CHECK(compacted.tower_extrusion < ordinary.tower_extrusion);
    CHECK_FALSE(ordinary.tower_gcode.empty());
    CHECK_FALSE(compacted.tower_gcode.empty());
    REQUIRE(ordinary.object_segments.size() == compacted.object_segments.size());
    REQUIRE_FALSE(ordinary.object_segments.empty());
    CHECK(compacted.object_segments == ordinary.object_segments);
}

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

TEST_CASE("WipeTower2 gap walls are disabled for cone walls", "[WipeTower][WipeTowerExtended][WipeTowerGapWall]")
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
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwRectangle;
    CHECK_FALSE(WipeTower2::use_gap_wall(config));
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwRib;
    CHECK_FALSE(WipeTower2::use_gap_wall(config));
    config.wipe_tower_wall_type.value = WipeTowerWallType::wtwCone;
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

namespace legacy_tower_direct_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

FullPrintConfig legacy_config(bool semm = false,
                              WipeTowerWallType wall = WipeTowerWallType::wtwRectangle,
                              bool sparse = false,
                              bool arc_fitting = false)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", semm},
        {"enable_prime_tower", true},
        {"prime_tower_width", 35},
        {"prime_tower_infill_gap", 150},
        {"nozzle_diameter", "0.4"},
        {"filament_diameter", "1.75,1.75"},
        {"prime_tower_brim_width", 0},
        {"wipe_tower_x", 20},
        {"wipe_tower_y", 20},
        {"wipe_tower_wall_type", wall == WipeTowerWallType::wtwRib ? "rib" : "rectangle"},
        {"prime_tower_skip_points", false},
        {"wipe_tower_no_sparse_layers", !sparse},
        {"wipe_tower_extra_flow", 100},
        {"wipe_tower_extra_spacing", 100},
        {"enable_arc_fitting", arc_fitting},
        {"filament_prime_volume", "80,80"},
        {"filament_prime_volume_nc", "80,80"},
        {"filament_change_length", "10,10"},
        {"filament_change_length_nc", "30,30"},
        {"flush_volumes_matrix", "0,80,80,0"},
        {"filament_adhesiveness_category", "0,1"},
    });
    FullPrintConfig full;
    full.apply(config);
    return full;
}

struct DirectRun {
    std::vector<std::vector<WipeTower::ToolChangeResult>> layers;
    float depth = 0.f;
    float width = 0.f;
    float height = 0.f;
    float brim = 0.f;
    int toolchanges = 0;
};

std::optional<MultiNozzleUtils::LayeredNozzleGroupResult> group_for_two_filaments(bool same_extruder)
{
    std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
    nozzles.push_back({"0.4", nvtStandard, 0, 0});
    nozzles.push_back({"0.4", nvtStandard, same_extruder ? 0 : 1, 1});
    return MultiNozzleUtils::LayeredNozzleGroupResult::create(
        std::vector<int>{0, 1}, nozzles, std::vector<unsigned int>{0, 1});
}

DirectRun run_direct(const std::vector<std::tuple<float, float, unsigned int, unsigned int, float>> &plan,
                     bool semm = false,
                     WipeTowerWallType wall = WipeTowerWallType::wtwRectangle,
                     bool sparse = false,
                     bool arc_fitting = false,
                     bool same_extruder = false)
{
    FullPrintConfig config = legacy_config(semm, wall, sparse, arc_fitting);
    auto group = group_for_two_filaments(same_extruder);
    REQUIRE(group.has_value());
    WipeTower tower(config, 0, Vec3d::Zero(), 0, 8.f,
                    std::vector<unsigned int>{0, 1});
    tower.set_nozzle_group_result(*group);
    tower.set_used_filament_ids({0, 1});
    tower.set_filament_categories({0, 1});
    tower.set_extruder(0, config);
    tower.set_extruder(1, config);
    for (const auto &[z, h, old_tool, new_tool, purge] : plan)
        tower.plan_toolchange(z, h, old_tool, new_tool, purge, purge, purge);

    DirectRun run;
    tower.generate_new(run.layers);
    run.depth = tower.get_depth();
    run.width = tower.width();
    run.height = tower.get_height();
    run.brim = tower.get_brim_width();
    run.toolchanges = tower.get_number_of_toolchanges();
    return run;
}

float extruded_length(WipeTower::ToolChangeResult tcr)
{
    return tcr.total_extrusion_length_in_plane();
}

std::size_t nonempty_layers(const DirectRun &run)
{
    return static_cast<std::size_t>(std::count_if(run.layers.begin(), run.layers.end(),
        [](const auto &layer) { return !layer.empty(); }));
}

void require_finite_run(const DirectRun &run)
{
    REQUIRE_FALSE(run.layers.empty());
    REQUIRE(run.width > 0.f);
    REQUIRE(run.depth > 0.f);
    CHECK(run.height > 0.f);
    for (const auto &layer : run.layers) {
        for (const auto &tcr : layer) {
            CHECK(std::isfinite(double(tcr.print_z)));
            CHECK(std::isfinite(double(tcr.layer_height)));
            CHECK(std::isfinite(double(tcr.elapsed_time)));
            CHECK(std::isfinite(double(tcr.purge_volume)));
            CHECK(tcr.layer_height > 0.f);
            CHECK(tcr.elapsed_time >= 0.f);
            CHECK(extruded_length(tcr) > 0.f);
            CHECK(std::isfinite(double(tcr.start_pos.x())));
            CHECK(std::isfinite(double(tcr.start_pos.y())));
            CHECK(std::isfinite(double(tcr.end_pos.x())));
            CHECK(std::isfinite(double(tcr.end_pos.y())));
        }
    }
}

} // namespace

TEST_CASE("Legacy WipeTower emits one planned result for every requested layer", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 0, 80.f},
        {0.6f, 0.2f, 0, 1, 80.f},
    });
    require_finite_run(run);
    REQUIRE(run.layers.size() == 3);
    CHECK(nonempty_layers(run) == 3);
    CHECK(run.toolchanges == 3);
    REQUIRE_FALSE(run.layers[0].empty());
    REQUIRE_FALSE(run.layers[1].empty());
    REQUIRE_FALSE(run.layers[2].empty());
    CHECK(run.layers[0].front().initial_tool == 0);
    CHECK(run.layers[0].front().new_tool == 1);
    CHECK(run.layers[1].front().initial_tool == 1);
    CHECK(run.layers[1].front().new_tool == 0);
    CHECK(run.layers[2].front().initial_tool == 0);
    CHECK(run.layers[2].front().new_tool == 1);
}

TEST_CASE("Legacy WipeTower records the configured purge volume on each transition", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun run = run_direct({{0.2f, 0.2f, 0, 1, 55.f}});
    require_finite_run(run);
    REQUIRE_FALSE(run.layers.front().empty());
    CHECK_THAT(run.layers.front().front().purge_volume, WithinAbs(55.f, 1e-4f));
}

TEST_CASE("Legacy WipeTower does not create a tool change for an unchanged tool", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 0, 80.f},
        {0.4f, 0.2f, 0, 0, 80.f},
    });
    CHECK(run.layers.empty());
    CHECK_THAT(run.depth, WithinAbs(0.f, 1e-5f));
    CHECK(run.toolchanges == 0);
    CHECK(nonempty_layers(run) == 0);
}

TEST_CASE("Legacy WipeTower preserves layer heights in generated results", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun run = run_direct({
        {0.25f, 0.25f, 0, 1, 80.f},
        {0.55f, 0.30f, 1, 0, 80.f},
    });
    require_finite_run(run);
    REQUIRE(run.layers.size() == 2);
    REQUIRE_FALSE(run.layers[0].empty());
    REQUIRE_FALSE(run.layers[1].empty());
    CHECK_THAT(run.layers[0].front().layer_height, WithinAbs(0.25f, 1e-5f));
    CHECK_THAT(run.layers[1].front().layer_height, WithinAbs(0.30f, 1e-5f));
    CHECK_THAT(run.layers[0].front().print_z, WithinAbs(0.25f, 1e-5f));
    CHECK_THAT(run.layers[1].front().print_z, WithinAbs(0.55f, 1e-5f));
    CHECK_THAT(run.height, WithinAbs(0.55f, 1e-5f));
}

TEST_CASE("Legacy WipeTower geometry responds to purge demand", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun low = run_direct({{0.2f, 0.2f, 0, 1, 20.f}});
    const DirectRun high = run_direct({{0.2f, 0.2f, 0, 1, 160.f}});
    require_finite_run(low);
    require_finite_run(high);
    CHECK(high.depth >= low.depth);
    // Oval extrusion area is a rectangle plus two semicircular ends. At
    // 0.2mm height and 0.5mm width, 20/160mm3 require 7/52 whole purge lines.
    // Ramming 10mm of 1.75mm filament requires four wider (1mm) lines.
    // The explicit 150% infill gap adds 0.25mm to both line spacings.
    const double pi = std::acos(-1.0);
    const auto oval_area = [pi](double width) {
        return 0.2 * (width - 0.2) + pi * 0.1 * 0.1;
    };
    const double ramming_lines = std::ceil(10.0 * pi * 1.75 * 1.75 / 4.0 /
                                           oval_area(1.0) / (35.0 - 1.0 - 0.5));
    const auto expected_depth = [&](double volume) {
        const double purge_lines = std::ceil(volume / oval_area(0.5) / (35.0 - 1.0));
        return 0.5 + purge_lines * 0.75 + ramming_lines * 1.25;
    };
    CHECK_THAT(low.depth, WithinAbs(expected_depth(20.0), 1e-3));
    CHECK_THAT(high.depth, WithinAbs(expected_depth(160.0), 1e-3));

}

TEST_CASE("Legacy WipeTower emits a rectangle footprint for rectangle walls", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun run = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false,
                                     WipeTowerWallType::wtwRectangle);
    require_finite_run(run);
    CHECK_THAT(run.width, WithinAbs(35.f, 1e-4f));
    CHECK(run.depth > 0.f);
    CHECK_THAT(run.brim, WithinAbs(0.f, 1e-5f));
}

TEST_CASE("Legacy WipeTower rib wall remains finite and changes the footprint contract", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun rectangle = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false,
                                           WipeTowerWallType::wtwRectangle);
    const DirectRun rib = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false,
                                     WipeTowerWallType::wtwRib);
    require_finite_run(rectangle);
    require_finite_run(rib);
    CHECK(rib.width < rectangle.width);
    CHECK(rib.depth >= rectangle.depth);
}

TEST_CASE("Legacy WipeTower separates same-extruder nozzle changes from extruder changes", "[WipeTower][WipeTowerLegacyExtended]")
{
    const DirectRun extruder = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false,
                                          WipeTowerWallType::wtwRectangle, false, false, false);
    const DirectRun nozzle = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false,
                                        WipeTowerWallType::wtwRectangle, false, false, true);
    require_finite_run(extruder);
    require_finite_run(nozzle);
    REQUIRE_FALSE(extruder.layers.front().empty());
    REQUIRE_FALSE(nozzle.layers.front().empty());
    CHECK(extruder.layers.front().front().is_tool_change);
    CHECK(nozzle.layers.front().front().is_tool_change);
    // Changing 30mm rather than 10mm of filament requires twelve rather
    // than four ramming lines, each spaced at the configured 1.25mm.
    CHECK_THAT(nozzle.depth - extruder.depth, WithinAbs(8.f * 1.25f, 1e-4f));
}


namespace {

struct PrimeTowerSettingRun {
    std::vector<std::vector<WipeTower::ToolChangeResult>> layers;
    std::vector<double> layer_extrusion;
    std::vector<float> layer_positive_y_span;
    std::string gcode;
    std::size_t travel_moves = 0;
    std::size_t extrusion_moves = 0;
    float depth = 0.f;
    float height = 0.f;
};

FullPrintConfig prime_tower_setting_config(bool framework, bool flat_ironing, bool gap_wall,
                                           float ironing_area = 16.f)
{
    FullPrintConfig config = legacy_config();
    config.prime_tower_enable_framework.value = framework;
    config.prime_tower_flat_ironing.value = flat_ironing;
    config.prime_tower_skip_points.value = gap_wall;
    config.filament_tower_ironing_area.values.assign(2, ironing_area);
    config.filament_adhesiveness_category.values.assign(2, 0);
    return config;
}

PrimeTowerSettingRun run_prime_tower_settings(
    const std::vector<std::tuple<float, float, unsigned int, unsigned int, float>> &plan,
    bool framework, bool flat_ironing, bool gap_wall, float ironing_area = 16.f)
{
    const FullPrintConfig config = prime_tower_setting_config(framework, flat_ironing, gap_wall, ironing_area);
    const auto group = group_for_two_filaments(false);
    REQUIRE(group.has_value());

    WipeTower tower(config, 0, Vec3d::Zero(), 0, 8.f, {0, 1});
    tower.set_nozzle_group_result(*group);
    tower.set_used_filament_ids({0, 1});
    tower.set_filament_categories({0, 0});
    tower.set_extruder(0, config);
    tower.set_extruder(1, config);
    for (const auto &[z, height, old_tool, new_tool, purge] : plan)
        tower.plan_toolchange(z, height, old_tool, new_tool, purge, purge, purge);

    PrimeTowerSettingRun result;
    tower.generate_new(result.layers);
    result.depth = tower.get_depth();
    result.height = tower.get_height();
    result.layer_extrusion.reserve(result.layers.size());
    result.layer_positive_y_span.reserve(result.layers.size());
    for (const auto &layer : result.layers) {
        double length = 0.;
        float min_y = std::numeric_limits<float>::infinity();
        float max_y = -std::numeric_limits<float>::infinity();
        for (const auto &change : layer) {
            result.gcode += change.gcode;
            for (size_t i = 1; i < change.extrusions.size(); ++i) {
                if (change.extrusions[i].width > 0.f) {
                    length += (change.extrusions[i].pos - change.extrusions[i - 1].pos).norm();
                    min_y = std::min({min_y, change.extrusions[i - 1].pos.y(), change.extrusions[i].pos.y()});
                    max_y = std::max({max_y, change.extrusions[i - 1].pos.y(), change.extrusions[i].pos.y()});
                }
            }
        }
        result.layer_extrusion.push_back(length);
        result.layer_positive_y_span.push_back(max_y - min_y);
    }

    GCodeReader parser;
    parser.parse_buffer(result.gcode, [&result](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("G1") || (!line.has_x() && !line.has_y()))
            return;
        if (line.has_e())
            ++result.extrusion_moves;
        else
            ++result.travel_moves;
    });
    return result;
}

void require_prime_tower_run(const PrimeTowerSettingRun &run, std::size_t expected_layers)
{
    REQUIRE(run.layers.size() == expected_layers);
    REQUIRE(run.layer_extrusion.size() == expected_layers);
    REQUIRE(run.layer_positive_y_span.size() == expected_layers);
    REQUIRE_FALSE(run.gcode.empty());
    REQUIRE(run.travel_moves > 0);
    REQUIRE(run.extrusion_moves > 0);
    CHECK(std::isfinite(double(run.depth)));
    CHECK(std::isfinite(double(run.height)));
    CHECK(run.depth > 0.f);
    CHECK(run.height > 0.f);
    for (const auto &layer : run.layer_extrusion) {
        CHECK(std::isfinite(layer));
        CHECK(layer > 0.);
    }
    for (const float span : run.layer_positive_y_span) {
        CHECK(std::isfinite(double(span)));
        CHECK(span > 0.f);
    }
}

} // namespace

TEST_CASE("Prime tower framework keeps later direct layers at the first purge depth", "[WipeTower][PrimeTowerFramework]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 160.f},
        {0.4f, 0.2f, 1, 0, 20.f},
        {0.6f, 0.2f, 0, 1, 20.f},
    };
    const PrimeTowerSettingRun disabled = run_prime_tower_settings(plan, false, false, false);
    const PrimeTowerSettingRun enabled = run_prime_tower_settings(plan, true, false, false);
    require_prime_tower_run(disabled, 3);
    require_prime_tower_run(enabled, 3);

    CHECK_THAT(enabled.depth, WithinAbs(disabled.depth, 1e-4f));
    CHECK_THAT(enabled.height, WithinAbs(disabled.height, 1e-4f));
    CHECK(enabled.layer_extrusion[1] > disabled.layer_extrusion[1] + 0.5);
    CHECK(enabled.layer_extrusion[2] > disabled.layer_extrusion[2] + 0.5);
    CHECK(enabled.layer_positive_y_span[1] > disabled.layer_positive_y_span[1] + 0.5f);
    CHECK(enabled.layer_positive_y_span[2] > disabled.layer_positive_y_span[2] + 0.5f);
    CHECK_THAT(enabled.layer_positive_y_span[1], WithinAbs(enabled.layer_positive_y_span[0], 0.5f));
    CHECK_THAT(enabled.layer_positive_y_span[2], WithinAbs(enabled.layer_positive_y_span[0], 0.5f));
}

TEST_CASE("Prime tower flat ironing adds area dependent travel after the gap wall", "[WipeTower][PrimeTowerFlatIroning]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f},
    };
    const PrimeTowerSettingRun disabled = run_prime_tower_settings(plan, false, false, true);
    const PrimeTowerSettingRun small_area = run_prime_tower_settings(plan, false, true, true, 4.f);
    const PrimeTowerSettingRun large_area = run_prime_tower_settings(plan, false, true, true, 16.f);
    require_prime_tower_run(disabled, 1);
    require_prime_tower_run(small_area, 1);
    require_prime_tower_run(large_area, 1);

    CHECK(small_area.travel_moves > disabled.travel_moves);
    CHECK(large_area.travel_moves > small_area.travel_moves);
    CHECK(small_area.extrusion_moves == disabled.extrusion_moves);
    CHECK(large_area.extrusion_moves == disabled.extrusion_moves);
    CHECK(small_area.layer_extrusion.front() > 0.);
    CHECK(large_area.layer_extrusion.front() > 0.);
}

} // namespace legacy_tower_direct_contracts

namespace multimaterial_wipe_variance_contracts {
using namespace Slic3r;
using namespace Slic3r::Test;
using Catch::Matchers::WithinAbs;

namespace {

DynamicPrintConfig tower_dynamic(std::initializer_list<ConfigBase::SetDeserializeItem> extra = {})
{
    DynamicPrintConfig config = multifilament_config(2, {
        {"single_extruder_multi_material", true},
        {"purge_in_prime_tower", true},
        {"enable_prime_tower", true},
        {"prime_tower_width", 35},
        {"prime_tower_brim_width", 0},
        {"wipe_tower_x", 20},
        {"wipe_tower_y", 20},
        {"wipe_tower_wall_type", "rectangle"},
        {"prime_tower_skip_points", false},
        {"wipe_tower_no_sparse_layers", false},
        {"wipe_tower_extra_flow", 100},
        {"wipe_tower_extra_spacing", 100},
        {"wipe_tower_bridging", 10},
        {"wipe_tower_max_purge_speed", 90},
        {"enable_arc_fitting", false},
        {"filament_ramming_parameters", "100 100 20 20 20"},
        {"filament_cooling_moves", "0,0"},
        {"filament_diameter", "1.75,1.75"},
    });
    if (!extra.size())
        return config;
    config.set_deserialize_strict(extra);
    return config;
}

struct DirectRun {
    std::vector<std::vector<WipeTower::ToolChangeResult>> layers;
    float depth = 0.f;
    float height = 0.f;
    float width = 0.f;
    float brim = 0.f;
    float rib_width = 0.f;
    float rib_length = 0.f;
    BoundingBoxf box;
    double elapsed = 0.;
    double extrusion_length = 0.;
    size_t extrusion_count = 0;
    std::string gcode;
};

DirectRun run_tower(const DynamicPrintConfig &dynamic,
                    const std::vector<std::tuple<float, float, unsigned int, unsigned int, float>> &plan)
{
    FullPrintConfig full;
    full.apply(dynamic);
    const std::vector<std::vector<float>> wipe_matrix{{0.f, 120.f}, {120.f, 0.f}};
    WipeTower2 tower(full, static_cast<const PrintRegionConfig &>(full), 0, Vec3d::Zero(), wipe_matrix, 0);
    tower.set_extruder(0, full);
    tower.set_extruder(1, full);
    for (const auto &[z, h, old_tool, new_tool, purge] : plan)
        tower.plan_toolchange(z, h, old_tool, new_tool, purge);

    DirectRun result;
    tower.generate(result.layers);
    result.depth = tower.get_depth();
    result.height = tower.get_wipe_tower_height();
    result.width = tower.width();
    result.brim = tower.get_brim_width();
    result.rib_width = tower.get_rib_width();
    result.rib_length = tower.get_rib_length();
    result.box = tower.get_bbx();
    for (const auto &layer : result.layers)
        for (const auto &change : layer) {
            result.elapsed += change.elapsed_time;
            result.gcode += change.gcode;
            for (size_t i = 1; i < change.extrusions.size(); ++i) {
                if (change.extrusions[i].width > 0.f) {
                    result.extrusion_length += (change.extrusions[i].pos - change.extrusions[i - 1].pos).norm();
                    ++result.extrusion_count;
                }
            }
        }
    return result;
}

void require_finite(const DirectRun &run)
{
    REQUIRE(run.depth > 0.f);
    REQUIRE(run.height > 0.f);
    REQUIRE(run.extrusion_count > 0);
    CHECK(std::isfinite(double(run.depth)));
    CHECK(std::isfinite(double(run.elapsed)));
    CHECK(std::isfinite(run.box.min.x()));
    CHECK(std::isfinite(run.box.max.y()));
    CHECK_FALSE(run.gcode.empty());
}

std::vector<std::vector<float>> folded_flush_config(const DynamicPrintConfig &dynamic)
{
    return WipeTower2::extract_wipe_volumes(dynamic);
}

} // namespace

namespace {
std::set<int> feedrates(const std::string &gcode);
}

TEST_CASE("SEMM flush matrix folds every nozzle block and keeps destination purge floors", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(3);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", true},
        {"purge_in_prime_tower", true},
        {"nozzle_diameter", "0.4,0.8"},
        {"flush_volumes_matrix", "0,10,11,12,0,13,14,15,0,0,41,42,43,0,44,45,46,0"},
        {"flush_multiplier", "1,2"},
        {"filament_minimal_purge_on_wipe_tower", "3,7,17"},
    });

    const auto folded = folded_flush_config(config);
    REQUIRE(folded.size() == 3);
    REQUIRE(folded[0].size() == 3);
    // The second nozzle block is scaled by 2 and dominates each non-diagonal pair.
    CHECK_THAT(folded[0][1], WithinAbs(82.f, 1e-5f));
    CHECK_THAT(folded[1][0], WithinAbs(86.f, 1e-5f));
    CHECK_THAT(folded[1][2], WithinAbs(88.f, 1e-5f));
    CHECK_THAT(folded[2][1], WithinAbs(92.f, 1e-5f));
    // A destination floor applies even where the matrix entry is zero.
    CHECK_THAT(folded[0][0], WithinAbs(3.f, 1e-5f));
    CHECK_THAT(folded[0][2], WithinAbs(84.f, 1e-5f));
    CHECK_THAT(folded[2][2], WithinAbs(17.f, 1e-5f));

    config.set_deserialize_strict({{"purge_in_prime_tower", false}});
    const auto minimum_only = folded_flush_config(config);
    CHECK_THAT(minimum_only[0][1], WithinAbs(7.f, 1e-5f));
    CHECK_THAT(minimum_only[2][0], WithinAbs(3.f, 1e-5f));
}

TEST_CASE("WipeTower2 purge flow and spacing alter emitted tower geometry", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f}, {0.4f, 0.2f, 1, 0, 80.f}};
    const DirectRun baseline = run_tower(tower_dynamic(), plan);
    const DirectRun spaced = run_tower(tower_dynamic({{"wipe_tower_extra_spacing", 200}}), plan);
    const DirectRun high_flow = run_tower(tower_dynamic({{"wipe_tower_extra_flow", 200}}), plan);
    require_finite(baseline);
    require_finite(spaced);
    require_finite(high_flow);
    CHECK(spaced.depth > baseline.depth);
    CHECK(high_flow.extrusion_count != baseline.extrusion_count);
    CHECK(std::abs(high_flow.extrusion_length - baseline.extrusion_length) > 1e-6);
    CHECK(high_flow.gcode.find("G1") != std::string::npos);
}

TEST_CASE("WipeTower2 wall geometry honors brim, rib, fillet, cone and width settings", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 70.f}};
    const DirectRun rectangle = run_tower(tower_dynamic({
        {"prime_tower_brim_width", 0}, {"prime_tower_width", 32}, {"wipe_tower_wall_type", "rectangle"}}), plan);
    const DirectRun brim = run_tower(tower_dynamic({
        {"prime_tower_brim_width", 4}, {"prime_tower_width", 32}, {"wipe_tower_wall_type", "rectangle"}}), plan);
    const DirectRun rib = run_tower(tower_dynamic({
        {"prime_tower_width", 32}, {"wipe_tower_wall_type", "rib"},
        {"wipe_tower_rib_width", 6}, {"wipe_tower_extra_rib_length", 8}, {"wipe_tower_fillet_wall", true}}), plan);
    const DirectRun rib_plain = run_tower(tower_dynamic({
        {"prime_tower_width", 32}, {"wipe_tower_wall_type", "rib"},
        {"wipe_tower_rib_width", 6}, {"wipe_tower_extra_rib_length", 0}, {"wipe_tower_fillet_wall", false}}), plan);
    const auto tall_plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 70.f}, {10.f, 0.2f, 1, 0, 70.f}};
    const DirectRun cone0 = run_tower(tower_dynamic({
        {"prime_tower_width", 32}, {"wipe_tower_wall_type", "cone"}, {"wipe_tower_cone_angle", 0}}), tall_plan);
    const DirectRun cone90 = run_tower(tower_dynamic({
        {"prime_tower_width", 32}, {"wipe_tower_wall_type", "cone"}, {"wipe_tower_cone_angle", 90}}), tall_plan);
    require_finite(rectangle);
    require_finite(brim);
    require_finite(rib);
    require_finite(rib_plain);
    require_finite(cone0);
    require_finite(cone90);
    CHECK_THAT(rectangle.width, WithinAbs(32.f, 1e-4f));
    CHECK(brim.brim > rectangle.brim);
    CHECK(rib.rib_width > 0.f);
    CHECK(rib.rib_length > rib_plain.rib_length);
    CHECK(std::abs(rib.extrusion_length - rib_plain.extrusion_length) > 1e-6);
    CHECK(cone90.box.size().y() > cone0.box.size().y());
}

TEST_CASE("WipeTower2 ramming gate and cooling settings change unload geometry", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 60.f}, {0.4f, 0.2f, 1, 0, 60.f}};
    const DynamicPrintConfig enabled = tower_dynamic({
        {"enable_filament_ramming", true},
        {"filament_ramming_parameters", "100 100 30 30 30 30"},
        {"filament_cooling_moves", "2,2"},
        {"filament_cooling_initial_speed", "2,2"},
        {"filament_cooling_final_speed", "6,6"},
        {"filament_stamping_distance", "2,2"},
        {"filament_stamping_loading_speed", "20,20"},
        {"cooling_tube_length", 40},
        {"cooling_tube_retraction", 10},
        {"filament_toolchange_delay", "1,1"},
    });
    const DynamicPrintConfig disabled = tower_dynamic({
        {"enable_filament_ramming", false},
        {"filament_cooling_moves", "0,0"},
    });
    const DirectRun rammed = run_tower(enabled, plan);
    const DirectRun plain = run_tower(disabled, plan);
    require_finite(rammed);
    require_finite(plain);
    CHECK(rammed.gcode.find("; Ramming start") != std::string::npos);
    CHECK(rammed.gcode.find("; Cooling") != std::string::npos);
    CHECK(plain.gcode.find("; Ramming start") == std::string::npos);
    CHECK(rammed.extrusion_length > plain.extrusion_length);
    CHECK(rammed.elapsed > plain.elapsed);
}

TEST_CASE("WipeTower2 multi-tool ramming uses volume and flow as a finite purge move", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 50.f}};
    const DirectRun low = run_tower(tower_dynamic({
        {"single_extruder_multi_material", false},
        {"filament_multitool_ramming", true},
        {"filament_multitool_ramming_volume", "4,4"},
        {"filament_multitool_ramming_flow", "2,2"},
    }), plan);
    const DirectRun high = run_tower(tower_dynamic({
        {"single_extruder_multi_material", false},
        {"filament_multitool_ramming", true},
        {"filament_multitool_ramming_volume", "16,16"},
        {"filament_multitool_ramming_flow", "8,8"},
    }), plan);
    require_finite(low);
    require_finite(high);
    CHECK(high.extrusion_length > low.extrusion_length);
    CHECK(high.elapsed >= low.elapsed);
}

TEST_CASE("SEMM optional loading moves use parking and configured load rates", "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 50.f}};
    const DynamicPrintConfig plain_config = tower_dynamic({
        {"enable_filament_ramming", true},
        {"filament_ramming_parameters", "100 100 30 30 30 30"},
        {"filament_cooling_moves", "0,0"},
        {"cooling_tube_length", 40},
        {"cooling_tube_retraction", 10},
        {"parking_pos_retraction", 0},
        {"extra_loading_move", 0},
        {"high_current_on_filament_swap", false},
    });
    const DynamicPrintConfig loaded_config = tower_dynamic({
        {"enable_filament_ramming", true},
        {"filament_ramming_parameters", "100 100 30 30 30 30"},
        {"filament_cooling_moves", "0,0"},
        {"cooling_tube_length", 40},
        {"cooling_tube_retraction", 10},
        {"parking_pos_retraction", 4},
        {"extra_loading_move", 8},
        {"filament_loading_speed", "12,12"},
        {"filament_loading_speed_start", "12,12"},
        {"filament_unloading_speed", "35,35"},
        {"filament_unloading_speed_start", "55,55"},
        {"high_current_on_filament_swap", true},
    });
    const DirectRun plain = run_tower(plain_config, plan);
    const DirectRun loaded = run_tower(loaded_config, plan);
    require_finite(plain);
    require_finite(loaded);
    CHECK(plain.gcode.find("; CP TOOLCHANGE LOAD") == std::string::npos);
    CHECK(loaded.gcode.find("; CP TOOLCHANGE LOAD") != std::string::npos);
    CHECK(feedrates(loaded.gcode).count(2297) == 1);
    CHECK(feedrates(loaded.gcode).count(720) == 1);
    CHECK(feedrates(loaded.gcode).count(2100) == 1);
    CHECK(feedrates(loaded.gcode).count(3300) == 1);
    CHECK(feedrates(loaded.gcode).count(1608) == 1);
    CHECK(loaded.gcode.find("M907 E550") != std::string::npos);
    CHECK(loaded.gcode != plain.gcode);
}

namespace {

std::string command_section(const std::string &gcode, const std::string &begin, const std::string &end)
{
    const size_t first = gcode.find(begin);
    if (first == std::string::npos)
        return {};
    const size_t start = first + begin.size();
    const size_t last = gcode.find(end, start);
    const std::string body = gcode.substr(start, last == std::string::npos ? std::string::npos : last - start);
    std::string result;
    size_t line_start = 0;
    while (line_start < body.size()) {
        size_t line_end = body.find('\n', line_start);
        if (line_end == std::string::npos)
            line_end = body.size();
        const std::string line = body.substr(line_start, line_end - line_start);
        if (!line.empty() && line.front() != ';') {
            result += line;
            result.push_back('\n');
        }
        line_start = line_end + (line_end < body.size() ? 1 : 0);
    }
    return result;
}

std::set<int> feedrates(const std::string &gcode)
{
    std::set<int> result;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&result](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (line.has_f())
            result.insert(static_cast<int>(std::lround(line.f())));
    });
    return result;
}

std::string cooling_motion(const DirectRun &run)
{
    return command_section(run.gcode, "; Cooling\n", "; Cooling park\n");
}

std::string load_motion(const DirectRun &run)
{
    return command_section(run.gcode, "; CP TOOLCHANGE LOAD\n", "; CP TOOLCHANGE WIPE\n");
}

std::string unload_motion(const DirectRun &run)
{
    return command_section(run.gcode, "; CP TOOLCHANGE UNLOAD\n", "; CP TOOLCHANGE CHANGE\n");
}

std::string cooling_park_motion(const DirectRun &run)
{
    return command_section(run.gcode, "; Cooling park\n", "; Ramming end\n");
}

}

TEST_CASE("WipeTower2 cooling moves, rates and stamping settings alter emitted cooling motions",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 60.f}, {0.4f, 0.2f, 1, 0, 60.f}};
    const DynamicPrintConfig active = tower_dynamic({
        {"enable_filament_ramming", true},
        {"filament_ramming_parameters", "100 100 30 30 30 30"},
        {"filament_cooling_moves", "2,2"},
        {"filament_cooling_initial_speed", "2,2"},
        {"filament_cooling_final_speed", "6,6"},
        {"filament_stamping_distance", "2,2"},
        {"filament_stamping_loading_speed", "20,20"},
        {"cooling_tube_length", 40},
        {"cooling_tube_retraction", 10},
        {"filament_toolchange_delay", "1,1"},
    });
    const auto variant = [&](const char *key, const char *value) {
        DynamicPrintConfig changed = active;
        changed.set_deserialize_strict({{key, value}});
        return run_tower(changed, plan);
    };
    const DirectRun enabled = run_tower(active, plan);
    const DirectRun no_moves = variant("filament_cooling_moves", "0,0");
    const DirectRun initial_variant = variant("filament_cooling_initial_speed", "4,4");
    const DirectRun final_variant = variant("filament_cooling_final_speed", "8,8");
    const DirectRun no_stamp = variant("filament_stamping_distance", "0,0");
    const DirectRun faster_stamp = variant("filament_stamping_loading_speed", "50,50");
    require_finite(enabled);
    REQUIRE_FALSE(cooling_motion(enabled).empty());
    CHECK(cooling_motion(no_moves).empty());
    CHECK(cooling_motion(enabled) != cooling_motion(initial_variant));
    CHECK(cooling_motion(enabled) != cooling_motion(final_variant));
    CHECK(cooling_motion(enabled) != cooling_motion(no_stamp));
    CHECK(cooling_motion(enabled) != cooling_motion(faster_stamp));
    CHECK(feedrates(cooling_motion(enabled)).count(80) == 1);
    CHECK(feedrates(cooling_motion(enabled)).count(240) == 1);
}

TEST_CASE("WipeTower2 multi-tool flow is observed with fixed purge volume",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 50.f}};
    const auto multi = [&](float flow) {
        return run_tower(tower_dynamic({
            {"single_extruder_multi_material", false},
            {"filament_multitool_ramming", true},
            {"filament_multitool_ramming_volume", "16,16"},
            {"filament_multitool_ramming_flow", std::to_string(flow) + "," + std::to_string(flow)},
        }), plan);
    };
    const DirectRun slow = multi(2.f);
    const DirectRun fast = multi(8.f);
    require_finite(slow);
    require_finite(fast);
    CHECK(unload_motion(slow) != unload_motion(fast));
    CHECK(std::abs(slow.elapsed - fast.elapsed) > 1e-6);
}

TEST_CASE("WipeTower2 cooling delay appears as an independent wait command",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 60.f}};
    const auto with_delay = [&](float delay) {
        return run_tower(tower_dynamic({
            {"enable_filament_ramming", true},
            {"filament_ramming_parameters", "100 100 30 30 30 30"},
            {"filament_cooling_moves", "2,2"},
            {"filament_cooling_initial_speed", "2,2"},
            {"filament_cooling_final_speed", "6,6"},
            {"cooling_tube_length", 40},
            {"cooling_tube_retraction", 10},
            {"filament_toolchange_delay", std::to_string(delay) + "," + std::to_string(delay)},
        }), plan);
    };
    const DirectRun zero = with_delay(0.f);
    const DirectRun two = with_delay(2.f);
    require_finite(zero);
    require_finite(two);
    CHECK(cooling_park_motion(zero) != cooling_park_motion(two));
    CHECK(cooling_park_motion(two).find("G4") != std::string::npos);
}

TEST_CASE("WipeTower2 initial loading speed is observed in the load motion",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 50.f}};
    const auto loaded = [&](float start_speed) {
        return run_tower(tower_dynamic({
            {"enable_filament_ramming", true},
            {"filament_ramming_parameters", "100 100 30 30 30 30"},
            {"filament_cooling_moves", "0,0"},
            {"cooling_tube_length", 40},
            {"cooling_tube_retraction", 10},
            {"parking_pos_retraction", 4},
            {"extra_loading_move", 8},
            {"filament_loading_speed", "12,12"},
            {"filament_loading_speed_start", std::to_string(start_speed) + "," + std::to_string(start_speed)},
        }), plan);
    };
    const DirectRun twelve = loaded(12.f);
    const DirectRun twenty_four = loaded(24.f);
    require_finite(twelve);
    require_finite(twenty_four);
    REQUIRE_FALSE(load_motion(twelve).empty());
    CHECK(load_motion(twelve) != load_motion(twenty_four));
    CHECK(feedrates(load_motion(twelve)).count(720) == 1);
}

TEST_CASE("WipeTower2 extra flow changes purge motion at fixed requested volume",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{{0.2f, 0.2f, 0, 1, 120.f}};
    const DirectRun nominal = run_tower(tower_dynamic({{"wipe_tower_extra_flow", 100}}), plan);
    const DirectRun high = run_tower(tower_dynamic({{"wipe_tower_extra_flow", 200}}), plan);
    require_finite(nominal);
    require_finite(high);
    const std::string nominal_wipe = command_section(nominal.gcode, "; CP TOOLCHANGE WIPE\n", "; CP TOOLCHANGE END\n");
    const std::string high_wipe = command_section(high.gcode, "; CP TOOLCHANGE WIPE\n", "; CP TOOLCHANGE END\n");
    REQUIRE_FALSE(nominal_wipe.empty());
    CHECK(nominal_wipe != high_wipe);
}

TEST_CASE("WipeTower2 extra spacing changes later-layer purge depth",
          "[WipeTower][MultimaterialWipe][WipeTowerVariance]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f}, {0.4f, 0.2f, 1, 0, 900.f}};
    const DirectRun nominal = run_tower(tower_dynamic({{"enable_filament_ramming", false}, {"wipe_tower_extra_spacing", 100}}), plan);
    const DirectRun wide = run_tower(tower_dynamic({{"enable_filament_ramming", false}, {"wipe_tower_extra_spacing", 300}}), plan);
    require_finite(nominal);
    require_finite(wide);
    CHECK(wide.depth > nominal.depth);
}
namespace wipe_tower_solid_finish_contracts {
namespace {

struct GridTrace {
    bool saw_grid = false;
    std::size_t extrusion_moves = 0;
    std::size_t invalid_motion = 0;
    double path_length = 0.;
    double extrusion_delta = 0.;
};

static std::string layer_gcode(const std::vector<WipeTower::ToolChangeResult> &layer)
{
    std::string out;
    for (const WipeTower::ToolChangeResult &change : layer)
        out += change.gcode;
    return out;
}

static GridTrace trace_empty_grid(const std::vector<WipeTower::ToolChangeResult> &layer,
                                  const DynamicPrintConfig &config)
{
    GridTrace trace;
    bool in_grid = false;
    GCodeReader reader;
    reader.apply_config(config);
    reader.parse_buffer(layer_gcode(layer), [&trace, &in_grid](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        const std::string_view comment = line.comment();
        if (comment.find("CP EMPTY GRID START") != std::string_view::npos) {
            in_grid = true;
            trace.saw_grid = true;
            return;
        }
        if (comment.find("CP EMPTY GRID END") != std::string_view::npos) {
            in_grid = false;
            return;
        }
        if (!in_grid || !line.cmd_is("G1") || !line.has_e() || (!line.has_x() && !line.has_y()))
            return;
        const float distance = line.dist_XY(self);
        const float extrusion = line.dist_E(self);
        if (!std::isfinite(double(distance)) || !std::isfinite(double(extrusion))) {
            ++trace.invalid_motion;
            return;
        }
        if (!line.extruding(self) || distance <= 0.f || extrusion <= 0.f)
            return;
        ++trace.extrusion_moves;
        trace.path_length += double(distance);
        trace.extrusion_delta += double(extrusion);
    });
    return trace;
}

static void require_positive_grid(const GridTrace &trace)
{
    REQUIRE(trace.saw_grid);
    REQUIRE(trace.invalid_motion == 0);
    REQUIRE(trace.extrusion_moves > 0);
    REQUIRE(trace.path_length > 0.);
    REQUIRE(trace.extrusion_delta > 0.);
    CHECK(std::isfinite(trace.path_length));
    CHECK(std::isfinite(trace.extrusion_delta));
}

} // namespace

TEST_CASE("Soluble toolchanges on the next layer produce denser wipe-tower finish infill",
          "[WipeTower][WipeTowerSolidInfill][MultimaterialWipe]")
{
    // Row 1 looks ahead to 0 -> 1 (new tool soluble); row 2 looks ahead to
    // 1 -> 0 (old tool soluble). The 120 mm3 purge on the later row is
    // intentionally larger than row 2's 80 mm3 purge, so plan_tower() gives
    // rows 1 and 2 positive free depth for the marked grid.
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 0, 80.f},
        {0.6f, 0.2f, 0, 1, 80.f},
        {0.8f, 0.2f, 1, 0, 120.f},
    };
    const DynamicPrintConfig soluble_config = tower_dynamic({
        {"filament_soluble", "0,1"},
        {"filament_is_support", "0,0"},
        {"nozzle_diameter", "0.4"},
        {"enable_filament_ramming", true},
        {"use_relative_e_distances", true},
    });
    const DynamicPrintConfig control_config = tower_dynamic({
        {"filament_soluble", "0,0"},
        {"filament_is_support", "0,0"},
        {"nozzle_diameter", "0.4"},
        {"enable_filament_ramming", true},
        {"use_relative_e_distances", true},
    });
    const DirectRun soluble = run_tower(soluble_config, plan);
    const DirectRun control = run_tower(control_config, plan);
    require_finite(soluble);
    require_finite(control);
    REQUIRE(soluble.layers.size() == plan.size());
    REQUIRE(control.layers.size() == plan.size());

    const GridTrace soluble_new_only = trace_empty_grid(soluble.layers[1], soluble_config);
    const GridTrace soluble_old_only = trace_empty_grid(soluble.layers[2], soluble_config);
    const GridTrace control_new_only = trace_empty_grid(control.layers[1], control_config);
    const GridTrace control_old_only = trace_empty_grid(control.layers[2], control_config);
    require_positive_grid(soluble_new_only);
    require_positive_grid(soluble_old_only);
    require_positive_grid(control_new_only);
    require_positive_grid(control_old_only);

    // Solid infill uses repeated long rows while the all-insoluble control uses
    // the inverse-U and sparse zig-zag. These are emitted geometry/material
    // properties for each independent soluble direction.
    CHECK(soluble_new_only.path_length > control_new_only.path_length);
    CHECK(soluble_new_only.extrusion_moves > control_new_only.extrusion_moves);
    CHECK(soluble_new_only.extrusion_delta > control_new_only.extrusion_delta);
    CHECK(soluble_old_only.path_length > control_old_only.path_length);
    CHECK(soluble_old_only.extrusion_moves > control_old_only.extrusion_moves);
    CHECK(soluble_old_only.extrusion_delta > control_old_only.extrusion_delta);
}

} // namespace wipe_tower_solid_finish_contracts

} // namespace multimaterial_wipe_variance_contracts

namespace legacy_wipe_toolordering_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

FullPrintConfig legacy_config(bool semm = false,
                              WipeTowerWallType wall = WipeTowerWallType::wtwRectangle,
                              bool sparse = false,
                              bool arc_fitting = false)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"single_extruder_multi_material", semm},
        {"enable_prime_tower", true},
        {"prime_tower_width", 35},
        {"prime_tower_infill_gap", 150},
        {"nozzle_diameter", "0.4"},
        {"filament_diameter", "1.75,1.75"},
        {"prime_tower_brim_width", 0},
        {"wipe_tower_x", 20},
        {"wipe_tower_y", 20},
        {"wipe_tower_wall_type", wall == WipeTowerWallType::wtwRib ? "rib" : "rectangle"},
        {"prime_tower_skip_points", false},
        {"wipe_tower_no_sparse_layers", !sparse},
        {"wipe_tower_extra_flow", 100},
        {"wipe_tower_extra_spacing", 100},
        {"enable_arc_fitting", arc_fitting},
        {"filament_prime_volume", "80,80"},
        {"filament_prime_volume_nc", "80,80"},
        {"filament_change_length", "10,10"},
        {"filament_change_length_nc", "30,30"},
        {"flush_volumes_matrix", "0,80,80,0"},
        {"filament_adhesiveness_category", "0,1"},
    });
    FullPrintConfig full;
    full.apply(config);
    return full;
}

std::optional<MultiNozzleUtils::LayeredNozzleGroupResult> group_for_two_filaments(bool same_extruder = false)
{
    std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
    nozzles.push_back({"0.4", nvtStandard, 0, 0});
    nozzles.push_back({"0.4", nvtStandard, same_extruder ? 0 : 1, 1});
    return MultiNozzleUtils::LayeredNozzleGroupResult::create(
        std::vector<int>{0, 1}, nozzles, std::vector<unsigned int>{0, 1});
}

double polygon_area_abs(const Polygon &polygon)
{
    return std::abs(polygon.area()) * double(SCALING_FACTOR) * double(SCALING_FACTOR);
}

Polygon rectangle(double left, double bottom, double right, double top)
{
    return Polygon::new_scale(Pointfs{{left, bottom}, {right, bottom}, {right, top}, {left, top}});
}

struct DirectRun {
    std::vector<std::vector<WipeTower::ToolChangeResult>> layers;
    float depth = 0.f;
    float width = 0.f;
};

DirectRun run_direct(const std::vector<std::tuple<float, float, unsigned int, unsigned int, float>> &plan,
                     bool semm = false,
                     WipeTowerWallType wall = WipeTowerWallType::wtwRectangle,
                     bool sparse = false,
                     bool arc_fitting = false,
                     bool same_extruder = false)
{
    FullPrintConfig config = legacy_config(semm, wall, sparse, arc_fitting);
    auto group = group_for_two_filaments(same_extruder);
    REQUIRE(group.has_value());
    WipeTower tower(config, 0, Vec3d::Zero(), 0, 8.f, {0, 1});
    tower.set_nozzle_group_result(*group);
    tower.set_used_filament_ids({0, 1});
    tower.set_filament_categories({0, 1});
    tower.set_extruder(0, config);
    tower.set_extruder(1, config);
    for (const auto &[z, h, old_tool, new_tool, purge] : plan)
        tower.plan_toolchange(z, h, old_tool, new_tool, purge, purge, purge);
    DirectRun result;
    tower.generate_new(result.layers);
    result.depth = tower.get_depth();
    result.width = tower.width();
    return result;
}

void require_finite_result(const DirectRun &run)
{
    REQUIRE_FALSE(run.layers.empty());
    REQUIRE(run.width > 0.f);
    REQUIRE(run.depth > 0.f);
    for (const auto &layer : run.layers) {
        REQUIRE_FALSE(layer.empty());
        for (const auto &tcr : layer) {
            CHECK(std::isfinite(double(tcr.print_z)));
            CHECK(std::isfinite(double(tcr.layer_height)));
            CHECK(std::isfinite(double(tcr.elapsed_time)));
            CHECK(std::isfinite(double(tcr.purge_volume)));
            CHECK(tcr.layer_height > 0.f);
            CHECK(tcr.elapsed_time >= 0.f);
            float length = 0.f;
            for (size_t i = 1; i < tcr.extrusions.size(); ++i) {
                if (tcr.extrusions[i].width > 0.f)
                    length += (tcr.extrusions[i].pos - tcr.extrusions[i - 1].pos).norm();
            }
            CHECK(length > 0.f);
            CHECK(std::isfinite(double(tcr.start_pos.x())));
            CHECK(std::isfinite(double(tcr.start_pos.y())));
            CHECK(std::isfinite(double(tcr.end_pos.x())));
            CHECK(std::isfinite(double(tcr.end_pos.y())));
        }
    }
}

PrintRegion make_region()
{
    PrintRegionConfig config;
    config.outer_wall_filament_id.value = 1;
    config.inner_wall_filament_id.value = 2;
    config.sparse_infill_filament_id.value = 3;
    config.internal_solid_filament_id.value = 4;
    config.top_surface_filament_id.value = 5;
    config.bottom_surface_filament_id.value = 6;
    return PrintRegion(config);
}

ExtrusionEntityCollection one_path(ExtrusionRole role)
{
    ExtrusionPath path(role, 0.4, 0.5f, 0.2f);
    path.polyline = Polyline3(Polyline({Point::new_scale(0., 0.), Point::new_scale(10., 0.)}));
    ExtrusionEntityCollection collection;
    collection.append(std::move(path));
    return collection;
}

} // namespace

TEST_CASE("Legacy wipe tower estimators preserve their geometric limits", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    CHECK_THAT(WipeTower::get_limit_depth_by_height(0.f), WithinAbs(5.f, 1e-6f));
    CHECK_THAT(WipeTower::get_limit_depth_by_height(50.f), WithinAbs(12.1052637f, 1e-5f));
    CHECK_THAT(WipeTower::get_limit_depth_by_height(250.f), WithinAbs(40.f, 1e-6f));
    CHECK_THAT(WipeTower::get_limit_depth_by_height(500.f), WithinAbs(60.f, 1e-6f));
    CHECK_THAT(WipeTower::get_auto_brim_by_height(50.f), WithinAbs(4.f, 1e-6f));
    CHECK_THAT(WipeTower::get_auto_brim_by_height(100.f), WithinAbs(8.f, 1e-6f));
    CHECK_THAT(WipeTower::get_auto_brim_by_height(500.f), WithinAbs(8.f, 1e-6f));
    CHECK_THAT(WipeTower::get_wrapping_detection_depth(), WithinAbs(10.f, 1e-6f));
}

TEST_CASE("Legacy wipe tower nozzle and brim estimators honor nozzle geometry", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    CHECK_THAT(WipeTower::nozzle_change_perimeter_width(0.4f), WithinAbs(1.f, 1e-6f));
    CHECK_THAT(WipeTower::nozzle_change_perimeter_width(0.5f), WithinAbs(1.25f, 1e-6f));
    CHECK_THAT(WipeTower::estimate_brim_real_width(0.f, 0.4f, 0.2f, false), WithinAbs(0.f, 1e-6f));
    const float type1 = WipeTower::estimate_brim_real_width(4.f, 0.4f, 0.2f, false);
    const float type2 = WipeTower::estimate_brim_real_width(4.f, 0.4f, 0.2f, true);
    CHECK(type1 > type2);
    CHECK(type2 > 0.f);
    CHECK(WipeTower::estimate_brim_real_width(4.f, 0.4f, 0.5f, false) > 4.f);
}

TEST_CASE("Legacy wipe tower block estimates scale purge categories and ramming", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    using P = WipeTower::PurgeEstimate;
    CHECK_THAT(WipeTower::estimate_tower_blocks_depth({}, 35.f, 0.2f, 0.4f, 1.f), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(WipeTower::estimate_tower_blocks_depth({P{20.f, 0, 0.f, 1.75f}}, 1.f, 0.2f, 0.4f, 1.f), WithinAbs(0.f, 1e-6f));
    const float one_category = WipeTower::estimate_tower_blocks_depth(
        {P{20.f, 0, 0.f, 1.75f}, P{80.f, 0, 0.f, 1.75f}}, 35.f, 0.2f, 0.4f, 1.5f);
    const float two_categories = WipeTower::estimate_tower_blocks_depth(
        {P{20.f, 0, 0.f, 1.75f}, P{80.f, 1, 10.f, 1.75f}}, 35.f, 0.2f, 0.4f, 1.5f);
    CHECK_THAT(one_category, WithinAbs(20.f, 1e-5f));
    CHECK_THAT(two_categories, WithinAbs(30.25f, 1e-5f));
    CHECK(one_category > 0.f);
    CHECK(two_categories > one_category);
    CHECK_THAT(WipeTower::estimate_tower_blocks_depth({P{20.f, 0, 0.f, 1.75f}}, 35.f, 0.f, 0.4f, 1.f), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(WipeTower::estimate_tower_blocks_depth({P{20.f, 0, 0.f, 1.75f}}, 35.f, 0.2f, 0.f, 1.f), WithinAbs(0.f, 1e-6f));
}

TEST_CASE("Legacy rib footprint and mesh constructors preserve positive solids", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const float square = WipeTower::rib_footprint_side(30.f, 20.f, 2.f, 0.f, 20.f);
    const float extended = WipeTower::rib_footprint_side(30.f, 20.f, 2.f, 10.f, 20.f);
    CHECK(square >= 30.f);
    CHECK(extended > square);
    CHECK_THAT(WipeTower::rib_footprint_side(0.f, 20.f, 2.f, 0.f, 20.f), WithinAbs(0.f, 1e-6f));
    const float bbox = WipeTower::estimate_rib_tower_bbox_side(
        {{40.f, 0, 0.f, 1.75f}, {80.f, 1, 12.f, 1.75f}}, 35.f, 0.2f, 0.4f, 1.5f, 2.f, 3.f, 20.f);
    CHECK(bbox > 0.f);
    const Polygon section = WipeTower::rib_section(30.f, 20.f, 15.f, 2.f, false);
    REQUIRE(section.points.size() >= 4);
    CHECK(polygon_area_abs(section) > 0.0);
    const TriangleMesh tower = WipeTower::its_make_rib_tower(30.f, 20.f, 8.f, 60.f, 2.f, false);
    // The helper fills indexed_triangle_set directly and does not refresh TriangleMesh::m_stats.
    REQUIRE(!tower.its.indices.empty());
    REQUIRE(!tower.its.vertices.empty());
    CHECK_THAT(tower.its.vertices.front().z(), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(tower.its.vertices.back().z(), WithinAbs(8.f, 1e-6f));
    Polygon brim = rectangle(0., 0., 30., 20.);
    const TriangleMesh brim_mesh = WipeTower::its_make_rib_brim(brim, 0.2f);
    REQUIRE(!brim_mesh.its.indices.empty());
    REQUIRE(!brim_mesh.its.vertices.empty());
    CHECK_THAT(brim_mesh.its.vertices.front().z(), WithinAbs(0.f, 1e-6f));
    CHECK_THAT(brim_mesh.its.vertices.back().z(), WithinAbs(0.2f, 1e-6f));
}

TEST_CASE("Legacy rounding transforms corners while retaining footprint area", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    Polygon p = rectangle(0., 0., 30., 20.);
    const double original = polygon_area_abs(p);
    Polygon rounded = WipeTower::rounding_polygon(p, 2.0);
    REQUIRE(rounded.points.size() > p.points.size());
    CHECK(polygon_area_abs(rounded) > original * 0.8);
    CHECK(polygon_area_abs(rounded) < original * 1.01);
    Polygon degenerate;
    degenerate.points = {Point::new_scale(0., 0.), Point::new_scale(1., 0.)};
    CHECK(WipeTower::rounding_polygon(degenerate, 2.).points.size() == degenerate.points.size());
}

TEST_CASE("Legacy gap construction cuts skip points without losing wall area", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const Polygon wall = rectangle(0., 0., 30., 20.);
    Polygon inserted;
    const auto no_skips = construct_gap_for_skip_points(wall, {}, 30.f, 2.f, inserted);
    REQUIRE(no_skips.size() == 1);
    CHECK(inserted.points.size() == wall.points.size());
    Polygon inserted_one;
    const auto one_skip = construct_gap_for_skip_points(
        wall, {Vec2f(30.f, 10.f)}, 30.f, 2.f, inserted_one);
    REQUIRE_FALSE(one_skip.empty());
    CHECK(one_skip.size() == 2);
    CHECK_FALSE(inserted_one.points.empty());
    CHECK_THAT(polygon_area_abs(inserted_one), WithinAbs(600.0, 1e-3));
    Polygon inserted_two;
    const auto two_skips = construct_gap_for_skip_points(
        wall, {Vec2f(30.f, 10.f), Vec2f(0.f, 10.f)}, 30.f, 2.f, inserted_two);
    REQUIRE_FALSE(two_skips.empty());
    CHECK(two_skips.size() == 3);
    CHECK_THAT(polygon_area_abs(inserted_two), WithinAbs(600.0, 1e-3));
}

TEST_CASE("Sparse tower layer predicates and compacted heights agree", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    WipeTower::ToolChangeResult sparse;
    sparse.initial_tool = 1;
    sparse.new_tool = 1;
    WipeTower::ToolChangeResult change;
    change.initial_tool = 0;
    change.new_tool = 1;
    change.layer_height = 0.4f;
    REQUIRE(wipe_tower_layer_is_sparse({sparse}));
    REQUIRE_FALSE(wipe_tower_layer_is_sparse({change}));
    REQUIRE_FALSE(wipe_tower_layer_is_sparse({}));
    sparse.combined_away = true;
    REQUIRE(wipe_tower_layer_is_combined_away({sparse}));
    CHECK_FALSE(wipe_tower_layer_is_combined_away({}));
    const std::vector<std::vector<WipeTower::ToolChangeResult>> layers = {{change}, {sparse}, {change}};
    const auto z = compute_compacted_wipe_tower_z(layers, 0.1f);
    REQUIRE(z.size() == 3);
    CHECK_THAT(z[0], WithinAbs(0.5f, 1e-6f));
    CHECK_THAT(z[1], WithinAbs(0.5f, 1e-6f));
    CHECK_THAT(z[2], WithinAbs(0.9f, 1e-6f));
}

TEST_CASE("Sparse tower combination folds only layers within nozzle caps", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    std::vector<float> heights{0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
    const std::vector<char> sparse{0, 1, 1, 0, 1};
    const std::vector<float> caps{0.2f, 0.4f, 0.4f, 0.2f, 0.4f};
    const auto folded = combine_sparse_wipe_tower_layers(heights, sparse, caps, 0);
    REQUIRE(folded.size() == heights.size());
    CHECK(folded[0] == 0);
    CHECK(folded[1] == 1);
    CHECK(folded[2] == 0);
    CHECK_THAT(heights[2], WithinAbs(0.4f, 1e-6f));
    CHECK(folded[4] == 0);
}

TEST_CASE("Sparse tower plan applies filament-specific layer-height caps", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    struct Change {
        unsigned int old_tool;
        unsigned int new_tool;
    };
    struct Layer {
        float height;
        std::vector<Change> tool_changes;
        bool combined_away = false;
    };
    std::vector<Layer> plan(4);
    for (auto &layer : plan) layer.height = 0.2f;
    plan[0].tool_changes.push_back({0, 0});
    plan[3].tool_changes.push_back({0, 0});
    std::vector<WipeTower::FilamentParameters> filpar(2);
    filpar[0].max_layer_height = 0.4f;
    filpar[1].max_layer_height = 0.2f;
    combine_sparse_wipe_tower_plan(plan, filpar, 0, 0);
    CHECK(plan[1].combined_away);
    CHECK_FALSE(plan[2].combined_away);
    CHECK_THAT(plan[2].height, WithinAbs(0.4f, 1e-6f));
}

TEST_CASE("Legacy tower direct generation keeps purge, timing, and layer invariants", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 50.f},
        {0.4f, 0.2f, 1, 0, 120.f},
        {0.6f, 0.3f, 0, 1, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, false, true);
    require_finite_result(run);
    REQUIRE(run.layers.size() == 3);
    CHECK_THAT(run.layers[0].front().purge_volume, WithinAbs(50.f, 1e-5f));
    CHECK_THAT(run.layers[1].front().purge_volume, WithinAbs(120.f, 1e-5f));
    CHECK_THAT(run.layers[2].front().layer_height, WithinAbs(0.3f, 1e-5f));
    CHECK(run.layers[0].front().gcode.find("WIPE_TOWER_START") != std::string::npos);
    CHECK(run.layers[0].front().gcode.find("WIPE_TOWER_END") != std::string::npos);
}

TEST_CASE("Legacy tower rib generation changes wall footprint while preserving transitions", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const DirectRun rectangle_run = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false, WipeTowerWallType::wtwRectangle);
    const DirectRun rib_run = run_direct({{0.2f, 0.2f, 0, 1, 80.f}}, false, WipeTowerWallType::wtwRib);
    require_finite_result(rectangle_run);
    require_finite_result(rib_run);
    REQUIRE_FALSE(rectangle_run.layers.front().empty());
    REQUIRE_FALSE(rib_run.layers.front().empty());
    CHECK(rectangle_run.layers.front().front().initial_tool == rib_run.layers.front().front().initial_tool);
    CHECK(rectangle_run.layers.front().front().new_tool == rib_run.layers.front().front().new_tool);
    float rib_length = 0.f;
    for (size_t i = 1; i < rib_run.layers.front().front().extrusions.size(); ++i)
        if (rib_run.layers.front().front().extrusions[i].width > 0.f)
            rib_length += (rib_run.layers.front().front().extrusions[i].pos - rib_run.layers.front().front().extrusions[i - 1].pos).norm();
    CHECK(rib_length > 0.f);
    CHECK(rib_run.depth >= rectangle_run.depth);
}

TEST_CASE("Legacy tower sparse generation keeps compacted layer state finite", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.6f, 0.4f, 1, 0, 80.f},
        {1.0f, 0.4f, 0, 1, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true);
    require_finite_result(run);
    CHECK(run.layers.size() >= 2);
    for (const auto &layer : run.layers)
        for (const auto &tcr : layer)
            CHECK(tcr.layer_height > 0.f);
}

TEST_CASE("LayerTools resolves role filaments and explicit overrides", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    PrintRegion region = make_region();
    LayerTools tools(0.2);
    tools.extruders = {2, 0, 1};
    CHECK(tools.is_extruder_order(2, 0));
    CHECK(tools.is_extruder_order(0, 1));
    CHECK_FALSE(tools.is_extruder_order(1, 0));
    CHECK_FALSE(tools.is_extruder_order(7, 0));
    CHECK(tools.wall_extruder_id(region) == 0u);
    CHECK(tools.sparse_infill_filament_id(region) == 2u);
    CHECK(tools.internal_solid_filament_id(region) == 3u);
    tools.mixed_filament_resolution[2] = 0;
    CHECK(tools.sparse_infill_filament_id(region) == 0u);
    tools.extruder_override = 5;
    CHECK(tools.wall_extruder_id(region) == 4u);
    CHECK(tools.has_extruder(2));
    CHECK_FALSE(tools.has_extruder(8));
}

TEST_CASE("LayerTools selects role-specific extrusion filaments", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    PrintRegion region = make_region();
    LayerTools tools(0.2);
    auto perimeter = one_path(erPerimeter);
    auto sparse = one_path(erInternalInfill);
    auto top = one_path(erTopSolidInfill);
    auto bottom = one_path(erBottomSurface);
    CHECK(tools.extruder(perimeter, region) == 1u);
    CHECK(tools.extruder(sparse, region) == 2u);
    CHECK(tools.extruder(top, region) == 4u);
    CHECK(tools.extruder(bottom, region) == 5u);
    tools.extruder_override = 6;
    CHECK(tools.extruder(perimeter, region) == 5u);
}

TEST_CASE("ToolOrdering locates the nearest layer and honors custom order", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    ToolOrdering ordering;
    ordering.layer_tools().emplace_back(0.2);
    ordering.layer_tools().emplace_back(0.4);
    ordering.layer_tools().emplace_back(0.8);
    ordering.layer_tools()[0].extruders = {2, 0, 1};
    CHECK_THAT(ordering.tools_for_layer(0.4).print_z, WithinAbs(0.4, 1e-9));
    CHECK_THAT(ordering.tools_for_layer(0.8).print_z, WithinAbs(0.8, 1e-9));
    CHECK_THAT(ordering.tools_for_layer(0.4 + 0.25 * EPSILON).print_z, WithinAbs(0.4, 1e-9));
    CHECK(ordering.layer_tools()[0].is_extruder_order(2, 1));
}

TEST_CASE("WipingExtrusions gates object and support wiping by role", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_num_filaments(2);
    config.set_deserialize_strict({
        {"flush_into_objects", true}, {"flush_into_infill", true}, {"flush_into_support", true},
        {"support_filament", 0}, {"support_interface_filament", 0},
        {"filament_soluble", "0,0"}, {"filament_is_support", "0,0"},
        {"sparse_infill_filament_id", 1}, {"outer_wall_filament_id", 1},
        {"inner_wall_filament_id", 1}, {"internal_solid_filament_id", 1},
        {"top_surface_filament_id", 1}, {"bottom_surface_filament_id", 1},
    });
    Model model;
    model.add_object("cube", "", make_cube(10, 10, 10))->add_instance();
    Print print;
    print.apply(model, config);
    REQUIRE_FALSE(print.objects().empty());
    const PrintObject *object = print.objects().front();
    LayerTools tools(0.2);
    tools.extruders = {0, 1};
    WipingExtrusions &wiping = tools.wiping_extrusions();
    PrintRegionConfig region_config;
    region_config.apply(config, true);
    PrintRegion region(region_config);
    auto infill = one_path(erInternalInfill);
    auto perimeter = one_path(erPerimeter);
    CHECK(wiping.is_overriddable(infill, print.config(), *object, region));
    CHECK(wiping.is_overriddable(perimeter, print.config(), *object, region));
    DynamicPrintConfig no_objects = config;
    no_objects.set_deserialize_strict({{"flush_into_objects", false}, {"flush_into_infill", true}});
    print.apply(model, no_objects);
    object = print.objects().front();
    CHECK(wiping.is_overriddable(infill, print.config(), *object, region));
    CHECK_FALSE(wiping.is_overriddable(perimeter, print.config(), *object, region));
    CHECK(wiping.is_support_overriddable(erSupportMaterial, *object));
    CHECK(wiping.is_support_overriddable(erSupportMaterialInterface, *object));
    CHECK_FALSE(wiping.is_support_overriddable(erPerimeter, *object));
}

TEST_CASE("WipingExtrusions support role gates follow dedicated filament assignment", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"flush_into_support", true}, {"support_filament", 0}, {"support_interface_filament", 0}});
    Model model;
    model.add_object("cube", "", make_cube(10, 10, 10))->add_instance();
    Print print;
    print.apply(model, config);
    REQUIRE_FALSE(print.objects().empty());
    const PrintObject *object = print.objects().front();
    LayerTools tools(0.2);
    WipingExtrusions &wiping = tools.wiping_extrusions();
    CHECK(wiping.is_support_overriddable(erMixed, *object));
    CHECK(wiping.is_support_overriddable(erSupportMaterial, *object));
    CHECK(wiping.is_support_overriddable(erSupportMaterialInterface, *object));
    CHECK_FALSE(wiping.is_support_overriddable(erNone, *object));
}

TEST_CASE("ToolOrdering cyclic parser remains strict and 1-based", "[WipeTower][ToolOrderingLegacy][WipeTowerBroad]")
{
    CHECK(parse_cyclic_order("4, 2, 2, 0, 9, abc, 1x, 3", 4) == std::vector<unsigned int>{3, 1, 2});
    CHECK(parse_cyclic_order("", 4).empty());
    CHECK(parse_cyclic_order("-1,0,5", 4).empty());
}

TEST_CASE("Wipe tower sparse config gates distinguish skip and combine modes", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"wipe_tower_no_sparse_layers", true}, {"wipe_tower_sparse_layers_combination", true}});
    FullPrintConfig full;
    full.apply(config);
    CHECK(wipe_tower_sparse_layers_skipped(full));
    CHECK_FALSE(wipe_tower_sparse_layers_combined(full));
    full.timelapse_type.value = TimelapseType::tlSmooth;
    CHECK_FALSE(wipe_tower_sparse_layers_skipped(full));
    CHECK_FALSE(wipe_tower_sparse_layers_combined(full));
    full.timelapse_type.value = TimelapseType::tlTraditional;
    full.enable_wrapping_detection.value = true;
    CHECK_FALSE(wipe_tower_sparse_layers_skipped(full));
    CHECK_FALSE(wipe_tower_sparse_layers_combined(full));
}

TEST_CASE("Legacy direct tower preserves changing tool and purge records", "[WipeTower][WipeTowerLegacy][WipeTowerBroad]")
{
    const DirectRun run = run_direct({{0.2f, 0.2f, 0, 1, 40.f}}, true,
                                     WipeTowerWallType::wtwRectangle, false, false, true);
    require_finite_result(run);
    REQUIRE_FALSE(run.layers.front().empty());
    const auto &change = run.layers.front().front();
    CHECK(change.is_tool_change);
    CHECK(change.initial_tool == 0);
    CHECK(change.new_tool == 1);
    CHECK_THAT(change.purge_volume, WithinAbs(40.f, 1e-5f));
    CHECK_FALSE(change.gcode.empty());
}
} // namespace legacy_wipe_toolordering_contracts

namespace toolordering_public_routes_contracts {

TEST_CASE("Public ToolOrdering preserves extruder and logical nozzle-group assignments", "[WipeTower][ToolOrderingPublicRoutes]")
{
    SECTION("non-BBL multi-extruder maps each used filament to its matching extruder")
    {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4};
        config.option<ConfigOptionStrings>("extruder_nozzle_stats", true)->values = {"Standard#1", "Standard#1"};
        config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75};
        config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00"};
        config.option<ConfigOptionInts>("filament_map", true)->values = {1, 2};
        config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values = std::vector<double>(8, 0.0);
        config.option<ConfigOptionFloats>("flush_multiplier", true)->values = {1.0, 1.0};

        Model model;
        model.add_object("cube", "", make_cube(4, 4, 0.8))->add_instance();
        Print print;
        print.apply(model, config);
        REQUIRE_FALSE(print.is_BBL_printer());
        REQUIRE(print.config().filament_colour.values.size() == 2);
        REQUIRE(print.config().nozzle_diameter.values.size() == 2);

        const auto result = ToolOrdering::get_recommended_filament_maps(
            {{0, 1}, {1}}, &print, FilamentMapMode::fmmAutoForFlush, {}, {});
        REQUIRE(result.get_used_filaments() == std::vector<unsigned int>{0, 1});
        CHECK(result.get_extruder_map(false) == std::vector<int>{1, 2});
        CHECK(result.get_extruder_id(0) == 0);
        CHECK(result.get_extruder_id(1) == 1);
    }

    SECTION("fully-manual multi-nozzle mapping preserves distinct and shared logical groups")
    {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4};
        config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count", true)->values = {3};
        config.option<ConfigOptionStrings>("extruder_nozzle_stats", true)->values = {"Standard#3"};
        config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75, 1.75};
        config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#FF0000", "#00FF00", "#0000FF"};
        config.option<ConfigOptionInts>("filament_map", true)->values = {1, 1, 1};
        config.option<ConfigOptionInts>("filament_volume_map", true)->values = {
            static_cast<int>(nvtStandard), static_cast<int>(nvtStandard), static_cast<int>(nvtStandard)};
        // Input ids are logical equivalence classes: filaments 1 and 2 share id 2,
        // while filament 0 requests a distinct class. The resolver assigns those
        // classes to available matching physical nozzles in canonical order.
        config.option<ConfigOptionInts>("filament_nozzle_map", true)->values = {0, 2, 2};

        Model model;
        model.add_object("cube", "", make_cube(4, 4, 0.8))->add_instance();
        Print print;
        print.apply(model, config);
        REQUIRE(print.config().filament_colour.values.size() == 3);
        REQUIRE(print.config().extruder_nozzle_stats.values == std::vector<std::string>{"Standard#3"});
        REQUIRE(print.config().filament_nozzle_map.values == std::vector<int>{0, 2, 2});
        REQUIRE(print.config().extruder_max_nozzle_count.values == std::vector<int>{3});

        const auto result = ToolOrdering::get_recommended_filament_maps(
            {{0, 1, 2}}, &print, FilamentMapMode::fmmNozzleManual, {}, {});
        REQUIRE(result.get_used_filaments() == std::vector<unsigned int>{0, 1, 2});
        CHECK(result.get_extruder_map(false) == std::vector<int>{1, 1, 1});
        const auto nozzle_map = result.get_nozzle_map();
        REQUIRE(nozzle_map.size() == 3);
        CHECK(nozzle_map[0] != nozzle_map[1]);
        CHECK(nozzle_map[1] == nozzle_map[2]);
        CHECK(result.get_nozzle_id(0) != result.get_nozzle_id(1));
        CHECK(result.get_nozzle_id(1) == result.get_nozzle_id(2));
        CHECK(result.get_extruder_id(0) == 0);
        CHECK(result.get_extruder_id(1) == 0);
        CHECK(result.get_extruder_id(2) == 0);
        CHECK(result.get_volume_map() == std::vector<int>{static_cast<int>(nvtStandard), static_cast<int>(nvtStandard), static_cast<int>(nvtStandard)});
    }
}

namespace toolordering_support_interface_contracts {

namespace {

// Two separated overhangs keep the mixed-role witness independent of the
// ordinary object layer grid: each cap starts at a grid height, while the
// independent 0.1 mm contact gap puts its support contact at a non-grid Z.
// Their footprints differ, so a lower cap interface and higher cap base can
// coexist in the support slab without relying on a single-cap shortcut.
static TriangleMesh support_interface_capital()
{
    TriangleMesh base = make_cube(16., 16., 1.2);

    TriangleMesh lower_stem = make_cube(4., 4., 2.8);
    lower_stem.translate(2., 2., 1.2); // z = 1.2 .. 4.0
    TriangleMesh lower_cap = make_cube(10., 10., 1.2);
    lower_cap.translate(0., 0., 4.0); // z = 4.0 .. 5.2

    TriangleMesh upper_stem = make_cube(3., 3., 2.8);
    upper_stem.translate(6.5, 6.5, 5.2); // z = 5.2 .. 8.0
    TriangleMesh upper_cap = make_cube(14., 14., 1.2);
    upper_cap.translate(1., 1., 8.0); // z = 8.0 .. 9.2

    base.merge(lower_stem);
    base.merge(lower_cap);
    base.merge(upper_stem);
    base.merge(upper_cap);
    return base;
}

struct SupportInterfaceRun {
    std::vector<std::vector<unsigned int>> support_only_sequences;
    double support_volume = 0.;
    size_t non_grid_mixed_rows = 0;
};

static SupportInterfaceRun run_support_interface_case(bool interface_not_for_body)
{
    DynamicPrintConfig config = multifilament_config(2, {
        { "enable_support",                    true },
        { "enable_prime_tower",                 false },
        { "independent_support_layer_height",   true },
        { "support_type",                      "normal(auto)" },
        { "support_on_build_plate_only",       false },
        { "support_interface_top_layers",      3 },
        { "support_interface_bottom_layers",    0 },
        { "support_top_z_distance",             0.1 },
        { "support_bottom_z_distance",          0.0 },
        { "support_base_pattern_spacing",       2.5 },
        { "support_filament",                   0 },
        { "support_interface_filament",         2 },
        { "support_interface_not_for_body",    interface_not_for_body },
        { "filament_is_support",                "0,1" },
        { "filament_soluble",                   "0,0" },
        // The interface filament is row 1.  Its distinct rows make the
        // interface-tool choice observable without relying on a tie-break.
        { "flush_volumes_matrix",               "0,100,100,0" },
        { "layer_height",                       0.2 },
        { "initial_layer_print_height",         0.2 },
        { "wall_loops",                         1 },
        { "sparse_infill_density",              0 },
        { "top_shell_layers",                  0 },
        { "bottom_shell_layers",               0 },
        { "skirt_loops",                        0 },
        { "brim_width",                         0.0 },
        { "raft_layers",                        0 },
        { "use_relative_e_distances",           true },
        { "layer_change_gcode",                 "G92 E0\n" },
    });

    Print print;
    Model model;
    std::vector<TriangleMesh> meshes;
    meshes.emplace_back(support_interface_capital());
    init_print(std::move(meshes), print, model, config, nullptr, false);
    const StringObjectException validation = print.validate();
    INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    print.process();

    REQUIRE_FALSE(print.objects().empty());
    const PrintObject *object = print.objects().front();
    REQUIRE(object != nullptr);
    REQUIRE_FALSE(object->layers().empty());
    REQUIRE_FALSE(object->support_layers().empty());

    const ToolOrdering &ordering = print.tool_ordering();
    SupportInterfaceRun out;
    for (const SupportLayer *support_layer : object->support_layers()) {
        bool has_base = false;
        bool has_interface = false;
        for (const ExtrusionEntity *entity : support_layer->support_fills.entities) {
            REQUIRE(entity != nullptr);
            if (entity->role() == erSupportMaterial || entity->role() == erSupportTransition)
                has_base = true;
            if (entity->role() == erSupportMaterialInterface)
                has_interface = true;
        }
        if (!has_base || !has_interface)
            continue;

        const LayerTools &tools = ordering.tools_for_layer(support_layer->print_z);
        REQUIRE(tools.has_support);
        out.support_volume += support_layer->support_fills.total_volume();
        if (tools.has_object)
            continue;
        REQUIRE_FALSE(tools.extruders.empty());
        out.support_only_sequences.push_back(tools.extruders);
        const double layer_units = support_layer->print_z / 0.2;
        if (std::abs(layer_units - std::round(layer_units)) > 1e-4)
            ++out.non_grid_mixed_rows;
    }
    REQUIRE(std::isfinite(out.support_volume));
    REQUIRE(out.support_volume > 0.);
    REQUIRE_FALSE(out.support_only_sequences.empty());
    REQUIRE(out.non_grid_mixed_rows > 0);
    return out;
}

} // namespace

TEST_CASE("Support-only interface fallback honors body exclusion and flush costs",
          "[WipeTower][ToolOrdering][ToolOrderingSupportInterface]")
{
    const SupportInterfaceRun body_allowed = run_support_interface_case(false);
    const SupportInterfaceRun body_excluded = run_support_interface_case(true);

    REQUIRE(body_allowed.support_only_sequences.size() == body_excluded.support_only_sequences.size());
    REQUIRE(body_allowed.support_only_sequences.size() > 0);
    CHECK_THAT(body_allowed.support_volume, Catch::Matchers::WithinAbs(body_excluded.support_volume, 1e-6));

    // With no dedicated support filament, body-allowed support uses the lowest
    // flush-volume destination from the interface row: the pinned matrix makes
    // that the interface filament itself (tool 1 after zero-based reindexing).
    for (const auto &sequence : body_allowed.support_only_sequences)
        CHECK(sequence == std::vector<unsigned int>{1});

    // Excluding the interface tool as the current body filament forces the
    // support fallback to the other physical tool, then appends the interface
    // tool for the interface role.  The later ordering pass may rotate these
    // two tools, so the public contract here is membership rather than order.
    for (const auto &sequence : body_excluded.support_only_sequences) {
        REQUIRE(sequence.size() == 2);
        CHECK(std::find(sequence.begin(), sequence.end(), 0u) != sequence.end());
        CHECK(std::find(sequence.begin(), sequence.end(), 1u) != sequence.end());
    }
}

} // namespace toolordering_support_interface_contracts

} // namespace toolordering_public_routes_contracts

namespace tower_contact_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

using legacy_tower_direct_contracts::DirectRun;
using legacy_tower_direct_contracts::extruded_length;
using legacy_tower_direct_contracts::nonempty_layers;
using legacy_tower_direct_contracts::require_finite_run;
using legacy_tower_direct_contracts::group_for_two_filaments;

DirectRun run_direct(const std::vector<std::tuple<float, float, unsigned int, unsigned int, float>> &plan,
                     bool semm = false,
                     WipeTowerWallType wall = WipeTowerWallType::wtwRectangle,
                     bool sparse = false,
                     bool arc_fitting = false,
                     bool same_extruder = false,
                     bool interface_features = false,
                     float interface_dist = 2.f,
                     float interface_length = 3.f)
{
    FullPrintConfig config = legacy_tower_direct_contracts::legacy_config(semm, wall, sparse, arc_fitting);
    config.set_deserialize_strict({
        {"enable_tower_interface_features", interface_features},
        {"filament_tower_interface_print_temp", "215,220"},
        {"filament_tower_interface_pre_extrusion_dist", interface_dist},
        {"filament_tower_interface_pre_extrusion_length", interface_length},
        {"nozzle_temperature", "200,200"},
        {"printable_area", "0x0,200x0,200x200,0x200"},
    });
    auto group = group_for_two_filaments(same_extruder);
    REQUIRE(group.has_value());
    WipeTower tower(config, 0, Vec3d::Zero(), 0, 8.f,
                    std::vector<unsigned int>{0, 1});
    Polygon bed;
    for (const Vec2d &point : config.printable_area.values)
        bed.points.emplace_back(scaled(point.x()), scaled(point.y()));
    tower.set_shared_print_bed({bed});
    tower.set_nozzle_group_result(*group);
    tower.set_used_filament_ids({0, 1});
    tower.set_filament_categories({0, 1});
    tower.set_extruder(0, config);
    tower.set_extruder(1, config);
    for (const auto &[z, h, old_tool, new_tool, purge] : plan)
        tower.plan_toolchange(z, h, old_tool, new_tool, purge, purge, purge);

    DirectRun run;
    tower.generate_new(run.layers);
    run.depth = tower.get_depth();
    run.width = tower.width();
    run.height = tower.get_height();
    run.brim = tower.get_brim_width();
    run.toolchanges = tower.get_number_of_toolchanges();
    return run;
}

float total_extrusion(const DirectRun &run)
{
    float total = 0.f;
    for (const auto &layer : run.layers)
        for (const auto &tcr : layer)
            total += extruded_length(tcr);
    return total;
}

struct GridStats {
    int grid_starts = 0;
    int grid_ends = 0;
    int positive_extrusions = 0;
    int contact_speed_moves = 0;
    int other_speed_moves = 0;
    int m109_count = 0;
    int m104_count = 0;
    std::vector<float> feedrates;
    std::vector<int> m109_temps;
    std::vector<int> m104_temps;
    Vec2f first_extrusion = Vec2f::Zero();
    Vec2f last_extrusion = Vec2f::Zero();
    float extrusion_length = 0.f;
};

bool parse_float_token(const std::string &token, char prefix, float &value)
{
    if (token.size() < 2 || token.front() != prefix)
        return false;
    try {
        std::size_t used = 0;
        value = std::stof(token.substr(1), &used);
        return used == token.size() - 1 && std::isfinite(double(value));
    } catch (...) {
        return false;
    }
}

GridStats grid_stats(const WipeTower::ToolChangeResult &tcr)
{
    GridStats result;
    bool in_grid = false;
    for (const std::string &raw : [&]() {
        std::vector<std::string> lines;
        std::istringstream stream(tcr.gcode);
        for (std::string line; std::getline(stream, line);)
            lines.push_back(std::move(line));
        return lines;
    }()) {
        const std::string line = raw.substr(0, raw.find(';'));
        if (raw.find("CP EMPTY GRID START") != std::string::npos) {
            ++result.grid_starts;
            in_grid = true;
        }
        if (raw.find("CP EMPTY GRID END") != std::string::npos) {
            ++result.grid_ends;
            in_grid = false;
        }
        if (!in_grid)
            continue;
        std::istringstream words(line);
        std::string word;
        while (words >> word) {
            float value = 0.f;
            if (parse_float_token(word, 'F', value))
                result.feedrates.push_back(value);
        }
    }
    for (const std::string &raw : [&]() {
        std::vector<std::string> lines;
        std::istringstream stream(tcr.gcode);
        for (std::string line; std::getline(stream, line);)
            lines.push_back(std::move(line));
        return lines;
    }()) {
        const std::size_t m109 = raw.find("M109 S");
        const std::size_t m104 = raw.find("M104 S");
        auto parse_temp = [&](std::size_t offset, std::vector<int> &temps) {
            try {
                const int temp = std::stoi(raw.substr(offset));
                temps.push_back(temp);
            } catch (...) {
            }
        };
        if (m109 != std::string::npos) {
            ++result.m109_count;
            parse_temp(m109 + 6, result.m109_temps);
        }
        if (m104 != std::string::npos) {
            ++result.m104_count;
            parse_temp(m104 + 6, result.m104_temps);
        }
    }
    bool have_first = false;
    for (std::size_t i = 0; i < tcr.extrusions.size(); ++i) {
        const auto &extrusion = tcr.extrusions[i];
        if (extrusion.width <= 0.f)
            continue;
        ++result.positive_extrusions;
        if (!have_first) {
            result.first_extrusion = extrusion.pos;
            have_first = true;
        }
        result.last_extrusion = extrusion.pos;
        if (i > 0)
            result.extrusion_length += (extrusion.pos - tcr.extrusions[i - 1].pos).norm();
    }
    for (float feedrate : result.feedrates) {
        if (std::abs(feedrate - 1200.f) < 1e-3f)
            ++result.contact_speed_moves;
        else if (feedrate > 0.f)
            ++result.other_speed_moves;
    }
    return result;
}

std::pair<float, float> gcode_x_bounds(const DirectRun &run)
{
    float min_x = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    for (const auto &layer : run.layers)
        for (const auto &tcr : layer) {
            std::istringstream stream(tcr.gcode);
            for (std::string raw; std::getline(stream, raw);) {
                const std::size_t comment = raw.find(';');
                const std::string line = raw.substr(0, comment);
                if (line.find("G1") == std::string::npos && line.find("G0") == std::string::npos)
                    continue;
                std::istringstream words(line);
                for (std::string word; words >> word;) {
                    float value = 0.f;
                    if (parse_float_token(word, 'X', value)) {
                        min_x = std::min(min_x, value);
                        max_x = std::max(max_x, value);
                    }
                }
            }
        }
    return {min_x, max_x};
}

bool has_interface_pre_extrusion_move(const std::string &gcode, float amount = 5.f)
{
    std::istringstream stream(gcode);
    for (std::string raw; std::getline(stream, raw);) {
        const std::size_t comment = raw.find(';');
        const std::string line = raw.substr(0, comment);
        std::istringstream words(line);
        std::string command;
        words >> command;
        if (command != "G1")
            continue;
        bool has_e = false;
        bool has_f = false;
        float e = 0.f;
        float f = 0.f;
        for (std::string word; words >> word;) {
            if (parse_float_token(word, 'E', e))
                has_e = true;
            if (parse_float_token(word, 'F', f))
                has_f = true;
        }
        if (has_e && has_f && std::abs(e - amount) < 1e-4f && std::abs(f - 100.f) < 1e-4f)
            return true;
    }
    return false;
}

std::vector<GridStats> layer_grid_stats(const DirectRun &run)
{
    std::vector<GridStats> result;
    result.reserve(run.layers.size());
    for (const auto &layer : run.layers) {
        GridStats combined;
        for (const auto &tcr : layer) {
            const GridStats current = grid_stats(tcr);
            combined.grid_starts += current.grid_starts;
            combined.grid_ends += current.grid_ends;
            combined.positive_extrusions += current.positive_extrusions;
            combined.contact_speed_moves += current.contact_speed_moves;
            combined.other_speed_moves += current.other_speed_moves;
            combined.m109_count += current.m109_count;
            combined.m104_count += current.m104_count;
            combined.feedrates.insert(combined.feedrates.end(), current.feedrates.begin(), current.feedrates.end());
            combined.m109_temps.insert(combined.m109_temps.end(), current.m109_temps.begin(), current.m109_temps.end());
            combined.m104_temps.insert(combined.m104_temps.end(), current.m104_temps.begin(), current.m104_temps.end());
            combined.extrusion_length += current.extrusion_length;
            if (combined.positive_extrusions == current.positive_extrusions) {
                combined.first_extrusion = current.first_extrusion;
                combined.last_extrusion = current.last_extrusion;
            } else {
                combined.last_extrusion = current.last_extrusion;
            }
        }
        result.push_back(std::move(combined));
    }
    return result;
}

} // namespace


TEST_CASE("Wipe tower category changes create complete intermediate contact layers", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f},
        {0.8f, 0.2f, 1, 0, 80.f},
        {1.0f, 0.2f, 0, 0, 80.f},
        {1.2f, 0.2f, 0, 0, 80.f},
        {1.4f, 0.2f, 0, 1, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true);
    require_finite_run(run);
    REQUIRE(run.layers.size() == 7);
    CHECK(nonempty_layers(run) == 7);
    std::string emitted;
    for (const auto &layer : run.layers)
        for (const auto &change : layer)
            emitted += change.gcode;
    CHECK(emitted.find("CP EMPTY GRID START") != std::string::npos);
}

TEST_CASE("Wipe tower category transitions expose grid roles and contact feedrate", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f},
        {0.8f, 0.2f, 1, 0, 80.f},
        {1.0f, 0.2f, 0, 0, 80.f},
        {1.2f, 0.2f, 0, 0, 80.f},
        {1.4f, 0.2f, 0, 1, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true);
    require_finite_run(run);
    const auto stats = layer_grid_stats(run);
    REQUIRE(stats.size() == run.layers.size());
    for (std::size_t i = 0; i < stats.size(); ++i) {
        CAPTURE(i, stats[i].grid_starts, stats[i].grid_ends, stats[i].positive_extrusions,
                stats[i].contact_speed_moves, stats[i].other_speed_moves,
                stats[i].m109_count, stats[i].m104_count,
                stats[i].extrusion_length,
                stats[i].first_extrusion.x(), stats[i].first_extrusion.y(),
                stats[i].last_extrusion.x(), stats[i].last_extrusion.y());
        CHECK(stats[i].grid_starts == stats[i].grid_ends);
        CHECK(stats[i].positive_extrusions > 0);
        CHECK(stats[i].extrusion_length > 0.f);
    }
    CHECK(stats[0].contact_speed_moves == 0);
    CHECK(stats[1].contact_speed_moves > 0);
    CHECK(stats[2].contact_speed_moves > 0);
    CHECK(stats[3].contact_speed_moves == 0);
    CHECK(stats[4].contact_speed_moves > 0);
    CHECK(stats[5].contact_speed_moves > 0);
    CHECK(stats[6].contact_speed_moves == 0);
}

TEST_CASE("Wipe tower contact interfaces emit wait, pre-extrusion and temperature restore", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f},
        {0.8f, 0.2f, 1, 0, 80.f},
        {1.0f, 0.2f, 0, 0, 80.f},
        {1.2f, 0.2f, 0, 0, 80.f},
        {1.4f, 0.2f, 0, 1, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true, false, false, true, 2.f, 3.f);
    require_finite_run(run);
    int m109_count = 0;
    int m104_count = 0;
    int configured_waits = 0;
    int configured_restores = 0;
    int pre_extrusion_moves = 0;
    for (const auto &layer : run.layers)
        for (const auto &tcr : layer) {
            const GridStats stats = grid_stats(tcr);
            m109_count += stats.m109_count;
            m104_count += stats.m104_count;
            configured_waits += int(std::count(stats.m109_temps.begin(), stats.m109_temps.end(), 215) +
                                    std::count(stats.m109_temps.begin(), stats.m109_temps.end(), 220));
            configured_restores += int(std::count(stats.m104_temps.begin(), stats.m104_temps.end(), 200));
            if (has_interface_pre_extrusion_move(tcr.gcode))
                ++pre_extrusion_moves;
        }
    CHECK(m109_count > 0);
    CHECK(m104_count > 0);
    CHECK(configured_waits == m109_count);
    CHECK(configured_restores == m104_count);
    CHECK(pre_extrusion_moves > 0);
}

TEST_CASE("Wipe tower contact pre-extrusion is clipped to the configured rectangular bed", "[WipeTower][WipeTowerContact]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f},
        {0.8f, 0.2f, 1, 0, 80.f},
        {1.0f, 0.2f, 0, 0, 80.f},
        {1.2f, 0.2f, 0, 0, 80.f},
        {1.4f, 0.2f, 0, 1, 80.f},
    };
    const DirectRun run = run_direct(plan, false, WipeTowerWallType::wtwRectangle,
                                     true, false, false, true, 1000.f, 3.f);
    require_finite_run(run);
    const auto [min_x, max_x] = gcode_x_bounds(run);
    CAPTURE(min_x, max_x);
    // The configured 200 mm bed is translated by the 20 mm tower origin before
    // the extreme interface stop positions are clamped: x is therefore in
    // [-20, 180], while both ends remain inside that translated bed.
    CHECK(min_x >= -20.001f);
    CHECK(max_x <= 180.001f);
    CHECK(min_x < -19.9f);
    CHECK(max_x > 179.9f);
}

TEST_CASE("Wipe tower generate_new accepts an empty planned layer set", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({});
    CHECK(run.layers.empty());
    CHECK(run.toolchanges == 0);
}

TEST_CASE("Wipe tower solid geometry remains finite for rectangular and rib walls", "[WipeTower][WipeTowerContact]")
{
    const auto plan = std::vector<std::tuple<float, float, unsigned int, unsigned int, float>>{
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 0, 80.f},
        {0.8f, 0.2f, 0, 0, 80.f},
    };
    const DirectRun rectangle = run_direct(plan, false, WipeTowerWallType::wtwRectangle, true);
    const DirectRun rib = run_direct(plan, false, WipeTowerWallType::wtwRib, true);
    require_finite_run(rectangle);
    require_finite_run(rib);
    CHECK(total_extrusion(rectangle) > 0.f);
    CHECK(total_extrusion(rib) > 0.f);
    const bool geometry_diff = std::abs(rectangle.width - rib.width) > 1e-3f ||
                               std::abs(rectangle.depth - rib.depth) > 1e-3f ||
                               std::abs(total_extrusion(rectangle) - total_extrusion(rib)) > 1e-3f;
    CHECK(geometry_diff);
}

TEST_CASE("Wipe tower preserves nonuniform planned layer heights in solid results", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.1f, 0, 1, 80.f},
        {0.3f, 0.3f, 1, 0, 80.f},
        {0.6f, 0.15f, 0, 1, 80.f},
        {0.75f, 0.25f, 1, 0, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true);
    require_finite_run(run);
    REQUIRE(run.layers.size() == 4);
    for (std::size_t i = 0; i < run.layers.size(); ++i) {
        REQUIRE_FALSE(run.layers[i].empty());
        CHECK_THAT(run.layers[i].front().print_z, WithinAbs(std::array<float, 4>{0.2f, 0.3f, 0.6f, 0.75f}[i], 1e-4f));
        CHECK_THAT(run.layers[i].front().layer_height, WithinAbs(std::array<float, 4>{0.1f, 0.3f, 0.15f, 0.25f}[i], 1e-4f));
    }
}

TEST_CASE("Wipe tower reserves more solid geometry for a larger purge request", "[WipeTower][WipeTowerContact]")
{
    const DirectRun small = run_direct({{0.2f, 0.2f, 0, 1, 20.f}});
    const DirectRun large = run_direct({{0.2f, 0.2f, 0, 1, 160.f}});
    require_finite_run(small);
    require_finite_run(large);
    CHECK(large.depth > small.depth);
    CHECK(total_extrusion(large) > total_extrusion(small));
    CHECK(large.toolchanges == small.toolchanges);
}

TEST_CASE("Wipe tower logical filament transitions remain valid on one physical extruder", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f},
        {0.4f, 0.2f, 1, 0, 80.f},
        {0.6f, 0.2f, 0, 1, 80.f},
    }, true, WipeTowerWallType::wtwRectangle, true, false, true);
    require_finite_run(run);
    CHECK(run.toolchanges == 3);
    CHECK(nonempty_layers(run) == 3);
    CHECK(total_extrusion(run) > 0.f);
}

TEST_CASE("Disabled tower interfaces omit temperature waits and pre-extrusion", "[WipeTower][WipeTowerContact]")
{
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f}, {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f}, {0.8f, 0.2f, 1, 0, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true, false, false, false);
    require_finite_run(run);
    for (const auto &layer : run.layers)
        for (const auto &change : layer) {
            const GridStats stats = grid_stats(change);
            CHECK(stats.m109_count == 0);
            CHECK(stats.m104_count == 0);
            CHECK_FALSE(has_interface_pre_extrusion_move(change.gcode));
        }
}

TEST_CASE("Tower interface pre-extrusion follows the configured length", "[WipeTower][WipeTowerContact]")
{
    const float length = GENERATE(1.f, 5.f);
    const DirectRun run = run_direct({
        {0.2f, 0.2f, 0, 1, 80.f}, {0.4f, 0.2f, 1, 1, 80.f},
        {0.6f, 0.2f, 1, 1, 80.f}, {0.8f, 0.2f, 1, 0, 80.f},
    }, false, WipeTowerWallType::wtwRectangle, true, false, false, true, 2.f, length);
    require_finite_run(run);
    bool configured_move = false;
    for (const auto &layer : run.layers)
        for (const auto &change : layer) {
            configured_move |= has_interface_pre_extrusion_move(change.gcode, length + 2.f);
            CHECK_FALSE(has_interface_pre_extrusion_move(change.gcode, 5.f));
        }
    CHECK(configured_move);
}

}

namespace toolchange_ordering_contracts {

using namespace Slic3r;
using namespace Slic3r::Test;

TEST_CASE("Toolchange ordering preserves configured first and later layer sequences", "[WipeTower][ToolChangeOrdering][ToolOrderingVariance]")
{
    auto [ordering_name, cyclic_first_layer] = GENERATE(table<std::string, bool>({
        { "default", false },
        { "default", true  },
        { "cyclic",  false },
        { "cyclic",  true  },
    }));
    DYNAMIC_SECTION(ordering_name << " first_layer=" << (cyclic_first_layer ? "enabled" : "disabled")) {
        const bool cyclic = ordering_name == "cyclic";
        const bool expect_cyclic_first = cyclic && cyclic_first_layer;
        const ToolChangeOrderingType expected_ordering = cyclic ? ToolChangeOrderingType::Cyclic : ToolChangeOrderingType::Default;
        const std::vector<unsigned int> expected_first = expect_cyclic_first ? std::vector<unsigned int>{ 1, 0 } : std::vector<unsigned int>{ 0, 1 };
        const std::vector<unsigned int> expected_default_odd{ 1, 0 };
        const std::vector<unsigned int> expected_default_even{ 0, 1 };
        const std::vector<unsigned int> expected_cyclic_later{ 1, 0 };

        DynamicPrintConfig config = multifilament_config(2, {
            { "layer_height",                     0.4 },
            { "initial_layer_print_height",       0.2 },
            { "skin_infill_line_width",           0.5 },
            { "skeleton_infill_line_width",       0.5 },
            { "gcode_flavor",                     "marlin" },
            { "use_relative_e_distances",         true },
            { "layer_change_gcode",               "G92 E0\n" },
            { "bridge_line_width",                0.0 },
            { "wall_loops",                       2 },
            { "top_shell_layers",                 0 },
            { "bottom_shell_layers",              0 },
            { "sparse_infill_density",            0 },
            { "skirt_loops",                      0 },
            { "brim_type",                        "no_brim" },
            { "enable_prime_tower",               false },
            { "wipe_tower_filament",              0 },
            // Asymmetric but finite flush costs make the default later-layer choice non-tied.
            { "flush_volumes_matrix",              "0,100,280,0" },
            { "wipe_tower_type",                  "type1" },
            { "print_sequence",                   "by layer" },
            { "toolchange_ordering",              ordering_name },
            { "toolchange_cyclic_order",          "2,1" },
            { "toolchange_cyclic_first_layer",    cyclic_first_layer },
            { "first_layer_print_sequence",       "" },
            { "other_layers_print_sequence",      "0" },
            { "other_layers_print_sequence_nums", 0 },
        });
        const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides = {
            {},
            {
                { "outer_wall_filament_id",     2 },
                { "inner_wall_filament_id",     2 },
                { "sparse_infill_filament_id",  2 },
                { "internal_solid_filament_id", 2 },
                { "top_surface_filament_id",   2 },
                { "bottom_surface_filament_id", 2 },
            },
        };

        // The height is deliberately large enough to observe at least three successive rows; the
        // assertion still uses the processed layer count rather than assuming an exact rounded count.
        TriangleMesh large = make_cube(24., 24., 2.4);
        TriangleMesh small = make_cube(12., 12., 2.4);
        small.translate(40., 0., 0.);
        Print print;
        Model model;
        init_print(std::vector<TriangleMesh>{ std::move(large), std::move(small) }, print, model, config, &overrides, false);

        const auto validation = print.validate();
        INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
        REQUIRE(validation.string.empty());

        const PrintConfig &applied = print.config();
        CHECK(applied.toolchange_ordering == expected_ordering);
        CHECK(applied.toolchange_cyclic_order.value == "2,1");
        CHECK(applied.toolchange_cyclic_first_layer.value == cyclic_first_layer);
        CHECK(applied.wipe_tower_type == WipeTowerType::Type1);
        CHECK_FALSE(applied.enable_prime_tower);
        CHECK(applied.wipe_tower_filament.value == 0);
        CHECK(applied.first_layer_print_sequence.values.empty());
        CHECK(applied.other_layers_print_sequence.values == std::vector<int>{ 0 });
        CHECK(applied.other_layers_print_sequence_nums.value == 0);
        CHECK(applied.print_sequence == PrintSequence::ByLayer);
        CHECK(print.objects().size() == 2);

        // process() is required before layer-region and tool-ordering state is populated.
        print.process();

        size_t object_index = 0;
        size_t observed_layer_count = 0;
        for (const PrintObject *object : print.objects()) {
            const unsigned int expected_filament = static_cast<unsigned int>(object_index + 1);
            size_t positive_layers = 0;
            size_t layer_index = 0;
            for (const Layer *layer : object->layers()) {
                INFO("object=" << object_index << " layer=" << layer_index);
                REQUIRE(layer->has_extrusions());
                bool saw_expected_filament = false;
                double volume = 0.;
                for (const LayerRegion *region : layer->regions()) {
                    if (!region->has_extrusions())
                        continue;
                    volume += region->perimeters.total_volume();
                    volume += region->fills.total_volume();
                    const unsigned int wall_filament = region->region().extruder(FlowRole::frExternalPerimeter);
                    if (wall_filament == expected_filament)
                        saw_expected_filament = true;
                }
                REQUIRE(std::isfinite(volume));
                CHECK(volume > 0.);
                CHECK(saw_expected_filament);
                ++positive_layers;
                ++layer_index;
            }
            REQUIRE(positive_layers >= 2);
            if (object_index == 0)
                observed_layer_count = positive_layers;
            else
                CHECK(positive_layers == observed_layer_count);
            ++object_index;
        }
        REQUIRE(observed_layer_count >= 3);

        const ToolOrdering &ordering = print.tool_ordering();
        std::vector<std::vector<unsigned int>> sequences;
        for (const LayerTools &layer_tools : ordering) {
            REQUIRE(layer_tools.extruders.size() == 2);
            CHECK(layer_tools.has_extruder(0));
            CHECK(layer_tools.has_extruder(1));
            sequences.push_back(layer_tools.extruders);
        }
        REQUIRE(sequences.size() == observed_layer_count);
        CHECK(sequences.front() == expected_first);
        for (size_t layer_index = 1; layer_index < sequences.size(); ++layer_index) {
            if (cyclic)
                CHECK(sequences[layer_index] == expected_cyclic_later);
            else if (layer_index % 2 == 1)
                CHECK(sequences[layer_index] == expected_default_odd);
            else
                CHECK(sequences[layer_index] == expected_default_even);
        }
    }
}

} // namespace toolchange_ordering_contracts

namespace tower_contact_pipeline_contracts {

namespace {

struct PipelineTrace {
    std::size_t tower_blocks = 0;
    std::size_t tower_extrusions = 0;
    std::size_t interface_waits = 0;
    std::size_t targeted_interface_waits = 0;
    std::size_t correctly_targeted_interface_waits = 0;
    std::size_t interface_restores = 0;
    std::size_t pre_extrusions = 0;
    std::size_t model_extrusions = 0;
    double model_xy_path = 0.;
    double model_filament = 0.;
    bool saw_incoming_tool_wait = false;
};

DynamicPrintConfig contact_pipeline_config(bool enabled)
{
    return multifilament_config(2, {
        { "nozzle_diameter",                            "0.4,0.4" },
        { "printer_extruder_id",                        "1,2" },
        { "printer_extruder_variant",                   "Direct Drive Standard,Direct Drive Standard" },
        { "extruder_printable_height",                  "0,0" },
        { "nozzle_temperature_initial_layer",           "200,200" },
        { "nozzle_temperature",                         "200,200" },
        { "filament_tower_interface_print_temp",       "215,220" },
        { "filament_tower_interface_pre_extrusion_dist", "2,2" },
        { "filament_tower_interface_pre_extrusion_length", "3,3" },
        // Keep the public tool-change template's incoming-tool wait as the one
        // surviving M109.  append_tcr2 must remove the duplicate untagged wait
        // emitted by the contact tower itself when this option is off.
        { "change_filament_gcode",                         "T[next_extruder]\nM109 S[new_filament_temp] T[next_extruder]\n" },
        { "enable_tower_interface_features",           enabled },
        { "enable_tower_interface_cooldown_during_tower", false },
        { "enable_filament_ramming",                   false },
        { "enable_prime_tower",                         true },
        { "wipe_tower_type",                            "type2" },
        { "prime_tower_width",                          35 },
        { "wipe_tower_x",                               50 },
        { "wipe_tower_y",                               50 },
        { "prime_volume",                               80 },
        { "wipe_tower_no_sparse_layers",                true },
        { "wipe_tower_sparse_layers_combination",       false },
        { "wait_for_temp_on_wipe_tower",                false },
        { "preheat_time",                               0 },
        { "skirt_loops",                                0 },
        { "brim_type",                                  "no_brim" },
        { "layer_height",                               0.2 },
        { "gcode_flavor",                               "marlin" },
        { "use_relative_e_distances",                   true },
        { "layer_change_gcode",                         "G92 E0\n" },
    });
}

std::string export_contact_pipeline(const DynamicPrintConfig &config)
{
    Print print;
    Model model;
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides = {
        {{ "extruder", 1 }},
        {{ "extruder", 2 }},
    };
    init_print(std::vector<TriangleMesh>{ make_cube(6., 6., 0.6), make_cube(6., 6., 0.6) },
               print, model, config, &overrides);
    const auto validation = print.validate();
    INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    return gcode(print);
}

PipelineTrace trace_contact_pipeline(const std::string &gcode, const DynamicPrintConfig &config)
{
    PipelineTrace result;
    GCodeReader reader;
    reader.apply_config(config);
    bool in_tower = false;
    bool before_wipe = false;
    int current_tool = -1;
    reader.parse_buffer(gcode, [&result, &in_tower, &before_wipe, &current_tool](GCodeReader &self,
                                                                                  const GCodeReader::GCodeLine &line) {
        const std::string raw = line.raw();
        if (raw.find("CP TOOLCHANGE START") != std::string::npos) {
            in_tower = true;
            before_wipe = true;
            ++result.tower_blocks;
        }
        if (raw.find("CP TOOLCHANGE WIPE") != std::string::npos)
            before_wipe = false;

        const std::string_view command = line.cmd();
        if (command.size() >= 2 && command[0] == 'T' && std::isdigit(static_cast<unsigned char>(command[1])))
            current_tool = std::stoi(std::string(command.substr(1)));

        if (in_tower && line.cmd_is("M109")) {
            float temperature = 0.f;
            const bool has_temperature = line.has_value('S', temperature) && std::isfinite(temperature);
            const bool is_interface_wait = has_temperature &&
                                           (std::abs(temperature - 215.f) <= 1e-4f ||
                                            std::abs(temperature - 220.f) <= 1e-4f);
            if (is_interface_wait) {
                ++result.interface_waits;
                float target_tool_value = 0.f;
                if (line.has_value('T', target_tool_value) && std::isfinite(target_tool_value)) {
                    const int target_tool = int(std::lround(target_tool_value));
                    if (std::abs(target_tool_value - float(target_tool)) <= 1e-4f) {
                        ++result.targeted_interface_waits;
                        const int expected_temperature = target_tool == 0 ? 215 : (target_tool == 1 ? 220 : -1);
                        if (target_tool == current_tool && expected_temperature > 0 &&
                            std::abs(temperature - float(expected_temperature)) <= 1e-4f)
                            ++result.correctly_targeted_interface_waits;
                        if (target_tool == 1)
                            result.saw_incoming_tool_wait = true;
                    }
                }
            }
        }
        if (in_tower && !before_wipe && line.cmd_is("M104")) {
            float temperature = 0.f;
            if (line.has_value('S', temperature) && std::isfinite(temperature) &&
                std::abs(temperature - 200.f) <= 1e-4f)
                ++result.interface_restores;
        }

        if (line.extruding(self) && line.dist_XY(self) > 0.f) {
            if (in_tower) {
                ++result.tower_extrusions;
                if (before_wipe && line.dist_E(self) > 2.5f && line.dist_E(self) < 3.5f)
                    ++result.pre_extrusions;
            } else {
                ++result.model_extrusions;
                result.model_xy_path += line.dist_XY(self);
                result.model_filament += line.dist_E(self);
            }
        }

        if (raw.find("CP TOOLCHANGE END") != std::string::npos) {
            in_tower = false;
            before_wipe = false;
        }
    });
    return result;
}

} // namespace

TEST_CASE("Tower contact temperature override and restore reach exported type-2 tower", "[WipeTower][WipeTowerContact][WipeTowerContactPipeline]")
{
    const DynamicPrintConfig enabled_config = contact_pipeline_config(true);
    const DynamicPrintConfig disabled_config = contact_pipeline_config(false);
    const PipelineTrace enabled = trace_contact_pipeline(export_contact_pipeline(enabled_config), enabled_config);
    const PipelineTrace disabled = trace_contact_pipeline(export_contact_pipeline(disabled_config), disabled_config);

    REQUIRE(enabled.tower_blocks > 0);
    REQUIRE(disabled.tower_blocks == enabled.tower_blocks);
    REQUIRE(enabled.tower_extrusions > 0);
    REQUIRE(disabled.tower_extrusions > 0);
    REQUIRE(enabled.model_extrusions > 0);
    REQUIRE(disabled.model_extrusions > 0);

    CHECK(enabled.interface_waits > 0);
    CHECK(enabled.saw_incoming_tool_wait);
    CHECK(enabled.interface_restores > 0);
    CHECK(enabled.pre_extrusions > 0);
    // The custom template supplies one incoming-tool wait.  The type-2
    // contact block contains a second, untagged interface wait; append_tcr2's
    // deduplication pass must remove that duplicate.
    CHECK(enabled.interface_waits == enabled.pre_extrusions);
    CHECK(enabled.targeted_interface_waits == enabled.interface_waits);
    CHECK(enabled.correctly_targeted_interface_waits == enabled.targeted_interface_waits);
    CHECK(enabled.interface_restores == enabled.pre_extrusions);
    CHECK(disabled.interface_waits == 0);
    CHECK(disabled.targeted_interface_waits == 0);
    CHECK(disabled.correctly_targeted_interface_waits == 0);
    CHECK(disabled.interface_restores == 0);
    CHECK(disabled.pre_extrusions == 0);

    CHECK(enabled.model_extrusions == disabled.model_extrusions);
    CHECK_THAT(enabled.model_xy_path, Catch::Matchers::WithinRel(disabled.model_xy_path, 1e-6));
    CHECK_THAT(enabled.model_filament, Catch::Matchers::WithinRel(disabled.model_filament, 1e-6));
}

} // namespace tower_contact_pipeline_contracts

namespace toolordering_custom_gcode_contracts {

TEST_CASE("Custom G-code events follow print mode and layer assignment", "[WipeTower][ToolOrdering][CustomGCodeAssignment]")
{
    auto run = [](bool two_objects, CustomGCode::Mode model_mode,
                  const std::vector<CustomGCode::Type> &expected_types) {
        DynamicPrintConfig config = multifilament_config(2, {
            { "layer_height",                 0.2 },
            { "initial_layer_print_height",    0.2 },
            { "wall_loops",                   1 },
            { "outer_wall_filament_id",        1 },
            { "inner_wall_filament_id",        1 },
            { "sparse_infill_density",        0 },
            { "top_shell_layers",              0 },
            { "bottom_shell_layers",           0 },
            { "enable_support",            false },
            { "skirt_loops",                   0 },
            { "brim_type",              "no_brim" },
            { "enable_prime_tower",        false },
            { "print_sequence",       "by layer" },
            { "use_relative_e_distances",  true },
            { "before_layer_change_gcode", "G92 E0\n" },
        });

        Print print;
        Model model;
        if (two_objects) {
            const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides = {
                {},
                {{ "outer_wall_filament_id", 2 }},
            };
            TriangleMesh second = make_cube(4., 4., 0.8);
            second.translate(8., 0., 0.);
            init_print(std::vector<TriangleMesh>{ make_cube(4., 4., 0.8), std::move(second) }, print, model, config, &overrides, false);
        } else {
            init_print({ make_cube(4., 4., 0.8) }, print, model, config);
        }

        CustomGCode::Info info;
        info.mode = model_mode;
        info.gcodes = {
            { 0.21, CustomGCode::PausePrint, 1, "pause", "pause" },
            { 0.41, CustomGCode::ColorChange, 2, "#00ff00", "color" },
            { 0.79, CustomGCode::Custom, 1, "", "M117 done" },
        };
        model.plates_custom_gcodes[model.curr_plate_index] = info;

        // init_print applied the model before the plate events existed. Reapply
        // so Print's owned model copy receives the same custom-code data.
        print.apply(model, config);
        REQUIRE(print.config().print_sequence == PrintSequence::ByLayer);
        REQUIRE(print.object_extruders().size() == (two_objects ? 2 : 1));
        REQUIRE(print.model().get_curr_plate_custom_gcodes() == info);
        const auto validation = print.validate();
        INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
        REQUIRE(validation.string.empty());

        print.process();
        // Print::process uses a local ToolOrdering copy for this assignment.
        // Reuse that production ordering state and invoke the same public method
        // to make the assignment result directly inspectable here.
        ToolOrdering ordering = print.tool_ordering();
        ordering.assign_custom_gcodes(print);
        size_t layer_index = 0;
        for (const LayerTools &layer : ordering) {
            REQUIRE(layer_index < expected_types.size());
            CHECK_THAT(double(layer.print_z), Catch::Matchers::WithinAbs(0.2 + 0.2 * double(layer_index), 1e-6));
            const CustomGCode::Item *event = layer.custom_gcode;
            if (expected_types[layer_index] == CustomGCode::Unknown) {
                CHECK(event == nullptr);
                ++layer_index;
                continue;
            }
            REQUIRE(event != nullptr);
            CHECK(event->type == expected_types[layer_index]);
            if (expected_types[layer_index] == CustomGCode::ColorChange) {
                CHECK(event->extruder == 2);
                CHECK(event->color == "#00ff00");
            }
            if (expected_types[layer_index] == CustomGCode::PausePrint)
                CHECK(event->extra == "pause");
            if (expected_types[layer_index] == CustomGCode::Custom)
                CHECK(event->extra == "M117 done");
            ++layer_index;
        }
        REQUIRE(layer_index == expected_types.size());
    };

    // Two filament slots are configured, but one object uses only slot 1: an active color change
    // for filament 2 is ignored, while pause/custom events remain unconditional.
    run(false, CustomGCode::MultiAsSingle,
        { CustomGCode::PausePrint, CustomGCode::Unknown,
          CustomGCode::Unknown, CustomGCode::Custom });

    // Two objects print above the color-change layer with filament 2. The same
    // event is therefore applied, while the third layer remains an empty control.
    run(true, CustomGCode::MultiExtruder,
        { CustomGCode::PausePrint, CustomGCode::ColorChange,
          CustomGCode::Unknown, CustomGCode::Custom });

    // A model-mode/print-mode mismatch is a negative control for color changes,
    // while the mode-independent events remain assigned.
    run(true, CustomGCode::MultiAsSingle,
        { CustomGCode::PausePrint, CustomGCode::Unknown,
          CustomGCode::Unknown, CustomGCode::Custom });
}

} // namespace toolordering_custom_gcode_contracts

namespace wipe_tower_prime_public_contracts {
using namespace Slic3r;
using namespace Slic3r::Test;

struct PrimeRun
{
    std::vector<WipeTower::ToolChangeResult> results;
    std::vector<float> used_filament;
    std::string gcode;
    int toolchanges = 0;
};

static std::size_t count_token(const std::string &text, const std::string &token)
{
    std::size_t count = 0;
    std::size_t pos = 0;
    while ((pos = text.find(token, pos)) != std::string::npos) {
        ++count;
        pos += token.size();
    }
    return count;
}

static std::string section_between(const std::string &text,
                                   const std::string &begin,
                                   const std::string &end)
{
    const std::size_t begin_pos = text.find(begin);
    if (begin_pos == std::string::npos)
        return {};
    const std::size_t content_pos = begin_pos + begin.size();
    const std::size_t end_pos = text.find(end, content_pos);
    if (end_pos == std::string::npos)
        return {};
    return text.substr(content_pos, end_pos - content_pos);
}

static bool has_extrusion_motion(const std::string &text)
{
    bool found = false;
    GCodeReader reader;
    reader.parse_buffer(text, [&found](GCodeReader &, const GCodeReader::GCodeLine &line) {
        if (!line.cmd_is("G1") || !line.has_e())
            return;
        const float e = line.e();
        if (std::isfinite(double(e)) && e > 0.f)
            found = true;
    });
    return found;
}

static bool has_positive_wait(const std::string &text, float expected_seconds)
{
    bool found = false;
    GCodeReader reader;
    reader.parse_buffer(text, [expected_seconds, &found](GCodeReader &, const GCodeReader::GCodeLine &line) {
        float seconds = 0.f;
        if (!line.cmd_is("G4") || !line.has_value('S', seconds))
            return;
        if (std::isfinite(double(seconds)) && seconds > 0.f &&
            std::abs(seconds - expected_seconds) < 0.01f)
            found = true;
    });
    return found;
}

static DynamicPrintConfig prime_dynamic(unsigned int filament_count,
                                        bool semm,
                                        bool mk4_mmu,
                                        bool high_current,
                                        bool circular_bed = false)
{
    const std::string n = filament_count == 3 ? "0,0,0" : "0,0";
    const std::string cooling_moves = semm ? (filament_count == 3 ? "2,2,2" : "2,2") : n;
    const std::string temperatures = filament_count == 3 ? "200,205,210" : "200,205";
    const std::string printable_area = circular_bed
        ? "110x0,77.781x77.781,0x110,-77.781x77.781,-110x0,-77.781x-77.781,0x-110,77.781x-77.781"
        : "0x0,220x0,220x220,0x220";
    DynamicPrintConfig config = multifilament_config(filament_count, {
        {"single_extruder_multi_material", semm},
        {"enable_prime_tower", true},
        {"purge_in_prime_tower", true},
        {"prime_tower_width", 35},
        {"prime_tower_brim_width", 0},
        {"wipe_tower_x", 20},
        {"wipe_tower_y", 20},
        {"wipe_tower_wall_type", "rectangle"},
        {"prime_tower_skip_points", false},
        {"wipe_tower_no_sparse_layers", false},
        {"wipe_tower_extra_flow", 100},
        {"wipe_tower_extra_spacing", 100},
        {"wipe_tower_bridging", 10},
        {"wipe_tower_max_purge_speed", 90},
        {"enable_arc_fitting", false},
        {"gcode_flavor", "marlin"},
        {"printable_area", printable_area},
        {"filament_soluble", n},
        {"filament_is_support", n},
        {"filament_minimal_purge_on_wipe_tower", n},
        {"filament_cooling_moves", cooling_moves},
        {"filament_cooling_initial_speed", filament_count == 3 ? "2,2,2" : "2,2"},
        {"filament_cooling_final_speed", filament_count == 3 ? "6,6,6" : "6,6"},
        {"filament_ramming_parameters", "100 100 30 30 30 30"},
        {"filament_diameter", filament_count == 3 ? "1.75,1.75,1.75" : "1.75,1.75"},
        {"nozzle_diameter", filament_count == 3 ? "0.4,0.4,0.4" : "0.4,0.4"},
        {"nozzle_temperature", temperatures},
        {"nozzle_temperature_initial_layer", temperatures},
        {"nozzle_temperature_range_high", filament_count == 3 ? "220,225,230" : "220,225"},
        {"cooling_tube_length", 40},
        {"cooling_tube_retraction", 10},
        {"parking_pos_retraction", semm ? 4 : 0},
        {"extra_loading_move", semm ? 8 : 0},
        {"filament_loading_speed", filament_count == 3 ? "12,12,12" : "12,12"},
        {"filament_loading_speed_start", filament_count == 3 ? "12,12,12" : "12,12"},
        {"filament_unloading_speed", filament_count == 3 ? "35,35,35" : "35,35"},
        {"filament_unloading_speed_start", filament_count == 3 ? "55,55,55" : "55,55"},
        {"filament_toolchange_delay", filament_count == 3 ? "1,1,1" : "1,1"},
        {"filament_stamping_distance", filament_count == 3 ? "0,0,0" : "0,0"},
        {"filament_stamping_loading_speed", filament_count == 3 ? "20,20,20" : "20,20"},
        {"enable_filament_ramming", semm},
        {"high_current_on_filament_swap", high_current},
        {"printer_notes", mk4_mmu ? "PRINTER_MODEL_MK4 MMU" : ""},
    });
    return config;
}

static PrimeRun run_prime(const DynamicPrintConfig &dynamic,
                          const std::vector<unsigned int> &tools)
{
    FullPrintConfig full;
    full.apply(dynamic);
    std::vector<std::vector<float>> wipe_volumes(tools.size(), std::vector<float>(tools.size(), 0.f));
    for (std::size_t from = 0; from < tools.size(); ++from)
        for (std::size_t to = 0; to < tools.size(); ++to)
            if (from != to)
                wipe_volumes[from][to] = 40.f + float(10 * from + to);

    WipeTower2 tower(full, static_cast<const PrintRegionConfig &>(full), 0, Vec3d::Zero(), wipe_volumes, tools.front());
    for (unsigned int tool : tools)
        tower.set_extruder(tool, full);

    PrimeRun run;
    run.results = tower.prime(0.2f, tools, false);
    run.used_filament = tower.get_used_filament();
    run.toolchanges = tower.get_number_of_toolchanges();
    for (const auto &result : run.results)
        run.gcode += result.gcode;
    return run;
}

static double positive_result_length(const WipeTower::ToolChangeResult &result)
{
    double length = 0.;
    for (std::size_t j = 1; j < result.extrusions.size(); ++j)
        if (result.extrusions[j].width > 0.f)
            length += double((result.extrusions[j].pos - result.extrusions[j - 1].pos).norm());
    return length;
}

static void require_prime_contract(const PrimeRun &run,
                                   const std::vector<unsigned int> &tools)
{
    REQUIRE(run.results.size() == tools.size());
    CHECK(run.toolchanges == int(tools.size()));
    REQUIRE_FALSE(run.gcode.empty());
    REQUIRE(run.gcode.find("; CP PRIMING START") != std::string::npos);
    REQUIRE(run.gcode.find("; CP PRIMING END") != std::string::npos);
    CHECK(count_token(run.gcode, "; CP PRIMING START") == 1);
    CHECK(count_token(run.gcode, "; CP PRIMING END") == 1);

    double total_length = 0.;
    for (std::size_t i = 0; i < run.results.size(); ++i) {
        const auto &result = run.results[i];
        REQUIRE(result.priming);
        CHECK_FALSE(result.is_tool_change);
        CHECK(result.initial_tool == int(i == 0 ? tools.front() : tools[i - 1]));
        CHECK(result.new_tool == int(tools[i]));
        CHECK(std::isfinite(double(result.print_z)));
        CHECK(std::isfinite(double(result.layer_height)));
        CHECK(result.layer_height > 0.f);
        CHECK(std::isfinite(double(result.elapsed_time)));
        CHECK(result.elapsed_time > 0.f);
        CHECK(std::isfinite(double(result.start_pos.x())));
        CHECK(std::isfinite(double(result.start_pos.y())));
        CHECK(std::isfinite(double(result.end_pos.x())));
        CHECK(std::isfinite(double(result.end_pos.y())));

        const double result_length = positive_result_length(result);
        REQUIRE(result_length > 0.);
        total_length += result_length;
        bool has_positive_extrusion = false;
        for (const auto &extrusion : result.extrusions) {
            CHECK(std::isfinite(double(extrusion.pos.x())));
            CHECK(std::isfinite(double(extrusion.pos.y())));
            CHECK(std::isfinite(double(extrusion.width)));
            if (extrusion.width > 0.f) {
                has_positive_extrusion = true;
                CHECK(extrusion.tool == tools[i]);
            }
        }
        CHECK(has_positive_extrusion);
    }
    CHECK(total_length > 0.);

    REQUIRE(run.used_filament.size() >= tools.size());
    for (unsigned int tool : tools) {
        CHECK(std::isfinite(double(run.used_filament[tool])));
        CHECK(run.used_filament[tool] > 0.f);
    }
}

static void require_circular_bed_coordinates(const PrimeRun &run)
{
    bool finite = true;
    bool inside = true;
    std::size_t coordinate_moves = 0;
    GCodeReader reader;
    reader.parse_buffer(run.gcode, [&finite, &inside, &coordinate_moves](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (!(line.cmd_is("G0") || line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) ||
            (!line.has_x() && !line.has_y()))
            return;
        const float x = line.new_X(self);
        const float y = line.new_Y(self);
        if (!std::isfinite(double(x)) || !std::isfinite(double(y))) {
            finite = false;
            return;
        }
        ++coordinate_moves;
        if (double(x) * double(x) + double(y) * double(y) > 110.0 * 110.0 + 1.0)
            inside = false;
    });
    REQUIRE(coordinate_moves > 0);
    CHECK(finite);
    CHECK(inside);
}

TEST_CASE("Circular bed priming keeps emitted paths inside the configured radius",
          "[WipeTower][WipeTowerPrimePublic]")
{
    const PrimeRun run = run_prime(prime_dynamic(2, false, false, false, true), {0, 1});
    require_prime_contract(run, {0, 1});
    require_circular_bed_coordinates(run);
}

TEST_CASE("Public priming emits one finite result for each requested tool",
          "[WipeTower][WipeTowerPrimePublic]")
{
    const PrimeRun run = run_prime(prime_dynamic(2, false, false, false), {0, 1});
    require_prime_contract(run, {0, 1});
    CHECK(run.gcode.find("; CP TOOLCHANGE LOAD") == std::string::npos);
    CHECK(run.gcode.find("; CP TOOLCHANGE UNLOAD") != std::string::npos);
    CHECK(run.gcode.find("; CP TOOLCHANGE WIPE") != std::string::npos);
    CHECK(run.gcode.find("M591") == std::string::npos);
    CHECK(run.gcode.find("M907") == std::string::npos);

    // The final result must contain a real positive purge in its own WIPE section.
    // This is the witness for the final-wipe-volume mutation; generic LOAD or
    // earlier unload motion must not satisfy it.
    // prime() terminates the final result with CP PRIMING END; it does not emit
    // the per-toolchange END marker used by tool_change().
    const std::string final_wipe = section_between(run.results.back().gcode,
                                                   "; CP TOOLCHANGE WIPE\n",
                                                   "; CP PRIMING END\n");
    REQUIRE_FALSE(final_wipe.empty());
    CHECK(has_extrusion_motion(final_wipe));

    // With the pinned 0.4 mm nozzle, 1.25 width ratio, 0.2 mm layer and
    // configured 41 mm^3 pairwise volume, the requested WIPE path is about
    // 410 mm before row-turn overhead. A zero-volume mutant still emits one
    // mandatory row, but cannot meet this independent lower bound.
    constexpr double requested_wipe_volume = 41.0;
    constexpr double configured_line_area = 0.4 * 1.25 * 0.2;
    CHECK(positive_result_length(run.results.back()) >
          0.45 * requested_wipe_volume / configured_line_area);
}

TEST_CASE("Single extruder priming emits cooling and loading phases for each handoff",
          "[WipeTower][WipeTowerPrimePublic]")
{
    const PrimeRun run = run_prime(prime_dynamic(2, true, false, false), {0, 1});
    require_prime_contract(run, {0, 1});
    CHECK(run.gcode.find("; CP TOOLCHANGE LOAD") != std::string::npos);
    CHECK(run.gcode.find("; Cooling\n") != std::string::npos);
    CHECK(run.gcode.find("; Cooling park\n") != std::string::npos);
    CHECK(run.gcode.find("M591") == std::string::npos);

    // A configured 1 s unload/cooling delay must be emitted inside the
    // TOOLCHANGE section. The final planner flush emits G4 S0 and is not enough.
    const std::string unload_section = section_between(run.gcode,
                                                       "; CP TOOLCHANGE UNLOAD\n",
                                                       "; CP PRIMING END\n");
    REQUIRE_FALSE(unload_section.empty());
    CHECK(unload_section.find("; Cooling\n") != std::string::npos);
    CHECK(unload_section.find("; Cooling park\n") != std::string::npos);
    CHECK(has_positive_wait(unload_section, 1.f));
}

TEST_CASE("MK4 MMU priming brackets unloads with monitoring and cooling commands",
          "[WipeTower][WipeTowerPrimePublic]")
{
    const PrimeRun run = run_prime(prime_dynamic(3, true, true, true), {0, 1, 2});
    require_prime_contract(run, {0, 1, 2});
    CHECK(count_token(run.gcode, "M591 S0") == 2);
    CHECK(count_token(run.gcode, "M591 R") == 3);

    // Each unload must disable monitoring before the next monitoring restore.
    // This pairs the commands by stage instead of accepting aggregate counts.
    std::size_t monitor_cursor = 0;
    for (int unload_index = 0; unload_index < 2; ++unload_index) {
        const std::size_t disable = run.gcode.find("M591 S0", monitor_cursor);
        REQUIRE(disable != std::string::npos);
        const std::size_t restore = run.gcode.find("M591 R", disable);
        REQUIRE(restore != std::string::npos);
        CHECK(disable < restore);
        monitor_cursor = restore + std::string("M591 R").size();
    }
    const std::string unload_section = section_between(run.gcode,
                                                       "; CP TOOLCHANGE UNLOAD\n",
                                                       "; CP PRIMING END\n");
    REQUIRE_FALSE(unload_section.empty());
    CHECK(has_positive_wait(unload_section, 1.5f));
    CHECK(run.gcode.find("M907 E750") != std::string::npos);
    CHECK(run.gcode.find("M907 E550") != std::string::npos);
}
} // namespace wipe_tower_prime_public_contracts


namespace toolordering_active_gap_contracts {

namespace {

struct ToolOrderingGapEvidence {
    std::vector<unsigned int> first_layer_extruders;
    std::vector<unsigned int> all_extruders;
    size_t object_layer_count = 0;
    size_t tower_layer_count = 0;
    size_t dedicated_tower_layer_count = 0;
};

static DynamicPrintConfig tool_ordering_gap_config(bool dedicated_wipe_filament, bool soluble_second_filament)
{
    return multifilament_config(3, {
        { "enable_prime_tower",                 true },
        { "wipe_tower_filament",                dedicated_wipe_filament ? 3 : 0 },
        { "wipe_tower_type",                    "type2" },
        { "wipe_tower_wall_type",               "rectangle" },
        { "wipe_tower_x",                       50.0 },
        { "wipe_tower_y",                       50.0 },
        { "prime_tower_width",                  35.0 },
        { "prime_volume",                       20.0 },
        { "purge_in_prime_tower",               false },
        { "single_extruder_multi_material",     false },
        { "printable_area",                     "0x0,200x0,200x200,0x200" },
        { "layer_height",                       0.2 },
        { "initial_layer_print_height",         0.2 },
        { "wall_loops",                         1 },
        { "sparse_infill_density",              0 },
        { "top_shell_layers",                   0 },
        { "bottom_shell_layers",                0 },
        { "skirt_loops",                        0 },
        { "brim_width",                         0.0 },
        { "raft_layers",                        0 },
        { "spiral_mode",                        false },
        { "belt_printer",                       false },
        { "timelapse_type",                     0 },
        { "enable_wrapping_detection",          false },
        { "wipe_tower_no_sparse_layers",        false },
        { "wipe_tower_sparse_layers_combination", false },
        { "filament_soluble",                   soluble_second_filament ? "0,1,0" : "0,0,0" },
        { "filament_is_support",                "0,0,0" },
        { "use_relative_e_distances",           true },
        { "layer_change_gcode",                 "G92 E0\n" },
    });
}

static ToolOrderingGapEvidence run_tool_ordering_gap_case(bool dedicated_wipe_filament, bool soluble_second_filament, bool finalize_partitions)
{
    DynamicPrintConfig config = tool_ordering_gap_config(dedicated_wipe_filament, soluble_second_filament);
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides{
        {},
        { { "outer_wall_filament_id", 2 }, { "inner_wall_filament_id", 2 },
          { "sparse_infill_filament_id", 2 }, { "internal_solid_filament_id", 2 },
          { "top_surface_filament_id", 2 }, { "bottom_surface_filament_id", 2 } }
    };

    TriangleMesh second = cube(8.0);
    second.translate(12.0, 0.0, 0.0);
    Print print;
    Model model;
    init_print(std::vector<TriangleMesh>{ cube(8.0), std::move(second) }, print, model, config, &overrides, false);
    // Keep the explicit second apply: the first apply may normalize a prime tower before
    // per-object material assignments are reflected in the print's used-filament set.
    print.apply(model, config);
    REQUIRE(print.objects().size() == 2);
    REQUIRE(print.objects()[0] != nullptr);
    REQUIRE(print.objects()[1] != nullptr);
    REQUIRE(print.config().filament_diameter.values.size() == 3);
    REQUIRE(print.has_wipe_tower());
    const StringObjectException validation = print.validate();
    INFO("Print validation: " << validation.string << " [" << validation.opt_key << "]");
    REQUIRE(validation.string.empty());
    print.process();
    REQUIRE_FALSE(print.objects()[0]->layers().empty());
    REQUIRE_FALSE(print.objects()[1]->layers().empty());

    // Passing first_extruder explicitly selects the public overload that performs the
    // soluble-first-layer preference; the optional sort then executes the public
    // partition/insertion route for the dedicated tower filament.
    ToolOrdering ordering(print, 0);
    if (finalize_partitions)
        ordering.sort_and_build_data(print, 0);
    REQUIRE_FALSE(ordering.empty());

    ToolOrderingGapEvidence evidence;
    evidence.first_layer_extruders = ordering.front().extruders;
    evidence.all_extruders = ordering.all_extruders();
    for (const LayerTools &layer_tools : ordering) {
        if (layer_tools.has_object)
            ++evidence.object_layer_count;
        if (layer_tools.has_wipe_tower && layer_tools.wipe_tower_partitions > 0) {
            ++evidence.tower_layer_count;
            if (layer_tools.has_extruder(2))
                ++evidence.dedicated_tower_layer_count;
        }
    }
    return evidence;
}

} // namespace

TEST_CASE("A configured dedicated filament joins active wipe-tower layers",
          "[WipeTower][ToolOrdering][ToolOrderingGap]")
{
    const auto dedicated = run_tool_ordering_gap_case(true, false, true);
    const auto ordinary = run_tool_ordering_gap_case(false, false, true);

    REQUIRE(dedicated.object_layer_count > 0);
    REQUIRE(ordinary.object_layer_count > 0);
    REQUIRE(dedicated.tower_layer_count > 0);
    REQUIRE(ordinary.tower_layer_count > 0);
    CHECK(dedicated.dedicated_tower_layer_count > 0);
    CHECK(ordinary.dedicated_tower_layer_count == 0);
    CHECK(std::find(dedicated.all_extruders.begin(), dedicated.all_extruders.end(), 0) != dedicated.all_extruders.end());
    CHECK(std::find(dedicated.all_extruders.begin(), dedicated.all_extruders.end(), 1) != dedicated.all_extruders.end());
    CHECK(std::find(dedicated.all_extruders.begin(), dedicated.all_extruders.end(), 2) != dedicated.all_extruders.end());
    CHECK(std::find(ordinary.all_extruders.begin(), ordinary.all_extruders.end(), 2) == ordinary.all_extruders.end());
}

TEST_CASE("A soluble filament is preferred at the start of explicit first-layer ordering",
          "[WipeTower][ToolOrdering][ToolOrderingGap]")
{
    const auto ordinary = run_tool_ordering_gap_case(false, false, false);
    const auto soluble = run_tool_ordering_gap_case(false, true, false);

    REQUIRE(ordinary.first_layer_extruders.size() == 2);
    REQUIRE(soluble.first_layer_extruders.size() == 2);
    CHECK(ordinary.first_layer_extruders.front() == 0);
    CHECK(soluble.first_layer_extruders.front() == 1);
    CHECK(ordinary.first_layer_extruders[1] == 1);
    CHECK(soluble.first_layer_extruders[1] == 0);
}

} // namespace toolordering_active_gap_contracts
