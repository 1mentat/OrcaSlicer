#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include <catch2/catch_all.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_helpers.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

TriangleMesh short_mesh(TriangleMesh mesh)
{
    mesh.scale(Vec3f(1.0f, 1.0f, 0.1f));
    return mesh;
}

TriangleMesh two_disjoint_cubes()
{
    TriangleMesh first  = short_mesh(cube(20.0));
    TriangleMesh second = short_mesh(cube(20.0));
    second.translate(35.0f, 0.0f, 0.0f);
    first.merge(second);
    return first;
}

DynamicPrintConfig body_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        { "layer_height",               0.2 },
        { "initial_layer_print_height", 0.2 },
        { "nozzle_diameter",            0.4 },
        { "wall_generator",             "classic" },
        { "top_shell_thickness",        0 },
        { "bottom_shell_thickness",     0 },
        { "elefant_foot_compensation",   0 },
        { "layer_change_gcode",         "G92 E0\n" },
        { "top_shell_layers",           0 },
        { "bottom_shell_layers",        0 },
        { "sparse_infill_density",      "20%" },
        { "wall_loops",                 2 },
        { "z_hop",                      0 },
    });
    return config;
}

struct RetractionTotals {
    double retracted = 0.0;
    double unretracted = 0.0;
    size_t retract_moves = 0;
};

RetractionTotals retraction_totals(const std::string &gcode)
{
    RetractionTotals totals;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&totals](GCodeReader &self, const GCodeReader::GCodeLine &line) {
        if (line.retracting(self)) {
            totals.retracted += -line.dist_E(self);
            ++totals.retract_moves;
        } else if ((line.cmd_is("G1") || line.cmd_is("G2") || line.cmd_is("G3")) &&
                   line.has_e() && line.dist_E(self) > 0.0f && line.dist_XY(self) <= EPSILON) {
            // An E-only positive move is the corresponding unretraction.  A positive
            // E move with XY motion is actual material deposition and is excluded.
            totals.unretracted += line.dist_E(self);
        }
    });
    return totals;
}

} // namespace

TEST_CASE("A hollow mesh keeps its hole through every processed layer", "[Pipeline][PrintObject]")
{
    Print print;
    Model model;
    DynamicPrintConfig config = body_config();
    config.set_deserialize_strict({ { "sparse_infill_density", 0 }, { "wall_loops", 2 } });
    init_print({ short_mesh(mesh(TestMesh::cube_with_hole)) }, print, model, config);
    REQUIRE(print.validate().string.empty());
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() == 5);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices.size() == 1);
        REQUIRE(layer->lslices.front().is_valid());
        REQUIRE(layer->lslices.front().holes.size() == 1);

        const LayerRegion &region = *layer->regions().front();
        // Two wall loops are made for both the outside and the hole.  This
        // catches a pipeline stage that keeps the raw hole but drops its wall.
        CHECK(region.perimeters.items_count() == 4);
    }
}

TEST_CASE("Disjoint bodies keep independent perimeters and internal infill", "[Pipeline][PrintObject]")
{
    Print print;
    Model model;
    const DynamicPrintConfig config = body_config();
    init_print({ two_disjoint_cubes() }, print, model, config);
    REQUIRE(print.validate().string.empty());
    print.process();

    REQUIRE(print.objects().size() == 1);
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() == 10);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices.size() == 2);

        const LayerRegion &region = *layer->regions().front();
        CHECK(region.perimeters.items_count() >= 4);
        const size_t internal_surfaces = static_cast<size_t>(std::count_if(
            region.fill_surfaces.surfaces.begin(), region.fill_surfaces.surfaces.end(),
            [](const Surface &surface) { return surface.is_internal(); }));
        // Each island has a sparse fill surface after top/bottom shells are disabled.
        CHECK(internal_surfaces == 2);
        CHECK(region.fills.flatten().has_infill());
    }
}

TEST_CASE("Per-object wall and infill settings stay isolated for equal meshes", "[Pipeline][PrintObject][Regression]")
{
    DynamicPrintConfig config = body_config();
    config.set_deserialize_strict({ { "sparse_infill_density", 0 }, { "wall_loops", 1 } });
    const std::vector<std::vector<ConfigBase::SetDeserializeItem>> overrides {
        {},
        {
            { "sparse_infill_density", "20%" },
            { "wall_loops",            3 },
        },
    };

    Print print;
    Model model;
    init_print(std::vector<TriangleMesh> { short_mesh(cube(20.0)), short_mesh(cube(20.0)) }, print, model,
               config, &overrides, true);
    REQUIRE(print.validate().string.empty());
    print.process();

    REQUIRE(print.objects().size() == 2);
    const PrintObject &without_infill = *print.objects()[0];
    const PrintObject &with_infill    = *print.objects()[1];
    REQUIRE(without_infill.layer_count() == with_infill.layer_count());
    REQUIRE(without_infill.layer_count() > 0);
    for (size_t i = 0; i < without_infill.layer_count(); ++i) {
        const Layer &a = *without_infill.get_layer(static_cast<int>(i));
        const Layer &b = *with_infill.get_layer(static_cast<int>(i));
        REQUIRE(a.lslices.size() == 1);
        REQUIRE(b.lslices.size() == 1);
        CHECK(a.regions().front()->perimeters.items_count() == 1);
        CHECK(b.regions().front()->perimeters.items_count() == 3);
        CHECK(a.regions().front()->fills.empty());
        CHECK(b.regions().front()->fills.flatten().has_infill());
    }
}

TEST_CASE("Configured retraction is emitted around long object travels", "[Pipeline][GCode][Regression]")
{
    Print print;
    Model model;
    place_two_cubes_apart(10.0, {
        { "layer_height",               0.3 },
        { "initial_layer_print_height", 0.3 },
        { "nozzle_diameter",            0.4 },
        { "layer_change_gcode",         "G92 E0\n" },
        { "retraction_length",           1.25 },
        { "retraction_minimum_travel",   0.5 },
        { "use_firmware_retraction",     false },
        { "wipe",                        false },
        { "retract_when_changing_layer", false },
        { "retract_before_wipe",         "0%" },
        { "retract_after_wipe",          "0%" },
        { "z_hop",                       0 },
    }, print, model);
    REQUIRE(print.validate().string.empty());
    const std::string gcode = Test::gcode(print);

    const RetractionTotals totals = retraction_totals(gcode);
    REQUIRE(totals.retract_moves > 0);
    CHECK_THAT(totals.retracted, Catch::Matchers::WithinRel(1.25 * totals.retract_moves, 0.02));
    CHECK_THAT(totals.unretracted, Catch::Matchers::WithinRel(totals.retracted, 0.02));
}
