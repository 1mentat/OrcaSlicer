#include <catch2/catch_all.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/GCode/TimelapsePosPicker.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintBase.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "test_helpers.hpp"

#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cstddef>
#include <utility>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

DynamicPrintConfig picker_config(double clearance_radius, double rod_height,
                                 const std::vector<Vec2ds> &extruder_areas = {},
                                 bool two_nozzles = false)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups(extruder_areas));
    config.set_deserialize_strict({
        {"printable_area", "0x0,220x0,220x220,0x220"},
        {"print_sequence", "by object"},
        {"timelapse_type", "0"},
        {"extruder_clearance_radius", clearance_radius},
        {"extruder_clearance_height_to_rod", rod_height},
        {"enable_prime_tower", false},
        {"skirt_loops", 0},
        {"raft_layers", 0},
        {"brim_type", "no_brim"},
    });
    if (two_nozzles) {
        config.set_num_extruders(2);
        config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.4}));
    }
    return config;
}

void init_picker_print(Print &print, Model &model, std::vector<TriangleMesh> meshes,
                       const DynamicPrintConfig &config)
{
    std::vector<BoundingBoxf3> input_bounds;
    input_bounds.reserve(meshes.size());
    for (const TriangleMesh &mesh : meshes)
        input_bounds.emplace_back(mesh.bounding_box());

    Test::init_print(std::move(meshes), print, model, config, nullptr, false);
    const auto error = print.validate();
    INFO("Print validation error: " << error.string << " [" << error.opt_key << "]");
    REQUIRE(error.string.empty());
    REQUIRE(model.objects.size() == input_bounds.size());
    for (size_t idx = 0; idx < input_bounds.size(); ++idx) {
        const BoundingBoxf3 actual = model.objects[idx]->instance_bounding_box(0);
        const BoundingBoxf3 &expected = input_bounds[idx];
        CHECK_THAT(actual.min.x(), WithinAbs(expected.min.x(), 1e-5));
        CHECK_THAT(actual.min.y(), WithinAbs(expected.min.y(), 1e-5));
        CHECK_THAT(actual.min.z(), WithinAbs(expected.min.z(), 1e-5));
        CHECK_THAT(actual.max.x(), WithinAbs(expected.max.x(), 1e-5));
        CHECK_THAT(actual.max.y(), WithinAbs(expected.max.y(), 1e-5));
        CHECK_THAT(actual.max.z(), WithinAbs(expected.max.z(), 1e-5));
    }
}

TriangleMesh placed_cube(double x, double y, double height)
{
    TriangleMesh mesh = make_cube(10., 10., height);
    mesh.translate(x, y, 0.);
    return mesh;
}

Layer *context_layer(PrintObject *object, double print_z = 0.2)
{
    return object->add_layer(0, print_z, print_z, print_z / 2.);
}

bool outside_xy_box(const Point &point, double min_x, double min_y,
                    double max_x, double max_y, double margin)
{
    return point.x() <= min_x - margin || point.x() >= max_x + margin ||
           point.y() <= min_y - margin || point.y() >= max_y + margin;
}

bool is_x0_clear(double clearance_radius, double current_y)
{
    DynamicPrintConfig config = picker_config(clearance_radius, 2.);
    Print print;
    Model model;
    init_picker_print(print, model,
                      {placed_cube(20., 0., 10.), placed_cube(100., 0., 1.)}, config);
    const auto &objects = print.objects();
    REQUIRE(objects.size() == 2);

    TimelapsePosPicker picker;
    picker.init(&print, Point(0, 0));
    PosPickCtx ctx;
    ctx.curr_pos = Point(scale_(100.), scale_(current_y));
    ctx.curr_layer = context_layer(print.get_object(1));
    ctx.picture_extruder_id = 0;
    ctx.curr_extruder_id = 0;
    ctx.printed_objects = std::vector<const PrintObject *>{objects[0], objects[1]};
    return picker.get_is_clear_to_x0(ctx);
}

Point pick_by_object_at_rod_height(double rod_height)
{
    DynamicPrintConfig config = picker_config(2., rod_height);
    Print print;
    Model model;
    init_picker_print(print, model,
                      {placed_cube(20., 20., 2.), placed_cube(100., 20., 1.)}, config);
    const auto &objects = print.objects();
    REQUIRE(objects.size() == 2);

    TimelapsePosPicker picker;
    picker.init(&print, Point(0, 0));
    PosPickCtx ctx;
    ctx.curr_pos = Point(scale_(105.), scale_(25.));
    ctx.curr_layer = context_layer(print.get_object(1), 1.0);
    ctx.picture_extruder_id = 0;
    ctx.curr_extruder_id = 0;
    ctx.printed_objects = std::vector<const PrintObject *>{objects[0], objects[1]};
    return picker.pick_pos(ctx);
}

} // namespace

TEST_CASE("Timelapse X0 clearance uses the configured XY corridor", "[GCode][TimelapsePosPickerGap]")
{
    CHECK_FALSE(is_x0_clear(6., 14.));
    CHECK(is_x0_clear(6., 20.));
    CHECK(is_x0_clear(2., 14.));
}

TEST_CASE("Timelapse X0 clearance ignores the object currently being printed",
          "[GCode][TimelapsePosPickerGap]")
{
    DynamicPrintConfig config = picker_config(4., 2.);
    Print print;
    Model model;
    init_picker_print(print, model, {placed_cube(20., 20., 2.)}, config);
    const auto &objects = print.objects();
    REQUIRE(objects.size() == 1);

    TimelapsePosPicker picker;
    picker.init(&print, Point(0, 0));
    PosPickCtx ctx;
    ctx.curr_pos = Point(scale_(25.), scale_(25.));
    ctx.curr_layer = context_layer(print.get_object(0));
    ctx.picture_extruder_id = 0;
    ctx.curr_extruder_id = 0;
    ctx.printed_objects = std::vector<const PrintObject *>{objects.front()};

    CHECK(picker.get_is_clear_to_x0(ctx));
}

TEST_CASE("By-object timelapse parks only when a rod-height corridor exists",
          "[GCode][TimelapsePosPickerGap]")
{
    // The picker stores rod height as an integer. At print_z 1 mm, rod height
    // 1 mm makes the 2 mm prior object lie exactly on the collision threshold.
    // Raising it to 2 mm removes that candidate, leaving a safe-position control.
    const Point blocked = pick_by_object_at_rod_height(1.0);
    const Point clear = pick_by_object_at_rod_height(2.0);

    CHECK(blocked == DefaultTimelapsePos);
    CHECK(clear != DefaultTimelapsePos);
    CHECK(clear.x() >= 0);
    CHECK(clear.x() <= 220);
    CHECK(clear.y() >= 0);
    CHECK(clear.y() <= 220);
    CHECK(outside_xy_box(clear, 20., 20., 30., 30., 1.));
    CHECK(outside_xy_box(clear, 100., 20., 110., 30., 1.));
}

TEST_CASE("Timelapse parking honors the selected extruder printable region",
          "[GCode][TimelapsePosPickerGap]")
{
    const std::vector<Vec2ds> areas{
        {Vec2d(0., 0.), Vec2d(220., 0.), Vec2d(220., 220.), Vec2d(0., 220.)},
        {Vec2d(170., 0.), Vec2d(185., 0.), Vec2d(185., 220.), Vec2d(170., 220.)},
    };
    DynamicPrintConfig config = picker_config(2., 2., areas, true);
    config.set_deserialize_strict({{"print_sequence", "by layer"}});
    Print print;
    Model model;
    init_picker_print(print, model, {placed_cube(100., 100., 1.)}, config);
    const auto &objects = print.objects();
    REQUIRE(objects.size() == 1);
    REQUIRE(print.config().nozzle_diameter.size() == 2);
    REQUIRE(print.config().extruder_printable_area.size() == 2);

    TimelapsePosPicker picker;
    picker.init(&print, Point(0, 0));
    PosPickCtx ctx;
    ctx.curr_pos = Point(scale_(105.), scale_(105.));
    ctx.curr_layer = context_layer(print.get_object(0));
    ctx.picture_extruder_id = 1;
    ctx.curr_extruder_id = 0;
    const Point restricted_area = picker.pick_pos(ctx);
    CHECK(restricted_area != DefaultTimelapsePos);
    CHECK(restricted_area.x() >= 170);
    CHECK(restricted_area.x() <= 185);
    CHECK(restricted_area.y() >= 0);
    CHECK(restricted_area.y() <= 220);
}
