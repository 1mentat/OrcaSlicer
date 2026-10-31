#include "libslic3r/Exception.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <initializer_list>
#include <algorithm>
#include <catch2/catch_all.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include <string>
#include "libslic3r/Config.hpp"
#include <vector>
#include <limits>
#include <cstddef>
#include <cmath>

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/MultiMaterialSegmentation.hpp"

using namespace Slic3r;
using namespace Slic3r::arrangement;

namespace {

using Catch::Matchers::WithinRel;

// Square of the given (scaled) side, lower-left at the origin. bed_idx starts at
// 0 because arrange() seeds the nester's bin from it (see ModelArrange.cpp).
ArrangePolygon make_square(coord_t side)
{
    ArrangePolygon ap;
    Polygon        p;
    p.points = {Point(0, 0), Point(side, 0), Point(side, side), Point(0, side)};
    ap.poly  = ExPolygon(p);
    ap.bed_idx = 0;
    return ap;
}

ArrangePolygons squares(int n, double side_mm, double height_mm = 0.)
{
    ArrangePolygons items;
    for (int i = 0; i < n; ++i) {
        items.emplace_back(make_square(scaled(side_mm)));
        items.back().height = height_mm;
    }
    return items;
}

// Bed [0,0]..[w,h] in scaled coordinates.
BoundingBox bed(double w_mm, double h_mm)
{
    return BoundingBox(Point(0, 0), Point(scaled(w_mm), scaled(h_mm)));
}

// The default progress callback prints to stdout; silence it.
ArrangeParams quiet_params(coord_t min_dist = 0)
{
    ArrangeParams p{min_dist};
    p.progressind = [](unsigned, std::string) {};
    return p;
}

ExPolygons placed_shapes(const ArrangePolygons &items)
{
    ExPolygons out;
    out.reserve(items.size());
    for (const ArrangePolygon &ap : items)
        out.emplace_back(ap.transformed_poly());
    return out;
}

// Area double-counted across the shapes: the sum counts overlaps twice, the
// union once, so the difference is the overlapping area (0 when disjoint).
double overlap_area(const ExPolygons &shapes)
{
    double sum = 0;
    for (const ExPolygon &e : shapes)
        sum += e.area();
    double uni = 0;
    for (const ExPolygon &e : union_ex(shapes))
        uni += e.area();
    return sum - uni;
}

// Relative tolerance absorbs the area-unit rounding the clipper union introduces.
bool disjoint(const ExPolygons &shapes)
{
    double total = 0;
    for (const ExPolygon &e : shapes)
        total += e.area();
    return overlap_area(shapes) <= total * 1e-9;
}

void require_no_overlap(const ArrangePolygons &items)
{
    REQUIRE(disjoint(placed_shapes(items)));
}

// The sequential-print floor is chosen by comparing object height against the nozzle,
// so the two are defined together and every expectation is derived from them.
constexpr double NOZZLE_HEIGHT_MM = 2.5;
constexpr double CLEARANCE_MM     = 30.;
constexpr double NOZZLE_FLOOR_MM  = MAX_OUTER_NOZZLE_DIAMETER / 2.;

ArrangeParams seq_print_params(coord_t min_dist)
{
    ArrangeParams p       = quiet_params(min_dist);
    p.is_seq_print        = true;
    p.clearance_radius    = float(CLEARANCE_MM);
    p.nozzle_height       = float(NOZZLE_HEIGHT_MM);
    p.object_skirt_offset = 0.f;
    return p;
}

// update_selected_items_inflation reads the bed out of the config to cap inflation.
DynamicPrintConfig bed_config()
{
    DynamicPrintConfig c;
    c.set_key_value("printable_area", new ConfigOptionPoints{{0, 0}, {200, 0}, {200, 200}, {0, 200}});
    return c;
}

ArrangePolygons squares_of_heights(const std::vector<double> &heights_mm)
{
    ArrangePolygons items;
    for (double height_mm : heights_mm)
        items.push_back(squares(1, 20., height_mm).front());
    return items;
}

} // namespace

// Prove the overlap check the other tests rely on actually detects overlap.
TEST_CASE("overlap_area detects overlap and ignores touching edges", "[Arrange]")
{
    auto square_at = [](double x_mm) {
        ArrangePolygon ap = make_square(scaled(20.));
        ap.translation    = Vec2crd(scaled(x_mm), 0);
        return ap.transformed_poly();
    };
    ExPolygon a = square_at(0.);

    SECTION("disjoint shapes are reported disjoint") {
        REQUIRE(disjoint({a, square_at(30.)}));
    }
    SECTION("edge-touching shapes are reported disjoint") {
        REQUIRE(disjoint({a, square_at(20.)}));
    }
    SECTION("overlapping shapes are not, and the area is measured") {
        REQUIRE_FALSE(disjoint({a, square_at(10.)}));
        REQUIRE_THAT(overlap_area({a, square_at(10.)}),
                     WithinRel(double(scaled(10.)) * scaled(20.), 1e-9)); // 10x20 mm
    }
}

TEST_CASE("Arrange places every item on the physical bed", "[Arrange]")
{
    ArrangePolygons items = squares(5, 20.);
    arrange(items, bed(200, 200), quiet_params(scaled(1.)));

    for (const ArrangePolygon &ap : items)
        REQUIRE(ap.bed_idx == 0);
}

TEST_CASE("Arranged items stay within the bed", "[Arrange]")
{
    ArrangePolygons items = squares(6, 30.);
    arrange(items, bed(200, 200), quiet_params(scaled(1.)));

    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        REQUIRE(bed(200, 200).contains(ap.transformed_poly().contour.bounding_box()));
    }
}

TEST_CASE("Arranged items do not overlap", "[Arrange]")
{
    ArrangePolygons items = squares(6, 40.);
    arrange(items, bed(250, 250), quiet_params(scaled(2.)));

    require_no_overlap(items);
}

TEST_CASE("Arrange spaces items by their inflation", "[Arrange]")
{
    // Per-item inflation is how the arranger enforces clearance (the GUI fills it
    // from min_obj_distance). Two items inflated 4mm each end up >= 8mm apart.
    ArrangePolygons items = squares(4, 20.);
    for (ArrangePolygon &ap : items)
        ap.inflation = scaled(4.);
    arrange(items, bed(200, 200), quiet_params());

    // Axis-aligned squares are their own bounding boxes, so the clearance between
    // a pair is the distance between their boxes (1mm slack for nester rounding).
    std::vector<BoundingBox> boxes;
    for (const ExPolygon &e : placed_shapes(items))
        boxes.push_back(e.contour.bounding_box());

    double min_gap = std::numeric_limits<double>::max();
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size(); ++j) {
            coord_t sx = std::max<coord_t>(0, std::max(boxes[j].min.x() - boxes[i].max.x(),
                                                       boxes[i].min.x() - boxes[j].max.x()));
            coord_t sy = std::max<coord_t>(0, std::max(boxes[j].min.y() - boxes[i].max.y(),
                                                       boxes[i].min.y() - boxes[j].max.y()));
            min_gap = std::min(min_gap, std::sqrt(double(sx) * sx + double(sy) * sy));
        }

    REQUIRE(min_gap >= double(scaled(8.)) - double(scaled(0.5)));
}

TEST_CASE("An item larger than the bed cannot be placed", "[Arrange]")
{
    ArrangePolygons items;
    items.emplace_back(make_square(scaled(20.)));
    items.emplace_back(make_square(scaled(400.))); // far bigger than the bed

    arrange(items, bed(200, 200), quiet_params(scaled(1.)));

    REQUIRE(items[0].bed_idx == 0);
    REQUIRE(items[1].bed_idx == UNARRANGED);
}

TEST_CASE("Items overflowing one bed spill onto virtual beds", "[Arrange]")
{
    ArrangePolygons items = squares(8, 90.); // eight 90mm squares cannot share a 200x200 bed
    arrange(items, bed(200, 200), quiet_params(scaled(2.)));

    int max_bed = 0;
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx >= 0); // placed somewhere
        max_bed = std::max(max_bed, ap.bed_idx);
    }
    REQUIRE(max_bed >= 1); // at least one on a virtual bed
}

TEST_CASE("Arrange handles an empty input", "[Arrange]")
{
    ArrangePolygons items;
    REQUIRE_NOTHROW(arrange(items, bed(200, 200), quiet_params()));
    REQUIRE(items.empty());
}

TEST_CASE("Arrange without final alignment keeps items disjoint", "[Arrange]")
{
    // do_final_align = false selects Alignment::DONT_ALIGN (skips recentering).
    ArrangePolygons items  = squares(6, 40.);
    ArrangeParams   params = quiet_params(scaled(2.));
    params.do_final_align  = false;

    arrange(items, bed(250, 250), params);

    for (const ArrangePolygon &ap : items)
        REQUIRE(ap.bed_idx == 0);
    require_no_overlap(items);
}

TEST_CASE("Arrange aligns the pile to a custom center", "[Arrange]")
{
    // align_center != (0.5, 0.5) selects Alignment::USER_DEFINED.
    ArrangePolygons items  = squares(5, 30.);
    ArrangeParams   params = quiet_params(scaled(2.));
    params.align_center    = Vec2d(0.3, 0.7);

    arrange(items, bed(250, 250), params);

    for (const ArrangePolygon &ap : items)
        REQUIRE(ap.bed_idx == 0);
    require_no_overlap(items);
}

// A belt printer starts its parts at the leading end of the belt (best_object_pos 0.5, 0.05).
// Centring a pile on a point that close to the edge pushed everything longer than the room
// around it off the bed: four 90 mm parts on a 95 x 500 mm belt ended with one across the
// edge and one outside, with 290 mm of belt free behind them. The pile stops at the edge.
TEST_CASE("Arrange keeps a pile aligned near an edge on the bed", "[Arrange][belt]")
{
    const BoundingBox belt   = bed(95, 500);
    ArrangePolygons   items  = squares(4, 90.);
    ArrangeParams     params = quiet_params(scaled(2.));
    params.align_center      = Vec2d(0.5, 0.05);
    params.is_belt           = true;
    params.belt_axis         = 1;
    params.belt_tilt_slope   = 1.f;

    arrange(items, belt, params);

    coord_t lowest = std::numeric_limits<coord_t>::max();
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        const BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        CHECK(belt.contains(bb));
        lowest = std::min(lowest, bb.min.y());
    }
    // Snapped to the edge it was aimed at, less the spacing margin, not re-centred.
    CHECK(lowest < scaled(10.));
    require_no_overlap(items);
}

// The clamp is a belt feature. Printers whose best_object_pos is off-centre (the A1 mini
// and the H2 family) keep their final alignment: the pile is centred on that point, even
// when that puts part of it outside the bed.
TEST_CASE("Arrange leaves the final alignment of a flat bed unclamped", "[Arrange]")
{
    const BoundingBox bed_   = bed(95, 500);
    ArrangePolygons   items  = squares(4, 90.);
    ArrangeParams     params = quiet_params(scaled(2.));
    params.align_center      = Vec2d(0.5, 0.05);

    arrange(items, bed_, params);

    BoundingBox pile;
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        pile.merge(ap.transformed_poly().contour.bounding_box());
    }
    // Centred on the 5% mark of the bed's length, not pushed inside it.
    CHECK_THAT(unscaled<double>(pile.center().y()), Catch::Matchers::WithinAbs(0.05 * 500., 15.));
    CHECK(pile.min.y() < 0);
    require_no_overlap(items);
}

// On a belt the parts print in belt order, so two colours that alternate along the
// belt, or sit side by side, cost a filament change on every shared layer. Arrange
// keeps each colour together: no part shares belt length with a part of another
// colour, counting the tilted layers that run cot(angle) * height past its far edge,
// whichever end of the belt prints first.
TEST_CASE("Arrange groups the colours of a belt print along the belt", "[Arrange][belt]")
{
    const bool reversed = GENERATE(false, true);
    CAPTURE(reversed);
    const BoundingBox belt   = bed(95, 500);
    ArrangePolygons   items  = squares(6, 30., 20.);
    for (size_t i = 0; i < items.size(); ++i)
        items[i].extrude_ids = { int(i % 3) + 1 };   // three colours, two parts each
    ArrangeParams params     = quiet_params(scaled(2.));
    params.align_center      = Vec2d(0.5, 0.05);
    params.is_belt           = true;
    params.belt_axis         = 1;
    params.belt_reversed     = reversed;
    params.belt_tilt_slope   = 1.f;   // 45 degrees

    arrange(items, belt, params);
    require_no_overlap(items);

    // Belt position in print order, so the same check serves both directions.
    const coord_t dir = reversed ? -1 : 1;
    auto start = [&](const ArrangePolygon &ap) { const BoundingBox bb = ap.transformed_poly().contour.bounding_box(); return dir * (reversed ? bb.max.y() : bb.min.y()); };
    auto end   = [&](const ArrangePolygon &ap) { const BoundingBox bb = ap.transformed_poly().contour.bounding_box(); return dir * (reversed ? bb.min.y() : bb.max.y()) + scaled(ap.height * params.belt_tilt_slope); };

    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        CHECK(belt.contains(ap.transformed_poly().contour.bounding_box()));
    }
    for (const ArrangePolygon &a : items)
        for (const ArrangePolygon &b : items) {
            if (a.extrude_ids == b.extrude_ids)
                continue;
            // The part printed later starts after the earlier one has finished.
            const coord_t earlier_end = start(a) <= start(b) ? end(a) : end(b);
            const coord_t later_start = std::max(start(a), start(b));
            INFO("colour " << a.extrude_ids.front() << " vs " << b.extrude_ids.front());
            CHECK(earlier_end <= later_start);
        }
}

TEST_CASE("Sequential print floors the object distance by object height", "[Arrange]")
{
    // The only place sequential-print clearance is enforced. The arrange menu offers
    // no floor of its own, so a stored 0 has to be raised here or not at all.
    struct Case
    {
        std::string         description;
        std::vector<double> heights;
        double              skirt_offset_mm;
        double              expected_floor_mm;
    };

    auto c = GENERATE(values<Case>({
        {"objects taller than the nozzle need the full clearance",     {NOZZLE_HEIGHT_MM * 2, NOZZLE_HEIGHT_MM * 2}, 0., CLEARANCE_MM},
        {"an object exactly at the nozzle height counts as tall",      {NOZZLE_HEIGHT_MM,     NOZZLE_HEIGHT_MM},     0., CLEARANCE_MM},
        {"one tall object among short ones is enough",                 {NOZZLE_HEIGHT_MM / 2, NOZZLE_HEIGHT_MM * 2}, 0., CLEARANCE_MM},
        {"objects the nozzle clears keep only the nozzle-width floor", {NOZZLE_HEIGHT_MM / 2, NOZZLE_HEIGHT_MM / 2}, 0., NOZZLE_FLOOR_MM},
        {"a wide skirt raises the floor for short objects",            {NOZZLE_HEIGHT_MM / 2, NOZZLE_HEIGHT_MM / 2}, 3., 6.},
    }));

    DYNAMIC_SECTION(c.description)
    {
        ArrangePolygons    items = squares_of_heights(c.heights);
        DynamicPrintConfig cfg   = bed_config();
        ArrangeParams      p     = seq_print_params(0);
        p.object_skirt_offset    = float(c.skirt_offset_mm);

        update_selected_items_inflation(items, &cfg, p);

        CHECK(p.min_obj_distance >= scaled(c.expected_floor_mm));
        CHECK(p.min_obj_distance <= scaled(c.expected_floor_mm + 0.01));
        // Half each, so a pair ends up a full min_obj_distance apart.
        CHECK(items.front().inflation == p.min_obj_distance / 2);
    }
}

TEST_CASE("Sequential print keeps an object distance already above the floor", "[Arrange]")
{
    const coord_t      stored = scaled(CLEARANCE_MM * 2);
    ArrangePolygons    items  = squares_of_heights({NOZZLE_HEIGHT_MM * 2, NOZZLE_HEIGHT_MM * 2});
    DynamicPrintConfig cfg    = bed_config();
    ArrangeParams      p      = seq_print_params(stored);

    update_selected_items_inflation(items, &cfg, p);
    CHECK(p.min_obj_distance == stored);
}

TEST_CASE("Layered printing does not floor the object distance", "[Arrange]")
{
    ArrangePolygons    items = squares_of_heights({NOZZLE_HEIGHT_MM * 2, NOZZLE_HEIGHT_MM * 2});
    DynamicPrintConfig cfg   = bed_config();
    ArrangeParams      p     = seq_print_params(0);
    p.is_seq_print           = false;

    update_selected_items_inflation(items, &cfg, p);
    CHECK(p.min_obj_distance == 0);
}

namespace model_arrange_contracts {
using Catch::Matchers::WithinAbs;

// One 20 mm cube object with an instance at each given XY offset.
Model arrange_model(std::initializer_list<Vec2d> offsets = {Vec2d::Zero()})
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "model-arrange-contract.stl";
    object->add_volume(TriangleMesh(its_make_cube(20., 20., 20.)));
    for (const Vec2d &offset : offsets)
        object->add_instance()->set_offset(to_3d(offset, 0.));
    return model;
}

// Supplies only the fields ModelArrange reads, with a distinct value per filament
// so the tests can tell which filament's value was picked.
DynamicPrintConfig arrange_config(const char *support_type, bool enable_support)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"enable_support", enable_support},
        {"support_type", support_type},
        {"curr_bed_type", "Cool Plate"},
        {"cool_plate_temp", "65,66"},
        {"cool_plate_temp_initial_layer", "70,71"},
        {"nozzle_temperature", "205,206"},
        {"nozzle_temperature_initial_layer", "215,216"},
        {"temperature_vitrification", "90,91"}
    });
    config.set_key_value("filament_type", new ConfigOptionStrings({"PLA", "PC"}));
    return config;
}

// Known bug: get_arrange_poly(const Model &) rotates and translates the hull accumulated so far instead of
// each instance's own footprint, so the other instances lose their own offset and Z rotation; fixed by the follow-up
// fix PR.
TEST_CASE("Model arrange polygon is the convex hull of every instance at its position and rotation", "[Arrange][ModelArrange][!shouldfail]")
{
    // The first cube turned a quarter about Z spans x 30..50 at its offset (70 if left unrotated),
    // the second spans 0..20.
    Model model = arrange_model({Vec2d(50., 0.), Vec2d(0., 0.)});
    model.objects.front()->instances.front()->set_rotation(Z, PI / 2.);

    const ArrangePolygon polygon = get_arrange_poly(model);
    const BoundingBox box = polygon.poly.contour.bounding_box();
    CHECK_THAT(unscaled<double>(box.min.x()), WithinAbs(0., .01));
    CHECK_THAT(unscaled<double>(box.max.x()), WithinAbs(50., .01));
    CHECK_THAT(unscaled<double>(box.min.y()), WithinAbs(0., .01));
    CHECK_THAT(unscaled<double>(box.max.y()), WithinAbs(20., .01));
}

TEST_CASE("Model object duplication copies only the original instances", "[Arrange][ModelArrange]")
{
    Model model = arrange_model({Vec2d(1., 2.), Vec2d(11., 12.)});
    const ModelObject *object = model.objects.front();

    duplicate_objects(model, 3);

    REQUIRE(object->instances.size() == 6);
    int first = 0, second = 0;
    for (const ModelInstance *instance : object->instances) {
        const Vec3d offset = instance->get_offset();
        first += offset.isApprox(Vec3d(1., 2., 0.));
        second += offset.isApprox(Vec3d(11., 12., 0.));
    }
    CHECK(first == 3);
    CHECK(second == 3);
}

TEST_CASE("Model instance arrange polygon setter writes an arranged result back to the instance", "[Arrange][ModelArrange]")
{
    Model model = arrange_model();
    ModelInstance *instance = model.objects.front()->instances.front();

    ArrangePolygon polygon = get_arrange_poly(instance, arrange_config("normal(auto)", false));
    CHECK(polygon.bed_idx == 0);
    REQUIRE(polygon.setter);

    SECTION("an arranged result moves the instance") {
        polygon.translation = Vec2crd{scaled(3.), scaled(4.)};
        polygon.rotation = .5;
        polygon.itemid = 17;
        polygon.apply();

        CHECK(polygon.is_applied == 1);
        CHECK_THAT(instance->get_offset(X), WithinAbs(3., 1e-9));
        CHECK_THAT(instance->get_offset(Y), WithinAbs(4., 1e-9));
        CHECK_THAT(instance->get_rotation(Z), WithinAbs(.5, 1e-9));
        CHECK(instance->arrange_order == 17);
    }
    SECTION("an unarranged result leaves the instance where it was") {
        polygon.translation = Vec2crd{scaled(3.), scaled(4.)};
        polygon.bed_idx = arrangement::UNARRANGED;
        polygon.apply();

        CHECK(polygon.is_applied == 1);
        CHECK_THAT(instance->get_offset(X), WithinAbs(0., 1e-9));
        CHECK_THAT(instance->get_offset(Y), WithinAbs(0., 1e-9));
    }
}

TEST_CASE("Model instance arrangement reads plate and filament metadata for the instance's filament", "[Arrange][ModelArrange]")
{
    Model model = arrange_model();
    ModelObject *object = model.objects.front();
    object->volumes.front()->config.set("extruder", 2);
    ModelInstance *instance = object->instances.front();

    const ArrangePolygon polygon = get_instance_arrange_poly(instance, arrange_config("normal(auto)", false));

    REQUIRE(polygon.extrude_ids == std::vector<int>{2});
    CHECK(polygon.bed_temp == 66);
    CHECK(polygon.first_bed_temp == 71);
    CHECK(polygon.print_temp == 206);
    CHECK(polygon.first_print_temp == 216);
    CHECK(polygon.vitrify_temp == 91);
    CHECK(polygon.filament_temp_type == FilamentTempType::HighTemp); // PC
    CHECK(polygon.name == object->name);
    CHECK_THAT(polygon.height, WithinAbs(20., 1e-6));
}

TEST_CASE("Model instance arrangement sizes the brim margin by the support type", "[Arrange][ModelArrange]")
{
    Model model = arrange_model();
    ModelInstance *instance = model.objects.front()->instances.front();

    SECTION("disabled support keeps the 1 mm default even for tree support") {
        const ArrangePolygon polygon = get_instance_arrange_poly(instance, arrange_config("tree(manual)", false));
        CHECK_THAT(polygon.brim_width, WithinAbs(1., 1e-9));
        CHECK_FALSE(polygon.has_tree_support);
    }
    SECTION("normal support adds 6 mm") {
        const ArrangePolygon polygon = get_instance_arrange_poly(instance, arrange_config("normal(manual)", true));
        CHECK_THAT(polygon.brim_width, WithinAbs(6., 1e-9));
        CHECK_FALSE(polygon.has_tree_support);
    }
    SECTION("tree support adds 24 mm and marks the plate") {
        const ArrangePolygon polygon = get_instance_arrange_poly(instance, arrange_config("tree(manual)", true));
        CHECK_THAT(polygon.brim_width, WithinAbs(24., 1e-9));
        CHECK(polygon.has_tree_support);
    }
}

TEST_CASE("Model arrange default out-of-bed callback throws with the bed index", "[Arrange][ModelArrange]")
{
    ArrangePolygon polygon;
    polygon.bed_idx = 1;

    REQUIRE_THROWS_AS(throw_if_out_of_bed(polygon), RuntimeError);
    REQUIRE_THROWS_WITH(throw_if_out_of_bed(polygon), Catch::Matchers::ContainsSubstring("bed_idx==1"));
}

TEST_CASE("Model arrange collects instances and applies reachable bed results", "[Arrange][ModelArrange]")
{
    Model model = arrange_model({Vec2d::Zero(), Vec2d::Zero(), Vec2d::Zero()});
    ModelInstancePtrs instances;
    ArrangePolygons polygons = get_arrange_polys(model, instances);

    REQUIRE(polygons.size() == 3);
    REQUIRE(instances.size() == 3);
    for (size_t i = 0; i < instances.size(); ++i) {
        REQUIRE(instances[i] == model.objects.front()->instances[i]);
        CHECK(polygons[i].bed_idx == 0);
    }

    polygons[0].translation = Vec2crd{scaled(4.), scaled(5.)};
    polygons[1].translation = Vec2crd{scaled(8.), scaled(9.)};
    polygons[1].bed_idx = 2;
    polygons[2].translation = Vec2crd{scaled(12.), scaled(13.)};
    polygons[2].bed_idx = arrangement::UNARRANGED;

    std::vector<int> callback_beds;
    const bool all_on_physical_bed = apply_arrange_polys(
        polygons, instances, [&callback_beds](ArrangePolygon &polygon) {
            callback_beds.push_back(polygon.bed_idx);
        });

    // Virtual and unarranged results both reach the callback, but only a
    // result on some bed moves its instance.
    CHECK_FALSE(all_on_physical_bed);
    CHECK(callback_beds == std::vector<int>{2, arrangement::UNARRANGED});
    CHECK_THAT(instances[0]->get_offset(X), WithinAbs(4., 1e-9));
    CHECK_THAT(instances[0]->get_offset(Y), WithinAbs(5., 1e-9));
    CHECK_THAT(instances[1]->get_offset(X), WithinAbs(8., 1e-9));
    CHECK_THAT(instances[1]->get_offset(Y), WithinAbs(9., 1e-9));
    CHECK_THAT(instances[2]->get_offset(X), WithinAbs(0., 1e-9));
    CHECK_THAT(instances[2]->get_offset(Y), WithinAbs(0., 1e-9));
}

TEST_CASE("Model arrange reports success when every result is on the physical bed", "[Arrange][ModelArrange]")
{
    Model model = arrange_model({Vec2d::Zero(), Vec2d::Zero()});
    ModelInstancePtrs instances;
    ArrangePolygons polygons = get_arrange_polys(model, instances);
    REQUIRE(polygons.size() == 2);
    polygons[0].translation = Vec2crd{scaled(1.), scaled(2.)};
    polygons[1].translation = Vec2crd{scaled(3.), scaled(4.)};

    int callbacks = 0;
    CHECK(apply_arrange_polys(polygons, instances, [&callbacks](ArrangePolygon &) { ++callbacks; }));
    CHECK(callbacks == 0);
    CHECK_THAT(instances[0]->get_offset(X), WithinAbs(1., 1e-9));
    CHECK_THAT(instances[0]->get_offset(Y), WithinAbs(2., 1e-9));
    CHECK_THAT(instances[1]->get_offset(X), WithinAbs(3., 1e-9));
    CHECK_THAT(instances[1]->get_offset(Y), WithinAbs(4., 1e-9));
}

TEST_CASE("Model arrange duplicates each source instance for every requested placement", "[Arrange][ModelArrange]")
{
    Model model = arrange_model({Vec2d(1., 2.), Vec2d(11., 12.)});
    const ModelObject *object = model.objects.front();

    // Each copy's translation is added to every source instance's offset.
    ArrangePolygon first = make_square(scaled(8.));
    first.translation = Vec2crd{scaled(4.), scaled(5.)};
    ArrangePolygon virtual_bed = make_square(scaled(8.));
    virtual_bed.translation = Vec2crd{scaled(-6.), scaled(7.)};
    virtual_bed.bed_idx = 2;
    ArrangePolygons copies{first, virtual_bed};

    std::vector<int> callback_beds;
    duplicate(model, copies, [&callback_beds](ArrangePolygon &polygon) {
        callback_beds.push_back(polygon.bed_idx);
    });

    REQUIRE(object->instances.size() == 4);
    CHECK(callback_beds == std::vector<int>{2, 2});
    CHECK_THAT(object->instances[0]->get_offset(X), WithinAbs(5., 1e-9));
    CHECK_THAT(object->instances[0]->get_offset(Y), WithinAbs(7., 1e-9));
    CHECK_THAT(object->instances[1]->get_offset(X), WithinAbs(-5., 1e-9));
    CHECK_THAT(object->instances[1]->get_offset(Y), WithinAbs(9., 1e-9));
    CHECK_THAT(object->instances[2]->get_offset(X), WithinAbs(15., 1e-9));
    CHECK_THAT(object->instances[2]->get_offset(Y), WithinAbs(17., 1e-9));
    CHECK_THAT(object->instances[3]->get_offset(X), WithinAbs(5., 1e-9));
    CHECK_THAT(object->instances[3]->get_offset(Y), WithinAbs(19., 1e-9));
}

} // namespace model_arrange_contracts
