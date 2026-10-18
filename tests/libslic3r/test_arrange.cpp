#include "libslic3r/Exception.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <initializer_list>
#include <nlohmann/json.hpp>
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

namespace arrange_surface_contracts {
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

ArrangePolygon rectangle(double width_mm, double height_mm, double x_mm = 0., double y_mm = 0.)
{
    const coord_t x = scaled(x_mm);
    const coord_t y = scaled(y_mm);
    ArrangePolygon result;
    result.poly = ExPolygon(Polygon{{Point(x, y), Point(x + scaled(width_mm), y),
                                     Point(x + scaled(width_mm), y + scaled(height_mm)),
                                     Point(x, y + scaled(height_mm))}});
    result.bed_idx = 0;
    result.extrude_ids = {1};
    return result;
}

double area(const ArrangePolygon &item)
{
    return item.transformed_poly().area();
}

double ex_area(const ExPolygons &polygons)
{
    double total = 0.;
    for (const ExPolygon &polygon : polygons)
        total += polygon.area();
    return total;
}

bool inside_circle(const BoundingBox &box, const Point &center, double radius)
{
    for (const Point &corner : {box.min, Point(box.min.x(), box.max.y()),
                                Point(box.max.x(), box.min.y()), box.max}) {
        const double dx = double(corner.x() - center.x());
        const double dy = double(corner.y() - center.y());
        if (std::hypot(dx, dy) > radius + scaled(.01))
            return false;
    }
    return true;
}

TEST_CASE("ArrangePolygon transforms the retained contour without mutating its source", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon item = rectangle(20., 10.);
    const ExPolygon source = item.poly;
    item.rotation = .35;
    item.translation = Vec2crd(scaled(7.), scaled(9.));

    const ExPolygon transformed = item.transformed_poly();
    CHECK(item.poly == source);
    CHECK(transformed != source);
    CHECK_THAT(transformed.area(), WithinRel(source.area(), 1e-6));
    const double c = std::cos(item.rotation);
    const double s = std::sin(item.rotation);
    REQUIRE(transformed.contour.points.size() == source.contour.points.size());
    for (size_t i = 0; i < source.contour.points.size(); ++i) {
        const Point &point = source.contour.points[i];
        const Point expected(
            coord_t(std::round(c * double(point.x()) - s * double(point.y()))) + item.translation.x(),
            coord_t(std::round(c * double(point.y()) + s * double(point.x()))) + item.translation.y());
        CHECK(transformed.contour.points[i] == expected);
    }
}

TEST_CASE("ArrangePolygon applies its setter once and records that it was applied", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon item = rectangle(10., 10.);
    int calls = 0;
    item.setter = [&](const ArrangePolygon &observed) {
        ++calls;
        CHECK(observed.poly.area() > 0.);
        CHECK(observed.bed_idx == 0);
    };
    item.apply();
    item.apply();
    CHECK(calls == 1);
    CHECK(item.is_applied == 1);
    CHECK(item.is_arranged());
}

TEST_CASE("ArrangeParams serializes the distance and optimization controls", "[Arrange][ArrangeSurface]")
{
    ArrangeParams params = quiet_params(scaled(1.5));
    params.accuracy = .25f;
    params.parallel = false;
    params.allow_rotations = true;
    params.do_final_align = false;
    params.is_seq_print = true;
    params.bed_shrink_x = 2.f;
    params.bed_shrink_y = 3.f;
    params.brim_skirt_distance = 4.f;
    params.clearance_height_to_rod = 5.f;
    params.clearance_height_to_lid = 6.f;
    params.clearance_radius = 7.f;
    params.printable_height = 8.f;
    // ArrangeParams::to_json() currently returns the object body with a final
    // comma; close that serialization fragment before parsing so
    // field values are checked structurally rather than by substrings.
    std::string serialized = params.to_json();
    REQUIRE(!serialized.empty());
    REQUIRE(serialized.back() == ',');
    serialized.back() = '}';
    const nlohmann::json json = nlohmann::json::parse(serialized);
    CHECK(json.at("min_obj_distance") == params.min_obj_distance);
    CHECK_THAT(json.at("accuracy").get<double>(), WithinAbs(.25, 1e-12));
    CHECK(json.at("parallel") == 0);
    CHECK(json.at("allow_rotations") == 1);
    CHECK(json.at("do_final_align") == 0);
    CHECK(json.at("is_seq_print") == 1);
    CHECK_THAT(json.at("bed_shrink_x").get<double>(), WithinAbs(2., 1e-12));
    CHECK_THAT(json.at("bed_shrink_y").get<double>(), WithinAbs(3., 1e-12));
    CHECK_THAT(json.at("brim_skirt_distance").get<double>(), WithinAbs(4., 1e-12));
    CHECK_THAT(json.at("clearance_height_to_rod").get<double>(), WithinAbs(5., 1e-12));
    CHECK_THAT(json.at("clearance_height_to_lid").get<double>(), WithinAbs(6., 1e-12));
    CHECK_THAT(json.at("clearance_radius").get<double>(), WithinAbs(7., 1e-12));
    CHECK_THAT(json.at("printable_height").get<double>(), WithinAbs(8., 1e-12));
}

TEST_CASE("Arrange dispatches an empty point bed as an infinite bed", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(12., 8.)};
    arrange(items, Points{}, quiet_params());
    REQUIRE(items.size() == 1);
    CHECK(items.front().bed_idx == 0);
    CHECK(area(items.front()) > 0.);
}

TEST_CASE("Arrange dispatches a one-point bed around its supplied center", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(12., 8.)};
    arrange(items, Points{Point(scaled(100.), scaled(75.))}, quiet_params());
    REQUIRE(items.size() == 1);
    CHECK(items.front().bed_idx == 0);
    const BoundingBox box = items.front().transformed_poly().contour.bounding_box();
    CHECK(box.defined);
    CHECK_THAT(unscaled<double>(box.center().x()), WithinAbs(100., .01));
    CHECK_THAT(unscaled<double>(box.center().y()), WithinAbs(75., .01));
}

TEST_CASE("Arrange accepts a rectangular point bed and keeps all parts inside", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(20., 10.), rectangle(15., 15.), rectangle(8., 22.)};
    const Points bed{Point(0, 0), Point(scaled(100.), 0), Point(scaled(100.), scaled(80.)), Point(0, scaled(80.))};
    arrange(items, bed, quiet_params(scaled(1.)));
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(BoundingBox(Point(0, 0), Point(scaled(100.), scaled(80.))).contains(
            item.transformed_poly().contour.bounding_box()));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange uses a circular point bed for a regular polygon", "[Arrange][ArrangeSurface]")
{
    // This rectangle fits inside the circle but cannot fit inside the
    // inscribed diamond made by the same four points.  Rotations are disabled
    // so the dispatch choice is observable rather than hidden by a fitter.
    ArrangePolygons items{rectangle(140., 80.)};
    const Point center(scaled(100.), scaled(100.));
    Points bed;
    // Cardinal points are exactly equidistant after integer scaling, so this
    // selects CircleBed rather than the polygon fallback in call_with_bed.
    for (int i = 0; i < 4; ++i) {
        const double angle = 2. * PI * double(i) / 4.;
        bed.emplace_back(center.x() + scaled(100. * std::cos(angle)),
                         center.y() + scaled(100. * std::sin(angle)));
    }
    ArrangeParams params = quiet_params(scaled(1.));
    params.allow_rotations = false;
    arrange(items, bed, params);
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(inside_circle(item.transformed_poly().contour.bounding_box(), center, scaled(101.)));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange leaves invalid contours unarranged while placing valid contours", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon invalid;
    invalid.poly.contour.points = {Point(0, 0), Point(scaled(10.), 0)};
    ArrangePolygons items{rectangle(10., 10.), invalid};
    arrange(items, bed(100., 100.), quiet_params());
    REQUIRE(items[0].bed_idx == 0);
    CHECK(items[1].bed_idx == UNARRANGED);
}

TEST_CASE("Arrange supports a CircleBed and preserves every footprint area", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(20., 10.), rectangle(12., 18.), rectangle(9., 9.)};
    const std::vector<double> original{area(items[0]), area(items[1]), area(items[2])};
    const Point center(scaled(100.), scaled(100.));
    arrange(items, CircleBed(center, scaled(100.)), quiet_params(scaled(1.)));
    for (size_t i = 0; i < items.size(); ++i) {
        REQUIRE(items[i].bed_idx == 0);
        CHECK_THAT(area(items[i]), WithinRel(original[i], 1e-8));
        CHECK(inside_circle(items[i].transformed_poly().contour.bounding_box(), center, scaled(101.)));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange keeps convex polygon-bed footprints inside and disjoint", "[Arrange][ArrangeSurface]")
{
    Polygon bed_poly{{Point(scaled(10.), scaled(10.)), Point(scaled(190.), scaled(10.)),
                      Point(scaled(200.), scaled(100.)), Point(scaled(190.), scaled(190.)),
                      Point(scaled(10.), scaled(190.)), Point(0, scaled(100.))}};
    ArrangePolygons items{rectangle(20., 20.), rectangle(18., 12.)};
    arrange(items, bed_poly, quiet_params(scaled(1.)));
    const ExPolygon bed_shape(bed_poly);
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        const BoundingBox box = item.transformed_poly().contour.bounding_box();
        CHECK(box.defined);
        CHECK_THAT(ex_area(intersection_ex(item.transformed_poly(), bed_shape)),
                   WithinRel(area(item), 1e-8));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange keeps a fixed exclusion outside every arranged footprint", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon fixed = rectangle(30., 30.);
    ArrangePolygons items{rectangle(20., 20.), rectangle(18., 12.)};
    const ExPolygon fixed_shape = fixed.transformed_poly();
    arrange(items, {fixed}, bed(120., 120.), quiet_params(scaled(1.)));
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(intersection_ex(item.transformed_poly(), fixed_shape).empty());
    }
}

TEST_CASE("Arrange keeps an initially unrotated rectangular item unrotated when rotations are disabled", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(35., 12.)};
    ArrangeParams params = quiet_params();
    params.allow_rotations = false;
    arrange(items, bed(100., 100.), params);
    REQUIRE(items.front().bed_idx == 0);
    CHECK_THAT(items.front().rotation, WithinAbs(0., 1e-12));
}

TEST_CASE("Arrange may rotate a rectangular item while preserving its area", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(35., 12.)};
    const double source_area = area(items.front());
    ArrangeParams params = quiet_params();
    params.allow_rotations = true;
    arrange(items, bed(100., 100.), params);
    REQUIRE(items.front().bed_idx == 0);
    CHECK_THAT(area(items.front()), WithinRel(source_area, 1e-6));
    CHECK(items.front().transformed_poly().contour.bounding_box().defined);
}

TEST_CASE("Arrange rotates an item only when the bed requires it", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons disabled{rectangle(120., 30.)};
    ArrangeParams no_rotations = quiet_params();
    no_rotations.allow_rotations = false;
    arrange(disabled, bed(50., 130.), no_rotations);
    REQUIRE(disabled.front().bed_idx == UNARRANGED);

    ArrangePolygons enabled{rectangle(120., 30.)};
    ArrangeParams rotations = quiet_params();
    rotations.allow_rotations = true;
    arrange(enabled, bed(50., 130.), rotations);
    REQUIRE(enabled.front().bed_idx == 0);
    CHECK(std::abs(std::sin(enabled.front().rotation)) > .9);
    CHECK(bed(50., 130.).contains(enabled.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("Axis alignment rotates an asymmetric contour while preserving area", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(20., 60.)};
    items.front().rotation = .37;
    const double source_area = area(items.front());
    ArrangeParams params = quiet_params();
    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    CHECK(std::abs(items.front().rotation - .37) > .01);
    const BoundingBox box = items.front().transformed_poly().contour.bounding_box();
    CHECK_THAT(unscaled<double>(box.size().x()), WithinAbs(20., .01));
    CHECK_THAT(unscaled<double>(box.size().y()), WithinAbs(60., .01));
    CHECK_THAT(area(items.front()), WithinRel(source_area, 1e-6));
}

TEST_CASE("Axis alignment leaves a symmetric square orientation unchanged", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(30., 30.)};
    items.front().rotation = .37;
    ArrangeParams params = quiet_params();
    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    CHECK_THAT(items.front().rotation, WithinAbs(.37, 1e-12));
}

TEST_CASE("Axis alignment ignores a degenerate contour", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon item;
    item.poly.contour.points = {Point(0, 0), Point(scaled(10.), 0)};
    item.rotation = .37;
    ArrangePolygons items{item};
    ArrangeParams params = quiet_params();
    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    CHECK_THAT(items.front().rotation, WithinAbs(.37, 1e-12));
}

TEST_CASE("Selected inflation uses the maximum tree branch brim for the plate", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(20., 20.), rectangle(20., 20.)};
    items[0].brim_width = 1.;
    items[1].brim_width = 4.;
    items[1].has_tree_support = true;
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params();
    update_selected_items_inflation(items, &config, params);
    CHECK(items[0].inflation == scaled(2.));
    CHECK(items[1].inflation == scaled(2.));
}

TEST_CASE("Selected inflation gives an explicit minimum distance precedence over brim", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(20., 20.)};
    items.front().brim_width = 8.;
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params(scaled(6.));
    update_selected_items_inflation(items, &config, params);
    CHECK(items.front().inflation == scaled(3.));
}

TEST_CASE("Unselected inflation distinguishes ordinary, virtual, and calibration regions", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(10., 10.), rectangle(10., 10.), rectangle(10., 10.)};
    items[0].brim_width = 3.;
    items[1].is_virt_object = true;
    items[2].is_virt_object = true;
    items[2].is_extrusion_cali_object = true;
    const DynamicPrintConfig config;
    const ArrangeParams params = quiet_params();
    update_unselected_items_inflation(items, &config, params);
    CHECK(items[0].inflation == scaled(3.));
    CHECK(items[1].inflation == scaled(1.));
    CHECK(items[2].inflation == 0);
}

TEST_CASE("Sequential virtual exclusion regions move by the configured shrink vector", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(10., 10., 20., 30.)};
    items.front().is_virt_object = true;
    const Point before = items.front().poly.contour.points.front();
    ArrangeParams params = quiet_params();
    params.is_seq_print = true;
    params.bed_shrink_x = 4.f;
    params.bed_shrink_y = 6.f;
    update_unselected_items_inflation(items, nullptr, params);
    const Point after = items.front().poly.contour.points.front();
    CHECK(after.x() == before.x() + scaled(4.));
    CHECK(after.y() == before.y() + scaled(6.));
}

TEST_CASE("Arrange parameter update adds the real skirt clearance and sequential correction", "[Arrange][ArrangeSurface]")
{
    DynamicPrintConfig config;
    config.set_key_value("skirt_height", new ConfigOptionInt(1));
    config.set_key_value("skirt_loops", new ConfigOptionInt(2));
    config.set_key_value("skirt_distance", new ConfigOptionFloat(3.f));
    config.set_key_value("line_width", new ConfigOptionFloat(.4f));
    config.set_key_value("initial_layer_line_width", new ConfigOptionFloatOrPercent(.4, false));
    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({.4}));
    config.set_key_value("printable_area", new ConfigOptionPoints{{0, 0}, {200, 0}, {200, 200}, {0, 200}});
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = 10.f;
    params.bed_shrink_y = 11.f;
    params.is_seq_print = true;
    params.clearance_radius = 4.f;
    update_arrange_params(params, &config, {});
    CHECK_THAT(params.brim_skirt_distance, WithinAbs(3.8, 1e-5));
    CHECK_THAT(params.bed_shrink_x, WithinAbs(11.8, 1e-5));
    CHECK_THAT(params.bed_shrink_y, WithinAbs(12.8, 1e-5));
}

TEST_CASE("Shrinking a rectangular bed moves every edge toward its center", "[Arrange][ArrangeSurface]")
{
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = 5.f;
    params.bed_shrink_y = 7.f;
    const Points points = get_shrink_bedpts(&config, params);
    REQUIRE(points.size() == 4);
    BoundingBox box(points);
    CHECK(box.min.x() == scaled(5.));
    CHECK(box.max.x() == scaled(195.));
    CHECK(box.min.y() == scaled(7.));
    CHECK(box.max.y() == scaled(193.));
}

} // namespace arrange_surface_contracts

namespace model_arrange_contracts {

Model arrange_model(size_t instance_count = 1)
{
    Model model;
    ModelObject *object = model.add_object();
    object->name += "model-arrange-contract.stl";
    TriangleMesh cube(its_make_cube(20., 20., 20.));
    object->add_volume(cube);
    object->add_instance();
    for (size_t i = 1; i < instance_count; ++i)
        object->add_instance();
    return model;
}

DynamicPrintConfig arrange_config(const char *support_type, bool enable_support)
{
    // This fixture supplies only fields read by ModelArrange metadata paths.
    // Every per-filament value is intentionally two entries; no Print/G-code is built.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"enable_support", enable_support},
        {"support_type", support_type},
        {"support_filament", enable_support ? 1 : 0},
        {"support_interface_filament", enable_support ? 2 : 0},
        {"curr_bed_type", "Cool Plate"},
        {"cool_plate_temp", "65,66"},
        {"cool_plate_temp_initial_layer", "70,71"},
        {"nozzle_temperature", "205,206"},
        {"nozzle_temperature_initial_layer", "215,216"},
        {"temperature_vitrification", "90,91"}
    });
    config.set_key_value("filament_type", new ConfigOptionStrings({"PLA", "PETG"}));
    return config;
}

void require_bounded_arrange_polygon(const ArrangePolygon &polygon)
{
    REQUIRE_FALSE(polygon.poly.contour.points.empty());
    REQUIRE(polygon.poly.contour.points.size() <= 10000);
    CHECK(polygon.poly.contour.area() > 0.0);
    CHECK(std::isfinite(polygon.height));
    CHECK(std::isfinite(polygon.brim_width));
}


TEST_CASE("Model arrange polygon returns a bounded convex hull", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model(2);

    const ArrangePolygon polygon = get_arrange_poly(model);
    require_bounded_arrange_polygon(polygon);
    CHECK(polygon.bed_idx == arrangement::UNARRANGED);
}

TEST_CASE("Model arrange object duplication uses the original instance snapshot", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    ModelObject *object = model.objects.front();

    duplicate_objects(model, 4);

    REQUIRE(object->instances.size() == 4);
    CHECK(object->instances.front() != nullptr);
}

TEST_CASE("Model instance arrange polygon applies its bounded setter", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    ModelInstance *instance = model.objects.front()->instances.front();
    DynamicPrintConfig config = arrange_config("normal(auto)", false);

    ArrangePolygon polygon = get_arrange_poly(instance, config);
    require_bounded_arrange_polygon(polygon);
    CHECK(polygon.bed_idx == 0);
    REQUIRE(static_cast<bool>(polygon.setter));
    polygon.translation = Vec2crd{scaled(3.), scaled(4.)};
    polygon.itemid = 17;
    polygon.apply();

    CHECK(polygon.is_applied == 1);
    CHECK(instance->arrange_order == 17);
}

TEST_CASE("Model instance arrangement carries plate and filament metadata", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    ModelObject *object = model.objects.front();
    ModelInstance *instance = object->instances.front();
    DynamicPrintConfig config = arrange_config("normal(auto)", false);

    const ArrangePolygon polygon = get_instance_arrange_poly(instance, config);

    CHECK(polygon.bed_temp == 65);
    CHECK(polygon.first_bed_temp == 70);
    CHECK(polygon.print_temp == 205);
    CHECK(polygon.first_print_temp == 215);
    CHECK(polygon.vitrify_temp == 90);
    CHECK(polygon.filament_temp_type != -1);
    CHECK(polygon.name == object->name);
    CHECK(polygon.height > 0.0);
    CHECK_THAT(polygon.brim_width, Catch::Matchers::WithinAbs(1.0, 1e-9));
    CHECK_FALSE(polygon.has_tree_support);
}

TEST_CASE("Model instance normal support selects the normal brim contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    DynamicPrintConfig config = arrange_config("normal(manual)", true);

    const ArrangePolygon polygon = get_instance_arrange_poly(model.objects.front()->instances.front(), config);

    CHECK_THAT(polygon.brim_width, Catch::Matchers::WithinAbs(6.0, 1e-9));
    CHECK_FALSE(polygon.has_tree_support);
}

TEST_CASE("Model instance tree support selects the tree brim contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    DynamicPrintConfig config = arrange_config("tree(manual)", true);

    const ArrangePolygon polygon = get_instance_arrange_poly(model.objects.front()->instances.front(), config);

    CHECK_THAT(polygon.brim_width, Catch::Matchers::WithinAbs(24.0, 1e-9));
    CHECK(polygon.has_tree_support);
}

TEST_CASE("Model arrange out of bed callback has a stable exception contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    ArrangePolygon polygon;
    polygon.bed_idx = 1;

    REQUIRE_THROWS_AS(throw_if_out_of_bed(polygon), RuntimeError);
}


TEST_CASE("Model arrange collects instances and applies reachable bed results", "[Arrange][ModelArrange][ModelArrangeActive]")
{
    Model model = arrange_model(3);
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

    REQUIRE_FALSE(all_on_physical_bed);
    REQUIRE(callback_beds.size() == 2);
    CHECK(callback_beds[0] == 2);
    CHECK(callback_beds[1] == arrangement::UNARRANGED);
    CHECK_THAT(instances[0]->get_offset(X), Catch::Matchers::WithinAbs(4., 1e-9));
    CHECK_THAT(instances[0]->get_offset(Y), Catch::Matchers::WithinAbs(5., 1e-9));
    CHECK_THAT(instances[1]->get_offset(X), Catch::Matchers::WithinAbs(8., 1e-9));
    CHECK_THAT(instances[1]->get_offset(Y), Catch::Matchers::WithinAbs(9., 1e-9));
    CHECK_THAT(instances[2]->get_offset(X), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(instances[2]->get_offset(Y), Catch::Matchers::WithinAbs(0., 1e-9));

    Model physical_model = arrange_model(2);
    ModelInstancePtrs physical_instances;
    ArrangePolygons physical_polygons = get_arrange_polys(physical_model, physical_instances);
    REQUIRE(physical_polygons.size() == 2);
    REQUIRE(physical_instances.size() == 2);
    physical_polygons[0].translation = Vec2crd{scaled(1.), scaled(2.)};
    physical_polygons[1].translation = Vec2crd{scaled(3.), scaled(4.)};
    std::vector<int> physical_callbacks;
    REQUIRE(apply_arrange_polys(
        physical_polygons, physical_instances, [&physical_callbacks](ArrangePolygon &polygon) {
            physical_callbacks.push_back(polygon.bed_idx);
        }));
    CHECK(physical_callbacks.empty());
    CHECK_THAT(physical_instances[0]->get_offset(X), Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(physical_instances[0]->get_offset(Y), Catch::Matchers::WithinAbs(2., 1e-9));
    CHECK_THAT(physical_instances[1]->get_offset(X), Catch::Matchers::WithinAbs(3., 1e-9));
    CHECK_THAT(physical_instances[1]->get_offset(Y), Catch::Matchers::WithinAbs(4., 1e-9));
}

TEST_CASE("Model arrange duplicates each source instance for every requested placement", "[Arrange][ModelArrange][ModelArrangeActive]")
{
    Model model = arrange_model(2);
    ModelObject *object = model.objects.front();
    object->instances[0]->set_offset(Vec3d(1., 2., 0.));
    object->instances[1]->set_offset(Vec3d(11., 12., 0.));

    ArrangePolygon first = make_square(scaled(8.));
    first.translation = Vec2crd{scaled(4.), scaled(5.)};
    first.bed_idx = 0;
    ArrangePolygon virtual_bed = make_square(scaled(8.));
    virtual_bed.translation = Vec2crd{scaled(-6.), scaled(7.)};
    virtual_bed.bed_idx = 2;
    ArrangePolygons copies{first, virtual_bed};

    std::vector<int> callback_beds;
    duplicate(model, copies, [&callback_beds](ArrangePolygon &polygon) {
        callback_beds.push_back(polygon.bed_idx);
    });

    REQUIRE(object->instances.size() == 4);
    REQUIRE(callback_beds.size() == 2);
    CHECK(callback_beds[0] == 2);
    CHECK(callback_beds[1] == 2);

    CHECK_THAT(object->instances[0]->get_offset(X), Catch::Matchers::WithinAbs(5., 1e-9));
    CHECK_THAT(object->instances[0]->get_offset(Y), Catch::Matchers::WithinAbs(7., 1e-9));
    CHECK_THAT(object->instances[1]->get_offset(X), Catch::Matchers::WithinAbs(-5., 1e-9));
    CHECK_THAT(object->instances[1]->get_offset(Y), Catch::Matchers::WithinAbs(9., 1e-9));
    CHECK_THAT(object->instances[2]->get_offset(X), Catch::Matchers::WithinAbs(15., 1e-9));
    CHECK_THAT(object->instances[2]->get_offset(Y), Catch::Matchers::WithinAbs(17., 1e-9));
    CHECK_THAT(object->instances[3]->get_offset(X), Catch::Matchers::WithinAbs(5., 1e-9));
    CHECK_THAT(object->instances[3]->get_offset(Y), Catch::Matchers::WithinAbs(19., 1e-9));
}


} // namespace model_arrange_contracts
