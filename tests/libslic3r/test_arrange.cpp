#include <memory>
#include "libslic3r/Exception.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include <algorithm>
#include <nlohmann/json.hpp>
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
#include <initializer_list>

#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/Arrange.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/MultiMaterialSegmentation.hpp"

using namespace Slic3r;
using namespace Slic3r::arrangement;

namespace {

using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// Rectangle of the given (scaled) size, lower-left at the origin. bed_idx starts at
// 0 because arrange() seeds the nester's bin from it (see ModelArrange.cpp).
ArrangePolygon make_rect(coord_t width, coord_t depth)
{
    ArrangePolygon ap;
    ap.poly    = ExPolygon(Polygon({{0, 0}, {width, 0}, {width, depth}, {0, depth}}));
    ap.bed_idx = 0;
    return ap;
}

ArrangePolygon make_square(coord_t side) { return make_rect(side, side); }

ArrangePolygons squares(int n, double side_mm, double height_mm = 0.)
{
    ArrangePolygons items;
    for (int i = 0; i < n; ++i) {
        items.emplace_back(make_square(scaled(side_mm)));
        items.back().height = height_mm;
    }
    return items;
}

// 64-gon disc of the given diameter, centred on the origin.
ArrangePolygon make_disc(double diameter_mm, double height_mm)
{
    Polygon p;
    for (int i = 0; i < 64; ++i) {
        const double a = 2. * PI * i / 64;
        p.points.emplace_back(scaled(diameter_mm / 2 * std::cos(a)), scaled(diameter_mm / 2 * std::sin(a)));
    }
    ArrangePolygon ap;
    ap.poly    = ExPolygon(p);
    ap.bed_idx = 0;
    ap.height  = height_mm;
    return ap;
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

// Shortest distance between two bounding boxes, 0 when they touch or overlap.
double box_gap(const BoundingBox &a, const BoundingBox &b)
{
    coord_t sx = std::max<coord_t>(0, std::max(b.min.x() - a.max.x(), a.min.x() - b.max.x()));
    coord_t sy = std::max<coord_t>(0, std::max(b.min.y() - a.max.y(), a.min.y() - b.max.y()));
    return std::sqrt(double(sx) * sx + double(sy) * sy);
}

// The sequential-print floor is chosen by comparing object height against the nozzle,
// so the two are defined together and every expectation is derived from them.
constexpr double NOZZLE_HEIGHT_MM = 2.5;
constexpr double CLEARANCE_MM     = 30.;
constexpr double NOZZLE_FLOOR_MM  = MAX_OUTER_NOZZLE_DIAMETER;

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

constexpr double SKIRT_DISTANCE_MM = 2.;

// A skirt of `loops` loops of `width` on the bed_config() bed, SKIRT_DISTANCE_MM from the items.
DynamicPrintConfig skirt_config(int loops, const ConfigOption &width)
{
    DynamicPrintConfig cfg = bed_config();
    cfg.set_key_value("skirt_loops", new ConfigOptionInt(loops));
    cfg.set_key_value("skirt_height", new ConfigOptionInt(1));
    cfg.set_key_value("skirt_distance", new ConfigOptionFloat(SKIRT_DISTANCE_MM));
    cfg.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4}));
    cfg.set_key_value("initial_layer_line_width", width.clone());
    return cfg;
}

ArrangePolygons squares_of_heights(const std::vector<double> &heights_mm)
{
    ArrangePolygons items;
    for (double height_mm : heights_mm)
        items.push_back(squares(1, 20., height_mm).front());
    return items;
}

// Arranges as the arrange job does for a sequential print, on the bed_config() bed.
void arrange_by_object(ArrangePolygons &items, ArrangePolygons &fixed)
{
    DynamicPrintConfig cfg = bed_config();
    ArrangeParams      p   = seq_print_params(0);
    p.bed_shrink_x         = BED_SHRINK_SEQ_PRINT;
    p.bed_shrink_y         = BED_SHRINK_SEQ_PRINT;
    // Rod and lid above the objects, so they need not each go to a bed of their own.
    p.clearance_height_to_rod = float(NOZZLE_HEIGHT_MM * 10);
    p.clearance_height_to_lid = float(NOZZLE_HEIGHT_MM * 40);
    for (ArrangePolygon &ap : items)
        ap.extrude_ids = {1}; // the sequential sort reads extrude_ids.front()

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    update_unselected_items_inflation(fixed, &cfg, p);
    arrange(items, fixed, get_shrink_bedpts(&cfg, p), p);
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
    // a pair is the distance between their boxes (0.5mm slack for nester rounding).
    std::vector<BoundingBox> boxes;
    for (const ExPolygon &e : placed_shapes(items))
        boxes.push_back(e.contour.bounding_box());

    double min_gap = std::numeric_limits<double>::max();
    for (size_t i = 0; i < boxes.size(); ++i)
        for (size_t j = i + 1; j < boxes.size(); ++j)
            min_gap = std::min(min_gap, box_gap(boxes[i], boxes[j]));

    REQUIRE(min_gap >= double(scaled(8.)) - double(scaled(0.5)));
}

TEST_CASE("An item larger than the bed cannot be placed", "[Arrange]")
{
    ArrangePolygons items;
    items.emplace_back(make_square(scaled(20.)));
    items.emplace_back(make_square(scaled(400.))); // far bigger than the bed
    // An item that cannot be placed keeps the position it came in with, so the
    // caller can leave it where the user had it.
    items[1].translation = {scaled(10.), scaled(20.)};

    arrange(items, bed(200, 200), quiet_params(scaled(1.)));

    REQUIRE(items[0].bed_idx == 0);
    REQUIRE(items[1].bed_idx == UNARRANGED);
    REQUIRE(items[1].translation.x() == scaled(10.));
    REQUIRE(items[1].translation.y() == scaled(20.));
}

TEST_CASE("An item filling the printable area is arranged", "[Arrange]")
{
    // The arrange bed keeps a margin from the printable area and the item is widened by
    // its brim. Neither may keep an item that fits the printable area from being placed.
    DynamicPrintConfig cfg   = bed_config(); // 200 x 200
    ArrangeParams      p     = quiet_params();

    ArrangePolygons items = squares(1, 197.);
    items.front().brim_width = 1.;
    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);
    REQUIRE(items.front().bed_idx == 0);

    // Larger than the printable area is still refused.
    ArrangeParams   p_large = quiet_params();
    ArrangePolygons large   = squares(1, 201.);
    update_arrange_params(p_large, &cfg, large);
    update_selected_items_inflation(large, &cfg, p_large);
    arrange(large, get_shrink_bedpts(&cfg, p_large), p_large);
    REQUIRE(large.front().bed_idx == UNARRANGED);
}

TEST_CASE("Arranging uses the whole bed when the extruders share no printable area", "[Arrange]")
{
    // Two half-bed areas that only touch. An item wider than either half still fits the bed.
    DynamicPrintConfig cfg = bed_config(); // 200 x 200
    cfg.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups{
        { { 0., 0. }, { 100., 0. }, { 100., 200. }, { 0., 200. } },
        { { 100., 0. }, { 200., 0. }, { 200., 200. }, { 100., 200. } } });
    ArrangeParams p = quiet_params();

    ArrangePolygons items = squares(1, 150.);
    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    const Points bedpts = get_shrink_bedpts(&cfg, p);
    // The arrange jobs read the four corners.
    REQUIRE(bedpts.size() == 4);
    arrange(items, bedpts, p);
    CHECK(items.front().bed_idx == 0);

    // The bed still bounds the arrangement.
    ArrangeParams   p_large = quiet_params();
    ArrangePolygons large   = squares(1, 201.);
    update_arrange_params(p_large, &cfg, large);
    update_selected_items_inflation(large, &cfg, p_large);
    arrange(large, get_shrink_bedpts(&cfg, p_large), p_large);
    CHECK(large.front().bed_idx == UNARRANGED);
}

TEST_CASE("Items almost as wide as the bed are arranged on one bed", "[Arrange]")
{
    struct Case
    {
        std::string description;
        double      distance_mm;
        double      brim_width_mm;
        bool        align_to_y_axis;
        int         skirt_loops;
        double      gap_mm;
    };
    // A distance the user set is kept. A brim is cut to the 5 mm the bed leaves on each side of the
    // item, or to 2.5 mm with the skirt.
    auto c = GENERATE(values<Case>({
        {"with a distance the user set",       20., 0., false, 0, 20.},
        {"with a brim",                        0.,  8., false, 0, 10.},
        {"when turned to the Y axis",          0.,  8., true,  0, 10.},
        {"with a skirt",                       0.,  8., false, 1, 5.},
        {"with a distance and a skirt",        20., 0., false, 1, 20.},
    }));

    DYNAMIC_SECTION(c.description)
    {
        constexpr double   LINE_WIDTH_MM = 0.5;
        DynamicPrintConfig cfg = skirt_config(c.skirt_loops, ConfigOptionFloatOrPercent(LINE_WIDTH_MM, false));
        ArrangeParams      p   = quiet_params(scaled(c.distance_mm));
        p.align_to_y_axis      = c.align_to_y_axis;

        ArrangePolygons items(2, make_rect(scaled(190.), scaled(20.)));
        for (ArrangePolygon &ap : items)
            ap.brim_width = c.brim_width_mm;
        ArrangePolygons fixed;
        update_arrange_items(items, fixed, &cfg, p);
        arrange(items, get_shrink_bedpts(&cfg, p), p);

        // The skirt goes around the items, so they keep its room from the bed edge.
        const coord_t skirt_room = c.skirt_loops > 0 ? scaled(SKIRT_DISTANCE_MM + c.skirt_loops * LINE_WIDTH_MM) : 0;
        BoundingBox   printable  = bed(200, 200);
        printable.offset(-skirt_room);
        for (const ArrangePolygon &ap : items) {
            REQUIRE(ap.bed_idx == 0);
            REQUIRE(printable.contains(ap.transformed_poly().contour.bounding_box()));
        }
        // 0.5 mm slack for nester rounding.
        CHECK(box_gap(items[0].transformed_poly().contour.bounding_box(), items[1].transformed_poly().contour.bounding_box()) >=
              double(scaled(c.gap_mm - 0.5)));
    }
}

TEST_CASE("A distance the user set is kept when another item's brim is cut", "[Arrange]")
{
    DynamicPrintConfig cfg = bed_config();
    ArrangeParams      p   = quiet_params(scaled(20.));
    ArrangePolygons    items{make_rect(scaled(195.), scaled(20.)), make_rect(scaled(190.), scaled(20.))};
    for (ArrangePolygon &ap : items)
        ap.brim_width = 5.;

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        REQUIRE(bed(200, 200).contains(ap.transformed_poly().contour.bounding_box()));
    }
    // The 195 mm item keeps 7.5 mm of its inflation and the 190 mm item all 10 mm. 0.5 mm slack for nester rounding.
    CHECK(box_gap(items[0].transformed_poly().contour.bounding_box(), items[1].transformed_poly().contour.bounding_box()) >=
          double(scaled(17.5 - 0.5)));
}

TEST_CASE("A distance the user set is kept past the branches of tree supports", "[Arrange]")
{
    constexpr double   DISTANCE_MM = 20.;
    constexpr double   BRANCH_MM   = 10.; // tree support branch diameter, half of which lies around an item
    DynamicPrintConfig cfg         = bed_config();
    ArrangeParams      p           = quiet_params(scaled(DISTANCE_MM));
    ArrangePolygons    items(2, make_rect(scaled(190.), scaled(20.)));
    for (ArrangePolygon &ap : items) {
        ap.has_tree_support = true;
        ap.brim_width       = BRANCH_MM;
    }

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    // The arrange bed extends past the printable area by no more than the distance past the branches.
    const BoundingBox arrange_bed(get_shrink_bedpts(&cfg, p));
    CHECK(unscale<double>(arrange_bed.min.x()) >= -(DISTANCE_MM / 2 - BRANCH_MM / 2) - 1e-3);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    for (const ArrangePolygon &ap : items)
        REQUIRE(ap.bed_idx == 0);
    // 0.5 mm slack for nester rounding.
    CHECK(box_gap(items[0].transformed_poly().contour.bounding_box(), items[1].transformed_poly().contour.bounding_box()) >=
          double(scaled(DISTANCE_MM - 0.5)));
}

TEST_CASE("A distance the user set keeps the arrange bed inside a round bed", "[Arrange]")
{
    constexpr double   RADIUS_MM = 100.;
    DynamicPrintConfig cfg;
    std::vector<Vec2d> circle;
    for (int i = 0; i < 72; ++i)
        circle.emplace_back(RADIUS_MM + RADIUS_MM * std::cos(i * PI / 36), RADIUS_MM + RADIUS_MM * std::sin(i * PI / 36));
    cfg.set_key_value("printable_area", new ConfigOptionPoints(circle));
    ArrangeParams   p     = quiet_params(scaled(20.));
    ArrangePolygons items = squares(2, 20.);

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);

    const Vec2d center(RADIUS_MM, RADIUS_MM);
    for (const Point &pt : get_shrink_bedpts(&cfg, p))
        REQUIRE((unscaled(pt) - center).norm() <= RADIUS_MM + EPSILON);
}

TEST_CASE("An item cut to less inflation than the bed would grow keeps the bed from growing", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config();
    ArrangeParams      p     = quiet_params(scaled(36.));
    ArrangePolygons    items = {make_rect(scaled(199.), scaled(20.))};
    items.front().rotation   = PI / 18; // 10 degrees, so the corners of the inflated item lie past the inflation
    items.front().brim_width = 8.;

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    // The arrange bed extends past the printable area by no more than the item's inflation.
    const BoundingBox arrange_bed(get_shrink_bedpts(&cfg, p));
    CHECK(arrange_bed.min.x() >= -items.front().inflation);
    CHECK(arrange_bed.min.y() >= -items.front().inflation);

    arrange(items, get_shrink_bedpts(&cfg, p), p);
    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("An item filling the bed does not keep the bed from growing for the others", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config(); // 200 x 200
    ArrangeParams      p     = quiet_params(scaled(20.));
    ArrangePolygons    items = {make_rect(scaled(200.), scaled(20.)), make_rect(scaled(190.), scaled(20.))};

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    const BoundingBox printable = bed(200, 200).inflated(scaled(BuildVolume::BedEpsilon));
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        REQUIRE(printable.contains(ap.transformed_poly().contour.bounding_box()));
    }
    // 0.5 mm slack for nester rounding.
    CHECK(box_gap(items[0].transformed_poly().contour.bounding_box(), items[1].transformed_poly().contour.bounding_box()) >=
          double(scaled(20. - 0.5)));
}

TEST_CASE("An item too big for the bed is not placed with a distance the user set", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config(); // 200 x 200
    ArrangeParams      p     = quiet_params(scaled(20.));
    ArrangePolygons    items = {make_rect(scaled(205.), scaled(20.))};

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == UNARRANGED);
}

TEST_CASE("An item too big for the bed does not take the bed margin from the others", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config();
    ArrangeParams      p     = quiet_params();
    ArrangePolygons    items = {make_rect(scaled(210.), scaled(20.)), make_rect(scaled(20.), scaled(20.))};
    const float        margin_mm = p.bed_shrink_x;

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    const BoundingBox arrange_bed(get_shrink_bedpts(&cfg, p));
    CHECK_THAT(unscale<double>(arrange_bed.min.x()), WithinAbs(margin_mm, 1e-4));
    CHECK_THAT(unscale<double>(arrange_bed.min.y()), WithinAbs(margin_mm, 1e-4));
}

TEST_CASE("An item exactly as wide as the bed is arranged with its brim", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config(); // 200 x 200
    ArrangeParams      p     = quiet_params();
    ArrangePolygons    items = {make_rect(scaled(200.), scaled(50.))};
    items.front().brim_width = 1.;

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
}

TEST_CASE("An item turned by hand and almost as wide as the bed is arranged with its brim", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config();
    ArrangeParams      p     = quiet_params();
    ArrangePolygons    items = {make_rect(scaled(190.), scaled(20.))};
    items.front().rotation   = PI / 18; // 10 degrees, so the corners of the inflated item lie past the inflation
    items.front().brim_width = 8.;

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("A non-convex item turned by hand and almost as wide as the bed is arranged with its brim", "[Arrange]")
{
    DynamicPrintConfig cfg = bed_config(); // 200 x 200
    ArrangeParams      p   = quiet_params();
    ArrangePolygon     l_shape;
    l_shape.poly = ExPolygon(Polygon({{0, 0}, {scaled(190.), 0}, {scaled(190.), scaled(10.)},
                                      {scaled(20.), scaled(10.)}, {scaled(20.), scaled(60.)}, {0, scaled(60.)}}));
    l_shape.bed_idx    = 0;
    l_shape.rotation   = PI / 18; // 10 degrees, so the turned item is about 197.5 mm wide
    l_shape.brim_width = 8.;
    ArrangePolygons items{l_shape};

    update_arrange_params(p, &cfg, items);
    update_selected_items_inflation(items, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("With rotations allowed, an item that fits only on the diagonal of the bed is arranged", "[Arrange]")
{
    DynamicPrintConfig cfg   = bed_config(); // 200 x 200
    ArrangeParams      p     = quiet_params();
    p.allow_rotations        = true;
    ArrangePolygons    items = {make_rect(scaled(260.), scaled(20.))};
    items.front().brim_width = 1.;
    ArrangePolygons    fixed;

    update_arrange_items(items, fixed, &cfg, p);
    // The item keeps no inflation and gets the bed margin back.
    CHECK(items.front().inflation == 0);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("With rotations allowed, an item turned off the axis it fits is arranged", "[Arrange]")
{
    DynamicPrintConfig cfg;
    cfg.set_key_value("printable_area", new ConfigOptionPoints{{0, 0}, {250, 0}, {250, 210}, {0, 210}});
    ArrangeParams p   = quiet_params();
    p.allow_rotations = true;
    // Turned 45 degrees the item is about 219 mm wide and deep, more than the 210 mm the bed is deep.
    ArrangePolygons items    = {make_rect(scaled(249.5), scaled(60.))};
    items.front().rotation   = PI / 4;
    items.front().brim_width = 1.;
    ArrangePolygons fixed;

    update_arrange_items(items, fixed, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(250, 210).contains(items.front().transformed_poly().contour.bounding_box()));
}

TEST_CASE("With rotations allowed, items that have to be turned keep a distance the user set", "[Arrange]")
{
    constexpr double   DISTANCE_MM = 20.;
    DynamicPrintConfig cfg;
    cfg.set_key_value("printable_area", new ConfigOptionPoints{{0, 0}, {250, 0}, {250, 210}, {0, 210}});
    ArrangeParams p   = quiet_params(scaled(DISTANCE_MM));
    p.allow_rotations = true;
    // Turned to the Y axis the items are 240 mm deep, more than the 210 mm the bed is deep.
    ArrangePolygons items(2, make_rect(scaled(240.), scaled(20.)));
    for (ArrangePolygon &ap : items) {
        ap.rotation   = PI / 2;
        ap.brim_width = 1.;
    }
    ArrangePolygons fixed;

    update_arrange_items(items, fixed, &cfg, p);
    arrange(items, get_shrink_bedpts(&cfg, p), p);

    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        REQUIRE(bed(250, 210).contains(ap.transformed_poly().contour.bounding_box()));
    }
    // 0.5 mm slack for nester rounding.
    CHECK(box_gap(items[0].transformed_poly().contour.bounding_box(), items[1].transformed_poly().contour.bounding_box()) >=
          double(scaled(DISTANCE_MM - 0.5)));
}

TEST_CASE("The skirt room counts a percent line width against the nozzle", "[Arrange]")
{
    DynamicPrintConfig cfg = skirt_config(2, ConfigOptionFloatOrPercent(125., true));
    ArrangeParams      p   = quiet_params();

    update_arrange_params(p, &cfg, {});
    // Two loops 125% as wide as the 0.4 mm nozzle.
    CHECK_THAT(p.brim_skirt_distance, WithinAbs(SKIRT_DISTANCE_MM + 2 * 0.5, 1e-4));
}

TEST_CASE("The skirt room reads a line width stored as a plain number", "[Arrange]")
{
    DynamicPrintConfig cfg = skirt_config(2, ConfigOptionFloat(0.5));
    ArrangeParams      p   = quiet_params();

    update_arrange_params(p, &cfg, {});
    CHECK_THAT(p.brim_skirt_distance, WithinAbs(SKIRT_DISTANCE_MM + 2 * 0.5, 1e-4));
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

TEST_CASE("Arrange centers a pile that fits on the custom alignment point", "[Arrange]")
{
    ArrangePolygons items  = squares(4, 30.);
    ArrangeParams   params = quiet_params(scaled(2.));
    params.align_center    = Vec2d(0.3, 0.7);

    arrange(items, bed(250, 250), params);

    BoundingBox pile;
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        pile.merge(ap.transformed_poly().contour.bounding_box());
    }
    // best_object_pos is align_center scaled to the bed.
    const Point expected(scaled(250. * 0.3), scaled(250. * 0.7));
    REQUIRE(std::abs(pile.center().x() - expected.x()) <= scaled(0.5));
    REQUIRE(std::abs(pile.center().y() - expected.y()) <= scaled(0.5));
    require_no_overlap(items);
}

// A row of four 40 mm parts centred on the 30% mark of a 200 x 50 mm bed would start 24 mm
// left of it (#15727). The pile stops at the left edge instead.
TEST_CASE("Arrange keeps a custom-aligned pile within the bed", "[Arrange]")
{
    const BoundingBox bed_   = bed(200, 50);
    ArrangePolygons   items  = squares(4, 40.);
    ArrangeParams     params = quiet_params(scaled(2.));
    params.align_center      = Vec2d(0.3, 0.5);

    arrange(items, bed_, params);

    coord_t leftmost = std::numeric_limits<coord_t>::max();
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        const BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        CHECK(bed_.contains(bb));
        leftmost = std::min(leftmost, bb.min.x());
    }
    // Stopped at the edge it was aimed at, not re-centred.
    CHECK(leftmost < scaled(10.));
    require_no_overlap(items);
}

// Arranging a selection leaves the unselected parts where they are, but the final alignment
// centres the box around both, so the selection does not move by its own centring. The check
// against the bed and the fixed part must test where the selection actually lands.
TEST_CASE("Arrange keeps a selection on the bed and clear of a fixed part", "[Arrange]")
{
    ArrangePolygons fixed = squares(1, 40.);
    fixed.front().translation = {scaled(80.), scaled(80.)};
    ArrangePolygons items = squares(3, 40.);
    for (ArrangePolygon &ap : items)
        ap.inflation = scaled(1.);

    arrange(items, fixed, bed(200, 200), quiet_params());

    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        REQUIRE(bed(200, 200).contains(ap.transformed_poly().contour.bounding_box()));
    }
    items.insert(items.end(), fixed.begin(), fixed.end());
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

// Flat-bed printers with an off-centre best_object_pos (the A1 mini and the H2 family) get
// the same clamp: centring the pile on that point used to push it off the bed (#15727).
TEST_CASE("Arrange keeps a flat-bed pile aligned near an edge on the bed", "[Arrange]")
{
    const BoundingBox bed_   = bed(95, 500);
    ArrangePolygons   items  = squares(4, 90.);
    ArrangeParams     params = quiet_params(scaled(2.));
    params.align_center      = Vec2d(0.5, 0.05);

    arrange(items, bed_, params);

    coord_t lowest = std::numeric_limits<coord_t>::max();
    for (const ArrangePolygon &ap : items) {
        REQUIRE(ap.bed_idx == 0);
        const BoundingBox bb = ap.transformed_poly().contour.bounding_box();
        CHECK(bed_.contains(bb));
        lowest = std::min(lowest, bb.min.y());
    }
    // Stopped at the edge it was aimed at, not re-centred.
    CHECK(lowest < scaled(10.));
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

TEST_CASE("Sequential print keeps the clearance of an object almost as wide as the bed", "[Arrange]")
{
    ArrangePolygons items  = {make_rect(scaled(199.), scaled(20.))};
    items.front().height   = NOZZLE_HEIGHT_MM * 2;
    items.front().rotation = PI / 18; // 10 degrees, so the corners of the inflated object lie past the clearance
    DynamicPrintConfig cfg = bed_config();
    ArrangeParams      p   = seq_print_params(0);

    update_selected_items_inflation(items, &cfg, p);
    CHECK(p.min_obj_distance >= scaled(CLEARANCE_MM));
    CHECK(items.front().inflation == p.min_obj_distance / 2);
}

TEST_CASE("Sequential print with rotations allowed arranges an object that has to be turned", "[Arrange]")
{
    DynamicPrintConfig cfg = bed_config(); // 200 x 200
    ArrangeParams      p   = seq_print_params(scaled(40.));
    p.allow_rotations      = true;
    p.bed_shrink_x         = BED_SHRINK_SEQ_PRINT;
    p.bed_shrink_y         = BED_SHRINK_SEQ_PRINT;
    // Turned 10 degrees the object is about 205 mm wide, so it fits only turned back and with the bed margin given back.
    ArrangePolygons items     = {make_rect(scaled(198.), scaled(60.))};
    items.front().rotation    = PI / 18;
    items.front().height      = NOZZLE_HEIGHT_MM * 2;
    items.front().extrude_ids = {1}; // the sequential sort reads extrude_ids.front()
    ArrangePolygons fixed;

    update_arrange_items(items, fixed, &cfg, p);
    CHECK(items.front().inflation == p.min_obj_distance / 2);
    arrange(items, fixed, get_shrink_bedpts(&cfg, p), p);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
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
    return result;
}

double area(const ArrangePolygon &item)
{
    return item.transformed_poly().area();
}

// Serial evaluation keeps the placement independent of thread scheduling.
ArrangeParams serial_params(coord_t min_dist = 0)
{
    ArrangeParams params = quiet_params(min_dist);
    params.parallel      = false;
    return params;
}

bool inside_circle(const BoundingBox &box, const Point &center, double radius)
{
    for (const Point &corner : {box.min, Point(box.min.x(), box.max.y()),
                                Point(box.max.x(), box.min.y()), box.max}) {
        const double dx = double(corner.x() - center.x());
        const double dy = double(corner.y() - center.y());
        if (std::hypot(dx, dy) > radius)
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
        // Rotation about the origin first, then translation.
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

TEST_CASE("Arrange dispatches an empty point bed as an infinite bed centred on the origin", "[Arrange][ArrangeSurface]")
{
    // A 1 m square fits no finite printer bed, so only the infinite bed places it.
    ArrangePolygons items{rectangle(1000., 1000., 300., 400.)};
    arrange(items, Points{}, serial_params());
    REQUIRE(items.size() == 1);
    REQUIRE(std::isfinite(items.front().rotation));
    REQUIRE(items.front().bed_idx == 0);
    CHECK(items.front().transformed_poly().area() > 0.);
    const BoundingBox box = items.front().transformed_poly().contour.bounding_box();
    CHECK_THAT(unscaled<double>(box.center().x()), WithinAbs(0., .01));
    CHECK_THAT(unscaled<double>(box.center().y()), WithinAbs(0., .01));
}

TEST_CASE("Arrange dispatches a one-point bed as an infinite bed around that point", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(1000., 1000.)};
    arrange(items, Points{Point(scaled(100.), scaled(75.))}, serial_params());
    REQUIRE(items.size() == 1);
    REQUIRE(std::isfinite(items.front().rotation));
    REQUIRE(items.front().bed_idx == 0);
    const BoundingBox box = items.front().transformed_poly().contour.bounding_box();
    CHECK(box.defined);
    CHECK_THAT(unscaled<double>(box.center().x()), WithinAbs(100., .01));
    CHECK_THAT(unscaled<double>(box.center().y()), WithinAbs(75., .01));
}

TEST_CASE("Arrange keeps every part inside a rectangular point bed", "[Arrange][ArrangeSurface]")
{
    // The parts start outside the bed, so staying inside requires placement.
    ArrangePolygons items{rectangle(20., 10., -50.), rectangle(15., 15., -50.), rectangle(8., 22., -50.)};
    const Points bed{Point(0, 0), Point(scaled(100.), 0), Point(scaled(100.), scaled(80.)), Point(0, scaled(80.))};
    arrange(items, bed, serial_params(scaled(1.)));
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(BoundingBox(bed).contains(item.transformed_poly().contour.bounding_box()));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange treats equidistant bed points as a circular bed", "[Arrange][ArrangeSurface]")
{
    // The four cardinal points of a 100 mm circle. A 140x80 rectangle fits the
    // circle (half-diagonal 80.6 mm) but not the diamond those points enclose
    // (70 + 40 > 100), so it is placed only if the points become a CircleBed.
    // Rotations are disabled so no fitter can hide the dispatch choice.
    ArrangePolygons items{rectangle(140., 80.)};
    const Point center(scaled(100.), scaled(100.));
    const Points bed{Point(center.x() + scaled(100.), center.y()), Point(center.x(), center.y() + scaled(100.)),
                     Point(center.x() - scaled(100.), center.y()), Point(center.x(), center.y() - scaled(100.))};
    ArrangeParams params = serial_params();
    params.allow_rotations = false;
    arrange(items, bed, params);
    REQUIRE(items.front().bed_idx == 0);
    CHECK(inside_circle(items.front().transformed_poly().contour.bounding_box(), center, scaled(100.01)));
}

TEST_CASE("Arrange leaves an invalid contour unarranged while placing the valid contours before it", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon invalid;
    invalid.poly.contour.points = {Point(0, 0), Point(scaled(10.), 0)};
    ArrangePolygons items{rectangle(10., 10., 500.), invalid};
    arrange(items, bed(100., 100.), serial_params());
    REQUIRE(items[0].bed_idx == 0);
    CHECK(bed(100., 100.).contains(items[0].transformed_poly().contour.bounding_box()));
    CHECK(items[1].bed_idx == UNARRANGED);
}

TEST_CASE("Arrange keeps results aligned with their items when an invalid contour comes first", "[Arrange][ArrangeSurface]")
{
    // Callers seed bed_idx to 0 before arranging (see get_arrange_polys).
    ArrangePolygon invalid;
    invalid.poly.contour.points = {Point(0, 0), Point(scaled(10.), 0)};
    invalid.bed_idx = 0;
    invalid.translation = Vec2crd(scaled(5.), scaled(6.));
    invalid.rotation = .25;
    ArrangePolygons items{invalid, rectangle(10., 10., 500.)};
    arrange(items, bed(100., 100.), serial_params());
    CHECK(items[0].bed_idx == UNARRANGED);
    // The skipped item keeps its input transform.
    CHECK(items[0].translation == invalid.translation);
    CHECK_THAT(items[0].rotation, WithinAbs(.25, 1e-12));
    REQUIRE(items[1].bed_idx == 0);
    CHECK(bed(100., 100.).contains(items[1].transformed_poly().contour.bounding_box()));
}

TEST_CASE("Arrange keeps footprints inside a CircleBed and disjoint", "[Arrange][ArrangeSurface]")
{
    // The origin lies outside this circle, so the parts must be moved onto it.
    ArrangePolygons items{rectangle(20., 10.), rectangle(12., 18.), rectangle(9., 9.)};
    const std::vector<double> original_areas{
        items[0].transformed_poly().area(), items[1].transformed_poly().area(), items[2].transformed_poly().area()};
    const Point center(scaled(100.), scaled(100.));
    arrange(items, CircleBed(center, scaled(100.)), serial_params(scaled(1.)));
    REQUIRE(items.size() == original_areas.size());
    for (size_t i = 0; i < items.size(); ++i) {
        const ArrangePolygon &item = items[i];
        REQUIRE(std::isfinite(item.rotation));
        REQUIRE(item.bed_idx == 0);
        CHECK_THAT(item.transformed_poly().area(), WithinRel(original_areas[i], 1e-8));
        CHECK(inside_circle(item.transformed_poly().contour.bounding_box(), center, scaled(100.01)));
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange keeps convex polygon-bed footprints inside and disjoint", "[Arrange][ArrangeSurface]")
{
    // A hexagon that excludes the origin, where the parts start.
    const Polygon bed_poly{{Point(scaled(10.), scaled(10.)), Point(scaled(190.), scaled(10.)),
                            Point(scaled(200.), scaled(100.)), Point(scaled(190.), scaled(190.)),
                            Point(scaled(10.), scaled(190.)), Point(0, scaled(100.))}};
    ArrangePolygons items{rectangle(20., 20.), rectangle(18., 12.)};
    arrange(items, bed_poly, serial_params(scaled(1.)));
    REQUIRE(items.size() == 2);
    for (const ArrangePolygon &item : items) {
        REQUIRE(std::isfinite(item.rotation));
        REQUIRE(item.bed_idx == 0);
        const ExPolygon transformed = item.transformed_poly();
        REQUIRE_FALSE(transformed.contour.points.empty());
        REQUIRE(transformed.contour.bounding_box().defined);
        CHECK_THAT(transformed.area(), WithinRel(item.poly.area(), 1e-8));
        CHECK(diff_ex(transformed, bed_poly).empty());
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange keeps a fixed exclusion outside every arranged footprint", "[Arrange][ArrangeSurface]")
{
    // The fixed item covers the bed centre, where the pile would otherwise be aligned.
    const ArrangePolygon fixed = rectangle(30., 30., 45., 45.);
    ArrangePolygons items{rectangle(20., 20.), rectangle(18., 12.)};
    const ExPolygon fixed_shape = fixed.transformed_poly();
    arrange(items, {fixed}, bed(120., 120.), serial_params(scaled(1.)));
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(intersection_ex(item.transformed_poly(), fixed_shape).empty());
    }
    require_no_overlap(items);
}

TEST_CASE("Arrange rotates an oversized item to fit only when rotations are allowed", "[Arrange][ArrangeSurface]")
{
    // 120 mm wide does not fit a 50 mm wide bed, but 120 mm tall fits its 130 mm height.
    ArrangePolygons disabled{rectangle(120., 30.)};
    ArrangeParams no_rotations = serial_params();
    no_rotations.allow_rotations = false;
    arrange(disabled, bed(50., 130.), no_rotations);
    REQUIRE(disabled.size() == 1);
    REQUIRE(std::isfinite(disabled.front().rotation));
    CHECK(disabled.front().bed_idx == UNARRANGED);

    ArrangePolygons fitting{rectangle(35., 12.)};
    arrange(fitting, bed(100., 100.), no_rotations);
    REQUIRE(fitting.size() == 1);
    REQUIRE(std::isfinite(fitting.front().rotation));
    REQUIRE(fitting.front().bed_idx == 0);
    CHECK_THAT(fitting.front().rotation, WithinAbs(0., 1e-12));

    ArrangePolygons enabled{rectangle(120., 30.)};
    const double source_area = enabled.front().transformed_poly().area();
    ArrangeParams rotations = serial_params();
    rotations.allow_rotations = true;
    arrange(enabled, bed(50., 130.), rotations);
    REQUIRE(enabled.size() == 1);
    REQUIRE(std::isfinite(enabled.front().rotation));
    REQUIRE(enabled.front().bed_idx == 0);
    CHECK(std::abs(std::sin(enabled.front().rotation)) > .9);
    CHECK_THAT(enabled.front().transformed_poly().area(), WithinRel(source_area, 1e-6));
    CHECK(enabled.front().transformed_poly().contour.bounding_box().defined);
    CHECK(bed(50., 130.).contains(enabled.front().transformed_poly().contour.bounding_box()));
}

} // namespace arrange_surface_contracts

namespace arrange_param_contracts {
using Catch::Matchers::WithinAbs;
using arrange_surface_contracts::rectangle;

TEST_CASE("ArrangeParams serializes the distance and optimization controls as JSON", "[Arrange][ArrangeParams]")
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
    const nlohmann::json json = nlohmann::json::parse(params.to_json(), nullptr, false);
    REQUIRE_FALSE(json.is_discarded());
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

TEST_CASE("Axis alignment turns the long axis of an asymmetric contour onto Y", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items{rectangle(20., 60.)};
    items.front().rotation = .37;
    const double source_area = items.front().transformed_poly().area();
    ArrangeParams params = quiet_params();

    // The alignment only runs when it is requested.
    update_selected_items_axis_align(items, nullptr, params);
    REQUIRE(items.size() == 1);
    REQUIRE(std::isfinite(items.front().rotation));
    CHECK_THAT(items.front().rotation, WithinAbs(.37, 1e-12));

    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    REQUIRE(items.size() == 1);
    REQUIRE(std::isfinite(items.front().rotation));
    const BoundingBox box = items.front().transformed_poly().contour.bounding_box();
    CHECK_THAT(unscaled<double>(box.size().x()), WithinAbs(20., .01));
    CHECK_THAT(unscaled<double>(box.size().y()), WithinAbs(60., .01));
    CHECK_THAT(items.front().transformed_poly().area(), WithinRel(source_area, 1e-6));
}

TEST_CASE("Axis alignment leaves a contour without a dominant axis unrotated", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items{make_square(scaled(30.))};
    items.front().rotation = .37;
    ArrangeParams params = quiet_params();
    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    CHECK_THAT(items.front().rotation, WithinAbs(.37, 1e-12));
}

TEST_CASE("Axis alignment ignores a degenerate contour", "[Arrange][ArrangeParams]")
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

TEST_CASE("Selected inflation uses half the widest tree-support brim for the whole plate", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items = squares(2, 20.);
    items[0].brim_width = 1.;
    items[1].brim_width = 4.;
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params();

    // Without tree support each item keeps its own brim.
    update_selected_items_inflation(items, &config, params);
    CHECK(items[0].inflation == scaled(1.));
    CHECK(items[1].inflation == scaled(4.));

    items[1].has_tree_support = true;
    update_selected_items_inflation(items, &config, params);
    CHECK(items[0].inflation == scaled(2.));
    CHECK(items[1].inflation == scaled(2.));
}

TEST_CASE("Selected inflation gives an explicit minimum distance precedence over brim", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items = squares(1, 20.);
    items.front().brim_width = 8.;
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params(scaled(6.));
    update_selected_items_inflation(items, &config, params);
    CHECK(items.front().inflation == scaled(3.));
}

TEST_CASE("Unselected inflation distinguishes ordinary, virtual, and calibration regions", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items = squares(3, 10.);
    items[0].brim_width = 3.;
    items[1].is_virt_object = true;
    items[2].is_virt_object = true;
    items[2].is_extrusion_cali_object = true;
    const ArrangeParams params = quiet_params();
    update_unselected_items_inflation(items, nullptr, params);
    CHECK(items[0].inflation == scaled(3.));
    CHECK(items[1].inflation == scaled(1.));
    CHECK(items[2].inflation == 0);

    const DynamicPrintConfig empty_config;
    update_unselected_items_inflation(items, &empty_config, params);
    CHECK(items[0].inflation == scaled(3.));
    CHECK(items[1].inflation == scaled(1.));
    CHECK(items[2].inflation == 0);
}

// Only a grown bed (negative shrink) moves them; a real bed margin leaves them in place.
TEST_CASE("Sequential printing moves only virtual exclusion regions when the bed grows", "[Arrange][ArrangeParams]")
{
    ArrangePolygons items = squares(2, 10.);
    items[0].is_virt_object = true;
    const Point virt_before = items[0].poly.contour.points.front();
    const Point real_before = items[1].poly.contour.points.front();
    ArrangeParams params = quiet_params();
    params.is_seq_print = true;
    params.bed_shrink_x = -4.f;
    params.bed_shrink_y = -6.f;
    update_unselected_items_inflation(items, nullptr, params);
    CHECK(items[0].poly.contour.points.front() == Point(virt_before.x() - scaled(4.), virt_before.y() - scaled(6.)));
    CHECK(items[1].poly.contour.points.front() == real_before);

    ArrangePolygons margin_items = squares(1, 10.);
    margin_items[0].is_virt_object = true;
    const Point margin_before = margin_items[0].poly.contour.points.front();
    params.bed_shrink_x = 4.f;
    params.bed_shrink_y = 6.f;
    update_unselected_items_inflation(margin_items, nullptr, params);
    CHECK(margin_items[0].poly.contour.points.front() == margin_before);
}

TEST_CASE("Arrange parameter update adds the real skirt clearance in both print orders", "[Arrange][ArrangeParams]")
{
    // Skirt clearance is skirt_distance plus skirt_loops line widths: 3 + 2 * 0.4 = 3.8 mm.
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
    params.clearance_radius = 4.f;

    SECTION("by-layer printing adds the skirt clearance") {
        update_arrange_params(params, &config, {});
        CHECK_THAT(params.brim_skirt_distance, WithinAbs(3.8, 1e-5));
        CHECK_THAT(params.bed_shrink_x, WithinAbs(13.8, 1e-5));
        CHECK_THAT(params.bed_shrink_y, WithinAbs(14.8, 1e-5));
    }
    SECTION("sequential printing adds the same clearance; the bed grows later, with the item inflation") {
        params.is_seq_print = true;
        update_arrange_params(params, &config, {});
        CHECK_THAT(params.brim_skirt_distance, WithinAbs(3.8, 1e-5));
        CHECK_THAT(params.bed_shrink_x, WithinAbs(13.8, 1e-5));
        CHECK_THAT(params.bed_shrink_y, WithinAbs(14.8, 1e-5));
    }
}

TEST_CASE("Shrinking a rectangular bed moves every edge toward its center", "[Arrange][ArrangeParams]")
{
    DynamicPrintConfig config = bed_config();
    ArrangeParams params = quiet_params();
    params.bed_shrink_x = 5.f;
    params.bed_shrink_y = 7.f;
    const Points points = get_shrink_bedpts(&config, params);
    REQUIRE(points.size() == 4);
    const BoundingBox box(points);
    CHECK(box.min.x() == scaled(5.));
    CHECK(box.max.x() == scaled(195.));
    CHECK(box.min.y() == scaled(7.));
    CHECK(box.max.y() == scaled(193.));
}

} // namespace arrange_param_contracts

namespace model_arrange_contracts {
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

// One 20 mm cube object with an instance at each given XY offset.
Model arrange_model(const std::vector<Vec2d> &offsets = {Vec2d::Zero()})
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->name = "model-arrange-contract.stl";
    ModelVolume *volume = object->add_volume(TriangleMesh(its_make_cube(20., 20., 20.)));
    REQUIRE(volume != nullptr);
    REQUIRE(volume->get_convex_hull_shared_ptr() != nullptr);
    for (const Vec2d &offset : offsets) {
        ModelInstance *instance = object->add_instance();
        REQUIRE(instance != nullptr);
        instance->set_offset(to_3d(offset, 0.));
    }
    return model;
}

Model arrange_model(size_t instance_count)
{
    return arrange_model(std::vector<Vec2d>(instance_count, Vec2d::Zero()));
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

void require_bounded_arrange_polygon(const ArrangePolygon &polygon)
{
    REQUIRE_FALSE(polygon.poly.contour.points.empty());
    REQUIRE(polygon.poly.contour.points.size() <= 10000);
    CHECK(polygon.poly.contour.area() > 0.0);
    CHECK(std::isfinite(polygon.height));
    CHECK(std::isfinite(polygon.brim_width));
}

TEST_CASE("Model arrange polygon is the convex hull of every instance at its position and rotation", "[Arrange][ModelArrange]")
{
    // The first cube turned a quarter about Z covers x 30..50 at its offset (70 if left unrotated),
    // the second covers 0..20, so the hull is the 50x20 rectangle spanning both. Their inner corners
    // (x = 20 and x = 30) lie on its edges and must not be hull vertices.
    Model model = arrange_model({Vec2d(50., 0.), Vec2d(0., 0.)});
    model.objects.front()->instances.front()->set_rotation(Z, PI / 2.);

    const ArrangePolygon polygon = get_arrange_poly(model);
    CHECK(polygon.bed_idx == arrangement::UNARRANGED);
    CHECK(std::isfinite(polygon.height));
    CHECK(std::isfinite(polygon.brim_width));
    const Polygon       &hull    = polygon.poly.contour;
    CHECK(hull.is_counter_clockwise());
    CHECK_THAT(hull.area() * SCALING_FACTOR * SCALING_FACTOR, WithinRel(50. * 20., 1e-6));
    REQUIRE(hull.points.size() == 4);
    for (const Vec2d &corner : {Vec2d(0., 0.), Vec2d(50., 0.), Vec2d(50., 20.), Vec2d(0., 20.)})
        CHECK(std::any_of(hull.points.begin(), hull.points.end(), [&corner](const Point &p) {
            return (unscaled(p) - corner).norm() < .01;
        }));
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
    REQUIRE_FALSE(polygon.poly.contour.points.empty());
    REQUIRE(polygon.poly.contour.points.size() <= 10000);
    CHECK(polygon.poly.contour.area() > 0.);
    CHECK(std::isfinite(polygon.height));
    CHECK(std::isfinite(polygon.brim_width));
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
    ModelInstance *instance = object->instances.front();

    SECTION("unset object and volume assignments use the first filament") {
        REQUIRE_FALSE(object->config.has("extruder"));
        REQUIRE_FALSE(object->volumes.front()->config.has("extruder"));
        const ArrangePolygon polygon = get_instance_arrange_poly(instance, arrange_config("normal(auto)", false));
        REQUIRE(polygon.extrude_ids == std::vector<int>{1});
        CHECK(polygon.bed_temp == 65);
        CHECK(polygon.first_bed_temp == 70);
        CHECK(polygon.print_temp == 205);
        CHECK(polygon.first_print_temp == 215);
        CHECK(polygon.vitrify_temp == 90);
        CHECK(polygon.filament_temp_type != -1);
        CHECK(polygon.name == object->name);
        CHECK(polygon.height > 0.0);
        CHECK_THAT(polygon.brim_width, WithinAbs(1., 1e-9));
        CHECK_FALSE(polygon.has_tree_support);
    }
    SECTION("an explicit second filament keeps its own metadata") {
        object->volumes.front()->config.set("extruder", 2);
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

    // Reapply physical placements through the same fixture: they update the instances without virtual-bed callbacks.
    for (ModelInstance *instance : instances)
        instance->set_offset(Vec3d::Zero());
    ModelInstancePtrs physical_instances;
    ArrangePolygons physical_polygons = get_arrange_polys(model, physical_instances);
    REQUIRE(physical_polygons.size() == 3);
    REQUIRE(physical_instances.size() == 3);
    for (size_t i = 0; i < physical_polygons.size(); ++i) {
        physical_polygons[i].translation = Vec2crd{scaled(2. * i + 1.), scaled(2. * i + 2.)};
        physical_polygons[i].bed_idx = 0;
    }
    callback_beds.clear();
    REQUIRE(apply_arrange_polys(physical_polygons, physical_instances, [&callback_beds](ArrangePolygon &polygon) {
        callback_beds.push_back(polygon.bed_idx);
    }));
    CHECK(callback_beds.empty());
    for (size_t i = 0; i < physical_instances.size(); ++i) {
        CHECK_THAT(physical_instances[i]->get_offset(X), WithinAbs(2. * i + 1., 1e-9));
        CHECK_THAT(physical_instances[i]->get_offset(Y), WithinAbs(2. * i + 2., 1e-9));
    }
}

TEST_CASE("Model arrange reports success when every result is on the physical bed", "[Arrange][ModelArrange]")
{
    Model model = arrange_model({Vec2d::Zero(), Vec2d::Zero()});
    ModelInstancePtrs instances;
    ArrangePolygons polygons = get_arrange_polys(model, instances);
    REQUIRE(polygons.size() == 2);
    REQUIRE(instances.size() == 2);
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

namespace arrange_surface_contracts {
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
    // Check the complete object structurally rather than matching substrings.
    const std::string serialized = params.to_json();
    REQUIRE(!serialized.empty());
    REQUIRE(serialized.back() == '}');
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
TEST_CASE("Arrange dispatches an empty point bed as an infinite bed", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(12., 8.)};
    arrange(items, Points{}, quiet_params());
    REQUIRE(items.size() == 1);
    CHECK(items.front().bed_idx == 0);
    CHECK(area(items.front()) > 0.);
}
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
TEST_CASE("Arrange leaves invalid contours unarranged while placing valid contours", "[Arrange][ArrangeSurface]")
{
    ArrangePolygon invalid;
    invalid.poly.contour.points = {Point(0, 0), Point(scaled(10.), 0)};
    ArrangePolygons items{rectangle(10., 10.), invalid};
    arrange(items, bed(100., 100.), quiet_params());
    REQUIRE(items[0].bed_idx == 0);
    CHECK(items[1].bed_idx == UNARRANGED);
}
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
TEST_CASE("Arrange keeps an initially unrotated rectangular item unrotated when rotations are disabled", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(35., 12.)};
    ArrangeParams params = quiet_params();
    params.allow_rotations = false;
    arrange(items, bed(100., 100.), params);
    REQUIRE(items.front().bed_idx == 0);
    CHECK_THAT(items.front().rotation, WithinAbs(0., 1e-12));
}
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
TEST_CASE("Axis alignment leaves a symmetric square orientation unchanged", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(30., 30.)};
    items.front().rotation = .37;
    ArrangeParams params = quiet_params();
    params.align_to_y_axis = true;
    update_selected_items_axis_align(items, nullptr, params);
    CHECK_THAT(items.front().rotation, WithinAbs(.37, 1e-12));
}
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
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
} // namespace arrange_surface_contracts


namespace arrange_surface_contracts {
TEST_CASE("Sequential virtual exclusion regions move by a negative shrink vector", "[Arrange][ArrangeSurface]")
{
    ArrangePolygons items{rectangle(10., 10., 20., 30.)};
    items.front().is_virt_object = true;
    const Point before = items.front().poly.contour.points.front();
    ArrangeParams params = quiet_params();
    params.is_seq_print = true;
    params.bed_shrink_x = -4.f;
    params.bed_shrink_y = -6.f;
    update_unselected_items_inflation(items, nullptr, params);
    const Point after = items.front().poly.contour.points.front();
    CHECK(after.x() == before.x() - scaled(4.));
    CHECK(after.y() == before.y() - scaled(6.));
}
} // namespace arrange_surface_contracts


namespace model_arrange_contracts {
TEST_CASE("Model arrange polygon returns a bounded convex hull", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model(2);

    const ArrangePolygon polygon = get_arrange_poly(model);
    require_bounded_arrange_polygon(polygon);
    CHECK(polygon.bed_idx == arrangement::UNARRANGED);
}
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
TEST_CASE("Model arrange object duplication uses the original instance snapshot", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    ModelObject *object = model.objects.front();

    duplicate_objects(model, 4);

    REQUIRE(object->instances.size() == 4);
    CHECK(object->instances.front() != nullptr);
}
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
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
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
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
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
TEST_CASE("Model instance normal support selects the normal brim contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    DynamicPrintConfig config = arrange_config("normal(manual)", true);

    const ArrangePolygon polygon = get_instance_arrange_poly(model.objects.front()->instances.front(), config);

    CHECK_THAT(polygon.brim_width, Catch::Matchers::WithinAbs(6.0, 1e-9));
    CHECK_FALSE(polygon.has_tree_support);
}
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
TEST_CASE("Model instance tree support selects the tree brim contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    Model model = arrange_model();
    DynamicPrintConfig config = arrange_config("tree(manual)", true);

    const ArrangePolygon polygon = get_instance_arrange_poly(model.objects.front()->instances.front(), config);

    CHECK_THAT(polygon.brim_width, Catch::Matchers::WithinAbs(24.0, 1e-9));
    CHECK(polygon.has_tree_support);
}
} // namespace model_arrange_contracts


namespace model_arrange_contracts {
TEST_CASE("Model arrange out of bed callback has a stable exception contract", "[Arrange][ModelArrange][ModelArrangeContracts]")
{
    ArrangePolygon polygon;
    polygon.bed_idx = 1;

    REQUIRE_THROWS_AS(throw_if_out_of_bed(polygon), RuntimeError);
}
} // namespace model_arrange_contracts


TEST_CASE("An extruder without a printable area leaves the whole bed to arrange on", "[Arrange][BedShape]")
{
    const Vec2ds bed{Vec2d(0., 0.), Vec2d(20., 0.), Vec2d(20., 20.), Vec2d(0., 20.)};
    DynamicPrintConfig config;
    config.set_key_value("printable_area", new ConfigOptionPoints(bed));
    config.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups({bed, Vec2ds{}}));
    ArrangeParams params;
    CHECK(get_shrink_bedpts(&config, params).size() == 4);
    config.erase("extruder_printable_area");
    CHECK_FALSE(get_shrink_bedpts(&config, params).empty());
    // Missing configuration keeps the generic infinite-bed convention.
    config.erase("printable_area");
    CHECK(get_shrink_bedpts(&config, params).empty());
    config.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups({bed}));
    CHECK(get_shrink_bedpts(&config, params).empty());
}
// Short objects only need a nozzle's width between them, far less than the clearance
// radius, so the bed may only grow by half that width. A row of four 50 mm objects and
// their gaps is wider than the 200 mm bed but would fit one grown by half the clearance
// radius. Twenty of them overfill the bed, so the pile reaches its edges.
TEST_CASE("Sequential print keeps short objects on the bed and a nozzle apart", "[Arrange]")
{
    ArrangePolygons items = squares(20, 50., NOZZLE_HEIGHT_MM / 2);
    ArrangePolygons fixed;
    arrange_by_object(items, fixed);

    // Grown as Print::sequential_print_clearance_valid() grows them.
    ExPolygons grown;
    for (const ArrangePolygon &ap : items) {
        if (ap.bed_idx != 0)
            continue;
        CHECK(bed(200, 200).contains(ap.transformed_poly().contour.bounding_box()));
        append(grown, offset_ex(ap.transformed_poly(), float(scale_(0.5 * MAX_OUTER_NOZZLE_DIAMETER - 0.1))));
    }
    REQUIRE(grown.size() > 1);
    REQUIRE(disjoint(grown));
}

// Only the disc has to fit the bed; the half gap it is inflated by may reach past the
// edge. Within the bed margin of filling the bed, the arrange bed is cut to the disc's
// size, and rounding must not leave it a hair too small.
TEST_CASE("Sequential print arranges a short object filling the printable area", "[Arrange]")
{
    ArrangePolygons items{make_disc(196., NOZZLE_HEIGHT_MM / 2)};
    ArrangePolygons fixed;
    arrange_by_object(items, fixed);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

// The largest object and its clearance are wider than the bed less its margins, though
// the object itself fits within them, so every object keeps the margin.
TEST_CASE("Sequential print keeps the bed margin beside a large tall object", "[Arrange]")
{
    ArrangePolygons items = squares(3, 10., NOZZLE_HEIGHT_MM * 2);
    items.push_back(make_disc(170., NOZZLE_HEIGHT_MM * 2));
    ArrangePolygons fixed;
    arrange_by_object(items, fixed);

    const coord_t     margin = scaled(BED_SHRINK_SEQ_PRINT - 0.01); // 0.01 mm for rounding
    const BoundingBox inner(Point(margin, margin), Point(scaled(200.) - margin, scaled(200.) - margin));
    for (const ArrangePolygon &ap : items)
        if (ap.bed_idx == 0)
            CHECK(inner.contains(ap.transformed_poly().contour.bounding_box()));
}

// Short objects keep off an extrusion calibration region. One along the back edge catches
// the region being moved back by a positive bed shrink, which lets objects onto it.
TEST_CASE("Sequential print keeps short objects off a calibration region", "[Arrange]")
{
    const BoundingBox region_bb(Point(0, scaled(185.)), Point(scaled(200.), scaled(200.)));
    ArrangePolygon    region;
    region.poly                     = ExPolygon(region_bb.polygon());
    region.bed_idx                  = 0;
    region.is_virt_object           = true;
    region.is_extrusion_cali_object = true;
    ArrangePolygons items = squares(20, 40., NOZZLE_HEIGHT_MM / 2);
    ArrangePolygons fixed{region};
    arrange_by_object(items, fixed);

    for (const ArrangePolygon &ap : items)
        if (ap.bed_idx == 0)
            CHECK(ap.transformed_poly().contour.bounding_box().max.y() <= region_bb.min.y());
}

// The nester packs an object turned and inflated, and the inflated corners of a turned square
// reach past its inflation, so cutting the arrange bed to the object's size must measure that
// outline. The square is 196 mm wide turned 30 degrees.
TEST_CASE("Sequential print arranges a turned short object filling the printable area", "[Arrange]")
{
    ArrangePolygon item = make_square(scaled(196. / (std::cos(PI / 6) + std::sin(PI / 6))));
    item.height   = NOZZLE_HEIGHT_MM / 2;
    item.rotation = PI / 6;
    ArrangePolygons items{item};
    ArrangePolygons fixed;
    arrange_by_object(items, fixed);

    REQUIRE(items.front().bed_idx == 0);
    REQUIRE(bed(200, 200).contains(items.front().transformed_poly().contour.bounding_box()));
}

// An object closer to the bed edge than the bed margin, as one placed by hand, still takes its
// room when Fill bed adds objects around it, though its inflation crosses the arrange bed's edge.
TEST_CASE("Sequential print keeps short objects off one at the bed edge", "[Arrange]")
{
    ArrangePolygon obstacle = make_square(scaled(40.));
    obstacle.translation    = {scaled(1.), scaled(80.)};
    obstacle.height         = NOZZLE_HEIGHT_MM / 2;
    obstacle.extrude_ids    = {1};
    ArrangePolygons fixed{obstacle};
    ArrangePolygons items = squares(20, 40., NOZZLE_HEIGHT_MM / 2);
    arrange_by_object(items, fixed);

    ExPolygons placed{obstacle.transformed_poly()};
    for (const ArrangePolygon &ap : items)
        if (ap.bed_idx == 0)
            placed.emplace_back(ap.transformed_poly());
    REQUIRE(placed.size() > 1);
    REQUIRE(disjoint(placed));
}

TEST_CASE("Bed exclusions apply by physical nozzle and overlapping Z range", "[Arrange][ExclusionVolume][MultiNozzle]")
{
    ArrangePolygon object = make_square(scaled(20.));
    object.has_z_range = true;
    object.z_min = 0.0;
    object.z_max = 20.0;
    object.bed_exclusion_extruder_ids = {1};

    ArrangePolygon exclusion = make_square(scaled(20.));
    exclusion.is_bed_exclusion = true;
    exclusion.has_z_range = true;
    exclusion.z_min = 10.0;
    exclusion.z_max = 30.0;
    exclusion.bed_exclusion_extruder_id = 1;

    CHECK(bed_exclusion_applies(object, exclusion));

    exclusion.bed_exclusion_extruder_id = 0;
    CHECK_FALSE(bed_exclusion_applies(object, exclusion));

    // Empty physical-nozzle assignments mean unresolved, so arrangement stays conservative.
    object.bed_exclusion_extruder_ids.clear();
    CHECK(bed_exclusion_applies(object, exclusion));

    object.bed_exclusion_extruder_ids = {0};
    object.z_max = 9.0;
    CHECK_FALSE(bed_exclusion_applies(object, exclusion));

    // Touching at one Z plane is intentionally treated as a collision.
    object.z_max = 10.0;
    CHECK(bed_exclusion_applies(object, exclusion));
}

TEST_CASE("Shared bed exclusions apply to every arranged object", "[Arrange][ExclusionVolume]")
{
    ArrangePolygon object = make_square(scaled(20.));
    object.bed_exclusion_extruder_ids = {1};

    ArrangePolygon shared = make_square(scaled(20.));
    shared.is_bed_exclusion = true;
    shared.bed_exclusion_extruder_id = -1;

    CHECK(has_bed_exclusion_regions({shared}));
    CHECK_FALSE(has_bed_exclusion_regions({object}));
    CHECK(bed_exclusion_applies(object, shared));
}

TEST_CASE("Arrange keeps relevant objects outside conditional exclusion obstacles", "[Arrange][ExclusionVolume]")
{
    ArrangePolygon exclusion = make_square(scaled(30.));
    exclusion.translation = Vec2crd(scaled(35.0), scaled(35.0));
    exclusion.is_bed_exclusion = true;
    exclusion.bed_exclusion_extruder_id = 0;

    ArrangePolygons items = squares(4, 20.0);
    for (ArrangePolygon &item : items)
        item.bed_exclusion_extruder_ids = {0};

    arrange(items, {exclusion}, bed(100.0, 100.0), quiet_params(scaled(1.0)));

    const ExPolygon excluded_shape = exclusion.transformed_poly();
    for (const ArrangePolygon &item : items) {
        REQUIRE(item.bed_idx == 0);
        CHECK(intersection(ExPolygons{item.transformed_poly()}, ExPolygons{excluded_shape}).empty());
    }
    require_no_overlap(items);
}

TEST_CASE("Objects taller than the rod clearance stay out of the Y span of a compacted prime tower", "[Arrange]")
{
    // A tower spanning Y 180..240 at the right edge of the bed, with no extra growth on its polygon.
    ArrangePolygon tower;
    Polygon        outline;
    outline.points       = {Point(scaled(180.), scaled(180.)), Point(scaled(240.), scaled(180.)), Point(scaled(240.), scaled(240.)), Point(scaled(180.), scaled(240.))};
    tower.poly           = ExPolygon(outline);
    tower.bed_idx        = 0;
    tower.is_virt_object = true;
    tower.is_wipe_tower  = true;

    ArrangeParams params           = quiet_params(scaled(2.));
    params.clearance_height_to_rod = 24.f;
    params.clearance_dist_to_rod   = 40.f;
    params.compacted_tower_growth  = 0;

    // The pile gathers around the tower, so without the rule the objects end up beside it. A
    // best_object_pos away from the bed centre, as some printer profiles set, must not carry the
    // pile back into the span either.
    for (const Vec2d &align_center : {Vec2d(0.5, 0.5), Vec2d(0.3, 0.5), Vec2d(0.7, 0.5), Vec2d(0.5, 0.7)}) {
        INFO("best_object_pos " << align_center.x() << "x" << align_center.y());
        params.align_center   = align_center;
        ArrangePolygons items = squares(4, 40., 50.);
        arrange(items, {tower}, bed(250, 250), params);

        for (const ArrangePolygon &ap : items) {
            REQUIRE(ap.bed_idx == 0);
            CHECK(ap.transformed_poly().contour.bounding_box().max.y() <= scaled(180. - 40.));
        }
    }

    // Objects that stay under the rod may share the span.
    params.align_center = Vec2d(0.5, 0.5);
    ArrangePolygons low = squares(4, 40., 10.);
    arrange(low, {tower}, bed(250, 250), params);
    CHECK(std::any_of(low.begin(), low.end(), [](const ArrangePolygon &ap) {
        return ap.transformed_poly().contour.bounding_box().max.y() > scaled(180. - 40.);
    }));
}

TEST_CASE("Objects no taller than the nozzle height only keep the nozzle cone from a compacted prime tower", "[Arrange]")
{
    // A K2: toolhead radius 64 mm, nozzle height 4 mm.
    const double  toolhead_radius = 64.;
    ArrangeParams params;
    params.nozzle_height = 4.f;

    ArrangePolygon keychain;
    keychain.height = 3.;
    ArrangePolygon at_nozzle_height;
    at_nozzle_height.height = 4.;
    ArrangePolygon tall;
    tall.height = 30.;

    using Catch::Matchers::WithinAbs;
    CHECK_THAT(compacted_tower_arrange_clearance({keychain, at_nozzle_height}, params, toolhead_radius), WithinAbs(MAX_OUTER_NOZZLE_DIAMETER, EPSILON));
    // The tower is one polygon for all the objects placed, so one taller object gives every one of
    // them the toolhead radius.
    CHECK_THAT(compacted_tower_arrange_clearance({keychain, tall}, params, toolhead_radius), WithinAbs(toolhead_radius, EPSILON));
    CHECK_THAT(compacted_tower_arrange_clearance({tall}, params, toolhead_radius), WithinAbs(toolhead_radius, EPSILON));
    // find_new_position() passes no objects.
    CHECK_THAT(compacted_tower_arrange_clearance({}, params, toolhead_radius), WithinAbs(toolhead_radius, EPSILON));
}
