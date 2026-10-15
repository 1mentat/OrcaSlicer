#include "libslic3r/libslic3r.h"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/ExPolygon.hpp"
#include <utility>
#include <cstddef>
#include <catch2/catch_all.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "test_helpers.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Surface.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

struct RegionEvidence {
    double area = 0.;
    size_t surfaces = 0;
    size_t internal_surfaces = 0;
};

struct InterlockingFixture {
    Model model;
    Print print;
    DynamicPrintConfig config;

    explicit InterlockingFixture(bool enabled, double gap = 0., int boundary_avoidance = 2,
                                 int beam_layers = 2, int depth = 2, double beam_width = 0.8,
                                 double orientation = 22.5, bool different_extruders = true,
                                 double height = 4., double part_width = 10.)
        : config(multifilament_config(2, {
            {"layer_height", 0.2},
            {"initial_layer_print_height", 0.2},
            {"wall_loops", 2},
            {"top_shell_layers", 0},
            {"bottom_shell_layers", 0},
            {"sparse_infill_density", "0%"},
            {"skirt_loops", 0},
            {"brim_type", "no_brim"},
            {"layer_change_gcode", "G92 E0\n"},
            {"single_extruder_multi_material", 0},
            {"nozzle_diameter", "0.4,0.4"},
            {"printer_extruder_id", "1,2"},
            {"interlocking_beam", enabled},
            {"interlocking_beam_width", beam_width},
            {"interlocking_orientation", orientation},
            {"interlocking_beam_layer_count", beam_layers},
            {"interlocking_depth", depth},
            {"interlocking_boundary_avoidance", boundary_avoidance},
        }))
    {
        ModelObject *object = model.add_object();
        auto *left = object->add_volume(make_cube(part_width, 20., height), ModelVolumeType::MODEL_PART, false);
        TriangleMesh right_mesh = make_cube(part_width, 20., height);
        right_mesh.translate(float(part_width + gap), 0.f, 0.f);
        auto *right = object->add_volume(std::move(right_mesh), ModelVolumeType::MODEL_PART, false);
        left->config.set("extruder", 1);
        right->config.set("extruder", different_extruders ? 2 : 1);
        object->add_instance();
        object->ensure_on_bed();
        print.auto_assign_extruders(object);
        print.apply(model, config);
        const auto validation = print.validate();
        INFO("validation error: " << validation.string);
        REQUIRE(validation.string.empty());
        print.process();
        REQUIRE(print.objects().size() == 1);
        REQUIRE(print.objects().front()->num_printing_regions() == (different_extruders ? 2u : 1u));
    }
};

std::vector<RegionEvidence> evidence(const PrintObject &object)
{
    std::vector<RegionEvidence> result(object.num_printing_regions());
    for (const Layer *layer : object.layers()) {
        for (size_t region_idx = 0; region_idx < object.num_printing_regions(); ++region_idx) {
            const auto *region = layer->get_region(region_idx);
            for (const Surface &surface : region->slices.surfaces) {
                result[region_idx].area += surface.expolygon.area();
                ++result[region_idx].surfaces;
                if (surface.surface_type == stInternal)
                    ++result[region_idx].internal_surfaces;
            }
        }
    }
    return result;
}

std::vector<std::array<double, 2>> layer_signatures(const PrintObject &object, size_t region_idx)
{
    std::vector<std::array<double, 2>> result;
    result.reserve(object.layers().size());
    for (const Layer *layer : object.layers()) {
        double area = 0.;
        double boundary = 0.;
        for (const Surface &surface : layer->get_region(region_idx)->slices.surfaces) {
            area += surface.expolygon.area();
            boundary += surface.expolygon.contour.length();
            for (const Polygon &hole : surface.expolygon.holes)
                boundary += hole.length();
        }
        result.push_back({area, boundary});
    }
    return result;
}

ExPolygons layer_geometry(const PrintObject &object, size_t layer_idx, size_t region_idx)
{
    return union_ex(to_expolygons(object.layers()[layer_idx]->get_region(region_idx)->slices.surfaces));
}

double geometry_area(const ExPolygons &geometry)
{
    double area = 0.;
    for (const ExPolygon &expolygon : geometry)
        area += expolygon.area();
    return area;
}

double symmetric_difference_area(const ExPolygons &a, const ExPolygons &b)
{
    return geometry_area(xor_ex(a, b));
}

double area_mm2(double scaled_area)
{
    const double one_mm = double(scale_(1.));
    return scaled_area / (one_mm * one_mm);
}

double union_area(const PrintObject &object, size_t layer_idx)
{
    ExPolygons all;
    for (size_t region_idx = 0; region_idx < object.num_printing_regions(); ++region_idx)
        expolygons_append(all, layer_geometry(object, layer_idx, region_idx));
    return geometry_area(union_ex(all));
}

double all_layer_symmetric_difference(const PrintObject &a, const PrintObject &b, size_t region_idx)
{
    if (a.layers().size() != b.layers().size())
        return std::numeric_limits<double>::infinity();
    double area = 0.;
    for (size_t layer_idx = 0; layer_idx < a.layers().size(); ++layer_idx)
        area += symmetric_difference_area(layer_geometry(a, layer_idx, region_idx), layer_geometry(b, layer_idx, region_idx));
    return area;
}

bool signatures_differ(const std::vector<std::array<double, 2>> &a,
                       const std::vector<std::array<double, 2>> &b)
{
    if (a.size() != b.size())
        return true;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i][0] - b[i][0]) > 1e-3)
            return true;
        if (std::abs(a[i][1] - b[i][1]) > 1.)
            return true;
    }
    return false;
}

double total_union_area(const PrintObject &object)
{
    double total = 0.;
    for (size_t layer_idx = 0; layer_idx < object.layer_count(); ++layer_idx)
        total += union_area(object, layer_idx);
    return total;
}

} // namespace

TEST_CASE("Disabled interlocking leaves adjacent multipart region geometry unchanged", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture fixture(false);
    const auto *object = fixture.print.objects().front();
    const auto regions = evidence(*object);
    REQUIRE(regions.size() == 2);
    CHECK(regions[0].area > 0.);
    CHECK(regions[1].area > 0.);
    CHECK(regions[0].surfaces > 0);
    CHECK(regions[1].surfaces > 0);
    REQUIRE(object->layers().size() >= 2);
    for (size_t layer_idx = 0; layer_idx < object->layers().size(); ++layer_idx) {
        CHECK_THAT(area_mm2(union_area(*object, layer_idx)), Catch::Matchers::WithinAbs(400., 0.01));
        CHECK_THAT(area_mm2(geometry_area(layer_geometry(*object, layer_idx, 0))), Catch::Matchers::WithinAbs(200., 0.01));
        CHECK_THAT(area_mm2(geometry_area(layer_geometry(*object, layer_idx, 1))), Catch::Matchers::WithinAbs(200., 0.01));
    }
}

TEST_CASE("Enabled interlocking conserves union area while redistributing the shared interface", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture disabled(false);
    InterlockingFixture enabled(true);
    const auto before = evidence(*disabled.print.objects().front());
    const auto after = evidence(*enabled.print.objects().front());
    REQUIRE(before.size() == 2);
    REQUIRE(after.size() == 2);
    CHECK_THAT(total_union_area(*enabled.print.objects().front()), Catch::Matchers::WithinRel(total_union_area(*disabled.print.objects().front()), 1e-5));
    for (size_t layer_idx = 0; layer_idx < enabled.print.objects().front()->layers().size(); ++layer_idx)
        CHECK_THAT(area_mm2(union_area(*enabled.print.objects().front(), layer_idx)), Catch::Matchers::WithinAbs(400., 0.01));
    CHECK(after[0].area > 0.);
    CHECK(after[1].area > 0.);
    CHECK(after[0].internal_surfaces + after[1].internal_surfaces > 0);
    CHECK(std::abs(after[0].area - before[0].area) > 1.);
    CHECK(std::abs(after[1].area - before[1].area) > 1.);
}

TEST_CASE("Interlocking ignores multipart regions that use the same extruder", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture disabled(false);
    InterlockingFixture same_extruder(true, 0., 2, 2, 2, 0.8, 22.5, false);
    const auto before = evidence(*disabled.print.objects().front());
    const auto after = evidence(*same_extruder.print.objects().front());
    REQUIRE(before.size() == 2);
    REQUIRE(after.size() == 1);
    CHECK_THAT(after.front().area, Catch::Matchers::WithinRel(before[0].area + before[1].area, 1e-5));
}

TEST_CASE("Interlocking gate rejects zero beam depth and width without changing slices", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture baseline(false);
    InterlockingFixture zero_depth(true, 0., 2, 2, 0, 0.8);
    InterlockingFixture zero_width(true, 0., 2, 2, 2, 0.0);
    const auto expected = evidence(*baseline.print.objects().front());
    const auto depth = evidence(*zero_depth.print.objects().front());
    const auto width = evidence(*zero_width.print.objects().front());
    REQUIRE(expected.size() == 2);
    REQUIRE(depth.size() == 2);
    REQUIRE(width.size() == 2);
    for (size_t region_idx = 0; region_idx < 2; ++region_idx) {
        CHECK_THAT(area_mm2(all_layer_symmetric_difference(*baseline.print.objects().front(), *zero_depth.print.objects().front(), region_idx)), Catch::Matchers::WithinAbs(0., 0.01));
        CHECK_THAT(area_mm2(all_layer_symmetric_difference(*baseline.print.objects().front(), *zero_width.print.objects().front(), region_idx)), Catch::Matchers::WithinAbs(0., 0.01));
        CHECK_THAT(depth[region_idx].area, Catch::Matchers::WithinRel(expected[region_idx].area, 1e-5));
        CHECK_THAT(width[region_idx].area, Catch::Matchers::WithinRel(expected[region_idx].area, 1e-5));
    }
}

TEST_CASE("Interlocking beam layer count changes the vertical ownership pattern", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture one_layer(true, 0., 0, 1, 2, 0.8);
    InterlockingFixture two_layers(true, 0., 0, 2, 2, 0.8);
    const auto one = evidence(*one_layer.print.objects().front());
    const auto two = evidence(*two_layers.print.objects().front());
    REQUIRE(one.size() == 2);
    REQUIRE(two.size() == 2);
    CHECK(one[0].internal_surfaces + one[1].internal_surfaces > 0);
    CHECK(two[0].internal_surfaces + two[1].internal_surfaces > 0);
    const auto one_a = layer_signatures(*one_layer.print.objects().front(), 0);
    const auto one_b = layer_signatures(*one_layer.print.objects().front(), 1);
    const auto two_a = layer_signatures(*two_layers.print.objects().front(), 0);
    const auto two_b = layer_signatures(*two_layers.print.objects().front(), 1);
    CHECK(signatures_differ(one_a, two_a));
    CHECK(signatures_differ(one_b, two_b));
}

TEST_CASE("Interlocking retains a partial final beam group", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture odd_height(true, 0., 0, 2, 2, 0.8, 22.5, true, 4.2);
    const auto *object = odd_height.print.objects().front();
    REQUIRE(object->layers().size() % 2 == 1);
    REQUIRE(object->layers().size() >= 3);
    for (size_t layer_idx = 0; layer_idx < object->layers().size(); ++layer_idx)
        CHECK_THAT(area_mm2(union_area(*object, layer_idx)), Catch::Matchers::WithinAbs(400., 0.01));
}

TEST_CASE("Interlocking orientation changes the ownership partition without changing total union", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture horizontal(true, 0., 0, 2, 2, 0.8, 0.);
    InterlockingFixture rotated(true, 0., 0, 2, 2, 0.8, 90.);
    const auto h = evidence(*horizontal.print.objects().front());
    const auto r = evidence(*rotated.print.objects().front());
    REQUIRE(h.size() == 2);
    REQUIRE(r.size() == 2);
    CHECK_THAT(total_union_area(*horizontal.print.objects().front()), Catch::Matchers::WithinRel(total_union_area(*rotated.print.objects().front()), 1e-5));
    const auto h_a = layer_signatures(*horizontal.print.objects().front(), 0);
    const auto h_b = layer_signatures(*horizontal.print.objects().front(), 1);
    const auto r_a = layer_signatures(*rotated.print.objects().front(), 0);
    const auto r_b = layer_signatures(*rotated.print.objects().front(), 1);
    CHECK(signatures_differ(h_a, r_a));
    CHECK(signatures_differ(h_b, r_b));
}

TEST_CASE("Interlocking boundary avoidance changes outer geometry while preserving union", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture no_filter(true, 0., 0, 2, 2, 0.8);
    InterlockingFixture filtered(true, 0., 2, 2, 2, 0.8);
    const auto a = evidence(*no_filter.print.objects().front());
    const auto b = evidence(*filtered.print.objects().front());
    REQUIRE(a.size() == 2);
    REQUIRE(b.size() == 2);
    CHECK(a[0].area > 0.);
    CHECK(a[1].area > 0.);
    CHECK(b[0].area > 0.);
    CHECK(b[1].area > 0.);
    CHECK_THAT(total_union_area(*no_filter.print.objects().front()), Catch::Matchers::WithinRel(total_union_area(*filtered.print.objects().front()), 1e-5));
    CHECK(area_mm2(all_layer_symmetric_difference(*no_filter.print.objects().front(), *filtered.print.objects().front(), 0)) > 0.01);
    CHECK(area_mm2(all_layer_symmetric_difference(*no_filter.print.objects().front(), *filtered.print.objects().front(), 1)) > 0.01);
}

TEST_CASE("Interlocking cell width changes the periodic interface signature", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture narrow(true, 0., 0, 2, 2, 0.65, 0.);
    InterlockingFixture wide(true, 0., 0, 2, 2, 1.35, 0.);
    const auto narrow_a = layer_signatures(*narrow.print.objects().front(), 0);
    const auto wide_a = layer_signatures(*wide.print.objects().front(), 0);
    REQUIRE(narrow_a.size() == wide_a.size());
    CHECK(signatures_differ(narrow_a, wide_a));
    CHECK_THAT(total_union_area(*narrow.print.objects().front()), Catch::Matchers::WithinRel(total_union_area(*wide.print.objects().front()), 1e-5));

    // A 2.4 mm half exposes a partial 2*beam_width cell at the outer edge.
    // The configured two-beam cell therefore leaves a measurable ownership
    // imbalance on the first layer; halving cell_width incorrectly tiles the
    // half with a different phase and makes the two regions equal.
    InterlockingFixture partial(true, 0., 0, 2, 2, 0.8, 0., true, 4., 2.4);
    const auto *partial_object = partial.print.objects().front();
    const auto partial_a = geometry_area(layer_geometry(*partial_object, 0, 0));
    const auto partial_b = geometry_area(layer_geometry(*partial_object, 0, 1));
    CHECK(std::abs(area_mm2(partial_a) - area_mm2(partial_b)) > 0.1);
}

TEST_CASE("Interlocking does not bridge separated multipart components", "[Interlocking][InterlockingVariance]")
{
    InterlockingFixture disabled(false, 2.0);
    InterlockingFixture enabled(true, 2.0);
    const auto *before = disabled.print.objects().front();
    const auto *after = enabled.print.objects().front();
    REQUIRE(before->num_printing_regions() == 2);
    REQUIRE(after->num_printing_regions() == 2);
    CHECK_THAT(total_union_area(*after), Catch::Matchers::WithinRel(total_union_area(*before), 1e-5));
    for (size_t region_idx = 0; region_idx < 2; ++region_idx)
        CHECK_THAT(area_mm2(all_layer_symmetric_difference(*before, *after, region_idx)), Catch::Matchers::WithinAbs(0., 0.01));
}
