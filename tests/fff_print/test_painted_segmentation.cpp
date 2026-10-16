#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "test_helpers.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/MultiMaterialSegmentation.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <libslic3r/Model.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/Surface.hpp>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

namespace {

constexpr size_t PAINTED_STATES = 3; // default plus two painted extruders.

struct PaintedSegmentationFixture {
    Model                model;
    Print                print;
    DynamicPrintConfig   config;

    PaintedSegmentationFixture(int top_shell_layers = 0, int bottom_shell_layers = 0,
                               bool horizontal_paint = false,
                               double max_width = 0., double interlocking_depth = 0.,
                               bool interlocking_beam = false)
    {
        config = multifilament_config(2, {
            { "layer_height",                 0.2 },
            { "initial_layer_print_height",   0.2 },
            { "wall_loops",                   1 },
            { "top_shell_layers",             top_shell_layers },
            { "bottom_shell_layers",          bottom_shell_layers },
            { "sparse_infill_density",        "0%" },
            { "raft_layers",                  0 },
            { "mmu_segmented_region_max_width", max_width },
            { "mmu_segmented_region_interlocking_depth", interlocking_depth },
            { "interlocking_beam",             interlocking_beam },
        });

        // A cube keeps the fixture small while giving the slicer vertical,
        // top, and bottom facets to project into the real segmentation path.
        init_print({ make_cube(20.0, 20.0, 4.0) }, print, model, config);
        if (horizontal_paint)
            paint_by_horizontal_facets();
        else
            paint_by_facet_x();
        print.apply(model, config);

        REQUIRE(print.objects().size() == 1);
        PrintObject &object = *print.get_object(0);
        object.slice();
        REQUIRE_FALSE(object.layers().empty());
    }

    void paint_by_facet_x()
    {
        REQUIRE(model.objects.size() == 1);
        ModelObject &object = *model.objects.front();
        REQUIRE(object.volumes.size() == 1);
        ModelVolume &volume = *object.volumes.front();

        const TriangleMesh &mesh = volume.mesh();
        const BoundingBoxf3 bounds = mesh.bounding_box();
        const float split_x = 0.5f * (bounds.min.x() + bounds.max.x());
        TriangleSelector selector(mesh);
        size_t state_one_facets = 0;
        size_t state_two_facets = 0;

        for (size_t facet_idx = 0; facet_idx < mesh.its.indices.size(); ++facet_idx) {
            const stl_triangle_vertex_indices &triangle = mesh.its.indices[facet_idx];
            const Vec3f centroid = (mesh.its.vertices[triangle[0]] +
                                    mesh.its.vertices[triangle[1]] +
                                    mesh.its.vertices[triangle[2]]) / 3.f;
            const EnforcerBlockerType state = centroid.x() < split_x ?
                EnforcerBlockerType::Extruder1 : EnforcerBlockerType::Extruder2;
            selector.set_facet(static_cast<int>(facet_idx), state);
            if (state == EnforcerBlockerType::Extruder1)
                ++state_one_facets;
            else
                ++state_two_facets;
        }

        REQUIRE(state_one_facets > 0);
        REQUIRE(state_two_facets > 0);
        REQUIRE(volume.mmu_segmentation_facets.set(selector));
    }

    void paint_by_horizontal_facets()
    {
        REQUIRE(model.objects.size() == 1);
        ModelObject &object = *model.objects.front();
        REQUIRE(object.volumes.size() == 1);
        ModelVolume &volume = *object.volumes.front();

        const TriangleMesh &mesh = volume.mesh();
        const BoundingBoxf3 bounds = mesh.bounding_box();
        constexpr float z_epsilon = 1e-4f;
        size_t bottom_facets = 0;
        size_t top_facets = 0;
        TriangleSelector selector(mesh);

        for (size_t facet_idx = 0; facet_idx < mesh.its.indices.size(); ++facet_idx) {
            const stl_triangle_vertex_indices &triangle = mesh.its.indices[facet_idx];
            const Vec3f centroid = (mesh.its.vertices[triangle[0]] +
                                    mesh.its.vertices[triangle[1]] +
                                    mesh.its.vertices[triangle[2]]) / 3.f;
            if (centroid.z() <= bounds.min.z() + z_epsilon) {
                selector.set_facet(static_cast<int>(facet_idx), EnforcerBlockerType::Extruder2);
                ++bottom_facets;
            } else if (centroid.z() >= bounds.max.z() - z_epsilon) {
                selector.set_facet(static_cast<int>(facet_idx), EnforcerBlockerType::Extruder1);
                ++top_facets;
            }
        }

        REQUIRE(bottom_facets > 0);
        REQUIRE(top_facets > 0);
        REQUIRE(volume.mmu_segmentation_facets.set(selector));
    }

    PrintObject &object() { return *print.get_object(0); }
};

std::vector<std::vector<ExPolygons>> segment(PaintedSegmentationFixture &fixture,
                                             float                       max_width = 0.f,
                                             bool                        interlocking_beam = false,
                                             IncludeTopAndBottomLayers   include_top_and_bottom_layers = IncludeTopAndBottomLayers::Yes)
{
    return segmentation_by_painting(
        fixture.object(),
        [](const ModelVolume &volume) -> ModelVolumeFacetsInfo {
            return { volume.mmu_segmentation_facets, !volume.mmu_segmentation_facets.empty(), false };
        },
        PAINTED_STATES,
        max_width,
        0.f,
        interlocking_beam,
        include_top_and_bottom_layers,
        [] {});
}

double polygon_area(const ExPolygons &expolygons)
{
    double result = 0.;
    for (const ExPolygon &expolygon : expolygons)
        result += expolygon.area();
    return result;
}

ExPolygons sliced_layer(const Layer &layer)
{
    ExPolygons result;
    for (const LayerRegion *region : layer.regions())
        for (const Surface &surface : region->slices.surfaces)
            result.push_back(surface.expolygon);
    return union_ex(result);
}

double region_area(const std::vector<ExPolygons> &regions)
{
    ExPolygons all;
    for (const ExPolygons &region : regions)
        all.insert(all.end(), region.begin(), region.end());
    return polygon_area(all);
}

double union_region_area(const std::vector<ExPolygons> &regions)
{
    ExPolygons all;
    for (const ExPolygons &region : regions)
        all.insert(all.end(), region.begin(), region.end());
    if (all.empty())
        return 0.;
    return polygon_area(union_ex(all));
}

double outside_area(const std::vector<ExPolygons> &regions, const ExPolygons &container)
{
    ExPolygons all;
    for (const ExPolygons &region : regions)
        all.insert(all.end(), region.begin(), region.end());
    if (all.empty())
        return 0.;
    return polygon_area(diff_ex(union_ex(all), container));
}

} // namespace

TEST_CASE("painted segmentation partitions cube slices by painted facet state", "[PaintedSegmentation]")
{
    PaintedSegmentationFixture fixture;
    const auto segmented = segment(fixture);
    REQUIRE(segmented.size() == fixture.object().layers().size());

    std::array<double, PAINTED_STATES - 1> state_areas{};
    size_t layers_with_both_states = 0;
    for (size_t layer_idx = 0; layer_idx < segmented.size(); ++layer_idx) {
        REQUIRE(segmented[layer_idx].size() == PAINTED_STATES - 1);
        const double expected = polygon_area(sliced_layer(*fixture.object().layers()[layer_idx]));
        REQUIRE(expected > 0.);

        const double assigned = region_area(segmented[layer_idx]);
        const double assigned_union = union_region_area(segmented[layer_idx]);
        const double tolerance = std::max(100.0, expected * 1e-5);
        CHECK_THAT(assigned, Catch::Matchers::WithinAbs(assigned_union, tolerance)); // no overlap between states
        CHECK_THAT(assigned_union, Catch::Matchers::WithinAbs(expected, tolerance)); // no area lost from the slice

        bool both_states = true;
        for (size_t state_idx = 0; state_idx < segmented[layer_idx].size(); ++state_idx) {
            const double state_area = polygon_area(segmented[layer_idx][state_idx]);
            state_areas[state_idx] += state_area;
            both_states = both_states && state_area > 0.;
        }
        if (both_states)
            ++layers_with_both_states;
    }

    CHECK(layers_with_both_states > 0);
    CHECK(state_areas[0] > 0.);
    CHECK(state_areas[1] > 0.);
}

TEST_CASE("painted segmentation max width trims only the non-beam result", "[PaintedSegmentation]")
{
    PaintedSegmentationFixture fixture;
    const auto unbounded = segment(fixture);
    const auto bounded = segment(fixture, 1.0f, false);
    const auto beam = segment(fixture, 1.0f, true);

    REQUIRE(unbounded.size() == bounded.size());
    REQUIRE(unbounded.size() == beam.size());

    double unbounded_area = 0.;
    double bounded_area = 0.;
    double beam_area = 0.;
    for (size_t layer_idx = 0; layer_idx < unbounded.size(); ++layer_idx) {
        unbounded_area += union_region_area(unbounded[layer_idx]);
        bounded_area += union_region_area(bounded[layer_idx]);
        beam_area += union_region_area(beam[layer_idx]);
    }

    CHECK(bounded_area < unbounded_area);
    CHECK_THAT(beam_area, Catch::Matchers::WithinAbs(unbounded_area, std::max(100.0, unbounded_area * 1e-5)));
}

TEST_CASE("painted top and bottom layers honor the include gate and shell depth", "[PaintedSegmentation][PaintedShellProjection]")
{
    const auto [bottom_shell_layers, top_shell_layers] = GENERATE(table<int, int>({
        { 1, 1 }, { 3, 2 }, { 0, 2 }, { 3, 0 }, { 0, 0 }
    }));
    CAPTURE(bottom_shell_layers, top_shell_layers);

    PaintedSegmentationFixture fixture(top_shell_layers, bottom_shell_layers, true);
    REQUIRE(fixture.object().layers().size() > size_t(bottom_shell_layers + top_shell_layers));
    const auto excluded = segment(fixture, 0.f, false, IncludeTopAndBottomLayers::No);
    const auto included = segment(fixture, 0.f, false, IncludeTopAndBottomLayers::Yes);
    REQUIRE(excluded.size() == fixture.object().layers().size());
    REQUIRE(included.size() == excluded.size());

    for (size_t layer_idx = 0; layer_idx < excluded.size(); ++layer_idx) {
        REQUIRE(excluded[layer_idx].size() == PAINTED_STATES - 1);
        REQUIRE(included[layer_idx].size() == PAINTED_STATES - 1);
        for (const ExPolygons &region : excluded[layer_idx])
            CHECK(region.empty());

        const bool bottom_expected = layer_idx < size_t(bottom_shell_layers);
        const bool top_expected = layer_idx + size_t(top_shell_layers) >= included.size();
        const bool painted_expected = bottom_expected || top_expected;
        const ExPolygons source = sliced_layer(*fixture.object().layers()[layer_idx]);
        const double source_area = polygon_area(source);
        REQUIRE(source_area > 0.);
        const double assigned = region_area(included[layer_idx]);
        const double assigned_union = union_region_area(included[layer_idx]);
        const double tolerance = std::max(100.0, source_area * 1e-5);

        CHECK_THAT(assigned, Catch::Matchers::WithinAbs(assigned_union, tolerance));
        CHECK(outside_area(included[layer_idx], source) <= tolerance);
        if (!painted_expected) {
            CHECK(assigned_union <= tolerance);
            continue;
        }

        CHECK(assigned_union > 0.);
        if (bottom_expected)
            CHECK(polygon_area(included[layer_idx][1]) > 0.);
        if (top_expected)
            CHECK(polygon_area(included[layer_idx][0]) > 0.);
        if (bottom_expected && !top_expected)
            CHECK(included[layer_idx][0].empty());
        if (top_expected && !bottom_expected)
            CHECK(included[layer_idx][1].empty());
    }
}

TEST_CASE("painted segmentation wrapper honors width depth parity and beam bypass", "[PaintedSegmentation][PaintedCutWidth]")
{
    const auto [mode, mode_name] = GENERATE(table<int, const char *>({
        { 0, "valid-depth" }, { 1, "zero-depth" }, { 2, "beam-bypass" }
    }));
    CAPTURE(mode_name);

    if (mode == 0) {
        PaintedSegmentationFixture fixture(0, 0, false, 1.0, 0.5, false);
        const auto segmented = multi_material_segmentation_by_painting(fixture.object(), [] {});
        REQUIRE(segmented.size() == fixture.object().layers().size());
        REQUIRE(segmented.size() >= 2);

        for (size_t layer_idx = 0; layer_idx + 1 < segmented.size(); layer_idx += 2) {
            REQUIRE(segmented[layer_idx].size() == PAINTED_STATES - 1);
            REQUIRE(segmented[layer_idx + 1].size() == PAINTED_STATES - 1);
            const ExPolygons even_source = sliced_layer(*fixture.object().layers()[layer_idx]);
            const ExPolygons odd_source = sliced_layer(*fixture.object().layers()[layer_idx + 1]);
            const double even_source_area = polygon_area(even_source);
            const double odd_source_area = polygon_area(odd_source);
            REQUIRE(even_source_area > 0.);
            REQUIRE(odd_source_area > 0.);

            const double even_assigned = region_area(segmented[layer_idx]);
            const double odd_assigned = region_area(segmented[layer_idx + 1]);
            const double even_area = union_region_area(segmented[layer_idx]);
            const double odd_area = union_region_area(segmented[layer_idx + 1]);
            // Polygon areas are in squared scaled coordinates, not square millimeters.
            const double even_tolerance = std::max(100.0, even_source_area * 1e-5);
            const double odd_tolerance = std::max(100.0, odd_source_area * 1e-5);
            CHECK_THAT(even_assigned, Catch::Matchers::WithinAbs(even_area, even_tolerance));
            CHECK_THAT(odd_assigned, Catch::Matchers::WithinAbs(odd_area, odd_tolerance));
            CHECK(outside_area(segmented[layer_idx], even_source) <= even_tolerance);
            CHECK(outside_area(segmented[layer_idx + 1], odd_source) <= odd_tolerance);
            REQUIRE(even_area > 0.);
            REQUIRE(odd_area > 0.);
            CHECK(even_area < even_source_area - even_tolerance);
            CHECK(odd_area < odd_source_area - odd_tolerance);
            CHECK(even_area + even_tolerance < odd_area);
        }
    } else if (mode == 1) {
        PaintedSegmentationFixture fixture(0, 0, false, 1.0, 0., false);
        const auto segmented = multi_material_segmentation_by_painting(fixture.object(), [] {});
        REQUIRE_FALSE(segmented.empty());
        REQUIRE(segmented.size() == fixture.object().layers().size());
        const double reference = union_region_area(segmented.front());
        REQUIRE(reference > 0.);
        for (size_t layer_idx = 0; layer_idx < segmented.size(); ++layer_idx) {
            REQUIRE(segmented[layer_idx].size() == PAINTED_STATES - 1);
            const ExPolygons source = sliced_layer(*fixture.object().layers()[layer_idx]);
            REQUIRE(polygon_area(source) > 0.);
            const double area = union_region_area(segmented[layer_idx]);
            REQUIRE(area > 0.);
            CHECK_THAT(area, Catch::Matchers::WithinAbs(reference, std::max(100.0, reference * 1e-5)));
        }
    } else {
        PaintedSegmentationFixture beam_fixture(0, 0, false, 1.0, 0.5, true);
        PaintedSegmentationFixture unbounded_fixture;
        const auto beam = multi_material_segmentation_by_painting(beam_fixture.object(), [] {});
        const auto unbounded = multi_material_segmentation_by_painting(unbounded_fixture.object(), [] {});
        REQUIRE_FALSE(beam.empty());
        REQUIRE_FALSE(unbounded.empty());
        REQUIRE(beam.size() == beam_fixture.object().layers().size());
        REQUIRE(unbounded.size() == unbounded_fixture.object().layers().size());
        REQUIRE(beam.size() == unbounded.size());
        for (size_t layer_idx = 0; layer_idx < beam.size(); ++layer_idx) {
            REQUIRE(beam[layer_idx].size() == PAINTED_STATES - 1);
            REQUIRE(unbounded[layer_idx].size() == PAINTED_STATES - 1);
            const ExPolygons source = sliced_layer(*beam_fixture.object().layers()[layer_idx]);
            REQUIRE(polygon_area(source) > 0.);
            const double beam_area = union_region_area(beam[layer_idx]);
            const double unbounded_area = union_region_area(unbounded[layer_idx]);
            REQUIRE(beam_area > 0.);
            REQUIRE(unbounded_area > 0.);
            CHECK(outside_area(beam[layer_idx], source) <= std::max(100.0, unbounded_area * 1e-5));
            CHECK_THAT(beam_area,
                       Catch::Matchers::WithinAbs(unbounded_area,
                                                  std::max(100.0, unbounded_area * 1e-5)));
        }
    }
}
