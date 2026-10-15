#include <catch2/catch_all.hpp>
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

    PaintedSegmentationFixture()
    {
        config = multifilament_config(2, {
            { "layer_height",                 0.2 },
            { "initial_layer_print_height",   0.2 },
            { "wall_loops",                   1 },
            { "top_shell_layers",             0 },
            { "bottom_shell_layers",          0 },
            { "sparse_infill_density",        "0%" },
            { "raft_layers",                  0 },
        });

        // A cube keeps the fixture small while giving the slicer vertical,
        // top, and bottom facets to project into the real segmentation path.
        init_print({ make_cube(20.0, 20.0, 4.0) }, print, model, config);
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

    PrintObject &object() { return *print.get_object(0); }
};

std::vector<std::vector<ExPolygons>> segment(PaintedSegmentationFixture &fixture,
                                             float                       max_width = 0.f,
                                             bool                        interlocking_beam = false)
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
        IncludeTopAndBottomLayers::Yes,
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
    return polygon_area(union_ex(all));
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
