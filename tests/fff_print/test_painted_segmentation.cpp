#include <catch2/catch_all.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_message.hpp>

#include "catch2/catch_test_macros.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "test_helpers.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/MultiMaterialSegmentation.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <libslic3r/Model.hpp>
#include <libslic3r/PrintConfig.hpp>
#include <libslic3r/TriangleMesh.hpp>
#include <libslic3r/Point.hpp>
#include <libslic3r/Surface.hpp>
#include <vector>
#include <utility>

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
                               bool interlocking_beam = false,
                               bool fuzzy_fixture = false)
    {
        config = multifilament_config(fuzzy_fixture ? 1 : 2, {
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
        if (fuzzy_fixture) {
            // This route deliberately has one ordinary filament and no MMU paint
            // state, so the fuzzy annotation is the only facet-state input.
            config.set_deserialize_strict({
                { "single_extruder_multi_material", false },
                { "fuzzy_skin",                  "external" },
                { "outer_wall_line_width",       0.5 },
                { "nozzle_diameter",             "0.4" },
                { "xy_hole_compensation",        0.0 },
                { "xy_contour_compensation",    0.0 },
                { "elefant_foot_compensation",   0.0 },
                { "slice_closing_radius",        0.0 },
            });
        }

        // A cube keeps the fixture small while giving the slicer vertical,
        // top, and bottom facets to project into the real segmentation path.
        init_print({ make_cube(20.0, 20.0, 4.0) }, print, model, config, nullptr, !fuzzy_fixture);
        if (!fuzzy_fixture) {
            if (horizontal_paint)
                paint_by_horizontal_facets();
            else
                paint_by_facet_x();
        }
        apply_model_and_reslice();
    }

    void apply_model_and_reslice()
    {
        print.apply(model, config);
        REQUIRE(print.objects().size() == 1);
        PrintObject &object = *print.get_object(0);
        object.slice();
        REQUIRE_FALSE(object.layers().empty());
    }

    ExPolygon analytic_footprint() const
    {
        // init_print's ModelObject::add_volume centers this 20 mm cube to
        // [-10, 10] in XY and stores the +10 mesh offset. PrintObject's
        // centered slicing transform subtracts the same center offset, so
        // this model-space box is also the independent sliced-frame box.
        const BoundingBoxf3 bounds = model.objects.front()->volumes.front()->mesh().bounding_box();
        return ExPolygon({
            Point::new_scale(bounds.min.x(), bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.max.y()),
            Point::new_scale(bounds.min.x(), bounds.max.y())
        });
    }

    ExPolygon analytic_external_strip(double depth = 0.5) const
    {
        // With the centered cube this is the literal [9.5, 10] right strip
        // over [-10, 10] in the sliced frame.
        const BoundingBoxf3 bounds = model.objects.front()->volumes.front()->mesh().bounding_box();
        const double x0 = std::max<double>(bounds.min.x(), bounds.max.x() - depth);
        return ExPolygon({
            Point::new_scale(x0, bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.max.y()),
            Point::new_scale(x0, bounds.max.y())
        });
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

    static void paint_by_horizontal_facets(ModelVolume &volume)
    {
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

    void paint_by_horizontal_facets()
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        paint_by_horizontal_facets(*model.objects.front()->volumes.front());
    }

    void paint_all_facets(EnforcerBlockerType state)
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        ModelVolume &volume = *model.objects.front()->volumes.front();

        const TriangleMesh &mesh = volume.mesh();
        REQUIRE_FALSE(mesh.its.indices.empty());
        TriangleSelector selector(mesh);
        for (size_t facet_idx = 0; facet_idx < mesh.its.indices.size(); ++facet_idx)
            selector.set_facet(static_cast<int>(facet_idx), state);

        REQUIRE(volume.mmu_segmentation_facets.set(selector));
        apply_model_and_reslice();
    }

    void clear_mmu_paint()
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        model.objects.front()->volumes.front()->mmu_segmentation_facets.reset();
        apply_model_and_reslice();
    }

    void paint_fuzzy_by_vertical_facets()
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        ModelVolume &volume = *model.objects.front()->volumes.front();

        const TriangleMesh &mesh = volume.mesh();
        const BoundingBoxf3 bounds = mesh.bounding_box();
        TriangleSelector selector(mesh);
        size_t painted_facets = 0;
        for (size_t facet_idx = 0; facet_idx < mesh.its.indices.size(); ++facet_idx) {
            const stl_triangle_vertex_indices &triangle = mesh.its.indices[facet_idx];
            const Vec3f p0 = mesh.its.vertices[triangle[0]];
            const Vec3f p1 = mesh.its.vertices[triangle[1]];
            const Vec3f p2 = mesh.its.vertices[triangle[2]];
            const float z_span = std::max({p0.z(), p1.z(), p2.z()}) - std::min({p0.z(), p1.z(), p2.z()});
            const bool right_face = std::abs(p0.x() - bounds.max.x()) < 1e-4f &&
                                    std::abs(p1.x() - bounds.max.x()) < 1e-4f &&
                                    std::abs(p2.x() - bounds.max.x()) < 1e-4f;
            if (z_span > 1e-4f && right_face) {
                selector.set_facet(static_cast<int>(facet_idx), EnforcerBlockerType::Extruder1);
                ++painted_facets;
            }
        }
        REQUIRE(painted_facets > 0);
        REQUIRE(volume.fuzzy_skin_facets.set(selector));
        apply_model_and_reslice();
    }

    void paint_fuzzy_on_top_facets()
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        ModelVolume &volume = *model.objects.front()->volumes.front();

        const TriangleMesh &mesh = volume.mesh();
        const BoundingBoxf3 bounds = mesh.bounding_box();
        TriangleSelector selector(mesh);
        size_t painted_facets = 0;
        for (size_t facet_idx = 0; facet_idx < mesh.its.indices.size(); ++facet_idx) {
            const stl_triangle_vertex_indices &triangle = mesh.its.indices[facet_idx];
            const Vec3f centroid = (mesh.its.vertices[triangle[0]] +
                                    mesh.its.vertices[triangle[1]] +
                                    mesh.its.vertices[triangle[2]]) / 3.f;
            if (centroid.z() >= bounds.max.z() - 1e-4f) {
                selector.set_facet(static_cast<int>(facet_idx), EnforcerBlockerType::Extruder1);
                ++painted_facets;
            }
        }
        REQUIRE(painted_facets > 0);
        REQUIRE(volume.fuzzy_skin_facets.set(selector));
        apply_model_and_reslice();
    }

    void clear_fuzzy_paint()
    {
        REQUIRE(model.objects.size() == 1);
        REQUIRE(model.objects.front()->volumes.size() == 1);
        model.objects.front()->volumes.front()->fuzzy_skin_facets.reset();
        apply_model_and_reslice();
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

double area_mm2(double scaled_area)
{
    const double one_mm = double(scale_(1.));
    return scaled_area / (one_mm * one_mm);
}

struct LayerRegionAreaObservation {
    double sum_mm2 = 0.;
    double union_mm2 = 0.;
    double fuzzy_all_mm2 = 0.;
    size_t fuzzy_all_regions = 0;
    ExPolygons fuzzy_all_surfaces;
};

LayerRegionAreaObservation observe_layer_regions(const Layer &layer)
{
    ExPolygons all_surfaces;
    LayerRegionAreaObservation result;
    for (const LayerRegion *region : layer.regions()) {
        ExPolygons region_surfaces;
        for (const Surface &surface : region->slices.surfaces) {
            region_surfaces.push_back(surface.expolygon);
            all_surfaces.push_back(surface.expolygon);
        }

        const double region_mm2 = area_mm2(polygon_area(region_surfaces));
        result.sum_mm2 += region_mm2;
        if (region->region().config().fuzzy_skin.value == FuzzySkinType::All) {
            ++result.fuzzy_all_regions;
            result.fuzzy_all_mm2 += region_mm2;
            result.fuzzy_all_surfaces.insert(result.fuzzy_all_surfaces.end(), region_surfaces.begin(), region_surfaces.end());
        }
    }
    if (!all_surfaces.empty())
        result.union_mm2 = area_mm2(polygon_area(union_ex(all_surfaces)));
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

TEST_CASE("painted segmentation assigns or clears a fully painted layer", "[PaintedSegmentation][PaintedShortcut]")
{
    PaintedSegmentationFixture fixture;

    auto require_only_state = [&](const std::vector<std::vector<ExPolygons>> &segmented, size_t expected_state) {
        REQUIRE(segmented.size() == fixture.object().layers().size());
        for (size_t layer_idx = 0; layer_idx < segmented.size(); ++layer_idx) {
            REQUIRE(segmented[layer_idx].size() == PAINTED_STATES - 1);
            REQUIRE(expected_state < segmented[layer_idx].size());
            const ExPolygons source = sliced_layer(*fixture.object().layers()[layer_idx]);
            const double source_area = polygon_area(source);
            REQUIRE(source_area > 0.);
            const double tolerance = std::max(100.0, source_area * 1e-5);

            CHECK(outside_area({ segmented[layer_idx][expected_state] }, source) <= tolerance);
            const double selected_area = union_region_area({ segmented[layer_idx][expected_state] });
            CHECK_THAT(selected_area, Catch::Matchers::WithinAbs(source_area, tolerance));
            CHECK(selected_area > 0.);
            for (size_t state_idx = 0; state_idx < segmented[layer_idx].size(); ++state_idx) {
                if (state_idx != expected_state)
                    CHECK(union_region_area({ segmented[layer_idx][state_idx] }) <= tolerance);
            }
        }
    };

    auto require_cleared = [&](const std::vector<std::vector<ExPolygons>> &segmented) {
        REQUIRE(segmented.size() == fixture.object().layers().size());
        for (size_t layer_idx = 0; layer_idx < segmented.size(); ++layer_idx) {
            REQUIRE(segmented[layer_idx].size() == PAINTED_STATES - 1);
            const ExPolygons source = sliced_layer(*fixture.object().layers()[layer_idx]);
            const double source_area = polygon_area(source);
            REQUIRE(source_area > 0.);
            const double tolerance = std::max(100.0, source_area * 1e-5);
            for (const ExPolygons &state_region : segmented[layer_idx])
                CHECK(union_region_area({ state_region }) <= tolerance);
        }
    };

    fixture.paint_all_facets(EnforcerBlockerType::Extruder1);
    const auto first = multi_material_segmentation_by_painting(fixture.object(), [] {});
    require_only_state(first, 0);

    fixture.paint_all_facets(EnforcerBlockerType::Extruder2);
    const auto second = multi_material_segmentation_by_painting(fixture.object(), [] {});
    require_only_state(second, 1);

    fixture.clear_mmu_paint();
    const auto cleared = multi_material_segmentation_by_painting(fixture.object(), [] {});
    require_cleared(cleared);
}

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


TEST_CASE("Fuzzy painted vertical facets follow the external depth cap", "[FuzzyPaintedSegmentation][FuzzySkin]")
{
    PaintedSegmentationFixture fixture(0, 0, false, 0., 0., false, true);
    fixture.paint_fuzzy_by_vertical_facets();
    const auto segmented = fuzzy_skin_segmentation_by_painting(fixture.object(), [] {});
    REQUIRE(segmented.size() == fixture.object().layers().size());

    const ExPolygons footprint{fixture.analytic_footprint()};
    const ExPolygons external_strip{fixture.analytic_external_strip()};
    const double full_area = polygon_area(footprint);
    const double strip_area = polygon_area(external_strip);
    REQUIRE(full_area > strip_area);
    size_t positive_layers = 0;
    double painted_area = 0.;
    for (size_t layer_idx = 0; layer_idx < segmented.size(); ++layer_idx) {
        REQUIRE(segmented[layer_idx].size() == 1);
        const double area = polygon_area(segmented[layer_idx][0]);
        const double tolerance = std::max(100.0, strip_area * 1e-5);
        CHECK(area >= 0.);
        CHECK(area <= strip_area + tolerance);
        CHECK(outside_area(segmented[layer_idx], footprint) <= tolerance);
        CHECK(outside_area(segmented[layer_idx], external_strip) <= tolerance);
        if (area > tolerance)
            ++positive_layers;
        painted_area += area;
    }

    REQUIRE(positive_layers > 0);
    CHECK(painted_area > 0.);
    CHECK(painted_area < full_area * double(segmented.size()));

    // The public wrapper feeds this painted strip back into PrintObject::slice.
    // On a middle layer that produces a fuzzy override PrintRegion, all region
    // surfaces must still partition the 20 x 20 analytic slice exactly.
    REQUIRE(fixture.object().layers().size() >= 3);
    const Layer &middle_layer = *fixture.object().layers()[fixture.object().layers().size() / 2];
    const LayerRegionAreaObservation middle = observe_layer_regions(middle_layer);
    CHECK_THAT(middle.sum_mm2, Catch::Matchers::WithinAbs(400., 0.01));
    CHECK_THAT(middle.union_mm2, Catch::Matchers::WithinAbs(400., 0.01));
    CHECK_THAT(middle.sum_mm2, Catch::Matchers::WithinAbs(middle.union_mm2, 0.01));
    REQUIRE(middle.fuzzy_all_regions > 0);
    CHECK(middle.fuzzy_all_mm2 > 0.);
    CHECK(middle.fuzzy_all_mm2 <= area_mm2(strip_area) + 0.01);
    REQUIRE_FALSE(middle.fuzzy_all_surfaces.empty());
    const double fuzzy_all_outside_mm2 = area_mm2(polygon_area(diff_ex(union_ex(middle.fuzzy_all_surfaces), external_strip)));
    CHECK(fuzzy_all_outside_mm2 <= 0.01);
}

TEST_CASE("Clearing fuzzy facet paint after reslicing removes public regions", "[FuzzyPaintedSegmentation][FuzzySkin]")
{
    PaintedSegmentationFixture fixture(0, 0, false, 0., 0., false, true);
    const auto unpainted = fuzzy_skin_segmentation_by_painting(fixture.object(), [] {});
    REQUIRE(unpainted.size() == fixture.object().layers().size());
    for (const auto &layer : unpainted) {
        REQUIRE(layer.size() == 1);
        CHECK(layer[0].empty());
    }

    fixture.paint_fuzzy_by_vertical_facets();
    const auto painted = fuzzy_skin_segmentation_by_painting(fixture.object(), [] {});
    REQUIRE(painted.size() == unpainted.size());
    bool saw_paint = false;
    for (const auto &layer : painted) {
        REQUIRE(layer.size() == 1);
        saw_paint = saw_paint || !layer[0].empty();
    }
    REQUIRE(saw_paint);

    fixture.clear_fuzzy_paint();
    const auto cleared = fuzzy_skin_segmentation_by_painting(fixture.object(), [] {});
    REQUIRE(cleared.size() == unpainted.size());
    for (const auto &layer : cleared) {
        REQUIRE(layer.size() == 1);
        CHECK(layer[0].empty());
    }
}

TEST_CASE("Fuzzy painted top facets stay out of ordinary fuzzy regions", "[FuzzyPaintedSegmentation][FuzzySkin]")
{
    PaintedSegmentationFixture fixture(2, 0, false, 0., 0., false, true);
    fixture.paint_fuzzy_on_top_facets();
    const auto segmented = fuzzy_skin_segmentation_by_painting(fixture.object(), [] {});
    REQUIRE(segmented.size() == fixture.object().layers().size());

    for (const auto &layer : segmented) {
        REQUIRE(layer.size() == 1);
        CHECK(layer[0].empty());
    }
}

TEST_CASE("sunk bottom paint stays clipped below the printable footprint", "[PaintedSegmentation][PaintedSegmentationGap][PaintedSinkingShellProjection]")
{
    PaintedSegmentationFixture fixture(2, 2, true);
    ModelVolume &volume = *fixture.model.objects.front()->volumes.front();
    volume.set_offset(volume.get_offset() + Vec3d(0., 0., -0.5));
    fixture.apply_model_and_reslice();

    const Transform3d volume_trafo = fixture.object().trafo_sliced() * volume.get_matrix();
    const BoundingBoxf3 mesh_bounds = volume.mesh().bounding_box();
    double bottom_face_max_z = -std::numeric_limits<double>::infinity();
    size_t bottom_face_vertices = 0;
    for (const stl_vertex &vertex : volume.mesh().its.vertices) {
        if (std::abs(vertex.z() - mesh_bounds.min.z()) <= 1e-4f) {
            bottom_face_max_z = std::max(bottom_face_max_z, (volume_trafo * vertex.cast<double>()).z());
            ++bottom_face_vertices;
        }
    }
    REQUIRE(bottom_face_vertices > 0);
    REQUIRE(bottom_face_max_z < -0.1);

    const auto segmented = segment(fixture);
    REQUIRE(segmented.size() == fixture.object().layers().size());
    REQUIRE(segmented.size() >= 4);
    for (const auto &layer : segmented)
        REQUIRE(layer.size() == PAINTED_STATES - 1);

    const ExPolygons bottom_source = sliced_layer(*fixture.object().layers().front());
    const ExPolygons top_source = sliced_layer(*fixture.object().layers().back());
    const double bottom_source_area = polygon_area(bottom_source);
    const double top_source_area = polygon_area(top_source);
    REQUIRE(bottom_source_area > 0.);
    REQUIRE(top_source_area > 0.);

    const double top_tolerance = std::max(100.0, top_source_area * 1e-5);
    // The only Extruder2 facets are the bottom face, which the translated
    // transform places wholly below z=0. Public slicing clips that paint away;
    // it must not invent a first-layer footprint from the unclipped model.
    for (const auto &layer : segmented)
        CHECK(layer[1].empty());
    CHECK_THAT(polygon_area(segmented.back()[0]), Catch::Matchers::WithinAbs(top_source_area, top_tolerance));
    CHECK(outside_area({ segmented.back()[0] }, top_source) <= top_tolerance);
}

TEST_CASE("painted top and bottom projections merge across model volumes", "[PaintedSegmentation][PaintedSegmentationGap][PaintedVolumeMerge]")
{
    PaintedSegmentationFixture fixture(1, 1, true);
    ModelObject &object = *fixture.model.objects.front();
    REQUIRE(object.volumes.size() == 1);
    ModelVolume &first = *object.volumes.front();
    ModelVolume *second = object.add_volume(first.mesh(), false);
    REQUIRE(second != nullptr);
    second->set_offset(first.get_offset() + Vec3d(30., 0., 0.));

    // The fixture already painted the first volume during construction.
    // Paint only the newly added volume here.
    fixture.paint_by_horizontal_facets(*second);
    fixture.apply_model_and_reslice();

    const auto segmented = segment(fixture);
    REQUIRE(segmented.size() == fixture.object().layers().size());
    REQUIRE(segmented.size() >= 2);
    REQUIRE(object.volumes.size() == 2);
    for (const auto &layer : segmented)
        REQUIRE(layer.size() == PAINTED_STATES - 1);

    const ExPolygons bottom_source = sliced_layer(*fixture.object().layers().front());
    const ExPolygons top_source = sliced_layer(*fixture.object().layers().back());
    const double bottom_area = polygon_area(bottom_source);
    const double top_area = polygon_area(top_source);
    REQUIRE(bottom_source.size() == 2);
    REQUIRE(top_source.size() == 2);
    CHECK(segmented.front()[1].size() == 2);
    CHECK(segmented.back()[0].size() == 2);
    CHECK_THAT(polygon_area(segmented.front()[1]), Catch::Matchers::WithinAbs(bottom_area, std::max(100.0, bottom_area * 1e-5)));
    CHECK_THAT(polygon_area(segmented.back()[0]), Catch::Matchers::WithinAbs(top_area, std::max(100.0, top_area * 1e-5)));
    CHECK(outside_area({ segmented.front()[1] }, bottom_source) <= std::max(100.0, bottom_area * 1e-5));
    CHECK(outside_area({ segmented.back()[0] }, top_source) <= std::max(100.0, top_area * 1e-5));
}


namespace {

TriangleMesh acute_through_hole_prism(float z_min, float z_max)
{
    // The clockwise triangle's tip lies 4.3 mm from the painted square's
    // right edge and points toward the centered 2 x 2 mm patch.
    std::vector<Vec3f> vertices{
        { 5.3f,  0.0f, z_min }, { 20.0f,  5.5f, z_min }, { 20.0f, -5.5f, z_min },
        { 5.3f,  0.0f, z_max }, { 20.0f,  5.5f, z_max }, { 20.0f, -5.5f, z_max },
    };
    // Closed triangular prism. Cap normals point out of the prism; each side
    // quad is split into two triangles with consistent winding.
    std::vector<Vec3i32> faces{
        { 0, 1, 2 }, { 3, 5, 4 },
        { 0, 3, 4 }, { 0, 4, 1 },
        { 1, 4, 5 }, { 1, 5, 2 },
        { 2, 5, 3 }, { 2, 3, 0 },
    };
    return TriangleMesh(std::move(vertices), std::move(faces));
}

struct AcuteHolePaintedShellFixture {
    Model              model;
    Print              print;
    DynamicPrintConfig config;
    ModelVolume       *paint_volume = nullptr;

    explicit AcuteHolePaintedShellFixture(bool with_hole)
    {
        config = multifilament_config(2, {
            { "layer_height",                0.2 },
            { "initial_layer_print_height",  0.2 },
            { "wall_loops",                  1 },
            { "top_shell_layers",            4 },
            { "bottom_shell_layers",         4 },
            { "sparse_infill_density",       "0%" },
            { "raft_layers",                 0 },
            { "outer_wall_line_width",       0.42 },
        });
        // Resolve the morphology threshold through the documented enabled-gap-fill path.
        config.set_key_value("gap_infill_speed", new ConfigOptionFloatsNullable({ 30.0, 30.0 }));

        init_print({ make_cube(100.0, 100.0, 2.0) }, print, model, config, nullptr, false);
        REQUIRE(model.objects.size() == 1);
        ModelObject &object = *model.objects.front();
        REQUIRE(object.volumes.size() == 1);
        ModelVolume &plate = *object.volumes.front();

        // A coincident model-part volume contributes only the small painted
        // top/bottom facet patch; its union with the plate leaves slice shape unchanged.
        TriangleMesh patch_mesh = make_cube(2.0, 2.0, 2.0);
        patch_mesh.translate(Vec3f(-1.f, -1.f, -1.f)); // center its local coordinates on the plate.
        paint_volume = object.add_volume(std::move(patch_mesh), ModelVolumeType::MODEL_PART, false);
        REQUIRE(paint_volume != nullptr);
        paint_volume->set_offset(plate.get_offset());
        PaintedSegmentationFixture::paint_by_horizontal_facets(*paint_volume);

        if (with_hole) {
            ModelVolume *hole = object.add_volume(acute_through_hole_prism(-1.0f, 1.0f),
                                                  ModelVolumeType::NEGATIVE_VOLUME, false);
            REQUIRE(hole != nullptr);
            hole->set_offset(plate.get_offset());
        }

        print.apply(model, config);
        REQUIRE(print.objects().size() == 1);
        PrintObject &print_object = *print.get_object(0);
        print_object.slice();
        REQUIRE(print_object.layers().size() >= 8);
    }

    PrintObject &object() { return *print.get_object(0); }

    ExPolygon analytic_paint_patch()
    {
        const Transform3d volume_trafo = object().trafo_sliced() * paint_volume->get_matrix();
        const BoundingBoxf3 bounds = paint_volume->mesh().transformed_bounding_box(volume_trafo);
        return ExPolygon({
            Point::new_scale(bounds.min.x(), bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.min.y()),
            Point::new_scale(bounds.max.x(), bounds.max.y()),
            Point::new_scale(bounds.min.x(), bounds.max.y()),
        });
    }

    std::vector<std::vector<ExPolygons>> segments()
    {
        return segmentation_by_painting(
            object(),
            [](const ModelVolume &volume) -> ModelVolumeFacetsInfo {
                return { volume.mmu_segmentation_facets, !volume.mmu_segmentation_facets.empty(), false };
            },
            PAINTED_STATES,
            0.f,
            0.f,
            false,
            IncludeTopAndBottomLayers::Yes,
            [] {});
    }
};

} // namespace

TEST_CASE("acute through-holes remain visible in painted shell projections", "[PaintedSegmentation][PaintedHoleProjection]")
{
    AcuteHolePaintedShellFixture control(false);
    AcuteHolePaintedShellFixture with_hole(true);
    const auto control_regions = control.segments();
    const auto hole_regions = with_hole.segments();
    REQUIRE(control_regions.size() == control.object().layers().size());
    REQUIRE(hole_regions.size() == with_hole.object().layers().size());
    REQUIRE(control_regions.size() == hole_regions.size());
    REQUIRE(control_regions.size() >= 8);

    const ExPolygons expected_patch{ with_hole.analytic_paint_patch() };
    CHECK_THAT(area_mm2(polygon_area(expected_patch)), Catch::Matchers::WithinAbs(4.0, 0.01));

    for (const bool from_bottom : { false, true }) {
        CAPTURE(from_bottom);
        const size_t target_layer = from_bottom ? 3 : hole_regions.size() - 4;
        const size_t painted_state = from_bottom ? 1 : 0; // bottom uses Extruder2; top uses Extruder1.
        REQUIRE(hole_regions[target_layer].size() == PAINTED_STATES - 1);
        REQUIRE(control_regions[target_layer].size() == PAINTED_STATES - 1);

        const ExPolygons source_slice = sliced_layer(*with_hole.object().layers()[target_layer]);
        REQUIRE(source_slice.size() == 1);
        REQUIRE(source_slice.front().holes.size() == 1);
        CHECK_THAT(std::abs(area_mm2(source_slice.front().holes.front().area())),
                   Catch::Matchers::WithinAbs(80.85, 0.1));
        CHECK_THAT(area_mm2(polygon_area(intersection_ex(source_slice, expected_patch))),
                   Catch::Matchers::WithinAbs(4.0, 0.01));

        const ExPolygons control_projection = union_ex(control_regions[target_layer][painted_state]);
        const ExPolygons hole_projection = union_ex(hole_regions[target_layer][painted_state]);
        REQUIRE_FALSE(control_projection.empty());
        REQUIRE_FALSE(hole_projection.empty());
        CHECK_THAT(area_mm2(polygon_area(control_projection)),
                   Catch::Matchers::WithinAbs(4.0, 0.05));
        CHECK_THAT(area_mm2(polygon_area(diff_ex(control_projection, expected_patch))),
                   Catch::Matchers::WithinAbs(0.0, 0.01));

        const double outside_patch_mm2 = area_mm2(polygon_area(diff_ex(hole_projection, expected_patch)));
        const double symmetric_difference_mm2 = area_mm2(
            polygon_area(diff_ex(expected_patch, hole_projection)) +
            polygon_area(diff_ex(hole_projection, expected_patch)));
        CHECK_THAT(outside_patch_mm2, Catch::Matchers::WithinAbs(0.0, 0.01));
        CHECK_THAT(area_mm2(polygon_area(diff_ex(hole_projection, source_slice))),
                   Catch::Matchers::WithinAbs(0.0, 0.01));
        // At depth three the accumulated shrink is about 2.39124 mm.
        // The acute hole's miter reaches into the patch, leaving about
        // 1.64273 mm2 after the 0.105 mm morphological opening. Discarding
        // that external hole instead retains all 4 mm2 of the patch.
        CHECK_THAT(area_mm2(polygon_area(hole_projection)),
                   Catch::Matchers::WithinAbs(1.64273, 0.1));
        CHECK_THAT(symmetric_difference_mm2, Catch::Matchers::WithinAbs(2.35727, 0.1));
    }
}
