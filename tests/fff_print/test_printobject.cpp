#include "libslic3r/TriangleSelector.hpp"
#include <array>
#include <limits>
#include <algorithm>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Print.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/AABBTreeLines.hpp"

#include "test_helpers.hpp"

#include <cmath>
#include <cstddef>
#include <iterator>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Surface.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>
#include "libslic3r/ExPolygon.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/SurfaceCollection.hpp"

using namespace Slic3r;
using namespace Slic3r::Test;

SCENARIO("Object layer heights", "[PrintObject]") {
    GIVEN("A 20mm cube") {
        WHEN("sliced with a 2mm layer height and a 3mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               2 },
                { "nozzle_diameter",            3 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
            THEN("The output vector has 10 entries") {
                REQUIRE(layers.size() == 10);
            }
            AND_THEN("Each layer is approximately 2mm above the previous Z") {
                coordf_t last = 0.0;
                for (size_t i = 0; i < layers.size(); ++ i) {
                    REQUIRE_THAT(layers[i]->print_z - last, Catch::Matchers::WithinAbs(2.0, 1e-4));
                    last = layers[i]->print_z;
                }
            }
        }
        WHEN("sliced with a 10mm layer height and an 11mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               10 },
                { "nozzle_diameter",            11 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 3 entries") {
                REQUIRE(layers.size() == 3);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers.front()->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 12mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(12.0, 1e-4));
            }
        }
        WHEN("sliced with a 15mm layer height and a 16mm nozzle") {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "initial_layer_print_height", 2 },
                { "layer_height",               15 },
                { "nozzle_diameter",            16 }
	        });
            ConstLayerPtrsAdaptor layers = print.objects().front()->layers();
			THEN("The output vector has 2 entries") {
                REQUIRE(layers.size() == 2);
            }
            AND_THEN("Layer 0 is at 2mm") {
                REQUIRE_THAT(layers[0]->print_z, Catch::Matchers::WithinAbs(2.0, 1e-4));
            }
            AND_THEN("Layer 1 is at 17mm") {
                REQUIRE_THAT(layers[1]->print_z, Catch::Matchers::WithinAbs(17.0, 1e-4));
            }
        }
        WHEN("layer height exceeds the nozzle diameter") {
            // Orca does not clamp an over-large layer height to the nozzle; it
            // rejects the slice during flow computation. Pin that behavior.
            THEN("Slicing is rejected") {
                Slic3r::Print print;
                REQUIRE_THROWS(Slic3r::Test::init_and_process_print({cube(20)}, print, {
                    { "initial_layer_print_height", 0.3 },
                    { "layer_height",               0.5 },
                    { "nozzle_diameter",            0.4 }
                }));
            }
        }
    }
}

SCENARIO("Perimeter generation", "[PrintObject]") {
    GIVEN("20mm cube and default config") {
        WHEN("make_perimeters() is called")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, { { "sparse_infill_density", 0 } });
			const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 1 island of perimeters") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.entities.size() == 1);
            }
        }
        WHEN("wall_loops is set to 3")  {
            Slic3r::Print print;
            Slic3r::Test::init_and_process_print({cube(20)}, print, {
                { "sparse_infill_density", 0 },
                { "wall_loops",            3 }
            });
            const PrintObject &object = *print.objects().front();
            THEN("Every layer in region 0 has 3 perimeter loops") {
                for (const Layer *layer : object.layers())
                    REQUIRE(layer->regions().front()->perimeters.items_count() == 3);
            }
        }
    }
}

TEST_CASE("Initial layer height is honored", "[PrintObject]")
{
    const std::string gcode = Slic3r::Test::slice({cube(20)}, {
        { "initial_layer_print_height", 0.3 },
        { "layer_height",               0.2 },
        { "z_hop",                      0 } // keep recorded Z equal to the printed layer height
    });

    std::set<double> layer_zs;
    GCodeReader reader;
    reader.parse_buffer(gcode, [&layer_zs] (GCodeReader& self, const GCodeReader::GCodeLine& line) {
        if (line.extruding(self) && line.dist_XY(self) > 0)
            layer_zs.insert(self.z());
    });

    REQUIRE(layer_zs.size() > 1);
    REQUIRE_THAT(*layer_zs.begin(),            Catch::Matchers::WithinAbs(0.3, 1e-4));
    REQUIRE_THAT(*std::next(layer_zs.begin()), Catch::Matchers::WithinAbs(0.5, 1e-4));
}

static TriangleMesh internal_bridge_step()
{
    // Orca: The smaller tower leaves a shoulder whose solid skin needs internal bridges
    // over the sparse infill in the base, without relying on an external model file.
    TriangleMesh mesh = make_cube(30, 24, 3);
    TriangleMesh tower = make_cube(14, 10, 1);
    tower.translate(8, 7, 3);
    mesh.merge(tower);
    return mesh;
}

static DynamicPrintConfig internal_bridge_config(const std::string &pattern, int multiline)
{
    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"sparse_infill_pattern", pattern},
                                   {"fill_multiline", multiline},
                                   {"sparse_infill_density", "15%"},
                                   {"sparse_infill_smooth_factor", "100%"},
                                   {"infill_direction", 45},
                                   {"internal_bridge_angle", 0},
                                   {"thick_internal_bridges", true},
                                   {"top_shell_layers", 3},
                                   {"bottom_shell_layers", 2},
                                   {"top_shell_thickness", 0},
                                   {"bottom_shell_thickness", 0},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2}});
    return config;
}

TEST_CASE("Internal bridge angles follow the lower infill layer and model rotation", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral");
    // Orca: Cover both a central line (odd counts) and offset pairs (even counts).
    const int multiline = GENERATE(1, 2, 3);
    CAPTURE(multiline);
    const double rotation = GENERATE(23., -123.);
    const std::vector<double> cycle{10., 30., 70.};
    auto config = internal_bridge_config(pattern, multiline);
    config.set_deserialize_strict({{"sparse_infill_rotate_template", "10,30,70"},
                                   {"align_infill_direction_to_model", true},
                                   {"separated_infills", false}});
    Print print;
    Model model;
    init_print({internal_bridge_step()}, print, model, config, nullptr, false);
    model.objects.front()->instances.front()->set_rotation(Vec3d(0., 0., Geometry::deg2rad(rotation)));
    print.apply(model, config);
    print.process();
    const PrintObject &object = *print.objects().front();
    size_t bridges = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        // Orca: The support is one layer below the bridge. Check the template and model
        // rotation together, including normalization when the resulting angle is negative.
        double expected = std::fmod(cycle[(i - 1) % cycle.size()] + 90. + rotation, 180.);
        if (expected < 0.) expected += 180.;
        for (const LayerRegion *region : object.get_layer(i)->regions())
            for (const Surface *surface : region->fill_surfaces.filter_by_type(stInternalBridge)) {
                CAPTURE(pattern, rotation, i);
                CHECK_THAT(Geometry::rad2deg(surface->bridge_angle), Catch::Matchers::WithinAbs(expected, 0.001));
                ++bridges;
            }
    }
    REQUIRE(bridges > 0);
}

TEST_CASE("Turning infill does not replace the anchors of another region", "[PrintObject][InternalBridge][Regression]")
{
    // Orca: Keep the right-hand region fixed while changing the left-hand pattern in the
    // same object. Its bridge areas must be independent of a previous candidate's anchors.
    const int multiline = GENERATE(1, 2, 3);
    CAPTURE(multiline);
    auto right_bridges = [multiline](const std::string &left_pattern) {
        auto config = internal_bridge_config(left_pattern, multiline);
        Print print;
        Model model;
        init_print({internal_bridge_step()}, print, model, config, nullptr, false);
        TriangleMesh right = internal_bridge_step();
        right.translate(50, 0, 0);
        ModelVolume *volume = model.objects.front()->add_volume(std::move(right));
        volume->config.set_key_value("sparse_infill_pattern", new ConfigOptionEnum<InfillPattern>(ipRectilinear));
        volume->config.set_key_value("infill_direction", new ConfigOptionFloat(17.));
        print.apply(model, config);
        print.process();
        std::map<size_t, Polygons> result;
        const PrintObject &object = *print.objects().front();
        for (size_t i = 0; i < object.layer_count(); ++i)
            for (const LayerRegion *region : object.get_layer(i)->regions())
                if (region->region().config().infill_direction == 17.)
                    polygons_append(result[i], to_polygons(region->fill_surfaces.filter_by_type(stInternalBridge)));
        return result;
    };
    const auto baseline = right_bridges("rectilinear");
    const auto actual = right_bridges(GENERATE("hilbertcurve", "octagramspiral"));
    REQUIRE(actual.size() == baseline.size());
    double total_area = 0.;
    for (const auto &[layer, expected] : baseline) {
        CAPTURE(layer);
        const auto &polys = actual.at(layer);
        CHECK(area(diff(expected, polys)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
        CHECK(area(diff(polys, expected)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
        total_area += area(expected);
    }
    REQUIRE(total_area > 0.);
}

TEST_CASE("Rounded internal bridges end on printed support", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral");
    const bool separated = GENERATE(false, true);
    CAPTURE(pattern, separated);
    auto config = internal_bridge_config(pattern, 1);
    config.set_deserialize_strict({{"infill_wall_overlap", "0%"}, {"separated_infills", separated}});
    TriangleMesh mesh = internal_bridge_step();
    if (separated) {
        TriangleMesh second = internal_bridge_step();
        second.translate(50, 0, 0);
        mesh.merge(second);
    }
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    print.process();

    // Orca: Check final extrusion endpoints after polygon cleanup and fill generation.
    // A correct bridge angle and correct sparse anchors alone do not guarantee contact.
    const PrintObject &object = *print.objects().front();
    size_t checked = 0;
    for (size_t i = 1; i < object.layer_count(); ++i) {
        Polygons support;
        Polylines walls;
        for (const LayerRegion *region : object.get_layer(i - 1)->regions()) {
            region->perimeters.polygons_covered_by_width(support, 0.f);
            region->fills.polygons_covered_by_width(support, 0.f);
            region->perimeters.collect_polylines(walls);
        }
        REQUIRE_FALSE(support.empty());
        const AABBTreeLines::LinesDistancer<Line> support_tree(to_lines(union_(support)));
        const AABBTreeLines::LinesDistancer<Line> wall_tree(to_lines(walls));
        for (const LayerRegion *region : object.get_layer(i)->regions())
            for (const ExtrusionEntity *entity : region->fills.flatten().entities) {
                if (entity->role() != erInternalBridgeInfill)
                    continue;
                const auto *path = dynamic_cast<const ExtrusionPath *>(entity);
                REQUIRE(path != nullptr);
                for (const Line &line : path->polyline.to_polyline().lines()) {
                    // Orca: Sample span ends, excluding short connectors and wall overlap.
                    if (line.length() < scale_(std::max(0.7, 3. * path->width)))
                        continue;
                    for (const Point &point : {line.a, line.b}) {
                        if (wall_tree.distance_from_lines<false>(point) <= scale_(0.5))
                            continue;
                        CAPTURE(i, point.x(), point.y());
                        const double gap = unscale<double>(support_tree.distance_from_lines<true>(point)) - 0.5 * path->width;
                        CHECK(gap <= 0.1);
                        ++checked;
                    }
                }
            }
    }
    REQUIRE(checked > 0);
}

TEST_CASE("Enabling separated infill recomputes body origins", "[PrintObject][InternalBridge][Regression]")
{
    const std::string pattern = GENERATE("hilbertcurve", "octagramspiral", "archimedeanchords");
    CAPTURE(pattern);
    auto footprint = [&](bool reslice) {
        auto config = internal_bridge_config(pattern, 2);
        config.set_deserialize_strict({{"separated_infills", !reslice}});
        TriangleMesh mesh = internal_bridge_step();
        TriangleMesh second = internal_bridge_step();
        second.translate(50, 0, 0);
        mesh.merge(second);
        Print print;
        Model model;
        init_print({mesh}, print, model, config, nullptr, false);
        print.process();
        if (reslice) {
            // Orca: Enabling centering after a completed slice must rebuild the body
            // origins now shared by bridge preparation and printed infill.
            config.set_deserialize_strict({{"separated_infills", true}});
            print.apply(model, config);
            print.process();
        }
        Polygons result;
        for (const LayerRegion *region : print.objects().front()->get_layer(4)->regions())
            region->fills.polygons_covered_by_width(result, 0.f);
        return union_(result);
    };
    const Polygons fresh = footprint(false);
    const Polygons resliced = footprint(true);
    REQUIRE_FALSE(fresh.empty());
    CHECK(area(diff(fresh, resliced)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
    CHECK(area(diff(resliced, fresh)) < scaled<double>(1.) * scaled<double>(1.) * 1e-6);
}

TEST_CASE("Surface centering survives changes to separated infill settings", "[PrintObject][SurfaceInfill][SurfaceCenteringMatrix][Regression]")
{
    const std::string pattern = GENERATE("archimedeanchords", "octagramspiral");
    const std::string final_center = GENERATE("each_surface", "each_model", "each_assembly");
    const bool separated = GENERATE(false, true);
    const std::string top_order = GENERATE("default", "outward", "inward");
    const std::string bottom_order = top_order == "outward" ? "inward" : top_order == "inward" ? "outward" : "default";
    const std::string density = GENERATE("80%", "100%");
    CAPTURE(pattern, final_center, separated, top_order, bottom_order, density);

    auto final_config = DynamicPrintConfig::full_print_config();
    final_config.set_deserialize_strict({{"top_surface_pattern", pattern},
                                         {"bottom_surface_pattern", pattern},
                                         {"top_surface_fill_order", top_order},
                                         {"bottom_surface_fill_order", bottom_order},
                                         {"top_surface_density", density},
                                         {"bottom_surface_density", density},
                                         {"center_of_surface_pattern", final_center},
                                         {"separated_infills", separated},
                                         {"sparse_infill_pattern", "rectilinear"},
                                         {"sparse_infill_density", "15%"},
                                         {"top_shell_layers", 2},
                                         {"bottom_shell_layers", 2},
                                         {"top_shell_thickness", 0},
                                         {"bottom_shell_thickness", 0},
                                         {"layer_height", 0.2},
                                         {"initial_layer_print_height", 0.2}});

    // Orca: Two disconnected bodies exercise per-body centering. The offset tower also
    // makes each-surface and each-model centering differ on the top surfaces.
    TriangleMesh mesh = make_cube(12, 10, 1.2);
    TriangleMesh tower = make_cube(4, 3, 0.6);
    tower.translate(2, 2, 1.2);
    mesh.merge(tower);
    TriangleMesh second = mesh;
    second.translate(20, 0, 0);
    mesh.merge(second);

    // Orca: Equal footprints can hide reordered or reversed paths. Retain their point
    // sequences and ordering protection to cover the directional surface behavior too.
    struct SurfaceFillSnapshot {
        std::map<bool, std::vector<Points>> paths;
        bool protected_order = true;
    };
    auto surface_fills = [](const Print &print) {
        std::map<std::pair<size_t, ExtrusionRole>, SurfaceFillSnapshot> result;
        const PrintObject &object = *print.objects().front();
        for (size_t i = 0; i < object.layer_count(); ++i) {
            auto collect = [&](const auto &self, const ExtrusionEntity &entity, bool no_sort) -> void {
                if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection *>(&entity)) {
                    for (const ExtrusionEntity *child : collection->entities)
                        self(self, *child, no_sort || collection->no_sort);
                } else if (entity.role() == erTopSolidInfill || entity.role() == erBottomSurface) {
                    const auto *path = dynamic_cast<const ExtrusionPath *>(&entity);
                    REQUIRE(path != nullptr);
                    auto &snapshot = result[{i, entity.role()}];
                    // Orca: The centered test model has one body on either side of X=0.
                    // Their traversal order may vary; preserve path order within each body.
                    Points points = path->polyline.to_polyline().points;
                    REQUIRE_FALSE(points.empty());
                    snapshot.paths[points.front().x() > 0].push_back(std::move(points));
                    snapshot.protected_order &= no_sort && !path->can_reverse();
                }
            };
            for (const LayerRegion *region : object.get_layer(i)->regions())
                collect(collect, region->fills, false);
        }
        return result;
    };

    // Orca: The final-state reference depends only on the outer settings. Reuse it for
    // each initial center instead of slicing the same target print three times.
    const auto fresh = [&] {
        Print fresh_print;
        Model fresh_model;
        init_print({mesh}, fresh_print, fresh_model, final_config, nullptr, false);
        fresh_print.process();
        return surface_fills(fresh_print);
    }();

    for (const char *initial_center_value : {"each_surface", "each_model", "each_assembly"}) {
        const std::string initial_center = initial_center_value;
        const bool change_center = initial_center != final_center;
        CAPTURE(initial_center, change_center);

        auto config = final_config;
        config.set_deserialize_strict({{"center_of_surface_pattern", initial_center},
                                       {"separated_infills", change_center ? separated : !separated}});
        Print print;
        Model model;
        init_print({mesh}, print, model, config, nullptr, false);
        print.process();
        const auto initial = surface_fills(print);
        config.set_deserialize_strict({{"center_of_surface_pattern", final_center}, {"separated_infills", separated}});
        print.apply(model, config);
        // Orca: Preparation owns the body origins, and its invalidation must also force
        // regeneration of top/bottom extrusion paths, even when sparse infill is unchanged.
        CHECK_FALSE(print.objects().front()->is_step_done(posPrepareInfill));
        CHECK_FALSE(print.objects().front()->is_step_done(posInfill));
        print.process();
        const auto resliced = surface_fills(print);

        REQUIRE_FALSE(fresh.empty());
        REQUIRE(resliced.size() == fresh.size());
        std::set<ExtrusionRole> roles;
        bool changed_paths = false;
        for (const auto &entry : fresh) {
            CAPTURE(entry.first.first, entry.first.second);
            REQUIRE_FALSE(entry.second.paths.empty());
            roles.insert(entry.first.second);
            REQUIRE(resliced.count(entry.first) == 1);
            REQUIRE(initial.count(entry.first) == 1);
            const auto &actual = resliced.at(entry.first);
            const auto &expected = entry.second;
            const auto &before = initial.at(entry.first);
            CHECK((actual.paths == expected.paths));
            if (!change_center)
                CHECK((actual.paths == before.paths));
            if (top_order != "default") {
                CHECK(expected.protected_order);
                CHECK(actual.protected_order);
                CHECK(before.protected_order);
            }
            changed_paths |= expected.paths != before.paths;
        }
        CHECK(roles.count(erTopSolidInfill) == 1);
        CHECK(roles.count(erBottomSurface) == 1);
        // Orca: Guard against a vacuous comparison: changing surface centering must change
        // the printed pattern, while toggling separated sparse infill must leave it alone.
        CHECK(changed_paths == change_center);
    }
}
TEST_CASE("Separated infill keeps fragmented and nested bodies independent", "[PrintObject][SurfaceInfill][Regression]")
{
    constexpr size_t grid_size = 8;
    TriangleMesh mesh;
    auto add_box = [&](double x, double y, double width, double depth) {
        TriangleMesh box = make_cube(width, depth, 0.6);
        box.translate(x, y, 0);
        mesh.merge(box);
    };
    // Orca: Many small islands exercise spatial pruning and the tree's original
    // island indices. A pillar inside a frame also overlaps its bounding box,
    // but must remain a separate body because it lies entirely inside the hole.
    for (size_t x = 0; x < grid_size; ++ x)
        for (size_t y = 0; y < grid_size; ++ y)
            add_box(6 * x, 6 * y, 3, 3);
    add_box(54, 0, 20, 4);
    add_box(54, 16, 20, 4);
    add_box(54, 0, 4, 20);
    add_box(70, 0, 4, 20);
    add_box(62, 8, 4, 4);

    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"separated_infills", true},
                                   {"center_of_surface_pattern", "each_surface"},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2},
                                   {"elefant_foot_compensation", 0},
                                   {"wall_loops", 1}});
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    // Orca: Prepare body bounds through the public pipeline, then inspect the object read-only.
    print.process();
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() > 1);
    CHECK(object.separated_body_bboxes().size() == grid_size * grid_size + 2);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices.size() == grid_size * grid_size + 2);
        REQUIRE(layer->lslices_separated_component_ids.size() == layer->lslices.size());
        size_t holes = 0;
        for (size_t i = 0; i < layer->lslices.size(); ++ i) {
            const BoundingBox &body = object.separated_body_bboxes()[layer->lslices_separated_component_ids[i]];
            const BoundingBox &island = layer->lslices_bboxes[i];
            CHECK(body.min == island.min);
            CHECK(body.max == island.max);
            holes += layer->lslices[i].holes.size();
        }
        CHECK(holes == 1);
    }
}

TEST_CASE("Body centering survives islands merging and splitting between layers", "[PrintObject][SurfaceInfill][Regression]")
{
    const bool separated = GENERATE(false, true);
    CAPTURE(separated);
    // Orca: Four posts join through horizontal then vertical rails, creating a
    // cycle of overlaps before splitting into four islands again. This exercises
    // redundant connections and indexing either adjacent layer. A fifth post
    // stays separate at every height.
    TriangleMesh mesh;
    for (int x : {0, 8})
        for (int y : {0, 8}) {
            TriangleMesh post = make_cube(4, 4, 1);
            post.translate(x, y, 0);
            mesh.merge(post);
        }
    for (int y : {0, 8}) {
        TriangleMesh rail = make_cube(12, 4, 0.2);
        rail.translate(0, y, 0.2);
        mesh.merge(rail);
    }
    for (int x : {0, 8}) {
        TriangleMesh rail = make_cube(4, 12, 0.2);
        rail.translate(x, 0, 0.4);
        mesh.merge(rail);
    }
    TriangleMesh isolated = make_cube(4, 4, 1);
    isolated.translate(20, 0, 0);
    mesh.merge(isolated);

    auto config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({{"separated_infills", separated},
                                   {"center_of_surface_pattern", separated ? "each_surface" : "each_model"},
                                   {"layer_height", 0.2},
                                   {"initial_layer_print_height", 0.2},
                                   {"elefant_foot_compensation", 0},
                                   {"wall_loops", 1}});
    Print print;
    Model model;
    init_print({mesh}, print, model, config, nullptr, false);
    // Orca: Prepare body bounds through the public pipeline, then inspect the object read-only.
    print.process();
    const PrintObject &object = *print.objects().front();
    REQUIRE(object.layer_count() == 5);
    REQUIRE(object.get_layer(0)->lslices.size() == 5);
    REQUIRE(object.get_layer(1)->lslices.size() == 3);
    REQUIRE(object.get_layer(2)->lslices.size() == 3);
    REQUIRE(object.get_layer(4)->lslices.size() == 5);
    CHECK(object.separated_body_bboxes().size() == 2);

    BoundingBox isolated_bbox = object.get_layer(0)->lslices_bboxes.front();
    for (const BoundingBox &bbox : object.get_layer(0)->lslices_bboxes)
        if (bbox.min.x() > isolated_bbox.min.x())
            isolated_bbox = bbox;
    BoundingBox connected_bbox;
    for (const Layer *layer : object.layers())
        for (const BoundingBox &bbox : layer->lslices_bboxes)
            if (bbox.min.x() < isolated_bbox.min.x())
                connected_bbox.merge(bbox);
    for (const Layer *layer : object.layers()) {
        REQUIRE(layer->lslices_separated_component_ids.size() == layer->lslices.size());
        for (size_t i = 0; i < layer->lslices.size(); ++ i) {
            const BoundingBox &expected = layer->lslices_bboxes[i].min.x() < isolated_bbox.min.x() ? connected_bbox : isolated_bbox;
            const BoundingBox &actual = object.separated_body_bboxes()[layer->lslices_separated_component_ids[i]];
            CHECK(actual.min == expected.min);
            CHECK(actual.max == expected.max);
        }
    }
}

namespace painted_projection_contracts {

namespace {
DynamicPrintConfig projection_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.2},
        {"initial_layer_print_height", 0.2},
        {"wall_loops", 1},
        {"raft_layers", 0},
        {"skirt_loops", 0},
        {"nozzle_diameter", 0.4},
        {"outer_wall_line_width", 0.4},
        {"initial_layer_line_width", 0.4},
    });
    return config;
}

struct PaintedFixture {
    Model model;
    Print print;
    DynamicPrintConfig config = projection_config();

    explicit PaintedFixture(TriangleMesh mesh = Test::cube(20))
    {
        Test::init_print({std::move(mesh)}, print, model, config);
    }

    ModelVolume &volume() { return *model.objects.front()->volumes.front(); }

    PrintObject &slice()
    {
        print.apply(model, config);
        auto *object = print.get_object(0);
        REQUIRE(object != nullptr);
        object->slice();
        REQUIRE_FALSE(object->layers().empty());
        return *object;
    }
};

std::vector<size_t> bottom_facets(const TriangleMesh &mesh)
{
    const float min_z = mesh.bounding_box().min.z();
    std::vector<size_t> facets;
    for (size_t i = 0; i < mesh.its.indices.size(); ++i) {
        const auto &tri = mesh.its.indices[i];
        if (std::abs(mesh.its.vertices[tri[0]].z() - min_z) < 1e-5f &&
            std::abs(mesh.its.vertices[tri[1]].z() - min_z) < 1e-5f &&
            std::abs(mesh.its.vertices[tri[2]].z() - min_z) < 1e-5f)
            facets.push_back(i);
    }
    return facets;
}

size_t bottom_facet(const TriangleMesh &mesh)
{
    const std::vector<size_t> facets = bottom_facets(mesh);
    REQUIRE_FALSE(facets.empty());
    return facets.front();
}

size_t side_facet(const TriangleMesh &mesh)
{
    const float min_z = mesh.bounding_box().min.z();
    const float max_z = mesh.bounding_box().max.z();
    for (size_t i = 0; i < mesh.its.indices.size(); ++i) {
        const auto &tri = mesh.its.indices[i];
        int on_min = 0, on_max = 0;
        for (int j = 0; j < 3; ++j) {
            on_min += std::abs(mesh.its.vertices[tri[j]].z() - min_z) < 1e-5f;
            on_max += std::abs(mesh.its.vertices[tri[j]].z() - max_z) < 1e-5f;
        }
        if (on_min > 0 && on_max > 0)
            return i;
    }
    FAIL("cube has no vertical side facet");
    return 0;
}

size_t sloped_facet(const TriangleMesh &mesh)
{
    for (size_t i = 0; i < mesh.its.indices.size(); ++i) {
        const auto &tri = mesh.its.indices[i];
        const Vec3f a = mesh.its.vertices[tri[0]];
        const Vec3f b = mesh.its.vertices[tri[1]];
        const Vec3f c = mesh.its.vertices[tri[2]];
        const float zmin = std::min({a.z(), b.z(), c.z()});
        const float zmax = std::max({a.z(), b.z(), c.z()});
        const float zcomp = (b-a).cross(c-a).z();
        if (zmax - zmin > 0.1f && std::abs(zcomp) > 0.01f)
            return i;
    }
    FAIL("mesh has no sloped facet");
    return 0;
}

double polygon_area_mm2(const Polygons &polygons)
{
    double area = 0.;
    for (const Polygon &polygon : polygons)
        area += std::abs(polygon.area());
    return area * (SCALING_FACTOR * SCALING_FACTOR);
}

BoundingBox extents(const Polygons &polygons)
{
    BoundingBox bbox;
    for (const Polygon &polygon : polygons)
        bbox.merge(get_extents(polygon));
    return bbox;
}

void paint_one(ModelVolume &volume, size_t facet, EnforcerBlockerType state)
{
    TriangleSelector selector(volume.mesh());
    selector.set_facet(static_cast<int>(facet), state);
    REQUIRE(volume.supported_facets.set(selector));
}

void paint_seam_one(ModelVolume &volume, size_t facet)
{
    TriangleSelector selector(volume.mesh());
    selector.set_facet(static_cast<int>(facet), EnforcerBlockerType::ENFORCER);
    REQUIRE(volume.seam_facets.set(selector));
}

Polygon rectangle_mm(double x0, double y0, double x1, double y1)
{
    return Polygon({Point::new_scale(x0, y0), Point::new_scale(x1, y0), Point::new_scale(x1, y1), Point::new_scale(x0, y1)});
}

double expolygons_area_mm2(const ExPolygons &polygons)
{
    double area = 0.;
    for (const ExPolygon &polygon : polygons)
        area += polygon.area();
    return area * (SCALING_FACTOR * SCALING_FACTOR);
}

bool polygons_contain(const Polygons &polygons, const Point &point)
{
    return std::any_of(polygons.begin(), polygons.end(), [&point](const Polygon &polygon) {
        return polygon.contains(point);
    });
}

std::array<Vec3d, 3> transformed_triangle(const PrintObject &object, const ModelVolume &volume, size_t facet)
{
    std::array<Vec3d, 3> result;
    const auto &triangle = volume.mesh().its.indices[facet];
    for (int i = 0; i < 3; ++i)
        result[i] = object.trafo_sliced() * volume.get_matrix() * volume.mesh().its.vertices[triangle[i]].cast<double>();
    return result;
}

Point triangle_centroid_xy(const std::array<Vec3d, 3> &triangle)
{
    return Point::new_scale((triangle[0].x() + triangle[1].x() + triangle[2].x()) / 3.,
                            (triangle[0].y() + triangle[1].y() + triangle[2].y()) / 3.);
}

std::pair<size_t, size_t> projected_slab_range(const PrintObject &object, const std::array<Vec3d, 3> &triangle)
{
    double min_z = triangle[0].z();
    double max_z = triangle[0].z();
    for (size_t i = 1; i < triangle.size(); ++i) {
        min_z = std::min(min_z, triangle[i].z());
        max_z = std::max(max_z, triangle[i].z());
    }
    size_t first = 0;
    while (first < object.layers().size() && float(object.layers()[first]->slice_z) < min_z + EPSILON)
        ++first;
    REQUIRE(first < object.layers().size());
    size_t last = first;
    while (last + 1 < object.layers().size() && float(object.layers()[last]->slice_z) <= max_z)
        ++last;
    return {first, last};
}

std::vector<Vec3d> clip_z(const std::vector<Vec3d> &polygon, double z, bool keep_above)
{
    std::vector<Vec3d> clipped;
    if (polygon.empty())
        return clipped;
    auto inside = [z, keep_above](const Vec3d &point) {
        return keep_above ? point.z() >= z : point.z() <= z;
    };
    for (size_t i = 0; i < polygon.size(); ++i) {
        const Vec3d &a = polygon[i];
        const Vec3d &b = polygon[(i + 1) % polygon.size()];
        const bool a_inside = inside(a);
        const bool b_inside = inside(b);
        if (a_inside != b_inside) {
            const double dz = b.z() - a.z();
            if (std::abs(dz) > 1e-12)
                clipped.emplace_back(a + ((z - a.z()) / dz) * (b - a));
        }
        if (b_inside)
            clipped.push_back(b);
    }
    return clipped;
}

double convex_xy_area_mm2(std::vector<Vec3d> polygon)
{
    std::vector<std::pair<double, double>> points;
    points.reserve(polygon.size());
    for (const Vec3d &point : polygon)
        points.emplace_back(point.x(), point.y());
    std::sort(points.begin(), points.end());
    points.erase(std::unique(points.begin(), points.end()), points.end());
    if (points.size() < 3)
        return 0.;
    auto cross = [](const auto &o, const auto &a, const auto &b) {
        return (a.first - o.first) * (b.second - o.second) - (a.second - o.second) * (b.first - o.first);
    };
    std::vector<std::pair<double, double>> hull;
    hull.reserve(points.size() * 2);
    for (const auto &point : points) {
        while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 1e-12)
            hull.pop_back();
        hull.push_back(point);
    }
    const size_t lower_size = hull.size();
    for (auto it = points.rbegin(); it != points.rend(); ++it) {
        while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *it) <= 1e-12)
            hull.pop_back();
        hull.push_back(*it);
    }
    hull.pop_back();
    double area = 0.;
    for (size_t i = 0; i < hull.size(); ++i)
        area += hull[i].first * hull[(i + 1) % hull.size()].second - hull[(i + 1) % hull.size()].first * hull[i].second;
    return std::abs(area) * 0.5;
}

std::vector<double> independently_projected_slab_areas(const PrintObject &object, const std::array<Vec3d, 3> &triangle)
{
    const auto range = projected_slab_range(object, triangle);
    double min_z = triangle[0].z();
    double max_z = triangle[0].z();
    for (size_t i = 1; i < triangle.size(); ++i) {
        min_z = std::min(min_z, triangle[i].z());
        max_z = std::max(max_z, triangle[i].z());
    }
    std::vector<double> areas;
    for (size_t layer_id = range.first; layer_id <= range.second; ++layer_id) {
        const double lower = layer_id == range.first ? min_z : double(object.layers()[layer_id - 1]->slice_z);
        const double upper = layer_id == range.second ? max_z : double(object.layers()[layer_id]->slice_z);
        std::vector<Vec3d> clipped(triangle.begin(), triangle.end());
        clipped = clip_z(clipped, lower, true);
        clipped = clip_z(clipped, upper, false);
        areas.push_back(convex_xy_area_mm2(std::move(clipped)));
    }
    return areas;
}

void install_bridge_fixture(PrintObject &object, size_t layer_id)
{
    Layer *lower = object.get_layer(static_cast<int>(layer_id - 1));
    Layer *current = object.get_layer(static_cast<int>(layer_id));
    REQUIRE(lower != nullptr);
    REQUIRE(current != nullptr);
    lower->lslices = {ExPolygon(rectangle_mm(-12., -2., -8., 2.)), ExPolygon(rectangle_mm(8., -2., 12., 2.))};
    lower->lslices_bboxes.clear();
    for (const ExPolygon &slice : lower->lslices)
        lower->lslices_bboxes.push_back(get_extents(slice));

    LayerRegion *region = current->get_region(0);
    REQUIRE(region != nullptr);
    region->perimeters.clear();
    ExtrusionPath path(erExternalPerimeter, 0.08, 0.4f, 0.2f);
    path.polyline = Polyline3(Polyline(Points{Point::new_scale(-10., 0.), Point::new_scale(10., 0.)}));
    region->perimeters.append(path);
}
}

TEST_CASE("Painted seam projection assigns a horizontal facet to its slice slab", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    ModelVolume &volume = fixture.volume();
    const size_t facet = bottom_facet(volume.mesh());
    paint_seam_one(volume, facet);

    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);

    REQUIRE(projected.size() == object.layers().size());
    REQUIRE_FALSE(projected.front().empty());
    CHECK_THAT(polygon_area_mm2(projected.front()), Catch::Matchers::WithinAbs(200., 0.01));
    for (size_t i = 1; i < projected.size(); ++i)
        CHECK(projected[i].empty());
}

TEST_CASE("Painted seam projection follows the volume transform", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    ModelVolume &volume = fixture.volume();
    const size_t facet = bottom_facet(volume.mesh());
    volume.set_offset(Vec3d(3., -4., 0.));
    paint_seam_one(volume, facet);

    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);
    REQUIRE_FALSE(projected.empty());
    REQUIRE_FALSE(projected.front().empty());

    const BoundingBox bbox = extents(projected.front());
    BoundingBoxf3 expected_bbox;
    const auto &triangle = volume.mesh().its.indices[facet];
    for (int i = 0; i < 3; ++i) {
        const Vec3d expected_world = object.trafo_sliced() * volume.get_matrix() * volume.mesh().its.vertices[triangle[i]].cast<double>();
        expected_bbox.merge(expected_world);
    }
    CHECK_THAT(unscale<double>(bbox.min.x()), Catch::Matchers::WithinAbs(expected_bbox.min.x(), 0.02));
    CHECK_THAT(unscale<double>(bbox.min.y()), Catch::Matchers::WithinAbs(expected_bbox.min.y(), 0.02));
    CHECK_THAT(polygon_area_mm2(projected.front()), Catch::Matchers::WithinAbs(200., 0.01));
}

TEST_CASE("Painted support projection isolates enforcer and blocker facets", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    ModelVolume &volume = fixture.volume();
    const std::vector<size_t> bottoms = bottom_facets(volume.mesh());
    REQUIRE(bottoms.size() >= 2);
    const size_t enforcer = bottoms[0];
    const size_t blocker = bottoms[1];
    TriangleSelector selector(volume.mesh());
    selector.set_facet(static_cast<int>(enforcer), EnforcerBlockerType::ENFORCER);
    selector.set_facet(static_cast<int>(blocker), EnforcerBlockerType::BLOCKER);
    REQUIRE(volume.supported_facets.set(selector));

    PrintObject &object = fixture.slice();
    std::vector<Polygons> enforcer_layers;
    std::vector<Polygons> blocker_layers;
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, enforcer_layers);
    object.project_and_append_custom_facets(false, EnforcerBlockerType::BLOCKER, blocker_layers);

    REQUIRE(enforcer_layers.size() == object.layers().size());
    REQUIRE(blocker_layers.size() == object.layers().size());
    REQUIRE_FALSE(enforcer_layers.front().empty());
    REQUIRE_FALSE(blocker_layers.front().empty());
    CHECK_THAT(polygon_area_mm2(enforcer_layers.front()), Catch::Matchers::WithinAbs(200., 0.05));
    CHECK_THAT(polygon_area_mm2(blocker_layers.front()), Catch::Matchers::WithinAbs(200., 0.05));
    CHECK_THAT(polygon_area_mm2(blocker_layers.front()), Catch::Matchers::WithinAbs(polygon_area_mm2(enforcer_layers.front()), 0.05));
    const Point enforcer_center = triangle_centroid_xy(transformed_triangle(object, volume, enforcer));
    const Point blocker_center = triangle_centroid_xy(transformed_triangle(object, volume, blocker));
    CHECK(polygons_contain(enforcer_layers.front(), enforcer_center));
    CHECK_FALSE(polygons_contain(enforcer_layers.front(), blocker_center));
    CHECK(polygons_contain(blocker_layers.front(), blocker_center));
    CHECK_FALSE(polygons_contain(blocker_layers.front(), enforcer_center));
    for (size_t i = 1; i < blocker_layers.size(); ++i)
        CHECK(blocker_layers[i].empty());
}

TEST_CASE("Empty painted facets leave public projections empty", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    PrintObject &object = fixture.slice();
    std::vector<Polygons> seam;
    std::vector<Polygons> support;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, seam);
    object.project_and_append_custom_facets(false, EnforcerBlockerType::BLOCKER, support);
    CHECK(seam.empty());
    CHECK(support.empty());
}


TEST_CASE("Vertical painted seams are projected on every crossed slab", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    paint_seam_one(fixture.volume(), side_facet(fixture.volume().mesh()));
    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);
    REQUIRE(projected.size() == object.layers().size());
    const auto range = projected_slab_range(object, transformed_triangle(object, fixture.volume(), side_facet(fixture.volume().mesh())));
    size_t populated = 0;
    for (size_t layer_id = 0; layer_id < projected.size(); ++layer_id) {
        const Polygons &layer = projected[layer_id];
        const bool crosses_triangle = layer_id >= range.first && layer_id <= range.second;
        if (crosses_triangle)
            CHECK_FALSE(layer.empty());
        else
            CHECK(layer.empty());
        if (!layer.empty()) {
            ++populated;
            size_t points = 0;
            for (const Polygon &polygon : layer) {
                points += polygon.points.size();
                for (const Point &point : polygon.points) {
                    CHECK(std::isfinite(unscale<double>(point.x())));
                    CHECK(std::isfinite(unscale<double>(point.y())));
                }
            }
            CHECK(points >= 3);
        }
    }
    CHECK(populated == range.second - range.first + 1);
}

TEST_CASE("Sloped painted seams interpolate their footprint across layers", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture(Test::mesh(Test::TestMesh::pyramid));
    paint_seam_one(fixture.volume(), sloped_facet(fixture.volume().mesh()));
    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);
    REQUIRE(projected.size() == object.layers().size());
    const auto range = projected_slab_range(object, transformed_triangle(object, fixture.volume(), sloped_facet(fixture.volume().mesh())));
    const std::vector<double> expected_areas = independently_projected_slab_areas(
        object, transformed_triangle(object, fixture.volume(), sloped_facet(fixture.volume().mesh())));
    REQUIRE(expected_areas.size() == range.second - range.first + 1);
    size_t populated = 0;
    double first_area = 0., last_area = 0.;
    double expected_total = 0.;
    double actual_total = 0.;
    for (size_t layer_id = 0; layer_id < projected.size(); ++layer_id) {
        const Polygons &layer = projected[layer_id];
        if (layer_id < range.first || layer_id > range.second) {
            CHECK(layer.empty());
            continue;
        }
        CHECK_FALSE(layer.empty());
        if (!layer.empty()) {
            const double a = polygon_area_mm2(layer);
            const double expected = expected_areas[populated];
            if (populated == 0) first_area = a;
            last_area = a;
            expected_total += expected;
            actual_total += a;
            CHECK_THAT(a, Catch::Matchers::WithinAbs(expected, 0.05));
            ++populated;
            CHECK(a > 0.);
        }
    }
    REQUIRE(populated > 2);
    CHECK_THAT(actual_total, Catch::Matchers::WithinAbs(expected_total, 0.1));
    CHECK(first_area > last_area);
    CHECK(first_area > 2. * last_area);
}

TEST_CASE("Painted projection handles a triangle with three distinct heights", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture(Test::mesh(Test::TestMesh::pyramid));
    ModelVolume &volume = fixture.volume();
    volume.set_rotation(Vec3d(Geometry::deg2rad(11.), Geometry::deg2rad(-7.), 0.));
    const size_t facet = sloped_facet(volume.mesh());
    paint_seam_one(volume, facet);
    PrintObject &object = fixture.slice();
    const auto triangle = transformed_triangle(object, volume, facet);
    CHECK(std::abs(triangle[0].z() - triangle[1].z()) > 0.001);
    CHECK(std::abs(triangle[1].z() - triangle[2].z()) > 0.001);
    CHECK(std::abs(triangle[0].z() - triangle[2].z()) > 0.001);
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);
    REQUIRE(projected.size() == object.layers().size());
    const auto range = projected_slab_range(object, triangle);
    const std::vector<double> expected = independently_projected_slab_areas(object, triangle);
    REQUIRE(expected.size() == range.second - range.first + 1);
    for (size_t layer_id = 0; layer_id < projected.size(); ++layer_id) {
        if (layer_id < range.first || layer_id > range.second) {
            CHECK(projected[layer_id].empty());
            continue;
        }
        REQUIRE_FALSE(projected[layer_id].empty());
        CHECK_THAT(polygon_area_mm2(projected[layer_id]),
                   Catch::Matchers::WithinAbs(expected[layer_id - range.first], 0.05));
    }
}

TEST_CASE("Painted support projection reports vertical facet endpoints", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    paint_one(fixture.volume(), side_facet(fixture.volume().mesh()), EnforcerBlockerType::ENFORCER);
    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    std::vector<std::pair<Vec3f, Vec3f>> vertical_points;
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, projected, &vertical_points);
    REQUIRE(projected.size() == object.layers().size());
    REQUIRE_FALSE(vertical_points.empty());
    const size_t facet = side_facet(fixture.volume().mesh());
    BoundingBoxf3 selected_bbox;
    const auto &triangle = fixture.volume().mesh().its.indices[facet];
    for (int i = 0; i < 3; ++i) {
        const Vec3d point = object.trafo_sliced() * fixture.volume().get_matrix() * fixture.volume().mesh().its.vertices[triangle[i]].cast<double>();
        selected_bbox.merge(point);
    }
    for (const auto &segment : vertical_points) {
        CHECK(std::isfinite(segment.first.x()));
        CHECK(std::isfinite(segment.first.y()));
        CHECK(std::isfinite(segment.first.z()));
        CHECK(std::isfinite(segment.second.x()));
        CHECK(std::isfinite(segment.second.y()));
        CHECK(std::isfinite(segment.second.z()));
        CHECK(segment.first.z() >= selected_bbox.min.z() - 0.02f);
        CHECK(segment.first.z() <= selected_bbox.max.z() + 0.02f);
        CHECK(segment.second.z() >= selected_bbox.min.z() - 0.02f);
        CHECK(segment.second.z() <= selected_bbox.max.z() + 0.02f);
        CHECK(std::abs(segment.first.z() - segment.second.z()) > 0.01f);
    }
}

TEST_CASE("Projection visits every painted model volume", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    ModelVolume &primary = fixture.volume();
    ModelVolume *secondary = fixture.model.objects.front()->add_volume(Test::cube(10.));
    secondary->set_offset(Vec3d(15., 0., 0.));
    paint_seam_one(primary, bottom_facet(primary.mesh()));
    paint_seam_one(*secondary, bottom_facet(secondary->mesh()));
    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(true, EnforcerBlockerType::ENFORCER, projected);
    REQUIRE_FALSE(projected.empty());
    REQUIRE_FALSE(projected.front().empty());
    CHECK_THAT(polygon_area_mm2(projected.front()), Catch::Matchers::WithinAbs(250., 0.02));
    CHECK(projected.front().size() >= 2);
}

TEST_CASE("Support projection merges successive facet selections", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    ModelVolume &volume = fixture.volume();
    const std::vector<size_t> bottoms = bottom_facets(volume.mesh());
    REQUIRE(bottoms.size() >= 2);
    TriangleSelector selector(volume.mesh());
    selector.set_facet(static_cast<int>(bottoms[0]), EnforcerBlockerType::ENFORCER);
    selector.set_facet(static_cast<int>(bottoms[1]), EnforcerBlockerType::BLOCKER);
    REQUIRE(volume.supported_facets.set(selector));
    PrintObject &object = fixture.slice();
    std::vector<Polygons> projected;
    object.project_and_append_custom_facets(false, EnforcerBlockerType::ENFORCER, projected);
    object.project_and_append_custom_facets(false, EnforcerBlockerType::BLOCKER, projected);
    REQUIRE_FALSE(projected.empty());
    REQUIRE_FALSE(projected.front().empty());
    CHECK_THAT(polygon_area_mm2(projected.front()), Catch::Matchers::WithinAbs(400., 0.05));
}

TEST_CASE("Bridge contact removal drops only the unsupported middle span", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    PrintObject &object = fixture.slice();
    REQUIRE(object.layers().size() >= 2);
    install_bridge_fixture(object, 1);

    ExPolygons contacts{ExPolygon(rectangle_mm(-10., -2., 10., 2.))};
    const double before = expolygons_area_mm2(contacts);
    PrintObject::remove_bridges_from_contacts(object.get_layer(0), object.get_layer(1),
        float(scale_(0.4)), &contacts, float(scale_(100.)), false);
    REQUIRE_FALSE(contacts.empty());
    CHECK_THAT(before, Catch::Matchers::WithinAbs(80., 1e-3));
    CHECK(expolygons_area_mm2(contacts) < before);
    CHECK_FALSE(std::any_of(contacts.begin(), contacts.end(), [](const ExPolygon &polygon) { return polygon.contains(Point::new_scale(0., 0.)); }));
    CHECK(std::any_of(contacts.begin(), contacts.end(), [](const ExPolygon &polygon) { return polygon.contains(Point::new_scale(-9., 0.)); }));
    CHECK(std::any_of(contacts.begin(), contacts.end(), [](const ExPolygon &polygon) { return polygon.contains(Point::new_scale(9., 0.)); }));
}

TEST_CASE("Breaking a long bridge removes each unsupported segment", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    PrintObject &object = fixture.slice();
    REQUIRE(object.layers().size() >= 2);
    install_bridge_fixture(object, 1);
    ExPolygons contacts{ExPolygon(rectangle_mm(-10., -2., 10., 2.))};
    PrintObject::remove_bridges_from_contacts(object.get_layer(0), object.get_layer(1),
        float(scale_(0.4)), &contacts, float(scale_(4.)), true);
    REQUIRE_FALSE(contacts.empty());
    CHECK(expolygons_area_mm2(contacts) > 0.);
    CHECK(expolygons_area_mm2(contacts) < 80.);
}

TEST_CASE("An overlong bridge is retained when bridge breaking is disabled", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    PrintObject &object = fixture.slice();
    REQUIRE(object.layers().size() >= 2);
    install_bridge_fixture(object, 1);
    ExPolygons contacts{ExPolygon(rectangle_mm(-10., -2., 10., 2.))};
    PrintObject::remove_bridges_from_contacts(object.get_layer(0), object.get_layer(1),
        float(scale_(0.4)), &contacts, float(scale_(4.)), false);
    REQUIRE_FALSE(contacts.empty());
    CHECK_THAT(expolygons_area_mm2(contacts), Catch::Matchers::WithinAbs(80., 0.01));
    CHECK(std::any_of(contacts.begin(), contacts.end(), [](const ExPolygon &polygon) { return polygon.contains(Point::new_scale(0., 0.)); }));
}


TEST_CASE("Bridge surface metadata also removes its contact footprint", "[PrintObject][PrintObjectPainted]")
{
    PaintedFixture fixture;
    PrintObject &object = fixture.slice();
    REQUIRE(object.layers().size() >= 2);
    install_bridge_fixture(object, 1);
    LayerRegion *region = object.get_layer(1)->get_region(0);
    REQUIRE(region != nullptr);
    Surface bridge_surface(stBottomBridge, ExPolygon(rectangle_mm(-10., -2., 10., 2.)));
    bridge_surface.bridge_angle = 0.;
    region->fill_surfaces.surfaces.push_back(std::move(bridge_surface));
    ExPolygons contacts{ExPolygon(rectangle_mm(-10., -2., 10., 2.))};
    PrintObject::remove_bridges_from_contacts(object.get_layer(0), object.get_layer(1),
        float(scale_(0.4)), &contacts, float(scale_(100.)), false);
    CHECK(contacts.empty());
}

} // namespace painted_projection_contracts
