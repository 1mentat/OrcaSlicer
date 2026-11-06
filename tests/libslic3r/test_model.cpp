#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/BuildVolume.hpp"
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/ObjectID.hpp"
#include <catch2/catch_all.hpp>
#include <algorithm>
#include <utility>
#include <vector>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"

using namespace Slic3r;

// convex_hull_2d does not clip geometry below the bed, so these cases avoid
// sinking transforms.
TEST_CASE("A part's 2D convex hull is its footprint projected onto the bed", "[Model]")
{
    Model model;
    ModelObject* object = model.add_object();
    // Keep the cube's raw coordinates ([0,20] on every axis): the default
    // add_volume re-centers the geometry, which would move the footprint.
    object->add_volume(make_cube(20, 20, 20), ModelVolumeType::MODEL_PART, false);

    SECTION("identity transform yields the 20 mm square") {
        const Polygon hull   = object->convex_hull_2d(Geometry::Transformation{}.get_matrix());
        const BoundingBox bb = hull.bounding_box();
        CHECK(hull.size() == 4);
        CHECK(bb.min.x() == scaled(0.));
        CHECK(bb.min.y() == scaled(0.));
        CHECK(bb.max.x() == scaled(20.));
        CHECK(bb.max.y() == scaled(20.));
    }

    SECTION("scaling and offset move and grow the footprint") {
        Geometry::Transformation t;
        t.set_scaling_factor({2, 2, 2}); // cube now spans [0,40]
        t.set_offset({10, 5, 0});        // then shift +10 in X, +5 in Y

        const Polygon hull   = object->convex_hull_2d(t.get_matrix());
        const BoundingBox bb = hull.bounding_box();
        CHECK(hull.size() == 4);
        CHECK(bb.min.x() == scaled(10.));
        CHECK(bb.min.y() == scaled(5.));
        CHECK(bb.max.x() == scaled(50.));
        CHECK(bb.max.y() == scaled(45.));
    }
}

TEST_CASE("An object's raw mesh keeps the triangles of each part on its own vertices", "[Model]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(30, 0, 0);
    object->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);

    // Two separate cubes stay two closed components, one around each cube.
    const std::vector<indexed_triangle_set> parts = its_split(object->raw_indexed_triangle_set());
    REQUIRE(parts.size() == 2);
    std::vector<double> min_x;
    for (const indexed_triangle_set &part : parts) {
        CHECK(part.indices.size() == 12);
        min_x.push_back(bounding_box(part).min.x());
    }
    std::sort(min_x.begin(), min_x.end());
    CHECK_THAT(min_x.front(), Catch::Matchers::WithinAbs(0., 1e-4));
    CHECK_THAT(min_x.back(), Catch::Matchers::WithinAbs(30., 1e-4));
}

TEST_CASE("Model lifecycle preserves material identity and mesh sharing", "[Model][ModelLifecycleSharing]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->name = "Lifecycle part";
    object->input_file = "lifecycle.stl";
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    REQUIRE(part != nullptr);
    const ObjectID object_id = object->id();
    const ObjectID volume_id = part->id();

    CHECK(model.add_default_instances());
    REQUIRE(object->instances.size() == 1);
    CHECK(model.add_default_instances());
    CHECK(object->instances.size() == 1);

    ModelMaterial *material = model.add_material("red");
    REQUIRE(material != nullptr);
    material->apply({ { "color", "#ff0000" } });
    part->set_material_id("red");
    CHECK(part->material() == material);
    CHECK(material->attributes.at("color") == "#ff0000");
    CHECK(object->materials_count() == 1);

    ModelVolume *shared = object->add_volume_with_shared_mesh(*part, ModelVolumeType::NEGATIVE_VOLUME);
    REQUIRE(shared != nullptr);
    REQUIRE(object->volumes.size() == 2);
    CHECK(shared->mesh_ptr() == part->mesh_ptr());
    CHECK(shared->is_negative_volume());
    CHECK(object->parts_count() == 1);
    CHECK(object->facets_count() == 12);

    ModelObject *clone = model.add_object(*object);
    REQUIRE(clone != nullptr);
    REQUIRE(clone->volumes.size() == object->volumes.size());
    REQUIRE(clone->volumes.size() == 2);
    REQUIRE(clone->volumes.front() != nullptr);
    CHECK(clone->id() != object_id);
    CHECK(clone->volumes.front()->id() != volume_id);
    CHECK(clone->volumes.front()->mesh_ptr() == part->mesh_ptr());
    CHECK(clone->instances.size() == 1);

    CHECK(model.delete_object(clone));
    CHECK_FALSE(model.delete_object(ObjectID()));
    CHECK(model.delete_object(object_id));
    CHECK(model.objects.empty());
}



namespace model_bounds_contracts {
namespace {

void check_vec3d(const Vec3d &actual, const Vec3d &expected, double epsilon = 1e-5)
{
    CHECK_THAT(actual.x(), Catch::Matchers::WithinAbs(expected.x(), epsilon));
    CHECK_THAT(actual.y(), Catch::Matchers::WithinAbs(expected.y(), epsilon));
    CHECK_THAT(actual.z(), Catch::Matchers::WithinAbs(expected.z(), epsilon));
}


} // namespace
} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model instance transforms have analytic world and local bounds", "[Model][ModelInstanceBounds]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *instance = object->add_instance(Vec3d(20, 30, -5), Vec3d(2, 1, 1), Vec3d::Zero(), Vec3d::Ones());
    REQUIRE(instance != nullptr);

    const BoundingBoxf3 world_box = object->instance_bounding_box(*instance);
    check_vec3d(world_box.min, Vec3d(20, 30, -5), 1e-5);
    check_vec3d(world_box.max, Vec3d(40, 40, 5), 1e-5);
    const BoundingBoxf3 no_offset_box = object->instance_bounding_box(*instance, true);
    check_vec3d(no_offset_box.min, Vec3d(0, 0, 0), 1e-5);
    check_vec3d(no_offset_box.max, Vec3d(20, 10, 10), 1e-5);
    check_vec3d(object->raw_mesh_bounding_box().min, Vec3d(0, 0, 0), 1e-5);
    check_vec3d(object->raw_mesh_bounding_box().max, Vec3d(10, 10, 10), 1e-5);
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-5., 1e-6));
    CHECK_THAT(object->max_z(), Catch::Matchers::WithinAbs(5., 1e-6));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model instance centering and bed placement retain projected dimensions", "[Model][ModelInstancePlacement]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *instance = object->add_instance(Vec3d(20, 30, -5), Vec3d(2, 1, 1), Vec3d::Zero(), Vec3d::Ones());
    REQUIRE(instance != nullptr);

    CHECK(model.center_instances_around_point(Vec2d(0, 0)));
    CHECK_FALSE(model.center_instances_around_point(Vec2d(0, 0)));
    const Vec3d centered = object->bounding_box_exact().center();
    CHECK_THAT(centered.x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(centered.y(), Catch::Matchers::WithinAbs(0., 1e-5));

    object->translate_instances(Vec3d(0, 0, -10));
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-15., 1e-6));
    object->ensure_on_bed();
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(0., 1e-6));

    const Polygon instance_hull = instance->convex_hull_2d();
    CHECK(instance_hull.bounding_box().size().x() == scaled(20.));
    CHECK(instance_hull.bounding_box().size().y() == scaled(10.));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model grid duplication rejects empty and multiple-object models", "[Model][ModelGridRejection]")
{
    Model empty_model;
    CHECK_THROWS(empty_model.duplicate_objects_grid(1, 1, 1.));

    Model two_objects;
    for (int i = 0; i < 2; ++i) {
        ModelObject *object = two_objects.add_object();
        REQUIRE(object != nullptr);
        REQUIRE(object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false) != nullptr);
    }
    REQUIRE(two_objects.objects.size() == 2);
    CHECK_THROWS(two_objects.duplicate_objects_grid(1, 1, 1.));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

// Known bug: grid duplication clears instances before measuring their bounds,
// so the copies are spaced only by the requested gap. Both initial states must expose it.
// Fixed by the separate follow-up Model grid-spacing fix (M118).
TEST_CASE("Model grid duplication uses mesh dimensions for spacing", "[Model][ModelGridSpacing][!shouldfail]")
{
    const bool has_initial_instance = GENERATE(false, true);
    CAPTURE(has_initial_instance);
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    REQUIRE(object->add_volume(make_cube(10, 20, 7), ModelVolumeType::MODEL_PART, false) != nullptr);
    if (has_initial_instance)
        REQUIRE(object->add_instance() != nullptr);

    model.duplicate_objects_grid(2, 2, 5.);
    REQUIRE(object->instances.size() == 4);
    for (const ModelInstance *instance : object->instances)
        REQUIRE(instance != nullptr);
    check_vec3d(model.bounding_box_exact().size(), Vec3d(25, 45, 7), 1e-5);
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model print-volume classification follows instance movement", "[Model][ModelPrintVolumeState]")
{
    Model state_model;
    ModelObject *state_object = state_model.add_object();
    REQUIRE(state_object != nullptr);
    REQUIRE(state_object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    ModelInstance *state_instance = state_object->add_instance();
    REQUIRE(state_instance != nullptr);
    const BuildVolume bed({Vec2d(0, 0), Vec2d(100, 0), Vec2d(100, 100), Vec2d(0, 100)}, 100., {}, {});
    CHECK(state_model.update_print_volume_state(bed) == 1);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Inside);
    state_instance->set_offset(Vec3d(95, 0, 0));
    state_object->invalidate_bounding_box();
    CHECK(state_model.update_print_volume_state(bed) == 0);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Partly_Outside);
    state_instance->set_offset(Vec3d(200, 0, 0));
    state_object->invalidate_bounding_box();
    CHECK(state_model.update_print_volume_state(bed) == 0);
    CHECK(state_instance->print_volume_state == ModelInstancePVS_Fully_Outside);
}

} // namespace model_bounds_contracts


namespace model_bounds_contracts {

TEST_CASE("Model object splitting preserves disconnected components and positions", "[Model][ModelObjectSplit]")
{
    Model split_model;
    ModelObject *source = split_model.add_object();
    REQUIRE(source != nullptr);
    source->name = "Disconnected";
    TriangleMesh disconnected = make_cube(10, 10, 10);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    disconnected.merge(second);
    REQUIRE(source->add_volume(std::move(disconnected), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(source->add_instance() != nullptr);
    ModelObjectPtrs split_objects;
    source->split(&split_objects, false);
    REQUIRE(split_objects.size() == 2);
    for (ModelObject *object : split_objects) {
        REQUIRE(object != nullptr);
        REQUIRE(object->volumes.size() == 1);
        REQUIRE(object->volumes.front() != nullptr);
        CHECK(object->parts_count() == 1);
        CHECK(object->facets_count() == 12);
        REQUIRE(object->instances.size() == 1);
        REQUIRE(object->instances.front() != nullptr);
    }
    std::vector<double> split_min_x;
    for (ModelObject *object : split_objects)
        split_min_x.push_back(object->instance_bounding_box(0).min.x());
    std::sort(split_min_x.begin(), split_min_x.end());
    REQUIRE(split_min_x.size() == 2);
    CHECK_THAT(split_min_x[0], Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(split_min_x[1], Catch::Matchers::WithinAbs(30., 1e-5));
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model volume merge preserves geometry and facet count", "[Model][ModelObjectMerge]")
{
    Model merge_model;
    ModelObject *merged = merge_model.add_object();
    REQUIRE(merged != nullptr);
    REQUIRE(merged->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false) != nullptr);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    REQUIRE(merged->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false) != nullptr);
    REQUIRE(merged->add_instance() != nullptr);
    CHECK(merged->parts_count() == 2);
    CHECK(merged->facets_count() == 24);
    merged->merge();
    REQUIRE(merged->volumes.size() == 1);
    REQUIRE(merged->volumes.front() != nullptr);
    CHECK(merged->parts_count() == 1);
    CHECK(merged->facets_count() == 24);
    check_vec3d(merged->raw_mesh_bounding_box().size(), Vec3d(40, 10, 10), 1e-5);
}

} // namespace model_bounds_contracts

namespace model_bounds_contracts {

TEST_CASE("Model volume types preserve material and extruder semantics", "[Model][ModelVolumeSemantics]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    ModelVolume *negative = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::NEGATIVE_VOLUME, false);
    REQUIRE(part != nullptr);
    REQUIRE(negative != nullptr);
    object->config.set("extruder", 2);
    part->set_material_id("part-material");
    negative->set_material_id("negative-material");
    part->config.set("extruder", 3);

    CHECK(object->parts_count() == 1);
    CHECK(object->facets_count() == 12);
    CHECK(object->materials_count() == 2);
    CHECK(object->has_solid_mesh());
    CHECK(part->extruder_id() == 3);
    part->config.erase("extruder");
    CHECK(part->extruder_id() == 2);
    CHECK(part->get_extruders() == std::vector<int>{ 2 });
    CHECK(negative->get_extruders().empty());
    ModelVolume *modifier = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::PARAMETER_MODIFIER, false);
    ModelVolume *support = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::SUPPORT_BLOCKER, false);
    REQUIRE(modifier != nullptr);
    REQUIRE(support != nullptr);
    CHECK(support->get_extruders().empty());
    CHECK(modifier->is_modifier());
    CHECK(support->is_support_blocker());
    CHECK_FALSE(part->is_splittable());
    CHECK(ModelVolume::type_from_string(ModelVolume::type_to_string(ModelVolumeType::SUPPORT_ENFORCER)) == ModelVolumeType::SUPPORT_ENFORCER);
}

} // namespace model_bounds_contracts
