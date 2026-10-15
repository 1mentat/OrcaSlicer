#include <cstddef>
#include "libslic3r/libslic3r.h"
#include "libslic3r/Config.hpp"
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <catch2/catch_all.hpp>
#include <algorithm>
#include <utility>
#include <vector>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Point.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/BuildVolume.hpp"
#include "libslic3r/ObjectID.hpp"

using namespace Slic3r;

namespace {

TriangleMesh disconnected_cubes()
{
    TriangleMesh first = make_cube(10, 10, 10);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    first.merge(second);
    return first;
}

BuildVolume rectangular_bed()
{
    return BuildVolume({ Vec2d(0, 0), Vec2d(100, 0), Vec2d(100, 100), Vec2d(0, 100) }, 100., {}, {});
}

void check_vec3d(const Vec3d &actual, const Vec3d &expected, double epsilon)
{
    CHECK_THAT(actual.x(), Catch::Matchers::WithinAbs(expected.x(), epsilon));
    CHECK_THAT(actual.y(), Catch::Matchers::WithinAbs(expected.y(), epsilon));
    CHECK_THAT(actual.z(), Catch::Matchers::WithinAbs(expected.z(), epsilon));
}

} // namespace

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

TEST_CASE("Model lifecycle preserves material identity and mesh sharing", "[Model][ModelContracts]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "Lifecycle part";
    object->input_file = "lifecycle.stl";
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
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
    CHECK(shared->mesh_ptr() == part->mesh_ptr());
    CHECK(shared->is_negative_volume());
    CHECK(object->parts_count() == 1);
    CHECK(object->facets_count() == 12);

    ModelObject *clone = model.add_object(*object);
    REQUIRE(clone != nullptr);
    REQUIRE(clone->volumes.size() == object->volumes.size());
    CHECK(clone->id() != object_id);
    CHECK(clone->volumes.front()->id() != volume_id);
    CHECK(clone->volumes.front()->mesh_ptr() == part->mesh_ptr());
    CHECK(clone->instances.size() == 1);

    CHECK(model.delete_object(clone));
    CHECK_FALSE(model.delete_object(ObjectID()));
    CHECK(model.delete_object(object_id));
    CHECK(model.objects.empty());
}

TEST_CASE("Model instance transforms have analytic bounds and bed states", "[Model][ModelContracts]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    ModelInstance *instance = object->add_instance(Vec3d(20, 30, -5), Vec3d(2, 1, 1), Vec3d::Zero(), Vec3d::Ones());

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

    Model empty_model;
    CHECK_THROWS(empty_model.duplicate_objects_grid(1, 1, 1.));

    Model two_objects;
    two_objects.add_object()->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    two_objects.add_object()->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    CHECK_THROWS(two_objects.duplicate_objects_grid(1, 1, 1.));

    Model state_model;
    ModelObject *state_object = state_model.add_object();
    state_object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    ModelInstance *state_instance = state_object->add_instance();
    const BuildVolume bed = rectangular_bed();
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

TEST_CASE("Model grid duplication uses mesh dimensions for spacing", "[Model][ModelContracts]")
{
    const bool has_initial_instance = GENERATE(false, true);
    Model grid_model;
    ModelObject *grid_object = grid_model.add_object();
    grid_object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    if (has_initial_instance)
        grid_object->add_instance();

    grid_model.duplicate_objects_grid(2, 2, 5.);
    REQUIRE(grid_object->instances.size() == 4);
    check_vec3d(grid_model.bounding_box_exact().size(), Vec3d(25, 25, 10), 1e-5);
}

TEST_CASE("Model split and merge preserve disconnected component geometry", "[Model][ModelContracts]")
{
    Model split_model;
    ModelObject *source = split_model.add_object();
    source->name = "Disconnected";
    source->add_volume(disconnected_cubes(), ModelVolumeType::MODEL_PART, false);
    source->add_instance();
    ModelObjectPtrs split_objects;
    source->split(&split_objects, false);
    REQUIRE(split_objects.size() == 2);
    for (ModelObject *object : split_objects) {
        REQUIRE(object != nullptr);
        CHECK(object->volumes.size() == 1);
        CHECK(object->parts_count() == 1);
        CHECK(object->facets_count() == 12);
        CHECK(object->instances.size() == 1);
    }
    std::vector<double> split_min_x;
    for (ModelObject *object : split_objects)
        split_min_x.push_back(object->instance_bounding_box(0).min.x());
    std::sort(split_min_x.begin(), split_min_x.end());
    REQUIRE(split_min_x.size() == 2);
    CHECK_THAT(split_min_x[0], Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(split_min_x[1], Catch::Matchers::WithinAbs(30., 1e-5));

    Model merge_model;
    ModelObject *merged = merge_model.add_object();
    merged->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    merged->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    merged->add_instance();
    CHECK(merged->parts_count() == 2);
    CHECK(merged->facets_count() == 24);
    merged->merge();
    REQUIRE(merged->volumes.size() == 1);
    CHECK(merged->parts_count() == 1);
    CHECK(merged->facets_count() == 24);
    check_vec3d(merged->raw_mesh_bounding_box().size(), Vec3d(40, 10, 10), 1e-5);
}

TEST_CASE("Model volume types and unit conversion retain semantic metadata", "[Model][ModelContracts]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *part = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    ModelVolume *negative = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::NEGATIVE_VOLUME, false);
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
    CHECK(support->get_extruders().empty());
    CHECK(modifier->is_modifier());
    CHECK(support->is_support_blocker());
    CHECK_FALSE(part->is_splittable());
    CHECK(ModelVolume::type_from_string(ModelVolume::type_to_string(ModelVolumeType::SUPPORT_ENFORCER)) == ModelVolumeType::SUPPORT_ENFORCER);

    Model unit_model;
    ModelObject *inch_object = unit_model.add_object();
    ModelVolume *inch_volume = inch_object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    CHECK(unit_model.looks_like_imperial_units());
    unit_model.convert_from_imperial_units(true);
    check_vec3d(inch_volume->mesh().bounding_box().size(), Vec3d(25.4, 25.4, 25.4), 1e-4);
    CHECK(inch_volume->source.is_converted_from_inches);
    CHECK_FALSE(inch_volume->source.is_converted_from_meters);

    Model meter_model;
    ModelObject *meter_object = meter_model.add_object();
    ModelVolume *meter_volume = meter_object->add_volume(make_cube(0.1, 0.1, 0.1), ModelVolumeType::MODEL_PART, false);
    CHECK(meter_model.looks_like_saved_in_meters());
    meter_model.convert_from_meters(true);
    check_vec3d(meter_volume->mesh().bounding_box().size(), Vec3d(100., 100., 100.), 1e-4);
    CHECK(meter_volume->source.is_converted_from_meters);
    CHECK_FALSE(meter_volume->source.is_converted_from_inches);

    Model cleanup_model;
    cleanup_model.add_object()->add_volume(TriangleMesh{}, ModelVolumeType::MODEL_PART, false);
    cleanup_model.add_object()->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    CHECK(cleanup_model.removed_objects_with_zero_volume() == 1);
    CHECK(cleanup_model.objects.size() == 1);
}

namespace model_extended_contracts {
using namespace Slic3r;

namespace {

void check_vec3d(const Vec3d &actual, const Vec3d &expected, double epsilon = 1e-5)
{
    CHECK_THAT(actual.x(), Catch::Matchers::WithinAbs(expected.x(), epsilon));
    CHECK_THAT(actual.y(), Catch::Matchers::WithinAbs(expected.y(), epsilon));
    CHECK_THAT(actual.z(), Catch::Matchers::WithinAbs(expected.z(), epsilon));
}

void check_box(const BoundingBoxf3 &box, const Vec3d &min, const Vec3d &max, double epsilon = 1e-5)
{
    check_vec3d(box.min.cast<double>(), min, epsilon);
    check_vec3d(box.max.cast<double>(), max, epsilon);
}

ModelObject *cube_object(Model &model, double size = 10., bool add_instance = true)
{
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(float(size), float(size), float(size)), ModelVolumeType::MODEL_PART, false);
    if (add_instance)
        object->add_instance();
    return object;
}

TriangleMesh disconnected_cubes()
{
    TriangleMesh first = make_cube(10, 10, 10);
    TriangleMesh second = make_cube(10, 10, 10);
    second.translate(Vec3f(30, 0, 0));
    first.merge(second);
    return first;
}

} // namespace

TEST_CASE("Model copy and clone retain topology while clone IDs are independent", "[Model][ModelExtended]")
{
    Model original;
    ModelObject *object = cube_object(original);
    object->name = "copy-source";
    object->volumes.front()->name = "body";
    object->volumes.front()->set_material_id("blue");
    original.get_material("blue")->apply({ { "color", "#0000ff" } });

    const ObjectID model_id = original.id();
    const ObjectID object_id = object->id();
    const ObjectID volume_id = object->volumes.front()->id();
    Model copied(original);
    REQUIRE(copied.id() == model_id);
    REQUIRE(copied.objects.size() == 1);
    CHECK(copied.objects.front()->id() == object_id);
    CHECK(copied.objects.front()->volumes.front()->id() == volume_id);
    CHECK(copied.objects.front() != object);
    CHECK(copied.objects.front()->volumes.front()->mesh_ptr() == object->volumes.front()->mesh_ptr());
    CHECK(copied.objects.front()->volumes.front()->material()->attributes.at("color") == "#0000ff");

    Model cloned = original.make_clone(original);
    REQUIRE(cloned.objects.size() == 1);
    CHECK(cloned.id() != model_id);
    CHECK(cloned.objects.front()->id() != object_id);
    CHECK(cloned.objects.front()->volumes.front()->id() != volume_id);
    CHECK(cloned.objects.front()->get_model() == &cloned);
    CHECK(cloned.objects.front()->volumes.front()->get_object() == cloned.objects.front());
    CHECK(model_object_list_equal(original, copied));
    CHECK_FALSE(model_object_list_equal(original, cloned));
}

TEST_CASE("Model and object meshes aggregate transformed instances exactly", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(5, 7, 2));
    ModelInstance *second = object->add_instance();
    second->set_offset(Vec3d(25, 7, 2));
    second->set_scaling_factor(Vec3d(2, 1, 1));

    const BoundingBoxf3 object_mesh_box = object->mesh().bounding_box();
    const BoundingBoxf3 model_mesh_box = model.mesh().bounding_box();
    check_box(object_mesh_box, Vec3d(5, 7, 2), Vec3d(45, 17, 12));
    check_box(model_mesh_box, Vec3d(5, 7, 2), Vec3d(45, 17, 12));
    check_box(object->instance_convex_hull_bounding_box(size_t(0)), Vec3d(5, 7, 2), Vec3d(15, 17, 12));
    check_box(object->instance_convex_hull_bounding_box(second), Vec3d(25, 7, 2), Vec3d(45, 17, 12));
}

TEST_CASE("Model instance transform helpers separate translation from vector geometry", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    ModelInstance *instance = object->instances.front();
    instance->set_offset(Vec3d(11, 13, 17));
    instance->set_scaling_factor(Vec3d(2, 3, 4));
    instance->set_rotation(Z, 0.5);

    const Vec3d transformed = instance->transform_vector(Vec3d(1, 0, 0));
    const Vec3d without_offset = instance->transform_vector(Vec3d(1, 0, 0), true);
    check_vec3d(without_offset, Vec3d(2. * std::cos(0.5), 2. * std::sin(0.5), 0.));
    check_vec3d(transformed, Vec3d(11. + 2. * std::cos(0.5), 13. + 2. * std::sin(0.5), 17.));
    check_vec3d(transformed - without_offset, Vec3d(11, 13, 17));

    BoundingBoxf3 source_box;
    source_box.merge(Vec3d::Zero());
    source_box.merge(Vec3d::Ones());
    check_box(instance->transform_bounding_box(source_box),
              Vec3d(11. - 3. * std::sin(0.5), 13., 17.),
              Vec3d(11. + 2. * std::cos(0.5), 13. + 2. * std::sin(0.5) + 3. * std::cos(0.5), 21.));

    Polygon polygon({ Point(0, 0), Point(scaled(1.), 0), Point(scaled(1.), scaled(1.)), Point(0, scaled(1.)) });
    instance->transform_polygon(&polygon);
    const double rotated_unit_extent = std::abs(std::cos(0.5)) + std::abs(std::sin(0.5));
    CHECK_THAT(double(polygon.bounding_box().size().x()), Catch::Matchers::WithinAbs(scaled(2. * rotated_unit_extent), 3.));
    CHECK_THAT(double(polygon.bounding_box().size().y()), Catch::Matchers::WithinAbs(scaled(3. * rotated_unit_extent), 3.));
}

TEST_CASE("Scaling to fit uses the limiting model dimension", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(10, 20, 30));
    const Vec3d before = object->bounding_box_exact().size().cast<double>();
    object->scale_to_fit(Vec3d(15, 30, 40));
    const Vec3d after = object->bounding_box_exact().size().cast<double>();

    const double factor = 15. / before.x();
    check_vec3d(after, before * factor);
    CHECK_THAT(after.x(), Catch::Matchers::WithinAbs(15., 1e-5));
    CHECK(after.y() < 30.);
    CHECK(after.z() < 40.);
}

TEST_CASE("Volume transformations update mesh bounds and handedness", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model, 4., false);
    ModelVolume *volume = object->volumes.front();
    volume->center_geometry_after_creation();
    check_box(volume->mesh().bounding_box(), Vec3d(-2, -2, -2), Vec3d(2, 2, 2));
    CHECK_THAT(volume->source.mesh_offset.x(), Catch::Matchers::WithinAbs(2., 1e-5));

    volume->translate(Vec3d(3, 4, 5));
    volume->scale(Vec3d(2, 3, 4));
    check_vec3d(volume->get_offset(), Vec3d(5, 6, 7));
    check_vec3d(volume->get_scaling_factor(), Vec3d(2, 3, 4));
    CHECK_FALSE(volume->is_left_handed());
    volume->mirror(X);
    CHECK(volume->is_left_handed());
    volume->rotate(0.5, Z);
    CHECK_THAT(volume->get_rotation(Z), Catch::Matchers::WithinAbs(0.5, 1e-5));
    CHECK(volume->get_convex_hull().facets_count() > 0);
}

TEST_CASE("Model volume split preserves each disconnected solid and its assignment", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *volume = object->add_volume(disconnected_cubes(), ModelVolumeType::MODEL_PART, false);
    volume->name = "multi";
    volume->config.set("extruder", 2);
    object->add_instance();
    REQUIRE(volume->is_splittable());

    const size_t created = volume->split(4, false);
    REQUIRE(created == 2);
    REQUIRE(object->volumes.size() == 2);
    std::vector<double> min_x;
    for (const ModelVolume *part : object->volumes) {
        REQUIRE(part->is_model_part());
        CHECK(part->mesh().facets_count() == 12);
        CHECK(part->get_extruders() == std::vector<int>{ 2 });
        CHECK(part->get_convex_hull().facets_count() > 0);
        min_x.push_back(part->mesh().transformed_bounding_box(part->get_matrix()).min.x());
    }
    std::sort(min_x.begin(), min_x.end());
    CHECK_THAT(min_x[0], Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(min_x[1], Catch::Matchers::WithinAbs(30., 1e-5));
}

TEST_CASE("Merging selected volumes retains unselected volume geometry", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *left = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    left->set_offset(Vec3d(0, 0, 0));
    ModelVolume *right = object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    right->set_offset(Vec3d(20, 0, 0));
    ModelVolume *modifier = object->add_volume(make_cube(5, 5, 5), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_offset(Vec3d(40, 0, 0));
    object->add_instance();

    std::vector<int> selected{ 0, 1 };
    // merge_volumes returns detached objects; Model provides their owning cleanup.
    Model merged_model;
    merged_model.objects = object->merge_volumes(selected);
    const ModelObjectPtrs &result = merged_model.objects;
    REQUIRE(result.size() == 1);
    REQUIRE(result.front()->volumes.size() == 2);
    CHECK(result.front()->volumes.front()->is_model_part());
    CHECK(result.front()->volumes.back()->is_modifier());
    check_box(result.front()->volumes.front()->mesh().transformed_bounding_box(result.front()->volumes.front()->get_matrix()),
              Vec3d(0, 0, 0), Vec3d(30, 10, 10));
    check_box(result.front()->volumes.back()->mesh().transformed_bounding_box(result.front()->volumes.back()->get_matrix()),
              Vec3d(40, 0, 0), Vec3d(45, 5, 5));
}

TEST_CASE("Multipart conversion preserves world geometry and material volume count", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *first = cube_object(model, 10.);
    first->name = "first";
    first->input_file = "/tmp/first.stl";
    first->instances.front()->set_offset(Vec3d(3, 4, 0));
    ModelObject *second = cube_object(model, 5.);
    second->name = "second";
    second->input_file = "/tmp/second.stl";
    second->instances.front()->set_offset(Vec3d(30, 4, 5));
    const BoundingBoxf3 before = model.mesh().bounding_box();
    REQUIRE(model.looks_like_multipart_object());

    model.convert_multipart_object(2);
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 2);
    CHECK(model.objects.front()->instances.empty());
    check_box(model.objects.front()->full_raw_mesh_bounding_box(), before.min.cast<double>(), before.max.cast<double>());
}

TEST_CASE("Baking rotated instances preserves world bounds and removes unsupported transforms", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    ModelInstance *first = object->instances.front();
    first->set_offset(Vec3d(10, 20, 0));
    first->set_rotation(Z, 0.37);
    first->set_scaling_factor(Vec3d(2, 1, 1));
    ModelInstance *second = object->add_instance(*first);
    second->set_offset(Vec3d(40, 20, 0));
    const BoundingBoxf3 before = object->mesh().bounding_box();

    object->bake_xy_rotation_into_meshes(0);
    const BoundingBoxf3 after = object->mesh().bounding_box();
    check_box(after, before.min.cast<double>(), before.max.cast<double>(), 1e-4);
    CHECK_THAT(first->get_rotation(Z), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(second->get_rotation(Z), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK(first->get_scaling_factor().x() > 0.);
    CHECK(first->get_mirror().isApprox(Vec3d::Ones(), 1e-6));
    CHECK(object->volumes.front()->source.input_file.empty());
}

TEST_CASE("Model predicates distinguish ordinary, multipart, and advanced features", "[Model][ModelExtended]")
{
    Model ordinary;
    ModelObject *ordinary_object = cube_object(ordinary);
    CHECK_FALSE(model_has_multi_part_objects(ordinary));
    CHECK_FALSE(model_has_advanced_features(ordinary));

    Model multipart;
    ModelObject *multipart_object = cube_object(multipart);
    multipart_object->add_volume(make_cube(2, 2, 2), ModelVolumeType::PARAMETER_MODIFIER, false);
    multipart_object->add_instance();
    CHECK(model_has_multi_part_objects(multipart));
    CHECK(model_has_advanced_features(multipart));

    Model configured;
    ModelObject *configured_object = cube_object(configured);
    configured_object->config.set("extruder", 1);
    CHECK_FALSE(model_has_advanced_features(configured));
    configured_object->config.set_key_value("custom_model_setting", new ConfigOptionInt(3));
    CHECK(model_has_advanced_features(configured));
    CHECK(ordinary_object->has_solid_mesh());
}

TEST_CASE("Model volume change predicates ignore filtered modifiers and detect transforms", "[Model][ModelExtended]")
{
    Model source;
    ModelObject *source_object = cube_object(source);
    source_object->add_volume(make_cube(2, 2, 2), ModelVolumeType::PARAMETER_MODIFIER, false);
    Model changed(source);
    REQUIRE(model_object_list_equal(source, changed));
    CHECK_FALSE(model_volume_list_changed(*source_object, *changed.objects.front(), ModelVolumeType::MODEL_PART));
    CHECK_FALSE(model_volume_list_changed(*source_object, *changed.objects.front(), { ModelVolumeType::MODEL_PART }));

    changed.objects.front()->volumes.front()->set_offset(Vec3d(2, 0, 0));
    CHECK(model_volume_list_changed(*source_object, *changed.objects.front(), ModelVolumeType::MODEL_PART));
    CHECK_FALSE(model_volume_list_changed(*source_object, *changed.objects.front(), ModelVolumeType::PARAMETER_MODIFIER));
    changed.objects.front()->volumes.front()->set_offset(Vec3d::Zero());
    changed.objects.front()->delete_volume(1);
    CHECK(model_volume_list_changed(*source_object, *changed.objects.front(), ModelVolumeType::PARAMETER_MODIFIER));
    CHECK(model_object_list_extended(source, changed) == false);
    changed.add_object();
    CHECK(model_object_list_extended(source, changed));
}

TEST_CASE("Filament slot remapping updates object and volume assignments together", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->config.set("extruder", 2);
    object->volumes.front()->config.set("extruder", 3);
    remap_model_filament_slots(model, { { 1, 0 }, { 2, 1 } });
    CHECK(object->config.extruder() == 1);
    CHECK(object->volumes.front()->config.extruder() == 2);

    remap_model_filament_slots(model, {});
    CHECK(object->config.extruder() == 1);
    CHECK(object->volumes.front()->config.extruder() == 2);
    object->volumes.front()->update_extruder_count(1);
    CHECK_FALSE(object->volumes.front()->config.has("extruder"));
    CHECK(object->volumes.front()->extruder_id() == 1);
}

TEST_CASE("Adjusting minimum Z preserves instance spacing while moving a sinking object", "[Model][ModelExtended]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(2, 3, -6));
    ModelInstance *fixed = object->add_instance();
    fixed->set_offset(Vec3d(40, 3, 2));
    fixed->auto_drop = false;
    const double fixed_z = fixed->get_offset(Z);
    model.adjust_min_z();
    CHECK_THAT(object->instances.front()->get_offset(Z), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT(fixed->get_offset(Z), Catch::Matchers::WithinAbs(fixed_z + 6., 1e-6));
    CHECK_THAT(model.bounding_box_exact().min.z(), Catch::Matchers::WithinAbs(0., 1e-6));
}

TEST_CASE("Volume type names round-trip every persisted model category", "[Model][ModelExtended]")
{
    const std::vector<std::pair<ModelVolumeType, std::string>> types{
        {ModelVolumeType::MODEL_PART, "normal_part"},
        {ModelVolumeType::NEGATIVE_VOLUME, "negative_part"},
        {ModelVolumeType::PARAMETER_MODIFIER, "modifier_part"},
        {ModelVolumeType::SUPPORT_ENFORCER, "support_enforcer"},
        {ModelVolumeType::SUPPORT_BLOCKER, "support_blocker"},
        {ModelVolumeType::PRECISE_SEAM_CENTER, "precise_seam_center"},
        {ModelVolumeType::PRECISE_SEAM_LEFT, "precise_seam_left"},
        {ModelVolumeType::PRECISE_SEAM_RIGHT, "precise_seam_right"},
        {ModelVolumeType::PRECISE_SEAM_ENFORCED, "precise_seam_enforced"},
        {ModelVolumeType::PRECISE_SEAM_BLOCKED, "precise_seam_blocked"},
        {ModelVolumeType::PRECISE_SEAM_NEUTRAL, "precise_seam_neutral"},
    };
    for (const auto &[type, name] : types) {
        CHECK(ModelVolume::type_to_string(type) == name);
        CHECK(ModelVolume::type_from_string(name) == type);
    }
    CHECK(ModelVolume::type_from_string("unrecognized") == ModelVolumeType::MODEL_PART);
}

} // namespace model_extended_contracts
