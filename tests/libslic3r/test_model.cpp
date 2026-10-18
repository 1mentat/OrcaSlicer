#include <boost/filesystem/operations.hpp>

#include <ios>
#include "libslic3r/Semver.hpp"
#include <array>
#include <cstdint>
#include <fstream>
#include <functional>
#include <iterator>
#include <tuple>
#include <boost/nowide/fstream.hpp>
#include "libslic3r/Exception.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/TexturePainting.hpp"
#include "libslic3r/Utils.hpp"
#include "test_utils.hpp"
#include "libslic3r/CustomGCode.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Arrange.hpp"
#include <set>
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
#include <catch2/matchers/catch_matchers_string.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/Format/STEP.hpp"
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

namespace model_extended_contracts {
TEST_CASE("Model copy and move assignment preserve parent links and ownership", "[Model][ModelExtendedVariance]")
{
    Model source;
    ModelObject *object = cube_object(source);
    object->name = "source";
    source.add_material("PLA")->attributes["type"] = "PLA";
    object->volumes.front()->set_material_id("PLA");
    const ObjectID model_id = source.id();

    Model copied;
    copied = source;
    REQUIRE(copied.id() == model_id);
    REQUIRE(copied.objects.size() == 1);
    CHECK(copied.objects.front()->get_model() == &copied);
    CHECK(copied.objects.front()->volumes.front()->get_object() == copied.objects.front());
    CHECK(copied.get_material("PLA")->get_model() == &copied);

    Model moved;
    moved = std::move(copied);
    CHECK(moved.id() == model_id);
    REQUIRE(moved.objects.size() == 1);
    CHECK(moved.objects.front()->get_model() == &moved);
    CHECK(copied.objects.empty());
    CHECK(copied.materials.empty());
}

TEST_CASE("Named object overload records source metadata and export filename", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object("part", "/tmp/input/source.stl", make_cube(4, 4, 4));
    REQUIRE(object->volumes.size() == 1);
    CHECK(object->name == "part");
    CHECK(object->input_file == "/tmp/input/source.stl");
    CHECK(object->volumes.front()->source.input_file == "/tmp/input/source.stl");
    CHECK(object->volumes.front()->source.object_idx == 0);
    CHECK(object->volumes.front()->source.volume_idx == 0);
    CHECK(object->config.extruder() == 1);
    CHECK(object->get_export_filename() == "/tmp/input/part");
    CHECK(model.propose_export_file_name_and_path().empty());
    object->add_instance();
    CHECK(model.propose_export_file_name_and_path() == "/tmp/input/part");
    CHECK(model.propose_export_file_name_and_path("3mf") == "/tmp/input/part.3mf");
}

TEST_CASE("Backup IDs remain stable and advance independently of object IDs", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *first = cube_object(model);
    ModelObject *second = cube_object(model);
    model.set_object_backup_id(*first, 1);
    CHECK(first->get_backup_id() == 1);
    CHECK(first->get_backup_id() == 1);
    model.set_object_backup_id(*second, 42);
    CHECK(second->get_backup_id() == 42);
    ModelObject *third = cube_object(model);
    CHECK(model.get_object_backup_id(*third) == 43);
    CHECK(model.get_object_backup_id(*third) == 43);
}

TEST_CASE("Model metadata and plate custom code have explicit current-plate selection", "[Model][ModelExtendedVariance]")
{
    Model model;
    model.SetDesigner("Ada", "user-7");
    REQUIRE(model.design_info != nullptr);
    CHECK(model.design_info->Designer == "Ada");
    CHECK(model.design_info->DesignerUserId == "user-7");
    model.plates_custom_gcodes[2].gcodes.push_back(CustomGCode::Item{1., CustomGCode::Custom, 1, "", "plate2"});
    model.curr_plate_index = 2;
    REQUIRE(model.get_curr_plate_custom_gcodes().gcodes.size() == 1);
    CHECK(model.get_curr_plate_custom_gcodes().gcodes.front().extra == "plate2");
    model.curr_plate_index = 9;
    CHECK(model.get_curr_plate_custom_gcodes().gcodes.empty());
}

TEST_CASE("Multipart detection distinguishes equal and different minimum Z", "[Model][ModelExtendedVariance]")
{
    Model model;
    cube_object(model, 10.);
    ModelObject *second = cube_object(model, 5.);
    second->instances.front()->set_offset(Vec3d(30, 0, 0));
    CHECK_FALSE(model.looks_like_multipart_object());
    second->instances.front()->set_offset(Vec3d(30, 0, 2));
    second->invalidate_bounding_box();
    CHECK(model.looks_like_multipart_object());
}

TEST_CASE("Centering can include modifiers while printable-only centering excludes them", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 10., false);
    object->add_volume(make_cube(4, 4, 4), ModelVolumeType::PARAMETER_MODIFIER, false)->set_offset(Vec3d(100, 0, 0));
    object->add_instance();
    object->center_around_origin(false);
    object->invalidate_bounding_box();
    CHECK_THAT(object->raw_mesh_bounding_box().center().x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(object->full_raw_mesh_bounding_box().center().x(), Catch::Matchers::WithinAbs(47., 1e-5));
    object->center_around_origin(true);
    CHECK_THAT(object->full_raw_mesh_bounding_box().center().x(), Catch::Matchers::WithinAbs(0., 1e-5));
}

TEST_CASE("Ensure-on-bed respects auto-drop and the negative-Z allowance", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->instances.front()->set_offset(Vec3d(0, 0, -5));
    object->invalidate_bounding_box();
    object->ensure_on_bed();
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(0., 1e-6));
    object->instances.front()->set_offset(Vec3d(0, 0, -5));
    object->invalidate_bounding_box();
    object->instances.front()->auto_drop = false;
    object->ensure_on_bed();
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-5., 1e-6));
    object->instances.front()->auto_drop = true;
    object->invalidate_bounding_box();
    object->ensure_on_bed(true);
    CHECK_THAT(object->min_z(), Catch::Matchers::WithinAbs(-5., 1e-6));
}

TEST_CASE("Deleting the last instance and clearing instances invalidate world geometry", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model);
    object->add_instance(Vec3d(30, 0, 0), Vec3d::Ones(), Vec3d::Zero(), Vec3d::Ones());
    REQUIRE(object->instances.size() == 2);
    CHECK(object->bounding_box_exact().size().x() > 30.);
    object->delete_last_instance();
    REQUIRE(object->instances.size() == 1);
    CHECK_THAT(object->bounding_box_exact().size().x(), Catch::Matchers::WithinAbs(10., 1e-5));
    object->clear_instances();
    CHECK(object->instances.empty());
    CHECK_THROWS(object->raw_bounding_box());
}

TEST_CASE("Volume sorting orders precise seam strength and semantic volume types", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(1, 1, 1), ModelVolumeType::SUPPORT_ENFORCER, false);
    object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    object->add_volume(make_cube(1, 1, 1), ModelVolumeType::PRECISE_SEAM_NEUTRAL, false);
    object->add_volume(make_cube(1, 1, 1), ModelVolumeType::PRECISE_SEAM_CENTER, false);
    object->add_volume(make_cube(1, 1, 1), ModelVolumeType::PARAMETER_MODIFIER, false);
    object->sort_volumes(true);
    CHECK(object->volumes[0]->is_model_part());
    CHECK(object->volumes[1]->is_modifier());
    CHECK(object->volumes[2]->is_support_enforcer());
    CHECK(object->volumes[3]->is_precise_seam_strong());
    CHECK(object->volumes[4]->is_precise_seam_weak());
    object->sort_volumes(false);
    CHECK(object->volumes.front()->is_model_part());
}

TEST_CASE("Deleting one of two volumes folds its transform into the remaining instance", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false)->set_offset(Vec3d(20, 0, 0));
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false)->set_offset(Vec3d(5, 0, 0));
    ModelInstance *instance = object->add_instance(Vec3d(7, 0, 0), Vec3d::Ones(), Vec3d::Zero(), Vec3d::Ones());
    const BoundingBoxf3 before_remaining = object->volumes[1]->mesh().transformed_bounding_box(instance->get_matrix() * object->volumes[1]->get_matrix());
    object->delete_volume(0);
    REQUIRE(object->volumes.size() == 1);
    check_vec3d(object->volumes.front()->get_offset(), Vec3d::Zero());
    check_box(object->instance_bounding_box(*instance), before_remaining.min.cast<double>(), before_remaining.max.cast<double>());
}

TEST_CASE("Raw printable mesh excludes modifiers and reverses mirrored winding", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *part = object->add_volume(make_cube(2, 2, 2), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(20, 20, 20), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_offset(Vec3d(100, 0, 0));
    const indexed_triangle_set before = object->raw_indexed_triangle_set();
    REQUIRE(before.indices.size() == part->mesh().its.indices.size());
    CHECK(before.indices.front() == part->mesh().its.indices.front());
    part->set_mirror(X, -1.);
    REQUIRE(part->is_left_handed());
    const indexed_triangle_set mirrored = object->raw_indexed_triangle_set();
    REQUIRE(mirrored.indices.size() == before.indices.size());
    CHECK(mirrored.indices.front() != before.indices.front());
    CHECK(mirrored.indices.front()[0] == before.indices.front()[1]);
    CHECK(mirrored.indices.front()[1] == before.indices.front()[0]);
    CHECK(mirrored.indices.front()[2] == before.indices.front()[2]);
}

TEST_CASE("Partial unit conversion clones only selected volumes and preserves metadata", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *inch = object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    inch->name = "inch";
    inch->source.input_file = "inch.stl";
    ModelVolume *untouched = object->add_volume(make_cube(2, 2, 2), ModelVolumeType::MODEL_PART, false);
    untouched->name = "millimeter";
    // convert_units returns detached objects; Model owns their cleanup.
    Model converted_model;
    ModelObjectPtrs &converted = converted_model.objects;
    object->convert_units(converted, ConversionType::CONV_FROM_INCH, {0});
    REQUIRE(converted.size() == 1);
    REQUIRE(converted.front()->volumes.size() == 2);
    check_vec3d(converted.front()->volumes[0]->mesh().bounding_box().size().cast<double>(), Vec3d(25.4, 25.4, 25.4), 1e-3);
    check_vec3d(converted.front()->volumes[1]->mesh().bounding_box().size().cast<double>(), Vec3d(2, 2, 2), 1e-5);
    CHECK(converted.front()->volumes[0]->source.is_converted_from_inches);
    CHECK_FALSE(converted.front()->volumes[1]->source.is_converted_from_inches);
    CHECK(converted.front()->volumes[0]->name == "inch");
    CHECK(converted.front()->volumes[0]->source.input_file == "inch.stl");
}

TEST_CASE("Volume conversion resets offset and records the source unit", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 1., false);
    ModelVolume *volume = object->volumes.front();
    volume->set_offset(Vec3d(5, 6, 7));
    volume->convert_from_imperial_units();
    check_vec3d(volume->mesh().bounding_box().size().cast<double>(), Vec3d(25.4, 25.4, 25.4), 1e-3);
    check_vec3d(volume->get_offset(), Vec3d::Zero());
    CHECK(volume->source.is_converted_from_inches);
}

TEST_CASE("Filament deletion remaps or removes stale explicit volume assignments", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 2.);
    ModelVolume *volume = object->volumes.front();
    volume->config.set("extruder", 6);
    volume->update_extruder_count_when_delete_filament(4, 1, 4, {0, 0, 0, 0, 0, 0});
    CHECK_FALSE(volume->config.has("extruder"));
    volume->config.set("extruder", 6);
    volume->update_extruder_count_when_delete_filament(4, 1, 4, {0, 0, 0, 0, 0, 1});
    CHECK(volume->config.extruder() == 6);
    volume->config.set("extruder", 2);
    volume->update_extruder_count(1);
    CHECK_FALSE(volume->config.has("extruder"));
}

TEST_CASE("Convex hull cache separates translation from shape changes", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 10., false);
    ModelVolume *volume = object->volumes.front();
    const Polygon &base = volume->get_convex_hull_2d(Transform3d::Identity());
    const auto base_box = base.bounding_box();
    Transform3d translated = Transform3d::Identity();
    translated.translation() = Vec3d(20, 30, 0);
    const Polygon &moved = volume->get_convex_hull_2d(translated);
    CHECK(moved.bounding_box().min.x() == base_box.min.x() + scaled(20.));
    CHECK(moved.bounding_box().min.y() == base_box.min.y() + scaled(30.));
    volume->set_scaling_factor(Vec3d(2, 1, 1));
    const Polygon &scaled_hull = volume->get_convex_hull_2d(Transform3d::Identity());
    CHECK(scaled_hull.bounding_box().size().x() == scaled(20.));
    CHECK(scaled_hull.bounding_box().size().y() == scaled(10.));
}

TEST_CASE("Only-one-part and connector cleanup distinguish model and auxiliary volumes", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 2., false);
    ModelVolume *part = object->volumes.front();
    CHECK(part->is_the_only_one_part());
    ModelVolume *modifier = object->add_volume(make_cube(1, 1, 1), ModelVolumeType::PARAMETER_MODIFIER, false);
    CHECK(part->is_the_only_one_part());
    ModelVolume *other = object->add_volume(make_cube(1, 1, 1), ModelVolumeType::MODEL_PART, false);
    CHECK_FALSE(part->is_the_only_one_part());
    other->cut_info.is_connector = true;
    other->cut_info.is_processed = true;
    object->delete_connectors();
    CHECK(object->volumes.size() == 2);
    object->invalidate_cut();
    CHECK_FALSE(modifier->cut_info.is_connector);
}

TEST_CASE("Material painting round-trips serialized triangle state and resets", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 2., false);
    ModelVolume *volume = object->volumes.front();
    volume->mmu_segmentation_facets.set_triangle_from_string(0, "8");
    CHECK(volume->mmu_segmentation_facets.get_triangle_as_string(0) == "8");
    CHECK_FALSE(volume->mmu_segmentation_facets.get_data().bitstream.empty());
    CHECK(volume->is_mm_painted());
    const auto saved = volume->save_painting();
    REQUIRE(saved.has_value());
    volume->reset_extra_facets();
    CHECK_FALSE(volume->is_any_painted());
    volume->restore_painting(saved);
    CHECK(volume->is_mm_painted());
    volume->reset_extra_facets();
    CHECK_FALSE(volume->save_painting().has_value());
}

TEST_CASE("OBJ face and vertex filament imports reject malformed cardinality and record painted slots", "[Model][ModelExtendedVariance]")
{
    Model face_model;
    ModelObject *face_object = cube_object(face_model, 2., false);
    const size_t faces = face_object->volumes.front()->mesh().its.indices.size();
    CHECK_FALSE(Model::obj_import_face_color_deal({}, 1, &face_model));
    CHECK_FALSE(Model::obj_import_face_color_deal(std::vector<unsigned char>(faces - 1, 2), 1, &face_model));
    CHECK(Model::obj_import_face_color_deal(std::vector<unsigned char>(faces, 2), 1, &face_model));
    CHECK(face_object->is_mm_painted());
    CHECK(face_object->volumes.front()->get_extruders_from_multi_material_painting() == std::vector<size_t>{1});

    Model vertex_model;
    ModelObject *vertex_object = cube_object(vertex_model, 2., false);
    const size_t vertices = vertex_object->volumes.front()->mesh().its.vertices.size();
    CHECK_FALSE(Model::obj_import_vertex_color_deal(std::vector<unsigned char>(vertices - 1, 2), 1, &vertex_model));
    CHECK(Model::obj_import_vertex_color_deal(std::vector<unsigned char>(vertices, 2), 1, &vertex_model));
    CHECK(vertex_object->is_mm_painted());
}

TEST_CASE("Print-volume classification distinguishes inside, outside, and mixed model parts", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false);
    object->add_volume(make_cube(10, 10, 10), ModelVolumeType::MODEL_PART, false)->set_offset(Vec3d(200, 0, 0));
    ModelInstance *instance = object->add_instance();
    CHECK(instance->calc_print_volume_state(rectangular_bed()) == ModelInstancePVS_Partly_Outside);
    object->volumes.back()->set_offset(Vec3d(200, 0, 0));
    instance->set_offset(Vec3d(200, 0, 0));
    CHECK(instance->calc_print_volume_state(rectangular_bed()) == ModelInstancePVS_Fully_Outside);
    instance->set_offset(Vec3d(10, 10, 0));
    object->volumes.back()->set_offset(Vec3d(0, 0, 0));
    CHECK(instance->calc_print_volume_state(rectangular_bed()) == ModelInstancePVS_Inside);
}

TEST_CASE("Instance Z queries follow scaling and translation of printable volumes", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 10.);
    ModelInstance *instance = object->instances.front();
    instance->set_scaling_factor(Vec3d(1, 1, 2));
    instance->set_offset(Vec3d(0, 0, 3));
    CHECK_THAT(object->get_instance_min_z(0), Catch::Matchers::WithinAbs(3., 1e-5));
    CHECK_THAT(object->get_instance_max_z(0), Catch::Matchers::WithinAbs(23., 1e-5));
}

TEST_CASE("Arrange polygon reports transformed footprint and material support assignments", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = cube_object(model, 10.);
    object->volumes.front()->config.set("extruder", 2);
    DynamicPrintConfig global;
    global.set_key_value("enable_support", new ConfigOptionBool(true));
    global.set_key_value("support_filament", new ConfigOptionInt(3));
    global.set_key_value("support_interface_filament", new ConfigOptionInt(2));
    arrangement::ArrangePolygon arranged;
    object->instances.front()->set_offset(Vec3d(12, 15, 4));
    object->instances.front()->set_rotation(Z, 0.25);
    object->instances.front()->get_arrange_polygon(&arranged, global);
    CHECK(arranged.poly.contour.size() == 4);
    CHECK(arranged.translation == Vec2crd(scaled(12.), scaled(15.)));
    CHECK_THAT(arranged.rotation, Catch::Matchers::WithinAbs(0.25, 1e-9));
    const std::set<int> extruders(arranged.extrude_ids.begin(), arranged.extrude_ids.end());
    CHECK(extruders == std::set<int>{2, 3});
}

TEST_CASE("Object export statistics use printable volume determinants and names", "[Model][ModelExtendedVariance]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "named";
    object->input_file = "/tmp/source.stl";
    ModelVolume *part = object->add_volume(make_cube(2, 2, 2), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(4, 4, 4), ModelVolumeType::PARAMETER_MODIFIER, false);
    modifier->set_offset(Vec3d(100, 0, 0));
    object->add_instance();
    CHECK(object->get_export_filename() == "/tmp/named");
    CHECK_THAT(object->get_object_stl_stats().volume, Catch::Matchers::WithinAbs(8., 1e-4));
    part->set_scaling_factor(Vec3d(2, 3, 4));
    CHECK_THAT(object->get_object_stl_stats().volume, Catch::Matchers::WithinAbs(192., 1e-3));
    CHECK(object->get_object_stl_stats().number_of_parts == 1);
    CHECK(object->has_solid_mesh());
    CHECK(object->facets_count() == 12);
}

TEST_CASE("ModelObject mirror reflects each axis and restores cached bounds", "[Model][ModelMirror]")
{
    const auto [axis, reflected_min, reflected_max] = GENERATE(table<Axis, Vec3d, Vec3d>({
        { X, Vec3d(-4., 0., 0.), Vec3d(0., 6., 8.) },
        { Y, Vec3d(0., -6., 0.), Vec3d(4., 0., 8.) },
        { Z, Vec3d(0., 0., -8.), Vec3d(4., 6., 0.) },
    }));

    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->add_volume(make_cube(4.f, 6.f, 8.f), ModelVolumeType::MODEL_PART, false);
    object->add_volume(make_cube(2.f, 3.f, 5.f), ModelVolumeType::MODEL_PART, false);
    REQUIRE(object->volumes.size() == 2);
    object->add_instance();

    const BoundingBoxf3 before_approx = object->bounding_box_approx();
    const BoundingBoxf3 before_exact = object->bounding_box_exact();
    check_box(before_approx, Vec3d::Zero(), Vec3d(4., 6., 8.));
    check_box(before_exact, Vec3d::Zero(), Vec3d(4., 6., 8.));

    object->mirror(axis);

    check_box(object->bounding_box_approx(), reflected_min, reflected_max);
    check_box(object->bounding_box_exact(), reflected_min, reflected_max);
    for (const ModelVolume *volume : object->volumes)
        CHECK(volume->is_left_handed());

    object->mirror(axis);

    check_box(object->bounding_box_approx(), Vec3d::Zero(), Vec3d(4., 6., 8.));
    check_box(object->bounding_box_exact(), Vec3d::Zero(), Vec3d(4., 6., 8.));
    for (const ModelVolume *volume : object->volumes)
        CHECK_FALSE(volume->is_left_handed());
}

} // namespace model_extended_contracts

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace model_import_contracts {
namespace {

void write_text(const std::string& path, const std::string& contents)
{
    boost::nowide::ofstream out(path);
    REQUIRE(out.good());
    out << contents;
    REQUIRE(out.good());
}

void write_binary(const std::string& path, const std::vector<unsigned char>& bytes)
{
    boost::nowide::ofstream out(path, std::ios::binary);
    REQUIRE(out.good());
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    REQUIRE(out.good());
}

std::string plain_obj()
{
    return "o triangle\n"
           "v 0 0 0\n"
           "v 12 0 0\n"
           "v 0 8 0\n"
           "v 0 0 5\n"
           "f 1 2 3\n"
           "f 1 4 2\n"
           "f 1 3 4\n"
           "f 2 4 3\n";
}

std::string colored_obj()
{
    return "o colored\n"
           "v 0 0 0 1 0 0\n"
           "v 10 0 0 0.5 0.5 0\n"
           "v 0 10 0 0 0 0.25\n"
           "v 0 0 10 0.25 1 0.75\n"
           "f 1 2 3\n"
           "f 1 4 2\n"
           "f 1 3 4\n"
           "f 2 4 3\n";
}

std::string material_obj_with_uv_without_texture(const std::string& mtl_name)
{
    return "mtllib " + mtl_name +
           "\n"
           "usemtl painted\n"
           "v 0 0 0\n"
           "v 10 0 0\n"
           "v 0 10 0\n"
           "vt 0.1 0.2\n"
           "vt 0.4 0.5\n"
           "vt 0.8 0.9\n"
           "f 1/1 2/2 3/3\n";
}

std::string textured_obj_without_uv(const std::string& mtl_name)
{
    return "mtllib " + mtl_name +
           "\n"
           "usemtl painted\n"
           "v 0 0 0\n"
           "v 10 0 0\n"
           "v 0 10 0\n"
           "f 1 2 3\n";
}

std::string material_obj(const std::string& mtl_name)
{
    return "mtllib " + mtl_name +
           "\n"
           "usemtl red\n"
           "v 0 0 0\n"
           "v 10 0 0\n"
           "v 0 10 0\n"
           "v 0 0 10\n"
           "f 1 2 3\n"
           "usemtl blue\n"
           "f 1 4 2\n"
           "f 1 3 4\n"
           "f 2 4 3\n";
}

std::string textured_obj(const std::string& mtl_name)
{
    return "mtllib " + mtl_name +
           "\n"
           "usemtl painted\n"
           "v 0 0 0\n"
           "v 10 0 0\n"
           "v 0 10 0\n"
           "vt 0.1 0.2\n"
           "vt 0.4 0.5\n"
           "vt 0.8 0.9\n"
           "f 1/1 2/2 3/3\n";
}

void write_cube_binary_stl(const std::string& path, size_t header_size = 80)
{
    const indexed_triangle_set cube = its_make_cube(10., 20., 30.);
    std::vector<unsigned char> bytes(header_size, 0);
    const auto append = [&bytes](const auto& value) {
        const auto* begin = reinterpret_cast<const unsigned char*>(&value);
        bytes.insert(bytes.end(), begin, begin + sizeof(value));
    };
    append(static_cast<uint32_t>(cube.indices.size()));
    const stl_normal normal = stl_normal::Zero();
    for (const Vec3i32& face : cube.indices) {
        append(normal);
        for (int i = 0; i < 3; ++i)
            append(cube.vertices[face[i]]);
        append(static_cast<uint16_t>(0));
    }
    write_binary(path, bytes);
}

void check_box(const ModelObject& object, const Vec3d& expected_size)
{
    const Vec3d size = object.raw_mesh_bounding_box().size();
    CHECK_THAT(size.x(), WithinAbs(expected_size.x(), 1e-5));
    CHECK_THAT(size.y(), WithinAbs(expected_size.y(), 1e-5));
    CHECK_THAT(size.z(), WithinAbs(expected_size.z(), 1e-5));
}

void write_valid_texture(const std::string& path)
{
    const std::vector<uint8_t> pixels = {
        255, 0, 0, 0, 255, 0, 0, 0, 255, 255, 255, 0,
    };
    REQUIRE(png::write_rgb_to_file(path, 2, 2, pixels));
}

DynamicPrintConfig archive_config() { return DynamicPrintConfig::full_print_config(); }

void store_archive(const std::string& path)
{
    Model model;
    ModelObject* object = model.add_object();
    object->name        = "archive-cube";
    object->add_volume(make_cube(10., 20., 30.));
    object->add_instance();

    ScopedTemporaryDir backup("orca_model_import_store");
    model.set_backup_path(backup.string());
    DynamicPrintConfig config = archive_config();
    PlateData plate;
    plate.plate_index = 0;
    plate.plate_name  = "Import fixture";

    StoreParams params;
    params.path   = path;
    params.model  = &model;
    params.config = &config;
    params.plate_data_list.push_back(&plate);
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));
}

Model read_archive(const std::string& path, En3mfType& type, LoadStrategy options = LoadStrategy::LoadModel | LoadStrategy::LoadConfig)
{
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    Semver version;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset* preset : presets)
            delete preset;
    });
    return Model::read_from_archive(path, &config, &substitutions, type, options, &plates, &presets, &version);
}

} // namespace

TEST_CASE("Model import reads binary STL geometry and records its source", "[Model][ModelImport][STL]")
{
    ScopedTemporaryFile stl(".stl");
    write_cube_binary_stl(stl.string());
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(stl.string()));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
    CHECK(model.objects.front()->input_file == stl.string());
    REQUIRE(model.objects.front()->instances.size() == 1);
    CHECK_THAT(model.objects.front()->raw_mesh().volume(), WithinRel(6000., 1e-5));
}

TEST_CASE("Model import can suppress STL default instances", "[Model][ModelImport][STL]")
{
    ScopedTemporaryFile stl(".stl");
    write_cube_binary_stl(stl.string());
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(stl.string(), nullptr, nullptr, LoadStrategy::Default));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->instances.empty());
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
}

TEST_CASE("OLTP import uses the STL path while preserving geometry", "[Model][ModelImport][OLTP]")
{
    ScopedTemporaryFile oltp(".oltp");
    write_cube_binary_stl(oltp.string(), 256);
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(oltp.string(), nullptr, nullptr, LoadStrategy::AddDefaultInstances));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
}

TEST_CASE("STL progress reports bounded progress and allows completion", "[Model][ModelImport][STL]")
{
    ScopedTemporaryFile stl(".stl");
    write_cube_binary_stl(stl.string());
    int callbacks                = 0;
    int last_current             = -1;
    int last_total               = -1;
    bool invalid_progress        = false;
    ImportstlProgressFn progress = [&](int current, int total, bool& cancel, std::string& id, std::string& stage) {
        cancel = false;
        ++callbacks;
        last_current = current;
        last_total   = total;
        if (current < 0 || total <= 0 || current > total || !id.empty() || !stage.empty())
            invalid_progress = true;
    };
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(stl.string(), nullptr, nullptr, LoadStrategy::Default, nullptr, nullptr, nullptr, nullptr,
                                                  nullptr, progress));
    REQUIRE(model.objects.size() == 1);
    CHECK(callbacks > 0);
    CHECK(last_total > 0);
    CHECK(last_current >= 0);
    CHECK_FALSE(invalid_progress);
}

TEST_CASE("STL cancellation is surfaced as an import failure", "[Model][ModelImport][STL]")
{
    ScopedTemporaryFile stl(".stl");
    write_cube_binary_stl(stl.string());
    bool callback_called         = false;
    ImportstlProgressFn progress = [&](int, int, bool& cancel, std::string&, std::string&) {
        callback_called = true;
        cancel          = true;
    };
    CHECK_THROWS_AS(Model::read_from_file(stl.string(), nullptr, nullptr, LoadStrategy::Default, nullptr, nullptr, nullptr, nullptr,
                                          nullptr, progress),
                    Slic3r::RuntimeError);
    CHECK(callback_called);
}

TEST_CASE("Model import accepts the repository ASCII STL fixture variants", "[Model][ModelImport][STL]")
{
    const std::array<const char*, 3> names = {"ASCII/20mmbox-LF.stl", "ASCII/20mmbox-CRLF.stl", "ASCII/20mmbox-nonstandard.stl"};
    for (const char* name : names) {
        Model model;
        REQUIRE_NOTHROW(model = Model::read_from_file(std::string(TEST_DATA_DIR) + "/test_stl/" + name));
        REQUIRE(model.objects.size() == 1);
        check_box(*model.objects.front(), Vec3d(20., 20., 20.));
    }
}

TEST_CASE("Model import rejects malformed STL and unknown extensions", "[Model][ModelImport][Errors]")
{
    ScopedTemporaryFile malformed(".stl");
    write_text(malformed.string(), "solid incomplete\n facet normal 0 0 1\n");
    CHECK_THROWS_AS(Model::read_from_file(malformed.string()), Slic3r::RuntimeError);

    ScopedTemporaryFile unknown(".model");
    write_text(unknown.string(), "geometry");
    CHECK_THROWS_AS(Model::read_from_file(unknown.string()), Slic3r::RuntimeError);
}

TEST_CASE("Model import preserves plain OBJ bounds and topology", "[Model][ModelImport][OBJ]")
{
    ScopedTemporaryFile obj(".obj");
    write_text(obj.string(), plain_obj());
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string(), nullptr, nullptr, LoadStrategy::Default));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    CHECK(model.objects.front()->raw_mesh().facets_count() == 4);
    check_box(*model.objects.front(), Vec3d(12., 8., 5.));
    CHECK(model.objects.front()->instances.empty());
}

TEST_CASE("Model import turns OBJ vertex colors into per-face colors", "[Model][ModelImport][OBJ][Colors]")
{
    ScopedTemporaryFile obj(".obj");
    write_text(obj.string(), colored_obj());
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string()));
    REQUIRE(model.texture_mesh != nullptr);
    REQUIRE(model.texture_mesh->precomputed_face_colors.size() == 4);
    REQUIRE(model.texture_mesh->precomputed_vertex_colors.size() == 4);
    CHECK(model.texture_mesh->precomputed_face_colors[0] == std::array<std::size_t, 3>{127, 42, 21});
    CHECK(model.texture_mesh->precomputed_face_colors[1] == std::array<std::size_t, 3>{148, 127, 63});
    CHECK(model.texture_mesh->precomputed_face_colors[2] == std::array<std::size_t, 3>{106, 85, 85});
    CHECK(model.texture_mesh->precomputed_face_colors[3] == std::array<std::size_t, 3>{63, 127, 85});
    const std::array<std::array<float, 4>, 4> expected_colors = {
        {{1.f, 0.f, 0.f, 1.f}, {.5f, .5f, 0.f, 1.f}, {0.f, 0.f, .25f, 1.f}, {.25f, 1.f, .75f, 1.f}}};
    for (size_t vertex = 0; vertex < expected_colors.size(); ++vertex)
        for (size_t channel = 0; channel < expected_colors[vertex].size(); ++channel)
            CHECK_THAT(model.texture_mesh->precomputed_vertex_colors[vertex][channel], WithinAbs(expected_colors[vertex][channel], 1e-6));
    const std::array<std::array<float, 3>, 4> expected_vertices = {{{0.f, 0.f, 0.f}, {10.f, 0.f, 0.f}, {0.f, 10.f, 0.f}, {0.f, 0.f, 10.f}}};
    REQUIRE(model.texture_mesh->vertices.size() == expected_vertices.size());
    for (size_t vertex = 0; vertex < expected_vertices.size(); ++vertex)
        for (size_t axis = 0; axis < expected_vertices[vertex].size(); ++axis)
            CHECK_THAT(model.texture_mesh->vertices[vertex][axis], WithinAbs(expected_vertices[vertex][axis] - 5.f, 1e-6));
    // Model volumes center their mesh; OBJ import repairs inward winding.
    CHECK(model.texture_mesh->indices == std::vector<std::array<int, 3>>{{0, 2, 1}, {0, 1, 3}, {0, 3, 2}, {1, 2, 3}});
    REQUIRE(model.objects.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 10., 10.));
    CHECK_THAT(model.objects.front()->raw_mesh().volume(), WithinRel(1000. / 6., 1e-5));
}

TEST_CASE("Model import maps OBJ material colors to face colors", "[Model][ModelImport][OBJ][Colors]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    write_text(mtl.string(), "newmtl red\nKd 1 0 0\nnewmtl blue\nKd 0 0 1\n");
    write_text(obj.string(), material_obj(mtl.path().filename().string()));
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string()));
    REQUIRE(model.texture_mesh != nullptr);
    REQUIRE(model.texture_mesh->precomputed_face_colors.size() == 4);
    CHECK(model.texture_mesh->precomputed_face_colors[0] == std::array<std::size_t, 3>{255, 0, 0});
    CHECK(model.texture_mesh->precomputed_face_colors[1] == std::array<std::size_t, 3>{0, 0, 255});
    CHECK(model.texture_mesh->precomputed_face_colors[2] == std::array<std::size_t, 3>{0, 0, 255});
    CHECK(model.texture_mesh->precomputed_face_colors[3] == std::array<std::size_t, 3>{0, 0, 255});
}

TEST_CASE("Model import retains OBJ texture bytes and flipped UVs", "[Model][ModelImport][OBJ][Texture]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    ScopedTemporaryFile texture(".png");
    write_valid_texture(texture.string());
    write_text(mtl.string(), "newmtl painted\nKd .2 .4 .6\nmap_Kd " + texture.path().filename().string() + "\n");
    write_text(obj.string(), textured_obj(mtl.path().filename().string()));
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string()));
    REQUIRE(model.texture_mesh != nullptr);
    REQUIRE(model.texture_mesh->textures.size() == 1);
    const auto& encoded = model.texture_mesh->textures[0].data;
    REQUIRE(png::is_png(png::ReadBuf{encoded.data(), encoded.size()}));
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{encoded.data(), encoded.size()}, decoded));
    CHECK(decoded.cols == 2);
    CHECK(decoded.rows == 2);
    CHECK(decoded.bytes_per_pixel == 3);
    // The colored PNG decoder returns rows bottom-up.
    CHECK(decoded.buf == std::vector<uint8_t>{0, 0, 255, 255, 255, 0, 255, 0, 0, 0, 255, 0});
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->input_file == obj.string());
    CHECK(model.objects.front()->raw_mesh().facets_count() == 1);
    check_box(*model.objects.front(), Vec3d(10., 10., 0.));
    REQUIRE(model.texture_mesh->uv_coords.size() == 3);
    CHECK_THAT(model.texture_mesh->uv_coords[0][0], WithinAbs(.1, 1e-6));
    CHECK_THAT(model.texture_mesh->uv_coords[0][1], WithinAbs(.8, 1e-6));
    CHECK_THAT(model.texture_mesh->uv_coords[1][0], WithinAbs(.4, 1e-6));
    CHECK_THAT(model.texture_mesh->uv_coords[1][1], WithinAbs(.5, 1e-6));
    CHECK_THAT(model.texture_mesh->uv_coords[2][0], WithinAbs(.8, 1e-6));
    CHECK_THAT(model.texture_mesh->uv_coords[2][1], WithinAbs(.1, 1e-6));
}

TEST_CASE("OBJ UVs without a texture retain material face colors", "[Model][ModelImport][OBJ][Texture]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    write_text(mtl.string(), "newmtl painted\nKd .2 .4 .6\n");
    write_text(obj.string(), material_obj_with_uv_without_texture(mtl.path().filename().string()));
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string()));
    REQUIRE(model.texture_mesh != nullptr);
    REQUIRE(model.texture_mesh->precomputed_face_colors.size() == 1);
    CHECK(model.texture_mesh->precomputed_face_colors[0] == std::array<std::size_t, 3>{51, 102, 153});
    CHECK(model.texture_mesh->textures.empty());
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->raw_mesh().facets_count() == 1);
}

TEST_CASE("OBJ texture material without UVs does not publish a textured mesh", "[Model][ModelImport][OBJ][Texture]")
{
    ScopedTemporaryFile obj(".obj");
    ScopedTemporaryFile mtl(".mtl");
    ScopedTemporaryFile texture(".png");
    write_valid_texture(texture.string());
    write_text(mtl.string(), "newmtl painted\nKd .2 .4 .6\nmap_Kd " + texture.path().filename().string() + "\n");
    write_text(obj.string(), textured_obj_without_uv(mtl.path().filename().string()));
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(obj.string()));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.texture_mesh == nullptr);
    CHECK(model.objects.front()->raw_mesh().facets_count() == 1);
}

TEST_CASE("Model import reports malformed OBJ files", "[Model][ModelImport][OBJ][Errors]")
{
    ScopedTemporaryFile obj(".obj");
    write_text(obj.string(), "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 99\n");
    CHECK_THROWS_AS(Model::read_from_file(obj.string()), Slic3r::RuntimeError);
}

TEST_CASE("Model import reads the supported AMF extension and rejects malformed XML", "[Model][ModelImport][AMF]")
{
    ScopedTemporaryFile amf(".amf");
    write_text(amf.string(), "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                             "<amf unit=\"millimeter\"><object id=\"0\"><mesh><vertices>"
                             "<vertex><coordinates><x>0</x><y>0</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>10</x><y>0</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>0</x><y>10</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>0</x><y>0</y><z>10</z></coordinates></vertex>"
                             "</vertices><volume><triangle><v1>0</v1><v2>2</v2><v3>1</v3></triangle>"
                             "<triangle><v1>0</v1><v2>1</v2><v3>3</v3></triangle>"
                             "<triangle><v1>0</v1><v2>3</v2><v3>2</v3></triangle>"
                             "<triangle><v1>1</v1><v2>2</v2><v3>3</v3></triangle></volume></mesh></object></amf>");
    bool is_inches = true;
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(amf.string(), nullptr, nullptr, LoadStrategy::Default, nullptr, nullptr, &is_inches));
    REQUIRE(model.objects.size() == 1);
    CHECK_FALSE(is_inches);
    CHECK_THAT(model.objects.front()->raw_mesh().volume(), WithinRel(1000. / 6., 1e-5));

    ScopedTemporaryFile bad(".amf");
    write_text(bad.string(), "<amf><object>");
    CHECK_THROWS_AS(Model::read_from_file(bad.string()), Slic3r::RuntimeError);
}

TEST_CASE("AMF inch units are reported while preserving source coordinates", "[Model][ModelImport][AMF]")
{
    ScopedTemporaryFile amf(".amf");
    write_text(amf.string(), "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                             "<amf unit=\"inch\"><object id=\"0\"><mesh><vertices>"
                             "<vertex><coordinates><x>0</x><y>0</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>1</x><y>0</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>0</x><y>1</y><z>0</z></coordinates></vertex>"
                             "<vertex><coordinates><x>0</x><y>0</y><z>1</z></coordinates></vertex>"
                             "</vertices><volume>"
                             "<triangle><v1>0</v1><v2>2</v2><v3>1</v3></triangle>"
                             "<triangle><v1>0</v1><v2>1</v2><v3>3</v3></triangle>"
                             "<triangle><v1>0</v1><v2>3</v2><v3>2</v3></triangle>"
                             "<triangle><v1>1</v1><v2>2</v2><v3>3</v3></triangle>"
                             "</volume></mesh></object></amf>");
    bool is_inches = false;
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(amf.string(), nullptr, nullptr, LoadStrategy::Default, nullptr, nullptr, &is_inches));
    REQUIRE(model.objects.size() == 1);
    CHECK(is_inches);
    const Vec3d size = model.objects.front()->raw_mesh_bounding_box().size();
    CHECK_THAT(size.x(), WithinAbs(1., 1e-5));
    CHECK_THAT(size.y(), WithinAbs(1., 1e-5));
    CHECK_THAT(size.z(), WithinAbs(1., 1e-5));
}

TEST_CASE("Model read_from_file imports a generated BBS archive with model options", "[Model][ModelImport][Archive]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_read");
    ScopedTemporaryFile archive(".3mf");
    store_archive(archive.string());
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    bool is_bbl = false;
    Semver version;
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_file(archive.string(), &config, &substitutions,
                                                  LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances,
                                                  nullptr, nullptr, &is_bbl, &version));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
    CHECK(model.objects.front()->input_file == archive.string());
}

TEST_CASE("Model read_from_file preserves an archive's explicit instance without adding another", "[Model][ModelImport][Archive]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_no_instances");
    ScopedTemporaryFile archive(".3mf");
    store_archive(archive.string());
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    Model model;
    REQUIRE_NOTHROW(
        model = Model::read_from_file(archive.string(), &config, &substitutions, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    REQUIRE(model.objects.size() == 1);
    CHECK(model.objects.front()->instances.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
}

TEST_CASE("Model read_from_archive identifies the generated archive origin and restores geometry", "[Model][ModelImport][Archive]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_archive");
    ScopedTemporaryFile archive(".3mf");
    store_archive(archive.string());
    En3mfType type = En3mfType::From_Other;
    Model model;
    REQUIRE_NOTHROW(model = read_archive(archive.string(), type,
                                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    check_box(*model.objects.front(), Vec3d(10., 20., 30.));
    CHECK(type == En3mfType::From_Orca);
    CHECK(model.objects.front()->input_file == archive.string());
}

TEST_CASE("Model read_from_archive reports progress stages after loading", "[Model][ModelImport][Archive]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_progress");
    ScopedTemporaryFile archive(".3mf");
    store_archive(archive.string());
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    En3mfType type = En3mfType::From_Other;
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    Semver version;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset* preset : presets)
            delete preset;
    });
    std::vector<std::tuple<int, int, int>> progress_records;
    Import3mfProgressFn progress = [&](int stage, int current, int total, bool& cancel) {
        progress_records.emplace_back(stage, current, total);
        cancel = false;
    };
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_archive(archive.string(), &config, &substitutions, type,
                                                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig, &plates, &presets, &version,
                                                     progress));
    REQUIRE(model.objects.size() == 1);
    REQUIRE_FALSE(progress_records.empty());
    CHECK(std::any_of(progress_records.begin(), progress_records.end(),
                      [](const auto& record) { return std::get<0>(record) == IMPORT_STAGE_UPDATE_GCODE; }));
    CHECK(std::any_of(progress_records.begin(), progress_records.end(),
                      [](const auto& record) { return std::get<0>(record) == IMPORT_STAGE_CHECK_MODE_GCODE; }));
    for (const auto& [stage, current, total] : progress_records) {
        CHECK(stage >= 0);
        CHECK(current >= 0);
        CHECK(total >= 0);
        CHECK(current <= total);
    }
}

TEST_CASE("Model read_from_archive cancellation is surfaced after archive load", "[Model][ModelImport][Archive]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_read");
    const std::string archive = std::string(TEST_DATA_DIR) + "/test_3mf/Geräte/Büchse.3mf";
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    En3mfType type = En3mfType::From_Other;
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    Semver version;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset* preset : presets)
            delete preset;
    });
    bool callback_called         = false;
    Import3mfProgressFn progress = [&](int stage, int, int, bool& cancel) {
        cancel = false;
        if (stage == IMPORT_STAGE_ADD_INSTANCE) {
            callback_called = true;
            cancel          = true;
        }
    };
    CHECK_THROWS_AS(Model::read_from_archive(archive, &config, &substitutions, type,
                                             LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::AddDefaultInstances,
                                             &plates, &presets, &version, progress),
                    Slic3r::RuntimeError);
    CHECK(callback_called);
}

TEST_CASE("Model read_from_archive rejects malformed archives and unsupported suffixes", "[Model][ModelImport][Archive][Errors]")
{
    ScopedSlic3rTemporaryDir backup("orca_model_import_errors");
    ScopedTemporaryFile malformed(".3mf");
    write_text(malformed.string(), "not a zip archive");
    DynamicPrintConfig config = archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    En3mfType type = En3mfType::From_Other;
    CHECK_THROWS_AS(Model::read_from_archive(malformed.string(), &config, &substitutions, type), Slic3r::RuntimeError);

    ScopedTemporaryFile unknown(".obj");
    write_text(unknown.string(), plain_obj());
    CHECK_THROWS_AS(Model::read_from_archive(unknown.string(), &config, &substitutions, type), Slic3r::RuntimeError);

    ScopedTemporaryFile amf(".zip.amf");
    write_text(amf.string(), "not an amf archive");
    CHECK_THROWS_AS(Model::read_from_archive(amf.string(), &config, &substitutions, type), Slic3r::RuntimeError);
}

} // namespace model_import_contracts

namespace bbs_project_lifecycle_contracts
{
using namespace Slic3r;

TEST_CASE("Model lifecycle transfers project metadata and the backup path",
          "[Model][ModelLifecycle][Bbs3mfLifecycle]")
{
    ScopedTemporaryDir backup{"orca_model_lifecycle_transfer"};
    Model source;
    source.set_backup_path(backup.string());
    source.design_info = std::make_shared<ModelDesignInfo>();
    source.design_info->Designer = "source-designer";
    source.model_info = std::make_shared<ModelInfo>();
    source.model_info->model_name = "source-model";
    source.profile_info = std::make_shared<ModelProfileInfo>();
    source.profile_info->ProfileTile = "source-profile";
    source.mk_name = "maker";
    source.md_name = {"key"};
    source.md_value = {"value"};

    Model destination;
    destination.load_from(source);
    CHECK(destination.get_backup_path() == backup.string());
    REQUIRE(destination.design_info != nullptr);
    CHECK(destination.design_info->Designer == "source-designer");
    REQUIRE(destination.model_info != nullptr);
    CHECK(destination.model_info->model_name == "source-model");
    REQUIRE(destination.profile_info != nullptr);
    CHECK(destination.profile_info->ProfileTile == "source-profile");
    CHECK(destination.mk_name == "maker");
    CHECK(destination.md_name == std::vector<std::string>{"key"});
    CHECK(destination.md_value == std::vector<std::string>{"value"});
    CHECK(source.design_info == nullptr);
    CHECK(source.model_info == nullptr);
    CHECK(source.profile_info == nullptr);

    const std::string auxiliary = destination.get_auxiliary_file_temp_path();
    CHECK(boost::filesystem::is_directory(auxiliary));
    destination.set_need_backup();
    CHECK(destination.is_need_backup());
    destination.remove_backup_path_if_exist();
    CHECK_FALSE(boost::filesystem::exists(backup.path()));
}

TEST_CASE("Model object lifecycle records source metadata, material copies, and transforms",
          "[Model][ModelLifecycle]")
{
    Model model;
    ModelObject *object = model.add_object("named-part", "source.stl", make_cube(10., 10., 10.));
    REQUIRE(object != nullptr);
    REQUIRE(object->volumes.size() == 1);
    object->add_instance();
    CHECK(object->name == "named-part");
    CHECK(object->input_file == "source.stl");
    CHECK(object->volumes.front()->name == "named-part");
    CHECK(object->volumes.front()->source.input_file == "source.stl");

    ModelMaterial *base = model.add_material("base");
    base->attributes["color"] = "#ff0000";
    ModelMaterial *copy = model.add_material("copy", *base);
    REQUIRE(copy != nullptr);
    CHECK(copy->attributes.at("color") == "#ff0000");

    CHECK_THAT(model.bounding_box_approx().max.z(), Catch::Matchers::WithinAbs(10., 1e-6));
    CHECK_THAT(model.max_z(), Catch::Matchers::WithinAbs(10., 1e-6));
    ModelVolume *volume = object->volumes.front();
    volume->set_offset(Vec3d(1., 2., 3.));
    volume->set_rotation(Z, Geometry::deg2rad(15.));
    volume->set_scaling_factor(Vec3d(1.5, 1.25, 1.));
    const Vec3d offset = volume->get_offset();
    CHECK_THAT(offset.x(), Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(offset.y(), Catch::Matchers::WithinAbs(2., 1e-9));
    CHECK_THAT(offset.z(), Catch::Matchers::WithinAbs(3., 1e-9));
    CHECK_THAT(volume->get_rotation(Z), Catch::Matchers::WithinAbs(Geometry::deg2rad(15.), 1e-9));
    CHECK_THAT(volume->get_scaling_factor(X), Catch::Matchers::WithinAbs(1.5, 1e-9));
    const Transform3d before_mirror = volume->get_transformation().get_matrix();
    volume->set_mirror(X, -1.);
    const Transform3d after_mirror = volume->get_transformation().get_matrix();
    CHECK(after_mirror.linear().col(0).isApprox(-before_mirror.linear().col(0), 1e-9));
    CHECK(after_mirror.linear().col(1).isApprox(before_mirror.linear().col(1), 1e-9));
    CHECK(after_mirror.linear().col(2).isApprox(before_mirror.linear().col(2), 1e-9));
    CHECK(after_mirror.translation().isApprox(before_mirror.translation(), 1e-9));
    CHECK(volume->is_left_handed());

    object->scale(2.);
    object->mirror(Y);
    CHECK_FALSE(object->is_multiparts());
    model.delete_material("copy");
    CHECK(model.get_material("copy") == nullptr);
    model.clear_materials();
    CHECK(model.materials.empty());
}

} // namespace bbs_project_lifecycle_contracts

TEST_CASE("ModelObject boolean replacement preserves analytic solid geometry", "[Model][ModelBoolean]")
{
    constexpr double body_edge = 10.;
    constexpr double cutter_edge = 4.;
    constexpr double cutter_height = 12.;
    constexpr double overlap_height = 10.; // The cutter spans the body's full Z interval.
    const std::string operation = GENERATE("A_NOT_B", "UNION");
    CAPTURE(operation);

    Model model;
    ModelObject *body = model.add_object();
    body->name = "body";
    body->add_volume(make_cube(float(body_edge), float(body_edge), float(body_edge)), ModelVolumeType::MODEL_PART, false);
    body->add_instance();

    ModelObject *cutter = model.add_object();
    cutter->add_volume(make_cube(float(cutter_edge), float(cutter_edge), float(cutter_height)), ModelVolumeType::MODEL_PART, false);
    cutter->add_instance();
    cutter->instances.front()->set_offset(Vec3d(3., 3., -1.));

    REQUIRE(body->make_boolean(cutter, operation));
    REQUIRE(body->volumes.size() == 1);
    REQUIRE(body->volumes.front()->is_model_part());

    const double body_volume = body_edge * body_edge * body_edge;
    const double cutter_volume = cutter_edge * cutter_edge * cutter_height;
    const double overlap_volume = cutter_edge * cutter_edge * overlap_height;
    const double expected_volume = operation == "A_NOT_B" ? body_volume - overlap_volume
                                                            : body_volume + cutter_volume - overlap_volume;
    const TriangleMesh result_mesh = body->volumes.front()->mesh();
    REQUIRE_FALSE(result_mesh.its.vertices.empty());
    REQUIRE_FALSE(result_mesh.its.indices.empty());
    for (const Vec3f &vertex : result_mesh.its.vertices)
        REQUIRE(vertex.allFinite());
    for (const Vec3i32 &face : result_mesh.its.indices) {
        REQUIRE(face[0] >= 0);
        REQUIRE(face[1] >= 0);
        REQUIRE(face[2] >= 0);
        REQUIRE(static_cast<size_t>(face[0]) < result_mesh.its.vertices.size());
        REQUIRE(static_cast<size_t>(face[1]) < result_mesh.its.vertices.size());
        REQUIRE(static_cast<size_t>(face[2]) < result_mesh.its.vertices.size());
    }
    const double actual_volume = std::abs(static_cast<double>(its_volume(result_mesh.its)));
    CHECK_THAT(actual_volume, Catch::Matchers::WithinRel(expected_volume, 2e-3));

    const TriangleMesh world_mesh = body->raw_mesh();
    const BoundingBoxf3 bounds = world_mesh.bounding_box();
    CHECK(bounds.min.allFinite());
    CHECK(bounds.max.allFinite());
    CHECK(bounds.min.x() <= 0.001f);
    CHECK(bounds.min.y() <= 0.001f);
    CHECK(bounds.max.x() >= 9.999f);
    CHECK(bounds.max.y() >= 9.999f);
}

TEST_CASE("ModelObject boolean rejects a multi-volume source without mutation", "[Model][ModelBoolean]")
{
    Model model;
    ModelObject *body = model.add_object();
    body->add_volume(make_cube(10.f, 10.f, 10.f), ModelVolumeType::MODEL_PART, false);
    TriangleMesh second = make_cube(2.f, 2.f, 2.f);
    second.translate(Vec3f(20.f, 0.f, 0.f));
    body->add_volume(std::move(second), ModelVolumeType::MODEL_PART, false);
    body->add_instance();

    const ObjectID first_id = body->volumes[0]->id();
    const ObjectID second_id = body->volumes[1]->id();
    const indexed_triangle_set first_its = body->volumes[0]->mesh().its;
    const indexed_triangle_set second_its = body->volumes[1]->mesh().its;
    TriangleMesh before_world = body->raw_mesh();
    const double source_volume = std::abs(static_cast<double>(its_volume(before_world.its)));

    ModelObject *cutter = model.add_object();
    cutter->add_volume(make_cube(4.f, 4.f, 12.f), ModelVolumeType::MODEL_PART, false);
    cutter->add_instance();
    cutter->instances.front()->set_offset(Vec3d(3., 3., -1.));

    CHECK_FALSE(body->make_boolean(cutter, "A_NOT_B"));
    REQUIRE(body->volumes.size() == 2);
    CHECK(body->volumes[0]->id() == first_id);
    CHECK(body->volumes[1]->id() == second_id);
    const indexed_triangle_set &after_first = body->volumes[0]->mesh().its;
    const indexed_triangle_set &after_second = body->volumes[1]->mesh().its;
    REQUIRE(after_first.vertices.size() == first_its.vertices.size());
    REQUIRE(after_first.indices.size() == first_its.indices.size());
    REQUIRE(after_second.vertices.size() == second_its.vertices.size());
    REQUIRE(after_second.indices.size() == second_its.indices.size());
    for (size_t i = 0; i < first_its.vertices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            CHECK_THAT(static_cast<double>(after_first.vertices[i][axis]),
                       Catch::Matchers::WithinAbs(static_cast<double>(first_its.vertices[i][axis]), 0.));
    for (size_t i = 0; i < first_its.indices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            CHECK(after_first.indices[i][axis] == first_its.indices[i][axis]);
    for (size_t i = 0; i < second_its.vertices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            CHECK_THAT(static_cast<double>(after_second.vertices[i][axis]),
                       Catch::Matchers::WithinAbs(static_cast<double>(second_its.vertices[i][axis]), 0.));
    for (size_t i = 0; i < second_its.indices.size(); ++i)
        for (int axis = 0; axis < 3; ++axis)
            CHECK(after_second.indices[i][axis] == second_its.indices[i][axis]);
    TriangleMesh after_world = body->raw_mesh();
    CHECK_THAT(std::abs(static_cast<double>(its_volume(after_world.its))), Catch::Matchers::WithinRel(source_volume, 1e-6));
}

TEST_CASE("Model speed and material state feed public process heuristics", "[Model][ModelProcessHeuristics]")
{
    const GlobalSpeedMap saved_speed_map = Model::printSpeedMap;
    const std::map<size_t, ExtruderParams> saved_extruder_params = Model::extruderParamsMap;
    ScopeGuard restore_model_static_state([&]() {
        Model::printSpeedMap = saved_speed_map;
        Model::extruderParamsMap = saved_extruder_params;
    });

    const auto [maximum_key, global_maximum, object_maximum] = GENERATE(
        table<const char *, double, double>({
            { "inner_wall_speed", 160., 96. },
            { "outer_wall_speed", 160., 96. },
            { "sparse_infill_speed", 160., 96. },
            { "internal_solid_infill_speed", 160., 96. },
            { "top_surface_speed", 160., 96. },
            { "support_speed", 160., 96. },
        }));
    CAPTURE(maximum_key);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("printable_area", new ConfigOptionPoints{
        Vec2d(0., 0.), Vec2d(100., 0.), Vec2d(100., 100.), Vec2d(0., 100.)
    });
    config.set_deserialize_strict({
        { "inner_wall_speed", "40" },
        { "outer_wall_speed", "55" },
        { "sparse_infill_speed", "70" },
        { "internal_solid_infill_speed", "85" },
        { "top_surface_speed", "95" },
        { "support_speed", "110" },
        { "nozzle_temperature", "240,210" },
    });
    config.set_key_value(maximum_key, new ConfigOptionFloatsNullable{ global_maximum });
    config.set_key_value("filament_type", new ConfigOptionStrings({ "PETG", "PLA" }));
    Model::setPrintSpeedTable(config, PrintConfig::defaults());
    Model::setExtruderParams(config, 2);

    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *petg = object->add_volume(make_cube(2., 2., 2.), ModelVolumeType::MODEL_PART, false);
    ModelVolume *pla = object->add_volume(make_cube(2., 2., 2.), ModelVolumeType::MODEL_PART, false);
    petg->config.set("extruder", 1);
    pla->config.set("extruder", 2);

    // Each row makes one of the six explicit global speed keys the independently
    // known maximum, so every aggregation branch is observable.
    CHECK_THAT(Model::findMaxSpeed(object), Catch::Matchers::WithinAbs(global_maximum, 1e-9));
    DynamicPrintConfig object_config;
    object_config.set_deserialize_strict({
        { "inner_wall_speed", "31" },
        { "outer_wall_speed", "32" },
        { "sparse_infill_speed", "33" },
        { "internal_solid_infill_speed", "34" },
        { "top_surface_speed", "35" },
        { "support_speed", "36" },
        { "small_perimeter_speed", "20" },
        { "small_support_perimeter_speed", "25" },
    });
    object->config.assign_config(object_config);
    object->config.set_key_value(maximum_key, new ConfigOptionFloatsNullable{ object_maximum });
    CHECK_THAT(Model::findMaxSpeed(object), Catch::Matchers::WithinAbs(object_maximum, 1e-9));

    // PETG and PLA have distinct public material thermal-length contracts; the group
    // overload must retain the shortest member rather than an insertion-order value.
    CHECK_THAT(Model::getThermalLength(petg), Catch::Matchers::WithinAbs(100., 1e-9));
    CHECK_THAT(Model::getThermalLength(pla), Catch::Matchers::WithinAbs(200., 1e-9));
    CHECK_THAT(Model::getThermalLength(std::vector<ModelVolume *>{ petg, pla }),
               Catch::Matchers::WithinAbs(100., 1e-9));
    CHECK_THAT(Model::getThermalLength(std::vector<ModelVolume *>{ pla, petg, nullptr }),
               Catch::Matchers::WithinAbs(100., 1e-9));
    CHECK_THAT(Model::getThermalLength(std::vector<ModelVolume *>{ nullptr, pla }),
               Catch::Matchers::WithinAbs(200., 1e-9));
}


TEST_CASE("Model reusable collection detaches volumes and preserves ownership metadata", "[Model][ModelReusableLifecycle]")
{
    Model model;
    ModelMaterial *material = model.add_material("reusable-material");
    ModelObject *object = model.add_object();
    ModelVolume *part = object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::MODEL_PART, false);
    ModelVolume *modifier = object->add_volume(make_cube(2., 2., 2.), ModelVolumeType::PARAMETER_MODIFIER, false);
    object->add_instance();
    const ObjectID part_id = part->id();
    const ObjectID modifier_id = modifier->id();

    REQUIRE(material->get_model() == &model);
    REQUIRE(object->get_model() == &model);
    REQUIRE(object->instances.front()->get_object() == object);
    REQUIRE(part->get_object() == object);
    REQUIRE(modifier->get_object() == object);

    const ModelVolumePtrs original_volumes{ part, modifier };
    const ModelObjectPtrs original_objects{ object };
    // collect_reusable_objects transfers raw pointers without deleting them. Restore
    // the exact original owners on every exit so retained-ownership mutants cannot
    // create duplicate owners or double-delete the detached objects.
    ScopeGuard restore_ownership([&]() {
        object->volumes = original_volumes;
        model.objects = original_objects;
    });

    std::vector<ObjectBase *> reusable;
    model.collect_reusable_objects(reusable);

    REQUIRE(model.objects.empty());
    REQUIRE(reusable.size() == 3);
    CHECK(reusable[0] == object);
    CHECK(reusable[1] == part);
    CHECK(reusable[2] == modifier);
    CHECK(object->volumes.empty());
    REQUIRE(object->volume_ids.size() == 2);
    CHECK(object->volume_ids[0] == part_id);
    CHECK(object->volume_ids[1] == modifier_id);
}

TEST_CASE("Model repaired mesh error counts aggregate each stored diagnostic", "[Model][ModelRepairedErrors]")
{
    const auto mesh_with_errors = [](const RepairedMeshErrors &errors) {
        TriangleMesh source = make_cube(4., 4., 4.);
        indexed_triangle_set its = source.its;
        return TriangleMesh(std::move(its), errors);
    };

    RepairedMeshErrors first_errors;
    first_errors.degenerate_facets = 1;
    first_errors.edges_fixed = 2;
    first_errors.facets_removed = 3;
    first_errors.facets_reversed = 4;
    first_errors.backwards_edges = 5;
    RepairedMeshErrors second_errors;
    second_errors.facets_reversed = 2;

    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *first = object->add_volume(mesh_with_errors(first_errors), ModelVolumeType::MODEL_PART, false);
    ModelVolume *second = object->add_volume(mesh_with_errors(second_errors), ModelVolumeType::MODEL_PART, false);

    CHECK(first->get_repaired_errors_count() == 15);
    CHECK(second->get_repaired_errors_count() == 2);
    CHECK(object->get_repaired_errors_count(0) == 15);
    CHECK(object->get_repaired_errors_count(1) == 2);
    CHECK(object->get_repaired_errors_count() == 17);
}

TEST_CASE("Model archive progress cancellation is honored at each post-load stage", "[Model][ModelImport][Archive][ModelImportCancellation]")
{
    const int cancelled_stage = GENERATE(IMPORT_STAGE_UPDATE_GCODE, IMPORT_STAGE_CHECK_MODE_GCODE);
    CAPTURE(cancelled_stage);
    ScopedSlic3rTemporaryDir backup("orca_model_import_post_load_cancel");
    ScopedTemporaryFile archive(".3mf");
    model_import_contracts::store_archive(archive.string());
    DynamicPrintConfig config = model_import_contracts::archive_config();
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    En3mfType type = En3mfType::From_Other;
    PlateDataPtrs plates;
    std::vector<Preset *> presets;
    Semver version;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset *preset : presets)
            delete preset;
    });
    bool callback_called = false;
    std::vector<int> progress_stages;
    Import3mfProgressFn progress = [&](int stage, int, int, bool &cancel) {
        progress_stages.push_back(stage);
        cancel = stage == cancelled_stage;
        if (cancel)
            callback_called = true;
    };

    CHECK_THROWS_AS(Model::read_from_archive(archive.string(), &config, &substitutions, type,
                                             LoadStrategy::LoadModel | LoadStrategy::LoadConfig,
                                             &plates, &presets, &version, progress),
                    Slic3r::RuntimeError);
    CHECK(callback_called);
    REQUIRE_FALSE(progress_stages.empty());
    const auto requested_stage = std::find(progress_stages.begin(), progress_stages.end(), cancelled_stage);
    REQUIRE(requested_stage != progress_stages.end());
    // Cancellation is terminal: callbacks before the selected post-load stage are
    // allowed, but no later stage may be observed. CHECK_MODE therefore proves that
    // UPDATE ran first, while UPDATE remains a one-callback cancellation.
    CHECK(requested_stage == progress_stages.end() - 1);
    if (cancelled_stage == IMPORT_STAGE_CHECK_MODE_GCODE) {
        const auto update_stage = std::find(progress_stages.begin(), progress_stages.end(), IMPORT_STAGE_UPDATE_GCODE);
        REQUIRE(update_stage != progress_stages.end());
        CHECK(update_stage < requested_stage);
    }
}

TEST_CASE("Model unit heuristics distinguish cut groups and unrestricted conversion", "[Model][ModelUnitConversion]")
{
    Model only_small_cut;
    ModelObject *small_cut_object = only_small_cut.add_object();
    small_cut_object->add_volume(make_cube(1., 1., 1.), ModelVolumeType::MODEL_PART, false);
    small_cut_object->cut_id = CutObjectBase(ObjectID(7001), 1, 0);
    CHECK(only_small_cut.looks_like_imperial_units());

    Model mixed_cut_group;
    ModelObject *small_part = mixed_cut_group.add_object();
    small_part->add_volume(make_cube(1., 1., 1.), ModelVolumeType::MODEL_PART, false);
    small_part->cut_id = CutObjectBase(ObjectID(7002), 1, 0);
    ModelObject *large_part = mixed_cut_group.add_object();
    large_part->add_volume(make_cube(3., 3., 3.), ModelVolumeType::MODEL_PART, false);
    large_part->cut_id = CutObjectBase(ObjectID(7002), 1, 0);
    CHECK_FALSE(mixed_cut_group.looks_like_imperial_units());

    Model imperial;
    ModelObject *imperial_object = imperial.add_object();
    ModelVolume *imperial_volume = imperial_object->add_volume(make_cube(3., 3., 3.), ModelVolumeType::MODEL_PART, false);
    imperial.convert_from_imperial_units(false);
    CHECK_THAT(imperial_volume->mesh().bounding_box().size().x(), Catch::Matchers::WithinAbs(76.2, 1e-4));
    CHECK(imperial_volume->source.is_converted_from_inches);

    Model metric;
    ModelObject *metric_object = metric.add_object();
    ModelVolume *metric_volume = metric_object->add_volume(make_cube(.3, .3, .3), ModelVolumeType::MODEL_PART, false);
    metric.convert_from_meters(false);
    CHECK_THAT(metric_volume->mesh().bounding_box().size().x(), Catch::Matchers::WithinAbs(300., 1e-3));
    CHECK(metric_volume->source.is_converted_from_meters);
}

TEST_CASE("Model instance auto brim width follows speed, geometry, and adhesion", "[Model][ModelAutoBrim]")
{
    const GlobalSpeedMap saved_speed_map = Model::printSpeedMap;
    const std::map<size_t, ExtruderParams> saved_extruder_params = Model::extruderParamsMap;
    ScopeGuard restore_model_static_state([&]() {
        Model::printSpeedMap = saved_speed_map;
        Model::extruderParamsMap = saved_extruder_params;
    });
    Model::printSpeedMap = {};
    Model::printSpeedMap.maxSpeed = 100.;
    Model::printSpeedMap.perimeterSpeed = 100.;
    Model::printSpeedMap.externalPerimeterSpeed = 100.;
    Model::printSpeedMap.infillSpeed = 100.;
    Model::printSpeedMap.solidInfillSpeed = 100.;
    Model::printSpeedMap.topSolidInfillSpeed = 100.;
    Model::printSpeedMap.supportSpeed = 100.;
    Model::printSpeedMap.smallPerimeterSpeed = 100.;
    Model::printSpeedMap.smallSupportPerimeterSpeed = 100.;
    Model::extruderParamsMap.clear();

    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(2., 2., 100.), ModelVolumeType::MODEL_PART, false);
    ModelInstance *instance = object->add_instance();

    const double narrow_brim = instance->get_auto_brim_width(0., 0.225);
    const double wide_brim = instance->get_auto_brim_width(0., 1.);
    CHECK_THAT(narrow_brim, Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(wide_brim, Catch::Matchers::WithinAbs(3. * std::sqrt(2.), 1e-9));
}

namespace model_step_wrapper_contracts
{
using namespace Slic3r;

std::string named_step_fixture_path()
{
    return std::string(TEST_DATA_DIR) + PATH_SEPARATOR + "utf8_part_names.step";
}

void write_malformed_step_fixture(const std::string &path)
{
    boost::nowide::ofstream output(path, std::ios::binary | std::ios::trunc);
    REQUIRE(output.good());
    output << "ISO-10303-21;\n"
              "HEADER;\n"
              "ENDSEC;\n"
              "DATA;\n"
              "#1 = ;\n"
              "ENDSEC;\n"
              "END-ISO-10303-21;\n";
    REQUIRE(output.good());
    output.flush();
    REQUIRE(output.good());
    output.close();
}

void require_named_step_model(const Model &model, const std::string &input_file, bool expect_default_instance)
{
    REQUIRE(model.objects.size() == 1);
    const ModelObject *object = model.objects.front();
    REQUIRE(object != nullptr);
    REQUIRE(object->volumes.size() == 3);
    CHECK(object->input_file == input_file);

    const std::array<const char *, 3> expected_names{
        "pi\xC3\xA8" "ce", "Geh\xC3\xA4use", "bracket"};
    for (std::size_t i = 0; i < expected_names.size(); ++i) {
        const ModelVolume *volume = object->volumes[i];
        REQUIRE(volume != nullptr);
        CHECK(volume->name == expected_names[i]);
        CHECK(volume->source.input_file == input_file);
        TriangleMesh mesh = volume->mesh();
        const double volume_mm3 = std::abs(static_cast<double>(mesh.volume()));
        CHECK(std::isfinite(volume_mm3));
        CHECK(volume_mm3 > 0.0);
        CHECK(volume->mesh().facets_count() > 0);
    }

    if (expect_default_instance)
        CHECK(object->instances.size() == 1);
    else
        CHECK(object->instances.empty());
}

TEST_CASE("Model STEP wrapper imports named solids and adds a default instance",
          "[Model][ModelImport][Step][ModelStepWrapper]")
{
    const std::string input_file = named_step_fixture_path();
    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_step(input_file,
                                                  LoadStrategy::AddDefaultInstances,
                                                  nullptr, nullptr, nullptr,
                                                  0.003, 0.5, false));
    require_named_step_model(model, input_file, true);
}

TEST_CASE("Model STEP wrapper passes requested options to the mesh callback",
          "[Model][ModelImport][Step][ModelStepWrapper]")
{
    const std::string input_file = named_step_fixture_path();
    int callback_calls = 0;
    double callback_linear_deflection = -1.0;
    double callback_angle_deflection = -1.0;
    bool callback_split_compound = true;
    auto mesh_options = [&](Step &, double &linear_deflection, double &angle_deflection,
                            bool &is_split_compound) {
        ++callback_calls;
        callback_linear_deflection = linear_deflection;
        callback_angle_deflection = angle_deflection;
        callback_split_compound = is_split_compound;
        linear_deflection = 0.05;
        angle_deflection = 0.25;
        is_split_compound = true;
        return 0;
    };

    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_step(input_file, LoadStrategy::Default,
                                                  nullptr, nullptr, mesh_options,
                                                  0.003, 0.5, false));
    REQUIRE(callback_calls == 1);
    CHECK_THAT(callback_linear_deflection, Catch::Matchers::WithinAbs(0.003, 1e-12));
    CHECK_THAT(callback_angle_deflection, Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_FALSE(callback_split_compound);
    require_named_step_model(model, input_file, false);
}

TEST_CASE("Model STEP wrapper cancellation returns an empty model",
          "[Model][ModelImport][Step][ModelStepWrapper]")
{
    const std::string input_file = named_step_fixture_path();
    bool callback_called = false;
    auto cancel = [&](Step &, double &, double &, bool &) {
        callback_called = true;
        return -1;
    };

    Model model;
    REQUIRE_NOTHROW(model = Model::read_from_step(input_file, LoadStrategy::Default,
                                                  nullptr, nullptr, cancel,
                                                  0.003, 0.5, false));
    REQUIRE(callback_called);
    CHECK(model.objects.empty());
    CHECK(model.materials.empty());
}

TEST_CASE("Model STEP wrapper classifies malformed input as a load failure",
          "[Model][ModelImport][Step][ModelStepWrapper][ModelImportErrors]")
{
    ScopedTemporaryFile malformed(".step");
    write_malformed_step_fixture(malformed.string());
    CHECK_THROWS_WITH(Model::read_from_step(malformed.string(), LoadStrategy::Default,
                                            nullptr, nullptr, nullptr,
                                            0.003, 0.5, false),
                      Catch::Matchers::ContainsSubstring("Loading of a model file failed."));
}

} // namespace model_step_wrapper_contracts

namespace model_next_active_contracts {
using namespace Slic3r;

TEST_CASE("Copying a model object through Model preserves owned volumes and instances", "[Model][ModelOwnership][ModelCopy]")
{
    Model model;
    ModelObject *source = model.add_object("copy-source", "source.stl", make_cube(4., 5., 6.));
    REQUIRE(source != nullptr);
    REQUIRE(source->volumes.size() == 1);
    source->add_instance();
    ModelInstance *second = source->add_instance();
    REQUIRE(second != nullptr);
    second->set_offset(Vec3d(20., 3., 1.));
    const ObjectID object_id = source->id();
    const ObjectID volume_id = source->volumes.front()->id();
    const Vec3d second_offset = second->get_offset();

    // Model::add_object is the public ownership/copy API: the clone receives
    // distinct IDs, remains parented by this Model, and leaves the source intact.
    ModelObject *copy = model.add_object(*source);

    REQUIRE(copy != nullptr);
    REQUIRE(copy != source);
    REQUIRE(copy->id() != object_id);
    REQUIRE(copy->volumes.size() == 1);
    REQUIRE(copy->instances.size() == 2);
    CHECK(copy->name == "copy-source");
    CHECK(copy->input_file == "source.stl");
    CHECK(copy->volumes.front()->id() != volume_id);
    CHECK(copy->volumes.front()->get_object() == copy);
    CHECK(copy->instances.front()->get_object() == copy);
    CHECK(copy->instances.back()->get_object() == copy);
    CHECK(copy->instances.back()->get_offset().isApprox(second_offset, 1e-9));
    CHECK(copy->get_model() == &model);
    CHECK(source->get_model() == &model);
    CHECK(source->volumes.size() == 1);
    CHECK(source->instances.size() == 2);
}

TEST_CASE("Named object insertion keeps source mesh geometry and records volume metadata", "[Model][ModelOwnership][ModelGeometry]")
{
    const TriangleMesh input = make_cube(4., 5., 6.);
    const BoundingBoxf3 input_box = input.bounding_box();
    TriangleMesh input_for_volume = input;
    const double input_volume = std::abs(static_cast<double>(input_for_volume.volume()));

    Model model;
    ModelObject *object = model.add_object("named-part", "input/source.stl", input);
    REQUIRE(object != nullptr);
    REQUIRE(object->volumes.size() == 1);
    const ModelVolume *volume = object->volumes.front();
    REQUIRE(volume != nullptr);

    CHECK(object->instances.empty());
    CHECK(object->name == "named-part");
    CHECK(object->input_file == "input/source.stl");
    CHECK(volume->name == "named-part");
    CHECK(volume->source.input_file == "input/source.stl");
    CHECK(volume->source.object_idx == 0);
    CHECK(volume->source.volume_idx == 0);
    CHECK(object->config.extruder() == 1);
    TriangleMesh volume_mesh = volume->mesh();
    CHECK_THAT(std::abs(static_cast<double>(volume_mesh.volume())),
               Catch::Matchers::WithinAbs(input_volume, 1e-5));
    CHECK_THAT(input.bounding_box().min.x(), Catch::Matchers::WithinAbs(input_box.min.x(), 1e-6));
    CHECK_THAT(input.bounding_box().max.y(), Catch::Matchers::WithinAbs(input_box.max.y(), 1e-6));
}

TEST_CASE("Shifting painted filament states moves only states at or above the threshold", "[Model][ModelPainting][ModelOwnership]")
{
    Model model;
    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    ModelVolume *volume = object->add_volume(make_cube(4., 4., 4.), ModelVolumeType::MODEL_PART, false);
    REQUIRE(volume != nullptr);

    TriangleSelector selector(volume->mesh());
    selector.set_facet(0, EnforcerBlockerType::Extruder2);
    selector.set_facet(1, EnforcerBlockerType::Extruder3);
    selector.set_facet(2, EnforcerBlockerType::Extruder4);
    REQUIRE(volume->mmu_segmentation_facets.set(selector));
    REQUIRE(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder2));
    REQUIRE(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder3));
    REQUIRE(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder4));

    volume->mmu_segmentation_facets.shift_states_above(*volume, EnforcerBlockerType::Extruder3, 2);

    CHECK(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder2));
    CHECK_FALSE(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder3));
    CHECK_FALSE(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder4));
    CHECK(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder5));
    CHECK(volume->mmu_segmentation_facets.has_facets(*volume, EnforcerBlockerType::Extruder6));
    CHECK(volume->mmu_segmentation_facets.get_facets(*volume, EnforcerBlockerType::Extruder2).indices.size() == 1);
    CHECK(volume->mmu_segmentation_facets.get_facets(*volume, EnforcerBlockerType::Extruder5).indices.size() == 1);
    CHECK(volume->mmu_segmentation_facets.get_facets(*volume, EnforcerBlockerType::Extruder6).indices.size() == 1);
}

} // namespace model_next_active_contracts

namespace volume_bed_transformation_contracts {
using namespace Slic3r;

namespace {

std::array<Vec3d, 8> centered_box_corners(const BoundingBoxf3 &bbox)
{
    const Vec3d lo = bbox.min.cast<double>();
    const Vec3d hi = bbox.max.cast<double>();
    return {
        Vec3d(lo.x(), lo.y(), lo.z()), Vec3d(lo.x(), lo.y(), hi.z()),
        Vec3d(lo.x(), hi.y(), lo.z()), Vec3d(lo.x(), hi.y(), hi.z()),
        Vec3d(hi.x(), lo.y(), lo.z()), Vec3d(hi.x(), lo.y(), hi.z()),
        Vec3d(hi.x(), hi.y(), lo.z()), Vec3d(hi.x(), hi.y(), hi.z())
    };
}

double max_corner_error(const Transform3d &chain, const BoundingBoxf3 &bbox)
{
    double max_error = 0.;
    for (const Vec3d &corner : centered_box_corners(bbox))
        max_error = std::max(max_error, (chain * corner - corner).norm());
    return max_error;
}

Vec3d transformed_box_size(const Transform3d &transform, const BoundingBoxf3 &bbox)
{
    const auto corners = centered_box_corners(bbox);
    Vec3d minimum = transform * corners.front();
    Vec3d maximum = minimum;
    for (size_t i = 1; i < corners.size(); ++i) {
        const Vec3d point = transform * corners[i];
        minimum = minimum.cwiseMin(point);
        maximum = maximum.cwiseMax(point);
    }
    return maximum - minimum;
}

} // namespace

TEST_CASE("Volume-to-bed transform cancels uniform world scale and translation", "[Model][Geometry][VolumeBedTransformation]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 2., 2.));
    instance.set_rotation(Z, Geometry::deg2rad(17.));
    instance.set_offset(Vec3d(37., -11., 5.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(max_corner_error(instance.get_matrix_no_offset() * volume.get_matrix(), bbox),
               Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(0.5, 1e-12));
}

TEST_CASE("Volume-to-bed transform cancels quarter-turn anisotropic mirror", "[Model][Geometry][VolumeBedTransformation]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 3., 4.));
    instance.set_rotation(Z, 0.5 * double(PI));
    instance.set_mirror(X, -1.);
    instance.set_offset(Vec3d(-17., 23., 9.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(max_corner_error(instance.get_matrix_no_offset() * volume.get_matrix(), bbox),
               Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(1. / 3., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(1. / 2., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(1. / 4., 1e-12));
    CHECK(volume.get_mirror().isApprox(Vec3d(1., -1., 1.), 1e-12));
}

TEST_CASE("Volume-to-bed transform keeps general rotation out of the volume", "[Model][Geometry][VolumeBedTransformation]")
{
    const BoundingBoxf3 bbox(Vec3d(-2., -3., -4.), Vec3d(2., 3., 4.));
    Geometry::Transformation instance;
    instance.set_scaling_factor(Vec3d(2., 3., 4.));
    instance.set_rotation(Z, Geometry::deg2rad(23.));
    instance.set_mirror(X, -1.);
    instance.set_offset(Vec3d(8., -19., 12.));

    const Geometry::Transformation volume = Geometry::Transformation::volume_to_bed_transformation(instance, bbox);

    CHECK(volume.get_offset().isApprox(Vec3d::Zero(), 1e-12));
    CHECK_THAT(volume.get_rotation().norm(), Catch::Matchers::WithinAbs(0., 1e-12));
    CHECK(volume.get_mirror().isApprox(Vec3d::Ones(), 1e-12));
    CHECK_THAT(volume.get_scaling_factor(X), Catch::Matchers::WithinAbs(0.5, 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Y), Catch::Matchers::WithinAbs(1. / 3., 1e-12));
    CHECK_THAT(volume.get_scaling_factor(Z), Catch::Matchers::WithinAbs(0.25, 1e-12));

    const Vec3d bed_size = transformed_box_size(volume.get_matrix(), bbox);
    CHECK_THAT(bed_size.x(), Catch::Matchers::WithinAbs(2., 1e-12));
    CHECK_THAT(bed_size.y(), Catch::Matchers::WithinAbs(2., 1e-12));
    CHECK_THAT(bed_size.z(), Catch::Matchers::WithinAbs(2., 1e-12));
}

} // namespace volume_bed_transformation_contracts
