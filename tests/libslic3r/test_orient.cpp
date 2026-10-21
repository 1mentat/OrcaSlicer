#include "libslic3r/PrintConfig.hpp"
#include "catch2/matchers/catch_matchers.hpp"
#include "libslic3r/BoundingBox.hpp"
#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "libslic3r/Geometry.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Orient.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using namespace Slic3r::orientation;

namespace {

constexpr double orient_pi = 3.14159265358979323846;

TriangleMesh tetrahedron()
{
    std::vector<Vec3f> vertices{
        { 0.f, 0.f, 0.f }, { 20.f, 0.f, 0.f },
        { 0.f, 20.f, 0.f }, { 0.f, 0.f, 20.f }};
    std::vector<Vec3i32> faces{
        { 0, 2, 1 }, { 0, 1, 3 },
        { 0, 3, 2 }, { 1, 2, 3 }};
    return TriangleMesh(std::move(vertices), std::move(faces));
}

OrientMesh orient_mesh(TriangleMesh mesh, const char *name)
{
    OrientMesh item;
    item.mesh = std::move(mesh);
    item.name = name;
    return item;
}

void check_mesh_unchanged(const indexed_triangle_set &before, const TriangleMesh &after)
{
    REQUIRE(before.vertices.size() == after.its.vertices.size());
    REQUIRE(before.indices.size() == after.its.indices.size());
    for (std::size_t i = 0; i < before.vertices.size(); ++i)
        CHECK_THAT((before.vertices[i] - after.its.vertices[i]).norm(), Catch::Matchers::WithinAbs(0., 1e-7));
    for (std::size_t i = 0; i < before.indices.size(); ++i)
        for (int corner = 0; corner < 3; ++corner)
            CHECK(before.indices[i][corner] == after.its.indices[i][corner]);
}

void check_rigid_orientation(const OrientMesh &item)
{
    CHECK(std::isfinite(item.orientation.x()));
    CHECK(std::isfinite(item.orientation.y()));
    CHECK(std::isfinite(item.orientation.z()));
    CHECK_THAT(item.orientation.norm(), Catch::Matchers::WithinAbs(1., 1e-5));

    const Matrix3d gram = item.rotation_matrix.transpose() * item.rotation_matrix;
    CHECK_THAT((gram - Matrix3d::Identity()).norm(), Catch::Matchers::WithinAbs(0., 1e-7));
    CHECK_THAT(item.rotation_matrix.determinant(), Catch::Matchers::WithinAbs(1., 1e-7));
    const Vec3d leveled = item.rotation_matrix * item.orientation;
    CHECK_THAT(leveled.x(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(leveled.y(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(leveled.z(), Catch::Matchers::WithinAbs(1., 1e-5));
    CHECK(item.angle >= -1e-8);
    CHECK(item.angle <= orient_pi + 1e-7);
    CHECK(std::isfinite(item.axis.x()));
    CHECK(std::isfinite(item.axis.y()));
    CHECK(std::isfinite(item.axis.z()));
    if (item.angle > 1e-6)
        CHECK_THAT(item.axis.norm(), Catch::Matchers::WithinAbs(1., 1e-6));
}

void check_finite_mesh(const TriangleMesh &mesh)
{
    for (const Vec3f &vertex : mesh.its.vertices) {
        CHECK(std::isfinite(vertex.x()));
        CHECK(std::isfinite(vertex.y()));
        CHECK(std::isfinite(vertex.z()));
    }
}


TriangleMesh cooling_prism()
{
    constexpr float length = 100.f;
    constexpr float height = 14.f;
    constexpr float slope_run = 2.f;
    constexpr float depth = 80.f;
    constexpr std::size_t profile_size = 6;
    const std::array<std::array<float, 2>, profile_size> profile{{
        {{0.f, 0.f}},
        {{length, 0.f}},
        {{length + slope_run, slope_run}},
        {{length + slope_run, height - slope_run}},
        {{length, height}},
        {{0.f, height}}
    }};

    std::vector<Vec3f> vertices;
    vertices.reserve(profile_size * 2);
    for (int plane = 0; plane != 2; ++plane) {
        const float y = plane == 0 ? 0.f : depth;
        for (const auto &point : profile)
            vertices.emplace_back(point[0], y, point[1]);
    }

    std::vector<Vec3i32> faces;
    faces.reserve(profile_size * 4 - 4);
    for (std::size_t i = 0; i < profile_size; ++i) {
        const std::size_t next = (i + 1) % profile_size;
        faces.emplace_back(static_cast<int>(i),
                           static_cast<int>(i + profile_size),
                           static_cast<int>(next + profile_size));
        faces.emplace_back(static_cast<int>(i),
                           static_cast<int>(next + profile_size),
                           static_cast<int>(next));
    }
    for (std::size_t i = 1; i + 1 < profile_size; ++i) {
        faces.emplace_back(0, static_cast<int>(i), static_cast<int>(i + 1));
        faces.emplace_back(static_cast<int>(profile_size),
                           static_cast<int>(profile_size + i + 1),
                           static_cast<int>(profile_size + i));
    }
    return TriangleMesh(std::move(vertices), std::move(faces));
}

} // namespace

TEST_CASE("Orientation returns a rigid leveling transform without changing source geometry", "[Orient]")
{
    OrientMesh item = orient_mesh(make_cube(40., 30., 2.), "flat cuboid");
    const bool tilted = GENERATE(false, true);
    if (tilted) {
        item.mesh.rotate_x(0.37f);
        item.mesh.rotate_y(0.47f);
    }
    const indexed_triangle_set original = item.mesh.its;
    OrientParams params;
    std::atomic<unsigned> progress_calls{0};
    params.progressind = [&progress_calls](unsigned, std::string) {
        progress_calls.fetch_add(1, std::memory_order_relaxed);
    };

    OrientMeshs items{std::move(item)};
    orient(items, {}, params);
    REQUIRE(items.size() == 1);
    CHECK(progress_calls.load(std::memory_order_relaxed) == 1);
    check_rigid_orientation(items.front());
    check_mesh_unchanged(original, items.front().mesh);

    TriangleMesh leveled = items.front().mesh;
    const double original_volume = std::abs(its_volume(leveled.its));
    leveled.transform(items.front().rotation_matrix);
    const BoundingBoxf3 box = leveled.bounding_box();
    CHECK_THAT(std::abs(its_volume(leveled.its)), Catch::Matchers::WithinRel(original_volume, 1e-6));
    CHECK(box.size().z() <= box.size().x() + 1e-5);
    CHECK(box.size().z() <= box.size().y() + 1e-5);
    CHECK_THAT(box.size().z(), Catch::Matchers::WithinAbs(2., 1e-4));

    const std::vector<Vec3f> source_normals = its_face_normals(items.front().mesh.its);
    const std::vector<Vec3f> leveled_normals = its_face_normals(leveled.its);
    REQUIRE(leveled_normals.size() == source_normals.size());
    for (std::size_t i = 0; i < source_normals.size(); ++i) {
        const Vec3f expected = (items.front().rotation_matrix * source_normals[i].cast<double>()).cast<float>();
        CHECK_THAT(expected.dot(leveled_normals[i]), Catch::Matchers::WithinAbs(1., 1e-5));
    }
}

TEST_CASE("Parallel orientation keeps per-item transforms rigid and reconstructible", "[Orient]")
{
    OrientMeshs items;
    TriangleMesh tilted = make_cube(12., 8., 4.);
    tilted.rotate_x(0.37f);
    tilted.rotate_y(0.47f);
    items.push_back(orient_mesh(std::move(tilted), "tilted cuboid"));
    items.push_back(orient_mesh(tetrahedron(), "tetrahedron"));
    const std::vector<indexed_triangle_set> originals{
        items[0].mesh.its, items[1].mesh.its};
    const std::vector<double> original_volumes{
        std::abs(its_volume(items[0].mesh.its)), std::abs(its_volume(items[1].mesh.its))};

    OrientParams params;
    params.parallel = true;
    std::atomic<unsigned> progress_calls{0};
    params.progressind = [&progress_calls](unsigned, std::string) {
        progress_calls.fetch_add(1, std::memory_order_relaxed);
    };
    orient(items, {}, params);

    REQUIRE(progress_calls.load(std::memory_order_relaxed) == items.size());
    for (std::size_t i = 0; i < items.size(); ++i) {
        check_rigid_orientation(items[i]);
        check_mesh_unchanged(originals[i], items[i].mesh);
        CHECK_THAT(std::abs(its_volume(items[i].mesh.its)), Catch::Matchers::WithinRel(original_volumes[i], 1e-6));

        const Matrix3d reconstructed = Geometry::rotation_transform(items[i].euler_angles).linear();
        CHECK_THAT((reconstructed - items[i].rotation_matrix).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
    }
}

TEST_CASE("Orienting a flat cuboid places its broad face on the bed", "[Orient]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(40., 30., 2.), ModelVolumeType::MODEL_PART, false);
    object->add_instance();
    const double original_volume = std::abs(its_volume(object->mesh().its));

    orientation::orient(object);

    const BoundingBoxf3 box = object->instance_bounding_box(0);
    CHECK_THAT(box.min.z(), Catch::Matchers::WithinAbs(0., 1e-5));
    CHECK_THAT(box.size().z(), Catch::Matchers::WithinAbs(2., 1e-4));
    CHECK_THAT(std::abs(its_volume(object->mesh().its)), Catch::Matchers::WithinRel(original_volume, 1e-6));
}

TEST_CASE("Cooling orientation leaves fanless and symmetric solids unchanged", "[Orient][FanDirection]")
{
    const TriangleMesh source = make_cube(10., 10., 10.);
    TriangleMesh fanless = source;
    orientation::orient_for_cooling(fanless, FanDirection::fdUndefine);
    check_mesh_unchanged(source.its, fanless);

    TriangleMesh right = source;
    orientation::orient_for_cooling(right, FanDirection::fdRight);
    check_mesh_unchanged(source.its, right);
    check_finite_mesh(right);

    TriangleMesh left = source;
    orientation::orient_for_cooling(left, FanDirection::fdLeft);
    check_mesh_unchanged(source.its, left);
    check_finite_mesh(left);

    TriangleMesh both = source;
    orientation::orient_for_cooling(both, FanDirection::fdBoth);
    check_mesh_unchanged(source.its, both);
    check_finite_mesh(both);
}

TEST_CASE("Cooling orientation turns an asymmetric overhang toward the configured fan", "[Orient][FanDirection]")
{
    TriangleMesh source = make_cube(40., 30., 2.);
    source.rotate_x(0.5f);
    TriangleMesh fanless = source;
    orientation::orient_for_cooling(fanless, FanDirection::fdUndefine);
    check_mesh_unchanged(source.its, fanless);

    const auto fan = GENERATE(FanDirection::fdRight, FanDirection::fdLeft, FanDirection::fdBoth);
    const Vec3f fan_axis = fan == FanDirection::fdRight ? Vec3f(1.f, 0.f, 0.f) : Vec3f(-1.f, 0.f, 0.f);
    TriangleMesh cooled = source;
    orientation::orient_for_cooling(cooled, fan);
    const auto original_normals = its_face_normals(source.its);
    const auto cooled_normals = its_face_normals(cooled.its);
    REQUIRE(cooled_normals.size() == original_normals.size());
    std::size_t overhangs = 0;
    for (std::size_t i = 0; i < original_normals.size(); ++i) {
        const Vec3f &normal = original_normals[i];
        // A 0.5-radian tilt puts the broad underside within the cooling
        // overhang band; its horizontal normal originally points along Y.
        if (normal.z() < -0.64f && normal.z() > -0.97f) {
            ++overhangs;
            if (fan == FanDirection::fdBoth) {
                // Both fans may validly select either X-facing direction.
                const float fan_alignment = std::abs(cooled_normals[i].dot(Vec3f(1.f, 0.f, 0.f)));
                CHECK(fan_alignment > 0.45f);
            } else {
                CHECK(cooled_normals[i].dot(fan_axis) > 0.45f);
            }
            CHECK_THAT(cooled_normals[i].z(), Catch::Matchers::WithinAbs(normal.z(), 1e-6));
        }
    }
    REQUIRE(overhangs > 0);
    CHECK_THAT(std::abs(its_volume(cooled.its)), Catch::Matchers::WithinRel(static_cast<double>(std::abs(its_volume(source.its))), 1e-6));
    CHECK_THAT(cooled.bounding_box().size().z(), Catch::Matchers::WithinAbs(source.bounding_box().size().z(), 1e-5));
}


TEST_CASE("Parallel fan orientation aims an exposed slope toward the configured cooling axis", "[Orient][FanDirection]")
{
    constexpr double inv_sqrt_two = 0.7071067811865475244;
    constexpr double input_yaw = 0.37;
    TriangleMesh source = cooling_prism();
    source.rotate_z(static_cast<float>(input_yaw));
    source.translate(12.f, -8.f, 5.f);
    const indexed_triangle_set original = source.its;
    const double expected_volume = (100. * 14. + 2. * 14. - 2. * 2.) * 80.;
    CHECK_THAT(std::abs(its_volume(source.its)), Catch::Matchers::WithinRel(expected_volume, 1e-6));

    const double fan_sign = GENERATE(-1., 1.);
    OrientMesh item = orient_mesh(source, "asymmetric fan prism");
    item.overhang_angle = 30.;
    item.has_cooling_fan = true;
    item.cooling_direction = Vec3d(fan_sign, 0., 0.);

    OrientParams params;
    params.parallel = true;
    params.progressind = [](unsigned, std::string) {};
    OrientMeshs items{std::move(item)};
    orient(items, {}, params);

    REQUIRE(items.size() == 1);
    const OrientMesh &result = items.front();
    check_rigid_orientation(result);
    check_mesh_unchanged(original, result.mesh);
    CHECK_THAT(std::abs(its_volume(result.mesh.its)), Catch::Matchers::WithinRel(expected_volume, 1e-6));
    CHECK(std::abs(result.orientation.z()) > 0.999);

    const double cos_yaw = std::cos(input_yaw);
    const double sin_yaw = std::sin(input_yaw);
    const Vec3d lower_slope_normal(cos_yaw * inv_sqrt_two, sin_yaw * inv_sqrt_two, -inv_sqrt_two);
    const Vec3d upper_slope_normal(cos_yaw * inv_sqrt_two, sin_yaw * inv_sqrt_two, inv_sqrt_two);
    const Vec3d lower_after_primary = result.rotation_matrix * lower_slope_normal;
    const Vec3d upper_after_primary = result.rotation_matrix * upper_slope_normal;
    const Vec3d &downward_slope = lower_after_primary.z() < upper_after_primary.z()
        ? lower_after_primary : upper_after_primary;
    CHECK_THAT(downward_slope.z(), Catch::Matchers::WithinAbs(-inv_sqrt_two, 1e-5));

    Vec3d expected_airflow_axis(downward_slope.x(), downward_slope.y(), 0.);
    CHECK_THAT(expected_airflow_axis.norm(), Catch::Matchers::WithinAbs(inv_sqrt_two, 1e-5));
    REQUIRE(expected_airflow_axis.norm() > 0.69);
    expected_airflow_axis.normalize();
    CHECK_THAT(result.orientation_vertical.dot(expected_airflow_axis), Catch::Matchers::WithinAbs(1., 0.001));

    const Matrix3d &vertical = result.rotation_matrix_vertical;
    CHECK_THAT((vertical.transpose() * vertical - Matrix3d::Identity()).norm(), Catch::Matchers::WithinAbs(0., 1e-7));
    CHECK_THAT(vertical.determinant(), Catch::Matchers::WithinAbs(1., 1e-7));
    CHECK_THAT((vertical * result.orientation_vertical - result.cooling_direction).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT((vertical * downward_slope).dot(result.cooling_direction), Catch::Matchers::WithinAbs(inv_sqrt_two, 0.01));

    TriangleMesh fully_oriented = result.mesh;
    fully_oriented.transform(result.rotation_matrix);
    fully_oriented.transform(result.rotation_matrix_vertical);
    CHECK_THAT(std::abs(its_volume(fully_oriented.its)), Catch::Matchers::WithinRel(expected_volume, 1e-6));
}

TEST_CASE("Parallel fan orientation keeps the configured axis for a symmetric solid", "[Orient][FanDirection]")
{
    const double fan_sign = GENERATE(-1., 1.);
    OrientMesh item = orient_mesh(make_cube(10., 10., 10.), "symmetric fan cube");
    item.has_cooling_fan = true;
    item.cooling_direction = Vec3d(fan_sign, 0., 0.);
    const indexed_triangle_set original = item.mesh.its;

    OrientParams params;
    params.parallel = true;
    params.progressind = [](unsigned, std::string) {};
    OrientMeshs items{std::move(item)};
    orient(items, {}, params);

    REQUIRE(items.size() == 1);
    const OrientMesh &result = items.front();
    check_rigid_orientation(result);
    check_mesh_unchanged(original, result.mesh);
    CHECK_THAT((result.orientation_vertical - result.cooling_direction).norm(), Catch::Matchers::WithinAbs(0., 1e-6));
    CHECK_THAT((result.rotation_matrix_vertical - Matrix3d::Identity()).norm(), Catch::Matchers::WithinAbs(0., 1e-7));
}
