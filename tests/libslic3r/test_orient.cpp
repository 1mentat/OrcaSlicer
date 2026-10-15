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

TEST_CASE("Cooling orientation leaves fanless and symmetric solids unchanged", "[Orient]")
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
}

TEST_CASE("Cooling orientation turns an asymmetric overhang toward the configured fan", "[Orient]")
{
    TriangleMesh source = make_cube(40., 30., 2.);
    source.rotate_x(0.5f);
    TriangleMesh fanless = source;
    orientation::orient_for_cooling(fanless, FanDirection::fdUndefine);
    check_mesh_unchanged(source.its, fanless);

    const auto fan = GENERATE(FanDirection::fdRight, FanDirection::fdLeft);
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
            CHECK(cooled_normals[i].dot(fan_axis) > 0.45f);
            CHECK_THAT(cooled_normals[i].z(), Catch::Matchers::WithinAbs(normal.z(), 1e-6));
        }
    }
    REQUIRE(overhangs > 0);
    CHECK_THAT(std::abs(its_volume(cooled.its)), Catch::Matchers::WithinRel(static_cast<double>(std::abs(its_volume(source.its))), 1e-6));
    CHECK_THAT(cooled.bounding_box().size().z(), Catch::Matchers::WithinAbs(source.bounding_box().size().z(), 1e-5));
}
