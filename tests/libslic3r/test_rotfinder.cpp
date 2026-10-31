#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/SLA/Rotfinder.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <mutex>
#include <utility>
#include <vector>

using namespace Slic3r;

namespace {

constexpr double ROT_EPS = 1e-4;

struct RotObject
{
    Model model;
    ModelObject *object;

    explicit RotObject(TriangleMesh mesh)
    {
        object = model.add_object();
        object->add_volume(std::move(mesh), ModelVolumeType::MODEL_PART, false);
        object->add_instance();
    }
};

RotObject asymmetric_prism()
{
    return RotObject(make_cube(3., 7., 11.));
}

// The X-Z profile below, extruded 20 mm along Y: a 20 mm wide top bar on two legs splaying out
// to a 40 mm footprint, with the notch between the legs open to the bed.
TriangleMesh splayed_frame()
{
    const std::vector<Vec2f> profile{{0.f, 0.f},   {2.f, 0.f},   {11.f, 18.f}, {29.f, 18.f},
                                     {38.f, 0.f},  {40.f, 0.f},  {30.f, 20.f}, {10.f, 20.f}};
    // The profile is not convex, so its caps are the left leg, the top bar and the right leg.
    const std::vector<Vec3i32> cap{{0, 1, 2}, {0, 2, 7}, {7, 2, 3}, {7, 3, 6}, {3, 4, 5}, {3, 5, 6}};
    constexpr float depth = 20.f;
    const int n = int(profile.size());

    std::vector<Vec3f> vertices;
    for (const float y : {0.f, depth})
        for (const Vec2f &point : profile)
            vertices.emplace_back(point.x(), y, point.y());
    std::vector<Vec3i32> faces;
    for (int i = 0; i < n; ++i) {
        const int next = (i + 1) % n;
        faces.emplace_back(i, i + n, next + n);
        faces.emplace_back(i, next + n, next);
    }
    for (const Vec3i32 &tri : cap) {
        faces.push_back(tri);
        faces.emplace_back(tri.x() + n, tri.z() + n, tri.y() + n);
    }
    return TriangleMesh(std::move(vertices), std::move(faces));
}

Transform3f rotation_transform(const Vec2d &rotation)
{
    Transform3f transform = Transform3f::Identity();
    transform.rotate(Eigen::AngleAxisf(float(rotation.y()), Vec3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(float(rotation.x()), Vec3f::UnitX()));
    return transform;
}

BoundingBoxf3 transformed_box(const TriangleMesh &mesh, const Vec2d &rotation)
{
    const Transform3f transform = rotation_transform(rotation);
    Vec3f minimum = transform * mesh.its.vertices.front();
    Vec3f maximum = minimum;
    for (const Vec3f &vertex : mesh.its.vertices) {
        const Vec3f transformed = transform * vertex;
        minimum = minimum.cwiseMin(transformed);
        maximum = maximum.cwiseMax(transformed);
    }
    return {minimum.cast<double>(), maximum.cast<double>()};
}

double alignment_score(const TriangleMesh &mesh, const Vec2d &rotation)
{
    const Transform3f transform = rotation_transform(rotation);
    double total = 0.;
    for (const auto &face : mesh.its.indices) {
        const Vec3f a = transform * mesh.its.vertices[face(0)];
        const Vec3f b = transform * mesh.its.vertices[face(1)];
        const Vec3f c = transform * mesh.its.vertices[face(2)];
        const Vec3f cross = (b - a).cross(c - a);
        const double area = 0.5 * double(cross.norm());
        const Vec3f normal = cross.normalized();
        total += area * (std::abs(double(normal.x()))
                         + std::abs(double(normal.y()))
                         + std::abs(double(normal.z())));
    }
    return total / double(mesh.its.indices.size());
}

double elevated_support_score(const TriangleMesh &mesh, const Vec2d &rotation)
{
    const Transform3f transform = rotation_transform(rotation);
    double total = 0.;
    for (const auto &face : mesh.its.indices) {
        const Vec3f a = transform * mesh.its.vertices[face(0)];
        const Vec3f b = transform * mesh.its.vertices[face(1)];
        const Vec3f c = transform * mesh.its.vertices[face(2)];
        const Vec3f cross = (b - a).cross(c - a);
        const double area = 0.5 * double(cross.norm());
        const Vec3f normal = cross.normalized();
        const double phi = 1. - std::acos(std::clamp(double(normal.dot(Vec3f(0.f, 0.f, -1.f))), -1., 1.)) / PI;
        total += std::sqrt(area) * phi * phi * phi;
    }
    return total / double(mesh.its.indices.size());
}

double floor_contact_area(const TriangleMesh &mesh, const Vec2d &rotation)
{
    const Transform3f transform = rotation_transform(rotation);
    std::vector<Vec3f> vertices;
    vertices.reserve(mesh.its.vertices.size());
    float minimum_z = std::numeric_limits<float>::max();
    for (const Vec3f &vertex : mesh.its.vertices) {
        vertices.push_back(transform * vertex);
        minimum_z = std::min(minimum_z, vertices.back().z());
    }

    double total = 0.;
    for (const auto &face : mesh.its.indices) {
        const Vec3f &a = vertices[face(0)];
        const Vec3f &b = vertices[face(1)];
        const Vec3f &c = vertices[face(2)];
        if (std::abs(a.z() - minimum_z) > 1e-3f
            || std::abs(b.z() - minimum_z) > 1e-3f
            || std::abs(c.z() - minimum_z) > 1e-3f)
            continue;
        total += 0.5 * std::abs(double((b - a).cross(c - a).z()));
    }
    return total;
}

void check_rotation_range(const Vec2d &rotation)
{
    CHECK(std::isfinite(rotation.x()));
    CHECK(std::isfinite(rotation.y()));
    CHECK(rotation.x() >= -PI - ROT_EPS);
    CHECK(rotation.x() <= PI + ROT_EPS);
    CHECK(rotation.y() >= -PI - ROT_EPS);
    CHECK(rotation.y() <= PI + ROT_EPS);
}

struct StatusLog
{
    std::mutex mutex;
    std::vector<int> values;
    bool cancel = false;

    bool operator()(int status)
    {
        std::lock_guard<std::mutex> lock(mutex);
        values.push_back(status);
        return !(cancel && status < 0);
    }
};

void check_status_values(const StatusLog &log)
{
    REQUIRE_FALSE(log.values.empty());
    bool saw_progress = false;
    for (int status : log.values) {
        CHECK(status >= -1);
        CHECK(status <= 100);
        saw_progress = saw_progress || status >= 0;
    }
    CHECK(saw_progress);
}

} // namespace

TEST_CASE("Best misalignment rotation tilts an axis-aligned prism away from the reference planes", "[Rotfinder]")
{
    RotObject fixture = asymmetric_prism();
    StatusLog status;
    const sla::RotOptimizeParams params = sla::RotOptimizeParams{}
        .accuracy(0.1f)
        .statucb([&status](int value) { return status(value); });

    const Vec2d rotation = sla::find_best_misalignment_rotation(*fixture.object, params);
    check_rotation_range(rotation);
    check_status_values(status);
    const double identity_score = alignment_score(fixture.object->raw_mesh(), Vec2d::Zero());
    const double result_score = alignment_score(fixture.object->raw_mesh(), rotation);
    // The optimizer intentionally seeks face-normal misalignment: tilting an
    // axis-aligned rectangular prism improves the independent L1 alignment
    // score. A minimizing mutant falls back near the identity score.
    CHECK(result_score >= identity_score + 5.);
}

TEST_CASE("Elevated least-support rotation beats every axis-aligned pose of the prism", "[Rotfinder]")
{
    RotObject fixture = asymmetric_prism();
    fixture.object->config.set_key_value("support_object_elevation", new ConfigOptionFloat(4.0));
    StatusLog status;
    const sla::RotOptimizeParams params = sla::RotOptimizeParams{}
        .accuracy(0.1f)
        .statucb([&status](int value) { return status(value); });

    const Vec2d rotation = sla::find_least_supports_rotation(*fixture.object, params);
    check_rotation_range(rotation);
    check_status_values(status);
    // elevated_support_score restates the documented scoring rule (sqrt(area) * phi^3 per face).
    // Every axis-aligned pose leaves one face fully overhanging; a tilted pose must beat all six.
    double best_aligned = std::numeric_limits<double>::max();
    for (const Vec2d &pose : {Vec2d(0., 0.), Vec2d(PI / 2., 0.), Vec2d(-PI / 2., 0.), Vec2d(PI, 0.), Vec2d(0., PI / 2.), Vec2d(0., -PI / 2.)})
        best_aligned = std::min(best_aligned, elevated_support_score(fixture.object->raw_mesh(), pose));
    const double result_score = elevated_support_score(fixture.object->raw_mesh(), rotation);
    INFO("best axis-aligned " << best_aligned << ", result " << result_score);
    CHECK(result_score < best_aligned);
}

TEST_CASE("Floor least-support rotation prefers full contact over the largest hull face", "[Rotfinder]")
{
    // A table-like frame with splayed legs. Its largest convex-hull face is the 40 x 20 footprint,
    // which is the first candidate the search sees, yet the frame only touches that plane with the
    // two 2 x 20 leg feet. The 20 x 20 top and the 22.4 x 20 outer leg faces rest flat on the bed.
    RotObject fixture(splayed_frame());
    // pad_around_object makes an elevated object count as on the floor, so both settings reach
    // the convex-hull face search.
    const bool pad_around = GENERATE(false, true);
    fixture.object->config.set_key_value("support_object_elevation", new ConfigOptionFloat(pad_around ? 4.0 : 0.0));
    fixture.object->config.set_key_value("pad_around_object", new ConfigOptionBool(pad_around));
    StatusLog status;
    const sla::RotOptimizeParams params = sla::RotOptimizeParams{}
        .accuracy(0.1f)
        .statucb([&status](int value) { return status(value); });

    const Vec2d rotation = sla::find_least_supports_rotation(*fixture.object, params);
    check_rotation_range(rotation);
    check_status_values(status);
    const TriangleMesh &mesh = fixture.object->raw_mesh();
    // Profile area 114 mm² (600 mm² trapezoid minus the 486 mm² notch) times the 20 mm depth; a
    // positive volume also confirms that the faces point outward.
    REQUIRE_THAT(its_volume(mesh.its), Catch::Matchers::WithinRel(114. * 20., 1e-6));
    REQUIRE_THAT(floor_contact_area(mesh, Vec2d::Zero()), Catch::Matchers::WithinAbs(2. * 2. * 20., 1e-3));
    const double result_contact = floor_contact_area(mesh, rotation);
    INFO("contact " << result_contact);
    // Resting on the footprint gives 80 mm² of contact; the top gives 400 and an outer leg face 447.
    CHECK(result_contact > 390.);
}

TEST_CASE("Minimum Z rotation selects the prism's thinnest dimension", "[Rotfinder]")
{
    RotObject fixture = asymmetric_prism();
    const int scaling_case = GENERATE(0, 1, 2);
    const Vec3d scaling = scaling_case == 0 ? Vec3d(1., 1., 1.)
                        : scaling_case == 1 ? Vec3d(2., 0.25, 1.) : Vec3d(2., 1., 0.1);
    fixture.object->instances.front()->set_scaling_factor(scaling);
    TriangleMesh scaled_mesh = fixture.object->raw_mesh();
    for (Vec3f &vertex : scaled_mesh.its.vertices)
        vertex = vertex.cwiseProduct(scaling.cast<float>());
    const double smallest_dimension = std::min({3. * scaling.x(), 7. * scaling.y(), 11. * scaling.z()});
    StatusLog status;
    const sla::RotOptimizeParams params = sla::RotOptimizeParams{}
        .accuracy(0.1f)
        .statucb([&status](int value) { return status(value); });

    const Vec2d rotation = sla::find_min_z_height_rotation(*fixture.object, params);
    check_rotation_range(rotation);
    check_status_values(status);
    const BoundingBoxf3 result_box = transformed_box(scaled_mesh, rotation);
    CHECK(result_box.size().z() > 0.);
    CHECK_THAT(result_box.size().z(), Catch::Matchers::WithinAbs(smallest_dimension, 1e-3));
}

TEST_CASE("Canceled minimum Z search stops before evaluating any rotation", "[Rotfinder]")
{
    RotObject fixture = asymmetric_prism();
    StatusLog status;
    status.cancel = true;
    const sla::RotOptimizeParams params = sla::RotOptimizeParams{}
        .accuracy(0.1f)
        .statucb([&status](int value) { return status(value); });

    const Vec2d rotation = sla::find_min_z_height_rotation(*fixture.object, params);
    check_rotation_range(rotation);
    REQUIRE_FALSE(status.values.empty());
    bool saw_cancellation = false;
    for (int value : status.values)
        saw_cancellation = saw_cancellation || value < 0;
    CHECK(saw_cancellation);
    CHECK(std::none_of(status.values.begin(), status.values.end(), [](int value) { return value >= 0; }));
}
