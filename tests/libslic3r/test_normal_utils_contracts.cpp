#include <admesh/stl.h>
#include <Eigen/Core>

#include <cmath>
#include <cstddef>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/NormalUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

constexpr float kPi = 3.14159265358979323846f;

void check_vec(const Vec3f &actual, const Vec3f &expected, float tolerance = 1e-5f)
{
    CHECK_THAT(actual.x(), WithinAbs(expected.x(), tolerance));
    CHECK_THAT(actual.y(), WithinAbs(expected.y(), tolerance));
    CHECK_THAT(actual.z(), WithinAbs(expected.z(), tolerance));
}

void require_finite_normals(const NormalUtils::Normals &normals)
{
    REQUIRE_FALSE(normals.empty());
    for (const Vec3f &normal : normals) {
        CHECK(std::isfinite(normal.x()));
        CHECK(std::isfinite(normal.y()));
        CHECK(std::isfinite(normal.z()));
        CHECK(normal.norm() > 0.f);
    }
}

indexed_triangle_set right_triangle()
{
    indexed_triangle_set its;
    its.vertices = {stl_vertex(0.f, 0.f, 0.f), stl_vertex(4.f, 0.f, 0.f), stl_vertex(0.f, 4.f, 0.f)};
    its.indices = {Vec3i32(0, 1, 2)};
    return its;
}

indexed_triangle_set asymmetric_fan()
{
    indexed_triangle_set its;
    its.vertices = {
        stl_vertex(0.f, 0.f, 0.f),
        stl_vertex(4.f, 0.f, 0.f),
        stl_vertex(0.f, 4.f, 0.f),
        stl_vertex(0.f, 3.f, 0.f),
        stl_vertex(0.f, 1.5f, 2.598076211f),
    };
    // The origin is index 0 in the XY face and index 2 in the YZ face.
    // The face normals are +Z and +X; the origin angles are 90 and 60 degrees.
    its.indices = {Vec3i32(0, 1, 2), Vec3i32(3, 4, 0)};
    return its;
}

} // namespace

TEST_CASE("Triangle normals preserve orientation, angles, and rigid-scale invariance", "[NormalUtils][NormalUtilsAngles]")
{
    const indexed_triangle_set base = right_triangle();
    const Vec3i32 indices = base.indices.front();
    const Vec3f expected_normal(0.f, 0.f, 1.f);

    check_vec(NormalUtils::create_triangle_normal(indices, base.vertices), expected_normal);
    const NormalUtils::Normals triangle_normals = NormalUtils::create_triangle_normals(base);
    REQUIRE(triangle_normals.size() == 1);
    check_vec(triangle_normals.front(), expected_normal);

    CHECK_THAT(NormalUtils::indice_angle(0, indices, base.vertices), WithinAbs(kPi / 2.f, 1e-5f));
    CHECK_THAT(NormalUtils::indice_angle(1, indices, base.vertices), WithinAbs(kPi / 4.f, 1e-5f));
    CHECK_THAT(NormalUtils::indice_angle(2, indices, base.vertices), WithinAbs(kPi / 4.f, 1e-5f));

    indexed_triangle_set transformed = base;
    for (stl_vertex &vertex : transformed.vertices)
        vertex = vertex * 3.f + Vec3f(7.f, -2.f, 11.f);
    const NormalUtils::Normals transformed_normals = NormalUtils::create_triangle_normals(transformed);
    REQUIRE(transformed_normals.size() == triangle_normals.size());
    check_vec(transformed_normals.front(), expected_normal);
}

TEST_CASE("Angle-weighted normals honor noncontiguous face indices", "[NormalUtils][NormalUtilsAngles]")
{
    const indexed_triangle_set fan = asymmetric_fan();
    const Vec3i32 second_face = fan.indices[1];

    // This is an explicit index-map regression: local index 2 names global
    // vertex 0, whose actual angle on the YZ face is 60 degrees.
    CHECK_THAT(NormalUtils::indice_angle(0, second_face, fan.vertices), WithinAbs(kPi / 3.f, 1e-5f));
    CHECK_THAT(NormalUtils::indice_angle(1, second_face, fan.vertices), WithinAbs(kPi / 3.f, 1e-5f));
    CHECK_THAT(NormalUtils::indice_angle(2, second_face, fan.vertices), WithinAbs(kPi / 3.f, 1e-5f));

    const NormalUtils::Normals average = NormalUtils::create_normals(fan, NormalUtils::VertexNormalType::AverageNeighbor);
    const NormalUtils::Normals angle = NormalUtils::create_normals(fan, NormalUtils::VertexNormalType::AngleWeighted);
    const NormalUtils::Normals selected_nelson = NormalUtils::create_normals(fan, NormalUtils::VertexNormalType::NelsonMaxWeighted);
    const NormalUtils::Normals default_normals = NormalUtils::create_normals(fan);
    REQUIRE(average.size() == fan.vertices.size());
    REQUIRE(angle.size() == fan.vertices.size());
    REQUIRE(selected_nelson.size() == fan.vertices.size());
    REQUIRE(default_normals.size() == fan.vertices.size());
    require_finite_normals(average);
    require_finite_normals(angle);
    require_finite_normals(selected_nelson);
    require_finite_normals(default_normals);

    // The face normals are +Z and +X.  The average is equal weighting, while
    // the corrected angle-weighted result uses 90 and 60 degrees at origin.
    check_vec(average[0], Vec3f(.5f, 0.f, .5f));
    check_vec(angle[0], Vec3f(.4f, 0.f, .6f));
    check_vec(default_normals[0], selected_nelson[0]);
    CHECK((angle[0] - average[0]).norm() > .05f);
}
