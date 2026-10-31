#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/MeasureUtils.hpp"
#include "libslic3r/Point.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>

using namespace Slic3r;
using namespace Slic3r::Measure;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {

void check_poly(const Polynomial1 &p, std::initializer_list<double> expected)
{
    REQUIRE(p.GetDegree() + 1 == expected.size());
    std::size_t i = 0;
    for (double coefficient : expected)
        CHECK_THAT(p[static_cast<uint32_t>(i++)], WithinAbs(coefficient, 1e-12));
}

double eval_reference(std::initializer_list<double> coefficients, double x)
{
    double result = 0.0;
    double power = 1.0;
    for (double coefficient : coefficients) {
        result += coefficient * power;
        power *= x;
    }
    return result;
}

} // namespace

TEST_CASE("MeasureUtils polynomials retain coefficients and evaluate independently", "[MeasureUtils]")
{
    Polynomial1 p{2.0, -3.0, 0.5};
    CHECK(p.GetDegree() == 2);
    for (double x : {-2.0, -0.25, 0.0, 1.5, 4.0})
        CHECK_THAT(p(x), WithinRel(eval_reference({2.0, -3.0, 0.5}, x), 1e-12));

    // The initializer-list constructor drops trailing zero coefficients.
    Polynomial1 reduced{4.0, 0.0, 0.0};
    CHECK(reduced.GetDegree() == 0);
    CHECK_THAT(reduced(2.5), WithinAbs(4.0, 1e-12));
    Polynomial1 zero_leading{0.0, 0.0, 0.0};
    CHECK(zero_leading.GetDegree() == 0);
    CHECK_THAT(zero_leading(11.0), WithinAbs(0.0, 1e-12));
}

TEST_CASE("MeasureUtils polynomial arithmetic matches convolution and pointwise sums", "[MeasureUtils]")
{
    const Polynomial1 p{1.0, 2.0, 3.0};
    const Polynomial1 q{4.0, -1.0, 0.0, 2.0};
    const Polynomial1 sum = q + p;
    const Polynomial1 difference = q - p;
    const Polynomial1 product = p * q;
    check_poly(sum, {5.0, 1.0, 3.0, 2.0});
    check_poly(difference, {3.0, -3.0, -3.0, 2.0});
    check_poly(product, {4.0, 7.0, 10.0, -1.0, 4.0, 6.0});
    for (double x : {-1.0, 0.5, 2.0})
        CHECK_THAT(product(x), WithinRel(p(x) * q(x), 1e-12));

    const Polynomial1 scaled = 2.5 * p;
    check_poly(scaled, {2.5, 5.0, 7.5});
}

TEST_CASE("MeasureUtils mutable polynomial degree and coefficient reset are explicit", "[MeasureUtils]")
{
    Polynomial1 p(4);
    CHECK(p.GetDegree() == 4);
    p.SetCoefficients(3.0);
    CHECK(p.GetDegree() == 4);
    CHECK_THAT(p(2.0), WithinAbs(3.0 * 31.0, 1e-12));
    p[4] = 0.0;
    p.EliminateLeadingZeros();
    CHECK(p.GetDegree() == 3);
    p[3] = 0.0;
    p.EliminateLeadingZeros();
    CHECK(p.GetDegree() == 2);
}

TEST_CASE("MeasureUtils root finder handles linear, constant, and zero polynomials", "[MeasureUtils]")
{
    const double linear[] = {-6.0, 2.0};
    double roots[4] = {};
    CHECK(RootsPolynomial::Find(1, linear, 64, roots) == 1);
    CHECK_THAT(roots[0], WithinAbs(3.0, 1e-12));

    const double constant[] = {7.0};
    CHECK(RootsPolynomial::Find(0, constant, 64, roots) == 0);
    // An identically zero polynomial reports the single root 0 (the GeometricTools convention).
    const double zero[] = {0.0, 0.0, 0.0};
    roots[0] = -1.0;
    CHECK(RootsPolynomial::Find(2, zero, 64, roots) == 1);
    CHECK_THAT(roots[0], WithinAbs(0.0, 1e-12));
    CHECK(RootsPolynomial::Find(-1, linear, 64, roots) == 0);
    CHECK(RootsPolynomial::Find(1, nullptr, 64, roots) == 0);
}

TEST_CASE("MeasureUtils root finder returns all simple quadratic roots", "[MeasureUtils]")
{
    const double coefficients[] = {6.0, -5.0, 1.0};
    double roots[4] = {};
    const int count = RootsPolynomial::Find(2, coefficients, 128, roots);
    REQUIRE(count == 2);
    std::sort(roots, roots + count);
    CHECK_THAT(roots[0], WithinAbs(2.0, 1e-8));
    CHECK_THAT(roots[1], WithinAbs(3.0, 1e-8));
}

TEST_CASE("MeasureUtils root finder handles repeated and leading-zero equations", "[MeasureUtils]")
{
    const double repeated[] = {-4.0, 4.0, -1.0};
    double roots[4] = {};
    const int repeated_count = RootsPolynomial::Find(2, repeated, 128, roots);
    REQUIRE(repeated_count >= 1);
    REQUIRE(repeated_count <= 2);
    for (int i = 0; i < repeated_count; ++i)
        CHECK_THAT(roots[i], WithinAbs(2.0, 1e-7));

    const double leading_zero[] = {-2.0, 1.0, 0.0};
    const int count = RootsPolynomial::Find(2, leading_zero, 128, roots);
    REQUIRE(count == 1);
    CHECK_THAT(roots[0], WithinAbs(2.0, 1e-8));
}

TEST_CASE("MeasureUtils interval root finder honors endpoint and bracket contracts", "[MeasureUtils]")
{
    const double coefficients[] = {-2.0, 1.0};
    double root = -1.0;
    // A root at an endpoint is returned as that endpoint.
    CHECK(RootsPolynomial::Find(1, coefficients, 2.0, 4.0, 64, root));
    CHECK_THAT(root, WithinAbs(2.0, 1e-12));
    // No sign change: the interval is not known to bracket a root.
    CHECK_FALSE(RootsPolynomial::Find(1, coefficients, 3.0, 4.0, 64, root));
    // A sign change with tmin > tmax is rejected as an invalid interval.
    CHECK_FALSE(RootsPolynomial::Find(1, coefficients, 4.0, 0.0, 64, root));

    const double quadratic[] = {-6.0, 0.0, 1.0};
    CHECK(RootsPolynomial::Find(2, quadratic, 0.0, 3.0, 64, root));
    CHECK_THAT(root, WithinAbs(std::sqrt(6.0), 1e-8));
}

TEST_CASE("MeasureUtils bisection converges to an irrational root", "[MeasureUtils]")
{
    const double coefficients[] = {-0.125, 0.0, 1.0};
    double root = 0.0;
    REQUIRE(RootsPolynomial::Find(2, coefficients, 0.0, 1.0, 128, root));
    CHECK_THAT(root, WithinAbs(std::sqrt(0.125), 1e-12));
}

TEST_CASE("MeasureUtils orthogonal vectors satisfy dot and length invariants", "[MeasureUtils]")
{
    for (const Vec3d &v : {Vec3d(5.0, 1.0, 2.0), Vec3d(1.0, 5.0, 2.0), Vec3d(1.0, 2.0, 5.0), Vec3d(-3.0, 4.0, -2.0)}) {
        const Vec3d u = get_orthogonal(v, false);
        const Vec3d unit = get_orthogonal(v, true);
        CHECK(u.norm() > 0.0);
        CHECK_THAT(u.dot(v), WithinAbs(0.0, 1e-12));
        CHECK_THAT(unit.dot(v), WithinAbs(0.0, 1e-12));
        CHECK_THAT(unit.norm(), WithinAbs(1.0, 1e-12));
    }
}
