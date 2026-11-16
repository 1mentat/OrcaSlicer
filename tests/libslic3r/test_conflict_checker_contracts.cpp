#include <catch2/catch_test_macros.hpp>

#include <cstddef>

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/GCode/ConflictChecker.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

ConflictComputeOpt find_pairwise_conflict(const LineWithIDs &lines)
{
    for (std::size_t i = 0; i < lines.size(); ++i) {
        for (std::size_t j = 0; j < i; ++j) {
            if (ConflictComputeOpt conflict = ConflictChecker::line_intersect(lines[i], lines[j]))
                return conflict;
        }
    }
    return {};
}

bool same_unordered_pair(const ConflictComputeResult &lhs, const ConflictComputeResult &rhs)
{
    return (lhs._obj1 == rhs._obj1 && lhs._obj2 == rhs._obj2) ||
           (lhs._obj1 == rhs._obj2 && lhs._obj2 == rhs._obj1);
}

} // namespace

TEST_CASE(
    "Conflict checker matches pairwise results for overlap and separation",
    "[ConflictChecker][ConflictCheckerContracts]")
{
    const int object_a = 0;
    const int object_b = 1;

    const LineWithIDs overlapping{
        {Line(Point::new_scale(0., 0.), Point::new_scale(10., 0.)), &object_a, ExtrusionRole::erPerimeter},
        {Line(Point::new_scale(5., 0.), Point::new_scale(15., 0.)), &object_b, ExtrusionRole::erPerimeter},
    };
    const ConflictComputeOpt overlap_reference = find_pairwise_conflict(overlapping);
    const ConflictComputeOpt overlap_result = ConflictChecker::find_inter_of_lines(overlapping);
    REQUIRE(overlap_reference.has_value());
    REQUIRE(overlap_result.has_value());
    CHECK(overlap_reference->_obj1 != overlap_reference->_obj2);
    CHECK(overlap_result->_obj1 != overlap_result->_obj2);
    CHECK(same_unordered_pair(*overlap_result, *overlap_reference));

    const LineWithIDs separated{
        {Line(Point::new_scale(0., 0.), Point::new_scale(10., 0.)), &object_a, ExtrusionRole::erPerimeter},
        {Line(Point::new_scale(100., 0.), Point::new_scale(110., 0.)), &object_b, ExtrusionRole::erPerimeter},
    };
    const ConflictComputeOpt separated_reference = find_pairwise_conflict(separated);
    const ConflictComputeOpt separated_result = ConflictChecker::find_inter_of_lines(separated);
    CHECK_FALSE(separated_reference.has_value());
    CHECK_FALSE(separated_result.has_value());
}
