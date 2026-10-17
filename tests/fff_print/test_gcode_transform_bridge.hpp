#pragma once

#include "libslic3r/Point.hpp"

#include <string>

namespace Slic3r {

// Test-owned declaration for the externally linked helper defined in GCode.cpp.
// Keep this local to the test overlay so old PR heads need no production-header edit.
std::string transform_gcode(const std::string &gcode, Vec2f pos,
                            const Vec2f &translation, float angle);

} // namespace Slic3r
