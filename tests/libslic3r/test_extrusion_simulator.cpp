#include <cstddef>
#include <vector>

#include <catch2/catch_message.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionSimulator.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r;

namespace {

constexpr int image_width = 64;
constexpr int image_height = 64;

class SimulatorFixture
{
public:
    SimulatorFixture()
    {
        simulator.set_image_size(Point(image_width, image_height));
        simulator.set_viewport(BoundingBox(Point(0, 0), Point(image_width, image_height)));
        simulator.set_bounding_box(BoundingBox(Point(0, 0), Point(scale_(8.0), scale_(8.0))));
        simulator.reset_accumulator();
    }

    std::vector<unsigned char> image(ExtrusionSimulationType mode)
    {
        simulator.evaluate_accumulator(mode);
        const auto *pixels = static_cast<const unsigned char *>(simulator.image_ptr());
        REQUIRE(pixels != nullptr);
        return {pixels, pixels + image_width * image_height * 4};
    }

    ExtrusionSimulator simulator;
};

ExtrusionPath right_angle_path()
{
    ExtrusionPath path(erPerimeter, 0.1, 0.5f, 0.2f);
    path.polyline.points = {
        Point3::new_scale(2.0, 2.0, 0.0),
        Point3::new_scale(5.0, 2.0, 0.0),
        Point3::new_scale(5.0, 5.0, 0.0),
    };
    return path;
}

size_t opaque_pixels(const std::vector<unsigned char> &image)
{
    size_t count = 0;
    for (size_t offset = 3; offset < image.size(); offset += 4)
        count += image[offset] != 0;
    return count;
}

} // namespace

TEST_CASE("extrusion simulation paints a bounded path and resets to an empty image", "[ExtrusionSimulator]")
{
    const auto mode = GENERATE(ExtrusionSimulationSimple, ExtrusionSimulationDontSpread,
                              ExtrisopmSimulationSpreadNotOverfilled, ExtrusionSimulationSpreadFull,
                              ExtrusionSimulationSpreadExcess);
    CAPTURE(mode);
    SimulatorFixture fixture;
    CHECK(opaque_pixels(fixture.image(mode)) == 0);

    fixture.simulator.extrude_to_accumulator(right_angle_path(), Point(0, 0), mode);
    const auto painted = fixture.image(mode);
    CHECK(opaque_pixels(painted) > 0);
    CHECK(opaque_pixels(painted) < image_width * image_height / 4);
    // The path and its radius lie well inside the image in every mode.
    for (int x = 0; x < image_width; ++x) {
        CHECK(painted[x * 4 + 3] == 0);
        CHECK(painted[((image_height - 1) * image_width + x) * 4 + 3] == 0);
    }
    for (int y = 0; y < image_height; ++y) {
        CHECK(painted[(y * image_width) * 4 + 3] == 0);
        CHECK(painted[(y * image_width + image_width - 1) * 4 + 3] == 0);
    }

    fixture.simulator.reset_accumulator();
    CHECK(opaque_pixels(fixture.image(mode)) == 0);
}

TEST_CASE("a whole-pixel extrusion shift translates the rendered material", "[ExtrusionSimulator]")
{
    const auto mode = GENERATE(ExtrusionSimulationSimple, ExtrusionSimulationDontSpread,
                              ExtrisopmSimulationSpreadNotOverfilled, ExtrusionSimulationSpreadFull,
                              ExtrusionSimulationSpreadExcess);
    CAPTURE(mode);
    SimulatorFixture original;
    SimulatorFixture shifted;
    const ExtrusionPath path = right_angle_path();
    original.simulator.extrude_to_accumulator(path, Point(0, 0), mode);
    shifted.simulator.extrude_to_accumulator(path, Point(scale_(1.0), scale_(2.0)), mode);
    const auto before = original.image(mode);
    const auto after = shifted.image(mode);
    REQUIRE(opaque_pixels(before) > 0);
    CHECK(opaque_pixels(after) == opaque_pixels(before));

    // The 8 mm bounding box maps to 64 pixels. Unequal X/Y shifts also
    // distinguish the two axes instead of accepting an accidental swap.
    constexpr int delta_x = 8;
    constexpr int delta_y = 16;
    for (int y = 0; y < image_height - delta_y; ++y)
        for (int x = 0; x < image_width - delta_x; ++x)
            for (int channel = 0; channel < 4; ++channel)
                CHECK(before[(y * image_width + x) * 4 + channel] ==
                      after[((y + delta_y) * image_width + x + delta_x) * 4 + channel]);
}
