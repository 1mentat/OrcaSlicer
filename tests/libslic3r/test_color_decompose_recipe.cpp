#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_test_run_info.hpp>
#include <catch2/interfaces/catch_interfaces_reporter.hpp>
#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>

#include "libslic3r/ColorDecomposeRecipe.hpp"
#include "libslic3r/Utils.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

using namespace Slic3r;

namespace {

struct ScopedRepositoryResources
{
    std::string previous;

    ScopedRepositoryResources() : previous(resources_dir())
    {
        const std::filesystem::path data_dir(TEST_DATA_DIR);
        set_resources_dir((data_dir.parent_path().parent_path() / "resources").string());
    }

    ~ScopedRepositoryResources() { set_resources_dir(previous); }
};

// standard_entries() loads resources/filament_mixing/standard_color_recipes.json once per process, from
// whatever resources_dir() is at the first lookup, and caches it. Other tests (e.g. the FilamentMixer
// blend test) can reach that lookup first with no resources set, which would cache an empty table.
// Prime the cache from the repository table before any test runs, then restore resources_dir().
class PrimeStandardRecipeTable : public Catch::EventListenerBase
{
public:
    using Catch::EventListenerBase::EventListenerBase;
    void testRunStarting(Catch::TestRunInfo const &) override
    {
        const ScopedRepositoryResources resources;
        (void) lookup_standard_recipe({0, 0, 0}, ColorDecomposeRecipeMode::CMYW, "");
    }
};

ColorDecomposePhysicalFilament filament(const char *hex, const char *type,
                                        unsigned int index, bool mixed = false)
{
    return {hex, "slot-" + std::to_string(index), type, mixed, index};
}

void require_rgb(const ColorDecomposeRgb &actual, unsigned char r,
                 unsigned char g, unsigned char b)
{
    CHECK(actual.r == r);
    CHECK(actual.g == g);
    CHECK(actual.b == b);
}

void require_valid_shape(const ColorDecomposeRecipeResult &result,
                         ColorDecomposeRecipeMode mode)
{
    REQUIRE(result.valid);
    CHECK(result.mode == mode);
    REQUIRE_FALSE(result.matched_color_hex.empty());
    REQUIRE_FALSE(result.components.empty());
    int ratio_sum = 0;
    for (const auto &component : result.components) {
        CHECK(component.ratio > 0);
        ratio_sum += component.ratio;
    }
    CHECK(ratio_sum == 100);
}

bool contains_index(const ColorDecomposeRecipeResult &result, unsigned int index)
{
    return std::any_of(result.components.begin(), result.components.end(),
                       [index](const auto &component) { return component.filament_index == index; });
}

// Independent CIE L*a*b* (D65) to 8-bit sRGB conversion for checking interpolated colours.
std::array<int, 3> lab_to_srgb_bytes(const std::array<double, 3> &lab)
{
    const auto f_inv = [](double v) {
        const double v3 = v * v * v;
        return v3 > 216.0 / 24389.0 ? v3 : (116.0 * v - 16.0) / (24389.0 / 27.0);
    };
    const double fy = (lab[0] + 16.0) / 116.0;
    const double x  = 0.95047 * f_inv(fy + lab[1] / 500.0);
    const double y  = f_inv(fy);
    const double z  = 1.08883 * f_inv(fy - lab[2] / 200.0);
    const std::array<double, 3> linear{3.2404542 * x - 1.5371385 * y - 0.4985314 * z,
                                       -0.9692660 * x + 1.8760108 * y + 0.0415560 * z,
                                       0.0556434 * x - 0.2040259 * y + 1.0572252 * z};
    std::array<int, 3> out{};
    for (std::size_t i = 0; i < 3; ++i) {
        const double c = std::clamp(linear[i], 0.0, 1.0);
        const double s = c <= 0.0031308 ? 12.92 * c : 1.055 * std::pow(c, 1.0 / 2.4) - 0.055;
        out[i] = int(std::lround(s * 255.0));
    }
    return out;
}

} // namespace

CATCH_REGISTER_LISTENER(PrimeStandardRecipeTable)

TEST_CASE("Recipe RGB encoding uses uppercase six-digit bytes", "[ColorDecomposeRecipe]")
{
    CHECK(color_decompose_rgb_to_hex({0, 0, 0}) == "#000000");
    CHECK(color_decompose_rgb_to_hex({0x01, 0xA2, 0xFF}) == "#01A2FF");
    CHECK(color_decompose_rgb_to_hex({255, 128, 16}) == "#FF8010");
}

TEST_CASE("Recipe RGB decoding accepts canonical upper and lower case", "[ColorDecomposeRecipe]")
{
    ColorDecomposeRgb upper{1, 2, 3};
    ColorDecomposeRgb lower{4, 5, 6};
    REQUIRE(color_decompose_hex_to_rgb("#A1b2C3", upper));
    REQUIRE(color_decompose_hex_to_rgb("#a1B2c3", lower));
    require_rgb(upper, 0xA1, 0xB2, 0xC3);
    require_rgb(lower, 0xA1, 0xB2, 0xC3);
    CHECK(color_decompose_rgb_to_hex(upper) == "#A1B2C3");
}

TEST_CASE("Recipe RGB decoding rejects malformed input without changing output", "[ColorDecomposeRecipe]")
{
    for (const std::string &input : {"", "123456", "#1234", "#GG0000", "#12GG00"}) {
        ColorDecomposeRgb output{17, 34, 51};
        CHECK_FALSE(color_decompose_hex_to_rgb(input, output));
        require_rgb(output, 17, 34, 51);
    }
}

TEST_CASE("Recipe RGB encoding and decoding round trip byte boundaries", "[ColorDecomposeRecipe]")
{
    for (const ColorDecomposeRgb rgb : {ColorDecomposeRgb{0, 0, 0},
                                       ColorDecomposeRgb{1, 127, 254},
                                       ColorDecomposeRgb{255, 255, 255}}) {
        ColorDecomposeRgb decoded{99, 99, 99};
        const std::string encoded = color_decompose_rgb_to_hex(rgb);
        REQUIRE(color_decompose_hex_to_rgb(encoded, decoded));
        require_rgb(decoded, rgb.r, rgb.g, rgb.b);
    }
}

TEST_CASE("Physical recommendation returns an exact matching filament", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#123456", "PLA Basic", 4),
        filament("#FFFFFF", "PLA Basic", 5),
    };
    const auto result = recommend_from_physical_filaments({0x12, 0x34, 0x56}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    REQUIRE(result.components.size() == 1);
    CHECK(result.matched_color_hex == "#123456");
    CHECK(result.components[0].color_hex == "#123456");
    CHECK(result.components[0].ratio == 100);
    CHECK(result.components[0].filament_index == 4);
}

TEST_CASE("Physical recommendation matches a Basic material alias", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#010203", "PETG", 8),
        filament("#010203", "PLA Basic", 7),
        filament("#FFFFFF", "PLA Basic", 9),
    };
    const auto result = recommend_from_physical_filaments({1, 2, 3}, filaments, "PLA");
    REQUIRE(result.valid);
    REQUIRE(result.components.size() == 1);
    CHECK(result.components[0].filament_index == 7);
}

TEST_CASE("Physical recommendation ignores mixed and malformed slots", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#not-a-color", "PLA", 1),
        filament("#965E9E", "PLA", 2, true),
        filament("#0086D6", "PLA", 3),
        filament("#EC008C", "PLA", 4),
    };
    const auto result = recommend_from_physical_filaments({0x96, 0x5E, 0x9E}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    REQUIRE(result.components.size() == 2);
    CHECK(result.matched_color_hex == "#965E9E");
    CHECK(result.components[0].filament_index == 3);
    CHECK(result.components[1].filament_index == 4);
    CHECK(result.components[0].ratio == 20);
    CHECK(result.components[1].ratio == 80);
}

TEST_CASE("Physical recommendation falls back to valid filaments when material has no pair", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#FF0000", "PLA", 1),
        filament("#0000FF", "PETG", 2),
    };
    const auto result = recommend_from_physical_filaments({128, 0, 128}, filaments, "ASA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK(result.components.size() == 2);
    CHECK(contains_index(result, 1));
    CHECK(contains_index(result, 2));
}

TEST_CASE("Physical recommendation falls back from one preferred slot to the full valid list", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#FF0000", "PLA", 1),
        filament("#0000FF", "PETG", 2),
    };
    const auto result = recommend_from_physical_filaments({128, 0, 128}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK(contains_index(result, 1));
    CHECK(contains_index(result, 2));
}

TEST_CASE("Physical recommendation reports no recipe for fewer than two usable filaments", "[ColorDecomposeRecipe]")
{
    // The target is not any slot's colour, so the exact-match early exit cannot answer.
    const ColorDecomposeRgb target{200, 10, 10};
    CHECK_FALSE(recommend_from_physical_filaments(target, {}, "PLA").valid);
    CHECK_FALSE(recommend_from_physical_filaments(target,
                                                  {filament("#102030", "PLA", 1)}, "PLA").valid);
    CHECK_FALSE(recommend_from_physical_filaments(target,
                                                  {filament("#102030", "PLA", 1, true),
                                                   filament("#203040", "PLA", 2, true)}, "PLA").valid);
}

TEST_CASE("Physical recommendation keeps only the eight candidates nearest the target", "[ColorDecomposeRecipe]")
{
    // Nine usable slots: black first, then eight reds. The search keeps the eight nearest in Lab, so
    // black (the farthest from dark red) is dropped even though a red/black mix would be closer to
    // the target than any red/red mix. Keeping the first eight instead would keep black.
    std::vector<ColorDecomposePhysicalFilament> filaments;
    filaments.push_back(filament("#000000", "PLA", 1));
    for (unsigned int i = 2; i <= 9; ++i)
        filaments.push_back(filament("#FF0000", "PLA", i));

    const auto result = recommend_from_physical_filaments({128, 0, 0}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK_FALSE(contains_index(result, 1));
}

// Measured colors and ratios are golden anchors from
// resources/filament_mixing/standard_color_recipes.json.
TEST_CASE("Standard CMYW lookup returns the exact measured pair anchor", "[ColorDecomposeRecipe]")
{
    const auto result = lookup_standard_recipe({0x96, 0x5E, 0x9E},
                                                ColorDecomposeRecipeMode::CMYW, "PLA Basic");
    require_valid_shape(result, ColorDecomposeRecipeMode::CMYW);
    REQUIRE(result.components.size() == 2);
    CHECK(result.matched_color_hex == "#965E9E");
    CHECK(result.components[0].base_color == "Cyan");
    CHECK(result.components[1].base_color == "Magenta");
    CHECK(result.components[0].ratio == 20);
    CHECK(result.components[1].ratio == 80);
}

TEST_CASE("Standard RYBW lookup returns the exact measured pair anchor", "[ColorDecomposeRecipe]")
{
    const auto result = lookup_standard_recipe({0xDB, 0x78, 0x39},
                                                ColorDecomposeRecipeMode::RYBW, "PLA Basic");
    require_valid_shape(result, ColorDecomposeRecipeMode::RYBW);
    REQUIRE(result.components.size() == 2);
    CHECK(result.matched_color_hex == "#DB7839");
    CHECK(result.components[0].base_color == "Red");
    CHECK(result.components[1].base_color == "Yellow");
    CHECK(result.components[0].ratio == 20);
    CHECK(result.components[1].ratio == 80);
}

TEST_CASE("Standard lookup keeps the requested mode when selecting a different recipe family", "[ColorDecomposeRecipe]")
{
    const auto result = lookup_standard_recipe({0x96, 0x5E, 0x9E},
                                                ColorDecomposeRecipeMode::RYBW, "PLA Basic");
    require_valid_shape(result, ColorDecomposeRecipeMode::RYBW);
    const std::array<std::string, 4> allowed = {"Red", "Yellow", "Blue", "White"};
    CHECK(std::all_of(result.components.begin(), result.components.end(), [&allowed](const auto &component) {
        return std::find(allowed.begin(), allowed.end(), component.base_color) != allowed.end();
    }));
}

TEST_CASE("Standard lookup falls back when the preferred material has no table entries", "[ColorDecomposeRecipe]")
{
    const auto preferred = lookup_standard_recipe({0x96, 0x5E, 0x9E},
                                                   ColorDecomposeRecipeMode::CMYW, "PETG");
    const auto unfiltered = lookup_standard_recipe({0x96, 0x5E, 0x9E},
                                                    ColorDecomposeRecipeMode::CMYW, "");
    require_valid_shape(preferred, ColorDecomposeRecipeMode::CMYW);
    REQUIRE(unfiltered.valid);
    CHECK(preferred.matched_color_hex == unfiltered.matched_color_hex);
}

TEST_CASE("Measured blend lookup returns the exact two-component anchor", "[ColorDecomposeRecipe]")
{
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C"}, {20, 80}) == "#965E9E");
}

TEST_CASE("Measured blend lookup canonicalizes order, case, and scaled ratios", "[ColorDecomposeRecipe]")
{
    const std::string forward = lookup_measured_blend_color({"#0086D6", "#EC008C"}, {200, 800});
    const std::string reverse = lookup_measured_blend_color({"#ec008c", "#0086d6"}, {800, 200});
    CHECK(forward == "#965E9E");
    CHECK(reverse == forward);
}

TEST_CASE("Measured blend lookup interpolates linearly in Lab between the bracketing anchors", "[ColorDecomposeRecipe]")
{
    // Cyan/magenta 78/22 lies between the measured 75/25 and 80/20 anchors, whose stored Lab values are
    // (49.58, 11.74, -35.13) and (50.88, 5.61, -36.40). Linear interpolation at t = 0.6 gives the Lab
    // below; converting it with the standard D65 sRGB formulas gives the expected colour. The two
    // anchors are #6572B1 and #5679B7, far enough apart that the nearest anchor or a wrong weight
    // misses the +-1 tolerance.
    const double t = (78.0 - 75.0) / (80.0 - 75.0);
    const std::array<double, 3> lab{49.58 + t * (50.88 - 49.58), 11.74 + t * (5.61 - 11.74),
                                    -35.13 + t * (-36.40 + 35.13)};
    const std::array<int, 3> expected = lab_to_srgb_bytes(lab);

    const std::string interpolated = lookup_measured_blend_color({"#0086D6", "#EC008C"}, {78, 22});
    ColorDecomposeRgb decoded{};
    REQUIRE(color_decompose_hex_to_rgb(interpolated, decoded));
    CHECK(std::abs(int(decoded.r) - expected[0]) <= 1);
    CHECK(std::abs(int(decoded.g) - expected[1]) <= 1);
    CHECK(std::abs(int(decoded.b) - expected[2]) <= 1);
    CHECK(interpolated != "#6572B1");
    CHECK(interpolated != "#5679B7");
}

TEST_CASE("Measured blend lookup accepts the known three-component measured anchor", "[ColorDecomposeRecipe]")
{
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C", "#FFFFFF"}, {20, 20, 60}) == "#978BC2");
}

TEST_CASE("Measured blend lookup canonicalizes three-component order and scaled weights", "[ColorDecomposeRecipe]")
{
    const std::string result = lookup_measured_blend_color(
        {"#FFFFFF", "#ec008c", "#0086d6"}, {600, 200, 200});
    CHECK(result == "#978BC2");
}

TEST_CASE("Measured blend lookup rejects malformed or unsupported requests", "[ColorDecomposeRecipe]")
{
    CHECK(lookup_measured_blend_color({"#0086D6"}, {100}).empty());
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C"}, {20}).empty());
    CHECK(lookup_measured_blend_color({"bad", "#EC008C"}, {20, 80}).empty());
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C"}, {10, 90}).empty());
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C", "#FFFFFF"}, {0, 0, 0}).empty());
}

TEST_CASE("Measured blend lookup accepts a stored interpolated source anchor", "[ColorDecomposeRecipe]")
{
    CHECK(lookup_measured_blend_color({"#0086D6", "#EC008C", "#FFFFFF"}, {20, 25, 55}) == "#9484BD");
}

TEST_CASE("Standard recipe components carry no physical filament indices", "[ColorDecomposeRecipe]")
{
    const auto result = lookup_standard_recipe({0x97, 0x8B, 0xC2},
                                                ColorDecomposeRecipeMode::CMYW, "PLA Basic");
    require_valid_shape(result, ColorDecomposeRecipeMode::CMYW);
    for (const auto &component : result.components)
        CHECK(component.filament_index == 0);
}

TEST_CASE("Physical recipes carry one-based physical filament indices", "[ColorDecomposeRecipe]")
{
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#0086D6", "PLA", 2),
        filament("#EC008C", "PLA", 5),
    };
    const auto result = recommend_from_physical_filaments({0x96, 0x5E, 0x9E}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK(contains_index(result, 2));
    CHECK(contains_index(result, 5));
}

TEST_CASE("Physical recommendation reproduces a measured pair anchor", "[ColorDecomposeRecipe]")
{
    // #6572B1 is the measured colour of cyan/magenta 75/25; with exactly those two filaments loaded the
    // recommendation should land on that anchor rather than on a polynomial estimate.
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#0086D6", "PLA", 2),
        filament("#EC008C", "PLA", 5),
    };
    const auto result = recommend_from_physical_filaments({0x65, 0x72, 0xB1}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK(result.matched_color_hex == "#6572B1");
    REQUIRE(result.components.size() == 2);
    for (const auto &component : result.components)
        CHECK(component.ratio == (component.filament_index == 2 ? 75 : 25));
}

TEST_CASE("Physical recipe ratios stay within the 20 to 80 percent mixing range", "[ColorDecomposeRecipe]")
{
    // Outcome guard only: the 20..80 bound is enforced both by the ratio grid and by the measured
    // blend lookup, so this fixture cannot tell which one does it. The lookup's own rejection of
    // {10, 90} is covered by "Measured blend lookup rejects malformed or unsupported requests".
    const std::vector<ColorDecomposePhysicalFilament> filaments = {
        filament("#FF0000", "PLA", 1),
        filament("#00FF00", "PLA", 2),
        filament("#0000FF", "PLA", 3),
    };
    const auto result = recommend_from_physical_filaments({64, 80, 96}, filaments, "PLA");
    require_valid_shape(result, ColorDecomposeRecipeMode::MaterialList);
    CHECK(std::all_of(result.components.begin(), result.components.end(), [](const auto &component) {
        return component.ratio >= 20 && component.ratio <= 80;
    }));
}
