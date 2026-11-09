#include <catch2/catch_test_macros.hpp>

#include <map>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "libslic3r/Config.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/FilamentGroupUtils.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"

using namespace Slic3r;
using namespace Slic3r::FilamentGroupUtils;
using namespace Slic3r::MultiNozzleUtils;

namespace {

DynamicPrintConfig filament_config(const std::string &colour, const std::string &type,
                                   const std::string &tray, bool is_support)
{
    DynamicPrintConfig config;
    config.set_key_value("filament_colour", new ConfigOptionStrings({colour}));
    config.set_key_value("filament_type", new ConfigOptionStrings({type}));
    config.set_key_value("tray_name", new ConfigOptionStrings({tray}));
    config.set_key_value("filament_is_support", new ConfigOptionBools({is_support}));
    return config;
}

NozzleInfo nozzle(int extruder_id, int group_id)
{
    NozzleInfo info;
    info.diameter = "0.4";
    info.volume_type = nvtStandard;
    info.extruder_id = extruder_id;
    info.group_id = group_id;
    return info;
}

} // namespace

TEST_CASE("Filament colors preserve RGB and alpha serialization", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const Color rgb("#112233");
    const Color rgba("#11223380");
    REQUIRE(rgb.to_hex_str() == "#112233");
    REQUIRE(rgb.to_hex_str(true) == "#112233ff");
    REQUIRE(rgba.to_hex_str(true) == "#11223380");
    CHECK(rgb != rgba);
    CHECK(Color(0, 2, 0, 0) < Color(0, 3, 0, 0));
    CHECK(Color(0, 0, 2, 0) < Color(0, 0, 3, 0));
    CHECK(Color(0, 0, 0, 2) < Color(0, 0, 0, 3));
    CHECK(Color(4, 5, 6, 7) == Color(4, 5, 6, 7));
}

TEST_CASE("Filament group sizes reserve an external slot only when requested", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    std::vector<std::map<int, int>> ams_counts(2);
    ams_counts[0] = {{1, 2}, {2, 3}};
    // The second extruder has no AMS entries, so the external-filament gate is
    // observable independently of the first group's weighted total.
    const auto with_external_slot = calc_max_group_size(ams_counts, false);
    const auto without_external_slot = calc_max_group_size(ams_counts, true);
    REQUIRE(with_external_slot == std::vector<int>{8, 1});
    REQUIRE(without_external_slot == std::vector<int>{8, 0});
}

TEST_CASE("Machine filament records normalize preset types and select external trays", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const DynamicPrintConfig external_support = filament_config("#010203", "Sup.PLA", "Ext", true);
    const DynamicPrintConfig ams_model = filament_config("#040506", "ABS-S", "AMS-1", false);
    const DynamicPrintConfig raw_model = filament_config("#070809", "PETG", "AMS-2", false);
    DynamicPrintConfig missing_fields;

    std::vector<std::vector<DynamicPrintConfig>> configs = {
        {external_support, missing_fields},
        {ams_model, raw_model},
    };
    std::vector<std::map<int, int>> ams_counts(2);
    ams_counts[0][1] = 1;
    ams_counts[1][1] = 1;

    const auto selected = build_machine_filaments(configs, ams_counts, false);
    REQUIRE(selected.size() == 2);
    REQUIRE(selected[0].size() == 1);
    CHECK(selected[0][0].type == "PLA");
    CHECK(selected[0][0].is_support);
    CHECK(selected[0][0].is_extended);
    CHECK(selected[0][0].extruder_id == 0);
    REQUIRE(selected[1].size() == 2);
    CHECK(selected[1][0].type == "ABS");
    CHECK_FALSE(selected[1][0].is_support);
    CHECK_FALSE(selected[1][0].is_extended);
    CHECK(selected[1][0].extruder_id == 1);
    CHECK(selected[1][1].type == "PETG");
    CHECK(selected[1][1].extruder_id == 1);

    const auto ignore_external = build_machine_filaments(configs, ams_counts, true);
    CHECK(ignore_external[0].empty());
    REQUIRE(ignore_external[1].size() == 2);
    CHECK(ignore_external[1][0].type == "ABS");
    CHECK(ignore_external[1][1].type == "PETG");
}

TEST_CASE("Unprintable limits preserve disjoint group ownership", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const std::vector<std::set<int>> physical = {{1, 2}, {3}};
    const std::vector<std::set<int>> geometric = {{4}, {5}};
    std::vector<std::set<int>> limits;
    REQUIRE(collect_unprintable_limits(physical, geometric, limits));
    REQUIRE(limits.size() == 2);
    CHECK(limits[0] == std::set<int>{1, 2, 4});
    CHECK(limits[1] == std::set<int>{3, 5});
}

TEST_CASE("Unprintable limits reject shared physical and geometric constraints", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    std::vector<std::set<int>> limits;
    const std::vector<std::set<int>> shared_physical = {{1, 2}, {2, 3}};
    const std::vector<std::set<int>> no_geometric = {{}, {}};
    CHECK_FALSE(collect_unprintable_limits(shared_physical, no_geometric, limits));
    REQUIRE(limits.size() == 2);
    CHECK(limits[0] == std::set<int>{1});
    CHECK(limits[1] == std::set<int>{3});

    const std::vector<std::set<int>> shared_across_sources = {{4}, {}};
    const std::vector<std::set<int>> cross_source = {{}, {4}};
    CHECK_FALSE(collect_unprintable_limits(shared_across_sources, cross_source, limits));
}

TEST_CASE("Unprintable filament indices preserve used positions across map forms", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const std::vector<unsigned int> used_filaments = {10, 20, 30};
    const std::vector<std::set<int>> elements = {{20, 40}, {10}};

    std::vector<std::set<int>> indices;
    extract_indices(used_filaments, elements, indices);
    REQUIRE(indices.size() == 2);
    CHECK(indices[0] == std::set<int>{1});
    CHECK(indices[1] == std::set<int>{0});

    std::map<int, int> map_limits;
    extract_unprintable_limit_indices(elements, used_filaments, map_limits);
    CHECK(map_limits == std::map<int, int>{{0, 1}, {1, 0}});

    std::unordered_map<int, std::vector<int>> vector_limits;
    extract_unprintable_limit_indices(elements, used_filaments, vector_limits);
    REQUIRE(vector_limits.size() == 2);
    CHECK(vector_limits.at(0) == std::vector<int>{1});
    CHECK(vector_limits.at(1) == std::vector<int>{0});
}

TEST_CASE("Printability rejects a filament blocked on its own group", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const std::vector<std::set<int>> groups = {{4, 5}, {7}};
    CHECK_FALSE(check_printable(groups, {{4, 0}}));
    CHECK(check_printable(groups, {{4, 1}}));
    CHECK(check_printable(groups, {{7, 0}}));
    CHECK_FALSE(check_printable(groups, {{7, 1}}));
}

TEST_CASE("Nozzle estimates count extruder and nozzle transitions", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const std::vector<NozzleInfo> nozzles = {nozzle(0, 0), nozzle(0, 1), nozzle(1, 2)};
    const std::vector<std::vector<int>> maps = {{0, 1, 2}};
    const std::vector<std::vector<unsigned int>> sequences = {{0, 1, 2}};
    const auto result = LayeredNozzleGroupResult::create(maps, nozzles, {0, 1, 2}, sequences);
    REQUIRE(result.has_value());

    const std::vector<std::vector<unsigned int>> layer_filaments = {{0, 1, 2}};
    CHECK(get_estimate_extruder_change_count(layer_filaments, *result) == 1);
    CHECK(get_estimate_nozzle_change_count(layer_filaments, *result) == 1);
    const std::pair<int, int> expected_changes{1, 2};
    CHECK(get_estimate_extruder_filament_change_count(*result) == expected_changes);
}

TEST_CASE("Nozzle inventories sort group ids per extruder", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const auto list = build_extruder_nozzle_list({nozzle(0, 1), nozzle(0, 0), nozzle(0, 1), nozzle(1, 2)});
    REQUIRE(list.size() == 2);
    CHECK(list.at(0) == std::vector<int>{0, 1, 1});
    CHECK(list.at(1) == std::vector<int>{2});
}

TEST_CASE("Used filament values update only valid used slots", "[FilamentGroupUtils][FilamentGroupUtilsGap]")
{
    const std::vector<int> old_values = {1, 2, 3};
    const std::vector<int> new_values = {4, 5};
    const std::vector<unsigned int> used = {0, 99, 2};
    CHECK(update_used_filament_values(old_values, new_values, used) == std::vector<int>{4, 2, 3});
}
