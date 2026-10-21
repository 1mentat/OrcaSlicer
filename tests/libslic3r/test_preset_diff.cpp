#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"
#include <libslic3r/CommonDefs.hpp>

#include <algorithm>
#include "libslic3r/Config.hpp"
#include <vector>
#include <cstddef>
#include <string>
#include <set>

using namespace Slic3r;

// Regression test for the python-plugin branch's intentional divergence from
// upstream in add_correct_opts_to_diff() (src/libslic3r/Preset.cpp): a vector
// option entry whose index is beyond the reference vector's length is reported
// dirty even when it duplicates an existing value. On main these duplicates
// were NOT flagged. See the comment on add_correct_opts_to_diff() in src/libslic3r/Preset.cpp.
TEST_CASE("deep_diff flags new vector entries that duplicate values[0]", "[PresetDiff][Config]")
{
    // reference: single-extruder vector (one entry)
    Preset reference(Preset::TYPE_PRINTER, "ref");
    reference.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4});

    // edited: a second extruder entry was added whose value duplicates the first
    Preset edited(Preset::TYPE_PRINTER, "edited");
    edited.config.set_key_value("nozzle_diameter", new ConfigOptionFloats{0.4, 0.4});

    // deep_compare = true routes through deep_diff() -> add_correct_opts_to_diff()
    std::vector<std::string> diff =
        PresetCollection::dirty_options(&edited, &reference, /*deep_compare=*/true);

    // The new index #1 is reported dirty even though 0.4 == values[0] (0.4).
    REQUIRE(std::find(diff.begin(), diff.end(), "nozzle_diameter#1") != diff.end());

    // Sanity: the unchanged existing index #0 is NOT reported, so the rule is
    // specific to new indices rather than flagging the whole vector.
    REQUIRE(std::find(diff.begin(), diff.end(), "nozzle_diameter#0") == diff.end());
}

TEST_CASE("deep_diff distinguishes absolute and percentage speeds for each variant", "[PresetDiff][Config]")
{
    const size_t changed_index = GENERATE(size_t(0), size_t(1));
    Preset reference(Preset::TYPE_PRINT, "ref");
    reference.config.set_key_value("small_perimeter_speed", new ConfigOptionFloatsOrPercents{{50., false}, {50., false}});

    Preset edited = reference;
    edited.config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed")->values[changed_index].percent = true;

    const auto diff = PresetCollection::dirty_options(&edited, &reference, /*deep_compare=*/true);
    REQUIRE(diff == std::vector<std::string>{"small_perimeter_speed#" + std::to_string(changed_index)});

    DynamicPrintConfig transferred = reference.config;
    transferred.apply_only(edited.config, diff);
    REQUIRE(*transferred.option("small_perimeter_speed") == *edited.config.option("small_perimeter_speed"));
}


TEST_CASE("dirty_options distinguishes deep per-index and shallow whole-option reports",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    Preset reference(Preset::TYPE_PRINTER, "reference");
    reference.config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.4}));
    Preset edited = reference;
    edited.config.option<ConfigOptionFloats>("nozzle_diameter")->values[1] = 0.5;

    const auto deep_diff = PresetCollection::dirty_options(&edited, &reference, true);
    CHECK(deep_diff == std::vector<std::string>{"nozzle_diameter#1"});
    const auto shallow_diff = PresetCollection::dirty_options(&edited, &reference, false);
    CHECK(shallow_diff == std::vector<std::string>{"nozzle_diameter"});
}

TEST_CASE("deep_diff reports nullable transitions while shallow diff reports the option",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    using NullableFloats = ConfigOptionFloatsNullable;

    Preset reference(Preset::TYPE_FILAMENT, "reference");
    reference.config.set_key_value("filament_retraction_length",
                                   new NullableFloats({1.0, 1.0}));
    Preset edited = reference;
    edited.config.option<NullableFloats>("filament_retraction_length")->values[0] =
        NullableFloats::nil_value();

    const auto deep_nil = PresetCollection::dirty_options(&edited, &reference, true);
    CHECK(deep_nil == std::vector<std::string>{"filament_retraction_length#0"});
    const auto shallow_nil = PresetCollection::dirty_options(&edited, &reference, false);
    CHECK(shallow_nil == std::vector<std::string>{"filament_retraction_length"});

    Preset nil_reference(Preset::TYPE_FILAMENT, "nil reference");
    nil_reference.config.set_key_value("filament_retraction_length",
                                       new NullableFloats({NullableFloats::nil_value(), 1.0}));
    Preset explicit_value = nil_reference;
    explicit_value.config.option<NullableFloats>("filament_retraction_length")->values[0] = 2.0;

    const auto deep_explicit = PresetCollection::dirty_options(&explicit_value, &nil_reference, true);
    CHECK(deep_explicit == std::vector<std::string>{"filament_retraction_length#0"});
    const auto shallow_explicit = PresetCollection::dirty_options(&explicit_value, &nil_reference, false);
    CHECK(shallow_explicit == std::vector<std::string>{"filament_retraction_length"});
}

TEST_CASE("deep_diff flags a nullable vector entry added by growth",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    using NullableFloats = ConfigOptionFloatsNullable;
    Preset reference(Preset::TYPE_FILAMENT, "reference");
    reference.config.set_key_value("filament_retraction_length",
                                   new NullableFloats({1.0, 1.0}));
    Preset grown = reference;
    grown.config.option<NullableFloats>("filament_retraction_length")->values.push_back(
        NullableFloats::nil_value());

    const auto deep_diff = PresetCollection::dirty_options(&grown, &reference, true);
    CHECK(std::find(deep_diff.begin(), deep_diff.end(), "filament_retraction_length#2") != deep_diff.end());
    const auto shallow_diff = PresetCollection::dirty_options(&grown, &reference, false);
    CHECK(shallow_diff == std::vector<std::string>{"filament_retraction_length"});
}

TEST_CASE("deep_diff reports every vector slot when an option exists on only one side",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    const bool option_is_edited = GENERATE(false, true);
    Preset edited(Preset::TYPE_PRINTER, "edited");
    Preset reference(Preset::TYPE_PRINTER, "reference");
    Preset& present = option_is_edited ? edited : reference;
    present.config.set_key_value("nozzle_diameter", new ConfigOptionFloats({0.4, 0.6}));

    const auto diff = PresetCollection::dirty_options(&edited, &reference, /*deep_compare=*/true);
    CHECK(diff == std::vector<std::string>{"nozzle_diameter#0", "nozzle_diameter#1"});
}

TEST_CASE("deep_diff dispatches changed extruder vectors by registered option type",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    Preset reference(Preset::TYPE_PRINTER, "reference");
    reference.config.set_key_value("nozzle_temperature", new ConfigOptionInts({200, 210}));
    Preset edited = reference;
    edited.config.option<ConfigOptionInts>("nozzle_temperature")->values[1] = 215;
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"nozzle_temperature#1"});

    reference.config.set_key_value("retract_when_changing_layer", new ConfigOptionBools({false, false}));
    edited = reference;
    edited.config.option<ConfigOptionBools>("retract_when_changing_layer")->values[1] = true;
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"retract_when_changing_layer#1"});

    reference.config.set_key_value("retract_before_wipe", new ConfigOptionPercents({10., 20.}));
    edited = reference;
    edited.config.option<ConfigOptionPercents>("retract_before_wipe")->values[1] = 25.;
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"retract_before_wipe#1"});

    reference.config.set_key_value("extruder_offset",
                                   new ConfigOptionPoints({Vec2d(0., 0.), Vec2d(1., 2.)}));
    edited = reference;
    edited.config.option<ConfigOptionPoints>("extruder_offset")->values[1] = Vec2d(3., 4.);
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"extruder_offset#1"});

    reference.config.set_key_value("nozzle_type",
                                   new ConfigOptionEnumsGenericNullable({ntUndefine, ntHardenedSteel}));
    edited = reference;
    edited.config.option<ConfigOptionEnumsGenericNullable>("nozzle_type")->values[1] = ntBrass;
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"nozzle_type#1"});
}

TEST_CASE("deep_diff reports changed and one-sided scalar options by key",
          "[Preset][PresetDiff][PresetDeepDiff]")
{
    Preset reference(Preset::TYPE_PRINT, "reference");
    reference.config.set_key_value("spiral_mode", new ConfigOptionBool(false));
    Preset edited = reference;
    edited.config.set_key_value("spiral_mode", new ConfigOptionBool(true));
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{"spiral_mode"});

    Preset absent(Preset::TYPE_PRINT, "absent");
    CHECK(PresetCollection::dirty_options(&reference, &absent, true) ==
          std::vector<std::string>{"spiral_mode"});
    CHECK(PresetCollection::dirty_options(&absent, &reference, true) ==
          std::vector<std::string>{"spiral_mode"});
}

TEST_CASE("dirty option reports preserve optional compatibility keys and filter exact ignores",
          "[Preset][PresetDiff]")
{
    const std::string option_key =
        GENERATE(std::string{"compatible_printers"}, std::string{"compatible_prints"});
    const auto preset_type = option_key == "compatible_printers" ? Preset::TYPE_PRINTER : Preset::TYPE_FILAMENT;

    Preset reference(preset_type, "reference");
    reference.config.set_key_value(option_key, new ConfigOptionStrings{"Printer or process A"});
    Preset edited = reference;
    edited.config.set_key_value(option_key, new ConfigOptionStrings{"Printer or process B"});
    CHECK(PresetCollection::dirty_options(&edited, &reference, true) ==
          std::vector<std::string>{option_key});

    const std::set<std::string> no_ignored_keys;
    const std::set<std::string> exact_ignored_key{option_key};
    const std::string other_compatibility_key = option_key == "compatible_printers" ?
        "compatible_prints" : "compatible_printers";
    const std::set<std::string> different_ignored_key{other_compatibility_key};

    Preset absent(preset_type, "absent");
    Preset explicitly_empty(preset_type, "explicitly empty");
    explicitly_empty.config.set_key_value(option_key, new ConfigOptionStrings{});
    CHECK(PresetCollection::is_dirty(&explicitly_empty, &absent));
    CHECK(PresetCollection::dirty_options(&explicitly_empty, &absent, false) ==
          std::vector<std::string>{option_key});
    CHECK(PresetCollection::dirty_options_without_option_list(
              &explicitly_empty, &absent, no_ignored_keys, false) == std::vector<std::string>{option_key});
    CHECK(PresetCollection::dirty_options_without_option_list(
              &explicitly_empty, &absent, different_ignored_key, false) == std::vector<std::string>{option_key});
    CHECK(PresetCollection::dirty_options_without_option_list(
              &explicitly_empty, &absent, exact_ignored_key, false).empty());
    CHECK(PresetCollection::dirty_options_without_option_list(
              &edited, &reference, exact_ignored_key, true).empty());
}
