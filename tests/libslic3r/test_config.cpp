#include <cstdlib>
#include <exception>
#include <boost/nowide/cstdlib.hpp>
#include "libslic3r/Point.hpp"
#include <string>
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <ostream>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/PrintConfigConstants.hpp"
#include "libslic3r/LocalesUtils.hpp"

#include "test_utils.hpp"

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>

#include <boost/nowide/fstream.hpp>
#include "libslic3r/Config.hpp"
#include <array>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <ios>
#include <initializer_list>
#include <limits>
#include <memory>
#include <map>
#include <iterator>
#include <nlohmann/json.hpp>

#include <set>
#include <sstream>
#include <vector>
#include <utility>
#include <catch2/matchers/catch_matchers_vector.hpp>

using namespace Slic3r;

SCENARIO("Generic config validation performs as expected.", "[Config]") {
    GIVEN("A config generated from default options") {
        Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
        WHEN( "outer_wall_line_width is set to 250%, a valid value") {
            config.set_deserialize_strict("outer_wall_line_width", "250%");
            THEN( "The config is read as valid.") {
                REQUIRE(config.validate().empty());
            }
        }
        WHEN( "outer_wall_line_width is set to -10, an invalid value") {
            config.set("outer_wall_line_width", -10);
            THEN( "Validate returns error") {
                REQUIRE_FALSE(config.validate().empty());
            }
        }

        WHEN( "wall_loops is set to -10, an invalid value") {
            config.set("wall_loops", -10);
            THEN( "Validate returns error") {
                REQUIRE_FALSE(config.validate().empty());
            }
        }
    }
}

SCENARIO("Config accessor functions perform as expected.", "[Config]") {
    GIVEN("A config generated from default options") {
        Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();
        WHEN("A boolean option is set to a boolean value") {
            REQUIRE_NOTHROW(config.set("gcode_comments", true));
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionBool>("gcode_comments")->getBool() == true);
            }
        }
        WHEN("A boolean option is set to a string value representing a 0 or 1") {
            CHECK_NOTHROW(config.set_deserialize_strict("gcode_comments", "1"));
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionBool>("gcode_comments")->getBool() == true);
            }
        }
        WHEN("A boolean option is set to a string value representing something other than 0 or 1") {
            THEN("A BadOptionTypeException exception is thrown.") {
                REQUIRE_THROWS_AS(config.set("gcode_comments", "Z"), BadOptionTypeException);
            }
            AND_THEN("Value is unchanged.") {
                REQUIRE(config.opt<ConfigOptionBool>("gcode_comments")->getBool() == false);
            }
        }
        WHEN("A boolean option is set to an int value") {
            THEN("A BadOptionTypeException exception is thrown.") {
                REQUIRE_THROWS_AS(config.set("gcode_comments", 1), BadOptionTypeException);
            }
        }
        WHEN("A numeric option is set from serialized string") {
            config.set_deserialize_strict("raft_layers", "20");
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionInt>("raft_layers")->getInt() == 20);
            }
        }
	WHEN("An integer-based option is set through the integer interface") {
	    config.set("raft_layers", 100);
	    THEN("The underlying value is set correctly.") {
		REQUIRE(config.opt<ConfigOptionInt>("raft_layers")->getInt() == 100);
	    }
        }
        WHEN("An floating-point option is set through the integer interface") {
            config.set("max_bridge_length", 10);
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionFloat>("max_bridge_length")->getFloat() == 10.0);
            }
        }
        WHEN("A floating-point option is set through the double interface") {
            config.set("max_bridge_length", 5.5);
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionFloat>("max_bridge_length")->getFloat() == 5.5);
            }
        }
        WHEN("An integer-based option is set through the double interface") {
            THEN("A BadOptionTypeException exception is thrown.") {
                REQUIRE_THROWS_AS(config.set("top_shell_layers", 5.5), BadOptionTypeException);
            }
        }
        WHEN("A numeric option is set to a non-numeric value.") {
	    auto prev_value = config.opt<ConfigOptionFloat>("max_bridge_length")->getFloat();
            THEN("A BadOptionTypeException exception is thrown.") {
                REQUIRE_THROWS_AS(config.set_deserialize_strict("max_bridge_length", "zzzz"), BadOptionValueException);
            }
            THEN("The value does not change.") {
                REQUIRE(config.opt<ConfigOptionFloat>("max_bridge_length")->getFloat() == prev_value);
            }
        }
        WHEN("A string option is set through the string interface") {
            config.set("machine_end_gcode", "100");
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionString>("machine_end_gcode")->value == "100");
            }
        }
        WHEN("A string option is set through the integer interface") {
            config.set("machine_end_gcode", 100);
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionString>("machine_end_gcode")->value == "100");
            }
        }
        WHEN("A string option is set through the double interface") {
            config.set("machine_end_gcode", 100.5);
            THEN("The underlying value is set correctly.") {
                REQUIRE(config.opt<ConfigOptionString>("machine_end_gcode")->value == float_to_string_decimal_point(100.5));
            }
        }
        WHEN("A float or percent is set as a percent through the string interface.") {
            config.set_deserialize_strict("initial_layer_line_width", "100%");
            THEN("Value and percent flag are 100/true") {
                auto tmp = config.opt<ConfigOptionFloatOrPercent>("initial_layer_line_width");
                REQUIRE(tmp->percent == true);
                REQUIRE(tmp->value == 100);
            }
        }
        WHEN("A float or percent is set as a float through the string interface.") {
            config.set_deserialize_strict("initial_layer_line_width", "100");
            THEN("Value and percent flag are 100/false") {
                auto tmp = config.opt<ConfigOptionFloatOrPercent>("initial_layer_line_width");
                REQUIRE(tmp->percent == false);
                REQUIRE(tmp->value == 100);
            }
        }
        WHEN("A float or percent is set as a float through the int interface.") {
            config.set("initial_layer_line_width", 100);
            THEN("Value and percent flag are 100/false") {
                auto tmp = config.opt<ConfigOptionFloatOrPercent>("initial_layer_line_width");
                REQUIRE(tmp->percent == false);
                REQUIRE(tmp->value == 100);
            }
        }
        WHEN("A float or percent is set as a float through the double interface.") {
            config.set("initial_layer_line_width", 100.5);
            THEN("Value and percent flag are 100.5/false") {
                auto tmp = config.opt<ConfigOptionFloatOrPercent>("initial_layer_line_width");
                REQUIRE(tmp->percent == false);
                REQUIRE(tmp->value == 100.5);
            }
        }
        WHEN("A numeric vector is set from serialized string") {
	    config.set_deserialize_strict("temperature_vitrification", "10,20");
            THEN("The underlying value is set correctly.") {
                CHECK(config.opt<ConfigOptionInts>("temperature_vitrification")->get_at(0) == 10);
                CHECK(config.opt<ConfigOptionInts>("temperature_vitrification")->get_at(1) == 20);
            }
        }
	// FIXME: Design better accessors for vector elements
	// The following isn't supported and probably shouldn't be:
	// WHEN("An integer-based vector option is set through the integer interface") {
	//     config.set("temperature_vitrification", 100);
	//     THEN("The underlying value is set correctly.") {
	// 	REQUIRE(config.opt<ConfigOptionInts>("temperature_vitrification")->get_at(0) == 100);
	//     }
        // }
	WHEN("An integer-based vector option is set through the set_key_value interface") {
	    config.set_key_value("temperature_vitrification", new ConfigOptionInts{10,20});
	    THEN("The underlying value is set correctly.") {
                CHECK(config.opt<ConfigOptionInts>("temperature_vitrification")->get_at(0) == 10);
                CHECK(config.opt<ConfigOptionInts>("temperature_vitrification")->get_at(1) == 20);
	    }
        }
        WHEN("An invalid option is requested during set.") {
            THEN("A BadOptionTypeException exception is thrown.") {
                REQUIRE_THROWS_AS(config.set("deadbeef_invalid_option", 1), UnknownOptionException);
                REQUIRE_THROWS_AS(config.set("deadbeef_invalid_option", 1.0), UnknownOptionException);
                REQUIRE_THROWS_AS(config.set("deadbeef_invalid_option", "1"), UnknownOptionException);
                REQUIRE_THROWS_AS(config.set("deadbeef_invalid_option", true), UnknownOptionException);
            }
        }

        WHEN("An invalid option is requested during get.") {
            THEN("A UnknownOptionException exception is thrown.") {
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionString>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionFloat>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionInt>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionBool>("deadbeef_invalid_option", false), UnknownOptionException);
            }
        }
        WHEN("An invalid option is requested during opt.") {
            THEN("A UnknownOptionException exception is thrown.") {
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionString>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionFloat>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionInt>("deadbeef_invalid_option", false), UnknownOptionException);
                REQUIRE_THROWS_AS(config.option_throw<ConfigOptionBool>("deadbeef_invalid_option", false), UnknownOptionException);
            }
        }

        WHEN("getX called on an unset option.") {
            THEN("The default is returned.") {
                REQUIRE(config.opt_float("layer_height") == INITIAL_LAYER_HEIGHT);
                REQUIRE(config.opt_int("raft_layers") == INITIAL_RAFT_LAYERS);
                REQUIRE(config.opt_bool("reduce_crossing_wall") == INITIAL_REDUCE_CROSSING_WALL);
            }
        }

        WHEN("opt_float called on an option that has been set.") {
            config.set("layer_height", INITIAL_LAYER_HEIGHT*2);
            THEN("The set value is returned.") {
                REQUIRE(config.opt_float("layer_height") == INITIAL_LAYER_HEIGHT*2);
            }
        }
    }
}

SCENARIO("Config ini load/save interface", "[Config]") {
    WHEN("new_from_ini is called") {
		Slic3r::DynamicPrintConfig config;
		std::string path = std::string(TEST_DATA_DIR) + "/test_config/new_from_ini.ini";
		config.load_from_ini(path, ForwardCompatibilitySubstitutionRule::Disable);
        THEN("Config object contains ini file options.") {
			REQUIRE(config.option_throw<ConfigOptionStrings>("filament_colour", false)->values.size() == 1);
			REQUIRE(config.option_throw<ConfigOptionStrings>("filament_colour", false)->values.front() == "#ABCD");
        }
    }
}

TEST_CASE("Flush-volume warning predicate respects used filament transitions", "[Config][Regression]")
{
    const std::vector<double> multipliers = {1.0};

    SECTION("Single used filament does not trigger warning with zero transition entries")
    {
        const std::vector<double> matrix = {
            0.0, 0.0,
            0.0, 0.0
        };
        const std::vector<int> used_filaments = {1};

        REQUIRE_FALSE(has_zero_flush_volume_for_used_filaments(matrix, multipliers, used_filaments));
    }

    SECTION("Two used filaments trigger warning when transition flush entry is zero")
    {
        const std::vector<double> matrix = {
            0.0, 0.0,
            0.0, 0.0
        };
        const std::vector<int> used_filaments = {1, 2};

        REQUIRE(has_zero_flush_volume_for_used_filaments(matrix, multipliers, used_filaments));
    }

    SECTION("Two used filaments do not trigger warning when transitions are non-zero")
    {
        const std::vector<double> matrix = {
            0.0, 280.0,
            280.0, 0.0
        };
        const std::vector<int> used_filaments = {1, 2};

        REQUIRE_FALSE(has_zero_flush_volume_for_used_filaments(matrix, multipliers, used_filaments));
    }

    SECTION("Zero multiplier still triggers warning when multiple filaments are used")
    {
        const std::vector<double> matrix = {
            0.0, 280.0,
            280.0, 0.0
        };
        const std::vector<double> zero_multiplier = {0.0};
        const std::vector<int> used_filaments = {1, 2};

        REQUIRE(has_zero_flush_volume_for_used_filaments(matrix, zero_multiplier, used_filaments));
    }
}

// TODO: https://github.com/SoftFever/OrcaSlicer/issues/11269 - Is this test still relevant? Delete if not.
// It was failing so at least "nozzle_type" and "extruder_printable_area" could not be serialized
// and an exception was thrown, but "nozzle_type" has been around for at least 3 months now.
// So maybe this test and the serialization logic in Config.?pp should be deleted if it doesn't get used.
SCENARIO("DynamicPrintConfig serialization", "[Config]") {
    WHEN("DynamicPrintConfig is serialized and deserialized") {
        FullPrintConfig full_print_config;
        DynamicPrintConfig cfg;
        cfg.apply(full_print_config, false);

        std::string serialized;
        // try {
            std::ostringstream ss;
            cereal::BinaryOutputArchive oarchive(ss);
            oarchive(cfg);
            serialized = ss.str();
        // } catch (const std::runtime_error & /* e */) {
        //     // e.what();
        // }
	CAPTURE(serialized.length());

        THEN("Config object contains ini file options.") {
            DynamicPrintConfig cfg2;
            // try {
                std::stringstream ss(serialized);
                cereal::BinaryInputArchive iarchive(ss);
                iarchive(cfg2);
            // } catch (const std::runtime_error & /* e */) {
            //     // e.what();
            // }
	    CAPTURE(cfg.diff_report(cfg2));
            REQUIRE(cfg == cfg2);
        }
    }
}

SCENARIO("update_non_diff_values_to_base_config preserves child vectors when child has more extruders than parent",
         "[Config][Variant]") {
    GIVEN("A 2-extruder child printer config inheriting from a 1-extruder parent") {
        Slic3r::DynamicPrintConfig child;
        Slic3r::DynamicPrintConfig parent;

        child.set_key_value("nozzle_diameter",           new Slic3r::ConfigOptionFloats({0.4, 0.4}));
        child.set_key_value("printer_extruder_id",       new Slic3r::ConfigOptionInts({1, 2}));
        child.set_key_value("printer_extruder_variant",  new Slic3r::ConfigOptionStrings({"Direct Drive Standard", "Direct Drive Standard"}));
        child.set_key_value("retraction_length",         new Slic3r::ConfigOptionFloats({1.5, 1.5}));

        parent.set_key_value("nozzle_diameter",          new Slic3r::ConfigOptionFloats({0.4}));
        parent.set_key_value("printer_extruder_id",      new Slic3r::ConfigOptionInts({1}));
        parent.set_key_value("printer_extruder_variant", new Slic3r::ConfigOptionStrings({"Direct Drive Standard"}));
        parent.set_key_value("retraction_length",        new Slic3r::ConfigOptionFloats({0.8}));

        const Slic3r::t_config_option_keys keys = {
            "retraction_length", "printer_extruder_id", "printer_extruder_variant"
        };
        const std::set<std::string> different_keys = {
            "retraction_length", "printer_extruder_id", "printer_extruder_variant"
        };

        WHEN("update_non_diff_values_to_base_config is called") {
            std::string id_name  = "printer_extruder_id";
            std::string var_name = "printer_extruder_variant";
            child.update_non_diff_values_to_base_config(
                parent, keys, different_keys, id_name, var_name,
                Slic3r::printer_options_with_variant_1,
                Slic3r::printer_options_with_variant_2);

            THEN("printer_extruder_id retains size 2") {
                REQUIRE(child.option<Slic3r::ConfigOptionInts>("printer_extruder_id")->values.size() == 2);
            }
            THEN("printer_extruder_variant retains size 2") {
                REQUIRE(child.option<Slic3r::ConfigOptionStrings>("printer_extruder_variant")->values.size() == 2);
            }
            THEN("retraction_length retains size 2") {
                REQUIRE(child.option<Slic3r::ConfigOptionFloats>("retraction_length")->values.size() == 2);
            }
            THEN("printer_extruder_id values are preserved for both extruders") {
                auto* pe_id = child.option<Slic3r::ConfigOptionInts>("printer_extruder_id");
                REQUIRE(pe_id->values.size() == 2);
                REQUIRE(pe_id->values[0] == 1);
                REQUIRE(pe_id->values[1] == 2);
            }
        }
    }
}

SCENARIO("update_diff_values_to_child_config tolerates legacy machine-limit vector sizes",
         "[Config][Variant]") {
    // Regression: loading a user printer preset that inherits a non-BBL multi-extruder base and
    // overrides stride-2 machine limits used to throw in ConfigOptionVector::set_only_diff
    // ("invalid diff_index size"). The base's machine-limit vectors get length-extended by the
    // nozzle count while it carries no printer_extruder_variant, so the base length (nozzles*2)
    // no longer matches variant_index.size()*2. The throw was caught upstream and DELETED the
    // user's preset file. The merge must instead degrade gracefully.
    GIVEN("A 4-nozzle parent with stride-2 limits extended to nozzles*2 but no printer_extruder_variant") {
        Slic3r::DynamicPrintConfig parent;
        Slic3r::DynamicPrintConfig child;

        parent.set_key_value("nozzle_diameter",
            new Slic3r::ConfigOptionFloats({0.4, 0.4, 0.4, 0.4}));
        parent.set_key_value("machine_max_acceleration_x",
            new Slic3r::ConfigOptionFloats({25000, 25000, 25000, 25000, 25000, 25000, 25000, 25000}));

        // Child user preset declares 4 extruder variants and overrides the machine limit.
        child.set_key_value("printer_extruder_id",
            new Slic3r::ConfigOptionInts({1, 2, 3, 4}));
        child.set_key_value("printer_extruder_variant",
            new Slic3r::ConfigOptionStrings({"Direct Drive Standard", "Direct Drive Standard",
                                             "Direct Drive Standard", "Direct Drive Standard"}));
        child.set_key_value("machine_max_acceleration_x",
            new Slic3r::ConfigOptionFloats({8000, 8000, 8000, 8000, 8000, 8000, 8000, 8000}));

        WHEN("update_diff_values_to_child_config merges the child overrides") {
            std::string id_name  = "printer_extruder_id";
            std::string var_name = "printer_extruder_variant";

            THEN("it does not throw on the legacy size mismatch") {
                REQUIRE_NOTHROW(parent.update_diff_values_to_child_config(
                    child, id_name, var_name,
                    Slic3r::printer_options_with_variant_1,
                    Slic3r::printer_options_with_variant_2));

                AND_THEN("the child's overridden machine limit is preserved") {
                    auto* mx = parent.option<Slic3r::ConfigOptionFloats>("machine_max_acceleration_x");
                    REQUIRE(mx != nullptr);
                    REQUIRE(mx->values.size() >= 2);
                    REQUIRE_THAT(mx->values[0], Catch::Matchers::WithinAbs(8000.0, 1e-6));
                    REQUIRE_THAT(mx->values[1], Catch::Matchers::WithinAbs(8000.0, 1e-6));
                }
            }
        }
    }
}

TEST_CASE("A variant index comes from the same variant and id, else the id's first variant", "[Config][Variant]") {
    const std::vector<std::string> variant_list{"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard"};
    const std::vector<int>         variant_ids{1, 1, 2};

    SECTION("same variant and id") {
        CHECK(Slic3r::find_variant_index("Direct Drive High Flow", 1, variant_list, variant_ids) == 1);
        CHECK(Slic3r::find_variant_index("Direct Drive Standard", 2, variant_list, variant_ids) == 2);
    }
    SECTION("a variant the id lacks falls back to the id's first variant") {
        CHECK(Slic3r::find_variant_index("Bowden Standard", 1, variant_list, variant_ids) == 0);
        CHECK(Slic3r::find_variant_index("Direct Drive High Flow", 2, variant_list, variant_ids) == 2);
    }
    SECTION("an id with no variants matches none") {
        CHECK(Slic3r::find_variant_index("Direct Drive Standard", 3, variant_list, variant_ids) == -1);
    }
    SECTION("a negative id or a list without ids matches any id") {
        CHECK(Slic3r::find_variant_index("Direct Drive High Flow", -1, variant_list, variant_ids) == 1);
        CHECK(Slic3r::find_variant_index("Direct Drive High Flow", 2, variant_list, {}) == 1);
    }
    SECTION("a variant past a shorter id list gets no variant index") {
        CHECK(Slic3r::map_variant_indices(variant_list, {1}, variant_list, variant_ids) == std::vector<int>{0, -1, -1});
    }
    SECTION("a list without variant strings has one variant per id, and an empty one a single variant") {
        CHECK(Slic3r::map_variant_indices(variant_list, variant_ids, {}, {1, 2}) == std::vector<int>{0, 0, 1});
        CHECK(Slic3r::map_variant_indices(variant_list, variant_ids, {}, {}) == std::vector<int>{0, 0, 0});
    }
}

SCENARIO("update_diff_values_to_child_config keeps a child's values on variants it does not list",
         "[Config][Variant]") {
    std::set<std::string> no_keys;
    auto variants = [](std::initializer_list<std::string> names) { return new Slic3r::ConfigOptionStrings(names); };

    GIVEN("A filament parent with three variants") {
        Slic3r::DynamicPrintConfig parent;
        parent.set_key_value("filament_extruder_variant",
            variants({"Direct Drive Standard", "Bowden Standard", "Direct Drive High Flow"}));
        parent.set_deserialize_strict("nozzle_temperature", "220,220,220");

        WHEN("the child was saved when the parent had only its first variant") {
            Slic3r::DynamicPrintConfig child;
            child.set_key_value("filament_extruder_variant", variants({"Direct Drive Standard"}));
            child.set_deserialize_strict("nozzle_temperature", "199");
            parent.update_diff_values_to_child_config(child, "", "filament_extruder_variant",
                                                      Slic3r::filament_options_with_variant, no_keys);
            THEN("the child's value applies to every variant") {
                REQUIRE(parent.opt_serialize("nozzle_temperature") == "199,199,199");
            }
        }
        WHEN("the child lists every variant, in another order") {
            Slic3r::DynamicPrintConfig child;
            child.set_key_value("filament_extruder_variant",
                variants({"Bowden Standard", "Direct Drive High Flow", "Direct Drive Standard"}));
            child.set_deserialize_strict("nozzle_temperature", "190,205,199");
            parent.update_diff_values_to_child_config(child, "", "filament_extruder_variant",
                                                      Slic3r::filament_options_with_variant, no_keys);
            THEN("each variant keeps its own value") {
                REQUIRE(parent.opt_serialize("nozzle_temperature") == "199,190,205");
            }
        }
        WHEN("the child lists no variants") {
            Slic3r::DynamicPrintConfig child;
            child.set_deserialize_strict("nozzle_temperature", "199");
            parent.update_diff_values_to_child_config(child, "", "filament_extruder_variant",
                                                      Slic3r::filament_options_with_variant, no_keys);
            THEN("the child's value applies to every variant") {
                REQUIRE(parent.opt_serialize("nozzle_temperature") == "199,199,199");
            }
        }
    }

    GIVEN("A two-extruder printer parent with two variants per extruder") {
        Slic3r::DynamicPrintConfig parent;
        parent.set_key_value("printer_extruder_variant",
            variants({"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"}));
        parent.set_key_value("printer_extruder_id", new Slic3r::ConfigOptionInts({1, 1, 2, 2}));
        parent.set_deserialize_strict("retraction_length", "0.8,0.8,0.8,0.8");

        WHEN("the child lists only the Standard variant of each extruder") {
            Slic3r::DynamicPrintConfig child;
            child.set_key_value("printer_extruder_variant", variants({"Direct Drive Standard", "Direct Drive Standard"}));
            child.set_key_value("printer_extruder_id", new Slic3r::ConfigOptionInts({1, 2}));
            child.set_deserialize_strict("retraction_length", "1.1,2.2");
            parent.update_diff_values_to_child_config(child, "printer_extruder_id", "printer_extruder_variant",
                                                      Slic3r::printer_options_with_variant_1,
                                                      Slic3r::printer_options_with_variant_2);
            THEN("each extruder's High Flow variant takes that extruder's value") {
                REQUIRE(parent.opt_serialize("retraction_length") == "1.1,1.1,2.2,2.2");
            }
        }
        WHEN("the child lists no variants") {
            Slic3r::DynamicPrintConfig child;
            child.set_deserialize_strict("retraction_length", "1.1");
            parent.update_diff_values_to_child_config(child, "printer_extruder_id", "printer_extruder_variant",
                                                      Slic3r::printer_options_with_variant_1,
                                                      Slic3r::printer_options_with_variant_2);
            THEN("only the first extruder's variants take the child's value") {
                REQUIRE(parent.opt_serialize("retraction_length") == "1.1,1.1,0.8,0.8");
            }
        }
    }
}

SCENARIO("update_non_diff_values_to_base_config keeps a project's changed values on variants it does not list",
         "[Config][Variant]") {
    std::set<std::string> no_keys;
    auto variants = [](std::initializer_list<std::string> names) { return new Slic3r::ConfigOptionStrings(names); };

    GIVEN("A filament base with three variants") {
        Slic3r::DynamicPrintConfig base;
        base.set_key_value("filament_extruder_variant",
            variants({"Direct Drive Standard", "Bowden Standard", "Direct Drive High Flow"}));
        base.set_deserialize_strict("nozzle_temperature", "220,220,220");

        WHEN("the project was saved when the base had only its first variant") {
            Slic3r::DynamicPrintConfig project;
            project.set_key_value("filament_extruder_variant", variants({"Direct Drive Standard"}));
            project.set_deserialize_strict("nozzle_temperature", "199");

            AND_WHEN("the project lists the value as changed") {
                project.update_non_diff_values_to_base_config(base, project.keys(), {"nozzle_temperature"}, "", "filament_extruder_variant",
                                                              Slic3r::filament_options_with_variant, no_keys);
                THEN("the project's value applies to every variant") {
                    REQUIRE(project.opt_serialize("nozzle_temperature") == "199,199,199");
                }
            }
            AND_WHEN("the project does not list the value as changed") {
                project.update_non_diff_values_to_base_config(base, project.keys(), {}, "", "filament_extruder_variant",
                                                              Slic3r::filament_options_with_variant, no_keys);
                THEN("the base's values replace it") {
                    REQUIRE(project.opt_serialize("nozzle_temperature") == "220,220,220");
                }
            }
        }
        WHEN("the project lists every variant, in another order") {
            Slic3r::DynamicPrintConfig project;
            project.set_key_value("filament_extruder_variant",
                variants({"Bowden Standard", "Direct Drive High Flow", "Direct Drive Standard"}));
            project.set_deserialize_strict("nozzle_temperature", "190,205,199");
            project.update_non_diff_values_to_base_config(base, project.keys(), {"nozzle_temperature"}, "", "filament_extruder_variant",
                                                          Slic3r::filament_options_with_variant, no_keys);
            THEN("each variant keeps its own value") {
                REQUIRE(project.opt_serialize("nozzle_temperature") == "199,190,205");
            }
        }
        WHEN("the project lists no variants") {
            Slic3r::DynamicPrintConfig project;
            project.set_deserialize_strict("nozzle_temperature", "199");
            project.update_non_diff_values_to_base_config(base, project.keys(), {"nozzle_temperature"}, "", "filament_extruder_variant",
                                                          Slic3r::filament_options_with_variant, no_keys);
            THEN("the project's value applies to every variant") {
                REQUIRE(project.opt_serialize("nozzle_temperature") == "199,199,199");
            }
        }
    }

    GIVEN("A two-extruder printer base with two variants per extruder") {
        Slic3r::DynamicPrintConfig base;
        base.set_key_value("printer_extruder_variant",
            variants({"Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"}));
        base.set_key_value("printer_extruder_id", new Slic3r::ConfigOptionInts({1, 1, 2, 2}));
        base.set_deserialize_strict("retraction_length", "0.8,0.8,0.8,0.8");

        WHEN("the project lists only the Standard variant of each extruder") {
            Slic3r::DynamicPrintConfig project;
            project.set_key_value("printer_extruder_variant", variants({"Direct Drive Standard", "Direct Drive Standard"}));
            project.set_key_value("printer_extruder_id", new Slic3r::ConfigOptionInts({1, 2}));
            project.set_deserialize_strict("retraction_length", "1.1,2.2");
            project.update_non_diff_values_to_base_config(base, project.keys(), {"retraction_length"}, "printer_extruder_id", "printer_extruder_variant",
                                                          Slic3r::printer_options_with_variant_1,
                                                          Slic3r::printer_options_with_variant_2);
            THEN("each extruder's High Flow variant takes that extruder's value") {
                REQUIRE(project.opt_serialize("retraction_length") == "1.1,1.1,2.2,2.2");
            }
        }
        WHEN("the project lists only the Standard variant of each extruder, and the variant lists as changed") {
            Slic3r::DynamicPrintConfig project;
            project.set_key_value("printer_extruder_variant", variants({"Direct Drive Standard", "Direct Drive Standard"}));
            project.set_key_value("printer_extruder_id", new Slic3r::ConfigOptionInts({1, 2}));
            project.set_deserialize_strict("machine_max_speed_x", "300,100,400,150");
            base.set_deserialize_strict("machine_max_speed_x", "500,200,500,200,500,200,500,200");
            project.update_non_diff_values_to_base_config(base, project.keys(),
                                                          {"machine_max_speed_x", "printer_extruder_id", "printer_extruder_variant"},
                                                          "printer_extruder_id", "printer_extruder_variant",
                                                          Slic3r::printer_options_with_variant_1,
                                                          Slic3r::printer_options_with_variant_2);
            THEN("the variant lists are the base's") {
                REQUIRE(project.opt_serialize("printer_extruder_variant") == base.opt_serialize("printer_extruder_variant"));
                REQUIRE(project.opt_serialize("printer_extruder_id") == "1,1,2,2");
            }
            THEN("each extruder's High Flow variant takes that extruder's pair of limits") {
                REQUIRE(project.opt_serialize("machine_max_speed_x") == "300,100,300,100,400,150,400,150");
            }
        }
        WHEN("the project lists no variants") {
            Slic3r::DynamicPrintConfig project;
            project.set_deserialize_strict("retraction_length", "1.1");
            project.update_non_diff_values_to_base_config(base, project.keys(), {"retraction_length"}, "printer_extruder_id", "printer_extruder_variant",
                                                          Slic3r::printer_options_with_variant_1,
                                                          Slic3r::printer_options_with_variant_2);
            THEN("only the first extruder's variants take the project's value") {
                REQUIRE(project.opt_serialize("retraction_length") == "1.1,1.1,0.8,0.8");
            }
        }
    }
}

// SCENARIO("DynamicPrintConfig JSON serialization", "[Config]") {
//     WHEN("DynamicPrintConfig is serialized and deserialized") {
// 	auto now = std::chrono::high_resolution_clock::now();
// 	auto timestamp = now.time_since_epoch().count();
// 	std::stringstream ss;
// 	ss << "catch_test_serialization_" << timestamp << ".json";
// 	std::string filename = (fs::temp_directory_path() / ss.str()).string();

// TODO: Finish making a unit test for JSON serialization
//         FullPrintConfig full_print_config;
//         DynamicPrintConfig cfg;
//         cfg.apply(full_print_config, false);

//         std::string serialized;
//         try {
//             std::ostringstream ss;
//             cereal::BinaryOutputArchive oarchive(ss);
//             oarchive(cfg);
//             serialized = ss.str();
//         } catch (const std::runtime_error & /* e */) {
//             // e.what();
//         }
// 	CAPTURE(serialized.length());

//         THEN("Config object contains ini file options.") {
//             DynamicPrintConfig cfg2;
//             try {
//                 std::stringstream ss(serialized);
//                 cereal::BinaryInputArchive iarchive(ss);
//                 iarchive(cfg2);
//             } catch (const std::runtime_error & /* e */) {
//                 // e.what();
//             }
// 	    CAPTURE(cfg.diff_report(cfg2));
//             REQUIRE(cfg == cfg2);
//         }
//     }
// }

TEST_CASE("save_to_json round-trips plugin capability references as strings", "[Config][plugins]") {
    ScopedTemporaryFile tmp(".json");
    const std::vector<std::string> refs = {
        "local_plugin;;inset",
        "cloud_plugin;550e8400-e29b-41d4-a716-446655440000;inset"
    };

    std::unique_ptr<DynamicPrintConfig> config_ptr(
        DynamicPrintConfig::new_from_defaults_keys({"slicing_pipeline_plugin"}));
    DynamicPrintConfig config = std::move(*config_ptr);
    config.option<ConfigOptionStrings>("slicing_pipeline_plugin", true)->values = refs;
    config.save_to_json(tmp.string(), "test_preset", "User", "1.0.0.0");

    nlohmann::json j;
    {
        boost::nowide::ifstream ifs(tmp.string());
        ifs >> j;
    }
    REQUIRE(j["slicing_pipeline_plugin"] == nlohmann::json(refs));
    CHECK_FALSE(j.contains("plugins"));

    DynamicPrintConfig reloaded = DynamicPrintConfig::full_print_config();
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    std::map<std::string, std::string> key_values;
    std::string reason;
    REQUIRE(reloaded.load_from_json(tmp.string(), substitutions, true, key_values, reason) == 0);
    CHECK(reason.empty());
    CHECK(reloaded.option<ConfigOptionStrings>("slicing_pipeline_plugin")->values == refs);
}

TEST_CASE("load_from_json hands a preset's include list to the caller instead of the config", "[Config]") {
    ScopedTemporaryFile tmp(".json");
    {
        boost::nowide::ofstream ofs(tmp.string());
        ofs << R"({"type":"machine","name":"P","instantiation":"true","include":["T start","T end"],"machine_end_gcode":"M84"})";
    }
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    std::map<std::string, std::string> key_values;
    std::string reason;
    REQUIRE(config.load_from_json(tmp.string(), substitutions, false, key_values, reason) == 0);
    CHECK(reason.empty());
    CHECK(key_values["include"] == R"(["T start","T end"])");
    CHECK_FALSE(config.has("include"));
    CHECK(substitutions.unrecogized_keys.empty());
    CHECK(config.opt_string("machine_end_gcode") == "M84");
}

TEST_CASE("save_to_json writes the same document to a stream as to a file", "[Config]") {
    DynamicPrintConfig config;
    config.set_key_value("layer_height", new ConfigOptionFloat(0.2));
    config.set_key_value("wall_loops", new ConfigOptionInt(3));
    config.set_key_value("filament_type", new ConfigOptionStrings({ "PLA", "PETG" }));
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28\nG1 Z5"));

    ScopedTemporaryFile tmp(".json");
    config.save_to_json(tmp.string(), "test_preset", "User", "1.0.0.0");
    std::string file_contents;
    {
        boost::nowide::ifstream ifs(tmp.string());
        file_contents.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());
    }
    // The file format: one tab per nesting level and a trailing newline.
    REQUIRE_FALSE(file_contents.empty());
    CHECK(file_contents.rfind("{\n\t\"", 0) == 0);
    CHECK(file_contents.back() == '\n');

    std::ostringstream strict, replaced;
    config.save_to_json(strict, "test_preset", "User", "1.0.0.0");
    config.save_to_json(replaced, "test_preset", "User", "1.0.0.0", true);
    CHECK(strict.str() == file_contents);
    CHECK(replaced.str() == file_contents);
    CHECK(nlohmann::json::parse(strict.str())["machine_start_gcode"] == "G28\nG1 Z5");
}

TEST_CASE("save_to_json replaces invalid UTF-8 in a stream only when asked", "[Config]") {
    DynamicPrintConfig config;
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28 ; \xff"));

    std::ostringstream strict, replaced;
    CHECK_THROWS_AS(config.save_to_json(strict, "test_preset", "User", "1.0.0.0"), nlohmann::json::type_error);
    REQUIRE_NOTHROW(config.save_to_json(replaced, "test_preset", "User", "1.0.0.0", true));
    CHECK(nlohmann::json::parse(replaced.str())["machine_start_gcode"] == "G28 ; \xEF\xBF\xBD");
}

TEST_CASE("save_to_json leaves an existing file untouched when the config cannot be serialized", "[Config]") {
    DynamicPrintConfig config;
    config.set_key_value("machine_start_gcode", new ConfigOptionString("G28 ; \xff"));

    ScopedTemporaryFile tmp(".json");
    {
        boost::nowide::ofstream ofs(tmp.string());
        ofs << "previous";
    }
    CHECK_THROWS_AS(config.save_to_json(tmp.string(), "test_preset", "User", "1.0.0.0"), nlohmann::json::type_error);

    boost::nowide::ifstream ifs(tmp.string());
    const std::string contents((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
    CHECK(contents == "previous");
}

TEST_CASE("plugin capability references survive string-map serialization", "[Config][plugins]") {
    const std::vector<std::string> refs = {
        "master_plugin;;header-stamp",
        "Sample Plugin;1f998ea9-0183-4cc5-957f-4eef659ba4e6;G-code Benchmark (.py)"
    };

    DynamicPrintConfig original = DynamicPrintConfig::full_print_config();
    original.option<ConfigOptionStrings>("slicing_pipeline_plugin", true)->values = refs;

    std::map<std::string, std::string> serialized{
        {"slicing_pipeline_plugin", original.option<ConfigOptionStrings>("slicing_pipeline_plugin")->serialize()}
    };
    CHECK(serialized["slicing_pipeline_plugin"].find("\"master_plugin;;header-stamp\"") != std::string::npos);

    DynamicPrintConfig reloaded = DynamicPrintConfig::full_print_config();
    reloaded.load_string_map(serialized, ForwardCompatibilitySubstitutionRule::Disable);

    CHECK(reloaded.option<ConfigOptionStrings>("slicing_pipeline_plugin")->values == refs);
}

TEST_CASE("parse_capability_ref parses local and cloud references", "[Config][plugin]") {
    const auto local = Slic3r::parse_capability_ref("local_plugin;;post_process");
    REQUIRE(local.has_value());
    CHECK(local->name == "local_plugin");
    CHECK(local->capability_name == "post_process");
    CHECK(local->uuid.empty());

    const auto cloud = Slic3r::parse_capability_ref(
        "cloud_plugin;550e8400-e29b-41d4-a716-446655440000;post_process");
    REQUIRE(cloud.has_value());
    CHECK(cloud->name == "cloud_plugin");
    CHECK(cloud->capability_name == "post_process");
    CHECK(cloud->uuid == "550e8400-e29b-41d4-a716-446655440000");
}

TEST_CASE("parse_capability_ref rejects malformed input", "[Config][plugin]") {
    CHECK_FALSE(Slic3r::parse_capability_ref("").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref("plugin").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref("plugin;uuid").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref(";;capability").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref(";uuid;capability").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref("plugin;;").has_value());
    CHECK_FALSE(Slic3r::parse_capability_ref("plugin;uuid;").has_value());
}

namespace {
// Installs a stub capability resolver that echoes the capability type into the reference, so tests
// can assert each plugin-backed option resolved with its own ConfigOptionDef::plugin_type. Resets
// the global resolver on teardown -- tests run in random order and other cases assert the
// no-resolver behavior (an absent "plugins" manifest).
struct PluginResolverFixture {
    PluginResolverFixture() {
        ConfigBase::set_resolve_capability_fn([](const std::string& name, const std::string& type) {
            return name.empty() ? std::string() : name + ";;" + type;
        });
    }
    ~PluginResolverFixture() { ConfigBase::set_resolve_capability_fn(nullptr); }
};
} // namespace

TEST_CASE_METHOD(PluginResolverFixture,
    "update_plugin_manifest derives references generically from plugin-backed options",
    "[Config][plugins]") {
    // Both scalar (printer_agent) and vector (slicing_pipeline_plugin) options opt in via a non-empty
    // ConfigOptionDef::plugin_type (is_plugin_backed) and are resolved with it -- there is no hardcoded
    // per-option switch. printer_agent in particular relies on its plugin_type metadata being wired up
    // (it is edited via a dedicated widget, not the plugin_picker).
    std::unique_ptr<DynamicPrintConfig> config_ptr(DynamicPrintConfig::new_from_defaults_keys(
        {"slicing_pipeline_plugin", "printer_agent"}));
    DynamicPrintConfig config = std::move(*config_ptr);
    config.option<ConfigOptionStrings>("slicing_pipeline_plugin", true)->values = {"sp"};
    config.option<ConfigOptionString>("printer_agent", true)->value            = "agent";

    config.update_plugin_manifest();
    const std::vector<std::string> manifest = config.option<ConfigOptionStrings>("plugins")->values;

    using Catch::Matchers::VectorContains;
    REQUIRE_THAT(manifest, VectorContains(std::string("sp;;slicing-pipeline")));
    REQUIRE_THAT(manifest, VectorContains(std::string("agent;;printer-connection")));
    CHECK(manifest.size() == 2);
}

TEST_CASE_METHOD(PluginResolverFixture,
    "update_plugin_manifest de-duplicates references and skips unset options",
    "[Config][plugins]") {
    std::unique_ptr<DynamicPrintConfig> config_ptr(DynamicPrintConfig::new_from_defaults_keys(
        {"slicing_pipeline_plugin", "printer_agent"}));
    DynamicPrintConfig config = std::move(*config_ptr);
    config.option<ConfigOptionStrings>("slicing_pipeline_plugin", true)->values = {"x", "x"};  // duplicate
    // printer_agent stays at its default empty value -> contributes nothing to the manifest.

    config.update_plugin_manifest();
    const std::vector<std::string> manifest = config.option<ConfigOptionStrings>("plugins")->values;

    CHECK(manifest == std::vector<std::string>{"x;;slicing-pipeline"});
}

TEST_CASE("H2C/A2L-era multi-nozzle and pre-heat config keys exist", "[config]") {
    // Foundation keys backing H2C 6-nozzle cluster grouping, the pre-heat/pre-cool time
    // model, and wipe-tower nozzle-change handling. Defaults must keep existing
    // single-nozzle printers behaving identically.
    Slic3r::DynamicPrintConfig config = Slic3r::DynamicPrintConfig::full_print_config();

    // Printer / per-extruder options
    REQUIRE(config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count") != nullptr);
    REQUIRE(config.option<ConfigOptionIntsNullable>("extruder_max_nozzle_count")->values == std::vector<int>{1});
    REQUIRE(config.option<ConfigOptionBool>("enable_pre_heating") != nullptr);
    REQUIRE(config.option<ConfigOptionBool>("enable_pre_heating")->value == false);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("hotend_cooling_rate") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("hotend_heating_rate") != nullptr);
    REQUIRE(config.option<ConfigOptionFloat>("machine_hotend_change_time") != nullptr);
    REQUIRE(config.option<ConfigOptionFloat>("machine_prepare_compensation_time") != nullptr);

    // Filament pre-cooling / ramming / nozzle-change (nc) options
    REQUIRE(config.option<ConfigOptionIntsNullable>("filament_pre_cooling_temperature") != nullptr);
    REQUIRE(config.option<ConfigOptionIntsNullable>("filament_pre_cooling_temperature_nc") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_preheat_temperature_delta") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_retract_length_nc") != nullptr);
    REQUIRE(config.option<ConfigOptionFloats>("filament_change_length_nc") != nullptr);
    REQUIRE(config.option<ConfigOptionFloats>("filament_prime_volume_nc") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_ramming_travel_time") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_ramming_travel_time_nc") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_ramming_volumetric_speed") != nullptr);
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_ramming_volumetric_speed_nc") != nullptr);

    // Spot-check defaults that must not alter existing behavior.
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_retract_length_nc")->values == std::vector<double>{10.});
    REQUIRE(config.option<ConfigOptionFloats>("filament_prime_volume_nc")->values == std::vector<double>{60.});
    REQUIRE(config.option<ConfigOptionIntsNullable>("filament_pre_cooling_temperature_nc")->values == std::vector<int>{0});
    REQUIRE(config.option<ConfigOptionFloatsNullable>("filament_ramming_volumetric_speed")->values == std::vector<double>{-1});
}

SCENARIO("ConfigOptionVector::set_to_index with stride=1 copies values correctly", "[Config][set_to_index]") {
    GIVEN("A destination vector and a source vector with 3 values") {
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionFloats src({10.0, 20.0, 30.0});
        std::vector<int> variant_index = {0, 1, 2};
        int stride = 1;

        WHEN("set_to_index is called with stride=1") {
            dest.set_to_index(&src, variant_index, stride);

            THEN("The destination contains the source values") {
                REQUIRE(dest.values.size() == 3);
                REQUIRE(dest.values[0] == 10.0);
                REQUIRE(dest.values[1] == 20.0);
                REQUIRE(dest.values[2] == 30.0);
            }
        }
    }

    GIVEN("A destination vector and a source vector with subset mapping") {
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionFloats src({100.0, 200.0, 300.0});
        std::vector<int> variant_index = {1, 2};
        int stride = 1;

        WHEN("set_to_index maps only indices 1 and 2") {
            dest.set_to_index(&src, variant_index, stride);

            THEN("Only the mapped values are copied, default fills the others") {
                REQUIRE(dest.values.size() == 2);
                REQUIRE(dest.values[0] == 200.0);
                REQUIRE(dest.values[1] == 300.0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index with stride=2 copies grouped values correctly", "[Config][set_to_index]") {
    GIVEN("A destination vector and a source vector with stride=2 (e.g., nozzle groups)") {
        // Source has 4 groups of 2 values each: (10,11), (20,21), (30,31), (40,41)
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionFloats src({10.0, 11.0, 20.0, 21.0, 30.0, 31.0, 40.0, 41.0});
        int stride = 2;

        WHEN("set_to_index maps groups 0, 1, 3") {
            std::vector<int> variant_index = {0, 1, 3};
            dest.set_to_index(&src, variant_index, stride);

            THEN("The destination has 3 groups (6 values) mapped correctly") {
                REQUIRE(dest.values.size() == 6);
                // Group 0: (10, 11)
                REQUIRE(dest.values[0] == 10.0);
                REQUIRE(dest.values[1] == 11.0);
                // Group 1: (20, 21)
                REQUIRE(dest.values[2] == 20.0);
                REQUIRE(dest.values[3] == 21.0);
                // Group 3: (40, 41)
                REQUIRE(dest.values[4] == 40.0);
                REQUIRE(dest.values[5] == 41.0);
            }
        }
    }

    GIVEN("A destination and a single-group source") {
        Slic3r::ConfigOptionFloats dest({0.0});
        // Source has 1 group of 2 values
        Slic3r::ConfigOptionFloats src({50.0, 60.0});
        int stride = 2;

        WHEN("set_to_index maps group 0 from a single-group source") {
            std::vector<int> variant_index = {0};
            dest.set_to_index(&src, variant_index, stride);

            THEN("The destination contains the single group correctly") {
                REQUIRE(dest.values.size() == 2);
                REQUIRE(dest.values[0] == 50.0);
                REQUIRE(dest.values[1] == 60.0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index handles empty dest_index", "[Config][set_to_index]") {
    GIVEN("A destination and source with stride=2") {
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionFloats src({10.0, 11.0, 20.0, 21.0});
        std::vector<int> variant_index = {};
        int stride = 2;

        WHEN("set_to_index is called with an empty index vector") {
            dest.set_to_index(&src, variant_index, stride);

            THEN("The destination is resized to 0") {
                REQUIRE(dest.values.size() == 0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index handles nil values in source", "[Config][set_to_index]") {
    GIVEN("A source with a nil group (stride=2)") {
        Slic3r::ConfigOptionFloatsNullable dest({0.0});
        Slic3r::ConfigOptionFloatsNullable src({10.0, 11.0,
            Slic3r::ConfigOptionFloatsNullable::nil_value(), Slic3r::ConfigOptionFloatsNullable::nil_value(),
            30.0, 31.0});
        int stride = 2;

        WHEN("set_to_index maps all groups including the nil one") {
            std::vector<int> variant_index = {0, 1, 2};
            dest.set_to_index(&src, variant_index, stride);

            THEN("Non-nil groups are copied and the nil group keeps the default") {
                REQUIRE(dest.values.size() == 6);
                // Group 0: (10, 11) — copied
                REQUIRE(dest.values[0] == 10.0);
                REQUIRE(dest.values[1] == 11.0);
                // Group 1: nil — keeps default (the front value = 10.0)
                REQUIRE(dest.values[2] == 10.0);
                REQUIRE(dest.values[3] == 10.0);
                // Group 2: (30, 31) — copied
                REQUIRE(dest.values[4] == 30.0);
                REQUIRE(dest.values[5] == 31.0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index handles out-of-bounds dest_index", "[Config][set_to_index]") {
    GIVEN("A source with only 2 groups (4 values) but dest_index references group 3") {
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionFloats src({10.0, 11.0, 20.0, 21.0}); // 2 groups of stride 2
        int stride = 2;

        WHEN("set_to_index maps group 3 which is out of bounds") {
            std::vector<int> variant_index = {0, 3}; // group 3 is out of range
            dest.set_to_index(&src, variant_index, stride);

            THEN("Group 0 is copied, group 3 falls back to default without crashing") {
                REQUIRE(dest.values.size() == 4);
                // Group 0: (10, 11) — copied
                REQUIRE(dest.values[0] == 10.0);
                REQUIRE(dest.values[1] == 11.0);
                // Group 3: out of bounds — keeps default (10.0 = src.values.front())
                REQUIRE(dest.values[2] == 10.0);
                REQUIRE(dest.values[3] == 10.0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index handles negative dest_index values", "[Config][set_to_index]") {
    GIVEN("A destination and source with a negative entry in dest_index") {
        // The dest is initially empty, so resize fills all slots with src.values.front().
        Slic3r::ConfigOptionFloats dest;
        Slic3r::ConfigOptionFloats src({100.0, 101.0, 200.0, 201.0});
        int stride = 2;

        WHEN("set_to_index maps group 0 and a negative index") {
            std::vector<int> variant_index = {-1, 0};
            dest.set_to_index(&src, variant_index, stride);

            THEN("The negative index is skipped, the valid group is copied") {
                REQUIRE(dest.values.size() == 4);
                // Position 0 (variant_index[0] = -1): skipped, keeps default fill
                // from resize (src.values.front() = 100.0, applied to all new elements)
                REQUIRE(dest.values[0] == 100.0);
                REQUIRE(dest.values[1] == 100.0);
                // Position 1 (variant_index[1] = 0): copied from group 0 of src
                REQUIRE(dest.values[2] == 100.0);
                REQUIRE(dest.values[3] == 101.0);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index handles single-element groups with stride=1", "[Config][set_to_index]") {
    GIVEN("A destination re-mapping one variant index with a stride=1 source") {
        // Simulates the PrintObject.cpp code path: stride=1, variant_index={1}
        Slic3r::ConfigOptionFloats dest({99.0, 99.0, 99.0, 99.0}); // pre-sized for 4 extruders
        Slic3r::ConfigOptionFloats src({0.5, 0.6, 0.7, 0.8});      // 4 extruder values
        std::vector<int> variant_index = {1}; // only extruder 1 is active
        int stride = 1;

        WHEN("set_to_index is called") {
            dest.set_to_index(&src, variant_index, stride);

            THEN("Only the mapped value is copied, rest are defaulted") {
                REQUIRE(dest.values.size() == 1);
                REQUIRE(dest.values[0] == 0.6);
            }
        }
    }
}

SCENARIO("ConfigOptionVector::set_to_index throws on incompatible type", "[Config][set_to_index]") {
    GIVEN("A Floats destination and an Ints source") {
        Slic3r::ConfigOptionFloats dest({0.0});
        Slic3r::ConfigOptionInts src({1, 2, 3});
        std::vector<int> variant_index = {0};
        int stride = 1;

        WHEN("set_to_index is called with mismatched types") {
            THEN("A ConfigurationError is thrown") {
                REQUIRE_THROWS_AS(dest.set_to_index(&src, variant_index, stride), Slic3r::ConfigurationError);
            }
        }
    }
}

TEST_CASE("read_cli applies valid values and collects non-option arguments", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--nozzle-temperature", "210,190", "--reduce-crossing-wall=1", "model.3mf"};
    REQUIRE(config.read_cli(5, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionInts>("nozzle_temperature")->values == std::vector<int>{210, 190});
    REQUIRE(config.opt<ConfigOptionBool>("reduce_crossing_wall")->value);
    REQUIRE(extra == t_config_option_keys{"model.3mf"});
    REQUIRE(keys == t_config_option_keys{"nozzle_temperature", "reduce_crossing_wall"});
}

TEST_CASE("read_cli rejects nil for a non-nullable vector option", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--nozzle-temperature", "nil"};
    REQUIRE_FALSE(config.read_cli(3, argv, &extra, &keys));
}

TEST_CASE("read_cli rejects an invalid boolean value", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--reduce-crossing-wall=maybe"};
    REQUIRE_FALSE(config.read_cli(2, argv, &extra, &keys));
}

TEST_CASE("read_cli accepts the common spellings of a boolean value", "[Config]") {
    const auto [text, expected] = GENERATE(table<const char*, bool>({
        {"--reduce-crossing-wall=1", true},
        {"--reduce-crossing-wall=true", true},
        {"--reduce-crossing-wall=Yes", true},
        {"--reduce-crossing-wall=on", true},
        {"--reduce-crossing-wall=enabled", true},
        {"--reduce-crossing-wall=TRUE", true},
        {"--reduce-crossing-wall=oN", true},
        {"--reduce-crossing-wall=0", false},
        {"--reduce-crossing-wall=false", false},
        {"--reduce-crossing-wall=No", false},
        {"--reduce-crossing-wall=off", false},
        {"--reduce-crossing-wall=disabled", false},
        {"--reduce-crossing-wall=FALSE", false},
        {"--reduce-crossing-wall=DiSaBlEd", false},
    }));

    DYNAMIC_SECTION(text) {
        Slic3r::DynamicPrintConfig config;
        t_config_option_keys extra, keys;
        const char* argv[] = {"orca-slicer", text};
        REQUIRE(config.read_cli(2, argv, &extra, &keys));
        REQUIRE(config.opt<ConfigOptionBool>("reduce_crossing_wall")->value == expected);
    }
}

TEST_CASE("read_cli accepts the common boolean spellings inside a bools vector", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=true,no,1"};
    REQUIRE(config.read_cli(2, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{1, 0, 1});
}

TEST_CASE("read_cli trims whitespace around boolean spellings", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--reduce-crossing-wall= true ", "--filament-soluble= true , no ,1"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBool>("reduce_crossing_wall")->value);
    REQUIRE(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{1, 0, 1});
}

TEST_CASE("read_cli normalizes boolean spellings when a bools vector is repeated", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=true", "--filament-soluble=off"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{1, 0});
}

TEST_CASE("read_cli keeps nil alongside boolean spellings in a nullable bools vector", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--enable-overhang-speed=nil,yes,off"};
    REQUIRE(config.read_cli(2, argv, &extra, &keys));
    auto* opt = config.opt<ConfigOptionBoolsNullable>("enable_overhang_speed");
    REQUIRE(opt != nullptr);
    REQUIRE(opt->values.size() == 3);
    REQUIRE(opt->is_nil(0));
    REQUIRE(opt->values[1] == 1);
    REQUIRE(opt->values[2] == 0);
}

TEST_CASE("read_cli rejects an empty item inside a bools vector", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=true,,1"};
    REQUIRE_FALSE(config.read_cli(2, argv, &extra, &keys));
}

TEST_CASE("read_cli rejects an unknown spelling next to a valid one in a bools vector", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=true,affirmative"};
    REQUIRE_FALSE(config.read_cli(2, argv, &extra, &keys));
}

// The normalization lives in read_cli's boolean branches, so options of other types keep the
// value verbatim - a path named "on" or a colour named "true" must not turn into "1".
TEST_CASE("read_cli leaves boolean spellings alone for non-boolean options", "[Config]") {
    SECTION("string option") {
        Slic3r::DynamicPrintAndCLIConfig config;
        t_config_option_keys extra, keys;
        const char* argv[] = {"orca-slicer", "--logfile=true"};
        REQUIRE(config.read_cli(2, argv, &extra, &keys));
        REQUIRE(config.opt<ConfigOptionString>("logfile")->value == "true");
    }
    SECTION("strings vector option") {
        Slic3r::DynamicPrintConfig config;
        t_config_option_keys extra, keys;
        const char* argv[] = {"orca-slicer", "--filament-colour=on;off"};
        REQUIRE(config.read_cli(2, argv, &extra, &keys));
        REQUIRE(config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"on", "off"});
    }
}

TEST_CASE("read_cli treats a bare boolean flag as true without consuming the next argument", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--reduce-crossing-wall", "model.3mf"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBool>("reduce_crossing_wall")->value);
    REQUIRE(extra == t_config_option_keys{"model.3mf"});
}

TEST_CASE("read_cli rejects an invalid scalar numeric value", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--top-shell-layers", "several"};
    REQUIRE_FALSE(config.read_cli(3, argv, &extra, &keys));
}

TEST_CASE("read_cli appends values when a vector option is repeated", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--nozzle-temperature", "210", "--nozzle-temperature", "190,200"};
    REQUIRE(config.read_cli(5, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionInts>("nozzle_temperature")->values == std::vector<int>{210, 190, 200});
    // the key is recorded once, on first use
    REQUIRE(keys == t_config_option_keys{"nozzle_temperature"});
}

TEST_CASE("read_cli parses a bools vector given in the --flag=values form", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=1,0,1"};
    REQUIRE(config.read_cli(2, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{1, 0, 1});
}

TEST_CASE("read_cli rejects an invalid value inside a bools vector", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble=1,maybe"};
    REQUIRE_FALSE(config.read_cli(2, argv, &extra, &keys));
}

TEST_CASE("read_cli appends true for a bare bools vector flag", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-soluble"};
    REQUIRE(config.read_cli(2, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{1});
}

TEST_CASE("read_cli splits a strings vector on semicolons and unescapes quoted items", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-colour", "#FF0000;\"a\\nb\";#00FF00"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    auto& values = config.opt<ConfigOptionStrings>("filament_colour")->values;
    REQUIRE(values == std::vector<std::string>{"#FF0000", "a\nb", "#00FF00"});
}

TEST_CASE("read_cli rejects a strings vector with an unterminated quote", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-colour", "\"oops"};
    REQUIRE_FALSE(config.read_cli(3, argv, &extra, &keys));
}

TEST_CASE("read_cli parses a points vector in the NxM coordinate form", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--printable-area", "0x0,200x0,200x200,0x200"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    auto& points = config.opt<ConfigOptionPoints>("printable_area")->values;
    REQUIRE(points.size() == 4);
    REQUIRE_THAT(points[1].x(), Catch::Matchers::WithinAbs(200.0, 1e-9));
    REQUIRE_THAT(points[1].y(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(points[3].x(), Catch::Matchers::WithinAbs(0.0, 1e-9));
    REQUIRE_THAT(points[3].y(), Catch::Matchers::WithinAbs(200.0, 1e-9));
}

// logfile is a CLI-only option, so it needs the config type whose def pulls in cli_misc_config_def.
TEST_CASE("read_cli stores the log file path as a string", "[Config]") {
    Slic3r::DynamicPrintAndCLIConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--logfile", "orca.log"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    REQUIRE(config.opt<ConfigOptionString>("logfile")->value == "orca.log");
}

TEST_CASE("read_cli accepts nil entries for a nullable vector option", "[Config]") {
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {"orca-slicer", "--filament-retraction-length", "nil,2.5"};
    REQUIRE(config.read_cli(3, argv, &extra, &keys));
    auto* opt = config.opt<ConfigOptionFloatsNullable>("filament_retraction_length");
    REQUIRE(opt != nullptr);
    REQUIRE(opt->values.size() == 2);
    REQUIRE(opt->is_nil(0));
    REQUIRE_FALSE(opt->is_nil(1));
    REQUIRE_THAT(opt->values[1], Catch::Matchers::WithinAbs(2.5, 1e-9));
}

// get_at() returns values.front() for an out-of-range index, so calling it on an empty vector
// option is UB. filament_id and filament_is_support are unpopulated on a CLI from-scratch slice.
TEST_CASE("get_filament_type treats empty vector options as absent", "[Config][Filament]")
{
    DynamicPrintConfig config;
    std::string displayed;

    SECTION("an empty filament_type yields no type at all")
    {
        config.set_key_value("filament_type", new ConfigOptionStrings());
        REQUIRE(config.get_filament_type(displayed, 0) == "");
    }

    SECTION("an empty filament_is_support falls back to the plain filament type")
    {
        config.set_key_value("filament_type", new ConfigOptionStrings({"PETG"}));
        config.set_key_value("filament_is_support", new ConfigOptionBools());
        REQUIRE(config.get_filament_type(displayed, 0) == "PETG");
        REQUIRE(displayed == "PETG");
    }

    SECTION("a support filament with an empty filament_id resolves from the type alone")
    {
        config.set_key_value("filament_type", new ConfigOptionStrings({"PLA"}));
        config.set_key_value("filament_is_support", new ConfigOptionBools({true}));
        config.set_key_value("filament_id", new ConfigOptionStrings());
        REQUIRE(config.get_filament_type(displayed, 0) == "PLA-S");
        REQUIRE(displayed == "Sup.PLA");
    }

    SECTION("a populated filament_id still selects the support type by id")
    {
        config.set_key_value("filament_type", new ConfigOptionStrings({"PETG"}));
        config.set_key_value("filament_is_support", new ConfigOptionBools({true}));
        config.set_key_value("filament_id", new ConfigOptionStrings({"GFS00"}));
        REQUIRE(config.get_filament_type(displayed, 0) == "PLA-S");
        REQUIRE(displayed == "Sup.PLA");
    }
}

namespace {

// min_object_distance reads exactly these three options.
DynamicPrintConfig spacing_config(PrinterTechnology tech, PrintSequence seq, double clearance_radius)
{
    DynamicPrintConfig c;
    c.set_key_value("printer_technology", new ConfigOptionEnum<PrinterTechnology>(tech));
    c.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(seq));
    c.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(clearance_radius));
    return c;
}

} // namespace

TEST_CASE("min_object_distance floors object spacing per print sequence", "[Config]")
{
    struct Case
    {
        std::string       description;
        PrinterTechnology tech;
        PrintSequence     sequence;
        double            clearance_radius;
        double            expected;
    };

    auto c = GENERATE(values<Case>({
        {"sequential FFF takes a clearance radius above the floor", ptFFF, PrintSequence::ByObject, 12., 12.},
        {"sequential FFF holds the floor at the radius",            ptFFF, PrintSequence::ByObject,  6.,  6.},
        {"sequential FFF holds the floor below the radius",         ptFFF, PrintSequence::ByObject,  4.,  6.},
        {"layered FFF ignores the clearance radius",                ptFFF, PrintSequence::ByLayer,  12.,  6.},
        {"SLA is a flat 6mm",                                       ptSLA, PrintSequence::ByObject, 12.,  6.},
        {"SLA ignores the print sequence too",                      ptSLA, PrintSequence::ByLayer,  12.,  6.},
    }));

    DYNAMIC_SECTION(c.description)
    {
        CHECK_THAT(min_object_distance(spacing_config(c.tech, c.sequence, c.clearance_radius)),
                   Catch::Matchers::WithinAbs(c.expected, 1e-9));
    }
}

TEST_CASE("min_object_distance yields no floor when an FFF config lacks the options", "[Config]")
{
    // Missing options yield 0 rather than an error, so a caller gets no floor at all.
    SECTION("no clearance radius") {
        DynamicPrintConfig c;
        c.set_key_value("printer_technology", new ConfigOptionEnum<PrinterTechnology>(ptFFF));
        c.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByObject));
        CHECK_THAT(min_object_distance(c), Catch::Matchers::WithinAbs(0., 1e-9));
    }

    SECTION("no print sequence") {
        DynamicPrintConfig c;
        c.set_key_value("printer_technology", new ConfigOptionEnum<PrinterTechnology>(ptFFF));
        c.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(12.));
        CHECK_THAT(min_object_distance(c), Catch::Matchers::WithinAbs(0., 1e-9));
    }

    SECTION("nothing at all") {
        CHECK_THAT(min_object_distance(DynamicPrintConfig{}), Catch::Matchers::WithinAbs(0., 1e-9));
    }

    SECTION("an unset printer technology is treated as FFF") {
        DynamicPrintConfig c;
        c.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByObject));
        c.set_key_value("extruder_clearance_radius", new ConfigOptionFloat(12.));
        CHECK_THAT(min_object_distance(c), Catch::Matchers::WithinAbs(12., 1e-9));
    }
}

TEST_CASE("Static print configs compare, order and hash by their option values", "[Config]")
{
    // PrintObjectConfig comes from PRINT_CONFIG_CLASS_DEFINE; PrintConfig combines MachineEnvelopeConfig
    // and GCodeConfig through PRINT_CONFIG_CLASS_DERIVED_DEFINE. Both generate hash(), operator==,
    // operator< and the option registration from the same option list. The hash inequalities use fixed
    // inputs, so they are deterministic; they check that hash() covers the changed option.
    SECTION("default-constructed configs are equal and find their options by key")
    {
        PrintObjectConfig a, b;
        REQUIRE(a == b);
        REQUIRE(a.hash() == b.hash());
        REQUIRE_FALSE(a < b);
        REQUIRE_FALSE(b < a);
        REQUIRE(a.optptr("layer_height") == &a.layer_height);
        REQUIRE(a.optptr("brim_object_gap") == &a.brim_object_gap);
    }

    SECTION("one differing option makes the configs unequal and orders them")
    {
        PrintObjectConfig a, b;
        b.layer_height.value = a.layer_height.value + 0.05;
        REQUIRE(a != b);
        REQUIRE(a.hash() != b.hash());
        REQUIRE(a < b);
        REQUIRE_FALSE(b < a);
    }

    SECTION("ordering is decided by the first option in declaration order that differs")
    {
        PrintObjectConfig a, b;
        a.brim_object_gap.value = b.brim_object_gap.value + 1.0;  // declared first
        a.layer_height.value    = b.layer_height.value - 0.05;    // declared later, points the other way
        REQUIRE(b < a);
        REQUIRE_FALSE(a < b);
    }

    SECTION("a derived config sees differences in its parents and in its own options")
    {
        PrintConfig a, b;
        REQUIRE(a == b);
        REQUIRE(a.hash() == b.hash());

        b.gcode_flavor.value = b.gcode_flavor.value == gcfMarlinLegacy ? gcfKlipper : gcfMarlinLegacy;  // GCodeConfig parent
        REQUIRE(a != b);
        REQUIRE(a.hash() != b.hash());

        PrintConfig c, d;
        d.skirt_distance.value = c.skirt_distance.value + 1.0;  // PrintConfig's own list
        REQUIRE(c != d);
        REQUIRE(c.hash() != d.hash());
        REQUIRE(c.optptr("skirt_distance") == &c.skirt_distance);
        REQUIRE(c.optptr("gcode_flavor") == &c.gcode_flavor);
    }
}

namespace {

// Keys whose values differ between two full configs, compared as text so enum names count too.
std::vector<std::string> differing_keys(const FullPrintConfig &a, const FullPrintConfig &b)
{
    std::vector<std::string> keys;
    for (const std::string &key : a.keys())
        if (a.opt_serialize(key) != b.opt_serialize(key))
            keys.push_back(key);
    return keys;
}

// Applies source to one full config member by member and to another key by key, as apply() did before
// static configs could apply themselves.
template<class Source> void check_member_apply_matches_key_apply(const Source &source)
{
    FullPrintConfig by_member;
    FullPrintConfig by_key;
    by_member.apply(source);
    by_key.apply_only(source, source.keys());
    CHECK(differing_keys(by_member, by_key).empty());
    CHECK_FALSE(differing_keys(by_member, FullPrintConfig()).empty());
}

bool is_numeric_option_type(const ConfigOptionType type)
{
    switch (type) {
    case coFloat:
    case coFloats:
    case coInt:
    case coInts:
    case coPercent:
    case coPercents:
    case coFloatOrPercent:
    case coFloatsOrPercents:
        return true;
    default:
        return false;
    }
}

bool is_integer_option_type(const ConfigOptionType type)
{
    return type == coInt || type == coInts;
}

bool is_percent_option_type(const ConfigOptionType type)
{
    return type == coPercent || type == coPercents;
}

std::string serialize_numeric_boundary(const ConfigOptionType type, const double value)
{
    std::ostringstream stream;
    if (is_integer_option_type(type))
        stream << static_cast<long long>(std::llround(value));
    else
        stream << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    if (is_percent_option_type(type))
        stream << '%';
    return stream.str();
}

// An empty aggregate has no unambiguous serialized representation: its
// deserializer treats empty input as one empty group. Keep the one registered
// case explicit while exercising every other default below.
const std::set<std::string> expected_empty_aggregate_defaults = {"extruder_printable_area"};

std::string representative_serialization(const ConfigOptionDef &def, const ConfigOption &option)
{
    const std::string serialized = option.serialize();
    if (!serialized.empty())
        return serialized;

    switch (def.type) {
    case coString:
        return "schema-roundtrip";
    case coStrings:
        return "schema-roundtrip;second-value";
    case coPoint:
        return "1,2";
    case coPoints:
        return "1x2,3x4";
    case coPointsGroups:
        return "1x2,3x4#5x6";
    case coIntsGroups:
        return "1,2#3";
    case coBools:
        return "0,1";
    case coFloats:
    case coInts:
    case coPercents:
    case coFloatsOrPercents:
        return is_percent_option_type(def.type) ? "0%" : "0";
    default:
        return serialized;
    }
}

} // namespace

TEST_CASE("A static config applies itself onto a config of its type as a lookup by name would", "[Config]")
{
    SECTION("region config")
    {
        PrintRegionConfig region;
        region.sparse_infill_pattern.value = ipGyroid;
        region.outer_wall_speed.values     = {37.};
        region.sparse_infill_density.value = 35.;
        FullPrintConfig full;
        REQUIRE(region.apply_to(full));
        check_member_apply_matches_key_apply(region);
    }
    SECTION("object config")
    {
        PrintObjectConfig object;
        object.seam_position.value  = spRear;
        object.wall_generator.value = PerimeterGeneratorType::Arachne;
        object.support_speed.values = {33.};
        object.enable_support.value = true;
        FullPrintConfig full;
        REQUIRE(object.apply_to(full));
        check_member_apply_matches_key_apply(object);
    }
    SECTION("G-code config, whose enum lists carry their names through a keys map")
    {
        GCodeConfig gcode;
        gcode.z_hop_types.values       = {int(zhtSpiral)};
        gcode.retraction_length.values = {1.5};
        FullPrintConfig full;
        REQUIRE(gcode.apply_to(full));
        check_member_apply_matches_key_apply(gcode);
    }
}

TEST_CASE("A static config applied onto a config of another type falls back to a lookup by name", "[Config]")
{
    PrintRegionConfig region;
    region.sparse_infill_pattern.value = ipGyroid;
    DynamicPrintConfig dynamic;
    REQUIRE_FALSE(region.apply_to(dynamic));
    dynamic.apply(region);
    CHECK(dynamic.opt_serialize("sparse_infill_pattern") == "gyroid");
}

TEST_CASE("Default options of enum lists get their definition's keys map", "[Config]")
{
    size_t enum_lists = 0;
    for (const auto &[key, def] : print_config_def.options) {
        if (def.type != coEnums || !def.default_value)
            continue;
        INFO(key);
        const std::unique_ptr<ConfigOption> opt(def.create_default_option());
        CHECK(*opt == *def.default_value);
        const auto *nullable_enums = dynamic_cast<const ConfigOptionEnumsGenericNullable *>(opt.get());
        const auto *enums          = dynamic_cast<const ConfigOptionEnumsGeneric *>(opt.get());
        REQUIRE((nullable_enums != nullptr) != (enums != nullptr));
        CHECK((nullable_enums != nullptr ? nullable_enums->keys_map : enums->keys_map) == def.enum_keys_map);
        ++enum_lists;
    }
    CHECK(enum_lists > 0);
}

TEST_CASE("Static and dynamic print defaults retain named enum lists", "[Config][StaticEnumDefaults]")
{
    const auto check_named_enums = [](ConfigBase &config) {
        REQUIRE(config.opt_serialize("z_hop_types") == "Slope Lift");
        REQUIRE(config.opt_serialize("retract_lift_enforce") == "All Surfaces");

        config.set_deserialize_strict("z_hop_types", "Normal Lift,Spiral Lift");
        config.set_deserialize_strict("retract_lift_enforce", "Top and Bottom");

        const auto *z_hop = config.option<ConfigOptionEnumsGeneric>("z_hop_types");
        const auto *surfaces = config.option<ConfigOptionEnumsGeneric>("retract_lift_enforce");
        REQUIRE(z_hop != nullptr);
        REQUIRE(surfaces != nullptr);
        const std::vector<int> expected_hops{zhtNormal, zhtSpiral};
        const std::vector<int> expected_surfaces{rletTopAndBottom};
        CHECK(z_hop->values == expected_hops);
        CHECK(surfaces->values == expected_surfaces);
        CHECK(config.opt_serialize("z_hop_types") == "Normal Lift,Spiral Lift");
        CHECK(config.opt_serialize("retract_lift_enforce") == "Top and Bottom");
    };

    SECTION("Static G-code defaults")
    {
        GCodeConfig config;
        check_named_enums(config);
    }
    SECTION("Dynamic copy of full print defaults")
    {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        check_named_enums(config);
    }
}

TEST_CASE("Every registered print option has a complete schema contract", "[Config][Schema]")
{
    // `extruder` is an object-level override whose zero value means "inherit". It is
    // intentionally represented by an empty option rather than a preset default. Any
    // newly registered option without a default should make this allowlist fail loudly.
    const std::set<std::string> expected_defaultless = {"extruder"};
    std::set<std::string>       actual_defaultless;

    for (const auto &[key, def] : print_config_def.options) {
        INFO(key);
        CHECK(def.opt_key == key);
        CHECK(def.serialization_key_ordinal > 0);

        if (def.default_value.get() == nullptr)
            actual_defaultless.insert(key);
        else
            CHECK(def.default_value->type() == def.type);

        // Every definition must be able to materialize its normal dynamic value. The
        // two grouped point/int types are static-only containers and are deliberately
        // excluded from create_empty_option()'s scalar/vector factory.
        const std::unique_ptr<ConfigOption> default_option(def.create_default_option());
        REQUIRE(default_option != nullptr);
        CHECK(default_option->type() == def.type);

        // Nullable enum lists carry their meaningful empty state in the default
        // clone (create_empty_option() intentionally has no nullable coEnums
        // branch). Keep this exception explicit so a new unsupported nullable
        // type cannot silently bypass the factory contract.
        const bool empty_factory_supported = !(def.nullable && def.type == coEnums);
        if (def.type != coPointsGroups && def.type != coIntsGroups && empty_factory_supported) {
            const std::unique_ptr<ConfigOption> empty_option(def.create_empty_option());
            REQUIRE(empty_option != nullptr);
            CHECK(empty_option->type() == def.type);
            CHECK(empty_option->nullable() == def.nullable);
        }

        if (def.type == coEnum || def.type == coEnums) {
            REQUIRE(def.enum_keys_map != nullptr);
            REQUIRE_FALSE(def.enum_values.empty());
            const std::set<std::string> unique_values(def.enum_values.begin(), def.enum_values.end());
            CHECK(unique_values.size() == def.enum_values.size());
            for (const std::string &value : def.enum_values) {
                const bool canonical = def.enum_keys_map->find(value) != def.enum_keys_map->end();
                if (!canonical) {
                    // filament_map_mode keeps the UI-only "Default" menu
                    // sentinel in enum_values even though it is not a
                    // serialized enum key. Any other missing map key is a
                    // schema drift that this inventory must report.
                    CHECK(key == "filament_map_mode");
                    CHECK(value == "Default");
                } else {
                    CHECK(canonical);
                }
            }
            // enum_values is the UI-facing menu. It may intentionally omit
            // canonical aliases or firmware tokens that remain valid through
            // enum_keys_map; the round-trip test below exercises every map key.
        }
    }

    CHECK(actual_defaultless == expected_defaultless);
}

TEST_CASE("Every registered option has a strict default serialization round-trip", "[Config][Schema][RoundTrip]")
{
    std::set<std::string> empty_aggregate_defaults;
    size_t                roundtripped = 0;

    for (const auto &[key, def] : print_config_def.options) {
        INFO(key);
        const std::unique_ptr<ConfigOption> source(def.create_default_option());
        REQUIRE(source != nullptr);
        const std::string serialized = source->serialize();

        if (serialized.empty() && (def.type == coPointsGroups || def.type == coIntsGroups)) {
            empty_aggregate_defaults.insert(key);
            continue;
        }

        const std::unique_ptr<ConfigOption> restored(def.create_default_option());
        REQUIRE(restored != nullptr);
        REQUIRE(restored->deserialize(serialized));
        // A few legacy vector defaults originate from float literals (for
        // example 0.01f) but are stored as doubles.  Their decimal wire form
        // is intentionally shorter than the promoted binary value, so the
        // object equality operator is too strict for this serialization
        // contract.  Stable canonical serialization is the invariant here.
        CHECK(restored->serialize() == serialized);
        ++roundtripped;
    }

    CHECK(empty_aggregate_defaults == expected_empty_aggregate_defaults);
    CHECK(roundtripped + empty_aggregate_defaults.size() == print_config_def.options.size());
}

TEST_CASE("Registered option representatives serialize and deserialize strictly", "[Config][Schema][RoundTrip]")
{
    size_t roundtripped = 0;

    for (const auto &[key, def] : print_config_def.options) {
        INFO(key);
        const std::unique_ptr<ConfigOption> source(def.create_default_option());
        REQUIRE(source != nullptr);
        const std::string representative = representative_serialization(def, *source);
        REQUIRE_FALSE(representative.empty());

        const std::unique_ptr<ConfigOption> restored(def.create_default_option());
        REQUIRE(restored != nullptr);
        REQUIRE(restored->deserialize(representative));
        CHECK(restored->serialize() == representative);
        ++roundtripped;
    }

    CHECK(roundtripped == print_config_def.options.size());
}

TEST_CASE("All registered enum values round-trip through their serialized form", "[Config][Schema][Enum]")
{
    size_t enum_options = 0;
    for (const auto &[key, def] : print_config_def.options) {
        if (def.type != coEnum && def.type != coEnums)
            continue;
        ++enum_options;
        REQUIRE(def.enum_keys_map != nullptr);
        // Iterate the enum key map, because enum_values can also contain a
        // UI-only menu sentinel (filament_map_mode/Default).
        for (const auto &[value, enum_value] : *def.enum_keys_map) {
            (void)enum_value;
            INFO(key << " = " << value);
            const bool empty_factory_supported = !(def.nullable && def.type == coEnums);
            const std::unique_ptr<ConfigOption> option(empty_factory_supported ?
                def.create_empty_option() : def.create_default_option());
            REQUIRE(option != nullptr);
            REQUIRE(option->deserialize(value));
            CHECK(option->serialize() == value);
        }
    }
    CHECK(enum_options > 0);
}

TEST_CASE("All registered boolean settings accept only the canonical serialized values", "[Config][Schema][Bool]")
{
    size_t bool_options = 0;
    for (const auto &[key, def] : print_config_def.options) {
        if (def.type != coBool && def.type != coBools)
            continue;
        ++bool_options;

        INFO(key);
        const std::unique_ptr<ConfigOption> option(def.create_empty_option());
        REQUIRE(option != nullptr);
        REQUIRE(option->deserialize("0"));
        CHECK(option->serialize() == "0");
        REQUIRE(option->deserialize("1"));
        CHECK(option->serialize() == "1");
        CHECK_FALSE(option->deserialize("2"));

        if (def.type == coBools) {
            REQUIRE(option->deserialize("0,1"));
            CHECK(option->serialize() == "0,1");
            if (def.nullable) {
                REQUIRE(option->deserialize("nil"));
                CHECK(option->serialize() == "nil");
            }
        }
    }
    CHECK(bool_options > 0);
}

TEST_CASE("Numeric option definitions enforce their declared finite boundaries", "[Config][Schema][Numeric]")
{
    const double float_limit = static_cast<double>(std::numeric_limits<float>::max()) / 2.;
    size_t       bounded_options = 0;

    for (const auto &[key, def] : print_config_def.options) {
        switch (def.type) {
        case coFloat:
        case coFloats:
        case coInt:
        case coInts:
        case coPercent:
        case coPercents:
        case coFloatOrPercent:
        case coFloatsOrPercents:
            break;
        default:
            continue;
        }

        const bool has_min = static_cast<double>(def.min) > -float_limit;
        const bool has_max = static_cast<double>(def.max) < float_limit;
        if (!has_min && !has_max)
            continue;
        ++bounded_options;

        INFO(key);
        if (has_min) {
            CHECK(def.is_value_valid(def.min));
            const double delta = std::max(1., std::abs(static_cast<double>(def.min)) * 0.01);
            CHECK_FALSE(def.is_value_valid(static_cast<double>(def.min) - delta));
        }
        if (has_max) {
            CHECK(def.is_value_valid(def.max));
            const double delta = std::max(1., std::abs(static_cast<double>(def.max)) * 0.01);
            CHECK_FALSE(def.is_value_valid(static_cast<double>(def.max) + delta));
        }
    }

    CHECK(bounded_options > 0);
}

TEST_CASE("Registered numeric boundaries round-trip through strict serialization", "[Config][Schema][Numeric][RoundTrip]")
{
    const double float_limit = static_cast<double>(std::numeric_limits<float>::max()) / 2.;
    size_t       boundary_values = 0;

    for (const auto &[key, def] : print_config_def.options) {
        if (!is_numeric_option_type(def.type))
            continue;

        const bool has_min = static_cast<double>(def.min) > -float_limit;
        const bool has_max = static_cast<double>(def.max) < float_limit;
        const std::array<double, 2> boundaries = {
            has_min ? static_cast<double>(def.min) : std::numeric_limits<double>::quiet_NaN(),
            has_max ? static_cast<double>(def.max) : std::numeric_limits<double>::quiet_NaN(),
        };
        for (const double boundary : boundaries) {
            if (!std::isfinite(boundary))
                continue;

            INFO(key << " boundary=" << boundary);
            CHECK(def.is_value_valid(boundary));
            const std::string serialized_boundary = serialize_numeric_boundary(def.type, boundary);
            const std::unique_ptr<ConfigOption> source(def.create_default_option());
            REQUIRE(source != nullptr);
            REQUIRE(source->deserialize(serialized_boundary));

            const std::string canonical = source->serialize();
            const std::unique_ptr<ConfigOption> restored(def.create_default_option());
            REQUIRE(restored != nullptr);
            REQUIRE(restored->deserialize(canonical));
            CHECK(restored->serialize() == canonical);
            ++boundary_values;
        }
    }

    CHECK(boundary_values > 0);
}

TEST_CASE("Core FFF numeric validation rejects values below their semantic floors", "[Config][Schema][Validation]")
{
    struct Boundary {
        const char *key;
        const char *invalid;
        const char *valid;
    };
    // These floors are semantic constraints enforced by validate(), beyond the GUI
    // metadata (which intentionally permits zero for several editable fields).
    const Boundary boundaries[] = {
        {"layer_height", "0", "0.2"},
        {"filament_diameter", "0.9", "1.0"},
        {"nozzle_diameter", "0.004", "0.005"},
        {"wall_loops", "-1", "0"},
        {"top_shell_layers", "-1", "0"},
        {"bottom_shell_layers", "-1", "0"},
        {"skirt_height", "-1", "0"},
        {"bridge_flow", "0", "1"},
        {"internal_bridge_flow", "0", "1"},
        {"extruder_clearance_radius", "0", "0.1"},
        {"nozzle_height", "0", "0.1"},
        {"filament_flow_ratio", "0", "0.01"},
    };

    for (const Boundary &boundary : boundaries) {
        DYNAMIC_SECTION(boundary.key) {
            DynamicPrintConfig invalid = DynamicPrintConfig::full_print_config();
            invalid.set_deserialize_strict(boundary.key, boundary.invalid);
            const auto invalid_errors = invalid.validate();
            CHECK(invalid_errors.find(boundary.key) != invalid_errors.end());

            DynamicPrintConfig valid = DynamicPrintConfig::full_print_config();
            valid.set_deserialize_strict(boundary.key, boundary.valid);
            const auto valid_errors = valid.validate();
            CHECK(valid_errors.find(boundary.key) == valid_errors.end());
        }
    }
}

namespace print_config_behavior_contracts {
using namespace Slic3r;

namespace {

bool has_error(const std::map<std::string, std::string> &errors, const char *key)
{
    return errors.find(key) != errors.end();
}

void require_all_false(const ConfigOptionBools *option)
{
    REQUIRE(option != nullptr);
    REQUIRE(option->values.size() == 2);
    CHECK(std::all_of(option->values.begin(), option->values.end(), [](unsigned char value) { return value == 0; }));
}

DynamicPrintConfig complete_config()
{
    DynamicPrintConfig config;
    config.apply(FullPrintConfig::defaults(), false);
    return config;
}

} // namespace

TEST_CASE("PrintConfig legacy aliases preserve their exact key and value contract", "[PrintConfigBehavior][Config]")
{
    struct Migration {
        const char *old_key;
        const char *old_value;
        const char *new_key;
        const char *new_value;
    };
    const Migration migrations[] = {
        {"enable_wipe_tower", "1", "enable_prime_tower", "1"},
        {"wipe_tower_width", "42", "prime_tower_width", "42"},
        {"bridge_fan_speed", "77", "overhang_fan_speed", "77"},
        {"support_material_angle", "55", "support_angle", "55"},
        {"support_material_enforce_layers", "3", "enforce_support_layers", "3"},
        {"top_one_wall_type", "outer", "only_one_wall_top", "1"},
        {"sparse_infill_anchor", "35", "infill_anchor", "35"},
        {"chamber_temperatures", "60,65", "chamber_temperature", "60,65"},
        {"thumbnail_size", "96x96", "thumbnails", "96x96"},
        {"counterbole_hole_bridging", "bridges", "counterbore_hole_bridging", "bridges"},
    };

    for (const Migration &migration : migrations) {
        DYNAMIC_SECTION(migration.old_key << " -> " << migration.new_key) {
            t_config_option_key key = migration.old_key;
            std::string value       = migration.old_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key == migration.new_key);
            CHECK(value == migration.new_value);
        }
    }
}

TEST_CASE("PrintConfig legacy enum spellings become current canonical values", "[PrintConfigBehavior][Config]")
{
    struct Conversion {
        const char *key;
        const char *old_value;
        const char *new_value;
    };
    const Conversion conversions[] = {
        {"support_type", "normal", "normal(manual)"},
        {"support_type", "tree", "tree(manual)"},
        {"support_type", "hybrid(auto)", "tree(auto)"},
        {"support_base_pattern", "none", "hollow"},
        {"sparse_infill_pattern", "zig-zag", "rectilinear"},
        {"top_surface_pattern", "zig-zag", "rectilinear"},
        {"filament_map_mode", "Auto", "Auto For Flush"},
        {"extruder_type", "DirectDrive", "Direct Drive"},
        {"enable_power_loss_recovery", "true", "enable"},
        {"ensure_vertical_shell_thickness", "0", "ensure_moderate"},
        {"wall_direction", "auto", "ccw"},
    };

    for (const Conversion &conversion : conversions) {
        DYNAMIC_SECTION(conversion.key << " = " << conversion.old_value) {
            t_config_option_key key = conversion.key;
            std::string value       = conversion.old_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key == conversion.key);
            CHECK(value == conversion.new_value);
        }
    }

    t_config_option_key key = "rotate_solid_infill_direction";
    std::string value       = "1";
    PrintConfigDef::handle_legacy(key, value);
    CHECK(key == "solid_infill_rotate_template");
    CHECK(value == "0,90");
}

TEST_CASE("PrintConfig legacy values that cannot be represented are discarded", "[PrintConfigBehavior][Config]")
{
    const std::pair<const char *, const char *> obsolete[] = {
        {"initial_layer_speed", "50%"},
        {"outer_wall_speed", "80%"},
        {"support_object_xy_distance", "25%"},
        {"scale", "1.2"},
        {"bed_size", "220,220"},
        {"key_from_a_future_release", "1"},
    };
    for (const auto &[old_key, old_value] : obsolete) {
        DYNAMIC_SECTION(old_key) {
            t_config_option_key key = old_key;
            std::string value       = old_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key.empty());
        }
    }
}

TEST_CASE("Legacy SLA vector corrections are expanded into axis options", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config;
    config.set_key_value("relative_correction", new ConfigOptionFloats({1.1, 1.2}));
    config.set_key_value("material_correction", new ConfigOptionFloats({0.9, 0.8}));
    handle_legacy_sla(config);

    CHECK_THAT(config.opt_float("relative_correction_x"), Catch::Matchers::WithinAbs(1.1, 1e-9));
    CHECK_THAT(config.opt_float("relative_correction_y"), Catch::Matchers::WithinAbs(1.1, 1e-9));
    CHECK_THAT(config.opt_float("relative_correction_z"), Catch::Matchers::WithinAbs(1.2, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_x"), Catch::Matchers::WithinAbs(0.9, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_y"), Catch::Matchers::WithinAbs(0.9, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_z"), Catch::Matchers::WithinAbs(0.8, 1e-9));
}

TEST_CASE("Legacy composite thumbnail values become canonical definitions", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config = complete_config();
    config.set("thumbnails", "16x16,32x32/JPG");
    config.handle_legacy_composite();
    CHECK(config.opt_string("thumbnails") == "16x16/PNG, 32x32/JPG");
}

TEST_CASE("Legacy composite wiping matrix detects whether custom entries exist", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig defaults;
    defaults.set_key_value("wiping_volumes_matrix", new ConfigOptionFloats({0., 140., 140., 0.}));
    defaults.handle_legacy_composite();
    CHECK_FALSE(defaults.opt_bool("wiping_volumes_use_custom_matrix"));

    DynamicPrintConfig custom;
    custom.set_key_value("wiping_volumes_matrix", new ConfigOptionFloats({0., 141., 140., 0.}));
    custom.handle_legacy_composite();
    CHECK(custom.opt_bool("wiping_volumes_use_custom_matrix"));
}

TEST_CASE("Firmware retraction validation checks flavor and wipe independently", "[PrintConfigBehavior][Config]")
{
    const bool firmware = GENERATE(false, true);
    const bool wipe = GENERATE(false, true);
    const std::string flavor = GENERATE("teacup", "klipper");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set("use_firmware_retraction", firmware);
    config.set_deserialize_strict("gcode_flavor", flavor);
    config.set_deserialize_strict("wipe", wipe ? "1" : "0");
    const bool incompatible = firmware && (wipe || flavor == "teacup");
    CHECK(has_error(config.validate(true), "use_firmware_retraction") == incompatible);
}

TEST_CASE("PrintConfig validation reports spiral mode's incompatible process settings", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set("spiral_mode", true);
    config.set("wall_loops", 2);
    config.set_deserialize_strict("sparse_infill_density", "15");
    config.set("top_shell_layers", 1);
    config.set("enable_support", true);
    config.set("enforce_support_layers", 1);
    const auto errors = config.validate(true);
    CHECK(has_error(errors, "wall_loops"));
    CHECK(has_error(errors, "sparse_infill_density"));
    CHECK(has_error(errors, "top_shell_layers"));
    CHECK(has_error(errors, "enable_support"));
    CHECK(has_error(errors, "enforce_support_layers"));

    config.set("wall_loops", 1);
    config.set_deserialize_strict("sparse_infill_density", "0");
    config.set("top_shell_layers", 0);
    config.set("enable_support", false);
    config.set("enforce_support_layers", 0);
    CHECK_FALSE(has_error(config.validate(true), "wall_loops"));
    CHECK_FALSE(has_error(config.validate(true), "sparse_infill_density"));
    CHECK_FALSE(has_error(config.validate(true), "top_shell_layers"));
    CHECK_FALSE(has_error(config.validate(true), "enable_support"));
    CHECK_FALSE(has_error(config.validate(true), "enforce_support_layers"));
}

TEST_CASE("PrintConfig validation applies nozzle-relative bridge width limits", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig too_wide = DynamicPrintConfig::full_print_config();
    too_wide.set_deserialize_strict("nozzle_diameter", "0.4");
    too_wide.set_deserialize_strict("bridge_line_width", "0.41");
    CHECK(has_error(too_wide.validate(), "bridge_line_width"));

    DynamicPrintConfig at_limit = DynamicPrintConfig::full_print_config();
    at_limit.set_deserialize_strict("nozzle_diameter", "0.4");
    at_limit.set_deserialize_strict("bridge_line_width", "0.4");
    CHECK_FALSE(has_error(at_limit.validate(), "bridge_line_width"));
}

TEST_CASE("normalize_fdm migrates a legacy extruder without overwriting explicit roles", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config = complete_config();
    config.option<ConfigOptionInt>("extruder", true)->value = 2;
    config.set("outer_wall_filament_id", 3);
    config.set("inner_wall_filament_id", 0);
    config.set("sparse_infill_filament_id", 0);
    config.set("internal_solid_filament_id", 0);
    config.set("top_surface_filament_id", 0);
    config.set("bottom_surface_filament_id", 0);
    config.normalize_fdm();

    CHECK_FALSE(config.has("extruder"));
    CHECK(config.opt_int("sparse_infill_filament_id") == 2);
    CHECK(config.opt_int("outer_wall_filament_id") == 3);
    CHECK(config.opt_int("inner_wall_filament_id") == 2);
    CHECK(config.opt_int("internal_solid_filament_id") == 2);
    CHECK(config.opt_int("top_surface_filament_id") == 2);
    CHECK(config.opt_int("bottom_surface_filament_id") == 2);
}

TEST_CASE("normalize_fdm clamps spiral process values and the resolution floor", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config = complete_config();
    config.set("spiral_mode", true);
    config.set("wall_loops", 4);
    config.set("top_shell_layers", 3);
    config.set_deserialize_strict("sparse_infill_density", "20");
    config.set_deserialize_strict("resolution", "0.0001");
    config.set_deserialize_strict("retract_when_changing_layer", "1,1");
    config.option<ConfigOptionBoolsNullable>("filament_retract_when_changing_layer", true)->values = {false, false};
    config.normalize_fdm();

    CHECK(config.opt_int("wall_loops") == 1);
    CHECK(config.opt_int("top_shell_layers") == 0);
    CHECK_THAT(config.opt<ConfigOptionPercent>("sparse_infill_density")->value, Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(config.opt_float("resolution"), Catch::Matchers::WithinAbs(0.001, 1e-9));
    require_all_false(config.opt<ConfigOptionBools>("retract_when_changing_layer"));
}

TEST_CASE("normalize_fdm_2 records prime tower changes for one filament and keeps two-filament tower rules", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig one = DynamicPrintConfig::full_print_config();
    one.set("enable_prime_tower", true);
    one.set("independent_support_layer_height", true);
    const auto one_changed = one.normalize_fdm_2(1, 1);
    CHECK_FALSE(one.opt_bool("enable_prime_tower"));
    CHECK(std::find(one_changed.begin(), one_changed.end(), "enable_prime_tower") != one_changed.end());

    DynamicPrintConfig two = DynamicPrintConfig::full_print_config();
    two.set("enable_prime_tower", true);
    two.set("independent_support_layer_height", true);
    const auto two_changed = two.normalize_fdm_2(1, 2);
    CHECK(two.opt_bool("enable_prime_tower"));
    CHECK_FALSE(two.opt_bool("independent_support_layer_height"));
    CHECK(std::find(two_changed.begin(), two_changed.end(), "independent_support_layer_height") != two_changed.end());
}

TEST_CASE("get_parameter_size uses variant cardinality and stride-specific machine dimensions", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config;
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow"}));
    config.set_key_value("filament_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"}));
    config.set_key_value("print_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard", "Bowden High Flow"}));

    CHECK(config.get_parameter_size("machine_max_speed_x", 9) == 4);
    CHECK(config.get_parameter_size("machine_max_acceleration_x", 9) == 4);
    CHECK(config.get_parameter_size("filament_flow_ratio", 9) == 3);
    CHECK(config.get_parameter_size("outer_wall_speed", 9) == 4);
    CHECK(config.get_parameter_size("unknown_option", 9) == 9);
}

TEST_CASE("AMS count serialization is sorted and preserves empty extruder slots", "[PrintConfigBehavior][Config]")
{
    const std::vector<std::map<int, int>> source = {
        {{2, 5}, {1, 3}},
        {},
        {{4, 1}},
    };
    const auto serialized = save_extruder_ams_count_to_string(source);
    CHECK(serialized == std::vector<std::string>{"1#3|2#5", "", "4#1"});
    CHECK(get_extruder_ams_count(serialized) == source);
}

TEST_CASE("Nozzle statistics skip malformed tokens while retaining valid inventory", "[PrintConfigBehavior][Config]")
{
    const auto parsed = get_extruder_nozzle_stats({"Standard#2|Broken|High Flow#1", "Unknown#9|E3D High Flow#3"});
    REQUIRE(parsed.size() == 2);
    CHECK(parsed[0].at(nvtStandard) == 2);
    CHECK(parsed[0].at(nvtHighFlow) == 1);
    CHECK(parsed[1].at(nvtE3DHighFlow) == 3);
    CHECK(parsed[1].count(nvtStandard) == 0);
    CHECK(save_extruder_nozzle_stats_to_string(parsed) == std::vector<std::string>{"Standard#2|High Flow#1", "E3D High Flow#3"});
}

TEST_CASE("PrintConfig validates mixed filament metadata cardinality", "[PrintConfigBehavior][Config]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_is_mixed", new ConfigOptionBools({false, false, true}));
    config.set_key_value("filament_mixed_components", new ConfigOptionStrings({"", "", "1,2"}));
    config.set_key_value("filament_mixed_sublayer_ratios", new ConfigOptionStrings({"", "", "0.5,0.5"}));
    config.set_key_value("filament_mixed_gradient", new ConfigOptionBools({false, false, true}));
    config.set_key_value("filament_mixed_gradient_range", new ConfigOptionStrings({"", "", "0.2,0.8"}));
    config.set_key_value("filament_mixed_gradient_curve", new ConfigOptionStrings({"", "", "0,0;1,1"}));
    CHECK_FALSE(config.validate().count("filament_mixed_components"));
    CHECK_FALSE(config.validate().count("filament_mixed_sublayer_ratios"));

    config.set_key_value("filament_mixed_sublayer_ratios", new ConfigOptionStrings({"", "", "0.4"}));
    CHECK(config.validate().count("filament_mixed_sublayer_ratios"));
}

} // namespace print_config_behavior_contracts

namespace config_multi_mapping_contracts {
namespace {

void check_speeds(const DynamicPrintConfig &config, const std::vector<double> &expected)
{
    const auto *option = config.option<ConfigOptionFloats>("outer_wall_speed");
    REQUIRE(option != nullptr);
    REQUIRE(option->values.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(option->values[i], Catch::Matchers::WithinAbs(expected[i], 1e-9));
}

const std::set<std::string> kMappingKeys{
    "print_extruder_id",
    "print_extruder_variant",
    "outer_wall_speed",
    "small_perimeter_speed",
    "enable_overhang_speed",
};

void set_variant_columns(DynamicPrintConfig &config,
                         std::vector<int> ids,
                         std::vector<std::string> variants)
{
    config.option<ConfigOptionInts>("print_extruder_id", true)->values = std::move(ids);
    config.option<ConfigOptionStrings>("print_extruder_variant", true)->values = std::move(variants);
}

void set_single_arrays(DynamicPrintConfig &config, double outer_speed, bool overhang)
{
    config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {outer_speed};
    config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {overhang};
}

void set_multi_arrays(DynamicPrintConfig &config,
                      std::vector<double> outer_speed,
                      std::vector<FloatOrPercent> small_speed,
                      std::vector<unsigned char> overhang)
{
    config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = std::move(outer_speed);
    config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed", true)->values = std::move(small_speed);
    config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = std::move(overhang);
}

} // namespace

TEST_CASE("Single-to-multi copies variant identity and limits numeric columns", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_variant_columns(multi, {1, 1, 2}, {"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"});
    set_variant_columns(single, {99}, {"old"});
    set_single_arrays(single, 100., true);
    single.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed", true)->values = {FloatOrPercent(50., true)};
    set_multi_arrays(multi, {80., 160., 240.},
                     {FloatOrPercent(40., false), FloatOrPercent(60., true), FloatOrPercent(30., false)},
                     {0, 1, 0});

    std::set<std::string> keys = kMappingKeys;
    REQUIRE(single.update_values_from_single_to_multi(multi, keys,
                                                       "print_extruder_id", "print_extruder_variant") == 0);
    CHECK(single.option<ConfigOptionInts>("print_extruder_id")->values == std::vector<int>({1, 1, 2}));
    CHECK(single.option<ConfigOptionStrings>("print_extruder_variant")->values ==
          std::vector<std::string>({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"}));
    // The source defaults cap the user's single value independently in each target column.
    check_speeds(single, {80., 100., 100.});
    const auto &small = single.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed")->values;
    REQUIRE(small.size() == 3);
    CHECK_THAT(small[0].value, Catch::Matchers::WithinAbs(40., 1e-9));
    CHECK(small[0].percent == false);
    CHECK_THAT(small[1].value, Catch::Matchers::WithinAbs(50., 1e-9));
    CHECK(small[1].percent == true);
    CHECK_THAT(small[2].value, Catch::Matchers::WithinAbs(30., 1e-9));
    CHECK(small[2].percent == false);
    // Bools retain the user's single value for every new variant; the source bool array only supplies width.
    CHECK(single.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({1, 1, 1}));
}

TEST_CASE("Single-to-multi leaves unselected and source-missing settings untouched", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_variant_columns(multi, {1, 2}, {"A", "B"});
    set_single_arrays(single, 42., false);
    single.option<ConfigOptionInt>("wall_loops", true)->value = 7;
    // Deliberately omit outer_wall_speed from the source. The variant metadata is valid,
    // but a missing source option must not erase the caller's current value.
    std::set<std::string> keys{"outer_wall_speed", "wall_loops"};

    REQUIRE(single.update_values_from_single_to_multi(multi, keys,
                                                       "print_extruder_id", "print_extruder_variant") == 0);
    check_speeds(single, {42.});
    CHECK(single.option<ConfigOptionInt>("wall_loops")->value == 7);
}

TEST_CASE("Single-to-multi rejects a missing variant option without changing values", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_single_arrays(single, 42., true);
    const auto before = single.option<ConfigOptionFloats>("outer_wall_speed", true)->values;
    std::set<std::string> keys{"outer_wall_speed"};

    CHECK(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "missing_variant") == -1);
    check_speeds(single, before);
}

TEST_CASE("Single-to-multi ignores unknown keys while applying known mappings", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_variant_columns(multi, {1, 2}, {"A", "B"});
    set_single_arrays(single, 90., false);
    multi.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {30., 40.};
    std::set<std::string> keys{"unknown_mapping_key", "outer_wall_speed"};

    REQUIRE(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "print_extruder_variant") == 0);
    check_speeds(single, {30., 40.});
}

TEST_CASE("Multi-to-multi remaps duplicate variants and preserves numeric minima", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_variant_columns(old_config, {1, 1, 2}, {"A", "B", "A"});
    set_variant_columns(new_config, {1, 1, 2, 2}, {"A", "B", "A", "B"});
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {10., 20., 30.};
    old_config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed", true)->values =
        {FloatOrPercent(1., false), FloatOrPercent(2., false), FloatOrPercent(3., false)};
    old_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 1, 0};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {100., 200., 300., 400.};
    new_config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed", true)->values =
        {FloatOrPercent(50., true), FloatOrPercent(60., true), FloatOrPercent(70., true), FloatOrPercent(80., true)};
    new_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 0, 0, 0};
    std::vector<std::string> new_extruder_variants{"A", "B"};

    std::set<std::string> keys = kMappingKeys;
    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                         "print_extruder_id", "print_extruder_variant",
                                                         new_extruder_variants) == 0);
    CHECK(old_config.option<ConfigOptionInts>("print_extruder_id")->values == std::vector<int>({1, 1, 2, 2}));
    CHECK(old_config.option<ConfigOptionStrings>("print_extruder_variant")->values ==
          std::vector<std::string>({"A", "B", "A", "B"}));
    check_speeds(old_config, {10., 200., 300., 20.});
    const auto &small = old_config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed")->values;
    REQUIRE(small.size() == 4);
    CHECK_THAT(small[0].value, Catch::Matchers::WithinAbs(1., 1e-9));
    CHECK_THAT(small[1].value, Catch::Matchers::WithinAbs(60., 1e-9));
    CHECK_THAT(small[2].value, Catch::Matchers::WithinAbs(70., 1e-9));
    CHECK_THAT(small[3].value, Catch::Matchers::WithinAbs(2., 1e-9));
    CHECK_FALSE(small[0].percent);
    CHECK(small[1].percent);
    CHECK(small[2].percent);
    CHECK_FALSE(small[3].percent);
    CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({0, 0, 0, 1}));
}

TEST_CASE("Multi-to-multi keeps destination values when no new extruder target matches", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_variant_columns(old_config, {1, 2}, {"A", "B"});
    set_variant_columns(new_config, {1, 2}, {"A", "B"});
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {11., 22.};
    old_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {1, 0};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {101., 202.};
    new_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 0};
    std::vector<std::string> new_extruder_variants{"C"};
    std::set<std::string> keys{"outer_wall_speed", "enable_overhang_speed"};

    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                         "print_extruder_id", "print_extruder_variant",
                                                         new_extruder_variants) == 0);
    check_speeds(old_config, {101., 202.});
    CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({0, 0}));
}

TEST_CASE("Multi-to-multi skips an unmatched target row but still copies identity arrays", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_variant_columns(old_config, {1, 2}, {"A", "B"});
    set_variant_columns(new_config, {1, 2}, {"A", "B"});
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {11., 22.};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {101., 202.};
    std::vector<std::string> new_extruder_variants{"B", "A"};
    std::set<std::string> keys{"outer_wall_speed"};

    // Neither (id=1, B) nor (id=2, A) exists in the destination row.
    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                         "print_extruder_id", "print_extruder_variant",
                                                         new_extruder_variants) == 0);
    check_speeds(old_config, {101., 202.});
}

TEST_CASE("Multi-to-multi treats unknown keys as no-ops without suppressing known keys", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_variant_columns(old_config, {1}, {"A"});
    set_variant_columns(new_config, {1}, {"A"});
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {111.};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {101.};
    std::vector<std::string> variants{"A"};
    std::set<std::string> keys{"unknown_mapping_key", "outer_wall_speed"};

    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                         "print_extruder_id", "print_extruder_variant", variants) == 0);
    check_speeds(old_config, {101.});
}

TEST_CASE("Multi-to-multi rejects absent identity columns without touching the source", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {11.};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {101.};
    std::vector<std::string> variants{"A"};
    std::set<std::string> keys{"outer_wall_speed"};

    CHECK(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                       "missing_id", "missing_variant", variants) == -1);
    check_speeds(old_config, {11.});
}

TEST_CASE("Multi-to-multi with no target extruders still copies destination arrays", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_variant_columns(old_config, {1}, {"A"});
    set_variant_columns(new_config, {1}, {"A"});
    old_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {11.};
    new_config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = {101.};
    std::vector<std::string> variants;
    std::set<std::string> keys{"outer_wall_speed"};

    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys,
                                                         "print_extruder_id", "print_extruder_variant", variants) == 0);
    check_speeds(old_config, {101.});
}

} // namespace config_multi_mapping_contracts

namespace config_definition_contracts {
using namespace Slic3r;

struct HelpDefinition : ConfigDef {
    using ConfigDef::add;
};

std::string help_text(const ConfigDef &definition, bool defaults = true)
{
    std::ostringstream out;
    definition.print_cli_help(out, defaults);
    return out.str();
}

void check_schema(const ConfigDef &definition,
                  std::initializer_list<std::pair<const std::string, ConfigOptionType>> expected)
{
    const std::map<std::string, ConfigOptionType> types(expected);
    REQUIRE(definition.options.size() == types.size());
    REQUIRE(definition.by_serialization_key_ordinal.size() == types.size());
    for (const auto &[key, type] : types) {
        INFO(key);
        const ConfigOptionDef *option = definition.get(key);
        REQUIRE(option != nullptr);
        CHECK(option->opt_key == key);
        CHECK(option->type == type);
        CHECK_FALSE(option->label.empty());
        REQUIRE(option->serialization_key_ordinal > 0);
        const auto entry = definition.by_serialization_key_ordinal.find(option->serialization_key_ordinal);
        REQUIRE(entry != definition.by_serialization_key_ordinal.end());
        CHECK(entry->second == option);
    }
}

TEST_CASE("CLI help expands aliases and explicit parameter labels", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    auto *option = definition.add("layer_height", coFloat);
    option->cli = "h|layer-height";
    option->cli_params = "HEIGHT";
    option->tooltip = "Set the layer thickness.";
    const std::string help = help_text(definition);
    std::istringstream tokens(help);
    std::string short_alias, short_parameter, long_alias, long_parameter;
    REQUIRE(static_cast<bool>(tokens >> short_alias >> short_parameter >> long_alias >> long_parameter));
    CHECK(short_alias == "-h");
    CHECK(short_parameter == "HEIGHT,");
    CHECK(long_alias == "--layer-height");
    CHECK(long_parameter == "HEIGHT");
    CHECK(help.find("Set the layer thickness.") != std::string::npos);
}

TEST_CASE("CLI help derives dashed option names when aliases are absent", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    definition.add("machine_start_gcode", coString)->tooltip = "Startup instructions";
    const std::string help = help_text(definition);
    CHECK(help.find("--machine-start-gcode") != std::string::npos);
    CHECK(help.find("machine_start_gcode") == std::string::npos);
}

TEST_CASE("CLI help filters hidden options and entire excluded categories", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    auto *visible = definition.add("visible", coInt);
    visible->category = "Visible category";
    auto *hidden = definition.add("hidden", coInt);
    hidden->category = visible->category;
    hidden->cli = ConfigOptionDef::nocli;
    auto *excluded = definition.add("excluded", coInt);
    excluded->category = "Excluded category";
    std::ostringstream out;
    std::ostream &returned = definition.print_cli_help(out, true, [](const ConfigOptionDef &option) {
        return option.opt_key != "excluded";
    });
    CHECK(&returned == &out);
    const std::string help = out.str();
    CHECK(help.find("Visible category:") != std::string::npos);
    CHECK(help.find("--visible") != std::string::npos);
    CHECK(help.find("--hidden") == std::string::npos);
    CHECK(help.find("Excluded category") == std::string::npos);
    CHECK(help.find("--excluded") == std::string::npos);
}

TEST_CASE("CLI help labels uncategorized options only among multiple categories", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    definition.add("uncategorized", coInt);
    CHECK(help_text(definition).find("Misc options:") == std::string::npos);
    definition.add("categorized", coInt)->category = "Printer";
    const std::string help = help_text(definition);
    CHECK(help.find("Misc options:") != std::string::npos);
    CHECK(help.find("Printer:") != std::string::npos);
    CHECK(help.find("--uncategorized") != std::string::npos);
    CHECK(help.find("--categorized") != std::string::npos);
}

TEST_CASE("CLI help includes numeric defaults and their physical units on request", "[Config][ConfigDefinition][CLIHelp]")
{
    const bool defaults = GENERATE(false, true);
    HelpDefinition definition;
    auto *option = definition.add("height", coFloat);
    option->tooltip = "Layer thickness";
    option->sidetext = "mm";
    option->set_default_value(new ConfigOptionFloat(.25));
    const std::string help = help_text(definition, defaults);
    CHECK((help.find("(mm, default: 0.25)") != std::string::npos) == defaults);
    CHECK(help.find("Layer thickness") != std::string::npos);
}

TEST_CASE("CLI help lists enumerated choices before their default", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    auto *option = definition.add("strategy", coString);
    option->enum_values = {"slow", "fast"};
    option->set_default_value(new ConfigOptionString("fast"));
    const std::string help = help_text(definition);
    CHECK(help.find("(slow, fast; default: fast)") != std::string::npos);
}

TEST_CASE("CLI help omits boolean and empty string defaults", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    definition.add("enabled", coBool)->set_default_value(new ConfigOptionBool(true));
    definition.add("empty", coString)->set_default_value(new ConfigOptionString());
    definition.add("no_default", coInt);
    const std::string help = help_text(definition);
    CHECK(help.find("--enabled") != std::string::npos);
    CHECK(help.find("--empty") != std::string::npos);
    CHECK(help.find("--no-default") != std::string::npos);
    CHECK(help.find("default:") == std::string::npos);
}

TEST_CASE("CLI help always exposes the configuration compatibility default", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    definition.add("config_compatibility", coString)->set_default_value(new ConfigOptionString("enable"));
    definition.add("ordinary", coString)->set_default_value(new ConfigOptionString("ordinary-default"));
    const std::string help = help_text(definition, false);
    CHECK(help.find("default: enable") != std::string::npos);
    CHECK(help.find("ordinary-default") == std::string::npos);
}

TEST_CASE("CLI help wraps long descriptions without dropping or reordering words", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    auto *option = definition.add("a_deliberately_long_command_line_option", coString);
    std::vector<std::string> expected;
    for (int i = 0; i < 35; ++i) {
        expected.push_back("parameter" + std::to_string(i));
        option->tooltip += expected.back() + " ";
    }
    std::istringstream lines(help_text(definition));
    std::string line;
    REQUIRE(static_cast<bool>(std::getline(lines, line)));
    CHECK(line.find("--a-deliberately-long-command-line-option") != std::string::npos);
    CHECK(line.find("parameter0") == std::string::npos);
    std::vector<std::string> actual;
    std::size_t wrapped_lines = 0;
    while (std::getline(lines, line)) {
        REQUIRE(line.size() >= 21);
        CHECK(line.substr(0, 21) == std::string(21, ' '));
        CHECK(line.size() <= 101);
        std::istringstream words(line.substr(21));
        std::string word;
        while (words >> word)
            actual.push_back(word);
        ++wrapped_lines;
    }
    CHECK(wrapped_lines > 1);
    CHECK(actual == expected);
}

TEST_CASE("CLI help leaves an empty or fully filtered definition blank", "[Config][ConfigDefinition][CLIHelp]")
{
    HelpDefinition definition;
    CHECK(help_text(definition).empty());
    definition.add("excluded", coInt)->category = "Excluded";
    std::ostringstream out;
    definition.print_cli_help(out, true, [](const ConfigOptionDef &) { return false; });
    CHECK(out.str().empty());
}

TEST_CASE("Read-only slicing placeholders expose the current numeric Z-hop", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(ReadOnlySlicingStatesConfigDef{}, {{"zhop", coFloat}});
}

TEST_CASE("Writable slicing placeholders retain vectors for per-axis and per-extruder state", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(ReadWriteSlicingStatesConfigDef{}, {
        {"position", coFloats}, {"e_retracted", coFloats}, {"e_restart_extra", coFloats}, {"e_position", coFloats}});
}

TEST_CASE("Slicing context placeholders distinguish indexes flags and names", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(OtherSlicingStatesConfigDef{}, {
        {"current_extruder", coInt}, {"current_object_idx", coInt}, {"has_wipe_tower", coBool},
        {"initial_extruder", coInt}, {"initial_tool", coInt}, {"is_extruder_used", coBools},
        {"num_extruders", coInt}, {"has_single_extruder_multi_material_priming", coBool},
        {"initial_no_support_extruder", coInt}, {"in_head_wrap_detect_zone", coBool}, {"curr_bed_type", coString}});
}

TEST_CASE("Print statistics placeholders distinguish totals from per-extruder quantities", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(PrintStatisticsConfigDef{}, {
        {"extruded_volume", coFloats}, {"total_toolchanges", coInt}, {"extruded_volume_total", coFloat},
        {"extruded_weight", coFloats}, {"extruded_weight_total", coFloat}, {"total_layer_count", coInt},
        {"normal_print_time", coString}, {"print_time", coString}, {"silent_print_time", coString},
        {"total_cost", coFloat}, {"total_weight", coFloat}, {"total_wipe_tower_cost", coFloat},
        {"total_wipe_tower_filament", coFloat}, {"used_filament", coFloat}, {"print_time_total_sec", coString},
        {"print_time_day", coString}, {"print_time_hour", coString}, {"print_time_minute", coString},
        {"print_time_sec", coString}, {"used_filament_length", coString}});
}

TEST_CASE("Object placeholders expose counts filenames and per-object scaling", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(ObjectsInfoConfigDef{}, {
        {"num_objects", coInt}, {"num_instances", coInt}, {"scale", coStrings},
        {"input_filename_base", coString}, {"input_filename", coString}, {"plate_name", coString}});
}

TEST_CASE("Dimension placeholders distinguish planar geometry from scalar height", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(DimensionsConfigDef{}, {
        {"first_layer_print_convex_hull", coPoints}, {"first_layer_print_min", coFloats},
        {"first_layer_print_max", coFloats}, {"first_layer_print_size", coFloats},
        {"print_bed_min", coFloats}, {"print_bed_max", coFloats}, {"print_bed_size", coFloats},
        {"first_layer_center_no_wipe_tower", coFloats}, {"first_layer_height", coFloat}});
}

TEST_CASE("Temperature placeholders distinguish per-filament values from aggregate values", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(TemperaturesConfigDef{}, {
        {"bed_temperature", coInts}, {"bed_temperature_initial_layer", coInts},
        {"bed_temperature_initial_layer_single", coInt}, {"chamber_temperature", coInts},
        {"chamber_minimal_temperature", coInts}, {"overall_chamber_temperature", coInt},
        {"first_layer_bed_temperature", coInts}, {"first_layer_temperature", coInts}});
}

TEST_CASE("Timestamp placeholders retain textual timestamps and numeric calendar fields", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(TimestampsConfigDef{}, {
        {"timestamp", coString}, {"year", coInt}, {"month", coInt}, {"day", coInt},
        {"hour", coInt}, {"minute", coInt}, {"second", coInt}});
}

TEST_CASE("Preset placeholders retain the public textual name fields", "[Config][ConfigDefinition][PlaceholderSchema]")
{
    check_schema(OtherPresetsConfigDef{}, {
        {"print_preset", coString}, {"filament_preset", coString},
        {"printer_preset", coString}, {"physical_printer_preset", coString}});
}
} // namespace config_definition_contracts

namespace config_footer_contracts {
using namespace Slic3r;

struct FooterDefinition : ConfigDef {
    FooterDefinition()
    {
        add("Alpha_key", coInt)->set_default_value(new ConfigOptionInt(0));
        add("text_value", coString)->set_default_value(new ConfigOptionString());
        add("decimal_value", coFloat)->set_default_value(new ConfigOptionFloat(0.));
        for (int i = 0; i < 80; ++i)
            add("option_" + std::to_string(i), coInt)->set_default_value(new ConfigOptionInt(0));
    }
};

struct FooterConfig : DynamicConfig {
    FooterDefinition definition;
    int composite_calls = 0;
    const ConfigDef *def() const override { return &definition; }
    void handle_legacy_composite() override { ++composite_calls; }
};

std::size_t load_legacy(FooterConfig &config, const std::string &data)
{
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Disable);
    std::size_t count = 0;
    REQUIRE_NOTHROW(count = ConfigBase::load_from_gcode_string_legacy(config, data.c_str(), context));
    CHECK(context.substitutions.empty());
    return count;
}

void write_file(const ScopedTemporaryFile &file, const std::string &data)
{
    boost::nowide::ofstream out(file.string(), std::ios::binary);
    REQUIRE(out.is_open());
    out << data;
    out.close();
    REQUIRE(out.good());
}

std::string block(int count = 80)
{
    std::string data = "; CONFIG_BLOCK_START\n";
    for (int i = 0; i < count; ++i)
        data += "; option_" + std::to_string(i) + " = " + std::to_string(10 + i) + "\n";
    return data + "; CONFIG_BLOCK_END\n";
}

void check_block_values(const FooterConfig &config)
{
    REQUIRE(config.keys().size() == 80);
    for (int i = 0; i < 80; ++i) {
        const std::string key = "option_" + std::to_string(i);
        CAPTURE(key);
        const auto *value = config.opt<ConfigOptionInt>(key);
        REQUIRE(value != nullptr);
        CHECK(value->value == 10 + i);
    }
    CHECK(config.composite_calls == 1);
}

TEST_CASE("Legacy G-code config ignores a null buffer without mutating state", "[Config][ConfigFooter]")
{
    FooterConfig config;
    config.set_deserialize_strict("Alpha_key", "17");
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Disable);
    CHECK(ConfigBase::load_from_gcode_string_legacy(config, nullptr, context) == 0);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 17);
    CHECK(config.composite_calls == 0);
    CHECK(context.substitutions.empty());
}

TEST_CASE("Legacy G-code config reads a contiguous tail across line endings", "[Config][ConfigFooter]")
{
    for (const std::string ending : {"\n", "\r\n", "\r"}) {
        CAPTURE(ending);
        FooterConfig config;
        const std::string data = "G1 X10" + ending + "; Alpha_key = 42" + ending +
                                 "; decimal_value = 0.375" + ending + ending;
        CHECK(load_legacy(config, data) == 2);
        REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
        CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 42);
        REQUIRE(config.opt<ConfigOptionFloat>("decimal_value") != nullptr);
        CHECK_THAT(config.opt<ConfigOptionFloat>("decimal_value")->value, Catch::Matchers::WithinAbs(0.375, 1e-12));
        CHECK(config.composite_calls == 1);
    }
}

TEST_CASE("Legacy G-code config removes numbered line prefixes", "[Config][ConfigFooter]")
{
    FooterConfig config;
    CHECK(load_legacy(config, "N1 G1 X10\nN20 ; Alpha_key = 37\nN300    ; option_7 = 83\n") == 2);
    REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
    REQUIRE(config.opt<ConfigOptionInt>("option_7") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 37);
    CHECK(config.opt<ConfigOptionInt>("option_7")->value == 83);
}

TEST_CASE("Legacy G-code config retains values containing equals and escaped newlines", "[Config][ConfigFooter]")
{
    FooterConfig config;
    CHECK(load_legacy(config, "G1 X0\n; text_value = first=second\\nthird\n") == 1);
    REQUIRE(config.opt<ConfigOptionString>("text_value") != nullptr);
    CHECK(config.opt<ConfigOptionString>("text_value")->value == "first=second\nthird");
}

TEST_CASE("Legacy G-code config skips unknown keys without terminating the tail", "[Config][ConfigFooter]")
{
    FooterConfig config;
    CHECK(load_legacy(config, "G1 X0\n; Alpha_key = 19\n; future_option = opaque\n; option_7 = 27\n") == 2);
    REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
    REQUIRE(config.opt<ConfigOptionInt>("option_7") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 19);
    CHECK(config.opt<ConfigOptionInt>("option_7")->value == 27);
    CHECK_FALSE(config.has("future_option"));
}

TEST_CASE("Legacy G-code config stops at malformed tail records", "[Config][ConfigFooter]")
{
    for (const std::string malformed : {"G1 X7", ";Alpha_key = 9", "; 3invalid = 9", "; invalid-key = 9",
                                        "; ab = 900", "; Alpha_key= 9", "; Alpha_key =9", "; Alpha_key 9"}) {
        CAPTURE(malformed);
        FooterConfig config;
        CHECK(load_legacy(config, "G1 X0\n; Alpha_key = 19\n" + malformed + "\n; option_7 = 27\n") == 1);
        CHECK_FALSE(config.has("Alpha_key"));
        REQUIRE(config.opt<ConfigOptionInt>("option_7") != nullptr);
        CHECK(config.opt<ConfigOptionInt>("option_7")->value == 27);
        CHECK(config.composite_calls == 1);
    }
}

TEST_CASE("Legacy G-code config refuses configuration followed by motion", "[Config][ConfigFooter]")
{
    FooterConfig config;
    CHECK(load_legacy(config, "G90\n; Alpha_key = 99\n; option_7 = 27\nG1 X10\n") == 0);
    CHECK(config.keys().empty());
    CHECK(config.composite_calls == 1);
}

TEST_CASE("Commented INI config removes generated headers and comment prefixes", "[Config][ConfigFooter]")
{
    FooterConfig config;
    ConfigSubstitutions substitutions;
    REQUIRE_NOTHROW(substitutions = config.load_from_ini_string_commented(
        "  ; generated by OrcaSlicer test\r\n\n\t; Alpha_key = 51\r\n ; decimal_value = 0.125\r\n"
        "text_value = alpha=beta\n; future_option = ignored\n", ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(substitutions.empty());
    REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
    REQUIRE(config.opt<ConfigOptionFloat>("decimal_value") != nullptr);
    REQUIRE(config.opt<ConfigOptionString>("text_value") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 51);
    CHECK_THAT(config.opt<ConfigOptionFloat>("decimal_value")->value, Catch::Matchers::WithinAbs(0.125, 1e-12));
    CHECK(config.opt<ConfigOptionString>("text_value")->value == "alpha=beta");
    CHECK_FALSE(config.has("future_option"));
    CHECK(config.composite_calls == 1);
}

TEST_CASE("Delimited G-code config finds both accepted headers after a preamble", "[Config][ConfigFooter]")
{
    for (const std::string header : {"; OrcaSlicer test", "; generated by OrcaSlicer test", " \tN12 ; OrcaSlicer test"}) {
        CAPTURE(header);
        ScopedTemporaryFile file(".gcode");
        write_file(file, "G90\nG1 X20\n" + header + "\n" + block() + "G1 X25\n; Alpha_key = 999\n");
        FooterConfig config;
        ConfigSubstitutions substitutions;
        REQUIRE_NOTHROW(substitutions = config.load_from_gcode_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable));
        CHECK(substitutions.empty());
        check_block_values(config);
        CHECK_FALSE(config.has("Alpha_key"));
    }
}

TEST_CASE("Delimited G-code config trims records and ignores malformed or unknown entries", "[Config][ConfigFooter]")
{
    ScopedTemporaryFile file(".gcode");
    std::string records = block();
    const auto insertion = records.find("; CONFIG_BLOCK_END");
    REQUIRE(insertion != std::string::npos);
    records.insert(insertion, "G1 X99\n; ignored text\n;future_option = ignored\n;\t Alpha_key \t= 61 \t\n");
    write_file(file, "; OrcaSlicer test\n" + records);
    FooterConfig config;
    ConfigSubstitutions substitutions;
    REQUIRE_NOTHROW(substitutions = config.load_from_gcode_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(substitutions.empty());
    REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 61);
    CHECK(config.keys().size() == 81);
    CHECK_FALSE(config.has("future_option"));
    CHECK(config.composite_calls == 1);
}

TEST_CASE("Delimited G-code config enforces the eighty-record threshold", "[Config][ConfigFooter]")
{
    ScopedTemporaryFile file(".gcode");
    write_file(file, "; OrcaSlicer test\n" + block(79));
    FooterConfig config;
    CHECK_THROWS_WITH(config.load_from_gcode_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable),
                      Catch::Matchers::ContainsSubstring("Suspiciously low number"));
    CHECK(config.composite_calls == 0);
}

TEST_CASE("Delimited G-code config reports missing headers and block delimiters", "[Config][ConfigFooter]")
{
    const std::vector<std::pair<std::string, std::string>> cases{
        {"G1 X0\n" + block(), "Not a gcode file generated by"},
        {"; OrcaSlicer test\nG1 X0\n", "CONFIG_BLOCK_START"},
        {"; OrcaSlicer test\n; CONFIG_BLOCK_START\n; Alpha_key = 3\n", "CONFIG_BLOCK_END"},
    };
    for (const auto &[data, error] : cases) {
        ScopedTemporaryFile file(".gcode");
        write_file(file, data);
        FooterConfig config;
        CHECK_THROWS_WITH(config.load_from_gcode_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable),
                          Catch::Matchers::ContainsSubstring(error));
        CHECK(config.composite_calls == 0);
    }
}

} // namespace config_footer_contracts

namespace config_residual_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

void check_values(const std::vector<double> &actual, const std::vector<double> &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(actual[i], WithinAbs(expected[i], 1e-9));
}

struct EnvironmentRestore {
    std::string name;
    std::string old_value;
    bool had_value;

    explicit EnvironmentRestore(std::string key) : name(std::move(key))
    {
        const char *value = std::getenv(name.c_str());
        had_value = value != nullptr;
        if (had_value)
            old_value = value;
    }

    ~EnvironmentRestore()
    {
        if (had_value)
            boost::nowide::setenv(name.c_str(), old_value.c_str(), 1);
        else
            boost::nowide::unsetenv(name.c_str());
    }
};

DynamicConfig dynamic_pair(double left, double right)
{
    DynamicConfig config;
    config.set_key_value("left", new ConfigOptionFloat(left));
    config.set_key_value("right", new ConfigOptionFloat(right));
    return config;
}

} // namespace

TEST_CASE("FloatOrPercent keeps absolute and percentage semantics distinct", "[Config][ConfigContracts]")
{
    const FloatOrPercent absolute(12.5, false);
    const FloatOrPercent percentage(12.5, true);

    CHECK_THAT(absolute.get_abs_value(80.), WithinAbs(12.5, 1e-9));
    CHECK_THAT(percentage.get_abs_value(80.), WithinAbs(10., 1e-9));
    CHECK(absolute != percentage);
    CHECK(absolute < percentage);
}

TEST_CASE("ConfigOptionFloatOrPercent parses and serializes both forms", "[Config][ConfigContracts]")
{
    ConfigOptionFloatOrPercent option;
    REQUIRE(option.deserialize("37.5%"));
    CHECK(option.percent);
    CHECK_THAT(option.value, WithinAbs(37.5, 1e-9));
    CHECK(option.serialize() == "37.5%");
    CHECK_THAT(option.get_abs_value(80.), WithinAbs(30., 1e-9));

    REQUIRE(option.deserialize("4.25"));
    CHECK_FALSE(option.percent);
    CHECK_THAT(option.value, WithinAbs(4.25, 1e-9));
    CHECK(option.serialize() == "4.25");
}

TEST_CASE("Nullable float-or-percent vectors preserve nil and value positions", "[Config][ConfigContracts]")
{
    ConfigOptionFloatsOrPercentsNullable option;
    REQUIRE(option.deserialize("nil,25%,3.5"));
    REQUIRE(option.values.size() == 3);
    CHECK(option.is_nil(0));
    CHECK_FALSE(option.is_nil(1));
    CHECK_FALSE(option.is_nil(2));
    CHECK(option.vserialize() == std::vector<std::string>{"nil", "25%", "3.5"});
    CHECK(option.serialize() == "nil,25%,3.5");
}

TEST_CASE("ConfigOptionPoint3 round trips comma and x separators", "[Config][ConfigContracts]")
{
    ConfigOptionPoint3 point(Vec3d(1.25, -2.5, 9.75));
    const std::string encoded = point.serialize();
    CHECK(encoded == "1.25,-2.5,9.75");

    ConfigOptionPoint3 decoded;
    REQUIRE(decoded.deserialize("1.25,-2.5,9.75"));
    CHECK_THAT(decoded.value.x(), WithinAbs(1.25, 1e-9));
    CHECK_THAT(decoded.value.y(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(decoded.value.z(), WithinAbs(9.75, 1e-9));
    REQUIRE(decoded.deserialize("1.25x-2.5x9.75"));
    CHECK_THAT(decoded.value.x(), WithinAbs(1.25, 1e-9));
    CHECK_THAT(decoded.value.y(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(decoded.value.z(), WithinAbs(9.75, 1e-9));
    CHECK_FALSE(decoded.deserialize("1.25x-2.5"));
}

TEST_CASE("ConfigOptionPointsGroups preserves group and point boundaries", "[Config][ConfigContracts]")
{
    const std::vector<Vec2ds> groups{
        Vec2ds{Vec2d(1., 2.), Vec2d(3., 4.)},
        Vec2ds{Vec2d(-5., 6.)}
    };
    ConfigOptionPointsGroups source(groups);
    const std::string encoded = source.serialize();
    CHECK(encoded == "1x2,3x4#-5x6");

    ConfigOptionPointsGroups decoded;
    decoded.values = {Vec2ds{Vec2d(99., 99.)}};
    REQUIRE(decoded.deserialize(encoded));
    REQUIRE(decoded.values.size() == 2);
    REQUIRE(decoded.values[0].size() == 2);
    REQUIRE(decoded.values[1].size() == 1);
    CHECK_THAT(decoded.values[0][1].x(), WithinAbs(3., 1e-9));
    CHECK_THAT(decoded.values[0][1].y(), WithinAbs(4., 1e-9));
    CHECK_THAT(decoded.values[1][0].x(), WithinAbs(-5., 1e-9));
    CHECK(decoded.vserialize_single(0) == std::vector<std::string>{"1x2", "3x4"});
}

TEST_CASE("ConfigOptionIntsGroups preserves append and per-group serialization", "[Config][ConfigContracts]")
{
    ConfigOptionIntsGroups groups({std::vector<int>{1, 2}, std::vector<int>{7}});
    CHECK(groups.serialize() == "1,2#7");
    CHECK(groups.vserialize() == std::vector<std::string>{"1,2", "7"});

    REQUIRE(groups.deserialize("9,10", true));
    REQUIRE(groups.values.size() == 3);
    CHECK(groups.values[0] == std::vector<int>{1, 2});
    CHECK(groups.values[2] == std::vector<int>{9, 10});
    CHECK(groups.vserialize_single(1) == std::vector<std::string>{"7"});
    REQUIRE(groups.deserialize("4,5", false));
    REQUIRE(groups.values.size() == 1);
    CHECK(groups.values[0] == std::vector<int>{4, 5});
}

TEST_CASE("ConfigSubstitutionContext empty tracks substitution records, not unknown keys", "[Config][ConfigContracts]")
{
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(context.empty());
    context.unrecogized_keys.emplace_back("future_option");
    CHECK(context.empty());
    CHECK(context.unrecogized_keys == std::vector<std::string>{"future_option"});
}

TEST_CASE("DynamicConfig equality, diff, equal, and skipped keys are coherent", "[Config][ConfigContracts]")
{
    DynamicConfig left = dynamic_pair(1., 2.);
    DynamicConfig right = dynamic_pair(1., 5.);

    CHECK_FALSE(left.equals(right));
    CHECK(left.equal(right) == t_config_option_keys{"left"});
    CHECK(left.diff(right) == t_config_option_keys{"right"});

    const auto report = left.diff_report(right);
    REQUIRE(report.is_different());
    REQUIRE(report.differences.count("right") == 1);
    CHECK(report.differences.at("right").is_different_value());
    CHECK(report.differences.at("right").left_value == "2");
    CHECK(report.differences.at("right").right_value == "5");

    const std::set<std::string> skipped{"right"};
    CHECK(left.equals(right, &skipped));
}

TEST_CASE("DynamicConfig diff report distinguishes missing keys", "[Config][ConfigContracts]")
{
    DynamicConfig left;
    DynamicConfig right;
    left.set_key_value("only_left", new ConfigOptionInt(3));
    right.set_key_value("only_right", new ConfigOptionString("yes"));

    const auto report = left.diff_report(right);
    REQUIRE(report.differences.size() == 2);
    REQUIRE(report.differences.count("only_left") == 1);
    REQUIRE(report.differences.count("only_right") == 1);
    CHECK(report.differences.at("only_left").left_value == "3");
    CHECK_FALSE(report.differences.at("only_left").right_value.has_value());
    CHECK_FALSE(report.differences.at("only_right").left_value.has_value());
    CHECK(report.differences.at("only_right").right_value == "yes");
}

TEST_CASE("DynamicConfig copy, move, plus, swap, and erase preserve ownership", "[Config][ConfigContracts]")
{
    DynamicConfig original;
    original.set_key_value("count", new ConfigOptionInt(4));

    DynamicConfig copied(original);
    copied.opt<ConfigOptionInt>("count")->value = 9;
    CHECK(original.opt<ConfigOptionInt>("count")->value == 4);
    CHECK_FALSE(original == copied);

    // Exercise the final, all-keys-equal return path as well as the early
    // mismatch path above.  A mutation of that final return must be visible
    // to this independent equality contract.
    DynamicConfig identical(original);
    CHECK(original == identical);

    DynamicConfig extra;
    extra.set_key_value("label", new ConfigOptionString("extra"));
    copied += extra;
    REQUIRE(copied.has("label"));
    CHECK(copied.opt_string("label") == "extra");

    DynamicConfig moved(std::move(copied));
    CHECK(copied.empty());
    CHECK(moved.size() == 2);
    DynamicConfig swapped;
    swapped.set_key_value("other", new ConfigOptionBool(true));
    moved.swap(swapped);
    CHECK(moved.has("other"));
    CHECK(swapped.has("count"));
    CHECK(swapped.erase("count"));
    CHECK_FALSE(swapped.erase("count"));
}

TEST_CASE("ConfigOptionFloats appends and resizes values with explicit defaults", "[Config][ConfigContracts]")
{
    ConfigOptionFloats storage({1., 2.});
    ConfigOptionFloats *values = &storage;
    const ConfigOptionFloat appended(3.);
    values->append(&appended);
    check_values(values->values, {1., 2., 3.});
    values->resize(5);
    check_values(values->values, {1., 2., 3., 1., 1.});
    values->clear();
    ConfigOptionFloat default_value(8.);
    values->resize(2, &default_value);
    check_values(values->values, {8., 8.});
}

TEST_CASE("DynamicConfig remove_nil_options removes only fully nil vectors", "[Config][ConfigContracts]")
{
    DynamicConfig config;
    config.set_key_value("nil", new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value()}));
    config.set_key_value("mixed", new ConfigOptionFloatsNullable({ConfigOptionFloatsNullable::nil_value(), 2.}));
    config.set_key_value("value", new ConfigOptionInt(7));

    const size_t removed = config.remove_nil_options();
    CHECK(removed == 1);
    CHECK_FALSE(config.has("nil"));
    CHECK(config.has("mixed"));
    CHECK(config.has("value"));
}

TEST_CASE("DynamicPrintConfig null_nullables clears nullable values without removing options", "[Config][ConfigContracts]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    auto *speed = config.option<ConfigOptionFloatsOrPercentsNullable>("small_perimeter_speed");
    REQUIRE(speed != nullptr);
    speed->values = {FloatOrPercent(37., false), FloatOrPercent(12., true)};
    std::string exception;
    bool threw = false;
    try {
        config.null_nullables();
    } catch (const std::exception &error) {
        threw = true;
        exception = error.what();
    } catch (...) {
        threw = true;
        exception = "non-standard exception";
    }
    INFO(exception);
    REQUIRE_FALSE(threw);
    REQUIRE(config.has("small_perimeter_speed"));
    CHECK(speed->is_nil());
    CHECK(speed->serialize() == "nil");
}

TEST_CASE("ConfigBase setenv serializes the selected option using uppercase names", "[Config][ConfigContracts]")
{
    DynamicConfig config;
    config.set_key_value("layer_height", new ConfigOptionFloat(0.275));
    EnvironmentRestore restore("SLIC3R_LAYER_HEIGHT");
    REQUIRE(boost::nowide::unsetenv("SLIC3R_LAYER_HEIGHT") == 0);

    config.setenv_();
    const char *value = std::getenv("SLIC3R_LAYER_HEIGHT");
    REQUIRE(value != nullptr);
    CHECK(std::string(value) == config.option<ConfigOptionFloat>("layer_height")->serialize());
}

TEST_CASE("Static print config exposes only its initialized option set", "[Config][ConfigContracts]")
{
    PrintObjectConfig object_config;
    const auto keys = object_config.keys();
    REQUIRE_FALSE(keys.empty());
    CHECK(object_config.has("layer_height"));
    CHECK_FALSE(object_config.has("not_a_print_object_option"));
    CHECK(object_config.def() != nullptr);
    CHECK(object_config.def()->has("layer_height"));
    CHECK_FALSE(object_config.def()->options.empty());
}

TEST_CASE("Config definitions retain defaults and metadata for a static option", "[Config][ConfigContracts]")
{
    PrintObjectConfig object_config;
    const ConfigOptionDef *definition = object_config.def()->get("layer_height");
    REQUIRE(definition != nullptr);
    REQUIRE(static_cast<bool>(definition->default_value));
    CHECK(definition->type == coFloat);
    CHECK(definition->default_value->serialize() == object_config.option("layer_height")->serialize());
    CHECK(object_config.def()->keys().size() >= object_config.keys().size());
}

TEST_CASE("DynamicPrintConfig set_deserialize updates typed values and absolute ratios", "[Config][ConfigContracts]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", 0.24},
        {"outer_wall_line_width", "120%"},
        {"inner_wall_line_width", 0.42}
    });
    CHECK_THAT(config.option<ConfigOptionFloat>("layer_height")->value, WithinAbs(0.24, 1e-9));
    CHECK(config.option<ConfigOptionFloatOrPercent>("outer_wall_line_width")->percent);
    CHECK_THAT(config.get_abs_value("outer_wall_line_width", 0.42), WithinAbs(0.504, 1e-9));
}

TEST_CASE("ConfigOptionFloats vector set_at and set range preserve defaults", "[Config][ConfigContracts]")
{
    ConfigOptionFloats destination({10., 10., 10.});
    ConfigOptionFloats source({1., 2., 3., 4.});
    destination.set_at(&source, 1, 1);
    check_values(destination.values, {10., 2., 10.});
    destination.set(&source, 2, 2);
    check_values(destination.values, {3., 4.});
}

TEST_CASE("ConfigOptionFloats nullable index assignment retains nil state", "[Config][ConfigContracts]")
{
    ConfigOptionFloatsNullable option({1., 2., 3.});
    option.set_at_to_nil(1);
    REQUIRE(option.is_nil(1));
    CHECK_FALSE(option.is_nil(0));
    CHECK(std::isnan(option.get_at(1)));
    CHECK(option.serialize() == "1,nil,3");
}

TEST_CASE("ConfigOptionPoint3 clone and assignment retain all coordinates", "[Config][ConfigContracts]")
{
    ConfigOptionPoint3 source(Vec3d(4., 5., 6.));
    std::unique_ptr<ConfigOption> clone(source.clone());
    auto *typed = dynamic_cast<ConfigOptionPoint3 *>(clone.get());
    REQUIRE(typed != nullptr);
    CHECK(*typed == source);

    ConfigOptionPoint3 assigned;
    assigned = &source;
    CHECK_THAT((assigned.value - Vec3d(4., 5., 6.)).norm(), WithinAbs(0., 1e-9));
}

TEST_CASE("ConfigOptionPointsGroups append and clone preserve nested values", "[Config][ConfigContracts]")
{
    ConfigOptionPointsGroups source({Vec2ds{Vec2d(1., 1.)}});
    ConfigOptionPointsGroups extra({Vec2ds{Vec2d(2., 2.), Vec2d(3., 3.)}});
    source.append(&extra);
    REQUIRE(source.values.size() == 2);
    CHECK(source.values[1].size() == 2);
    std::unique_ptr<ConfigOption> clone(source.clone());
    auto *typed = dynamic_cast<ConfigOptionPointsGroups *>(clone.get());
    REQUIRE(typed != nullptr);
    CHECK(*typed == source);
}

TEST_CASE("Grouped integer settings form a strict lexicographic order", "[Config][ConfigContracts][ConfigGroupOrder]")
{
    ConfigOptionIntsGroups value({std::vector<int>{1, 9}});
    REQUIRE_FALSE(value < value);
    const ConfigOptionIntsGroups identical(value);
    CHECK_FALSE(value < identical);
    CHECK_FALSE(identical < value);
    const ConfigOptionIntsGroups empty;
    CHECK_FALSE(empty < empty);
    CHECK(empty < value);
    CHECK_FALSE(value < empty);
    const ConfigOptionIntsGroups longer({std::vector<int>{1, 9}, std::vector<int>{2}});
    CHECK(value < longer);
    CHECK_FALSE(longer < value);
    const ConfigOptionIntsGroups higher({std::vector<int>{2, 0}});
    CHECK(value < higher);
    CHECK_FALSE(higher < value);
}

TEST_CASE("ConfigBase equality ignores missing options but reports present differences", "[Config][ConfigContracts]")
{
    DynamicConfig left;
    DynamicConfig right;
    left.set_key_value("shared", new ConfigOptionString("same"));
    left.set_key_value("left_only", new ConfigOptionInt(1));
    right.set_key_value("shared", new ConfigOptionString("same"));
    right.set_key_value("right_only", new ConfigOptionInt(2));

    const ConfigBase &left_base = left;
    const ConfigBase &right_base = right;
    CHECK(left_base.equals(right_base));
    CHECK(left_base.equal(right_base) == t_config_option_keys{"shared"});
    CHECK(left_base.diff(right_base).empty());

    right.opt_string("shared") = "changed";
    CHECK_FALSE(left_base.equals(right_base));
    CHECK(left_base.diff(right_base) == t_config_option_keys{"shared"});

    const std::set<std::string> skipped{"shared"};
    CHECK(left_base.equals(right_base, &skipped));
}

TEST_CASE("ConfigOptionString empty distinguishes empty and nonempty values", "[Config][ConfigContracts]")
{
    ConfigOptionString empty;
    ConfigOptionString populated("config");
    CHECK(empty.empty());
    CHECK_FALSE(populated.empty());
    CHECK(populated.serialize() == "config");
    REQUIRE(populated.deserialize(""));
    CHECK(populated.empty());
}

namespace {

void check_point_components(const Vec2d &actual, double expected_x, double expected_y)
{
    CHECK_THAT(actual.x(), WithinAbs(expected_x, 1e-9));
    CHECK_THAT(actual.y(), WithinAbs(expected_y, 1e-9));
}

} // namespace

TEST_CASE("ConfigOption base accessors fail closed and scalar overrides copy values", "[Config][ConfigContracts][ConfigBase]")
{
    ConfigOptionString text("left");
    ConfigOption &base = text;

    REQUIRE_THROWS_AS(base.getInt(), BadOptionTypeException);
    REQUIRE_THROWS_AS(base.getFloat(), BadOptionTypeException);
    REQUIRE_THROWS_AS(base.getBool(), BadOptionTypeException);
    REQUIRE_THROWS_AS(base.setInt(9), BadOptionTypeException);

    ConfigOptionInt number(3);
    ConfigOption &number_base = number;
    number_base.setInt(-7);
    CHECK(number_base.getInt() == -7);

    ConfigOptionString same("left");
    ConfigOptionString replacement("right");
    std::vector<int> default_index{0};
    CHECK_FALSE(base.overriden_by(&same));
    CHECK(base.overriden_by(&replacement));
    REQUIRE(base.apply_override(&replacement, default_index));
    CHECK(text.value == "right");
    CHECK_FALSE(base.apply_override(&text, default_index));
}

TEST_CASE("ConfigOptionPoint preserves scalar geometry through parsing and ownership", "[Config][ConfigContracts][ConfigPoint]")
{
    ConfigOptionPoint source(Vec2d(1.25, -2.5));
    CHECK(source.serialize() == "1.25,-2.5");

    REQUIRE(source.deserialize("1.25x-2.5"));
    check_point_components(source.value, 1.25, -2.5);
    REQUIRE(source.deserialize(" 3.5 , 4.25 "));
    CHECK(source.serialize() == "3.5,4.25");
    CHECK_FALSE(source.deserialize("3.5x4.25x9"));
    CHECK_FALSE(source.deserialize("not-a-point"));
    REQUIRE(source.deserialize("3.5,4.25"));
    check_point_components(source.value, 3.5, 4.25);

    ConfigOptionPoint assigned;
    assigned = &source;
    CHECK(assigned == source);
    std::unique_ptr<ConfigOption> clone(source.clone());
    auto *typed_clone = dynamic_cast<ConfigOptionPoint *>(clone.get());
    REQUIRE(typed_clone != nullptr);
    CHECK(*typed_clone == source);

    ConfigOptionPoint lower(Vec2d(3.5, 4.0));
    CHECK(lower < source);
    CHECK_FALSE(source < lower);
}

TEST_CASE("ConfigOptionPoints bounds vector parsing, order, and append semantics", "[Config][ConfigContracts][ConfigPoints]")
{
    const std::vector<Vec2d> values{Vec2d(1., 2.), Vec2d(-3., 4.)};
    ConfigOptionPoints source(values);
    CHECK(source.serialize() == "1x2,-3x4");
    CHECK(source.vserialize() == std::vector<std::string>{"1x2", "-3x4"});
    CHECK_FALSE(source.is_nil(0));

    ConfigOptionPoints decoded;
    REQUIRE(decoded.deserialize("1x2,-3x4,5x6"));
    REQUIRE(decoded.values.size() == 3);
    check_point_components(decoded.values[2], 5., 6.);
    CHECK(source < decoded);
    CHECK_FALSE(decoded < source);

    ConfigOptionPoints assigned(3, Vec2d(9., 9.));
    assigned = &source;
    CHECK(assigned == source);
    std::unique_ptr<ConfigOption> clone(source.clone());
    auto *typed_clone = dynamic_cast<ConfigOptionPoints *>(clone.get());
    REQUIRE(typed_clone != nullptr);
    CHECK(*typed_clone == source);

    ConfigOptionPoints extra({Vec2d(8., 9.)});
    source.append(&extra);
    CHECK(source.values.size() == 3);
    CHECK(source.serialize() == "1x2,-3x4,8x9");
}

TEST_CASE("Grouped point options preserve empty group boundaries", "[Config][ConfigContracts][ConfigGrouped]")
{
    ConfigOptionPointsGroups source({Vec2ds{}, Vec2ds{Vec2d(1., 2.)}});
    CHECK(source.serialize() == "#1x2");
    CHECK(source.vserialize() == std::vector<std::string>{"", "1x2"});
    CHECK(source.vserialize_single(0).empty());
    CHECK_FALSE(source.is_nil(0));

    ConfigOptionPointsGroups decoded;
    REQUIRE(decoded.deserialize("#1x2"));
    REQUIRE(decoded.values.size() == 2);
    CHECK(decoded.values[0].empty());
    REQUIRE(decoded.values[1].size() == 1);
    check_point_components(decoded.values[1][0], 1., 2.);
}

TEST_CASE("Nullable integer vectors retain nil positions and canonical serialization", "[Config][ConfigContracts][ConfigNullable]")
{
    ConfigOptionIntsNullable option({1, ConfigOptionIntsNullable::nil_value(), -2});
    CHECK(option.nullable());
    CHECK_FALSE(option.is_nil());
    CHECK_FALSE(option.is_nil(0));
    CHECK(option.is_nil(1));
    CHECK(option.serialize() == "1,nil,-2");
    CHECK(option.vserialize() == std::vector<std::string>{"1", "nil", "-2"});
    const ConfigOptionIntsNullable identical(option);
    CHECK(option == identical);
    const ConfigOptionIntsNullable higher({2, ConfigOptionIntsNullable::nil_value(), -2});
    CHECK(option < higher);

    ConfigOptionIntsNullable nilled({4, 5});
    nilled.set_at_to_nil(1);
    CHECK(nilled.is_nil(1));

    ConfigOptionIntsNullable decoded;
    REQUIRE(decoded.deserialize("nil,4"));
    REQUIRE(decoded.values.size() == 2);
    CHECK(decoded.is_nil(0));
    CHECK(decoded.values[1] == 4);
    REQUIRE(decoded.deserialize("8", true));
    CHECK(decoded.values == std::vector<int>{ConfigOptionIntsNullable::nil_value(), 4, 8});

    ConfigOptionInts non_nullable({1});
    REQUIRE_THROWS_AS(non_nullable.deserialize("nil"), ConfigurationError);
}

TEST_CASE("Boolean substitution helpers distinguish enum-shaped and truthy values", "[Config][ConfigContracts][ConfigDeserialize]")
{
    const std::vector<std::pair<std::string, bool>> enum_cases{
        {"enabled", true}, {" On ", true}, {"off", true},
        {"yes please", false}, {"1future", false}, {"a.b", false},
        {"", false}, {std::string(64, 'a'), true}, {std::string(65, 'a'), false}
    };
    for (const auto &[value, expected] : enum_cases) {
        CAPTURE(value);
        CHECK(ConfigHelpers::looks_like_enum_value(value) == expected);
    }
    CHECK(ConfigHelpers::enum_looks_like_true_value(" ENABLED "));
    CHECK(ConfigHelpers::enum_looks_like_true_value("on"));
    CHECK_FALSE(ConfigHelpers::enum_looks_like_true_value("yes"));
    CHECK_FALSE(ConfigHelpers::enum_looks_like_true_value("true"));

    ConfigOptionBoolsNullable nullable;
    CHECK(nullable.deserialize_with_substitutions(
              "nil, ENABLED, off", false, ConfigHelpers::DeserializationSubstitution::DefaultsToFalse)
          == ConfigHelpers::DeserializationResult::Substituted);
    CHECK(nullable.values == std::vector<unsigned char>{ConfigOptionBoolsNullable::nil_value(), 1, 0});
    CHECK(nullable.serialize() == "nil,1,0");
    CHECK(nullable.deserialize("1,maybe") == false);
}

namespace {

struct MigrationDefinition : ConfigDef {
    MigrationDefinition()
    {
        add("new_int", coInt)->set_default_value(new ConfigOptionInt(0));
    }
};

struct MigrationConfig : DynamicConfig {
    MigrationDefinition definition;

    const ConfigDef *def() const override { return &definition; }

protected:
    void handle_legacy(t_config_option_key &key, std::string &value) const override
    {
        if (key == "legacy_int") {
            key = "new_int";
            value = "42";
        } else if (key == "removed_int") {
            key.clear();
        }
    }
};

} // namespace

TEST_CASE("ConfigBase migration rewrites and records removed keys, then rejects unknown keys", "[Config][ConfigContracts][ConfigMigration]")
{
    MigrationConfig config;
    ConfigSubstitutionContext context(ForwardCompatibilitySubstitutionRule::Disable);

    REQUIRE(config.set_deserialize_nothrow("legacy_int", "7", context));
    REQUIRE(config.opt<ConfigOptionInt>("new_int") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("new_int")->value == 42);

    REQUIRE(config.set_deserialize_nothrow("removed_int", "ignored", context));
    REQUIRE(config.set_deserialize_nothrow("removed_int", "ignored-again", context));
    CHECK(context.unrecogized_keys == std::vector<std::string>{"removed_int"});
    CHECK(context.empty());
    REQUIRE_THROWS_AS(config.set_deserialize_strict("future_int", "7"), UnknownOptionException);
}

TEST_CASE("Representative scalar options copy through the virtual override setter", "[Config][ConfigContracts][ConfigScalarOverride]")
{
    const std::vector<int> initial_index{7, -1};
    std::vector<int> default_index = initial_index;

    ConfigOptionString text("left");
    ConfigOptionString replacement_text("right");
    ConfigOption &text_base = text;
    REQUIRE(text_base.apply_override(&replacement_text, default_index));
    CHECK(text.value == "right");

    ConfigOptionInt number(3);
    ConfigOptionInt replacement_number(7);
    ConfigOption &number_base = number;
    REQUIRE(number_base.apply_override(&replacement_number, default_index));
    CHECK(number.value == 7);

    ConfigOptionFloat decimal(1.25);
    ConfigOptionFloat replacement_decimal(2.5);
    ConfigOption &decimal_base = decimal;
    REQUIRE(decimal_base.apply_override(&replacement_decimal, default_index));
    CHECK_THAT(decimal.value, WithinAbs(2.5, 1e-12));

    ConfigOptionBool flag(false);
    ConfigOptionBool replacement_flag(true);
    ConfigOption &flag_base = flag;
    REQUIRE(flag_base.apply_override(&replacement_flag, default_index));
    CHECK(flag.value);

    ConfigOptionPoint point(Vec2d(1., 2.));
    ConfigOptionPoint replacement_point(Vec2d(3., 4.));
    ConfigOption &point_base = point;
    REQUIRE(point_base.apply_override(&replacement_point, default_index));
    CHECK_THAT(point.value.x(), WithinAbs(3., 1e-12));
    CHECK_THAT(point.value.y(), WithinAbs(4., 1e-12));

    CHECK_FALSE(number_base.apply_override(&replacement_number, default_index));
    CHECK(default_index == initial_index);
}

TEST_CASE("Derived scalar options preserve subtype state through override", "[Config][ConfigContracts][ConfigScalarOverride]")
{
    const std::vector<int> initial_index{3, 1};
    std::vector<int> default_index = initial_index;

    ConfigOptionFloatOrPercent absolute(12.5, false);
    ConfigOptionFloatOrPercent percentage(25., true);
    ConfigOption &absolute_base = absolute;
    REQUIRE(absolute_base.apply_override(&percentage, default_index));
    CHECK_THAT(absolute.value, WithinAbs(25., 1e-12));
    CHECK(absolute.percent);
    CHECK(absolute.serialize() == "25%");

    const t_config_enum_values keys{{"marlin", int(gcfMarlinLegacy)},
                                    {"reprap", int(gcfRepRapFirmware)}};
    ConfigOptionEnumGeneric generic(&keys, int(gcfMarlinLegacy));
    ConfigOptionEnum<GCodeFlavor> specialized(gcfRepRapFirmware);
    ConfigOption &generic_base = generic;
    REQUIRE(generic_base.apply_override(&specialized, default_index));
    CHECK(generic.value == int(gcfRepRapFirmware));
    CHECK(generic.keys_map == &keys);
    CHECK(generic.serialize() == "reprap");

    ConfigOptionEnum<GCodeFlavor> specialized_target(gcfMarlinLegacy);
    ConfigOption &specialized_base = specialized_target;
    REQUIRE(specialized_base.apply_override(&generic, default_index));
    CHECK(specialized_target.value == gcfRepRapFirmware);
    CHECK(default_index == initial_index);
}

TEST_CASE("Scalar overrides reject incompatible types before changing state", "[Config][ConfigContracts][ConfigScalarOverride]")
{
    const std::vector<int> initial_index{5};
    std::vector<int> default_index = initial_index;
    ConfigOptionString text("left");
    ConfigOptionInt incompatible(7);
    ConfigOption &text_base = text;

    REQUIRE_THROWS_AS(text_base.apply_override(&incompatible, default_index), ConfigurationError);
    CHECK(text.value == "left");
    CHECK(default_index == initial_index);
}

} // namespace config_residual_contracts

TEST_CASE("Prime tower normalization preserves wrapping and smooth timelapse exceptions", "[PrintConfigBehavior][Config][NormalizeFdm2Variance]")
{
    auto make_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set("enable_prime_tower", true);
        config.set("independent_support_layer_height", true);
        config.set("enable_wrapping_detection", false);
        config.set_deserialize_strict("filament_is_mixed", "0");
        config.set_deserialize_strict("timelapse_type", "0");
        config.set_deserialize_strict("print_sequence", "by layer");
        return config;
    };

    SECTION("wrapping detection keeps the tower but still disables independent support layers")
    {
        DynamicPrintConfig config = make_config();
        config.set("enable_wrapping_detection", true);
        const t_config_option_keys changed = config.normalize_fdm_2(1, 1);
        REQUIRE(config.opt_bool("enable_prime_tower"));
        REQUIRE_FALSE(config.opt_bool("independent_support_layer_height"));
        CHECK(changed == t_config_option_keys{"independent_support_layer_height"});

        const t_config_option_keys second = config.normalize_fdm_2(1, 1);
        CHECK(second.empty());
        CHECK(config.opt_bool("enable_prime_tower"));
        CHECK_FALSE(config.opt_bool("independent_support_layer_height"));
    }

    SECTION("smooth timelapse keeps the tower but still disables independent support layers")
    {
        DynamicPrintConfig config = make_config();
        config.set_deserialize_strict("timelapse_type", "1");
        const t_config_option_keys changed = config.normalize_fdm_2(1, 1);
        REQUIRE(config.opt_bool("enable_prime_tower"));
        REQUIRE_FALSE(config.opt_bool("independent_support_layer_height"));
        CHECK(changed == t_config_option_keys{"independent_support_layer_height"});

        const t_config_option_keys second = config.normalize_fdm_2(1, 1);
        CHECK(second.empty());
        CHECK(config.opt_bool("enable_prime_tower"));
        CHECK_FALSE(config.opt_bool("independent_support_layer_height"));
    }

    SECTION("multiple by-object objects disable the tower")
    {
        DynamicPrintConfig config = make_config();
        config.set_deserialize_strict("print_sequence", "by object");
        const t_config_option_keys changed = config.normalize_fdm_2(2, 2);
        REQUIRE_FALSE(config.opt_bool("enable_prime_tower"));
        CHECK(config.opt_bool("independent_support_layer_height"));
        CHECK(changed == t_config_option_keys{"enable_prime_tower"});

        const t_config_option_keys second = config.normalize_fdm_2(2, 2);
        CHECK(second.empty());
        CHECK_FALSE(config.opt_bool("enable_prime_tower"));
        CHECK(config.opt_bool("independent_support_layer_height"));
    }
}
