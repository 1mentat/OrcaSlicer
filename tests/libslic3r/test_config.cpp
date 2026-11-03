#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/PrintConfigConstants.hpp"
#include "libslic3r/LocalesUtils.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/libslic3r.h"

#include "test_utils.hpp"

#include <cereal/types/polymorphic.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/archives/binary.hpp>

#include <boost/nowide/fstream.hpp>
#include "libslic3r/Config.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <iomanip>
#include <ios>
#include <limits>
#include <string>
#include <initializer_list>
#include <memory>
#include <ostream>
#include <map>
#include <iterator>
#include <nlohmann/json.hpp>

#include <set>
#include <sstream>
#include <string>
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

TEST_CASE("Config JSON loading parses grouped geometry in flat and nested forms", "[Config]") {
    const std::string area_json = GENERATE(
        std::string(R"(["0x0,20x0,20x20,0x20","5x5,15x5,15x15,5x15"])"),
        std::string(R"([["0x0,20x0,20x20,0x20"],["5x5,15x5,15x15,5x15"]])"));
    ScopedTemporaryFile tmp(".json");
    {
        boost::nowide::ofstream out(tmp.string());
        REQUIRE(out.is_open());
        out << "{\"extruder_printable_area\":" << area_json << "}";
        out.close();
        REQUIRE(out.good());
    }

    // Exercise ConfigBase's schema-driven JSON loader without DynamicPrintConfig's
    // print-specific legacy filtering, while retaining the registered print option types.
    struct PrintSchemaConfig : DynamicConfig {
        const ConfigDef *def() const override { return &print_config_def; }
    };
    PrintSchemaConfig config;
    const ConfigSubstitutions substitutions = config.load(
        tmp.string(), ForwardCompatibilitySubstitutionRule::Disable);
    CHECK(substitutions.empty());

    const ConfigOptionPointsGroups *areas = config.option<ConfigOptionPointsGroups>("extruder_printable_area");
    REQUIRE(areas != nullptr);
    REQUIRE(areas->values.size() == 2);
    const Vec2ds expected_areas[] = {
        { Vec2d(0., 0.), Vec2d(20., 0.), Vec2d(20., 20.), Vec2d(0., 20.) },
        { Vec2d(5., 5.), Vec2d(15., 5.), Vec2d(15., 15.), Vec2d(5., 15.) },
    };
    for (size_t group = 0; group < areas->values.size(); ++group) {
        REQUIRE(areas->values[group].size() == expected_areas[group].size());
        for (size_t point = 0; point < expected_areas[group].size(); ++point) {
            CHECK_THAT(areas->values[group][point].x(), Catch::Matchers::WithinAbs(expected_areas[group][point].x(), 1e-9));
            CHECK_THAT(areas->values[group][point].y(), Catch::Matchers::WithinAbs(expected_areas[group][point].y(), 1e-9));
        }
    }
}

TEST_CASE("Config JSON loading joins an aliased array in its target option's format", "[Config]") {
    // The only alias in print_config_def is a floats option, whose ',' separator
    // is also the loader's fallback, so declare an aliased strings option here.
    // Strings must be joined as quoted, escaped, ';'-separated items; only the
    // loader's own alias lookup can tell that "legacy_labels" is a strings option.
    struct AliasConfigDef : ConfigDef {
        AliasConfigDef()
        {
            ConfigOptionDef *def = this->add("labels", coStrings);
            def->aliases = { "legacy_labels" };
            def->set_default_value(new ConfigOptionStrings());
        }
    };
    static const AliasConfigDef alias_config_def;
    struct AliasConfig : DynamicConfig {
        const ConfigDef *def() const override { return &alias_config_def; }
    };

    ScopedTemporaryFile tmp(".json");
    {
        boost::nowide::ofstream out(tmp.string());
        REQUIRE(out.is_open());
        out << R"({"legacy_labels":["first, with comma","second; \"quoted\""]})";
        out.close();
        REQUIRE(out.good());
    }

    AliasConfig config;
    const ConfigSubstitutions substitutions = config.load(tmp.string(), ForwardCompatibilitySubstitutionRule::Disable);
    CHECK(substitutions.empty());
    const ConfigOptionStrings *labels = config.option<ConfigOptionStrings>("labels");
    REQUIRE(labels != nullptr);
    CHECK(labels->values == std::vector<std::string>{"first, with comma", "second; \"quoted\""});
    CHECK(config.option("legacy_labels") == nullptr);
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

TEST_CASE("read_cli stops option parsing after the positional delimiter", "[Config][ConfigCLI]")
{
    Slic3r::DynamicPrintConfig config;
    t_config_option_keys extra, keys;
    const char* argv[] = {
        "orca-slicer", "--nozzle-temperature", "210", "--", "--top-shell-layers", "model.3mf"
    };

    REQUIRE(config.read_cli(6, argv, &extra, &keys));
    const auto* temperature = config.opt<Slic3r::ConfigOptionInts>("nozzle_temperature");
    REQUIRE(temperature != nullptr);
    CHECK(temperature->values == std::vector<int>{210});
    CHECK(extra == t_config_option_keys{"--top-shell-layers", "model.3mf"});
    CHECK(keys == t_config_option_keys{"nozzle_temperature"});
}

TEST_CASE("read_cli rejects unknown options and missing scalar values", "[Config][ConfigCLI]")
{
    SECTION("unknown option")
    {
        Slic3r::DynamicPrintConfig config;
        t_config_option_keys extra, keys;
        const char* argv[] = {"orca-slicer", "--not-an-orca-setting"};

        CHECK_FALSE(config.read_cli(2, argv, &extra, &keys));
        CHECK(extra.empty());
        CHECK(keys.empty());
        CHECK(config.keys().empty());
    }

    SECTION("missing scalar value")
    {
        Slic3r::DynamicPrintConfig config;
        t_config_option_keys extra, keys;
        const char* argv[] = {"orca-slicer", "--top-shell-layers"};

        CHECK_FALSE(config.read_cli(2, argv, &extra, &keys));
        CHECK(extra.empty());
        CHECK(keys.empty());
        CHECK(config.keys().empty());
    }
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

TEST_CASE("Forward-compatible settings import substitutes typed defaults and records changes",
          "[Config][ConfigSubstitution]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const ConfigOption *flavor_before = config.option("gcode_flavor");
    REQUIRE(flavor_before != nullptr);
    const std::string default_flavor = flavor_before->serialize();
    // Start each option away from its default so a substitution with the default is observable.
    config.set_deserialize_strict("gcode_flavor", default_flavor == "klipper" ? "reprapfirmware" : "klipper");
    REQUIRE(config.option("gcode_flavor")->serialize() != default_flavor);

    ConfigSubstitutionContext enum_context(ForwardCompatibilitySubstitutionRule::Enable);
    config.set_deserialize("gcode_flavor", "future_flavor", enum_context);
    REQUIRE(enum_context.substitutions.size() == 1);
    CHECK(enum_context.substitutions.front().old_value == "future_flavor");
    REQUIRE(enum_context.substitutions.front().new_value != nullptr);
    CHECK(enum_context.substitutions.front().new_value->serialize() == default_flavor);
    CHECK(config.option("gcode_flavor")->serialize() == default_flavor);

    config.set_deserialize_strict("gcode_comments", "0");
    ConfigSubstitutionContext bool_context(ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(config.set_deserialize_nothrow("gcode_comments", "on", bool_context));
    CHECK(config.opt<ConfigOptionBool>("gcode_comments")->value);
    CHECK(bool_context.substitutions.empty());

    config.set_deserialize_strict("enable_overhang_bridge_fan", "0");
    ConfigSubstitutionContext default_true_context(ForwardCompatibilitySubstitutionRule::Enable);
    config.set_deserialize("enable_overhang_bridge_fan", "future_cooling_value", default_true_context);
    CHECK(config.opt<ConfigOptionBools>("enable_overhang_bridge_fan")->values == std::vector<unsigned char>{1});
    REQUIRE(default_true_context.substitutions.size() == 1);
    CHECK(default_true_context.substitutions.front().new_value->serialize() == "1");

    config.set_deserialize_strict("filament_soluble", "1");
    ConfigSubstitutionContext default_false_context(ForwardCompatibilitySubstitutionRule::Enable);
    config.set_deserialize("filament_soluble", "future_material_value", default_false_context);
    CHECK(config.opt<ConfigOptionBools>("filament_soluble")->values == std::vector<unsigned char>{0});
    REQUIRE(default_false_context.substitutions.size() == 1);
    CHECK(default_false_context.substitutions.front().new_value->serialize() == "0");

    ConfigSubstitutionContext numeric_context(ForwardCompatibilitySubstitutionRule::Enable);
    CHECK_FALSE(config.set_deserialize_nothrow("layer_height", "not-a-number", numeric_context));
    CHECK(numeric_context.substitutions.empty());

    ConfigSubstitutionContext nil_context(ForwardCompatibilitySubstitutionRule::Enable);
    CHECK_THROWS_AS(config.set_deserialize("filament_soluble", "nil", nil_context), ConfigurationError);

    DynamicConfig no_definition;
    ConfigSubstitutionContext no_definition_context(ForwardCompatibilitySubstitutionRule::Disable);
    CHECK_THROWS_AS(no_definition.set_deserialize("some_option", "1", no_definition_context), NoDefinitionException);
}

TEST_CASE("Config substitutions order by option key and old value", "[Config][ConfigSubstitution]")
{
    ConfigOptionDef first_definition;
    first_definition.opt_key = "first_option";
    ConfigOptionDef second_definition;
    second_definition.opt_key = "second_option";

    ConfigSubstitution first;
    first.opt_def = &first_definition;
    first.old_value = "old-a";
    ConfigSubstitution same_value;
    same_value.opt_def = &first_definition;
    same_value.old_value = "old-a";
    ConfigSubstitution later_value;
    later_value.opt_def = &first_definition;
    later_value.old_value = "old-b";
    ConfigSubstitution later_option;
    later_option.opt_def = &second_definition;
    // An older value on a later key: the key orders first, then the old value.
    later_option.old_value = "old-0";

    CHECK(first == same_value);
    CHECK_FALSE(first == later_value);
    CHECK(first < later_value);
    CHECK_FALSE(later_value < first);
    CHECK(first < later_option);
    CHECK_FALSE(later_option < first);
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

TEST_CASE("Legacy SLA vector corrections are expanded into axis options", "[Config][ConfigLegacy]")
{
    DynamicPrintConfig config;
    config.set_key_value("relative_correction", new ConfigOptionFloats({1.1, 1.2}));
    config.set_key_value("material_correction", new ConfigOptionFloats({0.9, 0.8}));
    // An axis option that is already present is not overwritten from the legacy vector.
    config.set_key_value("material_correction_y", new ConfigOptionFloat(0.7));
    handle_legacy_sla(config);

    CHECK_THAT(config.opt_float("relative_correction_x"), Catch::Matchers::WithinAbs(1.1, 1e-9));
    CHECK_THAT(config.opt_float("relative_correction_y"), Catch::Matchers::WithinAbs(1.1, 1e-9));
    CHECK_THAT(config.opt_float("relative_correction_z"), Catch::Matchers::WithinAbs(1.2, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_x"), Catch::Matchers::WithinAbs(0.9, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_y"), Catch::Matchers::WithinAbs(0.7, 1e-9));
    CHECK_THAT(config.opt_float("material_correction_z"), Catch::Matchers::WithinAbs(0.8, 1e-9));
}

TEST_CASE("Legacy composite thumbnail values become canonical definitions", "[Config][ConfigLegacy]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    // A size without a format takes the legacy thumbnails_format, which is not the default PNG here.
    config.set_deserialize_strict("thumbnails_format", "JPG");
    config.set("thumbnails", "16x16,32x32/PNG");
    config.handle_legacy_composite();
    CHECK(config.opt_string("thumbnails") == "16x16/JPG, 32x32/PNG");
}

TEST_CASE("Legacy composite wiping matrix detects whether custom entries exist", "[Config][ConfigLegacy]")
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

TEST_CASE("ConfigBase migration rewrites and records removed keys, then rejects unknown keys", "[Config][ConfigMigration]")
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

namespace {

struct AliasContractDefinition : ConfigDef {
    AliasContractDefinition()
    {
        ConfigOptionDef *canonical = add("canonical_int", coInt);
        canonical->set_default_value(new ConfigOptionInt(0));
        canonical->aliases.emplace_back("legacy_int_alias");
    }
};

struct AliasContractConfig : DynamicConfig {
    AliasContractDefinition definition;

    const ConfigDef *def() const override { return &definition; }
};

} // namespace

TEST_CASE("Config aliases deserialize into their canonical setting",
          "[Config][ConfigMigration]")
{
    AliasContractConfig config;
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    config.set_deserialize("legacy_int_alias", "42", substitutions);

    const ConfigOptionInt *canonical = config.opt<ConfigOptionInt>("canonical_int");
    REQUIRE(canonical != nullptr);
    CHECK(canonical->value == 42);
    CHECK(config.opt<ConfigOptionInt>("legacy_int_alias") == nullptr);
    CHECK_THROWS_AS(config.set_deserialize("unknown_config_contract_key", "1", substitutions), UnknownOptionException);
    CHECK(canonical->value == 42);
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

// A value of the option's type that differs from its default, used to prove that
// deserialization really replaces the stored value. For enums, the decoded values
// it must produce.
struct Representative {
    std::string      text;
    std::vector<int> enum_values;
};

Representative representative_value(const ConfigOptionDef &def, const ConfigOption &default_option)
{
    switch (def.type) {
    case coFloat: return {"12.5", {}};
    case coFloats: return {"12.5,7.25", {}};
    case coInt: return {"7", {}};
    case coInts: return {"7,3", {}};
    case coPercent: return {"37%", {}};
    case coPercents: return {"37%,12%", {}};
    case coFloatOrPercent: return {"37%", {}};
    case coFloatsOrPercents: return {"37%,1.5", {}};
    case coString: return {"schema-roundtrip", {}};
    case coStrings: return {"schema-roundtrip;second-value", {}};
    case coPoint: return {"1,2", {}};
    case coPoints: return {"1x2,3x4", {}};
    case coPoint3: return {"1,2,3", {}};
    case coPointsGroups: return {"1x2,3x4#5x6", {}};
    case coIntsGroups: return {"1,2#3", {}};
    case coBool: return {default_option.getBool() ? "0" : "1", {}};
    case coBools: return {default_option.serialize() == "1,0" ? "0,1" : "1,0", {}};
    case coEnum:
    case coEnums: {
        // Keys whose values differ from each other and from the first default value.
        const auto *default_list = dynamic_cast<const ConfigOptionInts *>(&default_option);
        const int   default_value = default_list == nullptr ? default_option.getInt() :
                                    default_list->values.empty() ? -1 : default_list->values.front();
        Representative out;
        for (const auto &[key, value] : *def.enum_keys_map) {
            if (value == default_value || (!out.enum_values.empty() && value == out.enum_values.front()))
                continue;
            if (!out.text.empty())
                out.text += ',';
            out.text += key;
            out.enum_values.push_back(value);
            if (def.type == coEnum || out.enum_values.size() == 2)
                break;
        }
        return out;
    }
    default: return {};
    }
}

template<class T> const T &option_as(const ConfigOption &option)
{
    const T *typed = dynamic_cast<const T *>(&option);
    REQUIRE(typed != nullptr);
    return *typed;
}

// Checks every decoded field of an option deserialized from representative_value().
void check_representative_values(const ConfigOptionDef &def, const ConfigOption &option, const Representative &representative)
{
    using Catch::Matchers::WithinAbs;
    switch (def.type) {
    case coFloat: CHECK_THAT(option_as<ConfigOptionFloat>(option).value, WithinAbs(12.5, 1e-12)); break;
    case coPercent: CHECK_THAT(option_as<ConfigOptionPercent>(option).value, WithinAbs(37., 1e-12)); break;
    case coFloats:
    case coPercents: {
        const auto &values = option_as<ConfigOptionVector<double>>(option).values;
        REQUIRE(values.size() == 2);
        CHECK_THAT(values[0], WithinAbs(def.type == coFloats ? 12.5 : 37., 1e-12));
        CHECK_THAT(values[1], WithinAbs(def.type == coFloats ? 7.25 : 12., 1e-12));
        break;
    }
    case coFloatOrPercent: {
        const auto &value = option_as<ConfigOptionFloatOrPercent>(option);
        CHECK_THAT(value.value, WithinAbs(37., 1e-12));
        CHECK(value.percent);
        break;
    }
    case coFloatsOrPercents: {
        const auto &values = option_as<ConfigOptionVector<FloatOrPercent>>(option).values;
        REQUIRE(values.size() == 2);
        CHECK_THAT(values[0].value, WithinAbs(37., 1e-12));
        CHECK(values[0].percent);
        CHECK_THAT(values[1].value, WithinAbs(1.5, 1e-12));
        CHECK_FALSE(values[1].percent);
        break;
    }
    case coInt: CHECK(option_as<ConfigOptionInt>(option).value == 7); break;
    case coInts: CHECK(option_as<ConfigOptionVector<int>>(option).values == std::vector<int>{7, 3}); break;
    case coString: CHECK(option_as<ConfigOptionString>(option).value == "schema-roundtrip"); break;
    case coStrings:
        CHECK(option_as<ConfigOptionStrings>(option).values == std::vector<std::string>{"schema-roundtrip", "second-value"});
        break;
    case coPoint: CHECK(option_as<ConfigOptionPoint>(option).value == Vec2d(1., 2.)); break;
    case coPoints: CHECK(option_as<ConfigOptionPoints>(option).values == std::vector<Vec2d>{Vec2d(1., 2.), Vec2d(3., 4.)}); break;
    case coPoint3: CHECK(option_as<ConfigOptionPoint3>(option).value == Vec3d(1., 2., 3.)); break;
    case coPointsGroups: {
        const auto &groups = option_as<ConfigOptionPointsGroups>(option).values;
        REQUIRE(groups.size() == 2);
        CHECK(groups[0] == Vec2ds{Vec2d(1., 2.), Vec2d(3., 4.)});
        CHECK(groups[1] == Vec2ds{Vec2d(5., 6.)});
        break;
    }
    case coIntsGroups:
        CHECK(option_as<ConfigOptionIntsGroups>(option).values == std::vector<std::vector<int>>{{1, 2}, {3}});
        break;
    case coBool: CHECK(option_as<ConfigOptionBool>(option).value == (representative.text == "1")); break;
    case coBools:
        CHECK(option_as<ConfigOptionVector<unsigned char>>(option).values ==
              (representative.text == "1,0" ? std::vector<unsigned char>{1, 0} : std::vector<unsigned char>{0, 1}));
        break;
    case coEnum: CHECK(option.getInt() == representative.enum_values.front()); break;
    case coEnums: CHECK(option_as<ConfigOptionInts>(option).values == representative.enum_values); break;
    default: FAIL("unhandled option type " << int(def.type));
    }
}

// The decoded value and percent flag of a numeric option's first element.
std::pair<double, bool> first_numeric_value(const ConfigOption &option)
{
    switch (option.type()) {
    case coFloat:
    case coPercent: return {static_cast<const ConfigOptionFloat &>(option).value, option.type() == coPercent};
    case coFloatOrPercent: {
        const auto &value = static_cast<const ConfigOptionFloatOrPercent &>(option);
        return {value.value, value.percent};
    }
    case coFloats:
    case coPercents: return {static_cast<const ConfigOptionVector<double> &>(option).values.at(0), option.type() == coPercents};
    case coFloatsOrPercents: {
        const FloatOrPercent &value = static_cast<const ConfigOptionVector<FloatOrPercent> &>(option).values.at(0);
        return {value.value, value.percent};
    }
    case coInt: return {double(static_cast<const ConfigOptionInt &>(option).value), false};
    case coInts: return {double(static_cast<const ConfigOptionVector<int> &>(option).values.at(0)), false};
    default: return {std::numeric_limits<double>::quiet_NaN(), false};
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
                }
            }
            // enum_values is the UI-facing menu. It may intentionally omit
            // canonical aliases or firmware tokens that remain valid through
            // enum_keys_map; the round-trip test below exercises every map key.
        }
    }

    CHECK(actual_defaultless == expected_defaultless);
    CHECK(print_config_def.options.size() >= 750); // 861 today
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

        // Start from a different value, so a deserializer that accepts the text without
        // storing it cannot pass.
        const std::unique_ptr<ConfigOption> restored(def.create_default_option());
        REQUIRE(restored != nullptr);
        REQUIRE(restored->deserialize(representative_value(def, *source).text));
        REQUIRE(restored->serialize() != serialized);
        REQUIRE(restored->deserialize(serialized));
        // A few legacy vector defaults originate from float literals (for example 0.01f)
        // but are stored as doubles. Their decimal wire form is intentionally shorter than
        // the promoted binary value, so object equality is too strict here; every field
        // is compared through its canonical serialization instead.
        CHECK(restored->serialize() == serialized);
        ++roundtripped;
    }

    CHECK(empty_aggregate_defaults == expected_empty_aggregate_defaults);
    CHECK(roundtripped + empty_aggregate_defaults.size() == print_config_def.options.size());
    CHECK(roundtripped >= 750); // 860 registered today; a floor so the sweep cannot silently shrink
}

TEST_CASE("Registered options decode a non-default value of their type", "[Config][Schema][RoundTrip]")
{
    std::map<ConfigOptionType, size_t> decoded_by_type;

    for (const auto &[key, def] : print_config_def.options) {
        INFO(key);
        const std::unique_ptr<ConfigOption> source(def.create_default_option());
        REQUIRE(source != nullptr);
        const Representative representative = representative_value(def, *source);
        REQUIRE_FALSE(representative.text.empty());
        REQUIRE(representative.text != source->serialize());

        const std::unique_ptr<ConfigOption> restored(def.create_default_option());
        REQUIRE(restored != nullptr);
        REQUIRE(restored->deserialize(representative.text));
        check_representative_values(def, *restored, representative);
        CHECK(restored->serialize() == representative.text);
        ++decoded_by_type[def.type];
    }

    size_t decoded = 0;
    for (const auto &[type, count] : decoded_by_type)
        decoded += count;
    CHECK(decoded == print_config_def.options.size());
    CHECK(decoded_by_type.size() >= 15); // 17 option types are registered today
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
    CHECK(enum_options >= 60); // 72 today
}

TEST_CASE("All registered boolean settings accept 0 and 1 and reject other values", "[Config][Schema][Bool]")
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
        CHECK_FALSE(option->deserialize("true"));

        if (def.type == coBool) {
            // A scalar boolean takes the first element of comma-separated input.
            REQUIRE(option->deserialize("1,0"));
            CHECK(option->serialize() == "1");
            REQUIRE(option->deserialize("0,1"));
            CHECK(option->serialize() == "0");
        }

        if (def.type == coBools) {
            REQUIRE(option->deserialize("0,1"));
            CHECK(option->serialize() == "0,1");
            if (def.nullable) {
                REQUIRE(option->deserialize("nil"));
                CHECK(option->serialize() == "nil");
            }
        }
    }
    CHECK(bool_options >= 140); // 164 today
}

TEST_CASE("Numeric option definitions enforce their declared finite boundaries", "[Config][Schema][Numeric]")
{
    const double float_limit = static_cast<double>(std::numeric_limits<float>::max()) / 2.;
    size_t       bounded_options = 0;

    for (const auto &[key, def] : print_config_def.options) {
        if (!is_numeric_option_type(def.type))
            continue;

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

    CHECK(bounded_options >= 380); // 447 today
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
            const double expected = is_integer_option_type(def.type) ? double(std::llround(boundary)) : boundary;
            const auto [decoded, percent] = first_numeric_value(*source);
            CHECK_THAT(decoded, Catch::Matchers::WithinAbs(expected, 1e-9 * std::max(1., std::abs(expected))));
            CHECK(percent == is_percent_option_type(def.type));

            const std::string canonical = source->serialize();
            const std::unique_ptr<ConfigOption> restored(def.create_default_option());
            REQUIRE(restored != nullptr);
            REQUIRE(restored->deserialize(canonical));
            CHECK(restored->serialize() == canonical);
            // Round trip is decode -> serialize -> decode: the restored option must hold the same boundary.
            // The canonical text drops digits beyond float precision (def.min/max are floats, so
            // e.g. 0.1f widens to 0.10000000149), hence the looser relative tolerance.
            const auto [restored_value, restored_percent] = first_numeric_value(*restored);
            CHECK_THAT(restored_value, Catch::Matchers::WithinAbs(expected, 1e-6 * std::max(1., std::abs(expected))));
            CHECK(restored_percent == percent);
            ++boundary_values;
        }
    }

    CHECK(boundary_values >= 550); // 654 today
}

// Known bug: coEnums defaults such as z_hop_types are built without a keys map, and static configs copy that
// null map, so their enum lists serialize as "" (a deserialize would dereference it); fixed by the follow-up fix PR.
TEST_CASE("Static and dynamic print defaults retain named enum lists", "[Config][StaticEnumDefaults][!shouldfail]")
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

TEST_CASE("Core FFF numeric validation rejects values below their semantic floors", "[Config][Validation]")
{
    struct Boundary {
        const char               *key;
        std::vector<const char *> invalid;
        std::vector<const char *> valid;
    };
    // validate() enforces these floors on top of the option metadata, which allows
    // the invalid values below (checked first, so the generic range check cannot be
    // what reports them). wall_loops and the shell layer counts are not listed: their
    // semantic floor is the same as their metadata minimum. Valid values sit at or just
    // above the floor and differ from the defaults, so they are really applied.
    // layer_height's "not a multiple of SCALING_FACTOR" branch is not covered: the
    // remainder of fmod() by SCALING_FACTOR is always below its 1e-4 threshold.
    const Boundary boundaries[] = {
        {"layer_height", {"0"}, {"0.05"}},
        {"initial_layer_print_height", {"0"}, {"0.05"}},
        {"filament_diameter", {"0.9", "0.999"}, {"1.0"}},
        {"nozzle_diameter", {"0.004", "0.0049"}, {"0.005"}},
        {"skirt_height", {"-1"}, {"0"}},
        {"bridge_flow", {"0"}, {"0.01"}},
        {"internal_bridge_flow", {"0"}, {"0.01"}},
        {"extruder_clearance_radius", {"0"}, {"0.1"}},
        {"extruder_clearance_height_to_rod", {"0"}, {"0.1"}},
        {"extruder_clearance_height_to_lid", {"0"}, {"0.1"}},
        {"nozzle_height", {"0"}, {"0.1"}},
        {"filament_flow_ratio", {"0"}, {"0.01"}},
    };

    for (const Boundary &boundary : boundaries) {
        DYNAMIC_SECTION(boundary.key) {
            const ConfigOptionDef *def = print_config_def.get(boundary.key);
            REQUIRE(def != nullptr);
            for (const char *value : boundary.invalid) {
                INFO("invalid " << value);
                REQUIRE(def->is_value_valid(std::stod(value)));
                DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
                config.set_deserialize_strict(boundary.key, value);
                const auto errors = config.validate();
                CHECK(errors.find(boundary.key) != errors.end());
            }
            for (const char *value : boundary.valid) {
                INFO("valid " << value);
                DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
                REQUIRE(config.opt_serialize(boundary.key) != value);
                config.set_deserialize_strict(boundary.key, value);
                const auto errors = config.validate();
                CHECK(errors.find(boundary.key) == errors.end());
            }
        }
    }
}

TEST_CASE("normalize_fdm applies spiral mode safety values and is idempotent", "[Config][NormalizeFdm]")
{
    const bool split_step = GENERATE(false, true);
    CAPTURE(split_step);
    const auto normalize = [split_step](DynamicPrintConfig &config) {
        if (split_step)
            config.normalize_fdm_1();
        else
            config.normalize_fdm(0);
    };
    const auto make_config = [](bool spiral) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set("spiral_mode", spiral);
        config.set("alternate_extra_wall", true);
        config.set("wall_loops", 4);
        config.set("top_shell_layers", 3);
        config.set("bottom_shell_layers", 5);
        config.set_deserialize_strict("sparse_infill_density", "20");
        config.set_deserialize_strict("retract_when_changing_layer", "1,1");
        config.option<ConfigOptionBoolsNullable>("filament_retract_when_changing_layer", true)->values = {true, true};
        return config;
    };
    const auto check_unchanged = [](const DynamicPrintConfig &config) {
        CHECK(config.opt_bool("alternate_extra_wall"));
        CHECK(config.opt_int("wall_loops") == 4);
        CHECK(config.opt_int("top_shell_layers") == 3);
        CHECK_THAT(config.opt<ConfigOptionPercent>("sparse_infill_density")->value, Catch::Matchers::WithinAbs(20., 1e-12));
        CHECK(config.opt<ConfigOptionBools>("retract_when_changing_layer")->values == std::vector<unsigned char>{1, 1});
        CHECK(config.opt<ConfigOptionBoolsNullable>("filament_retract_when_changing_layer")->values == std::vector<unsigned char>{1, 1});
    };

    SECTION("spiral mode forces a single hollow wall without retraction on layer change")
    {
        DynamicPrintConfig config = make_config(true);
        for (int pass = 0; pass < 2; ++pass) {
            CAPTURE(pass); // the second pass proves idempotence
            normalize(config);
            CHECK(config.opt_int("wall_loops") == 1);
            CHECK_FALSE(config.opt_bool("alternate_extra_wall"));
            CHECK(config.opt_int("top_shell_layers") == 0);
            CHECK_THAT(config.opt<ConfigOptionPercent>("sparse_infill_density")->value, Catch::Matchers::WithinAbs(0., 1e-12));
            CHECK(config.opt<ConfigOptionBools>("retract_when_changing_layer")->values == std::vector<unsigned char>{0, 0});
            CHECK(config.opt<ConfigOptionBoolsNullable>("filament_retract_when_changing_layer")->values == std::vector<unsigned char>{0, 0});
            // Unrelated shell settings are untouched.
            CHECK(config.opt_int("bottom_shell_layers") == 5);
        }
    }

    SECTION("the same values are left alone when spiral mode is off")
    {
        DynamicPrintConfig config = make_config(false);
        normalize(config);
        check_unchanged(config);
        CHECK(config.opt_int("bottom_shell_layers") == 5);
    }
}

TEST_CASE("normalize_fdm raises resolution to the 1 micron floor and keeps larger values", "[Config][NormalizeFdm]")
{
    const bool split_step = GENERATE(false, true);
    CAPTURE(split_step);
    const std::pair<const char *, double> rows[] = {
        {"0.0001", 0.001},
        {"0.0009", 0.001},
        {"0.001",  0.001},
        {"0.0011", 0.0011},
        {"0.5",    0.5},
    };
    for (const auto &[input, expected] : rows) {
        DYNAMIC_SECTION("resolution " << input) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("resolution", input);
            REQUIRE(config.opt_serialize("resolution") == input);
            if (split_step)
                config.normalize_fdm_1();
            else
                config.normalize_fdm(0);
            CHECK_THAT(config.opt_float("resolution"), Catch::Matchers::WithinAbs(expected, 1e-12));
        }
    }
}

TEST_CASE("Project JSON migration records support and wall-order differences", "[Config][ConfigJsonMigration]")
{
    const auto [seed_existing, keep_wall_order] = GENERATE(
        table<bool, bool>({
            {false, false},
            {false, true},
            {true, false},
            {true, true},
        }));
    CAPTURE(seed_existing, keep_wall_order);

    nlohmann::json document;
    document["name"] = "project_settings";
    document["support_type"] = "hybrid(auto)";
    document["filament_settings_id"] = {"filament-a", "filament-b"};
    if (seed_existing) {
        document["different_settings_to_system"] = {"wall_sequence;outer_wall_filament_id"};
    } else {
        document["different_settings_to_system"] = nlohmann::json::array();
    }
    if (keep_wall_order)
        document["wall_infill_order"] = "infill/outer wall/inner wall";

    ScopedTemporaryFile tmp(".json");
    {
        boost::nowide::ofstream out(tmp.string());
        REQUIRE(out.is_open());
        out << document.dump();
        out.close();
        REQUIRE(out.good());
    }

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("is_infill_first", "0");
    ConfigSubstitutionContext substitutions(ForwardCompatibilitySubstitutionRule::Disable);
    std::map<std::string, std::string> key_values;
    std::string reason;
    REQUIRE(config.load_from_json(tmp.string(), substitutions, true, key_values, reason) == 0);
    CHECK(reason.empty());
    CHECK(key_values["name"] == "project_settings");
    CHECK(substitutions.unrecogized_keys.empty());
    CHECK(config.opt_serialize("support_type") == "tree(auto)");
    CHECK(config.opt_serialize("support_style") == "tree_hybrid");
    CHECK(config.opt_bool("is_infill_first") == keep_wall_order);

    const ConfigOptionStrings *different = config.option<ConfigOptionStrings>("different_settings_to_system");
    REQUIRE(different != nullptr);

    std::vector<std::string> expected;
    if (seed_existing) {
        expected.push_back(keep_wall_order
            ? "wall_sequence;outer_wall_filament_id;support_type;support_style;is_infill_first"
            : "outer_wall_filament_id;support_type;support_style");
    } else {
        expected = {
            keep_wall_order ? "support_type;support_style;is_infill_first" : "support_type;support_style",
            "",
            "",
            "",
        };
    }
    REQUIRE(different->values.size() == expected.size());
    for (size_t index = 0; index < expected.size(); ++index)
        CHECK(different->values[index] == expected[index]);
}

namespace {

// The migrated key and value must be accepted by the current option definition.
void check_migrated_value_deserializes(const std::string &key, const std::string &value)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    CHECK_NOTHROW(config.set_deserialize_strict(key, value));
    CHECK(config.opt_serialize(key) == value);
}

} // namespace

TEST_CASE("normalize_fdm_2 disables the prime tower or independent support layers and reports what it changed", "[Config][NormalizeFdm]")
{
    struct Row {
        const char *name;
        int         num_objects;
        int         used_filaments;
        const char *print_sequence;
        const char *timelapse_type; // "1" is smooth
        bool        wrapping_detection;
        bool        mixed_filament;
        bool        start_tower;
        bool        tower;
        bool        independent_layers;
        t_config_option_keys changed;
    };
    const t_config_option_keys tower_changed = {"enable_prime_tower"};
    const t_config_option_keys layers_changed = {"independent_support_layer_height"};
    const Row rows[] = {
        {"single filament drops the tower", 1, 1, "by layer", "0", false, false, true, false, true, tower_changed},
        {"several filaments keep the tower and drop independent layers", 1, 2, "by layer", "0", false, false, true, true, false, layers_changed},
        {"a mixed filament counts as several", 1, 1, "by layer", "0", false, true, true, true, false, layers_changed},
        {"wrapping detection keeps the tower", 1, 1, "by layer", "0", true, false, true, true, false, layers_changed},
        {"smooth timelapse keeps the tower", 1, 1, "by layer", "1", false, false, true, true, false, layers_changed},
        {"several by-object objects drop the tower", 2, 2, "by object", "0", false, false, true, false, true, tower_changed},
        {"a single by-object object keeps the tower", 1, 2, "by object", "0", false, false, true, true, false, layers_changed},
        {"no tower leaves independent layers alone", 1, 2, "by layer", "0", false, false, false, false, true, {}},
        {"no used filaments changes nothing", 1, 0, "by layer", "0", false, false, true, true, true, {}},
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.name) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set("enable_prime_tower", row.start_tower);
            config.set("independent_support_layer_height", true);
            config.set("enable_wrapping_detection", row.wrapping_detection);
            config.set_deserialize_strict("timelapse_type", row.timelapse_type);
            config.set_deserialize_strict("print_sequence", row.print_sequence);
            config.set_key_value("filament_is_mixed", new ConfigOptionBools({row.mixed_filament}));

            CHECK(config.normalize_fdm_2(row.num_objects, row.used_filaments) == row.changed);
            CHECK(config.opt_bool("enable_prime_tower") == row.tower);
            CHECK(config.opt_bool("independent_support_layer_height") == row.independent_layers);
            // Normalizing again has nothing left to change.
            CHECK(config.normalize_fdm_2(row.num_objects, row.used_filaments).empty());
        }
    }
}

TEST_CASE("normalize_fdm applies the used-filament prime tower policy", "[Config][NormalizeFdm]")
{
    struct Row {
        int         used_filaments;
        const char *print_sequence;
        const char *timelapse_type;
        bool        tower;
        bool        independent_layers;
    };
    const Row rows[] = {
        {1, "by layer",  "0", false, true},
        {2, "by layer",  "0", true,  false},
        {2, "by object", "0", false, true},
        {1, "by layer",  "1", true,  false},
        {0, "by layer",  "0", true,  true}, // no used filaments: the policy is skipped
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.used_filaments << " filaments, " << row.print_sequence << ", timelapse " << row.timelapse_type) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set("enable_prime_tower", true);
            config.set("independent_support_layer_height", true);
            config.set_deserialize_strict("print_sequence", row.print_sequence);
            config.set_deserialize_strict("timelapse_type", row.timelapse_type);

            config.normalize_fdm(row.used_filaments);

            CHECK(config.opt_bool("enable_prime_tower") == row.tower);
            CHECK(config.opt_bool("independent_support_layer_height") == row.independent_layers);
        }
    }
}

TEST_CASE("PrintConfig legacy aliases preserve their exact key and value contract", "[Config][ConfigLegacy]")
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
        {"counterbole_hole_bridging", "partiallybridge", "counterbore_hole_bridging", "partiallybridge"},
    };

    for (const Migration &migration : migrations) {
        DYNAMIC_SECTION(migration.old_key << " -> " << migration.new_key) {
            t_config_option_key key = migration.old_key;
            std::string value       = migration.old_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key == migration.new_key);
            CHECK(value == migration.new_value);
            check_migrated_value_deserializes(key, value);
        }
    }
}

TEST_CASE("PrintConfig legacy enum spellings become current canonical values", "[Config][ConfigLegacy]")
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
            // Known bug (see "Static and dynamic print defaults retain named enum lists"): coEnums defaults such as
            // extruder_type have no keys map, so deserializing into full_print_config() dereferences null.
            if (key != "extruder_type")
                check_migrated_value_deserializes(key, value);
        }
    }

    SECTION("rotate_solid_infill_direction = 1 becomes a rotation template") {
        t_config_option_key key = "rotate_solid_infill_direction";
        std::string value       = "1";
        PrintConfigDef::handle_legacy(key, value);
        CHECK(key == "solid_infill_rotate_template");
        CHECK(value == "0,90");
        check_migrated_value_deserializes(key, value);
    }
}

TEST_CASE("PrintConfig legacy values that cannot be represented are discarded", "[Config][ConfigLegacy]")
{
    // Percent speeds and distances are dropped by their own guard. "scale" and "bed_size" are on the
    // obsolete list and the future key is unregistered; for those the final unknown-key check would
    // also clear them, so these rows only pin the outcome.
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

TEST_CASE("PrintConfig legacy migrations normalize bed and wall-order spellings", "[Config][ConfigLegacy]")
{
    struct Row {
        const char *input_key;
        const char *input_value;
        const char *expected_key;
        const char *expected_value;
    };
    const Row rows[] = {
        {"curr_bed_type", "SuperTack Plate", "curr_bed_type", "Supertack Plate"},
        {"wall_infill_order", "inner wall/outer wall/infill", "wall_sequence", "inner wall/outer wall"},
        {"wall_infill_order", "infill/inner wall/outer wall", "wall_sequence", "inner wall/outer wall"},
        {"wall_infill_order", "outer wall/inner wall/infill", "wall_sequence", "outer wall/inner wall"},
        {"wall_infill_order", "infill/outer wall/inner wall", "wall_sequence", "outer wall/inner wall"},
        {"wall_infill_order", "inner-outer-inner wall/infill", "wall_sequence", "inner-outer-inner wall"},
        {"wall_infill_order", "legacy spelling", "wall_sequence", "legacy spelling"},
    };

    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.input_key << " = " << row.input_value) {
            t_config_option_key key = row.input_key;
            std::string value = row.input_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key == row.expected_key);
            CHECK(value == row.expected_value);
            // An unknown wall_infill_order spelling is passed through for wall_sequence to reject.
            if (std::string(row.input_value) != "legacy spelling")
                check_migrated_value_deserializes(key, value);
        }
    }
}

TEST_CASE("PrintConfig legacy handling leaves current keys and values unchanged", "[Config][ConfigLegacy]")
{
    // Current spellings next to the legacy ones converted above.
    const std::pair<const char *, const char *> current[] = {
        {"layer_height", "0.2"},
        {"support_type", "tree(auto)"},
        {"timelapse_type", "0"},
        {"overhang_fan_threshold", "25%"},
        {"sparse_infill_pattern", "gyroid"},
        {"curr_bed_type", "Supertack Plate"},
        {"wall_sequence", "inner wall/outer wall"},
        {"wall_direction", "cw"},
        {"filament_map_mode", "Manual"},
    };
    for (const auto &[current_key, current_value] : current) {
        DYNAMIC_SECTION(current_key << " = " << current_value) {
            t_config_option_key key = current_key;
            std::string value       = current_value;
            PrintConfigDef::handle_legacy(key, value);
            CHECK(key == current_key);
            CHECK(value == current_value);
        }
    }
}

namespace {

bool has_error(const std::map<std::string, std::string> &errors, const char *key)
{
    return errors.find(key) != errors.end();
}

// The reported message for `key`, or an empty string when validation did not flag it.
std::string error_message(const std::map<std::string, std::string> &errors, const char *key)
{
    const auto it = errors.find(key);
    return it == errors.end() ? std::string() : it->second;
}

} // namespace

namespace config_definition_contracts {

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
        {"print_preset", coString}, {"filament_preset", coStrings},
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
    records.insert(insertion, "G1 X99\n; ignored text\n;future_option = ignored\n;\t Alpha_key \t= 61 \t\n;text_value =  padded text \t\n");
    write_file(file, "; OrcaSlicer test\n" + records);
    FooterConfig config;
    ConfigSubstitutions substitutions;
    REQUIRE_NOTHROW(substitutions = config.load_from_gcode_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(substitutions.empty());
    REQUIRE(config.opt<ConfigOptionInt>("Alpha_key") != nullptr);
    CHECK(config.opt<ConfigOptionInt>("Alpha_key")->value == 61);
    REQUIRE(config.opt<ConfigOptionString>("text_value") != nullptr);
    CHECK(config.opt<ConfigOptionString>("text_value")->value == "padded text");
    CHECK(config.keys().size() == 82);
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

namespace config_option_value_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {

void check_point_components(const Vec2d &actual, double expected_x, double expected_y)
{
    CHECK_THAT(actual.x(), WithinAbs(expected_x, 1e-9));
    CHECK_THAT(actual.y(), WithinAbs(expected_y, 1e-9));
}

void check_values(const std::vector<double> &actual, const std::vector<double> &expected)
{
    REQUIRE(actual.size() == expected.size());
    for (size_t i = 0; i < expected.size(); ++i)
        CHECK_THAT(actual[i], WithinAbs(expected[i], 1e-9));
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
    ConfigOptionPoint3 x_decoded(Vec3d(7., 8., 9.));
    REQUIRE(x_decoded.deserialize("1.25x-2.5x9.75"));
    CHECK_THAT(x_decoded.value.x(), WithinAbs(1.25, 1e-9));
    CHECK_THAT(x_decoded.value.y(), WithinAbs(-2.5, 1e-9));
    CHECK_THAT(x_decoded.value.z(), WithinAbs(9.75, 1e-9));
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

TEST_CASE("ConfigOptionIntsGroups deserialize appends or replaces groups and serializes per group", "[Config][ConfigContracts]")
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
    REQUIRE(source.values[0].size() == 1);
    check_point_components(source.values[0][0], 1., 1.);
    REQUIRE(source.values[1].size() == 2);
    check_point_components(source.values[1][0], 2., 2.);
    check_point_components(source.values[1][1], 3., 3.);
    std::unique_ptr<ConfigOption> clone(source.clone());
    auto *typed = dynamic_cast<ConfigOptionPointsGroups *>(clone.get());
    REQUIRE(typed != nullptr);
    CHECK(typed != &source);
    REQUIRE(typed->values.size() == 2);
    REQUIRE(typed->values[0].size() == 1);
    check_point_components(typed->values[0][0], 1., 1.);
    REQUIRE(typed->values[1].size() == 2);
    check_point_components(typed->values[1][0], 2., 2.);
    check_point_components(typed->values[1][1], 3., 3.);
}

TEST_CASE("ConfigOptionPoint preserves scalar geometry through parsing and ownership", "[Config][ConfigContracts][ConfigPoint]")
{
    ConfigOptionPoint source(Vec2d(1.25, -2.5));
    CHECK(source.serialize() == "1.25,-2.5");

    ConfigOptionPoint x_decoded(Vec2d(7., 8.));
    REQUIRE(x_decoded.deserialize("1.25x-2.5"));
    check_point_components(x_decoded.value, 1.25, -2.5);
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
    // Ordering is element-wise before length: the first differing element decides.
    const ConfigOptionPoints differs({Vec2d(1., 2.), Vec2d(-3., 5.)});
    CHECK(source < differs);
    CHECK_FALSE(differs < source);
    const ConfigOptionPoints shorter_but_greater({Vec2d(2., 0.)});
    CHECK(source < shorter_but_greater);
    CHECK_FALSE(shorter_but_greater < source);

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

TEST_CASE("Grouped point options keep a trailing empty group when parsing", "[Config][ConfigContracts][ConfigGrouped][!shouldfail]")
{
    const ConfigOptionPointsGroups source({Vec2ds{Vec2d(1., 2.)}, Vec2ds{}});
    CHECK(source.serialize() == "1x2#");

    ConfigOptionPointsGroups decoded;
    REQUIRE(decoded.deserialize(source.serialize()));
    // Known bug: a trailing empty group is dropped by deserialize (std::getline ends before it); fixed by the follow-up fix PR
    REQUIRE(decoded.values.size() == 2);
    CHECK(decoded.values[1].empty());
}

TEST_CASE("Declared point and grouped integer option types round-trip through binary archives", "[Config][ConfigContracts][ConfigArchive]")
{
    // These are valid declared ConfigOptionDef types, not registered print settings.
    ConfigOptionDef point3_definition;
    point3_definition.opt_key = "archive_point3_contract";
    point3_definition.type = coPoint3;
    ConfigOptionDef groups_definition;
    groups_definition.opt_key = "archive_int_groups_contract";
    groups_definition.type = coIntsGroups;
    CHECK(point3_definition.is_scalar());
    CHECK_FALSE(groups_definition.is_scalar());

    const ConfigOptionPoint3 point3_source(Vec3d(1.25, -2.5, 9.75));
    const ConfigOptionIntsGroups groups_source({std::vector<int>{1, 4}, std::vector<int>{9}});

    std::stringstream bytes;
    {
        cereal::BinaryOutputArchive archive(bytes);
        point3_definition.save_option_to_archive(archive, &point3_source);
        groups_definition.save_option_to_archive(archive, &groups_source);
    }

    bytes.seekg(0);
    std::unique_ptr<ConfigOption> point3_restored;
    std::unique_ptr<ConfigOption> groups_restored;
    {
        cereal::BinaryInputArchive archive(bytes);
        point3_restored.reset(point3_definition.load_option_from_archive(archive));
        groups_restored.reset(groups_definition.load_option_from_archive(archive));
    }

    REQUIRE(point3_restored != nullptr);
    CHECK(point3_restored->type() == coPoint3);
    CHECK(point3_restored->is_scalar());
    CHECK_FALSE(point3_restored->nullable());
    CHECK(*point3_restored == point3_source);
    REQUIRE(groups_restored != nullptr);
    CHECK(groups_restored->type() == coIntsGroups);
    CHECK_FALSE(groups_restored->is_scalar());
    CHECK_FALSE(groups_restored->nullable());
    const auto *restored_groups = dynamic_cast<const ConfigOptionIntsGroups *>(groups_restored.get());
    REQUIRE(restored_groups != nullptr);
    CHECK_FALSE(restored_groups->is_nil(0));
    CHECK(*groups_restored == groups_source);
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

TEST_CASE("String settings preserve escaped scalar and delimited vector content",
          "[Config][ConfigSerialization]")
{
    const std::string scalar_value = "quote\" slash\\ carriage\r newline\n";
    ConfigOptionString scalar(scalar_value);
    const std::string scalar_serialized = scalar.serialize();
    ConfigOptionString scalar_decoded;
    REQUIRE(scalar_decoded.deserialize(scalar_serialized));
    CHECK(scalar_decoded.value == scalar_value);

    const std::vector<std::string> source_values{
        "plain", "", "semi;colon", "quote\" and slash\\", "tab\t line\n carriage\r"
    };
    ConfigOptionStrings source(source_values);
    const std::string serialized = source.serialize();
    ConfigOptionStrings decoded;
    REQUIRE(decoded.deserialize(serialized));
    CHECK(decoded.values == source_values);

    std::vector<std::string> trailing_empty;
    REQUIRE(unescape_strings_cstyle("last;", trailing_empty));
    CHECK(trailing_empty == std::vector<std::string>{"last", ""});

    std::vector<std::string> whitespace_only;
    REQUIRE(unescape_strings_cstyle(" \t", whitespace_only));
    CHECK(whitespace_only.empty());
    std::vector<std::string> quoted_then_spaces;
    REQUIRE(unescape_strings_cstyle("\"value\" \t", quoted_then_spaces));
    CHECK(quoted_then_spaces == std::vector<std::string>{"value"});

    std::vector<std::string> unterminated_quote;
    CHECK_FALSE(unescape_strings_cstyle("\"unfinished", unterminated_quote));
    std::vector<std::string> trailing_escape;
    CHECK_FALSE(unescape_strings_cstyle("\"unfinished\\", trailing_escape));
    std::vector<std::string> malformed_quoted_tail;
    CHECK_FALSE(unescape_strings_cstyle("\"value\"unexpected", malformed_quoted_tail));
    CHECK_FALSE(scalar_decoded.deserialize("dangling\\"));
}

TEST_CASE("ConfigOption vector override detection respects nil and regular slots", "[Config][ConfigVectorOverride]")
{
    ConfigOptionInts base({10, 20});
    ConfigOptionInts equal({10, 20});
    ConfigOptionInts changed({10, 21});
    ConfigOptionIntsNullable all_nil({ConfigOptionIntsNullable::nil_value(), ConfigOptionIntsNullable::nil_value()});
    ConfigOptionIntsNullable partial({ConfigOptionIntsNullable::nil_value(), 21});
    ConfigOptionIntsNullable extended({ConfigOptionIntsNullable::nil_value(), ConfigOptionIntsNullable::nil_value(), 30});

    CHECK_FALSE(base.overriden_by(&equal));
    CHECK(base.overriden_by(&changed));
    CHECK_FALSE(base.overriden_by(&all_nil));
    CHECK(base.overriden_by(&partial));
    CHECK(base.overriden_by(&extended));
}

TEST_CASE("ConfigOption vector overrides copy regular sources and preserve nullable defaults", "[Config][ConfigVectorOverride]")
{
    ConfigOptionInts regular_target({5, 10});
    ConfigOptionInts regular_source({6, 12});
    std::vector<int> regular_indices{0, 1};
    REQUIRE(regular_target.apply_override(&regular_source, regular_indices));
    const std::vector<int> expected_regular{6, 12};
    CHECK(regular_target.values == expected_regular);
    CHECK_FALSE(regular_target.apply_override(&regular_source, regular_indices));

    ConfigOptionInts empty_target;
    ConfigOptionIntsNullable incoming({7, 8});
    std::vector<int> empty_target_indices{0, 1};
    CHECK_FALSE(empty_target.apply_override(&incoming, empty_target_indices));
    const std::vector<int> expected_empty;
    CHECK(empty_target.values == expected_empty);

    ConfigOptionInts target({10, 20, 30});
    ConfigOptionIntsNullable replacement({11, ConfigOptionIntsNullable::nil_value(), 33});
    std::vector<int> default_index{0, 2, 1};
    REQUIRE(target.apply_override(&replacement, default_index));
    const std::vector<int> expected_nullable{11, 30, 33};
    CHECK(target.values == expected_nullable);
}

TEST_CASE("Config option vectors collect scalar values and first vector values", "[Config][ConfigContracts]")
{
    ConfigOptionFloat scalar(2.5);
    ConfigOptionFloats vector({3.5, 4.5});
    ConfigOptionFloats collected;
    const std::vector<const ConfigOption *> sources{&scalar, &vector};
    collected.set(sources);
    check_values(collected.values, {2.5, 3.5});

    ConfigOptionFloats slots({10., 20.});
    ConfigOptionFloat replacement(7.);
    slots.set_at(&replacement, 3, 0);
    check_values(slots.values, {10., 20., 10., 7.});
}

} // namespace config_option_value_contracts

TEST_CASE("Bridge line width is limited by the smallest nozzle while other widths use the largest", "[Config][Validation]")
{
    struct Row {
        const char *nozzles;
        const char *bridge_width;
        bool        rejected;
    };
    const Row rows[] = {
        {"0.4",     "0.41", true},
        {"0.4",     "0.4",  false},  // exactly the nozzle diameter
        {"0.4",     "0.39", false},
        {"0.4,0.6", "0.41", true},   // below the largest nozzle but above the smallest
        {"0.4,0.6", "0.4",  false},
        {"0.4",     "100%", false},
        {"0.4,0.6", "60%",  false},  // below 100% of every extruder's own nozzle
        {"0.4,0.6", "101%", true},   // above 100% of the 0.4 mm nozzle
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION("nozzles " << row.nozzles << ", bridge " << row.bridge_width) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("nozzle_diameter", row.nozzles);
            config.set_deserialize_strict("bridge_line_width", row.bridge_width);
            const auto errors = config.validate();
            CHECK(has_error(errors, "bridge_line_width") == row.rejected);
            if (row.rejected)
                CHECK_THAT(error_message(errors, "bridge_line_width"), Catch::Matchers::ContainsSubstring("must not exceed nozzle diameter"));
        }
    }

    // Ordinary line widths are capped at MAX_LINE_WIDTH_MULTIPLIER times the largest nozzle (3.0 mm here).
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict("nozzle_diameter", "0.4,0.6");
    config.set_deserialize_strict("outer_wall_line_width", "2.9");
    CHECK_FALSE(has_error(config.validate(), "outer_wall_line_width"));
    config.set_deserialize_strict("outer_wall_line_width", "3.1");
    CHECK_THAT(error_message(config.validate(), "outer_wall_line_width"), Catch::Matchers::ContainsSubstring("too large line width"));
}

// Printing resolves a percentage against the nozzle of the extruder that prints the bridge, so a percentage of at
// most 100% never exceeds that extruder's own nozzle.
TEST_CASE("Percent bridge line width is accepted when it fits each extruder's own nozzle", "[Config][Validation]")
{
    for (const char *width : {"70%", "100%"}) {
        DYNAMIC_SECTION("bridge " << width << " with 0.4 and 0.6 mm nozzles") {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("nozzle_diameter", "0.4,0.6");
            config.set_deserialize_strict("bridge_line_width", width);
            CHECK_FALSE(has_error(config.validate(), "bridge_line_width"));
        }
    }
}

TEST_CASE("Spiral mode validation reports each incompatible process setting only for the CLI", "[Config][Validation]")
{
    struct Rule {
        const char *key;
        const char *violating;
    };
    // Every other spiral-incompatible setting is held at its compliant value, so each
    // row violates exactly one rule. wall_loops != 1 is rejected on both sides of 1.
    const Rule rules[] = {
        {"wall_loops", "0"},
        {"wall_loops", "2"},
        {"sparse_infill_density", "0.01%"},  // any positive density
        {"sparse_infill_density", "15%"},
        {"top_shell_layers", "1"},
        {"enable_support", "1"},
        {"enforce_support_layers", "1"},
    };
    const char *const keys[] = {"wall_loops", "sparse_infill_density", "top_shell_layers", "enable_support", "enforce_support_layers"};

    for (const Rule &rule : rules) {
        DYNAMIC_SECTION(rule.key << " = " << rule.violating) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set("wall_loops", 1);
            config.set_deserialize_strict("sparse_infill_density", "0");
            config.set("top_shell_layers", 0);
            config.set("enable_support", false);
            config.set("enforce_support_layers", 0);
            config.set("spiral_mode", true);
            // The compliant baseline is accepted.
            for (const char *key : keys)
                CHECK_FALSE(has_error(config.validate(true), key));

            config.set_deserialize_strict(rule.key, rule.violating);
            REQUIRE(config.opt_serialize(rule.key) == rule.violating);
            const auto errors = config.validate(true);
            for (const char *key : keys)
                CHECK(has_error(errors, key) == (std::string(key) == rule.key));
            CHECK_THAT(error_message(errors, rule.key), Catch::Matchers::ContainsSubstring("spiral vase mode is enabled"));

            // Outside the CLI the settings are corrected by a dialog instead, so validate() stays silent.
            for (const char *key : keys)
                CHECK_FALSE(has_error(config.validate(false), key));

            // Without spiral mode the same values are ordinary settings.
            config.set("spiral_mode", false);
            for (const char *key : keys)
                CHECK_FALSE(has_error(config.validate(true), key));
        }
    }
}

TEST_CASE("Firmware retraction validation accepts only firmware flavors that implement it", "[Config][Validation]")
{
    // Every GCodeFlavor key. The unsupported ones are not among gcode_flavor's selectable enum
    // values, so validate() reports "gcode_flavor" for them too; only use_firmware_retraction is checked here.
    const std::pair<const char *, bool> flavors[] = {
        {"marlin", true},         {"marlin2", true},    {"klipper", true},   {"reprap", true},
        {"reprapfirmware", true}, {"repetier", true},   {"smoothie", true},  {"machinekit", true},
        {"teacup", false},        {"makerware", false}, {"sailfish", false}, {"mach3", false},
        {"no-extrusion", false},
    };
    for (const auto &[flavor, supported] : flavors) {
        DYNAMIC_SECTION(flavor) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("gcode_flavor", flavor);
            config.set_deserialize_strict("wipe", "0");

            config.set("use_firmware_retraction", true);
            const auto errors = config.validate();
            CHECK(has_error(errors, "use_firmware_retraction") == !supported);
            if (!supported)
                CHECK_THAT(error_message(errors, "use_firmware_retraction"), Catch::Matchers::ContainsSubstring("only supported by"));

            // The flavor restriction applies only while firmware retraction is requested.
            config.set("use_firmware_retraction", false);
            CHECK_FALSE(has_error(config.validate(), "use_firmware_retraction"));
        }
    }
}

TEST_CASE("Firmware retraction validation rejects wipe on any extruder", "[Config][Validation]")
{
    struct Row {
        const char *wipe;
        bool        firmware_retraction;
        bool        rejected;
    };
    const Row rows[] = {
        {"1",   true,  true},
        {"0,1", true,  true},   // only the second extruder wipes
        {"1,0", true,  true},
        {"0,0", true,  false},
        {"0",   true,  false},
        {"1,1", false, false},  // wipe alone is fine without firmware retraction
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION("wipe " << row.wipe << (row.firmware_retraction ? " with" : " without") << " firmware retraction") {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set_deserialize_strict("gcode_flavor", "klipper");
            config.set_deserialize_strict("wipe", row.wipe);
            config.set("use_firmware_retraction", row.firmware_retraction);
            const auto errors = config.validate();
            CHECK(has_error(errors, "use_firmware_retraction") == row.rejected);
            if (row.rejected)
                CHECK_THAT(error_message(errors, "use_firmware_retraction"), Catch::Matchers::ContainsSubstring("not compatible with --wipe"));
        }
    }

    // When both rules are violated the flavor message is the one reported (the wipe message
    // does not replace it).
    DynamicPrintConfig both = DynamicPrintConfig::full_print_config();
    both.set_deserialize_strict("gcode_flavor", "teacup");
    both.set_deserialize_strict("wipe", "1");
    both.set("use_firmware_retraction", true);
    CHECK_THAT(error_message(both.validate(), "use_firmware_retraction"), Catch::Matchers::ContainsSubstring("only supported by"));
}

TEST_CASE("normalize_fdm backfills unset filament roles from the legacy extruder and from related roles", "[Config][NormalizeFdm]")
{
    // normalize_fdm(0) and normalize_fdm_1() share the role logic; with 0 used filaments
    // normalize_fdm skips the prime tower step.
    const bool split_step = GENERATE(false, true);
    CAPTURE(split_step);

    constexpr int no_legacy_extruder = -1;
    // Role order: sparse infill, internal solid, top surface, bottom surface, outer wall, inner wall.
    struct Row {
        const char *name;
        int         legacy_extruder;
        int         roles[6];
        int         expected[6];
    };
    const Row rows[] = {
        {"legacy extruder fills every unset role but keeps an explicit one", 2, {0, 0, 0, 0, 3, 0}, {2, 2, 2, 2, 3, 2}},
        {"legacy extruder never overrides explicit roles",                   9, {2, 3, 4, 5, 6, 7}, {2, 3, 4, 5, 6, 7}},
        {"sparse infill role fills the unset solid roles",                   no_legacy_extruder, {4, 0, 0, 0, 0, 0}, {4, 4, 4, 4, 0, 0}},
        {"explicit solid role is not replaced by the sparse infill role",    no_legacy_extruder, {4, 3, 0, 0, 0, 0}, {4, 3, 3, 3, 0, 0}},
        {"solid role fills the unset top and bottom roles",                  no_legacy_extruder, {0, 7, 0, 0, 0, 0}, {0, 7, 7, 7, 0, 0}},
        {"explicit roles are left alone without a legacy extruder",          no_legacy_extruder, {2, 3, 4, 5, 6, 7}, {2, 3, 4, 5, 6, 7}},
    };
    const char *const role_keys[6] = {"sparse_infill_filament_id", "internal_solid_filament_id", "top_surface_filament_id",
                                      "bottom_surface_filament_id", "outer_wall_filament_id", "inner_wall_filament_id"};

    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.name) {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            if (row.legacy_extruder != no_legacy_extruder)
                config.option<ConfigOptionInt>("extruder", true)->value = row.legacy_extruder;
            for (size_t i = 0; i < 6; ++i)
                config.set(role_keys[i], row.roles[i]);

            if (split_step)
                config.normalize_fdm_1();
            else
                config.normalize_fdm(0);

            CHECK_FALSE(config.has("extruder"));
            for (size_t i = 0; i < 6; ++i) {
                INFO(role_keys[i]);
                CHECK(config.opt_int(role_keys[i]) == row.expected[i]);
            }
        }
    }
}

TEST_CASE("get_parameter_size scales per-extruder options by their variant counts", "[Config][ConfigVariants]")
{
    // The three variant counts differ (and none is twice another) so each option group is told apart.
    DynamicPrintConfig config;
    config.set_key_value("printer_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow"}));
    config.set_key_value("filament_extruder_variant",
                         new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"}));
    config.set_key_value("print_extruder_variant",
                         new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard", "Bowden High Flow", "Bowden Other"}));

    CHECK(config.get_parameter_size("nozzle_volume", 9) == 2);                // one value per printer variant
    CHECK(config.get_parameter_size("machine_max_acceleration_x", 9) == 4);   // two values (limit pair) per printer variant
    CHECK(config.get_parameter_size("machine_max_speed_x", 9) == 4);
    CHECK(config.get_parameter_size("filament_flow_ratio", 9) == 3);
    CHECK(config.get_parameter_size("outer_wall_speed", 9) == 5);
    CHECK(config.get_parameter_size("unknown_option", 9) == 9);               // everything else follows the extruder count

    // Without the variant lists every variant group has a single entry.
    DynamicPrintConfig bare;
    CHECK(bare.get_parameter_size("nozzle_volume", 9) == 1);
    CHECK(bare.get_parameter_size("machine_max_acceleration_x", 9) == 2);
    CHECK(bare.get_parameter_size("filament_flow_ratio", 9) == 1);
    CHECK(bare.get_parameter_size("outer_wall_speed", 9) == 1);
    CHECK(bare.get_parameter_size("unknown_option", 9) == 9);
}

// Known bug: PrintConfigDef never calls init_filament_option_keys(), so filament_option_keys() is empty and
// set_num_filaments() resizes nothing; fixed by the follow-up fix PR. Only assertions that fail on main are
// tagged: the prefix of the vectors is trivially unchanged today and is covered by the fix PR.
TEST_CASE("set_num_filaments grows filament vectors with their first value and shrinks them", "[Config][ConfigVariants][!shouldfail]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_colour", new ConfigOptionStrings({"#112233", "#445566"}));
    config.set_key_value("filament_diameter", new ConfigOptionFloats({1.70, 1.80}));

    const auto *colours   = config.option<ConfigOptionStrings>("filament_colour");
    const auto *diameters = config.option<ConfigOptionFloats>("filament_diameter");

    SECTION("growing")
    {
        config.set_num_filaments(3);
        REQUIRE(colours->values.size() == 3);
        REQUIRE(diameters->values.size() == 3);
        // ConfigOptionVector::resize extends a non-empty vector with its first value.
        CHECK(colours->values[2] == "#112233");
        CHECK_THAT(diameters->values[2], Catch::Matchers::WithinAbs(1.70, 1e-12));
    }

    SECTION("shrinking")
    {
        config.set_num_filaments(1);
        CHECK(colours->values.size() == 1);
        CHECK(diameters->values.size() == 1);
    }
}

namespace {

// Three slots: two physical filaments and one mixed slot blending them with a gradient. Component numbers are
// 1-based slot numbers of the physical filaments, which come first.
DynamicPrintConfig valid_mixed_filament_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_is_mixed", new ConfigOptionBools({false, false, true}));
    config.set_key_value("filament_mixed_components", new ConfigOptionStrings({"", "", "1,2"}));
    config.set_key_value("filament_mixed_sublayer_ratios", new ConfigOptionStrings({"", "", "0.5,0.5"}));
    config.set_key_value("filament_mixed_gradient", new ConfigOptionBools({false, false, true}));
    config.set_key_value("filament_mixed_gradient_range", new ConfigOptionStrings({"", "", "0.2,0.8"}));
    config.set_key_value("filament_mixed_gradient_curve", new ConfigOptionStrings({"", "", "0,0.2|1,0.8"}));
    return config;
}

void set_strings(DynamicPrintConfig &config, const char *key, std::vector<std::string> values)
{
    config.set_key_value(key, new ConfigOptionStrings(std::move(values)));
}

void set_bools(DynamicPrintConfig &config, const char *key, std::vector<unsigned char> values)
{
    config.set_key_value(key, new ConfigOptionBools(std::move(values)));
}

} // namespace

TEST_CASE("Mixed filament parameters validate against the filament slot cardinality", "[Config][Validation][MixedFilament]")
{
    REQUIRE(valid_mixed_filament_config().validate().empty());

    struct Row {
        const char                                *name;
        std::function<void(DynamicPrintConfig &)> mutate;
        const char                                *key;      // the only key reported; nullptr when accepted
        const char                                *message;
    };
    const Row rows[] = {
        {"components for a config without a mixed slot",
         [](DynamicPrintConfig &c) { c = DynamicPrintConfig::full_print_config(); set_strings(c, "filament_mixed_components", {"1"}); },
         "filament_is_mixed", "must be set"},
        {"ratios for a config without a mixed slot",
         [](DynamicPrintConfig &c) { c = DynamicPrintConfig::full_print_config(); set_strings(c, "filament_mixed_sublayer_ratios", {"0.5"}); },
         "filament_is_mixed", "must be set"},
        {"a gradient flag for a config without a mixed slot",
         [](DynamicPrintConfig &c) { c = DynamicPrintConfig::full_print_config(); set_bools(c, "filament_mixed_gradient", {1}); },
         "filament_is_mixed", "must be set"},
        {"gradient range and curve alone do not require a mixed slot",
         [](DynamicPrintConfig &c) {
             c = DynamicPrintConfig::full_print_config();
             set_strings(c, "filament_mixed_gradient_range", {"0.2,0.8"});
             set_strings(c, "filament_mixed_gradient_curve", {"0,0.2|1,0.8"});
         },
         nullptr, nullptr},
        {"components array shorter than the slot count",
         [](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_components", {"", ""}); },
         // The missing entry also leaves slot 3 without components, reported on the same key.
         "filament_mixed_components", "array size 2 does not match filament slot count 3; slot 3: empty component list"},
        {"ratios array shorter than the slot count",
         [](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_sublayer_ratios", {"", ""}); },
         "filament_mixed_sublayer_ratios", "does not match filament slot count 3"},
        {"gradient array shorter than the slot count",
         [](DynamicPrintConfig &c) { set_bools(c, "filament_mixed_gradient", {0, 1}); },
         "filament_mixed_gradient", "does not match filament slot count 3"},
        {"gradient range array shorter than the slot count",
         [](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_gradient_range", {"", "0.2,0.8"}); },
         "filament_mixed_gradient_range", "does not match filament slot count 3"},
        {"curve array shorter than the slot count",
         [](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_gradient_curve", {"", "0,0.2|1,0.8"}); },
         "filament_mixed_gradient_curve", "does not match filament slot count 3"},
        {"longer arrays are also a mismatch",
         [](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_components", {"", "", "1,2", ""}); },
         "filament_mixed_components", "array size 4 does not match filament slot count 3"},
        {"gradient arrays are not required when no slot has a gradient",
         [](DynamicPrintConfig &c) {
             set_bools(c, "filament_mixed_gradient", {0});
             set_strings(c, "filament_mixed_gradient_range", {});
             set_strings(c, "filament_mixed_gradient_curve", {});
         },
         nullptr, nullptr},
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.name) {
            DynamicPrintConfig config = valid_mixed_filament_config();
            row.mutate(config);
            const auto errors = config.validate();
            if (row.key == nullptr) {
                CHECK(errors.empty());
            } else {
                CHECK(errors.size() == 1);
                CHECK_THAT(error_message(errors, row.key), Catch::Matchers::ContainsSubstring(row.message));
            }
        }
    }
}

TEST_CASE("Mixed filament components, ratios and gradients validate their semantics", "[Config][Validation][MixedFilament]")
{
    struct Row {
        const char                                *name;
        std::function<void(DynamicPrintConfig &)> mutate;
        const char                                *key; // the only key reported; nullptr when accepted
        const char                                *message;
    };
    // Five slots: physical 1, 4 and 5; mixed 2 and 3. Components name slot numbers up to the physical
    // filament count (3), so both mixed slots below reference another mixed slot.
    const auto mixed_over_mixed = [](DynamicPrintConfig &c) {
        set_bools(c, "filament_is_mixed", {0, 1, 1, 0, 0});
        set_strings(c, "filament_mixed_components", {"", "3,1", "1,2", "", ""});
        set_strings(c, "filament_mixed_sublayer_ratios", {"", "1,1", "1,2", "", ""});
        set_bools(c, "filament_mixed_gradient", {0, 0, 0, 0, 0});
        set_strings(c, "filament_mixed_gradient_range", {"", "", "", "", ""});
        set_strings(c, "filament_mixed_gradient_curve", {"", "", "", "", ""});
    };
    const auto components = [](const char *value) {
        return [value](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_components", {"", "", value}); };
    };
    const auto ratios = [](const char *value) {
        return [value](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_sublayer_ratios", {"", "", value}); };
    };
    const auto range = [](const char *value) {
        return [value](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_gradient_range", {"", "", value}); };
    };
    const auto curve = [](const char *value) {
        return [value](DynamicPrintConfig &c) { set_strings(c, "filament_mixed_gradient_curve", {"", "", value}); };
    };
    const Row rows[] = {
        {"two physical components are accepted", components("2,1"), nullptr, nullptr},
        {"a component past the last physical filament", components("1,4"), "filament_mixed_components", "component 4 out of range"},
        {"component zero", components("0,2"), "filament_mixed_components", "must be >= 1"},
        {"a non-numeric component", components("a,2"), "filament_mixed_components", "invalid component index"},
        {"an empty component entry", components("1,,2"), "filament_mixed_components", "empty component index"},
        {"an empty component list", components(""), "filament_mixed_components", "empty component list"},
        {"a single component", components("1"), "filament_mixed_components", "at least 2 components"},
        {"a duplicated component", components("1,1"), "filament_mixed_components", "duplicate component index 1"},
        {"a mixed slot referencing itself",
         [](DynamicPrintConfig &c) {
             set_bools(c, "filament_is_mixed", {0, 0, 1, 0});
             set_strings(c, "filament_mixed_components", {"", "", "3,1", ""});
             set_strings(c, "filament_mixed_sublayer_ratios", {"", "", "1,1", ""});
             set_bools(c, "filament_mixed_gradient", {0, 0, 0, 0});
             set_strings(c, "filament_mixed_gradient_range", {"", "", "", ""});
             set_strings(c, "filament_mixed_gradient_curve", {"", "", "", ""});
         },
         "filament_mixed_components", "cannot reference itself"},
        {"a mixed slot used as a component", mixed_over_mixed,
         "filament_mixed_components", "references a mixed filament slot"},
        {"too few ratios", ratios("0.4"), "filament_mixed_sublayer_ratios", "expected 2 ratio(s), got 1"},
        {"too many ratios", ratios("1,1,1"), "filament_mixed_sublayer_ratios", "expected 2 ratio(s), got 3"},
        {"a zero ratio", ratios("0.5,0"), "filament_mixed_sublayer_ratios", "ratio must be positive"},
        {"a negative ratio", ratios("-1,2"), "filament_mixed_sublayer_ratios", "ratio must be positive"},
        {"a tiny positive ratio is accepted", ratios("0.001,1"), nullptr, nullptr},
        {"a non-numeric ratio", ratios("x,1"), "filament_mixed_sublayer_ratios", "invalid ratio"},
        {"an empty ratio entry", ratios("1,,1"), "filament_mixed_sublayer_ratios", "empty ratio value"},
        {"an empty ratio string is accepted", ratios(""), nullptr, nullptr},
        {"a gradient over three components",
         [](DynamicPrintConfig &c) {
             set_bools(c, "filament_is_mixed", {0, 0, 0, 1});
             set_strings(c, "filament_mixed_components", {"", "", "", "1,2,3"});
             set_strings(c, "filament_mixed_sublayer_ratios", {"", "", "", "1,1,1"});
             set_bools(c, "filament_mixed_gradient", {0, 0, 0, 1});
             set_strings(c, "filament_mixed_gradient_range", {"", "", "", "0.2,0.8"});
             set_strings(c, "filament_mixed_gradient_curve", {"", "", "", "0,0.2|1,0.8"});
         },
         "filament_mixed_gradient", "exactly 2 components"},
        {"range starting at zero", range("0,0.8"), "filament_mixed_gradient_range", "must be in (0, 1)"},
        {"range ending at one", range("0.2,1"), "filament_mixed_gradient_range", "must be in (0, 1)"},
        {"range just inside the open interval is accepted", range("0.001,0.999"), nullptr, nullptr},
        {"range ending at zero", range("0.5,0"), "filament_mixed_gradient_range", "must be in (0, 1)"},
        {"range starting at one", range("1,0.5"), "filament_mixed_gradient_range", "must be in (0, 1)"},
        {"range with one value", range("0.2"), "filament_mixed_gradient_range", "two comma-separated floats"},
        {"range that is not numeric", range("a,b"), "filament_mixed_gradient_range", "two comma-separated floats"},
        {"an empty range string is accepted", range(""), nullptr, nullptr},
        {"a curve with one control point", curve("0,0.2"), "filament_mixed_gradient_curve", "at least 2 valid control points"},
        {"a curve that is not numeric", curve("a,b|c,d"), "filament_mixed_gradient_curve", "at least 2 valid control points"},
        {"control points are separated by a bar, not a semicolon", curve("0,0.2;1,0.8"), "filament_mixed_gradient_curve", "at least 2 valid control points"},
        {"a curve with tangent fields is accepted", curve("0,0.2,,|1,0.8,,"), nullptr, nullptr},
        {"an empty curve string is accepted next to a slot with a curve",
         [](DynamicPrintConfig &c) {
             // Slot 4 keeps a curve, so curves are specified and slot 3's empty curve must be skipped, not parsed.
             set_bools(c, "filament_is_mixed", {0, 0, 1, 1});
             set_strings(c, "filament_mixed_components", {"", "", "1,2", "1,2"});
             set_strings(c, "filament_mixed_sublayer_ratios", {"", "", "0.5,0.5", "1,1"});
             set_bools(c, "filament_mixed_gradient", {0, 0, 1, 1});
             set_strings(c, "filament_mixed_gradient_range", {"", "", "0.2,0.8", "0.3,0.7"});
             set_strings(c, "filament_mixed_gradient_curve", {"", "", "", "0,0.2|1,0.8"});
         },
         nullptr, nullptr},
        {"a slot without a gradient is not checked for range or curve",
         [](DynamicPrintConfig &c) {
             // Slot 3 has a valid gradient; slot 4 has none, so its malformed range and curve are ignored.
             set_bools(c, "filament_is_mixed", {0, 0, 1, 1});
             set_strings(c, "filament_mixed_components", {"", "", "1,2", "1,2"});
             set_strings(c, "filament_mixed_sublayer_ratios", {"", "", "0.5,0.5", "1,1"});
             set_bools(c, "filament_mixed_gradient", {0, 0, 1, 0});
             set_strings(c, "filament_mixed_gradient_range", {"", "", "0.2,0.8", "0,5"});
             set_strings(c, "filament_mixed_gradient_curve", {"", "", "0,0.2|1,0.8", "x"});
         },
         nullptr, nullptr},
        {"every bad mixed slot is reported on the same key",
         [](DynamicPrintConfig &c) {
             set_bools(c, "filament_is_mixed", {0, 0, 1, 1});
             set_strings(c, "filament_mixed_components", {"", "", "1", "2,2"});
             set_strings(c, "filament_mixed_sublayer_ratios", {"", "", "", "1,1"});
             set_bools(c, "filament_mixed_gradient", {0, 0, 0, 0});
             set_strings(c, "filament_mixed_gradient_range", {"", "", "", ""});
             set_strings(c, "filament_mixed_gradient_curve", {"", "", "", ""});
         },
         "filament_mixed_components", "slot 3: at least 2 components required (got 1); slot 4: duplicate component index 2"},
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.name) {
            DynamicPrintConfig config = valid_mixed_filament_config();
            row.mutate(config);
            const auto errors = config.validate();
            if (row.key == nullptr) {
                CHECK(errors.empty());
            } else {
                CHECK(errors.size() == 1);
                CHECK_THAT(error_message(errors, row.key), Catch::Matchers::ContainsSubstring(row.message));
            }
        }
    }
}

TEST_CASE("get_real_skirt_dist follows skirt, loop, line width and nozzle fallbacks", "[Config][Skirt]")
{
    struct Row {
        int                 skirt_height;
        int                 skirt_loops;
        const char         *draft_shield;
        double              distance;
        double              initial_width;
        double              line_width;
        std::vector<double> nozzle_diameters;
        double              expected;
    };
    const Row rows[] = {
        {0, 0, "disabled", 1.5, 0.6, 0.4, {0.4}, 0.0},          // no skirt at all
        {1, 0, "disabled", 1.5, 0.6, 0.4, {0.4}, 0.0},          // no loops
        {0, 2, "disabled", 1.5, 0.6, 0.4, {0.4}, 0.0},          // no height
        {1, 2, "disabled", 1.5, 0.6, 0.4, {0.4}, 2.7},          // distance + loops * initial layer width
        {1, 2, "disabled", 1.0, 0.0, 0.5, {0.4}, 2.0},          // falls back to line_width
        {1, 1, "disabled", 0.2, 0.0, 0.0, {0.45, 0.6}, 0.8},    // then to the largest nozzle
        {1, 1, "disabled", 0.1, 0.0, 0.0, {}, 0.5},             // then to 0.4 mm
        {0, 3, "enabled",  0.3, 0.7, 0.4, {0.4}, 2.4},          // a draft shield alone counts the explicit loops
    };
    for (const Row &row : rows) {
        DYNAMIC_SECTION(row.skirt_height << " height, " << row.skirt_loops << " loops, draft " << row.draft_shield
                        << ", distance " << row.distance << ", widths " << row.initial_width << '/' << row.line_width
                        << ", " << row.nozzle_diameters.size() << " nozzles") {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.set("skirt_height", row.skirt_height);
            config.set("skirt_loops", row.skirt_loops);
            config.set("skirt_distance", row.distance);
            config.set_deserialize_strict("draft_shield", row.draft_shield);
            config.set("initial_layer_line_width", row.initial_width);
            config.set("line_width", row.line_width);
            config.option<ConfigOptionFloats>("nozzle_diameter")->values = row.nozzle_diameters;

            CHECK_THAT(get_real_skirt_dist(config), Catch::Matchers::WithinAbs(row.expected, 1e-5));
        }
    }
}

TEST_CASE("get_real_skirt_dist reserves no clearance for a draft shield without skirt loops", "[Config][Skirt]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set("skirt_height", 0);
    config.set("skirt_loops", 0);
    config.set("skirt_distance", 0.3);
    config.set_deserialize_strict("draft_shield", "enabled");
    config.set("initial_layer_line_width", 0.7);
    config.set("line_width", 0.4);
    config.option<ConfigOptionFloats>("nozzle_diameter")->values = {0.4};

    CHECK_THAT(get_real_skirt_dist(config), Catch::Matchers::WithinAbs(0., 1e-5));
}

TEST_CASE("AMS counts serialize sorted by id, keep empty extruders and parse back", "[Config][ExtruderInventory]")
{
    const std::vector<std::map<int, int>> source = {
        {{2, 5}, {1, 3}},
        {},
        {{4, 1}},
    };
    const auto serialized = save_extruder_ams_count_to_string(source);
    CHECK(serialized == std::vector<std::string>{"1#3|2#5", "", "4#1"});
    CHECK(get_extruder_ams_count(serialized) == source);
    CHECK(get_extruder_ams_count({}).empty());
    CHECK(save_extruder_ams_count_to_string({}).empty());
}

TEST_CASE("Nozzle inventory serializes every physical volume type by name and parses it back", "[Config][ExtruderInventory]")
{
    // Serialization follows the enum order, not the insertion order.
    const std::vector<std::map<NozzleVolumeType, int>> source = {
        {{nvtExtraHighFlow, 6}, {nvtE3DHighFlow, 5}, {nvtTPUHighFlow, 3}, {nvtHighFlow, 1}, {nvtStandard, 2}},
        {},
    };
    const auto serialized = save_extruder_nozzle_stats_to_string(source);
    CHECK(serialized == std::vector<std::string>{
        "Standard#2|High Flow#1|TPU High Flow#3|E3D High Flow#5|Extra High Flow#6", ""});
    CHECK(get_extruder_nozzle_stats(serialized) == source);
}

TEST_CASE("Nozzle inventory parsing skips tokens it cannot interpret and keeps the rest", "[Config][ExtruderInventory]")
{
    const auto parsed = get_extruder_nozzle_stats({"Standard#2|Broken|High Flow#1", "Unknown#9|E3D High Flow#3", ""});
    REQUIRE(parsed.size() == 3);
    // "Broken" has no count.
    CHECK(parsed[0] == std::map<NozzleVolumeType, int>{{nvtStandard, 2}, {nvtHighFlow, 1}});
    // "Unknown" is not a volume type name.
    CHECK(parsed[1] == std::map<NozzleVolumeType, int>{{nvtE3DHighFlow, 3}});
    CHECK(parsed[2].empty());
    CHECK(save_extruder_nozzle_stats_to_string(parsed) == std::vector<std::string>{"Standard#2|High Flow#1", "E3D High Flow#3", ""});
}

namespace {

double bed_area_mm2(const Points &points)
{
    return Polygon(points).area() * SCALING_FACTOR * SCALING_FACTOR;
}

} // namespace

TEST_CASE("get_bed_shape uses the intersection of the extruder areas only when sharing is requested", "[Config][BedShape]")
{
    const Vec2ds printable = {Vec2d(0., 0.), Vec2d(30., 0.), Vec2d(30., 16.), Vec2d(0., 16.)};
    // Two partially overlapping 20x20 extruder areas, (0,0)-(20,20) and (5,5)-(25,25), intersect in
    // (5,5)-(20,20) = 225 mm2, which is neither input (400 mm2 each). The printable area, (0,0)-(30,16) = 480 mm2, differs from both
    // extruder areas so that each call is pinned to the source it should read.
    const Vec2ds extruder_areas[] = {
        {Vec2d(0., 0.), Vec2d(20., 0.), Vec2d(20., 20.), Vec2d(0., 20.)},
        {Vec2d(5., 5.), Vec2d(25., 5.), Vec2d(25., 25.), Vec2d(5., 25.)},
    };
    const auto check_shared = [](const Points &shape) {
        CHECK_THAT(bed_area_mm2(shape), Catch::Matchers::WithinAbs(225., 1e-6));
        const BoundingBox box(shape);
        CHECK(box.min == Point(scaled(5.), scaled(5.)));
        CHECK(box.max == Point(scaled(20.), scaled(20.)));
    };

    const auto check_printable = [](const Points &shape) {
        CHECK_THAT(bed_area_mm2(shape), Catch::Matchers::WithinAbs(480., 1e-6));
        const BoundingBox box(shape);
        CHECK(box.min == Point(0, 0));
        CHECK(box.max == Point(scaled(30.), scaled(16.)));
    };

    DynamicPrintConfig dynamic;
    dynamic.set_key_value("printable_area", new ConfigOptionPoints(printable));
    dynamic.set_key_value("extruder_printable_area", new ConfigOptionPointsGroups({extruder_areas[0], extruder_areas[1]}));
    check_shared(get_bed_shape(dynamic, true));
    check_printable(get_bed_shape(dynamic, false));
    // The dynamic-config overload shares by default.
    check_shared(get_bed_shape(dynamic));

    FullPrintConfig full;
    full.printable_area.values = printable;
    full.extruder_printable_area.values = {extruder_areas[0], extruder_areas[1]};
    check_shared(get_bed_shape(full, true));
    check_printable(get_bed_shape(full, false));
    // The static-config overload does not share by default.
    check_printable(get_bed_shape(static_cast<const PrintConfig &>(full)));

    SECTION("a single extruder area replaces the printable area when sharing")
    {
        dynamic.set_key_value("extruder_printable_area",
                              new ConfigOptionPointsGroups({Vec2ds{Vec2d(0., 0.), Vec2d(10., 0.), Vec2d(10., 10.), Vec2d(0., 10.)}}));
        CHECK_THAT(bed_area_mm2(get_bed_shape(dynamic, true)), Catch::Matchers::WithinAbs(100., 1e-6));
        check_printable(get_bed_shape(dynamic, false));
    }

    SECTION("without extruder areas the printable area is used")
    {
        dynamic.erase("extruder_printable_area");
        full.extruder_printable_area.values.clear();
        check_printable(get_bed_shape(dynamic, true));
        check_printable(get_bed_shape(full, true));
    }
}

TEST_CASE("get_bed_shape returns a counter-clockwise polygon and handles a missing bed", "[Config][BedShape]")
{
    DynamicPrintConfig config;
    // A clockwise printable area.
    config.set_key_value("printable_area", new ConfigOptionPoints({Vec2d(0., 0.), Vec2d(0., 20.), Vec2d(30., 20.), Vec2d(30., 0.)}));
    const Points shape = get_bed_shape(config, false);
    REQUIRE(shape.size() == 4);
    CHECK(Polygon(shape).is_counter_clockwise());
    CHECK_THAT(bed_area_mm2(shape), Catch::Matchers::WithinAbs(600., 1e-6));

    // Without a printable area the shape is empty, or just the configured center.
    CHECK(get_bed_shape(DynamicPrintConfig{}, false).empty());
    DynamicPrintConfig centered;
    centered.set_key_value("center", new ConfigOptionPoint(Vec2d(12.5, 7.5)));
    const Points center = get_bed_shape(centered, true);
    REQUIRE(center.size() == 1);
    CHECK(center.front() == Point(scaled(12.5), scaled(7.5)));
}

TEST_CASE("Variant vector replacement restores kept slots and takes the rest from the source", "[Config][ConfigVariantRestore]")
{
    ConfigOptionInts destination({10, 20, 30, 40});
    ConfigOptionInts source({100, 200, 300});
    // Replace the two slots at 1..2. Each new slot restores the old destination value at its index, or takes
    // the source value when the index is -1 or past the old vector.
    std::vector<int> restore_index{3, -1, 99};

    destination.set_with_restore_2(&source, restore_index, 1, 2);

    CHECK(destination.values == std::vector<int>{10, 40, 200, 300, 40});
    CHECK(source.values == std::vector<int>{100, 200, 300});
    CHECK(restore_index == std::vector<int>{3, -1, 99});
}

TEST_CASE("Variant vector replacement clamps its range and pads a short source on request", "[Config][ConfigVariantRestore]")
{
    SECTION("the requested removal is clamped at the vector end") {
        ConfigOptionInts destination({1, 2, 3});
        ConfigOptionInts source({9});
        std::vector<int> restore_index{-1};

        destination.set_with_restore_2(&source, restore_index, 2, 8);

        CHECK(destination.values == std::vector<int>{1, 2, 9});
    }

    SECTION("a start at the end of the vector appends") {
        ConfigOptionInts destination({1, 2});
        ConfigOptionInts source({9});
        std::vector<int> restore_index{-1};

        destination.set_with_restore_2(&source, restore_index, 2, 0);

        CHECK(destination.values == std::vector<int>{1, 2, 9});
    }

    SECTION("a zero-length replacement inserts without deleting the following slot") {
        ConfigOptionInts destination({1, 2});
        ConfigOptionInts source({9});
        std::vector<int> restore_index{-1};

        destination.set_with_restore_2(&source, restore_index, 1, 0);

        CHECK(destination.values == std::vector<int>{1, 9, 2});
    }

    SECTION("a short source supplies its first value for unmatched slots when asked") {
        ConfigOptionInts destination({1, 2, 3, 4, 5});
        ConfigOptionInts source({8});
        std::vector<int> restore_index{1, -1, -1};

        destination.set_with_restore_2(&source, restore_index, 1, 2, true);

        CHECK(destination.values == std::vector<int>{1, 2, 8, 8, 4, 5});
    }
}

TEST_CASE("Variant vector replacement reports an invalid start, type or restore map", "[Config][ConfigVariantRestore]")
{
    ConfigOptionInts destination({1, 2});
    ConfigOptionInts integer_source({9});
    ConfigOptionFloats float_source({9.0});
    std::vector<int> one_slot{-1};

    CHECK_THROWS_WITH(destination.set_with_restore_2(&integer_source, one_slot, 3, 0),
                      Catch::Matchers::ContainsSubstring("invalid size found"));
    CHECK_THROWS_WITH(destination.set_with_restore_2(&float_source, one_slot, 0, 0),
                      Catch::Matchers::ContainsSubstring("incompatible type"));

    std::vector<int> wrong_shape{-1, -1};
    CHECK_THROWS_WITH(destination.set_with_restore_2(&integer_source, wrong_shape, 0, 1),
                      Catch::Matchers::ContainsSubstring("invalid restore_index size"));
}

// Known bug: set_with_restore_2 takes its source as const but const_casts it and resizes it when padding a short
// source, so the caller's vector grows; fixed by a follow-up fix PR.
TEST_CASE("Variant vector replacement leaves a short source unchanged when padding", "[Config][ConfigVariantRestore][!shouldfail]")
{
    ConfigOptionInts destination({1, 2, 3, 4, 5});
    ConfigOptionInts source({8}); // not const: the library const_casts it when padding
    std::vector<int> restore_index{1, -1, -1};

    destination.set_with_restore_2(&source, restore_index, 1, 2, true);

    CHECK(source.values == std::vector<int>{8});
}

namespace {

const std::set<std::string> variant_mapping_keys{
    "print_extruder_id", "print_extruder_variant", "outer_wall_speed", "small_perimeter_speed", "enable_overhang_speed"};

void set_variant_columns(DynamicPrintConfig &config, std::vector<int> ids, std::vector<std::string> variants)
{
    config.option<ConfigOptionInts>("print_extruder_id", true)->values = std::move(ids);
    config.option<ConfigOptionStrings>("print_extruder_variant", true)->values = std::move(variants);
}

void set_speeds(DynamicPrintConfig &config, std::vector<double> values)
{
    config.option<ConfigOptionFloats>("outer_wall_speed", true)->values = std::move(values);
}

std::vector<double> speeds(const DynamicPrintConfig &config)
{
    const auto *option = config.option<ConfigOptionFloats>("outer_wall_speed");
    REQUIRE(option != nullptr);
    return option->values;
}

void set_percent_speeds(DynamicPrintConfig &config, const std::vector<double> &values)
{
    auto &target = config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed", true)->values;
    target.clear();
    for (const double value : values)
        target.emplace_back(value, true);
}

std::vector<double> percent_speeds(const DynamicPrintConfig &config)
{
    const auto *option = config.option<ConfigOptionFloatsOrPercents>("small_perimeter_speed");
    REQUIRE(option != nullptr);
    std::vector<double> values;
    for (const FloatOrPercent &value : option->values) {
        CHECK(value.percent);
        values.push_back(value.value);
    }
    return values;
}

} // namespace

TEST_CASE("Single-to-multi copies the variant columns and caps each variant's value by the multi config", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_variant_columns(multi, {1, 1, 2}, {"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"});
    set_variant_columns(single, {99}, {"old"});
    set_speeds(single, {100.});
    set_percent_speeds(single, {50.});
    single.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {1};
    set_speeds(multi, {80., 160., 240.});
    set_percent_speeds(multi, {40., 60., 30.});
    multi.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 0, 0};

    std::set<std::string> keys = variant_mapping_keys;
    REQUIRE(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "print_extruder_variant") == 0);
    CHECK(single.option<ConfigOptionInts>("print_extruder_id")->values == std::vector<int>({1, 1, 2}));
    CHECK(single.option<ConfigOptionStrings>("print_extruder_variant")->values ==
          std::vector<std::string>({"Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"}));
    // The single value is spread over the variants and capped by the multi config's value in each column.
    CHECK(speeds(single) == std::vector<double>({80., 100., 100.}));
    CHECK(percent_speeds(single) == std::vector<double>({40., 50., 30.}));
    // Bools keep the single value for every variant; the multi config only supplies the width.
    CHECK(single.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({1, 1, 1}));
}

TEST_CASE("Single-to-multi leaves settings it cannot map untouched", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig single;
    DynamicPrintConfig multi;
    set_variant_columns(multi, {1, 2}, {"A", "B"});
    set_speeds(single, {42.});
    single.option<ConfigOptionInt>("wall_loops", true)->value = 7;
    multi.option<ConfigOptionInt>("wall_loops", true)->value = 3;

    SECTION("a key missing from the multi config, a scalar key and an unknown key")
    {
        std::set<std::string> keys{"outer_wall_speed", "wall_loops", "unknown_mapping_key"};
        REQUIRE(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "print_extruder_variant") == 0);
        CHECK(speeds(single) == std::vector<double>({42.}));
        CHECK(single.option<ConfigOptionInt>("wall_loops")->value == 7);
    }

    SECTION("an unknown key does not stop the known ones")
    {
        set_speeds(multi, {30., 40.});
        std::set<std::string> keys{"unknown_mapping_key", "outer_wall_speed"};
        REQUIRE(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "print_extruder_variant") == 0);
        CHECK(speeds(single) == std::vector<double>({30., 40.}));
    }

    SECTION("a multi config without the variant column is rejected")
    {
        std::set<std::string> keys{"outer_wall_speed"};
        CHECK(single.update_values_from_single_to_multi(multi, keys, "print_extruder_id", "missing_variant") == -1);
        CHECK(speeds(single) == std::vector<double>({42.}));
    }
}

TEST_CASE("Multi-to-multi keeps the smallest old value for each new variant", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    // Old rows are matched to the target variants by variant name only: the old print_extruder_id column is never
    // read, and both old A rows (30 and 10) feed the new A row, which keeps their minimum. The same name-only rule,
    // with the same minimum, is used by update_values_from_multi_to_multi_2. New rows are matched by extruder id
    // and variant.
    set_variant_columns(old_config, {1, 1, 2}, {"A", "B", "A"});
    set_variant_columns(new_config, {1, 1, 2, 2}, {"A", "B", "A", "B"});
    set_speeds(old_config, {30., 20., 10.});
    set_speeds(new_config, {100., 200., 300., 400.});
    set_percent_speeds(old_config, {30., 20., 10.});
    set_percent_speeds(new_config, {50., 60., 70., 80.});
    old_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 1, 0};
    new_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {1, 0, 0, 0};
    std::vector<std::string> new_extruder_variants{"A", "B"};

    std::set<std::string> keys = variant_mapping_keys;
    REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys, "print_extruder_id", "print_extruder_variant",
                                                         new_extruder_variants) == 0);
    CHECK(old_config.option<ConfigOptionInts>("print_extruder_id")->values == std::vector<int>({1, 1, 2, 2}));
    CHECK(old_config.option<ConfigOptionStrings>("print_extruder_variant")->values ==
          std::vector<std::string>({"A", "B", "A", "B"}));
    // New row 0 (extruder 1, A) takes the smaller of 100 and both old A values (30, 10); row 3 (extruder 2, B)
    // the smaller of 400 and the old B value 20; rows 1 and 2 have no matching extruder and keep the new values.
    CHECK(speeds(old_config) == std::vector<double>({10., 200., 300., 20.}));
    CHECK(percent_speeds(old_config) == std::vector<double>({10., 60., 70., 20.}));
    // An enabled old flag turns the matching new flag on; a disabled one leaves the new flag as it was.
    CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({1, 0, 0, 1}));
}

TEST_CASE("Multi-to-multi takes the new values where no target extruder matches", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    // The old config has no variant X; the new config has (extruder 1, X) and (extruder 2, B).
    set_variant_columns(old_config, {1, 2}, {"A", "B"});
    set_variant_columns(new_config, {1, 2}, {"X", "B"});
    set_speeds(old_config, {22., 11.});
    set_speeds(new_config, {101., 202.});
    old_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 1};
    new_config.option<ConfigOptionBools>("enable_overhang_speed", true)->values = {0, 0};
    std::set<std::string> keys{"outer_wall_speed", "enable_overhang_speed", "unknown_mapping_key"};

    SECTION("the extruder's variant is not in the old config, so every old row is considered")
    {
        std::vector<std::string> variants{"X"};
        REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys, "print_extruder_id", "print_extruder_variant", variants) == 0);
        // The smallest of all old speeds (22, 11) beats the new 101; any enabled old flag turns the new one on.
        CHECK(speeds(old_config) == std::vector<double>({11., 202.}));
        CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({1, 0}));
    }

    SECTION("no (extruder id, variant) pair exists in the new config")
    {
        std::vector<std::string> variants{"B", "A"};
        REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys, "print_extruder_id", "print_extruder_variant", variants) == 0);
        CHECK(speeds(old_config) == std::vector<double>({101., 202.}));
        CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({0, 0}));
    }

    SECTION("there are no target extruders")
    {
        std::vector<std::string> variants;
        REQUIRE(old_config.update_values_from_multi_to_multi(new_config, keys, "print_extruder_id", "print_extruder_variant", variants) == 0);
        CHECK(speeds(old_config) == std::vector<double>({101., 202.}));
        CHECK(old_config.option<ConfigOptionBools>("enable_overhang_speed")->values == std::vector<unsigned char>({0, 0}));
    }
}

TEST_CASE("Multi-to-multi rejects a config without the identity columns", "[Config][ConfigMultiMapping]")
{
    DynamicPrintConfig old_config;
    DynamicPrintConfig new_config;
    set_speeds(old_config, {11.});
    set_speeds(new_config, {101.});
    std::vector<std::string> variants{"A"};
    std::set<std::string> keys{"outer_wall_speed"};

    CHECK(old_config.update_values_from_multi_to_multi(new_config, keys, "missing_id", "missing_variant", variants) == -1);
    CHECK(speeds(old_config) == std::vector<double>({11.}));
}
