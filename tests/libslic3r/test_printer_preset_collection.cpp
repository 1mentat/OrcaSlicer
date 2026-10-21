#include <catch2/catch_test_macros.hpp>

#include <string>
#include <utility>

#include "libslic3r/Config.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"

using namespace Slic3r;

namespace {

void set_variant(DynamicPrintConfig &config, const std::string &model, const std::string &variant)
{
    config.option<ConfigOptionString>("printer_model", true)->value = model;
    config.option<ConfigOptionString>("printer_variant", true)->value = variant;
}

struct PrinterPresetLookupCollection : PrinterPresetCollection
{
    PrinterPresetLookupCollection()
        : PrinterPresetCollection(Preset::TYPE_PRINTER, Preset::printer_options(),
                                  static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {}

    void add_printer(const std::string &name, const std::string &model, const std::string &variant, bool is_system)
    {
        DynamicPrintConfig config = default_preset().config;
        set_variant(config, model, variant);
        Preset &preset = load_preset({}, name, std::move(config), false);
        preset.is_system = is_system;
    }
};

} // namespace

TEST_CASE("System printer lookup rejects empty models and skips matching user profiles",
          "[Preset][PresetDeep][PrinterPresetCollection]")
{
    PrinterPresetLookupCollection printers;
    printers.add_printer("A User Model A 0.4", "Model A", "0.4", false);
    printers.add_printer("B System Model B 0.4", "Model B", "0.4", true);
    printers.add_printer("C System Model A 0.4", "Model A", "0.4", true);
    printers.add_printer("D System Model A 0.6", "Model A", "0.6", true);

    Preset *user = printers.find_preset("A User Model A 0.4");
    Preset *other_model = printers.find_preset("B System Model B 0.4");
    Preset *system_04 = printers.find_preset("C System Model A 0.4");
    Preset *system_06 = printers.find_preset("D System Model A 0.6");
    REQUIRE(user != nullptr);
    REQUIRE(other_model != nullptr);
    REQUIRE(system_04 != nullptr);
    REQUIRE(system_06 != nullptr);

    CHECK_FALSE(user->is_system);
    CHECK(printers.find_system_preset_by_model_and_variant("", "0.4") == nullptr);
    CHECK(printers.find_system_preset_by_model_and_variant("Model A", "0.4") == system_04);
    CHECK(printers.find_system_preset_by_model_and_variant("Model A", "0.6") == system_06);
    CHECK(printers.find_system_preset_by_model_and_variant("Model A", "") == system_04);
    CHECK(printers.find_system_preset_by_model_and_variant("Model B", "0.4") == other_model);
    CHECK(printers.find_system_preset_by_model_and_variant("Model A", "0.8") == nullptr);
}

TEST_CASE("Custom printer lookup finds user profiles by model and variant",
          "[Preset][PresetDeep][PrinterPresetCollection]")
{
    PrinterPresetLookupCollection printers;
    printers.add_printer("A User Model A 0.4", "Model A", "0.4", false);
    printers.add_printer("B User Model B 0.4", "Model B", "0.4", false);
    printers.add_printer("C User Model A 0.6", "Model A", "0.6", false);

    Preset *model_a_04 = printers.find_preset("A User Model A 0.4");
    Preset *model_b_04 = printers.find_preset("B User Model B 0.4");
    Preset *model_a_06 = printers.find_preset("C User Model A 0.6");
    REQUIRE(model_a_04 != nullptr);
    REQUIRE(model_b_04 != nullptr);
    REQUIRE(model_a_06 != nullptr);

    CHECK(printers.find_custom_preset_by_model_and_variant("", "0.4") == nullptr);
    CHECK(printers.find_custom_preset_by_model_and_variant("Model A", "0.4") == model_a_04);
    CHECK(printers.find_custom_preset_by_model_and_variant("Model A", "0.6") == model_a_06);
    CHECK(printers.find_custom_preset_by_model_and_variant("Model A", "") == model_a_04);
    CHECK(printers.find_custom_preset_by_model_and_variant("Model A", "0.8") == nullptr);
    CHECK(printers.find_custom_preset_by_model_and_variant("Model B", "0.4") == model_b_04);
}

TEST_CASE("Only default printers ignores hidden presets and sees visible user presets",
          "[Preset][PresetDeep][PrinterPresetCollection]")
{
    PrinterPresetLookupCollection printers;
    CHECK(printers.only_default_printers());

    printers.add_printer("Custom Machine", "Model A", "0.4", false);
    Preset *user = printers.find_preset("Custom Machine");
    REQUIRE(user != nullptr);
    user->is_visible = false;
    CHECK(printers.only_default_printers());
    user->is_visible = true;
    CHECK_FALSE(printers.only_default_printers());
}
