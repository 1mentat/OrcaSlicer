#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <map>
#include "libslic3r/calib.hpp"
#include "libslic3r/PrintConfig.hpp"
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/operations.hpp>
#include <ios>
#include "libslic3r/Thread.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include "catch2/matchers/catch_matchers.hpp"
#include "catch2/matchers/catch_matchers_floating_point.hpp"
#include "libslic3r/Utils.hpp"
#include "test_utils.hpp"
#include <catch2/catch_all.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/Preset.hpp"

using namespace Slic3r;

TEST_CASE("AppConfig network version helpers", "[AppConfig]") {
    AppConfig config;

    SECTION("skipped versions starts empty") {
        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.empty());
    }

    SECTION("add and check skipped version") {
        config.add_skipped_network_version("02.01.01.52");
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
        REQUIRE_FALSE(config.is_network_version_skipped("02.03.00.62"));
    }

    SECTION("multiple skipped versions") {
        config.add_skipped_network_version("02.01.01.52");
        config.add_skipped_network_version("02.00.02.50");

        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.size() == 2);
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
        REQUIRE(config.is_network_version_skipped("02.00.02.50"));
    }

    SECTION("clear skipped versions") {
        config.add_skipped_network_version("02.01.01.52");
        config.clear_skipped_network_versions();
        REQUIRE_FALSE(config.is_network_version_skipped("02.01.01.52"));
    }

    SECTION("duplicate add is idempotent") {
        config.add_skipped_network_version("02.01.01.52");
        config.add_skipped_network_version("02.01.01.52");

        auto skipped = config.get_skipped_network_versions();
        REQUIRE(skipped.size() == 1);
        REQUIRE(config.is_network_version_skipped("02.01.01.52"));
    }
}

TEST_CASE("Remembered checkbox settings retain both selections", "[AppConfig][Regression]") {
    AppConfig config;
    const bool checked = GENERATE(false, true);

    config.set("recent", "checkbox", checked ? "1" : "0");
    CHECK(config.get("recent", "checkbox") == (checked ? "1" : "0"));
}

TEST_CASE("Boolean setters retain their established encoding", "[AppConfig][Regression]") {
    AppConfig config;
    const bool value = GENERATE(false, true);

    config.set("recent", "flag", value);
    CHECK(config.get("recent", "flag") == (value ? "true" : "false"));
}

TEST_CASE("Boolean reads use only the requested section", "[AppConfig][Regression]") {
    AppConfig config;
    const bool value = GENERATE(false, true);
    config.set("recent", "flag", std::string(value ? "1" : "0"));
    config.set("app", "flag", std::string(value ? "0" : "1"));

    CHECK(config.get_bool("recent", "flag") == value);
}

TEST_CASE("AppConfig Speed Dial recent count defaults, clamps and parses", "[AppConfig]") {
    AppConfig config;

    SECTION("unset falls back to the default") {
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_DEFAULT);
    }

    SECTION("zero disables recents") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "0");
        REQUIRE(config.get_speed_dial_recent_count() == 0);
    }

    SECTION("a value in range is returned as-is") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "7");
        REQUIRE(config.get_speed_dial_recent_count() == 7);
    }

    SECTION("the maximum is kept") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "10");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_MAX);
    }

    SECTION("values above the maximum clamp down") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "42");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_MAX);
    }

    SECTION("negative values clamp up to 0") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "-3");
        REQUIRE(config.get_speed_dial_recent_count() == 0);
    }

    SECTION("garbage falls back to the default") {
        config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "abc");
        REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_DEFAULT);
    }
}

namespace {

namespace fs = boost::filesystem;

// AppConfig reads and writes through process-wide data_dir().  Keep every test
// case in a unique directory and restore the caller's directory even on a
// failed assertion.
struct ScopedAppConfigDataDir {
    const std::string previous = data_dir();
    ScopedTemporaryDir directory {"orca-appconfig-contract"};

    ScopedAppConfigDataDir()
    {
        // Match application startup before exercising main-thread-only saves.
        save_main_thread_id();
        set_data_dir(directory.string());
    }
    ~ScopedAppConfigDataDir() { set_data_dir(previous); }

    const fs::path &path() const { return directory.path(); }
};

bool uses_json(const fs::path &path)
{
    return fs::extension(path) == ".conf";
}

void write_text(const fs::path &path, const std::string &text)
{
    std::ofstream out(path.string(), std::ios::binary | std::ios::trunc);
    REQUIRE(out.good());
    out << text;
    REQUIRE(out.good());
}

void write_config(const fs::path &path, const std::string &json, const std::string &ini)
{
    write_text(path, uses_json(path) ? json : ini);
}

void require_mouse_device(const AppConfig &config)
{
    const auto names = config.get_mouse_device_names();
    REQUIRE(names.size() == 1);
    REQUIRE(names.front() == "Draft Space Mouse");

    double translation_speed = 0.0;
    REQUIRE(config.get_mouse_device_translation_speed(names.front(), translation_speed));
    CHECK_THAT(translation_speed, Catch::Matchers::WithinAbs(1.25, 1e-6));

    double translation_deadzone = 0.0;
    REQUIRE(config.get_mouse_device_translation_deadzone(names.front(), translation_deadzone));
    CHECK_THAT(translation_deadzone, Catch::Matchers::WithinAbs(0.15, 1e-6));

    float rotation_speed = 0.0f;
    REQUIRE(config.get_mouse_device_rotation_speed(names.front(), rotation_speed));
    CHECK_THAT(rotation_speed, Catch::Matchers::WithinAbs(2.0, 1e-6));

    bool swap_yz = false;
    REQUIRE(config.get_mouse_device_swap_yz(names.front(), swap_yz));
    REQUIRE(swap_yz);

    bool invert_x = false;
    REQUIRE(config.get_mouse_device_invert_x(names.front(), invert_x));
    REQUIRE_FALSE(invert_x);

    bool invert_y = false;
    REQUIRE(config.get_mouse_device_invert_y(names.front(), invert_y));
    REQUIRE(invert_y);
}

} // namespace

TEST_CASE("AppConfig save/load preserves typed sections and collections", "[AppConfig][AppConfigContract]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    source.set("app", "draft_scalar", "alpha=beta");
    source.set_bool("draft_boolean", true);
    source.set_recent_projects({"first.3mf", "second.3mf"});
    source.set_mouse_device("Draft Space Mouse", 1.25, 0.15f, 2.0f, 0.25f, 3.5, true, false, true, false, true, false, true);
    source.set_variant("Vendor \"Draft\"", "Model\\Draft", "0.4\\special", true);
    source.set_printer_setting("Draft Printer", PRESET_PRINTER_NAME, "Draft Printer");
    source.set_printer_setting("Draft Printer", "curr_bed_type", "Textured PEI");
    source.set_filament_presets({"Draft PLA", "Draft PETG"});
    source.set_filament_colors({"#112233", "#445566"});

    BBLocalMachine machine;
    machine.dev_id = "draft-device-id";
    machine.dev_name = "Draft Printer";
    machine.dev_ip = "192.0.2.17";
    machine.printer_type = "draft-model";
    machine.printer_agent_id = "draft-agent";
    machine.access_code = "draft-secret";
    source.update_local_machine(machine);

    const fs::path config_path(source.config_path());
    source.save();
    REQUIRE(fs::exists(config_path));

    AppConfig loaded;
    const std::string load_error = loaded.load();
    INFO(load_error);
    REQUIRE(load_error.empty());
    REQUIRE(loaded.get("app", "draft_scalar") == "alpha=beta");
    REQUIRE(loaded.get_bool("draft_boolean"));
    REQUIRE(loaded.get_recent_projects() == std::vector<std::string>{"first.3mf", "second.3mf"});
    require_mouse_device(loaded);
    REQUIRE(loaded.get_variant("Vendor \"Draft\"", "Model\\Draft", "0.4\\special"));
    REQUIRE(loaded.get_printer_setting("Draft Printer", "curr_bed_type") == "Textured PEI");
    REQUIRE(loaded.get_filament_presets() == std::vector<std::string>{"Draft PLA", "Draft PETG"});
    REQUIRE(loaded.get_filament_colors() == std::vector<std::string>{"#112233", "#445566"});
    REQUIRE(loaded.get_local_machines().size() == 1);
    REQUIRE(loaded.get_local_machines().at("draft-device-id") == machine);
}

TEST_CASE("AppConfig missing load is side-effect free", "[AppConfig][AppConfigContract]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig config;
    const fs::path missing = scoped_dir.path() / "missing.conf";
    config.set_loading_path(missing.string());

    REQUIRE(config.load_if_exists().empty());
    REQUIRE_FALSE(fs::exists(missing));
    REQUIRE(config.get_bool("drop_project_action"));
}

TEST_CASE("AppConfig malformed input reports an error and keeps the file", "[AppConfig][AppConfigContract]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig config;
    const fs::path malformed = scoped_dir.path() / (uses_json(fs::path(config.config_path())) ? "malformed.conf" : "malformed.ini");
    const std::string json = R"({"app":{"draft_malformed":true})";
    const std::string ini = "[app\n draft_malformed = true\n";
    write_config(malformed, json, ini);
    config.set_loading_path(malformed.string());

    REQUIRE_FALSE(config.load().empty());
    REQUIRE(fs::exists(malformed));
    REQUIRE(config.get("app", "draft_malformed").empty());
}

TEST_CASE("AppConfig numeric defaults clamp and write back", "[AppConfig][AppConfigContract]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig config;
    const fs::path config_path(config.config_path());
    const std::string json =
        R"({"app":{"opengl_fps_cap":"999","plugin_pages_visible_count":"abc","speed_dial_recent_count":"-3"}})";
    const std::string ini =
        "[app]\n"
        "opengl_fps_cap = 999\n"
        "plugin_pages_visible_count = abc\n"
        "speed_dial_recent_count = -3\n";
    write_config(config_path, json, ini);
    REQUIRE(config.load().empty());
    REQUIRE(config.get(SETTING_OPENGL_FPS_CAP) == "240");
    REQUIRE(config.get_plugin_pages_visible_count() == PLUGIN_PAGES_VISIBLE_COUNT_DEFAULT);
    REQUIRE(config.get_speed_dial_recent_count() == SPEED_DIAL_RECENT_COUNT_MIN);

    config.set(SETTING_PLUGIN_PAGES_VISIBLE_COUNT, "-7");
    config.set(SETTING_SPEED_DIAL_RECENT_COUNT, "99");
    config.set_defaults();
    REQUIRE(config.get(SETTING_PLUGIN_PAGES_VISIBLE_COUNT) == std::to_string(PLUGIN_PAGES_VISIBLE_COUNT_MIN));
    REQUIRE(config.get(SETTING_SPEED_DIAL_RECENT_COUNT) == std::to_string(SPEED_DIAL_RECENT_COUNT_MAX));
    config.save();

    AppConfig reloaded;
    REQUIRE(reloaded.load().empty());
    REQUIRE(reloaded.get(SETTING_OPENGL_FPS_CAP) == "240");
    REQUIRE(reloaded.get(SETTING_PLUGIN_PAGES_VISIBLE_COUNT) == std::to_string(PLUGIN_PAGES_VISIBLE_COUNT_MIN));
    REQUIRE(reloaded.get(SETTING_SPEED_DIAL_RECENT_COUNT) == std::to_string(SPEED_DIAL_RECENT_COUNT_MAX));
}

TEST_CASE("AppConfig Editor load migrates legacy physical printer", "[AppConfig][AppConfigContract]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig config;
    const fs::path config_path(config.config_path());
    const std::string json = R"({"extras":{"physical_printer":"Legacy Draft Printer"},"app":{"installed_networking":false}})";
    const std::string ini =
        "[extras]\n"
        "physical_printer = Legacy Draft Printer\n"
        "\n[app]\n"
        "installed_networking = false\n";
    write_config(config_path, json, ini);

    REQUIRE(config.load().empty());
    REQUIRE(config.get("presets", "physical_printer") == "Legacy Draft Printer");
    REQUIRE_FALSE(config.has_section("extras"));
    REQUIRE_FALSE(config.get_bool("app", "installed_networking"));
}

namespace appconfig_extended_contracts {

namespace {

void require_close(float actual, float expected)
{
    CHECK_THAT(actual, Catch::Matchers::WithinAbs(expected, 1e-5));
}

void require_calibration(const PrinterCaliInfo &actual, const PrinterCaliInfo &expected)
{
    REQUIRE(actual.dev_id == expected.dev_id);
    REQUIRE(actual.cali_finished == expected.cali_finished);
    require_close(actual.cache_flow_ratio, expected.cache_flow_ratio);
    REQUIRE(actual.cache_flow_rate_calibration_type == expected.cache_flow_rate_calibration_type);
    REQUIRE(actual.selected_presets.size() == expected.selected_presets.size());
    REQUIRE(actual.selected_presets.size() == 1);

    const CaliPresetInfo &actual_preset = actual.selected_presets.front();
    const CaliPresetInfo &expected_preset = expected.selected_presets.front();
    REQUIRE(actual_preset.tray_id == expected_preset.tray_id);
    REQUIRE(actual_preset.extruder_id == expected_preset.extruder_id);
    REQUIRE(actual_preset.nozzle_volume_type == expected_preset.nozzle_volume_type);
    REQUIRE(actual_preset.bed_type == expected_preset.bed_type);
    require_close(actual_preset.nozzle_diameter, expected_preset.nozzle_diameter);
    REQUIRE(actual_preset.nozzle_pos_id == expected_preset.nozzle_pos_id);
    REQUIRE(actual_preset.nozzle_sn == expected_preset.nozzle_sn);
    REQUIRE(actual_preset.filament_id == expected_preset.filament_id);
    REQUIRE(actual_preset.setting_id == expected_preset.setting_id);
    REQUIRE(actual_preset.name == expected_preset.name);
}

} // namespace

TEST_CASE("calibration records survive save and load", "[AppConfig][AppConfigExtended]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;

    PrinterCaliInfo initial;
    initial.dev_id = "calibration-device";
    initial.cali_finished = false;
    initial.cache_flow_ratio = 0.987f;
    initial.cache_flow_rate_calibration_type = FINE_CALIBRATION;

    CaliPresetInfo preset;
    preset.tray_id = 7;
    preset.extruder_id = 1;
    preset.nozzle_volume_type = nvtHighFlow;
    preset.bed_type = btPEI;
    preset.nozzle_diameter = 0.6f;
    preset.nozzle_pos_id = 2;
    preset.nozzle_sn = "nozzle\\SN-7";
    preset.filament_id = "filament\\id";
    preset.setting_id = "setting-7";
    preset.name = "Calibration preset 7";
    initial.selected_presets.push_back(preset);

    source.save_printer_cali_infos(initial);
    REQUIRE(source.get_printer_cali_infos().size() == 1);

    PrinterCaliInfo update = initial;
    update.cali_finished = true;
    update.cache_flow_ratio = 1.023f;
    update.selected_presets.front().name = "Calibration preset 7 updated";
    source.save_printer_cali_infos(update, false);
    REQUIRE(source.get_printer_cali_infos().front().cali_finished == false);
    source.save_printer_cali_infos(update, true);
    REQUIRE(source.get_printer_cali_infos().front().cali_finished);

    source.save();
    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_printer_cali_infos().size() == 1);
    require_calibration(loaded.get_printer_cali_infos().front(), update);
}

TEST_CASE("printer snapshots remain separated by machine", "[AppConfig][AppConfigExtended]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    source.set_printer_setting("Printer A", PRESET_PRINTER_NAME, "Printer A");
    source.set_printer_setting("Printer A", PRESET_PRINT_NAME, "Quality A");
    source.set_printer_setting("Printer A", "curr_bed_type", "Engineering Plate");
    source.set_printer_setting("Printer B", PRESET_PRINTER_NAME, "Printer B");
    source.set_printer_setting("Printer B", PRESET_PRINT_NAME, "Quality B");
    source.set_printer_setting("Printer B", "curr_bed_type", "Textured PEI");

    source.save();
    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.has_printer_settings("Printer A"));
    REQUIRE(loaded.has_printer_settings("Printer B"));
    REQUIRE(loaded.get_printer_setting("Printer A", PRESET_PRINT_NAME) == "Quality A");
    REQUIRE(loaded.get_printer_setting("Printer A", "curr_bed_type") == "Engineering Plate");
    REQUIRE(loaded.get_printer_setting("Printer B", PRESET_PRINT_NAME) == "Quality B");
    REQUIRE(loaded.get_printer_setting("Printer B", "curr_bed_type") == "Textured PEI");
}

TEST_CASE("vendor records filter empty models and preserve escaped variants", "[AppConfig][AppConfigExtended]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;

    AppConfig::VendorMap vendors;
    vendors["Skipped Vendor"]["Skipped Model"] = {};
    vendors["Mixed Vendor"]["Empty Model"] = {};
    vendors["Mixed Vendor"]["Model\\with\\slashes"] = {"0.4\\draft", "0.6,quoted"};
    source.set_vendors(vendors);
    source.save();

    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_variant("Mixed Vendor", "Model\\with\\slashes", "0.4\\draft"));
    REQUIRE(loaded.get_variant("Mixed Vendor", "Model\\with\\slashes", "0.6,quoted"));
    REQUIRE_FALSE(loaded.get_variant("Mixed Vendor", "Empty Model", "0.4"));
    REQUIRE_FALSE(loaded.get_variant("Skipped Vendor", "Skipped Model", "0.4"));
}

TEST_CASE("cloud and skipped-version collections persist their public invariants", "[AppConfig][AppConfigExtended]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    source.set_cloud_providers({"bambu", "custom"});
    source.add_cloud_provider("custom");
    source.add_cloud_provider("new-provider");
    source.remove_cloud_provider("bambu");
    source.remove_cloud_provider("orca");
    source.add_skipped_network_version("02.01.01.52");
    source.add_skipped_network_version("02.03.00.62");
    source.add_skipped_network_version("02.01.01.52");

    source.save();
    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_cloud_providers() == std::vector<std::string>{"orca", "custom", "new-provider"});
    REQUIRE(loaded.has_cloud_provider("orca"));
    REQUIRE(loaded.has_cloud_provider("custom"));
    REQUIRE_FALSE(loaded.has_cloud_provider("bambu"));
    REQUIRE(loaded.get_skipped_network_versions() == std::vector<std::string>{"02.01.01.52", "02.03.00.62"});

    loaded.clear_skipped_network_versions();
    loaded.remove_cloud_provider("custom");
    loaded.save();
    AppConfig cleared;
    REQUIRE(cleared.load().empty());
    REQUIRE(cleared.get_skipped_network_versions().empty());
    REQUIRE(cleared.get_cloud_providers() == std::vector<std::string>{"orca", "new-provider"});
}

TEST_CASE("reset_selections removes persisted selection keys only", "[AppConfig][AppConfigExtended]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    source.set("presets", PRESET_PRINT_NAME, "Quality");
    source.set("presets", PRESET_FILAMENT_NAME, "PLA");
    source.set("presets", PRESET_PRINTER_NAME, "Printer A");
    source.set("presets", "sla_print", "SLA Quality");
    source.set("presets", "sla_material", "SLA Resin");
    source.set("presets", "physical_printer", "Printer A / LAN");
    source.set("presets", "keep_this", "retained");

    source.reset_selections();
    REQUIRE(source.get("presets", PRESET_PRINT_NAME).empty());
    REQUIRE(source.get("presets", PRESET_FILAMENT_NAME).empty());
    REQUIRE(source.get("presets", PRESET_PRINTER_NAME).empty());
    REQUIRE(source.get("presets", "sla_print").empty());
    REQUIRE(source.get("presets", "sla_material").empty());
    REQUIRE(source.get("presets", "physical_printer").empty());
    REQUIRE(source.get("presets", "keep_this") == "retained");

    source.save();
    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get("presets", "keep_this") == "retained");
    REQUIRE(loaded.get("presets", PRESET_PRINTER_NAME).empty());
}


} // namespace appconfig_extended_contracts

TEST_CASE("Directory history excludes the shapes gallery and persists export fallbacks",
          "[AppConfig][AppConfigPreferences]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    const std::string settings = (scoped_dir.path() / "settings").string();
    const std::string models = (scoped_dir.path() / "models").string();
    const std::string exports = (scoped_dir.path() / "exports").string();
    const std::string backups = (scoped_dir.path() / "backups").string();
    const std::string gallery = custom_shapes_dir();

    REQUIRE(source.get_last_dir().empty());
    REQUIRE(source.get_last_backup_dir().empty());
    REQUIRE(source.get_last_output_dir(exports) == exports);
    source.update_config_dir(settings);
    REQUIRE(source.get_last_dir() == settings);
    source.update_skein_dir(models);
    REQUIRE(source.get_last_dir() == models);
    source.update_skein_dir(gallery);
    REQUIRE(source.get_last_dir() == models);
    REQUIRE(source.get("recent", "last_opened_folder") == models);
    REQUIRE(source.get("recent", "settings_folder") == settings);
    REQUIRE(source.get_last_output_dir(gallery) == models);

    source.set("recent", "last_opened_folder", std::string());
    REQUIRE(source.get_last_dir() == settings);
    REQUIRE(source.get_last_output_dir(gallery) == settings);
    source.update_last_output_dir(exports);
    REQUIRE(source.get_last_output_dir(gallery) == exports);
    REQUIRE(source.get_last_output_dir(models, true) == exports);

    // Updating the backup directory saves all pending preferences immediately.
    source.update_last_backup_dir(backups);
    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_last_dir() == settings);
    REQUIRE(loaded.get_last_output_dir(models) == exports);
    REQUIRE(loaded.get_last_backup_dir() == backups);
}

TEST_CASE("Copied vendor selections remain independent of their source",
          "[AppConfig][AppConfigPreferences]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    source.set_variant("Vendor A", "Model A", "0.4", true);
    source.set_variant("Vendor A", "Model A", "0.6", true);
    source.set_variant("Vendor B", "Model B", "0.8", true);
    const AppConfig::VendorMap expected = source.vendors();

    AppConfig copied;
    copied.set_variant("Old Vendor", "Old Model", "0.2", true);
    copied.set_vendors(source);
    source.set_variant("Vendor A", "Model A", "0.4", false);
    source.set_variant("Vendor B", "Model B", "1.0", true);
    REQUIRE(copied.vendors() == expected);
    REQUIRE_FALSE(source.vendors() == expected);
    copied.save();

    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.vendors() == expected);
    REQUIRE_FALSE(loaded.get_variant("Old Vendor", "Old Model", "0.2"));
}

TEST_CASE("Custom palette updates retain color order after reload",
          "[AppConfig][AppConfigPreferences]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    REQUIRE(source.get_custom_color_from_config().empty());
    source.save_custom_color_to_config({});
    REQUIRE(source.get_custom_color_from_config().empty());

    const std::vector<std::string> initial {"#112233", "#AABBCC"};
    const std::vector<std::string> updated {"#445566", "#DDEEFF", "#778899"};
    source.save_custom_color_to_config(initial);
    REQUIRE(source.get_custom_color_from_config() == initial);
    source.save_custom_color_to_config(updated);
    REQUIRE(source.get_custom_color_from_config() == updated);
    source.save();

    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_custom_color_from_config() == updated);
}

TEST_CASE("Nozzle volume preferences update one printer without changing another",
          "[AppConfig][AppConfigPreferences]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig source;
    REQUIRE(source.get_nozzle_volume_types_from_config("Printer A").empty());
    source.save_nozzle_volume_types_to_config("Printer A", "0;1");
    source.save_nozzle_volume_types_to_config("Printer B", "1;0;1");
    source.save_nozzle_volume_types_to_config("Printer A", "1;1");
    REQUIRE(source.get_nozzle_volume_types_from_config("Printer A") == "1;1");
    REQUIRE(source.get_nozzle_volume_types_from_config("Printer B") == "1;0;1");
    REQUIRE(source.get_nozzle_volume_types_from_config("Unknown Printer").empty());
    source.save();

    AppConfig loaded;
    REQUIRE(loaded.load().empty());
    REQUIRE(loaded.get_nozzle_volume_types_from_config("Printer A") == "1;1");
    REQUIRE(loaded.get_nozzle_volume_types_from_config("Printer B") == "1;0;1");
    REQUIRE(loaded.get_nozzle_volume_types_from_config("Unknown Printer").empty());
}

TEST_CASE("AppConfig normalizes language codes", "[AppConfig][AppConfigPreferences][AppConfigLanguage]")
{
    AppConfig config;
    const auto [language, expected] = GENERATE(
        std::make_pair(std::string("zh_CN"), std::string("zh-cn")),
        std::make_pair(std::string("en_US"), std::string("en")),
        std::make_pair(std::string("x"), std::string("x")),
        std::make_pair(std::string(), std::string()));
    config.set("language", language);
    REQUIRE(config.get_language_code() == expected);
}

TEST_CASE("AppConfig uses the stable HMS host", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    REQUIRE(config.get_hms_host() == "e.bambulab.com");
}

TEST_CASE("AppConfig gates stealth mode on setup completion", "[AppConfig][AppConfigPreferences][AppConfigStealth]")
{
    AppConfig config;
    config.set_bool("stealth_mode", false);
    REQUIRE(config.get_stealth_mode());

    config.set("firstguide", "finish", true);
    REQUIRE_FALSE(config.get_stealth_mode());

    config.set_bool("stealth_mode", true);
    REQUIRE(config.get_stealth_mode());
}

TEST_CASE("AppConfig reads login panel visibility", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    const bool hidden = GENERATE(false, true);
    config.set_bool("hide_login_side_panel", hidden);
    REQUIRE(config.get_hide_login_side_panel() == hidden);
}

TEST_CASE("AppConfig maps region names to country codes", "[AppConfig][AppConfigPreferences][AppConfigRegion]")
{
    AppConfig config;
    const auto [region, country] = GENERATE(
        std::make_pair(std::string("CHN"), std::string("CN")),
        std::make_pair(std::string("China"), std::string("CN")),
        std::make_pair(std::string("USA"), std::string("US")),
        std::make_pair(std::string("Asia-Pacific"), std::string("Others")),
        std::make_pair(std::string("Europe"), std::string("US")),
        std::make_pair(std::string("North America"), std::string("US")),
        std::make_pair(std::string("Unknown"), std::string("Others")),
        std::make_pair(std::string(), std::string("Others")));
    config.set("region", region);
    REQUIRE(config.get_region() == region);
    REQUIRE(config.get_country_code() == country);
}

TEST_CASE("AppConfig identifies engineering environments", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    const auto [environment, engineering] = GENERATE(
        std::make_pair(std::string(ENV_DEV_HOST), true),
        std::make_pair(std::string(ENV_QAT_HOST), true),
        std::make_pair(std::string(ENV_PRE_HOST), true),
        std::make_pair(std::string(ENV_PRODUCT_HOST), false),
        std::make_pair(std::string("unknown"), false));
    config.set("iot_environment", environment);
    REQUIRE(config.is_engineering_region() == engineering);
}

TEST_CASE("AppConfig round-trips network update state", "[AppConfig][AppConfigPreferences]")
{
    AppConfig config;
    REQUIRE(config.get_network_plugin_version().empty());
    config.set_network_plugin_version("02.01.03.04");
    REQUIRE(config.get_network_plugin_version() == "02.01.03.04");

    const bool prompts_disabled = GENERATE(false, true);
    config.set_network_update_prompt_disabled(prompts_disabled);
    REQUIRE(config.is_network_update_prompt_disabled() == prompts_disabled);

    config.set_remind_network_update_later(true);
    REQUIRE(config.should_remind_network_update_later());
    config.clear_remind_network_update_later();
    REQUIRE_FALSE(config.should_remind_network_update_later());
}

TEST_CASE("AppConfig resolves endpoint overrides and config existence", "[AppConfig][AppConfigPreferences]")
{
    ScopedAppConfigDataDir scoped_dir;
    AppConfig config;

    REQUIRE_FALSE(config.exists());
    REQUIRE(config.version_check_url() == "https://check-version.orcaslicer.com/latest");
    REQUIRE(config.profile_update_url() == "https://check-version.orcaslicer.com/profile");

    config.set("version_check_url", "https://example.invalid/version");
    config.set("orca_updater_url", "https://example.invalid/profile");
    REQUIRE(config.version_check_url() == "https://example.invalid/version");
    REQUIRE(config.profile_update_url() == "https://example.invalid/profile");

    config.save();
    REQUIRE(config.exists());
}
