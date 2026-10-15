#include <boost/filesystem/path.hpp>
#include <boost/filesystem/convenience.hpp>
#include <boost/filesystem/operations.hpp>
#include <ios>
#include "libslic3r/Thread.hpp"
#include <boost/filesystem.hpp>
#include <fstream>
#include <string>
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
