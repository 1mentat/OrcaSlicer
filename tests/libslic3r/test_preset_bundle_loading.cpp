#include <nlohmann/json.hpp>
#include "libslic3r_version.h"
#include <boost/nowide/fstream.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/format.hpp>
#include <string_view>
#include "libslic3r/Semver.hpp"
#include <ios>

#include <memory>
#include <optional>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/directory.hpp>
#include <boost/filesystem/file_status.hpp>
#include <catch2/catch_all.hpp>

#include <algorithm>
#include <array>
#include <boost/filesystem.hpp>
#include <cstddef>
#include <fstream>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/generators/catch_generators.hpp>
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/ParallelResolve.hpp"
#include "libslic3r/AppConfig.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/miniz_extension.hpp"

#include "test_utils.hpp"

#include <iostream>
#include <initializer_list>
#include <vector>
#include <string>
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Preset.hpp"
#include <utility>
#include "libslic3r/Config.hpp"
#include <map>
#include "libslic3r/libslic3r.h"
#include "libslic3r/PublishSettings.hpp"
#include <set>
#include "libslic3r/TriangleSelector.hpp"
#include <iterator>
#include <miniz.h>

#ifndef _WIN32
#include <unistd.h> // geteuid
#endif

using namespace Slic3r;

namespace {

namespace fs = boost::filesystem;

// Whether a key is listed in a vector of keys (published_keys / skipped_keys).
bool contains_key(const std::vector<std::string> &keys, const std::string &key)
{
    return std::find(keys.begin(), keys.end(), key) != keys.end();
}

void check_double_vector(const std::vector<double> &actual, std::initializer_list<double> expected)
{
    REQUIRE(actual.size() == expected.size());
    size_t index = 0;
    for (double value : expected)
        REQUIRE_THAT(actual[index++], Catch::Matchers::WithinAbs(value, 1e-6));
}

void write_print_preset(const DynamicPrintConfig &default_config, const fs::path &file, const std::string &name, const std::string &inherits = {})
{
    DynamicPrintConfig config(default_config);
    config.option<ConfigOptionString>("print_settings_id", true)->value = name;
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;

    fs::create_directories(file.parent_path());
    config.save_to_json(file.string(), name, "User", "1.0.0");
}

// Write a preset json carrying a name and an "inherits" value, using the given collection's
// default config so it loads back into that collection. Works for any preset type.
void write_preset_with_inherits(const DynamicPrintConfig &default_config, const fs::path &file,
                                const std::string &name, const std::string &inherits)
{
    DynamicPrintConfig config(default_config);
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;

    fs::create_directories(file.parent_path());
    config.save_to_json(file.string(), name, "User", "1.0.0");
}

// A user preset file stating nothing but the preset it inherits, so every value it
// ends up with came from resolving that parent.
void write_minimal_child(const fs::path &file, const std::string &name, const std::string &inherits,
                         const std::string &type = "process")
{
    fs::create_directories(file.parent_path());
    std::ofstream(file.string())
        << R"({"type":")" << type << R"(","name":")" << name << R"(","from":"User","version":"1.0.0","inherits":")"
        << inherits << R"("})";
}

// Add an in-memory preset (no file) with the given inherits value (empty => root preset).
Preset &add_inmemory_preset(PresetCollection &coll, const std::string &name, const std::string &inherits = {})
{
    DynamicPrintConfig config(coll.default_preset().config);
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;
    return coll.load_preset(std::string(), name, config, /*select=*/false);
}

// Mark an already-loaded preset as renamed from one or more former names.
void set_renamed_from(PresetCollection &coll, const std::string &preset_name, std::vector<std::string> old_names)
{
    for (auto it = coll.begin(); it != coll.end(); ++it)
        if (it->name == preset_name)
            it->renamed_from = std::move(old_names);
}

// A single-slot PLA file config for the published-material load tests (mirrors what the GUI
// builds from a 3mf's project settings before load_config_model).
DynamicPrintConfig published_pla_file_config()
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
    config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99" };
    config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9 };
    return config;
}

// A standalone print preset collection that exposes the protected rename-map builder, so a
// renamed_from scenario can be set up without the full system-profile load pipeline.
// (PresetCollection is non-copyable - it holds a mutex - so it is constructed directly with
// the same type/keys/defaults PresetBundle uses for its print collection.)
struct RenameTestCollection : public PresetCollection
{
    RenameTestCollection()
        : PresetCollection(Preset::TYPE_PRINT, Preset::print_options(),
                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {}
    using PresetCollection::update_map_system_profile_renamed;
};

} // namespace

TEST_CASE("Preset identity is canonicalized from load path", "[Preset][Identity]")
{
    ScopedTemporaryDir         temp_dir;
    PresetBundle               bundle;
    PresetsConfigSubstitutions substitutions;

    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_PRINT_NAME / "User.json", "User");
    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_LOCAL_DIR / "bundle-1" / PRESET_PRINT_NAME / "LocalBundle.json", "LocalBundle");
    write_print_preset(bundle.prints.default_preset().config, temp_dir.path() / PRESET_SUBSCRIBED_DIR / "remote-1" / PRESET_PRINT_NAME / "Subscribed.json", "Subscribed");

    bundle.prints.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.load_presets((temp_dir.path() / PRESET_LOCAL_DIR / "bundle-1").string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    bundle.prints.load_presets((temp_dir.path() / PRESET_SUBSCRIBED_DIR / "remote-1").string(), PRESET_PRINT_NAME, substitutions, ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *root_user = bundle.prints.find_preset("User");
    REQUIRE(root_user != nullptr);
    CHECK(root_user->name == "User");
    CHECK_FALSE(root_user->is_from_bundle());

    const Preset *local_bundle = bundle.prints.find_preset("_local/bundle-1/LocalBundle");
    REQUIRE(local_bundle != nullptr);
    CHECK(local_bundle->name == "_local/bundle-1/LocalBundle");
    CHECK(local_bundle->is_from_bundle());

    const Preset *subscribed = bundle.prints.find_preset("_subscribed/remote-1/Subscribed");
    REQUIRE(subscribed != nullptr);
    CHECK(subscribed->name == "_subscribed/remote-1/Subscribed");
    CHECK(subscribed->is_from_bundle());
}

TEST_CASE("Legacy bundle import without bundle metadata stays in the user preset directory", "[Preset][Identity]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle  bundle;

    PresetsConfigSubstitutions substitutions;
    std::vector<std::string>   result;
    int                        overwrite = 0;
    std::string                file      = (temp_dir.path() / "legacy-bundle" / "Imported.json").string();
    const fs::path             user_root = temp_dir.path() / "user";

    write_print_preset(bundle.prints.default_preset().config, file, "Imported");
    fs::create_directories(user_root);
    bundle.prints.update_user_presets_directory(user_root.string(), PRESET_PRINT_NAME);

    REQUIRE(bundle.import_json_presets(
        substitutions,
        file,
        [](std::string const &) { return 1; },
        ForwardCompatibilitySubstitutionRule::Disable,
        overwrite,
        result));

    const Preset *imported = bundle.prints.find_preset("Imported");
    REQUIRE(imported != nullptr);
    CHECK(imported->name == "Imported");
    CHECK(imported->bundle_id.empty());
    CHECK_FALSE(imported->is_from_bundle());
    // Detached user presets (no inherits) are saved in the "base" subfolder of the user preset root.
    CHECK(fs::equivalent(fs::path(imported->file).parent_path().parent_path(), user_root / PRESET_PRINT_NAME));
}

TEST_CASE("Current vendor type tolerates missing printer model", "[Preset][Bundle]")
{
    PresetBundle bundle;

    VendorProfile orca_vendor; orca_vendor.id = "ORCA";
    VendorProfile::PrinterModel model;
    model.name = "Orca Test";
    orca_vendor.models.emplace_back(model);
    bundle.vendors.emplace("ORCA", std::move(orca_vendor));

    bundle.printers.get_edited_preset().config.erase("printer_model");

    CHECK(bundle.get_current_vendor_type() == VendorType::Unknown);
}

TEST_CASE("A malformed entry in a vendor's preset list is counted, not thrown", "[Preset][Bundle]")
{
    ScopedTemporaryDir dir;

    // A bare number where the list wants an object. An array element has no key,
    // so reporting one as if it did throws nlohmann's invalid_iterator - which is
    // not a parse_error, and escapes the catch around the vendor profile parse.
    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[123,)"
        << R"({"name":"0.20mm Standard @Acme","sub_path":"process/standard.json"}]})";
    fs::create_directories(dir.path() / "Acme" / "process");
    std::ofstream((dir.path() / "Acme" / "process" / "standard.json").string())
        << R"({"type":"process","name":"0.20mm Standard @Acme","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    PresetBundle bundle;
    size_t       loaded = 0;
    REQUIRE_NOTHROW(loaded = bundle.load_vendor_configs_from_json(
                        dir.path().string(), "Acme", PresetBundle::LoadSystem,
                        ForwardCompatibilitySubstitutionRule::EnableSilent).second);

    CHECK(bundle.error_count() > 0);   // the malformed element was counted
    CHECK(loaded == 1);                // the well-formed one beside it still loaded
}

TEST_CASE("Printer extruder count tolerates missing nozzle diameter", "[Preset][Bundle]")
{
    PresetBundle bundle;
    DynamicPrintConfig& config = bundle.printers.get_edited_preset().config;

    config.erase("nozzle_diameter");
    CHECK(bundle.get_printer_extruder_count() == 1);

    config.set_key_value("nozzle_diameter", new ConfigOptionFloats());
    CHECK(bundle.get_printer_extruder_count() == 1);

    config.set_key_value("nozzle_diameter", new ConfigOptionFloats({ 0.4, 0.6 }));
    CHECK(bundle.get_printer_extruder_count() == 2);
}

TEST_CASE("Selected printer uses its default or saved bed type", "[Preset][Bundle]")
{
    PresetBundle bundle;
    Preset& printer = add_inmemory_preset(bundle.printers, "Test Printer");
    printer.is_system = true;
    printer.config.option<ConfigOptionString>("printer_model")->value = "TEST-MODEL";
    printer.config.option<ConfigOptionString>("printer_variant")->value = "0.4";
    printer.config.option<ConfigOptionString>("default_bed_type")->value = "Engineering Plate";

    AppConfig app_config;
    app_config.set("curr_bed_type", std::to_string(static_cast<int>(btPTE)));
    PresetBundle::PresetPreferences preferred_selection;
    BedType expected_bed_type;

    SECTION("New printer uses its symbolic default") {
        expected_bed_type = btEP;
        preferred_selection = {"TEST-MODEL", "0.4"};
    }
    SECTION("Re-enabled printer uses its saved selection") {
        expected_bed_type = btPC;
        preferred_selection = {"TEST-MODEL", "0.4"};
        app_config.set_printer_setting("Test Printer", "curr_bed_type",
                                       std::to_string(static_cast<int>(expected_bed_type)));
    }
    SECTION("Existing printer keeps its saved selection after presets reload") {
        expected_bed_type = btPCT;
        app_config.set("presets", PRESET_PRINTER_NAME, "Test Printer");
        app_config.set_printer_setting("Test Printer", "curr_bed_type",
                                       std::to_string(static_cast<int>(expected_bed_type)));
    }

    bundle.load_selections(app_config, preferred_selection);
    bundle.export_selections(app_config);

    CHECK(bundle.project_config.opt_enum<BedType>("curr_bed_type") == expected_bed_type);
    CHECK(app_config.get_printer_setting("Test Printer", "curr_bed_type") == std::to_string(static_cast<int>(expected_bed_type)));
}

TEST_CASE("A directory of user presets loads with each one resolved against its parent", "[Preset][Bundle]")
{
    ScopedTemporaryDir   temp_dir;
    RenameTestCollection coll;

    Preset &parent = add_inmemory_preset(coll, "Parent Process");
    parent.config.option<ConfigOptionFloat>("layer_height", true)->value = 0.24;
    parent.is_system = true;

    constexpr int children = 400;
    for (int i = 0; i < children; ++ i)
        write_minimal_child(temp_dir.path() / PRESET_PRINT_NAME / ("Child " + std::to_string(i) + ".json"),
                            "Child " + std::to_string(i), "Parent Process");

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable);

    CHECK(coll.size() == size_t(children) + 2); // the children, the default preset and the parent
    CHECK(coll.error_count() == 0);
    for (int i = 0; i < children; ++ i) {
        const Preset *child = coll.find_preset("Child " + std::to_string(i));
        REQUIRE(child != nullptr);
        CHECK(child->inherits() == "Parent Process");
        CHECK(child->alias == "Child " + std::to_string(i));
        CHECK(child->loaded);
        REQUIRE(child->config.option<ConfigOptionFloat>("layer_height") != nullptr);
        CHECK_THAT(child->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    }
}

TEST_CASE("Repeated loads of a user preset directory produce the same presets", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;

    auto seed_directory = [&]() {
        for (int i = 0; i < 200; ++ i)
            write_minimal_child(temp_dir.path() / PRESET_PRINT_NAME / ("Child " + std::to_string(i) + ".json"),
                                "Child " + std::to_string(i), "Parent Process");
    };

    std::vector<std::vector<std::string>> names_per_run;
    for (int run = 0; run < 3; ++ run) {
        RenameTestCollection coll;
        Preset &parent = add_inmemory_preset(coll, "Parent Process");
        parent.is_system = true;
        if (run == 0)
            seed_directory();

        PresetsConfigSubstitutions substitutions;
        coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                          ForwardCompatibilitySubstitutionRule::Disable);

        std::vector<std::string> names;
        for (auto it = coll.begin(); it != coll.end(); ++ it)
            names.push_back(it->name + "|" + it->alias + "|" + it->inherits());
        names_per_run.push_back(std::move(names));
    }

    REQUIRE(names_per_run[0].size() > 200);
    CHECK(names_per_run[1] == names_per_run[0]);
    CHECK(names_per_run[2] == names_per_run[0]);
}

TEST_CASE("An unreadable user preset is counted and removed while the rest still load", "[Preset][Bundle]")
{
    ScopedTemporaryDir   temp_dir;
    RenameTestCollection coll;
    const fs::path       dir = temp_dir.path() / PRESET_PRINT_NAME;

    for (int i = 0; i < 20; ++ i)
        write_preset_with_inherits(coll.default_preset().config, dir / ("Good " + std::to_string(i) + ".json"),
                                   "Good " + std::to_string(i), std::string());
    fs::create_directories(dir);
    std::ofstream((dir / "Broken.json").string()) << "{not-json";

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::EnableSilent);

    CHECK(coll.error_count() == 1);
    CHECK(coll.find_preset("Broken") == nullptr);
    CHECK_FALSE(fs::exists(dir / "Broken.json"));
    for (int i = 0; i < 20; ++ i)
        CHECK(coll.find_preset("Good " + std::to_string(i)) != nullptr);
}

TEST_CASE("A user filament naming no compatible printer gets the one after its @, in memory and on disk", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle       bundle;
    const fs::path     file = temp_dir.path() / PRESET_FILAMENT_NAME / "My PLA @Test Printer.json";
    REQUIRE(bundle.filaments.default_preset().config.option<ConfigOptionStrings>("compatible_printers")->values.empty());
    write_preset_with_inherits(bundle.filaments.default_preset().config, file, "My PLA @Test Printer", std::string());

    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::EnableSilent);

    const std::vector<std::string> expected { "Test Printer" };
    const Preset *preset = bundle.filaments.find_preset("My PLA @Test Printer");
    REQUIRE(preset != nullptr);
    CHECK(preset->config.option<ConfigOptionStrings>("compatible_printers")->values == expected);

    DynamicPrintConfig                 saved;
    std::map<std::string, std::string> key_values;
    std::string                        reason;
    saved.load_from_json(file.string(), ForwardCompatibilitySubstitutionRule::EnableSilent, key_values, reason);
    REQUIRE(reason.empty());
    REQUIRE(saved.option<ConfigOptionStrings>("compatible_printers") != nullptr);
    CHECK(saved.option<ConfigOptionStrings>("compatible_printers")->values == expected);
}

TEST_CASE("A user preset's setting id equal to its base id is dropped in memory, not in the .info written back", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle       bundle;
    const fs::path     file = temp_dir.path() / PRESET_FILAMENT_NAME / "My PLA @Test Printer.json";
    write_preset_with_inherits(bundle.filaments.default_preset().config, file, "My PLA @Test Printer", std::string());
    fs::path info = file;
    info.replace_extension(".info");
    std::ofstream(info.string()) << "sync_info = \nuser_id = \nsetting_id = PFUS1\nbase_id = PFUS1\nupdated_time = 0\n";

    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::EnableSilent);

    const Preset *preset = bundle.filaments.find_preset("My PLA @Test Printer");
    REQUIRE(preset != nullptr);
    CHECK(preset->setting_id.empty());
    CHECK(preset->base_id == "PFUS1");
    Preset reloaded(Preset::TYPE_FILAMENT, "My PLA @Test Printer");
    reloaded.load_info(info.string());
    CHECK(reloaded.setting_id == "PFUS1");
}

TEST_CASE("A user preset that is not loaded still reports its substituted values", "[Preset][Bundle]")
{
    ScopedTemporaryDir   temp_dir;
    RenameTestCollection coll;
    const fs::path       dir = temp_dir.path() / PRESET_PRINT_NAME;
    fs::create_directories(dir);
    std::ofstream((dir / "Orphan.json").string())
        << R"({"type":"process","name":"Orphan","from":"User","version":"1.0.0","inherits":"No Such Parent",)"
        << R"("wall_generator":"no_such_generator"})";

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Enable);

    CHECK(coll.find_preset("Orphan") == nullptr);
    CHECK(coll.error_count() == 1);
    REQUIRE(substitutions.size() == 1);
    CHECK(substitutions.front().preset_name == "Orphan");
}

TEST_CASE("find_preset resolves a system preset's renamed_from", "[Preset][Rename]")
{
    RenameTestCollection coll;

    // "New Process" is the current preset; it was renamed from "Old Process".
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Old Process" });
    coll.update_map_system_profile_renamed();

    // The rename map knows the old name...
    const std::string *renamed = coll.get_preset_name_renamed("Old Process");
    REQUIRE(renamed != nullptr);
    CHECK(*renamed == "New Process");

    // ...and plain find_preset() now follows it (the core of this PR; previously this
    // resolution lived only in find_preset2 and a few call sites).
    const Preset *resolved = coll.find_preset("Old Process");
    REQUIRE(resolved != nullptr);
    CHECK(resolved->name == "New Process");

    // A genuinely unknown name still returns null (no spurious match).
    CHECK(coll.find_preset("Totally Unknown") == nullptr);

    // A child that still inherits the OLD name resolves through the runtime walker,
    // which uses plain find_preset().
    Preset       &child  = add_inmemory_preset(coll, "Child Process", "Old Process");
    const Preset *parent = coll.get_preset_parent(child);
    REQUIRE(parent != nullptr);
    CHECK(parent->name == "New Process");
}

TEST_CASE("find_preset resolves a preset renamed more than once", "[Preset][Rename]")
{
    RenameTestCollection coll;

    // "New Process" was renamed twice, so it carries both former names in renamed_from.
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Original Process", "Old Process" });
    coll.update_map_system_profile_renamed();

    // Each historical name resolves to the current preset.
    for (const char *old_name : { "Original Process", "Old Process" }) {
        INFO("resolving old name: " << old_name);
        const std::string *renamed = coll.get_preset_name_renamed(old_name);
        REQUIRE(renamed != nullptr);
        CHECK(*renamed == "New Process");

        const Preset *resolved = coll.find_preset(old_name);
        REQUIRE(resolved != nullptr);
        CHECK(resolved->name == "New Process");
    }

    // A child inheriting either former name resolves through the runtime walker.
    Preset &child = add_inmemory_preset(coll, "Child Process", "Original Process");
    REQUIRE(coll.get_preset_parent(child) != nullptr);
    CHECK(coll.get_preset_parent(child)->name == "New Process");
}

TEST_CASE("find_preset2 auto-matches removed Generic vendor profiles to the library", "[Preset][Rename]")
{
    PresetBundle bundle;

    // The OrcaFilamentLibrary replacement that removed empty "<vendor> Generic" profiles map to.
    add_inmemory_preset(bundle.filaments, "Generic PLA @System");

    // Plain lookups do NOT fuzzy-match a removed vendor profile.
    CHECK(bundle.filaments.find_preset("Voron Generic PLA") == nullptr);
    CHECK(bundle.filaments.find_preset2("Voron Generic PLA", /*auto_match=*/false) == nullptr);

    // With auto_match, the removed "Voron Generic PLA" resolves to "Generic PLA @System".
    const Preset *matched = bundle.filaments.find_preset2("Voron Generic PLA", /*auto_match=*/true);
    REQUIRE(matched != nullptr);
    CHECK(matched->name == "Generic PLA @System");

    // No library preset exists for an unrelated material => still no match.
    CHECK(bundle.filaments.find_preset2("BrandX Generic PETG", /*auto_match=*/true) == nullptr);
}

TEST_CASE("Renamed parent is normalized into a loaded preset's inherits", "[Preset][Rename]")
{
    ScopedTemporaryDir   temp_dir;
    RenameTestCollection coll;

    // Current parent, renamed from "Old Process".
    add_inmemory_preset(coll, "New Process");
    set_renamed_from(coll, "New Process", { "Old Process" });
    coll.update_map_system_profile_renamed();

    // A user preset on disk that still inherits the OLD name.
    write_preset_with_inherits(coll.default_preset().config,
                               temp_dir.path() / PRESET_PRINT_NAME / "Child.json", "Child", "Old Process");

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *child = coll.find_preset("Child");
    REQUIRE(child != nullptr);
    // The dangling "Old Process" was rewritten to the resolved parent name at load time,
    // so the runtime walker (plain find_preset) can resolve the chain.
    CHECK(child->inherits() == "New Process");
    REQUIRE(coll.get_preset_parent(*child) != nullptr);
    CHECK(coll.get_preset_parent(*child)->name == "New Process");
}

TEST_CASE("Removed Generic parent is normalized into a loaded filament's inherits", "[Preset][Rename]")
{
    ScopedTemporaryDir temp_dir;
    PresetBundle  bundle;

    add_inmemory_preset(bundle.filaments, "Generic PLA @System");

    // A user filament that still inherits a removed "<vendor> Generic PLA" profile.
    write_preset_with_inherits(bundle.filaments.default_preset().config,
                               temp_dir.path() / PRESET_FILAMENT_NAME / "MyPLA.json", "MyPLA", "Voron Generic PLA");

    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *child = bundle.filaments.find_preset("MyPLA");
    REQUIRE(child != nullptr);
    CHECK(child->inherits() == "Generic PLA @System");
    REQUIRE(bundle.filaments.get_preset_parent(*child) != nullptr);
    CHECK(bundle.filaments.get_preset_parent(*child)->name == "Generic PLA @System");
}

namespace {

// A live reference to a preset's compatible_printers / compatible_prints list. Fetches the *stored*
// preset (real=true) so writes and reads hit the same object; creates the option if absent.
std::vector<std::string> &compatible_list(PresetCollection &coll, const std::string &preset_name, const char *field_key)
{
    Preset *preset = coll.find_preset(preset_name, /*first_visible_if_not_found=*/false, /*real=*/true);
    REQUIRE(preset != nullptr);
    return preset->config.option<ConfigOptionStrings>(field_key, true)->values;
}

} // namespace

TEST_CASE("Renamed printer/process names are normalized into compatible lists on load", "[Preset][Rename]")
{
    PresetBundle bundle;

    // Current printer + process, each renamed from an older name.
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });
    add_inmemory_preset(bundle.prints, "New Process");
    set_renamed_from(bundle.prints, "New Process", { "Old Process" });

    // A user process still compatible with the OLD printer name.
    add_inmemory_preset(bundle.prints, "My Process");
    compatible_list(bundle.prints, "My Process", "compatible_printers") = { "Old Printer" };

    // A user filament referencing the OLD printer AND OLD process names, plus an unknown printer.
    add_inmemory_preset(bundle.filaments, "My Filament");
    compatible_list(bundle.filaments, "My Filament", "compatible_printers") = { "Old Printer", "Unknown Printer" };
    compatible_list(bundle.filaments, "My Filament", "compatible_prints")   = { "Old Process" };

    // Build the rename maps (done during system load in the real pipeline), then normalize.
    AppConfig app_config;
    bundle.load_installed_printers(app_config); // rebuilds every collection's rename map
    bundle.normalize_compatible_presets();

    // The stale printer name in a process' compatible_printers is rewritten to the current name.
    CHECK(compatible_list(bundle.prints, "My Process", "compatible_printers") == std::vector<std::string>{ "New Printer" });

    // The stale process name in a filament's compatible_prints is rewritten (this field has no
    // runtime rename fallback, so load-time normalization is the only fix).
    CHECK(compatible_list(bundle.filaments, "My Filament", "compatible_prints") == std::vector<std::string>{ "New Process" });

    // The renamed printer is rewritten while the unknown/deleted name is preserved as-is.
    CHECK(compatible_list(bundle.filaments, "My Filament", "compatible_printers") ==
          (std::vector<std::string>{ "New Printer", "Unknown Printer" }));

    // Normalizing rewrites config in place without flagging the preset dirty.
    CHECK_FALSE(bundle.prints.find_preset("My Process", false, true)->is_dirty);

    // A system preset that already references the current name is left untouched (idempotent no-op).
    bundle.normalize_compatible_presets();
    CHECK(compatible_list(bundle.prints, "My Process", "compatible_printers") == std::vector<std::string>{ "New Printer" });
}

TEST_CASE("Renamed names are normalized into a SYSTEM preset's compatible lists", "[Preset][Rename]")
{
    PresetBundle bundle;

    // Current printer + process, each renamed from an older name.
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });
    add_inmemory_preset(bundle.prints, "New Process");
    set_renamed_from(bundle.prints, "New Process", { "Old Process" });

    // A *system* (vendor) filament whose own compatible lists still reference the OLD names. A vendor
    // profile can point at a sibling preset that was later renamed, so system presets must be
    // normalized too (they are skipped by neither collection walk).
    add_inmemory_preset(bundle.filaments, "System Filament").is_system = true;
    compatible_list(bundle.filaments, "System Filament", "compatible_printers") = { "Old Printer" };
    compatible_list(bundle.filaments, "System Filament", "compatible_prints")   = { "Old Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config); // build the rename maps
    bundle.normalize_compatible_presets();

    // The stale references in the system preset are rewritten to the current names.
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_printers") ==
          std::vector<std::string>{ "New Printer" });
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_prints") ==
          std::vector<std::string>{ "New Process" });

    // The rewrite does not flag the system preset dirty, and is idempotent.
    CHECK_FALSE(bundle.filaments.find_preset("System Filament", false, true)->is_dirty);
    bundle.normalize_compatible_presets();
    CHECK(compatible_list(bundle.filaments, "System Filament", "compatible_printers") ==
          std::vector<std::string>{ "New Printer" });
}

TEST_CASE("compatible_prints on SLA materials resolves against sla_prints, not prints", "[Preset][Rename]")
{
    PresetBundle bundle;

    // A renamed SLA process, and a same-named FFF process that must NOT be picked up: resolving the
    // SLA material's compatible_prints against `prints` would wrongly rewrite to "Wrong FFF Process".
    add_inmemory_preset(bundle.sla_prints, "New SLA Process");
    set_renamed_from(bundle.sla_prints, "New SLA Process", { "Old SLA Process" });
    add_inmemory_preset(bundle.prints, "Wrong FFF Process");
    set_renamed_from(bundle.prints, "Wrong FFF Process", { "Old SLA Process" });

    add_inmemory_preset(bundle.sla_materials, "My SLA Material");
    compatible_list(bundle.sla_materials, "My SLA Material", "compatible_prints") = { "Old SLA Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config);
    bundle.normalize_compatible_presets();

    CHECK(compatible_list(bundle.sla_materials, "My SLA Material", "compatible_prints") ==
          std::vector<std::string>{ "New SLA Process" });
}

TEST_CASE("Profile validator flags dangling and renamed preset references", "[Preset][Validate]")
{
    PresetBundle bundle;

    // Current printers: a real one, and a renamed one (its old name resolves via renamed_from).
    add_inmemory_preset(bundle.printers, "Real Printer");
    add_inmemory_preset(bundle.printers, "New Printer");
    set_renamed_from(bundle.printers, "New Printer", { "Old Printer" });

    // A real process, referenced from a filament's compatible_prints.
    add_inmemory_preset(bundle.prints, "Real Process").is_system = true;

    // A fully valid system filament: references only current names.
    add_inmemory_preset(bundle.filaments, "Good Filament").is_system = true;
    compatible_list(bundle.filaments, "Good Filament", "compatible_printers") = { "Real Printer" };
    compatible_list(bundle.filaments, "Good Filament", "compatible_prints")   = { "Real Process" };

    AppConfig app_config;
    bundle.load_installed_printers(app_config); // build the rename maps

    // With only valid references, the validator is clean.
    CHECK_FALSE(bundle.check_preset_references());

    SECTION("deleted compatible_printers is flagged") {
        add_inmemory_preset(bundle.filaments, "Ghost Ref Filament").is_system = true;
        compatible_list(bundle.filaments, "Ghost Ref Filament", "compatible_printers") = { "Ghost Printer" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("renamed compatible_printers (old name) is flagged") {
        add_inmemory_preset(bundle.filaments, "Old Ref Filament").is_system = true;
        compatible_list(bundle.filaments, "Old Ref Filament", "compatible_printers") = { "Old Printer" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("deleted compatible_prints is flagged") {
        add_inmemory_preset(bundle.filaments, "Bad Process Ref").is_system = true;
        compatible_list(bundle.filaments, "Bad Process Ref", "compatible_prints") = { "Ghost Process" };
        CHECK(bundle.check_preset_references());
    }

    SECTION("deleted inherits parent is flagged") {
        add_inmemory_preset(bundle.filaments, "Orphan Filament", "Ghost Parent").is_system = true;
        CHECK(bundle.check_preset_references());
    }

    SECTION("non-system preset with a dangling reference is ignored") {
        add_inmemory_preset(bundle.filaments, "User Filament"); // is_system stays false
        compatible_list(bundle.filaments, "User Filament", "compatible_printers") = { "Ghost Printer" };
        CHECK_FALSE(bundle.check_preset_references());
    }
}

TEST_CASE("Every printer variant has a compatible default material", "[Preset][Validate][DefaultMaterials]")
{
    PresetBundle bundle;
    auto &vendor = bundle.vendors["Acme"];
    vendor.id = vendor.name = "Acme";
    vendor.models.emplace_back();
    auto &model = vendor.models.back();
    model.id = model.name = "Acme Printer";
    model.default_materials = {"Acme PLA @0.4", "Acme PLA @0.6"};

    for (const std::string variant : {"0.4", "0.6"}) {
        model.variants.emplace_back(variant);
        const std::string printer_name = "Acme Printer " + variant;
        Preset &printer = add_inmemory_preset(bundle.printers, printer_name);
        printer.is_system = true;
        printer.is_visible = false; // Validation covers uninstalled variants too.
        printer.vendor = &vendor;
        printer.config.option<ConfigOptionString>("printer_model")->value = model.id;
        printer.config.option<ConfigOptionString>("printer_variant")->value = variant;
        printer.config.option<ConfigOptionFloats>("nozzle_diameter")->values = {std::stod(variant)};

        Preset &filament = add_inmemory_preset(bundle.filaments, "Acme PLA @" + variant);
        filament.is_system = true;
        filament.vendor = &vendor;
        filament.alias = "Acme PLA";
        filament.config.option<ConfigOptionStrings>("compatible_printers")->values = {printer_name};
    }

    // A second system filament, compatible with a user printer this model does not have, so a
    // section can list a known-but-incompatible name without tripping the existence check.
    Preset &other_printer = add_inmemory_preset(bundle.printers, "Acme Printer 0.2");
    other_printer.vendor = &vendor;
    Preset &other_filament = add_inmemory_preset(bundle.filaments, "Acme PLA @0.2");
    other_filament.is_system = true;
    other_filament.vendor = &vendor;
    other_filament.alias = "Acme PLA";
    other_filament.config.option<ConfigOptionStrings>("compatible_printers")->values = {"Acme Printer 0.2"};

    CHECK_FALSE(bundle.has_errors());
    bool expected_errors = true;

    SECTION("A model default for one nozzle does not cover another nozzle") {
        model.default_materials = {"Acme PLA @0.4"};
    }
    SECTION("An empty default list leaves every variant uncovered") {
        model.default_materials.clear();
    }
    SECTION("An unknown filament cannot be a default") {
        model.default_materials = {"Missing PLA"};
    }
    SECTION("An unknown name is an error even when a compatible default covers the variant") {
        model.default_materials.insert(model.default_materials.begin(), "Missing PLA");
    }
    SECTION("An unknown default_filament_profile name is an error") {
        bundle.printers.find_preset("Acme Printer 0.6", false, true)
            ->config.option<ConfigOptionStrings>("default_filament_profile", true)->values = {"Missing PLA"};
    }
    SECTION("A known default_filament_profile name is not an error") {
        bundle.printers.find_preset("Acme Printer 0.6", false, true)
            ->config.option<ConfigOptionStrings>("default_filament_profile", true)->values = {"Acme PLA @0.6"};
        expected_errors = false;
    }
    SECTION("A short alias does not resolve as an installed default") {
        model.default_materials = {"Acme PLA"};
    }
    SECTION("A user filament cannot satisfy a shipped default") {
        bundle.filaments.find_preset("Acme PLA @0.6", false, true)->is_system = false;
    }
    SECTION("One compatible default per variant is sufficient") {
        model.default_materials.insert(model.default_materials.begin(), "Acme PLA @0.2");
        expected_errors = false;
    }
    SECTION("Compatibility conditions apply to each nozzle") {
        model.default_materials = {"Acme PLA @0.4"};
        Preset *filament = bundle.filaments.find_preset("Acme PLA @0.4", false, true);
        auto &library = bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
        library.id = library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
        filament->vendor = &library;
        filament->config.option<ConfigOptionStrings>("compatible_printers")->values.clear();
        filament->config.option<ConfigOptionString>("compatible_printers_condition")->value = "nozzle_diameter[0] == 0.4";
    }
    SECTION("Library defaults respect printer exclusions") {
        model.default_materials = {"Acme PLA @0.4"};
        Preset *filament = bundle.filaments.find_preset("Acme PLA @0.4", false, true);
        auto &library = bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
        library.id = library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
        filament->vendor = &library;
        filament->config.option<ConfigOptionStrings>("compatible_printers")->values.clear();
        CHECK_FALSE(bundle.check_printer_default_materials());
        filament->m_excluded_from.insert("Acme Printer 0.6");
    }
    SECTION("User printers do not need model defaults") {
        model.default_materials = {"Acme PLA @0.4"};
        bundle.printers.find_preset("Acme Printer 0.6", false, true)->is_system = false;
        expected_errors = false;
    }

    CHECK(bundle.check_printer_default_materials() == expected_errors);
    CHECK(bundle.has_errors() == expected_errors);
}

// Under a shared override key, the last preset merged into the full config overwrote the others', so an
// edited slicing-pipeline override never reached Print::apply's diff and re-configuring a plugin never
// re-sliced. Per-type keys make that collision impossible; guard the scoping here.
TEST_CASE("Plugin capability override keys are scoped per preset type", "[Preset][Plugin]")
{
    // Pin the key names: presets and 3mf files store them verbatim, so a rename is a format change.
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_PRINT)    == std::string("print_plugin_config_overrides"));
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_PRINTER)  == std::string("printer_plugin_config_overrides"));
    CHECK(Preset::plugin_overrides_key(Preset::TYPE_FILAMENT) == std::string("filament_plugin_config_overrides"));

    // ...and each key lives on exactly its own preset type's option list, so no two ever share a slot.
    const std::pair<Preset::Type, const std::vector<std::string>*> scopes[] = {
        {Preset::TYPE_PRINT,    &Preset::print_options()},
        {Preset::TYPE_PRINTER,  &Preset::printer_options()},
        {Preset::TYPE_FILAMENT, &Preset::filament_options()},
    };
    for (const auto &owner : scopes)
        for (const auto &scoped : scopes) {
            const std::string key = Preset::plugin_overrides_key(scoped.first);
            CAPTURE(owner.first, key);
            CHECK(contains(*owner.second, key) == (owner.first == scoped.first));
        }
}

namespace {

// A standalone filament collection that exposes the protected library masking builder, so the Orca
// Filament Library scenario can be set up without the full system-profile load pipeline.
struct LibraryFilamentTestCollection : public PresetCollection
{
    LibraryFilamentTestCollection()
        : PresetCollection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {}
    using PresetCollection::update_library_profile_excluded_from;
};

} // namespace

TEST_CASE("Missing app config is accepted as default CLI state", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    AppConfig app_config;
    app_config.set_loading_path((dir.path() / "missing.conf").string());
    CHECK(app_config.load_if_exists().empty());
}

TEST_CASE("Read-only user preset loading does not create or delete files", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    PresetBundle       bundle;
    PresetsConfigSubstitutions substitutions;

    const fs::path missing_root = dir.path() / "missing-user";
    bundle.prints.load_presets(missing_root.string(), PRESET_PRINT_NAME, substitutions,
                               ForwardCompatibilitySubstitutionRule::EnableSilent, nullptr,
                               PresetOrigin(), true);
    CHECK_FALSE(fs::exists(missing_root / PRESET_PRINT_NAME));

    const fs::path malformed = dir.path() / "existing-user" / PRESET_PRINT_NAME / "malformed.json";
    fs::create_directories(malformed.parent_path());
    std::ofstream(malformed.string()) << "{not-json";
    bundle.prints.load_presets((dir.path() / "existing-user").string(), PRESET_PRINT_NAME, substitutions,
                               ForwardCompatibilitySubstitutionRule::EnableSilent, nullptr,
                               PresetOrigin(), true);
    CHECK(fs::exists(malformed));
}

TEST_CASE("Typeless preset resolution probes loaded FFF collections", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "typeless-process.json";
    std::ofstream(source_file.string()) << R"({"name":"Typeless Process","from":"User"})";

    PresetBundle bundle;
    Preset      &process = add_inmemory_preset(bundle.prints, "Typeless Process");
    process.file = source_file.string();
    process.config.option<ConfigOptionFloats>("travel_speed", true)->values = {321.0};

    DynamicPrintConfig raw;
    Preset::Type       resolved_type = Preset::TYPE_INVALID;
    std::string        error;
    REQUIRE(bundle.resolve_preset_config_type(raw, resolved_type, source_file.string(),
                                              ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error.empty());
    CHECK(resolved_type == Preset::TYPE_PRINT);
    REQUIRE(raw.option<ConfigOptionFloats>("travel_speed")->values.size() == 1);
    CHECK_THAT(raw.option<ConfigOptionFloats>("travel_speed")->values.front(), Catch::Matchers::WithinAbs(321.0, 1e-6));
}

TEST_CASE("Typeless preset resolution preserves duplicate identity ambiguity", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "duplicate-process.json";
    std::ofstream(source_file.string()) << "{}";

    PresetBundle bundle;
    add_inmemory_preset(bundle.prints, "First Process Identity").file  = source_file.string();
    add_inmemory_preset(bundle.prints, "Second Process Identity").file = source_file.string();

    DynamicPrintConfig raw;
    Preset::Type       resolved_type = Preset::TYPE_INVALID;
    std::string        error;
    CHECK_FALSE(bundle.resolve_preset_config_type(raw, resolved_type, source_file.string(),
                                                  ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error == "Preset identity is ambiguous");
    CHECK(resolved_type == Preset::TYPE_INVALID);
}

TEST_CASE("Typeless preset resolution rejects cross-type ambiguity", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "ambiguous.json";
    std::ofstream(source_file.string()) << "{}";

    PresetBundle bundle;
    add_inmemory_preset(bundle.prints, "Process Identity").file       = source_file.string();
    add_inmemory_preset(bundle.filaments, "Filament Identity").file = source_file.string();

    DynamicPrintConfig raw;
    Preset::Type       resolved_type = Preset::TYPE_INVALID;
    std::string        error;
    CHECK_FALSE(bundle.resolve_preset_config_type(raw, resolved_type, source_file.string(),
                                                  ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error == "Preset type is ambiguous");
    CHECK(resolved_type == Preset::TYPE_INVALID);
}

TEST_CASE("Typeless preset resolution rejects a missing type candidate", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "unknown.json";
    std::ofstream(source_file.string()) << "{}";

    PresetBundle       bundle;
    DynamicPrintConfig raw;
    Preset::Type       resolved_type = Preset::TYPE_INVALID;
    std::string        error;
    CHECK_FALSE(bundle.resolve_preset_config_type(raw, resolved_type, source_file.string(),
                                                  ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error == "Preset type could not be resolved");
    CHECK(resolved_type == Preset::TYPE_INVALID);
}

TEST_CASE("Exact file resolution rejects multiple preset identities", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "duplicate.json";
    std::ofstream(source_file.string()) << "{}";

    PresetBundle bundle;
    Preset &first = add_inmemory_preset(bundle.prints, "First Identity");
    first.file = source_file.string();
    Preset &second = add_inmemory_preset(bundle.prints, "Second Identity");
    second.file = source_file.string();

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "Parent";

    std::string error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, source_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error == "Preset identity is ambiguous");
}

TEST_CASE("System preset resolution returns the canonical vendor configuration", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir source_dir;
    PresetBundle bundle;

    VendorProfile vendor("VendorB");
    vendor.name = "Vendor B";
    auto [vendor_it, inserted] = bundle.vendors.emplace(vendor.id, std::move(vendor));
    REQUIRE(inserted);

    Preset &resolved = add_inmemory_preset(bundle.prints, "Vendor B Process", "fdm_process_common");
    resolved.is_system = true;
    resolved.vendor    = &vendor_it->second;
    resolved.file      = (source_dir.path() / "vendor-b-process.json").string();
    std::ofstream(resolved.file) << "{}";
    resolved.config.option<ConfigOptionFloats>("travel_speed", true)->values = {321.0};
    resolved.config.option<ConfigOptionInt>("wall_loops", true)->value       = 2;

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";
    raw.option<ConfigOptionInt>("wall_loops", true)->value             = 5;

    std::string error;
    REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, resolved.file,
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error.empty());
    REQUIRE(raw.option<ConfigOptionFloats>("travel_speed")->values.size() == 1);
    CHECK_THAT(raw.option<ConfigOptionFloats>("travel_speed")->values.front(), Catch::Matchers::WithinAbs(321.0, 1e-6));
    CHECK(raw.option<ConfigOptionInt>("wall_loops")->value == 2);
}

TEST_CASE("Manifest-backed preset resolution loads the source vendor tree", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      vendor_dir = dir.path() / "Acme";
    const fs::path      child_file = vendor_dir / "process" / "nested" / "child.json";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"fdm_process_common","sub_path":"process/base.json"},)"
        << R"({"name":"Acme Process","sub_path":"process/nested/child.json"}]})";
    fs::create_directories(child_file.parent_path());
    std::ofstream((vendor_dir / "process" / "base.json").string())
        << R"({"type":"process","name":"fdm_process_common","from":"system",)"
        << R"("instantiation":"false","travel_speed":["321"]})";
    std::ofstream(child_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","inherits":"fdm_process_common","wall_loops":"5"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";
    raw.option<ConfigOptionInt>("wall_loops", true)->value             = 5;

    PresetBundle bundle;
    std::string  error;
    REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, child_file.string(),
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error.empty());
    REQUIRE(raw.option<ConfigOptionFloats>("travel_speed")->values.size() == 1);
    CHECK_THAT(raw.option<ConfigOptionFloats>("travel_speed")->values.front(), Catch::Matchers::WithinAbs(321.0, 1e-6));
    CHECK(raw.option<ConfigOptionInt>("wall_loops")->value == 5);
}

TEST_CASE("Manifest-backed resolution is scoped to the explicit source root", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    auto write_vendor = [&](const std::string &root_name, double travel_speed) {
        const fs::path root       = dir.path() / root_name;
        const fs::path child_file = root / "Acme" / "process" / "child.json";
        fs::create_directories(child_file.parent_path());
        std::ofstream((root / "Acme.json").string())
            << R"({"version":"1.0.0","name":"Acme","process_list":[)"
            << R"({"name":"fdm_process_common","sub_path":"process/base.json"},)"
            << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
        std::ofstream((root / "Acme" / "process" / "base.json").string())
            << R"({"type":"process","name":"fdm_process_common","from":"system",)"
            << R"("instantiation":"false","travel_speed":[")" << travel_speed << R"("]})";
        std::ofstream(child_file.string())
            << R"({"type":"process","name":"Acme Process","from":"system",)"
            << R"("instantiation":"true","inherits":"fdm_process_common"})";
        return child_file;
    };

    const fs::path source_a = write_vendor("root-a", 111.0);
    const fs::path source_b = write_vendor("root-b", 222.0);
    REQUIRE(fs::exists(source_a));

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "synthetic-parent-marker";

    PresetBundle bundle;
    std::string  error;
    REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, source_b.string(),
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    REQUIRE(raw.option<ConfigOptionFloats>("travel_speed")->values.size() == 1);
    CHECK_THAT(raw.option<ConfigOptionFloats>("travel_speed")->values.front(), Catch::Matchers::WithinAbs(222.0, 1e-6));
}

TEST_CASE("Exact-only resolution rejects an unconfigured manifest-backed file", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      source_file = dir.path() / "Acme" / "process" / "child.json";
    fs::create_directories(source_file.parent_path());
    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
    std::ofstream(source_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "Some Parent";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, source_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error, false));
    CHECK(error == "Preset was not found in the loaded bundle");
}

TEST_CASE("Vendor filament resolution uses the shared Orca library base", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      library_dir = dir.path() / PresetBundle::ORCA_FILAMENT_LIBRARY;
    const fs::path      vendor_dir  = dir.path() / "Acme";
    const fs::path      child_file  = vendor_dir / "filament" / "nested" / "petg.json";

    std::ofstream((dir.path() / (std::string(PresetBundle::ORCA_FILAMENT_LIBRARY) + ".json")).string())
        << R"({"version":"1.0.0","name":"OrcaFilamentLibrary","filament_list":[)"
        << R"({"name":"fdm_filament_pet","sub_path":"filament/pet.json","filament_id":"GFL99"}]})";
    fs::create_directories(library_dir / "filament");
    std::ofstream((library_dir / "filament" / "pet.json").string())
        << R"({"type":"filament","name":"fdm_filament_pet","from":"system",)"
        << R"("filament_id":"GFL99","instantiation":"false",)"
        << R"("filament_type":["PETG"],"filament_density":["1.27"]})";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","filament_list":[)"
        << R"({"name":"Acme PETG","sub_path":"filament/nested/petg.json","filament_id":"GFA00"}]})";
    fs::create_directories(child_file.parent_path());
    std::ofstream(child_file.string())
        << R"({"type":"filament","name":"Acme PETG","from":"system",)"
        << R"("filament_id":"GFA00","instantiation":"true","inherits":"fdm_filament_pet"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_filament_pet";

    PresetBundle bundle;
    std::string  error;
    REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_FILAMENT, child_file.string(),
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error.empty());
    CHECK(raw.opt_string("filament_type", 0u) == "PETG");
    REQUIRE(raw.option<ConfigOptionFloats>("filament_density")->values.size() == 1);
    CHECK_THAT(raw.option<ConfigOptionFloats>("filament_density")->values.front(), Catch::Matchers::WithinAbs(1.27, 1e-6));
}

TEST_CASE("Manifest-backed resolution rejects a missing parent", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      child_file = dir.path() / "Acme" / "process" / "child.json";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
    fs::create_directories(child_file.parent_path());
    std::ofstream(child_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","inherits":"Missing Parent","layer_height":"0.2"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "Missing Parent";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, child_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("Manifest-backed resolution rejects a vendor load with malformed entries", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      child_file = dir.path() / "Acme" / "process" / "child.json";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[123,)"
        << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
    fs::create_directories(child_file.parent_path());
    std::ofstream(child_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, child_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_FALSE(error.empty());
}

TEST_CASE("Manifest-backed resolution rejects files absent from the vendor manifest", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      listed_file   = dir.path() / "Acme" / "process" / "listed.json";
    const fs::path      unlisted_file = dir.path() / "Acme" / "process" / "unlisted.json";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"Listed Process","sub_path":"process/listed.json"}]})";
    fs::create_directories(listed_file.parent_path());
    std::ofstream(listed_file.string())
        << R"({"type":"process","name":"Listed Process","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";
    std::ofstream(unlisted_file.string())
        << R"({"type":"process","name":"Unlisted Process","from":"system",)"
        << R"("instantiation":"true","inherits":"fdm_process_common"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, unlisted_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error == "Source file is not an instantiated preset in its vendor manifest");
}

TEST_CASE("Manifest-backed resolution rejects a mismatched preset type", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      process_file = dir.path() / "Acme" / "process" / "child.json";

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
    fs::create_directories(process_file.parent_path());
    std::ofstream(process_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_filament_common";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_FILAMENT, process_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error == "Source file is not an instantiated preset in its vendor manifest");
}

TEST_CASE("Resolution terminates when no vendor manifest exists", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      detached_file = dir.path() / "detached.json";
    std::ofstream(detached_file.string()) << "{}";

    DynamicPrintConfig raw;
    raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "Missing Parent";

    PresetBundle bundle;
    std::string  error;
    CHECK_FALSE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, detached_file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error == "Preset was not found in the loaded bundle");
}

TEST_CASE("Manifest-backed resolution reuses the vendor tree it already loaded", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      process_dir = dir.path() / "Acme" / "process";
    fs::create_directories(process_dir);
    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","process_list":[)"
        << R"({"name":"fdm_process_common","sub_path":"process/base.json"},)"
        << R"({"name":"Acme First","sub_path":"process/first.json"},)"
        << R"({"name":"Acme Second","sub_path":"process/second.json"}]})";
    auto write_base = [&](double travel_speed) {
        std::ofstream((process_dir / "base.json").string())
            << R"({"type":"process","name":"fdm_process_common","from":"system",)"
            << R"("instantiation":"false","travel_speed":[")" << travel_speed << R"("]})";
    };
    auto write_child = [&](const std::string &file, const std::string &name) {
        std::ofstream((process_dir / file).string())
            << R"({"type":"process","name":")" << name << R"(","from":"system",)"
            << R"("instantiation":"true","inherits":"fdm_process_common"})";
    };
    write_base(111.0);
    write_child("first.json", "Acme First");
    write_child("second.json", "Acme Second");

    auto travel_speed = [&](PresetBundle &bundle, const std::string &file) {
        DynamicPrintConfig raw;
        raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";
        std::string error;
        REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, (process_dir / file).string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
        return raw.option<ConfigOptionFloats>("travel_speed")->values.front();
    };

    PresetBundle bundle;
    CHECK_THAT(travel_speed(bundle, "first.json"), Catch::Matchers::WithinAbs(111.0, 1e-6));

    // Only a reload would see this change.
    write_base(222.0);
    CHECK_THAT(travel_speed(bundle, "second.json"), Catch::Matchers::WithinAbs(111.0, 1e-6));

    PresetBundle fresh;
    CHECK_THAT(travel_speed(fresh, "second.json"), Catch::Matchers::WithinAbs(222.0, 1e-6));
}

TEST_CASE("Manifest-backed resolution does not keep a vendor tree that failed to load", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      child_file = dir.path() / "Acme" / "process" / "child.json";
    auto write_manifest = [&](const std::string &leading_entry) {
        std::ofstream((dir.path() / "Acme.json").string())
            << R"({"version":"1.0.0","name":"Acme","process_list":[)" << leading_entry
            << R"({"name":"Acme Process","sub_path":"process/child.json"}]})";
    };
    write_manifest("123,");
    fs::create_directories(child_file.parent_path());
    std::ofstream(child_file.string())
        << R"({"type":"process","name":"Acme Process","from":"system",)"
        << R"("instantiation":"true","layer_height":"0.2"})";

    PresetBundle bundle;
    auto resolve = [&](std::string &error) {
        DynamicPrintConfig raw;
        raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_process_common";
        return bundle.resolve_preset_config(raw, Preset::TYPE_PRINT, child_file.string(),
                                            ForwardCompatibilitySubstitutionRule::EnableSilent, error);
    };

    std::string error;
    CHECK_FALSE(resolve(error));
    CHECK_FALSE(error.empty());

    write_manifest("");
    error.clear();
    CHECK(resolve(error));
    CHECK(error.empty());
}

TEST_CASE("Manifest-backed resolution reuses the library base for type-probed files", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      library_pet  = dir.path() / PresetBundle::ORCA_FILAMENT_LIBRARY / "filament" / "pet.json";
    const fs::path      filament_dir = dir.path() / "Acme" / "filament";

    std::ofstream((dir.path() / (std::string(PresetBundle::ORCA_FILAMENT_LIBRARY) + ".json")).string())
        << R"({"version":"1.0.0","name":"OrcaFilamentLibrary","filament_list":[)"
        << R"({"name":"fdm_filament_pet","sub_path":"filament/pet.json","filament_id":"GFL99"}]})";
    fs::create_directories(library_pet.parent_path());
    auto write_library_pet = [&](double density) {
        std::ofstream(library_pet.string())
            << R"({"type":"filament","name":"fdm_filament_pet","from":"system",)"
            << R"("filament_id":"GFL99","instantiation":"false",)"
            << R"("filament_type":["PETG"],"filament_density":[")" << density << R"("]})";
    };
    write_library_pet(1.27);

    std::ofstream((dir.path() / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","filament_list":[)"
        << R"({"name":"Acme PETG","sub_path":"filament/petg.json","filament_id":"GFA00"},)"
        << R"({"name":"Acme PETG Matte","sub_path":"filament/petg_matte.json","filament_id":"GFA01"}]})";
    fs::create_directories(filament_dir);
    auto write_child = [&](const std::string &file, const std::string &name, const std::string &filament_id) {
        std::ofstream((filament_dir / file).string())
            << R"({"type":"filament","name":")" << name << R"(","from":"system",)"
            << R"("filament_id":")" << filament_id << R"(","instantiation":"true","inherits":"fdm_filament_pet"})";
    };
    write_child("petg.json", "Acme PETG", "GFA00");
    write_child("petg_matte.json", "Acme PETG Matte", "GFA01");

    auto density = [](const DynamicPrintConfig &config) {
        return config.option<ConfigOptionFloats>("filament_density")->values.front();
    };

    PresetBundle       bundle;
    DynamicPrintConfig first;
    first.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_filament_pet";
    std::string error;
    REQUIRE(bundle.resolve_preset_config(first, Preset::TYPE_FILAMENT, (filament_dir / "petg.json").string(),
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_THAT(density(first), Catch::Matchers::WithinAbs(1.27, 1e-6));

    // Only a reload would see this change.
    write_library_pet(1.5);

    DynamicPrintConfig second;
    Preset::Type       type = Preset::TYPE_INVALID;
    REQUIRE(bundle.resolve_preset_config_type(second, type, (filament_dir / "petg_matte.json").string(),
                                              ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(type == Preset::TYPE_FILAMENT);
    CHECK_THAT(density(second), Catch::Matchers::WithinAbs(1.27, 1e-6));
}

TEST_CASE("Manifest-backed resolution shares the library between vendors under one root", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir dir;
    const fs::path      library_dir = dir.path() / PresetBundle::ORCA_FILAMENT_LIBRARY / "filament";

    std::ofstream((dir.path() / (std::string(PresetBundle::ORCA_FILAMENT_LIBRARY) + ".json")).string())
        << R"({"version":"1.0.0","name":"OrcaFilamentLibrary","filament_list":[)"
        << R"({"name":"fdm_filament_pet","sub_path":"filament/pet.json","filament_id":"GFL99"},)"
        << R"({"name":"Generic PETG","sub_path":"filament/generic_petg.json","filament_id":"GFL98"}]})";
    fs::create_directories(library_dir);
    auto write_library_pet = [&](double density) {
        std::ofstream((library_dir / "pet.json").string())
            << R"({"type":"filament","name":"fdm_filament_pet","from":"system",)"
            << R"("filament_id":"GFL99","instantiation":"false",)"
            << R"("filament_type":["PETG"],"filament_density":[")" << density << R"("]})";
    };
    write_library_pet(1.27);
    std::ofstream((library_dir / "generic_petg.json").string())
        << R"({"type":"filament","name":"Generic PETG","from":"system",)"
        << R"("filament_id":"GFL98","instantiation":"true","inherits":"fdm_filament_pet"})";

    auto write_vendor = [&](const std::string &vendor, const std::string &filament_id) {
        const fs::path filament_dir = dir.path() / vendor / "filament";
        fs::create_directories(filament_dir);
        std::ofstream((dir.path() / (vendor + ".json")).string())
            << R"({"version":"1.0.0","name":")" << vendor << R"(","filament_list":[)"
            << R"({"name":")" << vendor << R"( PETG","sub_path":"filament/petg.json","filament_id":")" << filament_id << R"("}]})";
        std::ofstream((filament_dir / "petg.json").string())
            << R"({"type":"filament","name":")" << vendor << R"( PETG","from":"system",)"
            << R"("filament_id":")" << filament_id << R"(","instantiation":"true","inherits":"fdm_filament_pet"})";
        return filament_dir / "petg.json";
    };
    const fs::path acme_petg = write_vendor("Acme", "GFA00");
    const fs::path beta_petg = write_vendor("Beta", "GFB00");

    auto density = [&](PresetBundle &bundle, const fs::path &file) {
        DynamicPrintConfig raw;
        raw.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "fdm_filament_pet";
        std::string error;
        REQUIRE(bundle.resolve_preset_config(raw, Preset::TYPE_FILAMENT, file.string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
        return raw.option<ConfigOptionFloats>("filament_density")->values.front();
    };

    PresetBundle bundle;
    CHECK_THAT(density(bundle, acme_petg), Catch::Matchers::WithinAbs(1.27, 1e-6));

    // Only a reload would see this change.
    write_library_pet(1.5);
    CHECK_THAT(density(bundle, beta_petg), Catch::Matchers::WithinAbs(1.27, 1e-6));
    CHECK_THAT(density(bundle, library_dir / "generic_petg.json"), Catch::Matchers::WithinAbs(1.27, 1e-6));

    PresetBundle fresh;
    CHECK_THAT(density(fresh, beta_petg), Catch::Matchers::WithinAbs(1.5, 1e-6));
}

// Orca: a filament in the Orca Filament Library that names its compatible printers has to hide the generic
// library filament sharing its alias, the same way a vendor owned filament does. Otherwise both are compatible
// with that printer and the plater combo box lists the shared alias twice.
TEST_CASE("A printer specific filament supersedes the generic library filament with the same alias", "[Preset][Bundle]")
{
    LibraryFilamentTestCollection filaments;
    PresetCollection              printers(Preset::TYPE_PRINTER, Preset::printer_options(),
                                           static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    // The masking keys off the vendor name, which VendorProfile's constructor does not derive from the id.
    VendorProfile                 library(PresetBundle::ORCA_FILAMENT_LIBRARY);
    VendorProfile                 vendor("Vendor");
    library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
    vendor.name  = "Vendor";

    auto add_filament = [&filaments](const VendorProfile &owner, const std::string &name, std::vector<std::string> compatible_printers) {
        Preset &preset = add_inmemory_preset(filaments, name);
        preset.alias   = "Generic ABS";
        preset.vendor  = &owner;
        preset.config.option<ConfigOptionStrings>("compatible_printers", true)->values = std::move(compatible_printers);
    };

    add_filament(library, "Generic ABS @System", {});
    add_filament(library, "Generic ABS @Printer A", { "Printer A" });
    add_filament(vendor,  "Generic ABS @Printer B", { "Printer B" });

    filaments.update_library_profile_excluded_from();

    const Preset *generic = filaments.find_preset("Generic ABS @System");
    REQUIRE(generic != nullptr);
    CHECK(generic->m_excluded_from.count("Printer A") == 1);
    CHECK(generic->m_excluded_from.count("Printer B") == 1);
    CHECK(generic->m_excluded_from.size() == 2);

    // A printer specific profile names printers, so it is never the one being hidden - not even by itself.
    const Preset *specific = filaments.find_preset("Generic ABS @Printer A");
    REQUIRE(specific != nullptr);
    CHECK(specific->m_excluded_from.empty());

    // ...and the generic profile really drops out of the compatible set on the printer it is hidden from.
    add_inmemory_preset(printers, "Printer A");
    add_inmemory_preset(printers, "Printer C");
    const Preset *printer_a = printers.find_preset("Printer A");
    const Preset *printer_c = printers.find_preset("Printer C");
    REQUIRE(printer_a != nullptr);
    REQUIRE(printer_c != nullptr);

    const PresetWithVendorProfile generic_lib(*generic, &library);
    CHECK_FALSE(is_compatible_with_printer(generic_lib, PresetWithVendorProfile(*printer_a, nullptr)));
    CHECK(is_compatible_with_printer(generic_lib, PresetWithVendorProfile(*printer_c, nullptr)));
}

namespace {

// One system printer plus the filament presets a machine facing dialog has to choose between:
// an Orca Filament Library generic with no compatible_printers, a same alias vendor filament
// that names the printer, a library filament with no vendor twin, and a vendor filament that
// belongs to a different printer.
struct MachineFilaments
{
    PresetBundle  bundle;
    VendorProfile library{PresetBundle::ORCA_FILAMENT_LIBRARY};
    VendorProfile vendor{"Vendor"};

    MachineFilaments()
    {
        // VendorProfile's constructor takes an id; the library rule keys off the name.
        library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
        vendor.name  = "Vendor";

        Preset &printer = add_inmemory_preset(bundle.printers, "Printer A 0.4 nozzle");
        printer.is_system = true;
        printer.vendor    = &vendor;
        printer.config.option<ConfigOptionString>("printer_model", true)->value = "Printer A";

        add_filament(library, "Generic ABS @System",    "Generic ABS", {});
        add_filament(vendor,  "Generic ABS @Printer A", "Generic ABS", { "Printer A 0.4 nozzle" });
        add_filament(library, "FilAr ABS @System",      "FilAr ABS",   {});
        add_filament(vendor,  "Vendor PLA @Printer B",  "Vendor PLA",  { "Printer B 0.4 nozzle" });

        // update_library_profile_excluded_from() is protected and has its own test above; record
        // the exclusion it derives from the same alias vendor filament.
        Preset *shadowed = bundle.filaments.find_preset("Generic ABS @System");
        REQUIRE(shadowed != nullptr);
        shadowed->m_excluded_from.insert("Printer A 0.4 nozzle");
    }

    void add_filament(const VendorProfile &owner, const std::string &name, const std::string &alias,
                      std::vector<std::string> compatible_printers)
    {
        Preset &preset = add_inmemory_preset(bundle.filaments, name);
        preset.is_system = true;
        preset.alias     = alias;
        preset.vendor    = &owner;
        compatible_list(bundle.filaments, name, "compatible_printers") = std::move(compatible_printers);
    }

    bool offers(const std::string &preset_name, bool include_user_presets = false)
    {
        const std::vector<Preset *> offered =
            bundle.get_filament_presets_for_machine("Printer A", "0.4", include_user_presets);
        return std::any_of(offered.begin(), offered.end(),
                           [&preset_name](const Preset *p) { return p->name == preset_name; });
    }
};

} // namespace

TEST_CASE("Filaments offered for a machine follow the app's compatibility rule", "[Preset][Bundle]")
{
    MachineFilaments f;

    SECTION("a library filament with no compatible_printers is offered") {
        CHECK(f.offers("FilAr ABS @System"));
    }

    SECTION("a same alias vendor filament shadows the library generic") {
        CHECK(f.offers("Generic ABS @Printer A"));
        CHECK_FALSE(f.offers("Generic ABS @System"));
    }

    SECTION("a filament naming a different printer is not offered") {
        CHECK_FALSE(f.offers("Vendor PLA @Printer B"));
    }

    SECTION("a user filament is offered only when the printer supports user presets") {
        add_inmemory_preset(f.bundle.filaments, "My PLA");

        CHECK_FALSE(f.offers("My PLA", /*include_user_presets=*/false));
        CHECK(f.offers("My PLA", /*include_user_presets=*/true));
    }
}

TEST_CASE("Switching a printer reselects a compatible process and every filament slot", "[Preset][Bundle][Compatibility][FilamentMixer][PresetSelection]")
{
    PresetBundle bundle;

    Preset &printer_a = add_inmemory_preset(bundle.printers, "Printer A");
    Preset &printer_b = add_inmemory_preset(bundle.printers, "Printer B");
    for (Preset *printer : { &printer_a, &printer_b }) {
        printer->is_system = true;
        printer->config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = { 0.4, 0.4 };
    }
    printer_a.config.option<ConfigOptionString>("default_print_profile", true)->value = "Process A";
    printer_a.config.option<ConfigOptionStrings>("default_filament_profile", true)->values = { "Filament A", "Filament A" };
    printer_b.config.option<ConfigOptionString>("default_print_profile", true)->value = "Process B";
    printer_b.config.option<ConfigOptionStrings>("default_filament_profile", true)->values = { "Filament B", "Filament B" };

    Preset &process_a = add_inmemory_preset(bundle.prints, "Process A");
    Preset &process_b = add_inmemory_preset(bundle.prints, "Process B");
    compatible_list(bundle.prints, process_a.name, "compatible_printers") = { printer_a.name };
    compatible_list(bundle.prints, process_b.name, "compatible_printers") = { printer_b.name };
    process_a.config.option<ConfigOptionFloat>("layer_height", true)->value = 0.16;
    process_b.config.option<ConfigOptionFloat>("layer_height", true)->value = 0.24;

    Preset &filament_a = add_inmemory_preset(bundle.filaments, "Filament A");
    Preset &filament_b = add_inmemory_preset(bundle.filaments, "Filament B");
    compatible_list(bundle.filaments, filament_a.name, "compatible_printers") = { printer_a.name };
    compatible_list(bundle.filaments, filament_a.name, "compatible_prints") = { process_a.name };
    compatible_list(bundle.filaments, filament_b.name, "compatible_printers") = { printer_b.name };
    compatible_list(bundle.filaments, filament_b.name, "compatible_prints") = { process_b.name };

    bundle.printers.select_preset_by_name(printer_a.name, true);
    bundle.prints.select_preset_by_name(process_a.name, true);
    bundle.filaments.select_preset_by_name(filament_a.name, true);
    bundle.filament_presets = { filament_a.name, filament_a.name };
    bundle.update_compatible(PresetSelectCompatibleType::Always);

    bundle.printers.select_preset_by_name(printer_b.name, true);
    bundle.update_compatible(PresetSelectCompatibleType::Always);

    CHECK(bundle.prints.get_selected_preset_name() == process_b.name);
    CHECK_THAT(bundle.prints.get_selected_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    CHECK(bundle.filaments.get_selected_preset_name() == filament_b.name);
    CHECK(bundle.filament_presets == std::vector<std::string>{ filament_b.name, filament_b.name });
    CHECK_FALSE(bundle.filaments.find_preset(filament_a.name, false, true)->is_compatible);
    CHECK(bundle.filaments.find_preset(filament_b.name, false, true)->is_compatible);
}

TEST_CASE("A user filament preset round-trips inherited multi-variant vectors", "[Preset][Bundle][RoundTrip]")
{
    ScopedTemporaryDir temp_dir;
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    DynamicPrintConfig parent_config = collection.default_preset().config;
    parent_config.option<ConfigOptionFloats>("filament_diameter", true)->values = { 1.75, 2.85 };
    parent_config.option<ConfigOptionStrings>("filament_type", true)->values = { "PLA", "PETG" };
    parent_config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"
    };
    const fs::path filament_dir = temp_dir.path() / PRESET_FILAMENT_NAME;
    const fs::path parent_file = filament_dir / "base" / "Multi Base.json";
    const fs::path child_file = filament_dir / "Multi Child.json";
    write_preset_with_inherits(parent_config, parent_file, "Multi Base", "");
    write_minimal_child(child_file, "Multi Child", "Multi Base", "filament");

    PresetsConfigSubstitutions substitutions;
    collection.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                            ForwardCompatibilitySubstitutionRule::Disable);
    const Preset *child = collection.find_preset("Multi Child");
    REQUIRE(child != nullptr);
    CHECK(child->inherits() == "Multi Base");
    check_double_vector(child->config.option<ConfigOptionFloats>("filament_diameter")->values, { 1.75, 2.85 });
    CHECK(child->config.option<ConfigOptionStrings>("filament_type")->values == std::vector<std::string>{ "PLA", "PETG" });
    CHECK(child->config.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          std::vector<std::string>{ "Direct Drive Standard", "Direct Drive High Flow" });

    PresetCollection reloaded(Preset::TYPE_FILAMENT, Preset::filament_options(),
                              static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    PresetsConfigSubstitutions reload_substitutions;
    reloaded.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, reload_substitutions,
                          ForwardCompatibilitySubstitutionRule::Disable);
    const Preset *reloaded_child = reloaded.find_preset("Multi Child");
    REQUIRE(reloaded_child != nullptr);
    check_double_vector(reloaded_child->config.option<ConfigOptionFloats>("filament_diameter")->values, { 1.75, 2.85 });
    CHECK(reloaded_child->config.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          child->config.option<ConfigOptionStrings>("filament_extruder_variant")->values);
}


namespace {

const char *kMixedKeys[] = {
    "filament_is_mixed",
    "filament_mixed_components",
    "filament_mixed_sublayer_ratios",
    "filament_mixed_gradient",
    "filament_mixed_gradient_range",
    "filament_mixed_gradient_curve",
    "filament_mixed_gradient_per_part",
};

} // namespace

// Mixed-color filament metadata lives in project_config as parallel per-filament arrays.
// set_num_filaments() grows them alongside filament_colour; if it
// misses them, creating a mixed slot writes past the end of the short arrays.
TEST_CASE("set_num_filaments keeps mixed-color arrays in step with the filament count", "[Preset][Bundle][FilamentMixer]")
{
    auto mixed_array_size = [](const DynamicPrintConfig &cfg, const std::string &key) -> size_t {
        if (const auto *b = cfg.option<ConfigOptionBools>(key))
            return b->values.size();
        if (const auto *s = cfg.option<ConfigOptionStrings>(key))
            return s->values.size();
        return size_t(-1);   // key missing entirely
    };

    PresetBundle bundle;

    const unsigned int n = GENERATE(2u, 4u, 8u);
    bundle.set_num_filaments(n, std::string("#FF0000"));

    REQUIRE(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values.size() == n);
    for (const char *key : kMixedKeys) {
        DYNAMIC_SECTION("grown: " << key) {
            CHECK(mixed_array_size(bundle.project_config, key) == n);
        }
    }

    SECTION("shrinking keeps them in step too") {
        bundle.set_num_filaments(1, std::string("#00FF00"));
        REQUIRE(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values.size() == 1);
        for (const char *key : kMixedKeys)
            CHECK(mixed_array_size(bundle.project_config, key) == 1);
    }
}

// A mix is described by 1-based indices into the project's filament list, which Orca rebuilds
// from the selected printer's snapshot (filament_%02u / filament_colors) at startup and on every
// printer selection. Held anywhere but that same per-printer snapshot, the mixed arrays end up
// indexing a filament list they were never saved against.
TEST_CASE("Mixed-color filament metadata is snapshotted per printer, with its filament list", "[Preset][Bundle][FilamentMixer]")
{
    PresetBundle bundle;
    // export_selections skips the built-in "Default Printer" placeholder entirely.
    add_inmemory_preset(bundle.printers, "Test Printer");
    bundle.printers.select_preset_by_name("Test Printer", true);
    bundle.set_num_filaments(2u, std::string("#FF0000"));
    bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values          = { false, true };
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values = { "", "1,2" };
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "0.5,0.5" };

    AppConfig app_config;
    bundle.export_selections(app_config);

    const std::string printer_name = bundle.printers.get_selected_preset_name();
    for (const char *key : kMixedKeys) {
        DYNAMIC_SECTION("per printer, not global: " << key) {
            CHECK(app_config.has_printer_setting(printer_name, key));
            CHECK_FALSE(app_config.has("presets", key));
        }
    }

    SECTION("with the encoding load_selections reads back") {
        CHECK(app_config.get_printer_setting(printer_name, "filament_is_mixed") == "0,1");
        CHECK(app_config.get_printer_setting(printer_name, "filament_mixed_components") == "|1,2");
        CHECK(app_config.get_printer_setting(printer_name, "filament_mixed_sublayer_ratios") == "|0.5,0.5");
    }
}

// The gradient curve is the one mixed array whose values contain '|' themselves — it separates the
// control points — so it cannot be '|'-joined into the app config like its siblings without a
// multi-point curve being split across filament slots on the way back in.
TEST_CASE("A multi-point gradient curve survives the app-config snapshot", "[Preset][Bundle][FilamentMixer]")
{
    const std::vector<std::string> curves = { "", "", "0,0|0.5,0.3|1,1" };

    PresetBundle bundle;
    add_inmemory_preset(bundle.printers, "Test Printer");
    bundle.printers.select_preset_by_name("Test Printer", true);
    bundle.set_num_filaments(3u, std::string("#FF0000"));
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_gradient_curve")->values = curves;

    AppConfig app_config;
    bundle.export_selections(app_config);

    // Decoding the stored form returns the three slots intact, curve delimiters and all. A plain
    // '|' join would decode as five slots here instead of three.
    std::vector<std::string> decoded;
    REQUIRE(unescape_strings_cstyle(
        app_config.get_printer_setting(bundle.printers.get_selected_preset_name(), "filament_mixed_gradient_curve"), decoded));
    CHECK(decoded == curves);
}

// A multi-tool printer sizes the filament list from its nozzle count. Mixed-color slots are extra
// virtual filaments at the tail of that list with no nozzle of their own, so the count has to
// allow for them: sizing to the nozzle count alone drops the project's mixes and strips every
// painted facet above the new count.
TEST_CASE("Sizing the filament list to a multi-tool nozzle count keeps mixed slots", "[Preset][Bundle][FilamentMixer]")
{
    // The 5-slot layout of a 4-tool project carrying one mix of filaments 2 and 3.
    const size_t nozzle_count = 4;
    PresetBundle bundle;
    bundle.set_num_filaments(5u, std::string("#FF0000"));
    bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values =
        { false, false, false, false, true };
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values =
        { "", "", "", "", "2,3" };

    REQUIRE(bundle.num_mixed_filaments() == 1);

    SECTION("nozzle count plus the mixed slots preserves the mix") {
        bundle.set_num_filaments(nozzle_count + bundle.num_mixed_filaments(), std::string("#00FF00"));

        CHECK(bundle.filament_presets.size() == 5);
        CHECK(bundle.num_mixed_filaments() == 1);
        CHECK(bundle.is_mixed_filament(4));
        CHECK(bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values[4] == "2,3");
    }

    SECTION("the nozzle count alone is what truncated it away") {
        bundle.set_num_filaments(nozzle_count, std::string("#00FF00"));

        CHECK(bundle.filament_presets.size() == nozzle_count);
        CHECK(bundle.num_mixed_filaments() == 0);
    }
}

// A "published" 3MF keeps the user's currently-selected presets and overlays only the
// author-selected process keys onto the edited preset (mirrors the GUI load path: normalize
// before load_config_model, then the overlay in load_config_file_config).
TEST_CASE("Published 3MF overlays only the author-selected process keys onto the edited preset", "[Preset][Bundle][Published]")
{
    // The file config the GUI builds from a .3mf's project settings.
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // The loader derives the filament count from filament_colour and throws when it is
        // empty; a 3mf always carries it.
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        config.opt_float("layer_height") = 0.28;                       // process scalar
        config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values = { 140., 150. }; // matching-size vector
        config.opt<ConfigOptionStrings>("post_process")->values = { "script-a", "script-b" }; // mismatched vector
        config.opt<ConfigOptionInts>("nozzle_temperature")->values = { 220 }; // filament key (not applied anywhere)
        // Structural (denylisted) key: must be silently ignored. full_print_config() omits the
        // *_settings_id keys, so create one explicitly.
        config.opt_string("print_settings_id", true) = "file process";
        config.opt<ConfigOptionFloats>("flush_multiplier")->values = { 2., 2. }; // must NOT cross over
        config.opt<ConfigOptionFloats>("wipe_tower_x")->values = { 100. };       // plate geometry, does cross over
        config.opt<ConfigOptionFloat>("wipe_tower_rotation_angle")->value = 45.; // published-only plate geometry, crosses over
        config.option("curr_bed_type")->setInt(BedType::btPC);                   // must NOT cross over
        return config;
    };

    const std::vector<std::string> published_keys = {
        "layer_height", "wiping_volumes_extruders", "post_process", "nozzle_temperature", "print_settings_id"
    };

    PresetBundle bundle;
    const std::string pre_load_name = bundle.prints.get_edited_preset().name;
    const size_t      pre_load_size = bundle.prints.size();

    // The edited presets are the overlay targets: recognizable pre-load values.
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.1;
    bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values = { 10., 20. };
    bundle.prints.get_edited_preset().config.opt<ConfigOptionStrings>("post_process")->values = { "existing-script" };
    bundle.prints.get_edited_preset().config.opt_string("print_settings_id") = "user process";
    // Capture the ctor-seeded project_config values so the assertions below check the load
    // leaves them untouched rather than hardcoding the defaults.
    const std::vector<std::string> seed_filament_colour  = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
    const std::vector<double>      seed_flush_multiplier = bundle.project_config.opt<ConfigOptionFloats>("flush_multiplier")->values;
    const int                      seed_bed_type         = bundle.project_config.option("curr_bed_type")->getInt();

    DynamicPrintConfig config = make_file_config();
    // The GUI normalizes the config before load; mirror that so only the production path runs.
    Preset::normalize(config);

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = published_keys;
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // a) Process scalar overlaid; matching-size vector applied, mismatched one lands in
    // skipped_keys; applied keys are not reported.
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 0.000001));
    check_double_vector(bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values, { 140., 150. });
    CHECK(bundle.prints.get_edited_preset().config.opt<ConfigOptionStrings>("post_process")->values == std::vector<std::string>{ "existing-script" });
    CHECK(contains_key(pub.skipped_keys, "post_process"));
    CHECK_FALSE(contains_key(pub.skipped_keys, "layer_height"));
    CHECK_FALSE(contains_key(pub.skipped_keys, "wiping_volumes_extruders"));

    // b) A filament key is never applied anywhere and is reported as skipped.
    CHECK(bundle.prints.get_edited_preset().config.option("nozzle_temperature") == nullptr);
    CHECK(contains_key(pub.skipped_keys, "nozzle_temperature"));

    // c) A structural key is silently ignored: neither applied nor reported as skipped.
    CHECK(bundle.prints.get_edited_preset().config.opt_string("print_settings_id") == "user process");
    CHECK_FALSE(contains_key(pub.skipped_keys, "print_settings_id"));

    // d) Only plate/bed geometry crosses in published mode: filament/purge data and bed type
    // stay at the ctor seeds.
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == seed_filament_colour);
    CHECK(bundle.project_config.opt<ConfigOptionFloats>("flush_multiplier")->values == seed_flush_multiplier);
    CHECK(bundle.project_config.option("curr_bed_type")->getInt() == seed_bed_type);
    check_double_vector(bundle.project_config.opt<ConfigOptionFloats>("wipe_tower_x")->values, { 100. });
    CHECK_THAT(bundle.project_config.opt<ConfigOptionFloat>("wipe_tower_rotation_angle")->value, Catch::Matchers::WithinAbs(45., 0.000001));

    // e) The published path keeps the user's currently-selected presets: same preset, same size.
    CHECK(bundle.prints.get_edited_preset().name == pre_load_name);
    CHECK(bundle.prints.size() == pre_load_size);

    // f) Non-published control: the overlay is disabled, the file's presets are imported
    // instead, and no skipped_keys are produced.
    PresetBundle     control_bundle;
    const size_t     control_pre_size = control_bundle.prints.size();
    PublishedConfig  control_pub;
    control_pub.published      = false;
    control_pub.published_keys = published_keys;
    DynamicPrintConfig control_config = make_file_config();
    Preset::normalize(control_config);
    control_bundle.load_config_model("test.3mf", std::move(control_config), Semver(), &control_pub);

    CHECK(control_pub.skipped_keys.empty());
    CHECK(control_bundle.prints.size() > control_pre_size);
    // The file's layer_height reached the edited preset via the normal import, not the overlay.
    CHECK_THAT(control_bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 0.000001));
}

// The published printer overlay is restricted to the publishable retraction/z-hop allowlist:
// matching-size vectors apply, mismatched vectors are reported as skipped, and any other
// printer-class key (e.g. machine_start_gcode) is contract-excluded (never applied, never
// reported).
TEST_CASE("Published 3MF overlays only the allowlisted retraction and z-hop keys onto the edited printer preset", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    Preset::normalize(config);
    config.opt<ConfigOptionFloats>("retraction_length")->values = { 1.4 };         // matching size (1 extruder)
    config.opt<ConfigOptionFloats>("retraction_speed")->values = { 45., 55. };     // size 2: mismatched
    config.opt_string("machine_start_gcode") = "G28 ; from file";                  // outside the allowlist

    PresetBundle bundle;
    bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8 };
    // A recognizable non-default value: the skipped mismatch below must leave it untouched
    // (asserting the default instead would silently test PrintConfig's retraction_speed).
    bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_speed")->values = { 33. };
    bundle.printers.get_edited_preset().config.opt_string("machine_start_gcode") = "G28 ; user";

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = { "retraction_length", "retraction_speed", "machine_start_gcode" };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // Matching-size retraction vector applied; mismatched vector reported as skipped and the
    // receiver's own value survives.
    check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 1.4 });
    check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_speed")->values, { 33. });
    CHECK(contains_key(pub.skipped_keys, "retraction_speed"));
    // Contract-excluded printer key: silently ignored, absent from skipped_keys.
    CHECK(bundle.printers.get_edited_preset().config.opt_string("machine_start_gcode") == "G28 ; user");
    CHECK_FALSE(contains_key(pub.skipped_keys, "machine_start_gcode"));
}

// A published 3MF carries per-slot material keys; on load they are applied positionally to the
// receiver's slot N (a key-only entry has no type gate), written onto the slot's stored preset
// in place.
TEST_CASE("Published 3MF applies positional material keys onto the receiver's material presets", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // Two filament slots; filament_diameter drives the normalized per-slot vector sizes.
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        // Keep the multi-extruder consistency validation happy for a 2-slot config.
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFT99" };
        // Author per-slot retraction values. These are per-filament override keys that are not
        // members of the static PrintRegionConfig, so full_print_config() omits them and they
        // must be created explicitly (as nullable, matching the real 3MF project config).
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 1.2 };
        config.option<ConfigOptionFloatsNullable>("filament_z_hop", true)->values = { 0.2, 0.3 };
        return config;
    };

    PresetBundle bundle;
    Preset &pla  = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.filament_id = "GFL99";
    pla.config.opt_string("filament_type", 0u)   = "PLA";
    pla.config.opt_string("filament_vendor", 0u) = "Generic";
    pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    pla.config.opt<ConfigOptionStrings>("filament_settings_id")->values = { "receiver-pla" };
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.filament_id = "GFT99";
    petg.config.opt_string("filament_type", 0u)   = "PETG";
    petg.config.opt_string("filament_vendor", 0u) = "Generic";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    petg.config.opt<ConfigOptionFloatsNullable>("filament_z_hop", true)->values = { 0.1 };
    bundle.filament_presets = { "My PLA", "My PETG" };

    PublishedMaterialEntry pla_entry;
    pla_entry.filament_id = "GFL99";
    pla_entry.slot        = 0; // the author's PLA slot
    pla_entry.keys        = { "filament_retraction_length", "filament_settings_id" };
    PublishedMaterialEntry petg_entry;
    petg_entry.filament_id = "GFT99";
    petg_entry.slot        = 1; // the author's PETG slot
    petg_entry.keys        = { "filament_retraction_length", "filament_z_hop" };
    // A slot-less entry (no slot field, only possible in hand-crafted files): silently skipped.
    PublishedMaterialEntry noslot_entry;
    noslot_entry.filament_type = "ABS";
    noslot_entry.keys          = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published      = true;
    pub.material_keys  = { pla_entry, petg_entry, noslot_entry };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The author's slot values are written onto the receiver's stored presets in place.
    check_double_vector(bundle.filaments.find_preset("My PLA")->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    check_double_vector(bundle.filaments.find_preset("My PETG")->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 1.2 });
    check_double_vector(bundle.filaments.find_preset("My PETG")->config.opt<ConfigOptionFloatsNullable>("filament_z_hop")->values, { 0.3 });
    // Structural keys inside a material entry are silently ignored: the receiver's own
    // filament_settings_id is untouched and nothing is reported for it.
    CHECK(bundle.filaments.find_preset("My PLA")->config.opt<ConfigOptionStrings>("filament_settings_id")->values == std::vector<std::string>{ "receiver-pla" });
    CHECK_FALSE(contains_key(pub.skipped_keys, "material:GFL99 (filament_settings_id)"));
    // Everything applied; the slot-less entry produced no skipped entry.
    CHECK(pub.skipped_keys.empty());
}

// A "full publish" slot serializes the whole filament. On load the slot always receives a
// standalone detached copy of the author's material - created even when the receiver's own
// material matches the published type - and no receiver library preset is ever mutated.
TEST_CASE("Published 3MF full-published slots are imported as standalone detached copies", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // Two author slots; filament_diameter drives the normalized per-slot vector sizes.
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFT99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 1.2 };
        return config;
    };
    // The full dump of slot 0, publishing the whole filament as type "ABS".
    auto make_full_abs_entry = [] {
        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.full                = true;
        entry.publish_type        = true;
        entry.publish_type_value  = "ABS";
        entry.full_keys           = { "filament_retraction_length" };
        return entry;
    };

    SECTION("type match still creates a detached copy instead of mutating the receiver material") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        bundle.filament_presets = { "My PLA", "My PLA" };

        PublishedMaterialEntry full = make_full_abs_entry();
        full.publish_type_value     = "PLA"; // author requires PLA, receiver slot is PLA

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { full };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The type matches, but the import still detaches: the slot lands on a fresh copy
        // named from the published type (no identity fields in this entry), carrying the
        // author's values; the receiver's own material is untouched.
        CHECK(bundle.filament_presets[0] == "PLA");
        Preset *copy = bundle.filaments.find_preset("PLA", false, true);
        REQUIRE(copy != nullptr);
        check_double_vector(copy->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        CHECK(pub.skipped_keys.empty());
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 0: My PLA -> PLA");
    }

    SECTION("type mismatch also detaches: a fresh same-type copy replaces the slot") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        Preset &abs = add_inmemory_preset(bundle.filaments, "My ABS");
        abs.config.opt_string("filament_type", 0u) = "ABS";
        abs.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.3 };
        bundle.filament_presets = { "My PLA" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_full_abs_entry() };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 1);
        // No substitution search runs: a brand-new copy named after the published type is
        // created and both pre-existing presets stay exactly as they were.
        CHECK(bundle.filament_presets[0] == "ABS");
        Preset *copy = bundle.filaments.find_preset("ABS", false, true);
        REQUIRE(copy != nullptr);
        check_double_vector(copy->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        check_double_vector(bundle.filaments.find_preset("My ABS", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.3 });
        check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        CHECK(pub.skipped_keys.empty());
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 0: My PLA -> ABS");
    }

    SECTION("the author's identity rides on the copy when the type has no library match") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        Preset &other = add_inmemory_preset(bundle.filaments, "Other PLA");
        other.config.opt_string("filament_type", 0u) = "PLA";
        other.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.7 };
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry full = make_full_abs_entry();
        // The dump carries the identity too, so the created copy takes the author's type
        // and vendor instead of the baseline clone's.
        full.full_keys = { "filament_retraction_length", "filament_type", "filament_vendor" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { full };
        DynamicPrintConfig config = make_file_config();
        // The author's slot 0 really is ABS.
        config.opt<ConfigOptionStrings>("filament_type")->values = { "ABS", "PETG" };
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // No ABS preset needs to exist in the library: the copy carries the author's values,
        // type and vendor included. Neither receiver preset was touched.
        CHECK(bundle.filament_presets[0] == "ABS");
        Preset *copy = bundle.filaments.find_preset("ABS", false, true);
        REQUIRE(copy != nullptr);
        check_double_vector(copy->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        CHECK(copy->config.opt_string("filament_type", 0u) == "ABS");
        CHECK(copy->config.opt_string("filament_vendor", 0u) == "Generic");
        check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt_string("filament_type", 0u) == "PLA");
        check_double_vector(bundle.filaments.find_preset("Other PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.7 });
        CHECK(pub.skipped_keys.empty());
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 0: My PLA -> ABS");
    }
}

// The author's preset name travels in the file and names the created standalone copy (variant
// tail stripped), regardless of what the receiver's library holds: an exact-name library preset
// is never reused nor mutated.
TEST_CASE("Published 3MF imports a full material under the author's stripped name", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "OGFL99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9 };
        return config;
    };

    auto add_pla = [](PresetBundle &bundle, const char *name, const char *id, const char *vendor, const char *setting_id) {
        Preset &preset = add_inmemory_preset(bundle.filaments, name);
        preset.filament_id = id;
        preset.setting_id  = setting_id;
        preset.config.opt_string("filament_type", 0u) = "PLA";
        preset.config.opt_string("filament_vendor", 0u) = vendor;
        preset.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        return &preset;
    };

    SECTION("an exact-name library preset exists: a detached copy is created beside it") {
        PresetBundle bundle;
        Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
        add_pla(bundle, "Generic PLA @System", "OGFL99", "Generic", "RcBNzytWgwRrwXXz");
        add_pla(bundle, "Bambu PLA Basic @System", "OGFA00", "Bambu Lab", "zkc85XTKi4cb6cOw");
        bundle.filament_presets = { "My PETG" };

        PublishedMaterialEntry entry;
        entry.slot               = 0;
        entry.full               = true;
        entry.publish_type       = true;
        entry.publish_type_value = "PLA";
        entry.filament_id        = "OGFL99";
        entry.filament_vendor    = "Generic";
        entry.setting_id         = "RcBNzytWgwRrwXXz";
        entry.preset_name        = "Generic PLA @System";
        entry.full_keys          = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The copy is named after the stripped author name; the receiver's exact-name preset
        // keeps its own values.
        CHECK(bundle.filament_presets[0] == "Generic PLA");
        check_double_vector(bundle.filaments.find_preset("Generic PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        check_double_vector(bundle.filaments.find_preset("Generic PLA @System", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        check_double_vector(bundle.filaments.find_preset("My PETG", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.6 });
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Generic PLA");
        CHECK(pub.skipped_keys.empty());
    }

    SECTION("only the name is present (broken/stale ids): the copy is still created") {
        PresetBundle bundle;
        Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
        add_pla(bundle, "Generic PLA @System", "OGFL99", "Generic", "RcBNzytWgwRrwXXz");
        add_pla(bundle, "Bambu PLA Basic @System", "OGFA00", "Bambu Lab", "zkc85XTKi4cb6cOw");
        bundle.filament_presets = { "My PETG" };

        PublishedMaterialEntry entry;
        entry.slot               = 0;
        entry.full               = true;
        entry.publish_type       = true;
        entry.publish_type_value = "PLA";
        entry.preset_name        = "Generic PLA @System";
        entry.full_keys          = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets[0] == "Generic PLA");
        check_double_vector(bundle.filaments.find_preset("Bambu PLA Basic @System", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Generic PLA");
        CHECK(pub.skipped_keys.empty());
    }

    SECTION("grown slot (author slot 1) receives its own detached copy") {
        auto two_slot_config = [] {
            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
            config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
            config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
            config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#FFFF00" };
            config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA" };
            config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
            config.opt<ConfigOptionStrings>("filament_ids")->values = { "OGFL99", "OGFL99" };
            config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 0.8 };
            return config;
        };

        PresetBundle bundle;
        Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
        add_pla(bundle, "Generic PLA @System", "OGFL99", "Generic", "RcBNzytWgwRrwXXz");
        add_pla(bundle, "Bambu PLA Basic @System", "OGFA00", "Bambu Lab", "zkc85XTKi4cb6cOw");
        bundle.filament_presets = { "My PETG" };

        PublishedMaterialEntry entry;
        entry.slot               = 1;
        entry.full               = true;
        entry.publish_type       = true;
        entry.publish_type_value = "PLA";
        entry.preset_name        = "Generic PLA @System";
        entry.full_keys          = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = two_slot_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 2);
        CHECK(bundle.filament_presets[0] == "My PETG");
        // The grown slot seeds the receiver's last preset ("My PETG"), then the full material
        // detaches onto a copy carrying the author's slot-1 value; "Generic PLA @System" is
        // left untouched.
        CHECK(bundle.filament_presets[1] == "Generic PLA");
        check_double_vector(bundle.filaments.find_preset("Generic PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.8 });
        check_double_vector(bundle.filaments.find_preset("Generic PLA @System", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 1: My PETG -> Generic PLA");
        CHECK(pub.skipped_keys.empty());
    }
}

// The stripped author name can collide with an existing library preset ("Generic PLA"): the
// created copy must uniquify with the "(Published)" suffix rather than overwrite, reuse or
// mutate any of the receiver's own presets.
TEST_CASE("Published 3MF uniquifies an imported full material name on collision", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    // A legacy bundle preset literally named "Generic PLA" - collides with the stripped name.
    Preset &bare = add_inmemory_preset(bundle.filaments, "Generic PLA");
    bare.config.opt_string("filament_type", 0u) = "PLA";
    bare.config.opt_string("filament_vendor", 0u) = "Generic";
    bare.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    // The author's exact preset.
    Preset &qidi = add_inmemory_preset(bundle.filaments, "Generic PLA @Qidi Q2 0.4 nozzle");
    qidi.config.opt_string("filament_type", 0u) = "PLA";
    qidi.config.opt_string("filament_vendor", 0u) = "Generic";
    qidi.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    // The Orca library preset.
    Preset &sys = add_inmemory_preset(bundle.filaments, "Generic PLA @System");
    sys.config.opt_string("filament_type", 0u) = "PLA";
    sys.config.opt_string("filament_vendor", 0u) = "Generic";
    sys.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    bundle.filament_presets = { "My PETG" };

    PublishedMaterialEntry entry;
    entry.slot               = 0;
    entry.full               = true;
    entry.publish_type       = true;
    entry.publish_type_value = "PLA";
    entry.preset_name        = "Generic PLA @Qidi Q2 0.4 nozzle";
    entry.full_keys          = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The copy lands beside the collision, suffixed; every pre-existing preset keeps its own
    // values.
    CHECK(bundle.filament_presets[0] == "Generic PLA (Published)");
    check_double_vector(bundle.filaments.find_preset("Generic PLA (Published)", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    check_double_vector(bundle.filaments.find_preset("Generic PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
    check_double_vector(bundle.filaments.find_preset("Generic PLA @Qidi Q2 0.4 nozzle", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
    check_double_vector(bundle.filaments.find_preset("Generic PLA @System", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
    REQUIRE(pub.material_replacements.size() == 1);
    CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Generic PLA (Published)");
    CHECK(pub.skipped_keys.empty());
}

// The overlay writes onto the edited layer only when that layer survives the load (slot 0's
// preset still matches the edited preset). When the user views a non-first slot's material and
// the published entry targets that slot, the final re-select would destroy the edited layer -
// so the values must land on the stored preset instead and survive.
TEST_CASE("Published 3MF writes to the stored preset when the edited layer is re-selected away", "[Preset][Bundle][Published]")
{
    // Two-slot author config: slot 1 carries the published retraction value.
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PETG", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFT99", "GFL99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6, 0.9 };
        return config;
    };

    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    bundle.filament_presets = { "My PETG", "My PLA" };
    // The user is viewing slot 1's material.
    REQUIRE(bundle.filaments.select_preset_by_name("My PLA", false));

    PublishedMaterialEntry entry;
    entry.slot                = 1;
    entry.publish_type        = true;
    entry.publish_type_value  = "PLA";
    entry.publish_color       = true;
    entry.color               = "#ABCDEF";
    entry.keys                = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The published values survived on the stored preset (the edited layer was re-selected to
    // slot 0's material and must not have been the only copy).
    Preset *stored = bundle.filaments.find_preset("My PLA", false, true);
    REQUIRE(stored != nullptr);
    CHECK(stored->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    check_double_vector(stored->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    // The load re-selected slot 0's material, mirroring a normal project load.
    CHECK(bundle.filaments.get_edited_preset().name == "My PETG");
    // Selecting the slot's material afterwards surfaces the applied values.
    REQUIRE(bundle.filaments.select_preset_by_name("My PLA", false));
    CHECK(bundle.filaments.get_edited_preset().config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    check_double_vector(bundle.filaments.get_edited_preset().config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    CHECK(pub.skipped_keys.empty());
}

// The created copy is named after the author's preset with the "@variant" tail stripped;
// trailing whitespace left behind by the truncation must be trimmed away, and names without
// a tail pass through unchanged.
TEST_CASE("publish_material_base_name strips the variant tail from a published preset name", "[Preset][Bundle][Published]")
{
    CHECK(publish_material_base_name("Generic PLA @System") == "Generic PLA");
    CHECK(publish_material_base_name("Generic PLA @Qidi Q2 0.4 nozzle") == "Generic PLA");
    // Truncation at '@' leaves the space before the tail; it must not survive.
    CHECK(publish_material_base_name("Generic PLA  @System") == "Generic PLA");
    CHECK(publish_material_base_name("Voron Generic PLA") == "Voron Generic PLA");
    CHECK(publish_material_base_name("") == "");
    // A tail-only name strips to nothing; the caller falls back to identity fields.
    CHECK(publish_material_base_name("@System").empty());
}

// A full-published material arrives as a brand-new standalone preset: parentless, visible,
// project-embedded ("Preset Inside Project"), carrying the author's values and colour - and
// never touching any of the receiver's own presets.
TEST_CASE("Published 3MF imports a full material as a detached project-embedded preset", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    Preset &spare = add_inmemory_preset(bundle.filaments, "Spare PLA");
    spare.config.opt_string("filament_type", 0u) = "PLA";
    spare.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.4 };
    bundle.filament_presets = { "My PETG" };

    PublishedMaterialEntry entry;
    entry.slot                = 0;
    entry.full                = true;
    entry.publish_type        = true;
    entry.publish_type_value  = "PLA";
    entry.publish_color       = true;
    entry.color               = "#ABCDEF";
    entry.filament_id         = "AFL01";
    entry.setting_id          = "Sid000111222";
    entry.preset_name         = "Author PLA @Vendor";
    entry.full_keys           = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The slot lands on the freshly created copy, named after the stripped author name.
    CHECK(bundle.filament_presets[0] == "Author PLA");
    Preset *copy = bundle.filaments.find_preset("Author PLA", false, true);
    REQUIRE(copy != nullptr);
    // Detached + project-embedded contract.
    CHECK(copy->is_project_embedded);
    CHECK(copy->inherits().empty());
    CHECK(copy->setting_id.empty());
    CHECK(copy->vendor == nullptr);
    CHECK_FALSE(copy->is_system);
    CHECK_FALSE(copy->is_default);
    CHECK_FALSE(copy->is_external);
    CHECK(copy->is_visible);
    CHECK(copy->filament_id == "AFL01");
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_settings_id")->values == std::vector<std::string>{ "Author PLA" });
    // The published values and colour live on the copy.
    check_double_vector(copy->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    // Universally compatible: no printer/print restrictions survive the import.
    CHECK(copy->config.opt<ConfigOptionStrings>("compatible_printers")->values.empty());
    CHECK(copy->config.opt<ConfigOptionStrings>("compatible_prints")->values.empty());
    CHECK(copy->config.opt<ConfigOptionString>("compatible_printers_condition")->value.empty());
    CHECK(copy->config.opt<ConfigOptionString>("compatible_prints_condition")->value.empty());
    // Nothing pre-existing was touched.
    check_double_vector(bundle.filaments.find_preset("My PETG", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.6 });
    check_double_vector(bundle.filaments.find_preset("Spare PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.4 });
    CHECK(pub.skipped_keys.empty());
    REQUIRE(pub.material_replacements.size() == 1);
    CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Author PLA");
}

// Compatibility restrictions riding on the receiver's baseline preset must not leak onto the
// imported copy: a detached full material is usable with every printer and print profile.
TEST_CASE("Published 3MF clears printer restrictions on the imported full material", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &restricted = add_inmemory_preset(bundle.filaments, "Restricted PLA");
    restricted.config.opt_string("filament_type", 0u) = "PLA";
    restricted.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    restricted.config.set_key_value("compatible_printers", new ConfigOptionStrings({ "Unrelated Printer" }));
    restricted.config.set_key_value("compatible_prints", new ConfigOptionStrings({ "Unrelated Print" }));
    restricted.config.option<ConfigOptionString>("compatible_printers_condition", true)->value = "printer_settings_id==\"Nope\"";
    restricted.config.option<ConfigOptionString>("compatible_prints_condition", true)->value = "print_settings_id==\"Nope\"";
    bundle.filament_presets = { "Restricted PLA" };

    PublishedMaterialEntry entry;
    entry.slot                = 0;
    entry.full                = true;
    entry.publish_type        = true;
    entry.publish_type_value  = "PLA";
    entry.full_keys           = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The copy (named from the published type; no identity fields) has no restrictions left.
    CHECK(bundle.filament_presets[0] == "PLA");
    Preset *copy = bundle.filaments.find_preset("PLA", false, true);
    REQUIRE(copy != nullptr);
    CHECK(copy->config.opt<ConfigOptionStrings>("compatible_printers")->values.empty());
    CHECK(copy->config.opt<ConfigOptionStrings>("compatible_prints")->values.empty());
    CHECK(copy->config.opt<ConfigOptionString>("compatible_printers_condition")->value.empty());
    CHECK(copy->config.opt<ConfigOptionString>("compatible_prints_condition")->value.empty());
    // The receiver's own restricted preset keeps its restrictions.
    const Preset *original = bundle.filaments.find_preset("Restricted PLA", false, true);
    REQUIRE(original != nullptr);
    CHECK(original->config.opt<ConfigOptionStrings>("compatible_printers")->values == std::vector<std::string>{ "Unrelated Printer" });
    CHECK(original->config.opt<ConfigOptionString>("compatible_printers_condition")->value == "printer_settings_id==\"Nope\"");
    CHECK(pub.skipped_keys.empty());
}

// Identical Full materials (same setting_id + preset_name identity) share one created
// instance: an author who pointed several slots at one material gets one standalone copy,
// and the first entry's slot values win.
TEST_CASE("Published 3MF shares one imported copy between identical full slots", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "AFL01", "AFL01" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 0.8 };
        return config;
    };
    auto make_entry = [](int slot) {
        PublishedMaterialEntry entry;
        entry.slot                = slot;
        entry.full                = true;
        entry.publish_type        = true;
        entry.publish_type_value  = "PLA";
        entry.filament_id         = "AFL01";
        entry.setting_id          = "Sid000111222";
        entry.preset_name         = "Author PLA @Vendor";
        entry.full_keys           = { "filament_retraction_length" };
        return entry;
    };

    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    Preset &other = add_inmemory_preset(bundle.filaments, "Other PETG");
    other.config.opt_string("filament_type", 0u) = "PETG";
    other.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.65 };
    bundle.filament_presets = { "My PETG", "Other PETG" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_entry(0), make_entry(1) };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // Both slots point at the single shared copy; no second "(Published)" instance exists.
    REQUIRE(bundle.filament_presets.size() == 2);
    CHECK(bundle.filament_presets[0] == "Author PLA");
    CHECK(bundle.filament_presets[1] == "Author PLA");
    CHECK(bundle.filaments.find_preset("Author PLA", false, true) != nullptr);
    CHECK(bundle.filaments.find_preset("Author PLA (Published)", false, true) == nullptr);
    // The first entry's slot values won.
    check_double_vector(bundle.filaments.find_preset("Author PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    // Both slots reported, same target; originals untouched.
    REQUIRE(pub.material_replacements.size() == 2);
    CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Author PLA");
    CHECK(pub.material_replacements[1] == "slot 1: Other PETG -> Author PLA");
    check_double_vector(bundle.filaments.find_preset("My PETG", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.6 });
    check_double_vector(bundle.filaments.find_preset("Other PETG", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.65 });
    CHECK(pub.skipped_keys.empty());
}

// Without a preset name the copy falls back to the stable material id; fully anonymous
// hand-crafted entries never share instances between slots (their dedup key is slot-scoped).
TEST_CASE("Published 3MF names unidentified full materials from their fallback fields", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFT99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 1.2 };
        return config;
    };

    SECTION("empty preset_name falls back to the filament_id") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.full                = true;
        entry.publish_type        = true;
        entry.publish_type_value  = "PLA";
        entry.filament_id         = "AFL01";
        entry.full_keys           = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = published_pla_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets[0] == "AFL01");
        CHECK(bundle.filaments.find_preset("AFL01", false, true) != nullptr);
        CHECK(pub.skipped_keys.empty());
    }

    SECTION("fully anonymous entries get slot-scoped copies") {
        PresetBundle bundle;
        Preset &first = add_inmemory_preset(bundle.filaments, "First PLA");
        first.config.opt_string("filament_type", 0u) = "PLA";
        first.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        Preset &second = add_inmemory_preset(bundle.filaments, "Second PLA");
        second.config.opt_string("filament_type", 0u) = "PLA";
        second.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.55 };
        bundle.filament_presets = { "First PLA", "Second PLA" };

        PublishedMaterialEntry entry0;
        entry0.slot                = 0;
        entry0.full                = true;
        entry0.publish_type        = true;
        entry0.publish_type_value  = "ABS"; // the only naming field present
        entry0.full_keys           = { "filament_retraction_length" };
        PublishedMaterialEntry entry1 = entry0;
        entry1.slot                = 1;

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry0, entry1 };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // No shared identity: each slot gets its own uniquified copy.
        CHECK(bundle.filament_presets[0] == "ABS");
        CHECK(bundle.filament_presets[1] == "ABS (Published)");
        CHECK(bundle.filaments.find_preset("ABS", false, true) != nullptr);
        CHECK(bundle.filaments.find_preset("ABS (Published)", false, true) != nullptr);
        CHECK(pub.skipped_keys.empty());
    }
}

// Within-load dedup does not span loads: importing the same published file again into the same
// session creates a second, uniquified copy instead of mutating or reusing the first.
TEST_CASE("Re-importing a published full material uniquifies the second copy", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    bundle.filament_presets = { "My PETG" };

    auto make_entry = [] {
        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.full                = true;
        entry.publish_type        = true;
        entry.publish_type_value  = "PLA";
        entry.filament_id         = "AFL01";
        entry.setting_id          = "Sid000111222";
        entry.preset_name         = "Author PLA @Vendor";
        entry.full_keys           = { "filament_retraction_length" };
        return entry;
    };

    for (int round = 0; round < 2; ++round) {
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_entry() };
        DynamicPrintConfig config = published_pla_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        if (round == 0) {
            CHECK(bundle.filament_presets[0] == "Author PLA");
            CHECK(bundle.filaments.find_preset("Author PLA (Published)", false, true) == nullptr);
        } else {
            // The second import uniquifies beside the first instead of touching it.
            CHECK(bundle.filament_presets[0] == "Author PLA (Published)");
            check_double_vector(bundle.filaments.find_preset("Author PLA (Published)", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
            check_double_vector(bundle.filaments.find_preset("Author PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
            REQUIRE(pub.material_replacements.size() == 1);
            CHECK(pub.material_replacements[0] == "slot 0: Author PLA -> Author PLA (Published)");
        }
        CHECK(pub.skipped_keys.empty());
    }
}

// A partially-published slot can carry a curated type and/or colour. The colour is applied
// regardless of the type match; a type mismatch with no same-type replacement keeps the
// receiver's material and reports the slot's keys as skipped. The receiver's slot count grows
// only as far as the highest slot with published content.
TEST_CASE("Published 3MF partial slots apply colour and gate keys by the published type", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFT99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9, 1.2 };
        return config;
    };

    SECTION("matching type applies the keys and the colour") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.publish_type        = true;
        entry.publish_type_value  = "PLA";
        entry.publish_color       = true;
        entry.color               = "#ABCDEF";
        entry.keys                = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // Type matched: keys and colour applied onto the receiver's preset in place.
        Preset *pla_preset = bundle.filaments.find_preset("My PLA", false, true);
        REQUIRE(pla_preset != nullptr);
        check_double_vector(pla_preset->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        CHECK(pla_preset->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
        CHECK(pub.skipped_keys.empty());
        CHECK(pub.material_replacements.empty());
    }

    SECTION("type mismatch without a replacement keeps the material and skips the keys") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.publish_type        = true;
        entry.publish_type_value  = "ABS"; // no ABS in the receiver library
        entry.publish_color       = true;
        entry.color               = "#ABCDEF";
        entry.keys                = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The slot keeps the receiver's material: the colour applies in place, the keys are
        // skipped.
        CHECK(bundle.filament_presets[0] == "My PLA");
        Preset *pla_preset = bundle.filaments.find_preset("My PLA", false, true);
        REQUIRE(pla_preset != nullptr);
        check_double_vector(pla_preset->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        CHECK(pla_preset->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
        CHECK(contains_key(pub.skipped_keys, "material:ABS (filament_retraction_length)"));
        CHECK(pub.material_replacements.empty());
    }

    SECTION("receiver slot count grows to fit the highest published slot and assigns matching type preset") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.8 };
        // The receiver has a single slot; the file carries two, only slot 1 is published.
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry entry;
        entry.slot                = 1;
        entry.publish_type        = true;
        entry.publish_type_value  = "PETG";
        entry.keys                = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The slot list was grown so author slot 1 has a material (the PETG preset), which
        // then receives the author's values in place.
        REQUIRE(bundle.filament_presets.size() == 2);
        CHECK(bundle.filament_presets[1] == "My PETG");
        check_double_vector(bundle.filaments.find_preset("My PETG", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 1.2 });
        check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
    }
}

// The receiver's slot list grows only as far as the highest published slot: a file whose author
// published nothing (or only a low slot) must not pull filler materials into the receiver's
// setup, and the receiver never grows to the file's count.
TEST_CASE("Published 3MF grows the receiver's slots only as far as the published slots", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // Four author slots (a 4-filament model).
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99", "GFL99", "GFL99" };
        return config;
    };
    auto add_pla_preset = [](PresetBundle &bundle) {
        Preset &preset = add_inmemory_preset(bundle.filaments, "My PLA");
        preset.config.opt_string("filament_type", 0u) = "PLA";
        preset.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        return &preset;
    };
    auto make_color_entry = [](int slot) {
        PublishedMaterialEntry entry;
        entry.slot          = slot;
        entry.publish_color = true;
        entry.color         = "#ABCDEF";
        return entry;
    };

    // A file whose author published nothing for any slot: the receiver's setup is untouched.
    {
        PresetBundle bundle;
        add_pla_preset(bundle);
        bundle.filament_presets = { "My PLA" };

        PublishedConfig pub;
        pub.published = true; // no material entries at all
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets.size() == 1);
        CHECK(bundle.filaments.find_preset("My PLA")->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#123456" });
        CHECK(pub.skipped_keys.empty());
    }

    // Only slot 0 published: a single-slot receiver keeps its single slot; the file's other
    // three slots pull nothing in.
    {
        PresetBundle bundle;
        add_pla_preset(bundle);
        bundle.filament_presets = { "My PLA" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_color_entry(0) };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets.size() == 1);
        // The published colour lands on the slot's preset in place.
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    }

    // Slot 3 published: the receiver grows to 4 so the published slot exists. Unpublished
    // filler slots repeat the receiver's last preset ("Add one filament" behaviour).
    {
        PresetBundle bundle;
        add_pla_preset(bundle);
        bundle.filament_presets = { "My PLA" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_color_entry(3) };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 4);
        CHECK(bundle.filament_presets[1] == "My PLA");
        CHECK(bundle.filament_presets[2] == "My PLA");
        // Only "My PLA" exists in the library, so the published slot keeps the aliasing and the
        // colour is written onto the shared preset (every slot references it).
        CHECK(bundle.filament_presets[3] == "My PLA");
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
        // The project-level per-slot vectors were grown and seeded: fillers take their preset's
        // colour, the published slot its published colour.
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.size() == 4);
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[1] == "#123456");
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[3] == "#ABCDEF");
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values.size() == 4);
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour_type")->values.size() == 4);
        CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values.size() == 4);
        CHECK(bundle.project_config.opt<ConfigOptionFloats>("flush_volumes_matrix")->values.size() == 16);
    }

    // Slots 0 and 2 published: the receiver grows to 3, never to the file's 4.
    {
        PresetBundle bundle;
        add_pla_preset(bundle);
        bundle.filament_presets = { "My PLA" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_color_entry(0), make_color_entry(2) };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PLA");
    }

    // A receiver with more slots than the file's filament count keeps its setup: neither the
    // preset list nor the project-level vectors are shrunk to the file's smaller size.
    {
        PresetBundle bundle;
        add_pla_preset(bundle);
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA" };
        // Distinct project colours make a shrink observable.
        bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values       = { "#111111", "#222222", "#333333" };
        bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values = { "#111111", "#222222", "#333333" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_color_entry(0) }; // highest published slot: 0
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // A one-filament file: num_filaments (1) is below the receiver's slot count (3).
        config.opt<ConfigOptionFloats>("filament_diameter")->values          = { 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values          = { 1 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values           = { "#FF0000" };
        config.opt<ConfigOptionStrings>("filament_type")->values             = { "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values           = { "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values              = { "GFL99" };
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        // No shrink: all three project entries survive, with slot 0 synced to the published
        // colour at its unshifted index and slots 1-2 untouched.
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF", "#222222", "#333333" });
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values == std::vector<std::string>{ "#ABCDEF", "#222222", "#333333" });
        CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values.size() == 3);
        // The published colour still reached slot 0's preset in place.
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    }
}

// A published slot is seeded from an unused library preset and the values are written onto it
// in place, so the receiver's own material (slot 0) is never overwritten.
TEST_CASE("Published 3MF grows published slots to the receiver's last preset and recolors it in place", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99", "GFL99", "GFL99" };
        return config;
    };

    // Receiver with its own material plus one more library preset; author publishes only slot 4
    // (Red). Growth always repeats the receiver's last filament ("Add one filament"), so the
    // grown slot references the shared "My PLA" preset and the published red recolors it in
    // place; the unused "Other PLA" preset is left untouched.
    PresetBundle bundle;
    Preset &mine = add_inmemory_preset(bundle.filaments, "My PLA");
    mine.config.opt_string("filament_type", 0u) = "PLA";
    mine.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    Preset &other = add_inmemory_preset(bundle.filaments, "Other PLA");
    other.config.opt_string("filament_type", 0u) = "PLA";
    other.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#654321" };
    bundle.filament_presets = { "My PLA" };

    PublishedMaterialEntry entry;
    entry.slot          = 3;
    entry.publish_color = true;
    entry.color         = "#ABCDEF";
    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    REQUIRE(bundle.filament_presets.size() == 4);
    // Every grown slot (published or filler) repeats the receiver's last preset.
    CHECK(bundle.filament_presets[1] == "My PLA");
    CHECK(bundle.filament_presets[2] == "My PLA");
    // The published red recolors the shared "My PLA" preset in place; the unused "Other PLA"
    // preset is left untouched.
    CHECK(bundle.filament_presets[3] == "My PLA");
    CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    CHECK(bundle.filaments.find_preset("Other PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#654321" });
    // The project-level colours are sized and seeded for every grown slot.
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.size() == 4);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[1] == "#123456");
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[3] == "#ABCDEF");
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values.size() == 4);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour_type")->values.size() == 4);
    CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values.size() == 4);
}

// Growth always repeats the receiver's last preset; a published slot only lands on its
// material identity when the aliased grown slot is re-pointed (de-alias fires on a preset
// key). Lock the identity priority there: an exact filament_id outranks an arbitrary unused
// preset, and an entry carrying only a family constrains the pick to that family.
TEST_CASE("Published 3MF re-points an aliased grown slot by published identity or family without a type requirement", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99", "GFL99", "GFL99" };
        return config;
    };

    PublishedMaterialEntry entry;
    entry.slot            = 2;
    entry.filament_type   = "PLA";
    entry.filament_vendor = "Generic";
    entry.keys            = { "filament_retraction_length" };
    // An unused preset sorting before everything else: an unconstrained pick would take it.
    PresetBundle bundle;
    Preset &mine     = add_inmemory_preset(bundle.filaments, "My PLA");
    mine.config.opt_string("filament_type", 0u) = "PLA";
    Preset &arbitrary = add_inmemory_preset(bundle.filaments, "Aaa PLA");
    arbitrary.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets = { "My PLA" };

    SECTION("an exact filament_id outranks the first unused preset") {
        Preset &authored = add_inmemory_preset(bundle.filaments, "Zzz PLA");
        authored.config.opt_string("filament_type", 0u) = "PLA";
        authored.filament_id = "GFA00";
        entry.filament_id    = "GFA00";

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PLA");
        CHECK(bundle.filament_presets[2] == "Zzz PLA");
        // The exact-id preset outranks the type-only "Aaa PLA"; the re-point is reported.
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PLA -> Zzz PLA");
    }

    SECTION("a family-only entry picks an unused preset of that family") {
        Preset &petg = add_inmemory_preset(bundle.filaments, "Bbb PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        entry.filament_type = "PETG";

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        // The family pick lands on the only PETG preset (the PLA presets and the receiver's own
        // material lose), and the re-point is reported.
        CHECK(bundle.filament_presets[2] == "Bbb PETG");
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PLA -> Bbb PETG");
    }
}

// The GUI displays the edited preset, a snapshot of the selected collection preset taken at
// selection time. Since the overlay mutates the collection presets in place, the load must
// re-select the first slot's filament so the applied values - and slot replacements - surface
// in the GUI.
TEST_CASE("Published 3MF refreshes the edited preset so the applied material values surface", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9 };
        return config;
    };
    auto make_entry = [] {
        PublishedMaterialEntry entry;
        entry.slot                = 0;
        entry.publish_type        = true;
        entry.publish_type_value  = "PLA";
        entry.publish_color       = true;
        entry.color               = "#ABCDEF";
        entry.keys                = { "filament_retraction_length" };
        return entry;
    };

    SECTION("the edited preset carries the applied colour and keys") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        bundle.filament_presets = { "My PLA" };
        // Mirror the GUI: the displayed preset is the collection's edited preset.
        REQUIRE(bundle.filaments.select_preset_by_name("My PLA", false));

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_entry() };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        const Preset &edited = bundle.filaments.get_edited_preset();
        // The slot references the edited preset, so the overlay lands on the edited layer:
        // visible as a modification while the stored preset stays untouched.
        CHECK(edited.name == "My PLA");
        CHECK(edited.config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
        check_double_vector(edited.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#123456" });
        check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
        // The overlay is a visible, revertible modification of the edited preset.
        CHECK(bundle.filaments.current_is_dirty());
        CHECK(pub.skipped_keys.empty());
    }

    SECTION("a slot replacement is reflected in the edited preset") {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
        Preset &abs = add_inmemory_preset(bundle.filaments, "My ABS");
        abs.config.opt_string("filament_type", 0u) = "ABS";
        abs.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.3 };
        bundle.filament_presets = { "My PLA" };
        REQUIRE(bundle.filaments.select_preset_by_name("My PLA", false));

        PublishedMaterialEntry entry = make_entry();
        entry.publish_type_value     = "ABS"; // mismatch: replaced by the library's ABS
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets[0] == "My ABS");
        // The edited preset now displays the replacement with the author's values on top.
        const Preset &edited = bundle.filaments.get_edited_preset();
        CHECK(edited.name == "My ABS");
        CHECK(edited.config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
        check_double_vector(edited.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    }
}

// The overlay lands on the edited layer when the slot references the collection's edited
// preset, so the user's unsaved in-memory edits on it survive a published load (only the
// published keys are touched) and the change shows as a visible, revertible modification.
TEST_CASE("Published 3MF preserves unsaved edits on the edited filament preset", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    pla.config.opt<ConfigOptionFloatsNullable>("filament_z_hop", true)->values = { 0.1 };
    bundle.filament_presets = { "My PLA" };
    REQUIRE(bundle.filaments.select_preset_by_name("My PLA", false));

    // The user has unsaved in-memory edits on the preset being shown.
    bundle.filaments.get_edited_preset().config.opt<ConfigOptionFloatsNullable>("filament_z_hop")->values = { 0.7 };

    PublishedMaterialEntry entry;
    entry.slot                = 0;
    entry.publish_type        = true;
    entry.publish_type_value  = "PLA";
    entry.publish_color       = true;
    entry.color               = "#ABCDEF";
    entry.keys                = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // Published values land on the edited layer...
    const Preset &edited = bundle.filaments.get_edited_preset();
    CHECK(edited.name == "My PLA");
    CHECK(edited.config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    check_double_vector(edited.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    // ...the user's unsaved edit on a non-published key survives...
    check_double_vector(edited.config.opt<ConfigOptionFloatsNullable>("filament_z_hop")->values, { 0.7 });
    // ...and the stored preset is untouched.
    Preset *stored = bundle.filaments.find_preset("My PLA", false, true);
    REQUIRE(stored != nullptr);
    CHECK(stored->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#123456" });
    check_double_vector(stored->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.5 });
    check_double_vector(stored->config.opt<ConfigOptionFloatsNullable>("filament_z_hop")->values, { 0.1 });
    // The overlay is a visible, revertible modification of the edited preset.
    CHECK(bundle.filaments.current_is_dirty());
    CHECK(pub.skipped_keys.empty());
    CHECK(pub.material_replacements.empty());
}

// The published overlay must validate '#' variant indices: an out-of-range index is reported as
// skipped and must NOT resize/corrupt the receiver's vector, and a variant suffix on a scalar
// key is rejected instead of silently no-op'd.
TEST_CASE("Published 3MF rejects out-of-range vector variants and variant-suffixed scalar keys", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    // Vector key, size 2 (matches the receiver's resized vector); distinct values make the
    // applied element observable.
    config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values = { 140., 150. };
    config.opt_float("layer_height") = 0.28;
    Preset::normalize(config);

    PresetBundle bundle;
    bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values = { 10., 20. };
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.1;

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = { "wiping_volumes_extruders#5", "wiping_volumes_extruders#1", "wiping_volumes_extruders#abc", "layer_height#0" };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // In-range variant applied element-wise; the out-of-range one did not resize the vector.
    check_double_vector(bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values, { 10., 150. });
    CHECK(bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values.size() == 2);
    // Out-of-range variant, malformed variant and variant-suffixed scalar are reported as
    // skipped; the malformed one must not fall back to element 0.
    CHECK(contains_key(pub.skipped_keys, "wiping_volumes_extruders#5"));
    CHECK(contains_key(pub.skipped_keys, "wiping_volumes_extruders#abc"));
    CHECK(contains_key(pub.skipped_keys, "layer_height#0"));
    CHECK_FALSE(contains_key(pub.skipped_keys, "wiping_volumes_extruders#1"));
    // The scalar was never applied.
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.1, 0.000001));
}

// The print/printer overlay guards option types like the material pass does: a published key
// whose file-side option kind differs from the receiver's is reported as skipped instead of
// throwing ConfigurationError out of load_config_model, which would abort the whole project
// load. (Nullable variants share the type() of their non-nullable base, so this covers
// genuinely different option kinds - e.g. a string where a float vector is expected.)
TEST_CASE("Published 3MF reports type-mismatched keys as skipped instead of aborting", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    // The file carries the vector key as a string option (equal size to the receiver's)...
    config.set_key_value("wiping_volumes_extruders", new ConfigOptionStrings({ "140", "150" }));
    config.opt_float("layer_height") = 0.28;

    PresetBundle bundle;
    // ...while the receiver's edited print preset holds the float variant of the same key:
    // without the type guard, ConfigOptionVector::set() throws ConfigurationError out of
    // load_config_model.
    bundle.prints.get_edited_preset().config.set_key_value("wiping_volumes_extruders", new ConfigOptionFloats({ 10., 20. }));
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.1;

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = { "wiping_volumes_extruders", "layer_height" };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The load completes; the type-mismatched key is reported as skipped and the receiver's
    // value is untouched; the matching scalar key still applies.
    CHECK(contains_key(pub.skipped_keys, "wiping_volumes_extruders"));
    check_double_vector(bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values, { 10., 20. });
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 0.000001));
    CHECK_FALSE(contains_key(pub.skipped_keys, "layer_height"));
}

// A receiver filament preset missing its material identity (hand-edited file) must not crash
// the type gate: the gate reads it as a type mismatch, and the slot falls back to the
// "no replacement" path (keys skipped, colour still applied to the slot's preset).
TEST_CASE("Published 3MF survives a receiver preset missing its material identity", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
    config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99" };
    config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9 };
    Preset::normalize(config);

    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    // Malformed receiver preset: the identity options are missing entirely.
    pla.config.erase("filament_type");
    pla.config.erase("filament_vendor");
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    bundle.filament_presets = { "My PLA" };

    PublishedMaterialEntry entry;
    entry.filament_type        = "PLA";
    entry.filament_vendor      = "Generic";
    entry.filament_id          = "GFL99";
    entry.slot                 = 0;
    entry.publish_type         = true; // exercises the type gate against the missing identity
    // Require ABS: the receiver library (PLA-typed default preset, typeless slot preset) has no
    // ABS candidate, so the gate falls into the "no replacement" path (a PLA requirement would
    // legitimately replace the slot with the visible PLA default).
    entry.publish_type_value   = "ABS";
    entry.publish_color        = true;
    entry.color                = "#ABCDEF";
    entry.keys                 = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The gate reads the missing identity as a type mismatch; no same-type replacement exists,
    // so the keys are skipped while the colour applies to the slot's preset in place. No crash.
    CHECK(contains_key(pub.skipped_keys, "material:GFL99 (filament_retraction_length)"));
    CHECK(bundle.filament_presets[0] == "My PLA");
    Preset *mine_preset = bundle.filaments.find_preset("My PLA", false, true);
    REQUIRE(mine_preset != nullptr);
    CHECK(mine_preset->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
}

// Lock the exact contents and order of the printer allowlist (the union of the tab's
// "Retraction" and "Z-Hop" optgroup lists, Tab.cpp).
TEST_CASE("Printer publishable allowlist matches the printer tab's Retraction and Z-Hop optgroups", "[Preset][Bundle][Published]")
{
    auto keys_of = [](const std::vector<PublishablePrinterOption>& opts) {
        std::vector<std::string> keys;
        keys.reserve(opts.size());
        for (const PublishablePrinterOption& opt : opts)
            keys.emplace_back(opt.key);
        return keys;
    };

    const std::vector<std::string> expected_retraction = {
        "retraction_length", "retract_restart_extra", "retraction_speed", "deretraction_speed",
        "retraction_minimum_travel", "retract_when_changing_layer", "wipe", "wipe_distance",
        "retract_before_wipe", "retract_after_wipe"
    };
    const std::vector<std::string> expected_z_hop = {
        "retract_lift_enforce", "z_hop_types", "z_hop", "travel_slope", "retract_lift_above",
        "retract_lift_below"
    };

    CHECK(keys_of(publishable_printer_retraction_options()) == expected_retraction);
    CHECK(keys_of(publishable_printer_z_hop_options()) == expected_z_hop);

    std::set<std::string> expected_union(expected_retraction.begin(), expected_retraction.end());
    expected_union.insert(expected_z_hop.begin(), expected_z_hop.end());
    CHECK(publishable_printer_keys() == expected_union);
}

// Loading the same published file twice must not compound values on the receiver's presets:
// each load re-applies the same absolute values, so the result is idempotent.
TEST_CASE("Published 3MF reloading does not compound values on the receiver's presets", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.9 };
        return config;
    };
    auto make_entry = [] {
        PublishedMaterialEntry entry;
        entry.slot          = 0;
        entry.publish_color = true;
        entry.color         = "#ABCDEF";
        entry.keys          = { "filament_retraction_length" };
        return entry;
    };

    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    pla.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    bundle.filament_presets = { "My PLA" };

    auto load = [&] {
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_entry() };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);
    };
    load();
    // First load: the receiver's preset carries the published values (mutated in place).
    CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    load();
    // Each load re-applies the same values onto the (already mutated) preset: no accumulation.
    CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{ "#ABCDEF" });
    check_double_vector(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
}

// A receiver with several slots aliasing the same preset (multi-extruder profile with one
// filament) and an author publishing keys on several slots: each published slot is re-pointed
// at its own distinct preset so values never leak between slots.
TEST_CASE("Published 3MF gives each published slot its own preset on an aliased receiver", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard" };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99", "GFL99", "GFL99" };
        config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6, 0.9, 1.2, 1.5 };
        return config;
    };
    auto make_key_entry = [](int slot) {
        PublishedMaterialEntry entry;
        entry.slot = slot;
        entry.keys = { "filament_retraction_length" };
        return entry;
    };

    // A 4-extruder receiver with a single filament preset: the slots alias [A, A, A, A] before
    // the published pass.
    PresetBundle bundle;
    Preset &mine = add_inmemory_preset(bundle.filaments, "My PLA");
    mine.config.opt_string("filament_type", 0u) = "PLA";
    mine.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    // Spare library presets for the re-pointing to fall back on.
    for (const char *name : { "Extra PLA A", "Extra PLA B", "Extra PLA C" }) {
        Preset &extra = add_inmemory_preset(bundle.filaments, name);
        extra.config.opt_string("filament_type", 0u) = "PLA";
        extra.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    }
    bundle.filament_presets = { "My PLA", "My PLA", "My PLA", "My PLA" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_key_entry(0), make_key_entry(1), make_key_entry(2), make_key_entry(3) };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    REQUIRE(bundle.filament_presets.size() == 4);
    // Every published slot references its own distinct preset: slot 0 keeps the receiver's
    // material, slots 1-3 are re-pointed at the spare library presets.
    CHECK(bundle.filament_presets[0] == "My PLA");
    CHECK(bundle.filament_presets[1] != bundle.filament_presets[0]);
    CHECK(bundle.filament_presets[2] != bundle.filament_presets[0]);
    CHECK(bundle.filament_presets[2] != bundle.filament_presets[1]);
    CHECK(bundle.filament_presets[3] != bundle.filament_presets[0]);
    CHECK(bundle.filament_presets[3] != bundle.filament_presets[1]);
    CHECK(bundle.filament_presets[3] != bundle.filament_presets[2]);
    // Each slot's stored preset carries its own slot's retraction (mutated in place).
    const std::vector<double> expected = { 0.6, 0.9, 1.2, 1.5 };
    for (size_t slot = 0; slot < 4; ++slot) {
        Preset *preset = bundle.filaments.find_preset(bundle.filament_presets[slot], false, true);
        REQUIRE(preset != nullptr);
        check_double_vector(preset->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { expected[slot] });
    }
    CHECK(pub.skipped_keys.empty());
}

// De-aliasing runs on the exported identity even without a checked Type row: the re-pointed
// slot lands on the exact published material (by filament_id) rather than an arbitrary spare,
// and the formerly silent re-point is surfaced through the replacements notification list.
TEST_CASE("Published 3MF de-aliases an aliased slot by published identity without a type requirement", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive Standard" };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };
    config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99" };
    config.option<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6, 0.9 };

    PresetBundle bundle;
    Preset &mine = add_inmemory_preset(bundle.filaments, "My PLA");
    mine.config.opt_string("filament_type", 0u) = "PLA";
    mine.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    // A spare sorting before the exact match: an unconstrained pick would take it.
    Preset &spare = add_inmemory_preset(bundle.filaments, "Aaa PLA");
    spare.config.opt_string("filament_type", 0u) = "PLA";
    spare.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    Preset &match = add_inmemory_preset(bundle.filaments, "Zzz PLA");
    match.config.opt_string("filament_type", 0u) = "PLA";
    match.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    match.filament_id = "GFA00";
    bundle.filament_presets = { "My PLA", "My PLA" };

    PublishedMaterialEntry entry;
    entry.slot            = 1;
    entry.filament_type   = "PLA";
    entry.filament_vendor = "Generic";
    entry.filament_id     = "GFA00";
    entry.keys            = { "filament_retraction_length" };
    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    REQUIRE(bundle.filament_presets.size() == 2);
    CHECK(bundle.filament_presets[0] == "My PLA");
    CHECK(bundle.filament_presets[1] == "Zzz PLA");
    // The published key was written onto the re-pointed slot's own preset.
    Preset *target = bundle.filaments.find_preset("Zzz PLA", false, true);
    REQUIRE(target != nullptr);
    check_double_vector(target->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values, { 0.9 });
    REQUIRE(pub.material_replacements.size() == 1);
    CHECK(pub.material_replacements[0] == "slot 1: My PLA -> Zzz PLA");
    CHECK(pub.skipped_keys.empty());
}

// Printer retraction keys are published per-extruder ("#N"): a receiver with a different
// extruder count still receives the in-range elements; out-of-range variants are reported as
// skipped instead of corrupting the receiver's vector.
TEST_CASE("Published 3MF applies per-extruder printer keys across extruder-count mismatches", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        // Author has 4 extruders.
        config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.6, 0.9, 1.2, 1.5 };
        Preset::normalize(config);
        return config;
    };

    // Receiver with a single extruder: only "#0" is in range; "#1..#3" are skipped.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#0", "retraction_length#1", "retraction_length#2", "retraction_length#3" };
        DynamicPrintConfig config = make_file_config();
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.6 });
        CHECK(contains_key(pub.skipped_keys, "retraction_length#1"));
        CHECK(contains_key(pub.skipped_keys, "retraction_length#2"));
        CHECK(contains_key(pub.skipped_keys, "retraction_length#3"));
        CHECK_FALSE(contains_key(pub.skipped_keys, "retraction_length#0"));
    }

    // Receiver with four extruders and a 1-extruder author: only "#0" is published; the
    // receiver's other extruders keep their own values.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8, 0.8, 0.8, 0.8 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#0" };
        DynamicPrintConfig config = make_file_config();
        // The author's file carries a single-extruder value.
        config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.7 };
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.7, 0.8, 0.8, 0.8 });
        CHECK(pub.skipped_keys.empty());
    }
}

// A published mixed filament serializes its definition (components, ratios, gradient) into the
// receiver's project_config - the project-level parallel arrays, not a filament preset. The
// mix's own blended colour is carried as publish_color so the receiver renders the swatch.
TEST_CASE("Published 3MF applies a mixed filament definition onto the receiver's project config", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        // Three author slots: two physical PLA/PETG plus one virtual mixed slot (index 2)
        // blending slots 1 and 2 at 60/40 with a gradient.
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
            "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
        };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#0000FF", "#800080" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFT99", "GFL99" };
        // The mixed slot's definition. These keys are project-level arrays in the full config;
        // on export they are masked so only the published slot's entry survives.
        config.opt<ConfigOptionBools>("filament_is_mixed")->values = { 0, 0, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values = { "", "", "1,2" };
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "0.6,0.4" };
        config.opt<ConfigOptionBools>("filament_mixed_gradient")->values = { 0, 0, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values = { "", "", "0.9,0.1" };
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values = { "", "", "0,0.1|1,0.9" };
        config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values = { 0, 0, 1 };
        return config;
    };

    // A receiver that already carries the mix slot at index 2 as an actual mixed slot (e.g. a
    // two-physical-plus-one-mix project with the same layout): the incoming definition is a
    // like-for-like override of the virtual slot and applies in place without relocation.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
        petg.config.opt_string("filament_type", 0u) = "PETG";
        bundle.filament_presets = { "My PLA", "My PETG", "My PLA" };

        // Grow the receiver's project arrays to 3 slots first, as set_num_filaments would,
        // then mark the third slot as the receiver's own mixed filament.
        bundle.set_num_filaments(3);
        bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[2]                = 1;
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2]      = "1,1";
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] = "0.5,0.5";

        PublishedMaterialEntry mix;
        mix.filament_type   = "PLA";
        mix.filament_vendor = "Generic";
        mix.filament_id     = "GFL99";
        mix.slot            = 2;
        mix.publish_color   = true;
        mix.color           = "#800080";
        mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                                "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                                "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                                "filament_mixed_gradient_per_part" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The definition landed in project_config's parallel arrays at the author slot.
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 3);
        CHECK(is_mixed[2]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 3);
        CHECK(components[2] == "1,2");
        const auto &ratios = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values;
        REQUIRE(ratios.size() == 3);
        CHECK(ratios[2] == "0.6,0.4");
        const auto &gradient = bundle.project_config.opt<ConfigOptionBools>("filament_mixed_gradient")->values;
        CHECK(gradient[2]);
        const auto &range = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values;
        CHECK(range[2] == "0.9,0.1");
        const auto &curve = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values;
        CHECK(curve[2] == "0,0.1|1,0.9");
        const auto &per_part = bundle.project_config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values;
        CHECK(per_part[2]);
        // The mix's blended colour crossed into project_config for the swatch.
        const auto &colour = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
        REQUIRE(colour.size() == 3);
        CHECK(colour[2] == "#800080");
        // The other slots were not overwritten by the mask.
        CHECK_FALSE(is_mixed[0]);
        CHECK_FALSE(is_mixed[1]);
        // Nothing skipped: every serialized mixed key was applied.
        CHECK(pub.skipped_keys.empty());
        // Like-for-like override: no slot was relocated.
        CHECK(pub.material_replacements.empty());
    }

    // A receiver with fewer slots: the slot is grown and seeded before the definition applies.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA" };

        PublishedMaterialEntry mix;
        mix.slot = 2;
        mix.keys = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 3);
        CHECK(is_mixed[2]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 3);
        CHECK(components[2] == "1,2");
        CHECK(pub.skipped_keys.empty());
    }
}

// A published mixed filament whose definition cannot be applied is reported as skipped instead
// of aborting the load: the entry lists a mixed key that the file's payload does not carry.
TEST_CASE("Published 3MF reports an unappliable mixed filament definition as skipped", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets = { "My PLA" };

    // Two author slots (so slot 1 is in range) but the payload omits the mixed arrays: the
    // entry lists them, the file config does not.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic" };

    PublishedMaterialEntry mix;
    mix.slot = 1;
    mix.keys = { "filament_mixed_components", "filament_mixed_sublayer_ratios" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { mix };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The receiver grows to two slots; the missing payload keys are reported as skipped rather
    // than dropped silently (material_label is empty for this entry).
    REQUIRE(bundle.filament_presets.size() == 2);
    CHECK(contains_key(pub.skipped_keys, "material: (filament_mixed_components)"));
    CHECK(contains_key(pub.skipped_keys, "material: (filament_mixed_sublayer_ratios)"));
}

// A published mixed filament must never convert one of the receiver's real, physical slots
// into a virtual mix: definitions that collide with a physical slot are relocated past every
// positional (real-filament) destination, while ones colliding with an existing mixed slot
// override it in place.
TEST_CASE("Published 3MF relocates a mixed filament instead of overwriting a physical slot", "[Preset][Bundle][Published]")
{
    // An author project with <num_author_slots> slots whose last slot is a mixed filament.
    auto make_file_config = [](size_t num_author_slots, size_t num_tail_mixes = 1) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        std::vector<double> diameters(num_author_slots, 1.75);
        std::vector<int> self_index;
        std::vector<std::string> variants;
        std::vector<std::string> types;
        for (size_t i = 0; i < num_author_slots; ++i) {
            self_index.push_back(int(i + 1));
            variants.emplace_back("Direct Drive Standard");
            types.push_back(i % 2 == 0 ? "PLA" : "PETG");
        }
        config.opt<ConfigOptionFloats>("filament_diameter")->values          = diameters;
        config.opt<ConfigOptionInts>("filament_self_index")->values          = self_index;
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = variants;
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00AA00", "#0000FF", "#FFFF00", "#800080" };
        config.opt<ConfigOptionStrings>("filament_colour")->values.resize(num_author_slots, "#808080");
        config.opt<ConfigOptionStrings>("filament_type")->values     = types;
        config.opt<ConfigOptionStrings>("filament_vendor")->values.assign(num_author_slots, "Generic");
        config.opt<ConfigOptionStrings>("filament_ids")->values.resize(num_author_slots);
        // The last <num_tail_mixes> author slots are mixed ones (components differ per slot so
        // the definitions are distinguishable after relocation).
        const size_t first_mix_slot                                        = num_author_slots - num_tail_mixes;
        config.opt<ConfigOptionBools>("filament_is_mixed")->values.assign(num_author_slots, 0);
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values.assign(num_author_slots, "");
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values.assign(num_author_slots, "");
        for (size_t i = first_mix_slot; i < num_author_slots; ++i) {
            config.opt<ConfigOptionBools>("filament_is_mixed")->values[i]       = 1;
            config.opt<ConfigOptionStrings>("filament_mixed_components")->values[i] =
                i % 2 == 0 ? std::string("1,2") : std::string("1,3");
            config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[i] =
                i % 2 == 0 ? std::string("0.6,0.4") : std::string("0.3,0.7");
        }
        return config;
    };

    // The reported bug: an author publishes with physical filaments on slots 1-2 and a mixed
    // filament on slot 5; the receiver runs five real filaments of his own. Slot 5 must stay
    // untouched and the mix lands as a newly appended virtual slot 6.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA", "My PLA", "My PLA" };
        bundle.set_num_filaments(5, "#123456");
        const std::vector<std::string> receiver_colours =
            bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;

        PublishedMaterialEntry mix;
        mix.filament_type   = "PLA";
        mix.filament_vendor = "Generic";
        mix.slot            = 4;
        mix.publish_color   = true;
        mix.color           = "#800080";
        mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                                "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                                "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                                "filament_mixed_gradient_per_part" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config(5);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The receiver grew by exactly one extra virtual slot.
        REQUIRE(bundle.filament_presets.size() == 6);
        // All five physical slots kept their meaning: no mixed flag, untouched names/colours.
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 6);
        CHECK_FALSE(is_mixed[0]);
        CHECK_FALSE(is_mixed[1]);
        CHECK_FALSE(is_mixed[2]);
        CHECK_FALSE(is_mixed[3]);
        CHECK_FALSE(is_mixed[4]);
        CHECK(is_mixed[5]);
        CHECK(std::equal(receiver_colours.begin(), receiver_colours.end(),
                         bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.begin()));
        CHECK(bundle.filament_presets[0] == "My PLA");
        CHECK(bundle.filament_presets[4] == "My PLA");
        // The definition itself is readable at the new index.
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 6);
        CHECK(components[5] == "1,2");
        const auto &ratios = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values;
        REQUIRE(ratios.size() == 6);
        CHECK(ratios[5] == "0.6,0.4");
        // The blended colour seeds the swatch of the new slot only.
        const auto &colour = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
        REQUIRE(colour.size() == 6);
        CHECK(colour[5] == "#800080");
        // The relocation is surfaced to the user through the post-import notice (the de-alias
        // pass may contribute further messages, so presence is asserted, not the count).
        bool relocated_reported = false;
        for (const std::string &message : pub.material_replacements)
            if (message.find("slot 4 -> slot 5") != std::string::npos)
                relocated_reported = true;
        CHECK(relocated_reported);
        CHECK(pub.skipped_keys.empty());
    }

    // A definition colliding with the receiver's own mixed filament is overridden in place:
    // nothing grows, nothing is reported as moved.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA" };
        bundle.set_num_filaments(3);
        bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[2]                = 1;
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2]      = "1,1";
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] = "0.9,0.1";

        PublishedMaterialEntry mix;
        mix.slot            = 2;
        mix.publish_color   = true;
        mix.color           = "#800080";
        mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                                "filament_mixed_sublayer_ratios" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config(3);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets.size() == 3);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 3);
        CHECK_FALSE(is_mixed[0]);
        CHECK_FALSE(is_mixed[1]);
        CHECK(is_mixed[2]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 3);
        CHECK(components[2] == "1,2");
        const auto &ratios = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values;
        REQUIRE(ratios.size() == 3);
        CHECK(ratios[2] == "0.6,0.4");
        CHECK(pub.skipped_keys.empty());
        CHECK(pub.material_replacements.empty());
    }

    // Author publishes four physical filaments plus two mixed ones on slots 5 and 6; the
    // receiver runs five real filaments. Both mixes relocate onto consecutive fresh slots,
    // preserving their author order (slot 5 -> slot 6, slot 6 -> slot 7); no receiver slot is
    // converted into a virtual mix.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA", "My PLA", "My PLA" };
        bundle.set_num_filaments(5, "#123456");
        const std::vector<std::string> receiver_colours =
            bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;

        auto make_mix_entry = [](int authored_slot, const char *color) {
            PublishedMaterialEntry entry;
            entry.slot          = authored_slot;
            entry.publish_color = true;
            entry.color         = color;
            entry.keys          = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios" };
            return entry;
        };
        PublishedMaterialEntry mix_a = make_mix_entry(4, "#800080");
        PublishedMaterialEntry mix_b = make_mix_entry(5, "#FF69B4");

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix_a, mix_b };
        DynamicPrintConfig config = make_file_config(6, 2);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // Two fresh virtual slots were appended.
        REQUIRE(bundle.filament_presets.size() == 7);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 7);
        for (size_t i = 0; i < 5; ++i)
            CHECK_FALSE(is_mixed[i]);
        CHECK(is_mixed[5]);
        CHECK(is_mixed[6]);
        // The definitions follow their author order.
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 7);
        CHECK(components[5] == "1,2");
        CHECK(components[6] == "1,3");
        const auto &ratios = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values;
        REQUIRE(ratios.size() == 7);
        CHECK(ratios[5] == "0.6,0.4");
        CHECK(ratios[6] == "0.3,0.7");
        // The five real slots kept their colours; each mix's blended colour seeded its new slot.
        const auto &colour = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
        REQUIRE(colour.size() == 7);
        CHECK(std::equal(receiver_colours.begin(), receiver_colours.end(), colour.begin()));
        CHECK(colour[5] == "#800080");
        CHECK(colour[6] == "#FF69B4");
        // Both relocations are reported with the correct mapping.
        bool a_reported = false, b_reported = false;
        for (const std::string &message : pub.material_replacements) {
            if (message.find("slot 4 -> slot 5") != std::string::npos)
                a_reported = true;
            if (message.find("slot 5 -> slot 6") != std::string::npos)
                b_reported = true;
        }
        CHECK(a_reported);
        CHECK(b_reported);
        CHECK(pub.skipped_keys.empty());
        // The relocation table is exposed for the model-reference remapping.
        REQUIRE(pub.mixed_slot_relocations.size() == 2);
        CHECK(pub.mixed_slot_relocations.at(4) == 5);
        CHECK(pub.mixed_slot_relocations.at(5) == 6);
    }
}

// A receiver that already owns a MIXED filament must keep the physical-first invariant after a
// published-3MF import: when incoming physical filaments would land on (or ahead of) the
// receiver's mixed slot, that mix is displaced to a fresh tail slot instead of being left
// interleaved with them (the R,M,R bug).
TEST_CASE("Published 3MF relocates the receiver's mixed filament past the incoming physical slots", "[Preset][Bundle][Published]")
{
    // Build the receiver's tool-changer with three slots, the third being the receiver's own
    // mixed filament. A SEMM (single_extruder_multi_material) receiver sizes its slot list by
    // hand, so a lower slot count than the printer's nozzle count is preserved on load - a
    // non-SEMM tool-changer would top the preset list up to the nozzle count and shift the
    // expected sizes (the rebalance logic under test is the same either way).
    auto make_receiver = [](PresetBundle &bundle, const std::string &components, const std::string &ratios) {
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA" };
        bundle.set_num_filaments(3, "#123456");
        bundle.printers.get_edited_preset().config.opt<ConfigOptionBool>("single_extruder_multi_material", true)->value = true;
        bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[2]          = 1;
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2] = components;
        bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] = ratios;
        bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[2]        = "#800080";
        bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values[2] = "#800080";
    };
    auto make_real_entry = [](int slot, const char *color) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.filament_vendor = "Generic";
        entry.publish_color   = true;
        entry.color           = color;
        return entry;
    };
    // A four-physical author project with no mixed slots (colour publish only), as in the
    // reported Ferrari reference file.
    auto make_config_4_real = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
            "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
        };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#000000", "#FFFFFF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values   = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values    = { "GFL99", "GFL99", "GFL99", "GFL99" };
        return config;
    };

    // [R, R, M] + four colour-only physical slots at authored 0..3 -> [R, R, R, R, M].
    {
        PresetBundle bundle;
        make_receiver(bundle, "1,2", "0.5,0.5");

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_real_entry(0, "#FF0000"), make_real_entry(1, "#000000"),
                              make_real_entry(2, "#FFFFFF"), make_real_entry(3, "#FFFF00") };
        DynamicPrintConfig config = make_config_4_real();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The receiver grows by one extra virtual slot; the mix lands at the tail.
        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        for (size_t i = 0; i < 4; ++i)
            CHECK_FALSE(is_mixed[i]);
        CHECK(is_mixed[4]);
        // The definition travelled with its swatch colour; the vacated slot 2 became physical.
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 5);
        CHECK(components[4] == "1,2");
        CHECK(components[2].empty());
        const auto &colour = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
        REQUIRE(colour.size() == 5);
        CHECK(colour[4] == "#800080");
        CHECK(colour[2] == "#FFFFFF");
        CHECK(pub.mixed_slot_relocations.at(2) == 4);
        bool relocated_reported = false;
        for (const std::string &message : pub.material_replacements)
            if (message.find("slot 2 -> slot 4") != std::string::npos &&
                message.find("mixed filament") != std::string::npos)
                relocated_reported = true;
        CHECK(relocated_reported);
        CHECK(pub.skipped_keys.empty());
    }

    // [R, R, M] plus a payload mix authored at slot 3: the receiver mix (displaced to slot 3) sits
    // ahead of the appended payload mix (slot 4), preserving physical-first tail ordering.
    {
        PresetBundle bundle;
        make_receiver(bundle, "1,2", "0.5,0.5");

        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values         = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values         = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
            "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
        };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#000000", "#0000FF", "#800080" };
        config.opt<ConfigOptionStrings>("filament_type")->values   = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values    = { "GFL99", "GFL99", "GFL99", "GFL99" };
        // Authored slot 3 is a payload mixed definition blending slots 1 and 3.
        config.opt<ConfigOptionBools>("filament_is_mixed")->values                     = { 0, 0, 0, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values           = { "", "", "", "1,3" };
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values      = { "", "", "", "0.6,0.4" };
        config.opt<ConfigOptionBools>("filament_mixed_gradient")->values               = { 0, 0, 0, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values       = { "", "", "", "0.9,0.1" };
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values       = { "", "", "", "0,0.1|1,0.9" };
        config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values      = { 0, 0, 0, 1 };

        PublishedMaterialEntry mix;
        mix.slot          = 3;
        mix.filament_type = "PLA";
        mix.publish_color = true;
        mix.color         = "#800080";
        mix.keys          = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios",
                              "filament_mixed_gradient", "filament_mixed_gradient_range", "filament_mixed_gradient_curve",
                              "filament_mixed_gradient_per_part" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_real_entry(0, "#FF0000"), make_real_entry(1, "#000000"),
                              make_real_entry(2, "#0000FF"), mix };
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        CHECK_FALSE(is_mixed[0]);
        CHECK_FALSE(is_mixed[1]);
        CHECK_FALSE(is_mixed[2]);
        CHECK(is_mixed[3]); // receiver's mix, displaced to slot 3 first
        CHECK(is_mixed[4]); // payload's mix, appended after
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 5);
        CHECK(components[3] == "1,2");
        CHECK(components[4] == "1,3");
        REQUIRE(pub.mixed_slot_relocations.size() == 2);
        CHECK(pub.mixed_slot_relocations.at(2) == 3);
        CHECK(pub.mixed_slot_relocations.at(3) == 4);
        CHECK(pub.skipped_keys.empty());
    }
}

// Multiple receiver mixed slots interleaved with multiple incoming physical slots all rebalance
// onto consecutive tail slots in index order (no cascade/overlap).
TEST_CASE("Published 3MF rebalances several receiver mixed slots past the physical region", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets = { "My PLA", "My PLA", "My PLA" };
    bundle.set_num_filaments(3, "#123456");
    bundle.printers.get_edited_preset().config.opt<ConfigOptionBool>("single_extruder_multi_material", true)->value = true;
    // Receiver: slot 1 and slot 2 are mixed.
    bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[1]             = 1;
    bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[2]             = 1;
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[1]   = "1,2";
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2]   = "1,3";
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[1] = "0.5,0.5";
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] = "0.4,0.6";

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values         = { 1.75, 1.75, 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values         = { 1, 2, 3 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
        "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
    };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00AA00", "#0000FF" };
    config.opt<ConfigOptionStrings>("filament_type")->values   = { "PLA", "PLA", "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic" };
    config.opt<ConfigOptionStrings>("filament_ids")->values    = { "GFL99", "GFL99", "GFL99" };

    auto make_real_entry = [](int slot, const char *color) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.filament_vendor = "Generic";
        entry.publish_color   = true;
        entry.color           = color;
        return entry;
    };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_real_entry(0, "#FF0000"), make_real_entry(1, "#00AA00"), make_real_entry(2, "#0000FF") };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    REQUIRE(bundle.filament_presets.size() == 5);
    const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
    REQUIRE(is_mixed.size() == 5);
    for (size_t i = 0; i < 3; ++i)
        CHECK_FALSE(is_mixed[i]);
    CHECK(is_mixed[3]);
    CHECK(is_mixed[4]);
    const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
    REQUIRE(components.size() == 5);
    CHECK(components[3] == "1,2");
    CHECK(components[4] == "1,3");
    REQUIRE(pub.mixed_slot_relocations.size() == 2);
    CHECK(pub.mixed_slot_relocations.at(1) == 3);
    CHECK(pub.mixed_slot_relocations.at(2) == 4);
    CHECK(pub.skipped_keys.empty());
}

// The receiver's printer gates how many PHYSICAL filament slots a published 3MF may add: a
// non-SEMM tool-changer feeds filament N from nozzle N, so a published slot past the nozzle
// count cannot become a physical filament. It becomes an empty mixed-filament placeholder
// instead - a virtual tail slot the GUI flags (broken mix) and the user fills with components
// from their own filaments. SEMM receivers keep the ungated behaviour.
TEST_CASE("Published 3MF turns a surplus slot past the printer's filament capacity into an empty mixed placeholder", "[Preset][Bundle][Published]")
{
    // An author project with <num_author_slots> physical slots, no mixed ones.
    auto make_file_config = [](size_t num_author_slots) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        std::vector<double> diameters(num_author_slots, 1.75);
        std::vector<int> self_index;
        std::vector<std::string> variants;
        for (size_t i = 0; i < num_author_slots; ++i) {
            self_index.push_back(int(i + 1));
            variants.emplace_back("Direct Drive Standard");
        }
        config.opt<ConfigOptionFloats>("filament_diameter")->values          = diameters;
        config.opt<ConfigOptionInts>("filament_self_index")->values          = self_index;
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = variants;
        config.opt<ConfigOptionStrings>("filament_colour")->values.resize(num_author_slots, "#808080");
        config.opt<ConfigOptionStrings>("filament_type")->values.assign(num_author_slots, "PLA");
        config.opt<ConfigOptionStrings>("filament_vendor")->values.assign(num_author_slots, "Generic");
        config.opt<ConfigOptionStrings>("filament_ids")->values.resize(num_author_slots);
        return config;
    };
    // A non-SEMM receiver with <nozzles> nozzles running <slots> copies of one preset.
    auto make_receiver = [](PresetBundle &bundle, size_t nozzles, size_t slots) {
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets.assign(slots, "My PLA");
        bundle.set_num_filaments(slots, "#123456");
        auto &printer_config                                    = bundle.printers.get_edited_preset().config;
        printer_config.opt<ConfigOptionBool>("single_extruder_multi_material", true)->value = false;
        printer_config.opt<ConfigOptionFloats>("nozzle_diameter", true)->values.assign(nozzles, 0.4);
    };
    auto make_physical_entry = [](int slot, const char *color) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.filament_vendor = "Generic";
        entry.publish_color   = true;
        entry.color           = color;
        return entry;
    };

    // The reported case: an author publishes with a filament on slot 5; the receiver is a
    // 4-filament tool-changer. The receiver keeps its four physical slots and the surplus
    // material lands as an empty mixed placeholder at the tail.
    {
        PresetBundle bundle;
        make_receiver(bundle, 4, 4);
        const std::vector<std::string> receiver_colours =
            bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(4, "#ABCDEF") };
        DynamicPrintConfig config = make_file_config(5);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The receiver grew by exactly one virtual slot, not a fifth physical one.
        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        for (size_t i = 0; i < 4; ++i)
            CHECK_FALSE(is_mixed[i]);
        CHECK(is_mixed[4]);
        // The placeholder carries no definition: the GUI's integrity check flags it and
        // blocks slicing until the user assigns components.
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 5);
        CHECK(components[4].empty());
        // The four physical slots kept their meaning and colours.
        CHECK(std::equal(receiver_colours.begin(), receiver_colours.end(),
                         bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.begin()));
        CHECK(bundle.filament_presets[0] == "My PLA");
        CHECK(bundle.filament_presets[3] == "My PLA");
        // The published colour seeds the placeholder's swatch.
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[4] == "#ABCDEF");
        // The conversion is surfaced through the post-import notice.
        bool placeholder_reported = false;
        for (const std::string &message : pub.material_replacements)
            if (message.find("unassigned mixed filament") != std::string::npos)
                placeholder_reported = true;
        CHECK(placeholder_reported);
        CHECK(pub.skipped_keys.empty());
        CHECK(pub.mixed_slot_relocations.empty());
    }

    // Two surplus slots (5 and 6) become two consecutive empty placeholders.
    {
        PresetBundle bundle;
        make_receiver(bundle, 4, 4);

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(4, "#ABCDEF"), make_physical_entry(5, "#F0F0F0") };
        DynamicPrintConfig config = make_file_config(6);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 6);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 6);
        for (size_t i = 0; i < 4; ++i)
            CHECK_FALSE(is_mixed[i]);
        CHECK(is_mixed[4]);
        CHECK(is_mixed[5]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 6);
        CHECK(components[4].empty());
        CHECK(components[5].empty());
        const auto &colour = bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;
        REQUIRE(colour.size() == 6);
        CHECK(colour[4] == "#ABCDEF");
        CHECK(colour[5] == "#F0F0F0");
        CHECK(pub.skipped_keys.empty());
        CHECK(pub.mixed_slot_relocations.empty());
    }

    // A surplus slot past both the receiver's list and the capacity packs onto the next free
    // tail slot (never max(authored, next_free), which would grow filler physical slots past
    // the capacity), and the relocation is recorded for the model-reference remapping.
    {
        PresetBundle bundle;
        make_receiver(bundle, 2, 2);

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(3, "#ABCDEF") };
        DynamicPrintConfig config = make_file_config(4);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 3);
        CHECK_FALSE(is_mixed[0]);
        CHECK_FALSE(is_mixed[1]);
        CHECK(is_mixed[2]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 3);
        CHECK(components[2].empty());
        REQUIRE(pub.mixed_slot_relocations.size() == 1);
        CHECK(pub.mixed_slot_relocations.at(3) == 2);
        bool relocation_reported = false;
        for (const std::string &message : pub.material_replacements)
            if (message.find("slot 3 -> slot 2") != std::string::npos &&
                message.find("unassigned mixed filament") != std::string::npos)
                relocation_reported = true;
        CHECK(relocation_reported);
        CHECK(pub.skipped_keys.empty());
    }

    // A Full Publish entry past the capacity becomes a placeholder too: no standalone
    // detached copy is created for a material that got no physical slot.
    {
        PresetBundle bundle;
        make_receiver(bundle, 4, 4);

        PublishedMaterialEntry entry = make_physical_entry(4, "#ABCDEF");
        entry.full                   = true;
        entry.preset_name            = "Generic PLA @System";
        entry.filament_id            = "GFL99";
        entry.full_keys              = { "filament_retraction_length" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config(5);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        CHECK(is_mixed[4]);
        // No detached copy under the stripped name or its uniquified forms.
        CHECK(bundle.filaments.find_preset("Generic PLA", false, true) == nullptr);
        CHECK(bundle.filaments.find_preset("Generic PLA (Published)", false, true) == nullptr);
        CHECK(pub.skipped_keys.empty());
    }

    // A SEMM receiver (the default printer preset) sizes its slot list by hand: the published
    // slot past the nozzle count still grows physically, as before the capacity gate.
    {
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA", "My PLA", "My PLA", "My PLA" };
        bundle.set_num_filaments(4, "#123456");

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(4, "#ABCDEF") };
        DynamicPrintConfig config = make_file_config(5);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        for (size_t i = 0; i < 5; ++i)
            CHECK_FALSE(is_mixed[i]);
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[4] == "#ABCDEF");
        CHECK(pub.skipped_keys.empty());
    }

    // A pre-existing oversized slot list is never shrunk: a published entry pointing at one
    // of its slots is applied positionally even though the list exceeds the nozzle count.
    {
        PresetBundle bundle;
        make_receiver(bundle, 4, 5);

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(4, "#ABCDEF") };
        DynamicPrintConfig config = make_file_config(5);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        for (size_t i = 0; i < 5; ++i)
            CHECK_FALSE(is_mixed[i]);
        // The published colour reached the addressed slot's (shared) preset in place.
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values ==
              std::vector<std::string>{ "#ABCDEF" });
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[4] == "#ABCDEF");
        CHECK(pub.skipped_keys.empty());
    }

    // On a single-physical-slot receiver an empty mix could never be edited (the sidebar's
    // mixed section needs two physical filaments), so the surplus entry is dropped and
    // reported instead of becoming an unfixable placeholder.
    {
        PresetBundle bundle;
        make_receiver(bundle, 1, 1);

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { make_physical_entry(1, "#ABCDEF") };
        DynamicPrintConfig config = make_file_config(2);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        CHECK(bundle.filament_presets.size() == 1);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 1);
        CHECK_FALSE(is_mixed[0]);
        REQUIRE(pub.skipped_keys.size() == 1);
        CHECK(pub.skipped_keys.front().find("printer supports only 1") != std::string::npos);
    }

    // A payload mixed definition is exempt from the capacity gate: mixes are virtual slots
    // that consume no nozzle, so a published mix past the nozzle count still lands.
    {
        PresetBundle bundle;
        make_receiver(bundle, 4, 4);

        PublishedMaterialEntry mix;
        mix.slot          = 4;
        mix.publish_color = true;
        mix.color         = "#800080";
        mix.keys          = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios" };

        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config(5);
        config.opt<ConfigOptionBools>("filament_is_mixed")->values.assign(5, 0);
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values.assign(5, "");
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values.assign(5, "");
        config.opt<ConfigOptionBools>("filament_is_mixed")->values[4]                = 1;
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values[4]      = "1,2";
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[4] = "0.6,0.4";
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 5);
        const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
        REQUIRE(is_mixed.size() == 5);
        CHECK(is_mixed[4]);
        const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
        REQUIRE(components.size() == 5);
        CHECK(components[4] == "1,2");
        const auto &ratios = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values;
        REQUIRE(ratios.size() == 5);
        CHECK(ratios[4] == "0.6,0.4");
        CHECK(pub.skipped_keys.empty());
    }
}

// A single-extruder receiver collapses the author's per-extruder printer slots onto its single
// slot: the first serialized variant of a base key is applied, the remaining variants of that
// base key are reported as skipped.
TEST_CASE("Published 3MF collapses a multi-extruder publish onto a single-extruder receiver", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
        // Author has two extruders.
        config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.6, 0.9 };
        config.opt<ConfigOptionFloats>("retraction_speed")->values  = { 30.0, 40.0 };
        Preset::normalize(config);
        return config;
    };

    // Both extruders published: the first serialized variant (#0, left) lands on the receiver's
    // single slot; the second variant (#1) is reported as skipped.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8 };
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_speed")->values  = { 25.0 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#0", "retraction_length#1", "retraction_speed#0", "retraction_speed#1" };
        DynamicPrintConfig config = make_file_config();
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.6 });
        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_speed")->values, { 30.0 });
        CHECK(contains_key(pub.skipped_keys, "retraction_length#1"));
        CHECK(contains_key(pub.skipped_keys, "retraction_speed#1"));
        CHECK_FALSE(contains_key(pub.skipped_keys, "retraction_length#0"));
        CHECK_FALSE(contains_key(pub.skipped_keys, "retraction_speed#0"));
    }

    // Only the second extruder published: the single-extruder receiver still applies it (the
    // author's "right" is the only serialized slot) and reports nothing skipped.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#1" };
        DynamicPrintConfig config = make_file_config();
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.9 });
        CHECK(pub.skipped_keys.empty());
    }
}

// A multi-extruder "similar setup" receiver overrides each published extruder slot element-wise
// (no collapsing): each '#N' variant applies to the matching receiver slot, out-of-range ones are
// reported as skipped.
TEST_CASE("Published 3MF overrides each extruder slot on a similar multi-extruder receiver", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
        // Author has two extruders.
        config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.6, 0.9 };
        Preset::normalize(config);
        return config;
    };

    // Receiver with two extruders: both published slots override element-wise.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8, 0.8 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#0", "retraction_length#1" };
        DynamicPrintConfig config = make_file_config();
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.6, 0.9 });
        CHECK(pub.skipped_keys.empty());
    }

    // Receiver with three extruders: slots 0 and 1 override, slot 2 keeps its own value.
    {
        PresetBundle bundle;
        bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values = { 0.8, 0.8, 0.7 };
        PublishedConfig pub;
        pub.published      = true;
        pub.published_keys = { "retraction_length#0", "retraction_length#1" };
        DynamicPrintConfig config = make_file_config();
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        check_double_vector(bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("retraction_length")->values, { 0.6, 0.9, 0.7 });
        CHECK(pub.skipped_keys.empty());
    }
}

TEST_CASE("Loading incomplete mixed metadata normalizes slots before adding a filament", "[Preset][Bundle][FilamentMixer]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    const std::vector<std::string> colors = { "#000000", "#FFFFFF", "#5E5C64" };
    config.option<ConfigOptionStrings>("filament_colour")->values = colors;
    config.option<ConfigOptionBool>("single_extruder_multi_material")->value = true;
    config.option<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
    config.option<ConfigOptionStrings>("filament_settings_id", true)->values = { "Test PETG", "Test PLA", "Test TPU" };
    const std::vector<std::string> bool_keys = {
        "filament_is_mixed", "filament_mixed_gradient", "filament_mixed_gradient_per_part"
    };
    const std::vector<std::string> string_keys = {
        "filament_mixed_components", "filament_mixed_sublayer_ratios",
        "filament_mixed_gradient_range", "filament_mixed_gradient_curve"
    };
    const size_t metadata_size = GENERATE(0u, 1u, 4u);
    for (const auto &key : bool_keys) {
        if (metadata_size == 0)
            config.erase(key);
        else {
            auto &values = config.option<ConfigOptionBools>(key)->values;
            values.assign(metadata_size, false);
            if (metadata_size > colors.size())
                values.back() = true;
        }
    }
    for (const auto &key : string_keys) {
        if (metadata_size == 0)
            config.erase(key);
        else {
            auto &values = config.option<ConfigOptionStrings>(key)->values;
            values.assign(metadata_size, "");
            if (metadata_size > colors.size())
                values.back() = "stale";
        }
    }
    Preset::normalize(config);

    PresetBundle bundle;
    bundle.load_config_model("test.3mf", std::move(config), Semver());
    const auto presets = bundle.filament_presets;
    REQUIRE(presets.size() == colors.size());
    REQUIRE(presets[0] != presets[1]);
    REQUIRE(presets[1] != presets[2]);
    REQUIRE(presets[0] != presets[2]);
    for (const auto &key : bool_keys) {
        CAPTURE(key, metadata_size);
        CHECK(bundle.project_config.option<ConfigOptionBools>(key)->values ==
              std::vector<unsigned char>(colors.size(), false));
    }
    for (const auto &key : string_keys) {
        CAPTURE(key, metadata_size);
        CHECK(bundle.project_config.option<ConfigOptionStrings>(key)->values ==
              std::vector<std::string>(colors.size(), ""));
    }
    REQUIRE(bundle.num_physical_filaments() == colors.size());
    REQUIRE(bundle.num_mixed_filaments() == 0);

    bundle.set_num_filaments(bundle.num_physical_filaments() + bundle.num_mixed_filaments() + 1, "#FF0000");
    REQUIRE(bundle.filament_presets.size() == presets.size() + 1);
    const auto &actual_colors = bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values;
    REQUIRE(actual_colors.size() == colors.size() + 1);
    for (size_t i = 0; i < presets.size(); ++i) {
        CHECK(bundle.filament_presets[i] == presets[i]);
        CHECK(actual_colors[i] == colors[i]);
    }
    CHECK(bundle.num_physical_filaments() == colors.size() + 1);
    CHECK_FALSE(bundle.is_mixed_filament(colors.size()));
}

TEST_CASE("Loading a project preserves existing mixed filament definitions", "[Preset][Bundle][FilamentMixer]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.option<ConfigOptionBool>("single_extruder_multi_material")->value = true;
    config.option<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
    config.option<ConfigOptionStrings>("filament_colour")->values = { "#000000", "#FFFFFF", "#808080" };
    config.option<ConfigOptionBools>("filament_is_mixed")->values = { false, false, true };
    config.option<ConfigOptionStrings>("filament_mixed_components")->values = { "", "", "1,2" };
    config.option<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "1,2" };
    config.option<ConfigOptionBools>("filament_mixed_gradient")->values = { false, false, true };
    config.option<ConfigOptionStrings>("filament_mixed_gradient_range")->values = { "", "", "0,100" };
    config.option<ConfigOptionStrings>("filament_mixed_gradient_curve")->values = { "", "", "0,0.1|1,0.9" };
    config.option<ConfigOptionBools>("filament_mixed_gradient_per_part")->values = { false, false, true };
    Preset::normalize(config);
    const auto original = config;

    PresetBundle bundle;
    bundle.load_config_model("test.3mf", std::move(config), Semver());
    for (const auto *key : kMixedKeys) {
        CAPTURE(key);
        CHECK(*bundle.project_config.option(key) == *original.option(key));
    }
    CHECK(bundle.num_physical_filaments() == 2);
    CHECK(bundle.num_mixed_filaments() == 1);
}

TEST_CASE("Adding a filament preserves slots with incomplete mixed metadata", "[Preset][Bundle][FilamentMixer]")
{
    PresetBundle bundle;
    bundle.set_num_filaments(3u, std::string("#000000"));
    auto *colors = bundle.project_config.option<ConfigOptionStrings>("filament_colour");
    colors->values = { "#000000", "#FFFFFF", "#5E5C64" };
    bundle.filament_presets = { "Test PETG", "Test PLA", "Test TPU" };
    const auto original_presets = bundle.filament_presets;
    const auto original_colors = colors->values;
    auto *flags = bundle.project_config.option<ConfigOptionBools>("filament_is_mixed");
    flags->values = GENERATE(std::vector<unsigned char>{}, std::vector<unsigned char>{ false },
                            std::vector<unsigned char>{ false, false, false, true });
    const std::vector<std::string> string_keys = {
        "filament_mixed_components", "filament_mixed_sublayer_ratios",
        "filament_mixed_gradient_range", "filament_mixed_gradient_curve"
    };
    const std::vector<std::string> bool_keys = {
        "filament_mixed_gradient", "filament_mixed_gradient_per_part"
    };
    for (const auto &key : string_keys)
        bundle.project_config.option<ConfigOptionStrings>(key)->values = { "", "", "", "stale" };
    for (const auto &key : bool_keys)
        bundle.project_config.option<ConfigOptionBools>(key)->values = { false, false, false, true };

    REQUIRE(bundle.num_physical_filaments() == original_colors.size());
    REQUIRE(bundle.num_mixed_filaments() == 0);
    bundle.set_num_filaments(bundle.num_physical_filaments() + bundle.num_mixed_filaments() + 1, "#FF0000");

    REQUIRE(bundle.filament_presets.size() == original_presets.size() + 1);
    REQUIRE(colors->values.size() == original_colors.size() + 1);
    for (size_t i = 0; i < original_presets.size(); ++i) {
        CHECK(bundle.filament_presets[i] == original_presets[i]);
        CHECK(colors->values[i] == original_colors[i]);
    }
    CHECK(colors->values.back() == "#FF0000");
    CHECK(bundle.num_physical_filaments() == 4);
    CHECK(bundle.num_mixed_filaments() == 0);
    REQUIRE(flags->values.size() == 4);
    CHECK_FALSE(bundle.is_mixed_filament(3));
    for (const auto &key : string_keys) {
        CAPTURE(key);
        CHECK(bundle.project_config.option<ConfigOptionStrings>(key)->values ==
              std::vector<std::string>{ "", "", "", "" });
    }
    for (const auto &key : bool_keys) {
        CAPTURE(key);
        CHECK(bundle.project_config.option<ConfigOptionBools>(key)->values ==
              std::vector<unsigned char>{ false, false, false, false });
    }
}

// The nozzle-count top-up in update_multi_material_filament_presets() grows filament_presets on
// its own, so a physical count derived from that list reports a slot no per-filament array has
// yet. That is what made the extruder-count handler conclude there was nothing to add and leave
// the new sidebar combo with no colour to draw.
TEST_CASE("The physical filament count is not fooled by a lone filament_presets top-up", "[Preset][Bundle][FilamentMixer]")
{
    PresetBundle bundle;

    SECTION("no mixed slots") {
        bundle.set_num_filaments(4u, std::string("#FF0000"));
        bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter", true)->values =
            { 0.4, 0.4, 0.4, 0.4, 0.4 };
        bundle.update_multi_material_filament_presets();

        REQUIRE(bundle.filament_presets.size() == 5);   // the top-up moved this list on its own
        REQUIRE(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values.size() == 4);
        CHECK(bundle.num_physical_filaments() == 4);
    }

    SECTION("behind a mixed tail") {
        bundle.set_num_filaments(5u, std::string("#FF0000"));
        bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values =
            { false, false, false, false, true };
        bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter", true)->values =
            { 0.4, 0.4, 0.4, 0.4, 0.4, 0.4 };
        bundle.update_multi_material_filament_presets();

        REQUIRE(bundle.filament_presets.size() == 6);
        REQUIRE(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values.size() == 5);
        CHECK(bundle.num_physical_filaments() == 4);
        CHECK(bundle.num_mixed_filaments() == 1);
    }
}

// Which slots are new is a fact about the per-filament arrays, not about filament_presets, for the
// same reason. Keyed off the wrong one, a freshly opened slot silently keeps filament 1's colour.
TEST_CASE("New filament colours are placed by array position", "[Preset][Bundle][FilamentMixer]")
{
    PresetBundle bundle;
    bundle.set_num_filaments(4u, std::string("#FF0000"));
    bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter", true)->values =
        { 0.4, 0.4, 0.4, 0.4, 0.4 };
    bundle.update_multi_material_filament_presets();
    REQUIRE(bundle.filament_presets.size() == 5);
    REQUIRE(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values.size() == 4);

    // The call Sidebar::add_custom_filament makes once the extruder count opens a slot.
    bundle.set_num_filaments(5u, std::string("#00FF00"));

    const auto &colours = bundle.project_config.option<ConfigOptionStrings>("filament_colour")->values;
    REQUIRE(colours.size() == 5);
    CHECK(colours[4] == "#00FF00");   // not colours[0], which resize() would have padded with
}

// The mixed-slot flags are written into the app config on exit and read back on the next start.
// If the read side loses them the slots survive as filaments but stop being mixes, so the project
// comes back with the mix showing as an ordinary physical filament.
TEST_CASE("A saved mix is still a mix after an app restart", "[Preset][Bundle][FilamentMixer]")
{
    AppConfig app_config;

    // Last session: a 4-tool project carrying one mix of filaments 2 and 3 at the tail.
    {
        PresetBundle bundle;
        add_inmemory_preset(bundle.printers, "Test Printer");
        bundle.printers.select_preset_by_name("Test Printer", true);
        add_inmemory_preset(bundle.filaments, "Test Filament");
        bundle.filaments.select_preset_by_name("Test Filament", true);
        bundle.set_num_filaments(5u, std::string("#FF0000"));
        bundle.filament_presets.assign(5, "Test Filament");
        bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values =
            { false, false, false, false, true };
        bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values =
            { "", "", "", "", "2,3" };
        bundle.export_selections(app_config);

        REQUIRE(app_config.get_printer_setting("Test Printer", "filament_is_mixed") == "0,0,0,0,1");
    }

    // This session.
    PresetBundle bundle;
    add_inmemory_preset(bundle.printers, "Test Printer");
    add_inmemory_preset(bundle.filaments, "Test Filament");
    bundle.load_selections(app_config);

    CHECK(bundle.filament_presets.size() == 5);
    CHECK(bundle.num_mixed_filaments() == 1);
    CHECK(bundle.is_mixed_filament(4));
    CHECK(bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values[4] == "2,3");
}

// The same restart, on the printer shape that actually shows the bug: a 4-tool changer whose
// saved filament list is one longer than its nozzle count, because the extra slot is the mix.
TEST_CASE("A saved mix survives a restart on a multi-tool printer", "[Preset][Bundle][FilamentMixer]")
{
    auto make_toolchanger = [](PresetBundle &bundle) -> Preset & {
        Preset &p = add_inmemory_preset(bundle.printers, "Tool Changer");
        p.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = { 0.4, 0.4, 0.4, 0.4 };
        p.config.option<ConfigOptionBool>("single_extruder_multi_material", true)->value = false;
        return p;
    };

    AppConfig app_config;
    {
        PresetBundle bundle;
        make_toolchanger(bundle);
        bundle.printers.select_preset_by_name("Tool Changer", true);
        add_inmemory_preset(bundle.filaments, "Test Filament");
        bundle.filaments.select_preset_by_name("Test Filament", true);
        bundle.set_num_filaments(5u, std::string("#FF0000"));
        bundle.filament_presets.assign(5, "Test Filament");
        bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values =
            { false, false, false, false, true };
        bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values =
            { "", "", "", "", "1,2" };
        bundle.export_selections(app_config);
        REQUIRE(app_config.get_printer_setting("Tool Changer", "filament_is_mixed") == "0,0,0,0,1");
    }

    PresetBundle bundle;
    make_toolchanger(bundle);
    add_inmemory_preset(bundle.filaments, "Test Filament");
    bundle.load_selections(app_config);

    CHECK(bundle.filament_presets.size() == 5);
    CHECK(bundle.num_mixed_filaments() == 1);
    CHECK(bundle.is_mixed_filament(4));

    SECTION("and through the GUI startup calls that follow it") {
        // GUI_App::load_current_presets sizes the list for a non-SEMM printer, growing only.
        const size_t target = 4u + bundle.num_mixed_filaments();
        if (target > bundle.filament_presets.size())
            bundle.set_num_filaments(target);
        CHECK(bundle.num_mixed_filaments() == 1);

        // TabPrinter::extruders_count_changed.
        bundle.on_extruders_count_changed(4);
        CHECK(bundle.num_mixed_filaments() == 1);

        // Tab::select_preset re-reads the snapshot when remember_printer_config is on.
        bundle.update_selections(app_config);
        CHECK(bundle.filament_presets.size() == 5);
        CHECK(bundle.num_mixed_filaments() == 1);
        CHECK(bundle.is_mixed_filament(4));
    }
}

// The startup sizing in GUI_App::load_current_presets targets the nozzle count plus the mixes.
// That is a floor, never a ceiling: set_num_filaments() trims at the raw tail, which is exactly
// where the mixes live, so applying the target to a longer list deletes them. A list longer than
// the target is reachable - raising the extruder count without saving the printer preset leaves
// the extra physical slot behind on the next start - so the startup sizing must only ever grow.
TEST_CASE("Sizing down to the nozzle count plus mixes is what eats the mixed tail", "[Preset][Bundle][FilamentMixer]")
{
    // 5 physical + 1 mix, on a printer preset still reporting 4 nozzles.
    const size_t nozzle_count = 4;
    PresetBundle bundle;
    bundle.set_num_filaments(6u, std::string("#FF0000"));
    bundle.project_config.option<ConfigOptionBools>("filament_is_mixed")->values =
        { false, false, false, false, false, true };
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values =
        { "", "", "", "", "", "1,2" };
    REQUIRE(bundle.num_physical_filaments() == 5);

    const size_t target = nozzle_count + bundle.num_mixed_filaments();
    REQUIRE(target < bundle.filament_presets.size());

    SECTION("applied as written, the mix is gone and every slot reads physical") {
        bundle.set_num_filaments(target);

        CHECK(bundle.filament_presets.size() == target);
        CHECK(bundle.num_mixed_filaments() == 0);
        CHECK(bundle.num_physical_filaments() == target);
    }

    SECTION("applied as a floor, the mix is left alone") {
        if (target > bundle.filament_presets.size())
            bundle.set_num_filaments(target);

        CHECK(bundle.filament_presets.size() == 6);
        CHECK(bundle.num_mixed_filaments() == 1);
        CHECK(bundle.is_mixed_filament(5));
        CHECK(bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components")->values[5] == "1,2");
    }
}

// After a published-3MF import relocated mixed-filament definitions, the freshly loaded
// model's slot references must follow: object/volume "extruder" configs and multi-material
// color-painting states (which store the one-based slot number) are re-pointed to where each
// definition landed; everything else keeps its state.
TEST_CASE("remap_model_filament_slots repoints extruder configs and color painting", "[Preset][Bundle][Published]")
{
    auto make_model = [] {
        Model model;
        ModelObject *object_a = model.add_object();
        object_a->name        = "relocated mix";
        ModelVolume *vol_a    = object_a->add_volume(make_cube(10., 10., 10.));
        vol_a->config.set_key_value("extruder", new ConfigOptionInt(5)); // author slot 5 (0-based 4)
        // Author painted one facet with the mix (slot 5) and another with a physical (slot 2).
        {
            TriangleSelector selector(vol_a->mesh());
            selector.set_facet(0, EnforcerBlockerType(5));
            selector.set_facet(1, EnforcerBlockerType(2));
            vol_a->mmu_segmentation_facets.set_data(selector.serialize());
        }
        // A second object that does not reference the relocated slot at all. Painted with a
        // real, non-relocated state (NONE is never serialized: an unsplit triangle without a
        // state is the unpainted default and is skipped by TriangleSelector::serialize()).
        ModelObject *object_b = model.add_object();
        object_b->name        = "untouched";
        object_b->config.set_key_value("extruder", new ConfigOptionInt(1));
        ModelVolume *vol_b = object_b->add_volume(make_cube(5., 5., 5.));
        vol_b->config.set_key_value("extruder", new ConfigOptionInt(2));
        {
            TriangleSelector selector(vol_b->mesh());
            selector.set_facet(0, EnforcerBlockerType(2));
            vol_b->mmu_segmentation_facets.set_data(selector.serialize());
        }
        return model;
    };

    const std::map<int, int> relocations = {{4, 5}};

    Model model = make_model();
    Slic3r::remap_model_filament_slots(model, relocations);

    const ModelVolume *vol_a = model.objects[0]->volumes.front();
    CHECK(vol_a->config.extruder() == 6); // author slot 5 -> final slot 6
    // Painted states follow: the mix facet moved 5 -> 6, the physical one is untouched.
    REQUIRE(TriangleSelector::has_facets(vol_a->mmu_segmentation_facets.get_data(), EnforcerBlockerType(6)));
    REQUIRE_FALSE(TriangleSelector::has_facets(vol_a->mmu_segmentation_facets.get_data(), EnforcerBlockerType(5)));
    CHECK(TriangleSelector::has_facets(vol_a->mmu_segmentation_facets.get_data(), EnforcerBlockerType(2)));

    const ModelVolume *vol_b = model.objects[1]->volumes.front();
    CHECK(vol_b->config.extruder() == 2);
    // The untouched volume's paint (a non-relocated state) survives as-is.
    CHECK(TriangleSelector::has_facets(vol_b->mmu_segmentation_facets.get_data(), EnforcerBlockerType(2)));

    // The mapping is applied simultaneously: each entry reads the original slot number, so
    // relocating onto another relocated-from slot number must not chase chains. With the
    // 0-based relocations {3->4, 4->6} the 1-based config map is {4->5, 5->7}: a volume on
    // 1-based slot 4 lands on 5 and does NOT continue to 7.
    Model chained = make_model();
    chained.objects[0]->volumes.front()->config.set_key_value("extruder", new ConfigOptionInt(4));
    Slic3r::remap_model_filament_slots(chained, std::map<int, int>{{3, 4}, {4, 6}});
    CHECK(chained.objects[0]->volumes.front()->config.extruder() == 5);
    // The chained model's paint follows its own single-step mapping: painted state 5 -> 7,
    // and nothing lands back on 5.
    CHECK(TriangleSelector::has_facets(chained.objects[0]->volumes.front()->mmu_segmentation_facets.get_data(),
                                       EnforcerBlockerType(7)));
    CHECK_FALSE(TriangleSelector::has_facets(chained.objects[0]->volumes.front()->mmu_segmentation_facets.get_data(),
                                             EnforcerBlockerType(5)));
}

// The slot ceiling (EnforcerBlockerType::ExtruderMax) is what the color-painting encoding can
// address, so a mixed filament that does not fit must be dropped and reported instead of being
// forced onto one of the receiver's physical filaments.
TEST_CASE("Published 3MF drops a mixed filament that does not fit the slot limit and reports it", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets = { "My PLA" };
    // A receiver already at the format's slot ceiling.
    bundle.set_num_filaments(unsigned(EnforcerBlockerType::ExtruderMax), "#123456");
    const std::vector<std::string> receiver_colours =
        bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;

    PublishedMaterialEntry mix;
    mix.filament_type   = "PLA";
    mix.filament_vendor = "Generic";
    mix.slot            = int(EnforcerBlockerType::ExtruderMax) + 6; // beyond the ceiling
    mix.publish_color   = true;
    mix.color           = "#800080";
    mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                            "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                            "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                            "filament_mixed_gradient_per_part" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { mix };
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // Nothing grew and no slot became a virtual mix.
    REQUIRE(bundle.filament_presets.size() == size_t(EnforcerBlockerType::ExtruderMax));
    for (size_t i = 0; i < bundle.filament_presets.size(); ++i)
        CHECK_FALSE(bundle.is_mixed_filament(i));
    // The receiver's colours were not touched.
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == receiver_colours);
    // The mix was reported instead of being applied.
    REQUIRE(contains_key(pub.skipped_keys, "material:PLA (mixed filament definition: filament slot limit reached)"));
    CHECK(pub.material_replacements.empty());
}

// The publish dialog can only produce definitions whose components reference existing physical
// slots, so a payload whose components point at slots that do not exist (or at another mixed
// slot) or that carries fewer than two components is broken. The load reports it through the
// same channel as every other rejected input instead of shipping a mix the GUI integrity check
// would only flag later.
TEST_CASE("Published 3MF rejects a mixed filament definition with impossible components", "[Preset][Bundle][Published]")
{
    // A two-physical-plus-one-mix author file; the definition under test sits on slot 2.
    auto make_file_config = [](const std::string &components) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
            "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
        };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#0000FF", "#800080" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionBools>("filament_is_mixed")->values = { 0, 0, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values = { "", "", components };
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "0.6,0.4" };
        return config;
    };

    PublishedMaterialEntry mix;
    mix.filament_type   = "PLA";
    mix.filament_vendor = "Generic";
    mix.filament_id     = "GFL99";
    mix.slot            = 2;
    mix.publish_color   = true;
    mix.color           = "#800080";
    mix.keys            = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios" };

    for (const char *components : { "1,4", "1,3", "1" }) {
        // The claimed components: "1,4" names a slot past the final count, "1,3" names the mix
        // slot itself (1-based), "1" is not enough components to blend.
        INFO("components = " << components);
        PresetBundle bundle;
        Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
        pla.config.opt_string("filament_type", 0u) = "PLA";
        pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
        bundle.filament_presets = { "My PLA" };

        mix.keys = { "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios" };
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { mix };
        DynamicPrintConfig config = make_file_config(components);
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        // The definition was rejected, not applied: the grown slot is finalized as an empty
        // mixed placeholder instead of keeping the seeded preset and masquerading as a real
        // filament.
        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.is_mixed_filament(2));
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2].empty());
        CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2].empty());
        // Reported through the shared rejection channel.
        if (std::string(components) == "1")
            CHECK(contains_key(pub.skipped_keys, "material:GFL99 (mixed filament definition: needs at least two components)"));
        else
            CHECK(contains_key(pub.skipped_keys, "material:GFL99 (mixed filament definition: components reference missing slots)"));
        // The slot change was surfaced like the other slot adaptations.
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0].find("slot 2: mixed filament definition could not be imported") != std::string::npos);
        // The blended colour was not written into the (shared) slot preset either.
        CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values ==
              std::vector<std::string>{ "#123456" });
    }
}

// The reported scenario: an author publishes two mixed filaments whose components are
// full-published physical slots; the receiver is a smaller tool-changer, so some of those
// component slots become empty mixed placeholders. A mix whose component turned into a
// placeholder can never be valid (mixes cannot reference mixes): it is rejected, and its
// grown slot must be finalized as an empty mixed placeholder too - not keep the seeded
// preset and masquerade as a real filament carrying the mix identity.
TEST_CASE("Published 3MF finalizes a mixed filament rejected over a placeholder component as an empty placeholder", "[Preset][Bundle][Published]")
{
    // An author project: six physical slots plus two tail mixes, the second referencing the
    // sixth physical slot (H2C-style: slot 7 = 1+2, slot 8 = 2+6).
    auto make_file_config = [] {
        DynamicPrintConfig config                                              = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values            = std::vector<double>(8, 1.75);
        config.opt<ConfigOptionInts>("filament_self_index")->values            = { 1, 2, 3, 4, 5, 6, 7, 8 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values   = std::vector<std::string>(8, "Direct Drive Standard");
        config.opt<ConfigOptionStrings>("filament_colour")->values             = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00",
                                                                                   "#FF00FF", "#00FFFF", "#800080", "#804000" };
        config.opt<ConfigOptionStrings>("filament_type")->values.assign(8, "PLA");
        config.opt<ConfigOptionStrings>("filament_vendor")->values.assign(8, "Generic");
        config.opt<ConfigOptionBools>("filament_is_mixed")->values             = { 0, 0, 0, 0, 0, 0, 1, 1 };
        config.opt<ConfigOptionStrings>("filament_mixed_components")->values   = { "", "", "", "", "", "", "1,2", "2,6" };
        config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "", "", "", "", "0.6,0.4", "0.5,0.5" };
        // The export always serializes all seven masked mixed arrays, not just the ones in
        // use; the unused gradient arrays ride along as defaults.
        config.opt<ConfigOptionBools>("filament_mixed_gradient")->values       = { 0, 0, 0, 0, 0, 0, 0, 0 };
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values.assign(8, "");
        config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values.assign(8, "");
        config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values = { 0, 0, 0, 0, 0, 0, 0, 0 };
        return config;
    };
    // A non-SEMM receiver with four nozzles and four slots (tool-changer style).
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets.assign(4, "My PLA");
    bundle.set_num_filaments(4, "#123456");
    auto &printer_config = bundle.printers.get_edited_preset().config;
    printer_config.opt<ConfigOptionBool>("single_extruder_multi_material", true)->value = false;
    printer_config.opt<ConfigOptionFloats>("nozzle_diameter", true)->values.assign(4, 0.4);

    auto make_full_entry = [](int slot) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.filament_vendor = "Generic";
        entry.full            = true;
        entry.full_keys       = { "filament_retraction_length" };
        return entry;
    };
    auto make_mix_entry = [](int slot, const char *color) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.filament_vendor = "Generic";
        entry.publish_color   = true;
        entry.color           = color;
        entry.keys            = { "filament_is_mixed",       "filament_mixed_components",
                                  "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                                  "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                                  "filament_mixed_gradient_per_part" };
        return entry;
    };

    // The dialog's emit order: the full-published physical slots (1, 2, 5, 6) and both mixes.
    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_full_entry(0), make_full_entry(1), make_full_entry(4), make_full_entry(5),
                          make_mix_entry(6, "#800080"), make_mix_entry(7, "#804000") };
    DynamicPrintConfig config = make_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The receiver grew to the author's slot count, all virtual territory at the tail.
    REQUIRE(bundle.filament_presets.size() == 8);
    const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
    REQUIRE(is_mixed.size() == 8);
    // Slots 0-3 stayed physical; authored slots 5 and 6 (0-based 4 and 5) became capacity
    // placeholders.
    for (size_t i = 0; i < 4; ++i)
        CHECK_FALSE(is_mixed[i]);
    CHECK(is_mixed[4]);
    CHECK(is_mixed[5]);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[4].empty());
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[5].empty());
    // The first mix applied onto its uncontended tail slot.
    CHECK(is_mixed[6]);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[6] == "1,2");
    // The second mix was rejected - its second component (authored slot 6) turned into a
    // placeholder - and its slot was finalized as an empty mixed placeholder instead of
    // keeping the seeded preset as a phantom real filament.
    CHECK(is_mixed[7]);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[7].empty());
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[7].empty());
    REQUIRE(pub.skipped_keys.size() == 1);
    CHECK(pub.skipped_keys[0] == "material:PLA (mixed filament definition: components reference missing slots)");
    // Two Full Publish detach lines (slots 0-1), two capacity placeholder lines (slots 4-5),
    // and the rejected mix's finalization line (slot 7).
    REQUIRE(pub.material_replacements.size() == 5);
    CHECK(std::any_of(pub.material_replacements.begin(), pub.material_replacements.end(),
                      [](const std::string &line) {
                          return line.find("slot 7: mixed filament definition could not be imported") != std::string::npos;
                      }));
}

// The receiver must not overflow its physical capacity when it grows slots to reach a published
// mixed definition: an unpublished mixed slot that lands as a gap past the nozzle count becomes
// an empty mixed placeholder, not a physical filament. An author with six physical slots (0-5)
// and two tail mixes (slots 6 and 7) publishes only 0-5 and 7; the receiver has four nozzles.
// Slots 4 and 5 become surplus placeholders, slot 7 keeps its authored mix position, and the
// unpublished gap slot 6 is finalized as a virtual placeholder - never a fifth physical slot.
TEST_CASE("Published 3MF turns an unpublished gap slot past the printer's capacity into an empty mixed placeholder", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets.assign(4, "My PLA");
    bundle.set_num_filaments(4, "#123456");
    auto &printer_config                                    = bundle.printers.get_edited_preset().config;
    printer_config.opt<ConfigOptionBool>("single_extruder_multi_material", true)->value = false;
    printer_config.opt<ConfigOptionFloats>("nozzle_diameter", true)->values.assign(4, 0.4);

    // 8 authored slots: 0-5 physical, 6 unpublished mixed, 7 published mixed. The payload masks
    // the unpublished slot's mixed flag (filter_published_config), so slot 6 reads as physical.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = std::vector<double>(8, 1.75);
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4, 5, 6, 7, 8 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = std::vector<std::string>(8, "Direct Drive Standard");
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00",
                                                                   "#FF00FF", "#00FFFF", "#800080", "#804000" };
    config.opt<ConfigOptionStrings>("filament_type")->values.assign(8, "PLA");
    config.opt<ConfigOptionStrings>("filament_vendor")->values.assign(8, "Generic");
    config.opt<ConfigOptionBools>("filament_is_mixed")->values             = { 0, 0, 0, 0, 0, 0, 0, 1 };
    config.opt<ConfigOptionStrings>("filament_mixed_components")->values   = { "", "", "", "", "", "", "", "1,2" };
    config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "", "", "", "", "", "0.5,0.5" };
    config.opt<ConfigOptionBools>("filament_mixed_gradient")->values       = std::vector<unsigned char>(8, 0);
    config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values.assign(8, "");
    config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values.assign(8, "");
    config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values = std::vector<unsigned char>(8, 0);

    auto make_full_entry = [](int slot) {
        PublishedMaterialEntry entry;
        entry.slot          = slot;
        entry.filament_type = "PLA";
        entry.full          = true;
        entry.full_keys     = { "filament_retraction_length" };
        return entry;
    };
    auto make_mix_entry = [](int slot, const char *color) {
        PublishedMaterialEntry entry;
        entry.slot            = slot;
        entry.filament_type   = "PLA";
        entry.publish_color   = true;
        entry.color           = color;
        entry.keys            = { "filament_is_mixed",       "filament_mixed_components",
                                  "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                                  "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                                  "filament_mixed_gradient_per_part" };
        return entry;
    };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_full_entry(0), make_full_entry(1), make_full_entry(2), make_full_entry(3),
                          make_full_entry(4), make_full_entry(5), make_mix_entry(7, "#804000") };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    REQUIRE(bundle.filament_presets.size() == 8);
    const auto &is_mixed = bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values;
    REQUIRE(is_mixed.size() == 8);
    // Slots 0-3 stay physical; 4 and 5 are surplus placeholders; 6 is the unpublished gap; 7 is
    // the published mix. All four tail slots are virtual.
    for (size_t i = 0; i < 4; ++i)
        CHECK_FALSE(is_mixed[i]);
    CHECK(is_mixed[4]);
    CHECK(is_mixed[5]);
    CHECK(is_mixed[6]);
    CHECK(is_mixed[7]);
    // Surplus physical slots (4,5) and the unpublished gap (6) carry no definition; the
    // published mix on slot 7 keeps its own.
    const auto &components = bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values;
    REQUIRE(components.size() == 8);
    CHECK(components[4].empty());
    CHECK(components[5].empty());
    CHECK(components[6].empty());
    CHECK(components[7] == "1,2");
    // Exactly four physical slots remain (never a fifth past the nozzle count).
    size_t physical_count = 0;
    for (bool mixed : is_mixed)
        if (!mixed)
            ++physical_count;
    CHECK(physical_count == 4);
    // The unpublished gap's conversion is surfaced through the post-import notice.
    bool gap_reported = false;
    for (const std::string &message : pub.material_replacements)
        if (message.find("slot 6: unassigned mixed filament") != std::string::npos)
            gap_reported = true;
    CHECK(gap_reported);
    CHECK(pub.skipped_keys.empty());
}

// The relocation shifts cells inside the file's per-slot mixed arrays; a payload too short to
// actually carry the definition degrades to empty cells, which the definition validation then
// reports - an empty mix must not ship as a virtual slot.
TEST_CASE("Published 3MF reports a relocated mixed filament whose payload cells are missing", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    bundle.filament_presets = { "My PLA", "My PLA", "My PLA", "My PLA", "My PLA" };
    bundle.set_num_filaments(5, "#123456");
    const std::vector<std::string> receiver_colours =
        bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values;

    PublishedMaterialEntry mix;
    mix.filament_type   = "PLA";
    mix.filament_vendor = "Generic";
    mix.filament_id     = "GFL99";
    mix.slot            = 3; // authored slot 3; the receiver's five real slots occupy 0-4
    mix.publish_color   = true;
    mix.color           = "#800080";
    mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                            "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                            "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                            "filament_mixed_gradient_per_part" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { mix };
    // The file's mixed arrays only cover its single physical slot: the definition data for
    // slot 3 does not exist in the payload.
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard" };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic" };
    config.opt<ConfigOptionBools>("filament_is_mixed")->values = { 0 };
    config.opt<ConfigOptionStrings>("filament_mixed_components")->values = { "" };
    config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "" };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The mix was relocated past the physical territory...
    REQUIRE(pub.mixed_slot_relocations.size() == 1);
    CHECK(pub.mixed_slot_relocations.at(3) == 5);
    REQUIRE(pub.material_replacements.size() == 2);
    CHECK(pub.material_replacements[0].find("slot 3 -> slot 5") != std::string::npos);
    // ...and the receiver grew to hold the destination slot, but the definition itself was
    // rejected: the relocated cells degraded to empty defaults, the slot was finalized as an
    // empty mixed placeholder (not a real filament), and both facts were reported.
    REQUIRE(bundle.filament_presets.size() == 6);
    CHECK(bundle.is_mixed_filament(5));
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[5].empty());
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[5].empty());
    CHECK(contains_key(pub.skipped_keys, "material:GFL99 (mixed filament definition: needs at least two components)"));
    CHECK(pub.material_replacements[1].find("slot 5: mixed filament definition could not be imported") != std::string::npos);
    // The five real slots kept their colours.
    CHECK(std::equal(receiver_colours.begin(), receiver_colours.end(),
                     bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.begin()));
}

// A grown published slot always repeats the receiver's last preset; it can only move to the
// published material's identity when a replacement is warranted (an aliased slot that would
// otherwise leak keys, or a type mismatch). The tier priority candidate_score uses is locked
// here: exact preset name > bare name > exact setting_id > exact filament_id > vendor+type,
// with the lower tiers reported as a substitute.
TEST_CASE("Published 3MF re-points an aliased grown slot's material by identity tiers", "[Preset][Bundle][Published]")
{
    auto make_file_config = [] {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75, 1.75 };
        config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3, 4 };
        config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
            "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
        };
        config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00", "#0000FF", "#FFFF00" };
        config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PLA", "PLA", "PLA" };
        config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic", "Generic" };
        config.opt<ConfigOptionStrings>("filament_ids")->values = { "GFL99", "GFL99", "GFL99", "GFL99" };
        return config;
    };

    PublishedMaterialEntry entry;
    entry.slot          = 2;
    entry.filament_type = "PLA";

    SECTION("an exact preset name outranks the bare-name form")
    {
        PresetBundle bundle;
        Preset &mine  = add_inmemory_preset(bundle.filaments, "My PLA");
        mine.config.opt_string("filament_type", 0u) = "PLA";
        Preset &bare  = add_inmemory_preset(bundle.filaments, "Authored PLA");
        bare.config.opt_string("filament_type", 0u) = "PLA";
        Preset &exact = add_inmemory_preset(bundle.filaments, "Authored PLA @Vendor");
        exact.config.opt_string("filament_type", 0u) = "PLA";
        bundle.filament_presets = { "My PLA" };

        entry.preset_name = "Authored PLA @Vendor";
        entry.keys        = { "filament_retraction_length" };
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PLA");
        // The aliased grown slot is re-pointed at the exact-name preset; the bare-name and the
        // receiver's own preset lose.
        CHECK(bundle.filament_presets[2] == "Authored PLA @Vendor");
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PLA -> Authored PLA @Vendor");
    }

    SECTION("a bare name outranks an exact setting_id")
    {
        PresetBundle bundle;
        Preset &mine = add_inmemory_preset(bundle.filaments, "My PLA");
        mine.config.opt_string("filament_type", 0u) = "PLA";
        Preset &bare = add_inmemory_preset(bundle.filaments, "Authored PLA");
        bare.config.opt_string("filament_type", 0u) = "PLA";
        Preset &sid  = add_inmemory_preset(bundle.filaments, "Bbb PLA");
        sid.config.opt_string("filament_type", 0u) = "PLA";
        sid.setting_id = "SID123";
        bundle.filament_presets = { "My PLA" };

        entry.preset_name = "Authored PLA @Vendor"; // no library preset carries this name
        entry.setting_id  = "SID123";
        entry.keys        = { "filament_retraction_length" };
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PLA");
        CHECK(bundle.filament_presets[2] == "Authored PLA");
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PLA -> Authored PLA");
    }

    SECTION("an exact setting_id outranks an exact filament_id")
    {
        PresetBundle bundle;
        Preset &mine = add_inmemory_preset(bundle.filaments, "My PLA");
        mine.config.opt_string("filament_type", 0u) = "PLA";
        Preset &sid  = add_inmemory_preset(bundle.filaments, "Bbb PLA");
        sid.config.opt_string("filament_type", 0u) = "PLA";
        sid.setting_id = "SID123";
        Preset &fid  = add_inmemory_preset(bundle.filaments, "Ccc PLA");
        fid.config.opt_string("filament_type", 0u) = "PLA";
        fid.filament_id = "GFA00";
        bundle.filament_presets = { "My PLA" };

        entry.preset_name = "Authored PLA @Vendor"; // no library preset carries this name
        entry.setting_id  = "SID123";
        entry.filament_id = "GFA00";
        entry.keys        = { "filament_retraction_length" };
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PLA");
        CHECK(bundle.filament_presets[2] == "Bbb PLA");
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PLA -> Bbb PLA");
    }

    SECTION("a vendor+type match is reported as a substitute")
    {
        PresetBundle bundle;
        Preset &mine  = add_inmemory_preset(bundle.filaments, "My PETG");
        mine.config.opt_string("filament_type", 0u) = "PETG";
        Preset &exact_vendor = add_inmemory_preset(bundle.filaments, "Aaa PLA");
        exact_vendor.config.opt_string("filament_type", 0u) = "PLA";
        exact_vendor.config.opt_string("filament_vendor", 0u) = "Generic";
        Preset &other_vendor = add_inmemory_preset(bundle.filaments, "Zzz PLA");
        other_vendor.config.opt_string("filament_type", 0u) = "PLA";
        other_vendor.config.opt_string("filament_vendor", 0u) = "Other";
        bundle.filament_presets = { "My PETG" };

        entry.filament_vendor = "Generic"; // no name or id identity: the family tiers decide
        entry.publish_type    = true;
        entry.publish_type_value = "PLA"; // the grown slot seeds "My PETG" -> the gate reads a mismatch
        PublishedConfig pub;
        pub.published     = true;
        pub.material_keys = { entry };
        DynamicPrintConfig config = make_file_config();
        Preset::normalize(config);
        bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

        REQUIRE(bundle.filament_presets.size() == 3);
        CHECK(bundle.filament_presets[1] == "My PETG");
        // The same-vendor PLA outranks the type-only candidate...
        CHECK(bundle.filament_presets[2] == "Aaa PLA");
        // ...and since it is not an exact material match, the load says so.
        REQUIRE(pub.material_replacements.size() == 1);
        CHECK(pub.material_replacements[0] == "slot 2: My PETG -> Aaa PLA (substitute)");
    }
}

// Structural keys (identity links like filament_ids / inherits / printer_settings_id) are
// never applied onto the receiver and never reported: a hand-crafted file listing them must
// not trigger the "could not be applied" warning, while unknown keys still do.
TEST_CASE("Published 3MF silently ignores structural keys in published_keys", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    config.opt_float("layer_height") = 0.28;
    Preset::normalize(config);

    PresetBundle bundle;
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.1;
    const std::vector<std::string> ids_before =
        bundle.filaments.get_edited_preset().config.opt<ConfigOptionStrings>("filament_settings_id")->values;

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = { "filament_ids", "inherits", "printer_settings_id", "layer_height", "not_a_setting" };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The real setting applied...
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-6));
    CHECK_FALSE(contains_key(pub.skipped_keys, "layer_height"));
    // ...the structural keys were neither applied nor reported...
    CHECK_FALSE(contains_key(pub.skipped_keys, "filament_ids"));
    CHECK_FALSE(contains_key(pub.skipped_keys, "inherits"));
    CHECK_FALSE(contains_key(pub.skipped_keys, "printer_settings_id"));
    CHECK(bundle.filaments.get_edited_preset().config.opt<ConfigOptionStrings>("filament_settings_id")->values == ids_before);
    // ...while an unknown key still reports.
    CHECK(contains_key(pub.skipped_keys, "not_a_setting"));
}

// A whole-vector key requires the receiver's vector to have the same number of elements as the
// author's: pasting a 3-extruder list into a 2-extruder machine would overwrite the wrong
// elements, so the key is reported as skipped and the receiver keeps its own values.
TEST_CASE("Published 3MF skips a whole-vector key whose size does not match the receiver", "[Preset][Bundle][Published]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000" };
    config.opt_float("layer_height") = 0.28;
    // Author's wiping matrix sized for three extruders.
    config.set_key_value("wiping_volumes_extruders", new ConfigOptionFloats({ 10., 20., 30. }));
    Preset::normalize(config);

    PresetBundle bundle;
    // Receiver sized for two extruders.
    bundle.prints.get_edited_preset().config.set_key_value("wiping_volumes_extruders", new ConfigOptionFloats({ 40., 50. }));
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.1;

    PublishedConfig pub;
    pub.published      = true;
    pub.published_keys = { "wiping_volumes_extruders", "layer_height" };
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    check_double_vector(bundle.prints.get_edited_preset().config.opt<ConfigOptionFloats>("wiping_volumes_extruders")->values,
                        { 40., 50. });
    CHECK(contains_key(pub.skipped_keys, "wiping_volumes_extruders"));
    // The matching scalar key still applied.
    CHECK_FALSE(contains_key(pub.skipped_keys, "layer_height"));
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-6));
}

// The uniquify chain continues past the first suffix: with both "X" and "X (Published)" already
// present, the next imported copy of "X" lands as "X (Published 2)" and leaves the others alone.
TEST_CASE("Published 3MF uniquifies a second imported full material as (Published 2)", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    petg.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.6 };
    Preset &bare = add_inmemory_preset(bundle.filaments, "Generic PLA");
    bare.config.opt_string("filament_type", 0u) = "PLA";
    bare.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    Preset &pub1 = add_inmemory_preset(bundle.filaments, "Generic PLA (Published)");
    pub1.config.opt_string("filament_type", 0u) = "PLA";
    pub1.config.opt<ConfigOptionFloatsNullable>("filament_retraction_length", true)->values = { 0.5 };
    bundle.filament_presets = { "My PETG" };

    PublishedMaterialEntry entry;
    entry.slot               = 0;
    entry.full               = true;
    entry.publish_type       = true;
    entry.publish_type_value = "PLA";
    entry.preset_name        = "Generic PLA @Qidi Q2 0.4 nozzle";
    entry.full_keys          = { "filament_retraction_length" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { entry };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    CHECK(bundle.filament_presets[0] == "Generic PLA (Published 2)");
    check_double_vector(bundle.filaments.find_preset("Generic PLA (Published 2)", false, true)
                            ->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values,
                        { 0.9 });
    check_double_vector(bundle.filaments.find_preset("Generic PLA", false, true)
                            ->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values,
                        { 0.5 });
    check_double_vector(bundle.filaments.find_preset("Generic PLA (Published)", false, true)
                            ->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values,
                        { 0.5 });
    REQUIRE(pub.material_replacements.size() == 1);
    CHECK(pub.material_replacements[0] == "slot 0: My PETG -> Generic PLA (Published 2)");
    CHECK(pub.skipped_keys.empty());
}

// A mixed filament's blended colour is a swatch for the project's colour strip only: it must
// never be written into the slot's (possibly shared) preset config, or every slot referencing
// that preset would turn into the blend colour.
TEST_CASE("Published 3MF never writes a mixed filament's blended colour into the slot's preset", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#123456" };
    Preset &petg = add_inmemory_preset(bundle.filaments, "My PETG");
    petg.config.opt_string("filament_type", 0u) = "PETG";
    bundle.filament_presets = { "My PLA", "My PLA", "My PLA" };
    bundle.set_num_filaments(3);
    bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values[2]                = 1;
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2]      = "1,1";
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] = "0.5,0.5";

    PublishedMaterialEntry mix;
    mix.filament_type   = "PLA";
    mix.filament_vendor = "Generic";
    mix.filament_id     = "GFL99";
    mix.slot            = 2;
    mix.publish_color   = true;
    mix.color           = "#800080";
    mix.keys            = { "filament_is_mixed",       "filament_mixed_components",
                            "filament_mixed_sublayer_ratios", "filament_mixed_gradient",
                            "filament_mixed_gradient_range",  "filament_mixed_gradient_curve",
                            "filament_mixed_gradient_per_part" };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { mix };
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 2, 3 };
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = {
        "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
    };
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#0000FF", "#800080" };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA", "PETG", "PLA" };
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { "Generic", "Generic", "Generic" };
    config.opt<ConfigOptionBools>("filament_is_mixed")->values = { 0, 0, 1 };
    config.opt<ConfigOptionStrings>("filament_mixed_components")->values = { "", "", "1,2" };
    config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values = { "", "", "0.6,0.4" };
    config.opt<ConfigOptionBools>("filament_mixed_gradient")->values = { 0, 0, 1 };
    config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values = { "", "", "0.9,0.1" };
    config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values = { "", "", "0,0.1|1,0.9" };
    config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values = { 0, 0, 1 };
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The definition applied like-for-like onto the virtual slot...
    CHECK(bundle.filament_presets.size() == 3);
    CHECK(bundle.is_mixed_filament(2));
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values[2] == "1,2");
    // ...the blend colour landed in the project strip only...
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[2] == "#800080");
    // ...and the shared preset kept its own colour: slots 0 and 1 render unchanged.
    CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{ "#123456" });
    CHECK(bundle.filament_presets[0] == "My PLA");
    CHECK(bundle.filament_presets[1] == "My PLA");
    CHECK(pub.skipped_keys.empty());
    CHECK(pub.material_replacements.empty());
}

// Duplicate entries for the same authored slot only occur in hand-crafted files (the dialog
// emits one entry per slot); the load's contract under that input is deterministic last-wins,
// not corruption.
TEST_CASE("Published 3MF applies duplicate entries for one slot last-wins", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    Preset &pla = add_inmemory_preset(bundle.filaments, "My PLA");
    pla.config.opt_string("filament_type", 0u) = "PLA";
    pla.config.opt<ConfigOptionStrings>("filament_colour", true)->values = { "#000000" };
    bundle.filament_presets = { "My PLA" };

    auto make_entry = [](const char *color) {
        PublishedMaterialEntry entry;
        entry.slot          = 0;
        entry.publish_color = true;
        entry.color         = color;
        entry.keys          = { "filament_retraction_length" };
        return entry;
    };

    PublishedConfig pub;
    pub.published     = true;
    pub.material_keys = { make_entry("#AA0000"), make_entry("#BB0000") };
    DynamicPrintConfig config = published_pla_file_config();
    Preset::normalize(config);
    bundle.load_config_model("test.3mf", std::move(config), Semver(), &pub);

    // The second entry won both the project strip and the slot's preset.
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values[0] == "#BB0000");
    CHECK(bundle.filaments.find_preset("My PLA", false, true)->config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{ "#BB0000" });
    check_double_vector(bundle.filaments.find_preset("My PLA", false, true)
                            ->config.opt<ConfigOptionFloatsNullable>("filament_retraction_length")->values,
                        { 0.9 });
    CHECK(pub.skipped_keys.empty());
    CHECK(pub.material_replacements.empty());
}

// normalize_filament_type maps "PLA High Speed" onto the canonical family "PLA" (a space-
// separated modifier is dropped) but leaves dash-separated composite types like "PA-CF" intact,
// and passes through unknown types and the empty string unchanged.
TEST_CASE("normalize_filament_type strips a space modifier but keeps dash types", "[Preset][Bundle][Published]")
{
    CHECK(normalize_filament_type("PLA High Speed") == "PLA");
    CHECK(normalize_filament_type("PA-CF") == "PA-CF");
    CHECK(normalize_filament_type("PETG-CF") == "PETG-CF");
    CHECK(normalize_filament_type("PLA") == "PLA");
    CHECK(normalize_filament_type("ABC") == "ABC");
    CHECK(normalize_filament_type("") == "");
}

// collect_dirty_settings_keys feeds the Publish dialog's pre-check: it must be the set union of
// the dirty options across the edited print, printer and filament presets.
TEST_CASE("collect_dirty_settings_keys unions the dirty settings from all three presets", "[Preset][Bundle][Published]")
{
    PresetBundle bundle;
    // The edited preset is initialised as a copy of the selected (default) preset, so a single
    // edit makes exactly that option dirty. deep_diff reports scalar keys by name but per-element
    // vector keys as "key#<index>", so a vector edit surfaces as "key#0".
    bundle.prints.get_edited_preset().config.opt_float("layer_height") = 0.28;
    bundle.filaments.get_edited_preset().config.opt<ConfigOptionStrings>("filament_type", true)->values = { "ABS" };
    bundle.printers.get_edited_preset().config.opt<ConfigOptionFloats>("nozzle_diameter", true)->values = { 0.6 };

    const std::vector<std::string> dirty = collect_dirty_settings_keys(bundle);
    for (const char *key : { "layer_height", "filament_type#0", "nozzle_diameter#0" })
        CHECK(contains_key(dirty, key));
}

// The publish denylist and the mixed-key list are single sources of truth for the import path:
// lock their members so a silent edit to either cannot drift away from the contract the import
// and export masks rely on.
TEST_CASE("Published 3MF denylist and mixed-key sets match the import/export contract", "[Preset][Bundle][Published]")
{
    const std::set<std::string>& structural = publish_structural_keys();
    // Structural / inheritance keys must never be applied onto a receiver's presets.
    for (const char *key : { "printer_settings_id", "filament_settings_id", "print_settings_id",
                             "compatible_printers", "compatible_prints", "compatible_printers_condition",
                             "compatible_prints_condition", "default_filament_profile", "default_print_profile",
                             "inherits", "extruder_count", "printer_model", "filament_ids" })
        CHECK(structural.count(key) == 1);
    // ...but a per-slot publishable material key is not structural.
    CHECK(structural.count("filament_retraction_length") == 0);
    CHECK(structural.count("filament_colour") == 0);

    const std::set<std::string>& mixed = publish_mixed_keys();
    CHECK(mixed == std::set<std::string>{
        "filament_is_mixed", "filament_mixed_components", "filament_mixed_sublayer_ratios",
        "filament_mixed_gradient", "filament_mixed_gradient_range", "filament_mixed_gradient_curve",
        "filament_mixed_gradient_per_part" });
    // Mixed keys are project-level arrays, not material-preset keys, so none is structural.
    for (const std::string &key : mixed)
        CHECK(structural.count(key) == 0);
}

namespace {

// data_dir() is a process-wide global that import_presets extracts into; scope it to the test.
struct ScopedDataDir
{
    std::string previous = data_dir();
    explicit ScopedDataDir(const fs::path &dir) { set_data_dir(dir.string()); }
    ~ScopedDataDir() { set_data_dir(previous); }
};

// resources_dir() is process-wide too; system preset lookups scan its profiles directory.
struct ScopedResourcesDir
{
    std::string previous = resources_dir();
    explicit ScopedResourcesDir(const fs::path &dir) { set_resources_dir(dir.string()); }
    ~ScopedResourcesDir() { set_resources_dir(previous); }
};

// An "Acme" vendor under root whose "Acme Printer" inherits extruder_clearance_dist_to_rod from an
// abstract base, with the printer in a nested sub_path so the name cannot be derived from the file.
void write_acme_printer_vendor(const fs::path &root, double dist_to_rod)
{
    const fs::path machine_dir = root / "Acme" / "machine";
    fs::create_directories(machine_dir / "nested");
    std::ofstream((root / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme",)"
        << R"("machine_model_list":[{"name":"Acme One","sub_path":"machine/model.json"}],"machine_list":[)"
        << R"({"name":"fdm_acme_common","sub_path":"machine/base.json"},)"
        << R"({"name":"Acme Printer","sub_path":"machine/nested/printer.json"}]})";
    std::ofstream((machine_dir / "model.json").string())
        << R"({"type":"machine_model","name":"Acme One","nozzle_diameter":"0.4"})";
    std::ofstream((machine_dir / "base.json").string())
        << R"({"type":"machine","name":"fdm_acme_common","from":"system","instantiation":"false",)"
        << R"("extruder_clearance_dist_to_rod":")" << dist_to_rod << R"("})";
    std::ofstream((machine_dir / "nested" / "printer.json").string())
        << R"({"type":"machine","name":"Acme Printer","from":"system","instantiation":"true","inherits":"fdm_acme_common",)"
        << R"("printer_model":"Acme One","printer_variant":"0.4"})";
}

std::string read_file(const fs::path &file)
{
    std::ifstream in(file.string(), std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

void write_zip(const fs::path &zip_file, const std::vector<std::pair<std::string, std::string>> &entries)
{
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    REQUIRE(open_zip_writer(&zip, zip_file.string()));
    for (const auto &[name, content] : entries)
        REQUIRE(mz_zip_writer_add_mem(&zip, name.c_str(), content.data(), content.size(), MZ_DEFAULT_COMPRESSION));
    REQUIRE(mz_zip_writer_finalize_archive(&zip));
    REQUIRE(close_zip_writer(&zip));
}

bool any_filename_contains(const fs::path &root, const std::string &needle)
{
    for (fs::recursive_directory_iterator it(root), end; it != end; ++it)
        if (it->path().filename().string().find(needle) != std::string::npos)
            return true;
    return false;
}

} // namespace

TEST_CASE("Vendor validation checks nozzle diameters while preserving loadable profiles",
          "[PresetBundle][VendorVariantValidation]")
{
    auto [variant, diameters, matches] = GENERATE(table<std::string, std::vector<double>, bool>({
        { "0.4",     { 0.4 },      true },
        { "0.8HF",   { 0.8 },      true },
        { "0.4+0.6", { 0.4, 0.6 }, true },
        { "0.4",     { 0.6 },      false },
        { "0.4+0.6", { 0.4 },      false },
        { "HF",      { 0.4 },      false },
    }));
    CAPTURE(variant, diameters, matches);
    ScopedTemporaryDir temp("vendor-variant-validation");
    ScopedDataDir data(temp.path() / "data");
    const fs::path root = temp.path() / "profiles";
    const fs::path machine_directory = root / "VariantVendor" / "machine";
    fs::create_directories(machine_directory);
    const auto write = [](const fs::path &path, const nlohmann::json &document) {
        std::ofstream output(path.string());
        REQUIRE(output.good());
        output << document.dump();
        REQUIRE(output.good());
    };
    write(root / "VariantVendor.json", {
        { "version", "1.0.0" }, { "name", "VariantVendor" },
        { "machine_model_list", {{{ "name", "Variant Machine" }, { "sub_path", "machine/model.json" }}} },
        { "machine_list", {{{ "name", "Variant Printer" }, { "sub_path", "machine/printer.json" }}} },
    });
    write(machine_directory / "model.json", {
        { "type", "machine_model" }, { "name", "Variant Machine" }, { "nozzle_diameter", variant },
    });
    nlohmann::json nozzle_values = nlohmann::json::array();
    for (double diameter : diameters)
        nozzle_values.push_back(std::to_string(diameter));
    write(machine_directory / "printer.json", {
        { "type", "machine" }, { "name", "Variant Printer" }, { "from", "system" },
        { "instantiation", "true" }, { "printer_model", "Variant Machine" },
        { "printer_variant", variant }, { "nozzle_diameter", nozzle_values },
    });

    for (bool validation_mode : { false, true }) {
        DYNAMIC_SECTION("validation mode " << validation_mode) {
            PresetBundle bundle;
            bundle.set_is_validation_mode(validation_mode);
            const auto loaded = bundle.load_vendor_configs_from_json(
                root.string(), "VariantVendor", PresetBundle::LoadSystem,
                ForwardCompatibilitySubstitutionRule::Disable, nullptr, false);
            REQUIRE(loaded.second == 1);
            const Preset *printer = bundle.printers.find_preset("Variant Printer", false);
            REQUIRE(printer != nullptr);
            CHECK(printer->is_system);
            CHECK(printer->config.opt_string("printer_variant") == variant);
            const auto *nozzles = printer->config.option<ConfigOptionFloats>("nozzle_diameter");
            REQUIRE(nozzles != nullptr);
            REQUIRE(nozzles->values.size() == diameters.size());
            for (size_t i = 0; i < diameters.size(); ++i)
                CHECK_THAT(nozzles->values[i], Catch::Matchers::WithinAbs(diameters[i], 1e-9));
            const int expected_errors = validation_mode && !matches ? 1 : 0;
            CHECK(bundle.error_count() == expected_errors);
        }
    }
}

TEST_CASE("Config import confines zip entries, preset names and bundle ids to the preset directory", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir temp_dir;
    const fs::path     data_root = temp_dir.path() / "datadir";
    const fs::path     src_dir   = temp_dir.path() / "src";
    fs::create_directories(src_dir);
    ScopedDataDir scoped_data_dir(data_root);

    PresetBundle bundle;
    AppConfig    app_config;
    const auto   confirm = [](std::string const &) { return 1; };
    const auto   import  = [&](const fs::path &file) {
        std::vector<std::string> files{file.string()};
        bundle.import_presets(files, confirm, ForwardCompatibilitySubstitutionRule::Disable, app_config);
        return files;
    };

    const fs::path good_file = src_dir / "Good.json";
    write_print_preset(bundle.prints.default_preset().config, good_file, "Good");
    const std::string good_json = read_file(good_file);

    // Four levels up from where import_presets writes (<datadir>/user/default/temp) is temp_dir
    // itself, so anything that escapes lands where the scan below can see it.
    const std::string up     = "../../../../";
    const std::string up_win = "..\\..\\..\\..\\";

    SECTION("zip entry names with either separator are reduced to a basename") {
        const fs::path zip = src_dir / "bundle.zip";
        write_zip(zip, {{up + "zip-escape.json", "{}"}, {up_win + "zip-escape.json", "{}"}, {"presets/Good.json", good_json}});
        import(zip);
        CHECK(bundle.prints.find_preset("Good") != nullptr);
        CHECK_FALSE(any_filename_contains(temp_dir.path(), "zip-escape"));
    }

    SECTION("a preset name that walks out of the preset directory is rejected") {
        for (const std::string &name : {up + "name-escape", up_win + "name-escape"}) {
            const fs::path file = src_dir / "escape.json";
            write_print_preset(bundle.prints.default_preset().config, file, name);
            CHECK(import(file).empty());
            CHECK_FALSE(any_filename_contains(temp_dir.path(), "name-escape"));
        }
    }

    SECTION("a bundle id that walks out of the bundle directory is rejected") {
        const fs::path zip = src_dir / "bundle.zip";
        write_zip(zip, {{BUNDLE_STRUCTURE_JSON_NAME, "{\"id\": \"" + up + "bundle-escape\"}"}, {"Good.json", good_json}});
        CHECK(import(zip).empty());
        CHECK_FALSE(any_filename_contains(temp_dir.path(), "bundle-escape"));
    }
}

// A project saved before a key joined filament_options_with_variant stores it once per filament,
// while the keys that were already per variant store it once per filament variant. Loading such a
// project gives every variant of a filament that filament's value.
TEST_CASE("A project saved with pressure advance per filament applies it to every variant of the filament", "[Preset][Bundle]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionStrings>("filament_colour")->values = { "#FF0000", "#00FF00" };
    config.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
    config.option<ConfigOptionStrings>("filament_settings_id", true)->values = { "Project PLA", "Project PETG" };
    // A multi-variant printer: full_print_config() leaves the list out, and the loader splits the
    // variant keys per filament only when the project carries it.
    config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = { "Direct Drive Standard,Direct Drive High Flow" };
    // filament 1 defines Standard and High Flow, filament 2 Standard
    config.opt<ConfigOptionStrings>("filament_extruder_variant")->values = { "Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard" };
    config.opt<ConfigOptionInts>("filament_self_index")->values = { 1, 1, 2 };
    config.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values = { 0.95, 0.96, 0.97 }; // one per filament variant
    // One per filament, read through the loader that project files go through.
    config.load_from_ini_string("pressure_advance = 0.021,0.043", ForwardCompatibilitySubstitutionRule::Disable);
    // The CLI slices the config as loaded.
    check_double_vector(config.opt<ConfigOptionFloats>("pressure_advance")->values, { 0.021, 0.021, 0.043 });
    check_double_vector(config.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values, { 0.95, 0.96, 0.97 });
    // The GUI normalizes the config before load; mirror that so only the production path runs.
    Preset::normalize(config);

    PresetBundle bundle;
    bundle.load_config_model("test.3mf", std::move(config));

    REQUIRE(bundle.filament_presets.size() == 2);
    const DynamicPrintConfig &pla  = bundle.filaments.find_preset(bundle.filament_presets[0], false, true)->config;
    const DynamicPrintConfig &petg = bundle.filaments.find_preset(bundle.filament_presets[1], false, true)->config;
    check_double_vector(pla.opt<ConfigOptionFloats>("pressure_advance")->values, { 0.021, 0.021 });
    check_double_vector(petg.opt<ConfigOptionFloats>("pressure_advance")->values, { 0.043 });
    check_double_vector(pla.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values, { 0.95, 0.96 });
    check_double_vector(petg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values, { 0.97 });
}

TEST_CASE("A system preset resolves by name from the bundled profiles", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    ScopedDataDir      data(temp_dir.path() / "data");
    ScopedResourcesDir resources(temp_dir.path() / "resources");
    write_acme_printer_vendor(temp_dir.path() / "resources" / "profiles", 33.);

    PresetBundle       bundle;
    DynamicPrintConfig config;
    std::string        error;
    REQUIRE(bundle.resolve_system_preset(config, Preset::TYPE_PRINTER, "Acme Printer",
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK(error.empty());
    CHECK_THAT(config.opt_float("extruder_clearance_dist_to_rod"), Catch::Matchers::WithinAbs(33., 1e-6));
}

TEST_CASE("A system preset resolves from the data directory copy of its vendor", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    ScopedDataDir      data(temp_dir.path() / "data");
    ScopedResourcesDir resources(temp_dir.path() / "resources");
    write_acme_printer_vendor(temp_dir.path() / "resources" / "profiles", 33.);
    write_acme_printer_vendor(temp_dir.path() / "data" / PRESET_SYSTEM_DIR, 35.);

    PresetBundle       bundle;
    DynamicPrintConfig config;
    std::string        error;
    REQUIRE(bundle.resolve_system_preset(config, Preset::TYPE_PRINTER, "Acme Printer",
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_THAT(config.opt_float("extruder_clearance_dist_to_rod"), Catch::Matchers::WithinAbs(35., 1e-6));
}

TEST_CASE("A system preset resolves from a vendor shipped as its preset cache alone", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    ScopedDataDir      data(temp_dir.path() / "data");
    ScopedResourcesDir resources(temp_dir.path() / "resources");
    const fs::path     profiles = temp_dir.path() / "resources" / "profiles";
    write_acme_printer_vendor(profiles, 33.);

    PresetBundle writer;
    writer.set_generate_vendor_caches(true);
    writer.load_vendor_configs_from_json(profiles.string(), "Acme", PresetBundle::LoadSystem,
                                         ForwardCompatibilitySubstitutionRule::EnableSilent);
    REQUIRE(fs::exists(profiles / "Acme.opc"));
    // Release builds ship the cache and drop the profile JSONs, manifest included.
    fs::remove(profiles / "Acme.json");
    fs::remove_all(profiles / "Acme");

    PresetBundle       bundle;
    DynamicPrintConfig config;
    std::string        error;
    REQUIRE(bundle.resolve_system_preset(config, Preset::TYPE_PRINTER, "Acme Printer",
                                         ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_THAT(config.opt_float("extruder_clearance_dist_to_rod"), Catch::Matchers::WithinAbs(33., 1e-6));
}

TEST_CASE("A system preset no vendor lists is not resolved", "[Preset][Bundle]")
{
    ScopedTemporaryDir temp_dir;
    ScopedDataDir      data(temp_dir.path() / "data");
    ScopedResourcesDir resources(temp_dir.path() / "resources");
    write_acme_printer_vendor(temp_dir.path() / "resources" / "profiles", 33.);

    PresetBundle       bundle;
    DynamicPrintConfig config;
    std::string        error;
    CHECK_FALSE(bundle.resolve_system_preset(config, Preset::TYPE_PRINTER, "Unknown Printer",
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
    CHECK_FALSE(error.empty());
}

namespace {

// Writes each vendor's preset cache into dir, then deletes its profile JSONs: what a release build installs.
void reduce_vendors_to_caches(const fs::path &dir, const std::vector<std::string> &vendor_ids)
{
    const std::string lib(PresetBundle::ORCA_FILAMENT_LIBRARY);
    PresetBundle      library;
    if (fs::exists(dir / (lib + ".json"))) {
        library.set_generate_vendor_caches(true);
        library.load_vendor_configs_from_json(dir.string(), lib, PresetBundle::LoadSystem,
                                              ForwardCompatibilitySubstitutionRule::EnableSilent);
    }
    for (const std::string &vendor_id : vendor_ids) {
        if (vendor_id == lib)
            continue;
        PresetBundle writer;
        writer.set_generate_vendor_caches(true);
        writer.load_vendor_configs_from_json(dir.string(), vendor_id, PresetBundle::LoadSystem,
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, &library);
    }
    for (const std::string &vendor_id : vendor_ids) {
        REQUIRE(fs::exists(dir / (vendor_id + ".opc")));
        fs::remove(dir / (vendor_id + ".json"));
        fs::remove_all(dir / vendor_id);
    }
}

// The filament library with one abstract base filament, and an "Acme" vendor whose one filament inherits it.
void write_library_and_acme_filament(const fs::path &root)
{
    const std::string lib(PresetBundle::ORCA_FILAMENT_LIBRARY);
    fs::create_directories(root / lib / "filament");
    std::ofstream((root / (lib + ".json")).string())
        << R"({"version":"1.0.0","name":")" << lib << R"(",)"
        << R"("filament_list":[{"name":"Generic PLA","sub_path":"filament/generic_pla.json"}]})";
    std::ofstream((root / lib / "filament" / "generic_pla.json").string())
        << R"({"type":"filament","name":"Generic PLA","from":"system","instantiation":"false","filament_id":"GFL99","filament_cost":"27"})";
    fs::create_directories(root / "Acme" / "filament");
    std::ofstream((root / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","filament_list":[{"name":"Acme PLA","sub_path":"filament/pla.json"}]})";
    std::ofstream((root / "Acme" / "filament" / "pla.json").string())
        << R"({"type":"filament","name":"Acme PLA","from":"system","instantiation":"true","inherits":"Generic PLA"})";
}

} // namespace

TEST_CASE("A read-only load resolves a user preset against vendors installed as their cache alone", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir temp_dir;
    const fs::path     data   = temp_dir.path() / "data";
    const fs::path     system = data / PRESET_SYSTEM_DIR;
    ScopedDataDir      scoped_data(data);
    ScopedResourcesDir scoped_resources(temp_dir.path() / "resources");
    write_acme_printer_vendor(system, 33.);
    reduce_vendors_to_caches(system, {"Acme"});

    fs::create_directories(data / PRESET_USER_DIR / DEFAULT_USER_FOLDER_NAME / PRESET_PRINTER_NAME);
    std::ofstream((data / PRESET_USER_DIR / DEFAULT_USER_FOLDER_NAME / PRESET_PRINTER_NAME / "My Acme.json").string())
        << R"({"type":"machine","name":"My Acme","from":"User","version":"2.3.0.0","inherits":"Acme Printer","printable_height":"123"})";

    AppConfig    app_config;
    PresetBundle bundle;
    std::string  errors;
    bundle.load_presets(app_config, ForwardCompatibilitySubstitutionRule::EnableSilent, PresetBundle::PresetPreferences(),
                        &errors, true);
    CHECK(errors.empty());
    const Preset *preset = bundle.printers.find_preset("My Acme");
    REQUIRE(preset != nullptr);
    CHECK_THAT(preset->config.opt_float("printable_height"), Catch::Matchers::WithinAbs(123., 1e-6));
    CHECK_THAT(preset->config.opt_float("extruder_clearance_dist_to_rod"), Catch::Matchers::WithinAbs(33., 1e-6));
}

TEST_CASE("A read-only load writes no preset cache", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir temp_dir;
    const fs::path     system = temp_dir.path() / "data" / PRESET_SYSTEM_DIR;
    ScopedDataDir      scoped_data(temp_dir.path() / "data");
    ScopedResourcesDir scoped_resources(temp_dir.path() / "resources");
    write_acme_printer_vendor(system, 33.);

    AppConfig    app_config;
    PresetBundle bundle;
    std::string  errors;
    bundle.load_presets(app_config, ForwardCompatibilitySubstitutionRule::EnableSilent, PresetBundle::PresetPreferences(),
                        &errors, true);
    CHECK(errors.empty());
    CHECK(bundle.printers.find_preset("Acme Printer") != nullptr);
    CHECK_FALSE(fs::exists(system / "Acme.opc"));
}

TEST_CASE("A vendor updated over the air resolves against the library installed as its cache alone", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir temp_dir;
    const fs::path     system = temp_dir.path() / "data" / PRESET_SYSTEM_DIR;
    ScopedDataDir      scoped_data(temp_dir.path() / "data");
    ScopedResourcesDir scoped_resources(temp_dir.path() / "resources");
    // System presets are found by name through the bundled profiles.
    write_library_and_acme_filament(temp_dir.path() / "resources" / PRESET_PROFILES_DIR);
    // The release install, then an update that brings Acme back as JSONs while the library stays a cache.
    write_library_and_acme_filament(system);
    reduce_vendors_to_caches(system, {PresetBundle::ORCA_FILAMENT_LIBRARY, "Acme"});
    write_library_and_acme_filament(temp_dir.path() / "update");
    fs::copy_file(temp_dir.path() / "update" / "Acme.json", system / "Acme.json");
    fs::create_directories(system / "Acme" / "filament");
    fs::copy_file(temp_dir.path() / "update" / "Acme" / "filament" / "pla.json", system / "Acme" / "filament" / "pla.json");

    SECTION("by name") {
        PresetBundle       bundle;
        DynamicPrintConfig config;
        std::string        error;
        REQUIRE(bundle.resolve_system_preset(config, Preset::TYPE_FILAMENT, "Acme PLA",
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
        CHECK_THAT(config.opt<ConfigOptionFloats>("filament_cost")->values.front(), Catch::Matchers::WithinAbs(27., 1e-6));
    }
    SECTION("by its source file") {
        PresetBundle       bundle;
        DynamicPrintConfig config;
        config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "Generic PLA";
        std::string error;
        REQUIRE(bundle.resolve_preset_config(config, Preset::TYPE_FILAMENT, (system / "Acme" / "filament" / "pla.json").string(),
                                             ForwardCompatibilitySubstitutionRule::EnableSilent, error));
        CHECK_THAT(config.opt<ConfigOptionFloats>("filament_cost")->values.front(), Catch::Matchers::WithinAbs(27., 1e-6));
    }
}

TEST_CASE("The vendor and filament scans read the bundled vendors that ship as their cache alone", "[Preset][Bundle][Regression]")
{
    ScopedTemporaryDir temp_dir;
    const fs::path     profiles = temp_dir.path() / "resources" / PRESET_PROFILES_DIR;
    ScopedResourcesDir scoped_resources(temp_dir.path() / "resources");
    const std::string  lib(PresetBundle::ORCA_FILAMENT_LIBRARY);
    fs::create_directories(profiles / lib / "filament");
    std::ofstream((profiles / (lib + ".json")).string())
        << R"({"version":"1.0.0","name":")" << lib << R"(",)"
        << R"("filament_list":[{"name":"Generic PLA","sub_path":"filament/generic_pla.json"}]})";
    std::ofstream((profiles / lib / "filament" / "generic_pla.json").string())
        << R"({"type":"filament","name":"Generic PLA","from":"system","instantiation":"false","filament_id":"GFL99"})";
    fs::create_directories(profiles / "Acme" / "machine");
    fs::create_directories(profiles / "Acme" / "filament");
    std::ofstream((profiles / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme",)"
        << R"("machine_model_list":[{"name":"Acme One","sub_path":"machine/model.json"}],)"
        << R"("machine_list":[{"name":"Acme Printer","sub_path":"machine/printer.json"}],)"
        << R"("filament_list":[{"name":"Acme PLA","sub_path":"filament/pla.json"}]})";
    std::ofstream((profiles / "Acme" / "machine" / "model.json").string())
        << R"({"type":"machine_model","name":"Acme One","nozzle_diameter":"0.4"})";
    std::ofstream((profiles / "Acme" / "machine" / "printer.json").string())
        << R"({"type":"machine","name":"Acme Printer","from":"system","instantiation":"true","printer_model":"Acme One","printer_variant":"0.4"})";
    std::ofstream((profiles / "Acme" / "filament" / "pla.json").string())
        << R"({"type":"filament","name":"Acme PLA","from":"system","instantiation":"true","inherits":"Generic PLA","filament_id":"P0000001"})";

    auto installed = [](const PresetCollection &presets) {
        return std::count_if(presets.get_presets().begin(), presets.get_presets().end(), [](const Preset &preset) { return !preset.is_default; });
    };
    auto scan = [&installed] {
        std::vector<std::string> found;
        PresetBundle             models;
        models.load_system_models_from_json(ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(installed(models.printers) + installed(models.prints) + installed(models.filaments) == 0);
        for (const auto &[vendor_id, vendor] : models.vendors)
            for (const VendorProfile::PrinterModel &model : vendor.models)
                found.push_back(vendor_id + " model " + model.id);
        PresetBundle filaments;
        filaments.load_system_filaments_json(ForwardCompatibilitySubstitutionRule::EnableSilent);
        CHECK(installed(filaments.printers) + installed(filaments.prints) == 0);
        for (const Preset &preset : filaments.filaments.get_presets())
            if (!preset.is_default)
                found.push_back(preset.name + " " + preset.filament_id);
        return found;
    };
    const std::vector<std::string> expected{"Acme model Acme One", "Acme PLA P0000001"};
    REQUIRE(scan() == expected);
    reduce_vendors_to_caches(profiles, {lib, "Acme"});
    CHECK(scan() == expected);
}

namespace {

// A default preset config for type, built the way PresetBundle builds its default presets.
DynamicPrintConfig external_default_config(Preset::Type type)
{
    DynamicPrintConfig config;
    config.apply_only(static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()),
                      type == Preset::TYPE_PRINTER ? Preset::printer_options() : Preset::print_options());
    Preset::inherits(config);
    return config;
}

// A find_base callback that counts its calls and returns base.
auto base_finder(DynamicPrintConfig *base, int &calls)
{
    return [base, &calls](const std::string &) {
        ++calls;
        return base;
    };
}

} // namespace

TEST_CASE("A key missing from a project takes the default preset value when there is no base", "[Preset][ExternalPreset]")
{
    DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    defaults.set("wall_loops", 7, true);
    const DynamicPrintConfig project;
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {"sparse_infill_density"},
                                                                   base_finder(nullptr, calls));
    CHECK(calls == 1);
    CHECK(config.opt_int("wall_loops") == 7);
}

TEST_CASE("A key missing from a project takes its base preset value", "[Preset][ExternalPreset]")
{
    DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    defaults.set("wall_loops", 7, true);
    DynamicPrintConfig base = defaults;
    base.set("wall_loops", 5, true);
    const DynamicPrintConfig project;
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {"sparse_infill_density"},
                                                                   base_finder(&base, calls));
    CHECK(config.opt_int("wall_loops") == 5);
}

TEST_CASE("A project key listed as different keeps the project value", "[Preset][ExternalPreset]")
{
    const DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    DynamicPrintConfig       base     = defaults;
    base.set("wall_loops", 5, true);
    DynamicPrintConfig project;
    project.set("wall_loops", 4, true);
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {"wall_loops"},
                                                                   base_finder(&base, calls));
    CHECK(config.opt_int("wall_loops") == 4);
}

TEST_CASE("A project key not listed as different is refreshed to the base value", "[Preset][ExternalPreset]")
{
    const DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    DynamicPrintConfig       base     = defaults;
    base.set("wall_loops", 5, true);
    DynamicPrintConfig project;
    project.set("wall_loops", 4, true);
    project.set("inherits", "Base Process", true);
    std::string inherits;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {"sparse_infill_density"},
                                                                   [&base, &inherits](const std::string &name) {
                                                                       inherits = name;
                                                                       return &base;
                                                                   });
    CHECK(inherits == "Base Process");
    CHECK(config.opt_int("wall_loops") == 5);
}

TEST_CASE("An empty different settings list keeps the project values without a base", "[Preset][ExternalPreset]")
{
    DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    defaults.set("top_shell_layers", 7, true);
    DynamicPrintConfig base = defaults;
    base.set("wall_loops", 5, true);
    base.set("top_shell_layers", 9, true);
    DynamicPrintConfig project;
    project.set("wall_loops", 4, true);
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {}, base_finder(&base, calls));
    CHECK(calls == 0);
    CHECK(config.opt_int("wall_loops") == 4);
    CHECK(config.opt_int("top_shell_layers") == 7);
}

TEST_CASE("Print-host keys are never taken from a project", "[Preset][ExternalPreset]")
{
    DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINTER);
    defaults.set("print_host", "", true);
    defaults.set("printhost_apikey", "", true);
    DynamicPrintConfig project;
    project.set("print_host", "http://project-host", true);
    project.set("printhost_apikey", "project-key", true);
    project.set("printer_notes", "project notes", true);
    t_config_option_keys keys;
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINTER, defaults, project, {"printer_notes"},
                                                                   base_finder(nullptr, calls), &keys);
    CHECK(config.opt_string("print_host").empty());
    CHECK(config.opt_string("printhost_apikey").empty());
    CHECK(config.opt_string("printer_notes") == "project notes");
    CHECK_FALSE(contains_key(keys, "print_host"));
    CHECK_FALSE(contains_key(keys, "printhost_apikey"));
    CHECK(contains_key(keys, "printer_notes"));
}

TEST_CASE("A per-variant project value maps onto its base preset's variant layout", "[Preset][ExternalPreset]")
{
    const DynamicPrintConfig defaults = external_default_config(Preset::TYPE_PRINT);
    DynamicPrintConfig       base     = defaults;
    base.set_key_value("print_extruder_id", new ConfigOptionInts({1, 1}));
    base.set_key_value("print_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard", "Direct Drive High Flow"}));
    base.set_key_value("outer_wall_speed", new ConfigOptionFloats({200., 300.}));
    base.set_key_value("inner_wall_speed", new ConfigOptionFloats({250., 350.}));
    // A project saved before its printer had a High Flow variant.
    DynamicPrintConfig project;
    project.set_key_value("print_extruder_id", new ConfigOptionInts({1}));
    project.set_key_value("print_extruder_variant", new ConfigOptionStrings({"Direct Drive Standard"}));
    project.set_key_value("outer_wall_speed", new ConfigOptionFloats({100.}));
    project.set_key_value("inner_wall_speed", new ConfigOptionFloats({150.}));
    int calls = 0;

    const DynamicPrintConfig config = Preset::load_external_config(Preset::TYPE_PRINT, defaults, project, {"outer_wall_speed"},
                                                                   base_finder(&base, calls));
    CHECK(config.option<ConfigOptionStrings>("print_extruder_variant")->values ==
          std::vector<std::string>{"Direct Drive Standard", "Direct Drive High Flow"});
    // The listed key keeps the project's Standard value, and High Flow, which the project does not
    // list, takes it too, as a user preset's value does.
    check_double_vector(config.option<ConfigOptionFloats>("outer_wall_speed")->values, {100., 100.});
    check_double_vector(config.option<ConfigOptionFloats>("inner_wall_speed")->values, {250., 350.});
}

TEST_CASE("A project's different settings always keep the preset bookkeeping keys", "[Preset][ExternalPreset]")
{
    const std::set<std::string> keys = PresetBundle::project_different_keys(escape_strings_cstyle({"wall_loops", "top_shell_layers"}));
    for (const char *key : {"wall_loops", "top_shell_layers", "inherits", "print_settings_id", "filament_settings_id", "printer_settings_id"})
        CHECK(keys.count(key) == 1);
    CHECK(PresetBundle::project_different_keys(std::string()).count("inherits") == 1);
}

// A connected machine's tray is checked against the filament variant of the nozzle it feeds.
TEST_CASE("A filament's variant index follows the extruder type and nozzle volume type", "[Preset]")
{
    DynamicPrintConfig filament;
    filament.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {"Direct Drive Standard", "Direct Drive High Flow",
                                                                                       "Bowden Standard"};
    DynamicPrintConfig printer;
    printer.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {etDirectDrive, etBowden};

    CHECK(PresetBundle::get_filament_variant_index(filament, printer, 0, nvtStandard) == 0);
    CHECK(PresetBundle::get_filament_variant_index(filament, printer, 0, nvtHighFlow) == 1);
    CHECK(PresetBundle::get_filament_variant_index(filament, printer, 1, nvtStandard) == 2);
    // the filament defines no Bowden High Flow variant
    CHECK(PresetBundle::get_filament_variant_index(filament, printer, 1, nvtHighFlow) == 0);
}

TEST_CASE("A user preset saved over by another instance while its directory loads is read again under the lock", "[Preset][Bundle][InstanceLock]")
{
    ScopedTemporaryDir   temp_dir;
    ScopedDataDir        data_dir_scope(temp_dir.path());
    RenameTestCollection coll;

    Preset &parent = add_inmemory_preset(coll, "Parent Process");
    parent.config.option<ConfigOptionFloat>("layer_height", true)->value = 0.24;
    parent.is_system = true;

    // Fewer than one batch, so every file is read before the first one is committed.
    constexpr int  children = 8;
    static_assert(children <= int(resolve_batch_size));
    const fs::path dir      = temp_dir.path() / PRESET_PRINT_NAME;
    for (int i = 0; i < children; ++ i)
        write_minimal_child(dir / ("Child " + std::to_string(i) + ".json"), "Child " + std::to_string(i), "Parent Process");

    // The first commit stands in for another instance saving every other preset after
    // they were read and before they are committed.
    std::string first;
    auto save_the_others = [&](Preset &preset) {
        if (! first.empty())
            return;
        first = preset.name;
        for (int i = 0; i < children; ++ i) {
            const std::string name = "Child " + std::to_string(i);
            if (name != first)
                std::ofstream((dir / (name + ".json")).string())
                    << R"({"type":"process","name":")" << name
                    << R"(","from":"User","version":"1.0.0","inherits":"Parent Process","layer_height":"0.3"})";
        }
    };
    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable, save_the_others);

    REQUIRE_FALSE(first.empty());
    CHECK(coll.error_count() == 0);
    for (int i = 0; i < children; ++ i) {
        const std::string name  = "Child " + std::to_string(i);
        const Preset     *child = coll.find_preset(name);
        REQUIRE(child != nullptr);
        CHECK_THAT(child->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(name == first ? 0.24 : 0.3, 1e-9));
    }
}

TEST_CASE("A user preset removed by another instance while its directory loads is not installed", "[Preset][Bundle][InstanceLock]")
{
    ScopedTemporaryDir   temp_dir;
    ScopedDataDir        data_dir_scope(temp_dir.path());
    RenameTestCollection coll;

    Preset &parent = add_inmemory_preset(coll, "Parent Process");
    parent.is_system = true;

    // Fewer than one batch, so every file is read before the first one is committed.
    constexpr int  children = 8;
    static_assert(children <= int(resolve_batch_size));
    const fs::path dir      = temp_dir.path() / PRESET_PRINT_NAME;
    for (int i = 0; i < children; ++ i)
        write_minimal_child(dir / ("Child " + std::to_string(i) + ".json"), "Child " + std::to_string(i), "Parent Process");

    std::string first;
    auto remove_the_others = [&](Preset &preset) {
        if (! first.empty())
            return;
        first = preset.name;
        for (int i = 0; i < children; ++ i)
            if (const std::string name = "Child " + std::to_string(i); name != first)
                fs::remove(dir / (name + ".json"));
    };
    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable, remove_the_others);

    REQUIRE_FALSE(first.empty());
    CHECK(coll.error_count() == 0);
    CHECK(coll.size() == 3); // the default preset, the parent and the first child
    CHECK(coll.find_preset(first) != nullptr);
}

TEST_CASE("Without the instance lock a user preset loads but its file is not written back", "[Preset][Bundle][InstanceLock]")
{
    ScopedTemporaryDir temp_dir;
    ScopedDataDir      data_dir_scope(temp_dir.path());
    // A directory where the lock file belongs, so the lock cannot be taken.
    fs::create_directories(fs::path(user_presets_lock_path()));
    PresetBundle       bundle;
    const fs::path     file = temp_dir.path() / PRESET_FILAMENT_NAME / "My PLA @Test Printer.json";
    write_preset_with_inherits(bundle.filaments.default_preset().config, file, "My PLA @Test Printer", std::string());
    const std::string before = read_file(file);

    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::EnableSilent);

    const Preset *preset = bundle.filaments.find_preset("My PLA @Test Printer");
    REQUIRE(preset != nullptr);
    CHECK(preset->config.option<ConfigOptionStrings>("compatible_printers")->values == std::vector<std::string>{ "Test Printer" });
    CHECK(read_file(file) == before);
}

#ifndef _WIN32
// File permissions stop reading only on POSIX.
TEST_CASE("A user preset that cannot be read under the instance lock is counted and removed", "[Preset][Bundle][InstanceLock]")
{
    if (::geteuid() == 0)
        SKIP("file permissions do not stop root");
    ScopedTemporaryDir   temp_dir;
    ScopedDataDir        data_dir_scope(temp_dir.path());
    RenameTestCollection coll;

    Preset &parent = add_inmemory_preset(coll, "Parent Process");
    parent.is_system = true;
    const fs::path file = temp_dir.path() / PRESET_PRINT_NAME / "Unreadable.json";
    write_minimal_child(file, "Unreadable", "Parent Process");
    fs::permissions(file, fs::no_perms);

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable);

    CHECK(coll.error_count() == 1);
    CHECK(coll.find_preset("Unreadable") == nullptr);
    CHECK_FALSE(fs::exists(file));
}
#endif

#ifndef _WIN32
// Creating a symlink needs no privilege only on POSIX.
TEST_CASE("A user preset that is a symlink to itself is counted and removed while the rest still load", "[Preset][Bundle][InstanceLock]")
{
    ScopedTemporaryDir   temp_dir;
    ScopedDataDir        data_dir_scope(temp_dir.path());
    RenameTestCollection coll;

    Preset &parent = add_inmemory_preset(coll, "Parent Process");
    parent.is_system = true;
    const fs::path dir = temp_dir.path() / PRESET_PRINT_NAME;
    write_minimal_child(dir / "Good A.json", "Good A", "Parent Process");
    write_minimal_child(dir / "Good B.json", "Good B", "Parent Process");
    const fs::path loop = dir / "Loop.json";
    fs::create_symlink(loop.filename(), loop);

    PresetsConfigSubstitutions substitutions;
    REQUIRE_NOTHROW(coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                                      ForwardCompatibilitySubstitutionRule::Disable));

    CHECK(coll.error_count() == 1);
    CHECK(coll.find_preset("Good A") != nullptr);
    CHECK(coll.find_preset("Good B") != nullptr);
    CHECK(coll.find_preset("Loop") == nullptr);
    boost::system::error_code ec;
    CHECK(fs::symlink_status(loop, ec).type() == fs::file_not_found);
}
#endif

#ifndef _WIN32
// Creating a symlink needs no privilege only on POSIX.
TEST_CASE("A user preset symlinked to a missing target is counted but the link is kept", "[Preset][Bundle][InstanceLock]")
{
    ScopedTemporaryDir   temp_dir;
    ScopedDataDir        data_dir_scope(temp_dir.path());
    RenameTestCollection coll;

    const fs::path dir  = temp_dir.path() / PRESET_PRINT_NAME;
    const fs::path link = dir / "Linked.json";
    fs::create_directories(dir);
    fs::create_symlink(temp_dir.path() / "unmounted" / "Linked.json", link);

    PresetsConfigSubstitutions substitutions;
    coll.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                      ForwardCompatibilitySubstitutionRule::Disable);

    CHECK(coll.error_count() == 1);
    CHECK(coll.find_preset("Linked") == nullptr);
    boost::system::error_code ec;
    CHECK(fs::symlink_status(link, ec).type() == fs::symlink_file);
}
#endif

#ifndef _WIN32
// The read-only bit on a directory stops file creation only on POSIX.
TEST_CASE("A user filament whose derived compatible printer cannot be written back still loads with the failure counted", "[Preset][Bundle]")
{
    if (::geteuid() == 0)
        SKIP("a read-only directory does not stop root");
    ScopedTemporaryDir temp_dir;
    PresetBundle       bundle;
    const fs::path     dir  = temp_dir.path() / PRESET_FILAMENT_NAME;
    const fs::path     file = dir / "My PLA @Test Printer.json";
    write_preset_with_inherits(bundle.filaments.default_preset().config, file, "My PLA @Test Printer", std::string());
    const std::string before = read_file(file);

    fs::permissions(file, fs::owner_read);
    fs::permissions(dir, fs::owner_read | fs::owner_exe);
    PresetsConfigSubstitutions substitutions;
    bundle.filaments.load_presets(temp_dir.path().string(), PRESET_FILAMENT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::EnableSilent);
    // Restored before any assertion, so a failure never leaves an unremovable directory behind.
    fs::permissions(dir, fs::owner_all);
    fs::permissions(file, fs::owner_read | fs::owner_write);

    const Preset *preset = bundle.filaments.find_preset("My PLA @Test Printer");
    REQUIRE(preset != nullptr);
    CHECK(preset->config.option<ConfigOptionStrings>("compatible_printers")->values == std::vector<std::string>{ "Test Printer" });
    CHECK(bundle.filaments.error_count() == 1);
    CHECK(read_file(file) == before);
}
#endif

namespace {

namespace fs = boost::filesystem;

void write_process(const fs::path &file, const std::string &name, const std::string &inherits,
                   const std::string &layer_height = {}, const std::string &density = {})
{
    fs::create_directories(file.parent_path());
    std::ofstream out(file.string());
    out << R"({"type":"process","name":")" << name
        << R"(","from":"User","version":"1.0.0")";
    if (!inherits.empty())
        out << R"(,"inherits":")" << inherits << R"(")";
    if (!layer_height.empty())
        out << R"(,"layer_height":")" << layer_height << R"(")";
    if (!density.empty())
        out << R"(,"sparse_infill_density":")" << density << R"(")";
    out << "}\n";
}

std::map<std::string, std::string> cloud_values(const std::string &type, const std::string &setting_id,
                                                const std::string &user_id, const std::string &inherits = {})
{
    std::map<std::string, std::string> values{
        { BBL_JSON_KEY_TYPE, type },
        { BBL_JSON_KEY_VERSION, "1.0.0" },
        { BBL_JSON_KEY_SETTING_ID, setting_id },
        { BBL_JSON_KEY_USER_ID, user_id },
        { ORCA_JSON_KEY_UPDATE_TIME, "10" },
    };
    if (!inherits.empty())
        values.emplace(BBL_JSON_KEY_INHERITS, inherits);
    return values;
}

} // namespace

TEST_CASE("File preset loading resolves a three-level base chain", "[PresetFiles]")
{
    ScopedTemporaryDir temp_dir;
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    const fs::path print_dir = temp_dir.path() / PRESET_PRINT_NAME;

    // load_presets() intentionally reads each nested <type>/base directory
    // before its parent directory. Keep each inheritance edge in a completed
    // traversal level; sibling files are resolved concurrently.
    write_process(print_dir / "base" / "base" / "Base.json", "Base", {}, "0.24", "20%");
    write_process(print_dir / "base" / "Middle.json", "Middle", "Base", "0.28");
    write_process(print_dir / "Leaf.json", "Leaf", "Middle", {}, "42%");

    PresetsConfigSubstitutions substitutions;
    collection.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                            ForwardCompatibilitySubstitutionRule::Disable);

    const Preset *base = collection.find_preset("Base");
    const Preset *middle = collection.find_preset("Middle");
    const Preset *leaf = collection.find_preset("Leaf");
    REQUIRE(base != nullptr);
    REQUIRE(middle != nullptr);
    REQUIRE(leaf != nullptr);
    CHECK(collection.error_count() == 0);
    CHECK(middle->inherits() == "Base");
    CHECK(leaf->inherits() == "Middle");
    CHECK_THAT(middle->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));
    CHECK_THAT(leaf->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));
    CHECK_THAT(leaf->config.opt_float("sparse_infill_density"), Catch::Matchers::WithinAbs(42.0, 1e-9));
}

TEST_CASE("Cloud preset loading performs two-pass inheritance and forward substitution", "[PresetFiles]")
{
    PresetBundle bundle;
    AppConfig app_config;
    std::map<std::string, std::map<std::string, std::string>> cloud;

    // Lexicographic order puts the child first. PresetBundle's two-pass load
    // must still install the root before resolving the child.
    auto root = cloud_values(PRESET_IOT_PRINT_TYPE, "cloud-root-id", "account-1");
    root.emplace("layer_height", "0.24");
    root.emplace("wall_generator", "future-generator");
    cloud.emplace("A Child", cloud_values(PRESET_IOT_PRINT_TYPE, "cloud-child-id", "account-1", "Z Root"));
    cloud.emplace("Z Root", std::move(root));
    cloud.emplace("Ignored Type", cloud_values("future-profile", "ignored-id", "account-1"));
    cloud.emplace("Missing Type", std::map<std::string, std::string>{{ BBL_JSON_KEY_VERSION, "1.0.0" }});

    const PresetsConfigSubstitutions substitutions = bundle.load_user_presets(
        app_config, cloud, ForwardCompatibilitySubstitutionRule::Enable);

    const Preset *root_preset = bundle.prints.find_preset("Z Root");
    const Preset *child_preset = bundle.prints.find_preset("A Child");
    REQUIRE(root_preset != nullptr);
    REQUIRE(child_preset != nullptr);
    CHECK(bundle.prints.find_preset("Ignored Type") == nullptr);
    CHECK(bundle.prints.find_preset("Missing Type") == nullptr);
    CHECK(child_preset->inherits() == "Z Root");
    CHECK(child_preset->base_id == root_preset->setting_id);
    CHECK_THAT(child_preset->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    REQUIRE(substitutions.size() == 1);
    CHECK(substitutions.front().preset_name == "Z Root");
    CHECK(substitutions.front().substitutions.size() == 1);
    CHECK(substitutions.front().substitutions.front().old_value == "future-generator");
}

TEST_CASE("Cloud filament loading inherits null filament identity and rejects bad metadata", "[PresetFiles]")
{
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    PresetsConfigSubstitutions substitutions;

    auto parent_values = cloud_values(PRESET_IOT_FILAMENT_TYPE, "filament-setting-id", "account-1");
    parent_values.emplace(BBL_JSON_KEY_FILAMENT_ID, "filament-family-id");
    parent_values.emplace("filament_diameter", "1.75,2.85");
    REQUIRE(collection.load_user_preset("Cloud Filament", std::move(parent_values), substitutions,
                                        ForwardCompatibilitySubstitutionRule::EnableSilent));

    auto child_values = cloud_values(PRESET_IOT_FILAMENT_TYPE, "filament-child-id", "account-1", "Cloud Filament");
    child_values.emplace(BBL_JSON_KEY_FILAMENT_ID, "null");
    REQUIRE(collection.load_user_preset("Cloud Filament Child", std::move(child_values), substitutions,
                                        ForwardCompatibilitySubstitutionRule::EnableSilent));
    const Preset *child = collection.find_preset("Cloud Filament Child");
    REQUIRE(child != nullptr);
    CHECK(child->filament_id == "filament-family-id");
    CHECK(child->base_id == "filament-setting-id");
    REQUIRE(child->config.option<ConfigOptionFloats>("filament_diameter") != nullptr);
    CHECK(child->config.option<ConfigOptionFloats>("filament_diameter")->values.size() == 2);

    const size_t size_before_invalid = collection.size();
    auto missing_parent = cloud_values(PRESET_IOT_FILAMENT_TYPE, "missing-parent-id", "account-1", "No Such Filament");
    CHECK_FALSE(collection.load_user_preset("Orphan Filament", std::move(missing_parent), substitutions,
                                            ForwardCompatibilitySubstitutionRule::EnableSilent));
    CHECK(collection.find_preset("Orphan Filament") == nullptr);
    CHECK(collection.size() == size_before_invalid);

    auto missing_version = std::map<std::string, std::string>{
        { BBL_JSON_KEY_SETTING_ID, "no-version-id" },
        { BBL_JSON_KEY_USER_ID, "account-1" },
    };
    CHECK_FALSE(collection.load_user_preset("No Version", std::move(missing_version), substitutions,
                                            ForwardCompatibilitySubstitutionRule::EnableSilent));
    CHECK(collection.find_preset("No Version") == nullptr);

    auto missing_user = std::map<std::string, std::string>{
        { BBL_JSON_KEY_VERSION, "1.0.0" },
        { BBL_JSON_KEY_SETTING_ID, "no-user-id" },
    };
    CHECK_FALSE(collection.load_user_preset("No User", std::move(missing_user), substitutions,
                                            ForwardCompatibilitySubstitutionRule::EnableSilent));
    CHECK(collection.find_preset("No User") == nullptr);
}

TEST_CASE("File loading rejects an inheritance cycle without installing either node", "[PresetFiles]")
{
    ScopedTemporaryDir temp_dir;
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    const fs::path print_dir = temp_dir.path() / PRESET_PRINT_NAME;
    write_process(print_dir / "Cycle A.json", "Cycle A", "Cycle B");
    write_process(print_dir / "Cycle B.json", "Cycle B", "Cycle A");

    PresetsConfigSubstitutions substitutions;
    collection.load_presets(temp_dir.path().string(), PRESET_PRINT_NAME, substitutions,
                            ForwardCompatibilitySubstitutionRule::EnableSilent);

    CHECK(collection.find_preset("Cycle A") == nullptr);
    CHECK(collection.find_preset("Cycle B") == nullptr);
    CHECK(collection.error_count() == 2);
}

namespace preset_full_config_contracts {

namespace {

struct ConstructInputs {
    PresetBundle bundle;
    Preset printer{Preset::TYPE_PRINTER, "Test Printer"};
    Preset print{Preset::TYPE_PRINT, "Test Process"};
    std::vector<Preset> filaments;

    ConstructInputs()
    {
        printer.config = bundle.printers.default_preset().config;
        print.config   = bundle.prints.default_preset().config;

        Preset filament{Preset::TYPE_FILAMENT, "Test PLA"};
        filament.config = bundle.filaments.default_preset().config;
        filament.filament_id = "test-pla-id";
        filaments.emplace_back(std::move(filament));
    }

    void add_filament(const std::string &name, const std::string &id)
    {
        Preset filament{Preset::TYPE_FILAMENT, name};
        filament.config = bundle.filaments.default_preset().config;
        filament.filament_id = id;
        filaments.emplace_back(std::move(filament));
    }

    DynamicPrintConfig project() const { return bundle.project_config; }

    DynamicPrintConfig construct(
        bool apply_extruder = false,
        std::optional<std::vector<int>> filament_maps = std::nullopt,
        std::optional<std::vector<int>> filament_volume_maps = std::nullopt)
    {
        DynamicPrintConfig project_config = project();
        return PresetBundle::construct_full_config(
            printer, print, project_config, filaments, apply_extruder,
            std::move(filament_maps), std::move(filament_volume_maps));
    }
};

void set_dual_printer_layout(ConstructInputs &in)
{
    in.printer.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4};
    in.printer.config.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {
        etDirectDrive, etDirectDrive};
    in.printer.config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {
        nvtStandard, nvtStandard};
    in.printer.config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {
        "Direct Drive Standard", "Direct Drive Standard"};
    in.printer.config.option<ConfigOptionInts>("printer_extruder_id", true)->values = {1, 2};
    in.printer.config.option<ConfigOptionStrings>("printer_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive Standard"};
}

} // namespace

TEST_CASE("The complete preset configuration merges one filament and preserves preset metadata", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    in.print.config.option<ConfigOptionFloat>("layer_height", true)->value = 0.21;
    in.print.config.option<ConfigOptionInt>("support_filament", true)->value = 7;
    in.print.config.option<ConfigOptionInt>("support_interface_filament", true)->value = -2;
    in.print.inherits() = "Test Base Process";
    in.print.compatible_printers_condition() = "printer_model == 'Test Printer'";
    in.filaments[0].inherits() = "Test Base Filament";
    in.filaments[0].compatible_printers_condition() = "printer_model == 'Test Printer'";
    in.filaments[0].compatible_prints_condition() = "process == 'Test Process'";
    in.filaments[0].config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75};

    DynamicPrintConfig out = in.construct();

    CHECK_THAT(out.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.21, 1e-9));
    CHECK(out.opt_int("support_filament") == 1);
    CHECK(out.opt_int("support_interface_filament") == 0);
    CHECK(out.opt_string("print_settings_id") == "Test Process");
    CHECK(out.opt_string("printer_settings_id") == "Test Printer");
    CHECK(out.option<ConfigOptionStrings>("filament_settings_id")->values == std::vector<std::string>{"Test PLA"});
    CHECK(out.option<ConfigOptionStrings>("filament_ids")->values == std::vector<std::string>{"test-pla-id"});
    CHECK(out.option<ConfigOptionInts>("filament_self_index")->values == std::vector<int>{1});
    CHECK(out.opt_enum<PrinterTechnology>("printer_technology") == ptFFF);
    CHECK(out.option<ConfigOptionStrings>("compatible_machine_expression_group")->values ==
          std::vector<std::string>{"printer_model == 'Test Printer'", "printer_model == 'Test Printer'"});
    CHECK(out.option<ConfigOptionStrings>("compatible_process_expression_group")->values ==
          std::vector<std::string>{"process == 'Test Process'"});
    CHECK(out.option<ConfigOptionStrings>("inherits_group")->values ==
          std::vector<std::string>{"Test Base Process", "Test Base Filament"});
}

TEST_CASE("The complete preset configuration accepts explicit filament and volume maps", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    DynamicPrintConfig out = in.construct(false, std::vector<int>{4}, std::vector<int>{2});

    CHECK(out.option<ConfigOptionInts>("filament_map")->values == std::vector<int>{4});
    CHECK(out.option<ConfigOptionInts>("filament_volume_map")->values == std::vector<int>{2});
    CHECK(out.option<ConfigOptionInts>("filament_self_index")->values == std::vector<int>{1});
}

TEST_CASE("The complete preset configuration concatenates multiple filament vectors and tracks source slots", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    in.add_filament("Test PETG", "test-petg-id");
    in.filaments[0].config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75};
    in.filaments[0].config.option<ConfigOptionStrings>("filament_type", true)->values = {"PLA"};
    in.filaments[0].config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {"Direct Drive Standard"};
    in.filaments[1].config.option<ConfigOptionFloats>("filament_diameter", true)->values = {2.85, 2.85};
    in.filaments[1].config.option<ConfigOptionStrings>("filament_type", true)->values = {"PETG", "PETG-HF"};
    in.filaments[1].config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"};

    DynamicPrintConfig out = in.construct();

    check_double_vector(out.option<ConfigOptionFloats>("filament_diameter")->values, {1.75, 2.85, 2.85});
    CHECK(out.option<ConfigOptionStrings>("filament_type")->values == std::vector<std::string>{"PLA", "PETG", "PETG-HF"});
    CHECK(out.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          std::vector<std::string>{"Direct Drive Standard", "Direct Drive Standard", "Direct Drive High Flow"});
    CHECK(out.option<ConfigOptionInts>("filament_self_index")->values == std::vector<int>{1, 2, 2});
    CHECK(out.option<ConfigOptionStrings>("filament_settings_id")->values ==
          std::vector<std::string>{"Test PLA", "Test PETG"});
    CHECK(out.option<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 1});
}

TEST_CASE("The complete preset configuration resizes transient maps to the filament count", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    in.add_filament("Test PETG", "test-petg-id");
    DynamicPrintConfig project = in.project();
    project.option<ConfigOptionInts>("filament_map", true)->values = {2};
    project.option<ConfigOptionInts>("filament_volume_map", true)->values = {1};

    DynamicPrintConfig out = PresetBundle::construct_full_config(
        in.printer, in.print, project, in.filaments, false, std::nullopt, std::nullopt);

    // The public contract is one map entry per selected filament. A stale project array is
    // repaired to the current count and padded with the normal first-slot mapping.
    CHECK(out.option<ConfigOptionInts>("filament_map")->values == std::vector<int>{2, 1});
    CHECK(out.option<ConfigOptionInts>("filament_volume_map")->values == std::vector<int>{0, 0});
}

TEST_CASE("The complete preset configuration resolves a single filament through a dual-extruder printer", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    set_dual_printer_layout(in);
    in.filaments[0].config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"};
    in.filaments[0].config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75};

    DynamicPrintConfig out = in.construct(true, std::vector<int>{2}, std::vector<int>{nvtStandard});

    CHECK(out.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          std::vector<std::string>{"Direct Drive Standard"});
    // A single selected filament is resolved against both physical printer
    // extruders before its selected variant is retained.
    check_double_vector(out.option<ConfigOptionFloats>("filament_diameter")->values, {1.75, 1.75});
    CHECK(out.option<ConfigOptionInts>("filament_self_index")->values == std::vector<int>{1});
}

TEST_CASE("The complete preset configuration resolves multiple filaments while retaining source order", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    in.add_filament("Test PETG", "test-petg-id");
    set_dual_printer_layout(in);
    for (Preset &filament : in.filaments) {
        filament.config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
            "Direct Drive Standard", "Direct Drive High Flow"};
        filament.config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.75};
    }

    DynamicPrintConfig out = in.construct(true, std::vector<int>{1, 2}, std::vector<int>{nvtStandard, nvtStandard});

    CHECK(out.option<ConfigOptionStrings>("filament_settings_id")->values ==
          std::vector<std::string>{"Test PLA", "Test PETG"});
    CHECK(out.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          std::vector<std::string>{"Direct Drive Standard", "Direct Drive Standard"});
    check_double_vector(out.option<ConfigOptionFloats>("filament_diameter")->values, {1.75, 1.75});
}

TEST_CASE("The complete preset configuration removes preset-only bookkeeping from the merged config", "[Preset][Bundle][ConstructFullConfig]")
{
    ConstructInputs in;
    in.print.inherits() = "Base Process";
    in.filaments[0].inherits() = "Base Filament";
    in.print.compatible_printers_condition() = "machine";
    in.filaments[0].compatible_printers_condition() = "machine";
    in.print.config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"Test Printer"};

    DynamicPrintConfig out = in.construct();

    CHECK_FALSE(out.has("compatible_prints"));
    CHECK_FALSE(out.has("compatible_printers"));
    CHECK_FALSE(out.has("compatible_printers_condition"));
    CHECK_FALSE(out.has("compatible_prints_condition"));
    CHECK_FALSE(out.has("inherits"));
    CHECK_FALSE(out.has("different_settings_to_system"));
    CHECK(out.option<ConfigOptionStrings>("print_compatible_printers")->values ==
          std::vector<std::string>{"Test Printer"});
}

} // namespace preset_full_config_contracts

namespace preset_lifecycle_contracts {

namespace {

namespace fs = boost::filesystem;

struct ScopedPresetBundle {
    ScopedTemporaryDir directory {"orca-preset-lifecycle"};
    PresetBundle bundle;

    ScopedPresetBundle()
    {
        PresetsConfigSubstitutions substitutions;
        for (const char *kind : {PRESET_PRINT_NAME, PRESET_FILAMENT_NAME, PRESET_PRINTER_NAME})
            fs::create_directories(directory.path() / kind / "base");
        bundle.prints.load_presets(directory.string(), PRESET_PRINT_NAME, substitutions,
                                  ForwardCompatibilitySubstitutionRule::Disable);
        bundle.filaments.load_presets(directory.string(), PRESET_FILAMENT_NAME, substitutions,
                                     ForwardCompatibilitySubstitutionRule::Disable);
        bundle.printers.load_presets(directory.string(), PRESET_PRINTER_NAME, substitutions,
                                    ForwardCompatibilitySubstitutionRule::Disable);
    }
};

DynamicPrintConfig print_config(PresetBundle &bundle, double layer_height)
{
    DynamicPrintConfig config = bundle.prints.default_preset().config;
    config.set_key_value("layer_height", new ConfigOptionFloat(layer_height));
    return config;
}

DynamicPrintConfig filament_config(PresetBundle &bundle, const std::string &vendor,
                                   const std::string &printer)
{
    DynamicPrintConfig config = bundle.filaments.default_preset().config;
    config.opt<ConfigOptionStrings>("filament_vendor")->values = { vendor };
    config.opt<ConfigOptionStrings>("filament_type")->values = { "PLA" };
    config.opt<ConfigOptionStrings>("compatible_printers")->values = { printer };
    config.opt<ConfigOptionString>("compatible_printers_condition")->value.clear();
    return config;
}

DynamicPrintConfig printer_config(PresetBundle &bundle, const std::string &model)
{
    DynamicPrintConfig config = bundle.printers.default_preset().config;
    config.opt<ConfigOptionString>("printer_model", true)->value = model;
    config.opt<ConfigOptionString>("printer_variant", true)->value = "0.4";
    return config;
}

} // namespace

TEST_CASE("Preset bundle metadata round-trips names and timestamps", "[Preset][Bundle][PresetLifecycle][Metadata]")
{
    ScopedTemporaryDir temp;
    const fs::path metadata_path = temp.path() / "bundle_metadata.json";

    BundleMetadata saved;
    saved.id = "bundle-id";
    saved.name = "Cloud bundle";
    saved.version = "2.4.0";
    saved.description = "A test bundle";
    saved.author = "profile-owner";
    saved.imported_time = 101;
    saved.updated_time = 202;
    saved.print_presets = {"process/0.20mm.json", "nested/0.28mm.json"};
    saved.filament_presets = {"filament/PLA.json"};
    saved.printer_presets = {"printer/Machine.json"};

    REQUIRE(saved.save_to_json(metadata_path.string()));
    BundleMetadata loaded;
    REQUIRE(loaded.load_from_json(metadata_path.string()));
    CHECK(loaded.id == saved.id);
    CHECK(loaded.name == saved.name);
    CHECK(loaded.version == saved.version);
    CHECK(loaded.description == saved.description);
    CHECK(loaded.author == saved.author);
    CHECK(loaded.imported_time == 101);
    CHECK(loaded.updated_time == 202);
    // The metadata format stores the bundle-local filename, not an absolute or
    // nested path that would escape the bundle on extraction.
    CHECK(loaded.print_presets == std::vector<std::string>{"0.20mm.json", "0.28mm.json"});
    CHECK(loaded.filament_presets == std::vector<std::string>{"PLA.json"});
    CHECK(loaded.printer_presets == std::vector<std::string>{"Machine.json"});
}

TEST_CASE("Preset bundle metadata accepts legacy fields and rejects malformed files", "[Preset][Bundle][PresetLifecycle][Metadata]")
{
    ScopedTemporaryDir temp;
    const fs::path legacy = temp.path() / "legacy.json";
    std::ofstream(legacy.string())
        << R"({"bundle_id":"legacy-id","bundle_type":"legacy description","version":"1.0.0"})";

    BundleMetadata loaded;
    REQUIRE(loaded.load_from_json(legacy.string()));
    CHECK(loaded.name == "legacy-id");
    CHECK(loaded.description == "legacy description");

    const fs::path malformed = temp.path() / "malformed.json";
    std::ofstream(malformed.string()) << "{\"id\": [";
    REQUIRE(fs::file_size(malformed) > 0);
    CHECK_FALSE(loaded.load_from_json(malformed.string()));
    CHECK_FALSE(loaded.load_from_json((temp.path() / "missing.json").string()));
}

TEST_CASE("Cloning a filament for a printer rewrites compatibility and identity", "[Preset][Bundle][PresetLifecycle][Clone]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    const std::string source_name = "Source PLA";
    Preset &source = bundle.filaments.load_preset("", source_name,
                                                    filament_config(bundle, "Vendor A", "Source Printer"), false);
    source.filament_id = "source-family";
    std::vector<Preset const *> templates{&source};
    std::vector<std::string> failures;

    REQUIRE(bundle.filaments.clone_presets_for_printer(
        templates, failures, "Target Printer",
        [](std::string prefix) { return "derived-" + prefix; }));
    REQUIRE(failures.empty());
    const Preset *copy = bundle.filaments.find_preset("Source PLA @Target Printer");
    REQUIRE(copy != nullptr);
    CHECK(copy->is_user());
    CHECK(copy->filament_id == "derived-Source PLA");
    REQUIRE(copy->config.option<ConfigOptionStrings>("compatible_printers") != nullptr);
    CHECK(copy->config.opt<ConfigOptionStrings>("compatible_printers")->values ==
          std::vector<std::string>{"Target Printer"});
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_vendor")->values ==
          std::vector<std::string>{"Vendor A"});
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_settings_id")->values.front() == copy->name);
}

TEST_CASE("Cloning a filament with new material metadata preserves unrelated options", "[Preset][Bundle][PresetLifecycle][Clone]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    Preset &source = bundle.filaments.load_preset(
        "", "Base Material", filament_config(bundle, "Old Vendor", "Printer A"), false);
    source.filament_id = "base-family";
    source.config.opt<ConfigOptionStrings>("filament_colour", true)->values = {"#112233"};

    DynamicPrintConfig overrides;
    overrides.set_key_value("filament_vendor", new ConfigOptionStrings({"New Vendor"}));
    overrides.set_key_value("filament_type", new ConfigOptionStrings({"PETG"}));
    overrides.set_key_value("compatible_printers", new ConfigOptionStrings({"Printer B"}));
    std::vector<std::string> failures;
    REQUIRE(bundle.filaments.clone_presets_for_filament(
        &source, failures, "Derived Material", "derived-family", overrides, "Printer B"));
    REQUIRE(failures.empty());

    const Preset *copy = bundle.filaments.find_preset("Derived Material @Printer B");
    REQUIRE(copy != nullptr);
    CHECK(copy->filament_id == "derived-family");
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_vendor")->values ==
          std::vector<std::string>{"New Vendor"});
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_type")->values ==
          std::vector<std::string>{"PETG"});
    CHECK(copy->config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#112233"});
}

TEST_CASE("Preset cloning reports collisions and supports an explicit rewrite", "[Preset][Bundle][PresetLifecycle][Clone]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    Preset &source = bundle.prints.load_preset("", "Original Process", print_config(bundle, 0.20), false);
    std::vector<Preset const *> templates{&source};
    std::vector<std::string> failures;
    auto rename = [](Preset &preset, Preset::Type &) {
        preset.name = "Copied Process";
        preset.config.opt<ConfigOptionFloat>("layer_height")->value = 0.24;
    };

    REQUIRE(bundle.prints.clone_presets(templates, failures, rename));
    CHECK(bundle.prints.find_preset("Copied Process") != nullptr);
    failures.clear();
    Preset *source_again = bundle.prints.find_preset("Original Process");
    REQUIRE(source_again != nullptr);
    CHECK_FALSE(bundle.prints.clone_presets({source_again}, failures, rename));
    CHECK(failures == std::vector<std::string>{"Copied Process"});

    failures.clear();
    REQUIRE(bundle.prints.clone_presets({source_again}, failures, rename, true));
    REQUIRE(bundle.prints.find_preset("Copied Process") != nullptr);
    CHECK_THAT(bundle.prints.find_preset("Copied Process")->config.opt_float("layer_height"),
               Catch::Matchers::WithinAbs(0.24, 1e-9));
}

TEST_CASE("Saving a project preset preserves its parent and reset removes the embedded copy", "[Preset][Bundle][PresetLifecycle][Project]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    bundle.prints.load_preset("", "Editable Process", print_config(bundle, 0.18), true);
    bundle.prints.get_edited_preset().config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;

    // Keep the source preset as the parent so get_project_embedded_presets can
    // emit a meaningful child diff; project embedding and detaching are separate
    // contracts in the public API.
    bundle.prints.save_current_preset("Embedded Process", false, true);
    const Preset *embedded = bundle.prints.find_preset("Embedded Process");
    REQUIRE(embedded != nullptr);
    CHECK(embedded->is_project_embedded);
    CHECK(embedded->inherits() == "Editable Process");
    CHECK(embedded->config.opt_string("print_settings_id") == "Embedded Process");

    std::vector<Preset *> project_presets = bundle.prints.get_project_embedded_presets();
    const ScopeGuard free_presets([&project_presets] {
        for (Preset *preset : project_presets)
            delete preset;
    });
    REQUIRE(project_presets.size() == 1);
    CHECK(project_presets.front()->name == "Embedded Process");
    REQUIRE(bundle.prints.reset_project_embedded_presets());
    CHECK(bundle.prints.find_preset("Embedded Process") == nullptr);
}

TEST_CASE("Preset deletion protects system entries and removes user files when allowed", "[Preset][Bundle][PresetLifecycle][Delete]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    Preset &system = bundle.prints.load_preset("", "System Process", print_config(bundle, 0.20), false);
    system.is_system = true;
    CHECK_FALSE(bundle.prints.delete_preset("System Process"));
    CHECK(bundle.prints.find_preset("System Process") != nullptr);
    REQUIRE(bundle.prints.delete_preset("System Process", true));
    CHECK(bundle.prints.find_preset("System Process") == nullptr);

    const fs::path user_file = scoped.directory.path() / PRESET_PRINT_NAME / "User Process.json";
    const DynamicPrintConfig user_config = print_config(bundle, 0.22);
    write_print_preset(user_config, user_file, "User Process");
    bundle.prints.load_preset(user_file.string(), "User Process", user_config, false);
    REQUIRE(fs::exists(user_file));
    REQUIRE(bundle.prints.delete_preset("User Process"));
    CHECK(bundle.prints.find_preset("User Process") == nullptr);
    CHECK_FALSE(fs::exists(user_file));
    CHECK_FALSE(bundle.prints.delete_preset("does-not-exist"));
}

TEST_CASE("Preset selection export and load preserve multi-filament machine state", "[Preset][Bundle][PresetLifecycle][Selection]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    bundle.printers.load_preset("", "Selection Printer", printer_config(bundle, "selection-model"), false);
    bundle.prints.load_preset("", "Selection Process", print_config(bundle, 0.23), false);
    bundle.filaments.load_preset("", "Selection PLA", filament_config(bundle, "Vendor", "Selection Printer"), false);
    bundle.filaments.load_preset("", "Selection PETG", filament_config(bundle, "Vendor", "Selection Printer"), false);
    bundle.printers.select_preset_by_name("Selection Printer", true);
    bundle.prints.select_preset_by_name("Selection Process", true);
    bundle.filaments.select_preset_by_name("Selection PLA", true);
    bundle.filament_presets = {"Selection PLA", "Selection PETG"};
    bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values = {"#AA0000", "#00AA00"};
    bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values = {"#AA0000", "#00AA00"};
    bundle.project_config.opt<ConfigOptionStrings>("filament_colour_type")->values = {"1", "1"};

    AppConfig config;
    bundle.export_selections(config);
    CHECK(config.get("presets", PRESET_PRINTER_NAME) == "Selection Printer");
    CHECK(config.get_printer_setting("Selection Printer", PRESET_PRINT_NAME) == "Selection Process");
    CHECK(config.get_printer_setting("Selection Printer", PRESET_FILAMENT_NAME) == "Selection PLA");
    CHECK(config.get_printer_setting("Selection Printer", "filament_01") == "Selection PETG");
    CHECK(config.get_printer_setting("Selection Printer", "filament_colors") == "#AA0000,#00AA00");

    bundle.printers.select_preset(0);
    bundle.prints.select_preset(0);
    bundle.filaments.select_preset(0);
    bundle.load_selections(config);
    CHECK(bundle.printers.get_selected_preset_name() == "Selection Printer");
    CHECK(bundle.prints.get_selected_preset_name() == "Selection Process");
    CHECK(bundle.filament_presets == std::vector<std::string>{"Selection PLA", "Selection PETG"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#AA0000", "#00AA00"});
}

TEST_CASE("External preset loading selects a source profile without losing project values", "[Preset][Bundle][PresetLifecycle][External]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    DynamicPrintConfig external = print_config(bundle, 0.31);
    external.opt<ConfigOptionString>("print_settings_id", true)->value = "External Source";
    const std::set<std::string> different{"layer_height"};
    ScopedTemporaryFile source_file(".gcode");
    auto loaded = bundle.prints.load_external_preset(
        source_file.string(), "Imported Process", "", external, different,
        PresetCollection::LoadAndSelect::Always);
    REQUIRE(loaded.first != nullptr);
    CHECK_FALSE(loaded.second);
    CHECK(loaded.first->is_external);
    CHECK(bundle.prints.get_selected_preset_name() == "(Imported Process)");
    CHECK_THAT(loaded.first->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
}

TEST_CASE("Preset differencing emits only child values and preserves parent identity", "[Preset][Bundle][PresetLifecycle][Diff]")
{
    ScopedPresetBundle scoped;
    PresetBundle &bundle = scoped.bundle;
    Preset &parent = bundle.prints.load_preset("", "Diff Parent", print_config(bundle, 0.20), false);
    const std::string parent_name = parent.name;
    parent.setting_id = "parent-setting-id";
    DynamicPrintConfig child_config = parent.config;
    child_config.opt<ConfigOptionFloat>("layer_height")->value = 0.28;
    child_config.opt<ConfigOptionString>("inherits", true)->value = parent_name;
    Preset &child = bundle.prints.load_preset("", "Diff Child", child_config, false);
    child.base_id = "parent-setting-id";

    std::map<std::string, std::string> values;
    const int changed = bundle.prints.get_differed_values_to_update(child, values);
    // The legacy API populates key_values and returns zero on success; callers
    // use the map as the result and reserve negative values for invalid presets.
    CHECK(changed == 0);
    CHECK(values.find("layer_height") != values.end());
    CHECK(values.at("layer_height") == "0.28");
    std::unique_ptr<Preset> differed(bundle.prints.get_preset_differed_for_save(child));
    REQUIRE(differed != nullptr);
    CHECK(differed->inherits() == parent_name);
    CHECK_THAT(differed->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));
}

} // namespace preset_lifecycle_contracts

namespace preset_bundle_workflow_contracts {
using namespace Slic3r;

namespace {

namespace fs = boost::filesystem;

struct ScopedDataDir {
    std::string previous = data_dir();
    explicit ScopedDataDir(const fs::path &path) { set_data_dir(path.string()); }
    ~ScopedDataDir() { set_data_dir(previous); }
};

DynamicPrintConfig print_config(PresetBundle &bundle, const std::string &name, double layer_height)
{
    DynamicPrintConfig config = bundle.prints.default_preset().config;
    config.option<ConfigOptionString>("print_settings_id", true)->value = name;
    config.option<ConfigOptionFloat>("layer_height", true)->value = layer_height;
    return config;
}

DynamicPrintConfig printer_config(PresetBundle &bundle, const std::string &name)
{
    DynamicPrintConfig config = bundle.printers.default_preset().config;
    config.option<ConfigOptionString>("printer_settings_id", true)->value = name;
    config.option<ConfigOptionString>("printer_model", true)->value = "Workflow Printer";
    config.option<ConfigOptionString>("printer_variant", true)->value = "0.4";
    return config;
}

DynamicPrintConfig filament_config(PresetBundle &bundle, const std::string &name,
                                   const std::string &printer)
{
    DynamicPrintConfig config = bundle.filaments.default_preset().config;
    config.option<ConfigOptionStrings>("filament_settings_id", true)->values = {name};
    config.option<ConfigOptionStrings>("filament_vendor", true)->values = {"Workflow Vendor"};
    config.option<ConfigOptionStrings>("filament_type", true)->values = {"PLA"};
    config.option<ConfigOptionStrings>("compatible_printers", true)->values = {printer};
    config.option<ConfigOptionString>("compatible_printers_condition", true)->value.clear();
    return config;
}

void write_json_preset(const fs::path &file, const DynamicPrintConfig &config,
    const std::string &name)
{
    fs::create_directories(file.parent_path());
    config.save_to_json(file.string(), name, "User", "1.0.0");
    REQUIRE(fs::exists(file));
}

std::string read_text(const fs::path &file)
{
    std::ifstream in(file.string(), std::ios::binary);
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

void write_gcode_config(const fs::path &file, double layer_height, const std::string &colour)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.option<ConfigOptionFloat>("layer_height", true)->value = layer_height;
    config.option<ConfigOptionStrings>("filament_colour", true)->values = {colour};
    config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75};
    std::ofstream out(file.string());
    out << "; generated by OrcaSlicer\n"
        << "; CONFIG_BLOCK_START\n";
    for (const std::string &key : config.keys())
        out << "; " << key << " = " << config.opt_serialize(key) << "\n";
    out << "; CONFIG_BLOCK_END\n";
    out << "G1 X10 Y10 F1200\n";
}

std::map<std::string, std::string> cloud_values(const std::string &type,
                                                const std::string &setting_id,
                                                const std::string &user_id,
                                                const std::string &inherits = {})
{
    std::map<std::string, std::string> values{
        {BBL_JSON_KEY_TYPE, type},
        {BBL_JSON_KEY_VERSION, "1.0.0"},
        {BBL_JSON_KEY_SETTING_ID, setting_id},
        {BBL_JSON_KEY_USER_ID, user_id},
        {ORCA_JSON_KEY_UPDATE_TIME, "10"},
    };
    if (!inherits.empty())
        values.emplace(BBL_JSON_KEY_INHERITS, inherits);
    return values;
}

BundleMetadata subscribed_metadata(const std::string &id)
{
    BundleMetadata metadata;
    metadata.id = id;
    metadata.name = "Workflow cloud bundle";
    metadata.version = "1.0.0";
    metadata.description = "native workflow fixture";
    metadata.author = "test";
    metadata.updated_time = 42;
    return metadata;
}

} // namespace

TEST_CASE("current config export writes selected user profiles and can be reloaded", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-export");
    ScopedDataDir data(temp.path() / "data");

    PresetBundle bundle;
    bundle.printers.load_preset("", "Workflow Printer", printer_config(bundle, "Workflow Printer"), true);
    bundle.prints.load_preset("", "Workflow Process", print_config(bundle, "Workflow Process", 0.23), true);
    bundle.filaments.load_preset("", "Workflow PLA", filament_config(bundle, "Workflow PLA", "Workflow Printer"), true);
    bundle.filament_presets = {"Workflow PLA"};

    const fs::path out = temp.path() / "exported";
    fs::create_directories(out);
    const std::vector<std::string> files = bundle.export_current_configs(
        out.string(), [](std::string const &) { return 1; }, false);

    REQUIRE(files.size() == 3);
    for (const std::string &file : files)
        CHECK(fs::exists(file));

    DynamicPrintConfig imported;
    std::map<std::string, std::string> keys;
    std::string reason;
    REQUIRE_NOTHROW(imported.load_from_json((out / "Workflow Process.json").string(),
                                             ForwardCompatibilitySubstitutionRule::Disable,
                                             keys, reason));
    CHECK_THAT(imported.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.23, 1e-9));
    CHECK(keys.at(BBL_JSON_KEY_NAME) == "Workflow Process");
    CHECK(read_text(out / "Workflow Process.json").find("Workflow Process") != std::string::npos);
}

TEST_CASE("current config export chooses edited values only when requested", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-export-edited");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.printers.load_preset("", "Edited Printer", printer_config(bundle, "Edited Printer"), true);
    bundle.prints.load_preset("", "Edited Process", print_config(bundle, "Edited Process", 0.20), true);
    bundle.filaments.load_preset("", "Edited PLA", filament_config(bundle, "Edited PLA", "Edited Printer"), true);
    bundle.filament_presets = {"Edited PLA"};

    bundle.prints.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.31;
    const fs::path selected_dir = temp.path() / "selected";
    const fs::path edited_dir = temp.path() / "edited";
    fs::create_directories(selected_dir);
    fs::create_directories(edited_dir);
    bundle.export_current_configs(selected_dir.string(), [](std::string const &) { return 1; }, false);
    bundle.export_current_configs(edited_dir.string(), [](std::string const &) { return 1; }, true);

    DynamicPrintConfig selected;
    DynamicPrintConfig edited;
    std::map<std::string, std::string> selected_keys;
    std::map<std::string, std::string> edited_keys;
    std::string selected_reason;
    std::string edited_reason;
    selected.load_from_json((selected_dir / "Edited Process.json").string(),
                            ForwardCompatibilitySubstitutionRule::Disable, selected_keys, selected_reason);
    edited.load_from_json((edited_dir / "Edited Process.json").string(),
                          ForwardCompatibilitySubstitutionRule::Disable, edited_keys, edited_reason);
    CHECK_THAT(selected.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.20, 1e-9));
    CHECK_THAT(edited.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
}

TEST_CASE("current config export honors a per-file overwrite decision", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-overwrite");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.printers.load_preset("", "Overwrite Printer", printer_config(bundle, "Overwrite Printer"), true);
    bundle.prints.load_preset("", "Overwrite Process", print_config(bundle, "Overwrite Process", 0.19), true);
    bundle.filaments.load_preset("", "Overwrite PLA", filament_config(bundle, "Overwrite PLA", "Overwrite Printer"), true);
    bundle.filament_presets = {"Overwrite PLA"};
    const fs::path out = temp.path() / "out";
    fs::create_directories(out);
    bundle.export_current_configs(out.string(), [](std::string const &) { return 1; }, false);

    bundle.prints.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.37;
    const std::vector<std::string> skipped = bundle.export_current_configs(
        out.string(), [](std::string const &) { return 0; }, true);
    CHECK(skipped.empty());

    DynamicPrintConfig after_skip;
    std::map<std::string, std::string> keys;
    std::string reason;
    after_skip.load_from_json((out / "Overwrite Process.json").string(),
                              ForwardCompatibilitySubstitutionRule::Disable, keys, reason);
    CHECK_THAT(after_skip.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.19, 1e-9));
}

TEST_CASE("current config export omits built-in default selections", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-export-default");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.set_num_filaments(1, "#FFFFFF");
    const fs::path out = temp.path() / "out";
    fs::create_directories(out);
    const std::vector<std::string> files = bundle.export_current_configs(
        out.string(), [](std::string const &) { return 1; }, false);
    CHECK(files.empty());
    CHECK(fs::directory_iterator(out) == fs::directory_iterator());
}

TEST_CASE("JSON preset import installs a user profile under the scoped data directory", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-import");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle source;
    const fs::path file = temp.path() / "incoming" / "Imported Process.json";
    DynamicPrintConfig config = print_config(source, "Imported Process", 0.27);
    write_json_preset(file, config, "Imported Process");

    PresetBundle bundle;
    AppConfig app;
    std::vector<std::string> files{file.string()};
    const PresetsConfigSubstitutions substitutions = bundle.import_presets(
        files, [](std::string const &) { return 1; },
        ForwardCompatibilitySubstitutionRule::Disable, app);

    const Preset *imported = bundle.prints.find_preset("Imported Process");
    REQUIRE(imported != nullptr);
    CHECK(imported->is_external);
    CHECK_THAT(imported->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.27, 1e-9));
    CHECK(fs::exists(imported->file));
    CHECK(files.size() == 1);
    CHECK(substitutions.empty());
    CHECK(imported->file.find((temp.path() / "data").string()) == 0);
}

TEST_CASE("Preset bundle ZIP import preserves bundle identity and per-type files", "[PresetBundle][PresetBundleWorkflowsExtended][PresetBundleImport]")
{
    ScopedTemporaryDir temp("preset-workflow-bundle-import");
    ScopedDataDir      data(temp.path() / "data");
    PresetBundle       source;

    const std::string bundle_id = "bundle-import-id";
    const fs::path incoming_dir = temp.path() / "incoming";
    const fs::path process_file = incoming_dir / "Bundle Process.json";
    const fs::path filament_file = incoming_dir / "Bundle PLA.json";
    const fs::path printer_file = incoming_dir / "Bundle Printer.json";
    write_json_preset(process_file, print_config(source, "Bundle Process", 0.29), "Bundle Process");
    write_json_preset(filament_file, filament_config(source, "Bundle PLA", "Bundle Printer"), "Bundle PLA");
    write_json_preset(printer_file, printer_config(source, "Bundle Printer"), "Bundle Printer");

    const fs::path archive = incoming_dir / "workflow.orca_bundle";
    write_zip(archive, {
        {BUNDLE_STRUCTURE_JSON_NAME,
         R"({"id":"bundle-import-id","name":"Imported Workflow","version":"1.2.3",)"
         R"("description":"three-type import","author":"test"})"},
        {"profiles/process/Bundle Process.json", read_text(process_file)},
        {"profiles/filament/Bundle PLA.json", read_text(filament_file)},
        {"profiles/printer/Bundle Printer.json", read_text(printer_file)},
    });

    PresetBundle bundle;
    AppConfig    app_config;
    std::vector<std::string> files{archive.string()};
    const PresetsConfigSubstitutions substitutions = bundle.import_presets(
        files, [](std::string const &) { return 1; },
        ForwardCompatibilitySubstitutionRule::Disable, app_config);

    CHECK(substitutions.empty());
    REQUIRE(files.size() == 3);

    const fs::path bundle_root = temp.path() / "data" / PRESET_USER_DIR / DEFAULT_USER_FOLDER_NAME /
                                 PRESET_LOCAL_DIR / bundle_id;
    const fs::path metadata_file = bundle_root / PRESET_BUNDLE_METADATA;
    REQUIRE(fs::exists(metadata_file));
    for (const auto &[type, name] : std::vector<std::pair<const char *, const char *>>{
             {PRESET_PRINT_NAME, "Bundle Process.json"},
             {PRESET_FILAMENT_NAME, "Bundle PLA.json"},
             {PRESET_PRINTER_NAME, "Bundle Printer.json"}})
        CHECK(fs::exists(bundle_root / type / name));

    BundleMetadata stored;
    REQUIRE(stored.load_from_json(metadata_file.string()));
    CHECK(stored.id == bundle_id);
    CHECK(stored.name == "Imported Workflow");
    CHECK(stored.version == "1.2.3");
    CHECK(stored.description == "three-type import");
    CHECK(stored.author == "test");

    REQUIRE(bundle.bundles.m_bundles.size() == 1);
    const auto found = bundle.bundles.m_bundles.find(bundle_id);
    REQUIRE(found != bundle.bundles.m_bundles.end());
    const BundleMetadata &loaded = found->second;
    CHECK(loaded.bundle_type == BundleType::Local);
    CHECK_FALSE(loaded.is_subscribed);
    CHECK_FALSE(loaded.update_available);
    CHECK(fs::equivalent(fs::path(loaded.path), metadata_file));

    const std::string prefix = "_local/" + bundle_id + "/";
    const Preset *process = bundle.prints.find_preset(prefix + "Bundle Process", false);
    const Preset *filament = bundle.filaments.find_preset(prefix + "Bundle PLA", false);
    const Preset *printer = bundle.printers.find_preset(prefix + "Bundle Printer", false);
    REQUIRE(process != nullptr);
    REQUIRE(filament != nullptr);
    REQUIRE(printer != nullptr);
    for (const Preset *preset : {process, filament, printer}) {
        CHECK(preset->is_from_bundle());
        CHECK(preset->bundle_id == bundle_id);
        CHECK(preset->is_external);
    }
    CHECK_THAT(process->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.29, 1e-9));
    CHECK(filament->config.opt_string("filament_type", 0u) == "PLA");
    CHECK(printer->config.opt_string("printer_model") == "Workflow Printer");
    CHECK(fs::equivalent(fs::path(process->file), bundle_root / PRESET_PRINT_NAME / "Bundle Process.json"));
    CHECK(fs::equivalent(fs::path(filament->file), bundle_root / PRESET_FILAMENT_NAME / "Bundle PLA.json"));
    CHECK(fs::equivalent(fs::path(printer->file), bundle_root / PRESET_PRINTER_NAME / "Bundle Printer.json"));
}

TEST_CASE("JSON preset import rejects an unknown type without writing a profile", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-import-invalid");
    ScopedDataDir data(temp.path() / "data");
    const fs::path file = temp.path() / "incoming" / "Unknown.json";
    fs::create_directories(file.parent_path());
    std::ofstream(file.string()) << R"({"type":"future_profile","name":"Unknown","version":"1.0.0","layer_height":"0.2"})";

    PresetBundle bundle;
    AppConfig app;
    std::vector<std::string> files{file.string()};
    bundle.import_presets(files, [](std::string const &) { return 1; },
                          ForwardCompatibilitySubstitutionRule::Disable, app);
    CHECK(bundle.prints.find_preset("Unknown") == nullptr);
    CHECK(bundle.printers.find_preset("Unknown") == nullptr);
    CHECK(bundle.filaments.find_preset("Unknown") == nullptr);
    CHECK(files.empty());
}

TEST_CASE("JSON preset import rejects a name outside the scoped preset directory", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-import-confinement");
    ScopedDataDir data(temp.path() / "data");
    const fs::path file = temp.path() / "incoming" / "Escape.json";
    PresetBundle source;
    write_json_preset(file, print_config(source, "../../../../escape", 0.28), "../../../../escape");

    PresetBundle bundle;
    AppConfig app;
    std::vector<std::string> files{file.string()};
    bundle.import_presets(files, [](std::string const &) { return 1; },
                          ForwardCompatibilitySubstitutionRule::Disable, app);
    CHECK(files.empty());
    CHECK(bundle.prints.find_preset("../../../../escape") == nullptr);
    CHECK_FALSE(fs::exists(temp.path() / "escape.json"));
}

TEST_CASE("JSON preset import preserves the existing profile when overwrite is declined", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-import-overwrite");
    ScopedDataDir data(temp.path() / "data");
    const fs::path file = temp.path() / "incoming" / "Stable Process.json";
    PresetBundle source;
    write_json_preset(file, print_config(source, "Stable Process", 0.26), "Stable Process");

    PresetBundle bundle;
    AppConfig app;
    std::vector<std::string> first{file.string()};
    bundle.import_presets(first, [](std::string const &) { return 1; },
                          ForwardCompatibilitySubstitutionRule::Disable, app);
    const Preset *initial = bundle.prints.find_preset("Stable Process");
    REQUIRE(initial != nullptr);
    CHECK_THAT(initial->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.26, 1e-9));

    write_json_preset(file, print_config(source, "Stable Process", 0.39), "Stable Process");
    std::vector<std::string> second{file.string()};
    bundle.import_presets(second, [](std::string const &) { return 0; },
                          ForwardCompatibilitySubstitutionRule::Disable, app);
    CHECK(second.empty());
    const Preset *preserved = bundle.prints.find_preset("Stable Process");
    REQUIRE(preserved != nullptr);
    CHECK_THAT(preserved->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.26, 1e-9));
}

TEST_CASE("G-code config loading activates an external profile from recognized metadata", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-gcode");
    ScopedDataDir data(temp.path() / "data");
    const fs::path gcode = temp.path() / "metadata.gcode";
    write_gcode_config(gcode, 0.27, "#112233");

    preset_lifecycle_contracts::ScopedPresetBundle scoped_bundle;
    PresetBundle &bundle = scoped_bundle.bundle;
    REQUIRE_NOTHROW(bundle.load_config_file(gcode.string(), ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(bundle.filament_presets.size() == 1);
    CHECK(bundle.prints.get_selected_preset().is_external);
    CHECK_THAT(bundle.prints.get_edited_preset().config.opt_float("layer_height"),
               Catch::Matchers::WithinAbs(0.27, 1e-9));
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#112233"});
}

TEST_CASE("Unknown configuration extensions fail closed", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-unknown-config");
    ScopedDataDir data(temp.path() / "data");
    const fs::path file = temp.path() / "settings.json";
    std::ofstream(file.string()) << "{}";
    PresetBundle bundle;
    REQUIRE_THROWS(bundle.load_config_file(file.string(), ForwardCompatibilitySubstitutionRule::Disable));
}

TEST_CASE("save_changes_for_preset reverts unselected edits and embeds the saved copy", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-save");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    fs::create_directories(temp.path() / "user" / "default");
    bundle.prints.update_user_presets_directory((temp.path() / "user" / "default").string(), PRESET_PRINT_NAME);
    bundle.prints.load_preset("", "Save Parent", print_config(bundle, "Save Parent", 0.20), true);
    bundle.prints.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.34;

    bundle.save_changes_for_preset("Embedded Save", Preset::TYPE_PRINT, {"layer_height"}, true);
    const Preset *saved = bundle.prints.find_preset("Embedded Save");
    REQUIRE(saved != nullptr);
    CHECK(saved->is_project_embedded);
    CHECK_THAT(saved->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.20, 1e-9));
    // Project-embedded presets are serialized by the project writer and deliberately do not
    // write a user-library file; Preset::save() is a no-op for this origin.
    CHECK_FALSE(fs::exists(saved->file));

    REQUIRE(bundle.prints.reset_project_embedded_presets());
    CHECK(bundle.prints.find_preset("Embedded Save") == nullptr);
}

TEST_CASE("default suppression is applied consistently to all preset collections", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    PresetBundle bundle;
    bundle.set_default_suppressed(true);
    CHECK(bundle.prints.is_default_suppressed());
    CHECK(bundle.filaments.is_default_suppressed());
    CHECK(bundle.printers.is_default_suppressed());
    CHECK(bundle.sla_prints.is_default_suppressed());
    CHECK(bundle.sla_materials.is_default_suppressed());
    bundle.set_default_suppressed(false);
    CHECK_FALSE(bundle.prints.is_default_suppressed());
    CHECK_FALSE(bundle.filaments.is_default_suppressed());
    CHECK_FALSE(bundle.printers.is_default_suppressed());
}

TEST_CASE("local bundle metadata and preset identity survive user loading", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-local");
    ScopedDataDir data(temp.path() / "data");
    const fs::path local = temp.path() / "data" / PRESET_USER_DIR / "default" / PRESET_LOCAL_DIR / "local-id";
    fs::create_directories(local);

    BundleMetadata metadata = subscribed_metadata("local-id");
    metadata.name = "Local workflow bundle";
    REQUIRE(metadata.save_to_json((local / PRESET_BUNDLE_METADATA).string()));
    PresetBundle writer;
    write_json_preset(local / PRESET_PRINT_NAME / "Local Process.json",
                     print_config(writer, "Local Process", 0.24), "Local Process");

    PresetBundle bundle;
    const PresetsConfigSubstitutions substitutions = bundle.load_user_presets(
        "default", ForwardCompatibilitySubstitutionRule::Disable);
    const Preset *loaded = bundle.prints.find_preset("_local/local-id/Local Process");
    REQUIRE(loaded != nullptr);
    CHECK(loaded->is_from_bundle());
    CHECK(loaded->bundle_id == "local-id");
    CHECK_THAT(loaded->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    CHECK(substitutions.empty());
}

TEST_CASE("Subscribed bundle reload rebuilds typed preset membership from disk",
          "[PresetBundle][SubscribedBundleReload]")
{
    const bool read_only = GENERATE(false, true);
    CAPTURE(read_only);
    ScopedTemporaryDir temp("subscribed-reload");
    ScopedDataDir data(temp.path() / "data");
    const fs::path directory = temp.path() / "data" / PRESET_USER_DIR / "default" /
                               PRESET_SUBSCRIBED_DIR / "remote-reload";
    fs::create_directories(directory);
    const std::string prefix = "_subscribed/remote-reload/";
    BundleMetadata metadata = subscribed_metadata("remote-reload");
    // Cached membership from the last session is stale; files are authoritative on reload.
    metadata.print_presets = { "Stale Process" };
    metadata.filament_presets = { "Stale Filament" };
    metadata.printer_presets = { "Stale Printer" };
    const fs::path metadata_file = directory / PRESET_BUNDLE_METADATA;
    REQUIRE(metadata.save_to_json(metadata_file.string()));
    const std::string metadata_before = read_text(metadata_file);

    PresetBundle writer;
    write_json_preset(directory / PRESET_PRINT_NAME / "Reload Process.json",
                      print_config(writer, "Reload Process", 0.24), "Reload Process");
    DynamicPrintConfig material = filament_config(writer, "Reload PLA", prefix + "Reload Printer");
    material.option<ConfigOptionFloats>("filament_density", true)->values = { 1.27 };
    write_json_preset(directory / PRESET_FILAMENT_NAME / "Reload PLA.json", material, "Reload PLA");
    DynamicPrintConfig machine = printer_config(writer, "Reload Printer");
    machine.option<ConfigOptionFloats>("nozzle_diameter", true)->values = { 0.4 };
    write_json_preset(directory / PRESET_PRINTER_NAME / "Reload Printer.json", machine, "Reload Printer");

    PresetBundle loaded;
    REQUIRE(loaded.load_user_presets("default", ForwardCompatibilitySubstitutionRule::Disable,
                                    read_only).empty());
    const Preset *process = loaded.prints.find_preset(prefix + "Reload Process", false);
    const Preset *filament = loaded.filaments.find_preset(prefix + "Reload PLA", false);
    const Preset *printer = loaded.printers.find_preset(prefix + "Reload Printer", false);
    REQUIRE(process != nullptr);
    REQUIRE(filament != nullptr);
    REQUIRE(printer != nullptr);
    for (const Preset *preset : { process, filament, printer }) {
        CHECK(preset->is_from_bundle());
        CHECK(preset->bundle_id == "remote-reload");
    }
    CHECK_THAT(process->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    REQUIRE(filament->config.option<ConfigOptionFloats>("filament_density") != nullptr);
    REQUIRE(filament->config.option<ConfigOptionFloats>("filament_density")->values.size() == 1);
    CHECK_THAT(filament->config.option<ConfigOptionFloats>("filament_density")->values.front(),
               Catch::Matchers::WithinAbs(1.27, 1e-9));
    CHECK(printer->config.opt_string("printer_model") == "Workflow Printer");

    REQUIRE(loaded.bundles.m_bundles.size() == 1);
    const auto found = loaded.bundles.m_bundles.find("remote-reload");
    REQUIRE(found != loaded.bundles.m_bundles.end());
    const BundleMetadata &reloaded = found->second;
    CHECK(reloaded.is_subscribed);
    CHECK(reloaded.bundle_type == BundleType::Subscribed);
    CHECK_FALSE(reloaded.update_available);
    CHECK(reloaded.name == metadata.name);
    CHECK(reloaded.version == metadata.version);
    CHECK(reloaded.updated_time == metadata.updated_time);
    CHECK(fs::equivalent(reloaded.path, metadata_file));
    CHECK(reloaded.print_presets == std::vector<std::string>{ prefix + "Reload Process" });
    CHECK(reloaded.filament_presets == std::vector<std::string>{ prefix + "Reload PLA" });
    CHECK(reloaded.printer_presets == std::vector<std::string>{ prefix + "Reload Printer" });
    if (read_only)
        CHECK(read_text(metadata_file) == metadata_before);
}

TEST_CASE("subscribed bundle update writes metadata and prunes obsolete presets", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    ScopedTemporaryDir temp("preset-workflow-subscribed");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    AppConfig app;
    BundleMetadata metadata = subscribed_metadata("remote-id");

    std::map<std::string, std::map<std::string, std::string>> first;
    auto root = cloud_values(PRESET_IOT_PRINT_TYPE, "remote-process-id", "account");
    root.emplace("layer_height", "0.26");
    first.emplace("Remote Process", root);
    bundle.update_subscribed_presets(app, first, metadata, ForwardCompatibilitySubstitutionRule::EnableSilent);

    const Preset *remote = bundle.prints.find_preset("_subscribed/remote-id/Remote Process");
    REQUIRE(remote != nullptr);
    const fs::path remote_file(remote->file);
    CHECK(remote->is_from_bundle());
    CHECK(remote->bundle_id == "remote-id");
    CHECK_THAT(remote->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.26, 1e-9));
    CHECK(fs::exists(temp.path() / "data" / PRESET_USER_DIR / "default" / PRESET_SUBSCRIBED_DIR /
                     "remote-id" / PRESET_BUNDLE_METADATA));

    const std::map<std::string, std::map<std::string, std::string>> empty;
    bundle.update_subscribed_presets(app, empty, metadata, ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(bundle.prints.find_preset("_subscribed/remote-id/Remote Process") == nullptr);
    CHECK_FALSE(fs::exists(remote_file));
}

TEST_CASE("filament selection canonicalizes the actual dirty preset label", "[PresetBundle][PresetBundleWorkflowsExtended]")
{
    PresetBundle bundle;
    bundle.set_num_filaments(1, "#FF0000");
    Preset dirty(Preset::TYPE_FILAMENT, "Selected PLA");
    dirty.set_dirty();
    const std::string displayed = dirty.label(true);
    CHECK(displayed == Preset::suffix_modified() + "Selected PLA");
    bundle.set_filament_preset(0, displayed);
    CHECK(bundle.filament_presets.front() == "Selected PLA");
    const std::vector<std::string> before = bundle.filament_presets;
    bundle.set_filament_preset(bundle.filament_presets.size(), "ignored");
    CHECK(bundle.filament_presets == before);
}

} // namespace preset_bundle_workflow_contracts

namespace preset_workflow_contracts {
using Catch::Matchers::WithinAbs;

void write_text(const fs::path &path, const std::string &text)
{
    fs::create_directories(path.parent_path());
    std::ofstream out(path.string(), std::ios::binary);
    REQUIRE(out.good());
    out << text;
    REQUIRE(out.good());
}

DynamicPrintConfig physical_defaults()
{
    PhysicalPrinterCollection collection(PhysicalPrinter::printer_options());
    return collection.default_config();
}


TEST_CASE("Vendor profile INI header and model fields are parsed semantically", "[PresetBundleLoading][PresetDeep][Vendor]")
{
    ScopedTemporaryDir dir("preset-deep-vendor");
    const fs::path file = dir.path() / "Acme.ini";
    write_text(file,
        "[vendor]\nname=Acme\nconfig_version=1.2.3\nconfig_update_url=https://example.invalid/update\n"
        "changelog_url=https://example.invalid/changelog\n"
        "[printer_model:Alpha]\nname=Alpha Prime\ntechnology=FFF\nfamily=AlphaFamily\n"
        "variants=0.4;0.6\ndefault_materials=PLA;PETG;PLA\n"
        "[printer_model:Resin]\nname=Resin\ntechnology=SLA\nvariants=0.05\n"
        "[default_filaments]\nPLA=1\nPETG=0\n[default_sla_materials]\nResin=1\n");

    const VendorProfile header = VendorProfile::from_ini(file, false);
    CHECK(header.id == "Acme");
    CHECK(header.name == "Acme");
    CHECK(header.config_version == *Semver::parse("1.2.3"));
    CHECK(header.config_update_url.find("update") != std::string::npos);
    CHECK(header.models.empty());

    const VendorProfile full = VendorProfile::from_ini(file, true);
    REQUIRE(full.models.size() == 2);
    CHECK(full.models[0].id == "Alpha");
    CHECK(full.models[0].technology == ptFFF);
    CHECK(full.models[0].variants.size() == 2);
    CHECK(full.models[0].default_materials == std::vector<std::string>{"PETG", "PLA"});
    CHECK(full.models[1].technology == ptSLA);
    CHECK(full.default_filaments == std::set<std::string>{"PLA"});
    CHECK(full.default_sla_materials == std::set<std::string>{"Resin"});
}

TEST_CASE("Vendor profile infers BBL families and rejects missing sections", "[PresetBundleLoading][PresetDeep][Vendor]")
{
    ScopedTemporaryDir dir("preset-deep-vendor-invalid");
    const fs::path file = dir.path() / "BBL.ini";
    write_text(file,
        "[vendor]\nname=BBL\nconfig_version=2.0.0\n"
        "[printer_model:MK3]\nvariants=0.4\n"
        "[printer_model:MK3MMU2]\nvariants=0.4\nfamily=Shared\n"
        "[printer_model:Other]\nvariants=0.4\nfamily=Other\n");
    const VendorProfile profile = VendorProfile::from_ini(file, true);
    REQUIRE(profile.models.size() == 3);
    CHECK(profile.models[0].family == "MK3");
    CHECK(profile.models[1].family == "Shared");
    CHECK(profile.families() == std::vector<std::string>{"MK3", "Shared", "Other"});

    const fs::path malformed = dir.path() / "bad.ini";
    write_text(malformed, "[vendor]\nname=Bad\n");
    CHECK_THROWS(VendorProfile::from_ini(malformed, false));
}

TEST_CASE("Preset type mappings and setting IDs are stable at boundaries", "[PresetBundleLoading][PresetDeep][Identity]")
{
    CHECK(Preset::get_type_string(Preset::TYPE_PRINT) == PRESET_PRINT_NAME);
    CHECK(Preset::get_type_string(Preset::TYPE_FILAMENT) == PRESET_FILAMENT_NAME);
    CHECK(Preset::get_type_string(Preset::TYPE_PRINTER) == PRESET_PRINTER_NAME);
    CHECK(Preset::get_type_string(Preset::TYPE_SLA_PRINT) == "invalid");
    CHECK(Preset::get_iot_type_string(Preset::TYPE_PRINT) == PRESET_IOT_PRINT_TYPE);
    CHECK(Preset::get_type_from_string("print") == Preset::TYPE_PRINT);
    CHECK(Preset::get_type_from_string("process") == Preset::TYPE_PRINT);
    CHECK(Preset::get_type_from_string("filament") == Preset::TYPE_FILAMENT);
    CHECK(Preset::get_type_from_string("machine") == Preset::TYPE_PRINTER);
    CHECK(Preset::get_type_from_string("unknown") == Preset::TYPE_INVALID);
    CHECK(generate_preset_setting_id("Afinia", "filament", "Afinia ABS @Afinia H400") == "TL34qSVkppBvMvgH");
    CHECK(generate_preset_setting_id("Acme", "process", "Fine") != generate_preset_setting_id("Acme", "filament", "Fine"));
    CHECK(generate_preset_setting_id("Acme", "process", "Fine") != generate_preset_setting_id("Other", "process", "Fine"));
    CHECK(generate_preset_setting_id("", "process", "Fine").empty());
    CHECK(generate_preset_setting_id("Acme", "process", "").empty());
}

TEST_CASE("Preset modified labels are reversible and aliases remain distinct", "[PresetBundleLoading][PresetDeep][Identity]")
{
    const std::string old_suffix = Preset::suffix_modified();
    const ScopeGuard restore_suffix([&old_suffix] { Preset::update_suffix_modified(old_suffix); });
    Preset::update_suffix_modified(" [dirty]");
    Preset preset(Preset::TYPE_PRINT, "Process");
    preset.alias = "Friendly";
    preset.set_dirty(true);
    CHECK(preset.label(false) == " [dirty]Friendly");
    CHECK(preset.label(true) == " [dirty]Process");
    CHECK(Preset::remove_suffix_modified(preset.label(true)) == "Process");
    CHECK(Preset::remove_suffix_modified("Process") == "Process");
}

TEST_CASE("Preset invalid-key removal reports and removes only unknown options", "[PresetBundleLoading][PresetDeep][Config]")
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("preset_deep_unknown", new ConfigOptionString("x"));
    config.set_key_value("layer_height", new ConfigOptionFloat(0.31));
    const std::string removed = Preset::remove_invalid_keys(config, DynamicPrintConfig::full_print_config());
    CHECK(removed == "preset_deep_unknown");
    CHECK_FALSE(config.has("preset_deep_unknown"));
    CHECK_THAT(config.opt_float("layer_height"), WithinAbs(0.31, 1e-6));
}

TEST_CASE("External config preserves project settings but filters print-host credentials", "[PresetBundleLoading][PresetDeep][External]")
{
    DynamicPrintConfig defaults = DynamicPrintConfig::full_print_config();
    defaults.set_key_value("print_host", new ConfigOptionString("default-host"));
    defaults.set_key_value("printhost_apikey", new ConfigOptionString("default-secret"));
    DynamicPrintConfig project = defaults;
    project.opt<ConfigOptionFloat>("layer_height")->value = 0.31;
    project.opt<ConfigOptionString>("print_host")->value = "project-host";
    project.opt<ConfigOptionString>("printhost_apikey")->value = "secret";
    t_config_option_keys keys;
    DynamicPrintConfig loaded = Preset::load_external_config(
        Preset::TYPE_PRINT, defaults, project, {}, [](const std::string &) { return static_cast<DynamicPrintConfig *>(nullptr); }, &keys);
    CHECK_THAT(loaded.opt_float("layer_height"), WithinAbs(0.31, 1e-6));
    REQUIRE(loaded.option("print_host") != nullptr);
    REQUIRE(loaded.option("printhost_apikey") != nullptr);
    CHECK(loaded.opt_string("print_host") == defaults.opt_string("print_host"));
    CHECK(loaded.opt_string("printhost_apikey") == defaults.opt_string("printhost_apikey"));
    CHECK(std::find(keys.begin(), keys.end(), "layer_height") != keys.end());
}

TEST_CASE("External config refreshes non-different values from its named parent", "[PresetBundleLoading][PresetDeep][External]")
{
    DynamicPrintConfig defaults = DynamicPrintConfig::full_print_config();
    defaults.set_key_value("inherits", new ConfigOptionString(""));
    DynamicPrintConfig parent = defaults;
    parent.opt<ConfigOptionFloat>("layer_height")->value = 0.30;
    DynamicPrintConfig project = defaults;
    project.opt_string("inherits", true) = "Base";
    project.opt<ConfigOptionFloat>("layer_height")->value = 0.20;
    project.opt<ConfigOptionInt>("wall_loops")->value = 7;
    std::set<std::string> different{"wall_loops"};
    DynamicPrintConfig loaded = Preset::load_external_config(
        Preset::TYPE_PRINT, defaults, project, different,
        [&](const std::string &name) { return name == "Base" ? &parent : nullptr; }, nullptr);
    CHECK_THAT(loaded.opt_float("layer_height"), WithinAbs(0.30, 1e-6));
    CHECK(loaded.opt_int("wall_loops") == 7);
}

TEST_CASE("Preset info metadata round-trips null normalization and timestamps", "[PresetBundleLoading][PresetDeep][Files]")
{
    ScopedTemporaryDir dir("preset-deep-info");
    ScopedDataDir data(dir.path() / "data");
    Preset saved(Preset::TYPE_PRINT, "User");
    saved.file = (dir.path() / "User.json").string();
    saved.sync_info = "update";
    saved.user_id = "u-1";
    saved.setting_id = "s-1";
    saved.base_id = "b-1";
    saved.updated_time = 123456;
    saved.save_info();
    Preset loaded(Preset::TYPE_PRINT, "User");
    loaded.load_info((dir.path() / "User.info").string());
    CHECK(loaded.sync_info == "update");
    CHECK(loaded.user_id == "u-1");
    CHECK(loaded.setting_id == "s-1");
    CHECK(loaded.base_id == "b-1");
    CHECK(loaded.updated_time == 123456);

    Preset bundled(Preset::TYPE_PRINT, "Bundled");
    bundled.bundle_id = "bundle-1";
    bundled.file = (dir.path() / "Bundled.json").string();
    bundled.save_info();
    CHECK_FALSE(fs::exists(dir.path() / "Bundled.info"));

    write_text(dir.path() / "Null.info", "setting_id = null\nbase_id = null\nupdated_time = 4\n");
    Preset nulls(Preset::TYPE_PRINT, "Null");
    nulls.load_info((dir.path() / "Null.info").string());
    CHECK(nulls.setting_id.empty());
    CHECK(nulls.base_id.empty());
    CHECK(nulls.updated_time == 4);
}

TEST_CASE("Preset parent save writes a diff that reload restores over the parent", "[PresetBundleLoading][PresetDeep][Files]")
{
    ScopedTemporaryDir dir("preset-deep-save");
    Preset parent(Preset::TYPE_PRINT, "Base", false);
    parent.config = DynamicPrintConfig::full_print_config();
    parent.config.opt<ConfigOptionInt>("wall_loops")->value = 7;
    Preset child(Preset::TYPE_PRINT, "Child", false);
    child.config = parent.config;
    child.inherits() = parent.name;
    child.config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;
    child.file = (dir.path() / "Child.json").string();
    REQUIRE(child.save(&parent.config));
    REQUIRE(fs::exists(child.file));
    std::string json;
    load_string_file(child.file, json);
    CHECK(json.find("layer_height") != std::string::npos);
    CHECK(json.find("wall_loops") == std::string::npos);
    parent.config.opt<ConfigOptionInt>("wall_loops")->value = 9;
    child.config = DynamicPrintConfig::full_print_config();
    child.reload(parent);
    CHECK_THAT(child.config.opt_float("layer_height"), WithinAbs(0.31, 1e-6));
    CHECK(child.config.opt_int("wall_loops") == 9);
}

TEST_CASE("Preset remove_files deletes local info and marks cloud info for deletion", "[PresetBundleLoading][PresetDeep][Files]")
{
    ScopedTemporaryDir dir("preset-deep-remove");
    ScopedDataDir data(dir.path() / "data");
    SECTION("local") {
        Preset preset(Preset::TYPE_PRINT, "Local");
        preset.file = (dir.path() / "Local.json").string();
        write_text(preset.file, "{}");
        preset.save_info();
        preset.remove_files();
        CHECK_FALSE(fs::exists(preset.file));
        CHECK_FALSE(fs::exists(dir.path() / "Local.info"));
    }
    SECTION("cloud") {
        Preset preset(Preset::TYPE_PRINT, "Cloud");
        preset.file = (dir.path() / "Cloud.json").string();
        preset.setting_id = "cloud-id";
        write_text(preset.file, "{}");
        preset.save_info();
        preset.remove_files(false);
        CHECK_FALSE(fs::exists(preset.file));
        REQUIRE(fs::exists(dir.path() / "Cloud.info"));
        Preset loaded(Preset::TYPE_PRINT, "Cloud");
        loaded.load_info((dir.path() / "Cloud.info").string());
        CHECK(loaded.sync_info == "delete");
    }
}

TEST_CASE("Physical printer preserves preset membership and legacy serialized names", "[PresetBundleLoading][PresetDeep][PhysicalPrinter]")
{
    DynamicPrintConfig defaults = physical_defaults();
    PhysicalPrinter printer("Office", defaults);
    CHECK(printer.add_preset("0.4"));
    CHECK_FALSE(printer.add_preset("0.4"));
    CHECK(printer.add_preset("0.6"));
    printer.update_preset_names_in_config();
    CHECK(printer.config.opt<ConfigOptionStrings>("preset_names")->values == std::vector<std::string>{"0.4", "0.6"});
    CHECK(printer.config.opt_string("preset_name") == "0.4;0.6");
    CHECK(printer.delete_preset("0.4"));
    CHECK_FALSE(printer.delete_preset("missing"));
    printer.reset_presets();
    CHECK(printer.get_preset_names().empty());
    CHECK(printer.get_full_name("0.4") == "Office * 0.4");
    CHECK(PhysicalPrinter::get_short_name("Office * 0.4") == "Office");
    CHECK(PhysicalPrinter::get_preset_name("Office * 0.4") == "0.4");
}

TEST_CASE("Physical printer save and collection load preserve typed settings", "[PresetBundleLoading][PresetDeep][PhysicalPrinter]")
{
    ScopedTemporaryDir dir("preset-deep-printer-files");
    ScopedDataDir data(dir.path() / "data");
    PhysicalPrinter printer("Office", physical_defaults());
    printer.file = (dir.path() / "Office.json").string();
    printer.add_preset("0.4");
    printer.config.opt<ConfigOptionString>("print_host")->value = "printer.local";
    printer.update_preset_names_in_config();
    printer.save(nullptr);
    REQUIRE(fs::exists(printer.file));

    PhysicalPrinterCollection loaded(PhysicalPrinter::printer_options());
    PresetsConfigSubstitutions substitutions;
    loaded.load_printers(dir.path().string(), "", substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    REQUIRE(std::distance(loaded.begin(), loaded.end()) == 1);
    const PhysicalPrinter &roundtrip = loaded.printer(0);
    CHECK(roundtrip.name == "Office");
    CHECK(roundtrip.preset_names == std::set<std::string>{"0.4"});
    CHECK(roundtrip.config.opt_string("print_host") == "printer.local");
}

TEST_CASE("Physical printer collection selection and preset filters are consistent", "[PresetBundleLoading][PresetDeep][PhysicalPrinter]")
{
    ScopedTemporaryDir dir("preset-deep-printer-selection");
    ScopedDataDir data(dir.path() / "data");
    PhysicalPrinterCollection collection(PhysicalPrinter::printer_options());
    auto add = [&](const std::string &name, std::initializer_list<const char*> presets) {
        DynamicPrintConfig config = collection.default_config();
        config.opt<ConfigOptionStrings>("preset_names")->values.clear();
        for (const char *preset : presets)
            config.opt<ConfigOptionStrings>("preset_names")->values.emplace_back(preset);
        collection.load_printer((dir.path() / (name + ".json")).string(), name, std::move(config), false);
    };
    add("A", {"0.4", "0.6"});
    add("B", {"0.4"});
    add("C", {"0.8", "0.4"});
    CHECK(collection.get_printers_with_preset("0.4") == std::vector<std::string>{"A", "C"});
    CHECK(collection.get_printers_with_only_preset("0.4") == std::vector<std::string>{"B"});
    collection.select_printer("A", "0.6");
    CHECK(collection.has_selection());
    CHECK(collection.get_selected_full_printer_name() == "A * 0.6");
    collection.select_printer("missing", "0.4");
    CHECK_FALSE(collection.has_selection());
    collection.select_printer("A * 0.4");
    REQUIRE(collection.has_selection());
    CHECK(collection.is_selected(collection.begin(), "0.4"));
    collection.unselect_printer();
    CHECK_FALSE(collection.delete_selected_printer());
}

TEST_CASE("Preset collection detached creation stays embedded without writing a user file", "[PresetBundleLoading][PresetDeep][Collection]")
{
    ScopedTemporaryDir dir("preset-deep-detached");
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    collection.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);
    const std::string name = collection.add_detached_preset("Detached", DynamicPrintConfig::full_print_config());
    REQUIRE(collection.find_preset(name) != nullptr);
    CHECK(collection.find_preset(name)->inherits().empty());
    CHECK(collection.find_preset(name)->is_project_embedded);
    CHECK_FALSE(fs::exists(collection.find_preset(name)->file));
    collection.select_preset_by_name(name, true);
    REQUIRE(collection.delete_current_preset());
    CHECK(collection.find_preset(name) == nullptr);
    CHECK(collection.has_defaults_only());
}

TEST_CASE("Preset collection aliases and default suppression select visible entries", "[PresetBundleLoading][PresetDeep][Collection]")
{
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    DynamicPrintConfig config = collection.default_preset().config;
    Preset &one = collection.load_preset({}, "One", config, false);
    one.alias.clear();
    CHECK(collection.get_preset_alias(one, true) == "One");
    CHECK(collection.get_preset_name_by_alias("One") == "One");
    CHECK(collection.is_alias_exist("One"));
    CHECK_FALSE(collection.is_alias_exist("Missing"));
    collection.set_default_suppressed(true);
    CHECK(collection.is_default_suppressed());
    CHECK(collection.first_visible().name == "One");
}

TEST_CASE("Preset bundle utility keys and legacy filament names are deterministic", "[PresetBundleLoading][PresetDeep][Bundle]")
{
    const std::set<std::string> keys = PresetBundle::project_different_keys("layer_height;\"wall_loops\";layer_height");
    CHECK(keys.count("layer_height") == 1);
    CHECK(keys.count("wall_loops") == 1);
    CHECK(keys.count("inherits") == 1);
    CHECK(keys.count("print_settings_id") == 1);
    CHECK(keys.count("printer_settings_id") == 1);

    std::string filament = "Bambu PLA Basic @BBL H2D";
    PresetBundle::convert_filament_preset_name("Bambu Lab H2D 0.6 nozzle", filament);
    CHECK(filament == "Bambu PLA Basic @BBL H2D 0.6 nozzle");
    std::string unchanged = "Never Mapped";
    PresetBundle::convert_filament_preset_name("Unknown Machine", unchanged);
    CHECK(unchanged == "Never Mapped");
}

TEST_CASE("Preset bundle vendor model and fallback asset lookup use configured roots", "[PresetBundleLoading][PresetDeep][Bundle]")
{
    ScopedTemporaryDir dir("preset-deep-assets");
    ScopedDataDir data(dir.path() / "data");
    ScopedResourcesDir resources(dir.path() / "resources");
    fs::create_directories(dir.path() / "resources" / "profiles" / "Acme");
    write_text(dir.path() / "resources" / "profiles" / "hotend.stl", "solid fallback\n");
    write_text(dir.path() / "resources" / "profiles" / "Acme" / "bed.stl", "solid bed\n");
    write_text(dir.path() / "data" / "vendor" / "Acme" / "bed_local.stl", "solid local\n");
    VendorProfile profile("Acme");
    profile.name = "Acme";
    VendorProfile::PrinterModel model;
    model.id = "M1";
    model.name = "Acme One";
    model.bed_model = "bed.stl";
    model.bed_texture = "bed.png";
    model.hotend_model = "hotend.stl";
    profile.models.push_back(model);
    PresetBundle bundle;
    bundle.vendors["Acme"] = profile;
    CHECK(bundle.get_stl_model_for_printer_model("Acme One") == (dir.path() / "resources" / "profiles" / "Acme" / "bed.stl").string());
    CHECK(bundle.get_hotend_model_for_printer_model("Acme One") ==
          (dir.path() / "resources" / "profiles" / "hotend.stl").string());
    CHECK(bundle.get_stl_model_for_printer_model("missing").empty());
    CHECK(bundle.get_texture_for_printer_model("missing").empty());
    CHECK(bundle.get_hotend_model_for_printer_model("missing") ==
          (dir.path() / "resources" / "profiles" / "hotend.stl").string());
}

TEST_CASE("Preset bundle filament sizing keeps maps and mixed metadata aligned", "[PresetBundleLoading][PresetDeep][Bundle][Filament]")
{
    PresetBundle bundle;
    bundle.filament_presets = {"A", "B"};
    bundle.set_num_filaments(3, "#ABCDEF");
    CHECK(bundle.filament_presets.size() == 3);
    CHECK(bundle.filament_presets[2] == "B");
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.size() == 3);
    CHECK(bundle.project_config.option<ConfigOptionStrings>("filament_colour")->get_at(2) == "#ABCDEF");
    CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values.size() == 3);
    CHECK(bundle.num_physical_filaments() == 3);
    CHECK(bundle.num_mixed_filaments() == 0);

    bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values = {false, false, true};
    bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values = {"", "", "1,2"};
    CHECK(bundle.is_mixed_filament(2));
    CHECK(bundle.num_mixed_filaments() == 1);
    CHECK(bundle.num_physical_filaments() == 2);
    bundle.update_num_filaments(0);
    CHECK(bundle.filament_presets.size() == 2);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.size() == 2);
}

TEST_CASE("Preset bundle physical filament indices exclude only live mixed slots", "[PresetBundleLoading][PresetDeep][Bundle][Filament]")
{
    PresetBundle bundle;
    bundle.filament_presets = {"A", "B", "C", "D"};
    bundle.project_config = DynamicPrintConfig::full_print_config();
    bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values = {"a", "b", "c", "d"};
    bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values = {false, true, false, true};
    const std::vector<size_t> indexes = bundle.physical_filament_config_indices();
    CHECK(indexes == std::vector<size_t>{0, 2});
    CHECK(bundle.num_mixed_filaments() == 2);
    CHECK(bundle.num_physical_filaments() == 2);
}

TEST_CASE("Preset bundle full config applies current project values over selected presets", "[PresetBundleLoading][PresetDeep][Bundle][Config]")
{
    PresetBundle bundle;
    bundle.project_config = DynamicPrintConfig::full_print_config();
    bundle.project_config.opt<ConfigOptionFloat>("layer_height")->value = 0.22;
    bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values = {"#00FF00"};
    bundle.filament_presets = {bundle.filaments.default_preset().name};
    DynamicPrintConfig config = bundle.full_config(false);
    CHECK_THAT(config.opt_float("layer_height"), WithinAbs(0.22, 1e-6));
    CHECK(config.option<ConfigOptionStrings>("filament_colour")->get_at(0) == "#00FF00");
    const bool no_inherits = config.option("inherits") == nullptr ||
                             config.opt_string("inherits").empty();
    CHECK(no_inherits);
}

TEST_CASE("Preset bundle resets embedded presets without deleting user presets", "[PresetBundleLoading][PresetDeep][Bundle][Project]")
{
    ScopedTemporaryDir dir("preset-deep-project");
    PresetBundle bundle;
    bundle.prints.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);
    DynamicPrintConfig embedded_config = bundle.prints.default_preset().config;
    Preset &embedded = bundle.prints.load_preset({}, "Embedded", embedded_config, false,
                                                 Semver::parse("1.0.0").value_or(Semver()));
    embedded.is_project_embedded = true;
    embedded.inherits() = bundle.prints.default_preset().name;
    embedded.config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;
    bundle.prints.select_preset_by_name("Embedded", true);
    const fs::path user_file = dir.path() / PRESET_PRINT_NAME / "User.json";
    Preset &user = bundle.prints.load_preset(user_file.string(), "User", bundle.prints.default_preset().config, false);
    user.config.opt<ConfigOptionFloat>("layer_height")->value = 0.27;
    REQUIRE(user.save(nullptr));
    REQUIRE(fs::exists(user_file));
    CHECK(bundle.prints.get_project_embedded_presets().size() == 1);
    CHECK(bundle.prints.reset_project_embedded_presets());
    CHECK(bundle.prints.find_preset("Embedded") == nullptr);
    REQUIRE(bundle.prints.find_preset("User") != nullptr);
    CHECK(fs::exists(user_file));
}

TEST_CASE("Deleting a printer preset updates physical printer membership and files", "[PresetBundleLoading][PresetDeep][PhysicalPrinter][PhysicalPrinterDeletion]")
{
    ScopedTemporaryDir dir("preset-deep-printer-delete");
    ScopedDataDir data(dir.path() / "data");
    PhysicalPrinterCollection collection(PhysicalPrinter::printer_options());
    REQUIRE(collection.default_config().opt<ConfigOptionStrings>("preset_names") != nullptr);
    REQUIRE(collection.default_config().opt<ConfigOptionString>("print_host") != nullptr);

    auto save_printer = [&](const std::string &name, std::initializer_list<const char*> presets, const std::string &host) {
        PhysicalPrinter printer(name, collection.default_config());
        REQUIRE(printer.config.opt<ConfigOptionStrings>("preset_names") != nullptr);
        printer.file = (dir.path() / (name + ".json")).string();
        for (const char *preset : presets)
            REQUIRE(printer.add_preset(preset));
        REQUIRE(printer.config.opt<ConfigOptionString>("print_host") != nullptr);
        printer.config.opt<ConfigOptionString>("print_host")->value = host;
        printer.update_preset_names_in_config();
        printer.save(nullptr);
    };

    save_printer("Only", {"Removed"}, "only.local");
    save_printer("Shared", {"Removed", "Keep"}, "shared.local");
    save_printer("Unrelated", {"Keep"}, "keep.local");

    PresetsConfigSubstitutions substitutions;
    collection.load_printers(dir.path().string(), "", substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    REQUIRE(std::distance(collection.begin(), collection.end()) == 3);

    const fs::path only_file = dir.path() / "Only.json";
    const fs::path shared_file = dir.path() / "Shared.json";
    REQUIRE(fs::exists(only_file));
    REQUIRE(fs::exists(shared_file));

    collection.select_printer("Shared", "Keep");
    REQUIRE(collection.has_selection());
    CHECK(collection.get_selected_full_printer_name() == "Shared * Keep");

    REQUIRE(collection.delete_preset_from_printers("Removed"));
    CHECK_FALSE(collection.has_selection());
    CHECK(collection.find_printer("Only") == nullptr);
    CHECK_FALSE(fs::exists(only_file));
    CHECK(fs::exists(shared_file));

    const PhysicalPrinter *shared = collection.find_printer("Shared");
    REQUIRE(shared != nullptr);
    REQUIRE(shared->config.opt<ConfigOptionStrings>("preset_names") != nullptr);
    REQUIRE(shared->config.opt<ConfigOptionString>("preset_name") != nullptr);
    REQUIRE(shared->config.opt<ConfigOptionString>("print_host") != nullptr);
    CHECK(shared->preset_names == std::set<std::string>{"Keep"});
    CHECK(shared->config.opt<ConfigOptionStrings>("preset_names")->values == std::vector<std::string>{"Keep"});
    CHECK(shared->config.opt_string("preset_name") == "Keep");
    CHECK(shared->config.opt_string("print_host") == "shared.local");

    const PhysicalPrinter *unrelated = collection.find_printer("Unrelated");
    REQUIRE(unrelated != nullptr);
    REQUIRE(unrelated->config.opt<ConfigOptionStrings>("preset_names") != nullptr);
    REQUIRE(unrelated->config.opt<ConfigOptionString>("preset_name") != nullptr);
    REQUIRE(unrelated->config.opt<ConfigOptionString>("print_host") != nullptr);
    CHECK(unrelated->preset_names == std::set<std::string>{"Keep"});
    CHECK(unrelated->config.opt_string("print_host") == "keep.local");

    PhysicalPrinterCollection reloaded(PhysicalPrinter::printer_options());
    reloaded.load_printers(dir.path().string(), "", substitutions, ForwardCompatibilitySubstitutionRule::Disable);
    REQUIRE(std::distance(reloaded.begin(), reloaded.end()) == 2);
    const PhysicalPrinter *persisted = reloaded.find_printer("Shared");
    REQUIRE(persisted != nullptr);
    REQUIRE(persisted->config.opt<ConfigOptionStrings>("preset_names") != nullptr);
    REQUIRE(persisted->config.opt<ConfigOptionString>("preset_name") != nullptr);
    REQUIRE(persisted->config.opt<ConfigOptionString>("print_host") != nullptr);
    CHECK(persisted->preset_names == std::set<std::string>{"Keep"});
    CHECK(persisted->config.opt_string("print_host") == "shared.local");
}

} // namespace preset_workflow_contracts

namespace preset_extended_contracts {
using Catch::Matchers::WithinAbs;
namespace fs = boost::filesystem;

namespace {

PresetCollection make_print_collection()
{
    return PresetCollection(Preset::TYPE_PRINT, Preset::print_options(),
                            static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
}

Preset &stored_preset(PresetCollection &collection, const std::string &name)
{
    Preset *preset = collection.find_preset(name, false, true);
    REQUIRE(preset != nullptr);
    return *preset;
}

Preset &load_local_preset(PresetCollection &collection, const std::string &name, const std::string &inherits = {})
{
    DynamicPrintConfig config = collection.default_preset().config;
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;
    return collection.load_preset({}, name, std::move(config), false);
}

void set_string(DynamicPrintConfig &config, const std::string &key, const std::string &value)
{
    config.option<ConfigOptionString>(key, true)->value = value;
}

void set_strings(DynamicPrintConfig &config, const std::string &key, const std::vector<std::string> &values)
{
    config.option<ConfigOptionStrings>(key, true)->values = values;
}

void set_floats(DynamicPrintConfig &config, const std::string &key, const std::vector<double> &values)
{
    config.option<ConfigOptionFloats>(key, true)->values = values;
}

void set_variant(DynamicPrintConfig &config, const std::string &model, const std::string &variant)
{
    set_string(config, "printer_model", model);
    set_string(config, "printer_variant", variant);
}

VendorProfile::PrinterModel vendor_model(const std::string &name, const std::string &id,
                                         const std::string &family = {})
{
    VendorProfile::PrinterModel model;
    model.name = name;
    model.model_id = id;
    model.family = family;
    model.variants.emplace_back("0.4");
    model.variants.emplace_back("0.6");
    return model;
}

} // namespace

TEST_CASE("Preset visibility follows installed printer variants and renamed filament entries", "[Preset][PresetExtendedVariance]")
{
    PresetBundle bundle;
    VendorProfile vendor("Acme");
    vendor.name = "Acme";
    vendor.models.push_back(vendor_model("Acme One", "ACME-1"));
    bundle.vendors.emplace(vendor.id, vendor);

    Preset printer(Preset::TYPE_PRINTER, "Acme One");
    printer.vendor = &bundle.vendors.at("Acme");
    printer.config = DynamicPrintConfig::full_print_config();
    set_variant(printer.config, "Acme One", "0.4");
    AppConfig app;
    app.set_variant("Acme", "Acme One", "0.4", true);
    printer.set_visible_from_appconfig(app);
    CHECK(printer.is_visible);
    app.set_variant("Acme", "Acme One", "0.4", false);
    printer.set_visible_from_appconfig(app);
    CHECK_FALSE(printer.is_visible);

    Preset filament(Preset::TYPE_FILAMENT, "New PLA");
    filament.vendor = &bundle.vendors.at("Acme");
    filament.config = DynamicPrintConfig::full_print_config();
    filament.renamed_from = {"Old PLA"};
    app.set_section(AppConfig::SECTION_FILAMENTS, {{"Old PLA", "1"}});
    filament.set_visible_from_appconfig(app);
    CHECK(filament.is_visible);
    app.set_section(AppConfig::SECTION_FILAMENTS, {{"Old PLA", ""}});
    filament.set_visible_from_appconfig(app);
    CHECK_FALSE(filament.is_visible);
}

TEST_CASE("Preset printer model metadata drives current type lidar calibration and bed fallback", "[Preset][PresetExtendedVariance]")
{
    PresetBundle bundle;
    VendorProfile bbl("BBL");
    bbl.name = "BBL";
    bbl.models.push_back(vendor_model("X1", "BL-P001"));
    bundle.vendors.emplace(bbl.id, bbl);
    set_variant(bundle.printers.get_edited_preset().config, "X1", "0.4");

    Preset process(Preset::TYPE_PRINT, "Process");
    process.config = DynamicPrintConfig::full_print_config();
    set_variant(process.config, "X1", "0.4");
    CHECK(process.get_printer_type(&bundle) == "BL-P001");
    CHECK(process.get_current_printer_type(&bundle) == "BL-P001");
    set_string(process.config, "printer_model", "Other");
    CHECK(process.get_current_printer_type(&bundle).empty());
    set_string(process.config, "printer_model", "X1");
    CHECK(process.has_lidar(&bundle));
    CHECK(process.has_cali_lines(&bundle));
    set_string(process.config, "default_bed_type", "3");
    CHECK(process.get_default_bed_type(&bundle) == btPEI);
    set_string(process.config, "default_bed_type", "0");
    CHECK(process.get_default_bed_type(&bundle) == btPEI);
    set_string(process.config, "default_bed_type", "bad-bed");
    CHECK(process.get_default_bed_type(&bundle) == btPEI);
    set_string(process.config, "default_bed_type", "");
    CHECK(process.get_default_bed_type(&bundle) == btPC);

    VendorProfile other("Acme");
    other.name = "Acme";
    other.models.push_back(vendor_model("Other", "OTHER"));
    bundle.vendors.clear();
    bundle.vendors.emplace(other.id, other);
    CHECK_FALSE(process.has_lidar(&bundle));
    CHECK_FALSE(process.has_cali_lines(&bundle));
}

TEST_CASE("Preset extruder keyset selection is explicit for supported and unknown types", "[Preset][PresetExtendedVariance]")
{
    for (const auto type : {Preset::TYPE_PRINT, Preset::TYPE_PRINTER, Preset::TYPE_FILAMENT, Preset::TYPE_INVALID}) {
        std::string id, variant;
        std::set<std::string> *one = nullptr, *two = nullptr;
        Preset::get_extruder_names_and_keysets(type, id, variant, &one, &two);
        REQUIRE(one != nullptr);
        REQUIRE(two != nullptr);
        if (type == Preset::TYPE_PRINT) {
            CHECK(id == "print_extruder_id");
            CHECK(variant == "print_extruder_variant");
            CHECK(two->empty());
        } else if (type == Preset::TYPE_PRINTER) {
            CHECK(id == "printer_extruder_id");
            CHECK(variant == "printer_extruder_variant");
            CHECK_FALSE(one->empty());
            CHECK_FALSE(two->empty());
        } else if (type == Preset::TYPE_FILAMENT) {
            CHECK(id.empty());
            CHECK(variant == "filament_extruder_variant");
            CHECK(two->empty());
        } else {
            CHECK(id.empty());
            CHECK(variant.empty());
            CHECK(one->empty());
            CHECK(two->empty());
        }
    }
}

TEST_CASE("Preset differed save emits only child changes and rejects built-in presets", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    Preset &parent = load_local_preset(collection, "Parent");
    parent.setting_id = "parent-id";
    Preset &child = load_local_preset(collection, "Child", "Parent");
    child.config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;

    std::unique_ptr<Preset> diff(collection.get_preset_differed_for_save(child));
    REQUIRE(diff != nullptr);
    CHECK(diff->name == "Child");
    CHECK_THAT(diff->config.opt_float("layer_height"), WithinAbs(0.31, 1e-6));
    CHECK_FALSE(diff->config.has("wall_loops"));
    CHECK(collection.get_preset_differed_for_save(collection.default_preset()) == nullptr);
    std::unique_ptr<Preset> root_diff(collection.get_preset_differed_for_save(stored_preset(collection, "Parent")));
    CHECK(root_diff == nullptr);
}

TEST_CASE("Preset differed value export carries cloud metadata and changed density", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    Preset &parent = load_local_preset(collection, "Base");
    parent.setting_id = "base-id";
    Preset &child = load_local_preset(collection, "Child", "Base");
    child.type = Preset::TYPE_FILAMENT;
    child.base_id = parent.setting_id;
    set_floats(parent.config, "filament_density", {1.75});
    child.config = parent.config;
    child.inherits() = "Base";
    child.setting_id = "cloud-id";
    child.version = *Semver::parse("1.2.3");
    child.updated_time = 77;
    set_floats(child.config, "filament_density", {1.24});
    std::map<std::string, std::string> values;
    CHECK(collection.get_differed_values_to_update(child, values) == 0);
    CHECK(values.count("filament_density") == 1);
    CHECK(values.at(BBL_JSON_KEY_VERSION) == "1.2.0.3");
    CHECK(values.at(ORCA_JSON_KEY_UPDATE_TIME) == "77");
    CHECK(values.at(BBL_JSON_KEY_TYPE) == PRESET_IOT_FILAMENT_TYPE);
    CHECK(values.at(BBL_JSON_KEY_BASE_ID) == "base-id");
    CHECK(collection.get_differed_values_to_update(collection.default_preset(), values) == -1);
}

TEST_CASE("Project embedded presets resolve their parent and reject duplicate or unrelated entries", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    Preset &parent = load_local_preset(collection, "Parent");
    parent.config.opt<ConfigOptionFloat>("layer_height")->value = 0.2;
    auto embedded = std::make_unique<Preset>(Preset::TYPE_PRINT, "Embedded");
    embedded->is_project_embedded = true;
    embedded->config = parent.config;
    embedded->inherits() = "Parent";
    embedded->config.opt<ConfigOptionFloat>("layer_height")->value = 0.33;
    auto duplicate = std::make_unique<Preset>(*embedded);
    duplicate->name = "Parent";
    auto other = std::make_unique<Preset>(Preset::TYPE_FILAMENT, "Other");
    other->is_project_embedded = true;
    other->config = DynamicPrintConfig::full_print_config();
    std::vector<Preset*> project{embedded.get(), duplicate.get(), other.get()};
    PresetsConfigSubstitutions substitutions;
    collection.load_project_embedded_presets(project, PRESET_PRINT_NAME, substitutions,
                                              ForwardCompatibilitySubstitutionRule::Disable);
    REQUIRE(collection.find_preset("Embedded") != nullptr);
    CHECK_THAT(collection.find_preset("Embedded")->config.opt_float("layer_height"), WithinAbs(0.33, 1e-6));
    CHECK(collection.size() == collection.num_default_presets() + 2);
}

TEST_CASE("Cloud sync metadata updates a base and inherited child and reports freshness", "[Preset][PresetExtendedVariance]")
{
    ScopedTemporaryDir dir("preset-extended-sync");
    PresetCollection collection = make_print_collection();
    collection.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);
    Preset &base = load_local_preset(collection, "Base");
    base.file = collection.path_for_preset(base);
    base.setting_id = "old-base";
    REQUIRE(base.save(nullptr));
    Preset &child = load_local_preset(collection, "Child", "Base");
    child.file = collection.path_for_preset(child);
    child.setting_id = "child-id";
    REQUIRE(child.save(&base.config));

    CHECK(collection.need_sync("Child", "child-id", 1));
    collection.set_sync_info_and_save("Base", "new-base", "update", 12, "user-7");
    CHECK(base.setting_id == "new-base");
    CHECK(base.user_id == "user-7");
    CHECK(base.updated_time == 12);
    CHECK(child.base_id == "new-base");
    CHECK_FALSE(collection.need_sync("Child", "child-id", 0));
    CHECK_FALSE(collection.need_sync("Base", "new-base", 12));
}

TEST_CASE("User preset export filters cloud states and saves root and inherited profiles", "[Preset][PresetExtendedVariance]")
{
    ScopedTemporaryDir dir("preset-extended-user-export");
    PresetCollection collection = make_print_collection();
    collection.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);
    Preset &base = load_local_preset(collection, "Base");
    base.setting_id = "base-id";
    base.file = collection.path_for_preset(base);
    REQUIRE(base.save(nullptr));
    load_local_preset(collection, "Root").sync_info = "save";
    load_local_preset(collection, "Child", "Base");
    stored_preset(collection, "Child").sync_info = "save";
    stored_preset(collection, "Child").base_id = "base-id";
    stored_preset(collection, "Child").config.opt<ConfigOptionFloat>("layer_height")->value = 0.29;
    Preset &hold = load_local_preset(collection, "Hold");
    hold.setting_id = "hold-id";
    hold.sync_info = "hold";

    std::vector<Preset> users;
    CHECK(collection.get_user_presets(nullptr, users) == 2);
    CHECK(std::count_if(users.begin(), users.end(), [](const Preset &p) { return p.name == "Root"; }) == 1);
    std::map<std::string, std::string> deletes;
    collection.save_user_presets(dir.path().string(), PRESET_PRINT_NAME, deletes);
    const Preset &root = stored_preset(collection, "Root");
    const Preset &child = stored_preset(collection, "Child");
    CHECK(fs::exists(root.file));
    CHECK(fs::exists(child.file));
    CHECK(child.base_id == "base-id");
    CHECK(root.sync_info.empty());
    CHECK(child.sync_info.empty());
}

TEST_CASE("Cloud user loading rejects incomplete metadata and installs a canonical root", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    PresetsConfigSubstitutions substitutions;
    CHECK_FALSE(collection.load_user_preset("MissingVersion", {{BBL_JSON_KEY_SETTING_ID, "id"}, {BBL_JSON_KEY_USER_ID, "u"}}, substitutions,
                                            ForwardCompatibilitySubstitutionRule::Disable));
    CHECK_FALSE(collection.load_user_preset("BadVersion", {{BBL_JSON_KEY_VERSION, "bad"}, {BBL_JSON_KEY_SETTING_ID, "id"}, {BBL_JSON_KEY_USER_ID, "u"}}, substitutions,
                                            ForwardCompatibilitySubstitutionRule::Disable));
    CHECK_FALSE(collection.load_user_preset("MissingId", {{BBL_JSON_KEY_VERSION, "1.0.0"}, {BBL_JSON_KEY_USER_ID, "u"}}, substitutions,
                                            ForwardCompatibilitySubstitutionRule::Disable));
    CHECK_FALSE(collection.load_user_preset("MissingUser", {{BBL_JSON_KEY_VERSION, "1.0.0"}, {BBL_JSON_KEY_SETTING_ID, "id"}}, substitutions,
                                            ForwardCompatibilitySubstitutionRule::Disable));
    std::map<std::string, std::string> values{{BBL_JSON_KEY_VERSION, "1.0.0"}, {BBL_JSON_KEY_SETTING_ID, "id"},
                                               {BBL_JSON_KEY_USER_ID, "u"}, {ORCA_JSON_KEY_UPDATE_TIME, "4"},
                                               {"layer_height", "0.27"}};
    CHECK(collection.load_user_preset("CloudRoot", values, substitutions, ForwardCompatibilitySubstitutionRule::Disable));
    const Preset *root = collection.find_preset("CloudRoot");
    REQUIRE(root != nullptr);
    CHECK(root->setting_id == "id");
    CHECK(root->sync_info == "save");
    CHECK_THAT(root->config.opt_float("layer_height"), WithinAbs(0.27, 1e-6));
}

TEST_CASE("Cloud user loading updates newer profiles while preserving dirty local options", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    Preset &local = load_local_preset(collection, "Cloud");
    local.setting_id = "old";
    local.updated_time = 2;
    local.config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;
    collection.select_preset_by_name("Cloud", true);
    collection.get_edited_preset().config.opt<ConfigOptionInt>("wall_loops")->value = 7;
    local.is_dirty = true;
    collection.get_edited_preset().set_dirty(true);
    std::map<std::string, std::string> values{{BBL_JSON_KEY_VERSION, "1.0.0"}, {BBL_JSON_KEY_SETTING_ID, "new"},
                                               {BBL_JSON_KEY_USER_ID, "u"}, {ORCA_JSON_KEY_UPDATE_TIME, "3"},
                                               {"layer_height", "0.22"}, {"wall_loops", "3"}};
    PresetsConfigSubstitutions substitutions;
    CHECK_FALSE(collection.load_user_preset("Cloud", values, substitutions, ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(local.setting_id == "new");
    CHECK(local.updated_time == 3);
    CHECK_THAT(local.config.opt_float("layer_height"), WithinAbs(0.22, 1e-6));
    CHECK(collection.get_edited_preset().config.opt_int("wall_loops") == 7);
}

TEST_CASE("Preset validation rejects unknown inheritance and trusts system profiles", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    Preset &system = load_local_preset(collection, "System");
    system.is_system = true;
    Preset &child = load_local_preset(collection, "Child", "System");
    std::string inherit = "System";
    CHECK(collection.validate_preset("System", inherit));
    CHECK(collection.validate_preset("Child", inherit));
    inherit = "Missing";
    CHECK_FALSE(collection.validate_preset("Child", inherit));
    inherit = "System";
    CHECK(collection.validate_preset("Missing", inherit));
    inherit.clear();
    CHECK_FALSE(collection.validate_preset("Missing", inherit));
    (void)child;
}

TEST_CASE("Filament grouping and type lookup prefer visible compatible system bases", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    Preset &pla = load_local_preset(collection, "PLA Base");
    pla.type = Preset::TYPE_FILAMENT;
    pla.is_system = true;
    pla.filament_id = "pla-id";
    set_strings(pla.config, "filament_type", {"PLA"});
    Preset &high = load_local_preset(collection, "PETG Base");
    high.type = Preset::TYPE_FILAMENT;
    high.is_system = true;
    high.filament_id = "petg-id";
    set_strings(high.config, "filament_type", {"PETG"});
    const size_t pla_idx = collection.first_visible_idx_by_type("PLA High Speed");
    CHECK(collection.preset(pla_idx, true).name == "PLA Base");
    CHECK(collection.filament_id_by_type("PLA High Speed") == "pla-id");
    std::map<std::string, std::vector<Preset const *>> grouped = collection.get_filament_presets();
    CHECK(grouped.count("pla-id") == 1);
    CHECK(grouped.at("pla-id").size() == 1);
    (void)high;
}

TEST_CASE("Printer diameter lookup and forced selection use actual edited printer state", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection(Preset::TYPE_PRINTER, Preset::printer_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    Preset &a = load_local_preset(collection, "A");
    a.type = Preset::TYPE_PRINTER;
    set_variant(a.config, "Model", "0.4");
    Preset &b = load_local_preset(collection, "B");
    b.type = Preset::TYPE_PRINTER;
    set_variant(b.config, "Model", "0.6");
    Preset &other = load_local_preset(collection, "Other");
    other.type = Preset::TYPE_PRINTER;
    set_variant(other.config, "Other", "1.0");
    collection.select_preset_by_name("A", true);
    CHECK(collection.diameters_of_selected_printer() == std::vector<std::string>{"0.4", "0.6"});
    CHECK(collection.select_preset_by_name("B", false));
    CHECK_FALSE(collection.select_preset_by_name("B", false));
    CHECK(collection.select_preset_by_name("B", true));
}

TEST_CASE("Preset parent vendor lookup follows inheritance and base identity", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    VendorProfile vendor("Acme");
    vendor.name = "Acme";
    collection.load_preset({}, "Parent", collection.default_preset().config, false);
    Preset *parent_before = collection.find_preset("Parent", false, true);
    REQUIRE(parent_before != nullptr);
    parent_before->vendor = &vendor;
    Preset &child = load_local_preset(collection, "Child", "Parent");
    Preset *parent = collection.find_preset("Parent", false, true);
    REQUIRE(parent != nullptr);
    CHECK(collection.get_preset_parent(child) == parent);
    CHECK(collection.get_preset_base(child) == parent);
    CHECK(collection.get_preset_with_vendor_profile(child).vendor == &vendor);
    CHECK(collection.get_preset_with_vendor_profile(collection.default_preset()).vendor == nullptr);
}

TEST_CASE("Printer hold aliases are added and removed without changing preset identity", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    Preset &filament = load_local_preset(collection, "PLA");
    filament.type = Preset::TYPE_FILAMENT;
    filament.alias.clear();
    filament.config.opt<ConfigOptionStrings>("compatible_printers", true)->values = {"Printer A"};
    CHECK(collection.get_preset_alias(filament) == "PLA");
    collection.set_printer_hold_alias("PLA", filament);
    CHECK(collection.is_alias_exist("PLA", &filament));
    CHECK(filament.name == "PLA");
    collection.set_printer_hold_alias("PLA", filament, true);
    CHECK_FALSE(collection.is_alias_exist("PLA", &filament));
}

TEST_CASE("Compatibility conditions accept valid predicates and fail open for malformed predicates", "[Preset][PresetExtendedVariance]")
{
    Preset filament(Preset::TYPE_FILAMENT, "PLA");
    filament.config = DynamicPrintConfig::full_print_config();
    set_strings(filament.config, "compatible_printers", {});
    set_string(filament.config, "compatible_printers_condition", R"(printer_model == "X1")");
    Preset printer(Preset::TYPE_PRINTER, "X1");
    printer.config = DynamicPrintConfig::full_print_config();
    set_string(printer.config, "printer_model", "X1");
    CHECK(is_compatible_with_printer(PresetWithVendorProfile(filament, nullptr), PresetWithVendorProfile(printer, nullptr)));
    set_string(printer.config, "printer_model", "Other");
    CHECK_FALSE(is_compatible_with_printer(PresetWithVendorProfile(filament, nullptr), PresetWithVendorProfile(printer, nullptr)));
    set_string(printer.config, "printer_model", "X1");
    set_string(filament.config, "compatible_printers_condition", "(");
    CHECK(is_compatible_with_printer(PresetWithVendorProfile(filament, nullptr), PresetWithVendorProfile(printer, nullptr)));
    CHECK(is_compatible_with_printer(filament.config, Preset::TYPE_FILAMENT, printer.config, "X1"));
}

TEST_CASE("Physical printer config updates preserve typed fields and membership serialization", "[Preset][PresetExtendedVariance]")
{
    PhysicalPrinter printer("Office", PhysicalPrinterCollection(PhysicalPrinter::printer_options()).default_config());
    printer.add_preset("0.4");
    DynamicPrintConfig update = printer.config;
    update.opt<ConfigOptionString>("print_host")->value = "new-host";
    update.opt<ConfigOptionStrings>("preset_names")->values = {"0.6"};
    printer.update_from_config(update);
    CHECK(printer.config.opt_string("print_host") == "new-host");
    CHECK(printer.preset_names == std::set<std::string>{"0.4", "0.6"});
    printer.update_preset_names_in_config();
    CHECK(printer.config.opt_string("preset_name") == "0.4;0.6");
}

TEST_CASE("Preset paths canonicalize names under the configured user directory", "[Preset][PresetExtendedVariance]")
{
    ScopedTemporaryDir dir("preset-extended-paths");
    PresetCollection collection = make_print_collection();
    collection.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);
    CHECK(collection.path_from_name("Fine") == (dir.path() / PRESET_PRINT_NAME / "Fine.json").string());
    CHECK(collection.path_from_name("Fine.json") == (dir.path() / PRESET_PRINT_NAME / "Fine.json").string());
    CHECK(collection.path_from_name("Fine", true).find("Fine.json") != std::string::npos);
    Preset &preset = load_local_preset(collection, "Fine");
    CHECK(collection.path_for_preset(preset) == (dir.path() / PRESET_PRINT_NAME / "base" / "Fine.json").string());
}

TEST_CASE("Project embedded export returns independent diffs and reset only removes embedded entries", "[Preset][PresetExtendedVariance]")
{
    PresetCollection collection = make_print_collection();
    Preset &parent = load_local_preset(collection, "Parent");
    Preset &embedded = load_local_preset(collection, "Embedded", "Parent");
    embedded.is_project_embedded = true;
    embedded.config.opt<ConfigOptionFloat>("layer_height")->value = 0.28;
    std::vector<Preset*> exported = collection.get_project_embedded_presets();
    ScopeGuard cleanup([&exported] { for (Preset *preset : exported) delete preset; });
    REQUIRE(exported.size() == 1);
    CHECK(exported.front() != &embedded);
    CHECK_THAT(exported.front()->config.opt_float("layer_height"), WithinAbs(0.28, 1e-6));
    collection.select_preset_by_name("Embedded", true);
    CHECK(collection.reset_project_embedded_presets());
    CHECK(collection.find_preset("Embedded") == nullptr);
    const Preset *retained_parent = collection.find_preset("Parent");
    REQUIRE(retained_parent != nullptr);
    CHECK_THAT(retained_parent->config.opt_float("layer_height"), WithinAbs(0.2, 1e-6));
}

TEST_CASE("Preset JSON value extraction reports missing and malformed files without partial success", "[Preset][PresetExtendedVariance]")
{
    ScopedTemporaryDir dir("preset-extended-json");
    std::vector<std::string> keys{"name", "version"};
    std::map<std::string, std::string> values;
    CHECK(get_values_from_json((dir.path() / "missing.json").string(), keys, values) == 0);
    std::ofstream((dir.path() / "bad.json").string()) << "not-json";
    CHECK(get_values_from_json((dir.path() / "bad.json").string(), keys, values) == 0);
    std::ofstream((dir.path() / "good.json").string()) << R"({"name":"One","version":"1.0.0"})";
    CHECK(get_values_from_json((dir.path() / "good.json").string(), keys, values) == 2);
    CHECK(values.at("name") == "One");
}

TEST_CASE("Cloud preset timestamp reconciliation preserves local state for stale or equal updates", "[Preset][PresetExtendedVariance][CloudTimestamp]")
{
    const long long cloud_time = GENERATE(4LL, 5LL, 0LL);
    ScopedTemporaryDir dir("preset-cloud-timestamp");
    PresetCollection collection = make_print_collection();
    collection.update_user_presets_directory(dir.path().string(), PRESET_PRINT_NAME);

    Preset &local = load_local_preset(collection, "Cloud");
    local.file = collection.path_for_preset(local);
    local.setting_id = "old-setting";
    local.updated_time = 5;
    local.sync_info = "hold";
    local.config.opt<ConfigOptionFloat>("layer_height")->value = 0.21;
    REQUIRE(local.save(nullptr));
    const fs::path info_file = fs::path(local.file).replace_extension(".info");

    std::map<std::string, std::string> values{
        {BBL_JSON_KEY_TYPE, PRESET_IOT_PRINT_TYPE},
        {BBL_JSON_KEY_VERSION, "1.0.0"},
        {BBL_JSON_KEY_SETTING_ID, "cloud-setting"},
        {BBL_JSON_KEY_USER_ID, "cloud-user"},
        {ORCA_JSON_KEY_UPDATE_TIME, std::to_string(cloud_time)},
        {"layer_height", "0.37"},
    };
    PresetsConfigSubstitutions substitutions;

    CAPTURE(cloud_time);
    CHECK_FALSE(collection.load_user_preset("Cloud", values, substitutions,
                                             ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(fs::exists(info_file));

    Preset *reacquired = collection.find_preset("Cloud", false, true);
    REQUIRE(reacquired != nullptr);
    CHECK(reacquired->setting_id == "cloud-setting");
    CHECK(reacquired->updated_time == 5);
    CHECK(reacquired->sync_info == (cloud_time == 5 ? "" : "update"));
    CHECK_THAT(reacquired->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.21, 1e-9));

    Preset persisted(Preset::TYPE_PRINT, "Persisted");
    persisted.setting_id = "sentinel-setting";
    persisted.updated_time = -1;
    persisted.sync_info = "sentinel-sync";
    persisted.load_info(info_file.string());
    CHECK(persisted.setting_id == "cloud-setting");
    CHECK(persisted.updated_time == 5);
    CHECK(persisted.sync_info == (cloud_time == 5 ? "" : "update"));
}

} // namespace preset_extended_contracts

namespace preset_bundle_ams_contracts {
namespace {

DynamicPrintConfig tray(const std::string &filament_id, const std::string &color,
                        const std::string &type, const std::string &ams_id,
                        const std::string &slot_id, bool changed = true,
                        bool placeholder = false,
                        std::vector<std::string> multi = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_id", new ConfigOptionStrings({filament_id}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({color}));
    config.set_key_value("filament_colour_type", new ConfigOptionStrings({"1"}));
    config.set_key_value("filament_type", new ConfigOptionStrings({type}));
    config.set_key_value("filament_multi_colour", new ConfigOptionStrings(std::move(multi)));
    config.set_key_value("ams_id", new ConfigOptionStrings({ams_id}));
    config.set_key_value("slot_id", new ConfigOptionStrings({slot_id}));
    config.set_key_value("filament_changed", new ConfigOptionBool(changed));
    config.set_key_value("filament_slot_placeholder", new ConfigOptionBools({placeholder}));
    return config;
}

Preset &system_filament(PresetBundle &bundle, const std::string &name,
                        const std::string &id, const std::string &type)
{
    Preset &preset = bundle.filaments.load_preset({}, name,
                                                  bundle.filaments.default_preset().config,
                                                  false);
    preset.is_system = true;
    preset.is_visible = true;
    preset.is_compatible = true;
    preset.filament_id = id;
    preset.config.option<ConfigOptionStrings>("filament_type", true)->values = { type };
    return preset;
}

void project_slots(PresetBundle &bundle, std::vector<std::string> names,
                   std::vector<std::string> colors)
{
    REQUIRE(names.size() == colors.size());
    bundle.project_config.apply(FullPrintConfig::defaults());
    bundle.prints.get_edited_preset().config.apply(FullPrintConfig::defaults());
    bundle.filament_presets = std::move(names);
    bundle.project_config.option<ConfigOptionStrings>("filament_colour", true)->values = colors;
    bundle.project_config.option<ConfigOptionStrings>("filament_colour_type", true)->values.assign(colors.size(), "1");
    bundle.project_config.option<ConfigOptionInts>("filament_map", true)->values.assign(colors.size(), 1);
    bundle.project_config.option<ConfigOptionInts>("filament_volume_map", true)->values.assign(colors.size(), 0);
    bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour", true)->values = colors;
}

void add_tray(PresetBundle &bundle, int index, DynamicPrintConfig config)
{
    bundle.filament_ams_list.emplace(index, std::move(config));
}

} // namespace

TEST_CASE("AMS sync replaces physical slots with matched IDs and parallel metadata",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    system_filament(bundle, "Generic PETG", "PETG-ID", "PETG");
    project_slots(bundle, {"Old A", "Old B"}, {"#000000", "#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#112233", "PLA", "0", "0", true, false, {"#112233"}));
    add_tray(bundle, 1, tray("PETG-ID", "#445566", "PETG", "0", "1", true, false, {"#445566"}));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA", "Generic PETG"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#112233", "#445566"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour_type")->values ==
          std::vector<std::string>{"1", "1"});
    CHECK(unknowns.empty());
}

TEST_CASE("AMS sync retains the previous preset and reports an unknown material",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"Existing"}, {"#AAAAAA"});
    add_tray(bundle, 0, tray("MISSING-ID", "#123456", "Nylon", "0", "2"));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    REQUIRE(bundle.filament_presets.size() == 1);
    CHECK(bundle.filament_presets.front() == "Existing");
    REQUIRE(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.size() == 1);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values.front() == "#123456");
    REQUIRE(unknowns.size() == 1);
    CHECK(unknowns.front().first == &bundle.filament_ams_list.begin()->second);
    CHECK_FALSE(unknowns.front().second.empty());
}

TEST_CASE("AMS unchanged slots preserve their selected preset while refreshing color",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"Existing"}, {"#AAAAAA"});
    add_tray(bundle, 0, tray("PLA-ID", "#ABCDEF", "PLA", "0", "0", false));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Existing"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#ABCDEF"});
}

TEST_CASE("AMS placeholders preserve empty trays while loaded trays replace them",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"Existing A", "Existing B"}, {"#AAAAAA", "#BBBBBB"});
    add_tray(bundle, 0, tray("", "", "", "0", "0", true, true));
    add_tray(bundle, 1, tray("PLA-ID", "#123456", "PLA", "0", "1"));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Existing A", "Generic PLA"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#AAAAAA", "#123456"});
}

TEST_CASE("AMS color-only index synchronization preserves names and mapped multi-colors",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    system_filament(bundle, "Generic PETG", "PETG-ID", "PETG");
    project_slots(bundle, {"PLA selected", "PETG selected"}, {"#000000", "#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#101010", "PLA", "A", "0", true, false, {"#101010", "#202020"}));
    add_tray(bundle, 1, tray("PETG-ID", "#303030", "PETG", "A", "1", true, false, {"#303030"}));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}, {1, {"A", "1"}}};
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, true, maps, false, merge, true);
    CHECK(bundle.filament_presets == std::vector<std::string>{"PLA selected", "PETG selected"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#101010", "#303030"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values ==
          std::vector<std::string>{"#101010 #202020", "#303030"});
    CHECK(merge.is_empty());
}

TEST_CASE("AMS color-only synchronization falls back to tray order without a map",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"A", "B"}, {"#000000", "#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#111111", "PLA", "A", "0"));
    add_tray(bundle, 1, tray("PLA-ID", "#222222", "PLA", "A", "1"));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge, true);
    CHECK(bundle.filament_presets == std::vector<std::string>{"A", "B"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#111111", "#222222"});
}

TEST_CASE("AMS mapped synchronization appends unassigned trays and records duplicate mappings",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    system_filament(bundle, "Generic PETG", "PETG-ID", "PETG");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#111111", "PLA", "A", "0"));
    add_tray(bundle, 1, tray("PETG-ID", "#222222", "PETG", "A", "0"));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}, {1, {"A", "0"}}};
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, true, maps, true, merge);
    REQUIRE(bundle.filament_presets.size() == 2);
    CHECK(bundle.filament_presets.front() == "Generic PLA");
    CHECK(bundle.filament_presets.back() == "Generic PETG");
    REQUIRE(merge.merges.size() == 1);
    CHECK(merge.merges.front() == std::vector<int>{0, 1});
}

TEST_CASE("AMS mapped synchronization does not append when append is disabled",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    system_filament(bundle, "Generic PETG", "PETG-ID", "PETG");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#111111", "PLA", "A", "0"));
    add_tray(bundle, 1, tray("PETG-ID", "#222222", "PETG", "A", "1"));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}};
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, true, maps, false, merge);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#111111"});
}

TEST_CASE("AMS unknown tray mapping is removed without corrupting existing slots",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("", "#111111", "PLA", "A", "0"));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}};
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, true, maps, false, merge);
    CHECK(maps.empty());
    CHECK(bundle.filament_presets == std::vector<std::string>{"Existing"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#000000"});
}

TEST_CASE("AMS multi-color vectors are serialized back into project configuration",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("PLA-ID", "#AABBCC", "PLA", "A", "0", true, false,
                             {"#AABBCC", "#CCDDEE", "#EEFF00"}));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_multi_colour")->values ==
          std::vector<std::string>{"#AABBCC #CCDDEE #EEFF00"});
}

TEST_CASE("AMS empty input is a no-op for existing project slots",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    project_slots(bundle, {"Existing"}, {"#ABCDEF"});
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    CHECK(bundle.sync_ams_list(unknowns, false, maps, false, merge) == 0);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Existing"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#ABCDEF"});
}

TEST_CASE("AMS overwrite clamps support selections beyond the synchronized slot count",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "PLA-ID", "PLA");
    project_slots(bundle, {"A"}, {"#000000"});
    const int selected = GENERATE(0, 1, 9);
    bundle.prints.get_edited_preset().config.set_key_value("support_filament", new ConfigOptionInt(selected));
    bundle.prints.get_edited_preset().config.set_key_value("support_interface_filament", new ConfigOptionInt(selected));
    add_tray(bundle, 0, tray("PLA-ID", "#111111", "PLA", "A", "0"));
    std::vector<std::pair<DynamicPrintConfig *, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    bundle.sync_ams_list(unknowns, false, maps, false, merge);
    CHECK(bundle.prints.get_edited_preset().config.opt_int("support_filament") == (selected > 1 ? 0 : selected));
    CHECK(bundle.prints.get_edited_preset().config.opt_int("support_interface_filament") == (selected > 1 ? 0 : selected));
}

TEST_CASE("Vendor configuration overwrite replaces app selections and filament values",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    AppConfig app;
    app.set_section(AppConfig::SECTION_FILAMENTS, {{"Old PLA", "#000000"}});
    app.set_vendors({{"OldVendor", {{"Old Model", {"0.4"}}}}});
    const std::map<std::string, std::map<std::string, std::set<std::string>>> vendors{{"Custom", {}}};
    const std::map<std::string, std::string> filaments{{"New PLA", "#123456"}};
    ScopedTemporaryDir temporary("preset-bundle-vendor-overwrite");
    ScopedDataDir data(temporary.path() / "data");
    ScopedResourcesDir resources(temporary.path() / "resources");
    fs::create_directories(temporary.path() / "resources" / PRESET_PROFILES_DIR);
    fs::create_directories(temporary.path() / "data" / PRESET_SYSTEM_DIR);
    REQUIRE(bundle.apply_vendor_config(vendors, filaments, &app, true));
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS) ==
          std::map<std::string, std::string>{{"New PLA", "#123456"}});
    CHECK(app.vendors() == vendors);
}

TEST_CASE("Vendor configuration merge preserves old selections while adding new values",
          "[Preset][PresetBundleAmsVariance]")
{
    PresetBundle bundle;
    AppConfig app;
    app.set_section(AppConfig::SECTION_FILAMENTS, {{"Old PLA", "#000000"}});
    app.set_vendors({{"OldVendor", {{"Old Model", {"0.4"}}}}});
    const std::map<std::string, std::map<std::string, std::set<std::string>>> vendors{{"Custom", {}}};
    const std::map<std::string, std::string> filaments{{"New PLA", "#123456"}};
    ScopedTemporaryDir temporary("preset-bundle-vendor-merge");
    ScopedDataDir data(temporary.path() / "data");
    ScopedResourcesDir resources(temporary.path() / "resources");
    fs::create_directories(temporary.path() / "resources" / PRESET_PROFILES_DIR);
    fs::create_directories(temporary.path() / "data" / PRESET_SYSTEM_DIR);
    REQUIRE(bundle.apply_vendor_config(vendors, filaments, &app, false));
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS) ==
          std::map<std::string, std::string>{{"Old PLA", "#000000"}, {"New PLA", "#123456"}});
    CHECK(app.vendors().at("OldVendor").at("Old Model") == std::set<std::string>{"0.4"});
    CHECK(app.vendors().count("Custom") == 1);
}

TEST_CASE("Vendor configuration replaces a system filament when a vendor printer is selected",
          "[Preset][PresetBundleAmsVariance]")
{
    ScopedTemporaryDir temporary("preset-bundle-vendor");
    ScopedDataDir data(temporary.path() / "data");
    ScopedResourcesDir resources(temporary.path() / "resources");
    fs::create_directories(temporary.path() / "resources" / PRESET_PROFILES_DIR);
    fs::create_directories(temporary.path() / "data" / PRESET_SYSTEM_DIR);
    std::ofstream((temporary.path() / "data" / PRESET_SYSTEM_DIR / "Acme.json").string())
        << R"({"version":"1.0.0","name":"Acme","machine_model_list":[{"name":"Acme One","sub_path":"machine/model.json"}],"machine_list":[],"filament_list":[{"name":"Acme PLA","sub_path":"filament/pla.json"}]})";
    fs::create_directories(temporary.path() / "data" / PRESET_SYSTEM_DIR / "Acme" / "machine");
    fs::create_directories(temporary.path() / "data" / PRESET_SYSTEM_DIR / "Acme" / "filament");
    std::ofstream((temporary.path() / "data" / PRESET_SYSTEM_DIR / "Acme" / "machine" / "model.json").string())
        << R"({"type":"machine_model","name":"Acme One","nozzle_diameter":"0.4","default_materials":"Acme PLA"})";
    std::ofstream((temporary.path() / "data" / PRESET_SYSTEM_DIR / "Acme" / "filament" / "pla.json").string())
        << R"({"type":"filament","name":"Acme PLA","from":"system","instantiation":"true","filament_id":"ACME-PLA","filament_type":"PLA"})";
    AppConfig app;
    PresetBundle bundle;
    bundle.vendors["Acme"].default_filaments.insert("Generic PLA");
    const std::map<std::string, std::map<std::string, std::set<std::string>>> vendors{
        {"Acme", {{"Acme One", {"0.4"}}}}};
    const std::map<std::string, std::string> filaments{{"Generic PLA @System", "#101010"}};
    REQUIRE(bundle.apply_vendor_config(vendors, filaments, &app, true));
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS).count("Generic PLA @System") == 0);
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS).count("Generic PLA") == 1);
}


TEST_CASE("Vendor configuration retains the system filament when a default printer is also selected",
          "[Preset][PresetBundleAmsVariance]")
{
    ScopedTemporaryDir temporary("preset-bundle-vendor-default");
    ScopedDataDir data(temporary.path() / "data");
    ScopedResourcesDir resources(temporary.path() / "resources");
    fs::create_directories(temporary.path() / "resources" / PRESET_PROFILES_DIR);
    const fs::path system = temporary.path() / "data" / PRESET_SYSTEM_DIR;
    fs::create_directories(system);
    for (const std::string &vendor : {std::string("Acme"), std::string("Custom")}) {
        fs::create_directories(system / vendor / "machine");
        std::ofstream((system / (vendor + ".json")).string())
            << "{\"version\":\"1.0.0\",\"name\":\"" << vendor
            << "\",\"machine_model_list\":[{\"name\":\"" << vendor
            << " One\",\"sub_path\":\"machine/model.json\"}]}";
        std::ofstream((system / vendor / "machine" / "model.json").string())
            << "{\"type\":\"machine_model\",\"name\":\"" << vendor
            << " One\",\"nozzle_diameter\":\"0.4\"}";
    }

    AppConfig app;
    PresetBundle bundle;
    bundle.vendors["Acme"].default_filaments.insert("Generic PLA");
    const std::map<std::string, std::map<std::string, std::set<std::string>>> vendors{
        {"Acme", {{"Acme One", {"0.4"}}}},
        {"Custom", {{"Custom One", {"0.4"}}}}};
    const std::map<std::string, std::string> filaments{{"Generic PLA @System", "#101010"}};
    REQUIRE(bundle.apply_vendor_config(vendors, filaments, &app, true));
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS).count("Generic PLA @System") == 1);
    CHECK(app.get_section(AppConfig::SECTION_FILAMENTS).count("Generic PLA") == 1);
}

} // namespace preset_bundle_ams_contracts

namespace preset_bundle_lifecycle_extended_contracts {
using namespace Slic3r;
namespace fs = boost::filesystem;
using Catch::Matchers::WithinAbs;

namespace {

using preset_extended_contracts::make_print_collection;
using preset_extended_contracts::load_local_preset;
using preset_extended_contracts::set_variant;

void write_marker(const fs::path &path, const std::string &contents)
{
    fs::create_directories(path.parent_path());
    boost::nowide::ofstream out(path.string(), std::ios::binary);
    REQUIRE(out.good());
    out << contents;
    out.close();
    REQUIRE(out.good());
}

std::string read_marker(const fs::path &path)
{
    boost::nowide::ifstream in(path.string(), std::ios::binary);
    // A missing file yields an empty string and fails the caller's nonempty
    // content expectation without nesting a fatal Catch assertion.
    return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

} // namespace

TEST_CASE("PresetBundle copy repairs vendor pointers and preserves selected configurations",
          "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle original;
    VendorProfile vendor("Acme");
    vendor.name = "Acme";
    original.vendors.emplace(vendor.id, vendor);
    Preset &printer = original.printers.load_preset({}, "Acme Printer",
                                                    original.printers.default_preset().config, false);
    printer.vendor = &original.vendors.at("Acme");
    set_variant(printer.config, "Acme One", "0.4");
    original.printers.select_preset_by_name("Acme Printer", true);
    original.filament_presets = {"Acme PLA"};
    original.project_config.option<ConfigOptionStrings>("filament_colour", true)->values = {"#112233"};

    PresetBundle copied(original);
    REQUIRE(copied.printers.find_preset("Acme Printer", false, true) != nullptr);
    CHECK(copied.printers.get_selected_preset_name() == "Acme Printer");
    CHECK(copied.printers.find_preset("Acme Printer", false, true)->vendor == &copied.vendors.at("Acme"));
    CHECK(copied.printers.find_preset("Acme Printer", false, true)->vendor != printer.vendor);
    CHECK(copied.filament_presets == std::vector<std::string>{"Acme PLA"});
    CHECK(copied.project_config.option<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#112233"});

    PresetBundle assigned;
    assigned = original;
    REQUIRE(assigned.printers.find_preset("Acme Printer", false, true) != nullptr);
    CHECK(assigned.printers.find_preset("Acme Printer", false, true)->vendor == &assigned.vendors.at("Acme"));
    original.vendors.at("Acme").name = "Changed original";
    CHECK(copied.vendors.at("Acme").name == "Acme");
    CHECK(assigned.vendors.at("Acme").name == "Acme");
}

TEST_CASE("PresetBundle setup_directories creates the owned data layout", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-layout");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.setup_directories();
    CHECK(fs::is_directory(temp.path() / "data"));
    CHECK(fs::is_directory(temp.path() / "data" / "ota"));
    CHECK(fs::is_directory(temp.path() / "data" / PRESET_SYSTEM_DIR));
    CHECK(fs::is_directory(temp.path() / "data" / PRESET_USER_DIR));
}

TEST_CASE("PresetBundle copy_files preserves only the supported preset trees", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-copy");
    ScopedDataDir data(temp.path() / "data");
    const fs::path source = temp.path() / "source";
    fs::create_directories(temp.path() / "data" / "old");
    write_marker(source / PRESET_PRINT_NAME / "base" / "process.json", "print");
    write_marker(source / PRESET_FILAMENT_NAME / "base" / "filament.json", "filament");
    write_marker(source / PRESET_PRINTER_NAME / "base" / "printer.json", "printer");
    write_marker(source / "vendor" / "ignored.json", "ignored");

    PresetBundle bundle;
    bundle.copy_files(source.string());
    CHECK(read_marker(temp.path() / "data" / "old" / PRESET_PRINT_NAME / "base" / "process.json") == "print");
    CHECK(read_marker(temp.path() / "data" / "old" / PRESET_FILAMENT_NAME / "base" / "filament.json") == "filament");
    CHECK(read_marker(temp.path() / "data" / "old" / PRESET_PRINTER_NAME / "base" / "printer.json") == "printer");
    CHECK_FALSE(fs::exists(temp.path() / "data" / "old" / "vendor"));
}

TEST_CASE("PresetBundle backup_user_folder is successful once and idempotent", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-backup");
    ScopedDataDir data(temp.path() / "data");
    write_marker(temp.path() / "data" / PRESET_USER_DIR / "default" / "sentinel.json", "owned");
    PresetBundle bundle;
    REQUIRE(bundle.backup_user_folder());
    const fs::path backup = temp.path() / "data" / (boost::format("user_backup-v%1%") % SoftFever_VERSION).str();
    CHECK(read_marker(backup / "default" / "sentinel.json") == "owned");
    write_marker(temp.path() / "data" / PRESET_USER_DIR / "default" / "sentinel.json", "changed");
    CHECK_FALSE(bundle.backup_user_folder());
    CHECK(read_marker(backup / "default" / "sentinel.json") == "owned");
}

TEST_CASE("PresetBundle vendor version and current vendor type follow the configured model", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    VendorProfile bbl("BBL");
    bbl.name = "BBL";
    bbl.config_version = *Semver::parse("3.2.1");
    VendorProfile::PrinterModel model;
    model.name = "X1 Carbon";
    model.id = "X1C";
    model.variants.emplace_back("0.4");
    bbl.models.push_back(model);
    bundle.vendors.emplace(bbl.id, bbl);
    set_variant(bundle.printers.get_edited_preset().config, "X1 Carbon", "0.4");
    auto &config = bundle.printers.get_edited_preset().config;
    CHECK(bundle.get_vendor_profile_version("BBL").to_string() == "3.2.0.1");
    CHECK_FALSE(bundle.get_vendor_profile_version("Missing").valid());
    CHECK(bundle.get_current_vendor_type() == VendorType::Marlin_BBL);
    for (bool local_host : {false, true}) {
        for (const std::string &url : {std::string{}, std::string{"http://printer.local"}}) {
            CAPTURE(local_host, url);
            config.option<ConfigOptionBool>("bbl_use_printhost", true)->value = local_host;
            config.option<ConfigOptionString>("print_host_webui", true)->value = url;
            CHECK(bundle.use_bbl_network() == !local_host);
            const bool expect_device_tab = !local_host || url.empty();
            CHECK(bundle.use_bbl_device_tab() == expect_device_tab);
        }
    }
    set_variant(bundle.printers.get_edited_preset().config, "Unlisted Model", "0.4");
    config.option<ConfigOptionBool>("bbl_use_printhost", true)->value = false;
    config.option<ConfigOptionString>("print_host_webui", true)->value.clear();
    CHECK(bundle.get_current_vendor_type() == VendorType::Unknown);
    CHECK_FALSE(bundle.use_bbl_network());
    CHECK_FALSE(bundle.use_bbl_device_tab());

}

TEST_CASE("PresetBundle custom vendor models retain distinct variants", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    Preset &a = bundle.printers.load_preset({}, "Custom A", bundle.printers.default_preset().config, false);
    set_variant(a.config, "Maker", "0.4");
    a.is_system = false;
    Preset &b = bundle.printers.load_preset({}, "Custom B", bundle.printers.default_preset().config, false);
    set_variant(b.config, "Maker", "0.6");
    b.is_system = false;
    VendorProfile custom = bundle.get_custom_vendor_models();
    REQUIRE(custom.id == PRESET_CUSTOM_VENDOR);
    REQUIRE(custom.models.size() == 1);
    CHECK(custom.models.front().name == "Maker");
    CHECK(custom.models.front().variants.size() == 2);
    CHECK(custom.models.front().variant("0.4") != nullptr);
    CHECK(custom.models.front().variant("0.6") != nullptr);
}

TEST_CASE("PresetBundle filament lookup returns typed compatibility metadata", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    Preset &filament = bundle.filaments.load_preset({}, "Acme PLA", bundle.filaments.default_preset().config, false);
    filament.filament_id = "fil-42";
    filament.alias = "PLA alias";
    auto &config = filament.config;
    config.option<ConfigOptionStrings>("filament_type", true)->values = {"PLA"};
    config.option<ConfigOptionStrings>("filament_vendor", true)->values = {"Acme"};
    config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"X1"};
    config.option<ConfigOptionInts>("filament_printable", true)->values = {2};
    config.option<ConfigOptionInts>("filament_extruder_compatibility", true)->values = {17};
    CHECK_FALSE(bundle.get_filament_by_filament_id("").has_value());
    CHECK_FALSE(bundle.get_filament_by_filament_id("missing").has_value());
    auto any_printer = bundle.get_filament_by_filament_id("fil-42");
    REQUIRE(any_printer.has_value());
    CHECK(any_printer->filament_name == "PLA alias");
    CHECK(any_printer->filament_type == "PLA");
    CHECK(any_printer->vendor == "Acme");
    auto compatible = bundle.get_filament_by_filament_id("fil-42", "X1");
    REQUIRE(compatible.has_value());
    CHECK(compatible->filament_printable == 2);
    CHECK(compatible->get_filament_extruder_compatibility() == 17);
    CHECK_FALSE(bundle.get_filament_by_filament_id("fil-42", "Other").has_value());
}

TEST_CASE("PresetBundle dispatches differed saves only for supported preset types", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    Preset &parent = load_local_preset(bundle.prints, "Parent");
    parent.config.opt<ConfigOptionFloat>("layer_height")->value = 0.2;
    Preset &child = load_local_preset(bundle.prints, "Child", "Parent");
    child.config.opt<ConfigOptionFloat>("layer_height")->value = 0.31;
    std::unique_ptr<Preset> diff(bundle.get_preset_differed_for_save(child));
    REQUIRE(diff != nullptr);
    CHECK_THAT(diff->config.opt_float("layer_height"), WithinAbs(0.31, 1e-6));
    CHECK_FALSE(diff->config.has("wall_loops"));
    std::map<std::string, std::string> values;
    CHECK(bundle.get_differed_values_to_update(child, values) == 0);
    REQUIRE(values.count("layer_height") == 1);
    CHECK(values.at("layer_height") == "0.31");
    Preset invalid(Preset::TYPE_INVALID, "invalid");
    CHECK(bundle.get_preset_differed_for_save(invalid) == nullptr);
    CHECK(bundle.get_differed_values_to_update(invalid, values) == -1);
}

TEST_CASE("PresetBundle project embedded enumeration and reset are bundle-wide", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    Preset &parent = load_local_preset(bundle.prints, "Embedded Parent");
    auto embedded = std::make_unique<Preset>(Preset::TYPE_PRINT, "Embedded Process");
    embedded->is_project_embedded = true;
    embedded->config = bundle.prints.default_preset().config;
    embedded->inherits() = parent.name;
    embedded->config.opt<ConfigOptionFloat>("layer_height")->value = 0.33;
    std::vector<Preset *> project{embedded.get()};
    PresetsConfigSubstitutions substitutions = bundle.load_project_embedded_presets(
        project, ForwardCompatibilitySubstitutionRule::Disable);
    CHECK(substitutions.empty());
    auto current = bundle.get_current_project_embedded_presets();
    REQUIRE(current.size() == 1);
    CHECK(current.front()->name == "Embedded Process");
    CHECK_THAT(current.front()->config.opt_float("layer_height"), WithinAbs(0.33, 1e-9));
    bundle.reset_project_embedded_presets();
    CHECK(bundle.get_current_project_embedded_presets().empty());
}

TEST_CASE("PresetBundle save_user_presets writes all selected collection roots", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-save-user");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.update_user_presets_directory("alice");
    std::vector<std::pair<PresetCollection *, std::string>> collections{
        {&bundle.prints, "Alice Process"}, {&bundle.filaments, "Alice Filament"},
        {&bundle.printers, "Alice Printer"}};
    for (const auto &[collection, name] : collections) {
        Preset &preset = load_local_preset(*collection, name);
        preset.sync_info = "save";
        preset.file = collection->path_for_preset(preset);
    }
    AppConfig app;
    app.set("preset_folder", "alice");
    std::map<std::string, std::string> deletions;
    bundle.save_user_presets(app, deletions);
    for (const auto &[collection, name] : collections) {
        const Preset *preset = collection->find_preset(name, false);
        REQUIRE(preset != nullptr);
        CHECK(fs::exists(preset->file));
        CHECK(preset->sync_info.empty());
        CHECK(read_marker(preset->file).find(name) != std::string::npos);
    }

}

TEST_CASE("PresetBundle sync cleanup repairs invalid user metadata", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-sync-fix");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.update_user_presets_directory("alice");
    Preset &print = load_local_preset(bundle.prints, "Alice Process");
    print.file = bundle.prints.path_for_preset(print);
    REQUIRE(print.save(nullptr));
    print.user_id.clear();
    print.setting_id = "legacy-id";
    print.sync_info.clear();
    bundle.check_and_fix_user_presets_syncinfo("alice");
    CHECK(print.user_id == "alice");
    CHECK(print.setting_id.empty());
    CHECK(print.sync_info == "create");
}

TEST_CASE("PresetBundle removes only the selected user's deleted cloud profiles", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-remove-user");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.update_user_presets_directory("alice");
    Preset &print = load_local_preset(bundle.prints, "To Remove");
    print.user_id = "alice";
    print.setting_id = "cloud-id";
    print.file = bundle.prints.path_for_preset(print);
    REQUIRE(print.save(nullptr));
    const fs::path removed_file = print.file;
    Preset &other = load_local_preset(bundle.prints, "Other User");
    other.user_id = "bob";
    other.setting_id = "other-cloud-id";
    other.file = bundle.prints.path_for_preset(other);
    REQUIRE(other.save(nullptr));
    const fs::path retained_file = other.file;
    AppConfig app;
    app.set("preset_folder", "alice");
    std::map<std::string, std::map<std::string, std::string>> cloud;
    bundle.remove_users_preset(app, &cloud);
    CHECK(bundle.prints.find_preset("To Remove", false) == nullptr);
    CHECK_FALSE(fs::exists(removed_file));
    CHECK(bundle.prints.find_preset("Other User", false) != nullptr);
    CHECK(fs::exists(retained_file));

}

TEST_CASE("PresetBundle removes a user directory without touching an empty-folder request", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-bundle-remove-dir");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    bundle.update_user_presets_directory("alice");
    write_marker(temp.path() / "data" / PRESET_USER_DIR / "alice" / "sentinel", "owned");
    bundle.remove_user_presets_directory("");
    CHECK(read_marker(temp.path() / "data" / PRESET_USER_DIR / "alice" / "sentinel") == "owned");
    bundle.remove_user_presets_directory("alice");
    CHECK_FALSE(fs::exists(temp.path() / "data" / PRESET_USER_DIR / "alice"));
}

TEST_CASE("PresetBundle validates printer, filament, and modified G-code references", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    load_local_preset(bundle.prints, "Process");
    Preset &filament = bundle.filaments.load_preset({}, "PLA", bundle.filaments.default_preset().config, false);
    filament.inherits().clear();
    filament.is_system = true;
    Preset &filament_two = bundle.filaments.load_preset({}, "PETG", bundle.filaments.default_preset().config, false);
    filament_two.inherits().clear();
    filament_two.is_system = true;
    Preset &printer = bundle.printers.load_preset({}, "Printer", bundle.printers.default_preset().config, false);
    printer.inherits().clear();
    printer.is_system = true;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.option<ConfigOptionFloats>("filament_diameter")->values = {1.75, 1.75};
    config.option<ConfigOptionStrings>("filament_settings_id", true)->values = {"PLA", "PETG"};
    config.option<ConfigOptionString>("printer_settings_id", true)->value = "Printer";
    config.option<ConfigOptionStrings>("inherits_group", true)->values = {"", "", "", ""};
    config.option<ConfigOptionStrings>("different_settings_to_system", true)->values = {"", "", "", ""};
    std::set<std::string> different;
    CHECK(bundle.validate_presets("project.3mf", config, different) == VALIDATE_PRESETS_SUCCESS);
    config.option<ConfigOptionStrings>("different_settings_to_system", true)->values = {"", "machine_end_gcode", "", ""};
    different.clear();
    CHECK(bundle.validate_presets("project.3mf", config, different) == VALIDATE_PRESETS_MODIFIED_GCODES);
    CHECK(different.count("machine_end_gcode") == 1);
    config.option<ConfigOptionString>("printer_settings_id", true)->value = "Missing Printer";
    different.clear();
    CHECK(bundle.validate_presets("project.3mf", config, different) == VALIDATE_PRESETS_PRINTER_NOT_FOUND);
    config.option<ConfigOptionString>("printer_settings_id", true)->value = "Printer";
    config.option<ConfigOptionStrings>("different_settings_to_system", true)->values = {"", "", "", ""};
    config.option<ConfigOptionStrings>("filament_settings_id", true)->values = {"PLA", "Missing Filament"};
    different.clear();
    CHECK(bundle.validate_presets("project.3mf", config, different) == VALIDATE_PRESETS_FILAMENTS_NOT_FOUND);
}

TEST_CASE("PresetBundle alias lookup and conversion keep canonical identities", "[Preset][PresetBundleLifecycleExtended]")
{
    PresetBundle bundle;
    Preset &print = load_local_preset(bundle.prints, "Canonical Process");
    print.alias.clear();
    CHECK(bundle.prints.get_preset_alias(print, true) == "Canonical Process");
    CHECK(bundle.get_preset_name_by_alias(Preset::TYPE_PRINT, "Canonical Process") == "Canonical Process");
    CHECK(bundle.get_preset_name_by_alias(Preset::TYPE_INVALID, "literal") == "literal");
    std::string converted = "Generic PLA";
    PresetBundle::convert_filament_preset_name("Unknown Machine", converted);
    CHECK(converted == "Generic PLA");
}

TEST_CASE("PresetCollection sync repair reconciles a child base id", "[Preset][PresetBundleLifecycleExtended]")
{
    ScopedTemporaryDir temp("preset-collection-sync-child");
    PresetCollection collection = make_print_collection();
    collection.update_user_presets_directory(temp.path().string(), PRESET_PRINT_NAME);
    Preset &parent = load_local_preset(collection, "Parent");
    parent.setting_id = "parent-id-123";
    Preset &child = load_local_preset(collection, "Child", "Parent");
    child.setting_id = "child-id-123";
    child.user_id = "alice";
    child.base_id = "stale-base";
    child.file = collection.path_for_preset(child);
    Preset *stored_parent = collection.find_preset("Parent", false);
    REQUIRE(stored_parent != nullptr);
    REQUIRE(child.save(&stored_parent->config));
    collection.check_and_fix_syncinfo(child, "alice");
    CHECK(child.base_id == "parent-id-123");
    CHECK(child.sync_info == "update");
}

} // namespace preset_bundle_lifecycle_extended_contracts

namespace preset_bundle_contracts {
using namespace Slic3r;
namespace fs = boost::filesystem;

namespace {

struct BundleFixture
{
    ScopedTemporaryDir temp{"preset-bundle-contracts"};
    PresetBundle bundle;

    BundleFixture()
    {
        const fs::path user = temp.path() / "user";
        fs::create_directories(user);
        bundle.prints.update_user_presets_directory(user.string(), PRESET_PRINT_NAME);
        bundle.filaments.update_user_presets_directory(user.string(), PRESET_FILAMENT_NAME);
        bundle.printers.update_user_presets_directory(user.string(), PRESET_PRINTER_NAME);
        bundle.sla_prints.update_user_presets_directory(user.string(), PRESET_SLA_PRINT_NAME);
        bundle.sla_materials.update_user_presets_directory(user.string(), PRESET_SLA_MATERIALS_NAME);
    }
};

Preset& add_printer(BundleFixture& fx, const std::string& name, const std::string& model, const std::string& variant, bool system = false)
{
    DynamicPrintConfig config                                          = fx.bundle.printers.default_preset().config;
    config.option<ConfigOptionString>("printer_model", true)->value    = model;
    config.option<ConfigOptionString>("printer_variant", true)->value  = variant;
    config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4};
    Preset& preset                                                     = fx.bundle.printers.load_preset({}, name, std::move(config), false);
    preset.is_system                                                   = system;
    preset.is_visible                                                  = true;
    preset.is_compatible                                               = true;
    return preset;
}

Preset& add_filament(
    BundleFixture& fx, const std::string& name, const std::string& id, const std::string& type, bool system = false, bool select = false)
{
    DynamicPrintConfig config = fx.bundle.filaments.default_preset().config;
    config.set_key_value("filament_id", new ConfigOptionStrings({id}));
    config.set_key_value("filament_type", new ConfigOptionStrings({type}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({"#123456"}));
    config.set_key_value("filament_is_support", new ConfigOptionBools({false}));
    Preset& preset       = fx.bundle.filaments.load_preset({}, name, std::move(config), select);
    preset.is_system     = system;
    preset.is_visible    = true;
    preset.is_compatible = true;
    preset.filament_id   = id;
    return preset;
}

DynamicPrintConfig tray(const std::string& id, const std::string& type, const std::string& color, bool changed = true)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_id", new ConfigOptionStrings({id}));
    config.set_key_value("filament_type", new ConfigOptionStrings({type}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({color}));
    config.set_key_value("filament_multi_colour", new ConfigOptionStrings({color}));
    config.set_key_value("tray_name", new ConfigOptionStrings({"Tray"}));
    config.set_key_value("filament_changed", new ConfigOptionBool(changed));
    return config;
}

void set_two_extruder_project(BundleFixture& fx)
{
    fx.bundle.project_config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode", true)->value = PrimeVolumeMode::pvmDefault;
    fx.bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter", true)->values        = {0.4, 0.6};
    fx.bundle.printers.get_edited_preset().config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {"Standard",
                                                                                                                        "HighFlow"};
    fx.bundle.project_config.option<ConfigOptionStrings>("filament_type", true)->values                              = {"PLA", "PETG"};
    fx.bundle.project_config.option<ConfigOptionStrings>("filament_colour", true)->values      = {"#AA0000", "#00AA00"};
    fx.bundle.project_config.option<ConfigOptionStrings>("filament_colour_type", true)->values = {"1", "1"};
    fx.bundle.project_config.option<ConfigOptionInts>("filament_map", true)->values            = {1, 2};
    fx.bundle.project_config.option<ConfigOptionInts>("filament_volume_map", true)->values     = {0, 0};
    fx.bundle.filament_presets                                                                 = {"PLA", "PETG"};
}


void set_variant_aware_printer(BundleFixture& fx)
{
    set_two_extruder_project(fx);
    auto& printer = fx.bundle.printers.get_edited_preset().config;
    // Both logical extruders are hybrid: each exposes one Standard and one High Flow
    // physical nozzle, and the inventory describes four canonical slots in total.
    printer.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4};
    printer.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {int(etDirectDrive), int(etDirectDrive)};
    printer.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {int(nvtHybrid), int(nvtHybrid)};
    printer.option<ConfigOptionStrings>("extruder_nozzle_stats", true)->values = {
        "Standard#1|High Flow#1", "Standard#1|High Flow#1"};
    printer.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {
        "Direct Drive Standard,Direct Drive High Flow", "Direct Drive Standard,Direct Drive High Flow"};
    printer.option<ConfigOptionInts>("printer_extruder_id", true)->values = {1, 1, 2, 2};
    printer.option<ConfigOptionStrings>("printer_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow", "Direct Drive Standard", "Direct Drive High Flow"};

    // full_fff_config applies project_config after the printer preset. Keep the project-side
    // nozzle type hybrid as well, otherwise the project default silently collapses both extruders
    // back to Standard before variant expansion.
    fx.bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {
        int(nvtHybrid), int(nvtHybrid)};
}

void set_variant_aware_filament(Preset& preset, double standard_speed, double high_flow_speed)
{
    auto& config = preset.config;
    config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow"};
    // One physical diameter describes the filament; the two variant slots are selected by
    // filament_extruder_variant and must not be mistaken for two diameters.
    config.option<ConfigOptionFloats>("filament_diameter", true)->values = {1.75};
    config.option<ConfigOptionFloats>("filament_max_volumetric_speed", true)->values = {
        standard_speed, high_flow_speed};
}

} // namespace

TEST_CASE("secure full config removes host credentials while retaining slicing values", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    set_two_extruder_project(fx);
    auto& printer = fx.bundle.printers.get_edited_preset().config;
    printer.set_key_value("print_host", new ConfigOptionString("printer.local"));
    printer.set_key_value("print_host_webui", new ConfigOptionString("http://printer.local"));
    printer.set_key_value("printhost_apikey", new ConfigOptionString("secret"));
    printer.set_key_value("printhost_user", new ConfigOptionString("alice"));
    printer.set_key_value("printhost_password", new ConfigOptionString("password"));
    printer.set_key_value("printhost_port", new ConfigOptionString("443"));
    fx.bundle.prints.get_edited_preset().config.set_key_value("layer_height", new ConfigOptionFloat(0.23));

    const DynamicPrintConfig secure = fx.bundle.full_config_secure();
    CHECK_THAT(secure.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.23, 1e-9));
    for (const char* key : {"print_host", "print_host_webui", "printhost_apikey", "printhost_user", "printhost_password", "printhost_port",
                            "printhost_cafile"})
        CHECK_FALSE(secure.has(key));
}

TEST_CASE("flush matrix preserves dimensions and applies per extruder multipliers", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    set_two_extruder_project(fx);
    fx.bundle.project_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values = {10., 20., 30., 40., 50., 60., 70., 80.};
    fx.bundle.project_config.option<ConfigOptionFloats>("flush_multiplier", true)->values     = {2., 3.};
    const auto raw                                                                            = fx.bundle.get_full_flush_matrix(false);
    const auto scaled                                                                         = fx.bundle.get_full_flush_matrix(true);
    REQUIRE(raw.size() == 2);
    REQUIRE(scaled.size() == raw.size());
    for (size_t extruder = 0; extruder < raw.size(); ++extruder) {
        REQUIRE(raw[extruder].size() == 2);
        REQUIRE(scaled[extruder].size() == raw[extruder].size());
        for (size_t from = 0; from < 2; ++from) {
            REQUIRE(raw[extruder][from].size() == 2);
            REQUIRE(scaled[extruder][from].size() == 2);
            for (size_t to = 0; to < 2; ++to) {
                const float expected = 10.f * float(1 + 4 * extruder + 2 * from + to);
                CHECK_THAT(raw[extruder][from][to], Catch::Matchers::WithinAbs(expected, 1e-5));
                CHECK_THAT(scaled[extruder][from][to], Catch::Matchers::WithinAbs(expected * (extruder == 0 ? 2.f : 3.f), 1e-5));
            }
        }
    }
}

TEST_CASE("fast purge uses the fast multiplier independently of default multiplier", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    set_two_extruder_project(fx);
    fx.bundle.project_config.option<ConfigOptionFloats>("flush_volumes_matrix", true)->values  = {4., 8., 12., 16., 20., 24., 28., 32.};
    fx.bundle.project_config.option<ConfigOptionFloats>("flush_multiplier", true)->values      = {2., 2.};
    fx.bundle.project_config.option<ConfigOptionFloats>("flush_multiplier_fast", true)->values = {5., 7.};
    const auto regular                                                                         = fx.bundle.get_full_flush_matrix(true);
    fx.bundle.project_config.option<ConfigOptionEnum<PrimeVolumeMode>>("prime_volume_mode", true)->value = PrimeVolumeMode::pvmFast;
    const auto fast = fx.bundle.get_full_flush_matrix(true);
    REQUIRE(regular.size() == 2);
    REQUIRE(fast.size() == 2);
    for (size_t e = 0; e < 2; ++e) {
        REQUIRE(regular[e].size() == 2);
        REQUIRE(fast[e].size() == 2);
        for (size_t from = 0; from < 2; ++from) {
            REQUIRE(regular[e][from].size() == 2);
            REQUIRE(fast[e][from].size() == 2);
            for (size_t to = 0; to < 2; ++to) {
                const float volume = 4.f * float(1 + 4 * e + 2 * from + to);
                CHECK_THAT(regular[e][from][to], Catch::Matchers::WithinAbs(volume * 2.f, 1e-5));
                CHECK_THAT(fast[e][from][to], Catch::Matchers::WithinAbs(volume * (e == 0 ? 5.f : 7.f), 1e-5));
            }
        }
    }
}

TEST_CASE("filament nozzle volume defaults follow valid maps and standard-fallback maps", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    fx.bundle.project_config.option<ConfigOptionEnumsGeneric>("nozzle_volume_type", true)->values = {int(nvtHighFlow), int(nvtStandard)};
    std::vector<int> maps{1, 2, 9};
    const auto result = fx.bundle.get_default_nozzle_volume_types_for_filaments(maps);
    REQUIRE(result.size() == maps.size());
    CHECK(result == std::vector<int>{int(nvtHighFlow), int(nvtStandard), int(nvtStandard)});
}

TEST_CASE("different extruder support is determined by the variant set", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    set_two_extruder_project(fx);
    CHECK(fx.bundle.support_different_extruders());
    fx.bundle.printers.get_edited_preset().config.option<ConfigOptionStrings>("extruder_variant_list", true)->values = {"Standard",
                                                                                                                        "Standard"};
    CHECK_FALSE(fx.bundle.support_different_extruders());
}

TEST_CASE("extruder filament information partitions AMS cells by left and right extruder", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    set_two_extruder_project(fx);
    fx.bundle.filament_ams_list.emplace(0, tray("L", "PLA", "#110000"));
    fx.bundle.filament_ams_list.emplace(1, tray("L2", "PLA", "#220000"));
    fx.bundle.filament_ams_list.emplace(0x10000, tray("R", "PETG", "#001100"));
    const auto info = fx.bundle.get_extruder_filament_info();
    REQUIRE(info.size() == 2);
    CHECK(info[0].size() == 2);
    CHECK(info[1].size() == 1);
    CHECK(info[0][0].opt<ConfigOptionStrings>("filament_id")->values.front() == "L");
    CHECK(info[1][0].opt<ConfigOptionStrings>("filament_id")->values.front() == "R");
}

TEST_CASE("single extruder has no right-hand AMS configuration", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    fx.bundle.filament_ams_list.emplace(0x10000, tray("R", "PETG", "#001100"));
    CHECK(fx.bundle.get_extruder_filament_info().empty());
}

TEST_CASE("used TPU filament indices are zero-based and ignore unselected slots", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    add_filament(fx, "TPU", "TPU-ID", "TPU");
    add_filament(fx, "PLA", "PLA-ID", "PLA");
    fx.bundle.filament_presets = {"TPU", "PLA"};
    CHECK(fx.bundle.get_used_tpu_filaments({1, 99}) == std::vector<int>{0});
    CHECK(fx.bundle.get_used_tpu_filaments({2}) == std::vector<int>{});
}

TEST_CASE("only edited filament requires a unique selected slot", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    add_filament(fx, "TPU", "TPU-ID", "TPU", false, true);
    add_filament(fx, "PLA", "PLA-ID", "PLA");
    fx.bundle.filament_presets = {"TPU", "PLA"};
    CHECK(fx.bundle.is_the_only_edited_filament(0));
    fx.bundle.filament_presets = {"TPU", "TPU"};
    CHECK_FALSE(fx.bundle.is_the_only_edited_filament(0));
    CHECK_FALSE(fx.bundle.is_the_only_edited_filament(4));
}

TEST_CASE("AMS combo info resolves matching IDs and preserves changed false slots", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    add_filament(fx, "Generic PLA", "PLA-ID", "PLA", true);
    fx.bundle.filament_presets = {"Old Selection", "Another Selection"};
    fx.bundle.filament_ams_list.emplace(0, tray("PLA-ID", "PLA", "#AB12CD"));
    fx.bundle.filament_ams_list.emplace(1, tray("unknown", "PLA", "#00AA00", false));
    AMSComboInfo info;
    fx.bundle.get_ams_cobox_infos(info);
    REQUIRE(info.ams_filament_presets.size() == 2);
    CHECK(info.ams_filament_presets[0] == "Generic PLA");
    CHECK(info.ams_filament_presets[1] == "Another Selection");
    CHECK(info.ams_filament_colors == std::vector<std::string>{"#AB12CD", "#00AA00"});
}

TEST_CASE("AMS combo fallback prefers generic material then prior selection then compatible system material", "[PresetBundle][AmsComboFallback]")
{
    const int scenario = GENERATE(0, 1, 2, 3, 4);
    INFO("fallback scenario=" << scenario);
    BundleFixture fx;
    fx.bundle.filament_presets.clear();
    add_filament(fx, "Generic PLA Incompatible", "incompatible", "PLA", true).is_compatible = false;
    add_filament(fx, "Generic PLA User", "user", "PLA", false);
    if (scenario == 0 || scenario == 4)
        add_filament(fx, "Generic PLA Compatible", "generic", "PLA", true);
    if (scenario != 3)
        add_filament(fx, "System PETG", "system", "PETG", true);
    if (scenario == 0 || scenario == 1 || scenario == 4) {
        add_filament(fx, "Previously Selected", "previous", "PETG", false);
        fx.bundle.filament_presets = {"Previously Selected"};
    }

    DynamicPrintConfig physical_tray = tray(scenario == 4 ? "" : "unknown-material", "PLA", "#AB12CD");
    const std::vector<std::string> colors{"#AB12CD", "#0099FF"};
    physical_tray.set_key_value("filament_multi_colour", new ConfigOptionStrings(colors));
    physical_tray.set_key_value("tray_name", new ConfigOptionStrings({"Bay A"}));
    fx.bundle.filament_ams_list.emplace(7, std::move(physical_tray));

    AMSComboInfo info;
    info.ams_filament_presets = {"stale"};
    info.ams_filament_colors = {"stale"};
    info.ams_multi_color_filment = {{"stale"}};
    info.ams_names = {"stale"};
    fx.bundle.get_ams_cobox_infos(info);
    if (scenario >= 3) {
        CHECK(info.empty());
        CHECK(info.ams_filament_presets.empty());
        CHECK(info.ams_filament_colors.empty());
        CHECK(info.ams_multi_color_filment.empty());
        CHECK(info.ams_names.empty());
    } else {
        const std::array<std::string, 3> expected{{"Generic PLA Compatible", "Previously Selected", "System PETG"}};
        REQUIRE(info.ams_filament_presets.size() == 1);
        CHECK(info.ams_filament_presets.front() == expected[scenario]);
        REQUIRE(info.ams_filament_colors.size() == 1);
        CHECK(info.ams_filament_colors.front() == "#AB12CD");
        REQUIRE(info.ams_multi_color_filment.size() == 1);
        CHECK(info.ams_multi_color_filment.front() == colors);
        REQUIRE(info.ams_names.size() == 1);
        CHECK(info.ams_names.front() == "Bay A");
    }
}


TEST_CASE("similar printer chooses requested variant among the same model", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    Preset& a = add_printer(fx, "Acme 0.4", "Acme", "0.4", true);
    Preset& b = add_printer(fx, "Acme 0.6", "Acme", "0.6", true);
    fx.bundle.printers.select_preset_by_name(a.name, true);
    CHECK(fx.bundle.get_similar_printer_preset("Acme", "0.6") == &b);
    CHECK(fx.bundle.get_similar_printer_preset("Missing", "0.6") == nullptr);
}

TEST_CASE("full SLA config merges selected profiles and marks SLA technology", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    auto printer_config                                                               = fx.bundle.printers.default_preset().config;
    printer_config.option<ConfigOptionEnumGeneric>("printer_technology", true)->value = ptSLA;
    printer_config.set_key_value("printer_settings_id", new ConfigOptionString("SLA Printer"));
    fx.bundle.printers.load_preset({}, "SLA Printer", printer_config, true);
    auto sla_print_config = fx.bundle.sla_prints.default_preset().config;
    sla_print_config.set_key_value("sla_print_settings_id", new ConfigOptionString("SLA Print"));
    fx.bundle.sla_prints.load_preset({}, "SLA Print", sla_print_config, true);
    auto sla_material_config = fx.bundle.sla_materials.default_preset().config;
    sla_material_config.set_key_value("sla_material_settings_id", new ConfigOptionString("SLA Material"));
    fx.bundle.sla_materials.load_preset({}, "SLA Material", sla_material_config, true);
    const DynamicPrintConfig config = fx.bundle.full_config();
    CHECK(config.opt<ConfigOptionEnumGeneric>("printer_technology")->value == ptSLA);
    CHECK(config.opt_string("printer_settings_id") == "SLA Printer");
    CHECK(config.opt_string("sla_print_settings_id") == "SLA Print");
    CHECK(config.opt_string("sla_material_settings_id") == "SLA Material");
    CHECK_FALSE(config.has("compatible_printers"));
    CHECK_FALSE(config.has("compatible_printers_condition"));
    CHECK_FALSE(config.has("inherits"));
}

TEST_CASE("system setting IDs are attached only to matching printer entries", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    Preset& printer = add_printer(fx, "System Printer", "Acme", "0.4", true);
    std::map<std::string, std::map<std::string, std::string>> remote;
    remote[printer.name] = {{BBL_JSON_KEY_TYPE, PRESET_IOT_PRINTER_TYPE}, {BBL_JSON_KEY_SETTING_ID, "remote-id"}};
    fx.bundle.update_system_preset_setting_ids(remote);
    CHECK(printer.setting_id == "remote-id");
    remote["missing"] = {{BBL_JSON_KEY_TYPE, PRESET_IOT_PRINTER_TYPE}, {BBL_JSON_KEY_SETTING_ID, "other"}};
    fx.bundle.update_system_preset_setting_ids(remote);
    CHECK(printer.setting_id == "remote-id");
}

TEST_CASE("project embedded preset loading resolves its explicit parent", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    DynamicPrintConfig parent_config                                     = fx.bundle.prints.default_preset().config;
    parent_config.option<ConfigOptionFloat>("layer_height", true)->value = 0.2;
    Preset& parent                  = fx.bundle.prints.load_preset({}, "Contract Parent", parent_config, false);
    DynamicPrintConfig child_config = parent.config;
    child_config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = parent.name;
    child_config.option<ConfigOptionFloat>("layer_height", true)->value         = 0.27;
    auto child                 = std::make_unique<Preset>(Preset::TYPE_PRINT, "Contract Embedded");
    child->config              = std::move(child_config);
    child->is_project_embedded = true;
    std::vector<Preset*> input{child.get()};
    PresetsConfigSubstitutions substitutions;
    fx.bundle.prints.load_project_embedded_presets(input, PRESET_PRINT_NAME, substitutions,
                                                   ForwardCompatibilitySubstitutionRule::EnableSilent);
    Preset* loaded = fx.bundle.prints.find_preset("Contract Embedded", false, true);
    REQUIRE(loaded != nullptr);
    CHECK(loaded->is_project_embedded);
    CHECK_THAT(loaded->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.27, 1e-9));
}

TEST_CASE("project embedded reset removes embedded entries but retains ordinary presets", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    const std::string detached = fx.bundle.prints.add_detached_preset("Contract Detached", DynamicPrintConfig::full_print_config());
    REQUIRE(fx.bundle.prints.find_preset(detached) != nullptr);
    DynamicPrintConfig ordinary_config = fx.bundle.prints.default_preset().config;
    fx.bundle.prints.load_preset({}, "Contract Ordinary", ordinary_config, false);
    fx.bundle.prints.select_preset_by_name(detached, true);
    CHECK(fx.bundle.prints.reset_project_embedded_presets());
    CHECK(fx.bundle.prints.find_preset(detached) == nullptr);
    CHECK(fx.bundle.prints.find_preset("Contract Ordinary") != nullptr);
}

TEST_CASE("detached preset names are unique and never create a user file", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    const std::string first = fx.bundle.filaments.add_detached_preset("Published Filament", DynamicPrintConfig::full_print_config(), "fid");
    const std::string second = fx.bundle.filaments.add_detached_preset("Published Filament", DynamicPrintConfig::full_print_config(), "fid");
    REQUIRE(first != second);
    CHECK(second == "Published Filament (Published)");
    Preset* stored = fx.bundle.filaments.find_preset(second);
    REQUIRE(stored != nullptr);
    CHECK(stored->is_project_embedded);
    CHECK(stored->filament_id == "fid");
    CHECK_FALSE(fs::exists(stored->file));

    // Embedded presets are project-owned.  Their cleanup path must not remove
    // a file that happens to be associated with the in-memory preset.
    stored->file = (fx.temp.path() / "embedded-owned.json").string();
    std::ofstream owned(stored->file);
    owned << "owned";
    owned.close();
    stored->remove_files();
    CHECK(fs::exists(stored->file));
    CHECK(stored->save(nullptr));
    std::ifstream after_save(stored->file);
    std::string saved_text((std::istreambuf_iterator<char>(after_save)), std::istreambuf_iterator<char>());
    CHECK(saved_text == "owned");
}

TEST_CASE("External preset loading selects the imported profile", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    DynamicPrintConfig config                                           = fx.bundle.prints.default_preset().config;
    config.option<ConfigOptionFloat>("layer_height", true)->value       = 0.31;
    config.option<ConfigOptionString>("print_settings_id", true)->value = "Imported";
    ScopedTemporaryFile source(".gcode");
    const auto loaded = fx.bundle.prints.load_external_preset(source.string(), "Imported", "", config, {"layer_height"},
                                                              PresetCollection::LoadAndSelect::Always);
    REQUIRE(loaded.first != nullptr);
    CHECK(loaded.first->is_external);
    CHECK_THAT(loaded.first->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
    CHECK(fx.bundle.prints.get_selected_preset_name() == "(Imported)");
}

TEST_CASE("preset default suppression changes first visible selection", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    VendorProfile library(PresetBundle::ORCA_FILAMENT_LIBRARY);
    library.id                                             = PresetBundle::ORCA_FILAMENT_LIBRARY;
    fx.bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY] = library;
    VendorProfile& library_ref                             = fx.bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
    fx.bundle.prints.default_preset().vendor               = &library_ref;
    DynamicPrintConfig config                              = fx.bundle.prints.default_preset().config;
    Preset& visible                                        = fx.bundle.prints.load_preset({}, "Visible Process", config, false);
    visible.vendor                                         = &library_ref;
    fx.bundle.prints.set_default_suppressed(true);
    CHECK(fx.bundle.prints.is_default_suppressed());
    CHECK(fx.bundle.prints.first_visible().name == "Visible Process");
    fx.bundle.prints.set_default_suppressed(false);
    CHECK(fx.bundle.prints.first_visible().is_default);
}

TEST_CASE("preset parent and base preserve inheritance identity", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    DynamicPrintConfig parent_config = fx.bundle.prints.default_preset().config;
    Preset& parent                   = fx.bundle.prints.load_preset({}, "Parent", parent_config, false);
    const std::string parent_name    = parent.name;
    DynamicPrintConfig child_config  = parent.config;
    child_config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = parent_name;
    Preset& child                 = fx.bundle.prints.load_preset({}, "Child", child_config, false);
    const Preset* resolved_parent = fx.bundle.prints.get_preset_parent(child);
    const Preset* resolved_base   = fx.bundle.prints.get_preset_base(child);
    REQUIRE(resolved_parent != nullptr);
    REQUIRE(resolved_base != nullptr);
    CHECK(resolved_parent->name == parent_name);
    CHECK(resolved_base->name == parent_name);
    CHECK(fx.bundle.prints.get_preset_parent(*resolved_base) == nullptr);
}

TEST_CASE("invalid embedded parent is rejected without installing a preset", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    auto child    = std::make_unique<Preset>(Preset::TYPE_PRINT, "Orphan Embedded");
    child->config = fx.bundle.prints.default_preset().config;
    child->config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = "No Such Parent";
    child->is_project_embedded                                                   = true;
    std::vector<Preset*> input{child.get()};
    PresetsConfigSubstitutions substitutions;
    fx.bundle.prints.load_project_embedded_presets(input, PRESET_PRINT_NAME, substitutions,
                                                   ForwardCompatibilitySubstitutionRule::EnableSilent);
    CHECK(fx.bundle.prints.find_preset("Orphan Embedded", false, true) == nullptr);
}

TEST_CASE("collection names and section names identify their preset type", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    CHECK(fx.bundle.prints.name() == PRESET_PRINT_NAME);
    CHECK(fx.bundle.prints.section_name() == PRESET_PRINT_NAME);
    CHECK(fx.bundle.filaments.section_name() == PRESET_FILAMENT_NAME);
    CHECK(fx.bundle.printers.section_name() == PRESET_PRINTER_NAME);
}

TEST_CASE("empty detached names are rejected without changing collection", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    const size_t before = fx.bundle.prints.size();
    CHECK(fx.bundle.prints.add_detached_preset("", DynamicPrintConfig::full_print_config()).empty());
    CHECK(fx.bundle.prints.size() == before);
}

TEST_CASE("filament map selection keeps the selected edited slot unique", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    add_filament(fx, "Mapped", "mapped", "PLA");
    fx.bundle.filaments.get_edited_preset().name = "Mapped";
    fx.bundle.filament_presets                   = {"Mapped", "Other"};
    CHECK(fx.bundle.is_the_only_edited_filament(0));
    fx.bundle.filament_presets.push_back("Mapped");
    CHECK_FALSE(fx.bundle.is_the_only_edited_filament(0));
}

TEST_CASE("AMS combo info clears stale output before resolving an empty tray list", "[PresetBundle][PresetBundleContracts]")
{
    BundleFixture fx;
    AMSComboInfo info;
    info.ams_filament_presets = {"stale"};
    info.ams_names            = {"stale"};
    fx.bundle.get_ams_cobox_infos(info);
    CHECK(info.empty());
    CHECK(info.ams_filament_presets.empty());
}


TEST_CASE("Full FFF configuration applies explicit filament volume maps", "[PresetBundle][PresetBundleContracts][PresetBundleFullFffMaps]")
{
    const auto [first_volume, second_volume, first_speed, second_speed] = GENERATE(
        table<NozzleVolumeType, NozzleVolumeType, double, double>({
            {nvtStandard, nvtStandard, 12., 13.},
            {nvtStandard, nvtHighFlow, 12., 23.},
            {nvtHighFlow, nvtStandard, 22., 13.},
            {nvtHighFlow, nvtHighFlow, 22., 23.},
        }));
    CAPTURE(first_volume, second_volume);

    BundleFixture fx;
    set_variant_aware_printer(fx);
    add_filament(fx, "PLA", "PLA-ID", "PLA", false, true);
    add_filament(fx, "PETG", "PETG-ID", "PETG");
    Preset *pla = fx.bundle.filaments.find_preset("PLA", false, true);
    Preset *petg = fx.bundle.filaments.find_preset("PETG", false, true);
    REQUIRE(pla != nullptr);
    REQUIRE(petg != nullptr);
    set_variant_aware_filament(*pla, 12., 22.);
    set_variant_aware_filament(*petg, 13., 23.);
    REQUIRE(fx.bundle.filaments.select_preset_by_name("PLA", true));
    fx.bundle.filament_presets = {"PLA", "PETG"};
    fx.bundle.project_config.option<ConfigOptionInts>("filament_volume_map", true)->values = {
        int(nvtHighFlow), int(nvtHighFlow)};

    const DynamicPrintConfig out = fx.bundle.full_config(
        true, std::vector<int>{1, 2}, std::vector<int>{int(first_volume), int(second_volume)});
    CHECK(out.option<ConfigOptionInts>("filament_volume_map")->values ==
          std::vector<int>{int(first_volume), int(second_volume)});
    CHECK(out.option<ConfigOptionStrings>("filament_extruder_variant")->values ==
          std::vector<std::string>{
              first_volume == nvtStandard ? "Direct Drive Standard" : "Direct Drive High Flow",
              second_volume == nvtStandard ? "Direct Drive Standard" : "Direct Drive High Flow"});
    CHECK(out.option<ConfigOptionStrings>("filament_settings_id")->values ==
          std::vector<std::string>{"PLA", "PETG"});
    CHECK(out.option<ConfigOptionStrings>("filament_ids")->values ==
          std::vector<std::string>{"PLA-ID", "PETG-ID"});
    check_double_vector(out.option<ConfigOptionFloats>("filament_max_volumetric_speed")->values,
                        {first_speed, second_speed});
}

TEST_CASE("Full FFF configuration records selected system filament changes", "[PresetBundle][PresetBundleContracts][PresetBundleFullFffProvenance]")
{
    BundleFixture fx;
    add_filament(fx, "System PLA", "SYSTEM-PLA", "PLA", true, true);
    Preset& user = add_filament(fx, "User PETG", "USER-PETG", "PETG");
    user.config.option<ConfigOptionFloats>("filament_max_volumetric_speed", true)->values = {9.0};
    fx.bundle.filament_presets = {"System PLA", "User PETG"};
    REQUIRE(fx.bundle.filaments.select_preset_by_name("System PLA", true));
    REQUIRE(fx.bundle.filaments.get_edited_preset().is_system);

    // The selected system preset is copied into the public edited preset. Change that copy
    // after selection so full_fff_config must compare it with the saved system profile.
    auto& edited = fx.bundle.filaments.get_edited_preset();
    edited.config.option<ConfigOptionFloats>("filament_max_volumetric_speed", true)->values = {31.5};
    edited.inherits() = "ignored-parent-metadata";

    const DynamicPrintConfig out = fx.bundle.full_config(false);
    CHECK(out.option<ConfigOptionStrings>("filament_settings_id")->values ==
          std::vector<std::string>{"System PLA", "User PETG"});
    CHECK(out.option<ConfigOptionStrings>("filament_ids")->values ==
          std::vector<std::string>{"SYSTEM-PLA", "USER-PETG"});
    check_double_vector(out.option<ConfigOptionFloats>("filament_max_volumetric_speed")->values, {31.5, 9.0});
    const auto* different = out.option<ConfigOptionStrings>("different_settings_to_system");
    REQUIRE(different != nullptr);
    REQUIRE(different->values.size() == 4);
    CHECK(different->values[0].empty());
    CHECK_FALSE(different->values[1].empty());
    CHECK(different->values[2].empty());
    CHECK(different->values[3].empty());
    std::vector<std::string> changed_keys;
    REQUIRE(unescape_strings_cstyle(different->values[1], changed_keys));
    CHECK(std::find(changed_keys.begin(), changed_keys.end(), "filament_max_volumetric_speed") != changed_keys.end());
    CHECK(std::find(changed_keys.begin(), changed_keys.end(), "inherits") == changed_keys.end());
}

TEST_CASE("Removing selected cloud printer and filament restores configured defaults", "[PresetBundle][PresetCloudRemoval]")
{
    BundleFixture fx;
    ScopedDataDir data(fx.temp.path());

    Preset &fallback_printer = add_printer(
        fx, "Fallback Printer", PresetBundle::ORCA_DEFAULT_PRINTER_MODEL, "0.4", true);
    const std::string fallback_printer_name = fallback_printer.name;
    fallback_printer.config.option<ConfigOptionStrings>("default_filament_profile", true)->values = {"Fallback PLA"};
    Preset &fallback_filament = add_filament(fx, "Fallback PLA", "fallback-id", "PLA", true);
    const std::string fallback_filament_name = fallback_filament.name;

    auto persist_user = [](PresetCollection &collection, Preset &preset, const std::string &user_id,
                           const std::string &setting_id) {
        preset.user_id = user_id;
        preset.setting_id = setting_id;
        preset.file = collection.path_for_preset(preset);
        const fs::path file = preset.file;
        REQUIRE(preset.save(nullptr));
        return file;
    };

    Preset &removed_printer = add_printer(fx, "Alice Printer", "Alice Model", "0.4");
    const fs::path removed_printer_file = persist_user(fx.bundle.printers, removed_printer, "alice", "printer-cloud-id");
    Preset &removed_filament = add_filament(fx, "Alice PLA", "alice-id", "PLA");
    const fs::path removed_filament_file = persist_user(fx.bundle.filaments, removed_filament, "alice", "filament-cloud-id");

    REQUIRE(fx.bundle.printers.select_preset_by_name("Alice Printer", true));
    REQUIRE(fx.bundle.filaments.select_preset_by_name("Alice PLA", true));
    fx.bundle.filament_presets = {"Alice PLA", "Stale Missing Filament"};

    AppConfig app;
    app.set("preset_folder", "alice");
    std::map<std::string, std::map<std::string, std::string>> cloud;
    fx.bundle.remove_users_preset(app, &cloud);

    CHECK(fx.bundle.printers.find_preset("Alice Printer", false) == nullptr);
    CHECK_FALSE(fs::exists(removed_printer_file));
    CHECK(fx.bundle.filaments.find_preset("Alice PLA", false) == nullptr);
    CHECK_FALSE(fs::exists(removed_filament_file));
    CHECK(fx.bundle.printers.get_selected_preset_name() == fallback_printer_name);
    CHECK(fx.bundle.filaments.get_selected_preset_name() == fallback_filament_name);
    CHECK(fx.bundle.filament_presets == std::vector<std::string>{fallback_filament_name, fallback_filament_name});
}

TEST_CASE("Removing cloud presets preserves live metadata and clears a dirty setting id", "[PresetBundle][PresetCloudRemoval]")
{
    BundleFixture fx;
    ScopedDataDir data(fx.temp.path());

    auto persist_user_print = [&](const std::string &name, const std::string &setting_id,
                                  const std::string &sync_info, bool dirty) {
        Preset &preset = preset_extended_contracts::load_local_preset(fx.bundle.prints, name);
        preset.user_id = "alice";
        preset.setting_id = setting_id;
        preset.sync_info = sync_info;
        preset.is_dirty = dirty;
        preset.file = fx.bundle.prints.path_for_preset(preset);
        const fs::path file = preset.file;
        REQUIRE(preset.save(nullptr));
        return file;
    };

    const fs::path listed_file = persist_user_print("Cloud Listed", "listed-id", "", false);
    const fs::path syncing_file = persist_user_print("Syncing", "sync-id", "update", false);
    const fs::path no_id_file = persist_user_print("No Setting Id", "", "", false);
    const fs::path dirty_file = persist_user_print("Dirty", "dirty-id", "", false);
    const fs::path removed_file = persist_user_print("Deleted", "deleted-id", "", false);

    REQUIRE(fx.bundle.prints.select_preset_by_name("Cloud Listed", true));
    Preset *dirty_before_remove = fx.bundle.prints.find_preset("Dirty", false, true);
    REQUIRE(dirty_before_remove != nullptr);
    dirty_before_remove->is_dirty = true;
    std::map<std::string, std::map<std::string, std::string>> cloud;
    cloud.emplace("Cloud Listed", std::map<std::string, std::string>{{"setting_id", "listed-id"}});
    AppConfig app;
    app.set("preset_folder", "alice");
    fx.bundle.remove_users_preset(app, &cloud);

    const Preset *listed = fx.bundle.prints.find_preset("Cloud Listed", false);
    REQUIRE(listed != nullptr);
    CHECK(listed->setting_id == "listed-id");
    CHECK(fs::exists(listed_file));
    CHECK(fx.bundle.prints.get_selected_preset_name() == "Cloud Listed");

    const Preset *syncing = fx.bundle.prints.find_preset("Syncing", false);
    REQUIRE(syncing != nullptr);
    CHECK(syncing->sync_info == "update");
    CHECK(fs::exists(syncing_file));

    const Preset *no_id = fx.bundle.prints.find_preset("No Setting Id", false);
    REQUIRE(no_id != nullptr);
    CHECK(no_id->setting_id.empty());
    CHECK(fs::exists(no_id_file));

    const Preset *dirty_preset = fx.bundle.prints.find_preset("Dirty", false);
    REQUIRE(dirty_preset != nullptr);
    CHECK(dirty_preset->setting_id.empty());
    CHECK(fs::exists(dirty_file));

    CHECK(fx.bundle.prints.find_preset("Deleted", false) == nullptr);
    CHECK_FALSE(fs::exists(removed_file));
}

} // namespace preset_bundle_contracts

namespace {
Preset &ams_add(PresetCollection &collection, const std::string &name)
{
    auto config = collection.default_preset().config;
    config.option<ConfigOptionString>("inherits", true)->value.clear();
    return collection.load_preset({}, name, config, false);
}
struct AmsValidationFixture {
    PresetBundle bundle;
    AmsValidationFixture()
    {
        auto &vendor = bundle.vendors["Contract Vendor"];
        vendor.id = vendor.name = "Contract Vendor";
        auto &printer = ams_add(bundle.printers, "Contract Printer 0.4 nozzle");
        printer.is_system = true;
        printer.vendor = &vendor;
        printer.config.option<ConfigOptionString>("printer_model", true)->value = "Contract Printer";
        printer.config.option<ConfigOptionString>("printer_variant", true)->value = "0.4";
        printer.config.option<ConfigOptionStrings>("default_filament_profile", true)->values.clear();
    }
    Preset &filament(const std::string &name)
    {
        auto &preset = ams_add(bundle.filaments, name);
        preset.filament_id = "contract-material";
        preset.setting_id = "setting-" + name;
        preset.vendor = &bundle.vendors.at("Contract Vendor");
        preset.config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"Contract Printer 0.4 nozzle"};
        preset.config.option<ConfigOptionInts>("nozzle_temperature_range_low", true)->values = {195};
        preset.config.option<ConfigOptionInts>("nozzle_temperature_range_high", true)->values = {245};
        return preset;
    }
};
}
TEST_CASE("AMS tray temperatures reconcile only the identified compatible user material", "[PresetBundle][AmsValidation]")
{
    const int scenario = GENERATE(0, 1, 2, 3, 4, 5, 6, 7, 8);
    CAPTURE(scenario);
    AmsValidationFixture fixture;
    auto &preset = fixture.filament("User material");
    std::string nozzle = "0.4", material = "contract-material", tag = "1234567890129999";
    std::string low = "180", high = "260", setting = "unchanged";
    bool expected_equal = false;
    std::string expected_low = "195", expected_high = "245";
    switch (scenario) {
    case 1: low = expected_low; high = expected_high; expected_equal = true; break;
    case 2: low = expected_low; break;
    case 3: high = expected_high; break;
    case 4: tag = "0"; expected_equal = true; break;
    case 5: tag = "1234567890120199"; expected_equal = true; break;
    case 6: material = "absent"; expected_equal = true; break;
    case 7: preset.is_system = true; expected_equal = true; break;
    case 8: preset.config.option<ConfigOptionStrings>("compatible_printers")->values = {"Other Printer"}; expected_equal = true; break;
    default: break;
    }
    if (expected_equal) { expected_low = low; expected_high = high; }
    const bool equal = fixture.bundle.check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray(
        "Contract Printer", nozzle, material, tag, low, high, setting, 0, nvtStandard);
    CHECK(equal == expected_equal);
    CHECK(low == expected_low);
    CHECK(high == expected_high);
    CHECK(setting == (expected_equal ? "unchanged" : "setting-User material"));
}
TEST_CASE("AMS duplicate validation scopes material identifiers by compatible system printer", "[PresetBundle][AmsValidation]")
{
    const int scenario = GENERATE(0, 1, 2, 3, 4, 5, 6);
    CAPTURE(scenario);
    AmsValidationFixture fixture;
    fixture.filament("First").is_system = true;
    auto &model = fixture.bundle.vendors.at("Contract Vendor").models.emplace_back();
    model.id = model.name = "Contract Printer";
    model.variants.emplace_back("0.4");
    model.default_materials = {"First"};
    auto &second = fixture.filament("Second");
    second.is_system = true;
    bool duplicate = true;
    if (scenario == 1) { second.filament_id = "distinct"; duplicate = false; }
    if (scenario == 2) { second.filament_id.clear(); duplicate = false; }
    if (scenario == 3) { second.is_system = false; duplicate = false; }
    if (scenario == 4) {
        auto &library = fixture.bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
        library.id = library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
        second.vendor = &library;
        second.config.option<ConfigOptionStrings>("compatible_printers")->values.clear();
    }
    if (scenario == 5) {
        auto &library = fixture.bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
        library.id = library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;
        second.vendor = &library;
        second.config.option<ConfigOptionStrings>("compatible_printers")->values.clear();
        second.m_excluded_from.insert("Contract Printer 0.4 nozzle");
        duplicate = false;
    }
    if (scenario == 6) {
        fixture.bundle.printers.find_preset("Contract Printer 0.4 nozzle")->is_system = false;
        duplicate = false;
    }
    // The optional check is the only reason this otherwise valid bundle fails.
    REQUIRE_FALSE(fixture.bundle.has_errors(false));
    CHECK(fixture.bundle.has_errors(true) == duplicate);
}

TEST_CASE("AMS tray temperatures select the extruder and nozzle variant", "[PresetBundle][AmsValidation]")
{
    const int variant = GENERATE(0, 1, 2, 3);
    CAPTURE(variant);
    AmsValidationFixture fixture;
    auto &preset = fixture.filament("Variant material");
    preset.config.option<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive High Flow", "Bowden Standard"};
    preset.config.option<ConfigOptionInts>("nozzle_temperature_range_low")->values = {195, 205, 215};
    preset.config.option<ConfigOptionInts>("nozzle_temperature_range_high")->values = {245, 255, 265};
    fixture.bundle.printers.find_preset("Contract Printer 0.4 nozzle")
        ->config.option<ConfigOptionEnumsGeneric>("extruder_type", true)->values = {etDirectDrive, etBowden};
    std::string nozzle = "0.4", material = "contract-material", tag = "1234567890129999";
    std::string low = "170", high = "300", setting = "unchanged";
    const bool equal = fixture.bundle.check_filament_temp_equation_by_printer_type_and_nozzle_for_mas_tray(
        "Contract Printer", nozzle, material, tag, low, high, setting, variant / 2,
        variant % 2 == 0 ? nvtStandard : nvtHighFlow);
    CHECK_FALSE(equal);
    // No Bowden High Flow entry exists, so that nozzle uses the first variant.
    const std::vector<std::string> expected_low{"195", "205", "215", "195"};
    const std::vector<std::string> expected_high{"245", "255", "265", "245"};
    CHECK(low == expected_low[variant]);
    CHECK(high == expected_high[variant]);
    CHECK(setting == "setting-Variant material");
}

namespace preset_ams_sync_contracts {
namespace {

DynamicPrintConfig tray(const std::string& filament_id,
                        const std::string& color,
                        const std::string& type,
                        const std::string& ams_id      = "0",
                        const std::string& slot_id     = "0",
                        bool changed                   = true,
                        bool placeholder               = false,
                        std::vector<std::string> multi = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("filament_id", new ConfigOptionStrings({filament_id}));
    config.set_key_value("filament_colour", new ConfigOptionStrings({color}));
    config.set_key_value("filament_colour_type", new ConfigOptionStrings({"1"}));
    config.set_key_value("filament_type", new ConfigOptionStrings({type}));
    config.set_key_value("filament_multi_colour", new ConfigOptionStrings(std::move(multi)));
    config.set_key_value("ams_id", new ConfigOptionStrings({ams_id}));
    config.set_key_value("slot_id", new ConfigOptionStrings({slot_id}));
    config.set_key_value("filament_changed", new ConfigOptionBool(changed));
    config.set_key_value("filament_slot_placeholder", new ConfigOptionBools({placeholder}));
    return config;
}

Preset& system_filament(PresetBundle& bundle, const std::string& name, const std::string& id, const std::string& type)
{
    Preset& preset       = bundle.filaments.load_preset({}, name, bundle.filaments.default_preset().config, false);
    preset.is_system     = true;
    preset.is_visible    = true;
    preset.is_compatible = true;
    preset.filament_id   = id;
    preset.config.option<ConfigOptionStrings>("filament_type", true)->values = {type};
    return preset;
}

void project_slots(PresetBundle& bundle, std::vector<std::string> names, std::vector<std::string> colors)
{
    REQUIRE(names.size() == colors.size());
    bundle.project_config.apply(FullPrintConfig::defaults());
    bundle.prints.get_edited_preset().config.apply(FullPrintConfig::defaults());
    bundle.filament_presets                                                            = std::move(names);
    bundle.project_config.option<ConfigOptionStrings>("filament_colour", true)->values = colors;
    bundle.project_config.option<ConfigOptionStrings>("filament_colour_type", true)->values.assign(colors.size(), "1");
    bundle.project_config.option<ConfigOptionInts>("filament_map", true)->values.assign(colors.size(), 1);
    bundle.project_config.option<ConfigOptionInts>("filament_volume_map", true)->values.assign(colors.size(), 0);
    bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour", true)->values = colors;
}

void add_tray(PresetBundle& bundle, int index, DynamicPrintConfig config) { bundle.filament_ams_list.emplace(index, std::move(config)); }

void run_sync(PresetBundle& bundle,
              bool use_map,
              std::map<int, AMSMapInfo>& maps,
              bool enable_append,
              bool color_only,
              std::vector<std::pair<DynamicPrintConfig*, std::string>>& unknowns,
              MergeFilamentInfo& merge)
{
    bundle.sync_ams_list(unknowns, use_map, maps, enable_append, merge, color_only);
}

} // namespace

TEST_CASE("BBL HRC lookup filters vendor, missing values, and unknown filament types", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    VendorProfile& bbl   = bundle.vendors["BBL"];
    bbl.id               = "BBL";
    VendorProfile& other = bundle.vendors["Other"];
    other.id             = "Other";

    Preset& pla                                                              = system_filament(bundle, "BBL PLA", "bbl-pla", "PLA");
    pla.vendor                                                               = &bbl;
    pla.config.option<ConfigOptionInts>("required_nozzle_HRC", true)->values = {58};
    Preset& cf                                                               = system_filament(bundle, "BBL PA-CF", "bbl-pa-cf", "PA-CF");
    cf.vendor                                                                = &bbl;
    cf.config.option<ConfigOptionInts>("required_nozzle_HRC", true)->values  = {72};
    Preset& without_hrc                                                      = system_filament(bundle, "BBL TPU", "bbl-tpu", "TPU");
    without_hrc.vendor                                                       = &bbl;
    without_hrc.config.erase("required_nozzle_HRC");
    Preset& foreign                                                              = system_filament(bundle, "Other PLA", "other-pla", "PLA");
    foreign.vendor                                                               = &other;
    foreign.config.option<ConfigOptionInts>("required_nozzle_HRC", true)->values = {99};

    CHECK(bundle.get_required_hrc_by_filament_type("PLA") == 58);
    CHECK(bundle.get_required_hrc_by_filament_type("PA-CF") == 72);
    CHECK(bundle.get_required_hrc_by_filament_type("TPU") == 0);
    CHECK(bundle.get_required_hrc_by_filament_type("ABS") == 0);
}

TEST_CASE("AMS unknown type uses the longest whole-word generic preset match", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PA", "pa-id", "PA");
    system_filament(bundle, "Generic PA-CF", "pa-cf-id", "PA-CF");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("missing-id", "#112233", "PA-CF blend"));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, false, maps, false, false, unknowns, merge);

    REQUIRE(bundle.filament_presets.size() == 1);
    CHECK(bundle.filament_presets.front() == "Generic PA-CF");
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#112233"});
    REQUIRE(unknowns.size() == 1);
    CHECK(unknowns.front().first == &bundle.filament_ams_list.begin()->second);
}

TEST_CASE("AMS placeholder merge fills slots beyond the existing project selection", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    project_slots(bundle, {"Existing"}, {"#AAAAAA"});
    add_tray(bundle, 0, tray("", "", "", "0", "0", true, true));
    add_tray(bundle, 1, tray("", "", "", "0", "1", true, true));
    add_tray(bundle, 2, tray("", "", "", "0", "2", true, true));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, false, maps, false, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Existing", "Generic PLA", "Generic PLA"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values ==
          std::vector<std::string>{"#AAAAAA", "#CECECE", "#CECECE"});
    CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 1, 1});
}

TEST_CASE("AMS synchronization keeps left and right physical trays in map order", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    system_filament(bundle, "Generic PETG", "petg-id", "PETG");
    bundle.printers.get_edited_preset().config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.6};
    project_slots(bundle, {"Old left", "Old right"}, {"#000000", "#000000"});
    add_tray(bundle, 0, tray("pla-id", "#110000", "PLA", "left", "0"));
    add_tray(bundle, 0x10000, tray("petg-id", "#001100", "PETG", "right", "0"));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, false, maps, false, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA", "Generic PETG"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#110000", "#001100"});
    CHECK(unknowns.empty());
}

TEST_CASE("AMS color-only sync falls back to tray indices when a supplied map does not match", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    project_slots(bundle, {"PLA A", "PLA B"}, {"#AAAAAA", "#BBBBBB"});
    bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour")->values = {"#AAAAAA #AA0000", "#BBBBBB"};
    add_tray(bundle, 0, tray("pla-id", "#112233", "PLA", "A", "0", true, false, {"#112233", "#334455"}));
    add_tray(bundle, 1, tray("pla-id", "#445566", "PLA", "A", "1", true, false, {"#445566"}));
    std::map<int, AMSMapInfo> maps{{0, {"wrong", "slot"}}};

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    MergeFilamentInfo merge;
    run_sync(bundle, true, maps, false, true, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"PLA A", "PLA B"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#112233", "#445566"});
    CHECK(bundle.ams_multi_color_filment == std::vector<std::vector<std::string>>{{"#112233", "#334455"}, {"#445566"}});
    CHECK(merge.is_empty());
}

TEST_CASE("AMS mapped sync appends each unmapped tray once when append is enabled", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    system_filament(bundle, "Generic PETG", "petg-id", "PETG");
    project_slots(bundle, {"Existing"}, {"#000000"});
    add_tray(bundle, 0, tray("pla-id", "#112233", "PLA", "A", "0"));
    add_tray(bundle, 1, tray("petg-id", "#445566", "PETG", "A", "1"));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}};

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    MergeFilamentInfo merge;
    run_sync(bundle, true, maps, true, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA", "Generic PETG"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#112233", "#445566"});
    CHECK(bundle.project_config.opt<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 1});
}

TEST_CASE("AMS overwrite re-appends mixed slots without changing their metadata", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    project_slots(bundle, {"Physical", "Mixed"}, {"#101010", "#202020"});
    bundle.project_config.option<ConfigOptionBools>("filament_is_mixed", true)->values                = {false, true};
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_components", true)->values      = {"", "1,2"};
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_sublayer_ratios", true)->values = {"", "0.25,0.75"};
    bundle.project_config.option<ConfigOptionBools>("filament_mixed_gradient", true)->values          = {false, true};
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_gradient_range", true)->values  = {"", "2-8"};
    bundle.project_config.option<ConfigOptionStrings>("filament_mixed_gradient_curve", true)->values  = {"", "linear"};
    bundle.project_config.option<ConfigOptionBools>("filament_mixed_gradient_per_part", true)->values = {false, true};
    add_tray(bundle, 0, tray("pla-id", "#AABBCC", "PLA"));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, false, maps, false, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA", "Mixed"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#AABBCC", "#202020"});
    CHECK(bundle.project_config.opt<ConfigOptionBools>("filament_is_mixed")->values == std::vector<unsigned char>{0, 1});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_components")->values == std::vector<std::string>{"", "1,2"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values ==
          std::vector<std::string>{"", "0.25,0.75"});
    CHECK(bundle.project_config.opt<ConfigOptionBools>("filament_mixed_gradient")->values == std::vector<unsigned char>{0, 1});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values == std::vector<std::string>{"", "2-8"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values == std::vector<std::string>{"", "linear"});
    CHECK(bundle.project_config.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values == std::vector<unsigned char>{0, 1});
}

TEST_CASE("AMS unknown material without generic preset falls back to a compatible system preset", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Custom System", "custom-id", "PLA");
    project_slots(bundle, {}, {});
    add_tray(bundle, 0, tray("missing-id", "#334455", "Nylon"));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, false, maps, false, false, unknowns, merge);

    REQUIRE(bundle.filament_presets.size() == 1);
    CHECK(bundle.filament_presets.front() == "Custom System");
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#334455"});
    REQUIRE(unknowns.size() == 1);
    CHECK_FALSE(unknowns.front().second.empty());
}

TEST_CASE("AMS duplicate unmapped trays are appended only once", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    project_slots(bundle, {}, {});
    add_tray(bundle, 0, tray("pla-id", "#112233", "PLA", "A", "0"));
    add_tray(bundle, 1, tray("pla-id", "#112233", "PLA", "A", "1"));

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    std::map<int, AMSMapInfo> maps;
    MergeFilamentInfo merge;
    run_sync(bundle, true, maps, true, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA"});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#112233"});
}

TEST_CASE("AMS map merges record all project slots pointing at one physical tray", "[PresetBundle][PresetAmsSync]")
{
    PresetBundle bundle;
    system_filament(bundle, "Generic PLA", "pla-id", "PLA");
    project_slots(bundle, {"Old A", "Old B"}, {"#000000", "#000000"});
    add_tray(bundle, 0, tray("pla-id", "#112233", "PLA", "A", "0"));
    std::map<int, AMSMapInfo> maps{{0, {"A", "0"}}, {1, {"A", "0"}}};

    std::vector<std::pair<DynamicPrintConfig*, std::string>> unknowns;
    MergeFilamentInfo merge;
    run_sync(bundle, true, maps, false, false, unknowns, merge);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Generic PLA", "Generic PLA"});
    REQUIRE(merge.merges.size() == 1);
    CHECK(merge.merges.front() == std::vector<int>{0, 1});
    CHECK(bundle.project_config.opt<ConfigOptionStrings>("filament_colour")->values == std::vector<std::string>{"#112233", "#112233"});
}

} // namespace preset_ams_sync_contracts

namespace preset_bundle_calibration_contracts {
namespace {

bool contains_filament(const PresetBundle &bundle, const Preset *preset)
{
    return bundle.calibrate_filaments.find(preset) != bundle.calibrate_filaments.end();
}

} // namespace

TEST_CASE("Calibration printer selection follows filament compatibility profiles", "[PresetBundle][PresetCalibration]")
{
    PresetBundle bundle;

    // Loading another preset may insert into the middle of the collection. Keep
    // references only during setup, and resolve the expected pointers afterward.
    {
        Preset &printer = add_inmemory_preset(bundle.printers, "Calibration Printer");
        printer.is_system = true;
        printer.config.option<ConfigOptionString>("printer_model", true)->value = "Calibration Model";
        printer.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4};
    }
    {
        Preset &printer = add_inmemory_preset(bundle.printers, "Other Printer");
        printer.is_system = true;
        printer.config.option<ConfigOptionString>("printer_model", true)->value = "Other Model";
        printer.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.6};
    }
    {
        Preset &filament = add_inmemory_preset(bundle.filaments, "Direct Calibration Filament");
        filament.is_system = true;
        filament.config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"Calibration Printer"};
    }
    {
        Preset &filament = add_inmemory_preset(bundle.filaments, "Conditional Calibration Filament");
        filament.is_system = true;
        filament.config.option<ConfigOptionStrings>("compatible_printers", true)->values.clear();
        filament.config.option<ConfigOptionString>("compatible_printers_condition", true)->value =
            R"(printer_model == "Calibration Model")";
    }
    {
        Preset &filament = add_inmemory_preset(bundle.filaments, "Other Printer Filament");
        filament.is_system = true;
        filament.config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"Other Printer"};
    }
    const Preset *calibration_printer = bundle.printers.find_preset("Calibration Printer");
    const Preset *other_printer = bundle.printers.find_preset("Other Printer");
    const Preset *direct = bundle.filaments.find_preset("Direct Calibration Filament");
    const Preset *conditional = bundle.filaments.find_preset("Conditional Calibration Filament");
    const Preset *other_only = bundle.filaments.find_preset("Other Printer Filament");
    REQUIRE(calibration_printer != nullptr);
    REQUIRE(other_printer != nullptr);
    REQUIRE(direct != nullptr);
    REQUIRE(conditional != nullptr);
    REQUIRE(other_only != nullptr);

    bundle.set_calibrate_printer(calibration_printer->name);
    REQUIRE(bundle.calibrate_printer == calibration_printer);
    REQUIRE(bundle.calibrate_filaments.size() == 2);
    CHECK(contains_filament(bundle, direct));
    CHECK(contains_filament(bundle, conditional));
    CHECK_FALSE(contains_filament(bundle, other_only));

    bundle.set_calibrate_printer(other_printer->name);
    REQUIRE(bundle.calibrate_printer == other_printer);
    REQUIRE(bundle.calibrate_filaments.size() == 1);
    CHECK(contains_filament(bundle, other_only));
    CHECK_FALSE(contains_filament(bundle, direct));
    CHECK_FALSE(contains_filament(bundle, conditional));

    bundle.set_calibrate_printer("");
    CHECK(bundle.calibrate_filaments.empty());
}

} // namespace preset_bundle_calibration_contracts
namespace preset_installed_filament_contracts {

TEST_CASE("Installed filament discovery persists compatible vendor defaults", "[PresetBundle][PresetInstalledFilaments]")
{
    PresetBundle bundle;
    VendorProfile &vendor = bundle.vendors["Acme"];
    vendor.id = vendor.name = "Acme";
    VendorProfile::PrinterModel model;
    model.id = model.name = "Acme Model";
    model.variants.emplace_back("0.4");
    model.default_materials = {"Default PLA"};
    vendor.models.emplace_back(std::move(model));

    Preset &printer = add_inmemory_preset(bundle.printers, "Acme Printer");
    printer.is_system = true;
    printer.is_visible = true;
    printer.vendor = &vendor;
    printer.config.option<ConfigOptionString>("printer_model", true)->value = "Acme Model";
    printer.config.option<ConfigOptionString>("printer_variant", true)->value = "0.4";
    printer.config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4};
    REQUIRE(printer.printer_technology() == ptFFF);

    Preset &default_filament_setup = add_inmemory_preset(bundle.filaments, "Default PLA");
    default_filament_setup.is_system = true;
    default_filament_setup.is_visible = false;
    default_filament_setup.vendor = &vendor;

    Preset &installed_filament_setup = add_inmemory_preset(bundle.filaments, "Installed PLA");
    installed_filament_setup.is_system = true;
    installed_filament_setup.is_visible = false;
    installed_filament_setup.vendor = &vendor;
    installed_filament_setup.config.option<ConfigOptionStrings>("compatible_printers", true)->values = {"Acme Printer"};

    AppConfig app_config;
    app_config.set_variant("Acme", "Acme Model", "0.4", true);
    app_config.set("presets", PRESET_PRINTER_NAME, "Acme Printer");

    SECTION("missing installed section seeds model defaults")
    {
        REQUIRE_FALSE(app_config.has_section(AppConfig::SECTION_FILAMENTS));

        bundle.load_selections(app_config);
        REQUIRE(bundle.printers.get_selected_preset_name() == "Acme Printer");

        const Preset *default_filament = bundle.filaments.find_preset("Default PLA", false, true);
        const Preset *installed_filament = bundle.filaments.find_preset("Installed PLA", false, true);
        REQUIRE(default_filament != nullptr);
        REQUIRE(installed_filament != nullptr);
        REQUIRE(app_config.has_section(AppConfig::SECTION_FILAMENTS));
        const auto &installed = app_config.get_section(AppConfig::SECTION_FILAMENTS);
        REQUIRE(installed.count("Default PLA") == 1);
        CHECK(installed.at("Default PLA") == "true");
        CHECK(installed.count("Installed PLA") == 0);
        CHECK(default_filament->is_visible);
        CHECK_FALSE(installed_filament->is_visible);
    }

    SECTION("compatible installed section suppresses model defaults")
    {
        app_config.set_section(AppConfig::SECTION_FILAMENTS, {{"Installed PLA", "true"}});
        REQUIRE(app_config.has_section(AppConfig::SECTION_FILAMENTS));
        REQUIRE(app_config.get_section(AppConfig::SECTION_FILAMENTS).count("Installed PLA") == 1);

        bundle.load_selections(app_config);
        REQUIRE(bundle.printers.get_selected_preset_name() == "Acme Printer");

        const Preset *default_filament = bundle.filaments.find_preset("Default PLA", false, true);
        const Preset *installed_filament = bundle.filaments.find_preset("Installed PLA", false, true);
        REQUIRE(default_filament != nullptr);
        REQUIRE(installed_filament != nullptr);
        const auto &installed = app_config.get_section(AppConfig::SECTION_FILAMENTS);
        CHECK(installed.count("Default PLA") == 0);
        CHECK(installed.at("Installed PLA") == "true");
        CHECK_FALSE(default_filament->is_visible);
        CHECK(installed_filament->is_visible);
    }
}

} // namespace preset_installed_filament_contracts

namespace preset_bundle_workflow_contracts {

namespace {

void write_vendor_model_tree(const boost::filesystem::path &root, const std::string &vendor,
                             const std::string &model_name, const std::string &model_json)
{
    const boost::filesystem::path vendor_dir = root / vendor;
    fs::create_directories(vendor_dir / "machine");
    const nlohmann::json index_document = {
        {"version", "1.2.3"},
        {"name", vendor},
        {"machine_model_list", {{{"name", model_name}, {"sub_path", "machine/model.json"}}}},
    };
    std::ofstream index((root / (vendor + ".json")).string());
    REQUIRE(index.good());
    index << index_document.dump();
    index.flush();
    REQUIRE(index.good());

    std::ofstream model_file((vendor_dir / "machine" / "model.json").string());
    REQUIRE(model_file.good());
    model_file << model_json;
    model_file.flush();
    REQUIRE(model_file.good());
}

const VendorProfile::PrinterModel &only_model(const PresetBundle &bundle, const std::string &vendor)
{
    const auto vendor_it = bundle.vendors.find(vendor);
    REQUIRE(vendor_it != bundle.vendors.end());
    REQUIRE(vendor_it->second.models.size() == 1);
    return vendor_it->second.models.front();
}

size_t load_json_vendor(PresetBundle &bundle, const boost::filesystem::path &root, const std::string &vendor)
{
    return bundle.load_vendor_configs_from_json(
        root.string(), vendor, PresetBundle::LoadSystem,
        ForwardCompatibilitySubstitutionRule::EnableSilent, nullptr, false).second;
}

} // namespace

TEST_CASE("JSON vendor printer metadata preserves machine fields and normalizes lists",
          "[PresetBundle][VendorJson]")
{
    ScopedTemporaryDir dir;
    ScopedDataDir data(dir.path() / "json-vendor-data");
    PresetBundle bundle;
    const std::string vendor = "MetaVendor";
    write_vendor_model_tree(boost::filesystem::path(data_dir()), vendor, "Meta FFF",
        R"({"type":"machine_model","name":"Meta FFF","model_id":"FFF-42",
"nozzle_diameter":"0.4;0.6;0.4","machine_tech":"FFF","family":"CoreXY",
"bed_model":"plate.stl","bed_texture":"plate.png","image_bed_type":"textured",
"bottom_texture_end_name":"END","use_double_extruder_default_texture":"true",
"bottom_texture_rect":"1,2,3,4","bottom_texture_rect_longer":"5,6,7,8",
"middle_texture_rect":"9,10,11,12","hotend_model":"Acme Hotend",
"default_materials":";Vendor PETG;Vendor PLA;Vendor PETG",
"not_support_bed_type":"Cool; ;Textured;Cool"})");

    REQUIRE(load_json_vendor(bundle, boost::filesystem::path(data_dir()), vendor) == 0);
    REQUIRE(bundle.error_count() == 0);
    const VendorProfile::PrinterModel &model = only_model(bundle, vendor);
    const VendorProfile &profile = bundle.vendors.at(vendor);
    CHECK(profile.name == vendor);
    const auto expected_version = Semver::parse("1.2.3");
    REQUIRE(static_cast<bool>(expected_version));
    CHECK(profile.config_version == *expected_version);
    CHECK(model.id == "Meta FFF");
    CHECK(model.model_id == "FFF-42");
    CHECK(model.technology == ptFFF);
    CHECK(model.family == "CoreXY");
    REQUIRE(model.variants.size() == 2);
    CHECK(model.variants[0].name == "0.4");
    CHECK(model.variants[1].name == "0.6");
    CHECK(model.bed_model == "plate.stl");
    CHECK(model.bed_texture == "plate.png");
    CHECK(model.image_bed_type == "textured");
    CHECK(model.bottom_texture_end_name == "END");
    CHECK(model.use_double_extruder_default_texture == "true");
    CHECK(model.bottom_texture_rect == "1,2,3,4");
    CHECK(model.bottom_texture_rect_longer == "5,6,7,8");
    CHECK(model.middle_texture_rect == "9,10,11,12");
    CHECK(model.hotend_model == "Acme Hotend");
    const std::vector<std::string> expected_default_materials{"Vendor PETG", "Vendor PLA"};
    const std::vector<std::string> expected_unsupported_beds{"Cool", "Textured"};
    CHECK(model.default_materials == expected_default_materials);
    CHECK(model.not_support_bed_types == expected_unsupported_beds);
}

TEST_CASE("JSON vendor printer metadata preserves SLA technology and empty lists",
          "[PresetBundle][VendorJson]")
{
    ScopedTemporaryDir dir;
    ScopedDataDir data(dir.path() / "json-vendor-data");
    PresetBundle bundle;
    const std::string vendor = "ResinVendor";
    write_vendor_model_tree(boost::filesystem::path(data_dir()), vendor, "Resin One",
        R"({"type":"machine_model","name":"Resin One","model_id":"SLA-1",
"nozzle_diameter":"0.25","machine_tech":"SLA","family":"Resin",
"bed_model":"resin_plate.stl","bed_texture":"resin_plate.png",
"default_materials":"","not_support_bed_type":";;"})");

    REQUIRE(load_json_vendor(bundle, boost::filesystem::path(data_dir()), vendor) == 0);
    REQUIRE(bundle.error_count() == 0);
    const VendorProfile::PrinterModel &model = only_model(bundle, vendor);
    CHECK(model.technology == ptSLA);
    CHECK(model.family == "Resin");
    CHECK(model.model_id == "SLA-1");
    REQUIRE(model.variants.size() == 1);
    CHECK(model.variants.front().name == "0.25");
    CHECK(model.default_materials.empty());
    CHECK(model.not_support_bed_types.empty());
    CHECK(model.bed_model == "resin_plate.stl");
    CHECK(model.bed_texture == "resin_plate.png");
}

TEST_CASE("JSON vendor printer metadata counts malformed default material lists",
          "[PresetBundle][VendorJson]")
{
    ScopedTemporaryDir dir;
    ScopedDataDir data(dir.path() / "json-vendor-data");
    PresetBundle bundle;
    const std::string vendor = "BrokenMetadataVendor";
    // The JSON value is valid, but its C-style list has an unterminated quoted item.
    write_vendor_model_tree(boost::filesystem::path(data_dir()), vendor, "Broken Defaults",
        R"({"type":"machine_model","name":"Broken Defaults","model_id":"BROKEN-1",
"nozzle_diameter":"0.4","machine_tech":"FFF",
"default_materials":"\"unterminated"})");

    REQUIRE(load_json_vendor(bundle, boost::filesystem::path(data_dir()), vendor) == 0);
    REQUIRE(bundle.error_count() == 1);
    const VendorProfile::PrinterModel &model = only_model(bundle, vendor);
    REQUIRE(model.variants.size() == 1);
    CHECK(model.default_materials.empty());
}

} // namespace preset_bundle_workflow_contracts

namespace preset_bundle_workflow_contracts {

TEST_CASE("JSON vendor malformed list fields preserve their distinct load outcomes",
          "[PresetBundle][VendorJson]")
{
    const auto [vendor, model_name, model_json, expected_errors, expected_models] = GENERATE(
        table<std::string, std::string, std::string, int, int>({
            {
                "BrokenNozzleVendor", "Broken Nozzle",
                R"({"type":"machine_model","name":"Broken Nozzle","model_id":"BROKEN-NOZZLE",
"nozzle_diameter":"\"unterminated","machine_tech":"FFF",
"default_materials":"","not_support_bed_type":""})",
                1, 0,
            },
            {
                "BrokenBedListVendor", "Broken Bed List",
                R"({"type":"machine_model","name":"Broken Bed List","model_id":"BROKEN-BED",
"nozzle_diameter":"0.4","machine_tech":"FFF",
"default_materials":"","not_support_bed_type":"\"unterminated"})",
                0, 1,
            },
        }));
    CAPTURE(vendor, model_name);

    ScopedTemporaryDir dir("vendor-json-malformed-adjacent");
    ScopedDataDir data(dir.path() / "json-vendor-data");
    PresetBundle bundle;
    write_vendor_model_tree(boost::filesystem::path(data_dir()), vendor, model_name, model_json);

    // The public loader must return normally after the parser records a malformed field;
    // a format-arity exception is the regression this case prevents.
    REQUIRE(load_json_vendor(bundle, boost::filesystem::path(data_dir()), vendor) == 0);
    CHECK(bundle.error_count() == expected_errors);

    const VendorProfile &profile = bundle.vendors.at(vendor);
    REQUIRE(static_cast<int>(profile.models.size()) == expected_models);
    if (expected_models == 1) {
        const VendorProfile::PrinterModel &model = profile.models.front();
        REQUIRE(model.variants.size() == 1);
        CHECK(model.variants.front().name == "0.4");
        CHECK(model.default_materials.empty());
        CHECK(model.not_support_bed_types.empty());
    }
}

} // namespace preset_bundle_workflow_contracts

namespace preset_extended_contracts {

TEST_CASE("Preset parent save emits nil for inherited filament variants",
          "[Preset][PresetNullableSave][PresetExtendedVariance]")
{
    ScopedTemporaryDir dir("preset-nullable-save");
    ScopedDataDir data(dir.path() / "data");

    Preset parent(Preset::TYPE_FILAMENT, "Base", false);
    parent.config = DynamicPrintConfig::full_print_config();
    auto *parent_flow = parent.config.option<ConfigOptionFloatsNullable>("filament_flow_ratio", true);
    REQUIRE(parent_flow != nullptr);
    parent_flow->values = { 0.95, 0.96 };
    auto *parent_variants = parent.config.option<ConfigOptionStrings>("filament_extruder_variant", true);
    REQUIRE(parent_variants != nullptr);
    parent_variants->values = { "Direct Drive Standard", "Direct Drive High Flow" };

    Preset child(Preset::TYPE_FILAMENT, "Child", false);
    child.config = parent.config;
    child.inherits() = parent.name;
    auto *child_flow = child.config.option<ConfigOptionFloatsNullable>("filament_flow_ratio", true);
    REQUIRE(child_flow != nullptr);
    child_flow->values = { 0.99, 0.96 };
    child.file = (dir.path() / "Child.json").string();

    REQUIRE(child.save(&parent.config));
    REQUIRE(fs::exists(child.file));

    std::string serialized;
    load_string_file(child.file, serialized);
    const nlohmann::json saved = nlohmann::json::parse(serialized);
    REQUIRE(saved.contains("filament_flow_ratio"));
    const nlohmann::json &saved_flow = saved.at("filament_flow_ratio");
    REQUIRE(saved_flow.is_array());
    REQUIRE(saved_flow.size() == 2);
    REQUIRE(saved_flow.at(0).is_string());
    REQUIRE(saved_flow.at(1).is_string());
    CHECK(saved_flow.at(0).get<std::string>() == "0.99");
    CHECK(saved_flow.at(1).get<std::string>() == "nil");

    // The saved nil marker remains an inherited slot when the public reload path
    // applies the child delta over its parent.
    Preset reloaded(Preset::TYPE_FILAMENT, "Child", false);
    reloaded.file = child.file;
    reloaded.reload(parent);
    const auto *reloaded_flow = reloaded.config.option<ConfigOptionFloatsNullable>("filament_flow_ratio");
    REQUIRE(reloaded_flow != nullptr);
    REQUIRE(reloaded_flow->values.size() == 2);
    CHECK_THAT(reloaded_flow->values[0], WithinAbs(0.99, 1e-9));
    CHECK(reloaded_flow->is_nil(1));

    // Load both entries before taking pointers.  load_preset inserts into the
    // collection, so obtaining references between insertions would make the
    // fixture depend on container insertion details.
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    load_local_preset(collection, "Base");
    load_local_preset(collection, "Child", "Base");
    Preset *collection_parent = collection.find_preset("Base", false, true);
    Preset *collection_child = collection.find_preset("Child", false, true);
    REQUIRE(collection_parent != nullptr);
    REQUIRE(collection_child != nullptr);

    auto *collection_parent_flow = collection_parent->config.option<ConfigOptionFloatsNullable>("filament_flow_ratio", true);
    auto *collection_child_flow = collection_child->config.option<ConfigOptionFloatsNullable>("filament_flow_ratio", true);
    auto *collection_parent_variants = collection_parent->config.option<ConfigOptionStrings>("filament_extruder_variant", true);
    auto *collection_child_variants = collection_child->config.option<ConfigOptionStrings>("filament_extruder_variant", true);
    REQUIRE(collection_parent_flow != nullptr);
    REQUIRE(collection_child_flow != nullptr);
    REQUIRE(collection_parent_variants != nullptr);
    REQUIRE(collection_child_variants != nullptr);
    collection_parent_flow->values = { 0.95, 0.96 };
    collection_child_flow->values = { 0.99, 0.96 };
    collection_parent_variants->values = { "Direct Drive Standard", "Direct Drive High Flow" };
    collection_child_variants->values = { "Direct Drive Standard", "Direct Drive High Flow" };

    std::unique_ptr<Preset> differed(collection.get_preset_differed_for_save(*collection_child));
    REQUIRE(differed != nullptr);
    const auto *differed_flow = differed->config.option<ConfigOptionFloatsNullable>("filament_flow_ratio");
    REQUIRE(differed_flow != nullptr);
    REQUIRE(differed_flow->values.size() == 2);
    CHECK_THAT(differed_flow->values[0], WithinAbs(0.99, 1e-9));
    CHECK(differed_flow->is_nil(1));
}

} // namespace preset_extended_contracts


namespace preset_file_type_contracts {

TEST_CASE("Preset configuration trees select their public file type", "[Preset][ConfigFileType]")
{
    struct Classification {
        std::string name;
        ConfigFileType expected;
        std::vector<std::pair<std::string, std::string>> leaves;
    };
    // Each row supplies recognizable configuration content and an explicit public
    // result. Tie rows protect precedence; isolated bundle rows test every marker.
    const std::vector<Classification> rows{
        { "app export path", CONFIG_FILE_TYPE_APP_CONFIG,
          {{"last_export_path", "/tmp/export.gcode"}} },
        { "recent settings", CONFIG_FILE_TYPE_APP_CONFIG,
          {{"recent.settings_folder", "/tmp/settings"}} },
        { "recent opened folder", CONFIG_FILE_TYPE_APP_CONFIG,
          {{"recent.last_opened_folder", "/tmp/models"}} },
        { "presets plus app content", CONFIG_FILE_TYPE_APP_CONFIG,
          {{"presets.print", "Fine"}, {"last_export_path", "/tmp/export.gcode"}} },
        { "print bundle", CONFIG_FILE_TYPE_CONFIG_BUNDLE,
          {{"print:Fine.layer_height", "0.2"}} },
        { "filament bundle", CONFIG_FILE_TYPE_CONFIG_BUNDLE,
          {{"filament:PLA.filament_type", "PLA"}} },
        { "printer bundle", CONFIG_FILE_TYPE_CONFIG_BUNDLE,
          {{"printer:X1.printer_model", "X1"}} },
        { "settings bundle", CONFIG_FILE_TYPE_CONFIG_BUNDLE,
          {{"settings.version", "1.0.0"}} },
        { "ordinary config", CONFIG_FILE_TYPE_CONFIG,
          {{"nozzle_diameter", "0.4"}, {"filament_diameter", "1.75"}} },
        { "app and bundle tie", CONFIG_FILE_TYPE_CONFIG_BUNDLE,
          {{"presets.printer", "X1"}} },
        { "app and nozzle config tie", CONFIG_FILE_TYPE_CONFIG,
          {{"last_export_path", "/tmp/export.gcode"}, {"nozzle_diameter", "0.4"}} },
        { "app and filament config tie", CONFIG_FILE_TYPE_CONFIG,
          {{"last_export_path", "/tmp/export.gcode"}, {"filament_diameter", "1.75"}} },
        { "bundle and config tie", CONFIG_FILE_TYPE_CONFIG,
          {{"print:Fine.layer_height", "0.2"}, {"nozzle_diameter", "0.4"}} },
        { "empty tree", CONFIG_FILE_TYPE_CONFIG, {} },
        { "unrelated leaf", CONFIG_FILE_TYPE_CONFIG, {{"custom_note", "ignored"}} },
        { "unrelated recent child", CONFIG_FILE_TYPE_CONFIG, {{"recent.custom_note", "ignored"}} },
    };
    const auto index = GENERATE_COPY(Catch::Generators::range(std::size_t(0), rows.size()));
    const Classification &row = rows[index];
    CAPTURE(row.name);
    boost::property_tree::ptree tree;
    for (const auto &leaf : row.leaves)
        tree.put(leaf.first, leaf.second);
    CHECK(guess_config_file_type(tree) == row.expected);
}

} // namespace preset_file_type_contracts

namespace preset_workflow_contracts {

TEST_CASE("Vendor bed texture lookup honors model ids and asset-root fallback", "[Preset][PresetBundle][PresetAssetLookup]")
{
    ScopedTemporaryDir dir("preset-texture-lookup");
    ScopedDataDir data(dir.path() / "data");
    ScopedResourcesDir resources(dir.path() / "resources");
    const fs::path local = dir.path() / "data" / "vendor" / "Acme" / "bed.png";
    const fs::path fallback = dir.path() / "resources" / "profiles" / "Acme" / "bed.png";
    write_text(local, "local texture");
    write_text(fallback, "resource texture");

    VendorProfile profile("Acme");
    profile.name = "Acme";
    VendorProfile::PrinterModel model;
    model.id = "M1";
    model.name = "Acme One";
    model.bed_texture = "bed.png";
    profile.models.push_back(model);
    PresetBundle bundle;
    bundle.vendors["Acme"] = profile;

    CHECK(bundle.get_texture_for_printer_model("Acme One") == local.string());
    CHECK(bundle.get_texture_for_printer_model("M1") == local.string());
    fs::remove(local);
    CHECK(bundle.get_texture_for_printer_model("Acme One") == fallback.string());
    CHECK(bundle.get_texture_for_printer_model("missing").empty());
}

TEST_CASE("Root filament aliases preserve a name when the vendor suffix has no prefix", "[Preset][PresetAlias]")
{
    PresetCollection collection(Preset::TYPE_FILAMENT, Preset::filament_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    DynamicPrintConfig config = collection.default_preset().config;
    auto *inherits = config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true);
    REQUIRE(inherits != nullptr);
    inherits->value.clear();

    Preset &root = collection.load_preset({}, "@Vendor", config, false);
    root.alias.clear();
    CHECK(collection.get_preset_alias(root, true) == "@Vendor");

    Preset &named = collection.load_preset({}, "PLA @Vendor", config, false);
    named.alias.clear();
    CHECK(collection.get_preset_alias(named, true) == "PLA");
}

} // namespace preset_workflow_contracts

namespace preset_external_match_contracts {

TEST_CASE("External preset matches preserve identity and selection policy", "[Preset][PresetExternalMatch]")
{
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    DynamicPrintConfig config = collection.default_preset().config;
    auto *config_layer_height = config.opt<ConfigOptionFloat>("layer_height");
    REQUIRE(config_layer_height != nullptr);
    config_layer_height->value = 0.22;
    Preset &stored = collection.load_preset({}, "Stored Process", config, false);
    const std::string default_name = collection.default_preset().name;
    ScopedTemporaryFile source(".gcode");

    REQUIRE(collection.select_preset_by_name("Stored Process", true));
    const auto retained = collection.load_external_preset(
        source.string(), "Imported", "Stored Process", config, {}, PresetCollection::LoadAndSelect::Never);
    REQUIRE(retained.first == &stored);
    CHECK_FALSE(retained.second);
    CHECK(collection.get_selected_preset_name() == "Stored Process");
    CHECK_FALSE(collection.get_edited_preset().is_dirty);

    REQUIRE(collection.select_preset(0).is_default);
    const auto retained_from_default = collection.load_external_preset(
        source.string(), "Imported", "Stored Process", config, {}, PresetCollection::LoadAndSelect::Never);
    REQUIRE(retained_from_default.first == &stored);
    CHECK_FALSE(retained_from_default.second);
    CHECK(collection.get_selected_preset_name() == default_name);
    CHECK_FALSE(collection.get_edited_preset().is_dirty);

    const auto selected = collection.load_external_preset(
        source.string(), "Imported", "Stored Process", config, {}, PresetCollection::LoadAndSelect::Always);
    REQUIRE(selected.first == &stored);
    CHECK_FALSE(selected.second);
    CHECK(collection.get_selected_preset_name() == "Stored Process");
    CHECK_FALSE(collection.get_edited_preset().is_dirty);
}

TEST_CASE("External preset resolves an unchanged inherited system profile", "[Preset][PresetExternalMatch]")
{
    ScopedTemporaryDir temp("preset-external-inherited-match");
    ScopedDataDir data(temp.path());
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    collection.update_user_presets_directory(temp.path().string(), PRESET_PRINT_NAME);

    DynamicPrintConfig parent_config = collection.default_preset().config;
    auto *parent_layer_height = parent_config.opt<ConfigOptionFloat>("layer_height");
    REQUIRE(parent_layer_height != nullptr);
    parent_layer_height->value = 0.25;
    Preset &parent = collection.load_preset({}, "System Parent", parent_config, false);
    parent.is_system = true;
    const std::string parent_name = parent.name;

    // Start from the config actually stored by load_preset.  This keeps the
    // imported profile byte-for-byte equivalent to the system parent apart from
    // bookkeeping fields, which is the identity-resolution contract under test.
    DynamicPrintConfig external = parent.config;
    auto *inherits = external.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true);
    REQUIRE(inherits != nullptr);
    inherits->value = parent_name;
    auto *external_id = external.option<ConfigOptionString>("print_settings_id", true);
    REQUIRE(external_id != nullptr);
    external_id->value = "Imported Child";
    ScopedTemporaryFile source(".gcode");
    const size_t before = collection.size();
    // Match the production PresetBundle path: its helper retains the bookkeeping
    // keys (including inherits) while listing only layer_height as a differing setting.
    const std::set<std::string> different = PresetBundle::project_different_keys("layer_height");
    const auto loaded = collection.load_external_preset(
        source.string(), "Imported", "Unknown Original", external, different,
        PresetCollection::LoadAndSelect::Always);

    REQUIRE(loaded.first != nullptr);
    // load_external_preset returns the stored profile, so request that same lookup.
    const Preset *resolved = collection.find_preset(parent_name, false, true);
    REQUIRE(resolved != nullptr);
    CHECK(loaded.first == resolved);
    CHECK_FALSE(loaded.second);
    CHECK(collection.size() == before);
    CHECK(collection.get_selected_preset_name() == parent_name);
    CHECK(collection.find_preset("Unknown Original") == nullptr);
    CHECK_THAT(loaded.first->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.25, 1e-9));
}

TEST_CASE("External preset override marks an existing profile dirty", "[Preset][PresetExternalMatch]")
{
    PresetCollection collection(Preset::TYPE_PRINT, Preset::print_options(),
                                static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
    DynamicPrintConfig stored_config = collection.default_preset().config;
    auto *stored_layer_height = stored_config.opt<ConfigOptionFloat>("layer_height");
    REQUIRE(stored_layer_height != nullptr);
    stored_layer_height->value = 0.20;
    Preset &stored = collection.load_preset({}, "Stored Process", stored_config, false);
    REQUIRE(collection.select_preset_by_name("Stored Process", true));

    DynamicPrintConfig external = collection.default_preset().config;
    auto *external_layer_height = external.opt<ConfigOptionFloat>("layer_height");
    REQUIRE(external_layer_height != nullptr);
    external_layer_height->value = 0.31;
    auto *external_id = external.option<ConfigOptionString>("print_settings_id", true);
    REQUIRE(external_id != nullptr);
    external_id->value = stored.name;
    ScopedTemporaryFile source(".gcode");
    const auto loaded = collection.load_external_preset(
        source.string(), "Imported", stored.name, external, {"layer_height"},
        PresetCollection::LoadAndSelect::Always);

    REQUIRE(loaded.first == &stored);
    REQUIRE(loaded.second);
    CHECK(collection.get_selected_preset_name() == stored.name);
    CHECK(collection.get_edited_preset().is_dirty);
    CHECK_THAT(loaded.first->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.20, 1e-9));
    CHECK_THAT(collection.get_edited_preset().config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
}

} // namespace preset_external_match_contracts

namespace preset_project_reset_followup_contracts {

size_t embedded_preset_count(PresetBundle &bundle)
{
    auto presets = bundle.get_current_project_embedded_presets();
    ScopeGuard cleanup([&presets] { for (Preset *preset : presets) delete preset; });
    return presets.size();
}

struct ProjectPresetOwners {
    std::vector<std::unique_ptr<Preset>> values;
};

void add_embedded(ProjectPresetOwners &owners, std::vector<Preset *> &project,
                  Preset::Type type, const std::string &name,
                  const DynamicPrintConfig &base_config, const std::string &parent_name)
{
    auto preset = std::make_unique<Preset>(type, name);
    preset->is_project_embedded = true;
    preset->config = base_config;
    preset->inherits() = parent_name;
    project.push_back(preset.get());
    owners.values.emplace_back(std::move(preset));
}

struct ResetNames {
    std::string printer = "Reset Base Printer";
    std::string print = "Reset Base Print";
    std::string filament = "Reset Base Filament";
    std::string embedded_printer = "Project Printer";
    std::string embedded_print = "Project Print";
    std::string embedded_filament = "Project Filament";
    std::string fallback = "Fallback PLA";
    std::string preferred = "Preferred PLA";
};

ResetNames add_base_profiles(PresetBundle &bundle, const std::string &default_filament)
{
    ResetNames names;
    auto &library = bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
    library.id = library.name = PresetBundle::ORCA_FILAMENT_LIBRARY;

    Preset &printer = add_inmemory_preset(bundle.printers, names.printer);
    printer.is_system = true;
    printer.config.option<ConfigOptionString>("default_print_profile", true)->value = names.print;
    printer.config.option<ConfigOptionStrings>("default_filament_profile", true)->values = {default_filament};

    Preset &print = add_inmemory_preset(bundle.prints, names.print);
    print.is_system = true;
    compatible_list(bundle.prints, names.print, "compatible_printers") = {names.printer};

    Preset &filament = add_inmemory_preset(bundle.filaments, names.filament);
    filament.is_system = true;
    filament.vendor = &library;
    filament.config.opt_string("filament_type", 0u) = "PLA";
    compatible_list(bundle.filaments, names.filament, "compatible_printers") = {names.printer};
    compatible_list(bundle.filaments, names.filament, "compatible_prints") = {names.print};

    Preset &fallback = add_inmemory_preset(bundle.filaments, names.fallback);
    fallback.is_system = true;
    fallback.vendor = &library;
    fallback.config.opt_string("filament_type", 0u) = "PLA";
    compatible_list(bundle.filaments, names.fallback, "compatible_printers") = {names.printer};
    compatible_list(bundle.filaments, names.fallback, "compatible_prints") = {names.print};
    return names;
}

void select_base_profiles(PresetBundle &bundle, const ResetNames &names)
{
    REQUIRE(bundle.printers.select_preset_by_name(names.printer, true));
    REQUIRE(bundle.prints.select_preset_by_name(names.print, true));
    REQUIRE(bundle.filaments.select_preset_by_name(names.filament, true));
}

} // namespace preset_project_reset_followup_contracts

TEST_CASE("PresetBundle reset restores inherited printer and compatible selections",
          "[Preset][PresetProjectReset]")
{
    ScopedTemporaryDir temp("preset-project-reset-inherited");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    using namespace preset_project_reset_followup_contracts;
    const ResetNames names = add_base_profiles(bundle, "Reset Base Filament");
    select_base_profiles(bundle, names);

    ProjectPresetOwners owners;
    std::vector<Preset *> project;
    const Preset *base_printer = bundle.printers.find_preset(names.printer, false, true);
    const Preset *base_print = bundle.prints.find_preset(names.print, false, true);
    const Preset *base_filament = bundle.filaments.find_preset(names.filament, false, true);
    REQUIRE(base_printer != nullptr);
    REQUIRE(base_print != nullptr);
    REQUIRE(base_filament != nullptr);
    add_embedded(owners, project, Preset::TYPE_PRINTER, names.embedded_printer,
                 base_printer->config, names.printer);
    add_embedded(owners, project, Preset::TYPE_PRINT, names.embedded_print,
                 base_print->config, names.print);
    add_embedded(owners, project, Preset::TYPE_FILAMENT, names.embedded_filament,
                 base_filament->config, names.filament);
    REQUIRE(bundle.load_project_embedded_presets(project, ForwardCompatibilitySubstitutionRule::Disable).empty());

    REQUIRE(bundle.printers.select_preset_by_name(names.embedded_printer, true));
    REQUIRE(bundle.prints.select_preset_by_name(names.embedded_print, true));
    REQUIRE(bundle.filaments.select_preset_by_name(names.embedded_filament, true));
    bundle.filament_presets = {names.embedded_filament};
    REQUIRE(embedded_preset_count(bundle) == 3);

    bundle.reset_project_embedded_presets();

    CHECK(embedded_preset_count(bundle) == 0);
    CHECK(bundle.printers.get_selected_preset_name() == names.printer);
    CHECK(bundle.prints.get_selected_preset_name() == names.print);
    CHECK(bundle.filaments.get_selected_preset_name() == names.filament);
    REQUIRE(bundle.filament_presets.size() == 1);
    CHECK(bundle.filament_presets.front() == names.filament);
}

TEST_CASE("PresetBundle reset updates compatible selections without an embedded printer",
          "[Preset][PresetProjectReset]")
{
    ScopedTemporaryDir temp("preset-project-reset-compatible");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    using namespace preset_project_reset_followup_contracts;
    const ResetNames names = add_base_profiles(bundle, "Reset Base Filament");
    select_base_profiles(bundle, names);

    ProjectPresetOwners owners;
    std::vector<Preset *> project;
    const Preset *base_print = bundle.prints.find_preset(names.print, false, true);
    const Preset *base_filament = bundle.filaments.find_preset(names.filament, false, true);
    REQUIRE(base_print != nullptr);
    REQUIRE(base_filament != nullptr);
    add_embedded(owners, project, Preset::TYPE_PRINT, names.embedded_print,
                 base_print->config, names.print);
    add_embedded(owners, project, Preset::TYPE_FILAMENT, names.embedded_filament,
                 base_filament->config, names.filament);
    REQUIRE(bundle.load_project_embedded_presets(project, ForwardCompatibilitySubstitutionRule::Disable).empty());

    REQUIRE(bundle.printers.get_selected_preset_name() == names.printer);
    REQUIRE(bundle.prints.select_preset_by_name(names.embedded_print, true));
    REQUIRE(bundle.filaments.select_preset_by_name(names.embedded_filament, true));
    bundle.filament_presets = {names.embedded_filament};
    REQUIRE(embedded_preset_count(bundle) == 2);

    bundle.reset_project_embedded_presets();

    CHECK(embedded_preset_count(bundle) == 0);
    CHECK(bundle.printers.get_selected_preset_name() == names.printer);
    CHECK(bundle.prints.get_selected_preset_name() == names.print);
    CHECK(bundle.filaments.get_selected_preset_name() == names.filament);
    REQUIRE(bundle.filament_presets.size() == 1);
    CHECK(bundle.filament_presets.front() == names.filament);
}

TEST_CASE("PresetBundle reset maps unavailable filament slots to visible fallbacks",
          "[Preset][PresetProjectReset]")
{
    const int fallback_mode = GENERATE(0, 1, 2);
    DYNAMIC_SECTION("fallback mode " << fallback_mode) {
        ScopedTemporaryDir temp("preset-project-reset-filament-fallback");
        ScopedDataDir data(temp.path() / "data");
        PresetBundle bundle;
        using namespace preset_project_reset_followup_contracts;
        const std::string default_filament = "Missing Preferred";
        const ResetNames names = add_base_profiles(bundle, default_filament);
        // The helper creates a fresh bundle for every generated row; configure the
        // optional preferred profile only for the visible/hidden rows.
        if (fallback_mode != 0) {
            Preset &preferred = add_inmemory_preset(bundle.filaments, names.preferred);
            preferred.is_system = true;
            preferred.vendor = &bundle.vendors[PresetBundle::ORCA_FILAMENT_LIBRARY];
            preferred.is_visible = fallback_mode == 1;
            preferred.config.opt_string("filament_type", 0u) = "PLA";
            compatible_list(bundle.filaments, names.preferred, "compatible_printers") = {names.printer};
            compatible_list(bundle.filaments, names.preferred, "compatible_prints") = {names.print};
            Preset *base_printer = bundle.printers.find_preset(names.printer, false, true);
            REQUIRE(base_printer != nullptr);
            base_printer->config.option<ConfigOptionStrings>("default_filament_profile", true)->values = {names.preferred};
        }
        select_base_profiles(bundle, names);

        ProjectPresetOwners owners;
        std::vector<Preset *> project;
        const Preset *base_printer = bundle.printers.find_preset(names.printer, false, true);
        REQUIRE(base_printer != nullptr);
        add_embedded(owners, project, Preset::TYPE_PRINTER, names.embedded_printer,
                     base_printer->config, names.printer);
        REQUIRE(bundle.load_project_embedded_presets(project, ForwardCompatibilitySubstitutionRule::Disable).empty());
        REQUIRE(bundle.printers.select_preset_by_name(names.embedded_printer, true));
        bundle.filament_presets = {"Missing Slot"};

        bundle.reset_project_embedded_presets();

        const std::string expected = fallback_mode == 1 ? names.preferred : names.fallback;
        CHECK(bundle.printers.get_selected_preset_name() == names.printer);
        REQUIRE(bundle.filament_presets.size() == 1);
        CHECK(bundle.filament_presets.front() == expected);
        const Preset *resolved = bundle.filaments.find_preset(expected, false, true);
        REQUIRE(resolved != nullptr);
        CHECK(resolved->is_visible);
        if (fallback_mode == 2) {
            const Preset *hidden = bundle.filaments.find_preset(names.preferred, false, true);
            REQUIRE(hidden != nullptr);
            CHECK_FALSE(hidden->is_visible);
        }
    }
}
namespace preset_external_save_followup_contracts {

namespace {

fs::path make_preset_external_data_dir(const fs::path &root)
{
    const fs::path path = root / "data";
    fs::create_directories(path);
    return path;
}

struct PrintCollectionFixture {
    ScopedTemporaryDir temp {"preset-external-save"};
    ScopedDataDir data;
    PresetCollection collection;

    PrintCollectionFixture()
        : data(make_preset_external_data_dir(temp.path()))
        , collection(Preset::TYPE_PRINT, Preset::print_options(),
                     static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()))
    {
        fs::create_directories(temp.path() / PRESET_PRINT_NAME / "base");
        collection.update_user_presets_directory(temp.path().string(), PRESET_PRINT_NAME);
    }

    DynamicPrintConfig config(double layer_height) const
    {
        DynamicPrintConfig config = collection.default_preset().config;
        config.option<ConfigOptionFloat>("layer_height", true)->value = layer_height;
        return config;
    }
};

} // namespace

TEST_CASE("external .3mf presets retain source identity and distinct configs",
          "[Preset][PresetExternalSave]")
{
    PrintCollectionFixture fx;
    const size_t before = fx.collection.size();
    ScopedTemporaryFile source_a(".3mf");
    ScopedTemporaryFile source_b(".3mf");

    const auto first = fx.collection.load_external_preset(
        source_a.string(), "Model-A.3mf", "Project Process", fx.config(0.27), {},
        PresetCollection::LoadAndSelect::Always);
    REQUIRE(first.first != nullptr);
    const Preset *first_ptr = first.first;
    const std::string first_name = "Project Process(Model-A.3mf)";
    const std::string second_name = "Project Process(Model-B.3mf)";

    const auto second = fx.collection.load_external_preset(
        source_b.string(), "Model-B.3mf", "Project Process", fx.config(0.31), {},
        PresetCollection::LoadAndSelect::Always);
    REQUIRE(second.first != nullptr);
    const Preset *second_ptr = second.first;
    REQUIRE(first_ptr != second_ptr);

    // Re-find both entries after the sorted deque insertion and use the live lookups for all
    // field assertions. The pointer identities also verify that insertion did not duplicate or
    // replace either source-derived preset.
    Preset *first_after = fx.collection.find_preset(first_name, false, true);
    Preset *second_after = fx.collection.find_preset(second_name, false, true);
    REQUIRE(first_after != nullptr);
    REQUIRE(second_after != nullptr);
    CHECK(first_ptr == first_after);
    CHECK(second_ptr == second_after);
    CHECK(first_after->name == first_name);
    CHECK(second_after->name == second_name);
    CHECK(first_after->is_external);
    CHECK(second_after->is_external);
    CHECK(first_after->is_project_embedded);
    CHECK(second_after->is_project_embedded);
    CHECK(first_after->file == source_a.string());
    CHECK(second_after->file == source_b.string());
    CHECK(fx.collection.size() == before + 2);
    CHECK(fx.collection.get_selected_preset_name() == second_name);
    CHECK_THAT(first_after->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.27, 1e-9));
    CHECK_THAT(second_after->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
    CHECK_FALSE(fs::exists(fx.temp.path() / PRESET_PRINT_NAME / (first_name + ".json")));
    CHECK_FALSE(fs::exists(fx.temp.path() / PRESET_PRINT_NAME / (second_name + ".json")));
}

TEST_CASE("project embedded external preset refreshes stored config without dirtying it",
          "[Preset][PresetExternalSave]")
{
    PrintCollectionFixture fx;
    Preset &stored = fx.collection.load_preset({}, "Embedded Process", fx.config(0.20), true);
    stored.is_project_embedded = true;
    fx.collection.select_preset(0);
    const std::string default_name = fx.collection.default_preset().name;

    DynamicPrintConfig external = fx.config(0.31);
    ScopedTemporaryFile source(".3mf");
    const auto loaded = fx.collection.load_external_preset(
        source.string(), "Model.3mf", stored.name, external, {"layer_height"}, PresetCollection::LoadAndSelect::Never);
    REQUIRE(loaded.first == &stored);
    CHECK_FALSE(loaded.second);
    CHECK_THAT(stored.config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-9));
    CHECK_FALSE(stored.is_dirty);
    CHECK(fx.collection.get_selected_preset_name() == default_name);
    CHECK(fx.collection.size() == 2);
}

TEST_CASE("Saving an inherited preset keeps its parent",
          "[Preset][PresetExternalSave]")
{
    PrintCollectionFixture fx;
    fx.collection.load_preset({}, "Save Parent", fx.config(0.20), true);
    fx.collection.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.28;

    fx.collection.save_current_preset("Saved Child", false, false);
    Preset *saved = fx.collection.find_preset("Saved Child", false, true);
    REQUIRE(saved != nullptr);
    CHECK(saved->inherits() == "Save Parent");
    CHECK_FALSE(saved->is_project_embedded);
    CHECK(fx.collection.get_selected_preset_name() == "Saved Child");
    REQUIRE(fs::exists(saved->file));

    DynamicPrintConfig serialized;
    std::map<std::string, std::string> key_values;
    std::string reason;
    REQUIRE_NOTHROW(serialized.load_from_json(saved->file,
                                              ForwardCompatibilitySubstitutionRule::Disable,
                                              key_values, reason));
    CHECK(reason.empty());
    CHECK_THAT(serialized.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));
}

TEST_CASE("Saving a detached preset writes a standalone base",
          "[Preset][PresetExternalSave]")
{
    PrintCollectionFixture fx;
    fx.collection.load_preset({}, "Detach Parent", fx.config(0.20), true);
    fx.collection.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.29;

    fx.collection.save_current_preset("Detached Child", true, false);
    Preset *saved = fx.collection.find_preset("Detached Child", false, true);
    REQUIRE(saved != nullptr);
    CHECK(saved->inherits().empty());
    CHECK_FALSE(saved->is_project_embedded);
    CHECK(fx.collection.get_selected_preset_name() == "Detached Child");
    REQUIRE(fs::exists(saved->file));
    CHECK(fs::equivalent(fs::path(saved->file),
                         fx.temp.path() / PRESET_PRINT_NAME / "base" / "Detached Child.json"));

    DynamicPrintConfig serialized;
    std::map<std::string, std::string> key_values;
    std::string reason;
    REQUIRE_NOTHROW(serialized.load_from_json(saved->file,
                                              ForwardCompatibilitySubstitutionRule::Disable,
                                              key_values, reason));
    CHECK(reason.empty());
    CHECK_THAT(serialized.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.29, 1e-9));
}

TEST_CASE("Saving a root preset reloads its inherited child",
          "[Preset][PresetExternalSave]")
{
    PrintCollectionFixture fx;
    const fs::path parent_file =
        fx.temp.path() / PRESET_PRINT_NAME / "base" / "Reload Parent.json";
    fx.collection.load_preset(parent_file.string(), "Reload Parent", fx.config(0.20), true);
    fx.collection.save_current_preset("Reload Child", false, false);

    Preset *child = fx.collection.find_preset("Reload Child", false, true);
    REQUIRE(child != nullptr);
    REQUIRE(child->inherits() == "Reload Parent");
    REQUIRE(fs::exists(child->file));

    REQUIRE(fx.collection.select_preset_by_name("Reload Parent", true));
    fx.collection.get_edited_preset().config.option<ConfigOptionFloat>("layer_height")->value = 0.33;
    fx.collection.save_current_preset("Reload Parent", false, false);

    Preset *reloaded = fx.collection.find_preset("Reload Child", false, true);
    REQUIRE(reloaded != nullptr);
    CHECK_THAT(reloaded->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.33, 1e-9));
    CHECK(reloaded->inherits() == "Reload Parent");
}

} // namespace preset_external_save_followup_contracts


namespace preset_parent_reset_followup_contracts {
namespace fs = boost::filesystem;

namespace {

PresetCollection make_followup_print_collection()
{
    return PresetCollection(Preset::TYPE_PRINT, Preset::print_options(),
                            static_cast<const PrintRegionConfig &>(FullPrintConfig::defaults()));
}

DynamicPrintConfig followup_print_config(const PresetCollection &collection, const std::string &inherits = {})
{
    DynamicPrintConfig config = collection.default_preset().config;
    config.option<ConfigOptionString>(BBL_JSON_KEY_INHERITS, true)->value = inherits;
    return config;
}

} // namespace

TEST_CASE("Selected preset parent resolves inherited and missing bases",
          "[Preset][PresetParent]")
{
    PresetCollection collection = make_followup_print_collection();
    Preset &parent = collection.load_preset({}, "Parent Process",
                                             followup_print_config(collection), false);
    const std::string parent_name = parent.name;
    Preset &child = collection.load_preset({}, "Child Process",
                                            followup_print_config(collection, parent_name), false);
    REQUIRE(collection.select_preset_by_name(child.name, true));

    const Preset *resolved = collection.get_selected_preset_parent();
    REQUIRE(resolved != nullptr);
    const Preset *live_parent = collection.find_preset(parent_name, false, true);
    REQUIRE(live_parent != nullptr);
    CHECK(resolved == live_parent);
    CHECK(resolved->name == parent_name);

    Preset &orphan = collection.load_preset({}, "Orphan Process",
                                             followup_print_config(collection, "Missing Parent"), false);
    REQUIRE(collection.select_preset_by_name(orphan.name, true));
    CHECK(collection.get_selected_preset_parent() == nullptr);
}

TEST_CASE("Reset removes loaded presets and optionally their files",
          "[Preset][PresetReset]")
{
    const bool delete_files = GENERATE(false, true);
    ScopedTemporaryDir temp("preset-reset-followup");
    fs::create_directories(temp.path() / "data");
    preset_bundle_workflow_contracts::ScopedDataDir data(temp.path() / "data");
    PresetCollection collection = make_followup_print_collection();
    collection.update_user_presets_directory(temp.path().string(), PRESET_PRINT_NAME);
    fs::create_directories(temp.path() / PRESET_PRINT_NAME / "base");

    DynamicPrintConfig config = followup_print_config(collection);
    Preset &user = collection.load_preset({}, "Transient Process", std::move(config), false);
    user.file = collection.path_for_preset(user);
    REQUIRE(user.save(nullptr));
    const fs::path saved_file = user.file;
    REQUIRE(fs::exists(saved_file));
    REQUIRE(collection.select_preset_by_name(user.name, true));

    collection.reset(delete_files);

    CHECK(collection.size() == collection.num_default_presets());
    CHECK(collection.get_selected_preset().is_default);
    CHECK(collection.find_preset("Transient Process", false, true) == nullptr);
    if (delete_files)
        CHECK_FALSE(fs::exists(saved_file));
    else
        CHECK(fs::exists(saved_file));
}

} // namespace preset_parent_reset_followup_contracts

namespace high_yield_preset_import_contracts {

namespace {

namespace fs = boost::filesystem;

void write_multi_filament_gcode_config(const fs::path &file)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.opt<ConfigOptionFloat>("layer_height", true)->value = 0.27;
    config.opt<ConfigOptionStrings>("filament_colour", true)->values = {"#112233", "#445566", "#778899"};
    config.opt<ConfigOptionFloats>("filament_diameter", true)->values = {1.75, 1.80, 2.00};
    config.opt<ConfigOptionStrings>("filament_type", true)->values = {"PLA", "PETG", "ABS"};
    config.opt<ConfigOptionStrings>("filament_vendor", true)->values = {"Vendor A", "Vendor B", "Vendor C"};
    config.opt<ConfigOptionStrings>("filament_settings_id", true)->values = {
        "Red Source", "Green Source", "Blue Source"
    };
    config.opt<ConfigOptionInts>("filament_self_index", true)->values = {1, 2, 3};
    config.opt<ConfigOptionStrings>("filament_extruder_variant", true)->values = {
        "Direct Drive Standard", "Direct Drive Standard", "Direct Drive Standard"
    };
    config.opt<ConfigOptionStrings>("filament_ids", true)->values = {"FIL-A", "FIL-B", "FIL-C"};

    fs::create_directories(file.parent_path());
    std::ofstream out(file.string());
    REQUIRE(out.good());
    out << "; generated by OrcaSlicer\n"
        << "; CONFIG_BLOCK_START\n";
    for (const std::string &key : config.keys())
        out << "; " << key << " = " << config.opt_serialize(key) << "\n";
    out << "; CONFIG_BLOCK_END\n"
        << "G1 X10 Y10 F1200\n";
}

} // namespace

TEST_CASE("External G-code config splits each filament slot into its own source profile",
          "[PresetBundle][PresetBundleWorkflowsExtended][MultiFilament]")
{
    ScopedTemporaryDir temp("preset-workflow-multi-filament");
    ScopedDataDir data(temp.path() / "data");
    const fs::path gcode = temp.path() / "multi.gcode";
    write_multi_filament_gcode_config(gcode);

    preset_lifecycle_contracts::ScopedPresetBundle scoped_bundle;
    PresetBundle &bundle = scoped_bundle.bundle;
    REQUIRE_NOTHROW(bundle.load_config_file(gcode.string(), ForwardCompatibilitySubstitutionRule::Disable));

    const std::vector<std::string> expected_names = {
        "Red Source(multi.gcode)", "Green Source(multi.gcode)", "Blue Source(multi.gcode)"
    };
    const std::vector<double> expected_diameters = {1.75, 1.80, 2.00};
    const std::vector<std::string> expected_types = {"PLA", "PETG", "ABS"};
    REQUIRE(bundle.filament_presets == expected_names);
    CHECK(bundle.filaments.get_selected_preset_name() == expected_names.front());

    const auto *project_colours = bundle.project_config.option<ConfigOptionStrings>("filament_colour");
    REQUIRE(project_colours != nullptr);
    CHECK(project_colours->values == std::vector<std::string>{"#112233", "#445566", "#778899"});

    for (size_t index = 0; index < expected_names.size(); ++index) {
        CAPTURE(index);
        const Preset *preset = bundle.filaments.find_preset(expected_names[index]);
        REQUIRE(preset != nullptr);
        CHECK(preset->is_external);

        const auto *diameter = preset->config.option<ConfigOptionFloats>("filament_diameter");
        REQUIRE(diameter != nullptr);
        REQUIRE(diameter->values.size() == 1);
        CHECK_THAT(diameter->values.front(), Catch::Matchers::WithinAbs(expected_diameters[index], 1e-9));

        const auto *type = preset->config.option<ConfigOptionStrings>("filament_type");
        REQUIRE(type != nullptr);
        REQUIRE(type->values.size() == 1);
        CHECK(type->values.front() == expected_types[index]);
    }
}

} // namespace high_yield_preset_import_contracts


TEST_CASE("Cloud user loading routes printer and filament profiles through typed collections",
          "[PresetCloudSelection][PresetBundle]")
{
    PresetBundle bundle;
    AppConfig app_config;
    std::map<std::string, std::map<std::string, std::string>> cloud;

    auto process = cloud_values(PRESET_IOT_PRINT_TYPE, "cloud-process-id", "account-1");
    process.emplace("layer_height", "0.26");
    cloud.emplace("Cloud Process", std::move(process));

    auto filament_root = cloud_values(PRESET_IOT_FILAMENT_TYPE, "cloud-filament-id", "account-1");
    filament_root.emplace(BBL_JSON_KEY_FILAMENT_ID, "cloud-filament-family");
    filament_root.emplace("filament_diameter", "1.75,2.85");
    cloud.emplace("Cloud Filament Root", std::move(filament_root));

    auto filament_child = cloud_values(PRESET_IOT_FILAMENT_TYPE, "cloud-filament-child-id", "account-1",
                                       "Cloud Filament Root");
    filament_child.emplace(BBL_JSON_KEY_FILAMENT_ID, "null");
    cloud.emplace("Cloud Filament Child", std::move(filament_child));

    auto printer = cloud_values(PRESET_IOT_PRINTER_TYPE, "cloud-printer-id", "account-1");
    printer.emplace("printer_model", "Cloud Model");
    printer.emplace("nozzle_diameter", "0.4,0.6");
    cloud.emplace("Cloud Printer", std::move(printer));

    // A supported type with incomplete identity metadata is ignored, as is an unknown type.
    cloud.emplace("Incomplete Cloud", std::map<std::string, std::string>{
        { BBL_JSON_KEY_TYPE, PRESET_IOT_PRINTER_TYPE },
        { BBL_JSON_KEY_VERSION, "1.0.0" },
        { BBL_JSON_KEY_SETTING_ID, "incomplete-id" },
    });
    cloud.emplace("Unsupported Cloud", cloud_values("future-profile", "future-id", "account-1"));

    const PresetsConfigSubstitutions substitutions = bundle.load_user_presets(
        app_config, cloud, ForwardCompatibilitySubstitutionRule::EnableSilent);

    const Preset *loaded_process = bundle.prints.find_preset("Cloud Process");
    const Preset *loaded_root = bundle.filaments.find_preset("Cloud Filament Root");
    const Preset *loaded_child = bundle.filaments.find_preset("Cloud Filament Child");
    const Preset *loaded_printer = bundle.printers.find_preset("Cloud Printer");
    REQUIRE(loaded_process != nullptr);
    REQUIRE(loaded_root != nullptr);
    REQUIRE(loaded_child != nullptr);
    REQUIRE(loaded_printer != nullptr);
    CHECK_THAT(loaded_process->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.26, 1e-9));
    CHECK(loaded_root->setting_id == "cloud-filament-id");
    CHECK(loaded_child->base_id == "cloud-filament-id");
    CHECK(loaded_child->filament_id == "cloud-filament-family");
    REQUIRE(loaded_child->config.option<ConfigOptionFloats>("filament_diameter") != nullptr);
    check_double_vector(loaded_child->config.option<ConfigOptionFloats>("filament_diameter")->values,
                        {1.75, 2.85});
    CHECK(loaded_printer->user_id == "account-1");
    CHECK(loaded_printer->config.opt_string("printer_model") == "Cloud Model");
    REQUIRE(loaded_printer->config.option<ConfigOptionFloats>("nozzle_diameter") != nullptr);
    check_double_vector(loaded_printer->config.option<ConfigOptionFloats>("nozzle_diameter")->values,
                        {0.4, 0.6});
    CHECK(bundle.prints.find_preset("Incomplete Cloud") == nullptr);
    CHECK(bundle.filaments.find_preset("Incomplete Cloud") == nullptr);
    CHECK(bundle.printers.find_preset("Incomplete Cloud") == nullptr);
    CHECK(bundle.prints.find_preset("Unsupported Cloud") == nullptr);
    CHECK(bundle.filaments.find_preset("Unsupported Cloud") == nullptr);
    CHECK(bundle.printers.find_preset("Unsupported Cloud") == nullptr);
    CHECK(substitutions.empty());
}

TEST_CASE("Printer snapshot restores filament colors and replaces unavailable slots",
          "[PresetCloudSelection][PresetBundle]")
{
    PresetBundle bundle;

    DynamicPrintConfig printer_config = bundle.printers.default_preset().config;
    printer_config.option<ConfigOptionFloats>("nozzle_diameter", true)->values = {0.4, 0.4, 0.4};
    bundle.printers.load_preset("", "Snapshot Printer", printer_config, false);
    bundle.prints.load_preset("", "Snapshot Process", bundle.prints.default_preset().config, false);

    DynamicPrintConfig filament_config = bundle.filaments.default_preset().config;
    filament_config.option<ConfigOptionStrings>("compatible_printers", true)->values.clear();
    bundle.filaments.load_preset("", "Visible PLA", filament_config, false);
    Preset &hidden = bundle.filaments.load_preset("", "Hidden PLA", filament_config, false);
    hidden.is_visible = false;
    bundle.filaments.set_default_suppressed(true);

    bundle.printers.select_preset_by_name("Snapshot Printer", true);
    bundle.prints.select_preset_by_name("Snapshot Process", true);
    bundle.filaments.select_preset_by_name("Visible PLA", true);

    AppConfig app_config;
    app_config.set_printer_setting("Snapshot Printer", PRESET_PRINT_NAME, "Snapshot Process");
    app_config.set_printer_setting("Snapshot Printer", PRESET_FILAMENT_NAME, "Visible PLA");
    app_config.set_printer_setting("Snapshot Printer", "filament_01", "Missing PLA");
    app_config.set_printer_setting("Snapshot Printer", "filament_02", "Hidden PLA");
    app_config.set_printer_setting("Snapshot Printer", "filament_colors", "#111111,#222222,#333333");
    app_config.set_printer_setting("Snapshot Printer", "filament_multi_colors", "#AAAAAA,#BBBBBB");
    app_config.set_printer_setting("Snapshot Printer", "filament_color_types", "2");
    // Three nozzles and three filament slots use the public normalized shapes:
    // 3*3*3 matrix entries, two vector entries per filament, and one multiplier per nozzle.
    app_config.set_printer_setting("Snapshot Printer", "flush_volumes_matrix",
                                   "10|11|12|13|14|15|16|17|18|19|20|21|22|23|24|25|26|27|28|29|30|31|32|33|34|35|36");
    app_config.set_printer_setting("Snapshot Printer", "flush_volumes_vector", "100|101|102|103|104|105");
    app_config.set_printer_setting("Snapshot Printer", "flush_multiplier", "0.5|0.75|1.0");

    bundle.update_selections(app_config);

    REQUIRE(bundle.filaments.find_preset("Visible PLA") != nullptr);
    CHECK(bundle.filaments.find_preset("Visible PLA")->is_compatible);
    CHECK(bundle.filament_presets == std::vector<std::string>{"Visible PLA", "Visible PLA", "Visible PLA"});
    auto *snapshot_colours = bundle.project_config.option<ConfigOptionStrings>("filament_colour");
    auto *snapshot_multi_colours = bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour");
    auto *snapshot_colour_types = bundle.project_config.option<ConfigOptionStrings>("filament_colour_type");
    REQUIRE(snapshot_colours != nullptr);
    REQUIRE(snapshot_multi_colours != nullptr);
    REQUIRE(snapshot_colour_types != nullptr);
    CHECK(snapshot_colours->values == std::vector<std::string>{"#111111", "#222222", "#333333"});
    CHECK(snapshot_multi_colours->values == std::vector<std::string>{"#AAAAAA", "#BBBBBB"});
    CHECK(snapshot_colour_types->values == std::vector<std::string>{"2", "1", "1"});
    auto *flush_matrix = bundle.project_config.option<ConfigOptionFloats>("flush_volumes_matrix");
    auto *flush_vector = bundle.project_config.option<ConfigOptionFloats>("flush_volumes_vector");
    auto *flush_multiplier = bundle.project_config.option<ConfigOptionFloats>("flush_multiplier");
    REQUIRE(flush_matrix != nullptr);
    REQUIRE(flush_vector != nullptr);
    REQUIRE(flush_multiplier != nullptr);
    check_double_vector(flush_matrix->values,
                        {10., 11., 12., 13., 14., 15., 16., 17., 18., 19., 20., 21., 22., 23., 24., 25., 26., 27., 28., 29., 30., 31., 32., 33., 34., 35., 36.});
    check_double_vector(flush_vector->values,
                        {100., 101., 102., 103., 104., 105.});
    check_double_vector(flush_multiplier->values,
                        {0.5, 0.75, 1.0});
}

TEST_CASE("Printer snapshot supplies defaults when optional filament arrays are absent",
          "[PresetCloudSelection][PresetBundle]")
{
    PresetBundle bundle;
    bundle.printers.load_preset("", "Fallback Printer", bundle.printers.default_preset().config, false);
    bundle.prints.load_preset("", "Fallback Process", bundle.prints.default_preset().config, false);
    bundle.filaments.load_preset("", "Fallback PLA", bundle.filaments.default_preset().config, false);
    bundle.filaments.set_default_suppressed(true);
    bundle.printers.select_preset_by_name("Fallback Printer", true);
    bundle.prints.select_preset_by_name("Fallback Process", true);
    bundle.filaments.select_preset_by_name("Fallback PLA", true);
    bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour")->values = {"stale"};
    bundle.project_config.option<ConfigOptionStrings>("filament_colour_type")->values = {"stale"};

    AppConfig app_config;
    app_config.set_printer_setting("Fallback Printer", PRESET_PRINT_NAME, "Fallback Process");
    app_config.set_printer_setting("Fallback Printer", PRESET_FILAMENT_NAME, "Fallback PLA");
    app_config.set_printer_setting("Fallback Printer", "filament_01", "Ghost PLA");
    // No filament_colors, filament_multi_colors, or filament_color_types keys: the public
    // startup path must provide its documented per-slot defaults and resolve the missing slot.

    bundle.update_selections(app_config);

    CHECK(bundle.filament_presets == std::vector<std::string>{"Fallback PLA", "Fallback PLA"});
    auto *fallback_colours = bundle.project_config.option<ConfigOptionStrings>("filament_colour");
    auto *fallback_multi_colours = bundle.project_config.option<ConfigOptionStrings>("filament_multi_colour");
    auto *fallback_colour_types = bundle.project_config.option<ConfigOptionStrings>("filament_colour_type");
    REQUIRE(fallback_colours != nullptr);
    REQUIRE(fallback_multi_colours != nullptr);
    REQUIRE(fallback_colour_types != nullptr);
    CHECK(fallback_colours->values == std::vector<std::string>{"#26A69A", "#26A69A"});
    CHECK(fallback_multi_colours->values == std::vector<std::string>{"#26A69A", "#26A69A"});
    CHECK(fallback_colour_types->values == std::vector<std::string>{"1", "1"});
}

namespace presetbundle_subscribed_multitype_contracts {

static std::map<std::string, std::map<std::string, std::string>> remote_values(bool include_printer)
{
    std::map<std::string, std::map<std::string, std::string>> values;

    auto process_base = cloud_values(PRESET_IOT_PRINT_TYPE, "remote-process-base-id", "account-multi");
    process_base.emplace("layer_height", "0.24");
    values.emplace("A Remote Process Base", std::move(process_base));

    auto process_child = cloud_values(PRESET_IOT_PRINT_TYPE, "remote-process-child-id", "account-multi",
                                      "A Remote Process Base");
    process_child.emplace("layer_height", "0.28");
    values.emplace("Z Remote Process Child", std::move(process_child));

    auto filament = cloud_values(PRESET_IOT_FILAMENT_TYPE, "remote-filament-id", "account-multi");
    filament.emplace(BBL_JSON_KEY_FILAMENT_ID, "remote-filament-family");
    filament.emplace("filament_diameter", "1.75,2.85");
    values.emplace("Remote Filament", std::move(filament));

    if (include_printer) {
        auto printer = cloud_values(PRESET_IOT_PRINTER_TYPE, "remote-printer-id", "account-multi");
        printer.emplace("printer_model", "Remote Printer Model");
        printer.emplace("nozzle_diameter", "0.4,0.6");
        values.emplace("Remote Printer", std::move(printer));
    }

    // These rows exercise the public skip behavior without pretending that unknown types are
    // supported.  The missing-type row is also absent from every typed metadata list.
    values.emplace("Missing Remote Type", std::map<std::string, std::string>{
        { BBL_JSON_KEY_VERSION, "1.0.0" },
        { BBL_JSON_KEY_SETTING_ID, "missing-type-id" },
        { BBL_JSON_KEY_USER_ID, "account-multi" },
    });
    values.emplace("Unsupported Remote Type", cloud_values("future-profile", "future-id", "account-multi"));
    return values;
}

static std::string prefix()
{
    return "_subscribed/remote-multi/";
}

} // namespace presetbundle_subscribed_multitype_contracts

TEST_CASE("Subscribed bundle reconciliation installs all supported preset types and canonical inheritance",
          "[PresetBundle][PresetBundleSubscribedMultiType]")
{
    using namespace presetbundle_subscribed_multitype_contracts;
    ScopedTemporaryDir temp("preset-subscribed-multitype");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    AppConfig app;
    BundleMetadata metadata = preset_bundle_workflow_contracts::subscribed_metadata("remote-multi");

    const auto values = remote_values(true);
    REQUIRE_NOTHROW(bundle.update_subscribed_presets(
        app, values, metadata, ForwardCompatibilitySubstitutionRule::EnableSilent));

    const std::string process_prefix = prefix();
    const Preset *process_base = bundle.prints.find_preset(process_prefix + "A Remote Process Base");
    const Preset *process_child = bundle.prints.find_preset(process_prefix + "Z Remote Process Child");
    const Preset *filament = bundle.filaments.find_preset(process_prefix + "Remote Filament");
    const Preset *printer = bundle.printers.find_preset(process_prefix + "Remote Printer");
    REQUIRE(process_base != nullptr);
    REQUIRE(process_child != nullptr);
    REQUIRE(filament != nullptr);
    REQUIRE(printer != nullptr);

    for (const Preset *preset : {process_base, process_child, filament, printer}) {
        REQUIRE(preset->is_from_bundle());
        REQUIRE(preset->bundle_id == "remote-multi");
        REQUIRE(fs::exists(fs::path(preset->file)));
    }
    CHECK(process_child->inherits() == process_prefix + "A Remote Process Base");
    CHECK(process_child->base_id == process_base->setting_id);
    CHECK_THAT(process_base->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-9));
    CHECK_THAT(process_child->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));
    CHECK(filament->filament_id == "remote-filament-family");
    REQUIRE(filament->config.option<ConfigOptionFloats>("filament_diameter") != nullptr);
    REQUIRE(filament->config.option<ConfigOptionFloats>("filament_diameter")->values.size() == 2);
    CHECK_THAT(filament->config.option<ConfigOptionFloats>("filament_diameter")->values[0],
               Catch::Matchers::WithinAbs(1.75, 1e-9));
    CHECK_THAT(filament->config.option<ConfigOptionFloats>("filament_diameter")->values[1],
               Catch::Matchers::WithinAbs(2.85, 1e-9));
    CHECK(printer->config.opt_string("printer_model") == "Remote Printer Model");
    REQUIRE(printer->config.option<ConfigOptionFloats>("nozzle_diameter") != nullptr);
    REQUIRE(printer->config.option<ConfigOptionFloats>("nozzle_diameter")->values.size() == 2);
    CHECK_THAT(printer->config.option<ConfigOptionFloats>("nozzle_diameter")->values[0],
               Catch::Matchers::WithinAbs(0.4, 1e-9));
    CHECK_THAT(printer->config.option<ConfigOptionFloats>("nozzle_diameter")->values[1],
               Catch::Matchers::WithinAbs(0.6, 1e-9));

    CHECK(bundle.prints.find_preset(process_prefix + "Missing Remote Type") == nullptr);
    CHECK(bundle.filaments.find_preset(process_prefix + "Missing Remote Type") == nullptr);
    CHECK(bundle.printers.find_preset(process_prefix + "Missing Remote Type") == nullptr);
    CHECK(bundle.prints.find_preset(process_prefix + "Unsupported Remote Type") == nullptr);
    CHECK(bundle.filaments.find_preset(process_prefix + "Unsupported Remote Type") == nullptr);
    CHECK(bundle.printers.find_preset(process_prefix + "Unsupported Remote Type") == nullptr);

    REQUIRE(bundle.bundles.m_bundles.find("remote-multi") != bundle.bundles.m_bundles.end());
    const BundleMetadata &stored = bundle.bundles.m_bundles.at("remote-multi");
    CHECK(stored.print_presets == std::vector<std::string>{
        process_prefix + "A Remote Process Base", process_prefix + "Z Remote Process Child"});
    CHECK(stored.filament_presets == std::vector<std::string>{process_prefix + "Remote Filament"});
    CHECK(stored.printer_presets == std::vector<std::string>{process_prefix + "Remote Printer"});
}

TEST_CASE("Subscribed bundle reconciliation prunes obsolete typed entries while retaining metadata",
          "[PresetBundle][PresetBundleSubscribedMultiType]")
{
    using namespace presetbundle_subscribed_multitype_contracts;
    ScopedTemporaryDir temp("preset-subscribed-prune-multitype");
    ScopedDataDir data(temp.path() / "data");
    PresetBundle bundle;
    AppConfig app;
    BundleMetadata metadata = preset_bundle_workflow_contracts::subscribed_metadata("remote-multi");

    const auto initial = remote_values(true);
    REQUIRE_NOTHROW(bundle.update_subscribed_presets(
        app, initial, metadata, ForwardCompatibilitySubstitutionRule::EnableSilent));
    const Preset *initial_printer = bundle.printers.find_preset(prefix() + "Remote Printer");
    REQUIRE(initial_printer != nullptr);
    const fs::path printer_file(initial_printer->file);
    REQUIRE(fs::exists(printer_file));

    const auto replacement = remote_values(false);
    REQUIRE_NOTHROW(bundle.update_subscribed_presets(
        app, replacement, metadata, ForwardCompatibilitySubstitutionRule::EnableSilent));

    REQUIRE(bundle.prints.find_preset(prefix() + "A Remote Process Base") != nullptr);
    const Preset *child = bundle.prints.find_preset(prefix() + "Z Remote Process Child");
    const Preset *filament = bundle.filaments.find_preset(prefix() + "Remote Filament");
    REQUIRE(child != nullptr);
    REQUIRE(filament != nullptr);
    CHECK(bundle.printers.find_preset(prefix() + "Remote Printer") == nullptr);
    CHECK_FALSE(fs::exists(printer_file));
    CHECK(child->inherits() == prefix() + "A Remote Process Base");
    CHECK_THAT(child->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.28, 1e-9));

    REQUIRE(bundle.bundles.m_bundles.find("remote-multi") != bundle.bundles.m_bundles.end());
    const BundleMetadata &stored = bundle.bundles.m_bundles.at("remote-multi");
    CHECK(stored.print_presets == std::vector<std::string>{
        prefix() + "A Remote Process Base", prefix() + "Z Remote Process Child"});
    CHECK(stored.filament_presets == std::vector<std::string>{prefix() + "Remote Filament"});
    CHECK(stored.printer_presets.empty());
}
