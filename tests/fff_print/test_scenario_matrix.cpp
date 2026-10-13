#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_message.hpp>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/GCodeProcessor.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/ModelArrange.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include "libslic3r/libslic3r.h"

#include "test_helpers.hpp"

#include <nlohmann/json.hpp>

#include <boost/filesystem.hpp>

#include <cstdlib>
#include <exception>
#include <fstream>
#include <functional>
#include <map>
#include <string>
#include <vector>

using namespace Slic3r;
using namespace Slic3r::Test;

// Slices the cases of a generated scenario matrix and writes each case's G-code and results, so the
// outputs of two builds of the slicer can be compared case by case. Each case combines model features
// (modifiers, height ranges, painting, copies, materials) that a config file can't express with config
// overrides. The cases come from tests/scenario_matrix/generate_cases.py.
//
// Hidden; run it with
//   ORCA_MATRIX_CASES=cases.jsonl ORCA_MATRIX_OUT=out/ fff_print_tests "[ScenarioMatrix]"
// It writes out/<id>.gcode for every case that slices, and one line per case to out/results.jsonl.

namespace {

using json = nlohmann::json;

TestMesh test_mesh(const std::string &name)
{
    for (const auto &[mesh_id, mesh_name] : mesh_names)
        if (name == mesh_name)
            return mesh_id;
    throw std::invalid_argument("unknown mesh " + name);
}

// A cube modifier volume over the middle third of `object` in X and Y and its full height.
void add_center_volume(ModelObject &object, ModelVolumeType type, const DynamicPrintConfig &config)
{
    const BoundingBoxf3 box  = object.raw_mesh_bounding_box();
    const Vec3d         size = box.size();
    TriangleMesh        volume = make_cube(size.x() / 3., size.y() / 3., size.z() + 2.);
    volume.translate(float(box.min.x() + size.x() / 3.), float(box.min.y() + size.y() / 3.), float(box.min.z() - 1.));
    object.add_volume(std::move(volume), type, false)->config.apply(config);
}

// Paints the facets of the object's first volume for which `state_of` returns a state other than NONE.
void paint(ModelVolume &volume, FacetsAnnotation ModelVolume::*facets,
           const std::function<EnforcerBlockerType(const Vec3f &normal, float height)> &state_of)
{
    const TriangleMesh &mesh   = volume.mesh();
    const float         bottom = mesh.bounding_box().min.z();
    TriangleSelector    selector(mesh);
    for (size_t i = 0; i < mesh.its.indices.size(); ++i) {
        const Vec3f a = mesh.its.vertices[mesh.its.indices[i][0]];
        const Vec3f b = mesh.its.vertices[mesh.its.indices[i][1]];
        const Vec3f c = mesh.its.vertices[mesh.its.indices[i][2]];
        const EnforcerBlockerType state = state_of((b - a).cross(c - a).normalized(), (a.z() + b.z() + c.z()) / 3.f - bottom);
        if (state != EnforcerBlockerType::NONE)
            selector.set_facet(int(i), state);
    }
    (volume.*facets).set(selector);
}

// Builds the case's model: its objects and the features of the last object.
void build_model(Model &model, const json &c, int filaments, double layer_height)
{
    const TestMesh     shape_id = test_mesh(c.at("mesh"));
    const std::string &objects  = c.at("objects");
    const int          copies   = c.value("instances", 1);

    const auto add = [&model, copies](TriangleMesh mesh) {
        ModelObject *object = model.add_object();
        object->name        = "object" + std::to_string(model.objects.size()) + ".stl";
        object->add_volume(std::move(mesh));
        for (int i = 0; i < copies; ++i)
            object->add_instance();
        return object;
    };
    ModelObject *first = add(mesh(shape_id));
    if (objects == "two_separate")
        add(mesh(shape_id));
    else if (objects == "two_copied")
        model.add_object(*first);
    else if (objects == "two_different")
        add(cube(20));
    else if (objects != "one")
        throw std::invalid_argument("unknown objects " + objects);

    ModelObject &last = *model.objects.back();
    ModelVolume &part = *last.volumes.front();
    DynamicPrintConfig dense;
    dense.set_deserialize_strict({{"sparse_infill_density", "70%"}, {"wall_loops", 4}, {"top_shell_layers", 6}});

    const std::string modifier = c.value("modifier", "none");
    if (modifier == "dense")
        add_center_volume(last, ModelVolumeType::PARAMETER_MODIFIER, dense);
    else if (modifier != "none")
        throw std::invalid_argument("unknown modifier " + modifier);

    const std::string negative = c.value("negative_volume", "none");
    if (negative == "through")
        add_center_volume(last, ModelVolumeType::NEGATIVE_VOLUME, {});
    else if (negative != "none")
        throw std::invalid_argument("unknown negative_volume " + negative);

    const std::string support_volume = c.value("support_volume", "none");
    if (support_volume == "enforcer")
        add_center_volume(last, ModelVolumeType::SUPPORT_ENFORCER, {});
    else if (support_volume == "blocker")
        add_center_volume(last, ModelVolumeType::SUPPORT_BLOCKER, {});
    else if (support_volume != "none")
        throw std::invalid_argument("unknown support_volume " + support_volume);

    const std::string range = c.value("height_range", "none");
    if (range != "none") {
        const double height = last.raw_mesh_bounding_box().size().z();
        // The GUI always gives a height range its own layer height.
        ModelConfig range_config;
        range_config.set("layer_height", layer_height);
        if (range == "dense")
            range_config.apply(dense);
        else if (range == "fine_layers")
            range_config.set("layer_height", 0.1);
        else
            throw std::invalid_argument("unknown height_range " + range);
        last.layer_config_ranges[{height * 0.3, height * 0.6}] = range_config;
    }

    const std::string material = c.value("material", "none");
    if (material == "dense") {
        model.add_material("dense")->config.apply(dense);
        part.set_material_id("dense");
    } else if (material != "none")
        throw std::invalid_argument("unknown material " + material);

    const std::string paint_mm = c.value("paint_mm", "none");
    if (paint_mm == "top" && filaments > 1)
        paint(part, &ModelVolume::mmu_segmentation_facets, [](const Vec3f &normal, float) {
            return normal.z() > 0.9f ? EnforcerBlockerType::Extruder2 : EnforcerBlockerType::NONE;
        });
    else if (paint_mm == "bands" && filaments > 1)
        // 2 mm bands of every filament up the sides.
        paint(part, &ModelVolume::mmu_segmentation_facets, [filaments](const Vec3f &normal, float height) {
            return std::abs(normal.z()) < 0.5f ?
                EnforcerBlockerType(int(EnforcerBlockerType::Extruder1) + int(height / 2.f) % filaments) :
                EnforcerBlockerType::NONE;
        });
    else if (paint_mm != "none" && filaments > 1)
        throw std::invalid_argument("unknown paint_mm " + paint_mm);

    const std::string paint_seam = c.value("paint_seam", "none");
    if (paint_seam == "enforce_side")
        paint(part, &ModelVolume::seam_facets, [](const Vec3f &normal, float) {
            return normal.x() > 0.9f ? EnforcerBlockerType::ENFORCER : EnforcerBlockerType::NONE;
        });
    else if (paint_seam == "block_side")
        paint(part, &ModelVolume::seam_facets, [](const Vec3f &normal, float) {
            return normal.y() < -0.9f ? EnforcerBlockerType::BLOCKER : EnforcerBlockerType::NONE;
        });
    else if (paint_seam != "none")
        throw std::invalid_argument("unknown paint_seam " + paint_seam);

    const std::string paint_support = c.value("paint_support", "none");
    if (paint_support == "enforce_down")
        paint(part, &ModelVolume::supported_facets, [](const Vec3f &normal, float) {
            return normal.z() < -0.5f ? EnforcerBlockerType::ENFORCER : EnforcerBlockerType::NONE;
        });
    else if (paint_support != "none")
        throw std::invalid_argument("unknown paint_support " + paint_support);

    if (c.value("object_extruders", "same") == "alternate")
        for (size_t i = 0; i < model.objects.size(); ++i)
            model.objects[i]->config.set("extruder", int(i % size_t(filaments)) + 1);

    const std::string object_settings = c.value("object_settings", "none");
    if (object_settings == "dense")
        last.config.apply(dense);
    else if (object_settings != "none")
        throw std::invalid_argument("unknown object_settings " + object_settings);
}

// The print config of the case: defaults, the filament count, then the case's overrides. A single
// value given for a per-filament option applies to every filament.
DynamicPrintConfig build_config(const json &c, int filaments)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    if (filaments > 1)
        config.apply(multifilament_config(unsigned(filaments)));
    const json overrides = c.value("config", json::object());
    for (const auto &[key, value] : overrides.items()) {
        std::string serialized = value.get<std::string>();
        const ConfigOptionDef *def = print_config_def.get(key);
        if (def != nullptr && (def->type & coVectorType) && def->type != coStrings && serialized.find(',') == std::string::npos)
            for (int i = 1; i < filaments; ++i)
                serialized += "," + value.get<std::string>();
        config.set_deserialize_strict(key, serialized);
    }
    config.set_key_value("gcode_comments", new ConfigOptionBool(true));
    // Print::validate() rejects relative extrusion without an extruder reset at each layer.
    config.set_key_value("layer_change_gcode", new ConfigOptionString("G92 E0\n"));
    // The default prime tower position (y 220) is off the 200x200 test bed.
    config.set_deserialize_strict({{"wipe_tower_x", "20"}, {"wipe_tower_y", "20"}});
    return config;
}

json slice_case(const json &c, const boost::filesystem::path &out_dir)
{
    json result = {{"id", c.at("id")}};
    try {
        const int          filaments = c.value("filaments", 1);
        DynamicPrintConfig config    = build_config(c, filaments);
        Model              model;
        build_model(model, c, filaments, config.opt_float("layer_height"));
        arrange_objects(model, InfiniteBed{}, ArrangeParams{scaled(min_object_distance(config))});
        Print print;
        for (ModelObject *object : model.objects) {
            object->ensure_on_bed();
            print.auto_assign_extruders(object);
        }
        print.apply(model, config);
        std::vector<StringObjectException> warnings;
        if (const StringObjectException error = print.validate(&warnings); !error.string.empty()) {
            result["status"]  = "invalid";
            result["message"] = error.string;
            return result;
        }
        GCodeProcessorResult processed;
        const std::string    gcode = Test::gcode(print, &processed);
        std::ofstream((out_dir / (c.at("id").get<std::string>() + ".gcode")).string(), std::ios::binary) << gcode;
        result["status"]           = "ok";
        result["warnings"]         = warnings.size();
        result["conflict"]         = bool(print.get_conflict_result());
        result["toolpath_outside"] = processed.toolpath_outside;
        result["gcode_check"]      = processed.gcode_check_result.error_code;
    } catch (const std::exception &e) {
        result["status"]  = "error";
        result["message"] = e.what();
    }
    return result;
}

} // namespace

TEST_CASE("Scenario matrix cases slice and write their G-code", "[ScenarioMatrix][.]")
{
    const char *cases_path = std::getenv("ORCA_MATRIX_CASES");
    const char *out_path   = std::getenv("ORCA_MATRIX_OUT");
    if (cases_path == nullptr || out_path == nullptr)
        SKIP("set ORCA_MATRIX_CASES and ORCA_MATRIX_OUT");
    const boost::filesystem::path out_dir(out_path);
    boost::filesystem::create_directories(out_dir);
    const char *only = std::getenv("ORCA_MATRIX_ONLY"); // optional: slice only the case with this id

    std::ifstream cases(cases_path);
    REQUIRE(cases.good());
    std::ofstream results((out_dir / "results.jsonl").string(), std::ios::app);
    std::string   line;
    size_t        count = 0;
    while (std::getline(cases, line)) {
        if (line.empty())
            continue;
        const json c = json::parse(line);
        if (only != nullptr && c.at("id") != only)
            continue;
        results << slice_case(c, out_dir).dump() << '\n' << std::flush;
        ++count;
    }
    CHECK(count > 0);
}
