#include "libslic3r/SLA/Hollowing.hpp"
#include "libslic3r/SLA/SupportPoint.hpp"

#include <initializer_list>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdlib>
#include <mutex>

#include <array>
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <map>
#include <set>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Model.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/libslic3r.h"
#include "libslic3r/ObjectID.hpp"
#include "libslic3r/TextConfiguration.hpp"
#include "libslic3r/EmbossShape.hpp"
#include "libslic3r/Format/3mf.hpp"
#include "libslic3r/Format/bbs_3mf.hpp"
#include "libslic3r/Format/STL.hpp"
#include "libslic3r/miniz_extension.hpp"
#include "libslic3r/Zipper.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/Semver.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/MultiNozzleUtils.hpp"
#include "libslic3r/ProjectTask.hpp"
#include "libslic3r/PublishSettings.hpp"
#include "libslic3r/CustomGCode.hpp"

#include "test_utils.hpp"

#include "libslic3r/Config.hpp"
#include "libslic3r/Geometry.hpp"
#include <cstddef>
#include "libslic3r/Point.hpp"
#include "libslic3r/Utils.hpp"
#include <miniz.h>
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/TriangleSelector.hpp"
#include <memory>
#include <cstdint>
#include <ios>
#include <nlohmann/json.hpp>

#include <boost/filesystem/operations.hpp>
#include <boost/nowide/fstream.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <algorithm>
#include <functional>

#include <catch2/catch_tostring.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>
#include <sstream>
#include <type_traits> // for std::enable_if_t
#include <typeinfo>    // for typeid
#include <regex>
#include <vector>
#include <utility>

namespace Catch {
    template <typename T>
    struct is_eigen_matrix : std::is_base_of<Eigen::MatrixBase<T>, T> {};

    template <typename T>
    struct StringMaker<T, std::enable_if_t<is_eigen_matrix<T>::value>> {
        static std::string convert(const T& eigen_obj) {
            // Newline at end of rows
            Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss;
            ss << "Matrix<" << typeid(eigen_obj).name() << "> = \n";
            ss << eigen_obj.format(fmt);
            return ss.str();
        }
    };
    
    // We must manually specialize for Eigen::Transform as it doesn't derive from MatrixBase.
    // It's defined as: Eigen::Transform<Scalar, Dim, Mode, Options>
    template <typename Scalar, int Dim, int Mode, int Options>
    struct StringMaker<Eigen::Transform<Scalar, Dim, Mode, Options>> {
        static std::string convert(const Eigen::Transform<Scalar, Dim, Mode, Options>& trafo) {
            // We print the underlying matrix 
            const auto& matrix = trafo.matrix();

            // Newline at end of rows
            Eigen::IOFormat fmt(4, 0, ", ", "\n", "[", "]");
            std::stringstream ss;
            
            ss << "Transform<Mode=" << Mode << ", Dim=" << Dim << "> = \n"; 
            ss << matrix.format(fmt);
            return ss.str();
        }
    };
    
    // Quaternions also need an explicit specialization
    template <typename Scalar, int Options>
    struct StringMaker<Eigen::Quaternion<Scalar, Options>> {
        static std::string convert(const Eigen::Quaternion<Scalar, Options>& quat) {
            std::stringstream ss;
            ss << "Quaternion(w=" << quat.w() << ", x=" << quat.x() << ", y=" << quat.y() << ", z=" << quat.z() << ")";
            return ss.str();
        }
    };
} // end namespace Catch

#include <catch2/catch_all.hpp>
#include "libslic3r/BoundingBox.hpp"

using namespace Slic3r;


SCENARIO("Reading 3mf file", "[3mf]") {
    GIVEN("umlauts in the path of the file") {
        Model model;
        WHEN("3mf model is read") {
            std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/Geräte/Büchse.3mf";
            DynamicPrintConfig config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
            bool ret = load_3mf(path.c_str(), config, ctxt, &model, false);
            THEN("load should succeed") {
                REQUIRE(ret);
            }
        }
    }
}

SCENARIO("Export+Import geometry to/from 3mf file cycle", "[3mf]") {
    GIVEN("world vertices coordinates before save") {
        // load a model from stl file
        Model src_model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        load_stl(src_file.c_str(), &src_model);
        src_model.add_default_instances();

        ModelObject* src_object = src_model.objects.front();

        // apply generic transformation to the 1st volume
        Geometry::Transformation src_volume_transform;
        src_volume_transform.set_offset({ 10.0, 20.0, 0.0 });
        src_volume_transform.set_rotation({ Geometry::deg2rad(25.0), Geometry::deg2rad(35.0), Geometry::deg2rad(45.0) });
        src_volume_transform.set_scaling_factor({ 1.1, 1.2, 1.3 });
        src_volume_transform.set_mirror({ -1.0, 1.0, -1.0 });
        src_object->volumes.front()->set_transformation(src_volume_transform);

        // apply generic transformation to the 1st instance
        Geometry::Transformation src_instance_transform;
        src_instance_transform.set_offset({ 5.0, 10.0, 0.0 });
        src_instance_transform.set_rotation({ Geometry::deg2rad(12.0), Geometry::deg2rad(13.0), Geometry::deg2rad(14.0) });
        src_instance_transform.set_scaling_factor({ 0.9, 0.8, 0.7 });
        src_instance_transform.set_mirror({ 1.0, -1.0, -1.0 });
        src_object->instances.front()->set_transformation(src_instance_transform);

        WHEN("model is saved+loaded to/from 3mf file") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();
            store_3mf(test_file.c_str(), &src_model, nullptr, false);

            // load back the model from the 3mf file
            Model dst_model;
            DynamicPrintConfig dst_config;
            {
                ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Disable };
                load_3mf(test_file.c_str(), dst_config, ctxt, &dst_model, false);
            }

            // compare meshes
            TriangleMesh src_mesh = src_model.mesh();
            TriangleMesh dst_mesh = dst_model.mesh();

            bool res = src_mesh.its.vertices.size() == dst_mesh.its.vertices.size();
            if (res) {
                for (size_t i = 0; i < dst_mesh.its.vertices.size(); ++i) {
                    res &= dst_mesh.its.vertices[i].isApprox(src_mesh.its.vertices[i]);
                }
            }
            THEN("world vertices coordinates after load match") {
                REQUIRE(res);
            }
        }
    }
}

SCENARIO("Mirrored instance transforms survive 3mf round trips", "[3mf][Regression]") {
    const bool bbs_format = GENERATE(false, true);

    GIVEN("an instance with a valid mirrored transform") {
        Model model;
        const std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        Transform3d mirrored = Transform3d::Identity();
        mirrored.linear() <<
             4.4408921e-16,  0.819152044,  0.573576436,
             1.0,           -4.4408921e-16, -1.11022302e-16,
            -5.55111512e-17, -0.573576436,  0.819152044;
        mirrored.translation() = Vec3d(700.41477, -169.930939, 63.392571);
        REQUIRE_THAT(mirrored.linear().determinant(), Catch::Matchers::WithinAbs(-1.0, 1e-8));
        model.objects.front()->instances.front()->set_transformation(Geometry::Transformation(mirrored));
        const TriangleMesh expected_mesh = model.mesh();

        WHEN("the model is stored and loaded") {
            ScopedTemporaryDir backup_dir("orca_mirrored_transform");
            model.set_backup_path(backup_dir.string());
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            if (bbs_format) {
                StoreParams store_params;
                store_params.path     = test_file.c_str();
                store_params.model    = &model;
                store_params.config   = &config;
                store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
                REQUIRE(store_bbs_3mf(store_params));
            } else {
                REQUIRE(store_3mf(test_file.c_str(), &model, &config, false));
            }

            Model dst_model;
            ScopedTemporaryDir loaded_backup_dir("orca_mirrored_transform_loaded");
            dst_model.set_backup_path(loaded_backup_dir.string());
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs dst_plates;
            std::vector<Preset*> project_presets;
            ScopeGuard cleanup([&dst_plates, &project_presets]() {
                release_PlateData_list(dst_plates);
                for (Preset *preset : project_presets)
                    delete preset;
            });
            if (bbs_format) {
                bool is_bbl_3mf = false, is_orca_3mf = false;
                Semver file_version;
                REQUIRE(load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                     &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
            } else {
                REQUIRE(load_3mf(test_file.c_str(), dst_config, ctxt, &dst_model, false));
            }

            THEN("the mirrored transform and world geometry are preserved") {
                REQUIRE(dst_model.objects.size() == 1);
                REQUIRE(dst_model.objects.front()->instances.size() == 1);
                const Transform3d &loaded = dst_model.objects.front()->instances.front()->get_matrix();
                REQUIRE(loaded.linear().determinant() < 0.0);

                const TriangleMesh loaded_mesh = dst_model.mesh();
                REQUIRE(loaded_mesh.its.vertices.size() == expected_mesh.its.vertices.size());
                for (size_t i = 0; i < loaded_mesh.its.vertices.size(); ++i)
                    REQUIRE(loaded_mesh.its.vertices[i].isApprox(expected_mesh.its.vertices[i], 1e-5f));
            }
        }
    }
}

// The recipe is an opaque binary blob (CadDocument::serialize_recipe()), so the 3mf backend has
// to carry it byte-for-byte — no XML/text mangling, embedded NULs intact.
static std::string make_cad_recipe()
{
    // Built from an explicit length, not append(const char*), which would stop at the first
    // embedded NUL — the one thing this blob exists to prove survives the archive.
    static const char blob[] = "\x01" "RECIPE" "\0" "\xff\xfe\x00\x10" "cad-features-blob";
    return std::string(blob, sizeof(blob) - 1);
}

static const std::string CAD_RECIPE_ENTRY        = "Metadata/orca_cad.bin";
static const std::string LEGACY_CAD_RECIPE_ENTRY = "Metadata/SnapOrca_cad.bin";

// Pulls one named entry out of a 3mf archive; false when it is absent.
static bool read_3mf_entry(const std::string& path, std::string& out,
                                  const std::string& entry = CAD_RECIPE_ENTRY)
{
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    REQUIRE(open_zip_reader(&zip, path));
    const ScopeGuard close_zip([&zip] { close_zip_reader(&zip); });
    bool   found = false;
    mz_uint n    = mz_zip_reader_get_num_files(&zip);
    for (mz_uint i = 0; i < n; ++i) {
        mz_zip_archive_file_stat st;
        if (!mz_zip_reader_file_stat(&zip, i, &st)) continue;
        std::string name(st.m_filename);
        std::replace(name.begin(), name.end(), '\\', '/');
        if (boost::algorithm::iequals(name, entry)) {
            out.resize(st.m_uncomp_size);
            found = out.empty() || mz_zip_reader_extract_to_mem(&zip, i, out.data(), out.size(), 0) != 0;
            break;
        }
    }
    return found;
}

// Rewrites the archive at `path`, letting `edit` change the name or data of each entry; returns
// whether `edit` reported a change for any of them. miniz cannot edit in place and
// open_zip_writer truncates, so the entries are held across the switch.
static bool rewrite_3mf_entries(const std::string& path, const std::function<bool(std::string& name, std::string& data)>& edit,
                                const std::vector<std::pair<std::string, std::string>>& append = {})
{
    std::vector<std::pair<std::string, std::string>> entries;
    bool changed = false;
    {
        mz_zip_archive zip;
        mz_zip_zero_struct(&zip);
        REQUIRE(open_zip_reader(&zip, path));
        const ScopeGuard close_zip([&zip] { close_zip_reader(&zip); });
        mz_uint n = mz_zip_reader_get_num_files(&zip);
        for (mz_uint i = 0; i < n; ++i) {
            mz_zip_archive_file_stat st;
            REQUIRE(mz_zip_reader_file_stat(&zip, i, &st));
            if (st.m_is_directory) continue;
            std::string name(st.m_filename);
            std::replace(name.begin(), name.end(), '\\', '/');
            std::string data((size_t) st.m_uncomp_size, '\0');
            if (st.m_uncomp_size > 0)
                REQUIRE(mz_zip_reader_extract_to_mem(&zip, i, data.data(), data.size(), 0));
            changed |= edit(name, data);
            entries.emplace_back(std::move(name), std::move(data));
        }
    }

    entries.insert(entries.end(), append.begin(), append.end());
    Zipper out(path);
    for (const auto& e : entries)
        out.add_entry(e.first, e.second.data(), e.second.size());
    out.finalize();
    return changed || !append.empty();
}

// Rewrites the archive at `path` with the recipe entry back under the name it had before the
// rename, which is what every project saved by an earlier build looks like on disk. Generated
// rather than checked in because a whole project archive is not frozen evidence the way a bare
// recipe blob is -- it has to be whatever today's exporter writes, with only the name aged.
static void rename_cad_recipe_entry_to_legacy(const std::string& path)
{
    const bool renamed = rewrite_3mf_entries(path, [](std::string& name, std::string&) {
        if (!boost::algorithm::iequals(name, CAD_RECIPE_ENTRY))
            return false;
        name = LEGACY_CAD_RECIPE_ENTRY;
        return true;
    });
    // Without this the scenario would degrade silently into re-testing the new name if the
    // exporter's constant ever moved again: every load below would still pass.
    REQUIRE(renamed);
}

// Replaces the first occurrence of `from` in any entry whose name ends with `suffix`.
static bool replace_in_3mf_entry(const std::string& path, const std::string& suffix, const std::string& from, const std::string& to)
{
    bool replaced = false;
    rewrite_3mf_entries(path, [&](std::string& name, std::string& data) {
        if (replaced || !boost::algorithm::ends_with(name, suffix))
            return false;
        if (const size_t pos = data.find(from); pos != std::string::npos) {
            data.replace(pos, from.size(), to);
            replaced = true;
        }
        return replaced;
    });
    return replaced;
}

// Stores a one-plate project holding a cube whose first two facets are painted Extruder2 and
// Extruder3, which the exporter writes as paint_color="8" and paint_color="0C".
static void store_painted_cube(const std::string& path)
{
    Model        model;
    ModelObject* object = model.add_object();
    ModelVolume* volume = object->add_volume(make_cube(10., 10., 10.));
    object->add_instance();
    {
        TriangleSelector selector(volume->mesh());
        selector.set_facet(0, EnforcerBlockerType::Extruder2);
        selector.set_facet(1, EnforcerBlockerType::Extruder3);
        REQUIRE(volume->mmu_segmentation_facets.set(selector));
    }
    ScopedTemporaryDir backup_dir("orca_paint_src");
    model.set_backup_path(backup_dir.string());

    DynamicPrintConfig cfg;
    PlateData          plate;
    plate.plate_index = 0;
    StoreParams sp;
    sp.path     = path.c_str();
    sp.model    = &model;
    sp.config   = &cfg;
    sp.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    sp.plate_data_list.push_back(&plate);
    REQUIRE(store_bbs_3mf(sp));
}

// Loads `path` through the BBS importer into `model`, releasing the plates it returns. The
// importer stages metadata through `backup_dir`, which has to outlive the model.
static bool load_project(const std::string& path, Model& model, const ScopedTemporaryDir& backup_dir)
{
    model.set_backup_path(backup_dir.string());
    DynamicPrintConfig        config;
    ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
    PlateDataPtrs             plates;
    std::vector<Preset*>      project_presets;
    bool   is_bbl_3mf = false, is_orca_3mf = false;
    Semver file_version;
    const bool loaded = load_bbs_3mf(path.c_str(), &config, &ctxt, &model, &plates, &project_presets, &is_bbl_3mf,
                                     &is_orca_3mf, &file_version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    release_PlateData_list(plates);
    return loaded;
}

TEST_CASE("A project with a plate id below 1 fails to load", "[3mf][Regression]")
{
    const int plate_id = GENERATE(0, -1);
    INFO("plater_id " << plate_id);

    ScopedTemporaryFile temp(".3mf");
    store_painted_cube(temp.string());
    {
        ScopedTemporaryDir backup_dir("orca_plate_dst");
        Model              model;
        REQUIRE(load_project(temp.string(), model, backup_dir));
    }

    REQUIRE(replace_in_3mf_entry(temp.string(), "model_settings.config", "key=\"plater_id\" value=\"1\"",
                                 "key=\"plater_id\" value=\"" + std::to_string(plate_id) + "\""));
    ScopedTemporaryDir backup_dir("orca_plate_dst");
    Model              model;
    bool               loaded = true;
    REQUIRE_NOTHROW(loaded = load_project(temp.string(), model, backup_dir));
    REQUIRE_FALSE(loaded);
}

TEST_CASE("A project whose components reference themselves fails to load", "[3mf][Regression]")
{
    // One self-reference keeps the expansion going without ever reaching a mesh. A thousand also make
    // each expansion queue a thousand more, so the bound has to hold the work list, not just the loop.
    const int references = GENERATE(1, 1000);
    INFO("self-references " << references);

    ScopedTemporaryFile temp(".3mf");
    store_painted_cube(temp.string());

    // Point the component back at the object that holds it, repeated `references` times.
    REQUIRE(rewrite_3mf_entries(temp.string(), [references](std::string& name, std::string& data) {
        if (!boost::algorithm::ends_with(name, "3dmodel.model"))
            return false;
        std::smatch match;
        if (!std::regex_search(data, match, std::regex("<object id=\"([0-9]+)\"[^>]*>\\s*<components")))
            return false;
        data = std::regex_replace(data, std::regex("objectid=\"[0-9]+\""), "objectid=\"" + match[1].str() + "\"");
        std::smatch component;
        if (!std::regex_search(data, component, std::regex("<component [^>]*/>")))
            return false;
        std::string repeated;
        for (int i = 0; i < references; ++i)
            repeated += component.str();
        data.replace(component.position(), component.length(), repeated);
        return true;
    }));

    ScopedTemporaryDir backup_dir("orca_cycle_dst");
    Model              model;
    bool               loaded = true;
    REQUIRE_NOTHROW(loaded = load_project(temp.string(), model, backup_dir));
    REQUIRE_FALSE(loaded);
}

TEST_CASE("An object loads up to the component reference budget and fails past it", "[3mf][Regression]")
{
    // The importer queues at most 100000 component references per object. Every reference besides the
    // cube's own points at an object the file does not define: it counts toward the budget, then expands
    // to nothing, so the object stays a single part whatever the count.
    const auto [references, loads] = GENERATE(table<int, bool>({ { 100000, true }, { 100001, false } }));
    INFO("component references " << references);

    ScopedTemporaryFile temp(".3mf");
    store_painted_cube(temp.string());

    std::string dangling;
    for (int i = 1; i < references; ++i)
        dangling += "<component objectid=\"999999\"/>";
    REQUIRE(replace_in_3mf_entry(temp.string(), "3dmodel.model", "</components>", dangling + "</components>"));

    ScopedTemporaryDir backup_dir("orca_budget_dst");
    Model              model;
    bool               loaded = !loads;
    REQUIRE_NOTHROW(loaded = load_project(temp.string(), model, backup_dir));
    REQUIRE(loaded == loads);
    if (loads) {
        REQUIRE(model.objects.size() == 1);
        CHECK(model.objects.front()->volumes.size() == 1);
    }
}

TEST_CASE("A project with malformed paint data loads without the damaged facet", "[3mf][Regression]")
{
    ScopedTemporaryFile temp(".3mf");
    store_painted_cube(temp.string());
    // Split codes with no children behind them: the stream runs out mid-tree.
    REQUIRE(replace_in_3mf_entry(temp.string(), ".model", "paint_color=\"8\"", "paint_color=\"FFFFFFFFFFFFFFFF3\""));

    ScopedTemporaryDir backup_dir("orca_paint_dst");
    Model              model;
    REQUIRE(load_project(temp.string(), model, backup_dir));
    REQUIRE(model.objects.size() == 1);
    const ModelVolume& volume = *model.objects.front()->volumes.front();
    const auto&        data   = volume.mmu_segmentation_facets.get_data();
    REQUIRE_FALSE(data.used_states[size_t(EnforcerBlockerType::Extruder2)]);
    REQUIRE(data.used_states[size_t(EnforcerBlockerType::Extruder3)]);

    TriangleSelector selector(volume.mesh());
    REQUIRE_NOTHROW(selector.deserialize(data));
    REQUIRE(selector.num_facets(EnforcerBlockerType::Extruder2) == 0);
    REQUIRE(selector.num_facets(EnforcerBlockerType::Extruder3) == 1);
}

// The recipe lives only in the BBS-native backend, because that is the only one that runs:
// store_bbs_3mf is the sole exporter the app calls, and 3mf.cpp's load_3mf is reached only for
// files fingerprinted as PrusaSlicer's, which never carry a recipe. This locks in both halves:
// the archive entry is at the exact path the importer looks for, and the recipe comes back
// through the real importer.
SCENARIO("CAD recipe is embedded in the BBS 3mf archive", "[3mf][CAD]") {
    GIVEN("a model carrying a binary cad_recipe") {
        Model model;
        std::string src = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src.c_str(), &model));
        model.add_default_instances();

        // store_bbs_3mf stages its metadata through the model's backup path; point it at a
        // writable temp dir, as the sibling BBS scenarios do. The process-global
        // set_temporary_dir() would leak into every test that ran afterwards.
        ScopedTemporaryDir backup_dir("orca_cad");
        model.set_backup_path(backup_dir.string());

        const std::string recipe = make_cad_recipe();
        model.cad_recipe = recipe;

        WHEN("saved through the BBS backend (the format the GUI uses)") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig cfg;
            StoreParams sp;
            sp.path     = test_file.c_str();
            sp.model    = &model;
            sp.config   = &cfg;
            sp.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(sp));

            THEN("the archive entry is present byte-for-byte") {
                std::string got;
                REQUIRE(read_3mf_entry(test_file, got));
                REQUIRE(got.size() == recipe.size());
                REQUIRE(got == recipe);
            }

            THEN("the importer restores it onto the loaded model") {
                Model dst_model;
                ScopedTemporaryDir dst_backup_dir("orca_cad_dst");
                dst_model.set_backup_path(dst_backup_dir.string());

                DynamicPrintConfig dst_config;
                ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
                PlateDataPtrs        dst_plates;
                std::vector<Preset*> project_presets;
                bool   is_bbl_3mf = false, is_orca_3mf = false;
                Semver file_version;
                REQUIRE(load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                     &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
                REQUIRE(dst_model.cad_recipe.size() == recipe.size());
                REQUIRE(dst_model.cad_recipe == recipe);

                release_PlateData_list(dst_plates);
            }
        }

        WHEN("the same model is saved with no recipe") {
            model.cad_recipe.clear();
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig cfg;
            StoreParams sp;
            sp.path     = test_file.c_str();
            sp.model    = &model;
            sp.config   = &cfg;
            sp.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(sp));

            THEN("no entry is written at all") {
                std::string got;
                REQUIRE_FALSE(read_3mf_entry(test_file, got));
            }
        }
    }
}

// The recipe entry was renamed from Metadata/SnapOrca_cad.bin to Metadata/orca_cad.bin. Nothing
// in the blob marks that move, so a reader that knows only the new name loads a project written
// before it with an empty cad_recipe and no error at all — a feature tree gone with no symptom
// but an empty Design tab. The importer must still accept the old name; the exporter may never
// write it.
SCENARIO("a project saved under the pre-rename recipe name still loads", "[3mf][CAD]") {
    GIVEN("a project whose recipe entry carries the old name") {
        Model model;
        std::string src = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src.c_str(), &model));
        model.add_default_instances();
        const std::string recipe = make_cad_recipe();
        model.cad_recipe = recipe;

        WHEN("it was written by the BBS backend") {
            ScopedTemporaryDir backup_dir("orca_cad_legacy");
            model.set_backup_path(backup_dir.string());

            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();
            DynamicPrintConfig cfg;
            StoreParams sp;
            sp.path     = test_file.c_str();
            sp.model    = &model;
            sp.config   = &cfg;
            sp.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(sp));
            rename_cad_recipe_entry_to_legacy(test_file);

            // Catch2 replays the enclosing sections per THEN, so one load here serves both.
            Model dst_model;
            ScopedTemporaryDir dst_backup_dir("orca_cad_legacy_dst");
            dst_model.set_backup_path(dst_backup_dir.string());

            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            REQUIRE(load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                 &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                 LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
            release_PlateData_list(dst_plates);

            THEN("the recipe still comes back byte-for-byte") {
                REQUIRE(dst_model.cad_recipe == recipe);
            }

            THEN("re-saving migrates it to the new name and leaves the old one behind") {
                ScopedTemporaryFile again(".3mf");
                const std::string resaved = again.string();
                DynamicPrintConfig cfg2;
                StoreParams sp2;
                sp2.path     = resaved.c_str();
                sp2.model    = &dst_model;
                sp2.config   = &cfg2;
                sp2.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
                REQUIRE(store_bbs_3mf(sp2));

                std::string got;
                REQUIRE(read_3mf_entry(resaved, got));
                REQUIRE(got == recipe);
                REQUIRE_FALSE(read_3mf_entry(resaved, got, LEGACY_CAD_RECIPE_ENTRY));
            }
        }
    }
}

// .3mf multi-nozzle round-trip.
// Locks the load/save handling for the H2C multi-nozzle plate metadata:
//   * filament_volume_maps  -> plate config "filament_volume_map" (with the >1 -> 0 clamp)
//   * nozzle_volume_type    -> PlateData::nozzle_volume_types (previously write-only)
// and pins the deliberately-lossy keys (enable_filament_dynamic_map) so a future change has to
// consciously unpin them. Uses a store_bbs_3mf -> load_bbs_3mf cycle (no external fixture needed).
SCENARIO("H2C multi-nozzle .3mf round-trip", "[3mf][MultiNozzle]") {
    GIVEN("a plate carrying multi-nozzle filament assignment metadata") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        // store_bbs_3mf stages Metadata/project_settings.config through the model's backup path;
        // point it at a writable temp dir (the default lives under a read-only root in CI).
        ScopedTemporaryDir backup_dir("orca_mn");
        model.set_backup_path(backup_dir.string());

        // Global (printer) config: give nozzle_volume_type a non-default value so the slice_info
        // read-back is a meaningful assertion (High Flow == 1).
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_key_value("nozzle_volume_type",
                             new ConfigOptionEnumsGeneric({ (int) NozzleVolumeType::nvtHighFlow }));

        PlateData* plate = new PlateData();
        plate->plate_index      = 0;
        plate->is_sliced_valid  = true; // gate for the slice_info.config writer (nozzle_volume_type)
        plate->filament_maps    = { 1, 2, 1 }; // slice_info uses this; keep it == model_settings' value
        plate->config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        plate->config.set_key_value("filament_map", new ConfigOptionInts({ 1, 2, 1 }));
        // Deliberately include out-of-range volume-type ids (2 == Hybrid, 3 == TPU High Flow):
        // the loader must clamp them back to Standard (0).
        plate->config.set_key_value("filament_volume_map", new ConfigOptionInts({ 0, 2, 1, 3 }));
        // Known-lossy: a true value must NOT survive the round-trip (slice_info hardcodes false,
        // model_settings never writes it).
        plate->config.set_key_value("enable_filament_dynamic_map", new ConfigOptionBool(true));

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.plate_data_list.push_back(plate);
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            // LoadConfig is required for slice_info.config (nozzle_volume_type) to be parsed —
            // matches how the app loads projects.
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("every multi-nozzle key round-trips as expected") {
                REQUIRE(loaded);
                REQUIRE(dst_plates.size() >= 1);
                PlateData* rt = dst_plates.front();

                // filament_map (model_settings + slice_info; already round-tripped)
                auto* fmap = rt->config.option<ConfigOptionInts>("filament_map");
                REQUIRE(fmap != nullptr);
                REQUIRE(fmap->values == std::vector<int>({ 1, 2, 1 }));

                // filament_volume_map (model_settings) with the >1 -> 0 clamp
                auto* fvmap = rt->config.option<ConfigOptionInts>("filament_volume_map");
                REQUIRE(fvmap != nullptr);
                REQUIRE(fvmap->values == std::vector<int>({ 0, 0, 1, 0 }));

                // nozzle_volume_type read-back into PlateData::nozzle_volume_types
                REQUIRE(rt->nozzle_volume_types == "1");

                // enable_filament_dynamic_map pinned lossy: model_settings never serializes it and
                // slice_info hardcodes false, so the `true` we set is dropped. Pinned here
                // (absent or false, never true) so a future change that persists it must update this.
                auto* dyn = rt->config.option<ConfigOptionBool>("enable_filament_dynamic_map");
                const bool persisted_true = (dyn != nullptr && dyn->value);
                REQUIRE_FALSE(persisted_true);
            }

            release_PlateData_list(dst_plates);
        }
        delete plate; // store_bbs_3mf does not take ownership of the source plate
    }
}

// Saved nozzle diameter for a single-nozzle-per-extruder printer with a non-standard nozzle.
// The grouping result rounds every nozzle diameter to the nearest of {0.2,0.4,0.6,0.8} for its
// internal matching key. That rounded value must NOT reach the saved <filament>/<nozzle> metadata on
// a printer whose extruders each carry one nozzle: the exact per-extruder config diameter is written
// instead, so a 0.5 mm nozzle is preserved rather than saved as 0.4. (Only an extruder that carries a
// nozzle cluster, which the per-extruder config cannot express, keeps the grouping result's diameter.)
SCENARIO("Non-standard nozzle diameter survives .3mf save on a single-nozzle printer", "[3mf][MultiNozzle]") {
    GIVEN("a single-extruder plate whose nozzle is 0.5 mm and whose stamped diameter was rounded to 0.4") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        ScopedTemporaryDir backup_dir("orca_nd");
        model.set_backup_path(backup_dir.string());

        // Single extruder with a non-standard 0.5 mm nozzle; extruder_max_nozzle_count stays at its
        // default (no nozzle cluster), so the writer must emit the exact config diameter.
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_key_value("nozzle_diameter", new ConfigOptionFloats({ 0.5 }));

        PlateData* plate = new PlateData();
        plate->plate_index     = 0;
        plate->is_sliced_valid = true;      // gate for the slice_info.config writer
        plate->filament_maps   = { 1 };

        // Seed the stamped diameter with the grouping result's rounded value (0.5 -> 0.4) so the
        // assertion proves the writer ignores it and emits the exact config diameter instead.
        FilamentInfo fi;
        fi.id              = 0;
        fi.type            = "PLA";
        fi.color           = "#FFFFFFFF";
        fi.group_id        = { 0 };
        fi.nozzle_diameter = 0.4; // rounded; must NOT be the value written
        plate->slice_filaments_info.push_back(fi);

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.plate_data_list.push_back(plate);
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the saved nozzle diameter is the exact 0.5, not the rounded 0.4") {
                REQUIRE(loaded);
                REQUIRE(dst_plates.size() >= 1);
                PlateData* rt = dst_plates.front();

                // <nozzle> tag: device-facing per-nozzle diameter string, written verbatim.
                REQUIRE(rt->nozzles_info.size() >= 1);
                REQUIRE(rt->nozzles_info.front().diameter == "0.5");

                // <filament> tag: per-filament nozzle_diameter parsed back as 0.5, not 0.4.
                REQUIRE(rt->slice_filaments_info.size() >= 1);
                REQUIRE_THAT(rt->slice_filaments_info.front().nozzle_diameter, Catch::Matchers::WithinAbs(0.5, 1e-6));
            }

            release_PlateData_list(dst_plates);
        }
        delete plate; // store_bbs_3mf does not take ownership of the source plate
    }
}

// A legacy / foreign project (no multi-nozzle metadata) must load crash-safe through the BBS
// importer and must not fabricate a filament_volume_map.
SCENARIO("Legacy project loads crash-safe via load_bbs_3mf", "[3mf][MultiNozzle]") {
    GIVEN("a project without any multi-nozzle metadata") {
        std::string path = std::string(TEST_DATA_DIR) + "/test_3mf/Geräte/Büchse.3mf";
        Model                model;
        DynamicPrintConfig   config;
        ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
        PlateDataPtrs        plates;
        std::vector<Preset*> project_presets;
        bool   is_bbl_3mf = false, is_orca_3mf = false;
        Semver file_version;

        WHEN("loaded through the BBS importer") {
            bool loaded = false;
            REQUIRE_NOTHROW(loaded = load_bbs_3mf(path.c_str(), &config, &ctxt, &model, &plates,
                                                  &project_presets, &is_bbl_3mf, &is_orca_3mf,
                                                  &file_version, nullptr,
                                                  LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
            THEN("it does not crash and invents no per-filament volume map") {
                for (PlateData* p : plates) {
                    REQUIRE(p->config.option<ConfigOptionInts>("filament_volume_map") == nullptr);
                }
            }
            release_PlateData_list(plates);
        }
    }
}

// Device-side nozzle-grouping serialization surface.
// Direct unit coverage for the pure serialize/deserialize + StaticNozzleGroupResult helpers that the
// gcode.3mf writer/reader lean on.
SCENARIO("MultiNozzle serialization helpers", "[3mf][MultiNozzle]") {
    using namespace Slic3r::MultiNozzleUtils;

    GIVEN("NozzleInfo / NozzleGroupInfo") {
        NozzleInfo n0; n0.group_id = 0; n0.extruder_id = 0; n0.diameter = "0.4"; n0.volume_type = nvtStandard;
        NozzleInfo n1; n1.group_id = 1; n1.extruder_id = 1; n1.diameter = "0.4"; n1.volume_type = nvtHighFlow;

        THEN("NozzleInfo::serialize matches the <nozzle> tag attributes (extruder_id 1-based)") {
            REQUIRE(n0.serialize() == "id=\"0\" extruder_id=\"1\" nozzle_diameter=\"0.4\" volume_type=\"Standard\"");
            REQUIRE(n1.serialize() == "id=\"1\" extruder_id=\"2\" nozzle_diameter=\"0.4\" volume_type=\"High Flow\"");
        }
        THEN("NozzleGroupInfo serialize/deserialize round-trips and rejects malformed input") {
            NozzleGroupInfo g("0.4", nvtHighFlow, 1, 3);
            REQUIRE(g.serialize() == "1-0.4-High Flow-3");
            auto rt = NozzleGroupInfo::deserialize(g.serialize());
            REQUIRE(rt.has_value());
            REQUIRE(*rt == g);
            REQUIRE_FALSE(NozzleGroupInfo::deserialize("1-0.4-Standard").has_value()); // too few tokens
            REQUIRE_FALSE(NozzleGroupInfo::deserialize("x-0.4-Standard-3").has_value()); // non-numeric extruder
        }
    }

    GIVEN("a StaticNozzleGroupResult built from filament + nozzle infos") {
        std::vector<NozzleInfo> nozzles;
        { NozzleInfo n; n.group_id = 0; n.extruder_id = 0; n.diameter = "0.4"; n.volume_type = nvtStandard; nozzles.push_back(n); }
        { NozzleInfo n; n.group_id = 1; n.extruder_id = 1; n.diameter = "0.4"; n.volume_type = nvtHighFlow; nozzles.push_back(n); }

        std::vector<FilamentInfo> filaments(3);
        filaments[0].id = 0; filaments[0].group_id = { 0 };
        filaments[1].id = 1; filaments[1].group_id = { 1 };
        filaments[2].id = 2; filaments[2].group_id = { 0, 1 };

        auto result = StaticNozzleGroupResult::create(filaments, nozzles, { 0, 1, 2 }, { 0, 1, 0 }, false);
        REQUIRE(result.has_value());

        THEN("filament->nozzle queries resolve to the stored mapping") {
            REQUIRE(result->get_extruder_count() == 2);
            REQUIRE(result->get_used_extruders() == std::vector<int>({ 0, 1 }));
            REQUIRE(result->get_used_filaments() == std::vector<unsigned int>({ 0, 1, 2 }));
            REQUIRE(result->get_nozzles_for_filament(0).size() == 1);
            REQUIRE(result->get_nozzles_for_filament(2).size() == 2);
            // first-use resolves through the (filament,nozzle) change sequences.
            auto first = result->get_first_nozzle_for_filament(1);
            REQUIRE(first.has_value());
            REQUIRE(first->group_id == 1);
        }
        THEN("empty inputs yield nullopt") {
            REQUIRE_FALSE(StaticNozzleGroupResult::create({}, nozzles, {}, {}, false).has_value());
            REQUIRE_FALSE(StaticNozzleGroupResult::create(filaments, {}, {}, {}, false).has_value());
        }
    }

    GIVEN("load_nozzle_infos_with_compatibility fallbacks") {
        std::vector<NozzleInfo> new_format;
        { NozzleInfo n; n.group_id = 1; n.extruder_id = 1; n.diameter = "0.4"; n.volume_type = nvtHighFlow; new_format.push_back(n); }
        { NozzleInfo n; n.group_id = 0; n.extruder_id = 0; n.diameter = "0.4"; n.volume_type = nvtStandard; new_format.push_back(n); }

        THEN("new-format <nozzle> tags are returned sorted by logical id") {
            auto out = load_nozzle_infos_with_compatibility(new_format, {}, {}, {}, {});
            REQUIRE(out.size() == 2);
            REQUIRE(out[0].group_id == 0);
            REQUIRE(out[1].group_id == 1);
        }
        THEN("oldest single-nozzle 3mf (no tags, no filament group_id) rebuilds from diameters/volume types") {
            std::vector<NozzleVolumeType> vt = { nvtStandard, nvtHighFlow };
            std::vector<double>           dia = { 0.4, 0.4 };
            auto out = load_nozzle_infos_with_compatibility({}, {}, {}, vt, dia);
            REQUIRE(out.size() == 2);
            REQUIRE(out[0].extruder_id == 0);
            REQUIRE(out[0].volume_type == nvtStandard);
            REQUIRE(out[1].volume_type == nvtHighFlow);
        }
    }
}

// The layer-aware grouping result must survive the gcode.3mf write/read as
// <nozzle> tags and the enable_filament_dynamic_map flag. Proves the parse_filament_info stamping,
// the NOZZLE_TAG writer, the _handle_config_nozzle reader, and the nozzles_info plate copy.
SCENARIO("Nozzle-group metadata .3mf round-trip", "[3mf][MultiNozzle]") {
    GIVEN("a plate carrying a two-nozzle LayeredNozzleGroupResult") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        ScopedTemporaryDir backup_dir("orca_ng");
        model.set_backup_path(backup_dir.string());

        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();

        std::vector<MultiNozzleUtils::NozzleInfo> nozzles;
        { MultiNozzleUtils::NozzleInfo n; n.group_id = 0; n.extruder_id = 0; n.diameter = "0.4"; n.volume_type = NozzleVolumeType::nvtStandard; nozzles.push_back(n); }
        { MultiNozzleUtils::NozzleInfo n; n.group_id = 1; n.extruder_id = 1; n.diameter = "0.4"; n.volume_type = NozzleVolumeType::nvtHighFlow; nozzles.push_back(n); }
        auto group = MultiNozzleUtils::LayeredNozzleGroupResult::create(
            std::vector<int>{ 0, 1, 0 }, nozzles, std::vector<unsigned int>{ 0, 1, 2 });
        REQUIRE(group.has_value());

        PlateData* plate = new PlateData();
        plate->plate_index     = 0;
        plate->is_sliced_valid = true;
        plate->filament_maps   = { 1, 2, 1 };
        plate->nozzle_group_result = group;
        plate->config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        plate->config.set_key_value("filament_map", new ConfigOptionInts({ 1, 2, 1 }));

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.plate_data_list.push_back(plate);
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the <nozzle> tags round-trip into the loaded plate's nozzles_info") {
                REQUIRE(loaded);
                REQUIRE(dst_plates.size() >= 1);
                PlateData* rt = dst_plates.front();

                REQUIRE(rt->nozzles_info.size() == 2);
                // reader stores extruder_id 0-based (tag is 1-based), diameter/volume_type preserved.
                std::sort(rt->nozzles_info.begin(), rt->nozzles_info.end());
                REQUIRE(rt->nozzles_info[0].group_id == 0);
                REQUIRE(rt->nozzles_info[0].extruder_id == 0);
                REQUIRE(rt->nozzles_info[0].diameter == "0.4");
                REQUIRE(rt->nozzles_info[0].volume_type == NozzleVolumeType::nvtStandard);
                REQUIRE(rt->nozzles_info[1].group_id == 1);
                REQUIRE(rt->nozzles_info[1].extruder_id == 1);
                REQUIRE(rt->nozzles_info[1].volume_type == NozzleVolumeType::nvtHighFlow);

                // A static (non-selector) result must persist enable_filament_dynamic_map = false.
                auto* dyn = rt->config.option<ConfigOptionBool>("enable_filament_dynamic_map");
                const bool persisted_true = (dyn != nullptr && dyn->value);
                REQUIRE_FALSE(persisted_true);
            }

            release_PlateData_list(dst_plates);
        }
        delete plate;
    }
}

// A mixed-color filament occupies an ordinary filament slot, and painting with it stores an
// ordinary extruder state: a project saved by BambuStudio encodes filament 5 of a 5-slot setup
// as paint state 5, with the mix described by the parallel filament_mixed_* project arrays.
SCENARIO("Mixed-color filament setup and painting round-trip through a .3mf", "[3mf][MixedFilament]") {
    GIVEN("a painted model whose project config describes a mixed filament in the last slot") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        // Both the exporter and the importer stage Metadata/project_settings.config through the
        // model's backup path; point them at writable temp dirs.
        ScopedTemporaryDir backup_dir("orca_mixed_src");
        model.set_backup_path(backup_dir.string());

        ModelVolume* mv = model.objects.front()->volumes.front();
        {
            TriangleSelector selector(mv->mesh());
            selector.set_facet(0, EnforcerBlockerType::Extruder5); // the mixed slot
            selector.set_facet(1, EnforcerBlockerType::Extruder2);
            REQUIRE(mv->mmu_segmentation_facets.set(selector));
        }

        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_key_value("filament_colour", new ConfigOptionStrings(
            { "#00AE42", "#FFFF00", "#FF0000", "#0000FF", "#FF6A26" }));
        config.set_key_value("filament_is_mixed", new ConfigOptionBools(
            { false, false, false, false, true }));
        config.set_key_value("filament_mixed_components", new ConfigOptionStrings(
            { "", "", "", "", "3,2" }));
        config.set_key_value("filament_mixed_sublayer_ratios", new ConfigOptionStrings(
            { "", "", "", "", "0.4200,0.5800" }));

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            PlateData* plate = new PlateData();
            plate->plate_index = 0;

            StoreParams store_params;
            store_params.path     = test_file.c_str();
            store_params.model    = &model;
            store_params.config   = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            store_params.plate_data_list.push_back(plate);
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            ScopedTemporaryDir dst_backup_dir("orca_mixed_dst");
            dst_model.set_backup_path(dst_backup_dir.string());
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            REQUIRE(load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                 &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                 LoadStrategy::LoadModel | LoadStrategy::LoadConfig));

            THEN("the mixed-filament project keys survive") {
                auto* is_mixed = dst_config.option<ConfigOptionBools>("filament_is_mixed");
                REQUIRE(is_mixed != nullptr);
                REQUIRE(is_mixed->values == std::vector<unsigned char>({ 0, 0, 0, 0, 1 }));

                auto* components = dst_config.option<ConfigOptionStrings>("filament_mixed_components");
                REQUIRE(components != nullptr);
                REQUIRE(components->values.size() == 5);
                REQUIRE(components->values[4] == "3,2");

                auto* ratios = dst_config.option<ConfigOptionStrings>("filament_mixed_sublayer_ratios");
                REQUIRE(ratios != nullptr);
                REQUIRE(ratios->values.size() == 5);
                REQUIRE(ratios->values[4] == "0.4200,0.5800");
            }

            THEN("the painted facets survive, including the one painted with the mixed slot") {
                REQUIRE(dst_model.objects.size() == 1);
                ModelVolume* dst_mv = dst_model.objects.front()->volumes.front();
                REQUIRE_FALSE(dst_mv->mmu_segmentation_facets.empty());
                REQUIRE(dst_mv->mmu_segmentation_facets.has_facets(*dst_mv, EnforcerBlockerType::Extruder2));
                REQUIRE(dst_mv->mmu_segmentation_facets.has_facets(*dst_mv, EnforcerBlockerType::Extruder5));
            }

            release_PlateData_list(dst_plates);
            delete plate; // store_bbs_3mf does not take ownership of the source plate
        }
    }
}
// Locks the serialization contract of the "Publish" metadata: the orca_published flag and the
// orca_published_keys JSON array in model.model_info->metadata_items must survive a store_bbs_3mf ->
// load_bbs_3mf round-trip unchanged. (The full preset-preservation behavior is exercised
// headlessly in test_preset_bundle_loading.cpp.)
SCENARIO("Published 3MF round-trips the published flag and published_keys metadata", "[3mf]") {
    GIVEN("a model carrying published metadata") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->metadata_items[ORCA_PUBLISHED_TAG]      = "1";
        model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] = R"(["layer_height","wall_thickness"])";

        // store_bbs_3mf stages project_settings.config through the model's backup path; point
        // it at a writable temp dir (the default lives under a read-only root in CI).
        ScopedTemporaryDir backup_dir("orca_pub");
        model.set_backup_path(backup_dir.string());

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the published metadata round-trips unchanged") {
                REQUIRE(loaded);
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_TAG] == "1");
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] == R"(["layer_height","wall_thickness"])");

                // The orca_published_keys value is a JSON array of setting keys; it must parse back to
                // the same keys that were selected.
                nlohmann::json keys = nlohmann::json::parse(dst_model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG]);
                REQUIRE(keys.is_array());
                REQUIRE(keys.size() == 2);
                REQUIRE(keys[0] == "layer_height");
                REQUIRE(keys[1] == "wall_thickness");
            }
            release_PlateData_list(dst_plates);
        }
    }
}

// A normal 3MF (no Publish metadata) must load identically: the loader must not fabricate a
// "orca_published" flag or orca_published_keys for files that never carried them.
SCENARIO("Legacy 3MF without published metadata loads unchanged", "[3mf]") {
    GIVEN("a model without any published metadata") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        ScopedTemporaryDir backup_dir("orca_legacy");
        model.set_backup_path(backup_dir.string());

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("no published key is fabricated") {
                REQUIRE(loaded);
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items.count(ORCA_PUBLISHED_TAG) == 0);
                REQUIRE(dst_model.model_info->metadata_items.count(ORCA_PUBLISHED_KEYS_TAG) == 0);
            }
            release_PlateData_list(dst_plates);
        }
    }
}

// Locks the serialization contract of the orca_published_material_keys metadata: the per-entry JSON
// must survive a store_bbs_3mf -> load_bbs_3mf round-trip verbatim, exactly like orca_published_keys.
SCENARIO("Published 3MF round-trips the published_material_keys metadata", "[3mf]") {
    GIVEN("a model carrying published material keys metadata") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        const std::string material_keys_json =
            R"([{"material":{"filament_type":"PLA","filament_vendor":"Generic","filament_id":"GFL99"},"slot":0,"keys":["filament_retraction_length","filament_z_hop"]}])";

        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->metadata_items[ORCA_PUBLISHED_MATERIAL_TAG] = material_keys_json;

        ScopedTemporaryDir backup_dir("orca_pub_mat");
        model.set_backup_path(backup_dir.string());

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the published material keys metadata round-trips unchanged") {
                REQUIRE(loaded);
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_MATERIAL_TAG] == material_keys_json);

                // The value must parse back to one material entry carrying the nested identity
                // object, the author slot ordinal and the key list.
                nlohmann::json entries = nlohmann::json::parse(material_keys_json);
                REQUIRE(entries.is_array());
                REQUIRE(entries.size() == 1);
                REQUIRE(entries[0]["material"]["filament_type"] == "PLA");
                REQUIRE(entries[0]["material"]["filament_vendor"] == "Generic");
                REQUIRE(entries[0]["material"]["filament_id"] == "GFL99");
                REQUIRE(entries[0]["slot"] == 0);
                REQUIRE(entries[0]["keys"].is_array());
                REQUIRE(entries[0]["keys"].size() == 2);
                REQUIRE(entries[0]["keys"][0] == "filament_retraction_length");
            }
            release_PlateData_list(dst_plates);
        }
    }
}

SCENARIO("Minimal published 3MF omits project config, preset dumps and slicer tags", "[3mf]") {
    GIVEN("a multi-instance model carrying published metadata and a published_config payload") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();
        // A second instance: tag-less third-party files get their multi-instance objects split,
        // published files must not (the loader recognizes them by their metadata).
        model.objects.front()->add_instance();

        DynamicPrintConfig full_cfg = DynamicPrintConfig::full_print_config();
        full_cfg.set_key_value("layer_height", new ConfigOptionFloat(0.24));
        full_cfg.set_key_value("retraction_length", new ConfigOptionFloats({ 1.2 }));

        const std::vector<std::string> published_keys = { "layer_height", "retraction_length" };
        const std::vector<PublishedMaterialEntry> material_keys = {
            { "PLA", "Generic", "GFL99", "", "Generic PLA", 0, { "filament_retraction_length" } }
        };

        // The payload builder keeps the published and identity keys and drops everything else.
        DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, published_keys, material_keys);
        REQUIRE(filtered_cfg.option("layer_height") != nullptr);
        REQUIRE(filtered_cfg.option("retraction_length") != nullptr);
        REQUIRE(filtered_cfg.option("filament_colour") != nullptr);
        REQUIRE(filtered_cfg.option("filament_type") != nullptr);
        REQUIRE(filtered_cfg.option("wipe_tower_x") != nullptr);
        REQUIRE(filtered_cfg.option("sparse_infill_density") == nullptr);
        REQUIRE(filtered_cfg.option("machine_start_gcode") == nullptr);

        // Serialize the payload exactly like export_published_3mf does.
        std::string payload;
        for (const std::string &key : filtered_cfg.keys())
            payload += key + " = " + filtered_cfg.opt_serialize(key) + "\n";

        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->metadata_items[ORCA_PUBLISHED_TAG]      = "1";
        model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] = R"(["layer_height","retraction_length"])";
        model.model_info->metadata_items[ORCA_PUBLISHED_CONFIG_TAG] = payload;

        ScopedTemporaryDir backup_dir("orca_min_pub");
        model.set_backup_path(backup_dir.string());

        WHEN("stored using SaveStrategy::MinimalPublished and reloaded") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            // Create a fake project preset to verify MinimalPublished omits it.
            Preset preset(Preset::TYPE_PRINT, "TestPrintPreset");
            preset.config = full_cfg;
            std::vector<Preset*> project_presets = { &preset };

            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &filtered_cfg;
            store_params.project_presets = project_presets;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::MinimalPublished;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            ScopedTemporaryDir loaded_backup_dir("orca_min_pub_loaded");
            dst_model.set_backup_path(loaded_backup_dir.string());
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> loaded_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &loaded_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the 3MF loads without project config or embedded presets") {
                REQUIRE(loaded);
                REQUIRE(dst_config.empty());
                REQUIRE(loaded_presets.empty());
            }
            THEN("the file carries no slicer tags and classifies as a generic 3MF") {
                REQUIRE_FALSE(is_bbl_3mf);
                REQUIRE_FALSE(is_orca_3mf);
                // No Application / OrcaSlicer tag: old receivers import the geometry silently
                // instead of showing a baked-in, wrong "old version" popup.
                REQUIRE_FALSE(file_version.valid());
            }
            THEN("the geometry keeps BBS-grade handling: instances are not split") {
                REQUIRE(dst_model.objects.size() == 1);
                REQUIRE(dst_model.objects.front()->instances.size() == 2);
            }
            THEN("the published metadata and payload round-trip unchanged") {
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_TAG] == "1");
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] == R"(["layer_height","retraction_length"])");
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_CONFIG_TAG] == payload);
            }
            THEN("the payload parses back to the published values") {
                DynamicPrintConfig parsed_payload;
                parsed_payload.load_from_ini_string(dst_model.model_info->metadata_items[ORCA_PUBLISHED_CONFIG_TAG], ForwardCompatibilitySubstitutionRule::Enable);
                REQUIRE(parsed_payload.option("layer_height") != nullptr);
                REQUIRE_THAT(parsed_payload.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-6));
                REQUIRE(parsed_payload.option("retraction_length") != nullptr);
                REQUIRE_THAT(parsed_payload.opt<ConfigOptionFloats>("retraction_length")->get_at(0), Catch::Matchers::WithinAbs(1.2, 1e-6));
            }
            release_PlateData_list(dst_plates);
        }
    }
}

// A minimal published 3MF must not leak the slicer tags of the source project. The exporter seeds
// metadata_item_map from the input file's metadata_items, so re-publishing a project opened from a
// regular Orca/BBS 3MF (the typical remix flow) must strip the Application / OrcaSlicer tags it
// came with, otherwise old receivers route onto the baked-in "old version" popup.
SCENARIO("MinimalPublished strips slicer tags carried by the source project", "[3mf]") {
    GIVEN("a model loaded from a regular Orca/BBS 3MF whose metadata carries the slicer tags") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->metadata_items[ORCA_PUBLISHED_TAG] = "1";
        model.model_info->metadata_items["Application"]     = "BambuStudio-2.0.0";
        model.model_info->metadata_items["OrcaSlicer"]      = "2.1.0";

        ScopedTemporaryDir backup_dir("orca_strip_tags");
        model.set_backup_path(backup_dir.string());

        WHEN("stored using SaveStrategy::MinimalPublished and reloaded") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            StoreParams store_params;
            store_params.path    = test_file.c_str();
            store_params.model   = &model;
            store_params.config  = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::MinimalPublished;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> loaded_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &loaded_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the source slicer tags are stripped, not carried through") {
                REQUIRE(loaded);
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items.count("Application") == 0);
                REQUIRE(dst_model.model_info->metadata_items.count("OrcaSlicer") == 0);
                // The published marker itself must survive.
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_TAG] == "1");
            }
            THEN("the file classifies as a generic 3MF without a version popup") {
                REQUIRE_FALSE(is_bbl_3mf);
                REQUIRE_FALSE(is_orca_3mf);
                REQUIRE_FALSE(file_version.valid());
            }
            release_PlateData_list(dst_plates);
        }
    }
}

// An entry masks the non-published slots to their defaults so publishing slot 1 never leaks slot
// 0's value into the file. Both a full entry (the whole-slot key list) and a partial entry (a
// per-slot key) go through the same masking path in filter_published_config (keys and full_keys
// are filtered identically), so the two forms are exercised together.
SCENARIO("Published entries mask the other slots to their defaults", "[3mf]") {
    const bool full = GENERATE(true, false);
    GIVEN("a full print configuration with two filament slots") {
        DynamicPrintConfig full_cfg = DynamicPrintConfig::full_print_config();
        full_cfg.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        full_cfg.opt<ConfigOptionStrings>("filament_colour")->values = { "#111111", "#222222" };
        // filament_flow_ratio carries a non-empty option default (1.0) of the same type, so the
        // mask can restore it on the non-published slot.
        full_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio", true)->values = { 1.02, 0.98 };

        WHEN("filtering with a published entry for slot 1") {
            PublishedMaterialEntry entry;
            entry.slot = 1;
            if (full) {
                entry.full      = true;
                entry.full_keys = { "filament_flow_ratio" };
            } else {
                entry.keys = { "filament_flow_ratio" };
            }
            DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, {}, { entry });

            THEN("the selected key is present with the author's slot value") {
                REQUIRE(filtered_cfg.option("filament_flow_ratio") != nullptr);
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values[1], Catch::Matchers::WithinAbs(0.98, 1e-6));
            }
            THEN("the non-published slot is masked to its default") {
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
            }
            THEN("the identity keys stay present") {
                REQUIRE(filtered_cfg.option("filament_colour") != nullptr);
            }
        }
    }
}

// A key needing slot masking that cannot be masked (no registered option default of the same
// type) is dropped from the payload entirely instead of shipping the author's whole vector.
SCENARIO("Unmaskable keys are dropped from the published payload instead of leaking", "[3mf]") {
    GIVEN("a config carrying a synthetic def-less vector key and a maskable one") {
        DynamicPrintConfig full_cfg = DynamicPrintConfig::full_print_config();
        full_cfg.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75 };
        full_cfg.opt<ConfigOptionStrings>("filament_colour")->values  = { "#111111", "#222222" };
        // Not a PrintConfig key: print_config_def has no default to mask with.
        full_cfg.set_key_value("orca_synthetic_setting", new ConfigOptionFloats({ 9.9, 8.8 }));
        full_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio", true)->values = { 1.02, 0.98 };

        PublishedMaterialEntry partial_entry;
        partial_entry.slot = 1;
        partial_entry.keys = { "orca_synthetic_setting", "filament_flow_ratio" };

        WHEN("filtering with a partial entry for slot 1") {
            DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, {}, { partial_entry });

            THEN("the unmaskable synthetic key is not published") {
                REQUIRE(filtered_cfg.option("orca_synthetic_setting") == nullptr);
            }
            THEN("the maskable key is present, author slot kept, other slot masked") {
                REQUIRE(filtered_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio") != nullptr);
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values[1], Catch::Matchers::WithinAbs(0.98, 1e-6));
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloatsNullable>("filament_flow_ratio")->values[0], Catch::Matchers::WithinAbs(1.0, 1e-6));
            }
            THEN("the identity keys stay present") {
                REQUIRE(filtered_cfg.option("filament_colour") != nullptr);
            }
        }
    }
}

// A per-extruder printer key carrying a "#N" variant (e.g. retraction_length#1) must not serialize
// every extruder's value: the base is masked to the author's extruder and the other slots are
// restored to their option default, matching the material-side slot-masking invariant. A bare
// printer base key (no variant) keeps whole-vector serialization.
SCENARIO("Published per-extruder printer keys mask the other extruders to their defaults", "[3mf]") {
    GIVEN("a full print configuration with three extruders carrying per-extruder retraction values") {
        DynamicPrintConfig full_cfg = DynamicPrintConfig::full_print_config();
        // Non-default values on the un-selected slots, so a leak is distinguishable from the mask
        // restoring the option default (retraction_length defaults to {0.8}).
        full_cfg.opt<ConfigOptionFloats>("retraction_length")->values = { 3.0, 1.2, 4.0 };

        WHEN("filtering with only extruder 1's retraction_length checked") {
            DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, { "retraction_length#1" }, {});

            THEN("the author's extruder value survives") {
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[1], Catch::Matchers::WithinAbs(1.2, 1e-6));
            }
            THEN("the other extruders are masked to their default") {
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[0], Catch::Matchers::WithinAbs(0.8, 1e-6));
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[2], Catch::Matchers::WithinAbs(0.8, 1e-6));
            }
        }
        WHEN("filtering the bare base key without a '#N' variant") {
            DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, { "retraction_length" }, {});

            THEN("the whole vector is serialized unmasked") {
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[0], Catch::Matchers::WithinAbs(3.0, 1e-6));
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[1], Catch::Matchers::WithinAbs(1.2, 1e-6));
                REQUIRE_THAT(filtered_cfg.opt<ConfigOptionFloats>("retraction_length")->values[2], Catch::Matchers::WithinAbs(4.0, 1e-6));
            }
        }
    }
}

// The extended per-entry fields (full dump list, published type and colour) travel inside the
// published_material_keys metadata and round-trip unchanged.
SCENARIO("Published 3MF round-trips the extended material metadata", "[3mf]") {
    GIVEN("a model carrying extended published material keys metadata") {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();

        const std::string material_keys_json =
            R"([{"material":{"filament_type":"PLA","filament_vendor":"Generic","filament_id":"GFL99","setting_id":"RFs9eCKYOMUSmvZf","name":"Generic PLA Matte @System"},"slot":1,"keys":[],"full":true,"full_keys":["filament_retraction_length","filament_colour"],"publish_type":true,"type":"PLA","publish_color":false,"color":""}])";

        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->metadata_items[ORCA_PUBLISHED_MATERIAL_TAG] = material_keys_json;

        ScopedTemporaryDir backup_dir("orca_pub_mat2");
        model.set_backup_path(backup_dir.string());

        WHEN("stored to and reloaded from a .3mf") {
            ScopedTemporaryFile temp(".3mf");
            const std::string test_file = temp.string();

            DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
            StoreParams store_params;
            store_params.path     = test_file.c_str();
            store_params.model    = &model;
            store_params.config   = &config;
            store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
            REQUIRE(store_bbs_3mf(store_params));

            Model dst_model;
            DynamicPrintConfig dst_config;
            ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
            PlateDataPtrs        dst_plates;
            std::vector<Preset*> project_presets;
            bool   is_bbl_3mf = false, is_orca_3mf = false;
            Semver file_version;
            bool loaded = load_bbs_3mf(test_file.c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                       &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                       LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
            THEN("the extended material metadata round-trips unchanged") {
                REQUIRE(loaded);
                REQUIRE(dst_model.model_info != nullptr);
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_MATERIAL_TAG] == material_keys_json);

                // The value must parse back with every extended field intact.
                nlohmann::json entries = nlohmann::json::parse(material_keys_json);
                REQUIRE(entries.is_array());
                REQUIRE(entries.size() == 1);
                REQUIRE(entries[0]["full"].get<bool>() == true);
                REQUIRE(entries[0]["full_keys"].is_array());
                REQUIRE(entries[0]["full_keys"].size() == 2);
                REQUIRE(entries[0]["publish_type"].get<bool>() == true);
                REQUIRE(entries[0]["type"] == "PLA");
                REQUIRE(entries[0]["publish_color"].get<bool>() == false);
            }
            release_PlateData_list(dst_plates);
        }
    }
}

// A published mixed filament serializes its whole definition (components, ratios, gradient)
// masked to the author's slot: the mix slot's values survive, the non-published slots reset to
// their defaults, so a partial publish never leaks another slot's mix data.
SCENARIO("Published mixed-filament keys are masked to the author's slot", "[3mf]") {
    GIVEN("a full print configuration with three slots, one of them mixed") {
        DynamicPrintConfig full_cfg = DynamicPrintConfig::full_print_config();
        full_cfg.opt<ConfigOptionFloats>("filament_diameter")->values = { 1.75, 1.75, 1.75 };
        full_cfg.opt<ConfigOptionStrings>("filament_colour")->values  = { "#111111", "#222222", "#333333" };
        full_cfg.opt<ConfigOptionBools>("filament_is_mixed")->values  = { 0, 0, 1 };
        full_cfg.opt<ConfigOptionStrings>("filament_mixed_components")->values       = { "", "", "1,2" };
        full_cfg.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values   = { "", "", "0.6,0.4" };
        full_cfg.opt<ConfigOptionBools>("filament_mixed_gradient")->values            = { 0, 0, 1 };
        full_cfg.opt<ConfigOptionStrings>("filament_mixed_gradient_range")->values    = { "", "", "0.9,0.1" };
        full_cfg.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values    = { "", "", "0,0.1|1,0.9" };
        full_cfg.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values   = { 0, 0, 1 };

        PublishedMaterialEntry mix_entry;
        mix_entry.slot = 2;
        mix_entry.keys = {
            "filament_is_mixed",          "filament_mixed_components",       "filament_mixed_sublayer_ratios",
            "filament_mixed_gradient",    "filament_mixed_gradient_range",   "filament_mixed_gradient_curve",
            "filament_mixed_gradient_per_part"
        };

        WHEN("filtering with a mixed entry for slot 2") {
            DynamicPrintConfig filtered_cfg = filter_published_config(full_cfg, {}, { mix_entry });

            THEN("the author's mixed slot keeps its definition") {
                REQUIRE(filtered_cfg.option("filament_is_mixed") != nullptr);
                REQUIRE(filtered_cfg.opt<ConfigOptionBools>("filament_is_mixed")->values == std::vector<unsigned char>{ 0, 0, 1 });
                const auto& components = filtered_cfg.opt<ConfigOptionStrings>("filament_mixed_components")->values;
                REQUIRE(components.size() == 3);
                CHECK(components[2] == "1,2");
                CHECK(filtered_cfg.opt<ConfigOptionStrings>("filament_mixed_sublayer_ratios")->values[2] == "0.6,0.4");
                CHECK(filtered_cfg.opt<ConfigOptionStrings>("filament_mixed_gradient_curve")->values[2] == "0,0.1|1,0.9");
                CHECK(filtered_cfg.opt<ConfigOptionBools>("filament_mixed_gradient")->values[2]);
                CHECK(filtered_cfg.opt<ConfigOptionBools>("filament_mixed_gradient_per_part")->values[2]);
            }
            THEN("the non-published slots are masked to their defaults") {
                CHECK(filtered_cfg.opt<ConfigOptionStrings>("filament_mixed_components")->values[0] == "");
                CHECK(filtered_cfg.opt<ConfigOptionStrings>("filament_mixed_components")->values[1] == "");
                CHECK(filtered_cfg.opt<ConfigOptionBools>("filament_is_mixed")->values[0] == 0);
                CHECK(filtered_cfg.opt<ConfigOptionBools>("filament_is_mixed")->values[1] == 0);
            }
            THEN("the identity keys stay present") {
                REQUIRE(filtered_cfg.option("filament_colour") != nullptr);
            }
        }
    }
}

// The published flag is gated on the exact string "1": any other serialized value means "not
// published", so a receiver never treats a file as published on a loose truthiness check.
TEST_CASE("is_published_3mf_flag accepts only the literal \"1\"", "[3mf]") {
    CHECK(is_published_3mf_flag("1"));
    CHECK_FALSE(is_published_3mf_flag("0"));
    CHECK_FALSE(is_published_3mf_flag("false"));
    CHECK_FALSE(is_published_3mf_flag("true"));
    CHECK_FALSE(is_published_3mf_flag(""));
    CHECK_FALSE(is_published_3mf_flag("YES"));
}

// bbs_3mf_is_published is the lightweight metadata probe used to decide whether a file was
// produced by the publish feature (GUI "recently published" tracking). It must return true only
// for a file whose metadata carries the flag set to "1", and false for legacy files and for a
// file whose flag is present but not "1" (which loads as a normal, non-published 3MF).
SCENARIO("bbs_3mf_is_published detects only genuinely published 3MFs", "[3mf]") {
    auto store_model = [](const std::string &path, const std::string &flag_value, const std::string &keys_value) {
        Model model;
        std::string src_file = std::string(TEST_DATA_DIR) + "/test_3mf/Prusa.stl";
        REQUIRE(load_stl(src_file.c_str(), &model));
        model.add_default_instances();
        model.model_info = std::make_shared<ModelInfo>();
        // An empty flag_value means "don't write the flag at all" (a legacy file).
        if (!flag_value.empty())
            model.model_info->metadata_items[ORCA_PUBLISHED_TAG] = flag_value;
        model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] = keys_value;
        ScopedTemporaryDir backup_dir("orca_is_pub");
        model.set_backup_path(backup_dir.string());
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        StoreParams store_params;
        store_params.path     = path.c_str();
        store_params.model    = &model;
        store_params.config   = &config;
        store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
        REQUIRE(store_bbs_3mf(store_params));
    };

    GIVEN("a minimal published 3MF whose flag is \"1\"") {
        ScopedTemporaryFile temp(".3mf");
        store_model(temp.string(), "1", R"(["layer_height"])");
        WHEN("probed by bbs_3mf_is_published") {
            THEN("it is recognized as published") {
                CHECK(bbs_3mf_is_published(temp.string()));
            }
        }
    }
    GIVEN("a legacy 3MF without any published flag") {
        ScopedTemporaryFile temp(".3mf");
        store_model(temp.string(), "", R"(["layer_height"])");
        WHEN("probed by bbs_3mf_is_published") {
            THEN("it is not recognized as published") {
                CHECK_FALSE(bbs_3mf_is_published(temp.string()));
            }
        }
    }
    GIVEN("a 3MF carrying the flag set to \"0\"") {
        ScopedTemporaryFile temp(".3mf");
        store_model(temp.string(), "0", R"(["layer_height"])");
        WHEN("probed and loaded") {
            THEN("it is not recognized as published") {
                CHECK_FALSE(bbs_3mf_is_published(temp.string()));
            }
            THEN("it loads as a normal, non-published 3MF") {
                Model dst_model;
                DynamicPrintConfig dst_config;
                ConfigSubstitutionContext ctxt{ ForwardCompatibilitySubstitutionRule::Enable };
                PlateDataPtrs        dst_plates;
                std::vector<Preset*> project_presets;
                bool   is_bbl_3mf = false, is_orca_3mf = false;
                Semver file_version;
                REQUIRE(load_bbs_3mf(temp.string().c_str(), &dst_config, &ctxt, &dst_model, &dst_plates,
                                     &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
                REQUIRE(dst_model.model_info != nullptr);
                // The key is present but not "1", so nothing treats the file as published; the
                // stored keys still round-trip verbatim.
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_TAG] == "0");
                REQUIRE(dst_model.model_info->metadata_items[ORCA_PUBLISHED_KEYS_TAG] == R"(["layer_height"])");
                release_PlateData_list(dst_plates);
            }
        }
    }
}

TEST_CASE("BBS 3MF round-trips object layer and brim metadata", "[3mf][3MFModelMetadata]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10., 10., 2.));
    object->add_instance();

    const std::vector<coordf_t> expected_profile { 0., 0.12, 0.8, 0.2, 1.6, 0.28 };
    object->layer_height_profile.set(expected_profile);

    DynamicPrintConfig range_config;
    range_config.set_key_value("layer_height", new ConfigOptionFloat(0.31));
    object->layer_config_ranges[{ 0.4, 1.2 }].assign_config(std::move(range_config));

    object->brim_points.emplace_back(1.f, 2.f, 0.3f, 0.45f);
    object->brim_points.emplace_back(3.f, 4.f, 0.7f, 0.85f);

    ScopedTemporaryDir backup_dir("orca_3mf_metadata_src");
    model.set_backup_path(backup_dir.string());
    ScopedTemporaryFile temp(".3mf");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    StoreParams store_params;
    store_params.path     = temp.string().c_str();
    store_params.model    = &model;
    store_params.config   = &config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup_dir("orca_3mf_metadata_dst");
    REQUIRE(load_project(temp.string(), loaded_model, loaded_backup_dir));

    REQUIRE(loaded_model.objects.size() == 1);
    const ModelObject *loaded_object = loaded_model.objects.front();
    REQUIRE(loaded_object->volumes.size() == 1);
    REQUIRE(loaded_object->volumes.front()->mesh().its.vertices.size() == object->volumes.front()->mesh().its.vertices.size());

    const std::vector<coordf_t> loaded_profile = loaded_object->layer_height_profile.get();
    REQUIRE(loaded_profile.size() == expected_profile.size());
    for (size_t i = 0; i < expected_profile.size(); ++i)
        CHECK_THAT(loaded_profile[i], Catch::Matchers::WithinAbs(expected_profile[i], 1e-6));

    REQUIRE(loaded_object->layer_config_ranges.size() == 1);
    const auto &loaded_range = loaded_object->layer_config_ranges.begin();
    CHECK_THAT(loaded_range->first.first, Catch::Matchers::WithinAbs(0.4, 1e-6));
    CHECK_THAT(loaded_range->first.second, Catch::Matchers::WithinAbs(1.2, 1e-6));
    REQUIRE(loaded_range->second.has("layer_height"));
    CHECK_THAT(loaded_range->second.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-6));

    REQUIRE(loaded_object->brim_points.size() == 2);
    CHECK_THAT(loaded_object->brim_points[0].pos(0), Catch::Matchers::WithinAbs(1., 1e-5));
    CHECK_THAT(loaded_object->brim_points[0].pos(1), Catch::Matchers::WithinAbs(2., 1e-5));
    CHECK_THAT(loaded_object->brim_points[0].pos(2), Catch::Matchers::WithinAbs(0.3, 1e-5));
    CHECK_THAT(loaded_object->brim_points[0].head_front_radius, Catch::Matchers::WithinAbs(0.45, 1e-5));
    CHECK_THAT(loaded_object->brim_points[1].pos(0), Catch::Matchers::WithinAbs(3., 1e-5));
    CHECK_THAT(loaded_object->brim_points[1].pos(1), Catch::Matchers::WithinAbs(4., 1e-5));
    CHECK_THAT(loaded_object->brim_points[1].pos(2), Catch::Matchers::WithinAbs(0.7, 1e-5));
    CHECK_THAT(loaded_object->brim_points[1].head_front_radius, Catch::Matchers::WithinAbs(0.85, 1e-5));
}

TEST_CASE("BBS 3MF round-trips per-plate custom G-code", "[3mf][3MFCustomGCode]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10., 10., 2.));
    object->add_instance();

    CustomGCode::Info &gcode = model.plates_custom_gcodes[0];
    gcode.mode = CustomGCode::Mode::MultiExtruder;
    gcode.gcodes = {
        { 0.2, CustomGCode::ColorChange, 1, "#ff0000", "" },
        { 0.4, CustomGCode::PausePrint, 2, "pause", "hold for inspection" },
        { 0.6, CustomGCode::ToolChange, 3, "", "" },
        { 0.8, CustomGCode::Custom, 1, "", "M117 custom marker" },
    };

    ScopedTemporaryDir backup_dir("orca_3mf_custom_gcode_src");
    model.set_backup_path(backup_dir.string());
    ScopedTemporaryFile temp(".3mf");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    StoreParams store_params;
    store_params.path     = temp.string().c_str();
    store_params.model    = &model;
    store_params.config   = &config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup_dir("orca_3mf_custom_gcode_dst");
    REQUIRE(load_project(temp.string(), loaded_model, loaded_backup_dir));

    REQUIRE(loaded_model.plates_custom_gcodes.size() == 1);
    const auto loaded_it = loaded_model.plates_custom_gcodes.find(0);
    REQUIRE(loaded_it != loaded_model.plates_custom_gcodes.end());
    REQUIRE(loaded_it->second.mode == CustomGCode::Mode::MultiExtruder);
    REQUIRE(loaded_it->second.gcodes.size() == gcode.gcodes.size());
    for (size_t i = 0; i < gcode.gcodes.size(); ++i) {
        const CustomGCode::Item &expected = gcode.gcodes[i];
        const CustomGCode::Item &actual   = loaded_it->second.gcodes[i];
        CHECK_THAT(actual.print_z, Catch::Matchers::WithinAbs(expected.print_z, 1e-6));
        CHECK(actual.type == expected.type);
        CHECK(actual.extruder == expected.extruder);
        CHECK(actual.color == expected.color);
        CHECK(actual.extra == expected.extra);
    }
}

TEST_CASE("BBS 3MF round-trips cut object identity and connector metadata", "[3mf][3MFCutMetadata]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(10., 10., 2.));
    ModelVolume *connector = object->add_volume(make_cube(2., 2., 1.), ModelVolumeType::NEGATIVE_VOLUME);
    object->add_instance();

    object->cut_id.init();
    object->cut_id.set_check_sum(0x1234);
    object->cut_id.increase_connectors_cnt(1);
    connector->cut_info = ModelVolume::CutInfo(CutConnectorType::Dowel, 0.125f, 0.25f, true);
    const ObjectID expected_cut_id = object->cut_id.id();

    ScopedTemporaryDir backup_dir("orca_3mf_cut_src");
    model.set_backup_path(backup_dir.string());
    ScopedTemporaryFile temp(".3mf");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    StoreParams store_params;
    store_params.path     = temp.string().c_str();
    store_params.model    = &model;
    store_params.config   = &config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup_dir("orca_3mf_cut_dst");
    REQUIRE(load_project(temp.string(), loaded_model, loaded_backup_dir));

    REQUIRE(loaded_model.objects.size() == 1);
    const ModelObject *loaded_object = loaded_model.objects.front();
    REQUIRE(loaded_object->is_cut());
    CHECK(loaded_object->cut_id.id() == expected_cut_id);
    CHECK(loaded_object->cut_id.check_sum() == 0x1234);
    CHECK(loaded_object->cut_id.connectors_cnt() == 1);
    REQUIRE(loaded_object->volumes.size() == 2);
    const ModelVolume *loaded_connector = nullptr;
    for (const ModelVolume *volume : loaded_object->volumes) {
        if (volume->is_cut_connector()) {
            loaded_connector = volume;
            break;
        }
    }
    REQUIRE(loaded_connector != nullptr);
    REQUIRE(loaded_connector->is_cut_connector());
    CHECK(loaded_connector->is_negative_volume());
    CHECK(loaded_connector->cut_info.connector_type == CutConnectorType::Dowel);
    CHECK_THAT(loaded_connector->cut_info.radius_tolerance, Catch::Matchers::WithinAbs(0.125, 1e-6));
    CHECK_THAT(loaded_connector->cut_info.height_tolerance, Catch::Matchers::WithinAbs(0.25, 1e-6));
    CHECK(loaded_connector->mesh().its.vertices.size() == connector->mesh().its.vertices.size());
}

TEST_CASE("BBS 3MF round-trips text configuration metadata", "[3MF][EmbossMetadata]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *volume = object->add_volume(make_cube(10., 10., 2.));
    object->add_instance();

    TextConfiguration expected;
    expected.text = "A<&\"'\tline\nnext";
    expected.style.name = "Round-trip style";
    expected.style.path = "MacFontDescriptor<&\"";
    expected.style.type = EmbossStyle::Type::wx_mac_font_descr;
    expected.style.prop.char_gap = 3;
    expected.style.prop.line_gap = -2;
    expected.style.prop.boldness = 0.25f;
    expected.style.prop.skew = -0.125f;
    expected.style.prop.collection_number = 4;
    expected.style.prop.per_glyph = true;
    expected.style.prop.align = {FontProp::HorizontalAlign::right, FontProp::VerticalAlign::bottom};
    expected.style.prop.size_in_mm = 12.5f;
    expected.style.prop.family = "Noto & Sans";
    expected.style.prop.face_name = "Display<>\"";
    expected.style.prop.style = "Italic & Oblique";
    expected.style.prop.weight = "Semi&Bold";
    volume->text_configuration = expected;

    ScopedTemporaryDir backup_dir("orca_text_metadata");
    model.set_backup_path(backup_dir.string());
    ScopedTemporaryFile temp(".3mf");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    StoreParams store_params;
    store_params.path = temp.string().c_str();
    store_params.model = &model;
    store_params.config = &config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    Model loaded_model;
    ScopedTemporaryDir loaded_backup_dir("orca_text_metadata_loaded");
    loaded_model.set_backup_path(loaded_backup_dir.string());
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs plates;
    std::vector<Preset*> project_presets;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset *preset : project_presets)
            delete preset;
    });
    bool is_bbl_3mf = false;
    bool is_orca_3mf = false;
    Semver file_version;
    REQUIRE(load_bbs_3mf(temp.string().c_str(), &loaded_config, &substitutions, &loaded_model, &plates,
                         &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));

    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 1);
    const ModelVolume &loaded_volume = *loaded_model.objects.front()->volumes.front();
    REQUIRE(loaded_volume.text_configuration.has_value());
    const TextConfiguration &actual = *loaded_volume.text_configuration;
    CHECK(actual.text == expected.text);
    CHECK(actual.style.name == expected.style.name);
    CHECK(actual.style.path == expected.style.path);
    CHECK(actual.style.type == expected.style.type);
    CHECK(actual.style.prop.char_gap == expected.style.prop.char_gap);
    CHECK(actual.style.prop.line_gap == expected.style.prop.line_gap);
    REQUIRE(expected.style.prop.boldness.has_value());
    REQUIRE(actual.style.prop.boldness.has_value());
    CHECK_THAT(*actual.style.prop.boldness, Catch::Matchers::WithinAbs(*expected.style.prop.boldness, 1e-5f));
    REQUIRE(expected.style.prop.skew.has_value());
    REQUIRE(actual.style.prop.skew.has_value());
    CHECK_THAT(*actual.style.prop.skew, Catch::Matchers::WithinAbs(*expected.style.prop.skew, 1e-5f));
    CHECK(actual.style.prop.collection_number == expected.style.prop.collection_number);
    CHECK(actual.style.prop.per_glyph == expected.style.prop.per_glyph);
    CHECK(actual.style.prop.align == expected.style.prop.align);
    CHECK_THAT(actual.style.prop.size_in_mm, Catch::Matchers::WithinAbs(expected.style.prop.size_in_mm, 1e-5f));
    CHECK(actual.style.prop.family == expected.style.prop.family);
    CHECK(actual.style.prop.face_name == expected.style.prop.face_name);
    CHECK(actual.style.prop.style == expected.style.prop.style);
    CHECK(actual.style.prop.weight == expected.style.prop.weight);
}

TEST_CASE("BBS 3MF round-trips an embossed SVG source and projection metadata", "[3MF][EmbossMetadata]")
{
    Model model;
    ModelObject *object = model.add_object();
    ModelVolume *volume = object->add_volume(make_cube(10., 10., 2.));
    object->add_instance();

    // Emboss SVGs are discovered by the importer only below the 3D/ folder.
    const std::string svg_entry = "3D/emboss_source.svg";
    const std::string svg_data = "<svg xmlns=\"http://www.w3.org/2000/svg\"><path d=\"M0 0h1v1z\"/></svg>";
    EmbossShape shape;
    shape.scale = 2.5;
    shape.projection.depth = 1.75;
    shape.projection.use_surface = true;
    shape.final_shape.is_healed = false;
    EmbossShape::SvgFile svg_file;
    svg_file.path = "source.svg";
    svg_file.path_in_3mf = svg_entry;
    svg_file.file_data = std::make_shared<std::string>(svg_data);
    shape.svg_file = std::move(svg_file);
    volume->emboss_shape = std::move(shape);

    ScopedTemporaryDir backup_dir("orca_emboss_metadata");
    model.set_backup_path(backup_dir.string());
    ScopedTemporaryFile temp(".3mf");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    StoreParams store_params;
    store_params.path = temp.string().c_str();
    store_params.model = &model;
    store_params.config = &config;
    store_params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(store_params));

    std::string archived_svg;
    REQUIRE(read_3mf_entry(temp.string(), archived_svg, svg_entry));
    CHECK(archived_svg == svg_data);

    Model loaded_model;
    ScopedTemporaryDir loaded_backup_dir("orca_emboss_metadata_loaded");
    loaded_model.set_backup_path(loaded_backup_dir.string());
    DynamicPrintConfig loaded_config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs plates;
    std::vector<Preset*> project_presets;
    ScopeGuard cleanup([&]() {
        release_PlateData_list(plates);
        for (Preset *preset : project_presets)
            delete preset;
    });
    bool is_bbl_3mf = false;
    bool is_orca_3mf = false;
    Semver file_version;
    REQUIRE(load_bbs_3mf(temp.string().c_str(), &loaded_config, &substitutions, &loaded_model, &plates,
                         &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));

    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 1);
    const ModelVolume &loaded_volume = *loaded_model.objects.front()->volumes.front();
    REQUIRE(loaded_volume.emboss_shape.has_value());
    const EmbossShape &actual = *loaded_volume.emboss_shape;
    CHECK_THAT(actual.scale, Catch::Matchers::WithinAbs(2.5, 1e-6));
    CHECK_THAT(actual.projection.depth, Catch::Matchers::WithinAbs(1.75, 1e-6));
    CHECK(actual.projection.use_surface);
    CHECK_FALSE(actual.final_shape.is_healed);
    REQUIRE(actual.svg_file.has_value());
    CHECK(actual.svg_file->path == "source.svg");
    CHECK(actual.svg_file->path_in_3mf == svg_entry);
    REQUIRE(actual.svg_file->file_data != nullptr);
    CHECK(*actual.svg_file->file_data == svg_data);
}

// Writes a single-entry zip whose central directory carries a zip64 record declaring an
// uncompressed size beyond what the 32-bit expat buffer API can take, while the deflated
// payload inflates to only ~64 KiB. Built by hand because miniz never writes a size that
// disagrees with the data.
static void write_zip_with_oversized_entry(const std::string& path, const std::string& entry)
{
    const std::string xml = "<?xml version=\"1.0\"?><!--" + std::string(65536, 'A') + "--><a/>";
    size_t comp_len = 0;
    void*  comp     = tdefl_compress_mem_to_heap(xml.data(), xml.size(), &comp_len, TDEFL_DEFAULT_MAX_PROBES);
    REQUIRE(comp != nullptr);
    const std::string deflated(static_cast<const char*>(comp), comp_len);
    mz_free(comp);
    const uint32_t crc = static_cast<uint32_t>(mz_crc32(MZ_CRC32_INIT, reinterpret_cast<const unsigned char*>(xml.data()), xml.size()));
    const uint64_t claimed_size = (uint64_t(1) << 32) + 16;

    std::string out;
    auto put = [&out](uint64_t v, int bytes) {
        for (int i = 0; i < bytes; ++i)
            out.push_back(static_cast<char>((v >> (8 * i)) & 0xFF));
    };
    // local file header, with the true sizes
    put(0x04034b50, 4); put(45, 2); put(0, 2); put(8, 2); put(0, 2); put(0, 2);
    put(crc, 4); put(deflated.size(), 4); put(xml.size(), 4); put(entry.size(), 2); put(0, 2);
    out += entry + deflated;
    // central directory header, sizes deferred to the zip64 extra field
    const size_t cd_offset = out.size();
    put(0x02014b50, 4); put(45, 2); put(45, 2); put(0, 2); put(8, 2); put(0, 2); put(0, 2);
    put(crc, 4); put(0xFFFFFFFF, 4); put(0xFFFFFFFF, 4); put(entry.size(), 2); put(20, 2);
    put(0, 2); put(0, 2); put(0, 2); put(0, 4); put(0, 4);
    out += entry;
    put(0x0001, 2); put(16, 2); put(claimed_size, 8); put(deflated.size(), 8);
    const size_t cd_size = out.size() - cd_offset;
    // end of central directory
    put(0x06054b50, 4); put(0, 2); put(0, 2); put(1, 2); put(1, 2);
    put(cd_size, 4); put(cd_offset, 4); put(0, 2);

    boost::nowide::ofstream f(path, std::ios::binary);
    REQUIRE(f.good());
    f.write(out.data(), static_cast<std::streamsize>(out.size()));
    REQUIRE(f.good());
}

TEST_CASE("3MF XML entries declaring more than an int can hold fail to load", "[3mf]") {
    ScopedTemporaryFile temp(".3mf");
    const std::string   path = temp.string();

    SECTION("BBS importer") {
        write_zip_with_oversized_entry(path, "_rels/.rels");
        Model                     model;
        DynamicPrintConfig        config;
        ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Enable};
        PlateDataPtrs             plates;
        std::vector<Preset*>      project_presets;
        bool                      is_bbl_3mf = false, is_orca_3mf = false;
        Semver                    file_version;
        bool                      loaded = true;
        REQUIRE_NOTHROW(loaded = load_bbs_3mf(path.c_str(), &config, &ctxt, &model, &plates, &project_presets, &is_bbl_3mf,
                                              &is_orca_3mf, &file_version, nullptr, LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
        CHECK_FALSE(loaded);
        release_PlateData_list(plates);
    }
    SECTION("PrusaSlicer importer") {
        write_zip_with_oversized_entry(path, "Metadata/Slic3r_PE_model.config");
        Model                     model;
        DynamicPrintConfig        config;
        ConfigSubstitutionContext ctxt{ForwardCompatibilitySubstitutionRule::Disable};
        bool                      loaded = true;
        REQUIRE_NOTHROW(loaded = load_3mf(path.c_str(), config, ctxt, &model, false));
        CHECK_FALSE(loaded);
    }
    SECTION("PrusaSlicer fingerprint probe") {
        write_zip_with_oversized_entry(path, "3D/3dmodel.model");
        PrusaFileParser parser;
        CHECK_FALSE(parser.check_3mf_from_prusa(path));
    }
}

namespace bbs_archive_contracts {

namespace {

struct LoadedProject
{
    DynamicPrintConfig config;
    PlateDataPtrs       plates;
    std::vector<Preset*> presets;
    bool                is_bbl{false};
    bool                is_orca{false};
    Semver              version;
    std::unique_ptr<ScopedTemporaryDir> backup;

    ~LoadedProject();
};

void release_presets(std::vector<Preset*>& presets)
{
    for (Preset *preset : presets)
        delete preset;
    presets.clear();
}

LoadedProject::~LoadedProject()
{
    release_PlateData_list(plates);
    release_presets(presets);
}

void make_model(Model& model, bool two_objects = false, bool assembly = false)
{
    ModelObject *base = model.add_object();
    base->name = "base-object";
    base->module_name = "base-module";
    ModelVolume *base_volume = base->add_volume(make_cube(10., 10., 10.));
    base_volume->name = "base-volume";
    base_volume->set_material_id("PLA");
    base->add_instance();
    ModelInstance *second = base->add_instance();
    second->set_offset({30., 5., 0.});
    second->set_rotation(Z, Geometry::deg2rad(17.));
    second->set_scaling_factor({1.1, 0.9, 1.0});

    if (assembly) {
        Transform3d assembled = Transform3d::Identity();
        assembled.linear() = Eigen::AngleAxisd(Geometry::deg2rad(23.), Vec3d::UnitZ()).toRotationMatrix();
        assembled.translation() = Vec3d(4., 6., 8.);
        second->set_assemble_from_transform(assembled);
        second->set_offset_to_assembly({1.5, -2.5, 0.75});
    }

    if (two_objects) {
        ModelObject *other = model.add_object();
        other->name = "second-object";
        other->module_name = "second-module";
        ModelVolume *other_volume = other->add_volume(make_cube(6., 8., 4.));
        other_volume->name = "second-volume";
        other_volume->set_material_id("PETG");
        other->add_instance({60., 10., 0.}, {1., 1., 1.}, {0., 0., Geometry::deg2rad(11.)}, {1., 1., 1.});
    }

    model.add_material("PLA")->attributes["filament_type"] = "PLA";
    model.add_material("PETG")->attributes["filament_type"] = "PETG";
}

void make_multi_volume_model(Model& model)
{
    ModelObject *object = model.add_object();
    object->name = "multi-volume";
    ModelVolume *part = object->add_volume(make_cube(12., 10., 8.));
    part->name = "printable-part";
    part->set_material_id("PLA");
    ModelVolume *negative = object->add_volume(make_cube(3., 3., 3.), ModelVolumeType::NEGATIVE_VOLUME);
    negative->name = "negative-cavity";
    negative->set_material_id("PETG");
    negative->set_offset({2., 2., 2.});
    object->add_instance();
    model.add_material("PLA")->attributes["filament_type"] = "PLA";
    model.add_material("PETG")->attributes["filament_type"] = "PETG";
}

void store_model(const std::string& path, Model& model, DynamicPrintConfig& config,
                 PlateDataPtrs& plates, SaveStrategy strategy = SaveStrategy::Zip64 | SaveStrategy::Silence,
                 Export3mfProgressFn callback = nullptr)
{
    StoreParams params;
    params.path = path;
    params.model = &model;
    params.config = &config;
    params.plate_data_list = plates;
    params.strategy = strategy;
    params.proFn = std::move(callback);
    REQUIRE(store_bbs_3mf(params));
}

bool load_model(const std::string& path, Model& model, LoadedProject& loaded,
                Import3mfProgressFn callback = nullptr)
{
    loaded.backup = std::make_unique<ScopedTemporaryDir>("orca_bbs_ext_load");
    model.set_backup_path(loaded.backup->string());
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    const bool result = load_bbs_3mf(path.c_str(), &loaded.config, &substitutions, &model, &loaded.plates,
                                     &loaded.presets, &loaded.is_bbl, &loaded.is_orca, &loaded.version,
                                     std::move(callback), LoadStrategy::LoadModel | LoadStrategy::LoadConfig);
    return result;
}

void make_plate(PlateData& plate, int index, const std::vector<std::pair<int, int>>& objects)
{
    plate.plate_index = index;
    plate.plate_name = "plate-" + std::to_string(index + 1);
    plate.locked = index == 1;
    plate.objects_and_instances = objects;
    plate.printer_model_id = "printer-" + std::to_string(index);
    plate.nozzle_diameters = "0.4;0.6";
    plate.nozzle_volume_types = "standard;high_flow";
    plate.gcode_prediction = "{\"time\":42}";
    plate.gcode_weight = "12.5";
    plate.toolpath_outside = index == 1;
    plate.is_support_used = index == 0;
}

} // namespace

TEST_CASE("BBS 3MF preserves multiple objects, instances, and transformed world geometry", "[3MF][Bbs3mfExtended][Geometry]")
{
    Model source;
    make_model(source, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(0.24));
    ScopedTemporaryDir backup("orca_bbs_ext_store");
    source.set_backup_path(backup.string());

    PlateData plate;
    make_plate(plate, 0, {{0, 0}, {0, 1}, {1, 0}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    Model loaded_model;
    LoadedProject loaded;
    REQUIRE(load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 2);
    REQUIRE(loaded_model.objects[0]->instances.size() == 2);
    REQUIRE(loaded_model.objects[1]->instances.size() == 1);
    CHECK(loaded_model.objects[0]->name == "base-object");
    CHECK(loaded_model.objects[1]->name == "second-object");
    const BoundingBoxf3 source_bbox = source.mesh().bounding_box();
    const BoundingBoxf3 loaded_bbox = loaded_model.mesh().bounding_box();
    CHECK_THAT(loaded_bbox.min.x(), Catch::Matchers::WithinAbs(source_bbox.min.x(), 1e-5));
    CHECK_THAT(loaded_bbox.min.z(), Catch::Matchers::WithinAbs(source_bbox.min.z(), 1e-5));
    CHECK_THAT(loaded_bbox.min.y(), Catch::Matchers::WithinAbs(source_bbox.min.y(), 1e-5));
    CHECK_THAT(loaded_bbox.max.x(), Catch::Matchers::WithinAbs(source_bbox.max.x(), 1e-5));
    CHECK_THAT(loaded_bbox.max.y(), Catch::Matchers::WithinAbs(source_bbox.max.y(), 1e-5));
    CHECK_THAT(loaded_bbox.max.z(), Catch::Matchers::WithinAbs(source_bbox.max.z(), 1e-5));
    CHECK(loaded_model.mesh().its.indices.size() == source.mesh().its.indices.size());
    REQUIRE(loaded.config.option<ConfigOptionFloat>("layer_height") != nullptr);
    CHECK_THAT(loaded.config.option<ConfigOptionFloat>("layer_height")->value, Catch::Matchers::WithinAbs(0.24, 1e-6));
}

TEST_CASE("BBS 3MF preserves assembly transform and assembly offset", "[3MF][Bbs3mfExtended][Assembly]")
{
    Model source;
    make_model(source, false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_assembly_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}, {0, 1}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    Model loaded_model;
    LoadedProject loaded;
    REQUIRE(load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->instances.size() == 2);
    ModelInstance *instance = loaded_model.objects.front()->instances[1];
    REQUIRE(instance->is_assemble_initialized());
    CHECK(instance->get_assemble_transformation().get_matrix().isApprox(
        source.objects.front()->instances[1]->get_assemble_transformation().get_matrix(), 1e-5));
    CHECK(instance->get_offset_to_assembly().isApprox(Vec3d(1.5, -2.5, 0.75), 1e-5));
}

TEST_CASE("BBS 3MF rejects an assembly item that names a missing object",
          "[3MF][Bbs3mfExtended][Assembly][Bbs3mfAssemblyErrors]")
{
    Model source;
    make_model(source, false, true);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_bad_assembly_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}, {0, 1}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    std::string settings;
    REQUIRE(read_3mf_entry(archive.string(), settings, "Metadata/model_settings.config"));
    const std::string item_prefix = "<assemble_item object_id=\"";
    const size_t item_start = settings.find(item_prefix);
    REQUIRE(item_start != std::string::npos);
    const size_t id_end = settings.find('"', item_start + item_prefix.size());
    REQUIRE(id_end != std::string::npos);
    REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
        settings.substr(item_start, id_end - item_start + 1), item_prefix + "999999\""));

    Model loaded_model;
    LoadedProject loaded;
    CHECK_FALSE(load_model(archive.string(), loaded_model, loaded));
}

TEST_CASE("BBS 3MF preserves model metadata and volume types", "[3MF][Bbs3mfExtended][Metadata]")
{
    Model source;
    make_multi_volume_model(source);
    source.model_info = std::make_shared<ModelInfo>();
    source.model_info->model_name = "Archive contract";
    source.model_info->origin = "designer-origin";
    source.model_info->description = "metadata survives XML escaping: <&>";
    source.model_info->metadata_items["custom-key"] = "custom-value";
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_metadata_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    Model loaded_model;
    LoadedProject loaded;
    REQUIRE(load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.model_info != nullptr);
    CHECK(loaded_model.model_info->model_name == "Archive contract");
    CHECK(loaded_model.model_info->origin == "designer-origin");
    CHECK(loaded_model.model_info->description == "metadata survives XML escaping: <&>");
    CHECK(loaded_model.model_info->metadata_items.at("custom-key") == "custom-value");
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 2);
    CHECK(loaded_model.objects.front()->volumes[0]->name == "printable-part");
    CHECK(loaded_model.objects.front()->volumes[1]->name == "negative-cavity");
    CHECK(loaded_model.objects.front()->volumes[1]->type() == ModelVolumeType::NEGATIVE_VOLUME);
}

TEST_CASE("BBS 3MF preserves multiple plate records and plate-scoped metadata", "[3MF][Bbs3mfExtended][Plates]")
{
    Model source;
    make_model(source, false);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_plate_store");
    source.set_backup_path(backup.string());
    auto plate0 = std::make_unique<PlateData>();
    auto plate1 = std::make_unique<PlateData>();
    make_plate(*plate0, 0, {{0, 0}});
    make_plate(*plate1, 1, {{0, 1}});
    PlateDataPtrs plates{plate0.get(), plate1.get()};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    Model loaded_model;
    LoadedProject loaded;
    REQUIRE(load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 2);
    CHECK(loaded.plates[0]->plate_name == "plate-1");
    CHECK(loaded.plates[1]->plate_name == "plate-2");
    CHECK_FALSE(loaded.plates[0]->locked);
    CHECK(loaded.plates[1]->locked);
    CHECK(loaded_model.objects.size() == 1);
}

TEST_CASE("BBS 3MF gates slice metadata while retaining every plate instance mapping", "[3MF][Bbs3mfExtended][Plates][SliceInfo]")
{
    Model source;
    make_model(source, false);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_slice_info_store");
    source.set_backup_path(backup.string());

    auto unsliced = std::make_unique<PlateData>();
    auto sliced = std::make_unique<PlateData>();
    make_plate(*unsliced, 0, {{0, 0}});
    make_plate(*sliced, 1, {{0, 1}});

    // These values deliberately exist on both source records.  Only the valid sliced
    // record is allowed into slice_info.config; the model-settings plate entries still
    // carry the object/instance mapping for both records.
    unsliced->printer_model_id = "must-not-be-written";
    unsliced->gcode_prediction = "999";
    unsliced->gcode_weight = "888";
    unsliced->toolpath_outside = true;
    unsliced->is_support_used = true;
    unsliced->is_sliced_valid = false;

    sliced->printer_model_id = "sliced-printer";
    // These attributes are consumed as numeric strings by the GUI and are written
    // verbatim into XML attributes, so keep the fixture XML-safe as well.
    sliced->gcode_prediction = "42";
    sliced->gcode_weight = "12.5";
    sliced->toolpath_outside = true;
    sliced->is_support_used = true;
    sliced->filament_maps = {1};
    sliced->is_sliced_valid = true;

    PlateDataPtrs plates{unsliced.get(), sliced.get()};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    Model loaded_model;
    LoadedProject loaded;
    REQUIRE(load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 2);

    const PlateData &loaded_unsliced = *loaded.plates[0];
    const PlateData &loaded_sliced = *loaded.plates[1];

    // No slice-info plate is emitted for an unsliced record, so all four slice-only
    // values remain their parser defaults even though they were populated in memory.
    CHECK(loaded_unsliced.printer_model_id.empty());
    CHECK(loaded_unsliced.gcode_prediction.empty());
    CHECK(loaded_unsliced.gcode_weight.empty());
    CHECK_FALSE(loaded_unsliced.toolpath_outside);
    CHECK_FALSE(loaded_unsliced.is_support_used);

    // The valid record gets the complete slice-only metadata set.
    CHECK(loaded_sliced.printer_model_id == "sliced-printer");
    CHECK(loaded_sliced.gcode_prediction == "42");
    CHECK(loaded_sliced.gcode_weight == "12.5");
    CHECK(loaded_sliced.toolpath_outside);
    CHECK(loaded_sliced.is_support_used);

    // Plate/instance membership is model-settings data, independent of the slice-info
    // validity gate.  The object id is stable and the selected source instance differs.
    REQUIRE(loaded_unsliced.obj_inst_map.size() == 1);
    REQUIRE(loaded_sliced.obj_inst_map.size() == 1);
    CHECK(loaded_unsliced.obj_inst_map.begin()->first == loaded_sliced.obj_inst_map.begin()->first);
    CHECK(loaded_unsliced.obj_inst_map.begin()->second.first == 0);
    CHECK(loaded_sliced.obj_inst_map.begin()->second.first == 1);

}

TEST_CASE("BBS 3MF import cancellation returns false at the public progress seam", "[3MF][Bbs3mfExtended][Cancellation]")
{
    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_cancel_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    bool called = false;
    Model loaded_model;
    LoadedProject loaded;
    const bool result = load_model(archive.string(), loaded_model, loaded,
        [&called](int stage, int, int, bool& cancel) {
            called = true;
            cancel = stage == IMPORT_STAGE_OPEN;
        });
    CHECK(called);
    CHECK_FALSE(result);
}

TEST_CASE("BBS 3MF export cancellation stops before writing a project", "[3MF][Bbs3mfExtended][Cancellation]")
{
    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_export_cancel");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    bool called = false;
    StoreParams params;
    params.path = archive.string();
    params.model = &source;
    params.config = &config;
    params.plate_data_list = plates;
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    params.proFn = [&called](int, int, int, bool& cancel) {
        called = true;
        cancel = true;
    };
    CHECK_FALSE(store_bbs_3mf(params));
    CHECK(called);
}

TEST_CASE("BBS 3MF import cancellation leaves plate output untouched at loading",
          "[3MF][Bbs3mfLateCancellation][Import]")
{
    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_late_import_cancel_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");
    store_model(archive.string(), source, config, plates);

    std::vector<int> stages;
    Model loaded_model;
    LoadedProject loaded;
    const bool result = load_model(archive.string(), loaded_model, loaded,
        [&stages](int stage, int, int, bool& cancel) {
            stages.push_back(stage);
            cancel = stage == IMPORT_STAGE_LOADING_PLATES;
        });

    CHECK_FALSE(result);
    REQUIRE(std::find(stages.begin(), stages.end(), IMPORT_STAGE_LOADING_PLATES) != stages.end());
    CHECK(loaded.plates.empty());
}

TEST_CASE("BBS 3MF export cancellation removes unfinished archives at late stages",
          "[3MF][Bbs3mfLateCancellation][Export]")
{
    const int target_stage = GENERATE(EXPORT_STAGE_ADD_MODELS, EXPORT_STAGE_FINISH);
    const bool preserve_existing = GENERATE(false, true);
    INFO("target export stage " << target_stage);
    INFO("pre-existing destination " << preserve_existing);

    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_late_export_cancel_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};

    ScopedTemporaryDir output_dir("orca_bbs_late_export_cancel_output");
    const std::filesystem::path output = std::filesystem::path(output_dir.string()) /
                                         ("cancelled-" + std::to_string(target_stage) + ".3mf");
    std::vector<int> stages;
    StoreParams params;
    params.path = output.string();
    params.model = &source;
    params.config = &config;
    params.plate_data_list = plates;
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;

    std::string existing_archive;
    if (preserve_existing) {
        REQUIRE(store_bbs_3mf(params));
        boost::nowide::ifstream input(output.string(), std::ios::binary);
        REQUIRE(input.good());
        existing_archive.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
        REQUIRE_FALSE(existing_archive.empty());
    } else {
        CHECK_FALSE(std::filesystem::exists(output));
    }

    params.proFn = [&stages, target_stage](int stage, int, int, bool& cancel) {
        stages.push_back(stage);
        cancel = stage == target_stage;
    };

    CHECK_FALSE(store_bbs_3mf(params));
    REQUIRE(std::find(stages.begin(), stages.end(), target_stage) != stages.end());
    CHECK_FALSE(std::filesystem::exists(output.string() + ".tmp"));

    if (preserve_existing) {
        boost::nowide::ifstream input(output.string(), std::ios::binary);
        REQUIRE(input.good());
        const std::string preserved{std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
        CHECK(preserved == existing_archive);
    } else {
        CHECK_FALSE(std::filesystem::exists(output));
    }
}

TEST_CASE("BBS 3MF rejects missing package relationships and malformed model XML", "[3MF][Bbs3mfExtended][Errors]")
{
    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_error_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};

    SECTION("missing root relationship")
    {
        ScopedTemporaryFile archive(".3mf");
        store_model(archive.string(), source, config, plates);
        REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string& name, std::string&) {
            if (name == "_rels/.rels") {
                name = "_rels/missing.rels";
                return true;
            }
            return false;
        }));
        Model loaded_model;
        LoadedProject loaded;
        CHECK_FALSE(load_model(archive.string(), loaded_model, loaded));
    }

    SECTION("malformed model XML")
    {
        ScopedTemporaryFile archive(".3mf");
        store_model(archive.string(), source, config, plates);
        REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string& name, std::string& data) {
            if (name != "3D/3dmodel.model")
                return false;
            const size_t end = data.rfind("</model>");
            if (end == std::string::npos)
                return false;
            data.replace(end, 8, "</broken>");
            return true;
        }));
        Model loaded_model;
        LoadedProject loaded;
        CHECK_FALSE(load_model(archive.string(), loaded_model, loaded));
    }
}

TEST_CASE("Published probe distinguishes published archives from ordinary and missing files", "[3MF][Bbs3mfExtended][Probe]")
{
    Model source;
    make_model(source);
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_ext_probe_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    make_plate(plate, 0, {{0, 0}});
    PlateDataPtrs plates{&plate};

    SECTION("ordinary project")
    {
        ScopedTemporaryFile archive(".3mf");
        store_model(archive.string(), source, config, plates);
        CHECK_FALSE(bbs_3mf_is_published(archive.string()));
    }
    SECTION("published project")
    {
        source.model_info = std::make_shared<ModelInfo>();
        source.model_info->metadata_items[ORCA_PUBLISHED_TAG] = "1";
        ScopedTemporaryFile archive(".3mf");
        store_model(archive.string(), source, config, plates);
        CHECK(bbs_3mf_is_published(archive.string()));
    }
    CHECK_FALSE(bbs_3mf_is_published("/definitely/missing/project.3mf"));
}

} // namespace bbs_archive_contracts

namespace archive_variant_contracts {
using namespace Slic3r;

namespace {

using BbsLoaded = bbs_archive_contracts::LoadedProject;

Model make_cube_model()
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "archive-contract-cube";
    object->add_volume(make_cube(10., 10., 10.));
    object->add_instance();
    return model;
}

void store_generic(const std::string &path, Model &model, DynamicPrintConfig *config = nullptr,
                   const ThumbnailData *thumbnail = nullptr)
{
    REQUIRE(store_3mf(path.c_str(), &model, config, false, thumbnail));
}

void store_bbs(const std::string &path, Model &model, DynamicPrintConfig &config,
               PlateData &plate, StoreParams *extra = nullptr)
{
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};
    StoreParams params;
    params.path = path;
    params.model = &model;
    params.config = &config;
    params.plate_data_list = {&plate};
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    if (extra != nullptr) {
        params.thumbnail_data = extra->thumbnail_data;
        params.no_light_thumbnail_data = extra->no_light_thumbnail_data;
        params.top_thumbnail_data = extra->top_thumbnail_data;
        params.pick_thumbnail_data = extra->pick_thumbnail_data;
        params.calibration_thumbnail_data = extra->calibration_thumbnail_data;
    }
    REQUIRE(store_bbs_3mf(params));
}

bool load_generic(const std::string &path, Model &model, DynamicPrintConfig &config,
                  bool check_version = false)
{
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    return load_3mf(path.c_str(), config, substitutions, &model, check_version);
}

bool load_bbs(const std::string &path, Model &model, BbsLoaded &loaded,
              LoadStrategy strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig)
{
    loaded.backup = std::make_unique<ScopedTemporaryDir>("orca_3mf_variants_load");
    model.set_backup_path(loaded.backup->string());
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    return load_bbs_3mf(path.c_str(), &loaded.config, &substitutions, &model, &loaded.plates,
                        &loaded.presets, &loaded.is_bbl, &loaded.is_orca, &loaded.version,
                        nullptr, strategy);
}

std::string read_file(const std::string &path)
{
    std::ifstream input(path, std::ios::binary);
    return std::string(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
}

bool replace_once(const std::string &path, const std::string &entry,
                  const std::string &from, const std::string &to)
{
    bool replaced = false;
    const bool rewritten = rewrite_3mf_entries(path, [&](std::string &name, std::string &data) {
        if (name != entry || replaced)
            return false;
        const size_t pos = data.find(from);
        if (pos == std::string::npos)
            return false;
        data.replace(pos, from.size(), to);
        replaced = true;
        return true;
    });
    return rewritten && replaced;
}

bool insert_before_model_end(const std::string &path, const std::string &metadata)
{
    bool inserted = false;
    const bool rewritten = rewrite_3mf_entries(path, [&](std::string &name, std::string &data) {
        if (name != "3D/3dmodel.model" || inserted)
            return false;
        const size_t pos = data.rfind("</model>");
        if (pos == std::string::npos)
            return false;
        data.insert(pos, metadata);
        inserted = true;
        return true;
    });
    return rewritten && inserted;
}

void store_bbs_base(const std::string &path, Model &model, DynamicPrintConfig &config, PlateData &plate)
{
    ScopedTemporaryDir backup("orca_3mf_variants_store");
    model.set_backup_path(backup.string());
    store_bbs(path, model, config, plate);
}

} // namespace

TEST_CASE("BBS 3MF restores filament, nozzle, and optimal-assignment sequences", "[3MFArchiveVariants][Bbs3mfVariants][Sequences]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    plate.filament_change_sequence = {1, 3, 0};
    plate.nozzle_change_sequence = {2, 1};
    plate.optimal_assignment = {1, 0, 2};
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->filament_change_sequence == std::vector<unsigned int>{1, 3, 0});
    CHECK(loaded.plates.front()->nozzle_change_sequence == std::vector<unsigned int>{2, 1});
    CHECK(loaded.plates.front()->optimal_assignment == std::vector<int>{1, 0, 2});
}

TEST_CASE("BBS 3MF maps legacy custom-gcode records without a type attribute", "[3MFArchiveVariants][Bbs3mfVariants][Legacy]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    const std::string legacy =
        "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"
        "<custom_gcodes_per_layer><plate><plate_info id=\"1\"/>"
        "<layer top_z=\"0.4\" extruder=\"2\" color=\"#abc\" gcode=\"M600\"/>"
        "<mode value=\"SingleExtruder\"/></plate></custom_gcodes_per_layer>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Metadata/custom_gcode_per_layer.xml", legacy}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.plates_custom_gcodes.size() == 1);
    const auto &info = loaded_model.plates_custom_gcodes.at(0);
    REQUIRE(info.gcodes.size() == 1);
    CHECK(info.gcodes.front().type == CustomGCode::ColorChange);
    CHECK(info.gcodes.front().extruder == 2);
    CHECK(info.gcodes.front().color == "#abc");
}

TEST_CASE("BBS 3MF rejects an invalid embedded preset without manufacturing a preset", "[3MFArchiveVariants][Bbs3mfVariants][Errors]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Metadata/process_settings_invalid.config", "this is not a config"}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    CHECK(loaded.presets.empty());
}

TEST_CASE("BBS 3MF loads a valid embedded print preset with its identity", "[3MFArchiveVariants][Bbs3mfVariants][EmbeddedPreset]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);

    DynamicPrintConfig embedded;
    embedded.set_key_value("print_settings_id", new ConfigOptionString("Embedded print contract"));
    embedded.set_key_value("layer_height", new ConfigOptionFloat(.31));
    ScopedTemporaryFile embedded_file(".json");
    embedded.save_to_json(embedded_file.string(), "Embedded print contract", "project", "0.0.0");
    const std::string embedded_json = read_file(embedded_file.string());
    REQUIRE(!embedded_json.empty());
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Metadata/process_settings_1.config", embedded_json}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.presets.size() == 1);
    const Preset *preset = loaded.presets.front();
    REQUIRE(preset != nullptr);
    CHECK(preset->type == Preset::TYPE_PRINT);
    CHECK(preset->name == "Embedded print contract");
    CHECK(preset->is_project_embedded);
    REQUIRE(preset->config.option<ConfigOptionFloat>("layer_height") != nullptr);
    CHECK_THAT(preset->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(.31, 1e-6));
}

TEST_CASE("BBS 3MF project config keeps valid JSON values when unrelated entries are present", "[3MFArchiveVariants][Bbs3mfVariants][ProjectConfig]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(.27));
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Metadata/archive-variant-unrelated.txt", "ignored by the project loader"}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.config.option<ConfigOptionFloat>("layer_height") != nullptr);
    CHECK_THAT(loaded.config.opt_float("layer_height"), Catch::Matchers::WithinAbs(.27, 1e-6));
}

TEST_CASE("BBS 3MF skips auxiliary extraction when the strategy omits LoadAuxiliary", "[3MFArchiveVariants][Bbs3mfVariants][Auxiliary][Strategy]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Auxiliaries/docs/readme.txt", "strategy-gated"}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    const std::filesystem::path extracted = std::filesystem::path(loaded_model.get_auxiliary_file_temp_path()) / "docs/readme.txt";
    CHECK_FALSE(std::filesystem::exists(extracted));
    CHECK(loaded_model.objects.size() == 1);
}

TEST_CASE("Generic 3MF applies micron units to mesh coordinates", "[3MFArchiveVariants][3MF][Units]")
{
    Model source = make_cube_model();
    const double source_width = source.mesh().bounding_box().max.x() - source.mesh().bounding_box().min.x();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    REQUIRE(replace_once(archive.string(), "3D/3dmodel.model", "unit=\"millimeter\"", "unit=\"micron\""));

    Model loaded;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), loaded, config));
    const BoundingBoxf3 bbox = loaded.mesh().bounding_box();
    CHECK_THAT(double(bbox.max.x() - bbox.min.x()), Catch::Matchers::WithinAbs(source_width * .001, 1e-5));
}

TEST_CASE("Generic 3MF defaults unknown units to millimeters", "[3MFArchiveVariants][3MF][Units]")
{
    Model source = make_cube_model();
    const BoundingBoxf3 expected = source.mesh().bounding_box();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    REQUIRE(replace_once(archive.string(), "3D/3dmodel.model", "unit=\"millimeter\"", "unit=\"unknown-unit\""));

    Model loaded;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), loaded, config));
    const BoundingBoxf3 actual = loaded.mesh().bounding_box();
    CHECK_THAT(double(actual.max.z() - actual.min.z()), Catch::Matchers::WithinAbs(expected.max.z() - expected.min.z(), 1e-5));
}

TEST_CASE("Generic 3MF check_version rejects a future file version", "[3MFArchiveVariants][3MF][Compatibility]")
{
    Model source = make_cube_model();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    REQUIRE(replace_once(archive.string(), "3D/3dmodel.model", "slic3rpe:Version3mf\">1", "slic3rpe:Version3mf\">99"));

    Model unchecked;
    DynamicPrintConfig unchecked_config;
    CHECK(load_generic(archive.string(), unchecked, unchecked_config, false));
    Model checked;
    DynamicPrintConfig checked_config;
    CHECK_THROWS(load_generic(archive.string(), checked, checked_config, true));
}

TEST_CASE("Generic 3MF rejects a future supports-painting version when checking compatibility", "[3MFArchiveVariants][3MF][Compatibility]")
{
    Model source = make_cube_model();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    Model control;
    DynamicPrintConfig control_config;
    REQUIRE(load_generic(archive.string(), control, control_config, false));
    REQUIRE(insert_before_model_end(archive.string(),
        "<metadata name=\"slic3rpe:FdmSupportsPaintingVersion\">99</metadata>"));
    Model loaded;
    DynamicPrintConfig config;
    CHECK_THROWS(load_generic(archive.string(), loaded, config, true));
}

TEST_CASE("Generic 3MF rejects a future seam-painting version when checking compatibility", "[3MFArchiveVariants][3MF][Compatibility]")
{
    Model source = make_cube_model();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    Model control;
    DynamicPrintConfig control_config;
    REQUIRE(load_generic(archive.string(), control, control_config, false));
    REQUIRE(insert_before_model_end(archive.string(),
        "<metadata name=\"slic3rpe:SeamPaintingVersion\">99</metadata>"));
    Model loaded;
    DynamicPrintConfig config;
    CHECK_THROWS(load_generic(archive.string(), loaded, config, true));
}

TEST_CASE("Generic 3MF rejects a future multi-material painting version when checking compatibility", "[3MFArchiveVariants][3MF][Compatibility]")
{
    Model source = make_cube_model();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    Model control;
    DynamicPrintConfig control_config;
    REQUIRE(load_generic(archive.string(), control, control_config, false));
    REQUIRE(insert_before_model_end(archive.string(),
        "<metadata name=\"slic3rpe:MmPaintingVersion\">99</metadata>"));
    Model loaded;
    DynamicPrintConfig config;
    CHECK_THROWS(load_generic(archive.string(), loaded, config, true));
}

TEST_CASE("Generic 3MF keeps a non-printable build item non-printable", "[3MFArchiveVariants][3MF][Build]")
{
    Model source = make_cube_model();
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source);
    REQUIRE(replace_once(archive.string(), "3D/3dmodel.model", "printable=\"1\"", "printable=\"0\""));

    Model loaded;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), loaded, config));
    REQUIRE(loaded.objects.size() == 1);
    REQUIRE(loaded.objects.front()->instances.size() == 1);
    CHECK_FALSE(loaded.objects.front()->instances.front()->printable);
}

TEST_CASE("Generic 3MF exports a thumbnail that is a valid PNG archive entry", "[3MFArchiveVariants][3MF][Thumbnail]")
{
    Model source = make_cube_model();
    ThumbnailData thumbnail;
    thumbnail.set(4, 4);
    for (size_t i = 0; i < thumbnail.pixels.size(); i += 4) {
        thumbnail.pixels[i + 0] = 0x12;
        thumbnail.pixels[i + 1] = 0x34;
        thumbnail.pixels[i + 2] = 0x56;
        thumbnail.pixels[i + 3] = 0xff;
    }
    ScopedTemporaryFile archive(".3mf");
    store_generic(archive.string(), source, nullptr, &thumbnail);
    std::string png;
    REQUIRE(read_3mf_entry(archive.string(), png, "Metadata/thumbnail.png"));
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{png.data(), png.size()}, decoded));
    REQUIRE(decoded.cols == 4);
    REQUIRE(decoded.rows == 4);
    REQUIRE(decoded.bytes_per_pixel == 4);
    REQUIRE(decoded.buf.size() == 4 * 4 * 4);
    for (size_t i = 0; i < decoded.buf.size(); i += 4) {
        CHECK(decoded.buf[i + 0] == 0x12);
        CHECK(decoded.buf[i + 1] == 0x34);
        CHECK(decoded.buf[i + 2] == 0x56);
        CHECK(decoded.buf[i + 3] == 0xff);
    }
}

TEST_CASE("BBS 3MF LoadModel strategy does not load project configuration", "[3MFArchiveVariants][Bbs3mfVariants][Strategy]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(.27));
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded, LoadStrategy::LoadModel));
    CHECK(loaded.config.empty());
    REQUIRE(loaded_model.objects.size() == 1);
}

TEST_CASE("BBS 3MF extracts auxiliary files into the model-owned temporary root", "[3MFArchiveVariants][Bbs3mfVariants][Auxiliary]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Auxiliaries/docs/readme.txt", "auxiliary-contract"}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path extracted = std::filesystem::path(loaded_model.get_auxiliary_file_temp_path()) / "docs/readme.txt";
    REQUIRE(std::filesystem::exists(extracted));
    std::ifstream input(extracted, std::ios::binary);
    std::string contents((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    CHECK(contents == "auxiliary-contract");
}

TEST_CASE("BBS 3MF ignores auxiliary path traversal entries", "[3MFArchiveVariants][Bbs3mfVariants][Auxiliary][Security]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
        {{"Auxiliaries/../escaped.txt", "must-not-extract"}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path auxiliary_root = loaded_model.get_auxiliary_file_temp_path();
    REQUIRE(auxiliary_root.parent_path() == std::filesystem::path(loaded.backup->string()));
    const std::filesystem::path escaped = std::filesystem::path(loaded_model.get_auxiliary_file_temp_path()).parent_path() / "escaped.txt";
    REQUIRE(escaped.parent_path() == std::filesystem::path(loaded.backup->string()));
    CHECK_FALSE(std::filesystem::exists(escaped));
}

TEST_CASE("BBS 3MF thumbnail accessor returns the stored plate PNG", "[3MFArchiveVariants][Bbs3mfVariants][Thumbnail]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ThumbnailData thumbnail;
    thumbnail.set(64, 64);
    std::fill(thumbnail.pixels.begin(), thumbnail.pixels.end(), 0xff);
    StoreParams extras;
    extras.thumbnail_data = {&thumbnail};
    ScopedTemporaryFile archive(".3mf");
    ScopedTemporaryDir backup("orca_3mf_variants_thumb_store");
    source.set_backup_path(backup.string());
    store_bbs(archive.string(), source, config, plate, &extras);

    const std::string png = bbs_3mf_get_thumbnail(archive.string().c_str());
    REQUIRE(png.size() > 8);
    CHECK(png.compare(0, 8, "\x89PNG\r\n\x1a\n", 8) == 0);
    // The signature alone accepts truncated or unrelated PNG payloads.  Decode
    // the returned accessor data and check every channel of the known fixture;
    // this also proves that the plate accessor returned the stored image rather
    // than a generated/default thumbnail.
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{png.data(), png.size()}, decoded));
    REQUIRE(decoded.cols == 64);
    REQUIRE(decoded.rows == 64);
    REQUIRE(decoded.bytes_per_pixel == 4);
    REQUIRE(decoded.buf.size() == 4 * 64 * 64);
    CHECK(std::all_of(decoded.buf.begin(), decoded.buf.end(), [](unsigned char channel) {
        return channel == 0xff;
    }));
}

TEST_CASE("BBS 3MF rejects malformed project JSON without accepting a fake config", "[3MFArchiveVariants][Bbs3mfVariants][Errors]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    REQUIRE(replace_once(archive.string(), "Metadata/project_settings.config",
                         "{", "not-json"));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    CHECK(loaded.config.empty());
}

} // namespace archive_variant_contracts

namespace archive_variant_contracts {

// The central-directory name and Info-ZIP Unicode Path extra field can differ.
// Keep both explicit so the outer filename filter cannot stand in for the
// containment check performed after decoding the extra field.
void append_extra_u16(std::string &extra, uint16_t value)
{
    extra.push_back(static_cast<char>(value & 0xff));
    extra.push_back(static_cast<char>(value >> 8));
}

std::string unicode_path_extra(const std::string &stored_name, const std::string &decoded_name,
                               bool valid_crc = true, unsigned char version = 1)
{
    REQUIRE(decoded_name.size() <= std::numeric_limits<uint16_t>::max() - 5);
    std::string extra;
    append_extra_u16(extra, 0x7075); // Info-ZIP Unicode Path extra field.
    append_extra_u16(extra, static_cast<uint16_t>(5 + decoded_name.size()));
    extra.push_back(static_cast<char>(version));
    const mz_ulong crc = mz_crc32(0, reinterpret_cast<const unsigned char *>(stored_name.data()), stored_name.size()) ^ (valid_crc ? 0u : 1u);
    for (unsigned int byte = 0; byte < 4; ++byte)
        extra.push_back(static_cast<char>((crc >> (8 * byte)) & 0xff));
    return extra + decoded_name;
}

void append_entry_with_extra(const std::string &archive_path, const std::string &stored_name,
                             const std::string &payload, const std::string &extra)
{
    REQUIRE(extra.size() <= std::numeric_limits<uint16_t>::max());
    mz_zip_archive zip{};
    REQUIRE(mz_zip_reader_init_file(&zip, archive_path.c_str(), 0));
    ScopeGuard close_zip([&zip] { mz_zip_end(&zip); });
    REQUIRE(mz_zip_writer_init_from_reader(&zip, archive_path.c_str()));
    REQUIRE(mz_zip_writer_add_mem_ex_v2(&zip, stored_name.c_str(), payload.data(), payload.size(),
        nullptr, 0, MZ_DEFAULT_LEVEL | MZ_ZIP_FLAG_ASCII_FILENAME, 0, 0, nullptr,
        extra.data(), static_cast<mz_uint>(extra.size()), extra.data(), static_cast<mz_uint>(extra.size())));
    REQUIRE(mz_zip_writer_finalize_archive(&zip));
}

void append_unicode_auxiliary(const std::string &archive_path, const std::string &decoded_name,
                              bool trailing_field = true, bool valid_crc = true, unsigned char version = 1,
                              size_t prefix_extra_bytes = 0)
{
    const std::string stored_name = "Auxiliaries/docs/ascii.txt";
    std::string extra;
    REQUIRE(prefix_extra_bytes <= std::numeric_limits<uint16_t>::max());
    if (prefix_extra_bytes != 0) {
        // Padding exercises a large extra-field payload with short path components.
        append_extra_u16(extra, 0xcafe);
        append_extra_u16(extra, static_cast<uint16_t>(prefix_extra_bytes));
        extra.append(prefix_extra_bytes, 'x');
    }
    extra += unicode_path_extra(stored_name, decoded_name, valid_crc, version);
    if (trailing_field) {
        append_extra_u16(extra, 0xcafe); // Empty JAR marker after the Unicode field.
        append_extra_u16(extra, 0);
    }
    append_entry_with_extra(archive_path, stored_name, "unicode-auxiliary-contract", extra);
}

void append_truncated_unicode_auxiliary(const std::string &archive_path)
{
    const std::string stored_name = "Auxiliaries/docs/ascii.txt";
    std::string extra = unicode_path_extra(stored_name, "short");
    extra[2] = 32; // Declare more bytes than follow the four-byte field header.
    append_entry_with_extra(archive_path, stored_name, "truncated-unicode-contract", extra);
}

void add_unicode_model_alias(const std::string &archive_path)
{
    const std::string stored_name = "3D/model-payload.bin";
    const std::string decoded_name = "3D/3dmodel.model";
    std::string model_data;
    REQUIRE(rewrite_3mf_entries(archive_path, [&](std::string &name, std::string &data) {
        if (name != decoded_name)
            return false;
        model_data = data;
        name = "3D/original-payload.bin";
        return true;
    }));
    REQUIRE_FALSE(model_data.empty());
    // Filename lookup misses the referenced model path; only the Unicode
    // metadata on this distinct entry resolves it.
    append_entry_with_extra(archive_path, stored_name, model_data,
                            unicode_path_extra(stored_name, decoded_name));
}

TEST_CASE("BBS 3MF validates auxiliary paths after decoding Unicode metadata",
          "[3MFArchiveVariants][Bbs3mfVariants][AuxiliaryUnicode][Security]")
{
    const bool traversal = GENERATE(false, true);
    const std::string decoded_name = traversal ? "Auxiliaries/../escaped-unicode.txt" : "Auxiliaries/docs/unicode-é.txt";
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    append_unicode_auxiliary(archive.string(), decoded_name);

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path auxiliary_root = loaded_model.get_auxiliary_file_temp_path();
    REQUIRE(auxiliary_root.parent_path() == std::filesystem::path(loaded.backup->string()));
    CHECK_FALSE(std::filesystem::exists(auxiliary_root / "docs/ascii.txt"));
    if (traversal) {
        const std::filesystem::path escaped = auxiliary_root.parent_path() / "escaped-unicode.txt";
        REQUIRE(escaped.parent_path() == std::filesystem::path(loaded.backup->string()));
        CHECK_FALSE(std::filesystem::exists(escaped));
    } else {
        const std::filesystem::path extracted = auxiliary_root / "docs/unicode-é.txt";
        REQUIRE(std::filesystem::exists(extracted));
        CHECK(read_file(extracted.string()) == "unicode-auxiliary-contract");
    }
}

} // namespace archive_variant_contracts

namespace archive_variant_contracts {
TEST_CASE("BBS 3MF decodes final and long Unicode path metadata", "[3MFArchiveVariants][AuxiliaryUnicodeMetadata]")
{
    const bool trailing_field = GENERATE(false, true);
    const bool long_name = GENERATE(false, true);
    CAPTURE(trailing_field, long_name);
    const std::string leaf = long_name ? std::string(130, 'a') + "-é.txt" : "unicode-é.txt";
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    append_unicode_auxiliary(archive.string(), "Auxiliaries/docs/" + leaf, trailing_field);
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path root = loaded_model.get_auxiliary_file_temp_path();
    REQUIRE(std::filesystem::exists(root / "docs" / leaf));
    CHECK(read_file((root / "docs" / leaf).string()) == "unicode-auxiliary-contract");
    CHECK_FALSE(std::filesystem::exists(root / "docs/ascii.txt"));
}

TEST_CASE("BBS 3MF ignores Unicode metadata with an invalid checksum or version", "[3MFArchiveVariants][AuxiliaryUnicodeMetadata]")
{
    const bool bad_crc = GENERATE(false, true);
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    append_unicode_auxiliary(archive.string(), "Auxiliaries/docs/untrusted.txt", true,
                             !bad_crc, bad_crc ? 1 : 2);
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path root = loaded_model.get_auxiliary_file_temp_path();
    REQUIRE(std::filesystem::exists(root / "docs/ascii.txt"));
    CHECK(read_file((root / "docs/ascii.txt").string()) == "unicode-auxiliary-contract");
    CHECK_FALSE(std::filesystem::exists(root / "docs/untrusted.txt"));
}

TEST_CASE("BBS 3MF reads complete final Unicode metadata beyond 1024 bytes",
          "[3MFArchiveVariants][AuxiliaryUnicodeMetadata][UnicodeExtraFraming]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    append_unicode_auxiliary(archive.string(), "Auxiliaries/docs/unicode-é.txt", false, true, 1, 1020);

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path root = loaded_model.get_auxiliary_file_temp_path();
    const std::filesystem::path extracted = root / "docs/unicode-é.txt";
    REQUIRE(std::filesystem::exists(extracted));
    CHECK(read_file(extracted.string()) == "unicode-auxiliary-contract");
    CHECK_FALSE(std::filesystem::exists(root / "docs/ascii.txt"));
}

TEST_CASE("BBS 3MF resolves the model part through Unicode metadata",
          "[3MFArchiveVariants][AuxiliaryUnicodeMetadata][UnicodeExtraFraming]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    add_unicode_model_alias(archive.string());

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 1);
    REQUIRE(loaded_model.objects.front()->instances.size() == 1);
    TriangleMesh mesh = loaded_model.objects.front()->volumes.front()->mesh();
    CHECK(mesh.facets_count() == 12);
    CHECK_THAT(mesh.volume(), Catch::Matchers::WithinAbs(1000., 1e-4));
}

TEST_CASE("BBS 3MF falls back on a truncated Unicode metadata field",
          "[3MFArchiveVariants][AuxiliaryUnicodeMetadata][UnicodeExtraFraming]")
{
    Model source = make_cube_model();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_bbs_base(archive.string(), source, config, plate);
    append_truncated_unicode_auxiliary(archive.string());

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded,
                     LoadStrategy::LoadModel | LoadStrategy::LoadConfig | LoadStrategy::LoadAuxiliary));
    const std::filesystem::path root = loaded_model.get_auxiliary_file_temp_path();
    const std::filesystem::path extracted = root / "docs/ascii.txt";
    REQUIRE(std::filesystem::exists(extracted));
    CHECK(read_file(extracted.string()) == "truncated-unicode-contract");
    CHECK_FALSE(std::filesystem::exists(root / "docs/short"));
}
} // namespace archive_variant_contracts


namespace stream_archive_contracts {
using namespace bbs_archive_contracts;

namespace {
struct StreamArchive {
    ScopedTemporaryDir backup{"orca_stream_store"};
    ScopedTemporaryFile file{".3mf"};
    Model model;
    DynamicPrintConfig config{DynamicPrintConfig::full_print_config()};
    PlateData plate;

    explicit StreamArchive(bool image_paths = false)
    {
        make_model(model);
        model.set_backup_path(backup.string());
        model.model_info = std::make_shared<ModelInfo>();
        model.model_info->model_name = "Stream <model>";
        model.model_info->description = "Metadata & configuration";
        config.set_deserialize_strict({{"layer_height", 0.24}, {"wall_loops", 3}});
        make_plate(plate, 0, {{0, 0}, {0, 1}});
        plate.locked = true;
        if (image_paths) {
            plate.pick_file = "source-pick.png";
            plate.pattern_bbox_file = "source-bounds.json";
        }
        PlateDataPtrs plates{&plate};
        store_model(file.string(), model, config, plates);
    }
};

bool load_stream(const std::string &bytes, Model &model, LoadedProject &loaded)
{
    // Stream import may materialize the project configuration and thumbnail into
    // the destination model backup. Give the destination an owned writable path
    // just as the file importer helper does.
    loaded.backup = std::make_unique<ScopedTemporaryDir>("orca_stream_load");
    model.set_backup_path(loaded.backup->string());
    std::istringstream stream(bytes, std::ios::in | std::ios::binary);
    return load_gcode_3mf_from_stream(stream, &loaded.config, &model, &loaded.plates, &loaded.version);
}

bool load_stream_file(const StreamArchive &archive, Model &model, LoadedProject &loaded)
{
    return load_stream(archive_variant_contracts::read_file(archive.file.string()), model, loaded);
}
} // namespace

TEST_CASE("G-code archive streams preserve project metadata and configuration", "[3MF][3MFStream]")
{
    StreamArchive archive;
    Model model;
    LoadedProject loaded;
    REQUIRE(load_stream_file(archive, model, loaded));
    REQUIRE(model.model_info != nullptr);
    CHECK(model.model_info->model_name == "Stream <model>");
    CHECK(model.model_info->description == "Metadata & configuration");
    REQUIRE(loaded.config.option<ConfigOptionFloat>("layer_height") != nullptr);
    CHECK_THAT(loaded.config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-6));
    CHECK(loaded.config.opt_int("wall_loops") == 3);
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->plate_index == 0);
    CHECK(loaded.plates.front()->locked);
}

TEST_CASE("G-code archive streams preserve plate image and bounding-box paths", "[3MF][3MFStream]")
{
    const bool populated = GENERATE(false, true);
    StreamArchive archive(populated);
    Model model;
    LoadedProject loaded;
    REQUIRE(load_stream_file(archive, model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->pick_file == (populated ? "Metadata/pick_1.png" : ""));
    CHECK(loaded.plates.front()->pattern_bbox_file == (populated ? "Metadata/plate_1.json" : ""));
}

TEST_CASE("G-code archive streams reject missing relationships and model parts", "[3MF][3MFStream]")
{
    const std::string missing = GENERATE(std::string("_rels/.rels"), std::string("3D/3dmodel.model"));
    StreamArchive archive;
    REQUIRE(rewrite_3mf_entries(archive.file.string(), [&](std::string &name, std::string &) {
        if (name != missing) return false;
        name = "removed/" + name;
        return true;
    }));
    Model model;
    LoadedProject loaded;
    CHECK_FALSE(load_stream_file(archive, model, loaded));
    CHECK(loaded.plates.empty());
}

TEST_CASE("G-code archive streams reject malformed model and plate XML", "[3MF][3MFStream]")
{
    const std::string broken = GENERATE(std::string("3D/3dmodel.model"), std::string("Metadata/model_settings.config"));
    StreamArchive archive;
    REQUIRE(rewrite_3mf_entries(archive.file.string(), [&](std::string &name, std::string &data) {
        if (name != broken) return false;
        data = "<broken";
        return true;
    }));
    Model model;
    LoadedProject loaded;
    CHECK_FALSE(load_stream_file(archive, model, loaded));
    CHECK(loaded.plates.empty());
}

TEST_CASE("G-code archive streams reject empty and non-ZIP input", "[3MF][3MFStream]")
{
    const std::string bytes = GENERATE(std::string(), std::string("this is not a ZIP file"));
    Model model;
    LoadedProject loaded;
    CHECK_FALSE(load_stream(bytes, model, loaded));
    CHECK(model.objects.empty());
    CHECK(loaded.plates.empty());
}

TEST_CASE("G-code archive streams retain the encoded plate thumbnail bytes", "[3MF][3MFStream]")
{
    StreamArchive archive;
    REQUIRE(rewrite_3mf_entries(archive.file.string(), [](std::string &name, std::string &data) {
        if (name != "Metadata/model_settings.config") return false;
        const auto end = data.find("</plate>");
        REQUIRE(end != std::string::npos);
        data.insert(end, "<metadata key=\"thumbnail_file\" value=\"Metadata/contract-thumb.png\"/>");
        return true;
    }, {{"Metadata/contract-thumb.png", std::string("\x89PNG\r\n\x1a\n", 8)}}));
    Model model;
    LoadedProject loaded;
    REQUIRE(load_stream_file(archive, model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->thumbnail_file == "Metadata/contract-thumb.png");
    const auto &bytes = loaded.plates.front()->plate_thumbnail.pixels;
    CHECK(std::string(bytes.begin(), bytes.end()) == std::string("\x89PNG\r\n\x1a\n", 8));
}
} // namespace stream_archive_contracts


namespace legacy_3mf_contracts {

using namespace Slic3r;

namespace {

std::string transform_string(double tx = 0., double ty = 0., double tz = 0.)
{
    return "1 0 0 0 1 0 0 0 1 " + std::to_string(tx) + " " + std::to_string(ty) + " " + std::to_string(tz);
}

std::string cube_model_xml(const std::string &application = "ThirdParty",
                           const std::string &build = "  <item objectid=\"1\" printable=\"1\"/>\n",
                           bool reverse_winding = false,
                           const std::string &object_name = "Cube")
{
    static const int vertices[][3] = {
        {0, 0, 0}, {10, 0, 0}, {10, 10, 0}, {0, 10, 0},
        {0, 0, 10}, {10, 0, 10}, {10, 10, 10}, {0, 10, 10}
    };
    static const int triangles[][3] = {
        {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7},
        {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
        {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7}
    };

    std::string xml =
        "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
        "<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">\n";
    if (!application.empty())
        xml += " <metadata name=\"Application\">" + application + "</metadata>\n";
    xml += " <resources>\n  <object id=\"1\" type=\"model\" name=\"" + object_name + "\">\n"
           "   <mesh>\n    <vertices>\n";
    for (const auto &vertex : vertices)
        xml += "     <vertex x=\"" + std::to_string(vertex[0]) + "\" y=\"" + std::to_string(vertex[1]) + "\" z=\"" + std::to_string(vertex[2]) + "\"/>\n";
    xml += "    </vertices>\n    <triangles>\n";
    for (const auto &triangle : triangles) {
        const int v1 = triangle[0];
        const int v2 = reverse_winding ? triangle[2] : triangle[1];
        const int v3 = reverse_winding ? triangle[1] : triangle[2];
        xml += "     <triangle v1=\"" + std::to_string(v1) + "\" v2=\"" + std::to_string(v2) + "\" v3=\"" + std::to_string(v3) + "\"/>\n";
    }
    xml += "    </triangles>\n   </mesh>\n  </object>\n </resources>\n <build>\n";
    xml += build;
    xml += " </build>\n</model>\n";
    return xml;
}

bool write_archive(const std::string &path, const std::string &model_xml,
                   const std::string &model_config = "")
{
    try {
        Zipper archive(path);
        archive.add_entry("3D/3dmodel.model", model_xml.data(), model_xml.size());
        if (!model_config.empty())
            archive.add_entry("Metadata/Slic3r_PE_model.config", model_config.data(), model_config.size());
        archive.finalize();
        return true;
    } catch (...) {
        return false;
    }
}

bool load_generic(const std::string &path, Model &model, DynamicPrintConfig &config,
                  bool check_version = false)
{
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    return load_3mf(path.c_str(), config, substitutions, &model, check_version);
}

Model make_cube_model()
{
    Model model;
    ModelObject *object = model.add_object();
    object->name = "round-trip cube";
    object->add_volume(make_cube(10., 10., 10.));
    object->add_instance();
    return model;
}

} // namespace

TEST_CASE("Legacy 3MF public entry points reject null and missing paths", "[Legacy3MF][Errors]")
{
    Model model;
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    CHECK_FALSE(load_3mf(nullptr, config, substitutions, &model, false));
    ScopedTemporaryDir directory;
    const std::string missing = (directory.path() / "missing" / "model.3mf").string();
    CHECK_FALSE(load_3mf(missing.c_str(), config, substitutions, &model, false));
    CHECK_FALSE(store_3mf(nullptr, &model, nullptr, false));
    CHECK_FALSE(store_3mf(missing.c_str(), &model, nullptr, false));
}

TEST_CASE("Legacy Prusa file parser identifies the generator metadata", "[Legacy3MF][Metadata]")
{
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), cube_model_xml("PrusaSlicer-2.7.0")));
    PrusaFileParser parser;
    CHECK(parser.check_3mf_from_prusa(archive.string()));

    ScopedTemporaryFile third_party(".3mf");
    REQUIRE(write_archive(third_party.string(), cube_model_xml("OtherSlicer-1.0")));
    PrusaFileParser other_parser;
    CHECK_FALSE(other_parser.check_3mf_from_prusa(third_party.string()));
}

TEST_CASE("Legacy Prusa file parser rejects a non-archive and an archive without a model part", "[Legacy3MF][Errors]")
{
    ScopedTemporaryFile plain(".3mf");
    {
        boost::nowide::ofstream output(plain.string(), std::ios::binary | std::ios::trunc);
        REQUIRE(output.good());
        output << "not a zip archive";
        output.flush();
        REQUIRE(output.good());
    }
    PrusaFileParser parser;
    CHECK_FALSE(parser.check_3mf_from_prusa(plain.string()));

    ScopedTemporaryFile empty_zip(".3mf");
    {
        Zipper archive(empty_zip.string());
        archive.add_entry("README.txt", "no model", 8);
        archive.finalize();
    }
    PrusaFileParser empty_parser;
    CHECK_FALSE(empty_parser.check_3mf_from_prusa(empty_zip.string()));
}

TEST_CASE("Legacy importer scales coordinates and defaults absent vertex attributes", "[Legacy3MF][Geometry]")
{
    const auto [unit, scale] = GENERATE(
        std::pair<const char *, double>{"micron", 0.001},
        std::pair<const char *, double>{"millimeter", 1.},
        std::pair<const char *, double>{"centimeter", 10.},
        std::pair<const char *, double>{"inch", 25.4},
        std::pair<const char *, double>{"foot", 304.8},
        std::pair<const char *, double>{"meter", 1000.});
    CAPTURE(unit);
    const std::string xml =
        "<model unit=\"" + std::string(unit) + "\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">"
        "<resources><object id=\"1\" type=\"model\"><mesh><vertices>"
        "<vertex x=\"0\" y=\"0\"/><vertex x=\"1\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"1\" z=\"0\"/>"
        "</vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object></resources>"
        "<build><item objectid=\"1\"/></build></model>";
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), xml));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    const BoundingBoxf3 bbox = model.objects.front()->volumes.front()->mesh().bounding_box();
    CHECK_THAT(double(bbox.max.x() - bbox.min.x()), Catch::Matchers::WithinAbs(scale, std::max(1e-7, scale * 1e-6)));
    CHECK_THAT(double(bbox.max.y() - bbox.min.y()), Catch::Matchers::WithinAbs(scale, std::max(1e-7, scale * 1e-6)));
    CHECK_THAT(double(bbox.max.z() - bbox.min.z()), Catch::Matchers::WithinAbs(0., 1e-5));
}

TEST_CASE("Legacy importer expands a component alias with its transform", "[Legacy3MF][Geometry][Components]")
{
    const std::string model_xml =
        "<model unit=\"millimeter\" xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\">"
        "<resources><object id=\"1\" type=\"model\" name=\"part\"><mesh><vertices>"
        "<vertex x=\"0\" y=\"0\" z=\"0\"/><vertex x=\"2\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"2\" z=\"0\"/>"
        "</vertices><triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object>"
        "<object id=\"2\" type=\"model\"><components><component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 10 20 30\"/></components></object></resources>"
        "<build><item objectid=\"2\" printable=\"1\"/></build></model>";
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), model_xml));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    const BoundingBoxf3 bbox = model.mesh().bounding_box();
    CHECK_THAT(double(bbox.min.x()), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK_THAT(double(bbox.min.y()), Catch::Matchers::WithinAbs(20., 1e-5));
    CHECK_THAT(double(bbox.min.z()), Catch::Matchers::WithinAbs(30., 1e-5));
}

TEST_CASE("Legacy non-Prusa multi-instance files split independent model objects", "[Legacy3MF][Components]")
{
    const std::string items =
        "  <item objectid=\"1\" transform=\"" + transform_string(0., 0., 0.) + "\"/>\n"
        "  <item objectid=\"1\" transform=\"" + transform_string(20., 0., 0.) + "\"/>\n";
    std::string xml = cube_model_xml("", items);
    const size_t resources = xml.find(" <resources>");
    REQUIRE(resources != std::string::npos);
    xml.insert(resources, " <metadata name=\"slic3rpe:Version3mf\">1</metadata>\n");
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), xml));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 2);
    CHECK(model.objects[0]->instances.size() == 1);
    CHECK(model.objects[1]->instances.size() == 1);
    const BoundingBoxf3 bbox = model.mesh().bounding_box();
    CHECK_THAT(double(bbox.max.x() - bbox.min.x()), Catch::Matchers::WithinAbs(30., 1e-5));
}

TEST_CASE("BBS importer splits a foreign multi-instance archive into independent objects",
          "[3MF][Bbs3mfForeignMultiInstance]")
{
    // store_3mf supplies the normal package relationships and model part. Rewrite only its
    // Application marker so the public BBS importer takes the foreign-project path.
    Model source = make_cube_model();
    source.objects.front()->add_instance()->set_offset(Vec3d(20., 0., 0.));
    const double expected_volume = its_volume(source.objects.front()->raw_mesh().its);
    REQUIRE(expected_volume > 0.);

    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));
    REQUIRE(replace_in_3mf_entry(archive.string(), "3dmodel.model", "OrcaSlicer-", "ThirdParty-"));

    Model loaded;
    ScopedTemporaryDir backup("orca_bbs_foreign_multi_load");
    loaded.set_backup_path(backup.string());
    DynamicPrintConfig config;
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    PlateDataPtrs plates;
    std::vector<Preset*> project_presets;
    bool is_bbl_3mf = false;
    bool is_orca_3mf = false;
    Semver file_version;
    ScopeGuard cleanup([&] {
        // Detach before the temporary backup directory is destroyed; Model::~Model otherwise
        // queues cleanup against a path whose owner has already gone out of scope.
        loaded.set_backup_path("detach");
        release_PlateData_list(plates);
        for (Preset *preset : project_presets)
            delete preset;
    });

    REQUIRE(load_bbs_3mf(archive.string().c_str(), &config, &substitutions, &loaded, &plates,
                         &project_presets, &is_bbl_3mf, &is_orca_3mf, &file_version, nullptr,
                         LoadStrategy::LoadModel | LoadStrategy::LoadConfig));
    CHECK_FALSE(is_bbl_3mf);
    CHECK_FALSE(is_orca_3mf);
    REQUIRE(loaded.objects.size() == 2);

    for (const ModelObject *object : loaded.objects) {
        REQUIRE(object != nullptr);
        REQUIRE(object->instances.size() == 1);
        REQUIRE(!object->volumes.empty());
        CHECK_THAT(its_volume(object->raw_mesh().its), Catch::Matchers::WithinAbs(expected_volume, 1e-5));
    }

    // The two source instances span [0, 10] and [20, 30] in world X.  This catches a split
    // that preserves object count but loses an instance transform.
    const BoundingBoxf3 world_box = loaded.mesh().bounding_box();
    CHECK_THAT(double(world_box.max.x() - world_box.min.x()), Catch::Matchers::WithinAbs(30., 1e-5));
}


TEST_CASE("Legacy importer preserves non-printable and transformed build items", "[Legacy3MF][Geometry]")
{
    const std::string items =
        "  <item objectid=\"1\" printable=\"0\" transform=\"" + transform_string(7., 8., 9.) + "\"/>\n";
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), cube_model_xml("ThirdParty", items)));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->instances.size() == 1);
    CHECK_FALSE(model.objects.front()->instances.front()->printable);
    const BoundingBoxf3 bbox = model.mesh().bounding_box();
    CHECK_THAT(double(bbox.min.x()), Catch::Matchers::WithinAbs(7., 1e-5));
    CHECK_THAT(double(bbox.max.z()), Catch::Matchers::WithinAbs(19., 1e-5));
}

TEST_CASE("Legacy model configuration restores volume identity, type, and source fields", "[Legacy3MF][Metadata]")
{
    Model source = make_cube_model();
    ModelVolume *volume = source.objects.front()->volumes.front();
    source.objects.front()->name = "Configured object";
    volume->name = "Configured volume";
    volume->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    volume->source.input_file = "/tmp/original/source.stl";
    volume->source.object_idx = 4;
    volume->source.volume_idx = 2;
    volume->source.mesh_offset = Vec3d(1.25, -2.5, 3.75);

    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));
    Model loaded;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), loaded, config));
    REQUIRE(loaded.objects.size() == 1);
    REQUIRE(loaded.objects.front()->volumes.size() == 1);
    const ModelObject *object = loaded.objects.front();
    const ModelVolume *loaded_volume = object->volumes.front();
    CHECK(object->name == "Configured object");
    CHECK(loaded_volume->name == "Configured volume");
    CHECK(loaded_volume->is_modifier());
    CHECK(loaded_volume->source.input_file == "source.stl");
    CHECK(loaded_volume->source.object_idx == 4);
    CHECK(loaded_volume->source.volume_idx == 2);
    CHECK_THAT(loaded_volume->source.mesh_offset.x(), Catch::Matchers::WithinAbs(1.25, 1e-6));
    CHECK_THAT(loaded_volume->source.mesh_offset.y(), Catch::Matchers::WithinAbs(-2.5, 1e-6));
    CHECK_THAT(loaded_volume->source.mesh_offset.z(), Catch::Matchers::WithinAbs(3.75, 1e-6));
}

TEST_CASE("Legacy generic export and import preserve painted facet annotations", "[Legacy3MF][Painting]")
{
    Model source = make_cube_model();
    ModelVolume *volume = source.objects.front()->volumes.front();
    TriangleSelector selector(volume->mesh());
    selector.set_facet(0, EnforcerBlockerType::Extruder2);
    REQUIRE(volume->mmu_segmentation_facets.set(selector));
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));

    Model loaded;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), loaded, config));
    REQUIRE(loaded.objects.size() == 1);
    REQUIRE(loaded.objects.front()->volumes.size() == 1);
    const ModelVolume *loaded_volume = loaded.objects.front()->volumes.front();
    CHECK(loaded_volume->mmu_segmentation_facets.has_facets(*loaded_volume, EnforcerBlockerType::Extruder2));
    CHECK_FALSE(loaded_volume->mmu_segmentation_facets.has_facets(*loaded_volume, EnforcerBlockerType::Extruder3));
}

TEST_CASE("Legacy importer repairs a negatively wound volume", "[Legacy3MF][Geometry]")
{
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), cube_model_xml("ThirdParty", "  <item objectid=\"1\"/>\n", true)));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    TriangleMesh repaired = model.objects.front()->volumes.front()->mesh();
    CHECK_THAT(double(repaired.volume()), Catch::Matchers::WithinAbs(1000., 1e-3));
}

TEST_CASE("Legacy importer rejects out-of-range triangle vertex references", "[Legacy3MF][Errors]")
{
    const int invalid_vertex = GENERATE(-1, 8, 99);
    CAPTURE(invalid_vertex);
    std::string xml = cube_model_xml();
    const size_t attribute = xml.rfind("v3=\"2\"");
    REQUIRE(attribute != std::string::npos);
    xml.replace(attribute, 6, "v3=\"" + std::to_string(invalid_vertex) + "\"");
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), xml));
    Model model;
    DynamicPrintConfig config;
    CHECK_FALSE(load_generic(archive.string(), model, config));
}

TEST_CASE("Legacy importer rejects duplicate object identifiers", "[Legacy3MF][Errors]")
{
    std::string xml = cube_model_xml();
    const std::string duplicate =
        "<object id=\"1\" type=\"model\"><mesh><vertices><vertex x=\"0\" y=\"0\" z=\"0\"/>"
        "<vertex x=\"1\" y=\"0\" z=\"0\"/><vertex x=\"0\" y=\"1\" z=\"0\"/></vertices>"
        "<triangles><triangle v1=\"0\" v2=\"1\" v3=\"2\"/></triangles></mesh></object>";
    const size_t at = xml.find(" </resources>");
    REQUIRE(at != std::string::npos);
    xml.insert(at, duplicate);
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), xml));
    Model model;
    DynamicPrintConfig config;
    CHECK_FALSE(load_generic(archive.string(), model, config));
}

TEST_CASE("Legacy importer rejects an invalid component reference", "[Legacy3MF][Errors]")
{
    std::string xml = cube_model_xml();
    const size_t resources = xml.find(" </resources>");
    REQUIRE(resources != std::string::npos);
    xml.insert(resources,
        " <object id=\"2\" type=\"model\"><components><component objectid=\"999\"/></components></object>\n");
    xml.replace(xml.find("objectid=\"1\""), 12, "objectid=\"2\"");
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), xml));
    Model model;
    DynamicPrintConfig config;
    CHECK_FALSE(load_generic(archive.string(), model, config));
}

TEST_CASE("Legacy importer ignores a singular build transform", "[Legacy3MF][Geometry][Errors]")
{
    const std::string items = "  <item objectid=\"1\" transform=\"0 0 0 0 0 0 0 0 0 0 0 0\"/>\n";
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), cube_model_xml("ThirdParty", items)));
    Model model;
    DynamicPrintConfig config;
    REQUIRE(load_generic(archive.string(), model, config));
    REQUIRE(model.objects.size() == 1);
    const BoundingBoxf3 bbox = model.mesh().bounding_box();
    CHECK_THAT(double(bbox.max.x() - bbox.min.x()), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK_THAT(double(bbox.max.y() - bbox.min.y()), Catch::Matchers::WithinAbs(10., 1e-5));
    CHECK_THAT(double(bbox.max.z() - bbox.min.z()), Catch::Matchers::WithinAbs(10., 1e-5));
}

TEST_CASE("Legacy importer rejects malformed model configuration XML", "[Legacy3MF][Errors]")
{
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(write_archive(archive.string(), cube_model_xml(), "<config><object id=\"1\">") );
    Model model;
    DynamicPrintConfig config;
    CHECK_FALSE(load_generic(archive.string(), model, config));
}

TEST_CASE("Legacy exporter rejects an object with an empty volume mesh", "[Legacy3MF][Errors]")
{
    Model model;
    ModelObject *object = model.add_object();
    object->add_volume(TriangleMesh{});
    object->add_instance();
    ScopedTemporaryFile archive(".3mf");
    CHECK_FALSE(store_3mf(archive.string().c_str(), &model, nullptr, false));
}

} // namespace legacy_3mf_contracts

namespace bbs_3mf_metadata_contracts {

using namespace Slic3r;
using archive_variant_contracts::BbsLoaded;
using archive_variant_contracts::make_cube_model;
using archive_variant_contracts::read_file;
using archive_variant_contracts::replace_once;
using namespace Catch::Matchers;

using Setup = std::function<void(Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &)>;

void store_followup(const std::string &path, const Setup &setup,
                    SaveStrategy strategy = SaveStrategy::Zip64 | SaveStrategy::Silence)
{
    ScopedTemporaryDir backup("orca_bbs_followup_store");
    Model model = make_cube_model();
    model.set_backup_path(backup.string());
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    // Plate metadata is intentionally a complete print config.  The exporter
    // reads plate-local options and does not synthesize missing options.
    plate.config = config;
    StoreParams extra;
    setup(model, config, plate, extra, backup);
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};
    extra.path = path;
    extra.model = &model;
    extra.config = &config;
    extra.plate_data_list = {&plate};
    extra.strategy = strategy;
    REQUIRE(store_bbs_3mf(extra));
}

bool load_followup(const std::string &path, Model &model, BbsLoaded &loaded,
                   LoadStrategy strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig)
{
    // The importer writes Metadata/* payloads below the caller-owned backup
    // root.  Seed that directory so the public extraction path is exercised,
    // rather than silently accepting a declared filename after a failed write.
    loaded.backup = std::make_unique<ScopedTemporaryDir>("orca_bbs_followup_load");
    std::filesystem::create_directories(std::filesystem::path(loaded.backup->string()) / "Metadata");
    model.set_backup_path(loaded.backup->string());
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    return load_bbs_3mf(path.c_str(), &loaded.config, &substitutions, &model, &loaded.plates,
                        &loaded.presets, &loaded.is_bbl, &loaded.is_orca, &loaded.version,
                        nullptr, strategy);
}

std::string png_entry(const std::string &path, const std::string &name,
                      int expected_width, int expected_height)
{
    std::string encoded;
    REQUIRE(read_3mf_entry(path, encoded, name));
    png::ImageColorscale decoded;
    REQUIRE(png::decode_colored_png(png::ReadBuf{encoded.data(), encoded.size()}, decoded));
    REQUIRE(decoded.cols == expected_width);
    REQUIRE(decoded.rows == expected_height);
    REQUIRE(decoded.bytes_per_pixel == 4);
    return std::string(reinterpret_cast<const char *>(decoded.buf.data()), decoded.buf.size());
}

void check_rgba_pixels(const std::string &actual, const std::array<unsigned char, 16> &expected)
{
    REQUIRE(actual.size() == expected.size());
    CHECK(std::equal(actual.begin(), actual.end(), expected.begin(), [](char value, unsigned char wanted) {
        return static_cast<unsigned char>(value) == wanted;
    }));
}

namespace bbs_export_gap_contracts {

std::vector<std::pair<std::string, std::string>> read_layer_filament_lists(const std::string &xml)
{
    std::stringstream input(xml);
    boost::property_tree::ptree tree;
    REQUIRE_NOTHROW(boost::property_tree::read_xml(input, tree));
    std::vector<std::pair<std::string, std::string>> records;
    const auto lists = tree.get_child_optional("config.plate.layer_filament_lists");
    if (!lists)
        return records;
    for (const auto &node : *lists) {
        if (node.first != "layer_filament_list")
            continue;
        const auto &attributes = node.second.get_child("<xmlattr>");
        records.emplace_back(attributes.get<std::string>("filament_list"),
                             attributes.get<std::string>("layer_ranges"));
    }
    return records;
}

bool store_bbs_with_thumbnails(const std::string &path,
                               ThumbnailData &thumbnail,
                               const std::array<size_t, 4> &counts)
{
    ScopedTemporaryDir backup("orca_bbs_thumbnail_count_store");
    Model model = make_cube_model();
    model.set_backup_path(backup.string());
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    plate.config = config;
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};

    StoreParams params;
    params.path = path;
    params.model = &model;
    params.config = &config;
    params.plate_data_list = {&plate};
    params.thumbnail_data.assign(counts[0], &thumbnail);
    params.no_light_thumbnail_data.assign(counts[1], &thumbnail);
    params.top_thumbnail_data.assign(counts[2], &thumbnail);
    params.pick_thumbnail_data.assign(counts[3], &thumbnail);
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    return store_bbs_3mf(params);
}

} // namespace bbs_export_gap_contracts

TEST_CASE("BBS slice metadata preserves multiple layer filament ranges", "[3MF][Bbs3mfExportGaps][LayerFilaments]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.is_sliced_valid = true;
        plate.layer_filaments[std::vector<unsigned int>{1, 3}] = {{0, 5}, {10, 20}};
        plate.layer_filaments[std::vector<unsigned int>{2}] = {{5, 10}, {20, 30}};
    });

    std::string slice_info;
    REQUIRE(read_3mf_entry(archive.string(), slice_info, "Metadata/slice_info.config"));
    const auto records = bbs_export_gap_contracts::read_layer_filament_lists(slice_info);
    REQUIRE(records.size() == 2);
    CHECK(std::find(records.begin(), records.end(), std::make_pair(std::string("1 3"), std::string("0 5,10 20"))) != records.end());
    CHECK(std::find(records.begin(), records.end(), std::make_pair(std::string("2"), std::string("5 10,20 30"))) != records.end());

    ScopedTemporaryFile empty_archive(".3mf");
    store_followup(empty_archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.is_sliced_valid = true;
    });
    std::string empty_slice_info;
    REQUIRE(read_3mf_entry(empty_archive.string(), empty_slice_info, "Metadata/slice_info.config"));
    CHECK(bbs_export_gap_contracts::read_layer_filament_lists(empty_slice_info).empty());
}

TEST_CASE("BBS thumbnail lists obey per-channel plate cardinality", "[3MF][Bbs3mfExportGaps][ThumbnailValidation]")
{
    ThumbnailData thumbnail;
    thumbnail.set(2, 2);
    const std::array<unsigned char, 16> expected = {
        0x10, 0x20, 0x30, 0xff, 0x40, 0x50, 0x60, 0xff,
        0x70, 0x80, 0x90, 0xff, 0xa0, 0xb0, 0xc0, 0xff};
    std::copy(expected.begin(), expected.end(), thumbnail.pixels.begin());

    const std::array<const char *, 4> entries = {
        "Metadata/plate_1.png", "Metadata/plate_no_light_1.png",
        "Metadata/top_1.png", "Metadata/pick_1.png"};
    const std::array<std::array<size_t, 4>, 6> invalid_counts = {{
        {{2, 0, 0, 0}},
        {{0, 2, 0, 0}},
        {{0, 0, 2, 2}},
        {{0, 0, 1, 2}},
        {{0, 0, 0, 1}},
        {{0, 0, 1, 0}},
    }};

    const std::array<size_t, 4> valid_counts = {{1, 1, 1, 1}};
    ScopedTemporaryFile valid_archive(".3mf");
    REQUIRE(bbs_export_gap_contracts::store_bbs_with_thumbnails(valid_archive.string(), thumbnail, valid_counts));
    for (const char *entry : entries) {
        const std::string decoded = png_entry(valid_archive.string(), entry, 2, 2);
        check_rgba_pixels(decoded, expected);
    }

    for (const auto &counts : invalid_counts) {
        INFO("thumbnail counts plate=" << counts[0] << " no_light=" << counts[1]
             << " top=" << counts[2] << " pick=" << counts[3]);
        ScopedTemporaryFile rejected_archive(".3mf");
        CHECK_FALSE(bbs_export_gap_contracts::store_bbs_with_thumbnails(
            rejected_archive.string(), thumbnail, counts));
    }
}

TEST_CASE("BBS project preserves plate scheduling metadata", "[3mf][Bbs3mfMetadata][PlateMetadata]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.plate_name = "schedule metadata";
        plate.locked = true;
        plate.config.set_key_value("curr_bed_type", new ConfigOptionEnum<BedType>(btPEI));
        plate.config.set_key_value("print_sequence", new ConfigOptionEnum<PrintSequence>(PrintSequence::ByObject));
        plate.config.set_key_value("first_layer_print_sequence", new ConfigOptionInts({2, 1}));
        plate.config.set_key_value("other_layers_print_sequence", new ConfigOptionInts({1, 2}));
        plate.config.set_key_value("other_layers_print_sequence_nums", new ConfigOptionInt(3));
        plate.config.set_key_value("spiral_mode", new ConfigOptionBool(true));
    });

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    const PlateData &plate = *loaded.plates.front();
    CHECK(plate.plate_name == "schedule metadata");
    CHECK(plate.locked);
    REQUIRE(plate.config.option<ConfigOptionEnum<BedType>>("curr_bed_type") != nullptr);
    CHECK(plate.config.opt_enum<BedType>("curr_bed_type") == btPEI);
    CHECK(plate.config.opt_enum<PrintSequence>("print_sequence") == PrintSequence::ByObject);
    REQUIRE(plate.config.option<ConfigOptionInts>("first_layer_print_sequence") != nullptr);
    REQUIRE(plate.config.option<ConfigOptionInts>("other_layers_print_sequence") != nullptr);
    CHECK(plate.config.opt<ConfigOptionInts>("first_layer_print_sequence")->values == std::vector<int>{2, 1});
    CHECK(plate.config.opt<ConfigOptionInts>("other_layers_print_sequence")->values == std::vector<int>{1, 2});
    CHECK(plate.config.opt_int("other_layers_print_sequence_nums") == 3);
    CHECK(plate.config.opt_bool("spiral_mode"));
}

TEST_CASE("BBS project clamps plate map values at import", "[3mf][Bbs3mfMetadata][PlateMetadata][Maps]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.config.set_key_value("filament_map_mode", new ConfigOptionEnum<FilamentMapMode>(fmmManual));
        plate.config.set_key_value("filament_map", new ConfigOptionInts({0, -4, 2}));
        plate.config.set_key_value("filament_volume_map", new ConfigOptionInts({0, 2, 3}));
    });

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    const PlateData &plate = *loaded.plates.front();
    REQUIRE(plate.config.option<ConfigOptionInts>("filament_map") != nullptr);
    REQUIRE(plate.config.option<ConfigOptionInts>("filament_volume_map") != nullptr);
    CHECK(plate.filament_maps == std::vector<int>{1, 1, 2});
    CHECK(plate.config.opt<ConfigOptionInts>("filament_map")->values == std::vector<int>{1, 1, 2});
    CHECK(plate.config.opt<ConfigOptionInts>("filament_volume_map")->values == std::vector<int>{0, 0, 0});
}

TEST_CASE("BBS project retains slice statistics and booleans", "[3mf][Bbs3mfMetadata][SliceInfo]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.is_sliced_valid = true;
        plate.printer_model_id = "followup-printer";
        plate.gcode_prediction = "123.4";
        plate.gcode_weight = "45.6";
        plate.toolpath_outside = true;
        plate.is_support_used = true;
        plate.is_label_object_enabled = true;
        plate.limit_filament_maps = {1, 4};
    });

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    const PlateData &plate = *loaded.plates.front();
    CHECK(plate.printer_model_id == "followup-printer");
    CHECK(plate.gcode_prediction == "123.4");
    CHECK(plate.gcode_weight == "45.6");
    CHECK(plate.toolpath_outside);
    CHECK(plate.is_support_used);
    CHECK(plate.is_label_object_enabled);
    CHECK(plate.limit_filament_maps.empty()); // limit maps are write-only model metadata today
}

TEST_CASE("BBS project writes physical and mixed filament records", "[3mf][Bbs3mfMetadata][SliceInfo][Filament]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.is_sliced_valid = true;
        FilamentInfo physical;
        physical.id = 1;
        physical.type = "PLA";
        physical.color = "#123456";
        physical.filament_id = "tray-7";
        physical.used_m = 2.5f;
        physical.used_g = 6.25f;
        physical.used_for_object = true;
        physical.used_for_support = true;
        physical.group_id = {1, 2};
        physical.nozzle_diameter = .6;
        physical.nozzle_volume_type = "High Flow";
        plate.slice_filaments_info.push_back(physical);
        plate.mixed_filaments_info.push_back({9, "Mix", "#aabbcc", "2,3"});
    });

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    const PlateData &plate = *loaded.plates.front();
    REQUIRE(plate.slice_filaments_info.size() == 1);
    const FilamentInfo &physical = plate.slice_filaments_info.front();
    CHECK(physical.id == 1);
    CHECK(physical.type == "PLA");
    CHECK(physical.color == "#123456");
    CHECK(physical.filament_id == "tray-7");
    CHECK_THAT(double(physical.used_m), WithinAbs(2.5, 1e-5));
    CHECK_THAT(double(physical.used_g), WithinAbs(6.25, 1e-5));
    CHECK(physical.used_for_object);
    CHECK(physical.used_for_support);
    CHECK(physical.group_id == std::vector<int>{1, 2});
    std::string slice_info;
    REQUIRE(read_3mf_entry(archive.string(), slice_info, "Metadata/slice_info.config"));
    CHECK(slice_info.find("<mixed_filament") != std::string::npos);
    CHECK(slice_info.find("id=\"9\"") != std::string::npos);
    CHECK(slice_info.find("type=\"Mix\"") != std::string::npos);
    CHECK(slice_info.find("color=\"#aabbcc\"") != std::string::npos);
    CHECK(slice_info.find("components=\"2,3\"") != std::string::npos);
}

TEST_CASE("BBS project restores warning messages without treating them as config", "[3mf][Bbs3mfMetadata][SliceInfo][Warnings]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.is_sliced_valid = true;
        GCodeProcessorResult::SliceWarning warning;
        warning.msg = "followup warning";
        warning.level = 3;
        plate.warnings.push_back(warning);
    });

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    REQUIRE(loaded.plates.front()->warnings.size() == 1);
    CHECK(loaded.plates.front()->warnings.front().msg == "followup warning");
    CHECK(loaded.plates.front()->warnings.front().level == 3);
}

TEST_CASE("BBS project stores and extracts a sliced G-code payload", "[3mf][Bbs3mfMetadata][Gcode]")
{
    ScopedTemporaryFile gcode(".gcode");
    {
        std::ofstream output(gcode.string(), std::ios::binary);
        output << "; follow-up gcode\nG1 X1.25 Y2.5 E0.4\n";
    }
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.gcode_file = gcode.string();
        plate.is_sliced_valid = true;
    }, SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::WithGcode | SaveStrategy::WithSliceInfo);

    std::string archived;
    REQUIRE(read_3mf_entry(archive.string(), archived, "Metadata/plate_1.gcode"));
    CHECK(archived == "; follow-up gcode\nG1 X1.25 Y2.5 E0.4\n");
    std::string digest;
    CHECK(read_3mf_entry(archive.string(), digest, "Metadata/plate_1.gcode.md5"));
    CHECK(digest == "0891BA245E812E8989E6EA1927F84013");

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK(!loaded.plates.front()->gcode_file.empty());
    const std::filesystem::path extracted = std::filesystem::path(loaded.backup->string()) / "Metadata/plate_1.gcode";
    REQUIRE(std::filesystem::exists(extracted));
    CHECK(read_file(extracted.string()) == "; follow-up gcode\nG1 X1.25 Y2.5 E0.4\n");
}

TEST_CASE("BBS project omits an unavailable G-code source", "[3mf][Bbs3mfMetadata][Gcode][Errors]")
{
    ScopedTemporaryDir missing_source("orca_missing_gcode_source");
    const std::string missing_path = (std::filesystem::path(missing_source.string()) / "absent.gcode").string();
    REQUIRE_FALSE(std::filesystem::exists(missing_path));
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.gcode_file = missing_path;
        plate.is_sliced_valid = true;
    }, SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::WithGcode | SaveStrategy::WithSliceInfo);
    std::string payload;
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/plate_1.gcode"));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK_FALSE(std::filesystem::exists(loaded.plates.front()->gcode_file));
}

TEST_CASE("BBS project SkipStatic omits static payloads while retaining model geometry", "[3mf][Bbs3mfMetadata][Strategy]")
{
    ThumbnailData image;
    image.set(3, 2);
    std::fill(image.pixels.begin(), image.pixels.end(), 0x22);
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &params, ScopedTemporaryDir &) {
        plate.plate_thumbnail = image;
        params.thumbnail_data = {&image};
    }, SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::SkipStatic);
    std::string payload;
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/plate_1.png"));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    CHECK(loaded_model.mesh().facets_count() == 12);
}

TEST_CASE("BBS project exports and decodes all four plate thumbnail channels", "[3mf][Bbs3mfMetadata][Thumbnail]")
{
    ThumbnailData plate_image, no_light, top, pick;
    plate_image.set(2, 2);
    no_light.set(2, 2);
    top.set(2, 2);
    pick.set(2, 2);
    const std::array<unsigned char, 16> plate_pixels = {0x10, 0x20, 0x30, 0xff, 0x40, 0x50, 0x60, 0xff,
                                                        0x70, 0x80, 0x90, 0xff, 0xa0, 0xb0, 0xc0, 0xff};
    const std::array<unsigned char, 16> no_light_pixels = {0x01, 0x12, 0x23, 0xff, 0x34, 0x45, 0x56, 0xff,
                                                           0x67, 0x78, 0x89, 0xff, 0x9a, 0xab, 0xbc, 0xff};
    const std::array<unsigned char, 16> top_pixels = {0xd0, 0xc0, 0xb0, 0xff, 0xa0, 0x90, 0x80, 0xff,
                                                      0x70, 0x60, 0x50, 0xff, 0x40, 0x30, 0x20, 0xff};
    const std::array<unsigned char, 16> pick_pixels = {0x0a, 0x1b, 0x2c, 0xff, 0x3d, 0x4e, 0x5f, 0xff,
                                                       0x6a, 0x7b, 0x8c, 0xff, 0x9d, 0xae, 0xbf, 0xff};
    std::copy(plate_pixels.begin(), plate_pixels.end(), plate_image.pixels.begin());
    std::copy(no_light_pixels.begin(), no_light_pixels.end(), no_light.pixels.begin());
    std::copy(top_pixels.begin(), top_pixels.end(), top.pixels.begin());
    std::copy(pick_pixels.begin(), pick_pixels.end(), pick.pixels.begin());
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &params, ScopedTemporaryDir &) {
        plate.plate_thumbnail = plate_image;
        params.thumbnail_data = {&plate_image};
        params.no_light_thumbnail_data = {&no_light};
        params.top_thumbnail_data = {&top};
        params.pick_thumbnail_data = {&pick};
        plate.no_light_thumbnail_file = "unused-no-light.png";
        plate.top_file = "unused-top.png";
        plate.pick_file = "unused-pick.png";
    });
    const std::string p = png_entry(archive.string(), "Metadata/plate_1.png", 2, 2);
    const std::string n = png_entry(archive.string(), "Metadata/plate_no_light_1.png", 2, 2);
    const std::string t = png_entry(archive.string(), "Metadata/top_1.png", 2, 2);
    const std::string k = png_entry(archive.string(), "Metadata/pick_1.png", 2, 2);
    check_rgba_pixels(p, plate_pixels);
    check_rgba_pixels(n, no_light_pixels);
    check_rgba_pixels(t, top_pixels);
    check_rgba_pixels(k, pick_pixels);
}

TEST_CASE("BBS project archives sidecar thumbnail files under declared names", "[3mf][Bbs3mfMetadata][Thumbnail][Sidecar]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &backup) {
        const std::array<std::string, 4> bytes = {"sidecar-plate", "sidecar-no-light", "sidecar-top", "sidecar-pick"};
        const std::array<std::string, 4> names = {"plate.png", "no-light.png", "top.png", "pick.png"};
        for (size_t i = 0; i < names.size(); ++i) {
            std::ofstream output(std::filesystem::path(backup.string()) / names[i], std::ios::binary);
            output << bytes[i];
        }
        plate.thumbnail_file = (std::filesystem::path(backup.string()) / names[0]).string();
        plate.no_light_thumbnail_file = (std::filesystem::path(backup.string()) / names[1]).string();
        plate.top_file = (std::filesystem::path(backup.string()) / names[2]).string();
        plate.pick_file = (std::filesystem::path(backup.string()) / names[3]).string();
    });
    std::string payload;
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/plate_1.png"));
    CHECK(payload == "sidecar-plate");
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/plate_no_light_1.png"));
    CHECK(payload == "sidecar-no-light");
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/top_1.png"));
    CHECK(payload == "sidecar-top");
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/pick_1.png"));
    CHECK(payload == "sidecar-pick");
}

TEST_CASE("BBS exporter recursively archives model auxiliary files", "[3MF][Bbs3mfMetadata][BbsAuxiliaryExport]")
{
    ScopedTemporaryFile archive(".3mf");
    ScopedTemporaryDir backup("orca_bbs_auxiliary_export");
    Model model = make_cube_model();
    model.set_backup_path(backup.string());
    ScopeGuard detach_backup([&model] { model.set_backup_path("detach"); });

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    plate.config = config;
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};

    // This is the public model-owned auxiliary root passed to the exporter.
    const std::filesystem::path auxiliary_root = model.get_auxiliary_file_temp_path();
    const std::filesystem::path nested = auxiliary_root / "docs" / "nested";
    REQUIRE(std::filesystem::create_directories(nested));
    {
        std::ofstream root_file(auxiliary_root / "contract.txt", std::ios::binary);
        root_file << "aux-root-contract";
        REQUIRE(root_file.good());
        std::ofstream nested_file(nested / "contract.txt", std::ios::binary);
        nested_file << "aux-nested-contract";
        REQUIRE(nested_file.good());
    }

    // A sibling of Auxiliaries is a negative control for the recursive root.
    {
        std::ofstream outside_file(std::filesystem::path(backup.string()) / "outside.txt", std::ios::binary);
        outside_file << "must-not-archive";
        REQUIRE(outside_file.good());
    }

    StoreParams params;
    params.path = archive.string();
    params.model = &model;
    params.config = &config;
    params.plate_data_list = {&plate};
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));

    std::string payload;
    REQUIRE(read_3mf_entry(archive.string(), payload, "Auxiliaries/contract.txt"));
    CHECK(payload == "aux-root-contract");
    REQUIRE(read_3mf_entry(archive.string(), payload, "Auxiliaries/docs/nested/contract.txt"));
    CHECK(payload == "aux-nested-contract");
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Auxiliaries/outside.txt"));

    ScopedTemporaryFile without_auxiliary(".3mf");
    params.path = without_auxiliary.string();
    params.strategy = params.strategy | SaveStrategy::SkipAuxiliary;
    REQUIRE(store_bbs_3mf(params));
    REQUIRE(read_3mf_entry(without_auxiliary.string(), payload, "3D/3dmodel.model"));
    CHECK_FALSE(payload.empty());
    CHECK_FALSE(read_3mf_entry(without_auxiliary.string(), payload, "Auxiliaries/contract.txt"));
    CHECK_FALSE(read_3mf_entry(without_auxiliary.string(), payload, "Auxiliaries/docs/nested/contract.txt"));
}

TEST_CASE("BBS project persists plate bounding-box JSON", "[3mf][Bbs3mfMetadata][BBox]")
{
    PlateBBoxData bbox;
    bbox.bbox_all = {1, 2, 31, 42};
    bbox.filament_ids = {0, 2};
    bbox.filament_colors = {"#111111", "#222222"};
    bbox.bed_type = "Textured PEI Plate";
    bbox.first_layer_time = 8.5f;
    bbox.bbox_objs.push_back({7, {1, 2, 31, 42}, 120.f, .2f, "bbox-cube"});
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &params, ScopedTemporaryDir &) {
        plate.pattern_bbox_file = "bbox-source.json";
        params.id_bboxes = {&bbox};
    });
    std::string json_text;
    REQUIRE(read_3mf_entry(archive.string(), json_text, "Metadata/plate_1.json"));
    const nlohmann::json json = nlohmann::json::parse(json_text);
    CHECK(json.at("bbox_all").get<std::vector<coordf_t>>() == bbox.bbox_all);
    CHECK(json.at("filament_ids").get<std::vector<int>>() == bbox.filament_ids);
    CHECK(json.at("filament_colors").get<std::vector<std::string>>() == bbox.filament_colors);
    CHECK(json.at("bed_type").get<std::string>() == bbox.bed_type);
    CHECK_THAT(json.at("first_layer_time").get<float>(), WithinAbs(bbox.first_layer_time, 1e-5));
    CHECK(json.at("is_seq_print").get<bool>() == bbox.is_seq_print);
    CHECK(json.at("first_extruder").get<int>() == bbox.first_extruder);
    CHECK_THAT(json.at("nozzle_diameter").get<float>(), WithinAbs(bbox.nozzle_diameter, 1e-5));
    CHECK(json.at("version").get<int>() == bbox.version);
    REQUIRE(json.at("bbox_objects").size() == 1);
    const nlohmann::json &object = json.at("bbox_objects").at(0);
    CHECK(object.at("id").get<int>() == 7);
    CHECK(object.at("bbox").get<std::vector<coordf_t>>() == bbox.bbox_objs.front().bbox);
    CHECK_THAT(object.at("area").get<float>(), WithinAbs(120.f, 1e-5));
    CHECK_THAT(object.at("layer_height").get<float>(), WithinAbs(.2f, 1e-5));
    CHECK(object.at("name").get<std::string>() == "bbox-cube");
}

TEST_CASE("BBS project loads embedded filament and printer presets", "[3mf][Bbs3mfMetadata][EmbeddedPreset]")
{
    ScopedTemporaryFile filament_file(".json");
    DynamicPrintConfig filament;
    filament.set_key_value("filament_type", new ConfigOptionStrings({"PETG"}));
    filament.set_key_value("filament_diameter", new ConfigOptionFloats({1.75}));
    // The importer uses this field as the embedded preset's canonical name.
    filament.set_key_value("filament_settings_id", new ConfigOptionStrings({"Embedded filament contract"}));
    filament.save_to_json(filament_file.string(), "Embedded filament contract", "filament", "0.0.0");
    ScopedTemporaryFile printer_file(".json");
    DynamicPrintConfig printer;
    printer.set_key_value("printer_model", new ConfigOptionString("Contract Printer"));
    // The importer uses this field as the embedded preset's canonical name.
    printer.set_key_value("printer_settings_id", new ConfigOptionString("Embedded printer contract"));
    printer.save_to_json(printer_file.string(), "Embedded printer contract", "printer", "0.0.0");

    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &config, PlateData &, StoreParams &, ScopedTemporaryDir &) {
        config.set_key_value("filament_settings_id", new ConfigOptionStrings({"Embedded filament contract"}));
        config.set_key_value("printer_settings_id", new ConfigOptionString("Embedded printer contract"));
    });
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {
        {"Metadata/filament_settings_1.config", read_file(filament_file.string())},
        {"Metadata/machine_settings_1.config", read_file(printer_file.string())}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    const auto filament_preset = std::find_if(loaded.presets.begin(), loaded.presets.end(), [](const Preset *p) {
        return p != nullptr && p->type == Preset::TYPE_FILAMENT;
    });
    const auto printer_preset = std::find_if(loaded.presets.begin(), loaded.presets.end(), [](const Preset *p) {
        return p != nullptr && p->type == Preset::TYPE_PRINTER;
    });
    REQUIRE(filament_preset != loaded.presets.end());
    REQUIRE(printer_preset != loaded.presets.end());
    CHECK((*filament_preset)->name == "Embedded filament contract");
    CHECK((*printer_preset)->name == "Embedded printer contract");
    REQUIRE((*filament_preset)->config.option<ConfigOptionStrings>("filament_type") != nullptr);
    CHECK((*filament_preset)->config.opt<ConfigOptionStrings>("filament_type")->values == std::vector<std::string>{"PETG"});
    REQUIRE((*filament_preset)->config.option<ConfigOptionFloats>("filament_diameter") != nullptr);
    REQUIRE((*filament_preset)->config.opt<ConfigOptionFloats>("filament_diameter")->values.size() == 1);
    CHECK_THAT((*filament_preset)->config.opt<ConfigOptionFloats>("filament_diameter")->values.front(), WithinAbs(1.75, 1e-6));
    REQUIRE((*printer_preset)->config.option<ConfigOptionString>("printer_model") != nullptr);
    CHECK((*printer_preset)->config.opt_string("printer_model") == "Contract Printer");
}

TEST_CASE("BBS project ignores malformed filament sequence JSON without losing the model", "[3mf][Bbs3mfMetadata][Errors][Sequence]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.filament_change_sequence = {0, 1};
        plate.nozzle_change_sequence = {1};
    });
    REQUIRE(replace_once(archive.string(), "Metadata/filament_sequence.json", "{", "not-json"));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->filament_change_sequence.empty());
    CHECK(loaded.plates.front()->nozzle_change_sequence.empty());
}

TEST_CASE("BBS project ignores a sequence plate with no nozzle array", "[3mf][Bbs3mfMetadata][Errors][Sequence]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &, ScopedTemporaryDir &) {
        plate.filament_change_sequence = {0, 1};
        plate.nozzle_change_sequence = {1};
    });
    REQUIRE(replace_once(archive.string(), "Metadata/filament_sequence.json",
                         "\"nozzle_sequence\":[1]", "\"nozzle_missing\":[1]"));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded.plates.size() == 1);
    CHECK(loaded.plates.front()->filament_change_sequence.empty());
    CHECK(loaded.plates.front()->nozzle_change_sequence.empty());
}

TEST_CASE("BBS project keeps model geometry when project configuration is absent", "[3mf][Bbs3mfMetadata][Errors][Configuration]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &name, std::string &) {
        if (name == "Metadata/project_settings.config") {
            name = "Metadata/project_settings.removed";
            return true;
        }
        return false;
    }));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    CHECK(loaded_model.mesh().facets_count() == 12);
    CHECK(loaded.config.empty());
}

TEST_CASE("BBS project skips an invalid layer range object id", "[3mf][Bbs3mfMetadata][Errors][LayerRanges]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});
    const std::string invalid = "<?xml version=\"1.0\"?><objects><object id=\"0\"><range min_z=\"0\" max_z=\"1\"><option opt_key=\"layer_height\">0.25</option></range></object></objects>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {{"Metadata/layer_config_ranges.xml", invalid}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    CHECK(loaded_model.objects.front()->layer_config_ranges.empty());
}

TEST_CASE("BBS project keeps the first duplicate layer range object", "[3mf][Bbs3mfMetadata][Errors][LayerRanges]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});
    const std::string duplicate = "<?xml version=\"1.0\"?><objects>"
        "<object id=\"1\"><range min_z=\"0\" max_z=\"1\"><option opt_key=\"layer_height\">0.25</option></range></object>"
        "<object id=\"1\"><range min_z=\"0\" max_z=\"1\"><option opt_key=\"layer_height\">0.35</option></range></object>"
        "</objects>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {{"Metadata/layer_config_ranges.xml", duplicate}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->layer_config_ranges.size() == 1);
    CHECK_THAT(loaded_model.objects.front()->layer_config_ranges.begin()->second.get().opt_float("layer_height"), WithinAbs(.25, 1e-6));
}

TEST_CASE("BBS project ignores malformed brim point records", "[3mf][Bbs3mfMetadata][Errors][Brim]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});
    const std::string malformed = "version=0\nobject_id=1\nnot-a-coordinate\n";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {{"Metadata/brim_ear_points.txt", malformed}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    CHECK(loaded_model.objects.front()->brim_points.empty());
}

TEST_CASE("BBS project preserves each custom-gcode mode and typed item", "[3mf][Bbs3mfMetadata][CustomGcode]")
{
    const std::string mode = GENERATE("SingleExtruder", "MultiAsSingle", "MultiExtruder");
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [&](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {
    });
    const std::string xml = "<?xml version=\"1.0\"?><custom_gcodes_per_layer><plate><plate_info id=\"1\"/>"
        "<layer top_z=\"0.6\" type=\"1\" extruder=\"2\" color=\"#c0ffee\" extra=\"pause-note\" gcode=\"M601\"/>"
        "<mode value=\"" + mode + "\"/></plate></custom_gcodes_per_layer>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {{"Metadata/custom_gcode_per_layer.xml", xml}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.plates_custom_gcodes.size() == 1);
    const auto &info = loaded_model.plates_custom_gcodes.at(0);
    REQUIRE(info.gcodes.size() == 1);
    CHECK(info.gcodes.front().type == CustomGCode::PausePrint);
    CHECK(info.gcodes.front().extruder == 2);
    CHECK(info.gcodes.front().extra == "pause-note");
    const auto expected = mode == "SingleExtruder" ? CustomGCode::Mode::SingleExtruder :
        mode == "MultiAsSingle" ? CustomGCode::Mode::MultiAsSingle : CustomGCode::Mode::MultiExtruder;
    CHECK(info.mode == expected);
}

TEST_CASE("BBS project uses a safe default for an unknown custom-gcode mode", "[3mf][Bbs3mfMetadata][CustomGcode][Errors]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});
    const std::string xml = "<custom_gcodes_per_layer><plate><plate_info id=\"1\"/>"
        "<layer top_z=\"0.6\" type=\"3\" extruder=\"1\" color=\"#000000\" extra=\"tool_change\" gcode=\"tool_change\"/>"
        "<mode value=\"UnknownMode\"/></plate></custom_gcodes_per_layer>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; }, {{"Metadata/custom_gcode_per_layer.xml", xml}}));
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.plates_custom_gcodes.size() == 1);
    CHECK(loaded_model.plates_custom_gcodes.at(0).mode == CustomGCode::Mode::MultiExtruder);
}

TEST_CASE("Legacy custom-gcode entries without plate metadata map through the public project importer",
          "[3mf][Bbs3mfMetadata][LegacyCustomGcode]")
{
    ScopedTemporaryFile archive(".3mf");
    store_followup(archive.string(), [](Model &, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &) {});

    // Older projects placed layers directly below custom_gcodes_per_layer and encoded the
    // semantic type in the legacy gcode attribute.  Keep all four mappings in one archive so
    // this checks the public compatibility route, including the root-to-plate-1 fallback.
    const std::string xml = "<custom_gcodes_per_layer>"
        "<layer top_z=\"0.6\" extruder=\"1\" color=\"#112233\" gcode=\"M600\"/>"
        "<layer top_z=\"1.2\" extruder=\"2\" color=\"#223344\" gcode=\"M601\"/>"
        "<layer top_z=\"1.8\" extruder=\"3\" color=\"#334455\" gcode=\"tool_change\"/>"
        "<layer top_z=\"2.4\" extruder=\"4\" color=\"#445566\" gcode=\"M117\"/>"
        "</custom_gcodes_per_layer>";
    REQUIRE(rewrite_3mf_entries(archive.string(), [](std::string &, std::string &) { return false; },
                                 {{"Metadata/custom_gcode_per_layer.xml", xml}}));

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_followup(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.plates_custom_gcodes.size() == 1);
    const auto &info = loaded_model.plates_custom_gcodes.at(0);
    REQUIRE(info.gcodes.size() == 4);

    CHECK(info.gcodes[0].type == CustomGCode::ColorChange);
    CHECK_THAT(info.gcodes[0].print_z, WithinAbs(0.6, 1e-9));
    CHECK(info.gcodes[0].extruder == 1);
    CHECK(info.gcodes[0].color == "#112233");
    CHECK(info.gcodes[0].extra.empty());

    CHECK(info.gcodes[1].type == CustomGCode::PausePrint);
    CHECK_THAT(info.gcodes[1].print_z, WithinAbs(1.2, 1e-9));
    CHECK(info.gcodes[1].extruder == 2);
    CHECK(info.gcodes[1].color == "#223344");
    CHECK(info.gcodes[1].extra == "#223344");

    CHECK(info.gcodes[2].type == CustomGCode::ToolChange);
    CHECK_THAT(info.gcodes[2].print_z, WithinAbs(1.8, 1e-9));
    CHECK(info.gcodes[2].extruder == 3);
    CHECK(info.gcodes[2].extra.empty());

    CHECK(info.gcodes[3].type == CustomGCode::Custom);
    CHECK_THAT(info.gcodes[3].print_z, WithinAbs(2.4, 1e-9));
    CHECK(info.gcodes[3].extruder == 4);
    CHECK(info.gcodes[3].extra == "M117");
}

TEST_CASE("BBS project clears the plate pointer collection", "[3mf][Bbs3mfMetadata][Ownership]")
{
    std::set<std::pair<int, int>> objects{{0, 0}};
    PlateDataPtrs plates{new PlateData(0, objects, false), new PlateData(1, objects, true)};
    REQUIRE(plates.front() != nullptr);
    REQUIRE(plates.back() != nullptr);
    release_PlateData_list(plates);
    CHECK(plates.empty());
}

TEST_CASE("BBS project published flag accepts exactly the producer marker", "[3mf][Bbs3mfMetadata][Published]")
{
    CHECK(is_published_3mf_flag("1"));
    CHECK_FALSE(is_published_3mf_flag("0"));
    CHECK_FALSE(is_published_3mf_flag("true"));
    CHECK_FALSE(is_published_3mf_flag(" 1"));
    CHECK_FALSE(is_published_3mf_flag("1\n"));
}

} // namespace bbs_3mf_metadata_contracts


namespace plate_metadata_contracts {

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
void materials(GCodeProcessorResult& result)
{
    result.filament_diameters = {2.f, 1.f, 3.f};
    result.filament_densities = {1.25f, 2.f, .5f};
}

std::vector<MultiNozzleUtils::NozzleInfo> two_nozzles() { return {{"0.6", nvtStandard, 0, 0}, {"0.4", nvtHighFlow, 1, 1}}; }

std::shared_ptr<MultiNozzleUtils::LayeredNozzleGroupResult> layered_mapping()
{
    auto value = MultiNozzleUtils::LayeredNozzleGroupResult::create(std::vector<std::vector<int>>{{0}, {1}, {0}}, two_nozzles(),
                                                                    std::vector<unsigned int>{0},
                                                                    std::vector<std::vector<unsigned int>>{{0}, {0}, {0}});
    REQUIRE(value.has_value());
    return std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(*value);
}

const FilamentInfo& filament(const PlateData& plate, int id)
{
    const auto found = std::find_if(plate.slice_filaments_info.begin(), plate.slice_filaments_info.end(),
                                    [id](const FilamentInfo& info) { return info.id == id; });
    REQUIRE(found != plate.slice_filaments_info.end());
    return *found;
}

Preset make_preset(Preset::Type type, const std::string& name, double value)
{
    Preset preset(type, name);
    if (type == Preset::TYPE_PRINT) {
        preset.config.set_key_value("print_settings_id", new ConfigOptionString(name));
        preset.config.set_key_value("layer_height", new ConfigOptionFloat(value));
    } else if (type == Preset::TYPE_FILAMENT) {
        preset.config.set_key_value("filament_settings_id", new ConfigOptionStrings({name}));
        preset.config.set_key_value("filament_diameter", new ConfigOptionFloats({value}));
    } else if (type == Preset::TYPE_PRINTER) {
        preset.config.set_key_value("printer_settings_id", new ConfigOptionString(name));
        preset.config.set_key_value("nozzle_diameter", new ConfigOptionFloats({value}));
    }
    return preset;
}

using LoadedPresets = bbs_archive_contracts::LoadedProject;

void roundtrip_presets(std::vector<Preset*> presets,
                       LoadedPresets& loaded,
                       SaveStrategy strategy = SaveStrategy::Zip64 | SaveStrategy::Silence)
{
    ScopedTemporaryDir backup("orca_embedded_preset_store");
    ScopedTemporaryFile archive(".3mf");
    Model source;
    auto* object = source.add_object();
    object->add_volume(make_cube(2., 3., 4.));
    object->add_instance();
    source.set_backup_path(backup.string());
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    plate.plate_index = 0;
    StoreParams params;
    params.path            = archive.string();
    params.model           = &source;
    params.config          = &config;
    params.plate_data_list = {&plate};
    params.project_presets = std::move(presets);
    params.strategy        = strategy;
    REQUIRE(store_bbs_3mf(params));
    // A part name identifies one resource in a 3MF package. The importer can
    // iterate duplicate ZIP names, so round-trip counts alone miss collisions.
    {
        mz_zip_archive zip;
        mz_zip_zero_struct(&zip);
        REQUIRE(open_zip_reader(&zip, archive.string()));
        const ScopeGuard close([&] { close_zip_reader(&zip); });
        std::set<std::string> names;
        for (mz_uint index = 0; index < mz_zip_reader_get_num_files(&zip); ++index) {
            mz_zip_archive_file_stat stat;
            REQUIRE(mz_zip_reader_file_stat(&zip, index, &stat));
            REQUIRE(names.insert(stat.m_filename).second);
        }
    }
    Model imported;
    REQUIRE(bbs_3mf_metadata_contracts::load_followup(archive.string(), imported, loaded));
    REQUIRE(imported.objects.size() == 1);
    CHECK_THAT(imported.objects.front()->raw_mesh().volume(), WithinAbs(24., 1e-4));
}
} // namespace

TEST_CASE("Plate filament lengths and masses use each filament's physical properties", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder = {{0, 1000.}, {1, 2000.}};
    PlateData plate;
    plate.parse_filament_info(&result);
    REQUIRE(plate.slice_filaments_info.size() == 2);
    CHECK_THAT(filament(plate, 0).used_m, WithinAbs(1. / PI, 1e-6));
    CHECK_THAT(filament(plate, 0).used_g, WithinAbs(1.25, 1e-6));
    CHECK_THAT(filament(plate, 1).used_m, WithinAbs(8. / PI, 1e-6));
    CHECK_THAT(filament(plate, 1).used_g, WithinAbs(4., 1e-6));
}

TEST_CASE("Sparse filament identifiers select their own diameter and density", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder = {{2, 9000.}};
    PlateData plate;
    plate.parse_filament_info(&result);
    REQUIRE(plate.slice_filaments_info.size() == 1);
    CHECK_THAT(filament(plate, 2).used_m, WithinAbs(4. / PI, 1e-6));
    CHECK_THAT(filament(plate, 2).used_g, WithinAbs(4.5, 1e-6));
}

TEST_CASE("Zero filament diameter yields a finite zero length while retaining mass", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    materials(result);
    result.filament_diameters[0]                       = 0.f;
    result.print_statistics.total_volumes_per_extruder = {{0, 1000.}};
    PlateData plate;
    plate.parse_filament_info(&result);
    CHECK_THAT(filament(plate, 0).used_m, WithinAbs(0., 0.));
    CHECK_THAT(filament(plate, 0).used_g, WithinAbs(1.25, 1e-6));
}

TEST_CASE("Object and support usage flags follow their separate positive-volume records", "[3MF][PlateMetadata]")
{
    const bool object_used  = GENERATE(false, true);
    const bool support_used = GENERATE(false, true);
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder   = {{0, 1000.}, {1, 0.}};
    result.print_statistics.model_volumes_per_extruder   = {{0, object_used ? 3. : 0.}};
    result.print_statistics.support_volumes_per_extruder = {{0, support_used ? 7. : 0.}};
    PlateData plate;
    plate.parse_filament_info(&result);
    CHECK(filament(plate, 0).used_for_object == object_used);
    CHECK(filament(plate, 0).used_for_support == support_used);
    CHECK_FALSE(filament(plate, 1).used_for_object);
    CHECK_FALSE(filament(plate, 1).used_for_support);
}

TEST_CASE("Absent processor results leave existing plate metadata intact", "[3MF][PlateMetadata]")
{
    PlateData plate;
    FilamentInfo existing;
    existing.id     = 7;
    existing.used_g = 2.f;
    plate.slice_filaments_info.push_back(existing);
    plate.warnings.push_back({2, "existing", "E1", {"parameter"}});
    plate.parse_filament_info(nullptr);
    REQUIRE(plate.slice_filaments_info.size() == 1);
    CHECK_THAT(filament(plate, 7).used_g, WithinAbs(2., 0.));
    REQUIRE(plate.warnings.size() == 1);
    CHECK(plate.warnings.front().msg == "existing");
}

TEST_CASE("Plate warnings preserve all fields and own a copy of their parameters", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    result.warnings = {{2, "first", "E42", {"a", "b"}}, {1, "second", "W3", {"c"}}};
    PlateData plate;
    plate.parse_filament_info(&result);
    result.warnings.front().params.front() = "changed";
    REQUIRE(plate.warnings.size() == 2);
    CHECK(plate.warnings[0].level == 2);
    CHECK(plate.warnings[0].msg == "first");
    CHECK(plate.warnings[0].error_code == "E42");
    CHECK(plate.warnings[0].params == std::vector<std::string>{"a", "b"});
    CHECK(plate.warnings[1].level == 1);
    CHECK(plate.warnings[1].msg == "second");
    CHECK(plate.warnings[1].error_code == "W3");
    CHECK(plate.warnings[1].params == std::vector<std::string>{"c"});
    CHECK(plate.slice_filaments_info.empty());
}

TEST_CASE("Layered filament mappings retain every used nozzle and hybrid volume type", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder = {{0, 1000.}};
    result.nozzle_group_result                         = layered_mapping();
    PlateData plate;
    plate.parse_filament_info(&result);
    const auto& info = filament(plate, 0);
    CHECK(info.group_id == std::vector<int>{0, 1});
    CHECK_THAT(info.nozzle_diameter, WithinAbs(.4, 1e-8));
    CHECK(info.nozzle_volume_type == get_nozzle_volume_type_string(nvtHybrid));
    REQUIRE(plate.nozzle_group_result.has_value());
    CHECK(plate.nozzle_group_result->get_layer_filament_nozzle_maps() == std::vector<std::vector<int>>{{0}, {1}, {0}});
    CHECK(plate.nozzle_group_result->is_support_dynamic_nozzle_map());
}

TEST_CASE("A single assigned nozzle retains its exact nonstandard diameter and volume type", "[3MF][PlateMetadata]")
{
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder = {{0, 100.}};
    auto grouping                                      = MultiNozzleUtils::LayeredNozzleGroupResult::create(std::vector<int>{0},
                                                                                                            std::vector<MultiNozzleUtils::NozzleInfo>{
                                                                           {"0.25", nvtStandard, 0, 0}},
                                                                                                            std::vector<unsigned int>{0});
    REQUIRE(grouping.has_value());
    result.nozzle_group_result = std::make_shared<MultiNozzleUtils::LayeredNozzleGroupResult>(*grouping);
    PlateData plate;
    plate.parse_filament_info(&result);
    CHECK(filament(plate, 0).group_id == std::vector<int>{0});
    CHECK_THAT(filament(plate, 0).nozzle_diameter, WithinAbs(.25, 1e-8));
    CHECK(filament(plate, 0).nozzle_volume_type == get_nozzle_volume_type_string(nvtStandard));
    REQUIRE(plate.nozzle_group_result.has_value());
    CHECK_FALSE(plate.nozzle_group_result->is_support_dynamic_nozzle_map());
}

TEST_CASE("Device-side static grouping supplies metadata without becoming a layered result", "[3MF][PlateMetadata]")
{
    FilamentInfo input;
    input.id       = 0;
    input.group_id = {0, 1};
    auto grouping  = MultiNozzleUtils::StaticNozzleGroupResult::create({input}, two_nozzles(), {0, 0}, {0, 1}, false);
    REQUIRE(grouping.has_value());
    GCodeProcessorResult result;
    materials(result);
    result.print_statistics.total_volumes_per_extruder = {{0, 100.}};
    result.nozzle_group_result                         = std::make_shared<MultiNozzleUtils::StaticNozzleGroupResult>(*grouping);
    PlateData plate;
    plate.parse_filament_info(&result);
    CHECK(filament(plate, 0).group_id == std::vector<int>{0, 1});
    CHECK_THAT(filament(plate, 0).nozzle_diameter, WithinAbs(.4, 1e-8));
    CHECK_FALSE(plate.nozzle_group_result.has_value());
}

TEST_CASE("Embedded presets round trip their type identity and configured value", "[3MF][PlateMetadata]")
{
    const auto type    = GENERATE(Preset::TYPE_PRINT, Preset::TYPE_FILAMENT, Preset::TYPE_PRINTER);
    const double value = type == Preset::TYPE_PRINT ? .31 : type == Preset::TYPE_FILAMENT ? 1.82 : .25;
    Preset preset      = make_preset(type, "Embedded semantic contract", value);
    LoadedPresets loaded;
    roundtrip_presets({&preset}, loaded);
    REQUIRE(loaded.presets.size() == 1);
    const Preset& actual = *loaded.presets.front();
    CHECK(actual.name == preset.name);
    CHECK(actual.type == type);
    CHECK(actual.is_project_embedded);
    const char* key = type == Preset::TYPE_PRINT ? "layer_height" : type == Preset::TYPE_FILAMENT ? "filament_diameter" : "nozzle_diameter";
    if (type == Preset::TYPE_PRINT) {
        REQUIRE(actual.config.option<ConfigOptionFloat>(key) != nullptr);
        CHECK_THAT(actual.config.opt_float(key), WithinAbs(value, 1e-6));
    } else {
        const auto* values = actual.config.option<ConfigOptionFloats>(key);
        REQUIRE(values != nullptr);
        REQUIRE(values->values.size() == 1);
        CHECK_THAT(values->values.front(), WithinAbs(value, 1e-6));
    }
}

TEST_CASE("Independent preset counters preserve two entries of every supported type", "[3MF][PlateMetadata]")
{
    std::vector<Preset> presets;
    for (const auto type : {Preset::TYPE_PRINT, Preset::TYPE_FILAMENT, Preset::TYPE_PRINTER})
        for (int index : {1, 2})
            presets.push_back(make_preset(type, "preset-" + std::to_string(int(type)) + "-" + std::to_string(index), .1 * index));
    std::vector<Preset*> inputs{nullptr};
    for (auto& preset : presets)
        inputs.push_back(&preset);
    inputs.push_back(nullptr);
    LoadedPresets loaded;
    roundtrip_presets(inputs, loaded);
    REQUIRE(loaded.presets.size() == presets.size());
    for (const Preset& expected : presets) {
        const auto found = std::find_if(loaded.presets.begin(), loaded.presets.end(),
                                        [&](const Preset* actual) { return actual->name == expected.name; });
        REQUIRE(found != loaded.presets.end());
        CHECK((*found)->type == expected.type);
        CHECK((*found)->is_project_embedded);
        const char* key = expected.type == Preset::TYPE_PRINT    ? "layer_height" :
                          expected.type == Preset::TYPE_FILAMENT ? "filament_diameter" :
                                                                   "nozzle_diameter";
        if (expected.type == Preset::TYPE_PRINT) {
            REQUIRE((*found)->config.option<ConfigOptionFloat>(key) != nullptr);
            CHECK_THAT((*found)->config.opt_float(key), WithinAbs(expected.config.opt_float(key), 1e-6));
        } else {
            const auto* actual_values = (*found)->config.option<ConfigOptionFloats>(key);
            REQUIRE(actual_values != nullptr);
            REQUIRE(actual_values->values.size() == 1);
            CHECK_THAT(actual_values->values.front(), WithinAbs(expected.config.option<ConfigOptionFloats>(key)->values.front(), 1e-6));
        }
    }
}

TEST_CASE("Unsupported embedded preset types are omitted without losing supported presets", "[3MF][PlateMetadata]")
{
    Preset ignored(Preset::TYPE_SLA_PRINT, "unsupported-SLA");
    Preset retained = make_preset(Preset::TYPE_PRINT, "retained-print", .28);
    LoadedPresets loaded;
    roundtrip_presets({&ignored, nullptr, &retained}, loaded);
    REQUIRE(loaded.presets.size() == 1);
    CHECK(loaded.presets.front()->name == retained.name);
    CHECK_THAT(loaded.presets.front()->config.opt_float("layer_height"), WithinAbs(.28, 1e-6));
}

} // namespace plate_metadata_contracts

namespace split_model_contracts {
using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Entry = std::pair<std::string, std::string>;
struct LoadedSplit
{
    Model model;
    bbs_archive_contracts::LoadedProject project;
};

std::vector<Entry> entries(const std::string& path)
{
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    REQUIRE(open_zip_reader(&zip, path));
    const ScopeGuard close([&] { close_zip_reader(&zip); });
    std::vector<Entry> result;
    for (mz_uint index = 0; index < mz_zip_reader_get_num_files(&zip); ++index) {
        mz_zip_archive_file_stat stat;
        REQUIRE(mz_zip_reader_file_stat(&zip, index, &stat));
        if (stat.m_is_directory)
            continue;
        std::string data(static_cast<size_t>(stat.m_uncomp_size), '\0');
        if (!data.empty())
            REQUIRE(mz_zip_reader_extract_to_mem(&zip, index, data.data(), data.size(), 0));
        result.emplace_back(stat.m_filename, std::move(data));
    }
    return result;
}

void rewrite(const std::string& path, const std::function<void(std::vector<Entry>&)>& edit)
{
    auto contents = entries(path);
    edit(contents);
    Zipper zip(path);
    for (const auto& [name, data] : contents)
        zip.add_entry(name, data.data(), data.size());
    zip.finalize();
}

bool is_part(const std::string& name)
{
    return name.find("3D/Objects/") == 0 && name.size() >= 6 && name.substr(name.size() - 6) == ".model";
}

void edit_first_part(const std::string& path, const std::function<void(std::string&)>& edit)
{
    rewrite(path, [&](std::vector<Entry>& data) {
        const auto found = std::find_if(data.begin(), data.end(), [](const Entry& e) { return is_part(e.first); });
        REQUIRE(found != data.end());
        edit(found->second);
    });
}

void replace_text(std::string& text, const std::string& from, const std::string& to)
{
    auto where = text.find(from);
    REQUIRE(where != std::string::npos);
    text.replace(where, from.size(), to);
}

struct Fixture
{
    ScopedTemporaryDir backup{"orca_split_source"};
    ScopedTemporaryDir loaded_backup{"orca_split_loaded"};
    // Restore reads <backup-folder>/.3mf and external parts below that folder.
    // Its error path preserves the folder, so keep a separate cleanup owner.
    ScopedTemporaryDir archive_root{"orca_split_archive"};
    boost::filesystem::path archive = archive_root.path() / "project.3mf";
    boost::filesystem::path backup_archive = archive_root.path() / ".3mf";
    Model source;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    Fixture()
    {
        source.set_backup_path(backup.string());
        auto* first                                    = source.add_object();
        first->name                                    = "first-cube";
        first->add_volume(make_cube(2., 3., 4.))->name = "first-volume";
        first->add_instance()->set_offset(Vec3d(10., 20., 0.));
        auto* second                                    = source.add_object();
        second->name                                    = "second-cube";
        second->add_volume(make_cube(5., 2., 3.))->name = "second-volume";
        second->add_instance()->set_offset(Vec3d(30., 0., 0.));
        config.set_key_value("layer_height", new ConfigOptionFloat(.21));
    }
    boost::filesystem::path restore_root() const
    {
        return backup_archive.parent_path();
    }

    boost::filesystem::path backup_part_path(const ModelObject &object) const
    {
        return restore_root() / "3D" / "Objects" /
               (object.name + "_" + std::to_string(source.get_object_backup_id(object)) + ".model");
    }

    void materialize_backup_parts()
    {
        // SaveStrategy::Backup intentionally leaves SplitModel object parts outside the .3mf.
        // Export the object XML through the ordinary SplitModel path, then package each
        // part as the standalone ZIP expected at Restore's external fallback path.
        ScopedTemporaryFile embedded(".3mf");
        PlateData plate;
        plate.config                = config;
        plate.plate_index           = 0;
        plate.objects_and_instances = {{0, 0}, {1, 0}};
        StoreParams params;
        params.path            = embedded.string();
        params.model           = &source;
        params.config          = &config;
        params.plate_data_list = {&plate};
        params.strategy        = SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::SplitModel;
        REQUIRE(store_bbs_3mf(params));

        const boost::filesystem::path root = restore_root();
        size_t part_count = 0;
        for (const auto &[name, data] : entries(embedded.string())) {
            if (!is_part(name))
                continue;
            const std::string filename = boost::filesystem::path(name).filename().string();
            const auto separator = filename.rfind('_');
            REQUIRE(separator != std::string::npos);
            const std::string object_name = filename.substr(0, separator);
            const auto object = std::find_if(source.objects.begin(), source.objects.end(),
                [&](const ModelObject *candidate) { return candidate->name == object_name; });
            REQUIRE(object != source.objects.end());
            const std::string target_name = "3D/Objects/" + object_name + "_" +
                std::to_string(source.get_object_backup_id(**object)) + ".model";
            const boost::filesystem::path destination = root / target_name;
            boost::filesystem::create_directories(destination.parent_path());
            Zipper zip(destination.string());
            zip.add_entry(target_name, data.data(), data.size());
            zip.finalize();
            ++part_count;
        }
        REQUIRE(part_count == source.objects.size());
    }

    void store(SaveStrategy extra = SaveStrategy::Default)
    {
        PlateData plate;
        plate.config                = config;
        plate.plate_index           = 0;
        plate.objects_and_instances = {{0, 0}, {1, 0}};
        StoreParams params;
        params.path            = (extra & SaveStrategy::Backup ? backup_archive : archive).string();
        params.model           = &source;
        params.config          = &config;
        params.plate_data_list = {&plate};
        params.strategy        = SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::SplitModel | extra;
        REQUIRE(store_bbs_3mf(params));
        if (extra & SaveStrategy::Backup)
            materialize_backup_parts();
    }
    bool load(LoadedSplit& result, LoadStrategy strategy = LoadStrategy::LoadModel | LoadStrategy::LoadConfig)
    {
        result.model.set_backup_path(loaded_backup.string());
        ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
        bool bbl = false, orca = false;
        Semver version;
        const boost::filesystem::path &input = (strategy & LoadStrategy::Restore) ? backup_archive : archive;
        return load_bbs_3mf(input.string().c_str(), &result.project.config, &substitutions, &result.model, &result.project.plates,
                            &result.project.presets, &bbl, &orca, &version, nullptr, strategy);
    }
};

const ModelObject& named(const Model& model, const std::string& name)
{
    auto found = std::find_if(model.objects.begin(), model.objects.end(), [&](const ModelObject* object) { return object->name == name; });
    REQUIRE(found != model.objects.end());
    return **found;
}

void check_box(const BoundingBoxf3& actual, const BoundingBoxf3& expected)
{
    for (int axis = 0; axis < 3; ++axis) {
        CHECK_THAT(actual.min[axis], WithinAbs(expected.min[axis], 1e-5));
        CHECK_THAT(actual.max[axis], WithinAbs(expected.max[axis], 1e-5));
    }
}

TEST_CASE("Split model archives store each object in a distinct referenced part", "[3MF][SplitModel]")
{
    Fixture f;
    f.store();
    auto all = entries(f.archive.string());
    std::set<std::string> part_names;
    for (const auto& [name, data] : all)
        if (is_part(name)) {
            CHECK_FALSE(data.empty());
            REQUIRE(part_names.insert(name).second);
        }
    REQUIRE(part_names.size() == 2);
    auto rels = std::find_if(all.begin(), all.end(), [](const Entry& e) { return e.first == "3D/_rels/3dmodel.model.rels"; });
    REQUIRE(rels != all.end());
    for (const auto& name : part_names)
        CHECK(rels->second.find(name) != std::string::npos);
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    REQUIRE(loaded.model.objects.size() == 2);
    for (const auto* expected : f.source.objects) {
        const auto& actual = named(loaded.model, expected->name);
        CHECK_THAT(actual.raw_mesh().volume(), WithinAbs(expected->raw_mesh().volume(), 1e-5));
        check_box(actual.bounding_box_exact(), expected->bounding_box_exact());
    }
}

TEST_CASE("Split model Restore remaps production object ids and preserves geometry",
          "[3MF][SplitModelRestore]")
{
    Fixture f;
    // Non-sequential IDs make accidental sequential reconstruction observable.
    f.source.set_object_backup_id(*f.source.objects[0], 17);
    f.source.set_object_backup_id(*f.source.objects[1], 42);
    f.store(SaveStrategy::Backup);

    LoadedSplit loaded;
    REQUIRE(f.load(loaded, LoadStrategy::Restore));
    REQUIRE(loaded.model.objects.size() == f.source.objects.size());

    for (const ModelObject *expected : f.source.objects) {
        const ModelObject &actual = named(loaded.model, expected->name);
        CHECK(actual.instances.size() == expected->instances.size());
        CHECK_THAT(actual.raw_mesh().volume(), WithinAbs(expected->raw_mesh().volume(), 1e-5));
        check_box(actual.bounding_box_exact(), expected->bounding_box_exact());
        CHECK(loaded.model.get_object_backup_id(actual) ==
              f.source.get_object_backup_id(*expected));
    }
}

TEST_CASE("Split model Restore rejects a missing referenced object part",
          "[3MF][SplitModelRestore]")
{
    Fixture f;
    f.store(SaveStrategy::Backup);
    {
        LoadedSplit intact;
        REQUIRE(f.load(intact, LoadStrategy::Restore));
        REQUIRE(intact.model.objects.size() == f.source.objects.size());
        // Restore assigns the archive's parent as the model backup path. Detach before the
        // positive control dies so Model::~Model() cannot remove the staged part directory.
        intact.model.set_backup_path("detach");
    }
    const auto missing_part = f.backup_part_path(*f.source.objects.front());
    REQUIRE(boost::filesystem::exists(missing_part));
    REQUIRE(boost::filesystem::remove(missing_part));

    LoadedSplit loaded;
    CHECK_FALSE(f.load(loaded, LoadStrategy::Restore));
    CHECK(loaded.model.objects.empty());
}

TEST_CASE("Split model archives preserve object and volume settings independently", "[3MF][SplitModel]")
{
    Fixture f;
    f.source.objects[0]->config.set_key_value("wall_loops", new ConfigOptionInt(5));
    f.source.objects[1]->config.set_key_value("wall_loops", new ConfigOptionInt(2));
    f.source.objects[0]->volumes[0]->config.set_key_value("sparse_infill_density", new ConfigOptionPercent(37.));
    f.store();
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    CHECK_THAT(loaded.project.config.opt_float("layer_height"), WithinAbs(.21, 1e-8));
    CHECK(named(loaded.model, "first-cube").config.opt_int("wall_loops") == 5);
    CHECK(named(loaded.model, "second-cube").config.opt_int("wall_loops") == 2);
    const auto& first = named(loaded.model, "first-cube");
    REQUIRE(first.volumes.size() == 1);
    CHECK_THAT(first.volumes[0]->config.opt_float("sparse_infill_density"), WithinAbs(37., 1e-8));
}

TEST_CASE("Split model instances retain their individual placement and printability", "[3MF][SplitModel]")
{
    Fixture f;
    auto* duplicate = f.source.objects[0]->add_instance();
    duplicate->set_offset(Vec3d(-5., 7., 2.));
    duplicate->printable = false;
    f.store();
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    const auto& actual = named(loaded.model, "first-cube");
    REQUIRE(actual.instances.size() == 2);
    for (size_t i = 0; i < 2; ++i) {
        CHECK(actual.instances[i]->printable == f.source.objects[0]->instances[i]->printable);
        check_box(actual.instance_bounding_box(i), f.source.objects[0]->instance_bounding_box(i));
    }
}

TEST_CASE("Split model object names survive Unicode archive part encoding", "[3MF][SplitModel]")
{
    Fixture f;
    f.source.objects[0]->name = "Größe-部品";
    f.store();
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    const auto& object = named(loaded.model, "Größe-部品");
    CHECK_THAT(object.raw_mesh().volume(), WithinAbs(24., 1e-5));
}

TEST_CASE("Split model volumes preserve geometry types and relative transforms", "[3MF][SplitModel]")
{
    Fixture f;
    auto* modifier = f.source.objects[0]->add_volume(make_cube(1., 2., 3.));
    modifier->name = "modifier";
    modifier->set_type(ModelVolumeType::PARAMETER_MODIFIER);
    modifier->set_offset(Vec3d(4., -2., 1.));
    f.store();
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    const auto& actual = named(loaded.model, "first-cube");
    REQUIRE(actual.volumes.size() == 2);
    auto found = std::find_if(actual.volumes.begin(), actual.volumes.end(), [](const ModelVolume* v) { return v->name == "modifier"; });
    REQUIRE(found != actual.volumes.end());
    CHECK((*found)->type() == ModelVolumeType::PARAMETER_MODIFIER);
    CHECK_THAT(its_volume((*found)->mesh().its), WithinAbs(6., 1e-5));
    CHECK((*found)->get_matrix().matrix().isApprox(modifier->get_matrix().matrix(), 1e-6));
}

TEST_CASE("Split model face paint retains support seam and material selections", "[3MF][SplitModel]")
{
    Fixture f;
    auto* volume = f.source.objects[0]->volumes[0];
    TriangleSelector support(volume->mesh());
    support.set_facet(0, EnforcerBlockerType::ENFORCER);
    REQUIRE(volume->supported_facets.set(support));
    TriangleSelector seam(volume->mesh());
    seam.set_facet(1, EnforcerBlockerType::BLOCKER);
    REQUIRE(volume->seam_facets.set(seam));
    TriangleSelector material(volume->mesh());
    material.set_facet(2, EnforcerBlockerType::Extruder2);
    REQUIRE(volume->mmu_segmentation_facets.set(material));
    TriangleSelector fuzzy(volume->mesh());
    fuzzy.set_facet(3, EnforcerBlockerType::ENFORCER);
    REQUIRE(volume->fuzzy_skin_facets.set(fuzzy));
    f.store();
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    const auto& actual = named(loaded.model, "first-cube");
    REQUIRE(actual.volumes.size() == 1);
    CHECK(actual.volumes[0]->supported_facets.equals(volume->supported_facets));
    CHECK(actual.volumes[0]->seam_facets.equals(volume->seam_facets));
    CHECK(actual.volumes[0]->mmu_segmentation_facets.equals(volume->mmu_segmentation_facets));
    CHECK(actual.volumes[0]->fuzzy_skin_facets.equals(volume->fuzzy_skin_facets));
}

TEST_CASE("Split model part units scale vertex geometry independently of parent units", "[3MF][SplitModel]")
{
    const auto unit     = GENERATE(std::string("centimeter"), std::string("inch"), std::string("meter"));
    const double factor = unit == "centimeter" ? 10. : unit == "inch" ? 25.4 : 1000.;
    Fixture f;
    f.store();
    rewrite(f.archive.string(), [&](std::vector<Entry>& all) {
        for (auto& [name, data] : all)
            if (is_part(name))
                replace_text(data, "unit=\"millimeter\"", "unit=\"" + unit + "\"");
    });
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    for (const auto* expected : f.source.objects) {
        const auto& actual = named(loaded.model, expected->name);
        const auto size    = actual.raw_mesh().bounding_box().size();
        const auto before  = expected->raw_mesh().bounding_box().size();
        for (int axis = 0; axis < 3; ++axis)
            CHECK_THAT(size[axis], WithinAbs(before[axis] * factor, 1e-3));
        CHECK_THAT(actual.raw_mesh().volume(),
                   WithinAbs(expected->raw_mesh().volume() * factor * factor * factor, 1e-5 * factor * factor * factor));
    }
}

TEST_CASE("Split model loading rejects a missing referenced object part", "[3MF][SplitModel]")
{
    Fixture f;
    f.store();
    rewrite(f.archive.string(), [](std::vector<Entry>& all) {
        auto found = std::find_if(all.begin(), all.end(), [](const Entry& e) { return is_part(e.first); });
        REQUIRE(found != all.end());
        all.erase(found);
    });
    LoadedSplit loaded;
    CHECK_FALSE(f.load(loaded));
}

TEST_CASE("Split model loading rejects malformed XML within an object part", "[3MF][SplitModel]")
{
    Fixture f;
    f.store();
    edit_first_part(f.archive.string(), [](std::string& data) { data.resize(data.size() / 2); });
    LoadedSplit loaded;
    CHECK_FALSE(f.load(loaded));
}

TEST_CASE("Split model loading rejects nonfinite vertex coordinates", "[3MF][SplitModel]")
{
    const auto coordinate = GENERATE(std::string("nan"), std::string("inf"), std::string("-inf"));
    Fixture f;
    f.store();
    edit_first_part(f.archive.string(), [&](std::string& data) {
        const auto vertex = data.find("<vertex ");
        REQUIRE(vertex != std::string::npos);
        const auto start = data.find("x=\"", vertex);
        REQUIRE(start != std::string::npos);
        const auto end = data.find('"', start + 3);
        REQUIRE(end != std::string::npos);
        data.replace(start + 3, end - start - 3, coordinate);
    });
    LoadedSplit loaded;
    CHECK_FALSE(f.load(loaded));
}

TEST_CASE("Split model loading rejects duplicate object identifiers within one part", "[3MF][SplitModel]")
{
    Fixture f;
    f.store();
    edit_first_part(f.archive.string(), [](std::string& data) {
        const auto start = data.find("<object ");
        REQUIRE(start != std::string::npos);
        const auto end = data.find("</object>", start);
        REQUIRE(end != std::string::npos);
        const auto object    = data.substr(start, end + 9 - start);
        const auto resources = data.find("</resources>");
        REQUIRE(resources != std::string::npos);
        data.insert(resources, object);
    });
    LoadedSplit loaded;
    CHECK_FALSE(f.load(loaded));
}

TEST_CASE("Shared meshes in split archives retain separate object identities and placements", "[3MF][SplitModel]")
{
    Fixture f;
    auto* copy = f.source.add_object(*f.source.objects[0]);
    copy->name = "shared-copy";
    copy->instances[0]->set_offset(Vec3d(40., 50., 0.));
    f.store(SaveStrategy::ShareMesh);
    LoadedSplit loaded;
    REQUIRE(f.load(loaded));
    REQUIRE(loaded.model.objects.size() == 3);
    for (const auto* expected : f.source.objects) {
        const auto& actual = named(loaded.model, expected->name);
        CHECK_THAT(actual.raw_mesh().volume(), WithinAbs(expected->raw_mesh().volume(), 1e-5));
        check_box(actual.bounding_box_exact(), expected->bounding_box_exact());
    }
}
int first_object_id(const std::string &object)
{
    const size_t object_begin = object.find("<object ");
    REQUIRE(object_begin != std::string::npos);
    const size_t object_end = object.find('>', object_begin);
    REQUIRE(object_end != std::string::npos);
    const size_t id_begin = object.find("id=\"", object_begin);
    REQUIRE(id_begin != std::string::npos);
    REQUIRE(id_begin < object_end);
    const size_t value_begin = id_begin + std::string("id=\"").size();
    const size_t value_end = object.find('"', value_begin);
    REQUIRE(value_end != std::string::npos);
    REQUIRE(value_end < object_end);
    return std::stoi(object.substr(value_begin, value_end - value_begin));
}

void replace_first_object_id(std::string &object, int old_id, int new_id)
{
    const size_t object_begin = object.find("<object ");
    REQUIRE(object_begin != std::string::npos);
    const size_t object_end = object.find('>', object_begin);
    REQUIRE(object_end != std::string::npos);
    const std::string old_attr = "id=\"" + std::to_string(old_id) + "\"";
    const std::string new_attr = "id=\"" + std::to_string(new_id) + "\"";
    const size_t attr = object.find(old_attr, object_begin);
    REQUIRE(attr != std::string::npos);
    REQUIRE(attr < object_end);
    object.replace(attr, old_attr.size(), new_attr);
}

void set_object_uuid(std::string &object, const std::string &uuid)
{
    const size_t attr = object.find("p:UUID=\"");
    REQUIRE(attr != std::string::npos);
    const size_t begin = attr + std::string("p:UUID=\"").size();
    const size_t end = object.find('"', begin);
    REQUIRE(end != std::string::npos);
    object.replace(begin, end - begin, uuid);
}

size_t strip_bbs_version_metadata(std::string &xml)
{
    size_t removed = 0;
    for (const char *name : {"BambuStudio:3mfVersion", "bamboo_slicer:Version3mf"}) {
        const std::string open = "<metadata name=\"" + std::string(name) + "\">";
        const std::string close = "</metadata>";
        while (true) {
            const size_t begin = xml.find(open);
            if (begin == std::string::npos)
                break;
            const size_t end = xml.find(close, begin + open.size());
            REQUIRE(end != std::string::npos);
            xml.erase(begin, end + close.size() - begin);
            ++removed;
        }
    }
    return removed;
}

void rewrite_restore_component_fixture(Fixture &fixture, bool keep_bbs_marker)
{
    fixture.source.set_object_backup_id(*fixture.source.objects[0], 17);
    fixture.source.set_object_backup_id(*fixture.source.objects[1], 42);
    fixture.store(SaveStrategy::Backup);

    const boost::filesystem::path part = fixture.backup_part_path(*fixture.source.objects[0]);
    auto part_entries = entries(part.string());
    auto part_model = std::find_if(part_entries.begin(), part_entries.end(), [](const Entry &entry) {
        return is_part(entry.first);
    });
    REQUIRE(part_model != part_entries.end());

    const std::string original = part_model->second;
    const size_t first_object = original.find("<object ");
    REQUIRE(first_object != std::string::npos);
    const size_t first_object_end = original.find("</object>", first_object);
    REQUIRE(first_object_end != std::string::npos);
    REQUIRE(original.find("<object ", first_object + 1) == std::string::npos);
    const std::string original_object = original.substr(first_object, first_object_end + 9 - first_object);
    const int ordinary_part_id = first_object_id(original_object);
    // materialize_backup_parts uses ordinary SplitModel numbering: the first
    // volume object is id 1, even though its backup volume id is 0x10011.
    REQUIRE(ordinary_part_id == 1);
    REQUIRE(original.find("name=\"BambuStudio:3mfVersion\"") != std::string::npos);

    std::string child = original_object;
    set_object_uuid(child, "00000001-81cb-4c03-9d28-80fed5dfa1dc");

    // The external part's first ordinary object becomes a mesh-less restore
    // wrapper.  Its UUID encodes backup object 0x11, while its serialized id
    // is deliberately 123.  The child stays at ordinary id 1 until the
    // wrapper closes and the production branch moves it to 0x10011 (65553).
    std::string wrapper = original_object;
    replace_first_object_id(wrapper, ordinary_part_id, 123);
    set_object_uuid(wrapper, "00000011-61cb-4c03-9d28-80fed5dfa1dc");
    const size_t mesh_begin = wrapper.find("<mesh>");
    REQUIRE(mesh_begin != std::string::npos);
    const size_t mesh_end = wrapper.find("</mesh>", mesh_begin);
    REQUIRE(mesh_end != std::string::npos);
    wrapper.erase(mesh_begin, mesh_end + std::string("</mesh>").size() - mesh_begin);
    const size_t wrapper_close = wrapper.rfind("</object>");
    REQUIRE(wrapper_close != std::string::npos);
    wrapper.insert(wrapper_close,
                   "\n    <components>\n"
                   "      <component objectid=\"1\" transform=\"1 0 0 0 1 0 0 0 1 0 0 0\"/>\n"
                   "    </components>\n");

    // Parse the child first so ObjectImporter can move it from ordinary id 1
    // to 65553 when the wrapper closes.
    part_model->second = original.substr(0, first_object) + child + wrapper +
                         original.substr(first_object_end + 9);
    if (!keep_bbs_marker) {
        // The ordinary sub-model exporter writes the BBS marker in its model
        // header too.  Remove inherited part-level markers for the true
        // negative control; omitting a newly-added object marker is insufficient.
        REQUIRE(strip_bbs_version_metadata(part_model->second) >= 1);
    }
    rewrite(part.string(), [&](std::vector<Entry> &all) { all = part_entries; });

    // The root backup model normally points at external volume 65553.  Route
    // it through restored id 17 when marked, and serialized id 123 when
    // unmarked. Both inputs are valid graphs in their own format.
    // Suffix2 keeps the root wrapper
    // from running the separate main-model remap before external assembly.
    rewrite(fixture.backup_archive.string(), [&](std::vector<Entry> &all) {
        auto root_model = std::find_if(all.begin(), all.end(), [](const Entry &entry) {
            return entry.first == "3D/3dmodel.model";
        });
        REQUIRE(root_model != all.end());
        const std::string root_wrapper_uuid = "p:UUID=\"00000011-61cb-4c03-9d28-80fed5dfa1dc\"";
        const std::string root_shared_uuid = "p:UUID=\"00000011-71cb-4c03-9d28-80fed5dfa1dc\"";
        REQUIRE(root_model->second.find(root_wrapper_uuid) != std::string::npos);
        replace_text(root_model->second, root_wrapper_uuid, root_shared_uuid);

        const std::string path_token = "3D/Objects/first-cube_17.model";
        const size_t path = root_model->second.find(path_token);
        REQUIRE(path != std::string::npos);
        const size_t external_id = root_model->second.find("objectid=\"65553\"", path);
        REQUIRE(external_id != std::string::npos);
        root_model->second.replace(external_id, std::string("objectid=\"65553\"").size(),
                                   "objectid=\"" + std::to_string(keep_bbs_marker ? 17 : 123) + "\"");
    });
}

TEST_CASE("Split restore component graphs preserve geometry with and without BBS markers",
          "[3MF][SplitModelRestore][Bbs3mfRestore][Bbs3mfUUID]")
{
    for (bool keep_bbs_marker : {true, false}) {
        CAPTURE(keep_bbs_marker);
        Fixture fixture;
        rewrite_restore_component_fixture(fixture, keep_bbs_marker);
        LoadedSplit loaded;
        REQUIRE(fixture.load(loaded, LoadStrategy::Restore));
        REQUIRE(loaded.model.objects.size() == 2);
        for (const ModelObject *expected : fixture.source.objects) {
            CAPTURE(expected->name);
            const auto &actual = named(loaded.model, expected->name);
            REQUIRE(actual.volumes.size() == 1);
            REQUIRE(actual.instances.size() == 1);
            const double expected_volume = expected->name == "first-cube" ? 24. : 30.;
            CHECK_THAT(actual.raw_mesh().volume(), WithinAbs(expected_volume, 1e-5));
            check_box(actual.bounding_box_exact(), expected->bounding_box_exact());
        }
    }
}

} // namespace split_model_contracts

namespace bbs_project_lifecycle_contracts
{
using namespace Slic3r;

struct LoadedArchive
{
    // Keep the backup directory alive after Model so Model::~Model can clean it.
    // Declaration order is intentional: members are destroyed in reverse order.
    ScopedTemporaryDir backup{"orca_bbs_lifecycle_load"};
    Model model;
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateDataPtrs plates;
    std::vector<Preset*> presets;
    bool is_bbl{false};
    bool is_orca{false};
    Semver version;

    LoadedArchive() { model.set_backup_path(backup.string()); }

    ~LoadedArchive()
    {
        release_PlateData_list(plates);
        for (Preset *preset : presets)
            delete preset;
        presets.clear();
    }

    bool load(const std::string &path, BBLProject *project = nullptr)
    {
        ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
        return load_bbs_3mf(path.c_str(), &config, &substitutions, &model, &plates, &presets,
                            &is_bbl, &is_orca, &version, nullptr,
                            LoadStrategy::LoadModel | LoadStrategy::LoadConfig,
                            project, 0);
    }
};
void check_plate_mapping(const PlateData &plate0, const PlateData &plate1)
{
    // The importer stores plate records in obj_inst_map; it does not
    // reconstruct objects_and_instances. Identify the first object's key from
    // the requested plate relationship instead of depending on exporter IDs.
    REQUIRE(plate0.obj_inst_map.size() == 2);
    REQUIRE(plate1.obj_inst_map.size() == 2);

    const auto first_plate1 = std::find_if(
        plate1.obj_inst_map.begin(), plate1.obj_inst_map.end(),
        [](const auto &entry) { return entry.second.first == 1; });
    REQUIRE(first_plate1 != plate1.obj_inst_map.end());
    const int first_object_key = first_plate1->first;

    const auto first_plate0 = plate0.obj_inst_map.find(first_object_key);
    REQUIRE(first_plate0 != plate0.obj_inst_map.end());
    CHECK(first_plate0->second.first == 0);

    const auto remaining_plate1 = std::find_if(
        plate1.obj_inst_map.begin(), plate1.obj_inst_map.end(),
        [first_object_key](const auto &entry) { return entry.first != first_object_key; });
    REQUIRE(remaining_plate1 != plate1.obj_inst_map.end());
    const int remaining_object_key = remaining_plate1->first;

    const auto remaining_plate0 = plate0.obj_inst_map.find(remaining_object_key);
    REQUIRE(remaining_plate0 != plate0.obj_inst_map.end());
    CHECK(remaining_plate1->second.first == 0);
    CHECK(remaining_plate0->second.first == 0);
}

void make_two_plate_model(Model &model, PlateData &plate0, PlateData &plate1)
{
    ModelObject *first = model.add_object("first-part", "first.stl", make_cube(10., 11., 12.));
    first->module_name = "first-module";
    first->add_instance();
    first->add_instance()->set_offset(Vec3d(30., 5., 0.));

    ModelObject *second = model.add_object("second-part", "second.stl", make_cube(6., 7., 8.));
    second->module_name = "second-module";
    second->add_instance()->set_offset(Vec3d(60., 10., 0.));

    model.add_material("PLA")->attributes["filament_type"] = "PLA";
    model.add_material("PETG")->attributes["filament_type"] = "PETG";
    first->volumes.front()->set_material_id("PLA");
    second->volumes.front()->set_material_id("PETG");

    plate0.plate_index = 0;
    plate0.plate_name = "first plate";
    plate0.objects_and_instances = {{0, 0}, {1, 0}};
    plate0.locked = false;

    plate1.plate_index = 1;
    plate1.plate_name = "second plate";
    plate1.objects_and_instances = {{0, 1}, {1, 0}};
    plate1.locked = true;
}

Preset make_embedded_preset(Preset::Type type, const std::string &name, double value)
{
    Preset preset(type, name);
    if (type == Preset::TYPE_PRINT) {
        preset.config.set_key_value("print_settings_id", new ConfigOptionString(name));
        preset.config.set_key_value("layer_height", new ConfigOptionFloat(value));
    } else if (type == Preset::TYPE_FILAMENT) {
        preset.config.set_key_value("filament_settings_id", new ConfigOptionStrings({name}));
        preset.config.set_key_value("filament_diameter", new ConfigOptionFloats({value}));
    } else if (type == Preset::TYPE_PRINTER) {
        preset.config.set_key_value("printer_settings_id", new ConfigOptionString(name));
        preset.config.set_key_value("nozzle_diameter", new ConfigOptionFloats({value}));
    }
    return preset;
}

void store_archive(const std::string &path, Model &model, DynamicPrintConfig &config,
                   PlateDataPtrs plates, std::vector<Preset*> presets = {},
                   BBLProject *project = nullptr)
{
    StoreParams params;
    params.path = path;
    params.model = &model;
    params.config = &config;
    params.plate_data_list = std::move(plates);
    params.project_presets = std::move(presets);
    params.project = project;
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));
}

TEST_CASE("BBS project lifecycle preserves model identity, metadata, and two plates",
          "[3MF][Bbs3mfLifecycle][Project]")
{
    ScopedTemporaryDir backup{"orca_bbs_lifecycle_store"};
    ScopedTemporaryFile archive{".3mf"};
    Model source;
    source.set_backup_path(backup.string());
    source.design_info = std::make_shared<ModelDesignInfo>();
    source.design_info->Designer = "fixture-designer";
    source.model_info = std::make_shared<ModelInfo>();
    source.model_info->model_name = "Lifecycle model";
    source.model_info->origin = "project-import";
    source.model_info->description = "metadata survives <xml> escaping";
    source.model_info->metadata_items["lifecycle-key"] = "lifecycle-value";
    // ProfileInfo is imported from these model metadata keys.  Keep the
    // fixture on the public model metadata path because the exporter does not
    // serialize Model::profile_info directly.
    source.model_info->metadata_items["ProfileTitle"] = "Profile title";
    source.model_info->metadata_items["ProfileDescription"] = "Profile description";

    PlateData plate0;
    PlateData plate1;
    make_two_plate_model(source, plate0, plate1);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("layer_height", new ConfigOptionFloat(0.24));
    BBLProject project("Lifecycle project");
    project.project_model_id = "model-42";
    project.project_country_code = "US";

    store_archive(archive.string(), source, config, {&plate0, &plate1}, {}, &project);

    LoadedArchive loaded;
    BBLProject loaded_project;
    REQUIRE(loaded.load(archive.string(), &loaded_project));
    REQUIRE(loaded.model.objects.size() == 2);
    REQUIRE(loaded.plates.size() == 2);

    REQUIRE(loaded.model.model_info != nullptr);
    CHECK(loaded.model.model_info->model_name == "Lifecycle model");
    CHECK(loaded.model.model_info->origin == "project-import");
    CHECK(loaded.model.model_info->description == "metadata survives <xml> escaping");
    CHECK(loaded.model.model_info->metadata_items.at("lifecycle-key") == "lifecycle-value");
    REQUIRE(loaded.model.design_info != nullptr);
    CHECK(loaded.model.design_info->Designer == "fixture-designer");
    REQUIRE(loaded.model.profile_info != nullptr);
    CHECK(loaded.model.profile_info->ProfileTile == "Profile title");
    CHECK(loaded.model.profile_info->ProfileDescription == "Profile description");
    REQUIRE(loaded.model.objects[0] != nullptr);
    REQUIRE(loaded.model.objects[1] != nullptr);
    CHECK(loaded.model.objects[0]->name == source.objects[0]->name);
    CHECK(loaded.model.objects[1]->name == source.objects[1]->name);
    CHECK(loaded.model.objects[0]->instances.size() == source.objects[0]->instances.size());
    CHECK(loaded.model.objects[1]->instances.size() == source.objects[1]->instances.size());

    CHECK(loaded_project.project_model_id == "model-42");
    CHECK(loaded_project.project_country_code == "US");
    CHECK(loaded.plates[0]->plate_name == "first plate");
    CHECK(loaded.plates[1]->plate_name == "second plate");
    CHECK_FALSE(loaded.plates[0]->locked);
    CHECK(loaded.plates[1]->locked);
    check_plate_mapping(*loaded.plates[0], *loaded.plates[1]);
    REQUIRE(loaded.config.option<ConfigOptionFloat>("layer_height") != nullptr);
    CHECK_THAT(loaded.config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.24, 1e-6));
}

TEST_CASE("BBS project lifecycle preserves project identity beside embedded presets",
          "[3MF][Bbs3mfLifecycle][Presets]")
{
    ScopedTemporaryDir backup{"orca_bbs_lifecycle_preset_store"};
    ScopedTemporaryFile archive{".3mf"};
    Model source;
    source.set_backup_path(backup.string());
    ModelObject *object = source.add_object();
    object->add_volume(make_cube(4., 5., 6.));
    object->add_instance();
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};

    BBLProject project("Preset project");
    project.project_model_id = "preset-model-7";
    project.project_country_code = "DE";

    Preset print = make_embedded_preset(Preset::TYPE_PRINT, "embedded-print", 0.31);
    Preset filament = make_embedded_preset(Preset::TYPE_FILAMENT, "embedded-filament", 1.82);
    Preset printer = make_embedded_preset(Preset::TYPE_PRINTER, "embedded-printer", 0.25);
    store_archive(archive.string(), source, config, {&plate}, {&print, &filament, &printer}, &project);

    LoadedArchive loaded;
    BBLProject loaded_project;
    REQUIRE(loaded.load(archive.string(), &loaded_project));
    CHECK(loaded_project.project_model_id == "preset-model-7");
    CHECK(loaded_project.project_country_code == "DE");
    REQUIRE(loaded.presets.size() == 3);
    for (const Preset *expected : {&print, &filament, &printer}) {
        const auto found = std::find_if(loaded.presets.begin(), loaded.presets.end(),
                                        [&](const Preset *actual) { return actual->name == expected->name; });
        REQUIRE(found != loaded.presets.end());
        CHECK((*found)->type == expected->type);
        CHECK((*found)->is_project_embedded);
    }
    const auto print_found = std::find_if(loaded.presets.begin(), loaded.presets.end(),
                                          [](const Preset *preset) { return preset->name == "embedded-print"; });
    REQUIRE(print_found != loaded.presets.end());
    CHECK_THAT((*print_found)->config.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-6));
}

} // namespace bbs_project_lifecycle_contracts

namespace bbs_selective_plate_contracts {
using namespace Slic3r;
using namespace bbs_project_lifecycle_contracts;

namespace {

void make_selective_two_plate_model(Model &model, PlateData &plate0, PlateData &plate1)
{
    ModelObject *alpha = model.add_object("plate-alpha", "alpha.stl", make_cube(10., 11., 12.));
    alpha->add_instance();

    ModelObject *beta = model.add_object("plate-beta", "beta.stl", make_cube(6., 7., 8.));
    beta->add_instance();

    plate0.plate_index = 0;
    plate0.plate_name = "alpha plate";
    plate0.objects_and_instances = {{0, 0}};

    plate1.plate_index = 1;
    plate1.plate_name = "beta plate";
    plate1.objects_and_instances = {{1, 0}};
}

bool load_selected_plate(const std::string &path, int plate_id, LoadedArchive &loaded)
{
    ConfigSubstitutionContext substitutions{ForwardCompatibilitySubstitutionRule::Enable};
    return load_bbs_3mf(path.c_str(), &loaded.config, &substitutions, &loaded.model, &loaded.plates,
                        &loaded.presets, &loaded.is_bbl, &loaded.is_orca, &loaded.version, nullptr,
                        LoadStrategy::LoadModel | LoadStrategy::LoadConfig, nullptr, plate_id);
}

const ModelObject *find_named_object(const Model &model, const std::string &name)
{
    const auto found = std::find_if(model.objects.begin(), model.objects.end(),
                                    [&](const ModelObject *object) { return object->name == name; });
    return found == model.objects.end() ? nullptr : *found;
}

void check_geometry(const ModelObject &actual, const ModelObject &expected)
{
    REQUIRE(actual.volumes.size() == expected.volumes.size());
    REQUIRE(actual.instances.size() == expected.instances.size());
    CHECK_THAT(actual.raw_mesh().volume(), Catch::Matchers::WithinRel(expected.raw_mesh().volume(), 1e-5f));
    const BoundingBoxf3 &actual_box = actual.bounding_box_exact();
    const BoundingBoxf3 &expected_box = expected.bounding_box_exact();
    for (int axis = 0; axis < 3; ++axis) {
        CHECK_THAT(actual_box.min[axis], Catch::Matchers::WithinAbs(expected_box.min[axis], 1e-5));
        CHECK_THAT(actual_box.max[axis], Catch::Matchers::WithinAbs(expected_box.max[axis], 1e-5));
    }
}

void check_loaded_id(const ModelObject &actual, const ModelObject &expected,
                     const std::pair<int, int> &archive_mapping)
{
    REQUIRE(actual.instances.size() == 1);
    REQUIRE(expected.instances.size() == 1);
    CHECK(archive_mapping.first == 0);
    CHECK(archive_mapping.second == static_cast<int>(expected.instances.front()->id().id));
    CHECK(actual.instances.front()->loaded_id == static_cast<size_t>(archive_mapping.second));
}

} // namespace

TEST_CASE("BBS 3MF plate selection loads only the requested plate objects",
          "[3MF][Bbs3mfSelectivePlate]")
{
    ScopedTemporaryDir source_backup("orca_bbs_selective_plate_store");
    ScopedTemporaryFile archive{".3mf"};
    Model source;
    source.set_backup_path(source_backup.string());
    PlateData plate0;
    PlateData plate1;
    make_selective_two_plate_model(source, plate0, plate1);

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    store_archive(archive.string(), source, config, {&plate0, &plate1});

    const std::array<int, 3> selectors{{0, 1, 2}};
    for (const int plate_id : selectors) {
        INFO("plate_id " << plate_id);
        LoadedArchive loaded;
        REQUIRE(load_selected_plate(archive.string(), plate_id, loaded));
        REQUIRE(loaded.plates.size() == 2);
        CHECK(loaded.plates[0]->plate_name == "alpha plate");
        CHECK(loaded.plates[1]->plate_name == "beta plate");
        REQUIRE(loaded.plates[0]->obj_inst_map.size() == 1);
        REQUIRE(loaded.plates[1]->obj_inst_map.size() == 1);
        const auto alpha_mapping = loaded.plates[0]->obj_inst_map.begin();
        const auto beta_mapping = loaded.plates[1]->obj_inst_map.begin();
        CHECK(alpha_mapping->first != beta_mapping->first);

        if (plate_id == 0) {
            REQUIRE(loaded.model.objects.size() == 2);
            const ModelObject *alpha = find_named_object(loaded.model, "plate-alpha");
            const ModelObject *beta = find_named_object(loaded.model, "plate-beta");
            REQUIRE(alpha != nullptr);
            REQUIRE(beta != nullptr);
            check_geometry(*alpha, *source.objects[0]);
            check_geometry(*beta, *source.objects[1]);
            check_loaded_id(*alpha, *source.objects[0], alpha_mapping->second);
            check_loaded_id(*beta, *source.objects[1], beta_mapping->second);
        } else if (plate_id == 1) {
            REQUIRE(loaded.model.objects.size() == 1);
            const ModelObject *alpha = find_named_object(loaded.model, "plate-alpha");
            REQUIRE(alpha != nullptr);
            CHECK(find_named_object(loaded.model, "plate-beta") == nullptr);
            check_geometry(*alpha, *source.objects[0]);
            check_loaded_id(*alpha, *source.objects[0], alpha_mapping->second);
        } else {
            REQUIRE(loaded.model.objects.size() == 1);
            const ModelObject *beta = find_named_object(loaded.model, "plate-beta");
            REQUIRE(beta != nullptr);
            CHECK(find_named_object(loaded.model, "plate-alpha") == nullptr);
            check_geometry(*beta, *source.objects[1]);
            check_loaded_id(*beta, *source.objects[1], beta_mapping->second);
        }
    }
}

TEST_CASE("BBS restore probe reports a staged archive and origin", "[3MF][Bbs3mfRecovery]")
{
    ScopedTemporaryDir backup("orca_bbs_restore_probe");
    const boost::filesystem::path archive = backup.path() / ".3mf";
    const boost::filesystem::path origin_file = backup.path() / "origin.txt";
    {
        boost::nowide::ofstream output(archive.string(), std::ios::binary);
        REQUIRE(output.good());
        output << "staged-archive-placeholder";
    }
    {
        boost::nowide::ofstream output(origin_file.string(), std::ios::binary);
        REQUIRE(output.good());
        output << "/source/project.3mf";
    }

    std::string path = backup.string();
    std::string origin = "stale-origin";
    REQUIRE(has_restore_data(path, origin));
    CHECK(boost::filesystem::path(path).generic_string() == archive.generic_string());
    CHECK(origin == "/source/project.3mf");

    const boost::filesystem::path missing_path = backup.path() / "missing-backup";
    std::string missing = missing_path.string();
    origin = "unchanged-origin";
    CHECK_FALSE(has_restore_data(missing, origin));
    CHECK(boost::filesystem::path(missing).generic_string() == missing_path.generic_string());
    CHECK(origin == "unchanged-origin");
}

TEST_CASE("BBS restore probe rejects empty and locked paths", "[3MF][Bbs3mfRecovery]")
{
    ScopedTemporaryDir backup("orca_bbs_restore_lock");
    const boost::filesystem::path archive = backup.path() / ".3mf";
    const boost::filesystem::path lock = backup.path() / "lock.txt";
    {
        boost::nowide::ofstream output(archive.string(), std::ios::binary);
        REQUIRE(output.good());
        output << "staged-archive-placeholder";
    }

    const unsigned current_pid = get_current_pid();
    const std::string current_process_name = get_process_name(static_cast<int>(current_pid));
    REQUIRE_FALSE(current_process_name.empty());
    {
        boost::nowide::ofstream output(lock.string(), std::ios::binary);
        REQUIRE(output.good());
        output << current_pid;
    }

    std::string locked = backup.string();
    std::string origin = "old-origin";
    CHECK_FALSE(has_restore_data(locked, origin));
    CHECK(boost::filesystem::path(locked).generic_string() == backup.path().generic_string());
    CHECK(origin == "<lock>");

    {
        boost::nowide::ofstream output(lock.string(), std::ios::binary | std::ios::trunc);
        REQUIRE(output.good());
        output << "not-a-process-id";
    }
    std::string malformed = backup.string();
    origin = "malformed-lock-origin";
    CHECK_FALSE(has_restore_data(malformed, origin));
    CHECK(boost::filesystem::path(malformed).generic_string() == backup.path().generic_string());
    CHECK(origin == "malformed-lock-origin");

    std::string empty;
    origin = "old-origin";
    CHECK_FALSE(has_restore_data(empty, origin));
    CHECK(empty.empty());
    CHECK(origin == "<lock>");
}

} // namespace bbs_selective_plate_contracts
namespace bbs_thumbnail_downsample_contracts {

using namespace Slic3r;

TEST_CASE("BBS plate thumbnails downsample every source block into the small preview", "[3MF][Bbs3mfThumbnailDownsample]")
{
    const auto [width, height] = GENERATE(table<int, int>({{256, 256}, {384, 256}}));
    CAPTURE(width, height);
    REQUIRE(width % PLATE_THUMBNAIL_SMALL_WIDTH == 0);
    REQUIRE(height % PLATE_THUMBNAIL_SMALL_HEIGHT == 0);
    const int sw = width / PLATE_THUMBNAIL_SMALL_WIDTH;
    const int sh = height / PLATE_THUMBNAIL_SMALL_HEIGHT;
    REQUIRE(sw > 0);
    REQUIRE(sh > 0);

    ThumbnailData thumbnail;
    thumbnail.set(width, height);
    for (int y = 0; y < thumbnail.height; ++y) {
        for (int x = 0; x < thumbnail.width; ++x) {
            const int block_x = x / sw;
            const int block_y = y / sh;
            const unsigned char base = static_cast<unsigned char>(20 + 13 * ((block_x + 3 * block_y) % 12));
            const unsigned char delta = static_cast<unsigned char>((x % sw) * 7 + (y % sh) * 11);
            const size_t pixel = 4 * (static_cast<size_t>(y) * thumbnail.width + x);
            thumbnail.pixels[pixel + 0] = static_cast<unsigned char>(base + delta);
            thumbnail.pixels[pixel + 1] = static_cast<unsigned char>(base + 3 + delta);
            thumbnail.pixels[pixel + 2] = static_cast<unsigned char>(base + 6 + delta);
            thumbnail.pixels[pixel + 3] = static_cast<unsigned char>(170 + delta);
        }
    }

    std::vector<unsigned char> expected(4u * PLATE_THUMBNAIL_SMALL_WIDTH * PLATE_THUMBNAIL_SMALL_HEIGHT);
    for (int output_y = 0; output_y < PLATE_THUMBNAIL_SMALL_HEIGHT; ++output_y) {
        for (int output_x = 0; output_x < PLATE_THUMBNAIL_SMALL_WIDTH; ++output_x) {
            std::array<int, 4> sums{};
            for (int dy = 0; dy < sh; ++dy) {
                for (int dx = 0; dx < sw; ++dx) {
                    const size_t source = 4 * (static_cast<size_t>(output_y * sh + dy) * thumbnail.width +
                                                output_x * sw + dx);
                    for (int channel = 0; channel < 4; ++channel)
                        sums[channel] += thumbnail.pixels[source + channel];
                }
            }
            const size_t output = 4 * (static_cast<size_t>(output_y) * PLATE_THUMBNAIL_SMALL_WIDTH + output_x);
            for (int channel = 0; channel < 4; ++channel)
                expected[output + channel] = static_cast<unsigned char>(sums[channel] / (sw * sh));
        }
    }

    ScopedTemporaryFile archive(".3mf");
    bbs_3mf_metadata_contracts::store_followup(
        archive.string(),
        [&](Model &, DynamicPrintConfig &, PlateData &plate, StoreParams &params, ScopedTemporaryDir &) {
            plate.plate_thumbnail = thumbnail;
            params.thumbnail_data = {&thumbnail};
        });

    const std::string small = bbs_3mf_metadata_contracts::png_entry(
        archive.string(), "Metadata/plate_1_small.png",
        PLATE_THUMBNAIL_SMALL_WIDTH, PLATE_THUMBNAIL_SMALL_HEIGHT);
    REQUIRE(small.size() == expected.size());
    size_t mismatch = std::string::npos;
    for (size_t i = 0; i < expected.size(); ++i) {
        if (static_cast<unsigned char>(small[i]) != expected[i]) {
            mismatch = i;
            break;
        }
    }
    INFO("first RGBA mismatch byte " << mismatch << " of " << expected.size()
         << " for source " << width << "x" << height << " (sw=" << sw << ", sh=" << sh << ")");
    REQUIRE(mismatch == std::string::npos);
}

} // namespace bbs_thumbnail_downsample_contracts

namespace bbs_source_metadata_contracts {

using namespace Slic3r;

TEST_CASE("BBS 3MF preserves source-volume provenance and path policy",
          "[3MF][Bbs3mfSourceMetadata]")
{
    const bool full_path_sources = GENERATE(false, true);
    CAPTURE(full_path_sources);

    Model source;
    ModelObject *object = source.add_object();
    object->name = "source-provenance-object";
    ModelVolume *part = object->add_volume(make_cube(2., 3., 4.));
    part->name = "sourced-part";
    part->source.input_file = "/tmp/orca-source/component.stl";
    part->source.object_idx = 7;
    part->source.volume_idx = 11;
    part->source.mesh_offset = Vec3d(1.25, -2.5, 3.75);
    part->source.is_converted_from_inches = true;

    // This modifier is the disabled provenance control: it has no source record,
    // so the importer must not manufacture source IDs, offsets, or conversion flags.
    ModelVolume *modifier = object->add_volume(make_cube(1., 2., 1.), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->name = "unsourced-modifier";
    object->add_instance();

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    ScopedTemporaryDir backup("orca_bbs_source_metadata_store");
    source.set_backup_path(backup.string());
    PlateData plate;
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};
    PlateDataPtrs plates{&plate};
    ScopedTemporaryFile archive(".3mf");

    const SaveStrategy strategy = SaveStrategy::Zip64 | SaveStrategy::Silence |
                                   (full_path_sources ? SaveStrategy::FullPathSources : SaveStrategy::Default);
    bbs_archive_contracts::store_model(archive.string(), source, config, plates, strategy);

    Model loaded_model;
    bbs_archive_contracts::LoadedProject loaded;
    REQUIRE(bbs_archive_contracts::load_model(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 2);

    const ModelVolume *loaded_part = loaded_model.objects.front()->volumes[0];
    const ModelVolume *loaded_modifier = loaded_model.objects.front()->volumes[1];
    REQUIRE(loaded_part != nullptr);
    REQUIRE(loaded_modifier != nullptr);

    CHECK(loaded_part->name == "sourced-part");
    CHECK(loaded_part->type() == ModelVolumeType::MODEL_PART);
    CHECK_THAT(its_volume(loaded_part->mesh().its), Catch::Matchers::WithinAbs(24., 1e-5));
    const BoundingBoxf3 part_box = loaded_part->mesh().bounding_box();
    CHECK_THAT(part_box.max.x() - part_box.min.x(), Catch::Matchers::WithinAbs(2., 1e-6));
    CHECK_THAT(part_box.max.y() - part_box.min.y(), Catch::Matchers::WithinAbs(3., 1e-6));
    CHECK_THAT(part_box.max.z() - part_box.min.z(), Catch::Matchers::WithinAbs(4., 1e-6));
    CHECK(loaded_part->source.input_file ==
          (full_path_sources ? part->source.input_file : std::string("component.stl")));
    CHECK(loaded_part->source.object_idx == 7);
    CHECK(loaded_part->source.volume_idx == 11);
    CHECK_THAT(loaded_part->source.mesh_offset.x(), Catch::Matchers::WithinAbs(1.25, 1e-6));
    CHECK_THAT(loaded_part->source.mesh_offset.y(), Catch::Matchers::WithinAbs(-2.5, 1e-6));
    CHECK_THAT(loaded_part->source.mesh_offset.z(), Catch::Matchers::WithinAbs(3.75, 1e-6));
    CHECK(loaded_part->source.is_converted_from_inches);
    CHECK_FALSE(loaded_part->source.is_converted_from_meters);

    CHECK(loaded_modifier->name == "unsourced-modifier");
    CHECK(loaded_modifier->type() == ModelVolumeType::PARAMETER_MODIFIER);
    CHECK_THAT(its_volume(loaded_modifier->mesh().its), Catch::Matchers::WithinAbs(2., 1e-5));
    CHECK(loaded_modifier->source.input_file.empty());
    CHECK(loaded_modifier->source.object_idx == -1);
    CHECK(loaded_modifier->source.volume_idx == -1);
    CHECK_THAT(loaded_modifier->source.mesh_offset.x(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(loaded_modifier->source.mesh_offset.y(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_THAT(loaded_modifier->source.mesh_offset.z(), Catch::Matchers::WithinAbs(0., 1e-9));
    CHECK_FALSE(loaded_modifier->source.is_converted_from_inches);
    CHECK_FALSE(loaded_modifier->source.is_converted_from_meters);
}

} // namespace bbs_source_metadata_contracts

namespace bbs_3mf_metadata_contracts {

using namespace Slic3r;

TEST_CASE("BBS selected plate relationships target its generated thumbnails", "[3MF][Bbs3mfMetadata][BbsPlateRelationships]")
{
    ScopedTemporaryFile archive(".3mf");
    ScopedTemporaryDir backup("orca_bbs_plate_relationships");
    Model model = archive_variant_contracts::make_cube_model();
    model.add_object(*model.objects.front());
    REQUIRE(model.objects.size() == 2);
    REQUIRE(model.objects.back()->instances.size() == 1);
    model.set_backup_path(backup.string());
    ScopeGuard detach_backup([&model] { model.set_backup_path("detach"); });

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate0;
    PlateData plate1;
    plate0.config = config;
    plate1.config = config;
    plate0.plate_index = 0;
    plate1.plate_index = 1;
    plate0.objects_and_instances = {{0, 0}};
    plate1.objects_and_instances = {{1, 0}};
    PlateDataPtrs plates{&plate0, &plate1};

    ThumbnailData thumbnail0;
    ThumbnailData thumbnail1;
    thumbnail0.set(PLATE_THUMBNAIL_SMALL_WIDTH, PLATE_THUMBNAIL_SMALL_HEIGHT);
    thumbnail1.set(PLATE_THUMBNAIL_SMALL_WIDTH, PLATE_THUMBNAIL_SMALL_HEIGHT);
    std::fill(thumbnail0.pixels.begin(), thumbnail0.pixels.end(), 0x11);
    std::fill(thumbnail1.pixels.begin(), thumbnail1.pixels.end(), 0x22);
    REQUIRE(thumbnail0.is_valid());
    REQUIRE(thumbnail1.is_valid());

    StoreParams params;
    params.path = archive.string();
    params.model = &model;
    params.config = &config;
    params.plate_data_list = plates;
    params.thumbnail_data = {&thumbnail0, &thumbnail1};
    params.export_plate_idx = 1;
    params.strategy = SaveStrategy::Zip64 | SaveStrategy::Silence;
    REQUIRE(store_bbs_3mf(params));

    std::string relationships;
    REQUIRE(read_3mf_entry(archive.string(), relationships, "_rels/.rels"));
    std::stringstream relationship_stream(relationships);
    boost::property_tree::ptree relationship_tree;
    REQUIRE_NOTHROW(boost::property_tree::read_xml(relationship_stream, relationship_tree));
    const auto &relationship_root = relationship_tree.get_child("Relationships");
    const auto has_relationship = [&](const std::string &type, const std::string &target) {
        return std::any_of(relationship_root.begin(), relationship_root.end(),
                           [&](const boost::property_tree::ptree::value_type &child) {
                               if (child.first != "Relationship")
                                   return false;
                               const auto &attributes = child.second.get_child("<xmlattr>");
                               return attributes.get<std::string>("Type") == type &&
                                      attributes.get<std::string>("Target") == target;
                           });
    };
    const std::string thumbnail_type = "http://schemas.openxmlformats.org/package/2006/relationships/metadata/thumbnail";
    const std::string middle_type = "http://schemas.bambulab.com/package/2021/cover-thumbnail-middle";
    const std::string small_type = "http://schemas.bambulab.com/package/2021/cover-thumbnail-small";
    CHECK(has_relationship(thumbnail_type, "/Metadata/plate_2.png"));
    CHECK(has_relationship(middle_type, "/Metadata/plate_2.png"));
    CHECK(has_relationship(small_type, "/Metadata/plate_2_small.png"));
    CHECK_FALSE(has_relationship(thumbnail_type, "/Metadata/plate_1.png"));
    CHECK_FALSE(has_relationship(middle_type, "/Metadata/plate_1.png"));
    CHECK_FALSE(has_relationship(small_type, "/Metadata/plate_1_small.png"));

    const auto check_thumbnail = [&](const std::string &entry, unsigned char expected_channel) {
        const std::string decoded = png_entry(archive.string(), entry,
                                               PLATE_THUMBNAIL_SMALL_WIDTH,
                                               PLATE_THUMBNAIL_SMALL_HEIGHT);
        REQUIRE(decoded.size() == size_t(PLATE_THUMBNAIL_SMALL_WIDTH) * PLATE_THUMBNAIL_SMALL_HEIGHT * 4);
        CHECK(std::all_of(decoded.begin(), decoded.end(), [&](char value) {
            return static_cast<unsigned char>(value) == expected_channel;
        }));
    };

    check_thumbnail("Metadata/plate_1.png", 0x11);
    check_thumbnail("Metadata/plate_1_small.png", 0x11);
    check_thumbnail("Metadata/plate_2.png", 0x22);
    check_thumbnail("Metadata/plate_2_small.png", 0x22);
}

} // namespace bbs_3mf_metadata_contracts

namespace bbs_import_color_contracts {

// Root objects load from 3D/3dmodel.model, while a build item addresses the object in the related
// 3D/Objects/part.model. Each model owns its color groups; IDs are unique package-wide.
static std::string tetra_mesh_xml()
{
    return R"(<mesh>
      <vertices>
        <vertex x="0" y="0" z="0"/>
        <vertex x="10" y="0" z="0"/>
        <vertex x="0" y="10" z="0"/>
        <vertex x="0" y="0" z="10"/>
      </vertices>
      <triangles>
        <triangle v1="0" v2="2" v3="1"/>
        <triangle v1="0" v2="1" v3="3"/>
        <triangle v1="0" v2="3" v3="2"/>
        <triangle v1="1" v2="2" v3="3"/>
      </triangles>
    </mesh>)";
}

static void write_color_group_archive(const std::string &path)
{
    const std::string root_model =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
       xmlns:m="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06"
       requiredextensions="p">
  <metadata name="Application">OrcaSlicer-2.0.0</metadata>
  <resources>
    <m:colorgroup id="22"><m:color color="#2244AAFF"/></m:colorgroup>
    <m:colorgroup id="33"><m:color color="#CC3322FF"/></m:colorgroup>
    <object id="200" name="root blue part" type="model" pid="22">)" +
        tetra_mesh_xml() +
        R"(</object>
    <object id="300" name="root reused red part" type="model" pid="33">)" +
        tetra_mesh_xml() +
        R"(</object>
  </resources>
  <build>
    <item p:path="/3D/Objects/part.model" objectid="1" printable="1"/>
    <item objectid="200" printable="1"/>
    <item objectid="300" printable="1"/>
  </build>
</model>)";

    const std::string extension_model =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<model unit="millimeter" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02"
       xmlns:m="http://schemas.bambulab.com/package/2021" xmlns:p="http://schemas.microsoft.com/3dmanufacturing/production/2015/06"
       requiredextensions="p">
  <metadata name="Application">OrcaSlicer-2.0.0</metadata>
  <resources>
    <m:colorgroup id="11"><m:color color="#CC3322FF"/></m:colorgroup>
    <object id="1" name="extension red part" type="model" pid="11">)" +
        tetra_mesh_xml() +
        R"(</object>
  </resources>
</model>)";

    const std::string content_types =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/>
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
</Types>)";
    const std::string package_relationships =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/3dmodel.model" Id="rel-root"/>
</Relationships>)";
    const std::string model_relationships =
        R"(<?xml version="1.0" encoding="UTF-8"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel" Target="/3D/Objects/part.model" Id="rel-part"/>
</Relationships>)";

    Zipper archive(path);
    archive.add_entry("[Content_Types].xml", content_types.data(), content_types.size());
    archive.add_entry("_rels/.rels", package_relationships.data(), package_relationships.size());
    archive.add_entry("3D/3dmodel.model", root_model.data(), root_model.size());
    archive.add_entry("3D/_rels/3dmodel.model.rels", model_relationships.data(), model_relationships.size());
    archive.add_entry("3D/Objects/part.model", extension_model.data(), extension_model.size());
    archive.finalize();
}

static std::set<std::array<int, 3>> normalized_tetra_faces(const TriangleMesh &mesh)
{
    std::set<std::array<int, 3>> faces;
    for (const Vec3i32 &face : mesh.its.indices) {
        std::array<int, 3> normalized{face[0], face[1], face[2]};
        std::sort(normalized.begin(), normalized.end());
        faces.insert(normalized);
    }
    return faces;
}

TEST_CASE("BBS import maps scoped color groups and reuses equal colors publicly",
          "[3MF][Bbs3mfImportGaps][ColorGroups]")
{
    ScopedTemporaryFile archive(".3mf");
    write_color_group_archive(archive.string());

    // load_bbs owns its backup directory, plates, and presets through LoadedProject. Declare
    // it before Model so the model is destroyed before that backup owner at scope exit.
    archive_variant_contracts::BbsLoaded loaded_project;
    Model loaded_model;
    REQUIRE(archive_variant_contracts::load_bbs(archive.string(), loaded_model, loaded_project));
    REQUIRE(loaded_model.objects.size() == 3);

    const std::set<std::array<int, 3>> expected_faces{{0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}};
    const std::map<std::string, int> expected_extruders{
        {"extension red part", 1},
        {"root blue part", 2},
        {"root reused red part", 1},
    };
    std::map<std::string, int> actual_extruders;
    for (const ModelObject *object : loaded_model.objects) {
        REQUIRE(object != nullptr);
        REQUIRE(object->volumes.size() == 1);
        const ModelVolume *volume = object->volumes.front();
        REQUIRE(volume != nullptr);
        const TriangleMesh &mesh = volume->mesh();
        REQUIRE(mesh.facets_count() == 4);
        CHECK(mesh.its.vertices.size() == 4);
        CHECK(normalized_tetra_faces(mesh) == expected_faces);
        CHECK_THAT(its_volume(mesh.its), Catch::Matchers::WithinAbs(1000.0 / 6.0, 1e-3));
        // BBS consumes the raw color string into this public assignment; Model does not retain
        // an exposed raw color-group table after import.
        REQUIRE(object->config.has("extruder"));
        actual_extruders.emplace(object->name, object->config.opt_int("extruder"));
    }
    CHECK(actual_extruders == expected_extruders);
}

} // namespace bbs_import_color_contracts

namespace bbs_remaining_public_gap_contracts {

using namespace Slic3r;
using archive_variant_contracts::BbsLoaded;
using archive_variant_contracts::load_bbs;
using archive_variant_contracts::make_cube_model;

void store_sliced_archive(const std::string &path, Model &model, DynamicPrintConfig &config,
                          PlateData &plate, SaveStrategy strategy)
{
    ScopedTemporaryDir backup("orca_bbs_remaining_gap_store");
    model.set_backup_path(backup.string());
    plate.plate_index = 0;
    plate.objects_and_instances = {{0, 0}};
    plate.config = config;

    StoreParams params;
    params.path = path;
    params.model = &model;
    params.config = &config;
    params.plate_data_list = {&plate};
    params.strategy = strategy;
    REQUIRE(store_bbs_3mf(params));
}

std::string read_slice_info(const std::string &path)
{
    std::string slice_info;
    REQUIRE(read_3mf_entry(path, slice_info, "Metadata/slice_info.config"));
    return slice_info;
}

std::map<std::string, std::string> read_model_metadata(const std::string &xml)
{
    std::stringstream input(xml);
    boost::property_tree::ptree tree;
    REQUIRE_NOTHROW(boost::property_tree::read_xml(input, tree));

    std::map<std::string, std::string> metadata;
    const auto model_node = tree.get_child_optional("model");
    REQUIRE(static_cast<bool>(model_node));
    for (const auto &node : *model_node) {
        if (node.first != "metadata")
            continue;
        const auto attributes = node.second.get_child_optional("<xmlattr>");
        REQUIRE(static_cast<bool>(attributes));
        const auto name = attributes->get_optional<std::string>("name");
        REQUIRE(name);
        metadata[*name] = node.second.data();
    }
    return metadata;
}

TEST_CASE("BBS slice metadata labels external-firmware objects with safe names",
          "[3MF][BbsRemaining][BbsGcodeObjects]")
{
    const auto [flavor, expected_name] = GENERATE(table<GCodeFlavor, std::string>({
        {gcfKlipper,        "Bracket_A_B_id_0_copy_0"},
        {gcfMarlinLegacy,   "Bracket_A_B_id_0_copy_0"},
        {gcfMarlinFirmware, "Bracket_A_B_id_0_copy_0"},
        {gcfRepRapFirmware, "Bracket_A_B_id_0_copy_0"},
    }));

    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = false;

    Model model = make_cube_model();
    model.objects.front()->name = "  Bracket [A] @ B  ";
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(flavor));

    PlateData plate;
    plate.is_sliced_valid = true;
    ScopedTemporaryFile archive(".3mf");
    store_sliced_archive(archive.string(), model, config, plate,
                         SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::WithSliceInfo);

    const std::string slice_info = read_slice_info(archive.string());
    const std::string object_name = "name=\"" + expected_name + "\"";
    CHECK(slice_info.find(object_name) != std::string::npos);
    CHECK(slice_info.find("identify_id=\"") != std::string::npos);
}

TEST_CASE("BBS slice metadata keeps raw XML-escaped names for Bambu printers",
          "[3MF][BbsRemaining][BbsGcodeObjects]")
{
    const bool previous_bbl = GCodeProcessor::s_IsBBLPrinter;
    const ScopeGuard restore_bbl([previous_bbl] { GCodeProcessor::s_IsBBLPrinter = previous_bbl; });
    GCodeProcessor::s_IsBBLPrinter = true;

    Model model = make_cube_model();
    model.objects.front()->name = "Raw & <Label>";
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_key_value("gcode_flavor", new ConfigOptionEnum<GCodeFlavor>(gcfMarlinFirmware));

    PlateData plate;
    plate.is_sliced_valid = true;
    ScopedTemporaryFile archive(".3mf");
    store_sliced_archive(archive.string(), model, config, plate,
                         SaveStrategy::Zip64 | SaveStrategy::Silence | SaveStrategy::WithSliceInfo);

    const std::string slice_info = read_slice_info(archive.string());
    CHECK(slice_info.find("name=\"Raw &amp; &lt;Label&gt;\"") != std::string::npos);
    CHECK(slice_info.find("name=\"Raw_Label_id_0_copy_0\"") == std::string::npos);
}

TEST_CASE("BBS export round-trips XML-sensitive MakerLab and custom metadata",
          "[3MF][BbsRemaining][BbsMakerLabMetadata]")
{
    Model source = make_cube_model();
    const std::string maker_name = "Maker & <Lab> \"quoted\" 'single' &amp; &quot; &apos; &unknown; literal";
    const std::string maker_version = "maker-7 & <rev> \"quoted\" 'single' &amp; literal";
    const std::string workflow_value = "prototype & <inside> \"quoted\" 'single' &amp; literal";
    const std::string revision_value = "rev <7> &amp;";
    source.mk_name = maker_name;
    source.mk_version = maker_version;
    source.md_name = {"MakerWorkflow", "MakerRevision"};
    source.md_value = {workflow_value, revision_value};

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    PlateData plate;
    ScopedTemporaryFile archive(".3mf");
    store_sliced_archive(archive.string(), source, config, plate,
                         SaveStrategy::Zip64 | SaveStrategy::Silence);

    std::string model_xml;
    REQUIRE(read_3mf_entry(archive.string(), model_xml, "3D/3dmodel.model"));
    // Parsing the complete model XML is the serialization contract; this deliberately avoids
    // asserting indentation or element order while still rejecting malformed entity escaping.
    const auto serialized_metadata = read_model_metadata(model_xml);
    REQUIRE(serialized_metadata.find("MakerLab") != serialized_metadata.end());
    REQUIRE(serialized_metadata.find("MakerLabVersion") != serialized_metadata.end());
    REQUIRE(serialized_metadata.find("MakerWorkflow") != serialized_metadata.end());
    REQUIRE(serialized_metadata.find("MakerRevision") != serialized_metadata.end());

    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(load_bbs(archive.string(), loaded_model, loaded));
    REQUIRE(loaded_model.model_info != nullptr);
    const auto &loaded_metadata = loaded_model.model_info->metadata_items;
    const std::array<std::pair<const char *, std::string>, 4> expected_metadata = {{
        {"MakerLab", maker_name},
        {"MakerLabVersion", maker_version},
        {"MakerWorkflow", workflow_value},
        {"MakerRevision", revision_value},
    }};
    for (const auto &[key, expected] : expected_metadata) {
        const auto found = loaded_metadata.find(key);
        REQUIRE(found != loaded_metadata.end());
        CHECK(found->second == expected);
    }
}

} // namespace bbs_remaining_public_gap_contracts

namespace generic_3mf_public_gap_contracts {
using namespace Slic3r;

namespace {

Model make_three_object_model()
{
    Model model;
    for (size_t i = 0; i < 3; ++i) {
        ModelObject *object = model.add_object();
        object->name = "generic-metadata-object-" + std::to_string(i + 1);
        object->add_volume(make_cube(10. + double(i), 10., 2.));
        object->add_instance();
    }
    return model;
}

std::pair<int, std::vector<double>> parse_object_profile(const std::string &payload)
{
    std::istringstream lines(payload);
    std::string line;
    REQUIRE(std::getline(lines, line));
    const size_t separator = line.find('|');
    REQUIRE(separator != std::string::npos);
    REQUIRE(line.rfind("object_id=", 0) == 0);
    const int object_id = std::stoi(line.substr(10, separator - 10));

    std::vector<double> profile;
    std::istringstream values(line.substr(separator + 1));
    std::string value;
    while (std::getline(values, value, ';'))
        profile.push_back(std::stod(value));
    return {object_id, profile};
}

struct ParsedSupportPoint {
    Vec3f position;
    float radius;
    bool new_island;
};

std::pair<int, std::vector<ParsedSupportPoint>> parse_support_points(const std::string &payload)
{
    std::istringstream lines(payload);
    std::string header;
    REQUIRE(std::getline(lines, header));
    REQUIRE(header == "support_points_format_version=1");
    std::string line;
    REQUIRE(std::getline(lines, line));
    const size_t separator = line.find('|');
    REQUIRE(separator != std::string::npos);
    REQUIRE(line.rfind("object_id=", 0) == 0);
    const int object_id = std::stoi(line.substr(10, separator - 10));

    std::istringstream values(line.substr(separator + 1));
    std::vector<float> numbers;
    float value = 0.f;
    while (values >> value)
        numbers.push_back(value);
    REQUIRE(numbers.size() % 5 == 0);

    std::vector<ParsedSupportPoint> points;
    for (size_t i = 0; i < numbers.size(); i += 5)
        points.push_back({Vec3f(numbers[i], numbers[i + 1], numbers[i + 2]), numbers[i + 3], numbers[i + 4] != 0.f});
    return {object_id, points};
}

struct ParsedDrainHole {
    Vec3f position;
    Vec3f normal;
    float radius;
    float height;
};

std::pair<int, std::vector<ParsedDrainHole>> parse_drain_holes(const std::string &payload)
{
    std::istringstream lines(payload);
    std::string header;
    REQUIRE(std::getline(lines, header));
    REQUIRE(header == "drain_holes_format_version=1");
    std::string line;
    REQUIRE(std::getline(lines, line));
    const size_t separator = line.find('|');
    REQUIRE(separator != std::string::npos);
    REQUIRE(line.rfind("object_id=", 0) == 0);
    const int object_id = std::stoi(line.substr(10, separator - 10));

    std::istringstream values(line.substr(separator + 1));
    std::vector<float> numbers;
    float value = 0.f;
    while (values >> value)
        numbers.push_back(value);
    REQUIRE(numbers.size() % 8 == 0);

    std::vector<ParsedDrainHole> holes;
    for (size_t i = 0; i < numbers.size(); i += 8)
        holes.push_back({Vec3f(numbers[i], numbers[i + 1], numbers[i + 2]),
                         Vec3f(numbers[i + 3], numbers[i + 4], numbers[i + 5]),
                         numbers[i + 6], numbers[i + 7]});
    return {object_id, holes};
}

} // namespace

TEST_CASE("Generic 3MF metadata uses one-based model object indexing", "[3MF][Generic3MFMetadata]")
{
    Model source = make_three_object_model();
    ModelObject *middle = source.objects[1];
    const std::vector<double> expected_profile = {0., 0.12, 0.8, 0.2, 1.6, 0.28};
    middle->layer_height_profile.set(expected_profile);
    DynamicPrintConfig range_config;
    range_config.set_key_value("layer_height", new ConfigOptionFloat(0.31));
    middle->layer_config_ranges[{0.4, 1.2}].assign_config(std::move(range_config));

    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));

    std::string profile_payload;
    REQUIRE(read_3mf_entry(archive.string(), profile_payload, "Metadata/Slic3r_PE_layer_heights_profile.txt"));
    const auto [profile_object_id, serialized_profile] = parse_object_profile(profile_payload);
    REQUIRE(profile_object_id == 2);
    REQUIRE(serialized_profile.size() == expected_profile.size());
    for (size_t i = 0; i < expected_profile.size(); ++i)
        CHECK_THAT(serialized_profile[i], Catch::Matchers::WithinAbs(expected_profile[i], 1e-6));

    std::string ranges_payload;
    REQUIRE(read_3mf_entry(archive.string(), ranges_payload, "Metadata/Prusa_Slicer_layer_config_ranges.xml"));
    boost::property_tree::ptree ranges_tree;
    std::istringstream ranges_input(ranges_payload);
    REQUIRE_NOTHROW(boost::property_tree::read_xml(ranges_input, ranges_tree));
    const auto &objects = ranges_tree.get_child("objects");
    const boost::property_tree::ptree *object_node = nullptr;
    size_t object_count = 0;
    for (const auto &child : objects) {
        if (child.first == "object") {
            ++object_count;
            object_node = &child.second;
        }
    }
    REQUIRE(object_count == 1);
    REQUIRE(object_node != nullptr);
    REQUIRE(object_node->get<int>("<xmlattr>.id") == 2);
    const auto &range_node = object_node->get_child("range");
    CHECK_THAT(range_node.get<double>("<xmlattr>.min_z"), Catch::Matchers::WithinAbs(0.4, 1e-6));
    CHECK_THAT(range_node.get<double>("<xmlattr>.max_z"), Catch::Matchers::WithinAbs(1.2, 1e-6));
    REQUIRE(static_cast<bool>(range_node.get_child_optional("option")));
    CHECK(range_node.get_child("option").get<std::string>("<xmlattr>.opt_key") == "layer_height");
    const std::string serialized_layer_height = range_node.get_child("option").data();
    REQUIRE_FALSE(serialized_layer_height.empty());
    CHECK_THAT(std::stod(serialized_layer_height), Catch::Matchers::WithinAbs(0.31, 1e-6));

    // Export must not rewrite the source metadata while applying its persisted format.
    REQUIRE(middle->layer_height_profile.get().size() == expected_profile.size());
    for (size_t i = 0; i < expected_profile.size(); ++i)
        CHECK_THAT(middle->layer_height_profile.get()[i], Catch::Matchers::WithinAbs(expected_profile[i], 1e-6));
    REQUIRE(middle->layer_config_ranges.size() == 1);
    const auto source_range = middle->layer_config_ranges.begin();
    CHECK_THAT(source_range->first.first, Catch::Matchers::WithinAbs(0.4, 1e-6));
    CHECK_THAT(source_range->first.second, Catch::Matchers::WithinAbs(1.2, 1e-6));
    REQUIRE(source_range->second.has("layer_height"));
    CHECK_THAT(source_range->second.opt_float("layer_height"), Catch::Matchers::WithinAbs(0.31, 1e-6));
}

TEST_CASE("Generic 3MF preserves SLA support points and island flags", "[3MF][Generic3MFMetadata]")
{
    Model source = make_three_object_model();
    ModelObject *middle = source.objects[1];
    const std::vector<sla::SupportPoint> expected_support_points = {
        sla::SupportPoint(1.25f, 2.5f, 3.75f, 0.42f, false),
        sla::SupportPoint(-4.5f, 5.25f, 6.75f, 0.83f, true),
    };
    middle->sla_support_points = expected_support_points;

    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));
    REQUIRE(middle->sla_support_points.size() == expected_support_points.size());
    for (size_t i = 0; i < expected_support_points.size(); ++i) {
        const auto &expected = expected_support_points[i];
        const auto &actual = middle->sla_support_points[i];
        CHECK_THAT(actual.pos.x(), Catch::Matchers::WithinAbs(expected.pos.x(), 1e-5));
        CHECK_THAT(actual.pos.y(), Catch::Matchers::WithinAbs(expected.pos.y(), 1e-5));
        CHECK_THAT(actual.pos.z(), Catch::Matchers::WithinAbs(expected.pos.z(), 1e-5));
        CHECK_THAT(actual.head_front_radius, Catch::Matchers::WithinAbs(expected.head_front_radius, 1e-5));
        CHECK(actual.is_new_island == expected.is_new_island);
    }
    std::string payload;
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/Slic3r_PE_sla_support_points.txt"));
    const auto [object_id, serialized_points] = parse_support_points(payload);
    REQUIRE(object_id == 2);
    REQUIRE(serialized_points.size() == expected_support_points.size());
    for (size_t i = 0; i < serialized_points.size(); ++i) {
        const auto &expected = expected_support_points[i];
        CHECK_THAT(serialized_points[i].position.x(), Catch::Matchers::WithinAbs(expected.pos.x(), 1e-5));
        CHECK_THAT(serialized_points[i].position.y(), Catch::Matchers::WithinAbs(expected.pos.y(), 1e-5));
        CHECK_THAT(serialized_points[i].position.z(), Catch::Matchers::WithinAbs(expected.pos.z(), 1e-5));
        CHECK_THAT(serialized_points[i].radius, Catch::Matchers::WithinAbs(expected.head_front_radius, 1e-5));
        CHECK(serialized_points[i].new_island == expected.is_new_island);
    }

}

TEST_CASE("Generic 3MF serializes SLA drain holes with compatibility offset", "[3MF][Generic3MFMetadata]")
{
    Model source = make_three_object_model();
    ModelObject *middle = source.objects[1];
    const Vec3f source_normal(0.f, 3.f, 4.f);
    const std::vector<sla::DrainHole> expected_drain_holes = {
        sla::DrainHole(Vec3f(10.f, 20.f, 30.f), source_normal, 1.25f, 7.5f),
        sla::DrainHole(Vec3f(-4.f, 8.f, 12.f), Vec3f(2.f, -1.f, 2.f), 2.75f, 3.25f),
    };
    middle->sla_drain_holes = expected_drain_holes;

    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));
    REQUIRE(middle->sla_drain_holes.size() == expected_drain_holes.size());
    for (size_t i = 0; i < expected_drain_holes.size(); ++i) {
        const auto &expected = expected_drain_holes[i];
        const auto &actual = middle->sla_drain_holes[i];
        CHECK_THAT(actual.pos.x(), Catch::Matchers::WithinAbs(expected.pos.x(), 1e-5));
        CHECK_THAT(actual.pos.y(), Catch::Matchers::WithinAbs(expected.pos.y(), 1e-5));
        CHECK_THAT(actual.pos.z(), Catch::Matchers::WithinAbs(expected.pos.z(), 1e-5));
        CHECK_THAT(actual.normal.x(), Catch::Matchers::WithinAbs(expected.normal.x(), 1e-5));
        CHECK_THAT(actual.normal.y(), Catch::Matchers::WithinAbs(expected.normal.y(), 1e-5));
        CHECK_THAT(actual.normal.z(), Catch::Matchers::WithinAbs(expected.normal.z(), 1e-5));
        CHECK_THAT(actual.radius, Catch::Matchers::WithinAbs(expected.radius, 1e-5));
        CHECK_THAT(actual.height, Catch::Matchers::WithinAbs(expected.height, 1e-5));
    }
    std::string payload;
    REQUIRE(read_3mf_entry(archive.string(), payload, "Metadata/Slic3r_PE_sla_drain_holes.txt"));
    const auto [object_id, serialized_holes] = parse_drain_holes(payload);
    REQUIRE(object_id == 2);
    REQUIRE(serialized_holes.size() == expected_drain_holes.size());
    for (size_t i = 0; i < serialized_holes.size(); ++i) {
        const auto &expected = expected_drain_holes[i];
        const Vec3f stored_position = expected.pos - expected.normal.normalized();
        CHECK_THAT(serialized_holes[i].position.x(), Catch::Matchers::WithinAbs(stored_position.x(), 1e-5));
        CHECK_THAT(serialized_holes[i].position.y(), Catch::Matchers::WithinAbs(stored_position.y(), 1e-5));
        CHECK_THAT(serialized_holes[i].position.z(), Catch::Matchers::WithinAbs(stored_position.z(), 1e-5));
        CHECK_THAT(serialized_holes[i].normal.x(), Catch::Matchers::WithinAbs(expected.normal.x(), 1e-5));
        CHECK_THAT(serialized_holes[i].normal.y(), Catch::Matchers::WithinAbs(expected.normal.y(), 1e-5));
        CHECK_THAT(serialized_holes[i].normal.z(), Catch::Matchers::WithinAbs(expected.normal.z(), 1e-5));
        CHECK_THAT(serialized_holes[i].radius, Catch::Matchers::WithinAbs(expected.radius, 1e-5));
        CHECK_THAT(serialized_holes[i].height, Catch::Matchers::WithinAbs(expected.height + 1.f, 1e-5));
    }

}

TEST_CASE("Generic 3MF omits empty optional metadata entries", "[3MF][Generic3MFMetadata]")
{
    Model source = make_three_object_model();
    ScopedTemporaryFile archive(".3mf");
    REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));
    REQUIRE(source.objects.size() == 3);
    for (const ModelObject *object : source.objects) {
        REQUIRE(object != nullptr);
        CHECK(object->layer_height_profile.get().empty());
        CHECK(object->layer_config_ranges.empty());
        CHECK(object->sla_support_points.empty());
        CHECK(object->sla_drain_holes.empty());
    }

    std::string payload;
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/Slic3r_PE_layer_heights_profile.txt"));
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/Prusa_Slicer_layer_config_ranges.xml"));
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/Slic3r_PE_sla_support_points.txt"));
    CHECK_FALSE(read_3mf_entry(archive.string(), payload, "Metadata/Slic3r_PE_sla_drain_holes.txt"));
}

TEST_CASE("Generic 3MF writes height profiles only when they contain complete pairs",
          "[3MF][Generic3MFMetadata]")
{
    struct ProfileRow {
        size_t values;
        bool stored;
    };
    const std::vector<double> full_profile = {0., 0.12, 0.8, 0.2, 1.6, 0.28};
    const std::array<ProfileRow, 6> rows = {{{0, false}, {2, false}, {3, false},
                                          {4, true}, {5, false}, {6, true}}};
    for (const ProfileRow row : rows) {
        INFO("profile value count " << row.values);
        Model source = make_three_object_model();
        std::vector<double> expected_profile(full_profile.begin(), full_profile.begin() + row.values);
        source.objects[1]->layer_height_profile.set(expected_profile);
        ScopedTemporaryFile archive(".3mf");
        REQUIRE(store_3mf(archive.string().c_str(), &source, nullptr, false));

        std::string payload;
        const bool stored = read_3mf_entry(archive.string(), payload,
                                          "Metadata/Slic3r_PE_layer_heights_profile.txt");
        REQUIRE(stored == row.stored);
        if (stored) {
            const auto [object_id, profile] = parse_object_profile(payload);
            REQUIRE(object_id == 2);
            REQUIRE(profile.size() == expected_profile.size());
            for (size_t i = 0; i < profile.size(); ++i)
                CHECK_THAT(profile[i], Catch::Matchers::WithinAbs(expected_profile[i], 1e-6));
        }
        const auto &source_profile = source.objects[1]->layer_height_profile.get();
        REQUIRE(source_profile.size() == expected_profile.size());
        for (size_t i = 0; i < source_profile.size(); ++i)
            CHECK_THAT(source_profile[i], Catch::Matchers::WithinAbs(expected_profile[i], 1e-6));
    }
}

} // namespace generic_3mf_public_gap_contracts


namespace bbs_backup_contracts {

using namespace Slic3r;
using boost::property_tree::ptree;

namespace {

struct BackupCounts
{
    std::size_t worker_callbacks = 0;
    std::size_t ui_backup_callbacks = 0;
};

// The callback is copied by the worker before it is invoked.  Owning this state with a
// shared_ptr keeps a timed-out callback safe until that copied std::function is destroyed.
struct BackupEvents
{
    std::mutex mutex;
    std::condition_variable condition;
    std::size_t worker_callbacks = 0;
    std::size_t ui_backup_callbacks = 0;

    void notify(int type)
    {
        std::lock_guard<std::mutex> lock(mutex);
        if (type == 0)
            ++worker_callbacks;
        else if (type == 1)
            ++ui_backup_callbacks;
        condition.notify_all();
    }

    BackupCounts snapshot()
    {
        std::lock_guard<std::mutex> lock(mutex);
        return { worker_callbacks, ui_backup_callbacks };
    }
};

// A Backup task is a public queue barrier: it is appended after all earlier work, and its
// type-1 callback is emitted only after the UI task pump has processed that prefix.  The
// caller remains on the main thread while pumping, so no assertion runs from the worker.
bool flush_backup_barrier(const std::shared_ptr<BackupEvents> &events,
                          std::chrono::milliseconds timeout = std::chrono::seconds(5))
{
    const BackupCounts before = events->snapshot();
    const std::size_t target_ui_callbacks = before.ui_backup_callbacks + 1;
    backup_soon();
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        // A notification during or just after the pump must trigger another pump.
        const BackupCounts before_pump = events->snapshot();
        run_backup_ui_tasks();
        if (events->snapshot().ui_backup_callbacks >= target_ui_callbacks)
            return true;

        // Worker callback type 0 wakes this wait so the next iteration can pump the
        // corresponding UI task.  Type 1 is the completion predicate above.
        std::unique_lock<std::mutex> lock(events->mutex);
        events->condition.wait_until(lock, deadline, [&] {
            return events->ui_backup_callbacks >= target_ui_callbacks ||
                   events->worker_callbacks > before_pump.worker_callbacks;
        });
    }
    run_backup_ui_tasks();
    return events->snapshot().ui_backup_callbacks >= target_ui_callbacks;
}

struct BackupCleanup
{
    Model &model;
    std::shared_ptr<BackupEvents> events;

    ~BackupCleanup()
    {
        // RemoveBackup precedes the barrier. Cleanup is bounded best effort after a test
        // timeout; any worker-held callback still owns its state, and paths are unique.
        remove_backup(model, true);
        (void)flush_backup_barrier(events);
        set_backup_callback({});
        set_backup_interval(60);
        model.set_backup_path("detach");
    }
};

bool parse_xml_double(const ptree::value_type &node, const char *attribute, double &value)
{
    const auto raw = node.second.get_optional<std::string>(std::string("<xmlattr>.") + attribute);
    if (!raw)
        return false;
    char *end = nullptr;
    value = std::strtod(raw->c_str(), &end);
    return end != raw->c_str() && *end == '\0' && std::isfinite(value);
}

bool parse_xml_index(const ptree::value_type &node, const char *attribute, long &value)
{
    const auto raw = node.second.get_optional<std::string>(std::string("<xmlattr>.") + attribute);
    if (!raw)
        return false;
    char *end = nullptr;
    value = std::strtol(raw->c_str(), &end, 10);
    return end != raw->c_str() && *end == '\0';
}

} // namespace

TEST_CASE("BBS backup guards require both mesh and instance state", "[3MF][Bbs3mfBackup][Guard]")
{
    const std::string shape = GENERATE("backup disabled", "missing volume", "missing instance");
    CAPTURE(shape);

    // Declare the directory before Model so cleanup detaches the model before the directory dies.
    ScopedTemporaryDir backup("orca_bbs_backup_guard_v2");
    Model model;
    model.set_backup_path(backup.string());
    boost::filesystem::create_directories(backup.path() / "3D" / "Objects");

    ModelObject *object = model.add_object();
    REQUIRE(object != nullptr);
    object->name = "guarded-backup";
    object->input_file = "guarded.stl";
    if (shape != "missing volume")
        object->add_volume(make_cube(4., 5., 6.));
    if (shape != "missing instance")
        object->add_instance();
    if (shape != "backup disabled")
        model.set_need_backup();

    const auto events = std::make_shared<BackupEvents>();
    set_backup_interval(0);
    set_backup_callback([events](int type) { events->notify(type); });
    BackupCleanup cleanup{ model, events };

    REQUIRE(flush_backup_barrier(events));
    const int backup_id = model.get_object_backup_id(*object);
    const boost::filesystem::path expected = backup.path() / "3D" / "Objects" /
        (object->name + "_" + std::to_string(backup_id) + ".model");

    const BackupCounts before = events->snapshot();
    save_object_mesh(*object);
    REQUIRE(flush_backup_barrier(events));
    const BackupCounts after = events->snapshot();
    CHECK(after.ui_backup_callbacks == before.ui_backup_callbacks + 1);
    CHECK_FALSE(boost::filesystem::exists(expected));
}

TEST_CASE("BBS backup writes valid mesh geometry and completes its public lifecycle",
          "[3MF][Bbs3mfBackup][Snapshot]")
{
    ScopedTemporaryDir backup("orca_bbs_backup_snapshot_v2");
    Model model;
    model.set_backup_path(backup.string());
    boost::filesystem::create_directories(backup.path() / "3D" / "Objects");

    ModelObject *object = model.add_object("snapshot-part", "snapshot.stl", make_cube(10., 11., 12.));
    REQUIRE(object != nullptr);
    object->add_instance();
    model.set_need_backup();
    const int backup_id = model.get_object_backup_id(*object);
    const std::string entry = "3D/Objects/" + object->name + "_" + std::to_string(backup_id) + ".model";
    const boost::filesystem::path snapshot = backup.path() / entry;

    const auto events = std::make_shared<BackupEvents>();
    set_backup_interval(0);
    set_backup_callback([events](int type) { events->notify(type); });
    BackupCleanup cleanup{ model, events };

    REQUIRE(flush_backup_barrier(events));
    const BackupCounts before = events->snapshot();
    save_object_mesh(*object);
    REQUIRE(flush_backup_barrier(events));
    const BackupCounts after = events->snapshot();
    CHECK(after.ui_backup_callbacks == before.ui_backup_callbacks + 1);

    REQUIRE(boost::filesystem::exists(snapshot));
    std::string model_xml;
    REQUIRE(read_3mf_entry(snapshot.string(), model_xml, entry));
    ptree tree;
    std::stringstream model_stream(model_xml);
    REQUIRE_NOTHROW(boost::property_tree::read_xml(model_stream, tree));

    // A model document can contain component objects before the mesh-bearing resource;
    // select the actual object with a mesh rather than depending on resource order.
    const auto resources = tree.get_child_optional("model.resources");
    REQUIRE(static_cast<bool>(resources));
    const ptree *mesh_object = nullptr;
    const ptree *mesh = nullptr;
    for (const auto &node : *resources) {
        if (node.first != "object")
            continue;
        const auto candidate = node.second.get_child_optional("mesh");
        if (candidate) {
            mesh_object = &node.second;
            mesh = &candidate.get();
            break;
        }
    }
    REQUIRE(mesh_object != nullptr);
    REQUIRE(mesh != nullptr);
    const auto object_id = mesh_object->get_optional<unsigned>("<xmlattr>.id");
    REQUIRE(object_id);
    // 3MF resources have positive IDs; the source-derived filename above separately
    // verifies the backup identifier without depending on its packed internal encoding.
    CHECK(*object_id > 0u);

    const auto vertices = mesh->get_child_optional("vertices");
    const auto triangles = mesh->get_child_optional("triangles");
    REQUIRE(static_cast<bool>(vertices));
    REQUIRE(static_cast<bool>(triangles));

    std::size_t vertex_count = 0;
    bool vertex_values_ok = true;
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double min_z = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    for (const auto &node : *vertices) {
        if (node.first != "vertex")
            continue;
        ++vertex_count;
        double x = 0., y = 0., z = 0.;
        if (!parse_xml_double(node, "x", x) || !parse_xml_double(node, "y", y) || !parse_xml_double(node, "z", z)) {
            vertex_values_ok = false;
            continue;
        }
        min_x = std::min(min_x, x); min_y = std::min(min_y, y); min_z = std::min(min_z, z);
        max_x = std::max(max_x, x); max_y = std::max(max_y, y); max_z = std::max(max_z, z);
    }
    CHECK(vertex_count == 8);
    CHECK(vertex_values_ok);
    // ModelObject::add_volume defaults to center_geometry_after_creation(), so
    // the backup mesh stores the 10x11x12 source centered at the origin.
    CHECK_THAT(min_x, Catch::Matchers::WithinAbs(-5., 1e-6));
    CHECK_THAT(min_y, Catch::Matchers::WithinAbs(-5.5, 1e-6));
    CHECK_THAT(min_z, Catch::Matchers::WithinAbs(-6., 1e-6));
    CHECK_THAT(max_x, Catch::Matchers::WithinAbs(5., 1e-6));
    CHECK_THAT(max_y, Catch::Matchers::WithinAbs(5.5, 1e-6));
    CHECK_THAT(max_z, Catch::Matchers::WithinAbs(6., 1e-6));

    bool triangle_indices_ok = true;
    std::size_t triangle_count = 0;
    for (const auto &node : *triangles) {
        if (node.first != "triangle")
            continue;
        ++triangle_count;
        for (const char *attribute : { "v1", "v2", "v3" }) {
            long index = -1;
            if (!parse_xml_index(node, attribute, index) || index < 0 || index >= static_cast<long>(vertex_count))
                triangle_indices_ok = false;
        }
    }
    CHECK(triangle_count == 12);
    CHECK(triangle_indices_ok);

    put_other_changes();
    CHECK(has_other_changes(false));
    CHECK(has_other_changes(true));
    clear_other_changes(false);
    CHECK_FALSE(has_other_changes(false));
    CHECK(has_other_changes(true));

    const BackupCounts before_backup = events->snapshot();
    REQUIRE(flush_backup_barrier(events));
    const BackupCounts after_backup = events->snapshot();
    CHECK(after_backup.ui_backup_callbacks == before_backup.ui_backup_callbacks + 1);
    CHECK_FALSE(has_other_changes(true));

    remove_backup(model, true);
    const BackupCounts before_remove_barrier = events->snapshot();
    REQUIRE(flush_backup_barrier(events));
    const BackupCounts after_remove_barrier = events->snapshot();
    CHECK(after_remove_barrier.ui_backup_callbacks == before_remove_barrier.ui_backup_callbacks + 1);
    CHECK_FALSE(boost::filesystem::exists(backup.path()));
}

} // namespace bbs_backup_contracts

namespace bbs_text_alignment_compat_contracts {

using namespace Slic3r;
using BbsLoaded = bbs_3mf_metadata_contracts::BbsLoaded;

static void set_text_configuration(Model &model, DynamicPrintConfig &, PlateData &, StoreParams &, ScopedTemporaryDir &)
{
    REQUIRE(model.objects.size() == 1);
    REQUIRE(model.objects.front()->volumes.size() == 1);
    TextConfiguration text;
    text.text = "legacy alignment";
    text.style.name = "compatibility style";
    text.style.path = "font.ttf";
    text.style.type = EmbossStyle::Type::file_path;
    text.style.prop.size_in_mm = 8.f;
    text.style.prop.align = {FontProp::HorizontalAlign::right, FontProp::VerticalAlign::bottom};
    model.objects.front()->volumes.front()->text_configuration = std::move(text);
}

static FontProp::Align load_alignment(const std::string &archive)
{
    Model loaded_model;
    BbsLoaded loaded;
    REQUIRE(bbs_3mf_metadata_contracts::load_followup(archive, loaded_model, loaded));
    REQUIRE(loaded_model.objects.size() == 1);
    REQUIRE(loaded_model.objects.front()->volumes.size() == 1);
    const auto &text = loaded_model.objects.front()->volumes.front()->text_configuration;
    REQUIRE(text.has_value());
    return text->style.prop.align;
}

TEST_CASE("BBS text metadata maps legacy numeric alignment indexes", "[3MF][Bbs3mfTextAlignment]")
{
    const int index = GENERATE(0, 1, 2);
    CAPTURE(index);
    const FontProp::HorizontalAlign horizontal[] = {
        FontProp::HorizontalAlign::left, FontProp::HorizontalAlign::center, FontProp::HorizontalAlign::right
    };
    const FontProp::VerticalAlign vertical[] = {
        FontProp::VerticalAlign::top, FontProp::VerticalAlign::center, FontProp::VerticalAlign::bottom
    };
    ScopedTemporaryFile archive(".3mf");
    bbs_3mf_metadata_contracts::store_followup(archive.string(), set_text_configuration);
    REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                 "horizontal=\"right\"", "horizontal=\"" + std::to_string(index) + "\""));
    REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                 "vertical=\"bottom\"", "vertical=\"" + std::to_string(index) + "\""));
    const auto align = load_alignment(archive.string());
    CHECK(align.first == horizontal[index]);
    CHECK(align.second == vertical[index]);
}

TEST_CASE("BBS text metadata defaults missing or empty alignment to center", "[3MF][Bbs3mfTextAlignment]")
{
    const bool remove_attributes = GENERATE(false, true);
    CAPTURE(remove_attributes);
    ScopedTemporaryFile archive(".3mf");
    bbs_3mf_metadata_contracts::store_followup(archive.string(), set_text_configuration);
    if (remove_attributes) {
        REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                     "horizontal=\"right\"", ""));
        REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                     "vertical=\"bottom\"", ""));
    } else {
        REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                     "horizontal=\"right\"", "horizontal=\"\""));
        REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                     "vertical=\"bottom\"", "vertical=\"\""));
    }
    const auto align = load_alignment(archive.string());
    CHECK(align.first == FontProp::HorizontalAlign::center);
    CHECK(align.second == FontProp::VerticalAlign::center);
}

TEST_CASE("BBS text metadata defaults unknown alignment names to center", "[3MF][Bbs3mfTextAlignment]")
{
    ScopedTemporaryFile archive(".3mf");
    bbs_3mf_metadata_contracts::store_followup(archive.string(), set_text_configuration);
    REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                 "horizontal=\"right\"", "horizontal=\"diagonal\""));
    REQUIRE(replace_in_3mf_entry(archive.string(), "model_settings.config",
                                 "vertical=\"bottom\"", "vertical=\"upright\""));
    const auto align = load_alignment(archive.string());
    CHECK(align.first == FontProp::HorizontalAlign::center);
    CHECK(align.second == FontProp::VerticalAlign::center);
}

} // namespace bbs_text_alignment_compat_contracts
