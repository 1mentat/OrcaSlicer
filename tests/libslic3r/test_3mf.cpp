#include <array>
#include "libslic3r/GCode/GCodeProcessor.hpp"

#include "libslic3r/GCode/ThumbnailData.hpp"
#include "libslic3r/PNGReadWrite.hpp"
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <set>

#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/catch_message.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "libslic3r/Model.hpp"
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
