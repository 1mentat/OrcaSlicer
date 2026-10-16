#include "libslic3r/ProjectTask.hpp"

#include <catch2/catch_all.hpp>
#include <catch2/catch_test_macros.hpp>
#include <string>

namespace bbs_project_lifecycle_contracts
{
using namespace Slic3r;

struct TaskSubtaskOwner
{
    BBLTask &task;

    ~TaskSubtaskOwner()
    {
        for (BBLSubTask *subtask : task.subtasks)
            delete subtask;
        task.subtasks.clear();
    }
};

TEST_CASE("Project task lifecycle preserves parent links and valid service metadata",
          "[ProjectTask][Bbs3mfLifecycle]")
{
    BBLProject project("Cloud project");
    project.project_id = "project-1";
    BBLProfile profile(&project);
    profile.profile_id = "profile-1";
    BBLTask task(&profile);
    task.task_id = "task-1";
    TaskSubtaskOwner task_subtasks{task};

    CHECK(profile.profile_name == "N/A");
    CHECK(task.task_profile_id == "profile-1");
    CHECK(task.task_project_id == "project-1");
    BBLSubTask info(&task);
    CHECK(info.parent_id == "task-1");
    CHECK(info.task_bed_type == "auto");
    CHECK(info.parse_content_json(R"({"info":{"name":"plate-one","plate_idx":2,"printer":"A1"}})") == 0);
    CHECK(info.task_name == "plate-one");
    CHECK(info.task_partplate_idx == "2");
    CHECK(info.task_printer_dev_id == "A1");

    BBLSliceInfo slice(&profile);
    slice.index = "2";
    profile.slice_info.emplace(slice.index, &slice);
    CHECK(profile.get_slice_info("2") == &slice);
    CHECK(profile.get_slice_info("missing") == nullptr);

    CHECK(task.parse_content_json(R"({"subtasks":[
        {"id":"sub-1","name":"plate-one","create_time":"t1","plate_idx":"2","printer":"A1","weight":"1.5"},
        {"id":"sub-2","name":"plate-two","create_time":"t2","plate_idx":"3","printer":"A1","weight":"2.5"}
    ]})") == 0);
    REQUIRE(task.subtasks.size() == 2);
    CHECK(task.subtasks[0]->parent_id == "task-1");
    CHECK(task.subtasks[0]->task_partplate_idx == "2");
    CHECK(task.subtasks[0]->task_weight == "1.5");
    CHECK(task.subtasks[1]->task_id == "sub-2");
    CHECK(task.subtasks[1]->task_partplate_idx == "3");

    CHECK(BBLSubTask::parse_status("CREATED") == BBLSubTask::TASK_CREATED);
    CHECK(BBLSubTask::parse_status("READY") == BBLSubTask::TASK_READY);
    CHECK(BBLSubTask::parse_status("RUNNING") == BBLSubTask::TASK_RUNNING);
    CHECK(BBLSubTask::parse_status("PAUSE") == BBLSubTask::TASK_PAUSE);
    CHECK(BBLSubTask::parse_status("FAILED") == BBLSubTask::TASK_FAILED);
    CHECK(BBLSubTask::parse_status("FINISHED") == BBLSubTask::TASK_FINISHED);
    CHECK(BBLSubTask::parse_user_service_task_status(1) == BBLSubTask::TASK_RUNNING);
    CHECK(BBLSubTask::parse_user_service_task_status(2) == BBLSubTask::TASK_FINISHED);
    CHECK(BBLSubTask::parse_user_service_task_status(3) == BBLSubTask::TASK_FAILED);
    CHECK(BBLSubTask::parse_user_service_task_status(99) == BBLSubTask::TASK_UNKNOWN);

    FilamentInfo filament;
    filament.ams_id = "3";
    filament.slot_id = "7";
    filament.type = "PLA-S";
    CHECK(filament.get_ams_id() == 3);
    CHECK(filament.get_slot_id() == 7);
    CHECK(filament.get_display_filament_type() == "Sup.PLA");
    filament.type = "PETG";
    CHECK(filament.get_display_filament_type() == "PETG");

    project.project_name = "will-reset";
    project.project_url = "https://example.invalid/project";
    project.reset();
    CHECK(project.project_name.empty());
    CHECK(project.project_id.empty());
    CHECK(project.project_url.empty());

}

TEST_CASE("Project task null-parent constructors preserve public defaults",
          "[ProjectTask][Bbs3mfLifecycle]")
{
    BBLProfile profile(nullptr);
    CHECK(profile.project_ == nullptr);
    CHECK(profile.project_id.empty());
    CHECK(profile.profile_name == "N/A");

    BBLTask task(nullptr);
    CHECK(task.profile_ == nullptr);
    CHECK(task.task_project_id.empty());
    CHECK(task.task_profile_id.empty());

    BBLSubTask subtask(nullptr);
    CHECK(subtask.parent_task_ == nullptr);
    CHECK(subtask.parent_id.empty());
    CHECK(subtask.task_bed_type == "auto");

    BBLModelTask model_task;
    CHECK(model_task.job_id == -1);
    CHECK(model_task.design_id == -1);
    CHECK(model_task.profile_id == -1);
    CHECK(model_task.task_id.empty());
    CHECK(model_task.model_id.empty());
    CHECK(model_task.model_name.empty());
    CHECK(model_task.profile_name.empty());
    // instance_id is not initialized by the current constructor and is
    // intentionally outside this contract until production semantics change.
}

TEST_CASE("Project subtask parser preserves string plate indexes and reports missing info",
          "[ProjectTask][Bbs3mfLifecycle]")
{
    BBLSubTask subtask;
    CHECK(subtask.parse_content_json(
              R"({"info":{"name":"string-plate","plate_idx":"7","printer":"A1"}})") == 0);
    CHECK(subtask.task_name == "string-plate");
    CHECK(subtask.task_partplate_idx == "7");
    CHECK(subtask.task_printer_dev_id == "A1");
    CHECK(subtask.parse_content_json(R"({"metadata":{"name":"no-info"}})") == -1);
}

TEST_CASE("Project subtask unknown textual status defaults to created",
          "[ProjectTask][Bbs3mfLifecycle]")
{
    CHECK(BBLSubTask::parse_status("UNRECOGNIZED") == BBLSubTask::TASK_CREATED);
}

} // namespace bbs_project_lifecycle_contracts
