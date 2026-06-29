#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <moveit/task_constructor/task.h>

#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/move_to.h>

#include <moveit/task_constructor/solvers/pipeline_planner.h>

using namespace moveit::task_constructor;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>("mtc_node");

    Task task;

    task.stages()->setName("My First Task");

    auto planner =
    std::make_shared<solvers::PipelinePlanner>(node);

    task.add(
        std::make_unique<stages::CurrentState>(
            "Current State"));

    auto move =
        std::make_unique<stages::MoveTo>(
            "Move To Rest",
            planner);

    move->setGroup("arm");

    move->setGoal("rest");

    task.add(std::move(move));

    RCLCPP_INFO(node->get_logger(),
                "Task created successfully!");

    rclcpp::shutdown();

    return 0;
}