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

    auto node = std::make_shared<rclcpp::Node>(
    "mtc_node",
    rclcpp::NodeOptions()
        .automatically_declare_parameters_from_overrides(true)
        .parameter_overrides({
            rclcpp::Parameter("use_sim_time", true)
        }));

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    Task task;
    task.stages()->setName("My First Task");

    // Load robot model
    task.loadRobotModel(node);

    // Planner
    auto planner = std::make_shared<solvers::PipelinePlanner>(node);

    // Current State
    task.add(std::make_unique<stages::CurrentState>("Current State"));

    // Move to "rest"
    auto move = std::make_unique<stages::MoveTo>("Move To Rest", planner);
    move->setGroup("arm");
    move->setGoal("rest");

    // auto move = std::make_unique<stages::MoveTo>("move to cube", planner);
    // move->setGroup("arm");

    // geometry_msgs::msg::PoseStamped target_pose;

    // target_pose.header.frame_id = "world";
    // target_pose.header.stamp = node->now();

    // target_pose.pose.position.x = 0.09;
    // target_pose.pose.position.y = 0.35;
    // target_pose.pose.position.z = 0.07;

    // target_pose.pose.orientation.x = -0.59;
    // target_pose.pose.orientation.y = 0.81;
    // target_pose.pose.orientation.z = -0.02;
    // target_pose.pose.orientation.w = -0.02;

    move->setIKFrame("wrist_2");

    // move->setGoal(target_pose);

    task.add(std::move(move));

    // Initialize
    try
    {
        rclcpp::sleep_for(std::chrono::seconds(2));
        task.init();
        // rclcpp::sleep_for(std::chrono::seconds(30));
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(node->get_logger(),
                    "Task init failed: %s",
                    e.what());

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    if (!task.plan())
    {
        RCLCPP_ERROR(node->get_logger(),
                    "Planning failed");

        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(),
                "Planning successful!");
    executor.cancel();
    spinner.join();

    rclcpp::shutdown();

    return 0;
}