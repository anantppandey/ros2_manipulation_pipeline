// #include <memory>
// #include <stdexcept>
// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>

// int main(int argc, char *argv[])
// {
//     try
//     {
//         // Initialize ROS
//         rclcpp::init(argc, argv);
//         auto const node = std::make_shared<rclcpp::Node>(
//             "moveit_trial",
//             rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

//         // Create a ROS logger
//         auto const logger = rclcpp::get_logger("moveit_trial");
//         RCLCPP_INFO(logger, "Starting MoveIt motion planning trial...");

//         // Create MoveGroupInterface
//         RCLCPP_INFO(logger, "Initializing MoveGroup interface for 'arm'...");
//         using moveit::planning_interface::MoveGroupInterface;
//         auto move_group_interface = MoveGroupInterface(node, "arm");

//         // Set planning time and velocity/acceleration scaling
//         move_group_interface.setPlanningTime(10.0);
//         move_group_interface.setMaxVelocityScalingFactor(0.5);
//         move_group_interface.setMaxAccelerationScalingFactor(0.5);

//         // Create target pose
//         geometry_msgs::msg::Pose target_pose;

//         // Set position (from red cube detector output)
//         target_pose.position.x = 0.093;
//         target_pose.position.y = 0.239;
//         target_pose.position.z = 0.257;

//         // Set orientation
//         target_pose.orientation.x = -0.465;
//         target_pose.orientation.y = 0.812;
//         target_pose.orientation.z = -0.310;
//         target_pose.orientation.w = 0.167;

//         RCLCPP_INFO(logger,
//                     "Target pose: position=(%f, %f, %f), orientation=(%f, %f, %f, %f)",
//                     target_pose.position.x, target_pose.position.y, target_pose.position.z,
//                     target_pose.orientation.x, target_pose.orientation.y,
//                     target_pose.orientation.z, target_pose.orientation.w);

//         // Set target pose
//         RCLCPP_INFO(logger, "Setting pose target...");
//         move_group_interface.setPoseTarget(target_pose);

//         // Create plan
//         RCLCPP_INFO(logger, "Planning motion to target pose...");
//         moveit::planning_interface::MoveGroupInterface::Plan plan;
//         auto plan_result = move_group_interface.plan(plan);

//         if (plan_result.val != moveit::core::MoveItErrorCode::SUCCESS)
//         {
//             RCLCPP_ERROR(logger, "Planning failed! Error code: %d", plan_result.val);
//             RCLCPP_WARN(logger, "Check if target pose is within arm's reach or collision-free.");
//             rclcpp::shutdown();
//             return 1;
//         }

//         RCLCPP_INFO(logger, "Motion plan computed successfully!");
//         RCLCPP_INFO(logger, "Plan trajectory has %zu waypoints", plan.trajectory.joint_trajectory.points.size());

//         // Execute plan
//         RCLCPP_INFO(logger, "Executing motion plan...");
//         auto exec_result = move_group_interface.execute(plan);

//         if (exec_result.val != moveit::core::MoveItErrorCode::SUCCESS)
//         {
//             RCLCPP_ERROR(logger, "Execution failed! Error code: %d", exec_result.val);
//             rclcpp::shutdown();
//             return 1;
//         }

//         RCLCPP_INFO(logger, "Motion execution completed successfully!");

//         // Wait a moment for motion to complete
//         rclcpp::sleep_for(std::chrono::milliseconds(500));

//         RCLCPP_INFO(logger, "Trial completed. Shutting down...");
//         rclcpp::shutdown();
//         return 0;
//     }
//     catch (const std::exception &e)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("moveit_trial"),
//                      "Exception caught: %s", e.what());
//         rclcpp::shutdown();
//         return 1;
//     }
//     catch (...)
//     {
//         RCLCPP_ERROR(rclcpp::get_logger("moveit_trial"),
//                      "Unknown exception caught!");
//         rclcpp::shutdown();
//         return 1;
//     }
// }

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("hellu", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    // Create a ROS logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // Next step goes here
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "arm");

    geometry_msgs::msg::Pose target_pose;

    //Base Pose
    // target_pose.position.x = 0.009;
    // target_pose.position.y = 0.33;
    // target_pose.position.z = 0.183;

    // target_pose.orientation.x = -0.706;
    // target_pose.orientation.y = 0.707;
    // target_pose.orientation.z = -0.0029;
    // target_pose.orientation.w = -0.0028;


    target_pose.position.x = 0.3894;
    target_pose.position.y = 0.4182;
    target_pose.position.z = -0.1366;

    target_pose.orientation.x = -0.7068;
    target_pose.orientation.y = 0.7073;
    target_pose.orientation.z = -0.0029;
    target_pose.orientation.w = -0.0028;

    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface.plan(plan);

    move_group_interface.execute(plan);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}