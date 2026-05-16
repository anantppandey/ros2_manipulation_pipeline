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

    target_pose.position.x = 0.093;
    target_pose.position.y = 0.239;
    target_pose.position.z = 0.257;

    target_pose.orientation.x = -0.465;
    target_pose.orientation.y = 0.812;
    target_pose.orientation.z = -0.310;
    target_pose.orientation.w = 0.167;

    move_group_interface.setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group_interface.plan(plan);

    move_group_interface.execute(plan);

    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}