#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "moveit_trial",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .parameter_overrides({
                rclcpp::Parameter("use_sim_time", true)
            }));

    auto logger = rclcpp::get_logger("moveit_trial");

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]()
    {
        executor.spin();
    });

    using moveit::planning_interface::MoveGroupInterface;
    MoveGroupInterface move_group(node, "arm");

    move_group.startStateMonitor();
    rclcpp::sleep_for(std::chrono::seconds(2));
    move_group.setStartStateToCurrentState();

    move_group.setPlanningTime(10.0);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);

    //----------------------------------------------------
    // Information
    //----------------------------------------------------

    RCLCPP_INFO(logger,
        "Planning Frame : %s",
        move_group.getPlanningFrame().c_str());

    RCLCPP_INFO(logger,
        "EE Link : %s",
        move_group.getEndEffectorLink().c_str());

    //----------------------------------------------------
    // Current pose
    //----------------------------------------------------

    auto current_pose = move_group.getCurrentPose("wrist_2");

    RCLCPP_INFO(logger,
        "Current Position : %.3f %.3f %.3f",
        current_pose.pose.position.x,
        current_pose.pose.position.y,
        current_pose.pose.position.z);

    RCLCPP_INFO(logger,
        "Current Orientation : %.3f %.3f %.3f %.3f",
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);

    //----------------------------------------------------
    // Target pose
    //----------------------------------------------------

    geometry_msgs::msg::Pose target;

    // Position from tf2
    target.position.x = 0.09;
    target.position.y = 0.35;
    target.position.z = 0.07;

    // Orientation from tf2
    target.orientation.x = -0.59;
    target.orientation.y = 0.81;
    target.orientation.z = -0.02;
    target.orientation.w = -0.02;

    RCLCPP_INFO(logger,
        "Target Position : %.3f %.3f %.3f",
        target.position.x,
        target.position.y,
        target.position.z);

    RCLCPP_INFO(logger,
        "Target Orientation : %.3f %.3f %.3f %.3f",
        target.orientation.x,
        target.orientation.y,
        target.orientation.z,
        target.orientation.w);

    move_group.clearPoseTargets();

    move_group.setPoseTarget(target, "wrist_2");

    //----------------------------------------------------
    // Plan
    //----------------------------------------------------

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto result = move_group.plan(plan);

    if(result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(logger,"Planning SUCCESS");

        move_group.execute(plan);
    }
    else
    {
        RCLCPP_ERROR(logger,"Planning FAILED");
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}