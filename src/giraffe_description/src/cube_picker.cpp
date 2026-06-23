#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class MoveToRedCube : public rclcpp::Node
{
public:
    MoveToRedCube() : Node("move_to_red_cube")
    {
        // Declare parameters
        this->declare_parameter<std::string>("planning_group", "arm");
        this->declare_parameter<std::string>("end_effector_link", "wrist_2_link");
        this->declare_parameter<double>("planning_time", 5.0);
        this->declare_parameter<int>("planning_attempts", 10);
        this->declare_parameter<int>("movement_method", 2); // 1=Absolute, 2=Relative, 3=Cartesian

        // Get parameters
        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        planning_time_ = this->get_parameter("planning_time").as_double();
        planning_attempts_ = this->get_parameter("planning_attempts").as_int();
        movement_method_ = this->get_parameter("movement_method").as_int();

        // Subscribe to cube pose
        cube_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/moveit/end_effector_to_cube_pose", 10,
            std::bind(&MoveToRedCube::cubePoseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MoveToRedCube initialized (ROS2 C++)");
        RCLCPP_INFO_STREAM(this->get_logger(), "Planning group: " << planning_group_);
        RCLCPP_INFO_STREAM(this->get_logger(), "End effector link: " << end_effector_link_);
        RCLCPP_INFO(this->get_logger(), "Waiting for red cube detection...");

        // Create timer for execution after receiving pose
        execution_timer_ = this->create_wall_timer(
            500ms, std::bind(&MoveToRedCube::executeMovement, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_pose_sub_;
    rclcpp::TimerBase::SharedPtr execution_timer_;
    
    std::string planning_group_;
    std::string end_effector_link_;
    double planning_time_;
    int planning_attempts_;
    int movement_method_;
    
    geometry_msgs::msg::PoseStamped target_pose_;
    bool pose_received_ = false;
    bool movement_executed_ = false;

    void cubePoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        target_pose_ = *msg;
        pose_received_ = true;
    }

    void printCurrentState(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        RCLCPP_INFO(this->get_logger(), "=== Current Robot State ===");
        RCLCPP_INFO_STREAM(this->get_logger(), "Planning frame: " << move_group.getPlanningFrame());
        RCLCPP_INFO_STREAM(this->get_logger(), "End effector link: " << move_group.getEndEffectorLink());

        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current EE pose: x=%.3f, y=%.3f, z=%.3f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);

        std::vector<double> joint_values = move_group.getCurrentJointValues();
        RCLCPP_INFO(this->get_logger(), "Current joint values:");
        for (size_t i = 0; i < joint_values.size(); i++)
        {
            RCLCPP_INFO(this->get_logger(), "  Joint %zu: %.3f", i, joint_values[i]);
        }
    }

    bool moveToCubeAbsolute(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to cube using absolute positioning...");

        geometry_msgs::msg::Pose target_pose;
        target_pose.position = target_pose_.pose.position;
        target_pose.orientation = target_pose_.pose.orientation;

        move_group.setPoseTarget(target_pose, end_effector_link_);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
            move_group.execute(plan);
            RCLCPP_INFO(this->get_logger(), "Movement completed successfully!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }

        move_group.clearPoseTargets();
        return success;
    }

    bool moveToCubeRelative(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to cube using relative positioning...");

        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link_);

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.pose.position.x + target_pose_.pose.position.x;
        target_pose.position.y = current_pose.pose.position.y + target_pose_.pose.position.y;
        target_pose.position.z = current_pose.pose.position.z + target_pose_.pose.position.z;
        target_pose.orientation = target_pose_.pose.orientation;

        RCLCPP_INFO(this->get_logger(), "Target position: x=%.3f, y=%.3f, z=%.3f",
                    target_pose.position.x, target_pose.position.y, target_pose.position.z);

        move_group.setPoseTarget(target_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning successful. Executing...");
            move_group.execute(plan);
            RCLCPP_INFO(this->get_logger(), "Movement completed successfully!");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }

        move_group.clearPoseTargets();
        return success;
    }

    bool planAndExecuteCartesian(moveit::planning_interface::MoveGroupInterface& move_group)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to cube using Cartesian path...");

        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link_);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(current_pose.pose);

        // Pre-grasp pose (10cm above)
        geometry_msgs::msg::Pose pre_grasp_pose;
        pre_grasp_pose.position.x = current_pose.pose.position.x + target_pose_.pose.position.x;
        pre_grasp_pose.position.y = current_pose.pose.position.y + target_pose_.pose.position.y;
        pre_grasp_pose.position.z = current_pose.pose.position.z + target_pose_.pose.position.z + 0.1;
        pre_grasp_pose.orientation = target_pose_.pose.orientation;
        waypoints.push_back(pre_grasp_pose);

        // Final pose
        geometry_msgs::msg::Pose final_pose;
        final_pose.position.x = current_pose.pose.position.x + target_pose_.pose.position.x;
        final_pose.position.y = current_pose.pose.position.y + target_pose_.pose.position.y;
        final_pose.position.z = current_pose.pose.position.z + target_pose_.pose.position.z;
        final_pose.orientation = target_pose_.pose.orientation;
        waypoints.push_back(final_pose);

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.01;
        
        // Use the new API without jump_threshold parameter
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        if (fraction >= 0.8)
        {
            RCLCPP_INFO(this->get_logger(), "Cartesian path planned (%.1f%%). Executing...", fraction * 100);
            move_group.execute(trajectory);
            return true;
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cartesian path planning failed. Fraction: %.1f%%", fraction * 100);
            return false;
        }
    }

    void executeMovement()
    {
        if (!pose_received_ || movement_executed_)
            return;

        RCLCPP_INFO(this->get_logger(), "Cube detected! Moving to cube...");

        // Initialize MoveIt
        rclcpp::Node::SharedPtr node = shared_from_this();
        auto move_group = moveit::planning_interface::MoveGroupInterface(node, planning_group_);
        
        move_group.setPlanningTime(planning_time_);
        move_group.setNumPlanningAttempts(planning_attempts_);
        move_group.allowReplanning(true);
        move_group.setEndEffectorLink(end_effector_link_);

        printCurrentState(move_group);

        bool success = false;
        switch (movement_method_)
        {
            case 1:
                success = moveToCubeAbsolute(move_group);
                break;
            case 2:
                success = moveToCubeRelative(move_group);
                break;
            case 3:
                success = planAndExecuteCartesian(move_group);
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Invalid movement method!");
                break;
        }

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully moved to red cube!");
            printCurrentState(move_group);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to move to red cube");
        }

        movement_executed_ = true;
        execution_timer_->cancel();
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToRedCube>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}