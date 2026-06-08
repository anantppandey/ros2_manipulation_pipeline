#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class CubePickerNode : public rclcpp::Node
{
public:
    CubePickerNode() : Node("cube_picker")
    {
        // Subscribe to cube position (published by red_cube_detector)
        cube_position_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/red_cube/moveit_target",
            10,
            std::bind(&CubePickerNode::cube_position_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cube picker node started, waiting for cube position...");
    }

private:
    void cube_position_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received cube position: x=%.4f, y=%.4f, z=%.4f",
                    msg->point.x, msg->point.y, msg->point.z);

        // Create move group interface
        auto move_group_interface = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            this->shared_from_this(), "arm");

        // Create target pose above the cube (add some Z offset to avoid collision)
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = msg->point.x;
        target_pose.position.y = msg->point.y;
        target_pose.position.z = msg->point.z + 0.1; // 10cm above the cube

        // Set a default orientation (pointing down)
        target_pose.orientation.x = -0.465;
        target_pose.orientation.y = 0.812;
        target_pose.orientation.z = -0.310;
        target_pose.orientation.w = 0.167;

        move_group_interface->setPoseTarget(target_pose);

        RCLCPP_INFO(this->get_logger(), "Planning to target pose...");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Plan successful! Executing...");
            move_group_interface->execute(plan);
            RCLCPP_INFO(this->get_logger(), "Movement complete!");
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed!");
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr cube_position_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubePickerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
