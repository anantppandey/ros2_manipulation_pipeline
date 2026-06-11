#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>

class CubePickerNode : public rclcpp::Node, public std::enable_shared_from_this<CubePickerNode>
{
public:
    CubePickerNode() : Node("cube_picker")
    {
        cube_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/red_cube/target_pose",
            10,
            std::bind(&CubePickerNode::cube_pose_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Cube picker node started, waiting for target pose...");
    }

    void init_move_group()
    {
        if (!move_group_interface_)
        {
            move_group_interface_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                std::enable_shared_from_this<CubePickerNode>::shared_from_this(), "arm");
            move_group_interface_->setPlanningTime(10.0);
            move_group_interface_->setMaxVelocityScalingFactor(0.5);
            move_group_interface_->setMaxAccelerationScalingFactor(0.5);
        }
    }

private:
    void cube_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received target pose frame='%s' position=(%.4f, %.4f, %.4f)",
                    msg->header.frame_id.c_str(),
                    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

        move_group_interface_->setPoseTarget(*msg);
        move_group_interface_->setStartStateToCurrentState();

        RCLCPP_INFO(this->get_logger(), "Planning to received target pose...");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto plan_result = move_group_interface_->plan(plan);
        if (plan_result.val != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_WARN(this->get_logger(), "Planning failed with code %d", plan_result.val);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Plan successful! Executing...");
        auto exec_result = move_group_interface_->execute(plan);
        if (exec_result.val != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Execution failed with code %d", exec_result.val);
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Movement complete!");
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr cube_pose_sub_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CubePickerNode>();
    node->init_move_group();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
