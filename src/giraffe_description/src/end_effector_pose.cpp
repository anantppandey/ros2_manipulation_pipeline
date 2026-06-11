#include <cmath>
#include <memory>
#include <string>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

bool isPoseZero(const geometry_msgs::msg::PoseStamped &pose)
{
    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;
    return p.x == 0.0 && p.y == 0.0 && p.z == 0.0 && q.x == 0.0 && q.y == 0.0 && q.z == 0.0 && q.w == 1.0;
}

static void printPose(const std::string &label, const geometry_msgs::msg::PoseStamped &pose, rclcpp::Logger logger)
{
    const auto &p = pose.pose.position;
    const auto &q = pose.pose.orientation;
    const double quaternion_norm = std::sqrt(
        q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    RCLCPP_INFO(logger, "%s frame_id='%s' stamp=%u.%09u",
                label.c_str(), pose.header.frame_id.c_str(), pose.header.stamp.sec, pose.header.stamp.nanosec);
    RCLCPP_INFO(logger, "%s position: x=%.6f y=%.6f z=%.6f",
                label.c_str(), p.x, p.y, p.z);
    RCLCPP_INFO(logger, "%s orientation: x=%.6f y=%.6f z=%.6f w=%.6f norm=%.6f",
                label.c_str(), q.x, q.y, q.z, q.w, quaternion_norm);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("end_effector_pose");
    auto logger = rclcpp::get_logger("end_effector_pose");

    using namespace std::chrono_literals;
    using moveit::planning_interface::MoveGroupInterface;

    RCLCPP_INFO(logger, "Starting MoveIt end-effector pose reader...");
    MoveGroupInterface move_group(node, "arm");

    const std::string planning_frame = move_group.getPlanningFrame();
    const std::string ee_link = move_group.getEndEffectorLink();
    RCLCPP_INFO(logger, "MoveIt planning group='arm', planning_frame='%s', end-effector link='%s'",
                planning_frame.c_str(), ee_link.c_str());

    geometry_msgs::msg::PoseStamped current_pose;
    for (int attempt = 0; attempt < 20; ++attempt)
    {
        current_pose = move_group.getCurrentPose();
        if (!isPoseZero(current_pose))
        {
            break;
        }
        RCLCPP_INFO(logger, "Waiting for MoveIt robot state... (%d/20)", attempt + 1);
        rclcpp::sleep_for(250ms);
    }

    if (isPoseZero(current_pose))
    {
        RCLCPP_WARN(logger, "MoveIt current pose is still zero after waiting. Falling back to TF lookup.");
    }
    else
    {
        printPose("MoveIt current pose", current_pose, logger);
    }

    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    geometry_msgs::msg::PoseStamped tf_pose;
    bool tf_valid = false;

    try
    {
        auto transform = tf_buffer.lookupTransform(planning_frame, ee_link, tf2::TimePointZero, 5s);
        tf_pose.header.frame_id = planning_frame;
        tf_pose.header.stamp = rclcpp::Time(transform.header.stamp);
        tf_pose.pose.position.x = transform.transform.translation.x;
        tf_pose.pose.position.y = transform.transform.translation.y;
        tf_pose.pose.position.z = transform.transform.translation.z;
        tf_pose.pose.orientation = transform.transform.rotation;
        tf_valid = true;
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_WARN(logger, "TF lookup failed: %s", ex.what());
    }

    if (tf_valid)
    {
        printPose("TF pose", tf_pose, logger);
    }

    if (!isPoseZero(current_pose))
    {
        const auto &p = current_pose.pose.position;
        const auto &q = current_pose.pose.orientation;
        RCLCPP_INFO(logger, "Copy these MoveIt target values into moveit_trial.cpp:");
        RCLCPP_INFO(logger, "target_pose.position.x = %.6f;", p.x);
        RCLCPP_INFO(logger, "target_pose.position.y = %.6f;", p.y);
        RCLCPP_INFO(logger, "target_pose.position.z = %.6f;", p.z);
        RCLCPP_INFO(logger, "target_pose.orientation.x = %.6f;", q.x);
        RCLCPP_INFO(logger, "target_pose.orientation.y = %.6f;", q.y);
        RCLCPP_INFO(logger, "target_pose.orientation.z = %.6f;", q.z);
        RCLCPP_INFO(logger, "target_pose.orientation.w = %.6f;", q.w);
    }
    else if (tf_valid)
    {
        const auto &p = tf_pose.pose.position;
        const auto &q = tf_pose.pose.orientation;
        RCLCPP_INFO(logger, "Copy these TF target values into moveit_trial.cpp:");
        RCLCPP_INFO(logger, "target_pose.position.x = %.6f;", p.x);
        RCLCPP_INFO(logger, "target_pose.position.y = %.6f;", p.y);
        RCLCPP_INFO(logger, "target_pose.position.z = %.6f;", p.z);
        RCLCPP_INFO(logger, "target_pose.orientation.x = %.6f;", q.x);
        RCLCPP_INFO(logger, "target_pose.orientation.y = %.6f;", q.y);
        RCLCPP_INFO(logger, "target_pose.orientation.z = %.6f;", q.z);
        RCLCPP_INFO(logger, "target_pose.orientation.w = %.6f;", q.w);
    }
    else
    {
        RCLCPP_ERROR(logger, "Unable to obtain a valid end-effector pose from MoveIt or TF.");
    }

    rclcpp::shutdown();
    return 0;
}
