// #include <memory>
// #include <thread>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mutex>

// geometry_msgs::msg::Pose latest_pose;
// bool pose_received = false;

// std::mutex pose_mutex;

// void poseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(pose_mutex);

//     latest_pose = msg->pose;
//     pose_received = true;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "moveit_trial",
//         rclcpp::NodeOptions()
//             .automatically_declare_parameters_from_overrides(true)
//             .parameter_overrides({
//                 rclcpp::Parameter("use_sim_time", true)
//             }));

//     auto logger = rclcpp::get_logger("moveit_trial");

//     auto pose_sub =
//         node->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/cube_pose",
//             10,
//             poseCallback);

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);

//     std::thread spinner([&executor]()
//     {
//         executor.spin();
//     });

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "arm");

//     move_group.startStateMonitor();
//     rclcpp::sleep_for(std::chrono::seconds(2));
//     move_group.setStartStateToCurrentState();

//     move_group.setPlanningTime(10.0);
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     //----------------------------------------------------
//     // Information
//     //----------------------------------------------------

//     RCLCPP_INFO(logger,
//         "Planning Frame : %s",
//         move_group.getPlanningFrame().c_str());

//     RCLCPP_INFO(logger,
//         "EE Link : %s",
//         move_group.getEndEffectorLink().c_str());

//     //----------------------------------------------------
//     // Current pose
//     //----------------------------------------------------

//     auto current_pose = move_group.getCurrentPose("wrist_2");

//     RCLCPP_INFO(logger,
//         "Current Position : %.3f %.3f %.3f",
//         current_pose.pose.position.x,
//         current_pose.pose.position.y,
//         current_pose.pose.position.z);

//     RCLCPP_INFO(logger,
//         "Current Orientation : %.3f %.3f %.3f %.3f",
//         current_pose.pose.orientation.x,
//         current_pose.pose.orientation.y,
//         current_pose.pose.orientation.z,
//         current_pose.pose.orientation.w);

//     //----------------------------------------------------
//     // Target pose
//     //----------------------------------------------------

//     // geometry_msgs::msg::Pose target;

//     // // Position from tf2
//     // target.position.x = 0.09;
//     // target.position.y = 0.35;
//     // target.position.z = 0.07;

//     // // Orientation from tf2
//     // target.orientation.x = -0.59;
//     // target.orientation.y = 0.81;
//     // target.orientation.z = -0.02;
//     // target.orientation.w = -0.02;


//     geometry_msgs::msg::Pose target;

//     RCLCPP_INFO(logger, "Waiting for cube pose...");

//     while (rclcpp::ok() && !pose_received)
//     {
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     {
//         std::lock_guard<std::mutex> lock(pose_mutex);
//         target = latest_pose;
//     }

//     RCLCPP_INFO(logger,
//         "Target Position : %.3f %.3f %.3f",
//         target.position.x,
//         target.position.y,
//         target.position.z);

//     RCLCPP_INFO(logger,
//         "Target Orientation : %.3f %.3f %.3f %.3f",
//         target.orientation.x,
//         target.orientation.y,
//         target.orientation.z,
//         target.orientation.w);

//     move_group.clearPoseTargets();

//     move_group.setPoseTarget(target, "wrist_2");

//     //----------------------------------------------------
//     // Plan
//     //----------------------------------------------------

//     moveit::planning_interface::MoveGroupInterface::Plan plan;

//     auto result = move_group.plan(plan);

//     if(result == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//         RCLCPP_INFO(logger,"Planning SUCCESS");

//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger,"Planning FAILED");
//     }

//     executor.cancel();
//     spinner.join();

//     rclcpp::shutdown();
//     return 0;
// }

// ATTEMPT 2: Using Approximate IK for 5-DOF Arm WORKING BUT OFF

// #include <memory>
// #include <thread>
// #include <cmath> // For std::sqrt

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mutex>

// geometry_msgs::msg::Pose latest_pose;
// bool pose_received = false;

// std::mutex pose_mutex;

// void poseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(pose_mutex);

//     latest_pose = msg->pose;
//     pose_received = true;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "moveit_trial",
//         rclcpp::NodeOptions()
//             .automatically_declare_parameters_from_overrides(true)
//             .parameter_overrides({
//                 rclcpp::Parameter("use_sim_time", true)
//             }));

//     auto logger = rclcpp::get_logger("moveit_trial");

//     auto pose_sub =
//         node->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/cube_pose",
//             10,
//             poseCallback);

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);

//     std::thread spinner([&executor]()
//     {
//         executor.spin();
//     });

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "arm");

//     move_group.startStateMonitor();
//     rclcpp::sleep_for(std::chrono::seconds(2));
//     move_group.setStartStateToCurrentState();

//     move_group.setPlanningTime(10.0);
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     //----------------------------------------------------
//     // Information
//     //----------------------------------------------------

//     RCLCPP_INFO(logger,
//         "Planning Frame : %s",
//         move_group.getPlanningFrame().c_str());

//     RCLCPP_INFO(logger,
//         "EE Link : %s",
//         move_group.getEndEffectorLink().c_str());

//     //----------------------------------------------------
//     // Current pose
//     //----------------------------------------------------

//     auto current_pose = move_group.getCurrentPose("wrist_2");

//     RCLCPP_INFO(logger,
//         "Current Position : %.3f %.3f %.3f",
//         current_pose.pose.position.x,
//         current_pose.pose.position.y,
//         current_pose.pose.position.z);

//     RCLCPP_INFO(logger,
//         "Current Orientation : %.3f %.3f %.3f %.3f",
//         current_pose.pose.orientation.x,
//         current_pose.pose.orientation.y,
//         current_pose.pose.orientation.z,
//         current_pose.pose.orientation.w);

//     //----------------------------------------------------
//     // Target pose
//     //----------------------------------------------------

//     geometry_msgs::msg::Pose target;

//     RCLCPP_INFO(logger, "Waiting for cube pose...");

//     while (rclcpp::ok() && !pose_received)
//     {
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     {
//         std::lock_guard<std::mutex> lock(pose_mutex);
//         target = latest_pose;
//     }

//     // ==========================================
//     // QUATERNION NORMALIZATION & SIGN FLIPPING
//     // ==========================================
//     double norm = std::sqrt(
//         target.orientation.x * target.orientation.x +
//         target.orientation.y * target.orientation.y +
//         target.orientation.z * target.orientation.z +
//         target.orientation.w * target.orientation.w);

//     if (norm < 1e-6) {
//         RCLCPP_ERROR(logger, "Received quaternion has near-zero norm! Cannot normalize.");
//     } else {
//         // Normalize to unit length
//         target.orientation.x /= norm;
//         target.orientation.y /= norm;
//         target.orientation.z /= norm;
//         target.orientation.w /= norm;
//     }

//     // Check dot product with current orientation to ensure shortest path
//     double dot = current_pose.pose.orientation.x * target.orientation.x +
//                  current_pose.pose.orientation.y * target.orientation.y +
//                  current_pose.pose.orientation.z * target.orientation.z +
//                  current_pose.pose.orientation.w * target.orientation.w;

//     if (dot < 0.0) {
//         RCLCPP_INFO(logger, "Flipping target quaternion sign for shortest path.");
//         target.orientation.x *= -1.0;
//         target.orientation.y *= -1.0;
//         target.orientation.z *= -1.0;
//         target.orientation.w *= -1.0;
//     }
//     // ==========================================

//     RCLCPP_INFO(logger,
//         "Target Position : %.3f %.3f %.3f",
//         target.position.x,
//         target.position.y,
//         target.position.z);

//     RCLCPP_INFO(logger,
//         "Target Orientation : %.3f %.3f %.3f %.3f",
//         target.orientation.x,
//         target.orientation.y,
//         target.orientation.z,
//         target.orientation.w);

//     move_group.clearPoseTargets();

//     // Use current state as the IK seed (mimics RViz behavior)
//     move_group.setStartStateToCurrentState();

//     // ==========================================
//     // USE APPROXIMATE IK
//     // This is the magic that RViz uses! It finds the closest reachable 
//     // pose for your 5-DOF arm instead of failing on exact orientation.
//     // ==========================================
//     bool success = move_group.setApproximateJointValueTarget(target, "wrist_2");
    
//     if (!success) {
//         RCLCPP_ERROR(logger, "Approximate IK failed! The pose is entirely unreachable.");
        
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     RCLCPP_INFO(logger, "Approximate IK success! Planning to closest joint configuration.");

//     //----------------------------------------------------
//     // Plan
//     //----------------------------------------------------

//     moveit::planning_interface::MoveGroupInterface::Plan plan;

//     auto result = move_group.plan(plan);

//     if(result == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//         RCLCPP_INFO(logger,"Planning SUCCESS");

//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger,"Planning FAILED");
//     }

//     executor.cancel();
//     spinner.join();

//     rclcpp::shutdown();
//     return 0;
// }


// YAYY FINALLY WORKING WITH EXACT IK FOR 5-DOF ARM USING BRUTE-FORCE RANDOM SEEDS BUT GRIPPER OFFSET

// #include <memory>
// #include <thread>
// #include <cmath>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/robot_state/robot_state.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mutex>
// #include <Eigen/Geometry> // Added for FK math

// geometry_msgs::msg::Pose latest_pose;
// bool pose_received = false;

// std::mutex pose_mutex;

// void poseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(pose_mutex);
//     latest_pose = msg->pose;
//     pose_received = true;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "moveit_trial",
//         rclcpp::NodeOptions()
//             .automatically_declare_parameters_from_overrides(true)
//             .parameter_overrides({
//                 rclcpp::Parameter("use_sim_time", true)
//             }));

//     auto logger = rclcpp::get_logger("moveit_trial");

//     auto pose_sub =
//         node->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/cube_pose",
//             10,
//             poseCallback);

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);

//     std::thread spinner([&executor]()
//     {
//         executor.spin();
//     });

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "arm");

//     move_group.startStateMonitor();
//     rclcpp::sleep_for(std::chrono::seconds(2));
//     move_group.setStartStateToCurrentState();

//     move_group.setPlanningTime(10.0);
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     RCLCPP_INFO(logger, "Planning Frame : %s", move_group.getPlanningFrame().c_str());
//     RCLCPP_INFO(logger, "EE Link : %s", move_group.getEndEffectorLink().c_str());

//     auto current_pose = move_group.getCurrentPose("wrist_2");

//     RCLCPP_INFO(logger, "Current Position : %.3f %.3f %.3f",
//         current_pose.pose.position.x,
//         current_pose.pose.position.y,
//         current_pose.pose.position.z);

//     geometry_msgs::msg::Pose target;

//     RCLCPP_INFO(logger, "Waiting for cube pose...");
//     while (rclcpp::ok() && !pose_received)
//     {
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     {
//         std::lock_guard<std::mutex> lock(pose_mutex);
//         target = latest_pose;
//     }

//     // Normalize the quaternion
//     double norm = std::sqrt(
//         target.orientation.x * target.orientation.x +
//         target.orientation.y * target.orientation.y +
//         target.orientation.z * target.orientation.z +
//         target.orientation.w * target.orientation.w);

//     if (norm > 1e-6) {
//         target.orientation.x /= norm;
//         target.orientation.y /= norm;
//         target.orientation.z /= norm;
//         target.orientation.w /= norm;
//     }

//     RCLCPP_INFO(logger, "Target Position : %.3f %.3f %.3f",
//         target.position.x, target.position.y, target.position.z);

//     move_group.clearPoseTargets();
//     move_group.setStartStateToCurrentState();

//     // ==========================================
//     // BRUTE-FORCE IK WITH 100 RANDOM SEEDS
//     // ==========================================
//     moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//     if (!current_state) {
//         RCLCPP_ERROR(logger, "Failed to get current robot state!");
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     moveit::core::RobotState target_state(*current_state);
//     const moveit::core::JointModelGroup* jmg = target_state.getJointModelGroup("arm");

//     // Tell KDL to always return the closest solution it finds, even if it's not perfect
//     kinematics::KinematicsQueryOptions options;
//     options.return_approximate_solution = true;

//     bool found_exact = false;
//     double best_error = 1e9;
//     std::vector<double> best_joint_values;

//     RCLCPP_INFO(logger, "Computing IK with 100 random seeds to find absolute best match...");

//     moveit::core::GroupStateValidityCallbackFn constraint;

//     for (int i = 0; i < 100; ++i) {
//         // On attempt 0, use current state. On 1-99, randomize the joints
//         if (i > 0) {
//             target_state.setToRandomPositions(jmg);
//         }
        
//         // Try to solve IK (timeout 0.1s per attempt)
//         bool ik_success = target_state.setFromIK(jmg, target, "wrist_2", 0.1, constraint, options);
        
//         if (ik_success) {
//             // Calculate where the wrist actually ended up
//             Eigen::Isometry3d fk_pose = target_state.getGlobalLinkTransform("wrist_2");
            
//             // Measure Cartesian distance to the target
//             double dx = fk_pose.translation().x() - target.position.x;
//             double dy = fk_pose.translation().y() - target.position.y;
//             double dz = fk_pose.translation().z() - target.position.z;
//             double error = std::sqrt(dx*dx + dy*dy + dz*dz);
            
//             // If we find a virtually perfect match ( < 1mm error), stop searching
//             if (error < 0.001) {
//                 found_exact = true;
//                 best_joint_values.clear();
//                 target_state.copyJointGroupPositions("arm", best_joint_values);
//                 RCLCPP_INFO(logger, "Found exact IK solution on attempt %d! Error: %.6f m", i, error);
//                 break;
//             }
            
//             // Otherwise, keep track of the closest one we've found so far
//             if (error < best_error) {
//                 best_error = error;
//                 best_joint_values.clear();
//                 target_state.copyJointGroupPositions("arm", best_joint_values);
//             }
//         }
//     }

//     if (found_exact) {
//         move_group.setJointValueTarget(best_joint_values);
//         RCLCPP_INFO(logger, "Sending exact joint target to planner.");
//     } else if (!best_joint_values.empty()) {
//         RCLCPP_WARN(logger, "Could not find exact IK. Best Cartesian error: %.4f m. Using best approximate.", best_error);
//         move_group.setJointValueTarget(best_joint_values);
//     } else {
//         RCLCPP_ERROR(logger, "IK failed completely! Pose is physically unreachable.");
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     //----------------------------------------------------
//     // Plan
//     //----------------------------------------------------

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto result = move_group.plan(plan);

//     if(result == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//         RCLCPP_INFO(logger,"Planning SUCCESS");
//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger,"Planning FAILED");
//     }

//     executor.cancel();
//     spinner.join();

//     rclcpp::shutdown();
//     return 0;
// }



//GRIPPER OFFSET FIXED VERSION BUT HITTING GROUND


// #include <memory>
// #include <thread>
// #include <cmath>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/robot_state/robot_state.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mutex>
// #include <Eigen/Geometry> // Added for transform math

// geometry_msgs::msg::Pose latest_pose;
// bool pose_received = false;

// std::mutex pose_mutex;

// void poseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(pose_mutex);
//     latest_pose = msg->pose;
//     pose_received = true;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "moveit_trial",
//         rclcpp::NodeOptions()
//             .automatically_declare_parameters_from_overrides(true)
//             .parameter_overrides({
//                 rclcpp::Parameter("use_sim_time", true)
//             }));

//     auto logger = rclcpp::get_logger("moveit_trial");

//     auto pose_sub =
//         node->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/cube_pose",
//             10,
//             poseCallback);

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);

//     std::thread spinner([&executor]()
//     {
//         executor.spin();
//     });

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "arm");

//     move_group.startStateMonitor();
//     rclcpp::sleep_for(std::chrono::seconds(2));
//     move_group.setStartStateToCurrentState();

//     move_group.setPlanningTime(10.0);
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     RCLCPP_INFO(logger, "Planning Frame : %s", move_group.getPlanningFrame().c_str());
//     RCLCPP_INFO(logger, "EE Link : %s", move_group.getEndEffectorLink().c_str());

//     auto current_pose = move_group.getCurrentPose("wrist_2");

//     RCLCPP_INFO(logger, "Current Position : %.3f %.3f %.3f",
//         current_pose.pose.position.x,
//         current_pose.pose.position.y,
//         current_pose.pose.position.z);

//     geometry_msgs::msg::Pose target;

//     RCLCPP_INFO(logger, "Waiting for cube pose...");
//     while (rclcpp::ok() && !pose_received)
//     {
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     {
//         std::lock_guard<std::mutex> lock(pose_mutex);
//         target = latest_pose; // This is where we want the GRIPPER to go
//     }

//     // Normalize the quaternion
//     double norm = std::sqrt(
//         target.orientation.x * target.orientation.x +
//         target.orientation.y * target.orientation.y +
//         target.orientation.z * target.orientation.z +
//         target.orientation.w * target.orientation.w);

//     if (norm > 1e-6) {
//         target.orientation.x /= norm;
//         target.orientation.y /= norm;
//         target.orientation.z /= norm;
//         target.orientation.w /= norm;
//     }

//     move_group.clearPoseTargets();
//     move_group.setStartStateToCurrentState();

//     // ==========================================
//     // GET CURRENT ROBOT STATE
//     // ==========================================
//     moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
//     if (!current_state) {
//         RCLCPP_ERROR(logger, "Failed to get current robot state!");
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     // ==========================================
//     // TRANSFORM TARGET FROM GRIPPER TO WRIST_2
//     // ==========================================
//     // The cube pose targets the gripper, but our IK solver targets wrist_2.
//     // We need to shift the target pose backwards by the offset between them.
//     Eigen::Isometry3d wrist_2_pose = current_state->getGlobalLinkTransform("wrist_2");
//     Eigen::Isometry3d gripper_pose = current_state->getGlobalLinkTransform("gripper");
    
//     // Get the transform from wrist_2 to gripper
//     Eigen::Isometry3d wrist_2_to_gripper = wrist_2_pose.inverse() * gripper_pose;

//     // Convert ROS target pose to Eigen
//     Eigen::Isometry3d target_gripper_eigen = Eigen::Isometry3d::Identity();
//     target_gripper_eigen.translation() = Eigen::Vector3d(target.position.x, target.position.y, target.position.z);
//     Eigen::Quaterniond q_target(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
//     q_target.normalize();
//     target_gripper_eigen.rotate(q_target);

//     // Shift the target so it represents where wrist_2 needs to be
//     Eigen::Isometry3d target_wrist_2_eigen = target_gripper_eigen * wrist_2_to_gripper.inverse();

//     // Convert back to ROS Pose for KDL
//     geometry_msgs::msg::Pose target_wrist_2;
//     target_wrist_2.position.x = target_wrist_2_eigen.translation().x();
//     target_wrist_2.position.y = target_wrist_2_eigen.translation().y();
//     target_wrist_2.position.z = target_wrist_2_eigen.translation().z();
//     Eigen::Quaterniond q_wrist_2(target_wrist_2_eigen.rotation());
//     target_wrist_2.orientation.x = q_wrist_2.x();
//     target_wrist_2.orientation.y = q_wrist_2.y();
//     target_wrist_2.orientation.z = q_wrist_2.z();
//     target_wrist_2.orientation.w = q_wrist_2.w();

//     RCLCPP_INFO(logger, "Shifted Target for wrist_2 -> Position : %.3f %.3f %.3f",
//         target_wrist_2.position.x, target_wrist_2.position.y, target_wrist_2.position.z);

//     // ==========================================
//     // BRUTE-FORCE IK WITH 100 RANDOM SEEDS
//     // ==========================================
//     moveit::core::RobotState target_state(*current_state);
//     const moveit::core::JointModelGroup* jmg = target_state.getJointModelGroup("arm");

//     kinematics::KinematicsQueryOptions options;
//     options.return_approximate_solution = true;

//     bool found_exact = false;
//     double best_error = 1e9;
//     std::vector<double> best_joint_values;

//     RCLCPP_INFO(logger, "Computing IK with 100 random seeds to find absolute best match...");

//     moveit::core::GroupStateValidityCallbackFn constraint;

//     for (int i = 0; i < 100; ++i) {
//         if (i > 0) {
//             target_state.setToRandomPositions(jmg);
//         }
        
//         // Solve IK using the SHIFTED wrist_2 target
//         bool ik_success = target_state.setFromIK(jmg, target_wrist_2, "wrist_2", 0.1, constraint, options);
        
//         if (ik_success) {
//             Eigen::Isometry3d fk_pose = target_state.getGlobalLinkTransform("wrist_2");
            
//             double dx = fk_pose.translation().x() - target_wrist_2.position.x;
//             double dy = fk_pose.translation().y() - target_wrist_2.position.y;
//             double dz = fk_pose.translation().z() - target_wrist_2.position.z;
//             double error = std::sqrt(dx*dx + dy*dy + dz*dz);
            
//             if (error < 0.001) {
//                 found_exact = true;
//                 best_joint_values.clear();
//                 target_state.copyJointGroupPositions("arm", best_joint_values);
//                 RCLCPP_INFO(logger, "Found exact IK solution on attempt %d! Error: %.6f m", i, error);
//                 break;
//             }
            
//             if (error < best_error) {
//                 best_error = error;
//                 best_joint_values.clear();
//                 target_state.copyJointGroupPositions("arm", best_joint_values);
//             }
//         }
//     }

//     if (found_exact) {
//         move_group.setJointValueTarget(best_joint_values);
//         RCLCPP_INFO(logger, "Sending exact joint target to planner.");
//     } else if (!best_joint_values.empty()) {
//         RCLCPP_WARN(logger, "Could not find exact IK. Best Cartesian error: %.4f m. Using best approximate.", best_error);
//         move_group.setJointValueTarget(best_joint_values);
//     } else {
//         RCLCPP_ERROR(logger, "IK failed completely! Pose is physically unreachable.");
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     //----------------------------------------------------
//     // Plan
//     //----------------------------------------------------

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto result = move_group.plan(plan);

//     if(result == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//         RCLCPP_INFO(logger,"Planning SUCCESS");
//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger,"Planning FAILED");
//     }

//     executor.cancel();
//     spinner.join();

//     rclcpp::shutdown();
//     return 0;
// }


// Now with collision checking through MoveIt PlanningSceneMonitor and native collision checking

// #include <memory>
// #include <thread>
// #include <cmath>
// #include <vector>

// #include <rclcpp/rclcpp.hpp>
// #include <moveit/move_group_interface/move_group_interface.hpp>
// #include <moveit/robot_state/robot_state.hpp>
// #include <moveit/planning_scene_monitor/planning_scene_monitor.hpp> 
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <mutex>
// #include <Eigen/Geometry> 

// geometry_msgs::msg::Pose latest_pose;
// bool pose_received = false;

// std::mutex pose_mutex;

// void poseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     std::lock_guard<std::mutex> lock(pose_mutex);
//     latest_pose = msg->pose;
//     pose_received = true;
// }

// int main(int argc, char *argv[])
// {
//     rclcpp::init(argc, argv);

//     auto node = std::make_shared<rclcpp::Node>(
//         "moveit_trial",
//         rclcpp::NodeOptions()
//             .automatically_declare_parameters_from_overrides(true)
//             .parameter_overrides({
//                 rclcpp::Parameter("use_sim_time", true)
//             }));

//     auto logger = rclcpp::get_logger("moveit_trial");

//     auto pose_sub =
//         node->create_subscription<geometry_msgs::msg::PoseStamped>(
//             "/cube_pose",
//             10,
//             poseCallback);

//     rclcpp::executors::SingleThreadedExecutor executor;
//     executor.add_node(node);

//     std::thread spinner([&executor]()
//     {
//         executor.spin();
//     });

//     using moveit::planning_interface::MoveGroupInterface;
//     MoveGroupInterface move_group(node, "arm");

//     move_group.startStateMonitor();
//     rclcpp::sleep_for(std::chrono::seconds(2));
//     move_group.setStartStateToCurrentState();

//     move_group.setPlanningTime(10.0);
//     move_group.setMaxVelocityScalingFactor(0.2);
//     move_group.setMaxAccelerationScalingFactor(0.2);

//     RCLCPP_INFO(logger, "Planning Frame : %s", move_group.getPlanningFrame().c_str());

//     auto current_pose = move_group.getCurrentPose("wrist_2");

//     geometry_msgs::msg::Pose target;

//     RCLCPP_INFO(logger, "Waiting for cube pose...");
//     while (rclcpp::ok() && !pose_received)
//     {
//         rclcpp::sleep_for(std::chrono::milliseconds(100));
//     }

//     {
//         std::lock_guard<std::mutex> lock(pose_mutex);
//         target = latest_pose; 
//     }

//     // Normalize the quaternion
//     double norm = std::sqrt(
//         target.orientation.x * target.orientation.x +
//         target.orientation.y * target.orientation.y +
//         target.orientation.z * target.orientation.z +
//         target.orientation.w * target.orientation.w);

//     if (norm > 1e-6) {
//         target.orientation.x /= norm;
//         target.orientation.y /= norm;
//         target.orientation.z /= norm;
//         target.orientation.w /= norm;
//     }

//     move_group.clearPoseTargets();
//     move_group.setStartStateToCurrentState();

//     // ==========================================
//     // SETUP NATIVE MOVEIT COLLISION CHECKING (Jazzy API)
//     // ==========================================
//     auto planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
//         node, "robot_description", "planning_scene_monitor");
    
//     if (planning_scene_monitor->getPlanningScene()) {
//         planning_scene_monitor->startSceneMonitor("/move_group/monitored_planning_scene");
//         planning_scene_monitor->requestPlanningSceneState();
//         rclcpp::sleep_for(std::chrono::seconds(1)); 
//     } else {
//         RCLCPP_ERROR(logger, "Failed to initialize PlanningSceneMonitor! Collision checking will not work.");
//     }

//     moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

//     // Shift the target from gripper to wrist_2
//     Eigen::Isometry3d wrist_2_pose = current_state->getGlobalLinkTransform("wrist_2");
//     Eigen::Isometry3d gripper_pose = current_state->getGlobalLinkTransform("gripper");
//     Eigen::Isometry3d wrist_2_to_gripper = wrist_2_pose.inverse() * gripper_pose;

//     Eigen::Isometry3d target_gripper_eigen = Eigen::Isometry3d::Identity();
//     target_gripper_eigen.translation() = Eigen::Vector3d(target.position.x, target.position.y, target.position.z);
//     Eigen::Quaterniond q_target(target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z);
//     q_target.normalize();
//     target_gripper_eigen.rotate(q_target);

//     Eigen::Isometry3d target_wrist_2_eigen = target_gripper_eigen * wrist_2_to_gripper.inverse();

//     geometry_msgs::msg::Pose target_wrist_2;
//     target_wrist_2.position.x = target_wrist_2_eigen.translation().x();
//     target_wrist_2.position.y = target_wrist_2_eigen.translation().y();
//     target_wrist_2.position.z = target_wrist_2_eigen.translation().z();
//     Eigen::Quaterniond q_wrist_2(target_wrist_2_eigen.rotation());
//     target_wrist_2.orientation.x = q_wrist_2.x();
//     target_wrist_2.orientation.y = q_wrist_2.y();
//     target_wrist_2.orientation.z = q_wrist_2.z();
//     target_wrist_2.orientation.w = q_wrist_2.w();

//     // ==========================================
//     // NATIVE IK WITH COLLISION CHECKING
//     // ==========================================
//     moveit::core::RobotState target_state(*current_state);
//     const moveit::core::JointModelGroup* jmg = target_state.getJointModelGroup("arm");

//     moveit::core::GroupStateValidityCallbackFn constraint = 
//         [&planning_scene_monitor](moveit::core::RobotState* state, 
//                                   const moveit::core::JointModelGroup* group, 
//                                   const double* joint_values) 
//         {
//             state->setJointGroupPositions(group, joint_values);
//             planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor);
//             return !ls->isStateColliding(*state, group->getName());
//         };

//     kinematics::KinematicsQueryOptions options;
//     options.return_approximate_solution = true;

//     RCLCPP_INFO(logger, "Computing IK with Native MoveIt Collision Checking...");

//     bool found_ik = target_state.setFromIK(jmg, target_wrist_2, "wrist_2", 0.1, constraint, options);

//     if (found_ik) {
//         std::vector<double> joint_values;
//         target_state.copyJointGroupPositions("arm", joint_values);
//         move_group.setJointValueTarget(joint_values);
//         RCLCPP_INFO(logger, "IK Success! Found a collision-free joint target.");
//     } else {
//         RCLCPP_ERROR(logger, "IK failed! No collision-free solution exists for this pose.");
//         executor.cancel();
//         spinner.join();
//         rclcpp::shutdown();
//         return 1;
//     }

//     //----------------------------------------------------
//     // Plan
//     //----------------------------------------------------

//     moveit::planning_interface::MoveGroupInterface::Plan plan;
//     auto result = move_group.plan(plan);

//     if(result == moveit::core::MoveItErrorCode::SUCCESS)
//     {
//         RCLCPP_INFO(logger,"Planning SUCCESS");
//         move_group.execute(plan);
//     }
//     else
//     {
//         RCLCPP_ERROR(logger,"Planning FAILED");
//     }

//     executor.cancel();
//     spinner.join();

//     rclcpp::shutdown();
//     return 0;
// }


// FINALLY KINDA PROPERLY WORKING WITH EXACT IK FOR 5-DOF ARM USING BRUTE-FORCE RANDOM SEEDS, GRIPPER OFFSET, AND COLLISION CHECKING


#include <memory>
#include <thread>
#include <cmath>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <Eigen/Geometry> 

geometry_msgs::msg::Pose latest_pose;
bool pose_received = false;

std::mutex pose_mutex;

void poseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    latest_pose = msg->pose;
    pose_received = true;
}

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

    auto pose_sub =
        node->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/cube_pose",
            10,
            poseCallback);

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

    RCLCPP_INFO(logger, "Planning Frame : %s", move_group.getPlanningFrame().c_str());

    // Add Virtual Floor so move_group.plan() avoids the ground natively
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject floor;
    floor.id = "floor";
    floor.header.frame_id = move_group.getPlanningFrame();
    floor.primitives.resize(1);
    floor.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    floor.primitives[0].dimensions.resize(3);
    floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 2.0;
    floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0;
    floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.1;
    floor.pose.position.x = 0.0;
    floor.pose.position.y = 0.0;
    floor.pose.position.z = -0.05; 
    floor.pose.orientation.w = 1.0;
    floor.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(floor);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    geometry_msgs::msg::Pose target;

    RCLCPP_INFO(logger, "Waiting for cube pose...");
    while (rclcpp::ok() && !pose_received)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    {
        std::lock_guard<std::mutex> lock(pose_mutex);
        target = latest_pose; 
    }

    move_group.clearPoseTargets();
    move_group.setStartStateToCurrentState();

    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();

    // ==========================================
    // 5-DOF KINEMATIC FIX: TURN BASE TO FACE TARGET
    // ==========================================
    Eigen::Isometry3d sp_pose = current_state->getGlobalLinkTransform("shoulder_pan");
    Eigen::Isometry3d wrist_2_pose = current_state->getGlobalLinkTransform("wrist_2");
    Eigen::Isometry3d gripper_pose = current_state->getGlobalLinkTransform("gripper");

    // 1. Calculate the world Yaw angle needed to face the target X, Y
    double dx = target.position.x - sp_pose.translation().x();
    double dy = target.position.y - sp_pose.translation().y();
    double target_world_yaw = std::atan2(dy, dx);

    // 2. Get current base Yaw from rotation matrix
    Eigen::Matrix3d m = sp_pose.rotation();
    double current_base_yaw = std::atan2(m(1, 0), m(0, 0));
    double delta_yaw = target_world_yaw - current_base_yaw;

    // 3. Calculate new target orientation for wrist_2
    // We rotate the CURRENT wrist_2 orientation around the vertical Z axis by delta_yaw.
    // This tells the arm: "Turn the base to face the cube, but keep the elbow/wrist bends exactly as they are."
    Eigen::AngleAxisd rot_z(delta_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond target_q(rot_z * wrist_2_pose.rotation());
    target_q.normalize();

    // 4. Calculate target position for wrist_2 (Cube position + gripper offset)
    Eigen::Vector3d offset = wrist_2_pose.translation() - gripper_pose.translation();
    Eigen::Vector3d target_pos = Eigen::Vector3d(target.position.x, target.position.y, target.position.z) + offset;

    geometry_msgs::msg::Pose target_wrist_2;
    target_wrist_2.position.x = target_pos.x();
    target_wrist_2.position.y = target_pos.y();
    target_wrist_2.position.z = target_pos.z();
    target_wrist_2.orientation.x = target_q.x();
    target_wrist_2.orientation.y = target_q.y();
    target_wrist_2.orientation.z = target_q.z();
    target_wrist_2.orientation.w = target_q.w();

    // ==========================================
    // NATIVE IK WITH ALIGNED SEED
    // ==========================================
    moveit::core::RobotState target_state(*current_state);
    const moveit::core::JointModelGroup* jmg = target_state.getJointModelGroup("arm");

    // Manually set the base joint in the seed state so KDL doesn't have to guess it
    std::vector<double> seed_joints;
    current_state->copyJointGroupPositions("arm", seed_joints);
    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "base_link_shoulder_pan_joint") {
            double joint_1_target = target_world_yaw - 1.5708; // -1.5708 because base_link is rotated 90 deg in URDF
            // Normalize to -PI to PI
            while (joint_1_target > M_PI) joint_1_target -= 2 * M_PI;
            while (joint_1_target < -M_PI) joint_1_target += 2 * M_PI;
            seed_joints[i] = joint_1_target;
            break;
        }
    }
    target_state.setJointGroupPositions("arm", seed_joints);

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = true;
    moveit::core::GroupStateValidityCallbackFn constraint; 

    RCLCPP_INFO(logger, "Computing IK (Base aligned to target, keeping current wrist bends)...");

    // Single IK attempt because the seed is now perfect
    bool ik_success = target_state.setFromIK(jmg, target_wrist_2, "wrist_2", 0.1, constraint, options);

    if (ik_success) {
        std::vector<double> joint_values;
        target_state.copyJointGroupPositions("arm", joint_values);
        move_group.setJointValueTarget(joint_values);
        RCLCPP_INFO(logger, "IK Success! Sending joint target to native MoveIt planner.");
    } else {
        RCLCPP_ERROR(logger, "IK failed! Pose is physically unreachable.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    //----------------------------------------------------
    // NATIVE MOVEIT PLAN
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