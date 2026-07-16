#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <Eigen/Geometry> 
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

// --- MTC Includes ---
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/container.h>
#include <moveit/task_constructor/stage.h>
#include <moveit/task_constructor/stages/current_state.h>
#include <moveit/task_constructor/stages/fixed_state.h>
#include <moveit/task_constructor/stages/move_to.h>
#include <moveit/task_constructor/stages/move_relative.h>
#include <moveit/task_constructor/solvers/pipeline_planner.h>
#include <moveit/task_constructor/solvers/joint_interpolation.h>
#include <moveit/utils/moveit_error_code.hpp>

namespace mtc = moveit::task_constructor;

geometry_msgs::msg::Pose latest_pose;
bool pose_received = false;
std::mutex pose_mutex;

void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(pose_mutex);
    latest_pose = msg->pose;
    pose_received = true;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node>(
        "mtc_move_to_cube",
        rclcpp::NodeOptions()
            .automatically_declare_parameters_from_overrides(true)
            .parameter_overrides({
                rclcpp::Parameter("use_sim_time", true)
            }));

    auto logger = rclcpp::get_logger("mtc_move_to_cube");

    auto pose_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/cube_pose", 10, poseCallback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);

    std::thread spinner([&executor]() {
        executor.spin();
    });

    // ==========================================
    // 1. INITIALIZE TASK AND ROBOT MODEL
    // ==========================================
    mtc::Task task("move_to_cube");
    task.loadRobotModel(node);
    auto robot_model = task.getRobotModel();

    if (!robot_model) {
        RCLCPP_ERROR(logger, "Failed to load robot model");
        rclcpp::shutdown();
        return 1;
    }

    // ==========================================
    // 2. GET CURRENT STATE DIRECTLY FROM TF/JOINT_STATES
    // ==========================================
    auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
    auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    auto monitor = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(
        node, robot_model, tf_buffer, true);
    
    RCLCPP_INFO(logger, "Waiting for robot state...");
    monitor->waitForCompleteState("arm", 5.0);
    monitor->waitForCompleteState("gripper", 5.0);
    moveit::core::RobotStatePtr current_state = monitor->getCurrentState();
    
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get current robot state");
        rclcpp::shutdown();
        return 1;
    }

    // ==========================================
    // 3. ADD FLOOR
    // ==========================================
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject floor;
    floor.id = "floor";
    floor.header.frame_id = robot_model->getModelFrame();
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

    // ==========================================
    // 4. WAIT FOR CUBE POSE
    // ==========================================
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

    // ==========================================
    // 5-DOF KINEMATIC FIX
    // ==========================================
    Eigen::Isometry3d sp_pose = current_state->getGlobalLinkTransform("shoulder_pan");
    Eigen::Isometry3d wrist_2_pose = current_state->getGlobalLinkTransform("wrist_2");
    Eigen::Isometry3d gripper_pose = current_state->getGlobalLinkTransform("gripper");

    double dx = target.position.x - sp_pose.translation().x();
    double dy = target.position.y - sp_pose.translation().y();
    double target_world_yaw = std::atan2(dy, dx);

    Eigen::Matrix3d m = sp_pose.rotation();
    double current_base_yaw = std::atan2(m(1, 0), m(0, 0));
    double delta_yaw = target_world_yaw - current_base_yaw;

    Eigen::AngleAxisd rot_z(delta_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond target_q(rot_z * wrist_2_pose.rotation());
    target_q.normalize();

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

    moveit::core::RobotState target_state(*current_state);
    const moveit::core::JointModelGroup* jmg = target_state.getJointModelGroup("arm");

    std::vector<double> seed_joints;
    current_state->copyJointGroupPositions("arm", seed_joints);
    const std::vector<std::string>& joint_names = jmg->getVariableNames();
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "base_link_shoulder_pan_joint") {
            double joint_1_target = target_world_yaw - 1.5708;
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

    RCLCPP_INFO(logger, "Computing IK...");
    bool ik_success = target_state.setFromIK(jmg, target_wrist_2, "wrist_2", 0.1, constraint, options);

    if (!ik_success) {
        RCLCPP_ERROR(logger, "IK failed! Pose is physically unreachable.");
        executor.cancel();
        spinner.join();
        rclcpp::shutdown();
        return 1;
    }

    std::vector<double> joint_values;
    target_state.copyJointGroupPositions("arm", joint_values);
    RCLCPP_INFO(logger, "IK Success! Setting up MTC Task...");

    // ==========================================
    // MOVEIT TASK CONSTRUCTOR SETUP
    // ==========================================
    
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline->setProperty("max_velocity_scaling_factor", 0.2);
    pipeline->setProperty("max_acceleration_scaling_factor", 0.2);

    auto gripper_pipeline = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    gripper_pipeline->setMaxVelocityScalingFactor(0.1);
    gripper_pipeline->setMaxAccelerationScalingFactor(0.1);

    // Stage 1: Fixed Current State (Clamped to 0.0 to prevent Gazebo float errors)
    auto scene = std::make_shared<planning_scene::PlanningScene>(robot_model);
    moveit::core::RobotState& scene_state = scene->getCurrentStateNonConst();
    scene_state = *current_state;
    
    double safe_zero = 0.0;
    scene_state.setJointPositions("wrist_2_gripper_joint", &safe_zero);

    auto current_stage = std::make_unique<mtc::stages::FixedState>("current state");
    current_stage->setState(scene);
    task.add(std::move(current_stage));

    // Stage 2: Open Gripper
    auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", gripper_pipeline);
    open_gripper->setGroup("gripper");
    std::map<std::string, double> gripper_open;
    gripper_open["wrist_2_gripper_joint"] = 1.0; 
    open_gripper->setGoal(gripper_open);
    open_gripper->setProperty("timeout", 5.0);
    task.add(std::move(open_gripper));

    // Stage 3: Move Arm to Target Joints
    auto move_to = std::make_unique<mtc::stages::MoveTo>("move to target", pipeline);
    move_to->setGroup("arm");
    
    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        joint_targets[joint_names[i]] = joint_values[i];
    }
    move_to->setGoal(joint_targets); 
    move_to->setProperty("timeout", 10.0);
    task.add(std::move(move_to));

    // Stage 4: Rotate Wrist 90 Degrees
    auto rotate_wrist = std::make_unique<mtc::stages::MoveRelative>("rotate wrist", pipeline);
    rotate_wrist->setGroup("arm");
    
    std::map<std::string, double> joint_deltas;
    joint_deltas["wrist_1_wrist_2_joint"] = M_PI / 2.0; 
    
    rotate_wrist->setDirection(joint_deltas); 
    rotate_wrist->setProperty("timeout", 10.0);
    task.add(std::move(rotate_wrist));

    // ==========================================
    // PLAN & EXECUTE
    // ==========================================
    RCLCPP_INFO(logger, "Planning with MTC...");
    auto result = task.plan(1);

    if (result == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(logger, "MTC Planning SUCCESS! Extracting trajectory...");

        using moveit::planning_interface::MoveGroupInterface;
        MoveGroupInterface arm_group(node, "arm");
        MoveGroupInterface gripper_group(node, "gripper");
        arm_group.startStateMonitor();
        gripper_group.startStateMonitor();

        auto sol = task.solutions().front();
        auto compound = dynamic_cast<const mtc::SolutionSequence*>(sol.get());
        
        if (compound) {
            for (const auto& sub : compound->solutions()) {
                auto traj = dynamic_cast<const mtc::SubTrajectory*>(sub);
                if (traj && traj->trajectory()) {
                    
                    auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(*traj->trajectory());
                    std::string group_name = rt->getGroupName();

                    MoveGroupInterface::Plan plan;
                    
                    if (group_name == "gripper") {
                        RCLCPP_INFO(logger, "Executing Gripper trajectory...");
                        // TOTG will respect the 0.5 rad/s limit we set in the URDF
                        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                        totg.computeTimeStamps(*rt, 0.1, 0.1);
                    } else {
                        RCLCPP_INFO(logger, "Executing Arm trajectory...");
                        trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                        totg.computeTimeStamps(*rt, 0.1, 0.1);
                    }

                    rt->getRobotTrajectoryMsg(plan.trajectory);
                    
                    if (plan.trajectory.joint_trajectory.points.empty()) {
                        RCLCPP_WARN(logger, "Skipping empty trajectory for %s", group_name.c_str());
                        continue;
                    }

                    if (group_name == "gripper") {
                        gripper_group.execute(plan);
                    } else {
                        arm_group.execute(plan);
                    }
                }
            }
        } else {
            auto traj = dynamic_cast<const mtc::SubTrajectory*>(sol.get());
            if (traj && traj->trajectory()) {
                auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(*traj->trajectory());
                std::string group_name = rt->getGroupName();
                
                MoveGroupInterface::Plan plan;
                if (group_name == "gripper") {
                    RCLCPP_INFO(logger, "Executing Gripper trajectory...");
                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(*rt, 0.1, 0.1);
                } else {
                    RCLCPP_INFO(logger, "Executing Arm trajectory...");
                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(*rt, 0.1, 0.1);
                }
                
                rt->getRobotTrajectoryMsg(plan.trajectory);
                if (!plan.trajectory.joint_trajectory.points.empty()) {
                    if (group_name == "gripper") {
                        gripper_group.execute(plan);
                    } else {
                        arm_group.execute(plan);
                    }
                }
            }
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "MTC Planning FAILED");
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}