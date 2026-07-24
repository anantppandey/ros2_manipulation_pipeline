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

#include <moveit_msgs/msg/attached_collision_object.hpp>

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
#include <moveit/task_constructor/stages/modify_planning_scene.h>


#include <std_srvs/srv/trigger.hpp>
#include <std_srvs/srv/empty.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <random>

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

// ==========================================
// 5-DOF IK COMPUTATION FUNCTION
// ==========================================
/**
 * @brief Compute IK joint values for a 5-DOF arm to reach a target gripper (x,y,z) pose.
 * 
 * @param current_state  Pointer to the current RobotState (used for FK + IK seed)
 * @param logger         ROS logger for info/error messages
 * @param x              Target Gripper X position (meters, world frame)
 * @param y              Target Gripper Y position (meters, world frame)
 * @param z              Target Gripper Z position (meters, world frame)
 * @param joint_values   [out] Resulting joint values for the "arm" group on success
 * 
 * @return true  if IK succeeded (joint_values filled)
 * @return false if IK failed
 */
bool computeArmIKToTarget(moveit::core::RobotState* current_state,
                          const planning_scene::PlanningSceneConstPtr& scene,
                          rclcpp::Logger logger,
                          double x, double y, double z,
                          std::vector<double>& joint_values)
{
    Eigen::Isometry3d sp_pose      = current_state->getGlobalLinkTransform("shoulder_pan");
    Eigen::Isometry3d wrist_2_pose = current_state->getGlobalLinkTransform("wrist_2");
    Eigen::Isometry3d gripper_pose = current_state->getGlobalLinkTransform("gripper");

    Eigen::Isometry3d w2_to_grip_tf  = wrist_2_pose.inverse() * gripper_pose;
    Eigen::Vector3d   w2_to_grip_vec = w2_to_grip_tf.translation();

    const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = jmg->getVariableNames();

    // Still useful as a warm-start guess for joint_1 (speeds up convergence),
    // even though we no longer force the final orientation to match this yaw.
    double dx = x - sp_pose.translation().x();
    double dy = y - sp_pose.translation().y();
    double target_world_yaw = std::atan2(dy, dx);

    std::vector<double> seed_joints;
    current_state->copyJointGroupPositions("arm", seed_joints);
    for (size_t i = 0; i < joint_names.size(); ++i) {
        if (joint_names[i] == "base_link_shoulder_pan_joint") {
            double joint_1_target = target_world_yaw - 1.5708;
            while (joint_1_target >  M_PI) joint_1_target -= 2 * M_PI;
            while (joint_1_target < -M_PI) joint_1_target += 2 * M_PI;
            seed_joints[i] = joint_1_target;
            break;
        }
    }

    moveit::core::GroupStateValidityCallbackFn constraint =
        [&scene](moveit::core::RobotState* state,
                 const moveit::core::JointModelGroup* jmg2,
                 const double* joint_group_variable_values) -> bool
    {
        state->setJointGroupPositions(jmg2, joint_group_variable_values);
        state->update();
        collision_detection::CollisionRequest req;
        req.group_name = jmg2->getName();
        collision_detection::CollisionResult res;
        scene->checkCollision(req, res, *state, scene->getAllowedCollisionMatrix());
        return !res.collision;
    };

    kinematics::KinematicsQueryOptions options;
    options.return_approximate_solution = true;

    const double max_pos_error = 0.015;
    const double per_attempt_timeout = 0.3;
    const int max_attempts = 400;
    const double max_perturb_deg = 150.0;  // orientation is free, so allow a wide spread

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> gauss(0.0, 1.0);
    std::uniform_real_distribution<double> angle_dist(0.0, max_perturb_deg * M_PI / 180.0);

    RCLCPP_INFO(logger, "Computing position-priority IK for (%.3f, %.3f, %.3f)...", x, y, z);

    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        // Attempt 0: try the arm's natural current orientation, unperturbed.
        // After that: perturb around it by a random axis/angle — orientation
        // truly doesn't matter here, only reaching (x, y, z) does.
        double angle = (attempt == 0) ? 0.0 : angle_dist(gen);
        Eigen::Vector3d axis(gauss(gen), gauss(gen), gauss(gen));
        axis.normalize();
        Eigen::AngleAxisd perturb(angle, axis);
        Eigen::Quaterniond target_q(perturb * wrist_2_pose.rotation());
        target_q.normalize();

        Eigen::Vector3d target_gripper_pos(x, y, z);
        Eigen::Vector3d target_pos = target_gripper_pos - (target_q * w2_to_grip_vec);

        geometry_msgs::msg::Pose target_wrist_2;
        target_wrist_2.position.x = target_pos.x();
        target_wrist_2.position.y = target_pos.y();
        target_wrist_2.position.z = target_pos.z();
        target_wrist_2.orientation.x = target_q.x();
        target_wrist_2.orientation.y = target_q.y();
        target_wrist_2.orientation.z = target_q.z();
        target_wrist_2.orientation.w = target_q.w();

        moveit::core::RobotState attempt_state(*current_state);
        if (attempt == 0) {
            attempt_state.setJointGroupPositions("arm", seed_joints);
        } else {
            attempt_state.setToRandomPositions(jmg);
        }

        if (attempt_state.setFromIK(jmg, target_wrist_2, "wrist_2",
                                     per_attempt_timeout, constraint, options)) {
            Eigen::Isometry3d achieved = attempt_state.getGlobalLinkTransform("wrist_2");
            double pos_error = (achieved.translation() -
                                 Eigen::Vector3d(target_wrist_2.position.x,
                                                 target_wrist_2.position.y,
                                                 target_wrist_2.position.z)).norm();
            if (pos_error > max_pos_error) continue;

            attempt_state.copyJointGroupPositions("arm", joint_values);
            RCLCPP_INFO(logger, "IK found for (%.3f, %.3f, %.3f) on attempt %d, error %.1fmm",
                        x, y, z, attempt + 1, pos_error * 1000.0);
            return true;
        }
    }

    // None of the attempts succeeded — find out exactly why, instead of guessing again.
    {
        Eigen::Quaterniond nominal_q(wrist_2_pose.rotation());
        Eigen::Vector3d target_gripper_pos(x, y, z);
        Eigen::Vector3d target_pos = target_gripper_pos - (nominal_q * w2_to_grip_vec);

        geometry_msgs::msg::Pose debug_pose;
        debug_pose.position.x = target_pos.x();
        debug_pose.position.y = target_pos.y();
        debug_pose.position.z = target_pos.z();
        debug_pose.orientation.x = nominal_q.x();
        debug_pose.orientation.y = nominal_q.y();
        debug_pose.orientation.z = nominal_q.z();
        debug_pose.orientation.w = nominal_q.w();

        moveit::core::RobotState debug_state(*current_state);
        debug_state.setJointGroupPositions("arm", seed_joints);
        kinematics::KinematicsQueryOptions debug_options;
        debug_options.return_approximate_solution = true;

        if (debug_state.setFromIK(jmg, debug_pose, "wrist_2", 0.2,
                                  moveit::core::GroupStateValidityCallbackFn(), debug_options)) {
            collision_detection::CollisionRequest debug_req;
            debug_req.contacts = true;
            debug_req.max_contacts = 20;
            collision_detection::CollisionResult debug_res;
            scene->checkCollision(debug_req, debug_res, debug_state);
            if (debug_res.collision) {
                RCLCPP_ERROR(logger, "Pose (%.3f, %.3f, %.3f): blocked by collision. Contacts:", x, y, z);
                for (const auto& c : debug_res.contacts) {
                    RCLCPP_ERROR(logger, "  %s <-> %s", c.first.first.c_str(), c.first.second.c_str());
                }
            } else {
                RCLCPP_ERROR(logger, "Pose (%.3f, %.3f, %.3f): reachable and collision-free at nominal orientation — the random search just never landed under the position-error threshold. Try raising max_attempts or max_pos_error.", x, y, z);
            }
        } else {
            RCLCPP_ERROR(logger, "Pose (%.3f, %.3f, %.3f) can't be reached kinematically even at the nominal orientation, ignoring collision entirely.", x, y, z);
        }
    }

    RCLCPP_ERROR(logger, "No collision-free, accurate IK found for (%.3f, %.3f, %.3f) after %d attempts.",
                 x, y, z, max_attempts);
    return false;
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

    // Create service clients for Gazebo attach/detach
    auto attach_client = node->create_client<std_srvs::srv::Trigger>("/gripper/attach");
    auto detach_client = node->create_client<std_srvs::srv::Trigger>("/gripper/detach");

    // Wait for services
    RCLCPP_INFO(logger, "Waiting for Gazebo attach/detach services...");
    while (!attach_client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(logger, "Gazebo attach service not available, waiting...");
    }
    while (!detach_client->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
        RCLCPP_INFO(logger, "Gazebo detach service not available, waiting...");
    }
    RCLCPP_INFO(logger, "Gazebo services ready!");

    std::thread spinner([&executor]() {
        executor.spin();
    });

    // ==========================================
    // TWEAK THESE VALUES FOR TRIAL AND ERROR
    // ==========================================
    double CUBE_X_OFFSET = -0.005;     
    double CUBE_Y_OFFSET = 0.0;     
    double CUBE_Z_OFFSET = -0.055;     

    double GRIPPER_X_OFFSET = -0.031; 
    double GRIPPER_Y_OFFSET = -0.0275;
    double GRIPPER_Z_OFFSET = 0.0275;  

    double DESCEND_DISTANCE = 0.05; 
    double LIFT_DISTANCE = 0.10;    

    double PLACE_APPROACH_CLEARANCE = 0.095;  // TUNE: height above lift_z needed to clear the wall at the place x/y — verify in RViz first
    double PLACE_DESCEND_DISTANCE = 0.08;    // TUNE: how far to lower onto the place surface once clear of the wall

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
    
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(logger, "Waiting for robot state...");
    monitor->waitForCompleteState("arm", 5.0);
    monitor->waitForCompleteState("gripper", 5.0);
    moveit::core::RobotStatePtr current_state = monitor->getCurrentState();
    
    if (!current_state) {
        RCLCPP_ERROR(logger, "Failed to get current robot state");
        rclcpp::shutdown();
        return 1;
    }


    // NEW: this monitor is what actually syncs world geometry (incl. octomap) from move_group
    auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node, "robot_description");
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();   // <-- this is the one that carries octomap updates
    psm->startStateMonitor();

    // Force-clear any stale occupied voxels left from earlier testing (e.g. before the
    // padding was widened). Octomap cells only clear when re-observed as free by a sensor
    // ray, so old bad voxels can persist across mtc_node restarts even if move_group didn't restart.
    auto clear_octomap_client = node->create_client<std_srvs::srv::Empty>("/clear_octomap");
    RCLCPP_INFO(logger, "Waiting for /clear_octomap service...");
    if (clear_octomap_client->wait_for_service(std::chrono::seconds(3))) {
        auto clear_req = std::make_shared<std_srvs::srv::Empty::Request>();
        auto clear_future = clear_octomap_client->async_send_request(clear_req);
        if (clear_future.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
            RCLCPP_INFO(logger, "Octomap cleared.");
        } else {
            RCLCPP_WARN(logger, "/clear_octomap call timed out, continuing anyway.");
        }
    } else {
        RCLCPP_WARN(logger, "/clear_octomap service not available, skipping clear.");
    }

    // Let the sensors rebuild the octomap cleanly (with current padding) before syncing.
    rclcpp::sleep_for(std::chrono::seconds(3));

    psm->requestPlanningSceneState("/get_planning_scene"); // blocking one-shot sync, don't start empty


    Eigen::Isometry3d debug_sp = current_state->getGlobalLinkTransform("shoulder_pan");
    RCLCPP_INFO(logger, "Shoulder Pan is at: x=%f, y=%f, z=%f", 
        debug_sp.translation().x(), debug_sp.translation().y(), debug_sp.translation().z());

    // CAPTURE THE EXACT STARTING JOINTS HERE
    std::vector<double> initial_joint_values;
    current_state->copyJointGroupPositions("arm", initial_joint_values);

    // ==========================================
    // 3. ADD FLOOR
    // ==========================================
    moveit::planning_interface::PlanningSceneInterface psi;
    moveit_msgs::msg::CollisionObject floor;
    // floor.id = "floor";
    // floor.header.frame_id = robot_model->getModelFrame();
    // floor.primitives.resize(1);
    // floor.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    // floor.primitives[0].dimensions.resize(3);
    // floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 2.0;
    // floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 2.0;
    // floor.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.1;
    // floor.pose.position.x = 0.0;
    // floor.pose.position.y = 0.0;
    // floor.pose.position.z = -0.05; 
    // floor.pose.orientation.w = 1.0;
    // floor.operation = moveit_msgs::msg::CollisionObject::ADD;
    // psi.applyCollisionObject(floor);

    // ==========================================
    // 4. WAIT FOR CUBE POSE
    // ==========================================
    geometry_msgs::msg::Pose raw_pose;
    RCLCPP_INFO(logger, "Waiting for cube pose...");
    while (rclcpp::ok() && !pose_received)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    {
        std::lock_guard<std::mutex> lock(pose_mutex);
        raw_pose = latest_pose; 
    }

    RCLCPP_INFO(logger, "Received Cube Pose -> x: %f, y: %f, z: %f", 
        raw_pose.position.x, raw_pose.position.y, raw_pose.position.z);

    // ==========================================
    // 4b. ADD CUBE TO PLANNING SCENE (RViz)
    // ==========================================
    geometry_msgs::msg::Pose cube_pose = raw_pose;
    cube_pose.position.x += CUBE_X_OFFSET;
    cube_pose.position.y += CUBE_Y_OFFSET;
    cube_pose.position.z += CUBE_Z_OFFSET;

    moveit_msgs::msg::CollisionObject cube;
    cube.id = "red_cube";
    cube.header.frame_id = robot_model->getModelFrame();
    cube.primitives.resize(1);
    cube.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    cube.primitives[0].dimensions.resize(3);
    cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.03;
    cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.03;
    cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.03;
    cube.pose = cube_pose; 
    cube.operation = moveit_msgs::msg::CollisionObject::ADD;
    psi.applyCollisionObject(cube);

    // Build once, right after psi.applyCollisionObject(cube) — reuse everywhere below

    std::vector<std::string> touch_links;
    auto arm_links = robot_model->getJointModelGroup("arm")->getLinkModelNames();
    auto gripper_links = robot_model->getJointModelGroup("gripper")->getLinkModelNames();
    touch_links.insert(touch_links.end(), arm_links.begin(), arm_links.end());
    touch_links.insert(touch_links.end(), gripper_links.begin(), gripper_links.end());

    planning_scene::PlanningScenePtr ik_scene;
    {
        planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
        ik_scene = locked_scene->diff();
    }
    ik_scene->processCollisionObjectMsg(cube);  

    planning_scene::PlanningScenePtr grasp_scene = ik_scene->diff();
    grasp_scene->getAllowedCollisionMatrixNonConst().setEntry("red_cube", touch_links, true);
    grasp_scene->getAllowedCollisionMatrixNonConst().setEntry(
        planning_scene::PlanningScene::OCTOMAP_NS, touch_links, true);

    planning_scene::PlanningScenePtr transit_scene = ik_scene->diff();
    moveit_msgs::msg::AttachedCollisionObject attached_cube_msg;
    attached_cube_msg.link_name = "gripper";
    attached_cube_msg.object = cube;
    attached_cube_msg.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    attached_cube_msg.touch_links = touch_links;
    transit_scene->processAttachedCollisionObjectMsg(attached_cube_msg);


    // ==========================================
    // APPLY OFFSET TO CUBE POSE FOR GRIPPER IK
    // ==========================================
    // Target represents where we want the GRIPPER to be
    geometry_msgs::msg::Pose target = raw_pose;
    target.position.z += GRIPPER_Z_OFFSET; 
    target.position.x += GRIPPER_X_OFFSET;
    target.position.y += GRIPPER_Y_OFFSET; 

    // ==========================================
    // 5-DOF IK CALCULATIONS
    // ==========================================
    std::vector<double> joint_values;

    auto world_obj = ik_scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
    if (world_obj && !world_obj->shapes_.empty()) {
        auto octree_shape = std::dynamic_pointer_cast<const shapes::OcTree>(world_obj->shapes_[0]);
        if (octree_shape && octree_shape->octree) {
            auto node = octree_shape->octree->search(target.position.x, target.position.y, target.position.z);
            bool occupied = node && octree_shape->octree->isNodeOccupied(node);
            RCLCPP_INFO(logger, "Octomap synced OK. Target point (%.3f,%.3f,%.3f): %s",
                        target.position.x, target.position.y, target.position.z,
                        occupied ? "OCCUPIED (padded wall likely reaches here)" : "free");
        }
    } else {
        RCLCPP_WARN(logger, "ik_scene has NO octomap object — requestPlanningSceneState didn't pull it in.");
    }

    if (!computeArmIKToTarget(current_state.get(), ik_scene, logger, 
                              target.position.x, target.position.y, target.position.z, 
                              joint_values)) {
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Compute Grasp Pose IK (Descend from target)
    double grasp_x = target.position.x;
    double grasp_y = target.position.y;
    double grasp_z = target.position.z - DESCEND_DISTANCE;
    
    std::vector<double> grasp_joint_values;
    if (!computeArmIKToTarget(current_state.get(), grasp_scene, logger, 
                              grasp_x, grasp_y, grasp_z, 
                              grasp_joint_values)) {
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Compute Lift Pose IK (Lift from grasp)
    double lift_x = grasp_x;
    double lift_y = grasp_y;
    double lift_z = grasp_z + LIFT_DISTANCE;

    std::vector<double> lift_joint_values;
    if (!computeArmIKToTarget(current_state.get(), transit_scene, logger, 
                              lift_x, lift_y, lift_z, 
                              lift_joint_values)) {
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Compute Place APPROACH Pose IK (-0.15, 0.3, high enough to clear the wall)
    double place_x = -0.15;
    double place_y = 0.3-0.025;
    double place_approach_z = lift_z + PLACE_APPROACH_CLEARANCE;

    std::vector<double> place_approach_joint_values;
    if (!computeArmIKToTarget(current_state.get(), transit_scene, logger, 
                              place_x, place_y, place_approach_z, 
                              place_approach_joint_values)) {
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Compute final Place Pose IK (lower onto the surface once clear of the wall)
    double place_z = place_approach_z - PLACE_DESCEND_DISTANCE;

    std::vector<double> place_joint_values;
    if (!computeArmIKToTarget(current_state.get(), transit_scene, logger, 
                              place_x, place_y, place_z, 
                              place_joint_values)) {
        executor.cancel(); spinner.join(); rclcpp::shutdown(); return 1;
    }

    // Fetch joint names for MTC setup
    const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = jmg->getVariableNames();

    // ==========================================
    // MOVEIT TASK CONSTRUCTOR SETUP
    // ==========================================
    
    auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node);
    pipeline->setProperty("max_velocity_scaling_factor", 0.2);
    pipeline->setProperty("max_acceleration_scaling_factor", 0.2);

    auto gripper_pipeline = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    gripper_pipeline->setMaxVelocityScalingFactor(0.1);
    gripper_pipeline->setMaxAccelerationScalingFactor(0.1);

    // Stage 1: Fixed Current State — reuse the same synced snapshot the IK used
    planning_scene::PlanningScenePtr scene = ik_scene;

    RCLCPP_INFO(logger, "Scene world objects: %zu | has octomap: %s",
        scene->getWorld()->size(),
        scene->getWorld()->hasObject(planning_scene::PlanningScene::OCTOMAP_NS) ? "yes" : "no");

    moveit::core::RobotState& scene_state = scene->getCurrentStateNonConst();
    scene_state = *current_state;
    
    double safe_zero = 0.0;
    scene_state.setJointPositions("wrist_2_gripper_joint", &safe_zero);
    // cube already applied to ik_scene above — don't re-add it here

    scene->processCollisionObjectMsg(cube);

    auto current_stage = std::make_unique<mtc::stages::FixedState>("current state");
    current_stage->setState(scene);
    task.add(std::move(current_stage));

    // Stage 1.5: Allow Collision with Cube (EARLY!)
    auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper collision");
    allow_coll->allowCollisions("red_cube", touch_links, true);
    task.add(std::move(allow_coll));

    // Stage 4: Rotate Wrist 90 Degrees
    auto rotate_wrist = std::make_unique<mtc::stages::MoveRelative>("rotate wrist", pipeline);
    rotate_wrist->setGroup("arm");
    std::map<std::string, double> joint_deltas;
    joint_deltas["wrist_1_wrist_2_joint"] = M_PI / 2.0; 
    rotate_wrist->setDirection(joint_deltas); 
    rotate_wrist->setProperty("timeout", 10.0);
    task.add(std::move(rotate_wrist));

    // Stage 3: Move Arm to Target Joints
    auto move_to = std::make_unique<mtc::stages::MoveTo>("move to target", pipeline);
    move_to->setGroup("arm");
    std::map<std::string, double> joint_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        joint_targets[joint_names[i]] = joint_values[i];
        if (joint_names[i] == "wrist_1_wrist_2_joint") {
            joint_targets[joint_names[i]] += M_PI / 2.0;
        }
    }
    move_to->setGoal(joint_targets); 
    move_to->setProperty("timeout", 10.0);
    task.add(std::move(move_to));

    // Stage 2: Open Gripper
    auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", gripper_pipeline);
    open_gripper->setGroup("gripper");
    std::map<std::string, double> gripper_open;
    gripper_open["wrist_2_gripper_joint"] = 1.0; 
    open_gripper->setGoal(gripper_open);
    open_gripper->setProperty("timeout", 5.0);
    task.add(std::move(open_gripper));

    // Stage 4.5: Allow collision with octomap voxels local to the grasp (cube self-perception)
    auto allow_octomap_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-octomap collision");
    std::vector<std::string> octomap_allow_list = touch_links;
    octomap_allow_list.push_back("red_cube");  // camera sees the cube too — same voxels as the object
    allow_octomap_coll->allowCollisions(planning_scene::PlanningScene::OCTOMAP_NS, octomap_allow_list, true);
    task.add(std::move(allow_octomap_coll));


    // Stage 5: Descend to Cube (Joint Space)
    auto descend = std::make_unique<mtc::stages::MoveTo>("descend to cube", pipeline);
    descend->setGroup("arm");
    std::map<std::string, double> grasp_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        grasp_targets[joint_names[i]] = grasp_joint_values[i];
        if (joint_names[i] == "wrist_1_wrist_2_joint") {
            grasp_targets[joint_names[i]] += M_PI / 2.0;
        }
    }
    descend->setGoal(grasp_targets); 
    descend->setProperty("timeout", 10.0);
    task.add(std::move(descend));

    // Stage 6: Close Gripper
    auto close_gripper = std::make_unique<mtc::stages::MoveTo>("close gripper", gripper_pipeline);
    close_gripper->setGroup("gripper");
    std::map<std::string, double> gripper_close;
    gripper_close["wrist_2_gripper_joint"] = 0.475; 
    close_gripper->setGoal(gripper_close);
    close_gripper->setProperty("timeout", 5.0);
    task.add(std::move(close_gripper));

    // Stage 7: Attach Cube to Gripper
    auto attach_cube = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cube");
    attach_cube->attachObject("red_cube", "gripper"); 
    task.add(std::move(attach_cube));

    // Stage 8: Lift Up (Joint Space)
    auto lift = std::make_unique<mtc::stages::MoveTo>("lift up", pipeline);
    lift->setGroup("arm");
    std::map<std::string, double> lift_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        lift_targets[joint_names[i]] = lift_joint_values[i];
        if (joint_names[i] == "wrist_1_wrist_2_joint") {
            lift_targets[joint_names[i]] += M_PI / 2.0;
        }
    }
    lift->setGoal(lift_targets); 
    lift->setProperty("timeout", 10.0);
    task.add(std::move(lift));

    // Stage 8.5: Re-disallow octomap collision now that we're clear of the pickup zone
    auto disallow_octomap_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("disallow gripper-octomap collision");
    std::vector<std::string> octomap_disallow_list = touch_links;
    octomap_disallow_list.push_back("red_cube");
    disallow_octomap_coll->allowCollisions(planning_scene::PlanningScene::OCTOMAP_NS, octomap_disallow_list, false);
    task.add(std::move(disallow_octomap_coll));

    // ==========================================
    // Stage 9: Move to Place Location (approach height — clears the wall)
    // ==========================================
    auto move_to_place = std::make_unique<mtc::stages::MoveTo>("move to place", pipeline);
    move_to_place->setGroup("arm");
    std::map<std::string, double> place_approach_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        place_approach_targets[joint_names[i]] = place_approach_joint_values[i];
        if (joint_names[i] == "wrist_1_wrist_2_joint") {
            place_approach_targets[joint_names[i]] += M_PI / 2.0;
        }
    }
    move_to_place->setGoal(place_approach_targets); 
    move_to_place->setProperty("timeout", 10.0);
    task.add(std::move(move_to_place));

    // ==========================================
    // Stage 9.5: Descend to Place Surface (now clear of the wall)
    // ==========================================
    auto descend_to_place = std::make_unique<mtc::stages::MoveTo>("descend to place", pipeline);
    descend_to_place->setGroup("arm");
    std::map<std::string, double> place_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        place_targets[joint_names[i]] = place_joint_values[i];
        if (joint_names[i] == "wrist_1_wrist_2_joint") {
            place_targets[joint_names[i]] += M_PI / 2.0;
        }
    }
    descend_to_place->setGoal(place_targets); 
    descend_to_place->setProperty("timeout", 10.0);
    task.add(std::move(descend_to_place));

    // ==========================================
    // Stage 10: Open Gripper to Release
    // ==========================================
    auto open_gripper_place = std::make_unique<mtc::stages::MoveTo>("open gripper place", gripper_pipeline);
    open_gripper_place->setGroup("gripper");
    std::map<std::string, double> gripper_open_place;
    gripper_open_place["wrist_2_gripper_joint"] = 1.0; 
    open_gripper_place->setGoal(gripper_open_place);
    open_gripper_place->setProperty("timeout", 5.0);
    task.add(std::move(open_gripper_place));

    // ==========================================
    // Stage 11: Detach Cube
    // ==========================================
    auto detach_cube = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cube");
    detach_cube->detachObject("red_cube", "gripper");
    task.add(std::move(detach_cube));

    // ==========================================
    // Stage 12: Return to Start Position
    // ==========================================
    // Move back to the exact joint positions the robot was in when the node started.
    auto return_to_start = std::make_unique<mtc::stages::MoveTo>("return to start", pipeline);
    return_to_start->setGroup("arm");
    
    std::map<std::string, double> start_targets;
    for (size_t i = 0; i < joint_names.size(); ++i) {
        start_targets[joint_names[i]] = initial_joint_values[i];
    }
    return_to_start->setGoal(start_targets); 
    return_to_start->setProperty("timeout", 10.0);
    task.add(std::move(return_to_start));

    // ==========================================
    // Stage 13: Close Gripper at End
    // ==========================================
    auto close_gripper_final = std::make_unique<mtc::stages::MoveTo>("close gripper final", gripper_pipeline);
    close_gripper_final->setGroup("gripper");
    std::map<std::string, double> gripper_close_final;
    gripper_close_final["wrist_2_gripper_joint"] = 0.0; // Close completely
    close_gripper_final->setGoal(gripper_close_final);
    close_gripper_final->setProperty("timeout", 5.0);
    task.add(std::move(close_gripper_final));

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
                    std::string stage_name = traj->creator()->name();

                    MoveGroupInterface::Plan plan;
                    RCLCPP_INFO(logger, "Executing '%s' trajectory...", stage_name.c_str());
                    
                    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                    totg.computeTimeStamps(*rt, 0.1, 0.1);

                    rt->getRobotTrajectoryMsg(plan.trajectory);
                    
                    if (plan.trajectory.joint_trajectory.points.empty()) {
                        continue;
                    }

                    if (group_name == "gripper") {
                        if (stage_name == "open gripper place") {
                            RCLCPP_INFO(logger, "Manually detaching cube from MoveIt planning scene...");
                            arm_group.detachObject("red_cube");
                            
                            // DETACH IN GAZEBO
                            RCLCPP_INFO(logger, "Detaching cube from Gazebo...");
                            auto detach_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto detach_future = detach_client->async_send_request(detach_req);
                            
                            auto detach_status = detach_future.wait_for(std::chrono::seconds(2));
                            if (detach_status == std::future_status::ready) {
                                auto detach_res = detach_future.get();
                                if (detach_res->success) {
                                    RCLCPP_INFO(logger, "Cube detached from Gazebo: %s", detach_res->message.c_str());
                                } else {
                                    RCLCPP_ERROR(logger, "Failed to detach in Gazebo: %s", detach_res->message.c_str());
                                }
                            } else {
                                RCLCPP_ERROR(logger, "Gazebo detach service timeout!");
                            }
                            rclcpp::sleep_for(std::chrono::milliseconds(100));
                        }
                        gripper_group.execute(plan);
                    } else {
                        if (stage_name == "lift up") {
                            RCLCPP_INFO(logger, "Manually attaching cube to gripper in MoveIt planning scene...");
                            bool attached = arm_group.attachObject("red_cube", "gripper", touch_links);
                            if (attached) {
                                RCLCPP_INFO(logger, "Cube attached successfully in MoveIt!");
                            } else {
                                RCLCPP_ERROR(logger, "Cube FAILED to attach in MoveIt!");
                            }
                            
                            // ATTACH IN GAZEBO
                            RCLCPP_INFO(logger, "Attaching cube in Gazebo...");
                            auto attach_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto attach_future = attach_client->async_send_request(attach_req);
                            
                            auto attach_status = attach_future.wait_for(std::chrono::seconds(2));
                            if (attach_status == std::future_status::ready) {
                                auto attach_res = attach_future.get();
                                if (attach_res->success) {
                                    RCLCPP_INFO(logger, "Cube attached in Gazebo: %s", attach_res->message.c_str());
                                } else {
                                    RCLCPP_ERROR(logger, "Failed to attach in Gazebo: %s", attach_res->message.c_str());
                                }
                            } else {
                                RCLCPP_ERROR(logger, "Gazebo attach service timeout!");
                            }
                            rclcpp::sleep_for(std::chrono::milliseconds(100));
                        }
                        arm_group.execute(plan);
                    }
                }
            }
        } else {
            auto traj = dynamic_cast<const mtc::SubTrajectory*>(sol.get());
            if (traj && traj->trajectory()) {
                auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(*traj->trajectory());
                std::string group_name = rt->getGroupName();
                std::string stage_name = traj->creator()->name();
                
                MoveGroupInterface::Plan plan;
                RCLCPP_INFO(logger, "Executing '%s' trajectory...", stage_name.c_str());
                
                trajectory_processing::TimeOptimalTrajectoryGeneration totg;
                totg.computeTimeStamps(*rt, 0.1, 0.1);
                
                rt->getRobotTrajectoryMsg(plan.trajectory);
                if (!plan.trajectory.joint_trajectory.points.empty()) {
                    if (group_name == "gripper") {
                        if (stage_name == "open gripper place") {
                            RCLCPP_INFO(logger, "Manually detaching cube from MoveIt planning scene...");
                            arm_group.detachObject("red_cube");
                            
                            // DETACH IN GAZEBO
                            RCLCPP_INFO(logger, "Detaching cube from Gazebo...");
                            auto detach_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto detach_future = detach_client->async_send_request(detach_req);
                            
                            auto detach_status = detach_future.wait_for(std::chrono::seconds(2));
                            if (detach_status == std::future_status::ready) {
                                auto detach_res = detach_future.get();
                                if (detach_res->success) {
                                    RCLCPP_INFO(logger, "Cube detached from Gazebo: %s", detach_res->message.c_str());
                                } else {
                                    RCLCPP_ERROR(logger, "Failed to detach in Gazebo: %s", detach_res->message.c_str());
                                }
                            } else {
                                RCLCPP_ERROR(logger, "Gazebo detach service timeout!");
                            }
                            rclcpp::sleep_for(std::chrono::milliseconds(100));
                        }
                        gripper_group.execute(plan);
                    } else {
                        if (stage_name == "lift up") {
                            RCLCPP_INFO(logger, "Manually attaching cube to gripper in MoveIt planning scene...");
                            bool attached = arm_group.attachObject("red_cube", "gripper", touch_links);
                            if (attached) {
                                RCLCPP_INFO(logger, "Cube attached successfully in MoveIt!");
                            } else {
                                RCLCPP_ERROR(logger, "Cube FAILED to attach in MoveIt!");
                            }
                            
                            // ATTACH IN GAZEBO
                            RCLCPP_INFO(logger, "Attaching cube in Gazebo...");
                            auto attach_req = std::make_shared<std_srvs::srv::Trigger::Request>();
                            auto attach_future = attach_client->async_send_request(attach_req);
                            
                            auto attach_status = attach_future.wait_for(std::chrono::seconds(2));
                            if (attach_status == std::future_status::ready) {
                                auto attach_res = attach_future.get();
                                if (attach_res->success) {
                                    RCLCPP_INFO(logger, "Cube attached in Gazebo: %s", attach_res->message.c_str());
                                } else {
                                    RCLCPP_ERROR(logger, "Failed to attach in Gazebo: %s", attach_res->message.c_str());
                                }
                            } else {
                                RCLCPP_ERROR(logger, "Gazebo attach service timeout!");
                            }
                            rclcpp::sleep_for(std::chrono::milliseconds(100));
                        }
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

    // ==========================================
    // DEBUG: Print final positions
    // ==========================================
    moveit::core::RobotStatePtr final_state = monitor->getCurrentState();
    if (final_state) {
        Eigen::Isometry3d final_w2 = final_state->getGlobalLinkTransform("wrist_2");
        Eigen::Isometry3d final_grip = final_state->getGlobalLinkTransform("gripper");
        RCLCPP_INFO(logger, "Final Wrist 2 Pose -> x: %f, y: %f, z: %f", 
            final_w2.translation().x(), final_w2.translation().y(), final_w2.translation().z());
        RCLCPP_INFO(logger, "Final Gripper Pose -> x: %f, y: %f, z: %f", 
            final_grip.translation().x(), final_grip.translation().y(), final_grip.translation().z());
    }

    executor.cancel();
    spinner.join();

    rclcpp::shutdown();
    return 0;
}