#include <memory>
#include <thread>
#include <cmath>
#include <vector>
#include <map>
#include <string>
#include <mutex>
#include <atomic>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

#include <std_srvs/srv/empty.hpp>
#include <giraffe_gazebo_plugins/srv/attach_detach.hpp>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <random>

#include "giraffe_mtc/action/pick_place.hpp"

namespace mtc = moveit::task_constructor;

// ==========================================
// SHARED ORIENTATION HELPERS
// (unchanged from the original file - these don't touch node/goal state at
// all, they just take a seed state + scene + target and return joint values,
// so nothing about the action-server conversion affects them)
// ==========================================
double computeTargetWorldYaw(const Eigen::Vector3d& sp_translation, double x, double y)
{
    double dx = x - sp_translation.x();
    double dy = y - sp_translation.y();
    return std::atan2(dy, dx);
}

/**
 * @brief The "canonical" wrist orientation for facing a given (x,y) target: the ORIGINAL
 * wrist orientation (captured once at startup, before any motion), yawed to face the
 * target. This never depends on where the arm currently is — it's always anchored to the
 * same initial reference — so every fixed-orientation move for the same target column
 * (pre-grasp/descend/lift, or place-approach/descend) computes the exact same answer.
 */
Eigen::Quaterniond computeDesiredWristOrientation(const Eigen::Isometry3d& initial_sp_pose,
                                                   const Eigen::Isometry3d& initial_wrist2_pose,
                                                   double x, double y)
{
    double target_world_yaw = computeTargetWorldYaw(initial_sp_pose.translation(), x, y);
    Eigen::Matrix3d m = initial_sp_pose.rotation();
    double initial_base_yaw = std::atan2(m(1, 0), m(0, 0));
    double delta_yaw = target_world_yaw - initial_base_yaw;

    Eigen::AngleAxisd rot_z(delta_yaw, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond desired_q(rot_z * initial_wrist2_pose.rotation());
    desired_q.normalize();
    return desired_q;
}

void logIKFailureDiagnosis(moveit::core::RobotState* seed_state,
                            const planning_scene::PlanningSceneConstPtr& scene,
                            rclcpp::Logger logger,
                            const geometry_msgs::msg::Pose& nominal_wrist2_pose,
                            double x, double y, double z)
{
    const moveit::core::JointModelGroup* jmg = seed_state->getJointModelGroup("arm");
    moveit::core::RobotState debug_state(*seed_state);
    kinematics::KinematicsQueryOptions debug_options;
    debug_options.return_approximate_solution = true;

    if (debug_state.setFromIK(jmg, nominal_wrist2_pose, "wrist_2", 0.2,
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
            Eigen::Isometry3d achieved = debug_state.getGlobalLinkTransform("wrist_2");
            double pos_error = (achieved.translation() -
                                 Eigen::Vector3d(nominal_wrist2_pose.position.x,
                                                 nominal_wrist2_pose.position.y,
                                                 nominal_wrist2_pose.position.z)).norm();
            RCLCPP_ERROR(logger, "Pose (%.3f, %.3f, %.3f): collision-free, IK converges to %.1fmm error — above max_pos_error, not a collision or seeding issue.",
                         x, y, z, pos_error * 1000.0);
        }
    } else {
        RCLCPP_ERROR(logger, "Pose (%.3f, %.3f, %.3f) not kinematically reachable (checked ignoring collision).", x, y, z);
    }
}

// ==========================================
// FREE-ORIENTATION IK — for large Cartesian relocations (pre-grasp, place-approach).
// Wrist orientation is searched freely; whatever it lands on is returned via
// achieved_orientation for the caller (not currently consumed downstream, but kept
// for anyone who wants to inspect/log what orientation was actually used).
//
// IMPORTANT: seed_state must always be the PRISTINE current_state, never a state
// mutated by a previous IK call. Chaining IK results as seeds caused this solver's
// random full-joint search (setToRandomPositions touches ALL 5 joints, not just
// orientation) to leave joints 2-5 in an arbitrary configuration, which then made
// computeFixedOrientationIK's seed correction impossible to converge from — it was
// starting in the wrong basin entirely, no amount of jitter/attempts could fix that.
// ==========================================
bool computeFreeOrientationIK(moveit::core::RobotState* seed_state,
                               const planning_scene::PlanningSceneConstPtr& scene,
                               rclcpp::Logger logger,
                               double x, double y, double z,
                               std::vector<double>& joint_values,
                               Eigen::Quaterniond& achieved_orientation)
{
    Eigen::Isometry3d wrist_2_pose = seed_state->getGlobalLinkTransform("wrist_2");
    Eigen::Isometry3d gripper_pose = seed_state->getGlobalLinkTransform("gripper");
    Eigen::Vector3d w2_to_grip_vec = (wrist_2_pose.inverse() * gripper_pose).translation();

    const moveit::core::JointModelGroup* jmg = seed_state->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = jmg->getVariableNames();

    double target_world_yaw = computeTargetWorldYaw(
        seed_state->getGlobalLinkTransform("shoulder_pan").translation(), x, y);

    std::vector<double> seed_joints;
    seed_state->copyJointGroupPositions("arm", seed_joints);
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

    RCLCPP_INFO(logger, "Free-orientation IK for (%.3f, %.3f, %.3f)...", x, y, z);

    geometry_msgs::msg::Pose last_nominal_pose;

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

        Eigen::Vector3d target_pos = Eigen::Vector3d(x, y, z) - (target_q * w2_to_grip_vec);

        geometry_msgs::msg::Pose target_wrist_2;
        target_wrist_2.position.x = target_pos.x();
        target_wrist_2.position.y = target_pos.y();
        target_wrist_2.position.z = target_pos.z();
        target_wrist_2.orientation.x = target_q.x();
        target_wrist_2.orientation.y = target_q.y();
        target_wrist_2.orientation.z = target_q.z();
        target_wrist_2.orientation.w = target_q.w();
        if (attempt == 0) last_nominal_pose = target_wrist_2;

        moveit::core::RobotState attempt_state(*seed_state);
        if (attempt == 0) {
            attempt_state.setJointGroupPositions("arm", seed_joints);
        } else {
            attempt_state.setToRandomPositions(jmg);
        }

        if (attempt_state.setFromIK(jmg, target_wrist_2, "wrist_2",
                                     per_attempt_timeout, constraint, options)) {
            Eigen::Isometry3d achieved = attempt_state.getGlobalLinkTransform("wrist_2");
            double pos_error = (achieved.translation() - target_pos).norm();
            if (pos_error > max_pos_error) continue;

            attempt_state.copyJointGroupPositions("arm", joint_values);
            achieved_orientation = Eigen::Quaterniond(achieved.rotation());
            RCLCPP_INFO(logger, "Free IK found for (%.3f, %.3f, %.3f) on attempt %d, error %.1fmm",
                        x, y, z, attempt + 1, pos_error * 1000.0);
            return true;
        }
    }

    logIKFailureDiagnosis(seed_state, scene, logger, last_nominal_pose, x, y, z);
    RCLCPP_ERROR(logger, "Free-orientation IK failed for (%.3f, %.3f, %.3f) after %d attempts.",
                 x, y, z, max_attempts);
    return false;
}

// ==========================================
// FIXED-ORIENTATION IK — for short vertical motions (descend, lift). Orientation is
// held as close as possible to desired_orientation.
//
// IMPORTANT: seed_state must always be the PRISTINE current_state (same rule as
// above). This function internally re-derives the yaw-corrected joint_1 from
// seed_state — the same relationship computeDesiredWristOrientation was built
// around — so it always starts from the right basin regardless of which caller
// invokes it. It does NOT rely on inheriting a good joint_1 from a prior call.
//
// A 5-DOF arm generically cannot satisfy an arbitrary (position, EXACT orientation)
// pair — that's 6 constraints from 5 joints. At some (x,y) columns the desired
// orientation has a genuine structural residual (not a seeding problem — jittering
// the seed doesn't help, because the gap isn't due to a bad local minimum, it's
// because the constraint set is over-determined at that point). achieved_orientation
// reports back exactly what orientation was used: equal to desired_orientation in
// the normal case, or a small bounded deviation from it if the relaxed fallback
// tier below had to kick in. Callers doing multiple fixed-orientation moves along
// the same (x,y) column should feed each call's achieved_orientation into the next
// call's desired_orientation, so the whole column stays internally consistent
// instead of each stage separately fighting the same infeasibility.
// ==========================================
bool computeFixedOrientationIK(moveit::core::RobotState* seed_state,
                                const planning_scene::PlanningSceneConstPtr& scene,
                                rclcpp::Logger logger,
                                double x, double y, double z,
                                const Eigen::Quaterniond& desired_orientation,
                                std::vector<double>& joint_values,
                                Eigen::Quaterniond& achieved_orientation)
{
    Eigen::Isometry3d wrist_2_pose = seed_state->getGlobalLinkTransform("wrist_2");
    Eigen::Isometry3d gripper_pose = seed_state->getGlobalLinkTransform("gripper");
    Eigen::Vector3d w2_to_grip_vec = (wrist_2_pose.inverse() * gripper_pose).translation();

    const moveit::core::JointModelGroup* jmg = seed_state->getJointModelGroup("arm");
    const std::vector<std::string>& joint_names = jmg->getVariableNames();

    Eigen::Quaterniond target_q = desired_orientation.normalized();
    Eigen::Vector3d target_pos = Eigen::Vector3d(x, y, z) - (target_q * w2_to_grip_vec);

    geometry_msgs::msg::Pose target_wrist_2;
    target_wrist_2.position.x = target_pos.x();
    target_wrist_2.position.y = target_pos.y();
    target_wrist_2.position.z = target_pos.z();
    target_wrist_2.orientation.x = target_q.x();
    target_wrist_2.orientation.y = target_q.y();
    target_wrist_2.orientation.z = target_q.z();
    target_wrist_2.orientation.w = target_q.w();

    double target_world_yaw = computeTargetWorldYaw(
        seed_state->getGlobalLinkTransform("shoulder_pan").translation(), x, y);

    // Re-derive joint_1 from the PRISTINE seed_state's own shoulder_pan pose — this
    // is what makes this function safe to call with any (x,y), from any caller,
    // without depending on what a previous IK call left behind.
    moveit::core::RobotState corrected_seed(*seed_state);
    {
        std::vector<double> corrected_joints;
        corrected_seed.copyJointGroupPositions("arm", corrected_joints);
        for (size_t i = 0; i < joint_names.size(); ++i) {
            if (joint_names[i] == "base_link_shoulder_pan_joint") {
                double joint_1_target = target_world_yaw - 1.5708;
                while (joint_1_target >  M_PI) joint_1_target -= 2 * M_PI;
                while (joint_1_target < -M_PI) joint_1_target += 2 * M_PI;
                corrected_joints[i] = joint_1_target;
                break;
            }
        }
        corrected_seed.setJointGroupPositions("arm", corrected_joints);
        corrected_seed.update();
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

    // 15mm to match Free-Orientation IK's proven threshold — the numeric solver's
    // achievable accuracy is a few mm regardless of which IK call makes it. Cube is
    // 3cm and existing hand-tuned offsets already carry several mm of slack, so this
    // doesn't meaningfully change grasp reliability.
    const double max_pos_error = 0.015;
    const int per_tier_attempts = 10;
    const double per_attempt_timeout = 0.3;
    // Escalating jitter radii around the yaw-corrected seed. Stays a genuinely LOCAL
    // search the whole time — nothing like Free IK's 150° random restarts — just enough
    // to recover from small numerical local minima around the corrected seed.
    const double jitter_tiers[] = {0.0, 0.05, 0.15};

    RCLCPP_INFO(logger, "Fixed-orientation IK for (%.3f, %.3f, %.3f)...", x, y, z);

    int attempt_count = 0;
    for (double jitter : jitter_tiers) {
        // jitter=0.0 is a deterministic seed — one attempt tells you everything
        // repeating it would; only the jittered tiers benefit from multiple tries.
        int tier_attempts = (jitter == 0.0) ? 1 : per_tier_attempts;
        for (int i = 0; i < tier_attempts; ++i, ++attempt_count) {
            moveit::core::RobotState attempt_state(corrected_seed);
            if (jitter > 0.0) {
                attempt_state.setToRandomPositionsNearBy(jmg, corrected_seed, jitter);
            }

            if (attempt_state.setFromIK(jmg, target_wrist_2, "wrist_2",
                                         per_attempt_timeout, constraint, options)) {
                Eigen::Isometry3d achieved = attempt_state.getGlobalLinkTransform("wrist_2");
                double pos_error = (achieved.translation() - target_pos).norm();
                if (pos_error > max_pos_error) continue;

                attempt_state.copyJointGroupPositions("arm", joint_values);
                achieved_orientation = target_q;  // exact desired orientation was reachable here
                RCLCPP_INFO(logger, "Fixed IK found for (%.3f, %.3f, %.3f) on attempt %d (jitter=%.2f), error %.1fmm",
                            x, y, z, attempt_count + 1, jitter, pos_error * 1000.0);
                return true;
            }
        }
    }

    // Last resort: the exact desired orientation has a structural residual at this
    // (x,y) column for this 5-DOF arm — jittering the seed above couldn't fix it
    // because the problem isn't the seed, it's that 5 joints can't hit an arbitrary
    // (position, exact orientation) pair. Let orientation drift a SMALL bounded
    // amount (much narrower than Free-Orientation IK's 150°) to close the gap, and
    // report whatever orientation that ended up being.
    {
        const double relaxed_max_perturb_deg = 25.0;
        const int relaxed_attempts = 150;

        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> gauss(0.0, 1.0);
        std::uniform_real_distribution<double> angle_dist(0.0, relaxed_max_perturb_deg * M_PI / 180.0);

        RCLCPP_WARN(logger, "Fixed-orientation IK: exact orientation infeasible within %.1fmm at (%.3f, %.3f, %.3f). Trying a bounded (+/-%.0f deg) orientation relaxation...",
                    max_pos_error * 1000.0, x, y, z, relaxed_max_perturb_deg);

        for (int i = 0; i < relaxed_attempts; ++i, ++attempt_count) {
            double angle = angle_dist(gen);
            Eigen::Vector3d axis(gauss(gen), gauss(gen), gauss(gen));
            axis.normalize();
            Eigen::AngleAxisd perturb(angle, axis);
            Eigen::Quaterniond relaxed_q(perturb * target_q);
            relaxed_q.normalize();

            Eigen::Vector3d relaxed_target_pos = Eigen::Vector3d(x, y, z) - (relaxed_q * w2_to_grip_vec);
            geometry_msgs::msg::Pose relaxed_wrist_2;
            relaxed_wrist_2.position.x = relaxed_target_pos.x();
            relaxed_wrist_2.position.y = relaxed_target_pos.y();
            relaxed_wrist_2.position.z = relaxed_target_pos.z();
            relaxed_wrist_2.orientation.x = relaxed_q.x();
            relaxed_wrist_2.orientation.y = relaxed_q.y();
            relaxed_wrist_2.orientation.z = relaxed_q.z();
            relaxed_wrist_2.orientation.w = relaxed_q.w();

            moveit::core::RobotState attempt_state(corrected_seed);
            attempt_state.setToRandomPositions(jmg);

            if (attempt_state.setFromIK(jmg, relaxed_wrist_2, "wrist_2",
                                         per_attempt_timeout, constraint, options)) {
                Eigen::Isometry3d achieved = attempt_state.getGlobalLinkTransform("wrist_2");
                double pos_error = (achieved.translation() - relaxed_target_pos).norm();
                if (pos_error > max_pos_error) continue;

                attempt_state.copyJointGroupPositions("arm", joint_values);
                achieved_orientation = Eigen::Quaterniond(achieved.rotation());
                double deviation_deg = Eigen::AngleAxisd(target_q.inverse() * achieved_orientation).angle() * 180.0 / M_PI;
                RCLCPP_INFO(logger, "Fixed IK found for (%.3f, %.3f, %.3f) via relaxed orientation (attempt %d, %.1f deg off nominal), error %.1fmm",
                            x, y, z, i + 1, deviation_deg, pos_error * 1000.0);
                return true;
            }
        }
    }

    logIKFailureDiagnosis(&corrected_seed, scene, logger, target_wrist_2, x, y, z);
    RCLCPP_ERROR(logger, "Fixed-orientation IK failed for (%.3f, %.3f, %.3f) after %d attempts (including relaxed-orientation fallback).",
                 x, y, z, attempt_count);
    return false;
}

// ==========================================
// PICK-PLACE ACTION SERVER
//
// Everything above this point is untouched. Everything below replaces the
// old one-shot main(): instead of waiting on a /cube_pose topic, building
// one MTC task, executing it, and exiting, this node stays alive and runs
// the exact same setup + IK + MTC pipeline once per accepted goal.
//
// Behavior changes worth knowing about vs. the original script:
//   - On failure, the old code called rclcpp::shutdown() and exited the
//     process. Here, a failure aborts the CURRENT GOAL (goal_handle->abort)
//     and the node stays alive, ready for the next goal. That's the whole
//     point of making it an action server, but it's a real change from
//     "crash and relaunch" to "report failure and wait" — worth watching
//     for the first few runs.
//   - place_pose.position.z from the goal is intentionally NOT used yet.
//     The original code derived place_approach_z from lift_z (where the
//     arm ends up after lifting the cube) plus a hand-tuned clearance,
//     not from any real target height - so place_z ends up constant
//     regardless of what "place height" you'd ask for. I kept that exact
//     derivation rather than guess at swapping in goal z, since it's the
//     tuned behavior that's known to work. Revisit this once real box-
//     height detection feeds into place_pose.
//   - The collision object id is still the literal "red_cube" (via a
//     single OBJECT_ID constant now, instead of ~8 repeated string
//     literals) - not yet tied to which object was actually picked. That's
//     the next step, once the detector publishes labeled detections.
//   - The original file had two nearly-identical execution loops (one for
//     a multi-stage "compound" solution, one for a lone SubTrajectory).
//     They're unified into one runSubTrajectory lambda used by both paths
//     below - same logic, no duplication. In practice this task always
//     produces a compound solution (14 stages), so the "lone trajectory"
//     path is a fallback that's unlikely to run.
// ==========================================
class PickPlaceActionServer : public rclcpp::Node
{
public:
    using PickPlace = giraffe_mtc::action::PickPlace;
    using GoalHandlePickPlace = rclcpp_action::ServerGoalHandle<PickPlace>;

    PickPlaceActionServer()
    : Node("mtc_move_to_cube",
           rclcpp::NodeOptions()
               .automatically_declare_parameters_from_overrides(true)
               .parameter_overrides({rclcpp::Parameter("use_sim_time", true)}))
    {
        action_server_ = rclcpp_action::create_server<PickPlace>(
            this,
            "pick_place",
            std::bind(&PickPlaceActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickPlaceActionServer::handleCancel, this, std::placeholders::_1),
            std::bind(&PickPlaceActionServer::handleAccepted, this, std::placeholders::_1));

        attach_client_ = this->create_client<giraffe_gazebo_plugins::srv::AttachDetach>("/gripper/attach");
        detach_client_ = this->create_client<giraffe_gazebo_plugins::srv::AttachDetach>("/gripper/detach");

        auto logger = this->get_logger();
        RCLCPP_INFO(logger, "Waiting for Gazebo attach/detach services...");
        while (!attach_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(logger, "Gazebo attach service not available, waiting...");
        }
        while (!detach_client_->wait_for_service(std::chrono::seconds(1)) && rclcpp::ok()) {
            RCLCPP_INFO(logger, "Gazebo detach service not available, waiting...");
        }
        RCLCPP_INFO(logger, "Gazebo services ready! pick_place action server started.");
    }

private:
    rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID&,
                                            std::shared_ptr<const PickPlace::Goal> goal)
    {
        if (goal_active_.load()) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal - a pick-place is already in progress");
            return rclcpp_action::GoalResponse::REJECT;
        }
        const auto& p = goal->pick_pose.position;
        if (p.x == 0.0 && p.y == 0.0 && p.z == 0.0) {
            RCLCPP_WARN(this->get_logger(), "Rejecting goal - pick_pose looks uninitialized (0,0,0)");
            return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandlePickPlace>)
    {
        // Accepted, but not wired into the middle of an in-flight plan/execute
        // call below - a goal already planning or moving will still run to
        // completion. Real mid-task cancellation would need cooperative check
        // points inside the execution loop; flagging that honestly rather than
        // pretending it's there.
        RCLCPP_WARN(this->get_logger(), "Cancel requested - will not interrupt a motion already in progress");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
    {
        goal_active_.store(true);
        // MTC planning + execution blocks for a while, so it runs on its own
        // thread - the node's own executor thread stays free to keep servicing
        // tf, the planning scene monitor, MoveGroupInterface, and action
        // negotiation for the (currently rejected) next goal.
        std::thread{std::bind(&PickPlaceActionServer::execute, this, goal_handle)}.detach();
    }

    void execute(const std::shared_ptr<GoalHandlePickPlace> goal_handle)
    {
        auto logger = this->get_logger();
        auto node_ptr = shared_from_this();
        auto goal = goal_handle->get_goal();
        auto result = std::make_shared<PickPlace::Result>();

        auto publishStage = [&](const std::string& stage) {
            auto feedback = std::make_shared<PickPlace::Feedback>();
            feedback->current_stage = stage;
            goal_handle->publish_feedback(feedback);
        };

        auto abortWith = [&](const std::string& msg) {
            RCLCPP_ERROR(logger, "%s", msg.c_str());
            result->success = false;
            result->message = msg;
            goal_handle->abort(result);
            goal_active_.store(false);
        };

        // Comes from the orchestrator, which sets object_id on the goal from
        // whichever detection class_id matched its target_object parameter.
        // Falls back to "red_cube" if a goal is ever sent without it set
        // (e.g. a manual `ros2 action send_goal`), so this stays backward
        // compatible with single-cube use.
        const std::string OBJECT_ID = goal->object_id.empty() ? "red_cube" : goal->object_id;

        // ==========================================
        // TWEAK THESE VALUES FOR TRIAL AND ERROR (unchanged from the original file)
        // ==========================================
        double CUBE_X_OFFSET = -0.005;
        double CUBE_Y_OFFSET = 0.0;
        double CUBE_Z_OFFSET = -0.055;

        double GRIPPER_X_OFFSET = -0.0;
        double GRIPPER_Y_OFFSET = -0.0275;
        double GRIPPER_Z_OFFSET = 0.0275;

        double DESCEND_DISTANCE = 0.05;
        double LIFT_DISTANCE = 0.10;

        double PLACE_APPROACH_CLEARANCE = 0.08;
        double PLACE_DESCEND_DISTANCE = 0.08;

        publishStage("initializing");

        // ==========================================
        // 1. ROBOT MODEL - loaded ONCE, on the first goal, then reused for
        // every goal after. Previously this called task.loadRobotModel()
        // fresh every single goal, and PlanningSceneMonitor/MoveGroupInterface
        // below each independently loaded their OWN robot model too - four
        // separate pluginlib loads of the same kinematics plugin per goal.
        // That's what the "class_loader: SEVERE WARNING... will NOT be
        // unloaded" message at shutdown was actually about: multiple
        // independent loaders managing the same underlying plugin library.
        // Now there's exactly one robot_model_loader_ (a member, alive for
        // the node's whole lifetime), and task/psm/arm_group/gripper_group
        // below all share the one model it produced.
        // ==========================================
        if (!robot_model_) {
            RCLCPP_INFO(logger, "Loading robot model (first goal only - reused for every goal after this)...");
            robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_ptr, "robot_description");
            robot_model_ = robot_model_loader_->getModel();
        }

        if (!robot_model_) {
            abortWith("Failed to load robot model");
            return;
        }

        mtc::Task task("move_to_cube");
        task.setRobotModel(robot_model_);

        // ==========================================
        // 2. GET CURRENT STATE DIRECTLY FROM TF/JOINT_STATES
        // ==========================================
        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
        auto monitor = std::make_shared<planning_scene_monitor::CurrentStateMonitor>(
            node_ptr, robot_model_, tf_buffer, true);

        RCLCPP_INFO(logger, "Waiting for robot state...");
        monitor->waitForCompleteState("arm", 5.0);
        monitor->waitForCompleteState("gripper", 5.0);
        moveit::core::RobotStatePtr current_state = monitor->getCurrentState();

        if (!current_state) {
            abortWith("Failed to get current robot state");
            return;
        }

        // Captured ONCE per goal, before any motion — every desired (fixed)
        // orientation this run is anchored to this, never to a later/mutated state.
        Eigen::Isometry3d initial_sp_pose     = current_state->getGlobalLinkTransform("shoulder_pan");
        Eigen::Isometry3d initial_wrist2_pose = current_state->getGlobalLinkTransform("wrist_2");

        auto psm = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_ptr, robot_model_loader_);
        psm->startSceneMonitor();
        psm->startWorldGeometryMonitor();
        psm->startStateMonitor();

        publishStage("clearing octomap");

        // Force-clear any stale occupied voxels left from a previous goal or
        // earlier testing. Octomap cells only clear when re-observed as free
        // by a sensor ray, so old bad voxels could otherwise persist across goals.
        auto clear_octomap_client = this->create_client<std_srvs::srv::Empty>("/clear_octomap");
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

        // Let the sensors rebuild the octomap cleanly before syncing.
        rclcpp::sleep_for(std::chrono::seconds(3));

        psm->requestPlanningSceneState("/get_planning_scene");

        Eigen::Isometry3d debug_sp = current_state->getGlobalLinkTransform("shoulder_pan");
        RCLCPP_INFO(logger, "Shoulder Pan is at: x=%f, y=%f, z=%f",
            debug_sp.translation().x(), debug_sp.translation().y(), debug_sp.translation().z());

        std::vector<double> initial_joint_values;
        current_state->copyJointGroupPositions("arm", initial_joint_values);

        // ==========================================
        // 3. PICK POSE - comes straight from the goal now, no topic wait needed
        // ==========================================
        geometry_msgs::msg::Pose raw_pose = goal->pick_pose;
        RCLCPP_INFO(logger, "Pick pose from goal -> x: %f, y: %f, z: %f",
            raw_pose.position.x, raw_pose.position.y, raw_pose.position.z);

        // ==========================================
        // 4. ADD CUBE TO PLANNING SCENE (RViz)
        // ==========================================
        geometry_msgs::msg::Pose cube_pose = raw_pose;
        cube_pose.position.x += CUBE_X_OFFSET;
        cube_pose.position.y += CUBE_Y_OFFSET;
        cube_pose.position.z += CUBE_Z_OFFSET;

        moveit::planning_interface::PlanningSceneInterface psi;
        moveit_msgs::msg::CollisionObject cube;
        cube.id = OBJECT_ID;
        cube.header.frame_id = robot_model_->getModelFrame();
        cube.primitives.resize(1);
        cube.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
        cube.primitives[0].dimensions.resize(3);
        cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = 0.03;
        cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = 0.03;
        cube.primitives[0].dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = 0.03;
        cube.pose = cube_pose;
        cube.operation = moveit_msgs::msg::CollisionObject::ADD;
        psi.applyCollisionObject(cube);

        std::vector<std::string> touch_links;
        auto arm_links = robot_model_->getJointModelGroup("arm")->getLinkModelNames();
        auto gripper_links = robot_model_->getJointModelGroup("gripper")->getLinkModelNames();
        touch_links.insert(touch_links.end(), arm_links.begin(), arm_links.end());
        touch_links.insert(touch_links.end(), gripper_links.begin(), gripper_links.end());

        planning_scene::PlanningScenePtr ik_scene;
        {
            planning_scene_monitor::LockedPlanningSceneRO locked_scene(psm);
            ik_scene = locked_scene->diff();
        }
        ik_scene->processCollisionObjectMsg(cube);

        planning_scene::PlanningScenePtr grasp_scene = ik_scene->diff();
        grasp_scene->getAllowedCollisionMatrixNonConst().setEntry(OBJECT_ID, touch_links, true);
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
        geometry_msgs::msg::Pose target = raw_pose;
        target.position.z += GRIPPER_Z_OFFSET;
        target.position.x += GRIPPER_X_OFFSET;
        target.position.y += GRIPPER_Y_OFFSET;

        // ==========================================
        // 5-DOF IK CALCULATIONS
        // ==========================================
        auto world_obj = ik_scene->getWorld()->getObject(planning_scene::PlanningScene::OCTOMAP_NS);
        if (world_obj && !world_obj->shapes_.empty()) {
            auto octree_shape = std::dynamic_pointer_cast<const shapes::OcTree>(world_obj->shapes_[0]);
            if (octree_shape && octree_shape->octree) {
                auto oct_node = octree_shape->octree->search(target.position.x, target.position.y, target.position.z);
                bool occupied = oct_node && octree_shape->octree->isNodeOccupied(oct_node);
                RCLCPP_INFO(logger, "Octomap synced OK. Target point (%.3f,%.3f,%.3f): %s",
                            target.position.x, target.position.y, target.position.z,
                            occupied ? "OCCUPIED" : "free");
            }
        } else {
            RCLCPP_WARN(logger, "ik_scene has NO octomap object — requestPlanningSceneState didn't pull it in.");
        }

        publishStage("computing pick IK");

        // --- PICK LEG ---

        std::vector<double> joint_values;
        Eigen::Quaterniond achieved_grasp_orientation;
        if (!computeFreeOrientationIK(current_state.get(), ik_scene, logger,
                                       target.position.x, target.position.y, target.position.z,
                                       joint_values, achieved_grasp_orientation)) {
            abortWith("Free-orientation IK failed for pick pose");
            return;
        }

        Eigen::Quaterniond desired_grasp_orientation =
            computeDesiredWristOrientation(initial_sp_pose, initial_wrist2_pose,
                                            target.position.x, target.position.y);

        moveit::core::RobotState free_grasp_state(*current_state);
        free_grasp_state.setJointGroupPositions("arm", joint_values);
        free_grasp_state.update();

        std::vector<double> fix_orientation_grasp_joint_values;
        Eigen::Quaterniond achieved_grasp_fixed_orientation;
        if (!computeFixedOrientationIK(&free_grasp_state, ik_scene, logger,
                                        target.position.x, target.position.y, target.position.z,
                                        desired_grasp_orientation,
                                        fix_orientation_grasp_joint_values,
                                        achieved_grasp_fixed_orientation)) {
            abortWith("Fixed-orientation IK failed at pick pose");
            return;
        }

        // Refine the fixed-orientation IK using the first solution as the new seed
        moveit::core::RobotState refined_state(*current_state);
        refined_state.setJointGroupPositions("arm", fix_orientation_grasp_joint_values);
        refined_state.update();

        if (!computeFixedOrientationIK(&refined_state, ik_scene, logger,
                                    target.position.x, target.position.y, target.position.z,
                                    desired_grasp_orientation,
                                    fix_orientation_grasp_joint_values,
                                    achieved_grasp_fixed_orientation)) {
            abortWith("Second fixed-orientation IK refinement failed at pick pose");
            return;
        }

        double grasp_x = target.position.x;
        double grasp_y = target.position.y;
        double grasp_z = target.position.z - DESCEND_DISTANCE;

        std::vector<double> grasp_joint_values;
        Eigen::Quaterniond achieved_descend_orientation;
        if (!computeFixedOrientationIK(current_state.get(), grasp_scene, logger,
                                        grasp_x, grasp_y, grasp_z,
                                        achieved_grasp_fixed_orientation,
                                        grasp_joint_values,
                                        achieved_descend_orientation)) {
            abortWith("Fixed-orientation IK failed for descend-to-grasp");
            return;
        }

        double lift_x = grasp_x;
        double lift_y = grasp_y;
        double lift_z = grasp_z + LIFT_DISTANCE;

        std::vector<double> lift_joint_values;
        Eigen::Quaterniond achieved_lift_orientation;
        if (!computeFixedOrientationIK(current_state.get(), transit_scene, logger,
                                        lift_x, lift_y, lift_z,
                                        achieved_descend_orientation,
                                        lift_joint_values,
                                        achieved_lift_orientation)) {
            abortWith("Fixed-orientation IK failed for lift");
            return;
        }

        publishStage("computing place IK");

        // --- PLACE LEG ---
        // x,y come from the goal's place_pose. z is intentionally still derived
        // from lift_z (matches the original hand-tuned behavior) rather than
        // goal->place_pose.position.z - see the class-level comment for why.
        double place_x = goal->place_pose.position.x;
        double place_y = goal->place_pose.position.y;
        double place_approach_z = lift_z + PLACE_APPROACH_CLEARANCE;

        std::vector<double> place_approach_joint_values;
        Eigen::Quaterniond achieved_place_orientation;
        if (!computeFreeOrientationIK(current_state.get(), transit_scene, logger,
                                       place_x, place_y, place_approach_z,
                                       place_approach_joint_values, achieved_place_orientation)) {
            abortWith("Free-orientation IK failed for place-approach pose");
            return;
        }

        Eigen::Quaterniond desired_place_orientation =
            computeDesiredWristOrientation(initial_sp_pose, initial_wrist2_pose, place_x, place_y);

        moveit::core::RobotState free_place_state(*current_state);
        free_place_state.setJointGroupPositions("arm", place_approach_joint_values);
        free_place_state.update();

        std::vector<double> fix_orientation_place_joint_values;
        Eigen::Quaterniond achieved_place_fixed_orientation;
        if (!computeFixedOrientationIK(&free_place_state, transit_scene, logger,
                                        place_x, place_y, place_approach_z,
                                        desired_place_orientation,
                                        fix_orientation_place_joint_values,
                                        achieved_place_fixed_orientation)) {
            abortWith("Fixed-orientation IK failed at place-approach pose");
            return;
        }

        double place_z = place_approach_z - PLACE_DESCEND_DISTANCE;

        std::vector<double> place_joint_values;
        Eigen::Quaterniond achieved_place_descend_orientation;
        if (!computeFixedOrientationIK(current_state.get(), transit_scene, logger,
                                        place_x, place_y, place_z,
                                        achieved_place_fixed_orientation,
                                        place_joint_values,
                                        achieved_place_descend_orientation)) {
            abortWith("Fixed-orientation IK failed for descend-to-place");
            return;
        }

        const moveit::core::JointModelGroup* jmg = current_state->getJointModelGroup("arm");
        const std::vector<std::string>& joint_names = jmg->getVariableNames();

        // ==========================================
        // MOVEIT TASK CONSTRUCTOR SETUP (unchanged from the original file
        // aside from using OBJECT_ID instead of repeating "red_cube")
        // ==========================================
        auto pipeline = std::make_shared<mtc::solvers::PipelinePlanner>(node_ptr);
        pipeline->setProperty("max_velocity_scaling_factor", 0.2);
        pipeline->setProperty("max_acceleration_scaling_factor", 0.2);

        auto gripper_pipeline = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
        gripper_pipeline->setMaxVelocityScalingFactor(0.1);
        gripper_pipeline->setMaxAccelerationScalingFactor(0.1);

        planning_scene::PlanningScenePtr scene = ik_scene;

        RCLCPP_INFO(logger, "Scene world objects: %zu | has octomap: %s",
            scene->getWorld()->size(),
            scene->getWorld()->hasObject(planning_scene::PlanningScene::OCTOMAP_NS) ? "yes" : "no");

        moveit::core::RobotState& scene_state = scene->getCurrentStateNonConst();
        scene_state = *current_state;

        double safe_zero = 0.0;
        scene_state.setJointPositions("wrist_2_gripper_joint", &safe_zero);

        scene->processCollisionObjectMsg(cube);

        auto current_stage_mtc = std::make_unique<mtc::stages::FixedState>("current state");
        current_stage_mtc->setState(scene);
        task.add(std::move(current_stage_mtc));

        auto allow_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper collision");
        allow_coll->allowCollisions(OBJECT_ID, touch_links, true);
        task.add(std::move(allow_coll));

        auto rotate_wrist = std::make_unique<mtc::stages::MoveRelative>("rotate wrist", pipeline);
        rotate_wrist->setGroup("arm");
        std::map<std::string, double> joint_deltas;
        joint_deltas["wrist_1_wrist_2_joint"] = M_PI / 2.0;
        rotate_wrist->setDirection(joint_deltas);
        rotate_wrist->setProperty("timeout", 10.0);
        task.add(std::move(rotate_wrist));

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

        auto fix_orientation_grasp = std::make_unique<mtc::stages::MoveTo>("fix orientation (grasp)", pipeline);
        fix_orientation_grasp->setGroup("arm");
        std::map<std::string, double> fix_orientation_grasp_targets;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            fix_orientation_grasp_targets[joint_names[i]] = fix_orientation_grasp_joint_values[i];
            if (joint_names[i] == "wrist_1_wrist_2_joint") {
                fix_orientation_grasp_targets[joint_names[i]] += M_PI / 2.0;
            }
        }
        fix_orientation_grasp->setGoal(fix_orientation_grasp_targets);
        fix_orientation_grasp->setProperty("timeout", 10.0);
        task.add(std::move(fix_orientation_grasp));

        auto open_gripper = std::make_unique<mtc::stages::MoveTo>("open gripper", gripper_pipeline);
        open_gripper->setGroup("gripper");
        std::map<std::string, double> gripper_open;
        gripper_open["wrist_2_gripper_joint"] = 1.0;
        open_gripper->setGoal(gripper_open);
        open_gripper->setProperty("timeout", 5.0);
        task.add(std::move(open_gripper));

        auto allow_octomap_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("allow gripper-octomap collision");
        std::vector<std::string> octomap_allow_list = touch_links;
        octomap_allow_list.push_back(OBJECT_ID);
        allow_octomap_coll->allowCollisions(planning_scene::PlanningScene::OCTOMAP_NS, octomap_allow_list, true);
        task.add(std::move(allow_octomap_coll));

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

        auto close_gripper = std::make_unique<mtc::stages::MoveTo>("close gripper", gripper_pipeline);
        close_gripper->setGroup("gripper");
        std::map<std::string, double> gripper_close;
        gripper_close["wrist_2_gripper_joint"] = 0.475;
        close_gripper->setGoal(gripper_close);
        close_gripper->setProperty("timeout", 5.0);
        task.add(std::move(close_gripper));

        auto attach_cube = std::make_unique<mtc::stages::ModifyPlanningScene>("attach cube");
        attach_cube->attachObject(OBJECT_ID, "gripper");
        task.add(std::move(attach_cube));

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

        auto disallow_octomap_coll = std::make_unique<mtc::stages::ModifyPlanningScene>("disallow gripper-octomap collision");
        std::vector<std::string> octomap_disallow_list = touch_links;
        octomap_disallow_list.push_back(OBJECT_ID);
        disallow_octomap_coll->allowCollisions(planning_scene::PlanningScene::OCTOMAP_NS, octomap_disallow_list, false);
        task.add(std::move(disallow_octomap_coll));

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

        auto fix_orientation_place = std::make_unique<mtc::stages::MoveTo>("fix orientation (place)", pipeline);
        fix_orientation_place->setGroup("arm");
        std::map<std::string, double> fix_orientation_place_targets;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            fix_orientation_place_targets[joint_names[i]] = fix_orientation_place_joint_values[i];
            if (joint_names[i] == "wrist_1_wrist_2_joint") {
                fix_orientation_place_targets[joint_names[i]] += M_PI / 2.0;
            }
        }
        fix_orientation_place->setGoal(fix_orientation_place_targets);
        fix_orientation_place->setProperty("timeout", 10.0);
        task.add(std::move(fix_orientation_place));

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

        auto open_gripper_place = std::make_unique<mtc::stages::MoveTo>("open gripper place", gripper_pipeline);
        open_gripper_place->setGroup("gripper");
        std::map<std::string, double> gripper_open_place;
        gripper_open_place["wrist_2_gripper_joint"] = 1.0;
        open_gripper_place->setGoal(gripper_open_place);
        open_gripper_place->setProperty("timeout", 5.0);
        task.add(std::move(open_gripper_place));

        auto detach_cube = std::make_unique<mtc::stages::ModifyPlanningScene>("detach cube");
        detach_cube->detachObject(OBJECT_ID, "gripper");
        task.add(std::move(detach_cube));

        auto return_to_start = std::make_unique<mtc::stages::MoveTo>("return to start", pipeline);
        return_to_start->setGroup("arm");
        std::map<std::string, double> start_targets;
        for (size_t i = 0; i < joint_names.size(); ++i) {
            start_targets[joint_names[i]] = initial_joint_values[i];
        }
        return_to_start->setGoal(start_targets);
        return_to_start->setProperty("timeout", 10.0);
        task.add(std::move(return_to_start));

        auto close_gripper_final = std::make_unique<mtc::stages::MoveTo>("close gripper final", gripper_pipeline);
        close_gripper_final->setGroup("gripper");
        std::map<std::string, double> gripper_close_final;
        gripper_close_final["wrist_2_gripper_joint"] = 0.0;
        close_gripper_final->setGoal(gripper_close_final);
        close_gripper_final->setProperty("timeout", 5.0);
        task.add(std::move(close_gripper_final));

        // ==========================================
        // PLAN & EXECUTE
        // ==========================================
        publishStage("planning");
        RCLCPP_INFO(logger, "Planning with MTC...");
        auto plan_result = task.plan(1);

        if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
            abortWith("MTC planning failed");
            return;
        }

        RCLCPP_INFO(logger, "MTC Planning SUCCESS! Extracting trajectory...");

        using moveit::planning_interface::MoveGroupInterface;
        // Both reuse the one shared robot_model_ - see the note at the top of
        // this function on why (each of these would otherwise independently
        // load its own kinematics plugin).
        MoveGroupInterface::Options arm_opts("arm");
        arm_opts.robot_model = robot_model_;
        MoveGroupInterface arm_group(node_ptr, arm_opts);

        MoveGroupInterface::Options gripper_opts("gripper");
        gripper_opts.robot_model = robot_model_;
        MoveGroupInterface gripper_group(node_ptr, gripper_opts);
        arm_group.startStateMonitor();
        gripper_group.startStateMonitor();

        // ==========================================
        // RE-APPROACH - runs unconditionally at two points, right before
        // "descend to cube" and right before "open gripper place". Re-solves
        // IK for the SAME target position the preceding stage was already
        // aiming for, but seeded from the arm's ACTUAL current state instead
        // of the original seed_state every other stage in this task was
        // planned from (that seed was captured once, before any real motion
        // happened - by this point the arm may have drifted from what the
        // plan assumed). No distance measurement, no threshold - it always
        // re-targets the point fresh as one extra move.
        // ==========================================
        auto reapproachTarget = [&](const std::string& label,
                                     double x, double y, double z,
                                     const Eigen::Quaterniond& desired_orientation,
                                     const planning_scene::PlanningSceneConstPtr& scene) -> bool
        {
            moveit::core::RobotStatePtr live_state;
            {
                rclcpp::Time now = this->now();
                if (!monitor->waitForCurrentState(now, 1.0)) {
                    RCLCPP_WARN(logger, "[%s] Timed out waiting for a fresh robot state - using best available", label.c_str());
                }
                live_state = monitor->getCurrentState();
            }
            if (!live_state) {
                RCLCPP_ERROR(logger, "[%s] Couldn't get a robot state to re-approach from", label.c_str());
                return false;
            }

            std::vector<double> reapproach_joint_values;
            Eigen::Quaterniond reapproach_achieved_orientation;
            if (!computeFixedOrientationIK(live_state.get(), scene, logger, x, y, z,
                                            desired_orientation, reapproach_joint_values,
                                            reapproach_achieved_orientation)) {
                RCLCPP_ERROR(logger, "[%s] Re-approach IK failed", label.c_str());
                return false;
            }

            std::map<std::string, double> reapproach_targets;
            for (size_t i = 0; i < joint_names.size(); ++i) {
                reapproach_targets[joint_names[i]] = reapproach_joint_values[i];
                if (joint_names[i] == "wrist_1_wrist_2_joint") {
                    reapproach_targets[joint_names[i]] += M_PI / 2.0;
                }
            }

            arm_group.setJointValueTarget(reapproach_targets);
            moveit::core::MoveItErrorCode move_result = arm_group.move();
            if (move_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "[%s] Re-approach move failed to execute (error code %d)",
                             label.c_str(), move_result.val);
                return false;
            }

            RCLCPP_INFO(logger, "[%s] Re-approach complete", label.c_str());
            return true;
        };

        auto sol = task.solutions().front();
        auto compound = dynamic_cast<const mtc::SolutionSequence*>(sol.get());

        bool execution_failed = false;
        std::string failure_reason;

        // Unified execution of a single MTC sub-trajectory - used for every
        // stage regardless of whether the overall solution is a compound
        // sequence (the normal case here, 14 stages) or a lone trajectory.
        auto runSubTrajectory = [&](const mtc::SubTrajectory* traj) {
            if (!traj || !traj->trajectory() || execution_failed) return;

            auto rt = std::make_shared<robot_trajectory::RobotTrajectory>(*traj->trajectory());
            std::string group_name = rt->getGroupName();
            std::string stage_name = traj->creator()->name();

            if (stage_name == "descend to cube") {
                if (!reapproachTarget("pre-grasp re-approach", target.position.x, target.position.y, target.position.z,
                                       achieved_grasp_fixed_orientation, ik_scene)) {
                    execution_failed = true;
                    failure_reason = "Pre-grasp re-approach failed";
                    return;
                }
            } else if (stage_name == "open gripper place") {
                if (!reapproachTarget("place re-approach", place_x, place_y, place_z,
                                       achieved_place_descend_orientation, transit_scene)) {
                    execution_failed = true;
                    failure_reason = "Place re-approach failed";
                    return;
                }
            }

            MoveGroupInterface::Plan plan;
            RCLCPP_INFO(logger, "Executing '%s' trajectory...", stage_name.c_str());
            publishStage(stage_name);

            trajectory_processing::TimeOptimalTrajectoryGeneration totg;
            totg.computeTimeStamps(*rt, 0.1, 0.1);
            rt->getRobotTrajectoryMsg(plan.trajectory);

            if (plan.trajectory.joint_trajectory.points.empty()) return;

            moveit::core::MoveItErrorCode exec_result;
            if (group_name == "gripper") {
                if (stage_name == "open gripper place") {
                    RCLCPP_INFO(logger, "Manually detaching cube from MoveIt planning scene...");
                    arm_group.detachObject(OBJECT_ID);

                    RCLCPP_INFO(logger, "Detaching cube in Gazebo...");
                    auto detach_req = std::make_shared<giraffe_gazebo_plugins::srv::AttachDetach::Request>();
                    detach_req->model_name = OBJECT_ID;
                    auto detach_future = detach_client_->async_send_request(detach_req);
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
                exec_result = gripper_group.execute(plan);
            } else {
                if (stage_name == "lift up") {
                    RCLCPP_INFO(logger, "Manually attaching cube to gripper in MoveIt planning scene...");
                    bool attached = arm_group.attachObject(OBJECT_ID, "gripper", touch_links);
                    if (attached) {
                        RCLCPP_INFO(logger, "Cube attached successfully in MoveIt!");
                    } else {
                        RCLCPP_ERROR(logger, "Cube FAILED to attach in MoveIt!");
                    }

                    RCLCPP_INFO(logger, "Attaching cube in Gazebo...");
                    auto attach_req = std::make_shared<giraffe_gazebo_plugins::srv::AttachDetach::Request>();
                    attach_req->model_name = OBJECT_ID;
                    auto attach_future = attach_client_->async_send_request(attach_req);
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
                exec_result = arm_group.execute(plan);
            }

            // CRITICAL: stop immediately if a stage didn't actually execute -
            // otherwise every later stage (planned assuming this one HAD
            // moved the robot) keeps getting sent anyway, cascading into a
            // string of aborts and an arm that ends up somewhere nobody planned for.
            if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(logger, "Execution of '%s' FAILED (error code %d) — stopping here.",
                             stage_name.c_str(), exec_result.val);
                execution_failed = true;
                failure_reason = "Execution of '" + stage_name + "' failed";
            }
        };

        if (compound) {
            for (const auto& sub : compound->solutions()) {
                runSubTrajectory(dynamic_cast<const mtc::SubTrajectory*>(sub));
                if (execution_failed) break;
            }
        } else {
            runSubTrajectory(dynamic_cast<const mtc::SubTrajectory*>(sol.get()));
        }

        if (execution_failed) {
            abortWith(failure_reason);
            return;
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

        result->success = true;
        result->message = "Pick-place completed successfully";
        goal_handle->succeed(result);
        goal_active_.store(false);
    }

    rclcpp_action::Server<PickPlace>::SharedPtr action_server_;
    // Loaded once, on the first goal - see the note at the top of execute()
    moveit::core::RobotModelConstPtr robot_model_;
    robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
    rclcpp::Client<giraffe_gazebo_plugins::srv::AttachDetach>::SharedPtr attach_client_;
    rclcpp::Client<giraffe_gazebo_plugins::srv::AttachDetach>::SharedPtr detach_client_;
    std::atomic<bool> goal_active_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickPlaceActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}