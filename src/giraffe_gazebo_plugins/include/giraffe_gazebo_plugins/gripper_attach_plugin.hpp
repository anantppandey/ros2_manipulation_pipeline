// Copyright 2026
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

#pragma once

#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <sdf/Element.hh>

#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace giraffe_gazebo_plugins
{

/// @brief Gazebo Harmonic (gz-sim8) system plugin that creates or removes a
/// runtime fixed joint ("weld") between a link on the model this plugin is
/// attached to (the "parent link", e.g. a gripper) and a link on a
/// separately-spawned model (the "child model", e.g. an object to pick up).
///
/// This intentionally reuses the same underlying mechanism as Gazebo's
/// built-in `gz::sim::systems::DetachableJoint` system -- an entity carrying
/// a `components::DetachableJoint` component, which the physics system
/// interprets as a fixed joint constraint spanning two different models --
/// but exposes it through two ROS 2 `std_srvs/srv/Trigger` services
/// (`~/attach`, `~/detach`, default `/gripper/attach` and `/gripper/detach`)
/// rather than Gazebo Transport topics. That makes it directly callable,
/// synchronously, from a MoveIt Task Constructor (or any other ROS 2) node,
/// which is what makes pick-and-place reliable instead of depending on
/// friction-only grasping.
///
/// SDF parameters (all optional except noted, matching the stock
/// DetachableJoint system's naming where possible):
///   - `parent_link`  (required): link on this plugin's model to weld from.
///   - `child_model`  (required): name of the other model to weld to.
///   - `child_link`   (required): link on the child model to weld to.
///   - `max_attach_distance` (default 0.08 m): attach requests are refused
///        if the parent and child links are currently farther apart than
///        this. Prevents a stale/incorrect command from teleport-welding
///        the gripper to an object it isn't actually touching.
///   - `attach_service` (default "/gripper/attach")
///   - `detach_service` (default "/gripper/detach")
///
/// Threading model: the ROS 2 services run on a dedicated executor thread.
/// Service callbacks never touch the EntityComponentManager directly --
/// they hand a request off (mutex + condition_variable) to PreUpdate(),
/// which runs on the simulation loop thread and is the only place ECM
/// mutations happen, as required by gz-sim. The service call blocks
/// (up to a timeout) until PreUpdate() has actually processed the request,
/// so the ROS caller gets a real pass/fail result, not a fire-and-forget ack.
class CustomAttachPlugin :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  CustomAttachPlugin();
  ~CustomAttachPlugin() override;

  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  enum class RequestType
  {
    NONE,
    ATTACH,
    DETACH
  };

  // --- ROS service callbacks (run on the executor thread) ---
  void OnAttach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> _req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> _res);
  void OnDetach(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> _req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> _res);

  /// Hands a request to PreUpdate() and blocks until it's processed or
  /// times out. Safe to call from the ROS executor thread.
  bool SubmitRequest(RequestType _type, std::string & _message);

  // --- Simulation-thread helpers (only ever called from PreUpdate) ---
  bool ResolveChildEntities(gz::sim::EntityComponentManager & _ecm);
  bool DoAttach(gz::sim::EntityComponentManager & _ecm, std::string & _message);
  bool DoDetach(gz::sim::EntityComponentManager & _ecm, std::string & _message);

  // ---- SDF-configured parameters ----
  std::string parentLinkName{"gripper"};
  std::string childModelName{"red_cube"};
  std::string childLinkName{"link"};
  double maxAttachDistance{0.08};
  std::string attachServiceName{"/gripper/attach"};
  std::string detachServiceName{"/gripper/detach"};

  // ---- Resolved simulation entities (sim thread only) ----
  gz::sim::Model model;
  gz::sim::Entity parentLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity childModelEntity{gz::sim::kNullEntity};
  gz::sim::Entity childLinkEntity{gz::sim::kNullEntity};
  gz::sim::Entity detachableJointEntity{gz::sim::kNullEntity};
  bool isAttached{false};
  bool validConfig{false};

  // ---- Cross-thread request hand-off ----
  std::mutex requestMutex;
  std::condition_variable requestCv;
  RequestType pendingRequest{RequestType::NONE};
  bool requestComplete{false};
  bool requestSuccess{false};
  std::string requestMessage;

  // ---- Embedded ROS 2 node ----
  rclcpp::Node::SharedPtr rosNode;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attachSrv;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detachSrv;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  std::thread executorThread;
};

}  // namespace giraffe_gazebo_plugins
