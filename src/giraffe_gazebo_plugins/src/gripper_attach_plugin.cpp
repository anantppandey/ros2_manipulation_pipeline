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

#include "giraffe_gazebo_plugins/gripper_attach_plugin.hpp"

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>

using namespace std::chrono_literals;

namespace giraffe_gazebo_plugins
{

CustomAttachPlugin::CustomAttachPlugin() = default;

CustomAttachPlugin::~CustomAttachPlugin()
{
  if (this->executor)
  {
    this->executor->cancel();
  }
  if (this->executorThread.joinable())
  {
    this->executorThread.join();
  }
  // Deliberately NOT calling rclcpp::shutdown() here: other gz-sim system
  // plugins loaded in this same process (e.g. ign_ros2_control-system) may
  // still be relying on the global rclcpp context.
}

void CustomAttachPlugin::Configure(
  const gz::sim::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & /*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity);
  if (!this->model.Valid(_ecm))
  {
    gzerr << "[CustomAttachPlugin] Must be attached to a <model>. "
          << "Failed to initialize." << std::endl;
    return;
  }

  if (_sdf->HasElement("parent_link"))
  {
    this->parentLinkName = _sdf->Get<std::string>("parent_link");
  }
  this->parentLinkEntity = this->model.LinkByName(_ecm, this->parentLinkName);
  if (this->parentLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[CustomAttachPlugin] Link '" << this->parentLinkName
          << "' not found in model '" << this->model.Name(_ecm)
          << "'. Check the <parent_link> parameter. Failed to initialize."
          << std::endl;
    return;
  }

  if (_sdf->HasElement("child_model"))
  {
    this->childModelName = _sdf->Get<std::string>("child_model");
  }
  if (_sdf->HasElement("child_link"))
  {
    this->childLinkName = _sdf->Get<std::string>("child_link");
  }
  if (_sdf->HasElement("max_attach_distance"))
  {
    this->maxAttachDistance = _sdf->Get<double>("max_attach_distance");
  }
  if (_sdf->HasElement("attach_service"))
  {
    this->attachServiceName = _sdf->Get<std::string>("attach_service");
  }
  if (_sdf->HasElement("detach_service"))
  {
    this->detachServiceName = _sdf->Get<std::string>("detach_service");
  }

  this->validConfig = true;

  // Best-effort early resolution. It's fine if the child hasn't spawned
  // yet -- ResolveChildEntities() is re-run on every attach attempt too.
  this->ResolveChildEntities(_ecm);

  // ---- Bring up an embedded ROS 2 node for the attach/detach services ----
  if (!rclcpp::ok())
  {
    // Only touch the global context if nobody else in this process (e.g.
    // ign_ros2_control-system, also loaded via <gazebo>) has done it yet.
    rclcpp::init(0, nullptr);
  }

  this->rosNode = std::make_shared<rclcpp::Node>(
    "gripper_attach_plugin_" + this->model.Name(_ecm));

  this->attachSrv = this->rosNode->create_service<std_srvs::srv::Trigger>(
    this->attachServiceName,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      this->OnAttach(req, res);
    });

  this->detachSrv = this->rosNode->create_service<std_srvs::srv::Trigger>(
    this->detachServiceName,
    [this](
      const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
      this->OnDetach(req, res);
    });

  this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  this->executor->add_node(this->rosNode);
  this->executorThread = std::thread([this]() {this->executor->spin();});

  gzmsg << "[CustomAttachPlugin] Ready on model '" << this->model.Name(_ecm)
        << "'. Attach: '" << this->attachServiceName << "', Detach: '"
        << this->detachServiceName << "', parent_link='"
        << this->parentLinkName << "', child_model='" << this->childModelName
        << "', child_link='" << this->childLinkName << "'." << std::endl;
}

bool CustomAttachPlugin::ResolveChildEntities(gz::sim::EntityComponentManager & _ecm)
{
  if (this->childModelEntity == gz::sim::kNullEntity ||
    !_ecm.HasEntity(this->childModelEntity))
  {
    this->childModelEntity = _ecm.EntityByComponents(
      gz::sim::components::Model(),
      gz::sim::components::Name(this->childModelName));
    // Force re-lookup of the link too, in case the model was respawned
    // under the same name with a new entity id.
    this->childLinkEntity = gz::sim::kNullEntity;
  }

  if (this->childModelEntity == gz::sim::kNullEntity)
  {
    return false;
  }

  if (this->childLinkEntity == gz::sim::kNullEntity ||
    !_ecm.HasEntity(this->childLinkEntity))
  {
    this->childLinkEntity = _ecm.EntityByComponents(
      gz::sim::components::Link(),
      gz::sim::components::ParentEntity(this->childModelEntity),
      gz::sim::components::Name(this->childLinkName));
  }

  return this->childLinkEntity != gz::sim::kNullEntity;
}

bool CustomAttachPlugin::DoAttach(
  gz::sim::EntityComponentManager & _ecm, std::string & _message)
{
  if (this->isAttached)
  {
    _message = "Already attached";
    return true;
  }

  if (!this->ResolveChildEntities(_ecm))
  {
    _message = "Could not resolve child model '" + this->childModelName +
      "' / link '" + this->childLinkName +
      "' in the world -- has it spawned yet?";
    return false;
  }

  // Safety check: refuse to weld across the room. This is what stops a
  // stale MTC command, or a bad cube-pose estimate, from silently welding
  // the gripper to an object it never actually reached.
  std::optional<gz::math::Pose3d> parentPose =
    gz::sim::Link(this->parentLinkEntity).WorldPose(_ecm);
  std::optional<gz::math::Pose3d> childPose =
    gz::sim::Link(this->childLinkEntity).WorldPose(_ecm);
  if (parentPose.has_value() && childPose.has_value())
  {
    double distance = (parentPose->Pos() - childPose->Pos()).Length();
    if (distance > this->maxAttachDistance)
    {
      _message = "Refusing to attach: '" + this->parentLinkName + "' and '" +
        this->childModelName + "/" + this->childLinkName + "' are " +
        std::to_string(distance) + " m apart (max allowed " +
        std::to_string(this->maxAttachDistance) +
        " m). Descend further before attaching.";
      return false;
    }
  }

  // Create the runtime fixed joint. This is the same primitive the stock
  // gz::sim::systems::DetachableJoint system uses: an entity carrying a
  // DetachableJoint component, which the physics system turns into an
  // actual fixed-joint constraint between the two links.
  this->detachableJointEntity = _ecm.CreateEntity();
  _ecm.CreateComponent(
    this->detachableJointEntity,
    gz::sim::components::DetachableJoint(
      {this->parentLinkEntity, this->childLinkEntity, "fixed"}));

  this->isAttached = true;
  _message = "Attached '" + this->childModelName + "' to '" +
    this->parentLinkName + "'";
  gzmsg << "[CustomAttachPlugin] " << _message
        << " (joint entity " << this->detachableJointEntity << ")"
        << std::endl;
  return true;
}

bool CustomAttachPlugin::DoDetach(
  gz::sim::EntityComponentManager & _ecm, std::string & _message)
{
  if (!this->isAttached)
  {
    _message = "Already detached";
    return true;
  }

  if (this->detachableJointEntity != gz::sim::kNullEntity)
  {
    _ecm.RequestRemoveEntity(this->detachableJointEntity);
  }
  this->detachableJointEntity = gz::sim::kNullEntity;
  this->isAttached = false;
  _message = "Detached '" + this->childModelName + "' from '" +
    this->parentLinkName + "'";
  gzmsg << "[CustomAttachPlugin] " << _message << std::endl;
  return true;
}

bool CustomAttachPlugin::SubmitRequest(RequestType _type, std::string & _message)
{
  if (!this->validConfig)
  {
    _message = "Plugin failed to configure -- check the gz sim log";
    return false;
  }

  std::unique_lock<std::mutex> lock(this->requestMutex);
  if (this->pendingRequest != RequestType::NONE)
  {
    _message = "Another attach/detach request is already in flight";
    return false;
  }

  this->pendingRequest = _type;
  this->requestComplete = false;

  bool done = this->requestCv.wait_for(
    lock, 2s, [this] {return this->requestComplete;});

  if (!done)
  {
    _message = "Timed out waiting for the simulation loop to process the "
      "request (is the simulation running and unpaused?)";
    this->pendingRequest = RequestType::NONE;
    return false;
  }

  _message = this->requestMessage;
  return this->requestSuccess;
}

void CustomAttachPlugin::OnAttach(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*_req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> _res)
{
  std::string message;
  _res->success = this->SubmitRequest(RequestType::ATTACH, message);
  _res->message = message;
}

void CustomAttachPlugin::OnDetach(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*_req*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> _res)
{
  std::string message;
  _res->success = this->SubmitRequest(RequestType::DETACH, message);
  _res->message = message;
}

void CustomAttachPlugin::PreUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  gz::sim::EntityComponentManager & _ecm)
{
  if (!this->validConfig)
  {
    return;
  }

  std::unique_lock<std::mutex> lock(this->requestMutex);
  RequestType req = this->pendingRequest;
  lock.unlock();

  if (req == RequestType::NONE)
  {
    return;
  }

  bool success = false;
  std::string message;
  if (req == RequestType::ATTACH)
  {
    success = this->DoAttach(_ecm, message);
  }
  else if (req == RequestType::DETACH)
  {
    success = this->DoDetach(_ecm, message);
  }

  lock.lock();
  this->pendingRequest = RequestType::NONE;
  this->requestSuccess = success;
  this->requestMessage = message;
  this->requestComplete = true;
  lock.unlock();
  this->requestCv.notify_all();
}

}  // namespace giraffe_gazebo_plugins

GZ_ADD_PLUGIN(
  giraffe_gazebo_plugins::CustomAttachPlugin,
  gz::sim::System,
  giraffe_gazebo_plugins::CustomAttachPlugin::ISystemConfigure,
  giraffe_gazebo_plugins::CustomAttachPlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  giraffe_gazebo_plugins::CustomAttachPlugin,
  "giraffe_gazebo_plugins::CustomAttachPlugin")
