# ROS2 Manipulation Pipeline

A ROS 2 based robotic manipulation pipeline built using Gazebo, MoveIt 2, and RGB-D perception for autonomous pick-and-place simulation.

This project focuses on integrating robotic simulation, motion planning, perception, and manipulation into a unified pipeline for autonomous grasp execution in simulation.

The system combines:

* Gazebo-based robotic simulation
* MoveIt 2 trajectory planning and execution
* RGB-D based object localization
* Point cloud generation and processing
* TF2 frame transformations
* Autonomous pick-and-place task execution

The primary goal of this project is to build a coherent manipulation stack where perception data is fused with motion planning to enable robotic grasping and object interaction in simulation.

---

## System Overview

```text
RGB-D Camera
      ↓
Perception Pipeline
      ↓
Object Pose Estimation
      ↓
TF2 Frame Transformation
      ↓
MoveIt 2 Motion Planning
      ↓
Trajectory Execution
      ↓
Pick-and-Place Manipulation
```

---

## Getting Started

## MoveIt Task Constructor Setup

This project depends on MoveIt Task Constructor (MTC), which is not included in this repository.

### Clone the repository

```bash
cd ~/ft_ws/src

git clone --branch ros2 https://github.com/moveit/moveit_task_constructor.git
```

### Initialize submodules

```bash
cd moveit_task_constructor

git submodule update --init --recursive
```

### Install dependencies

```bash
cd ~/ft_ws

rosdep install --from-paths src --ignore-src -r -y
```

### Build the workspace

```bash
colcon build --symlink-install
```

After building, source the workspace:

```bash
source ~/ft_ws/install/setup.zsh
```


## Remarks per commit

* Need to add sensor plugin to world file for camera to work
* Changed camera to Depth Camera in URDF and changed sensor images to sensor depth images and point cloud in the bridge node in launch file along with adding Static TF node to Launch file which gives the correct position for the depth camera in Rviz and moveit. (Check to make this in urdf only instead of launching it as static TF node)
* changed depth_camera to rgbd_camera in the urdf file as that was not publishing image data and fixed getting proper data from the sensor. Now Image/ camera and image are normally working and for depth_image/camera and image to show normal images first need to open /images and the turn off normalize and set max value as 5 that gives proper image and then camera also starts working. The data is being published to /wrist_camera/camera_info, /wrist_camera/depth_image, /wrist_camera/image, /wrist_camera/points.
* To add Overhead camera added the link connected that link to base link made the gazebo refernce sensor added the things to bridge and then finally added the static_tf_2 to launch file.
 ### Stattic_TF node has the arguments as Yaw pitch Roll instead of Roll pitch yaw so you copy the Pose Values from the gazebo sensor in urdf and put there 

* Added incomplete cuber detector and arm mover C++ files but they not working right now and made changes to cmake and package.xml for those c++ files
* Commented out Wrist camera to make the moveit code work again
* Added front camera as well 
* Changed Urdf to have 0 variation in the pose of sensor and just rotated the camera itself and fixed launch tf for it as well and fixed the red cube detector to give the correct positioin of the cube
* Fixed movit trail as it was not getting the same movit parameters as launch file so made a launch file for it so now has kinametic.yaml file and all also fixed the timing issue by adding use sim time.

## MTC Started
* changes to cmake and code colcon build working
* The main issue was that MoveIt Task Constructor (MTC) could not initialize its internal OMPL planning pipeline, even though the robot model, SRDF, and kinematics were loading correctly. Initially, the MTC node was launched with manually loaded URDF, SRDF, kinematics, and OMPL YAML files, which did not recreate the complete MoveIt configuration expected by the planning pipeline. The solution was to launch the MTC node using MoveItConfigsBuilder and moveit_config.to_dict(), just like the official MoveIt/MTC demos. This automatically provided the complete planning pipeline configuration (including OMPL, adapters, joint limits, and planning parameters), allowing MTC to successfully initialize the planner and generate valid motion plans.
* Added Publisher to Red Cube detector and changes to mtc_node 
* Movit Trial finally working as intended By combining Eigen geometry (to calculate the base angle and rotate the orientation) with KDL (to solve the remaining wrist pitches), we essentially wrote a custom 5-DOF IK solver wrapper. We gave KDL an easy puzzle, and gave OMPL a joint-based goal. That is why it works flawlessly. (For more details see how movit trail works in documets)


### Implementation Notes: MoveIt Task Constructor (MTC) in ROS 2 Jazzy

* **Node Architecture:** Transitioned from standard `MoveGroupInterface` planning to an MTC `Task` pipeline, utilizing `PlanningSceneMonitor` to fetch the current robot state directly.
* **Custom 5-DOF IK Integration:** Preserved the custom inverse kinematics math (base yaw alignment + seeded IK) and passed the resulting joint targets to an MTC `MoveTo` stage using a `std::map<std::string, double>`.
* **Launch File Parameters:** Injected `planning_pipelines` (OMPL) and `robot_description_planning` (joint limits) directly into the node's namespace, as MTC runs the planning pipeline internally rather than querying the `move_group` node.
* **Time Parameterization Fix:** Added strict joint acceleration limits to `joint_limits.yaml` to prevent the `AddTimeOptimalParameterization` (TOTG) adapter from failing during trajectory generation.
* **Execution Server Workaround:** Bypassed the missing `execute_task_solution` MTC action server in Jazzy by dynamically casting the MTC `SolutionSequence` to extract the `RobotTrajectory`, then executing it sequentially via the standard `MoveGroupInterface::execute()`.
* **Relative Motion Stages:** Added a 90-degree wrist rotation using an MTC `MoveRelative` stage, noting that it requires `setDirection()` instead of `setGoal()`.

* Gripper Actuation in Gazebo: The gripper required a combination of URDF, YAML, and C++ adjustments to actuate smoothly. Realistic velocity limits (0.5 rad/s) and widened floating-point bounds (-0.05) were added to the URDF to prevent MoveIt bounds-checking failures and instant snapping. To bypass a missing MTC execution plugin in ROS 2 Jazzy, the final trajectory was extracted directly from the MTC SolutionSequence, re-parameterized via TOTG, and executed natively via MoveGroupInterface.


Implementation Notes: Grasping and Object Attachment

Grasp Sequence & Collision Handling: Added descent, close-gripper, and lift stages using pre-calculated 5-DOF IK targets. Expanded the allowCollisions stage to include both arm and gripper links, preventing OMPL from aborting when the gripper contacted the cube.
Manual Object Attachment: Because we bypassed MTC's internal execution server, MTC's attachObject stage didn't reach the real move_group node. We fixed this by intercepting the "lift up" trajectory in our execution loop and manually calling arm_group.attachObject() to synchronize the real MoveIt planning scene, ensuring the cube's collision geometry moved with the gripper.
Decoupled Offsets: Split the RViz cube pose offsets from the Gazebo gripper target offsets to independently tune visual accuracy and physical grasp alignment.


CLEARER EXPLAINATION OF CUSTOM PLUGIN NEEDED
