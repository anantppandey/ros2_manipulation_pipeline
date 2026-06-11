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

## Remarks per commit

* Need to add sensor plugin to world file for camera to work
* Changed camera to Depth Camera in URDF and changed sensor images to sensor depth images and point cloud in the bridge node in launch file along with adding Static TF node to Launch file which gives the correct position for the depth camera in Rviz and moveit. (Check to make this in urdf only instead of launching it as static TF node)
* changed depth_camera to rgbd_camera in the urdf file as that was not publishing image data and fixed getting proper data from the sensor. Now Image/ camera and image are normally working and for depth_image/camera and image to show normal images first need to open /images and the turn off normalize and set max value as 5 that gives proper image and then camera also starts working. The data is being published to /wrist_camera/camera_info, /wrist_camera/depth_image, /wrist_camera/image, /wrist_camera/points.
* To add Overhead camera added the link connected that link to base link made the gazebo refernce sensor added the things to bridge and then finally added the static_tf_2 to launch file.
 ### Stattic_TF node has the arguments as Yaw pitch Roll instead of Roll pitch yaw so you copy the Pose Values from the gazebo sensor in urdf and put there 

* Added incomplete cuber detector and arm mover C++ files but they not working right now and made changes to cmake and package.xml for those c++ files
* Commented out Wrist camera to make the moveit code work again
* Added front camera as well 


