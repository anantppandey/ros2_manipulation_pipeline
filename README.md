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
