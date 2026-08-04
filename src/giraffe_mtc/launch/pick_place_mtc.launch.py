import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as f:
            return f.read()
    except EnvironmentError:
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():

    ############################################################
    # Launch argument - which cube to pick
    ############################################################

    target_object_arg = DeclareLaunchArgument(
        "target_object",
        default_value="red_cube",
        description="Which cube to pick: red_cube, blue_cube, or yellow_cube",
    )

    ############################################################
    # Robot Description
    ############################################################

    xacro_path = os.path.join(
        get_package_share_directory("giraffe_description"),
        "urdf",
        "giraffe.urdf.xacro",
    )

    robot_description = {
        "robot_description": xacro.process_file(xacro_path).toxml()
    }

    ############################################################
    # Semantic Description
    ############################################################

    robot_description_semantic = {
        "robot_description_semantic":
        load_file(
            "giraffe_moveit_config",
            "config/giraffe.srdf",
        )
    }

    ############################################################
    # Kinematics
    ############################################################

    robot_description_kinematics = {
        "robot_description_kinematics":
        load_yaml(
            "giraffe_moveit_config",
            "config/kinematics.yaml",
        )
    }

    ############################################################
    # Joint Limits
    ############################################################

    joint_limits_yaml = load_yaml(
        "giraffe_moveit_config",
        "config/joint_limits.yaml",
    )

    robot_description_planning = {
        "robot_description_planning": joint_limits_yaml if joint_limits_yaml is not None else {}
    }

    ############################################################
    # Planning Pipelines (Required for MTC)
    ############################################################

    ompl_yaml = load_yaml(
        "giraffe_moveit_config",
        "config/ompl_planning.yaml",
    )

    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "ompl": ompl_yaml if ompl_yaml is not None else {}
    }

    ############################################################
    # 1. Perception - starts immediately, no MoveIt config needed
    ############################################################

    object_detector_node = Node(
        package="giraffe_description",
        executable="object_detector",
        output="screen",
        parameters=[
            {"use_sim_time": True},
        ],
    )

    ############################################################
    # 2. MTC action server - needs the full MoveIt config that used
    #    to live in the old pick_place_mtc.launch.py
    ############################################################

    mtc_node = Node(
        package="giraffe_mtc",
        executable="mtc_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    # Starts once object_detector's process has been spawned - see the
    # note in the chat reply about what this ordering does and doesn't
    # guarantee.
    start_mtc_after_detector = RegisterEventHandler(
        OnProcessStart(
            target_action=object_detector_node,
            on_start=[mtc_node],
        )
    )

    ############################################################
    # 3. Orchestrator - just needs to know which cube to go after
    ############################################################

    pick_place_orchestrator_node = Node(
        package="giraffe_mtc",
        executable="pick_place_orchestrator",
        output="screen",
        parameters=[
            {"target_object": LaunchConfiguration("target_object")},
            {"use_sim_time": True},
        ],
    )

    # Starts once mtc_node's process has been spawned.
    start_orchestrator_after_mtc = RegisterEventHandler(
        OnProcessStart(
            target_action=mtc_node,
            on_start=[pick_place_orchestrator_node],
        )
    )

    return LaunchDescription([
        target_object_arg,
        object_detector_node,
        start_mtc_after_detector,
        start_orchestrator_after_mtc,
    ])