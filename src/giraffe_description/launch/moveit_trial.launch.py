import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as f:
        return f.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    with open(absolute_file_path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():

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
    # OMPL
    ############################################################

    ompl_yaml = load_yaml(
        "giraffe_moveit_config",
        "config/ompl_planning.yaml",
    )

    ############################################################
    # Trial Node
    ############################################################

    moveit_trial = Node(
        package="giraffe_description",
        executable="moveit_trial",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            ompl_yaml,
            {
                "use_sim_time": True
            },
        ],
    )

    return LaunchDescription([
        moveit_trial,
    ])