from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    red_cube_detector = Node(
        package='giraffe_description',
        executable='red_cube_detector',
        name='red_cube_detector',
        output='screen',
    )

    return LaunchDescription([
        red_cube_detector,
    ])
