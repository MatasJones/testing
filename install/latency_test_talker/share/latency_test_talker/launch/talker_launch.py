import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))

    package_name = 'latency_test_talker'
    executable_name = 'talker'  # Ensure this matches the built executable

    node_talker = Node(
        package=package_name,
        executable=executable_name,
        output='screen',
        arguments=['--ros-args', '--log-level', logger],
    )
    ld.add_action(node_talker)

    return ld
