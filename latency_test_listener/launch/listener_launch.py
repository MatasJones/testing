
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    logger = LaunchConfiguration("log_level")

    ns = 'latency_test_listener'

    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))

    package_name = 'latency_test_listener'
    executable_name = 'latency_test_listener'

    # Launch the mux node with parameters
    node_mux = Node(
        package=package_name,
        executable=executable_name,
        namespace=ns,
        output='screen',
        arguments=['--ros-args',
            '--log-level', logger,
            ]
    )
    ld.add_action(node_mux)

    return ld