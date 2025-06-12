import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    
    logger = LaunchConfiguration("log_level")
    spacing_ms = LaunchConfiguration("spacing_ms")
    msg_size = LaunchConfiguration("msg_size")
    ns = 'latency_test_talker'
    
    ld.add_action(DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    ))
    
    
    ld.add_action(DeclareLaunchArgument(
        "spacing_ms",
        default_value="100",
        description="Spacing in milliseconds",
    ))

    ld.add_action(DeclareLaunchArgument(
        "msg_size",
        default_value="5",
    ))
    
    package_name = 'latency_test_talker'
    executable_name = 'latency_test_talker'
    
    # Launch the mux node with parameters
    node_mux = Node(
        package=package_name,
        executable=executable_name,
        namespace=ns,
        output='screen',
        arguments=['--ros-args',
                  '--log-level', logger,
        ],
        parameters=[
        {"spacing_ms": spacing_ms, "msg_size": msg_size}  # Explicitly convert to int
    ]
    )
    
    ld.add_action(node_mux)
    return ld