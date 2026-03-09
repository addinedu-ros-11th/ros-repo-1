from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=""),
        
        Node(
            package="pinky_comms",
            executable="bridge_node",
            name="communication_bridge",
            parameters=[params_file],
            output="screen"
        ),
    ])
