from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    port = LaunchConfiguration("port")
    params_file = LaunchConfiguration("params_file")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot02"),
            DeclareLaunchArgument("port", default_value="9090"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("enable_rosbridge", default_value="true"),
            
            LogInfo(msg=["[rosbridge] Starting with port: ", port]),
            
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[
                    {"port": 9090}, # Default fallback
                    params_file     # Overwrite with yaml if provided
                ],
                # rosbridge should generally listen globally, but we can respect namespace if needed.
                # Usually it sits at root to bridge everything.
                # namespace=robot_ns 
            ),
        ]
    )
