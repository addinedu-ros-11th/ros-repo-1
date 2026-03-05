from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Arguments
    # Launch arguments are kept for compatibility but are not forced into parameters
    # to allow YAML configuration (params_file) to take precedence or be the sole source.
    robot_name = LaunchConfiguration("robot_name")
    robot_id = LaunchConfiguration("robot_id")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="pinky_robot"),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("params_file", default_value=""),
            
            Node(
                package="pinky_control",
                executable="executor_node",
                name="executor_node",
                parameters=[
                    params_file, # Load from config/executor.yaml
                ],
                output="screen"
            ),
            
            Node(
                package="pinky_control",
                executable="safety_node",
                name="safety_node",
                parameters=[
                    params_file,
                ],
                output="screen"
            ),
        ]
    )
