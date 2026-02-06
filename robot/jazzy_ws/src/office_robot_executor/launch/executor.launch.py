from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot_a"),
            DeclareLaunchArgument("mock_mode", default_value="true"),
            DeclareLaunchArgument("execution_delay_sec", default_value="1.5"),
            Node(
                package="office_robot_executor",
                executable="office_robot_executor_node",
                name="office_robot_executor",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "mock_mode": LaunchConfiguration("mock_mode"),
                        "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                    }
                ],
            ),
        ]
    )
