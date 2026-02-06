from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot_a"),
            DeclareLaunchArgument("enable_rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("mock_mode", default_value="true"),
            DeclareLaunchArgument("execution_delay_sec", default_value="1.5"),
            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("office_robot_executor"),
                                    "launch",
                                    "executor.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "robot_name": robot_ns,
                            "mock_mode": LaunchConfiguration("mock_mode"),
                            "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("office_robot_bridge"),
                                    "launch",
                                    "rosbridge.launch.py",
                                ]
                            )
                        ),
                        condition=IfCondition(enable_rosbridge),
                        launch_arguments={
                            "robot_ns": robot_ns,
                            "enable_rosbridge": enable_rosbridge,
                            "port": rosbridge_port,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
