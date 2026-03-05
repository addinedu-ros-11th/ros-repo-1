from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    rviz_config = LaunchConfiguration("rviz_config")

    default_config = PathJoinSubstitution(
        [FindPackageShare("office_robot_bringup"), "config", "nav_debug.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot01"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("rviz_config", default_value=default_config),
            LogInfo(msg=["[rviz-debug] robot_ns: ", robot_ns]),
            LogInfo(msg=["[rviz-debug] config: ", rviz_config]),
            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="nav_debug_rviz",
                        output="screen",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                    ),
                ]
            ),
        ]
    )
