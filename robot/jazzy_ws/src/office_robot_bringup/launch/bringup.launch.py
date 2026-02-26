from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    robot_id = LaunchConfiguration("robot_id")
    use_nav2 = LaunchConfiguration("use_nav2")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    frame_id = LaunchConfiguration("frame_id")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    stop_cmd_vel_topic = LaunchConfiguration("stop_cmd_vel_topic")
    stop_publish_count = LaunchConfiguration("stop_publish_count")
    stop_publish_hz = LaunchConfiguration("stop_publish_hz")
    safety_lock_topic = LaunchConfiguration("safety_lock_topic")
    ai_link_topic = LaunchConfiguration("ai_link_topic")
    include_ai_link_in_status = LaunchConfiguration("include_ai_link_in_status")
    enable_display = LaunchConfiguration("enable_display")
    display_topic = LaunchConfiguration("display_topic")
    guide_display_period_sec = LaunchConfiguration("guide_display_period_sec")
    emit_command_received_event = LaunchConfiguration("emit_command_received_event")
    safety_stop_publish_hz = LaunchConfiguration("safety_stop_publish_hz")
    safety_lock_keepalive_hz = LaunchConfiguration("safety_lock_keepalive_hz")
    safety_stop_publish_count = LaunchConfiguration("safety_stop_publish_count")
    safety_params_file = PathJoinSubstitution(
        [FindPackageShare("office_robot_bringup"), "config", "safety.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot_1"),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("enable_rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("mock_mode", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("execution_delay_sec", default_value="1.5"),
            DeclareLaunchArgument("nav2_action_name", default_value="navigate_to_pose"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("goal_timeout_sec", default_value="60.0"),
            DeclareLaunchArgument("stop_cmd_vel_topic", default_value="cmd_vel"),
            DeclareLaunchArgument("stop_publish_count", default_value="10"),
            DeclareLaunchArgument("stop_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("safety_lock_topic", default_value="safety_lock"),
            DeclareLaunchArgument("ai_link_topic", default_value="ai_link"),
            DeclareLaunchArgument("include_ai_link_in_status", default_value="true"),
            DeclareLaunchArgument("enable_display", default_value="true"),
            DeclareLaunchArgument("display_topic", default_value="display"),
            DeclareLaunchArgument("guide_display_period_sec", default_value="2.0"),
            DeclareLaunchArgument("emit_command_received_event", default_value="true"),
            DeclareLaunchArgument("safety_stop_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("safety_lock_keepalive_hz", default_value="2.0"),
            DeclareLaunchArgument("safety_stop_publish_count", default_value="10"),
            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    Node(
                        package="office_robot_safety",
                        executable="office_robot_safety_node",
                        name="office_robot_safety",
                        parameters=[
                            safety_params_file,
                            {
                                "robot_name": robot_ns,
                                "robot_id": robot_id,
                                "cmd_topic": "commands",
                                "lock_topic": safety_lock_topic,
                                "stop_cmd_vel_topic": stop_cmd_vel_topic,
                                "stop_publish_hz": safety_stop_publish_hz,
                                "lock_keepalive_hz": safety_lock_keepalive_hz,
                                "stop_publish_count": safety_stop_publish_count,
                            },
                        ],
                    ),
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
                            "robot_id": robot_id,
                            "mock_mode": LaunchConfiguration("mock_mode"),
                            "use_nav2": use_nav2,
                            "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                            "nav2_action_name": nav2_action_name,
                            "frame_id": frame_id,
                            "goal_timeout_sec": goal_timeout_sec,
                            "stop_cmd_vel_topic": stop_cmd_vel_topic,
                            "stop_publish_count": stop_publish_count,
                            "stop_publish_hz": stop_publish_hz,
                            "safety_lock_topic": safety_lock_topic,
                            "ai_link_topic": ai_link_topic,
                            "include_ai_link_in_status": include_ai_link_in_status,
                            "enable_display": enable_display,
                            "display_topic": display_topic,
                            "guide_display_period_sec": guide_display_period_sec,
                            "emit_command_received_event": emit_command_received_event,
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
