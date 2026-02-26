from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot_1"),
            DeclareLaunchArgument("robot_id", default_value="1"),
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
            Node(
                package="office_robot_executor",
                executable="office_robot_executor_node",
                name="office_robot_executor",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "mock_mode": LaunchConfiguration("mock_mode"),
                        "use_nav2": LaunchConfiguration("use_nav2"),
                        "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                        "nav2_action_name": LaunchConfiguration("nav2_action_name"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
                        "stop_cmd_vel_topic": LaunchConfiguration("stop_cmd_vel_topic"),
                        "stop_publish_count": LaunchConfiguration("stop_publish_count"),
                        "stop_publish_hz": LaunchConfiguration("stop_publish_hz"),
                        "safety_lock_topic": LaunchConfiguration("safety_lock_topic"),
                        "ai_link_topic": LaunchConfiguration("ai_link_topic"),
                        "include_ai_link_in_status": LaunchConfiguration("include_ai_link_in_status"),
                        "enable_display": LaunchConfiguration("enable_display"),
                        "display_topic": LaunchConfiguration("display_topic"),
                        "guide_display_period_sec": LaunchConfiguration("guide_display_period_sec"),
                        "emit_command_received_event": LaunchConfiguration("emit_command_received_event"),
                    }
                ],
            ),
        ]
    )
