from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("camera_source", default_value="topic"),
            DeclareLaunchArgument("ai_server_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("ai_server_port", default_value="54321"),
            DeclareLaunchArgument("encoding_mode", default_value="mjpeg"),
            DeclareLaunchArgument("udp_payload_max", default_value="1400"),
            DeclareLaunchArgument("max_fps", default_value="8.0"),
            DeclareLaunchArgument("resize_width", default_value="640"),
            DeclareLaunchArgument("resize_height", default_value="360"),
            DeclareLaunchArgument("jpeg_quality", default_value="70"),
            DeclareLaunchArgument("udp_warn_throttle_sec", default_value="5.0"),
            DeclareLaunchArgument("ai_link_topic", default_value="/robot_1/ai_link"),
            DeclareLaunchArgument("ai_healthcheck_enabled", default_value="true"),
            DeclareLaunchArgument("ai_healthcheck_mode", default_value="tcp_port"),
            DeclareLaunchArgument("ai_healthcheck_port", default_value="50052"),
            DeclareLaunchArgument("ai_healthcheck_period_sec", default_value="2.0"),
            DeclareLaunchArgument("ai_healthcheck_timeout_sec", default_value="0.4"),
            DeclareLaunchArgument("ai_healthcheck_fail_threshold", default_value="3"),
            DeclareLaunchArgument("ai_healthcheck_recover_threshold", default_value="1"),
            DeclareLaunchArgument("skip_stream_when_ai_dead", default_value="true"),
            DeclareLaunchArgument("ai_dead_log_period_sec", default_value="30.0"),
            DeclareLaunchArgument("rpicam_cmd", default_value="rpicam-vid"),
            DeclareLaunchArgument("rpicam_restart_backoff_sec", default_value="2.0"),
            DeclareLaunchArgument("rpicam_use_system_libs", default_value="true"),
            Node(
                package="communication_node",
                executable="bridge_node",
                name="communication_bridge",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "camera_source": LaunchConfiguration("camera_source"),
                        "ai_server_ip": LaunchConfiguration("ai_server_ip"),
                        "ai_server_port": LaunchConfiguration("ai_server_port"),
                        "encoding_mode": LaunchConfiguration("encoding_mode"),
                        "udp_payload_max": LaunchConfiguration("udp_payload_max"),
                        "max_fps": LaunchConfiguration("max_fps"),
                        "resize_width": LaunchConfiguration("resize_width"),
                        "resize_height": LaunchConfiguration("resize_height"),
                        "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                        "udp_warn_throttle_sec": LaunchConfiguration("udp_warn_throttle_sec"),
                        "ai_link_topic": LaunchConfiguration("ai_link_topic"),
                        "ai_healthcheck_enabled": LaunchConfiguration("ai_healthcheck_enabled"),
                        "ai_healthcheck_mode": LaunchConfiguration("ai_healthcheck_mode"),
                        "ai_healthcheck_port": LaunchConfiguration("ai_healthcheck_port"),
                        "ai_healthcheck_period_sec": LaunchConfiguration("ai_healthcheck_period_sec"),
                        "ai_healthcheck_timeout_sec": LaunchConfiguration("ai_healthcheck_timeout_sec"),
                        "ai_healthcheck_fail_threshold": LaunchConfiguration("ai_healthcheck_fail_threshold"),
                        "ai_healthcheck_recover_threshold": LaunchConfiguration("ai_healthcheck_recover_threshold"),
                        "skip_stream_when_ai_dead": LaunchConfiguration("skip_stream_when_ai_dead"),
                        "ai_dead_log_period_sec": LaunchConfiguration("ai_dead_log_period_sec"),
                        "rpicam_cmd": LaunchConfiguration("rpicam_cmd"),
                        "rpicam_restart_backoff_sec": LaunchConfiguration("rpicam_restart_backoff_sec"),
                        "rpicam_use_system_libs": LaunchConfiguration("rpicam_use_system_libs"),
                    }
                ],
            ),
        ]
    )
