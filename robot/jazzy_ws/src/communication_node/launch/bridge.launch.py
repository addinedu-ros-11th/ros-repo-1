from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("ai_server_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("ai_server_port", default_value="54321"),
            DeclareLaunchArgument("encoding_mode", default_value="mjpeg"),
            DeclareLaunchArgument("udp_payload_max", default_value="1400"),
            Node(
                package="communication_node",
                executable="bridge_node",
                name="communication_bridge",
                parameters=[
                    {
                        "image_topic": LaunchConfiguration("image_topic"),
                        "ai_server_ip": LaunchConfiguration("ai_server_ip"),
                        "ai_server_port": LaunchConfiguration("ai_server_port"),
                        "encoding_mode": LaunchConfiguration("encoding_mode"),
                        "udp_payload_max": LaunchConfiguration("udp_payload_max"),
                    }
                ],
            ),
        ]
    )
