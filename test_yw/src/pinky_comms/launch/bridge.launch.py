from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Use LaunchConfiguration without 'default' argument
    params_file = LaunchConfiguration("params_file")
    image_topic = LaunchConfiguration("image_topic")
    ai_server_ip = LaunchConfiguration("ai_server_ip")
    ai_server_port = LaunchConfiguration("ai_server_port")

    return LaunchDescription(
        [
            # Declare arguments with default_value
            DeclareLaunchArgument("image_topic", default_value="camera/image_raw"),
            DeclareLaunchArgument("ai_server_ip", default_value="127.0.0.1"),
            DeclareLaunchArgument("ai_server_port", default_value="54321"),
            DeclareLaunchArgument("params_file", default_value=""),
            
            Node(
                package="pinky_comms",
                executable="bridge_node",
                name="communication_bridge",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "ai_server_ip": ai_server_ip,
                        "ai_server_port": ai_server_port,
                    },
                    params_file # If params_file is not empty, it will override the dict above
                ],
                output="screen"
            ),
        ]
    )
