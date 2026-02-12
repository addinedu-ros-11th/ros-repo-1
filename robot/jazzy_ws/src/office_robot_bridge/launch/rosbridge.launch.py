from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    port = LaunchConfiguration("port")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot"),
            DeclareLaunchArgument("port", default_value="9090"),
            DeclareLaunchArgument("enable_rosbridge", default_value="true"),
            LogInfo(msg=["[rosbridge] robot_ns: ", robot_ns]),
            LogInfo(msg=["[rosbridge] rosbridge_port: ", port]),
            LogInfo(msg=["[rosbridge] enable_rosbridge: ", enable_rosbridge]),
            LogInfo(
                msg=[
                    "[rosbridge] ws uri: ws://<robot_ip>:",
                    port,
                ]
            ),
            LogInfo(
                msg=[
                    "[rosbridge] check: ros2 node list | grep rosbridge",
                ]
            ),
            LogInfo(
                msg=[
                    "[rosbridge] check: ss -lntp | grep ",
                    port,
                ]
            ),
            LogInfo(
                msg=[
                    "[rosbridge] check: ros2 topic list | grep /",
                    robot_ns,
                ]
            ),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "rosbridge_server",
                    "rosbridge_websocket",
                    "--port",
                    port,
                ],
                output="screen",
            ),
        ]
    )
