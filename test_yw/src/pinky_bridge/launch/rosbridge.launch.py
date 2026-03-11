from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    port = LaunchConfiguration("port")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="9090"),
        DeclareLaunchArgument("params_file", default_value=""),
        
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            name="rosbridge_websocket",
            output="screen",
            parameters=[{
                "port": port,
                "address": "0.0.0.0",
                "retry_startup_delay": 5.0,
            }, params_file],
            # 노드가 네임스페이스 안에 들어갈 경우를 대비해 전역 /tf로 강제 매핑
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ]
        ),
    ])
