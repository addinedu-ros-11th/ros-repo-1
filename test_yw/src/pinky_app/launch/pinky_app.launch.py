import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
    # --- Arguments ---
    robot_ns = LaunchConfiguration("robot_ns")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # --- Paths ---
    pinky_app_share = FindPackageShare('pinky_app')
    pinky_navigation_share = FindPackageShare('pinky_navigation')
    
    # Config files (정적 할당된 YAML 파일 사용)
    camera_config = PathJoinSubstitution([pinky_app_share, 'config', 'camera.yaml'])
    executor_config = PathJoinSubstitution([pinky_app_share, 'config', 'executor.yaml'])
    bridge_config = PathJoinSubstitution([pinky_app_share, 'config', 'bridge.yaml'])
    nav2_params_file = PathJoinSubstitution([pinky_app_share, 'config', 'nav2_params.yaml'])
    default_map_file = PathJoinSubstitution([pinky_navigation_share, 'map', 'office_map.yaml'])
    map_yaml_file = LaunchConfiguration("map", default=default_map_file)

    return LaunchDescription([
        DeclareLaunchArgument("robot_ns", default_value="robot02"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("map", default_value=default_map_file),

        # 1. 네임스페이스 종속 그룹 (Hardware, Nav2, Control)
        GroupAction([
            PushRosNamespace(robot_ns),
            
            # Hardware & RSP
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"])
                ),
                launch_arguments={'use_sim_time': use_sim_time, 'namespace': 'robot02'}.items()
            ),

            # Navigation Stack
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_navigation"), "launch", "bringup_launch.xml"])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': nav2_params_file,
                    'namespace': 'robot02',
                    'use_composition': 'False',
                }.items(),
            ),

            # Control Logic
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_control"), "launch", "control.launch.py"])
                ),
                launch_arguments={
                    'params_file': executor_config,
                    'robot_name': 'robot02',
                    'odom_frame_id': 'robot02/odom',
                    'base_frame_id': 'robot02/base_footprint',
                }.items()
            ),
            Node(
                package="pinky_control",
                executable="camera_node",
                name="camera_driver",
                parameters=[camera_config],
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
            ),
        ]),

        # 2. 통신 및 브릿지 (전역 또는 명시적 네임스페이스 할당)
        # AI Server Bridge
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("pinky_comms"), "launch", "bridge.launch.py"])
            ),
            launch_arguments={
                'params_file': bridge_config,
                # 만약 bridge_node 내부에서 robot_name을 필요로 한다면 명시적으로 전달
            }.items()
        ),
        # Rosbridge WebSocket (포트 충돌 방지를 위해 전역에서 실행 권장)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("pinky_bridge"), "launch", "rosbridge.launch.py"])
            ),
            launch_arguments={'port': '9090'}.items() 
        ),
    ])
