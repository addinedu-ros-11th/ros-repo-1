import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    # --- Arguments ---
    robot_ns = LaunchConfiguration("robot_ns")
    use_slam = LaunchConfiguration("use_slam")
    
    # --- Paths ---
    pinky_app_share = FindPackageShare('pinky_app')
    pinky_navigation_share = FindPackageShare('pinky_navigation')
    
    camera_config = PathJoinSubstitution([pinky_app_share, 'config', 'camera.yaml'])
    executor_config = PathJoinSubstitution([pinky_app_share, 'config', 'executor.yaml'])
    bridge_config = PathJoinSubstitution([pinky_app_share, 'config', 'bridge.yaml'])
    nav2_params_file = PathJoinSubstitution([pinky_app_share, 'config', 'nav2_params.yaml'])
    
    # [중요] 맵 파일 이름이 office_map.yaml 인지 확인
    default_map_file = PathJoinSubstitution([pinky_navigation_share, 'map', 'office_map.yaml'])
    map_yaml_file = LaunchConfiguration("map", default=default_map_file)

    # Frame Prefix Logic (e.g., 'robot02/')
    frame_prefix = PythonExpression([
        "'", robot_ns, "/' if '", robot_ns, "' != '' else ''"
    ])

    # --- Dynamic Parameter Rewriting ---
    # Only rewrite frame names to include the namespace prefix
    param_substitutions = {
        'use_sim_time': 'False',
        'yaml_filename': map_yaml_file,
        'base_frame_id': PythonExpression(["'", frame_prefix, "base_footprint'"]),
        'odom_frame_id': PythonExpression(["'", frame_prefix, "odom'"]),
        'global_frame_id': PythonExpression(["'", frame_prefix, "map'"]),
        'robot_base_frame': PythonExpression(["'", frame_prefix, "base_footprint'"]),
        'local_frame': PythonExpression(["'", frame_prefix, "odom'"]),
        'frame_id': PythonExpression(["'", frame_prefix, "map'"]),
        'scan_topic': 'scan'
    }

    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=robot_ns, 
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot02"),
            DeclareLaunchArgument("use_slam", default_value="False"),
            DeclareLaunchArgument("map", default_value=default_map_file),

            # 모든 노드를 하나의 네임스페이스 그룹으로 묶습니다.
            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    
                    # 1. Hardware Drivers
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"])
                        ),
                        launch_arguments={
                            'use_sim_time': 'False',
                            'namespace': robot_ns, 
                            'frame_prefix': frame_prefix
                        }.items()
                    ),

                    # 2. Navigation Stack (여기로 이동하여 중복 네임스페이스 방지)
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_navigation"), "launch", "bringup_launch.xml"])
                        ),
                        launch_arguments={
                            'use_sim_time': 'False',
                            'map': map_yaml_file,
                            'params_file': configured_params, 
                            'autostart': 'True',
                            'slam': use_slam,
                            'use_namespace': 'True',
                            'namespace': robot_ns,
                            'use_composition': 'False',
                        }.items(),
                    ),

                    # 1-2. Camera Driver
                    Node(
                        package="pinky_control",
                        executable="camera_node",
                        name="camera_driver",
                        parameters=[camera_config], 
                        output="screen"
                    ),
                    
                    # 3. Application Logic
                    Node(
                        package="pinky_control",
                        executable="executor_node",
                        name="executor_node",
                        parameters=[executor_config], 
                        output="screen"
                    ),
                    Node(
                        package="pinky_control",
                        executable="safety_node",
                        name="safety_node",
                        parameters=[executor_config], 
                        output="screen"
                    ),
                    Node(
                        package="pinky_control",
                        executable="initial_pose_setter",
                        name="initial_pose_setter",
                        parameters=[executor_config],
                        output="screen"
                    ),

                    # 4. Communication
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_comms"), "launch", "bridge.launch.py"])
                        ),
                        launch_arguments={"params_file": bridge_config}.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_bridge"), "launch", "rosbridge.launch.py"])
                        ),
                        launch_arguments={
                            "robot_ns": robot_ns,
                            "params_file": bridge_config, 
                        }.items(),
                    ),
                ]
            ),
        ]
    )
