import os
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
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    # --- Paths ---
    pinky_app_share = FindPackageShare('pinky_app')
    pinky_navigation_share = FindPackageShare('pinky_navigation')
    
    # Config files (전용 YAML 파일 경로)
    camera_config = PathJoinSubstitution([pinky_app_share, 'config', 'camera.yaml'])
    executor_config = PathJoinSubstitution([pinky_app_share, 'config', 'executor.yaml'])
    bridge_config = PathJoinSubstitution([pinky_app_share, 'config', 'bridge.yaml'])
    nav2_params_file = PathJoinSubstitution([pinky_app_share, 'config', 'nav2_params.yaml'])
    default_map_file = PathJoinSubstitution([pinky_navigation_share, 'map', 'office_map.yaml'])
    map_yaml_file = LaunchConfiguration("map", default=default_map_file)

    # Frame Prefix (e.g., 'robot02/')
    frame_prefix = PythonExpression(["'", robot_ns, "/' if '", robot_ns, "' != '' else ''"])

    # --- Nav2 Parameter Rewriting ---
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'base_frame_id': [frame_prefix, 'base_footprint'],
        'odom_frame_id': [frame_prefix, 'odom'],
        'global_frame_id': [frame_prefix, 'map'], 
        'robot_base_frame': [frame_prefix, 'base_footprint'],
        'local_frame': [frame_prefix, 'odom'],
        # local_costmap과 global_costmap의 global_frame을 구분하기 위해
        # 아래와 같이 명시적으로 경로를 지정하거나, 공통 분모를 찾아 처리합니다.
        'local_costmap.local_costmap.ros__parameters.global_frame': [frame_prefix, 'odom'],
        'global_frame': [frame_prefix, 'map'], 
        'scan_topic': 'scan'
    }

    # 추가적인 로직: local_costmap의 global_frame만 odom으로 강제 치환
    # RewrittenYaml은 키 기반이므로, YAML 내부에서 local_costmap의 global_frame 키를
    # 'local_global_frame' 같은 임시 이름으로 바꾸고 여기서 매핑하는 것이 가장 깔끔합니다.


    configured_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key=robot_ns, 
        param_rewrites=param_substitutions,
        convert_types=True
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ns", default_value="robot02"),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("map", default_value=default_map_file),

        # 모든 요소를 네임스페이스 그룹으로 통합
        GroupAction([
            PushRosNamespace(robot_ns),
            
            # 1. Hardware & RSP (Robot State Publisher)
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"])
                ),
                launch_arguments={'use_sim_time': use_sim_time, 'namespace': robot_ns}.items()
            ),

            # 2. Navigation Stack (AMCL, Map Server, Nav2)
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_navigation"), "launch", "bringup_launch.xml"])
                ),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'map': map_yaml_file,
                    'params_file': configured_params, 
                    'namespace': robot_ns,
                    'use_composition': 'False',
                }.items(),
            ),

            # 3. Control Logic (Executor, Safety, Camera)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_control"), "launch", "control.launch.py"])
                ),
                launch_arguments={
                    'params_file': executor_config,
                    'robot_name': robot_ns,
                    'odom_frame_id': [frame_prefix, 'odom'],
                    'base_frame_id': [frame_prefix, 'base_footprint'],
                }.items()
            ),
            Node(
                package="pinky_control",
                executable="camera_node",
                name="camera_driver",
                parameters=[camera_config],
                remappings=[('tf', '/tf'), ('tf_static', '/tf_static')]
            ),

            # 4. Comms & Bridge (AI Server Bridge & Rosbridge)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_comms"), "launch", "bridge.launch.py"])
                ),
                launch_arguments={'params_file': bridge_config}.items()
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([FindPackageShare("pinky_bridge"), "launch", "rosbridge.launch.py"])
                ),
                launch_arguments={'port': '9090'}.items() # 필요 시 포트 변경
            ),
        ])
    ])
