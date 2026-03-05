import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # --- Arguments ---
    robot_ns = LaunchConfiguration("robot_ns")
    use_slam = LaunchConfiguration("use_slam")
    map_yaml_file = LaunchConfiguration("map")

    # --- Paths ---
    pinky_app_share = FindPackageShare('pinky_app')
    camera_config = PathJoinSubstitution([pinky_app_share, 'config', 'camera.yaml'])
    executor_config = PathJoinSubstitution([pinky_app_share, 'config', 'executor.yaml'])
    bridge_config = PathJoinSubstitution([pinky_app_share, 'config', 'bridge.yaml'])
    nav2_params_file = PathJoinSubstitution([pinky_app_share, 'config', 'nav2_params.yaml'])
    
    default_map_file = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'), 'map', 'office_map.yaml'
    ])

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot02"),
            DeclareLaunchArgument("use_slam", default_value="False"),
            DeclareLaunchArgument("map", default_value=default_map_file),

            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    
                    # 1. Hardware Drivers
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"])
                        ),
                        launch_arguments={'use_sim_time': 'False'}.items()
                    ),

                    # 1-2. Camera Driver
                    Node(
                        package="pinky_control",
                        executable="camera_node",
                        name="camera_driver",
                        parameters=[camera_config], 
                        output="screen"
                    ),
                    
                    # 2. Navigation Stack
                    # --- [ORIGINAL START] ---
                    # IncludeLaunchDescription(
                    #     AnyLaunchDescriptionSource(
                    #         PathJoinSubstitution([FindPackageShare("pinky_navigation"), "launch", "bringup_launch.xml"])
                    #     ),
                    #     launch_arguments={
                    #         'use_sim_time': 'False',
                    #         'map': map_yaml_file,
                    #         'params_file': nav2_params_file,
                    #         'autostart': 'True',
                    #         'slam': use_slam,
                    #         'use_namespace': 'False', 
                    #         'namespace': ''
                    #     }.items(),
                    # ),
                    # --- [ORIGINAL END] ---

                    # --- [REPLACEMENT: DIRECT LOCALIZATION] ---
                    Node(
                        package='nav2_map_server',
                        executable='map_server',
                        name='map_server',
                        output='screen',
                        parameters=[nav2_params_file, {'yaml_filename': map_yaml_file}]
                    ),
                    Node(
                        package='nav2_amcl',
                        executable='amcl',
                        name='amcl',
                        output='screen',
                        parameters=[nav2_params_file],
                        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
                    ),
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_localization',
                        output='screen',
                        parameters=[{
                            'use_sim_time': False,
                            'autostart': True,
                            'node_names': ['map_server', 'amcl']
                        }]
                    ),
                    
                    # 3. Path Planning & Control
                    IncludeLaunchDescription(
                        AnyLaunchDescriptionSource(
                            PathJoinSubstitution([FindPackageShare("pinky_navigation"), "launch", "navigation_launch.xml"])
                        ),
                        launch_arguments={
                            'use_sim_time': 'False',
                            'params_file': nav2_params_file,
                            'autostart': 'True',
                            'use_composition': 'False',
                        }.items(),
                    ),

                    # 4. Application Logic
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

                    # 5. Communication
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
