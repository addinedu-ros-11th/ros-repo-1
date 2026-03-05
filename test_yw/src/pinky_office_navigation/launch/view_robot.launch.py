import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # --- Arguments ---
    robot_ns = LaunchConfiguration('robot_ns')
    
    # 1. 맵과 센서 설정이 들어있는 기존 RViz 설정 파일 경로 (원본 패키지 참조)
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('pinky_navigation'), 'rviz', 'nav2_view.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ns',
            default_value='robot02',
            description='Namespace of the robot to visualize'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # 설정 파일 로드 인자 활성화
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': False}],
            output='screen',
            # 중요: 글로벌 경로들을 네임스페이스 경로로 자동 매핑
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/map', [robot_ns, '/map']),
                ('/scan', [robot_ns, '/scan']),
                ('/goal_pose', [robot_ns, '/goal_pose']),
                ('/initialpose', [robot_ns, '/initialpose']),
                ('/amcl_pose', [robot_ns, '/amcl_pose']),
                ('/odom', [robot_ns, '/odom']),
                ('/particlecloud', [robot_ns, '/particlecloud']),
            ]
        )
    ])
