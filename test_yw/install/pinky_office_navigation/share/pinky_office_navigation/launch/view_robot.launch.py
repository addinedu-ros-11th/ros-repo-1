import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # RViz 설정 파일 경로
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('pinky_office_navigation'),
        'rviz',
        'template_view.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ns',
            default_value='robot02',
            description='Namespace of the robot (used for reference, but using static rviz config)'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ])
