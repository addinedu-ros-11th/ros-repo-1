import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Arguments
    namespace = LaunchConfiguration('namespace')
    is_sim = LaunchConfiguration('is_sim')
    cam_tilt_deg = LaunchConfiguration('cam_tilt_deg')

    # 프레임 접두어 계산 (예: robot02/)
    frame_prefix = PythonExpression([
        "'", namespace, "/' if '", namespace, "' != '' else ''"
    ])

    # URDF 경로
    pkg_share = FindPackageShare('pinky_description')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])

    # 1. Robot State Publisher
    # namespace 속성을 비워두어 부모(pinky_app)의 PushRosNamespace를 그대로 따르게 합니다.
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': is_sim,
            'robot_description': Command([
                'xacro ', urdf_path,
                ' namespace:=', namespace,
                ' is_sim:=', is_sim,
                ' cam_tilt_deg:=', cam_tilt_deg
            ]),
            'frame_prefix': frame_prefix
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )

    # 2. Joint State Publisher (필요한 경우에만 사용)
    # bringup.py가 이미 joint_states를 발행하므로, 여기서는 URDF 연결을 위해 robot_description만 주입합니다.
    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': is_sim,
            'robot_description': Command([
                'xacro ', urdf_path,
                ' namespace:=', namespace,
                ' is_sim:=', is_sim,
                ' cam_tilt_deg:=', cam_tilt_deg
            ])
        }],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('tf', '/tf'),
            ('tf_static', '/tf_static')
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value=''),
        DeclareLaunchArgument('is_sim', default_value='false'),
        DeclareLaunchArgument('cam_tilt_deg', default_value='0'),
        rsp_node,
        jsp_node
    ])
