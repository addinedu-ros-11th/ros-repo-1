from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    robot_name = LaunchConfiguration("robot_name")
    odom_frame_id = LaunchConfiguration("odom_frame_id")
    base_frame_id = LaunchConfiguration("base_frame_id")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("robot_name", default_value="pinky"),
        DeclareLaunchArgument("odom_frame_id", default_value="odom"),
        DeclareLaunchArgument("base_frame_id", default_value="base_footprint"),
        
        # 1. Executor Node
        Node(
            package="pinky_control",
            executable="executor_node",
            name="executor_node",
            parameters=[params_file, {
                "robot_name": robot_name,
                "odom_frame_id": odom_frame_id,
                "base_frame_id": base_frame_id,
            }],
            output="screen"
        ),

        # 2. Safety Node
        Node(
            package="pinky_control",
            executable="safety_node",
            name="safety_node",
            parameters=[params_file],
            output="screen"
        ),

        # 3. Display Node (LCD Driver)
        Node(
            package="pinky_control",
            executable="display_node",
            name="display_node",
            parameters=[params_file],
            output="screen"
        ),

        # 3. Initial Pose Setter (Auto-localization)
        Node(
            package="pinky_control",
            executable="initial_pose_setter",
            name="initial_pose_setter",
            parameters=[params_file],
            remappings=[
                ('tf', '/tf'),
                ('tf_static', '/tf_static')
            ],
            output="screen"
        ),
    ])
