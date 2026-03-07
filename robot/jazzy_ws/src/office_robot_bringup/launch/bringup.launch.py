from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    robot_ns = LaunchConfiguration("robot_ns")
    enable_rosbridge = LaunchConfiguration("enable_rosbridge")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    robot_id = LaunchConfiguration("robot_id")
    use_nav2 = LaunchConfiguration("use_nav2")
    nav2_action_name = LaunchConfiguration("nav2_action_name")
    initial_battery = LaunchConfiguration("initial_battery")
    battery_topic = LaunchConfiguration("battery_topic")
    frame_id = LaunchConfiguration("frame_id")
    goal_timeout_sec = LaunchConfiguration("goal_timeout_sec")
    goal_response_timeout_sec = LaunchConfiguration("goal_response_timeout_sec")
    stop_cmd_vel_topic = LaunchConfiguration("stop_cmd_vel_topic")
    stop_publish_count = LaunchConfiguration("stop_publish_count")
    stop_publish_hz = LaunchConfiguration("stop_publish_hz")
    safety_lock_topic = LaunchConfiguration("safety_lock_topic")
    ai_link_topic = LaunchConfiguration("ai_link_topic")
    include_ai_link_in_status = LaunchConfiguration("include_ai_link_in_status")
    nav2_retry_attempts = LaunchConfiguration("nav2_retry_attempts")
    nav2_retry_delay_sec = LaunchConfiguration("nav2_retry_delay_sec")
    forward_first_enabled = LaunchConfiguration("forward_first_enabled")
    forward_first_max_sec = LaunchConfiguration("forward_first_max_sec")
    forward_first_stuck_timeout_sec = LaunchConfiguration("forward_first_stuck_timeout_sec")
    forward_first_min_progress_m = LaunchConfiguration("forward_first_min_progress_m")
    forward_first_controller_node = LaunchConfiguration("forward_first_controller_node")
    forward_first_allow_reversing_param = LaunchConfiguration(
        "forward_first_allow_reversing_param"
    )
    localization_required = LaunchConfiguration("localization_required")
    amcl_pose_topic = LaunchConfiguration("amcl_pose_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    amcl_pose_max_age_sec = LaunchConfiguration("amcl_pose_max_age_sec")
    amcl_pose_stale_check_enabled = LaunchConfiguration("amcl_pose_stale_check_enabled")
    amcl_covariance_xy_max = LaunchConfiguration("amcl_covariance_xy_max")
    amcl_covariance_yaw_max = LaunchConfiguration("amcl_covariance_yaw_max")
    localization_allow_degraded_covariance = LaunchConfiguration(
        "localization_allow_degraded_covariance"
    )
    nav2_require_map_odom_tf = LaunchConfiguration("nav2_require_map_odom_tf")
    nav2_tf_lookup_timeout_sec = LaunchConfiguration("nav2_tf_lookup_timeout_sec")
    localization_recovery_enabled = LaunchConfiguration("localization_recovery_enabled")
    localization_recovery_max_cycles = LaunchConfiguration("localization_recovery_max_cycles")
    localization_recovery_cooldown_sec = LaunchConfiguration("localization_recovery_cooldown_sec")
    localization_recovery_spin_duration_sec = LaunchConfiguration(
        "localization_recovery_spin_duration_sec"
    )
    localization_recovery_spin_angular_speed = LaunchConfiguration(
        "localization_recovery_spin_angular_speed"
    )
    global_localization_service_name = LaunchConfiguration("global_localization_service_name")
    global_localization_wait_sec = LaunchConfiguration("global_localization_wait_sec")
    amcl_nomotion_update_service_name = LaunchConfiguration("amcl_nomotion_update_service_name")
    amcl_nomotion_wait_sec = LaunchConfiguration("amcl_nomotion_wait_sec")
    nav2_lifecycle_check_enabled = LaunchConfiguration("nav2_lifecycle_check_enabled")
    nav2_required_active_nodes = LaunchConfiguration("nav2_required_active_nodes")
    nav2_lifecycle_get_state_timeout_sec = LaunchConfiguration(
        "nav2_lifecycle_get_state_timeout_sec"
    )
    nav2_lifecycle_reactivate_enabled = LaunchConfiguration("nav2_lifecycle_reactivate_enabled")
    nav2_lifecycle_manager_service_name = LaunchConfiguration("nav2_lifecycle_manager_service_name")
    nav2_lifecycle_manager_wait_sec = LaunchConfiguration("nav2_lifecycle_manager_wait_sec")
    localization_not_ready_event_name = LaunchConfiguration("localization_not_ready_event_name")
    localization_not_ready_event_min_interval_sec = LaunchConfiguration(
        "localization_not_ready_event_min_interval_sec"
    )
    enable_display = LaunchConfiguration("enable_display")
    display_topic = LaunchConfiguration("display_topic")
    guide_display_period_sec = LaunchConfiguration("guide_display_period_sec")
    emit_command_received_event = LaunchConfiguration("emit_command_received_event")
    qr_scan_local_enabled = LaunchConfiguration("qr_scan_local_enabled")
    qr_scan_image_topic = LaunchConfiguration("qr_scan_image_topic")
    qr_scan_timeout_sec = LaunchConfiguration("qr_scan_timeout_sec")
    qr_scan_poll_period_sec = LaunchConfiguration("qr_scan_poll_period_sec")
    qr_always_scan_enabled = LaunchConfiguration("qr_always_scan_enabled")
    qr_always_scan_event_name = LaunchConfiguration("qr_always_scan_event_name")
    qr_always_scan_poll_period_sec = LaunchConfiguration("qr_always_scan_poll_period_sec")
    qr_always_scan_min_interval_sec = LaunchConfiguration("qr_always_scan_min_interval_sec")
    safety_stop_publish_hz = LaunchConfiguration("safety_stop_publish_hz")
    safety_lock_keepalive_hz = LaunchConfiguration("safety_lock_keepalive_hz")
    safety_stop_publish_count = LaunchConfiguration("safety_stop_publish_count")
    obstacle_enabled = LaunchConfiguration("obstacle_enabled")
    obstacle_topic = LaunchConfiguration("obstacle_topic")
    safety_params_file = PathJoinSubstitution(
        [FindPackageShare("office_robot_bringup"), "config", "safety.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_ns", default_value="robot01"),
            DeclareLaunchArgument("robot_id", default_value="1"),
            DeclareLaunchArgument("enable_rosbridge", default_value="true"),
            DeclareLaunchArgument("rosbridge_port", default_value="9090"),
            DeclareLaunchArgument("mock_mode", default_value="false"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("execution_delay_sec", default_value="1.5"),
            DeclareLaunchArgument("initial_battery", default_value="0.0"),
            DeclareLaunchArgument("battery_topic", default_value="/battery/present"),
            DeclareLaunchArgument("nav2_action_name", default_value="navigate_to_pose"),
            DeclareLaunchArgument("frame_id", default_value="map"),
            DeclareLaunchArgument("goal_timeout_sec", default_value="60.0"),
            DeclareLaunchArgument("goal_response_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("stop_cmd_vel_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("stop_publish_count", default_value="10"),
            DeclareLaunchArgument("stop_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("safety_lock_topic", default_value="safety_lock"),
            DeclareLaunchArgument("ai_link_topic", default_value="ai_link"),
            DeclareLaunchArgument("include_ai_link_in_status", default_value="true"),
            DeclareLaunchArgument("nav2_retry_attempts", default_value="8"),
            DeclareLaunchArgument("nav2_retry_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("forward_first_enabled", default_value="true"),
            DeclareLaunchArgument("forward_first_max_sec", default_value="5.0"),
            DeclareLaunchArgument("forward_first_stuck_timeout_sec", default_value="2.5"),
            DeclareLaunchArgument("forward_first_min_progress_m", default_value="0.08"),
            DeclareLaunchArgument("forward_first_controller_node", default_value="controller_server"),
            DeclareLaunchArgument(
                "forward_first_allow_reversing_param",
                default_value="FollowPath.allow_reversing",
            ),
            DeclareLaunchArgument("localization_required", default_value="true"),
            DeclareLaunchArgument("amcl_pose_topic", default_value="amcl_pose"),
            DeclareLaunchArgument("odom_topic", default_value="/odom"),
            DeclareLaunchArgument("amcl_pose_max_age_sec", default_value="3.0"),
            DeclareLaunchArgument("amcl_pose_stale_check_enabled", default_value="false"),
            DeclareLaunchArgument("amcl_covariance_xy_max", default_value="0.8"),
            DeclareLaunchArgument("amcl_covariance_yaw_max", default_value="6.0"),
            DeclareLaunchArgument(
                "localization_allow_degraded_covariance", default_value="true"
            ),
            DeclareLaunchArgument("nav2_require_map_odom_tf", default_value="true"),
            DeclareLaunchArgument("nav2_tf_lookup_timeout_sec", default_value="0.05"),
            DeclareLaunchArgument("localization_recovery_enabled", default_value="true"),
            DeclareLaunchArgument("localization_recovery_max_cycles", default_value="2"),
            DeclareLaunchArgument("localization_recovery_cooldown_sec", default_value="8.0"),
            DeclareLaunchArgument("localization_recovery_spin_duration_sec", default_value="8.0"),
            DeclareLaunchArgument("localization_recovery_spin_angular_speed", default_value="0.8"),
            DeclareLaunchArgument(
                "global_localization_service_name", default_value="reinitialize_global_localization"
            ),
            DeclareLaunchArgument("global_localization_wait_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "amcl_nomotion_update_service_name", default_value="request_nomotion_update"
            ),
            DeclareLaunchArgument("amcl_nomotion_wait_sec", default_value="0.3"),
            DeclareLaunchArgument("nav2_lifecycle_check_enabled", default_value="true"),
            DeclareLaunchArgument(
                "nav2_required_active_nodes",
                default_value="planner_server,controller_server,bt_navigator,behavior_server",
            ),
            DeclareLaunchArgument("nav2_lifecycle_get_state_timeout_sec", default_value="0.15"),
            DeclareLaunchArgument("nav2_lifecycle_reactivate_enabled", default_value="true"),
            DeclareLaunchArgument(
                "nav2_lifecycle_manager_service_name",
                default_value="lifecycle_manager_navigation/manage_nodes",
            ),
            DeclareLaunchArgument("nav2_lifecycle_manager_wait_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "localization_not_ready_event_name", default_value="LOCALIZATION_NOT_READY"
            ),
            DeclareLaunchArgument(
                "localization_not_ready_event_min_interval_sec", default_value="2.0"
            ),
            DeclareLaunchArgument("enable_display", default_value="true"),
            DeclareLaunchArgument("display_topic", default_value="display"),
            DeclareLaunchArgument("guide_display_period_sec", default_value="2.0"),
            DeclareLaunchArgument("emit_command_received_event", default_value="true"),
            DeclareLaunchArgument("qr_scan_local_enabled", default_value="true"),
            DeclareLaunchArgument("qr_scan_image_topic", default_value="/camera/image_raw/compressed"),
            DeclareLaunchArgument("qr_scan_timeout_sec", default_value="8.0"),
            DeclareLaunchArgument("qr_scan_poll_period_sec", default_value="0.2"),
            DeclareLaunchArgument("qr_always_scan_enabled", default_value="false"),
            DeclareLaunchArgument("qr_always_scan_event_name", default_value="QR_DETECTED"),
            DeclareLaunchArgument("qr_always_scan_poll_period_sec", default_value="0.5"),
            DeclareLaunchArgument("qr_always_scan_min_interval_sec", default_value="3.0"),
            DeclareLaunchArgument("safety_stop_publish_hz", default_value="20.0"),
            DeclareLaunchArgument("safety_lock_keepalive_hz", default_value="2.0"),
            DeclareLaunchArgument("safety_stop_publish_count", default_value="10"),
            DeclareLaunchArgument("obstacle_enabled", default_value="false"),
            DeclareLaunchArgument("obstacle_topic", default_value="obstacles"),
            GroupAction(
                [
                    PushRosNamespace(robot_ns),
                    Node(
                        package="office_robot_safety",
                        executable="office_robot_safety_node",
                        name="office_robot_safety",
                        parameters=[
                            safety_params_file,
                            {
                                "robot_name": robot_ns,
                                "robot_id": robot_id,
                                "cmd_topic": "commands",
                                "lock_topic": safety_lock_topic,
                                "stop_cmd_vel_topic": stop_cmd_vel_topic,
                                "stop_publish_hz": safety_stop_publish_hz,
                                "lock_keepalive_hz": safety_lock_keepalive_hz,
                                "stop_publish_count": safety_stop_publish_count,
                                "obstacle_enabled": obstacle_enabled,
                                "obstacle_topic": obstacle_topic,
                            },
                        ],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("office_robot_executor"),
                                    "launch",
                                    "executor.launch.py",
                                ]
                            )
                        ),
                        launch_arguments={
                            "robot_name": robot_ns,
                            "robot_id": robot_id,
                            "mock_mode": LaunchConfiguration("mock_mode"),
                            "use_nav2": use_nav2,
                            "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                            "initial_battery": initial_battery,
                            "battery_topic": battery_topic,
                            "nav2_action_name": nav2_action_name,
                            "frame_id": frame_id,
                            "goal_timeout_sec": goal_timeout_sec,
                            "goal_response_timeout_sec": goal_response_timeout_sec,
                            "stop_cmd_vel_topic": stop_cmd_vel_topic,
                            "stop_publish_count": stop_publish_count,
                            "stop_publish_hz": stop_publish_hz,
                            "safety_lock_topic": safety_lock_topic,
                            "ai_link_topic": ai_link_topic,
                            "include_ai_link_in_status": include_ai_link_in_status,
                            "nav2_retry_attempts": nav2_retry_attempts,
                            "nav2_retry_delay_sec": nav2_retry_delay_sec,
                            "forward_first_enabled": forward_first_enabled,
                            "forward_first_max_sec": forward_first_max_sec,
                            "forward_first_stuck_timeout_sec": forward_first_stuck_timeout_sec,
                            "forward_first_min_progress_m": forward_first_min_progress_m,
                            "forward_first_controller_node": forward_first_controller_node,
                            "forward_first_allow_reversing_param": forward_first_allow_reversing_param,
                            "localization_required": localization_required,
                            "amcl_pose_topic": amcl_pose_topic,
                            "odom_topic": odom_topic,
                            "amcl_pose_max_age_sec": amcl_pose_max_age_sec,
                            "amcl_pose_stale_check_enabled": amcl_pose_stale_check_enabled,
                            "amcl_covariance_xy_max": amcl_covariance_xy_max,
                            "amcl_covariance_yaw_max": amcl_covariance_yaw_max,
                            "localization_allow_degraded_covariance": localization_allow_degraded_covariance,
                            "nav2_require_map_odom_tf": nav2_require_map_odom_tf,
                            "nav2_tf_lookup_timeout_sec": nav2_tf_lookup_timeout_sec,
                            "localization_recovery_enabled": localization_recovery_enabled,
                            "localization_recovery_max_cycles": localization_recovery_max_cycles,
                            "localization_recovery_cooldown_sec": localization_recovery_cooldown_sec,
                            "localization_recovery_spin_duration_sec": localization_recovery_spin_duration_sec,
                            "localization_recovery_spin_angular_speed": localization_recovery_spin_angular_speed,
                            "global_localization_service_name": global_localization_service_name,
                            "global_localization_wait_sec": global_localization_wait_sec,
                            "amcl_nomotion_update_service_name": amcl_nomotion_update_service_name,
                            "amcl_nomotion_wait_sec": amcl_nomotion_wait_sec,
                            "nav2_lifecycle_check_enabled": nav2_lifecycle_check_enabled,
                            "nav2_required_active_nodes": nav2_required_active_nodes,
                            "nav2_lifecycle_get_state_timeout_sec": nav2_lifecycle_get_state_timeout_sec,
                            "nav2_lifecycle_reactivate_enabled": nav2_lifecycle_reactivate_enabled,
                            "nav2_lifecycle_manager_service_name": nav2_lifecycle_manager_service_name,
                            "nav2_lifecycle_manager_wait_sec": nav2_lifecycle_manager_wait_sec,
                            "localization_not_ready_event_name": localization_not_ready_event_name,
                            "localization_not_ready_event_min_interval_sec": localization_not_ready_event_min_interval_sec,
                            "enable_display": enable_display,
                            "display_topic": display_topic,
                            "guide_display_period_sec": guide_display_period_sec,
                            "emit_command_received_event": emit_command_received_event,
                            "qr_scan_local_enabled": qr_scan_local_enabled,
                            "qr_scan_image_topic": qr_scan_image_topic,
                            "qr_scan_timeout_sec": qr_scan_timeout_sec,
                            "qr_scan_poll_period_sec": qr_scan_poll_period_sec,
                            "qr_always_scan_enabled": qr_always_scan_enabled,
                            "qr_always_scan_event_name": qr_always_scan_event_name,
                            "qr_always_scan_poll_period_sec": qr_always_scan_poll_period_sec,
                            "qr_always_scan_min_interval_sec": qr_always_scan_min_interval_sec,
                        }.items(),
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("office_robot_bridge"),
                                    "launch",
                                    "rosbridge.launch.py",
                                ]
                            )
                        ),
                        condition=IfCondition(enable_rosbridge),
                        launch_arguments={
                            "robot_ns": robot_ns,
                            "enable_rosbridge": enable_rosbridge,
                            "port": rosbridge_port,
                        }.items(),
                    ),
                ],
            ),
        ]
    )
