from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="robot01"),
            DeclareLaunchArgument("robot_id", default_value="1"),
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
                "global_localization_service_name",
                default_value="reinitialize_global_localization",
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
            Node(
                package="office_robot_executor",
                executable="office_robot_executor_node",
                name="office_robot_executor",
                parameters=[
                    {
                        "robot_name": LaunchConfiguration("robot_name"),
                        "robot_id": LaunchConfiguration("robot_id"),
                        "mock_mode": LaunchConfiguration("mock_mode"),
                        "use_nav2": LaunchConfiguration("use_nav2"),
                        "execution_delay_sec": LaunchConfiguration("execution_delay_sec"),
                        "initial_battery": LaunchConfiguration("initial_battery"),
                        "battery_topic": LaunchConfiguration("battery_topic"),
                        "nav2_action_name": LaunchConfiguration("nav2_action_name"),
                        "frame_id": LaunchConfiguration("frame_id"),
                        "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
                        "goal_response_timeout_sec": LaunchConfiguration("goal_response_timeout_sec"),
                        "stop_cmd_vel_topic": LaunchConfiguration("stop_cmd_vel_topic"),
                        "stop_publish_count": LaunchConfiguration("stop_publish_count"),
                        "stop_publish_hz": LaunchConfiguration("stop_publish_hz"),
                        "safety_lock_topic": LaunchConfiguration("safety_lock_topic"),
                        "ai_link_topic": LaunchConfiguration("ai_link_topic"),
                        "include_ai_link_in_status": LaunchConfiguration("include_ai_link_in_status"),
                        "nav2_retry_attempts": LaunchConfiguration("nav2_retry_attempts"),
                        "nav2_retry_delay_sec": LaunchConfiguration("nav2_retry_delay_sec"),
                        "localization_required": LaunchConfiguration("localization_required"),
                        "amcl_pose_topic": LaunchConfiguration("amcl_pose_topic"),
                        "odom_topic": LaunchConfiguration("odom_topic"),
                        "amcl_pose_max_age_sec": LaunchConfiguration("amcl_pose_max_age_sec"),
                        "amcl_pose_stale_check_enabled": LaunchConfiguration(
                            "amcl_pose_stale_check_enabled"
                        ),
                        "amcl_covariance_xy_max": LaunchConfiguration("amcl_covariance_xy_max"),
                        "amcl_covariance_yaw_max": LaunchConfiguration("amcl_covariance_yaw_max"),
                        "localization_allow_degraded_covariance": LaunchConfiguration(
                            "localization_allow_degraded_covariance"
                        ),
                        "nav2_require_map_odom_tf": LaunchConfiguration("nav2_require_map_odom_tf"),
                        "nav2_tf_lookup_timeout_sec": LaunchConfiguration(
                            "nav2_tf_lookup_timeout_sec"
                        ),
                        "localization_recovery_enabled": LaunchConfiguration(
                            "localization_recovery_enabled"
                        ),
                        "localization_recovery_max_cycles": LaunchConfiguration(
                            "localization_recovery_max_cycles"
                        ),
                        "localization_recovery_cooldown_sec": LaunchConfiguration(
                            "localization_recovery_cooldown_sec"
                        ),
                        "localization_recovery_spin_duration_sec": LaunchConfiguration(
                            "localization_recovery_spin_duration_sec"
                        ),
                        "localization_recovery_spin_angular_speed": LaunchConfiguration(
                            "localization_recovery_spin_angular_speed"
                        ),
                        "global_localization_service_name": LaunchConfiguration(
                            "global_localization_service_name"
                        ),
                        "global_localization_wait_sec": LaunchConfiguration(
                            "global_localization_wait_sec"
                        ),
                        "amcl_nomotion_update_service_name": LaunchConfiguration(
                            "amcl_nomotion_update_service_name"
                        ),
                        "amcl_nomotion_wait_sec": LaunchConfiguration("amcl_nomotion_wait_sec"),
                        "nav2_lifecycle_check_enabled": LaunchConfiguration(
                            "nav2_lifecycle_check_enabled"
                        ),
                        "nav2_required_active_nodes": LaunchConfiguration(
                            "nav2_required_active_nodes"
                        ),
                        "nav2_lifecycle_get_state_timeout_sec": LaunchConfiguration(
                            "nav2_lifecycle_get_state_timeout_sec"
                        ),
                        "nav2_lifecycle_reactivate_enabled": LaunchConfiguration(
                            "nav2_lifecycle_reactivate_enabled"
                        ),
                        "nav2_lifecycle_manager_service_name": LaunchConfiguration(
                            "nav2_lifecycle_manager_service_name"
                        ),
                        "nav2_lifecycle_manager_wait_sec": LaunchConfiguration(
                            "nav2_lifecycle_manager_wait_sec"
                        ),
                        "localization_not_ready_event_name": LaunchConfiguration(
                            "localization_not_ready_event_name"
                        ),
                        "localization_not_ready_event_min_interval_sec": LaunchConfiguration(
                            "localization_not_ready_event_min_interval_sec"
                        ),
                        "enable_display": LaunchConfiguration("enable_display"),
                        "display_topic": LaunchConfiguration("display_topic"),
                        "guide_display_period_sec": LaunchConfiguration("guide_display_period_sec"),
                        "emit_command_received_event": LaunchConfiguration("emit_command_received_event"),
                        "qr_scan_local_enabled": LaunchConfiguration("qr_scan_local_enabled"),
                        "qr_scan_image_topic": LaunchConfiguration("qr_scan_image_topic"),
                        "qr_scan_timeout_sec": LaunchConfiguration("qr_scan_timeout_sec"),
                        "qr_scan_poll_period_sec": LaunchConfiguration("qr_scan_poll_period_sec"),
                        "qr_always_scan_enabled": LaunchConfiguration("qr_always_scan_enabled"),
                        "qr_always_scan_event_name": LaunchConfiguration("qr_always_scan_event_name"),
                        "qr_always_scan_poll_period_sec": LaunchConfiguration(
                            "qr_always_scan_poll_period_sec"
                        ),
                        "qr_always_scan_min_interval_sec": LaunchConfiguration(
                            "qr_always_scan_min_interval_sec"
                        ),
                    }
                ],
            ),
        ]
    )
