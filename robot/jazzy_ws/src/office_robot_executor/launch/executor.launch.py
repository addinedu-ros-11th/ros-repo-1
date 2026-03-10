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
            DeclareLaunchArgument("safety_state_topic", default_value="safety_state"),
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
            DeclareLaunchArgument("robot_yield_right_enabled", default_value="true"),
            DeclareLaunchArgument("robot_yield_right_offset_m", default_value="0.18"),
            DeclareLaunchArgument("robot_yield_right_forward_m", default_value="0.20"),
            DeclareLaunchArgument("robot_yield_right_cooldown_sec", default_value="5.0"),
            DeclareLaunchArgument(
                "robot_yield_right_max_attempts_per_action", default_value="1"
            ),
            DeclareLaunchArgument("obstacle_slow_enabled", default_value="true"),
            DeclareLaunchArgument(
                "obstacle_slow_controller_node", default_value="controller_server"
            ),
            DeclareLaunchArgument(
                "obstacle_slow_linear_vel_param",
                default_value="FollowPath.desired_linear_vel",
            ),
            DeclareLaunchArgument("obstacle_slow_linear_vel", default_value="0.06"),
            DeclareLaunchArgument("dynamic_nav_profile_enabled", default_value="false"),
            DeclareLaunchArgument("dynamic_nav_profile_scan_topic", default_value="/scan"),
            DeclareLaunchArgument("dynamic_nav_profile_robot_width_m", default_value="0.12"),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_width_enter_m", default_value="0.38"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_width_exit_m", default_value="0.32"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_forward_enter_m", default_value="0.55"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_forward_exit_m", default_value="0.40"
            ),
            DeclareLaunchArgument("dynamic_nav_profile_enter_samples", default_value="3"),
            DeclareLaunchArgument("dynamic_nav_profile_exit_samples", default_value="2"),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_lookahead_dist", default_value="0.32"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_min_lookahead_dist", default_value="0.15"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_max_lookahead_dist", default_value="0.40"
            ),
            DeclareLaunchArgument(
                "dynamic_nav_profile_wide_rotate_to_heading_min_angle",
                default_value="0.45",
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
                "global_localization_service_name",
                default_value="reinitialize_global_localization",
            ),
            DeclareLaunchArgument("global_localization_wait_sec", default_value="0.5"),
            DeclareLaunchArgument(
                "amcl_nomotion_update_service_name", default_value="request_nomotion_update"
            ),
            DeclareLaunchArgument("amcl_nomotion_wait_sec", default_value="0.3"),
            DeclareLaunchArgument("startup_initial_pose_enabled", default_value="false"),
            DeclareLaunchArgument("startup_initial_pose_topic", default_value="initialpose"),
            DeclareLaunchArgument("startup_initial_pose_delay_sec", default_value="1.0"),
            DeclareLaunchArgument("startup_initial_pose_x", default_value="0.0"),
            DeclareLaunchArgument("startup_initial_pose_y", default_value="0.0"),
            DeclareLaunchArgument("startup_initial_pose_yaw", default_value="0.0"),
            DeclareLaunchArgument("startup_initial_pose_covariance_xy", default_value="0.25"),
            DeclareLaunchArgument("startup_initial_pose_covariance_yaw", default_value="0.5"),
            DeclareLaunchArgument("manual_initial_pose_refine_enabled", default_value="false"),
            DeclareLaunchArgument("manual_initial_pose_refine_cooldown_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "manual_initial_pose_refine_ignore_self_sec", default_value="1.0"
            ),
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
            DeclareLaunchArgument("led_topic", default_value="led_command"),
            DeclareLaunchArgument("idle_led_off_enabled", default_value="true"),
            DeclareLaunchArgument("employee_verification_enabled", default_value="false"),
            DeclareLaunchArgument(
                "employee_verification_topic", default_value="employee_verification"
            ),
            DeclareLaunchArgument(
                "employee_verification_min_confidence", default_value="0.5"
            ),
            DeclareLaunchArgument(
                "employee_verification_greeting_text", default_value="Hello, employee"
            ),
            DeclareLaunchArgument(
                "employee_verification_feedback_hold_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "employee_verification_cooldown_sec", default_value="5.0"
            ),
            DeclareLaunchArgument(
                "employee_verification_qr_fallback_enabled", default_value="true"
            ),
            DeclareLaunchArgument(
                "employee_verification_qr_prompt_text",
                default_value="QR 코드를 인증해주세요",
            ),
            DeclareLaunchArgument(
                "employee_verification_qr_on_success_event",
                default_value="QR_SCANNED",
            ),
            DeclareLaunchArgument(
                "employee_verification_qr_purpose", default_value="VISITOR_SCAN"
            ),
            DeclareLaunchArgument("guide_display_period_sec", default_value="2.0"),
            DeclareLaunchArgument("emit_command_received_event", default_value="true"),
            DeclareLaunchArgument("qr_scan_local_enabled", default_value="true"),
            DeclareLaunchArgument("qr_scan_image_topic", default_value="/camera/image_raw/compressed"),
            DeclareLaunchArgument("qr_scan_timeout_sec", default_value="15.0"),
            DeclareLaunchArgument("qr_scan_poll_period_sec", default_value="0.2"),
            DeclareLaunchArgument("qr_scan_min_dwell_sec", default_value="1.5"),
            DeclareLaunchArgument("qr_scan_confirm_count", default_value="2"),
            DeclareLaunchArgument(
                "qr_scan_ignore_commands_while_active", default_value="true"
            ),
            DeclareLaunchArgument(
                "qr_scan_failure_display_text", default_value="인증 실패"
            ),
            DeclareLaunchArgument(
                "qr_scan_failure_display_hold_sec", default_value="2.0"
            ),
            DeclareLaunchArgument(
                "qr_scan_not_detected_display_text",
                default_value="QR 코드가 보이지 않습니다",
            ),
            DeclareLaunchArgument(
                "qr_scan_decode_failed_display_text",
                default_value="QR 코드를 읽지 못했습니다",
            ),
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
                        "safety_state_topic": LaunchConfiguration("safety_state_topic"),
                        "ai_link_topic": LaunchConfiguration("ai_link_topic"),
                        "include_ai_link_in_status": LaunchConfiguration("include_ai_link_in_status"),
                        "nav2_retry_attempts": LaunchConfiguration("nav2_retry_attempts"),
                        "nav2_retry_delay_sec": LaunchConfiguration("nav2_retry_delay_sec"),
                        "forward_first_enabled": LaunchConfiguration("forward_first_enabled"),
                        "forward_first_max_sec": LaunchConfiguration("forward_first_max_sec"),
                        "forward_first_stuck_timeout_sec": LaunchConfiguration(
                            "forward_first_stuck_timeout_sec"
                        ),
                        "forward_first_min_progress_m": LaunchConfiguration(
                            "forward_first_min_progress_m"
                        ),
                        "forward_first_controller_node": LaunchConfiguration(
                            "forward_first_controller_node"
                        ),
                        "forward_first_allow_reversing_param": LaunchConfiguration(
                            "forward_first_allow_reversing_param"
                        ),
                        "robot_yield_right_enabled": LaunchConfiguration(
                            "robot_yield_right_enabled"
                        ),
                        "robot_yield_right_offset_m": LaunchConfiguration(
                            "robot_yield_right_offset_m"
                        ),
                        "robot_yield_right_forward_m": LaunchConfiguration(
                            "robot_yield_right_forward_m"
                        ),
                        "robot_yield_right_cooldown_sec": LaunchConfiguration(
                            "robot_yield_right_cooldown_sec"
                        ),
                        "robot_yield_right_max_attempts_per_action": LaunchConfiguration(
                            "robot_yield_right_max_attempts_per_action"
                        ),
                        "obstacle_slow_enabled": LaunchConfiguration(
                            "obstacle_slow_enabled"
                        ),
                        "obstacle_slow_controller_node": LaunchConfiguration(
                            "obstacle_slow_controller_node"
                        ),
                        "obstacle_slow_linear_vel_param": LaunchConfiguration(
                            "obstacle_slow_linear_vel_param"
                        ),
                        "obstacle_slow_linear_vel": LaunchConfiguration(
                            "obstacle_slow_linear_vel"
                        ),
                        "dynamic_nav_profile_enabled": LaunchConfiguration(
                            "dynamic_nav_profile_enabled"
                        ),
                        "dynamic_nav_profile_scan_topic": LaunchConfiguration(
                            "dynamic_nav_profile_scan_topic"
                        ),
                        "dynamic_nav_profile_robot_width_m": LaunchConfiguration(
                            "dynamic_nav_profile_robot_width_m"
                        ),
                        "dynamic_nav_profile_wide_width_enter_m": LaunchConfiguration(
                            "dynamic_nav_profile_wide_width_enter_m"
                        ),
                        "dynamic_nav_profile_wide_width_exit_m": LaunchConfiguration(
                            "dynamic_nav_profile_wide_width_exit_m"
                        ),
                        "dynamic_nav_profile_forward_enter_m": LaunchConfiguration(
                            "dynamic_nav_profile_forward_enter_m"
                        ),
                        "dynamic_nav_profile_forward_exit_m": LaunchConfiguration(
                            "dynamic_nav_profile_forward_exit_m"
                        ),
                        "dynamic_nav_profile_enter_samples": LaunchConfiguration(
                            "dynamic_nav_profile_enter_samples"
                        ),
                        "dynamic_nav_profile_exit_samples": LaunchConfiguration(
                            "dynamic_nav_profile_exit_samples"
                        ),
                        "dynamic_nav_profile_wide_lookahead_dist": LaunchConfiguration(
                            "dynamic_nav_profile_wide_lookahead_dist"
                        ),
                        "dynamic_nav_profile_wide_min_lookahead_dist": LaunchConfiguration(
                            "dynamic_nav_profile_wide_min_lookahead_dist"
                        ),
                        "dynamic_nav_profile_wide_max_lookahead_dist": LaunchConfiguration(
                            "dynamic_nav_profile_wide_max_lookahead_dist"
                        ),
                        "dynamic_nav_profile_wide_rotate_to_heading_min_angle": LaunchConfiguration(
                            "dynamic_nav_profile_wide_rotate_to_heading_min_angle"
                        ),
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
                        "startup_initial_pose_enabled": LaunchConfiguration(
                            "startup_initial_pose_enabled"
                        ),
                        "startup_initial_pose_topic": LaunchConfiguration(
                            "startup_initial_pose_topic"
                        ),
                        "startup_initial_pose_delay_sec": LaunchConfiguration(
                            "startup_initial_pose_delay_sec"
                        ),
                        "startup_initial_pose_x": LaunchConfiguration("startup_initial_pose_x"),
                        "startup_initial_pose_y": LaunchConfiguration("startup_initial_pose_y"),
                        "startup_initial_pose_yaw": LaunchConfiguration(
                            "startup_initial_pose_yaw"
                        ),
                        "startup_initial_pose_covariance_xy": LaunchConfiguration(
                            "startup_initial_pose_covariance_xy"
                        ),
                        "startup_initial_pose_covariance_yaw": LaunchConfiguration(
                            "startup_initial_pose_covariance_yaw"
                        ),
                        "manual_initial_pose_refine_enabled": LaunchConfiguration(
                            "manual_initial_pose_refine_enabled"
                        ),
                        "manual_initial_pose_refine_cooldown_sec": LaunchConfiguration(
                            "manual_initial_pose_refine_cooldown_sec"
                        ),
                        "manual_initial_pose_refine_ignore_self_sec": LaunchConfiguration(
                            "manual_initial_pose_refine_ignore_self_sec"
                        ),
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
                        "led_topic": LaunchConfiguration("led_topic"),
                        "idle_led_off_enabled": LaunchConfiguration("idle_led_off_enabled"),
                        "employee_verification_enabled": LaunchConfiguration(
                            "employee_verification_enabled"
                        ),
                        "employee_verification_topic": LaunchConfiguration(
                            "employee_verification_topic"
                        ),
                        "employee_verification_min_confidence": LaunchConfiguration(
                            "employee_verification_min_confidence"
                        ),
                        "employee_verification_greeting_text": LaunchConfiguration(
                            "employee_verification_greeting_text"
                        ),
                        "employee_verification_feedback_hold_sec": LaunchConfiguration(
                            "employee_verification_feedback_hold_sec"
                        ),
                        "employee_verification_cooldown_sec": LaunchConfiguration(
                            "employee_verification_cooldown_sec"
                        ),
                        "employee_verification_qr_fallback_enabled": LaunchConfiguration(
                            "employee_verification_qr_fallback_enabled"
                        ),
                        "employee_verification_qr_prompt_text": LaunchConfiguration(
                            "employee_verification_qr_prompt_text"
                        ),
                        "employee_verification_qr_on_success_event": LaunchConfiguration(
                            "employee_verification_qr_on_success_event"
                        ),
                        "employee_verification_qr_purpose": LaunchConfiguration(
                            "employee_verification_qr_purpose"
                        ),
                        "guide_display_period_sec": LaunchConfiguration("guide_display_period_sec"),
                        "emit_command_received_event": LaunchConfiguration("emit_command_received_event"),
                        "qr_scan_local_enabled": LaunchConfiguration("qr_scan_local_enabled"),
                        "qr_scan_image_topic": LaunchConfiguration("qr_scan_image_topic"),
                        "qr_scan_timeout_sec": LaunchConfiguration("qr_scan_timeout_sec"),
                        "qr_scan_poll_period_sec": LaunchConfiguration("qr_scan_poll_period_sec"),
                        "qr_scan_min_dwell_sec": LaunchConfiguration(
                            "qr_scan_min_dwell_sec"
                        ),
                        "qr_scan_confirm_count": LaunchConfiguration(
                            "qr_scan_confirm_count"
                        ),
                        "qr_scan_ignore_commands_while_active": LaunchConfiguration(
                            "qr_scan_ignore_commands_while_active"
                        ),
                        "qr_scan_failure_display_text": LaunchConfiguration(
                            "qr_scan_failure_display_text"
                        ),
                        "qr_scan_failure_display_hold_sec": LaunchConfiguration(
                            "qr_scan_failure_display_hold_sec"
                        ),
                        "qr_scan_not_detected_display_text": LaunchConfiguration(
                            "qr_scan_not_detected_display_text"
                        ),
                        "qr_scan_decode_failed_display_text": LaunchConfiguration(
                            "qr_scan_decode_failed_display_text"
                        ),
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
