import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, Float32, String

try:
    from std_srvs.srv import Empty
except Exception:  # pragma: no cover - runtime environment dependent
    Empty = None

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - runtime environment dependent
    NavigateToPose = None

try:
    from tf2_ros import Buffer, TransformException, TransformListener
except Exception:  # pragma: no cover - runtime environment dependent
    Buffer = None
    TransformListener = None
    TransformException = Exception


class OfficeRobotExecutor(Node):
    """
    Action sequence executor.
    Subscribes: commands (std_msgs/String), safety_lock (std_msgs/Bool), ai_link (std_msgs/Bool)
    Publishes: status/event/display (std_msgs/String), cmd_vel stop burst (geometry_msgs/Twist)
    """

    def __init__(self) -> None:
        super().__init__("office_robot_executor")

        self.declare_parameter("robot_name", "robot")
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("mock_mode", False)
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("execution_delay_sec", 1.5)
        self.declare_parameter("initial_battery", 0.0)
        self.declare_parameter("battery_topic", "/battery/present")
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("goal_response_timeout_sec", 8.0)
        self.declare_parameter("stop_cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("safety_lock_topic", "safety_lock")
        self.declare_parameter("ai_link_topic", "ai_link")
        self.declare_parameter("include_ai_link_in_status", True)
        self.declare_parameter("nav2_success_status_code", 4)
        self.declare_parameter("nav2_feedback_log_period_sec", 1.5)
        self.declare_parameter("nav2_abort_as_success_enabled", False)
        self.declare_parameter("nav2_abort_success_distance_tolerance", 0.35)
        self.declare_parameter("nav2_abort_success_error_codes", "103,106,208")
        self.declare_parameter("nav2_retry_attempts", 1)
        self.declare_parameter("nav2_retry_delay_sec", 1.0)
        self.declare_parameter("localization_required", True)
        self.declare_parameter("amcl_pose_topic", "amcl_pose")
        self.declare_parameter("amcl_pose_max_age_sec", 3.0)
        self.declare_parameter("amcl_pose_stale_check_enabled", False)
        self.declare_parameter("amcl_covariance_xy_max", 0.8)
        self.declare_parameter("amcl_covariance_yaw_max", 6.0)
        self.declare_parameter("localization_allow_degraded_covariance", True)
        self.declare_parameter("nav2_require_map_odom_tf", True)
        self.declare_parameter("nav2_tf_lookup_timeout_sec", 0.05)
        self.declare_parameter("localization_recovery_enabled", True)
        self.declare_parameter("localization_recovery_max_cycles", 2)
        self.declare_parameter("localization_recovery_cooldown_sec", 8.0)
        self.declare_parameter("localization_recovery_spin_duration_sec", 4.0)
        self.declare_parameter("localization_recovery_spin_angular_speed", 0.8)
        self.declare_parameter(
            "global_localization_service_name", "reinitialize_global_localization"
        )
        self.declare_parameter("global_localization_wait_sec", 0.5)
        self.declare_parameter("amcl_nomotion_update_service_name", "request_nomotion_update")
        self.declare_parameter("amcl_nomotion_wait_sec", 0.3)
        self.declare_parameter("enable_display", True)
        self.declare_parameter("display_topic", "display")
        self.declare_parameter("guide_display_period_sec", 2.0)
        self.declare_parameter("emit_command_received_event", True)
        self.declare_parameter("default_goto_success_event", "ARRIVED_AT_DESTINATION")

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.mock_mode = self.get_parameter("mock_mode").get_parameter_value().bool_value
        self.use_nav2 = self.get_parameter("use_nav2").get_parameter_value().bool_value
        self.execution_delay_sec = (
            self.get_parameter("execution_delay_sec").get_parameter_value().double_value
        )
        self.battery = (
            self.get_parameter("initial_battery").get_parameter_value().double_value
        )
        self.nav2_action_name = (
            self.get_parameter("nav2_action_name").get_parameter_value().string_value
        )
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.goal_timeout_sec = (
            self.get_parameter("goal_timeout_sec").get_parameter_value().double_value
        )
        self.goal_response_timeout_sec = (
            self.get_parameter("goal_response_timeout_sec").get_parameter_value().double_value
        )
        self.stop_cmd_vel_topic = (
            self.get_parameter("stop_cmd_vel_topic").get_parameter_value().string_value
        )
        self.stop_publish_count = (
            self.get_parameter("stop_publish_count").get_parameter_value().integer_value
        )
        self.stop_publish_hz = (
            self.get_parameter("stop_publish_hz").get_parameter_value().double_value
        )
        self.safety_lock_topic = (
            self.get_parameter("safety_lock_topic").get_parameter_value().string_value
        )
        self.ai_link_topic = self.get_parameter("ai_link_topic").get_parameter_value().string_value
        self.battery_topic = self.get_parameter("battery_topic").get_parameter_value().string_value
        self.include_ai_link_in_status = (
            self.get_parameter("include_ai_link_in_status").get_parameter_value().bool_value
        )
        self.nav2_success_status_code = (
            self.get_parameter("nav2_success_status_code").get_parameter_value().integer_value
        )
        self.nav2_feedback_log_period_sec = (
            self.get_parameter("nav2_feedback_log_period_sec").get_parameter_value().double_value
        )
        self.nav2_abort_as_success_enabled = (
            self.get_parameter("nav2_abort_as_success_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.nav2_abort_success_distance_tolerance = (
            self.get_parameter("nav2_abort_success_distance_tolerance")
            .get_parameter_value()
            .double_value
        )
        self.nav2_abort_success_error_codes = self._parse_int_set(
            self.get_parameter("nav2_abort_success_error_codes")
            .get_parameter_value()
            .string_value
        )
        self.nav2_retry_attempts = max(
            0, self.get_parameter("nav2_retry_attempts").get_parameter_value().integer_value
        )
        self.nav2_retry_delay_sec = max(
            0.2, self.get_parameter("nav2_retry_delay_sec").get_parameter_value().double_value
        )
        self.localization_required = (
            self.get_parameter("localization_required").get_parameter_value().bool_value
        )
        self.amcl_pose_topic = (
            self.get_parameter("amcl_pose_topic").get_parameter_value().string_value
        )
        self.amcl_pose_max_age_sec = max(
            0.5, self.get_parameter("amcl_pose_max_age_sec").get_parameter_value().double_value
        )
        self.amcl_pose_stale_check_enabled = (
            self.get_parameter("amcl_pose_stale_check_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.amcl_covariance_xy_max = max(
            0.01, self.get_parameter("amcl_covariance_xy_max").get_parameter_value().double_value
        )
        self.amcl_covariance_yaw_max = max(
            0.01, self.get_parameter("amcl_covariance_yaw_max").get_parameter_value().double_value
        )
        self.localization_allow_degraded_covariance = (
            self.get_parameter("localization_allow_degraded_covariance")
            .get_parameter_value()
            .bool_value
        )
        self.nav2_require_map_odom_tf = (
            self.get_parameter("nav2_require_map_odom_tf").get_parameter_value().bool_value
        )
        self.nav2_tf_lookup_timeout_sec = max(
            0.01,
            self.get_parameter("nav2_tf_lookup_timeout_sec").get_parameter_value().double_value,
        )
        self.localization_recovery_enabled = (
            self.get_parameter("localization_recovery_enabled")
            .get_parameter_value()
            .bool_value
        )
        self.localization_recovery_max_cycles = max(
            0,
            self.get_parameter("localization_recovery_max_cycles")
            .get_parameter_value()
            .integer_value,
        )
        self.localization_recovery_cooldown_sec = max(
            0.0,
            self.get_parameter("localization_recovery_cooldown_sec")
            .get_parameter_value()
            .double_value,
        )
        self.localization_recovery_spin_duration_sec = max(
            0.0,
            self.get_parameter("localization_recovery_spin_duration_sec")
            .get_parameter_value()
            .double_value,
        )
        self.localization_recovery_spin_angular_speed = float(
            self.get_parameter("localization_recovery_spin_angular_speed")
            .get_parameter_value()
            .double_value
        )
        self.global_localization_service_name = (
            self.get_parameter("global_localization_service_name")
            .get_parameter_value()
            .string_value
        )
        self.global_localization_wait_sec = max(
            0.1,
            self.get_parameter("global_localization_wait_sec")
            .get_parameter_value()
            .double_value,
        )
        self.amcl_nomotion_update_service_name = (
            self.get_parameter("amcl_nomotion_update_service_name")
            .get_parameter_value()
            .string_value
        )
        self.amcl_nomotion_wait_sec = max(
            0.1,
            self.get_parameter("amcl_nomotion_wait_sec")
            .get_parameter_value()
            .double_value,
        )
        self.enable_display = (
            self.get_parameter("enable_display").get_parameter_value().bool_value
        )
        self.display_topic = self.get_parameter("display_topic").get_parameter_value().string_value
        self.guide_display_period_sec = (
            self.get_parameter("guide_display_period_sec").get_parameter_value().double_value
        )
        self.emit_command_received_event = (
            self.get_parameter("emit_command_received_event").get_parameter_value().bool_value
        )
        self.default_goto_success_event = (
            self.get_parameter("default_goto_success_event")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self.location: Tuple[float, float] = (0.0, 0.0)
        self.current_status = "IDLE"
        self._current_task_id: Optional[Any] = None
        self._action_queue: List[Dict[str, Any]] = []
        self._action_timer = None
        self._current_action: Optional[Dict[str, Any]] = None
        self._current_goal_handle = None
        self._goal_started_at: Optional[float] = None
        self._goal_response_started_at: Optional[float] = None
        self._timeout_timer = None
        self._goal_target: Optional[Dict[str, Any]] = None
        self._last_nav_feedback: Optional[Dict[str, Any]] = None
        self._last_feedback_log_at = 0.0
        self._cancel_requested = False
        self._cancel_reason: Optional[str] = None
        self._guide_display_timer = None
        self._guide_display_toggle = False
        self._safety_locked = False
        self._ai_link_alive: Optional[bool] = None
        self._battery_received = False
        self._nav_retry_timer = None
        self._nav_retry_attempt_count = 0
        self._last_amcl_pose_mono = 0.0
        self._last_amcl_cov_xy: Optional[float] = None
        self._last_amcl_cov_yaw: Optional[float] = None
        self._localization_recovery_timer = None
        self._localization_recovery_active = False
        self._localization_recovery_until_mono = 0.0
        self._localization_recovery_cycle_count = 0
        self._last_localization_recovery_mono = 0.0

        self.command_sub = self.create_subscription(String, "commands", self._on_commands, 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)
        self.display_pub = self.create_publisher(String, self.display_topic, 10)
        self.stop_pub = self.create_publisher(Twist, self.stop_cmd_vel_topic, 10)
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.amcl_pose_topic, self._on_amcl_pose, 10
        )

        safety_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.safety_sub = self.create_subscription(
            Bool, self.safety_lock_topic, self._on_safety_lock, safety_qos
        )
        self.ai_link_sub = self.create_subscription(
            Bool, self.ai_link_topic, self._on_ai_link, safety_qos
        )
        self.battery_sub = self.create_subscription(
            Float32, self.battery_topic, self._on_battery, 10
        )

        self.nav_client = None
        if self.use_nav2 and not self.mock_mode:
            if NavigateToPose is None:
                self.get_logger().error("nav2_msgs not available; real GOTO execution disabled.")
            else:
                self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)
        self.tf_buffer = None
        self.tf_listener = None
        if Buffer is not None and TransformListener is not None:
            self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        elif self.nav2_require_map_odom_tf:
            self.get_logger().warn("tf2_ros unavailable; map->odom readiness check will be skipped.")
        self.global_localization_client = None
        self.amcl_nomotion_client = None
        if self.localization_recovery_enabled:
            if Empty is None:
                self.get_logger().warn(
                    "std_srvs/Empty unavailable; global localization reset will be skipped."
                )
            else:
                self.global_localization_client = self.create_client(
                    Empty, self.global_localization_service_name
                )
                self.amcl_nomotion_client = self.create_client(
                    Empty, self.amcl_nomotion_update_service_name
                )

        self.status_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info(
            f"Executor ready (robot_name={self.robot_name}, mock_mode={self.mock_mode}, use_nav2={self.use_nav2}, "
            f"safety_lock_topic={self.safety_lock_topic}, ai_link_topic={self.ai_link_topic}, "
            f"display_topic={self.display_topic}, command_received_event={self.emit_command_received_event}, "
            f"nav_retry_attempts={self.nav2_retry_attempts}, localization_required={self.localization_required}, "
            f"amcl_pose_topic={self.amcl_pose_topic}, amcl_stale_check={self.amcl_pose_stale_check_enabled}, "
            f"allow_degraded_cov={self.localization_allow_degraded_covariance}, "
            f"recovery_enabled={self.localization_recovery_enabled}, "
            f"recovery_cycles={self.localization_recovery_max_cycles})."
        )
        self._publish_display("대기", "idle")

    def _publish_heartbeat(self) -> None:
        self._check_goal_response_watchdog()
        self._publish_status(self.current_status, {"note": "heartbeat"})

    def _on_commands(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        if not self._is_for_this_robot(payload):
            target_robot_id = payload.get("robot_id")
            target_robot_name = payload.get("robot_name")
            self.get_logger().debug(
                f"Ignoring command not for this robot (robot_name={self.robot_name}, robot_id={self.robot_id}, "
                f"target_name={target_robot_name}, target_id={target_robot_id})."
            )
            return

        command_type = str(payload.get("type", "")).upper().strip()
        actions = self._extract_actions(payload)
        self._publish_command_received(payload, command_type, actions)

        if command_type in {"STOP", "PAUSE"}:
            self._set_safety_lock(True, source=f"command:{command_type}")
            return
        if command_type == "RESUME":
            self._set_safety_lock(False, source="command:RESUME")
            return
        if command_type == "CANCEL":
            self._cancel_active_sequence(reason=command_type)
            return

        if not actions:
            self.get_logger().warn("Received command message without executable actions.")
            return

        if self._safety_locked:
            previous_task_id = self._current_task_id
            incoming_task_id = self._extract_task_id(payload)
            if incoming_task_id is not None:
                self._current_task_id = incoming_task_id
            self._publish_event(
                "ACTION_FAILED",
                self._task_id_payload(
                    {
                        "reason": "safety_lock_active",
                        "reason_code": "safety_locked",
                        "status_code": 423,
                        "status_text": "safety lock active",
                    }
                ),
            )
            self._publish_status(
                "WAITING",
                self._task_id_payload(
                    {
                        "reason": "safety_lock_active",
                        "reason_code": "safety_locked",
                    }
                ),
            )
            self.get_logger().warn("Rejecting action sequence while safety lock is active.")
            self._current_task_id = previous_task_id
            return

        if self._action_queue or self._current_action is not None:
            self.get_logger().warn("Executor busy; rejecting incoming sequence.")
            return

        self._action_queue = actions
        self._current_task_id = self._extract_task_id(payload)
        self._publish_status("ASSIGNED", self._task_id_payload())
        self._run_next_action()

    def _on_safety_lock(self, msg: Bool) -> None:
        self._set_safety_lock(bool(msg.data), source="topic")

    def _on_ai_link(self, msg: Bool) -> None:
        previous = self._ai_link_alive
        self._ai_link_alive = bool(msg.data)
        if previous is None or previous == self._ai_link_alive:
            return
        self.get_logger().info(
            f"AI link state updated: {'alive' if self._ai_link_alive else 'dead'}."
        )

    def _on_battery(self, msg: Float32) -> None:
        value = float(msg.data)
        if math.isnan(value) or math.isinf(value):
            return
        clamped = max(0.0, min(100.0, value))
        if (not self._battery_received) or abs(clamped - self.battery) >= 0.1:
            self.battery = clamped
            self._battery_received = True

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._last_amcl_pose_mono = time.monotonic()
        covariance = list(msg.pose.covariance)
        if len(covariance) >= 36:
            self._last_amcl_cov_xy = max(float(covariance[0]), float(covariance[7]))
            self._last_amcl_cov_yaw = float(covariance[35])

    def _set_safety_lock(self, enabled: bool, source: str) -> None:
        if self._safety_locked == enabled:
            return
        self._safety_locked = enabled
        if enabled:
            self._enter_safety_lock(source)
        else:
            self._exit_safety_lock(source)

    def _enter_safety_lock(self, source: str) -> None:
        elapsed_sec = self._goal_elapsed_sec()
        self._stop_guide_display()
        self._stop_nav_retry()
        self._stop_localization_recovery("safety_lock")
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        self._stop_timeout_watchdog()

        if self._current_goal_handle is not None:
            try:
                self._cancel_requested = True
                self._cancel_reason = "safety_lock"
                self.get_logger().warn(
                    f"Safety lock cancel requested (task_id={self._current_task_id}, "
                    f"source={source}, elapsed_sec={elapsed_sec:.3f}, target={self._goal_target})."
                )
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self._on_nav_cancel_response(f, "cancel:safety_lock")
                )
            except Exception as exc:
                self.get_logger().warn(f"Safety lock goal cancel failed: {exc}")
            self._current_goal_handle = None

        self._publish_zero_cmd_vel_burst()
        self._action_queue = []
        self._current_action = None
        self._clear_nav_goal_context()
        self.current_status = "WAITING"
        payload = self._task_id_payload(
            {
                "reason": "safety_stop",
                "reason_code": "safety_locked",
                "source": source,
            }
        )
        self._publish_event("SAFETY_STOPPED", payload)
        self._publish_status("WAITING", payload)
        self._publish_display("일시정지", "pause")

    def _exit_safety_lock(self, source: str) -> None:
        self._stop_nav_retry()
        self._stop_localization_recovery("safety_resume")
        self._action_queue = []
        self._current_action = None
        self._clear_nav_goal_context()
        self._stop_guide_display()
        self._cancel_requested = False
        self._cancel_reason = None
        self.current_status = "IDLE"
        payload = self._task_id_payload(
            {
                "reason": "safety_resume",
                "reason_code": "safety_resumed",
                "source": source,
            }
        )
        self._publish_event("SAFETY_RESUMED", payload)
        self._publish_status("IDLE", payload)
        self._publish_display("대기", "idle")
        self._current_task_id = None

    def _run_next_action(self) -> None:
        self._stop_nav_retry()
        self._stop_localization_recovery("next_action")
        self._current_action = None
        if not self._action_queue:
            if self._safety_locked:
                self.current_status = "WAITING"
                self._publish_status(
                    "WAITING",
                    self._task_id_payload(
                        {
                            "reason": "safety_stop",
                            "reason_code": "safety_locked",
                        }
                    ),
                )
                self._publish_display("일시정지", "pause")
            else:
                self.current_status = "IDLE"
                self._publish_status("IDLE", self._task_id_payload())
                self._publish_display("대기", "idle")
            self._clear_nav_goal_context()
            self._stop_guide_display()
            self._current_task_id = None
            return

        action_msg = self._action_queue.pop(0)
        self._current_action = action_msg
        self._nav_retry_attempt_count = 0
        self._localization_recovery_cycle_count = 0
        self._last_localization_recovery_mono = 0.0
        action = str(action_msg.get("action", action_msg.get("type", ""))).upper().strip()
        params = action_msg.get("params", {}) or {}
        on_success = str(action_msg.get("on_success", "")).strip() or None

        if action in {"GOTO", "LEAD_GUEST"}:
            x = float(params.get("x", self.location[0]))
            y = float(params.get("y", self.location[1]))
            self.location = (x, y)
            self.current_status = "GUIDING" if action == "LEAD_GUEST" else "MOVING"
        elif action in {"DISPLAY_TEXT", "PAUSE", "QR_SCAN"}:
            self.current_status = "WAITING"
        elif action == "RESUME":
            self.current_status = "IDLE"
        else:
            self.current_status = "MOVING"

        self._publish_status(
            self.current_status,
            {
                **self._task_id_payload(),
                "action": action,
                "params": params,
            },
        )

        if action == "LEAD_GUEST":
            self._start_guide_display()
        else:
            self._stop_guide_display()
            if action == "GOTO":
                self._publish_display("배달 중", "delivery")
            elif action == "DISPLAY_TEXT":
                text = str(params.get("text", "")).strip() or "안내중"
                self._publish_display(text, "display")
            elif action == "QR_SCAN":
                self._publish_display("QR 코드를 인증해주세요", "qr")
            elif action == "PAUSE":
                self._publish_display("일시정지", "pause")
            elif action == "RESUME":
                self._publish_display("대기", "idle")

        if action in {"PAUSE", "STOP"}:
            self._set_safety_lock(True, source=f"action:{action}")
            return
        if action == "RESUME":
            self._set_safety_lock(False, source="action:RESUME")
            self._finish_action_once(on_success)
            return
        if self._safety_locked:
            self._fail_current_action(
                "safety_lock_active",
                {
                    "reason_code": "safety_locked",
                    "status_code": 423,
                    "status_text": "safety lock active",
                },
            )
            return
        if action == "CANCEL":
            self._cancel_active_sequence(reason=action)
            return
        if action == "QR_SCAN":
            self._action_timer = self.create_timer(
                self.execution_delay_sec, lambda: self._finish_action_once(on_success)
            )
            return

        if action in {"GOTO", "LEAD_GUEST"} and not self.mock_mode:
            if not self.use_nav2:
                self._fail_current_action(
                    "nav2_disabled", {"status_code": 501, "status_text": "nav2 disabled"}
                )
                return
            if not self._execute_nav2_goal(params):
                self._fail_current_action(
                    "nav2_goal_start_failed",
                    {"status_code": 500, "status_text": "nav2 goal start failed"},
                )
                return
            return

        if action == "DISPLAY_TEXT":
            self._action_timer = self.create_timer(
                self.execution_delay_sec, lambda: self._finish_action_once(on_success)
            )
            return

        if self.mock_mode:
            self._action_timer = self.create_timer(
                self.execution_delay_sec, lambda: self._finish_action_once(on_success)
            )
            return

        self._fail_current_action(
            "unsupported_action",
            {"status_code": 400, "status_text": "unsupported action", "action": action},
        )

    def _finish_action_once(self, on_success: Optional[str]) -> None:
        self._stop_nav_retry()
        self._stop_localization_recovery("action_finished")
        current_action = self._current_action or {}
        action_name = str(
            current_action.get("action", current_action.get("type", ""))
        ).upper().strip()
        success_event = on_success
        if not success_event and action_name == "GOTO" and self.default_goto_success_event:
            success_event = self.default_goto_success_event
        event_extra: Dict[str, Any] = {}
        if action_name == "QR_SCAN":
            params = current_action.get("params", {}) or {}
            scanned_data = params.get("scanned_data")
            if scanned_data is not None:
                event_extra["scanned_data"] = scanned_data

        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        self._clear_nav_goal_context()
        if action_name == "LEAD_GUEST":
            self._stop_guide_display()
        self._current_action = None
        if success_event:
            payload = self._task_id_payload(event_extra)
            self._publish_event(success_event, payload)
            self._publish_status(self.current_status, payload, event=success_event)
            if success_event in {"ARRIVED_AT_DESTINATION", "ARRIVED_AT_BASE"}:
                self._publish_display("도착완료", "arrived")
        self._run_next_action()

    def _extract_task_id(self, payload: Dict[str, Any]) -> Optional[Any]:
        for key in ("task_id", "sequence_id", "id"):
            value = payload.get(key)
            if value is not None:
                if isinstance(value, str) and value.strip():
                    return value.strip()
                if isinstance(value, (int, float)) and not isinstance(value, bool):
                    return value
        return None

    def _task_id_payload(self, extra: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        payload: Dict[str, Any] = {}
        if self._current_task_id is not None:
            payload["task_id"] = self._current_task_id
            payload["sequence_id"] = self._current_task_id
        if extra is not None:
            payload.update(extra)
        return payload

    @staticmethod
    def _norm_command_id(value: Any) -> Optional[str]:
        if value is None or isinstance(value, bool):
            return None
        if isinstance(value, (int, float)):
            return str(int(value))
        if isinstance(value, str):
            normalized = value.strip()
            return normalized or None
        return None

    def _is_for_this_robot(self, payload: Dict[str, Any]) -> bool:
        command_robot_id = self._norm_command_id(payload.get("robot_id"))
        command_robot_name = (
            str(payload.get("robot_name", "")).strip() if payload.get("robot_name") else ""
        )

        self_robot_id = str(self.robot_id)
        self_robot_name = str(self.robot_name or "")

        if command_robot_id is None and not command_robot_name:
            return True
        if command_robot_id is not None and command_robot_id == self_robot_id:
            return True
        if command_robot_name and command_robot_name == self_robot_name:
            return True
        return False

    def _execute_nav2_goal(self, params: Dict[str, Any]) -> bool:
        if self.nav_client is None:
            self.get_logger().error("Nav2 client unavailable.")
            return False
        localization_ready, localization_reason = self._is_localization_ready()
        if not localization_ready:
            if (
                self.localization_allow_degraded_covariance
                and self._is_covariance_only_block(localization_reason)
            ):
                self.get_logger().warn(
                    "Localization covariance above threshold but proceeding in degraded mode "
                    f"(reason={localization_reason}, task_id={self._current_task_id})."
                )
            else:
                if self._schedule_nav_retry(f"localization_not_ready:{localization_reason}"):
                    return True
                self.get_logger().error(
                    f"Localization not ready (reason={localization_reason}, task_id={self._current_task_id})."
                )
                return False
        if self._localization_recovery_cycle_count > 0:
            self.get_logger().info(
                f"Localization converged after recovery cycles={self._localization_recovery_cycle_count} "
                f"(task_id={self._current_task_id})."
            )
            self._localization_recovery_cycle_count = 0
            self._last_localization_recovery_mono = 0.0
        # Recovery spin publishes /cmd_vel; stop it before handing control to Nav2.
        self._stop_localization_recovery("nav_goal_start")
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            if self._schedule_nav_retry("action_server_not_ready"):
                return True
            self.get_logger().error(
                f"Nav2 action server not ready (action={self.nav2_action_name}, task_id={self._current_task_id})."
            )
            return False

        try:
            x = float(params.get("x"))
            y = float(params.get("y"))
            yaw = float(params.get("yaw", params.get("theta", 0.0)))
        except (TypeError, ValueError):
            self.get_logger().error("Invalid GOTO params: x/y(/yaw) required.")
            return False

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame_id
        # Cross-host mode: avoid clock skew issues.
        goal.pose.header.stamp.sec = 0
        goal.pose.header.stamp.nanosec = 0
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(yaw * 0.5)

        send_ts = time.time()
        self._goal_target = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "frame_id": self.frame_id,
            "action_name": self.nav2_action_name,
            "send_ts": send_ts,
        }
        self._last_nav_feedback = None
        self._last_feedback_log_at = 0.0
        self._cancel_requested = False
        self._cancel_reason = None
        self._goal_started_at = time.time()
        self._goal_response_started_at = self._goal_started_at
        self.get_logger().info(
            f"Sending Nav2 goal (task_id={self._current_task_id}, action={self.nav2_action_name}, "
            f"frame={self.frame_id}, target=({x:.3f}, {y:.3f}, yaw={yaw:.3f}), send_ts={send_ts:.3f})"
        )
        self._start_timeout_watchdog()
        send_future = self.nav_client.send_goal_async(
            goal, feedback_callback=self._on_nav_goal_feedback
        )
        send_future.add_done_callback(self._on_nav_goal_response)
        return True

    def _schedule_nav_retry(self, reason: str) -> bool:
        if self._safety_locked or self._current_action is None:
            return False

        action_name = str(
            self._current_action.get("action", self._current_action.get("type", ""))
        ).upper().strip()
        if action_name not in {"GOTO", "LEAD_GUEST"}:
            return False
        if self._nav_retry_attempt_count >= self.nav2_retry_attempts:
            return False

        self._nav_retry_attempt_count += 1
        attempt = self._nav_retry_attempt_count
        self._stop_timeout_watchdog()
        self._clear_nav_goal_context()
        self._stop_nav_retry()
        self._trigger_localization_recovery(reason)

        delay_sec = max(0.2, float(self.nav2_retry_delay_sec))
        self.get_logger().warn(
            f"Scheduling Nav2 retry (reason={reason}, attempt={attempt}/{self.nav2_retry_attempts}, "
            f"task_id={self._current_task_id}, delay_sec={delay_sec:.1f})."
        )

        def _retry_once() -> None:
            self._stop_nav_retry()
            if self._safety_locked or self._current_action is None:
                return
            retry_action = str(
                self._current_action.get("action", self._current_action.get("type", ""))
            ).upper().strip()
            if retry_action not in {"GOTO", "LEAD_GUEST"}:
                return
            retry_params = self._current_action.get("params", {}) or {}
            if not isinstance(retry_params, dict):
                self._fail_current_action(
                    "invalid_retry_params",
                    {"status_code": 400, "status_text": "invalid retry params"},
                )
                return
            if not self._execute_nav2_goal(retry_params):
                self._fail_current_action(
                    "nav2_goal_start_failed",
                    {"status_code": 500, "status_text": "nav2 goal start failed"},
                )

        self._nav_retry_timer = self.create_timer(delay_sec, _retry_once)
        return True

    def _stop_nav_retry(self) -> None:
        if self._nav_retry_timer is None:
            return
        self._nav_retry_timer.cancel()
        self._nav_retry_timer = None

    def _trigger_localization_recovery(self, reason: str) -> None:
        if not self.localization_recovery_enabled:
            return
        if not reason.startswith("localization_not_ready:"):
            return
        if self._localization_recovery_active:
            return
        if self._localization_recovery_cycle_count >= self.localization_recovery_max_cycles:
            return

        now = time.monotonic()
        if (
            self.localization_recovery_cooldown_sec > 0.0
            and self._last_localization_recovery_mono > 0.0
            and (now - self._last_localization_recovery_mono) < self.localization_recovery_cooldown_sec
        ):
            return

        self._localization_recovery_cycle_count += 1
        cycle = self._localization_recovery_cycle_count
        self._last_localization_recovery_mono = now

        reason_detail = reason.split(":", 1)[1] if ":" in reason else reason
        use_global_relocalization = not self._is_covariance_only_block(reason_detail)
        if reason_detail.startswith("amcl_pose_missing"):
            self._call_nomotion_update(cycle, reason)
            use_global_relocalization = cycle > 1
        elif reason_detail.startswith("amcl_pose_stale"):
            self._call_nomotion_update(cycle, reason)
            use_global_relocalization = False

        if use_global_relocalization:
            self._call_global_localization(cycle, reason)
        self._start_localization_spin(cycle, reason)

    @staticmethod
    def _is_covariance_only_block(reason: str) -> bool:
        return reason.startswith("amcl_cov_xy_high") or reason.startswith("amcl_cov_yaw_high")

    def _call_nomotion_update(self, cycle: int, reason: str) -> None:
        if self.amcl_nomotion_client is None:
            return
        try:
            if not self.amcl_nomotion_client.wait_for_service(timeout_sec=self.amcl_nomotion_wait_sec):
                return
            future = self.amcl_nomotion_client.call_async(Empty.Request())
            future.add_done_callback(lambda f: self._on_nomotion_update_response(f, cycle, reason))
            self.get_logger().info(
                f"Localization recovery cycle={cycle}: requested AMCL nomotion update "
                f"(service={self.amcl_nomotion_update_service_name}, reason={reason})."
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: nomotion update request failed ({exc})."
            )

    def _on_nomotion_update_response(self, future: Any, cycle: int, reason: str) -> None:
        try:
            _ = future.result()
            self.get_logger().info(
                f"Localization recovery cycle={cycle}: AMCL nomotion update completed (reason={reason})."
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: AMCL nomotion update response error ({exc})."
            )

    def _call_global_localization(self, cycle: int, reason: str) -> None:
        if self.global_localization_client is None:
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: global localization client unavailable "
                f"(reason={reason})."
            )
            return

        try:
            if not self.global_localization_client.wait_for_service(
                timeout_sec=self.global_localization_wait_sec
            ):
                self.get_logger().warn(
                    f"Localization recovery cycle={cycle}: global localization service not ready "
                    f"(service={self.global_localization_service_name})."
                )
                return
            future = self.global_localization_client.call_async(Empty.Request())
            future.add_done_callback(
                lambda f: self._on_global_localization_response(f, cycle, reason)
            )
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: requested global relocalization "
                f"(service={self.global_localization_service_name}, reason={reason})."
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: global relocalization request failed ({exc})."
            )

    def _on_global_localization_response(self, future: Any, cycle: int, reason: str) -> None:
        try:
            _ = future.result()
            self.get_logger().info(
                f"Localization recovery cycle={cycle}: global relocalization completed (reason={reason})."
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Localization recovery cycle={cycle}: global relocalization response error ({exc})."
            )

    def _start_localization_spin(self, cycle: int, reason: str) -> None:
        if self.localization_recovery_spin_duration_sec <= 0.0:
            return
        if abs(self.localization_recovery_spin_angular_speed) < 1e-3:
            return
        if self._safety_locked:
            return

        self._stop_localization_recovery(None)
        self._localization_recovery_active = True
        self._localization_recovery_until_mono = (
            time.monotonic() + self.localization_recovery_spin_duration_sec
        )

        self.get_logger().warn(
            f"Localization recovery cycle={cycle}: spinning for {self.localization_recovery_spin_duration_sec:.1f}s "
            f"(angular_z={self.localization_recovery_spin_angular_speed:.2f}, reason={reason})."
        )

        self._localization_recovery_timer = self.create_timer(
            0.1, lambda: self._on_localization_spin_timer(cycle, reason)
        )

    def _on_localization_spin_timer(self, cycle: int, reason: str) -> None:
        if not self._localization_recovery_active:
            return
        if self._safety_locked or self._current_action is None:
            self._stop_localization_recovery("interrupted")
            return
        if time.monotonic() >= self._localization_recovery_until_mono:
            self._stop_localization_recovery("completed")
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = float(self.localization_recovery_spin_angular_speed)
        self.stop_pub.publish(twist)

    def _stop_localization_recovery(self, reason: Optional[str]) -> None:
        was_active = self._localization_recovery_active
        self._localization_recovery_active = False
        self._localization_recovery_until_mono = 0.0
        if self._localization_recovery_timer is not None:
            self._localization_recovery_timer.cancel()
            self._localization_recovery_timer = None
        if not was_active:
            return

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.stop_pub.publish(twist)
        if reason:
            self.get_logger().info(f"Localization recovery spin stopped ({reason}).")

    def _is_localization_ready(self) -> Tuple[bool, str]:
        if not self.localization_required:
            return True, "disabled"

        if self._last_amcl_pose_mono <= 0.0:
            return False, "amcl_pose_missing"

        if self.amcl_pose_stale_check_enabled:
            pose_age = time.monotonic() - self._last_amcl_pose_mono
            if pose_age > self.amcl_pose_max_age_sec:
                return False, f"amcl_pose_stale:{pose_age:.2f}s"

        if (
            self._last_amcl_cov_xy is not None
            and self._last_amcl_cov_xy > self.amcl_covariance_xy_max
        ):
            return False, f"amcl_cov_xy_high:{self._last_amcl_cov_xy:.3f}"

        if (
            self._last_amcl_cov_yaw is not None
            and self._last_amcl_cov_yaw > self.amcl_covariance_yaw_max
        ):
            return False, f"amcl_cov_yaw_high:{self._last_amcl_cov_yaw:.3f}"

        if self.nav2_require_map_odom_tf and self.tf_buffer is not None:
            try:
                self.tf_buffer.lookup_transform(
                    "map",
                    "odom",
                    Time(),
                    timeout=Duration(seconds=self.nav2_tf_lookup_timeout_sec),
                )
            except TransformException:
                return False, "map_odom_tf_missing"

        return True, "ready"

    def _on_nav_goal_feedback(self, feedback_msg: Any) -> None:
        feedback = getattr(feedback_msg, "feedback", None)
        if feedback is None:
            return

        snapshot: Dict[str, Any] = {}
        if hasattr(feedback, "distance_remaining"):
            snapshot["distance_remaining"] = float(feedback.distance_remaining)
        if hasattr(feedback, "navigation_time"):
            snapshot["navigation_time_sec"] = self._duration_to_sec(feedback.navigation_time)
        if hasattr(feedback, "estimated_time_remaining"):
            snapshot["estimated_time_remaining_sec"] = self._duration_to_sec(
                feedback.estimated_time_remaining
            )
        if hasattr(feedback, "number_of_recoveries"):
            snapshot["number_of_recoveries"] = int(feedback.number_of_recoveries)
        if hasattr(feedback, "current_pose") and hasattr(feedback.current_pose, "pose"):
            pose = feedback.current_pose.pose
            snapshot["current_x"] = float(pose.position.x)
            snapshot["current_y"] = float(pose.position.y)

        self._last_nav_feedback = snapshot
        now = time.time()
        if (now - self._last_feedback_log_at) < self.nav2_feedback_log_period_sec:
            return
        self._last_feedback_log_at = now
        self.get_logger().info(
            "Nav2 feedback "
            f"(task_id={self._current_task_id}, distance_remaining={snapshot.get('distance_remaining')}, "
            f"navigation_time_sec={snapshot.get('navigation_time_sec')}, "
            f"estimated_time_remaining_sec={snapshot.get('estimated_time_remaining_sec')}, "
            f"recoveries={snapshot.get('number_of_recoveries')}, "
            f"current_pose=({snapshot.get('current_x')}, {snapshot.get('current_y')}))"
        )

    def _on_nav_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_response_started_at = None
            self._cancel_requested = False
            self._cancel_reason = None
            self.get_logger().error(
                f"Nav2 goal send failed: {exc} (task_id={self._current_task_id})"
            )
            self._fail_current_action(
                "goal_send_exception",
                {"failure_detail": "goal_send_exception", "error": str(exc)},
            )
            return

        if not goal_handle.accepted:
            self._goal_response_started_at = None
            self._cancel_requested = False
            self._cancel_reason = None
            if self._schedule_nav_retry("goal_rejected"):
                return
            self.get_logger().warn(
                f"Nav2 goal rejected (task_id={self._current_task_id}, action={self.nav2_action_name})."
            )
            self._fail_current_action("goal_rejected", {"failure_detail": "goal_rejected"})
            return

        self._goal_response_started_at = None
        self._current_goal_handle = goal_handle
        self.get_logger().info(
            f"Nav2 goal accepted (task_id={self._current_task_id}, action={self.nav2_action_name})."
        )
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_goal_result)

    def _on_nav_goal_result(self, future: Any) -> None:
        elapsed_sec = self._goal_elapsed_sec()
        feedback_snapshot = self._last_nav_feedback
        self._stop_timeout_watchdog()
        self._current_goal_handle = None
        try:
            result = future.result()
            status_code = int(result.status)
        except Exception as exc:
            self.get_logger().error(f"Nav2 result failed: {exc}")
            self._fail_current_action(
                "goal_result_exception",
                {"failure_detail": "goal_result_exception", "error": str(exc)},
            )
            self._cancel_requested = False
            self._cancel_reason = None
            return

        if status_code == 5 and self._cancel_requested:
            self.get_logger().info(
                f"Nav2 goal canceled as requested (reason={self._cancel_reason}, "
                f"task_id={self._current_task_id}, elapsed_sec={elapsed_sec:.3f})"
            )
            if feedback_snapshot:
                self.get_logger().info(f"Last Nav2 feedback before cancel: {feedback_snapshot}")
            self._cancel_requested = False
            self._cancel_reason = None
            return

        if status_code == self.nav2_success_status_code:
            on_success = None
            if self._current_action is not None:
                on_success = str(self._current_action.get("on_success", "")).strip() or None
            self.get_logger().info(
                f"Nav2 goal succeeded (task_id={self._current_task_id}, status={status_code}, "
                f"elapsed_sec={elapsed_sec:.3f}, target={self._goal_target})"
            )
            if feedback_snapshot:
                self.get_logger().info(f"Last Nav2 feedback before success: {feedback_snapshot}")
            self._finish_action_once(on_success)
            self._cancel_requested = False
            self._cancel_reason = None
            return

        detail = self._build_nav2_result_detail(status_code, result)
        fallback = self._build_abort_success_detail(status_code, detail, feedback_snapshot)
        if fallback is not None:
            on_success = None
            if self._current_action is not None:
                on_success = str(self._current_action.get("on_success", "")).strip() or None
            self.get_logger().warn(
                f"Nav2 goal aborted but treated as success (task_id={self._current_task_id}, "
                f"status={detail.get('status_text', status_code)}, "
                f"error_code={detail.get('error_code')}, "
                f"distance_remaining={fallback.get('distance_remaining')}, "
                f"distance_to_goal={fallback.get('distance_to_goal')}, "
                f"tolerance={self.nav2_abort_success_distance_tolerance})."
            )
            self._publish_event("NAV2_ABORT_TREATED_AS_SUCCESS", self._task_id_payload(fallback))
            self._finish_action_once(on_success)
            self._cancel_requested = False
            self._cancel_reason = None
            return

        self.get_logger().error(
            f"Nav2 goal failed (task_id={self._current_task_id}, status={detail.get('status_text', status_code)}, "
            f"error_code={detail.get('error_code')}, error_msg={detail.get('error_msg')}, "
            f"elapsed_sec={elapsed_sec:.3f}, target={self._goal_target})"
        )
        if feedback_snapshot:
            self.get_logger().error(f"Last Nav2 feedback before failure: {feedback_snapshot}")
        self._fail_current_action(f"goal_failed_status_{status_code}", detail)
        self._cancel_requested = False
        self._cancel_reason = None

    def _start_timeout_watchdog(self) -> None:
        self._stop_timeout_watchdog()
        self._timeout_timer = self.create_timer(1.0, self._check_goal_timeout)

    def _stop_timeout_watchdog(self) -> None:
        if self._timeout_timer is not None:
            self._timeout_timer.cancel()
            self._timeout_timer = None
        self._goal_started_at = None

    def _check_goal_timeout(self) -> None:
        self._check_goal_response_watchdog()
        if self._goal_started_at is None:
            return
        if (time.time() - self._goal_started_at) <= self.goal_timeout_sec:
            return

        elapsed_sec = self._goal_elapsed_sec()
        if self._current_goal_handle is not None:
            try:
                self.get_logger().warn(
                    f"Nav2 timeout cancel requested (task_id={self._current_task_id}, "
                    f"elapsed_sec={elapsed_sec:.3f}, target={self._goal_target})."
                )
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(lambda f: self._on_nav_cancel_response(f, "timeout"))
            except Exception as exc:
                self.get_logger().warn(f"Goal cancel request failed during timeout: {exc}")
        self._publish_zero_cmd_vel_burst()
        self._current_goal_handle = None
        self.get_logger().error(
            f"Nav2 goal timeout (task_id={self._current_task_id}, elapsed_sec={elapsed_sec:.3f}, "
            f"target={self._goal_target})."
        )
        self._fail_current_action(
            "goal_timeout",
            {"status_code": 408, "status_text": "goal timeout"},
        )

    def _cancel_active_sequence(self, reason: str) -> None:
        elapsed_sec = self._goal_elapsed_sec()
        self._stop_guide_display()
        self._stop_nav_retry()
        self._stop_localization_recovery("sequence_cancel")
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        self._stop_timeout_watchdog()

        if self._current_goal_handle is not None:
            try:
                self._cancel_requested = True
                self._cancel_reason = reason
                self.get_logger().warn(
                    f"Cancel requested (reason={reason}, task_id={self._current_task_id}, "
                    f"elapsed_sec={elapsed_sec:.3f}, target={self._goal_target})."
                )
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self._on_nav_cancel_response(f, f"cancel:{reason}")
                )
            except Exception as exc:
                self.get_logger().warn(f"Goal cancel failed: {exc}")
            self._current_goal_handle = None

        self._publish_zero_cmd_vel_burst()
        self._publish_event(
            "SEQUENCE_CANCELED",
            self._task_id_payload(
                {"reason": reason, "reason_code": self._normalize_reason_code(reason)}
            ),
        )
        self._action_queue = []
        self._current_action = None
        self._clear_nav_goal_context()
        if self._safety_locked:
            self.current_status = "WAITING"
            self._publish_status(
                "WAITING",
                self._task_id_payload(
                    {"reason": "safety_stop", "reason_code": "safety_locked"}
                ),
            )
            self._publish_display("일시정지", "pause")
            return

        self.current_status = "IDLE"
        cancel_status = self._task_id_payload(
            {"reason": reason, "reason_code": self._normalize_reason_code(reason)}
        )
        self._current_task_id = None
        self._publish_status("IDLE", cancel_status)
        self._publish_display("대기", "idle")

    def _publish_zero_cmd_vel_burst(self) -> None:
        if self.stop_publish_count <= 0:
            return

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        interval = 1.0 / max(self.stop_publish_hz, 1.0)
        count = {"n": 0}

        def _tick() -> None:
            if count["n"] >= self.stop_publish_count:
                timer.cancel()
                return
            self.stop_pub.publish(msg)
            count["n"] += 1

        timer = self.create_timer(interval, _tick)

    @staticmethod
    def _duration_to_sec(duration_msg: Any) -> Optional[float]:
        if duration_msg is None:
            return None
        sec = getattr(duration_msg, "sec", None)
        nanosec = getattr(duration_msg, "nanosec", None)
        if sec is None or nanosec is None:
            return None
        return float(sec) + float(nanosec) * 1e-9

    def _goal_elapsed_sec(self) -> float:
        if self._goal_started_at is None:
            return 0.0
        return max(0.0, time.time() - self._goal_started_at)

    def _check_goal_response_watchdog(self) -> None:
        if self._goal_response_started_at is None:
            return
        if self._current_goal_handle is not None:
            self._goal_response_started_at = None
            return
        if self._current_action is None:
            self._goal_response_started_at = None
            return

        action_name = str(
            self._current_action.get("action", self._current_action.get("type", ""))
        ).upper().strip()
        if action_name not in {"GOTO", "LEAD_GUEST"}:
            self._goal_response_started_at = None
            return

        elapsed = time.time() - self._goal_response_started_at
        if elapsed <= self.goal_response_timeout_sec:
            return

        self._goal_response_started_at = None
        self.get_logger().error(
            f"Nav2 goal response timeout (task_id={self._current_task_id}, "
            f"waited_sec={elapsed:.3f}, action={self.nav2_action_name}, target={self._goal_target})"
        )
        self._publish_zero_cmd_vel_burst()
        self._fail_current_action(
            "goal_response_timeout",
            {"status_code": 504, "status_text": "goal response timeout"},
        )

    def _clear_nav_goal_context(self) -> None:
        self._goal_target = None
        self._last_nav_feedback = None
        self._last_feedback_log_at = 0.0
        self._goal_response_started_at = None

    def _start_guide_display(self) -> None:
        self._stop_guide_display()
        self._guide_display_toggle = False
        self._publish_display("Follow me", "guide")

        period = max(0.5, self.guide_display_period_sec)

        def _tick() -> None:
            self._guide_display_toggle = not self._guide_display_toggle
            text = "안내중" if self._guide_display_toggle else "Follow me"
            self._publish_display(text, "guide")

        self._guide_display_timer = self.create_timer(period, _tick)

    def _stop_guide_display(self) -> None:
        if self._guide_display_timer is not None:
            self._guide_display_timer.cancel()
            self._guide_display_timer = None

    def _on_nav_cancel_response(self, future: Any, source: str) -> None:
        try:
            response = future.result()
            goals_canceling = len(getattr(response, "goals_canceling", []))
            self.get_logger().info(
                f"Cancel response received (source={source}, task_id={self._current_task_id}, "
                f"goals_canceling={goals_canceling})"
            )
        except Exception as exc:
            self.get_logger().warn(
                f"Cancel response failed (source={source}, task_id={self._current_task_id}, error={exc})"
            )

    @staticmethod
    def _goal_status_text(status_code: int) -> str:
        return {
            0: "STATUS_UNKNOWN",
            1: "STATUS_ACCEPTED",
            2: "STATUS_EXECUTING",
            3: "STATUS_CANCELING",
            4: "STATUS_SUCCEEDED",
            5: "STATUS_CANCELED",
            6: "STATUS_ABORTED",
            7: "STATUS_REJECTED",
            8: "STATUS_PREEMPTED",
            9: "STATUS_RECALLED",
            10: "STATUS_LOST",
        }.get(status_code, "STATUS_UNKNOWN")

    @staticmethod
    def _normalize_reason_code(reason: str) -> str:
        return str(reason or "unknown").strip().lower().replace(" ", "_")

    def _build_nav2_result_detail(self, status_code: int, result: Any) -> Dict[str, Any]:
        detail: Dict[str, Any] = self._task_id_payload(
            {"status_code": status_code, "status_text": self._goal_status_text(status_code)}
        )

        result_data = getattr(result, "result", None)
        if result_data is None:
            return detail
        if hasattr(result_data, "error_code"):
            detail["error_code"] = int(getattr(result_data, "error_code"))
        if hasattr(result_data, "error_msg"):
            detail["error_msg"] = str(getattr(result_data, "error_msg"))
        return detail

    @staticmethod
    def _parse_int_set(csv: str) -> set[int]:
        values: set[int] = set()
        for token in str(csv or "").split(","):
            token = token.strip()
            if not token:
                continue
            try:
                values.add(int(token))
            except ValueError:
                continue
        return values

    def _build_abort_success_detail(
        self,
        status_code: int,
        detail: Dict[str, Any],
        feedback_snapshot: Optional[Dict[str, Any]],
    ) -> Optional[Dict[str, Any]]:
        if not self.nav2_abort_as_success_enabled:
            return None
        if status_code != 6:
            return None

        error_code = detail.get("error_code")
        if self.nav2_abort_success_error_codes and error_code not in self.nav2_abort_success_error_codes:
            return None

        if not feedback_snapshot:
            return None

        distance_remaining = feedback_snapshot.get("distance_remaining")
        distance_to_goal = None
        if (
            self._goal_target is not None
            and "x" in self._goal_target
            and "y" in self._goal_target
            and "current_x" in feedback_snapshot
            and "current_y" in feedback_snapshot
        ):
            dx = float(self._goal_target["x"]) - float(feedback_snapshot["current_x"])
            dy = float(self._goal_target["y"]) - float(feedback_snapshot["current_y"])
            distance_to_goal = math.hypot(dx, dy)

        tolerance = max(0.0, float(self.nav2_abort_success_distance_tolerance))
        close_enough = False
        if isinstance(distance_remaining, (int, float)) and float(distance_remaining) <= tolerance:
            close_enough = True
        if isinstance(distance_to_goal, (int, float)) and float(distance_to_goal) <= tolerance:
            close_enough = True
        if not close_enough:
            return None

        fallback: Dict[str, Any] = {
            "status_code": status_code,
            "status_text": self._goal_status_text(status_code),
            "treated_as_success": True,
            "success_tolerance": tolerance,
        }
        if isinstance(distance_remaining, (int, float)):
            fallback["distance_remaining"] = float(distance_remaining)
        if isinstance(distance_to_goal, (int, float)):
            fallback["distance_to_goal"] = float(distance_to_goal)
        if error_code is not None:
            fallback["error_code"] = int(error_code)
        if "error_msg" in detail:
            fallback["error_msg"] = detail.get("error_msg")
        return fallback

    def _fail_current_action(self, reason: str, extra: Optional[Dict[str, Any]] = None) -> None:
        self._stop_timeout_watchdog()
        self._stop_guide_display()
        self._stop_nav_retry()
        self._stop_localization_recovery("action_failed")
        self._action_queue = []
        self._current_action = None
        self._current_goal_handle = None
        self._clear_nav_goal_context()
        self._cancel_requested = False
        self._cancel_reason = None
        self.current_status = "ERROR"
        payload = self._task_id_payload(
            {"reason": reason, "reason_code": self._normalize_reason_code(reason)}
        )
        if extra is not None:
            payload.update(extra)
        payload.setdefault("reason_code", self._normalize_reason_code(reason))
        self._publish_event("ACTION_FAILED", payload)
        self._publish_status("ERROR", payload)
        if self._safety_locked:
            self.current_status = "WAITING"
            self._publish_status(
                "WAITING",
                self._task_id_payload(
                    {"reason": "safety_stop", "reason_code": "safety_locked"}
                ),
            )
            self._publish_display("일시정지", "pause")
            return
        self.current_status = "IDLE"
        idle_status = self._task_id_payload(
            {"reason": "recover_after_error", "reason_code": "recover_after_error"}
        )
        self._current_task_id = None
        self._publish_status("IDLE", idle_status)
        self._publish_display("대기", "idle")

    def _publish_status(self, status: str, extra: Dict[str, Any], event: Optional[str] = None) -> None:
        data = {
            "robot_id": int(self.robot_id),
            "robot_name": self.robot_name,
            "status": status,
            "safety_lock": bool(self._safety_locked),
            "location": [float(self.location[0]), float(self.location[1])],
            "battery": float(self.battery),
            **extra,
        }
        if self.include_ai_link_in_status and self._ai_link_alive is not None:
            data["ai_link_alive"] = bool(self._ai_link_alive)
        if event:
            data["event"] = event
        self.status_pub.publish(String(data=json.dumps(data)))

    def _publish_event(self, event: str, extra: Dict[str, Any]) -> None:
        data = {
            "robot_id": int(self.robot_id),
            "robot_name": self.robot_name,
            "event": event,
            **extra,
        }
        self.event_pub.publish(String(data=json.dumps(data)))

    def _publish_command_received(
        self, payload: Dict[str, Any], command_type: str, actions: List[Dict[str, Any]]
    ) -> None:
        if not self.emit_command_received_event:
            return

        incoming_task_id = self._extract_task_id(payload)
        normalized_type = command_type or ("ACTION_SEQUENCE" if actions else "UNKNOWN")
        event_data: Dict[str, Any] = {
            "command_type": normalized_type,
            "action_count": len(actions),
            "has_actions": bool(actions),
            "source": "commands_topic",
            "received_at": time.time(),
        }
        if incoming_task_id is not None:
            event_data["task_id"] = incoming_task_id
            event_data["sequence_id"] = incoming_task_id
        self._publish_event("COMMAND_RECEIVED", event_data)

    def _publish_display(self, text: str, icon: str = "info") -> None:
        if not self.enable_display:
            return
        payload = {
            "robot_id": int(self.robot_id),
            "robot_name": self.robot_name,
            "text": text,
            "icon": icon,
            "ts": time.time(),
        }
        self.display_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    @staticmethod
    def _extract_actions(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        if payload.get("type") == "ACTION_SEQUENCE" and isinstance(payload.get("payload"), list):
            return payload["payload"]
        if payload.get("type") == "ACTION_SEQUENCE" and isinstance(payload.get("actions"), list):
            return payload["actions"]
        if "action" in payload or "type" in payload:
            return [payload]
        if "task_type" in payload and "destination" in payload:
            destination = payload.get("destination") or {}
            return [
                {
                    "action": "GOTO",
                    "params": {"x": destination.get("x", 0.0), "y": destination.get("y", 0.0)},
                }
            ]
        return []

    @staticmethod
    def _parse_payload(raw: str) -> Dict[str, Any]:
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            return {}
        if not isinstance(parsed, dict):
            return {}
        return parsed


def main() -> None:
    rclpy.init()
    node = OfficeRobotExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
