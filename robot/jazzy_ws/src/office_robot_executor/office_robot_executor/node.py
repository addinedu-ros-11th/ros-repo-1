import json
import math
import re
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - runtime environment dependent
    NavigateToPose = None


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
        self.declare_parameter("initial_battery", 100.0)
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("stop_cmd_vel_topic", "cmd_vel")
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("safety_lock_topic", "safety_lock")
        self.declare_parameter("ai_link_topic", "ai_link")
        self.declare_parameter("include_ai_link_in_status", True)
        self.declare_parameter("nav2_success_status_code", 4)
        self.declare_parameter("nav2_feedback_log_period_sec", 1.5)
        self.declare_parameter("enable_display", True)
        self.declare_parameter("display_topic", "display")
        self.declare_parameter("guide_display_period_sec", 2.0)
        self.declare_parameter("emit_command_received_event", True)
        self.declare_parameter("enable_legacy_topic_alias", True)
        self.declare_parameter("legacy_topic_alias_ns", "")

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
        self.include_ai_link_in_status = (
            self.get_parameter("include_ai_link_in_status").get_parameter_value().bool_value
        )
        self.nav2_success_status_code = (
            self.get_parameter("nav2_success_status_code").get_parameter_value().integer_value
        )
        self.nav2_feedback_log_period_sec = (
            self.get_parameter("nav2_feedback_log_period_sec").get_parameter_value().double_value
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
        self.enable_legacy_topic_alias = (
            self.get_parameter("enable_legacy_topic_alias").get_parameter_value().bool_value
        )
        self.legacy_topic_alias_ns = (
            self.get_parameter("legacy_topic_alias_ns").get_parameter_value().string_value.strip()
        )

        self._legacy_alias_ns = self._resolve_legacy_alias_ns(
            self.robot_name,
            self.enable_legacy_topic_alias,
            self.legacy_topic_alias_ns,
        )

        self.location: Tuple[float, float] = (0.0, 0.0)
        self.current_status = "IDLE"
        self._current_task_id: Optional[Any] = None
        self._action_queue: List[Dict[str, Any]] = []
        self._action_timer = None
        self._current_action: Optional[Dict[str, Any]] = None
        self._current_goal_handle = None
        self._goal_started_at: Optional[float] = None
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

        self.command_sub = self.create_subscription(String, "commands", self._on_commands, 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)
        self.display_pub = self.create_publisher(String, self.display_topic, 10)
        self.status_alias_pub = None
        self.event_alias_pub = None
        self.display_alias_pub = None
        if self._legacy_alias_ns:
            self.status_alias_pub = self.create_publisher(
                String, f"/{self._legacy_alias_ns}/status", 10
            )
            self.event_alias_pub = self.create_publisher(
                String, f"/{self._legacy_alias_ns}/event", 10
            )
            self.display_alias_pub = self.create_publisher(
                String, f"/{self._legacy_alias_ns}/{self.display_topic}", 10
            )
        self.stop_pub = self.create_publisher(Twist, self.stop_cmd_vel_topic, 10)

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

        self.nav_client = None
        if self.use_nav2 and not self.mock_mode:
            if NavigateToPose is None:
                self.get_logger().error("nav2_msgs not available; real GOTO execution disabled.")
            else:
                self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self.status_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info(
            f"Executor ready (robot_name={self.robot_name}, mock_mode={self.mock_mode}, use_nav2={self.use_nav2}, "
            f"safety_lock_topic={self.safety_lock_topic}, ai_link_topic={self.ai_link_topic}, "
            f"display_topic={self.display_topic}, command_received_event={self.emit_command_received_event}, "
            f"legacy_alias={self._legacy_alias_ns or 'disabled'})."
        )
        self._publish_display("대기", "idle")

    def _publish_heartbeat(self) -> None:
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
        current_action = self._current_action or {}
        action_name = str(
            current_action.get("action", current_action.get("type", ""))
        ).upper().strip()
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
        if on_success:
            payload = self._task_id_payload(event_extra)
            self._publish_event(on_success, payload)
            self._publish_status(self.current_status, payload, event=on_success)
            if on_success in {"ARRIVED_AT_DESTINATION", "ARRIVED_AT_BASE"}:
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
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
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
            self._cancel_requested = False
            self._cancel_reason = None
            self.get_logger().warn(
                f"Nav2 goal rejected (task_id={self._current_task_id}, action={self.nav2_action_name})."
            )
            self._fail_current_action("goal_rejected", {"failure_detail": "goal_rejected"})
            return

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

    def _clear_nav_goal_context(self) -> None:
        self._goal_target = None
        self._last_nav_feedback = None
        self._last_feedback_log_at = 0.0

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

    def _fail_current_action(self, reason: str, extra: Optional[Dict[str, Any]] = None) -> None:
        self._stop_timeout_watchdog()
        self._stop_guide_display()
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
        msg = String(data=json.dumps(data))
        self.status_pub.publish(msg)
        if self.status_alias_pub is not None:
            self.status_alias_pub.publish(msg)

    def _publish_event(self, event: str, extra: Dict[str, Any]) -> None:
        data = {
            "robot_id": int(self.robot_id),
            "robot_name": self.robot_name,
            "event": event,
            **extra,
        }
        msg = String(data=json.dumps(data))
        self.event_pub.publish(msg)
        if self.event_alias_pub is not None:
            self.event_alias_pub.publish(msg)

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
        msg = String(data=json.dumps(payload, ensure_ascii=False))
        self.display_pub.publish(msg)
        if self.display_alias_pub is not None:
            self.display_alias_pub.publish(msg)

    @staticmethod
    def _resolve_legacy_alias_ns(
        robot_name: str, enabled: bool, manual_alias: str
    ) -> Optional[str]:
        if not enabled:
            return None
        if manual_alias and manual_alias != robot_name:
            return manual_alias
        match = re.match(r"^(.*_)(\d+)$", str(robot_name or ""))
        if not match:
            return None
        prefix, digits = match.groups()
        if len(digits) != 1:
            return None
        alias = f"{prefix}{int(digits):02d}"
        if alias == robot_name:
            return None
        return alias

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
