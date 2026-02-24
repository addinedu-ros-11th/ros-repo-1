import json
import math
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

try:
    from nav2_msgs.action import NavigateToPose
except Exception:  # pragma: no cover - runtime environment dependent
    NavigateToPose = None


class OfficeRobotExecutor(Node):
    """
    Minimal action-sequence executor node.
    Subscribes: commands (std_msgs/String)
    Publishes: status, event (std_msgs/String)
    """

    def __init__(self) -> None:
        super().__init__("office_robot_executor")

        self.declare_parameter("robot_name", "robot")
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("use_nav2", True)
        self.declare_parameter("execution_delay_sec", 1.5)
        self.declare_parameter("initial_battery", 100.0)
        self.declare_parameter("nav2_action_name", "navigate_to_pose")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("goal_timeout_sec", 60.0)
        self.declare_parameter("stop_cmd_vel_topic", "cmd_vel")
        self.declare_parameter("stop_publish_count", 10)
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("nav2_success_status_code", 4)

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
        self.nav2_success_status_code = (
            self.get_parameter("nav2_success_status_code").get_parameter_value().integer_value
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

        self.command_sub = self.create_subscription(String, "commands", self._on_commands, 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)
        self.stop_pub = self.create_publisher(Twist, self.stop_cmd_vel_topic, 10)

        self.nav_client = None
        if self.use_nav2 and not self.mock_mode:
            if NavigateToPose is None:
                self.get_logger().error("nav2_msgs not available; real GOTO execution disabled.")
            else:
                self.nav_client = ActionClient(self, NavigateToPose, self.nav2_action_name)

        self.status_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info(
            f"Executor ready (robot_name={self.robot_name}, mock_mode={self.mock_mode}, use_nav2={self.use_nav2})."
        )

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

        command_type = str(payload.get("type", "")).upper()
        if command_type in {"STOP", "CANCEL"}:
            self._cancel_active_sequence(reason=command_type)
            return

        actions = self._extract_actions(payload)
        if not actions:
            self.get_logger().warn("Received command message without executable actions.")
            return

        if self._action_queue or self._current_action is not None:
            self.get_logger().warn("Executor busy; rejecting incoming sequence.")
            return

        self._action_queue = actions
        self._current_task_id = self._extract_task_id(payload)
        self._publish_status("ASSIGNED", self._task_id_payload())
        self._run_next_action()

    def _run_next_action(self) -> None:
        self._current_action = None
        if not self._action_queue:
            self.current_status = "IDLE"
            self._publish_status("IDLE", self._task_id_payload())
            self._current_task_id = None
            return

        action_msg = self._action_queue.pop(0)
        self._current_action = action_msg
        action = str(action_msg.get("action", "")).upper()
        params = action_msg.get("params", {}) or {}
        on_success = str(action_msg.get("on_success", "")).strip() or None

        if action in {"GOTO", "LEAD_GUEST"}:
            x = float(params.get("x", self.location[0]))
            y = float(params.get("y", self.location[1]))
            self.location = (x, y)
            self.current_status = "GUIDING" if action == "LEAD_GUEST" else "MOVING"
        elif action == "DISPLAY_TEXT":
            self.current_status = "WAITING"
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

        if action in {"STOP", "CANCEL"}:
            self._cancel_active_sequence(reason=action)
            return

        if action in {"GOTO", "LEAD_GUEST"} and not self.mock_mode:
            if not self.use_nav2:
                self._fail_current_action(
                    "nav2_disabled",
                    {"status_code": 501, "status_text": "nav2 disabled"},
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
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        self._current_action = None
        if on_success:
            self._publish_event(on_success, self._task_id_payload())
            self._publish_status(self.current_status, self._task_id_payload(), event=on_success)
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
        if value is None:
            return None
        if isinstance(value, bool):
            return None
        if isinstance(value, (int, float)):
            if isinstance(value, bool):
                return None
            return str(int(value))
        if isinstance(value, str):
            v = value.strip()
            if not v:
                return None
            return v
        return None

    def _is_for_this_robot(self, payload: Dict[str, Any]) -> bool:
        command_robot_id = self._norm_command_id(payload.get("robot_id"))
        command_robot_name = str(payload.get("robot_name", "")).strip() if payload.get("robot_name") else ""

        self_robot_id = str(self.robot_id)
        self_robot_name = str(self.robot_name or "")

        robot_id_only = command_robot_id is not None
        robot_name_only = bool(command_robot_name)

        if not robot_id_only and not robot_name_only:
            return True

        if robot_id_only and command_robot_id == self_robot_id:
            return True

        if robot_name_only and command_robot_name == self_robot_name:
            return True

        return False

    def _execute_nav2_goal(self, params: Dict[str, Any]) -> bool:
        if self.nav_client is None:
            self.get_logger().error("Nav2 client unavailable.")
            return False
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Nav2 action server not ready.")
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
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_started_at = time.time()
        self._start_timeout_watchdog()
        send_future = self.nav_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_nav_goal_response)
        return True

    def _on_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"Nav2 goal send failed: {exc} (task_id={self._current_task_id})"
            )
            self._fail_current_action(
                "goal_send_exception",
                {
                    "failure_detail": "goal_send_exception",
                    "error": str(exc),
                },
            )
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Nav2 goal rejected (task_id={self._current_task_id})"
            )
            self._fail_current_action(
                "goal_rejected",
                {"failure_detail": "goal_rejected"},
            )
            return

        self._current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_nav_goal_result)

    def _on_nav_goal_result(self, future) -> None:
        self._stop_timeout_watchdog()
        self._current_goal_handle = None
        try:
            result = future.result()
            status_code = int(result.status)
        except Exception as exc:
            self.get_logger().error(f"Nav2 result failed: {exc}")
            self._fail_current_action(
                "goal_result_exception",
                {
                    "failure_detail": "goal_result_exception",
                    "error": str(exc),
                },
            )
            return

        if status_code == self.nav2_success_status_code:
            on_success = None
            if self._current_action is not None:
                on_success = str(self._current_action.get("on_success", "")).strip() or None
            self.get_logger().info(
                f"Nav2 goal succeeded (task_id={self._current_task_id}, status={status_code})"
            )
            self._finish_action_once(on_success)
        else:
            detail = self._build_nav2_result_detail(status_code, result)
            self.get_logger().error(
                f"Nav2 goal failed (task_id={self._current_task_id}, status={detail.get('status_text', status_code)}, "
                f"error_code={detail.get('error_code')}, error_msg={detail.get('error_msg')})"
            )
            self._fail_current_action(f"goal_failed_status_{status_code}", detail)

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
        if (time.time() - self._goal_started_at) > self.goal_timeout_sec:
            if self._current_goal_handle is not None:
                try:
                    self._current_goal_handle.cancel_goal_async()
                except Exception as exc:
                    self.get_logger().warn(f"Goal cancel request failed during timeout: {exc}")
            self._publish_zero_cmd_vel_burst()
            self._current_goal_handle = None
            self.get_logger().error("Nav2 goal timeout.")
            self._fail_current_action(
                "goal_timeout",
                {
                    "status_code": 408,
                    "status_text": "goal timeout",
                },
            )

    def _cancel_active_sequence(self, reason: str) -> None:
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        self._stop_timeout_watchdog()

        if self._current_goal_handle is not None:
            try:
                self._current_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Goal cancel failed: {exc}")
            self._current_goal_handle = None

        self._publish_zero_cmd_vel_burst()
        self._publish_event(
            "SEQUENCE_CANCELED", self._task_id_payload({"reason": reason})
        )
        self.current_status = "IDLE"
        self._action_queue = []
        self._current_action = None
        cancel_status = self._task_id_payload({"reason": reason})
        self._current_task_id = None
        self._publish_status("IDLE", cancel_status)

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

    def _build_nav2_result_detail(self, status_code: int, result: Any) -> Dict[str, Any]:
        detail: Dict[str, Any] = self._task_id_payload(
            {
                "status_code": status_code,
                "status_text": self._goal_status_text(status_code),
            }
        )

        result_data = getattr(result, "result", None)
        if result_data is None:
            return detail

        if hasattr(result_data, "error_code"):
            detail["error_code"] = int(getattr(result_data, "error_code"))
        if hasattr(result_data, "error_msg"):
            detail["error_msg"] = str(getattr(result_data, "error_msg"))

        return detail

    def _fail_current_action(
        self, reason: str, extra: Optional[Dict[str, Any]] = None
    ) -> None:
        self._stop_timeout_watchdog()
        self._action_queue = []
        self._current_action = None
        self._current_goal_handle = None
        self.current_status = "ERROR"
        payload = self._task_id_payload({"reason": reason})
        if extra is not None:
            payload.update(extra)
        self._publish_event("ACTION_FAILED", payload)
        self._publish_status("ERROR", payload)
        self.current_status = "IDLE"
        idle_status = self._task_id_payload({"reason": "recover_after_error"})
        self._current_task_id = None
        self._publish_status("IDLE", idle_status)

    def _publish_status(self, status: str, extra: Dict[str, Any], event: Optional[str] = None) -> None:
        data = {
            "robot_id": int(self.robot_id),
            "robot_name": self.robot_name,
            "status": status,
            "location": [float(self.location[0]), float(self.location[1])],
            "battery": float(self.battery),
            **extra,
        }
        if event:
            data["event"] = event
        self.status_pub.publish(String(data=json.dumps(data)))

    def _publish_event(self, event: str, extra: Dict[str, Any]) -> None:
        data = {"robot_id": int(self.robot_id), "robot_name": self.robot_name, "event": event, **extra}
        self.event_pub.publish(String(data=json.dumps(data)))

    @staticmethod
    def _extract_actions(payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        if payload.get("type") == "ACTION_SEQUENCE" and isinstance(payload.get("payload"), list):
            return payload["payload"]
        if payload.get("type") == "ACTION_SEQUENCE" and isinstance(payload.get("actions"), list):
            return payload["actions"]
        if "action" in payload:
            return [payload]
        if "task_type" in payload and "destination" in payload:
            destination = payload.get("destination") or {}
            return [
                {
                    "action": "GOTO",
                    "params": {
                        "x": destination.get("x", 0.0),
                        "y": destination.get("y", 0.0),
                    },
                }
            ]
        return []

    @staticmethod
    def _parse_payload(raw: str) -> Dict[str, Any]:
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {}


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
