import json
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


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
        self.declare_parameter("execution_delay_sec", 1.5)
        self.declare_parameter("initial_battery", 100.0)

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.mock_mode = self.get_parameter("mock_mode").get_parameter_value().bool_value
        self.execution_delay_sec = (
            self.get_parameter("execution_delay_sec").get_parameter_value().double_value
        )
        self.battery = (
            self.get_parameter("initial_battery").get_parameter_value().double_value
        )
        self.location: Tuple[float, float] = (0.0, 0.0)
        self.current_status = "IDLE"
        self._current_task_id: Optional[Any] = None
        self._action_queue: List[Dict[str, Any]] = []
        self._action_timer = None

        self.command_sub = self.create_subscription(String, "commands", self._on_commands, 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)

        self.status_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info(
            f"Executor ready (robot_name={self.robot_name}, mock_mode={self.mock_mode})."
        )

    def _publish_heartbeat(self) -> None:
        self._publish_status(self.current_status, {"note": "heartbeat"})

    def _on_commands(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        actions = self._extract_actions(payload)
        if not actions:
            self.get_logger().warn("Received command message without executable actions.")
            return

        self._action_queue = actions
        self._current_task_id = payload.get("task_id")
        self._publish_status("ASSIGNED", {"task_id": self._current_task_id})
        self._run_next_action()

    def _run_next_action(self) -> None:
        if not self._action_queue:
            self.current_status = "IDLE"
            self._publish_status("IDLE", {"task_id": self._current_task_id})
            self._current_task_id = None
            return

        action_msg = self._action_queue.pop(0)
        action = str(action_msg.get("action", "")).upper()
        params = action_msg.get("params", {}) or {}
        on_success = action_msg.get("on_success")

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
                "task_id": self._current_task_id,
                "action": action,
                "params": params,
            },
        )

        if not self.mock_mode:
            self.get_logger().info("Non-mock mode not implemented yet. Falling back to mock.")

        self._action_timer = self.create_timer(
            self.execution_delay_sec, lambda: self._finish_action_once(on_success)
        )

    def _finish_action_once(self, on_success: Optional[str]) -> None:
        if self._action_timer is not None:
            self._action_timer.cancel()
            self._action_timer = None
        if on_success:
            self._publish_event(on_success, {"task_id": self._current_task_id})
            self._publish_status(self.current_status, {"task_id": self._current_task_id}, event=on_success)
        self._run_next_action()

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
