import json
from typing import Any, Dict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class OfficeRobotExecutor(Node):
    """
    Minimal task executor node.
    Subscribes: commands (std_msgs/String)
    Publishes: status, event (std_msgs/String)
    """

    def __init__(self) -> None:
        super().__init__("office_robot_executor")

        self.declare_parameter("robot_name", "robot_a")
        self.declare_parameter("mock_mode", True)
        self.declare_parameter("execution_delay_sec", 1.5)

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.mock_mode = self.get_parameter("mock_mode").get_parameter_value().bool_value
        self.execution_delay_sec = (
            self.get_parameter("execution_delay_sec").get_parameter_value().double_value
        )

        self.command_sub = self.create_subscription(String, "commands", self._on_task, 10)
        self.status_pub = self.create_publisher(String, "status", 10)
        self.event_pub = self.create_publisher(String, "event", 10)

        self.status_timer = self.create_timer(5.0, self._publish_heartbeat)

        self.get_logger().info(
            f"Executor ready (robot_name={self.robot_name}, mock_mode={self.mock_mode})."
        )

    def _publish_heartbeat(self) -> None:
        self._publish_status("idle", {"note": "heartbeat"})

    def _on_task(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        task_id = payload.get("task_id", "unknown")
        task_type = payload.get("task_type", "unknown")
        destination = payload.get("destination")

        self._publish_status("received", {"task_id": task_id, "task_type": task_type})
        self._publish_event("task_received", {"task_id": task_id})

        if not self.mock_mode:
            # Placeholder for actual navigation/action execution.
            self.get_logger().info("Non-mock mode not implemented yet. Falling back to mock.")

        self._publish_status("in_progress", {"task_id": task_id, "destination": destination})
        self._last_task_timer = self.create_timer(
            self.execution_delay_sec, lambda: self._finish_task_once(task_id, task_type)
        )

    def _finish_task_once(self, task_id: Any, task_type: Any) -> None:
        if getattr(self, "_last_task_timer", None) is not None:
            self._last_task_timer.cancel()
        self._publish_status("completed", {"task_id": task_id, "task_type": task_type})
        self._publish_event("task_completed", {"task_id": task_id})

    def _publish_status(self, state: str, extra: Dict[str, Any]) -> None:
        data = {"robot": self.robot_name, "state": state, **extra}
        self.status_pub.publish(String(data=json.dumps(data)))

    def _publish_event(self, event: str, extra: Dict[str, Any]) -> None:
        data = {"robot": self.robot_name, "event": event, **extra}
        self.event_pub.publish(String(data=json.dumps(data)))

    @staticmethod
    def _parse_payload(raw: str) -> Dict[str, Any]:
        try:
            return json.loads(raw)
        except json.JSONDecodeError:
            return {"task_type": "raw", "raw": raw}


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
