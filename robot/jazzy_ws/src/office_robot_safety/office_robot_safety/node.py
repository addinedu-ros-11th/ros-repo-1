import json
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class OfficeRobotSafety(Node):
    _LOCK_COMMANDS = {"STOP", "PAUSE"}
    _UNLOCK_COMMANDS = {"RESUME"}

    def __init__(self) -> None:
        super().__init__("office_robot_safety")

        self.declare_parameter("robot_name", "robot")
        self.declare_parameter("robot_id", 1)
        self.declare_parameter("cmd_topic", "commands")
        self.declare_parameter("lock_topic", "safety_lock")
        self.declare_parameter("stop_cmd_vel_topic", "cmd_vel")
        self.declare_parameter("stop_publish_hz", 20.0)
        self.declare_parameter("lock_keepalive_hz", 2.0)
        self.declare_parameter("stop_publish_count", 10)

        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value
        self.cmd_topic = self.get_parameter("cmd_topic").get_parameter_value().string_value
        self.lock_topic = self.get_parameter("lock_topic").get_parameter_value().string_value
        self.stop_cmd_vel_topic = (
            self.get_parameter("stop_cmd_vel_topic").get_parameter_value().string_value
        )
        self.stop_publish_hz = (
            self.get_parameter("stop_publish_hz").get_parameter_value().double_value
        )
        self.lock_keepalive_hz = (
            self.get_parameter("lock_keepalive_hz").get_parameter_value().double_value
        )
        self.stop_publish_count = (
            self.get_parameter("stop_publish_count").get_parameter_value().integer_value
        )

        self._lock_enabled = False

        lock_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.lock_pub = self.create_publisher(Bool, self.lock_topic, lock_qos)
        self.stop_pub = self.create_publisher(Twist, self.stop_cmd_vel_topic, 10)
        self.command_sub = self.create_subscription(String, self.cmd_topic, self._on_command, 10)

        keepalive_period = 1.0 / max(self.lock_keepalive_hz, 0.2)
        stop_period = 1.0 / max(self.stop_publish_hz, 1.0)
        self.keepalive_timer = self.create_timer(keepalive_period, self._publish_lock_state)
        self.stop_timer = self.create_timer(stop_period, self._on_stop_timer)

        self._publish_lock_state()
        self.get_logger().info(
            f"Safety node ready (robot_name={self.robot_name}, robot_id={self.robot_id}, "
            f"cmd_topic={self.cmd_topic}, lock_topic={self.lock_topic}, stop_cmd_vel_topic={self.stop_cmd_vel_topic})."
        )

    def _on_command(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        if not payload:
            return
        if not self._is_for_this_robot(payload):
            return

        command_type = str(payload.get("type", "")).upper().strip()
        if command_type in self._LOCK_COMMANDS:
            self._set_lock(True, f"top_level:{command_type}")
            return
        if command_type in self._UNLOCK_COMMANDS:
            self._set_lock(False, f"top_level:{command_type}")
            return

        if command_type != "ACTION_SEQUENCE":
            return

        action_types = self._extract_action_types(payload)
        for action_type in action_types:
            if action_type in self._LOCK_COMMANDS:
                self._set_lock(True, f"sequence:{action_type}")
                return
            if action_type in self._UNLOCK_COMMANDS:
                self._set_lock(False, f"sequence:{action_type}")
                return

    def _set_lock(self, enabled: bool, source: str) -> None:
        if self._lock_enabled == enabled:
            return
        self._lock_enabled = enabled
        self._publish_lock_state()

        if enabled:
            self._publish_zero_burst()
            self.get_logger().warn(
                f"Safety lock enabled (source={source}, robot_name={self.robot_name}, robot_id={self.robot_id})."
            )
        else:
            self.get_logger().info(
                f"Safety lock disabled (source={source}, robot_name={self.robot_name}, robot_id={self.robot_id})."
            )

    def _publish_lock_state(self) -> None:
        self.lock_pub.publish(Bool(data=self._lock_enabled))

    def _on_stop_timer(self) -> None:
        if not self._lock_enabled:
            return
        self.stop_pub.publish(self._zero_twist())

    def _publish_zero_burst(self) -> None:
        message = self._zero_twist()
        for _ in range(max(1, int(self.stop_publish_count))):
            self.stop_pub.publish(message)

    @staticmethod
    def _zero_twist() -> Twist:
        message = Twist()
        message.linear.x = 0.0
        message.linear.y = 0.0
        message.linear.z = 0.0
        message.angular.x = 0.0
        message.angular.y = 0.0
        message.angular.z = 0.0
        return message

    def _is_for_this_robot(self, payload: Dict[str, Any]) -> bool:
        command_robot_id = self._norm_command_id(payload.get("robot_id"))
        command_robot_name = str(payload.get("robot_name", "")).strip() if payload.get("robot_name") else ""

        self_robot_id = str(self.robot_id)
        self_robot_name = str(self.robot_name or "")

        if command_robot_id is None and not command_robot_name:
            return True
        if command_robot_id is not None and command_robot_id == self_robot_id:
            return True
        if command_robot_name and command_robot_name == self_robot_name:
            return True
        return False

    @staticmethod
    def _extract_action_types(payload: Dict[str, Any]) -> List[str]:
        actions: List[Dict[str, Any]] = []
        if isinstance(payload.get("payload"), list):
            actions = payload.get("payload")
        elif isinstance(payload.get("actions"), list):
            actions = payload.get("actions")

        output: List[str] = []
        for action in actions:
            if not isinstance(action, dict):
                continue
            action_type = str(action.get("action", action.get("type", ""))).upper().strip()
            if action_type:
                output.append(action_type)
        return output

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
    node = OfficeRobotSafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
