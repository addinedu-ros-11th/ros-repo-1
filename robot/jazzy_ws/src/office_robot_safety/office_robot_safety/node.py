import json
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


class OfficeRobotSafety(Node):
    _LOCK_COMMANDS = {"STOP", "PAUSE"}
    _UNLOCK_COMMANDS = {"RESUME"}
    _CLASS_ID_TO_NAME = {
        0: "person",
        1: "chair",
        2: "plant",
        3: "bag",
        4: "robot",
    }
    _CLASS_NAME_TO_ID = {
        "0": 0,
        "person": 0,
        "human": 0,
        "pedestrian": 0,
        "\uc0ac\ub78c": 0,
        "1": 1,
        "chair": 1,
        "\uc758\uc790": 1,
        "2": 2,
        "plant": 2,
        "potted_plant": 2,
        "\ud654\ubd84": 2,
        "3": 3,
        "bag": 3,
        "backpack": 3,
        "\uac00\ubc29": 3,
        "4": 4,
        "robot": 4,
        "\ub85c\ubd07": 4,
    }
    _DISTANCE_KEYS = (
        "distance_m",
        "distance",
        "dist_m",
        "dist",
        "range_m",
        "range",
        "depth_m",
        "depth",
        "estimated_distance",
    )

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
        self.declare_parameter("obstacle_enabled", False)
        self.declare_parameter("obstacle_topic", "obstacles")
        self.declare_parameter("obstacle_confidence_threshold", 0.6)
        self.declare_parameter("obstacle_timeout_sec", 1.5)
        self.declare_parameter("obstacle_clear_hold_sec", 1.0)
        self.declare_parameter("obstacle_person_stop_without_distance", False)
        self.declare_parameter("obstacle_person_stop_m", 0.65)
        self.declare_parameter("obstacle_person_slow_m", 0.95)
        self.declare_parameter("obstacle_chair_stop_m", 0.45)
        self.declare_parameter("obstacle_chair_slow_m", 0.75)
        self.declare_parameter("obstacle_plant_stop_m", 0.42)
        self.declare_parameter("obstacle_plant_slow_m", 0.72)
        self.declare_parameter("obstacle_bag_stop_m", 0.40)
        self.declare_parameter("obstacle_bag_slow_m", 0.70)
        self.declare_parameter("obstacle_robot_stop_m", 0.55)
        self.declare_parameter("obstacle_robot_slow_m", 0.85)

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
        self.obstacle_enabled = (
            self.get_parameter("obstacle_enabled").get_parameter_value().bool_value
        )
        self.obstacle_topic = (
            self.get_parameter("obstacle_topic").get_parameter_value().string_value
        )
        self.obstacle_confidence_threshold = max(
            0.0,
            min(
                1.0,
                self.get_parameter("obstacle_confidence_threshold")
                .get_parameter_value()
                .double_value,
            ),
        )
        self.obstacle_timeout_sec = max(
            0.1, self.get_parameter("obstacle_timeout_sec").get_parameter_value().double_value
        )
        self.obstacle_clear_hold_sec = max(
            0.0,
            self.get_parameter("obstacle_clear_hold_sec").get_parameter_value().double_value,
        )
        self.obstacle_person_stop_without_distance = (
            self.get_parameter("obstacle_person_stop_without_distance")
            .get_parameter_value()
            .bool_value
        )

        self._stop_thresholds = {
            0: self.get_parameter("obstacle_person_stop_m").get_parameter_value().double_value,
            1: self.get_parameter("obstacle_chair_stop_m").get_parameter_value().double_value,
            2: self.get_parameter("obstacle_plant_stop_m").get_parameter_value().double_value,
            3: self.get_parameter("obstacle_bag_stop_m").get_parameter_value().double_value,
            4: self.get_parameter("obstacle_robot_stop_m").get_parameter_value().double_value,
        }
        self._slow_thresholds = {
            0: self.get_parameter("obstacle_person_slow_m").get_parameter_value().double_value,
            1: self.get_parameter("obstacle_chair_slow_m").get_parameter_value().double_value,
            2: self.get_parameter("obstacle_plant_slow_m").get_parameter_value().double_value,
            3: self.get_parameter("obstacle_bag_slow_m").get_parameter_value().double_value,
            4: self.get_parameter("obstacle_robot_slow_m").get_parameter_value().double_value,
        }

        self._command_lock_enabled = False
        self._obstacle_lock_enabled = False
        self._lock_enabled = False
        self._last_obstacle_msg_mono = 0.0
        self._last_stop_trigger_mono = 0.0
        self._obstacle_state = "CLEAR"

        lock_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.lock_pub = self.create_publisher(Bool, self.lock_topic, lock_qos)
        self.stop_pub = self.create_publisher(Twist, self.stop_cmd_vel_topic, 10)
        self.command_sub = self.create_subscription(String, self.cmd_topic, self._on_command, 10)
        self.obstacle_sub = None
        if self.obstacle_enabled:
            self.obstacle_sub = self.create_subscription(
                String, self.obstacle_topic, self._on_obstacle, 10
            )

        keepalive_period = 1.0 / max(self.lock_keepalive_hz, 0.2)
        stop_period = 1.0 / max(self.stop_publish_hz, 1.0)
        self.keepalive_timer = self.create_timer(keepalive_period, self._on_keepalive_timer)
        self.stop_timer = self.create_timer(stop_period, self._on_stop_timer)

        self._publish_lock_state()
        self.get_logger().info(
            f"Safety node ready (robot_name={self.robot_name}, robot_id={self.robot_id}, "
            f"cmd_topic={self.cmd_topic}, lock_topic={self.lock_topic}, "
            f"stop_cmd_vel_topic={self.stop_cmd_vel_topic}, obstacle_enabled={self.obstacle_enabled}, "
            f"obstacle_topic={self.obstacle_topic})."
        )

    def _on_command(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        if not payload:
            return
        if not self._is_for_this_robot(payload):
            return

        command_type = str(payload.get("type", "")).upper().strip()
        if command_type in self._LOCK_COMMANDS:
            self._set_command_lock(True, f"top_level:{command_type}")
            return
        if command_type in self._UNLOCK_COMMANDS:
            self._set_command_lock(False, f"top_level:{command_type}")
            return

        if command_type != "ACTION_SEQUENCE":
            return

        action_types = self._extract_action_types(payload)
        for action_type in action_types:
            if action_type in self._LOCK_COMMANDS:
                self._set_command_lock(True, f"sequence:{action_type}")
                return
            if action_type in self._UNLOCK_COMMANDS:
                self._set_command_lock(False, f"sequence:{action_type}")
                return

    def _set_command_lock(self, enabled: bool, source: str) -> None:
        if self._command_lock_enabled == enabled:
            return
        self._command_lock_enabled = enabled
        self._refresh_lock_state(f"command:{source}")

    def _set_obstacle_lock(self, enabled: bool, source: str) -> None:
        if self._obstacle_lock_enabled == enabled:
            return
        self._obstacle_lock_enabled = enabled
        self._refresh_lock_state(f"obstacle:{source}")

    def _refresh_lock_state(self, source: str) -> None:
        enabled = self._command_lock_enabled or self._obstacle_lock_enabled
        if self._lock_enabled == enabled:
            return
        self._lock_enabled = enabled
        self._publish_lock_state()

        if enabled:
            self._publish_zero_burst()
            self.get_logger().warn(
                f"Safety lock enabled (source={source}, command_lock={self._command_lock_enabled}, "
                f"obstacle_lock={self._obstacle_lock_enabled}, robot_name={self.robot_name}, robot_id={self.robot_id})."
            )
            return
        self.get_logger().info(
            f"Safety lock disabled (source={source}, command_lock={self._command_lock_enabled}, "
            f"obstacle_lock={self._obstacle_lock_enabled}, robot_name={self.robot_name}, robot_id={self.robot_id})."
        )

    def _publish_lock_state(self) -> None:
        self.lock_pub.publish(Bool(data=self._lock_enabled))

    def _on_keepalive_timer(self) -> None:
        self._release_obstacle_lock_on_timeout()
        self._publish_lock_state()

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
        if command_robot_id is not None and command_robot_id in {self_robot_id, self_robot_name}:
            return True
        if command_robot_name and command_robot_name in {self_robot_name, self_robot_id}:
            return True
        return False

    def _on_obstacle(self, msg: String) -> None:
        payload = self._parse_payload(msg.data)
        if not payload:
            return
        if not self._is_for_this_robot(payload):
            return

        now = time.monotonic()
        self._last_obstacle_msg_mono = now
        detections = self._extract_detections(payload)
        stop_hit, slow_hit = self._evaluate_detections(detections)

        if stop_hit is not None:
            self._last_stop_trigger_mono = now
            self._set_obstacle_state("STOP", stop_hit)
            self._set_obstacle_lock(True, stop_hit["reason"])
            return

        if slow_hit is not None:
            self._set_obstacle_state("SLOW", slow_hit)
        else:
            self._set_obstacle_state("CLEAR", None)

        if self._obstacle_lock_enabled and (
            now - self._last_stop_trigger_mono >= self.obstacle_clear_hold_sec
        ):
            self._set_obstacle_lock(False, "clear_hold_elapsed")

    def _set_obstacle_state(self, state: str, detail: Optional[Dict[str, Any]]) -> None:
        if self._obstacle_state == state:
            return
        self._obstacle_state = state

        if state == "STOP" and detail is not None:
            self.get_logger().warn(
                f"Obstacle STOP trigger (class={detail['class_name']}, distance={detail.get('distance')}, "
                f"confidence={detail.get('confidence')}, stop={detail.get('stop_threshold')})."
            )
            return
        if state == "SLOW" and detail is not None:
            self.get_logger().info(
                f"Obstacle SLOW zone (class={detail['class_name']}, distance={detail.get('distance')}, "
                f"confidence={detail.get('confidence')}, slow={detail.get('slow_threshold')})."
            )
            return
        self.get_logger().info("Obstacle state cleared.")

    def _release_obstacle_lock_on_timeout(self) -> None:
        if not self._obstacle_lock_enabled:
            return
        if self._last_obstacle_msg_mono <= 0.0:
            self._set_obstacle_lock(False, "no_obstacle_messages")
            return

        now = time.monotonic()
        age_sec = now - self._last_obstacle_msg_mono
        if age_sec > self.obstacle_timeout_sec:
            self._set_obstacle_lock(False, "obstacle_message_timeout")
            self._set_obstacle_state("CLEAR", None)

    def _evaluate_detections(
        self, detections: List[Dict[str, Any]]
    ) -> Tuple[Optional[Dict[str, Any]], Optional[Dict[str, Any]]]:
        best_stop: Optional[Dict[str, Any]] = None
        best_slow: Optional[Dict[str, Any]] = None

        for det in detections:
            class_id = self._extract_class_id(det)
            if class_id is None:
                continue

            confidence = self._to_float(det.get("confidence"))
            if confidence is None:
                confidence = 1.0
            if confidence < self.obstacle_confidence_threshold:
                continue

            distance = self._extract_distance(det)
            stop_threshold = self._stop_thresholds.get(class_id)
            slow_threshold = self._slow_thresholds.get(class_id)
            class_name = self._CLASS_ID_TO_NAME.get(class_id, str(class_id))

            if (
                distance is None
                and class_id == 0
                and self.obstacle_person_stop_without_distance
            ):
                hit = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "distance": None,
                    "stop_threshold": stop_threshold,
                    "slow_threshold": slow_threshold,
                    "reason": "person_without_distance",
                }
                if best_stop is None:
                    best_stop = hit
                continue

            if distance is None:
                continue

            if stop_threshold is not None and distance <= stop_threshold:
                hit = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "distance": distance,
                    "stop_threshold": stop_threshold,
                    "slow_threshold": slow_threshold,
                    "reason": f"class_{class_id}_stop",
                }
                if best_stop is None or distance < float(best_stop.get("distance", 999.0)):
                    best_stop = hit
                continue

            if slow_threshold is not None and distance <= slow_threshold:
                hit = {
                    "class_id": class_id,
                    "class_name": class_name,
                    "confidence": confidence,
                    "distance": distance,
                    "stop_threshold": stop_threshold,
                    "slow_threshold": slow_threshold,
                    "reason": f"class_{class_id}_slow",
                }
                if best_slow is None or distance < float(best_slow.get("distance", 999.0)):
                    best_slow = hit

        return best_stop, best_slow

    def _extract_detections(self, payload: Dict[str, Any]) -> List[Dict[str, Any]]:
        root: Any = payload
        if (
            str(payload.get("type", "")).upper().strip() == "OBSTACLE_INFO"
            and isinstance(payload.get("payload"), dict)
        ):
            root = payload.get("payload")

        queue: List[Any] = [root]
        while queue:
            current = queue.pop(0)
            if isinstance(current, dict):
                if self._looks_like_detection(current):
                    return [current]
                for value in current.values():
                    if isinstance(value, (dict, list)):
                        queue.append(value)
            elif isinstance(current, list):
                dict_items = [item for item in current if isinstance(item, dict)]
                if dict_items and any(self._looks_like_detection(item) for item in dict_items):
                    return dict_items
                for value in current:
                    if isinstance(value, (dict, list)):
                        queue.append(value)
        return []

    @staticmethod
    def _looks_like_detection(item: Dict[str, Any]) -> bool:
        return any(
            key in item
            for key in (
                "class_id",
                "object_name",
                "label",
                "name",
                "box",
            )
        )

    def _extract_class_id(self, detection: Dict[str, Any]) -> Optional[int]:
        raw_id = self._to_int(detection.get("class_id"))
        if raw_id is not None and raw_id in self._CLASS_ID_TO_NAME:
            return raw_id

        for key in ("object_name", "name", "label", "class_name"):
            raw_name = detection.get(key)
            if raw_name is None:
                continue
            normalized = str(raw_name).strip().lower().replace(" ", "_")
            class_id = self._CLASS_NAME_TO_ID.get(normalized)
            if class_id is not None:
                return class_id
        return None

    def _extract_distance(self, detection: Dict[str, Any]) -> Optional[float]:
        for key in self._DISTANCE_KEYS:
            value = self._to_float(detection.get(key))
            if value is not None:
                return value
        return None

    @staticmethod
    def _to_float(value: Any) -> Optional[float]:
        if value is None or isinstance(value, bool):
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None

    @staticmethod
    def _to_int(value: Any) -> Optional[int]:
        if value is None or isinstance(value, bool):
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return None

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
