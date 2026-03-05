#!/usr/bin/env python3
import argparse
import json
from datetime import datetime
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def parse_json(raw: str) -> Dict[str, Any]:
    try:
        value = json.loads(raw)
    except Exception:
        return {"_raw": raw}
    if isinstance(value, dict):
        return value
    return {"_raw": str(value)}


def status_color(status: str) -> Tuple[float, float, float]:
    upper = status.upper().strip()
    if upper == "ERROR":
        return (1.0, 0.2, 0.2)
    if upper in {"WAITING", "PAUSE"}:
        return (1.0, 0.8, 0.2)
    if upper in {"MOVING", "GUIDING"}:
        return (0.2, 0.8, 1.0)
    if upper == "IDLE":
        return (0.4, 1.0, 0.4)
    return (0.8, 0.8, 0.8)


class RvizStatusOverlay(Node):
    def __init__(self, robot_ns: str, frame_id: str) -> None:
        node_name = f"{robot_ns}_rviz_status_overlay".replace("-", "_")
        super().__init__(node_name)

        self.robot_ns = robot_ns.strip("/")
        self.frame_id = frame_id

        self.status_topic = f"/{self.robot_ns}/status"
        self.event_topic = f"/{self.robot_ns}/event"
        self.marker_topic = f"/{self.robot_ns}/debug_markers"

        self.last_status: Dict[str, Any] = {}
        self.last_event: Dict[str, Any] = {}
        self.last_status_text = "status: (none)"
        self.last_event_text = "event: (none)"

        self.create_subscription(String, self.status_topic, self._on_status, 10)
        self.create_subscription(String, self.event_topic, self._on_event, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_timer(0.5, self._publish_overlay)

        self.get_logger().info(
            f"RViz overlay started (ns={self.robot_ns}, frame={self.frame_id}, "
            f"status_topic={self.status_topic}, event_topic={self.event_topic}, marker_topic={self.marker_topic})"
        )

    def _on_status(self, msg: String) -> None:
        payload = parse_json(msg.data)
        self.last_status = payload

        status = str(payload.get("status", "UNKNOWN"))
        reason = str(payload.get("reason", "")).strip()
        robot = str(payload.get("robot_name", self.robot_ns))
        task_id = payload.get("task_id", payload.get("sequence_id"))

        if reason:
            self.last_status_text = f"[{robot}] status={status} task={task_id} reason={reason}"
        else:
            self.last_status_text = f"[{robot}] status={status} task={task_id}"

    def _on_event(self, msg: String) -> None:
        payload = parse_json(msg.data)
        self.last_event = payload

        event = str(payload.get("event", "UNKNOWN"))
        reason = str(payload.get("reason", "")).strip()
        task_id = payload.get("task_id", payload.get("sequence_id"))

        if reason:
            self.last_event_text = f"event={event} task={task_id} reason={reason}"
        else:
            self.last_event_text = f"event={event} task={task_id}"

    def _build_text_marker(
        self,
        marker_id: int,
        text: str,
        z: float,
        rgb: Tuple[float, float, float],
    ) -> Marker:
        msg = Marker()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.ns = "rviz_debug_overlay"
        msg.id = marker_id
        msg.type = Marker.TEXT_VIEW_FACING
        msg.action = Marker.ADD
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        msg.scale.z = 0.22
        msg.color.r = rgb[0]
        msg.color.g = rgb[1]
        msg.color.b = rgb[2]
        msg.color.a = 0.95
        msg.text = text
        return msg

    def _publish_overlay(self) -> None:
        status = str(self.last_status.get("status", "UNKNOWN"))
        status_rgb = status_color(status)

        # Highlight event line if failure-like event appears.
        event_name = str(self.last_event.get("event", "")).upper().strip()
        if "FAILED" in event_name or "ERROR" in event_name:
            event_rgb = (1.0, 0.35, 0.35)
        else:
            event_rgb = (0.9, 0.9, 0.9)

        now_str = datetime.now().strftime("%H:%M:%S")
        markers = MarkerArray()
        markers.markers.append(
            self._build_text_marker(
                1,
                f"{self.last_status_text}",
                1.30,
                status_rgb,
            )
        )
        markers.markers.append(
            self._build_text_marker(
                2,
                f"{self.last_event_text}",
                1.05,
                event_rgb,
            )
        )
        markers.markers.append(
            self._build_text_marker(
                3,
                f"overlay_time={now_str}",
                0.82,
                (0.7, 0.7, 0.7),
            )
        )
        self.marker_pub.publish(markers)


def main() -> None:
    parser = argparse.ArgumentParser(description="RViz overlay for robot status/event")
    parser.add_argument("--robot-ns", default="robot01")
    parser.add_argument("--frame-id", default="map")
    args = parser.parse_args()

    rclpy.init()
    node: Optional[RvizStatusOverlay] = None
    try:
        node = RvizStatusOverlay(args.robot_ns, args.frame_id)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
