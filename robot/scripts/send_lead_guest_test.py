#!/usr/bin/env python3
import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LeadGuestTestPublisher(Node):
    def __init__(self, robot_ns: str) -> None:
        super().__init__("lead_guest_test_publisher")
        topic = f"/{robot_ns.strip('/')}/commands"
        self.publisher = self.create_publisher(String, topic, 10)
        self.topic = topic

    def publish_once(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(msg)
        self.get_logger().info(f"Published LEAD_GUEST test command to {self.topic}: {msg.data}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Publish a valid LEAD_GUEST ACTION_SEQUENCE.")
    parser.add_argument("--robot-ns", default="robot01")
    parser.add_argument("--task-id", type=int, default=9001)
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--theta", type=float, default=0.0)
    parser.add_argument("--robot-name", default=None)
    args = parser.parse_args()

    robot_ns = args.robot_ns.strip("/")
    robot_name = args.robot_name or robot_ns
    payload = {
        "robot_name": robot_name,
        "type": "ACTION_SEQUENCE",
        "task_id": args.task_id,
        "payload": [
            {
                "action": "LEAD_GUEST",
                "params": {
                    "x": args.x,
                    "y": args.y,
                    "theta": args.theta,
                },
                "on_success": "ARRIVED_AT_DESTINATION",
            }
        ],
    }

    rclpy.init()
    node = LeadGuestTestPublisher(robot_ns)
    try:
        # Give DDS discovery a moment so the first publish is not lost on a cold graph.
        end = time.time() + 1.0
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.publish_once(payload)
        for _ in range(5):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
