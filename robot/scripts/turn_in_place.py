#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class TurnInPlaceNode(Node):
    def __init__(self, odom_topic: str, cmd_vel_topic: str) -> None:
        super().__init__("turn_in_place_helper")
        self._yaw = None
        self._last_yaw = None
        self._accumulated_turn = 0.0
        self._odom_stamp_mono = 0.0
        self._started = False
        self._pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._sub = self.create_subscription(Odometry, odom_topic, self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        yaw = quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w)
        self._odom_stamp_mono = time.monotonic()
        if self._started and self._last_yaw is not None:
            self._accumulated_turn += wrap_angle(yaw - self._last_yaw)
        self._yaw = yaw
        self._last_yaw = yaw

    def wait_for_odom(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._yaw is not None:
                return True
        return False

    def reset_turn_tracking(self) -> None:
        self._accumulated_turn = 0.0
        self._started = True

    def publish_twist(self, angular_z: float) -> None:
        msg = Twist()
        msg.angular.z = float(angular_z)
        self._pub.publish(msg)

    def stop(self, burst_count: int = 10) -> None:
        for _ in range(max(1, burst_count)):
            self.publish_twist(0.0)
            rclpy.spin_once(self, timeout_sec=0.02)
            time.sleep(0.02)


def main() -> int:
    parser = argparse.ArgumentParser(description="Turn the robot in place using /odom feedback.")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--angle-deg", type=float, default=360.0)
    parser.add_argument("--angular-speed", type=float, default=0.35)
    parser.add_argument("--timeout-sec", type=float, default=40.0)
    parser.add_argument("--odom-wait-sec", type=float, default=5.0)
    parser.add_argument("--odom-stale-sec", type=float, default=1.0)
    args = parser.parse_args()

    target_rad = math.radians(float(args.angle_deg))
    angular_speed = abs(float(args.angular_speed))
    if math.isclose(target_rad, 0.0, abs_tol=1e-6) or math.isclose(angular_speed, 0.0, abs_tol=1e-6):
        print("[turn-in-place] nothing to do (zero target angle or angular speed)")
        return 0

    angular_cmd = angular_speed if target_rad >= 0.0 else -angular_speed

    rclpy.init()
    node = TurnInPlaceNode(args.odom_topic, args.cmd_vel_topic)
    try:
        if not node.wait_for_odom(args.odom_wait_sec):
            print("[turn-in-place] no odom received; aborting")
            return 1

        node.reset_turn_tracking()
        start_mono = time.monotonic()
        print(
            f"[turn-in-place] start target_deg={args.angle_deg:.1f} "
            f"angular_speed={angular_cmd:.3f} odom_topic={args.odom_topic}"
        )

        while rclpy.ok():
            now_mono = time.monotonic()
            if now_mono - start_mono > float(args.timeout_sec):
                print("[turn-in-place] timeout reached before target angle")
                return 2
            if node._odom_stamp_mono and now_mono - node._odom_stamp_mono > float(args.odom_stale_sec):
                print("[turn-in-place] odom became stale during turn")
                return 3
            if abs(node._accumulated_turn) >= abs(target_rad):
                break
            node.publish_twist(angular_cmd)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.02)

        node.stop()
        turned_deg = math.degrees(node._accumulated_turn)
        print(f"[turn-in-place] complete turned_deg={turned_deg:.1f}")
        return 0
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
