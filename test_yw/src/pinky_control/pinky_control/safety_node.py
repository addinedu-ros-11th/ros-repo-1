import json
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

class PinkySafety(Node):
    """
    Pinky Pro Safety Supervisor
    Monitors stop commands and manages safety lock state.
    """
    def __init__(self) -> None:
        super().__init__("safety_node")

        self.declare_parameter("robot_id", 1)
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value

        self._lock_enabled = False

        # QoS for latched topic
        lock_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)

        # Subscriptions
        self.command_sub = self.create_subscription(String, "commands", self._on_command, 10)
        
        # Publishers
        self.lock_pub = self.create_publisher(Bool, "safety_lock", lock_qos)
        self.stop_pub = self.create_publisher(Twist, "cmd_vel", 10)

        # Keepalive timer for stop signal and lock state
        self.timer = self.create_timer(0.5, self._publish_safety_state)

        self.get_logger().info(f"Pinky Safety Node initialized for Robot ID: {self.robot_id}")

    def _on_command(self, msg: String):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        target_id = payload.get("robot_id")
        if target_id is not None and int(target_id) != self.robot_id:
            return

        cmd_type = str(payload.get("type", "")).upper()
        
        if cmd_type in ["STOP", "PAUSE"]:
            self._set_lock(True, f"Command: {cmd_type}")
        elif cmd_type == "RESUME":
            self._set_lock(False, "Command: RESUME")

    def _set_lock(self, enabled: bool, source: str):
        if self._lock_enabled == enabled:
            return
        
        self._lock_enabled = enabled
        self.lock_pub.publish(Bool(data=self._lock_enabled))
        
        if enabled:
            self._publish_zero_burst()
            self.get_logger().warn(f"SAFETY LOCK ENABLED by {source}")
        else:
            self.get_logger().info(f"SAFETY LOCK DISABLED by {source}")

    def _publish_safety_state(self):
        self.lock_pub.publish(Bool(data=self._lock_enabled))
        if self._lock_enabled:
            self.stop_pub.publish(Twist())

    def _publish_zero_burst(self):
        msg = Twist()
        for _ in range(5):
            self.stop_pub.publish(msg)

def main():
    rclpy.init()
    node = PinkySafety()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
