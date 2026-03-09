import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import math
import time

class InitialPoseSetter(Node):
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Parameters
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('spin_speed', 0.8)
        self.declare_parameter('spin_duration', 10.0)
        
        self.x = self.get_parameter('init_x').value
        self.y = self.get_parameter('init_y').value
        self.yaw = self.get_parameter('init_yaw').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.spin_duration = self.get_parameter('spin_duration').value

        # Publishers
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # State Machine
        self.start_time = time.monotonic()
        self.pose_published = False
        self.spin_started = False
        
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(f"InitialPoseSetter: Starting in namespace: {self.get_namespace()}")

    def control_loop(self):
        now = time.monotonic()
        elapsed = now - self.start_time

        if not self.pose_published and elapsed > 2.0:
            self.publish_initial_pose()
            self.pose_published = True

        elif self.pose_published and elapsed > 3.0:
            spin_elapsed = elapsed - 3.0
            if spin_elapsed < self.spin_duration:
                msg = Twist()
                msg.angular.z = float(self.spin_speed)
                self.vel_pub.publish(msg)
                if not self.spin_started:
                    self.get_logger().warn("InitialPoseSetter: Starting auto-localization spin.")
                    self.spin_started = True
            else:
                self.stop_robot()
                self.get_logger().info("InitialPoseSetter: Spin complete.")
                self.timer.cancel()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map" # 공유 맵은 보통 전역 'map' 프레임 사용
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        self.pose_pub.publish(msg)

    def stop_robot(self):
        self.vel_pub.publish(Twist())

def main():
    rclpy.init()
    node = InitialPoseSetter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
