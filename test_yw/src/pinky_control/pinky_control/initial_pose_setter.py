import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
import math
import time

class InitialPoseSetter(Node):
    """
    AMCL 로컬라이제이션 활성화를 위해 초기 위치 설정 후 360도 회전을 수행하는 노드입니다.
    네임스페이스를 지원하기 위해 토픽 이름에 상대 경로를 사용합니다.
    """
    def __init__(self):
        super().__init__('initial_pose_setter')
        
        # Parameters
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('spin_speed', 0.8)    # 속도를 약간 높임 (0.6 -> 0.8)
        self.declare_parameter('spin_duration', 10.0) # 속도가 높아졌으므로 시간 조절
        
        self.x = self.get_parameter('init_x').value
        self.y = self.get_parameter('init_y').value
        self.yaw = self.get_parameter('init_yaw').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.spin_duration = self.get_parameter('spin_duration').value

        # Publishers (상대 경로 사용하여 네임스페이스 자동 적용)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # State Machine
        self.start_time = time.monotonic()
        self.pose_published = False
        self.spin_started = False
        
        # 0.1초 주기로 상태 체크 및 제어
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info(f"InitialPoseSetter: Starting auto-localization in namespace: {self.get_namespace()}")

    def control_loop(self):
        now = time.monotonic()
        elapsed = now - self.start_time

        # 1. 초기 좌표 발행 (시작 후 2초 지점 - 안정화 대기)
        if not self.pose_published and elapsed > 2.0:
            self.publish_initial_pose()
            self.pose_published = True
            self.get_logger().info("InitialPoseSetter: Pose published. Starting spin in 1s...")

        # 2. 360도 회전 시작 (시작 후 3초 지점부터)
        elif self.pose_published and elapsed > 3.0:
            spin_elapsed = elapsed - 3.0
            
            if spin_elapsed < self.spin_duration:
                self.send_spin_cmd()
                if not self.spin_started:
                    self.get_logger().warn("InitialPoseSetter: !!! ROBOT SHOULD START SPINNING NOW !!!")
                    self.spin_started = True
            else:
                # 3. 회전 완료 및 종료
                self.stop_robot()
                self.get_logger().info("InitialPoseSetter: Spin complete. Localization should be ready.")
                self.timer.cancel()

    def publish_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(self.x)
        msg.pose.pose.position.y = float(self.y)
        msg.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        msg.pose.covariance = [0.0] * 36
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.06
        self.pose_pub.publish(msg)

    def send_spin_cmd(self):
        msg = Twist()
        msg.angular.z = float(self.spin_speed)
        self.vel_pub.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.vel_pub.publish(msg)

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
