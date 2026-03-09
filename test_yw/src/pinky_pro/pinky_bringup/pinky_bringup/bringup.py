#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32

from .dynamixel_driver import DynamixelDriver

class Pinky(Node):
    def __init__(self):
        super().__init__('pinky_bringup')
        
        # Parameters for Frame IDs
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_footprint')
        
        self.odom_frame = self.get_parameter('odom_frame_id').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame_id').get_parameter_value().string_value

        self.get_logger().info(f'Starting Pinky with frames: {self.odom_frame} -> {self.base_frame}')
        
        # Hardcoded constants (remain same for hardware)
        self.SERIAL_PORT_NAME = "/dev/ttyAMA4"
        self.BAUDRATE = 1000000
        self.DYNAMIXEL_IDS = [1, 2]
        self.WHEEL_RAD = 0.028
        self.PULSE_PER_ROT = 4096 
        self.WHEEL_BASE = 0.0961
        self.RPM2RAD = 2 * math.pi / 60
        self.CIRCUMFERENCE = 2 * math.pi * self.WHEEL_RAD

        self.driver = DynamixelDriver(self.SERIAL_PORT_NAME, self.BAUDRATE, self.DYNAMIXEL_IDS)

        if not self.driver.begin() or not self.driver.initialize_motors():
            self.get_logger().error("Hardware initialization failed!")
            return

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.twist_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish)

        self.x, self.y, self.theta = 0.0, 0.0, 0.0
        _, _, self.last_encoder_l, self.last_encoder_r = self.driver.get_feedback()
        self.last_time = self.get_clock().now()

    def twist_callback(self, msg: Twist):
        v_l = msg.linear.x - (msg.angular.z * self.WHEEL_BASE / 2.0)
        v_r = msg.linear.x + (msg.angular.z * self.WHEEL_BASE / 2.0)
        rpm_l = (v_l / self.WHEEL_RAD) * 60.0 / (2 * math.pi)
        rpm_r = -(v_r / self.WHEEL_RAD) * 60.0 / (2 * math.pi)
        self.driver.set_double_rpm(rpm_l, rpm_r)

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        feedback = self.driver.get_feedback()
        if feedback[0] is None: return
        rpm_l, rpm_r, encoder_l, encoder_r = feedback

        delta_l = (encoder_l - self.last_encoder_l) / self.PULSE_PER_ROT * self.CIRCUMFERENCE
        delta_r = -(encoder_r - self.last_encoder_r) / self.PULSE_PER_ROT * self.CIRCUMFERENCE
        
        self.last_encoder_l, self.last_encoder_r = encoder_l, encoder_r

        d_dist = (delta_r + delta_l) / 2.0
        d_theta = (delta_r - delta_l) / self.WHEEL_BASE
        
        self.theta += d_theta
        self.x += d_dist * math.cos(self.theta)
        self.y += d_dist * math.sin(self.theta)
        
        # Publish TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

        # Publish Odom
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = self.x, self.y
        odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = d_dist/dt, d_theta/dt
        self.odom_pub.publish(odom)

        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = Pinky()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
