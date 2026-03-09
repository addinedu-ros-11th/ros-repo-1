import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from pinkylib import Battery 

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        
        self.battery = Battery()

        # Topics (상대 경로를 사용하여 네임스페이스 자동 적용)
        self.percentage_publisher = self.create_publisher(Float32, 'battery/present', 10)
        self.voltage_publisher = self.create_publisher(Float32, 'battery/voltage', 10)

        # Timer (파라미터화 가능하도록 처리)
        self.declare_parameter('publish_period', 5.0)
        timer_period = self.get_parameter('publish_period').value
        
        self.timer = self.create_timer(timer_period, self.publish_callback)
        self.get_logger().info(f"Battery Publisher started in namespace: {self.get_namespace()}")

    def publish_callback(self):
        try:
            # Percentage
            pct_msg = Float32()
            pct_msg.data = float(self.battery.battery_percentage())
            self.percentage_publisher.publish(pct_msg)

            # Voltage
            volt_msg = Float32()
            volt_msg.data = float(self.battery.get_voltage()) 
            self.voltage_publisher.publish(volt_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read battery data: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
