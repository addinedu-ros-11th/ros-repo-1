import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import numpy as np
import cv2 
import time

try:
    from picamera2 import Picamera2
except ImportError:
    Picamera2 = None

class PinkyCameraNode(Node):
    """
    Native Raspberry Pi Camera Node using Picamera2.
    Publishes both Raw and Compressed images in standard BGR format.
    """
    def __init__(self):
        super().__init__("camera_node")
        
        # --- Parameters ---
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 20)
        self.declare_parameter("frame_id", "camera_link")
        self.declare_parameter("hflip", True)
        self.declare_parameter("vflip", True)
        self.declare_parameter("rotation", 0)
        self.declare_parameter("quality", 80)

        self.width = self.get_parameter("width").value
        self.height = self.get_parameter("height").value
        self.fps = self.get_parameter("fps").value
        self.frame_id = self.get_parameter("frame_id").value
        self.hflip = self.get_parameter("hflip").value
        self.vflip = self.get_parameter("vflip").value
        self.rotation = self.get_parameter("rotation").value
        self.quality = self.get_parameter("quality").value

        self.raw_pub = self.create_publisher(Image, "camera/image_raw", 10)
        self.compressed_pub = self.create_publisher(CompressedImage, "camera/image_raw/compressed", 10)

        if Picamera2 is None:
            self.get_logger().error("Picamera2 library not found!")
            return

        try:
            self.picam2 = Picamera2()
            # Picamera2 outputs BGR888 directly for OpenCV
            config = self.picam2.create_preview_configuration(main={"format": "BGR888", "size": (self.width, self.height)})
            self.picam2.configure(config)
            self.picam2.start()
            
            self.get_logger().info(f"Picamera2 started ({self.width}x{self.height} @ {self.fps}fps) in BGR mode")
            self.timer = self.create_timer(1.0 / self.fps, self._timer_callback)
        except Exception as e:
            self.get_logger().error(f"Failed to initialize Picamera2: {e}")

    def _timer_callback(self):
        try:
            # 1. Capture original frame (BGR)
            frame_bgr = self.picam2.capture_array()
            
            # 2. Post-process using OpenCV
            if self.hflip and self.vflip:
                frame_bgr = cv2.flip(frame_bgr, -1)
            elif self.hflip:
                frame_bgr = cv2.flip(frame_bgr, 1)
            elif self.vflip:
                frame_bgr = cv2.flip(frame_bgr, 0)
            
            if self.rotation != 0:
                if self.rotation == 90: frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_CLOCKWISE)
                elif self.rotation == 180: frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_180)
                elif self.rotation == 270: frame_bgr = cv2.rotate(frame_bgr, cv2.ROTATE_90_COUNTERCLOCKWISE)

            stamp = self.get_clock().now().to_msg()

            # 3-1. Publish Raw Image (BGR8)
            raw_msg = Image()
            raw_msg.header.stamp = stamp
            raw_msg.header.frame_id = self.frame_id
            raw_msg.height, raw_msg.width, _ = frame_bgr.shape
            raw_msg.encoding = "bgr8" # Standard OpenCV encoding
            raw_msg.step = frame_bgr.shape[1] * 3
            raw_msg.data = frame_bgr.tobytes()
            self.raw_pub.publish(raw_msg)

            # 3-2. Publish Compressed Image (JPEG)
            comp_msg = CompressedImage()
            comp_msg.header.stamp = stamp
            comp_msg.header.frame_id = self.frame_id
            comp_msg.format = "jpeg"
            # cv2.imencode expects BGR for JPEG encoding
            _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            comp_msg.data = buffer.tobytes()
            self.compressed_pub.publish(comp_msg)

        except Exception as e:
            self.get_logger().warn(f"Frame capture failed: {e}")

    def destroy_node(self):
        if hasattr(self, 'picam2'):
            try: self.picam2.stop()
            except: pass
        super().destroy_node()

def main():
    rclpy.init()
    node = PinkyCameraNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
