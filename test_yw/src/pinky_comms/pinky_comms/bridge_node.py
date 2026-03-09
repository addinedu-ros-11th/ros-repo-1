import socket
import struct
import threading
import time
import subprocess
import os
from collections import deque
from typing import Deque, Optional, Tuple, List

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool


class CommunicationBridgeNode(Node):
    """
    Pinky Pro AI Server Bridge (UDP Data Plane)
    Streams image data to AI server via UDP and monitors connectivity.
    """

    def __init__(self) -> None:
        super().__init__("communication_bridge")

        # Parameters
        self.declare_parameter("image_topic", "camera/image_raw")
        self.declare_parameter("camera_source", "topic")  # topic or rpicam
        self.declare_parameter("ai_server_ip", "127.0.0.1")
        self.declare_parameter("ai_server_port", 54321)
        self.declare_parameter("encoding_mode", "mjpeg")  # mjpeg or h264
        self.declare_parameter("udp_payload_max", 1400)
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("resize_width", 640)
        self.declare_parameter("resize_height", 360)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("ai_link_topic", "ai_link")
        self.declare_parameter("ai_healthcheck_port", 50052)
        self.declare_parameter("ai_healthcheck_period_sec", 2.0)
        self.declare_parameter("skip_stream_when_ai_dead", True)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_source = self.get_parameter("camera_source").get_parameter_value().string_value.lower()
        self.ai_server_ip = self.get_parameter("ai_server_ip").get_parameter_value().string_value
        self.ai_server_port = self.get_parameter("ai_server_port").get_parameter_value().integer_value
        self.encoding_mode = self.get_parameter("encoding_mode").get_parameter_value().string_value.lower()
        self.udp_payload_max = self.get_parameter("udp_payload_max").get_parameter_value().integer_value
        self.max_fps = self.get_parameter("max_fps").get_parameter_value().double_value
        self.resize_width = self.get_parameter("resize_width").get_parameter_value().integer_value
        self.resize_height = self.get_parameter("resize_height").get_parameter_value().integer_value
        self.jpeg_quality = self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        self.ai_link_topic = self.get_parameter("ai_link_topic").get_parameter_value().string_value
        self.ai_healthcheck_port = self.get_parameter("ai_healthcheck_port").get_parameter_value().integer_value
        self.ai_healthcheck_period_sec = self.get_parameter("ai_healthcheck_period_sec").get_parameter_value().double_value
        self.skip_stream_when_ai_dead = self.get_parameter("skip_stream_when_ai_dead").get_parameter_value().bool_value

        self._ai_link_alive = False
        self._stop_event = threading.Event()
        self._frame_id = (int(time.time() * 1000) ^ os.getpid()) & 0xFFFFFFFF
        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_target = (self.ai_server_ip, self.ai_server_port)

        # Publisher for AI link status (Namespaced)
        link_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.ai_link_pub = self.create_publisher(Bool, self.ai_link_topic, link_qos)

        # Worker setup
        self._image_queue: Deque[Image] = deque(maxlen=1)
        self._image_event = threading.Event()
        
        if self.camera_source == "topic":
            self.image_sub = self.create_subscription(Image, self.image_topic, self._on_image, 10)
            self._worker_thread = threading.Thread(target=self._image_worker, daemon=True)
            self._worker_thread.start()
        else:
            self.get_logger().error(f"Camera source '{self.camera_source}' not yet fully integrated in this version.")

        # Healthcheck timer
        self.ai_check_timer = self.create_timer(self.ai_healthcheck_period_sec, self._on_ai_healthcheck_timer)

        self.get_logger().info(f"Pinky Comms Bridge started. Target AI Server: {self.ai_server_ip}:{self.ai_server_port}")

    def _on_image(self, msg: Image) -> None:
        if self.skip_stream_when_ai_dead and not self._ai_link_alive:
            return
        self._image_queue.append(msg)
        self._image_event.set()

    def _image_worker(self) -> None:
        next_send_time = 0.0
        while rclpy.ok() and not self._stop_event.is_set():
            if not self._image_event.wait(timeout=0.1):
                continue
            self._image_event.clear()
            if not self._image_queue:
                continue

            now = time.monotonic()
            if now < next_send_time:
                continue
            next_send_time = now + (1.0 / self.max_fps)

            msg = self._image_queue.pop()
            frame = self._ros_image_to_cv(msg)
            if frame is None:
                continue

            if self.resize_width > 0 and self.resize_height > 0:
                frame = cv2.resize(frame, (self.resize_width, self.resize_height))

            _, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
            self._send_udp(encoded.tobytes())

    def _ros_image_to_cv(self, msg: Image) -> Optional[np.ndarray]:
        try:
            if msg.encoding == "bgr8":
                return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == "rgb8":
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return None
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return None

    def _send_udp(self, data: bytes) -> None:
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        chunk_size = self.udp_payload_max
        total_chunks = (len(data) + chunk_size - 1) // chunk_size

        for i in range(total_chunks):
            header = struct.pack("<III", self._frame_id, i, total_chunks)
            payload = data[i * chunk_size : (i + 1) * chunk_size]
            try:
                self._udp_sock.sendto(header + payload, self._udp_target)
            except Exception:
                pass

    def _on_ai_healthcheck_timer(self) -> None:
        # Simple TCP connect check
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        reachable = sock.connect_ex((self.ai_server_ip, self.ai_healthcheck_port)) == 0
        sock.close()

        if reachable != self._ai_link_alive:
            self._ai_link_alive = reachable
            self.ai_link_pub.publish(Bool(data=self._ai_link_alive))
            self.get_logger().info(f"AI Link status changed: {'ALIVE' if reachable else 'DEAD'}")

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._image_event.set()
        try:
            self._udp_sock.close()
        except:
            pass
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CommunicationBridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
