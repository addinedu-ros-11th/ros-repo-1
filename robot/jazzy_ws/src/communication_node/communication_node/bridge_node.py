import socket
import struct
import threading
import time
from collections import deque
from typing import Deque, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CommunicationBridgeNode(Node):
    """
    Communication node bridging:
    - Control plane: ROS 2 topics exposed via rosbridge (handled externally)
    - Data plane: /camera/image_raw -> MJPEG/H.264 UDP stream
    """

    def __init__(self) -> None:
        super().__init__("communication_bridge")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("ai_server_ip", "127.0.0.1")
        self.declare_parameter("ai_server_port", 54321)
        self.declare_parameter("encoding_mode", "mjpeg")  # mjpeg or h264
        self.declare_parameter("udp_payload_max", 1400)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.ai_server_ip = self.get_parameter("ai_server_ip").get_parameter_value().string_value
        self.ai_server_port = (
            self.get_parameter("ai_server_port").get_parameter_value().integer_value
        )
        self.encoding_mode = (
            self.get_parameter("encoding_mode").get_parameter_value().string_value.lower()
        )
        self.udp_payload_max = (
            self.get_parameter("udp_payload_max").get_parameter_value().integer_value
        )

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._on_image, 10
        )

        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_target: Tuple[str, int] = (self.ai_server_ip, self.ai_server_port)
        self._frame_id = 0

        self._image_queue: Deque[Image] = deque(maxlen=1)
        self._image_event = threading.Event()
        self._image_thread = threading.Thread(target=self._image_worker, daemon=True)
        self._image_thread.start()

        self.get_logger().info(
            "Communication bridge ready "
            f"(ai={self.ai_server_ip}:{self.ai_server_port}, encoding={self.encoding_mode})."
        )

    def _on_image(self, msg: Image) -> None:
        self._image_queue.append(msg)
        self._image_event.set()

    def _image_worker(self) -> None:
        while rclpy.ok():
            self._image_event.wait(timeout=0.5)
            self._image_event.clear()
            if not self._image_queue:
                continue
            msg = self._image_queue.pop()
            frame = self._ros_image_to_bgr(msg)
            if frame is None:
                continue
            encoded = self._encode_frame(frame)
            if encoded is None:
                continue
            self._send_udp_frame(encoded)

    def _ros_image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding in {"bgr8", "rgb8"}:
            dtype = np.uint8
            row_stride = msg.step
            expected_row = msg.width * 3
            raw = np.frombuffer(msg.data, dtype=dtype)
            if row_stride == expected_row:
                frame = raw.reshape(msg.height, msg.width, 3)
            else:
                frame = raw.reshape(msg.height, row_stride)[:, :expected_row]
                frame = frame.reshape(msg.height, msg.width, 3)
            if msg.encoding == "rgb8":
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            return frame
        if msg.encoding == "mono8":
            dtype = np.uint8
            row_stride = msg.step
            raw = np.frombuffer(msg.data, dtype=dtype)
            if row_stride == msg.width:
                frame = raw.reshape(msg.height, msg.width)
            else:
                frame = raw.reshape(msg.height, row_stride)[:, : msg.width]
            return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}")
        return None

    def _encode_frame(self, frame: np.ndarray) -> Optional[bytes]:
        if self.encoding_mode == "h264":
            ok, buf = cv2.imencode(".h264", frame)
            if ok:
                return buf.tobytes()
            self.get_logger().warn("H.264 encode unsupported. Falling back to MJPEG.")

        ok, buf = cv2.imencode(".jpg", frame)
        if not ok:
            self.get_logger().warn("MJPEG encode failed.")
            return None
        return buf.tobytes()

    def _send_udp_frame(self, data: bytes) -> None:
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        timestamp = time.time_ns() & 0xFFFFFFFFFFFFFFFF
        payload_max = max(1, min(self.udp_payload_max, 1400))
        total_chunks = (len(data) + payload_max - 1) // payload_max
        if total_chunks == 0:
            return
        if total_chunks > 65535:
            self.get_logger().warn("Frame too large to chunk within u16 limit.")
            return

        for idx in range(total_chunks):
            start = idx * payload_max
            end = min(start + payload_max, len(data))
            chunk = data[start:end]
            header = struct.pack(">IQHH", self._frame_id, timestamp, idx, total_chunks)
            packet = header + chunk
            try:
                self._udp_sock.sendto(packet, self._udp_target)
            except OSError as exc:
                self.get_logger().warn(f"UDP send failed: {exc}.")
                break



def main() -> None:
    rclpy.init()
    node = CommunicationBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
