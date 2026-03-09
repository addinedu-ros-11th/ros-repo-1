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
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool


class CommunicationBridgeNode(Node):
    """
    Communication node bridging:
    - Control plane: ROS 2 topics exposed via rosbridge (handled externally)
    - Data plane: /camera/image_raw -> MJPEG/H.264 UDP stream
    """

    def __init__(self) -> None:
        super().__init__("communication_bridge")

        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("camera_source", "topic")  # topic or rpicam
        self.declare_parameter("ai_server_ip", "127.0.0.1")
        self.declare_parameter("ai_server_port", 54321)
        self.declare_parameter("encoding_mode", "mjpeg")  # mjpeg or h264
        self.declare_parameter("udp_payload_max", 1400)
        self.declare_parameter("max_fps", 8.0)
        self.declare_parameter("resize_width", 640)
        self.declare_parameter("resize_height", 360)
        self.declare_parameter("rotate_180", False)
        self.declare_parameter("jpeg_quality", 70)
        self.declare_parameter("udp_warn_throttle_sec", 5.0)
        self.declare_parameter("tx_stats_log_period_sec", 5.0)
        self.declare_parameter("frame_id_seed", -1)
        self.declare_parameter("ai_link_topic", "/robot01/ai_link")
        self.declare_parameter("publish_compressed_topic", True)
        self.declare_parameter("compressed_image_topic", "/camera/image_raw/compressed")
        self.declare_parameter("ai_healthcheck_enabled", True)
        self.declare_parameter("ai_healthcheck_mode", "tcp_port")  # tcp_port or none
        self.declare_parameter("ai_healthcheck_port", 50052)
        self.declare_parameter("ai_healthcheck_period_sec", 2.0)
        self.declare_parameter("ai_healthcheck_timeout_sec", 0.4)
        self.declare_parameter("ai_healthcheck_fail_threshold", 3)
        self.declare_parameter("ai_healthcheck_recover_threshold", 1)
        self.declare_parameter("skip_stream_when_ai_dead", True)
        self.declare_parameter("ai_dead_log_period_sec", 30.0)
        self.declare_parameter("rpicam_cmd", "rpicam-vid")
        self.declare_parameter("rpicam_still_cmd", "rpicam-still")
        self.declare_parameter("rpicam_restart_backoff_sec", 2.0)
        self.declare_parameter("rpicam_use_system_libs", True)
        self.declare_parameter("rpicam_awb_mode", "auto")
        self.declare_parameter("rpicam_awb_autoselect", False)
        self.declare_parameter(
            "rpicam_awb_candidates", "fluorescent,daylight,cloudy,tungsten,indoor"
        )
        self.declare_parameter("rpicam_awb_probe_width", 640)
        self.declare_parameter("rpicam_awb_probe_height", 360)
        self.declare_parameter("rpicam_awb_probe_timeout_ms", 1200)

        self.image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        self.camera_source = (
            self.get_parameter("camera_source").get_parameter_value().string_value.lower()
        )
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
        self.max_fps = self.get_parameter("max_fps").get_parameter_value().double_value
        self.resize_width = self.get_parameter("resize_width").get_parameter_value().integer_value
        self.resize_height = self.get_parameter("resize_height").get_parameter_value().integer_value
        self.rotate_180 = self.get_parameter("rotate_180").get_parameter_value().bool_value
        self.jpeg_quality = self.get_parameter("jpeg_quality").get_parameter_value().integer_value
        self.udp_warn_throttle_sec = (
            self.get_parameter("udp_warn_throttle_sec").get_parameter_value().double_value
        )
        self.tx_stats_log_period_sec = (
            self.get_parameter("tx_stats_log_period_sec").get_parameter_value().double_value
        )
        self.frame_id_seed = (
            self.get_parameter("frame_id_seed").get_parameter_value().integer_value
        )
        self.ai_link_topic = self.get_parameter("ai_link_topic").get_parameter_value().string_value
        self.publish_compressed_topic = (
            self.get_parameter("publish_compressed_topic").get_parameter_value().bool_value
        )
        self.compressed_image_topic = (
            self.get_parameter("compressed_image_topic").get_parameter_value().string_value
        )
        self.ai_healthcheck_enabled = (
            self.get_parameter("ai_healthcheck_enabled").get_parameter_value().bool_value
        )
        self.ai_healthcheck_mode = (
            self.get_parameter("ai_healthcheck_mode").get_parameter_value().string_value.lower()
        )
        self.ai_healthcheck_port = (
            self.get_parameter("ai_healthcheck_port").get_parameter_value().integer_value
        )
        self.ai_healthcheck_period_sec = (
            self.get_parameter("ai_healthcheck_period_sec").get_parameter_value().double_value
        )
        self.ai_healthcheck_timeout_sec = (
            self.get_parameter("ai_healthcheck_timeout_sec").get_parameter_value().double_value
        )
        self.ai_healthcheck_fail_threshold = (
            self.get_parameter("ai_healthcheck_fail_threshold").get_parameter_value().integer_value
        )
        self.ai_healthcheck_recover_threshold = (
            self.get_parameter("ai_healthcheck_recover_threshold")
            .get_parameter_value()
            .integer_value
        )
        self.skip_stream_when_ai_dead = (
            self.get_parameter("skip_stream_when_ai_dead").get_parameter_value().bool_value
        )
        self.ai_dead_log_period_sec = (
            self.get_parameter("ai_dead_log_period_sec").get_parameter_value().double_value
        )
        self.rpicam_cmd = self.get_parameter("rpicam_cmd").get_parameter_value().string_value
        self.rpicam_still_cmd = (
            self.get_parameter("rpicam_still_cmd").get_parameter_value().string_value
        )
        self.rpicam_restart_backoff_sec = (
            self.get_parameter("rpicam_restart_backoff_sec").get_parameter_value().double_value
        )
        self.rpicam_use_system_libs = (
            self.get_parameter("rpicam_use_system_libs").get_parameter_value().bool_value
        )
        self.rpicam_awb_mode = self._normalize_awb_mode(
            self.get_parameter("rpicam_awb_mode").get_parameter_value().string_value
        )
        self.rpicam_awb_autoselect = (
            self.get_parameter("rpicam_awb_autoselect").get_parameter_value().bool_value
        )
        self.rpicam_awb_candidates = self._parse_awb_candidates(
            self.get_parameter("rpicam_awb_candidates").get_parameter_value().string_value
        )
        self.rpicam_awb_probe_width = (
            self.get_parameter("rpicam_awb_probe_width").get_parameter_value().integer_value
        )
        self.rpicam_awb_probe_height = (
            self.get_parameter("rpicam_awb_probe_height").get_parameter_value().integer_value
        )
        self.rpicam_awb_probe_timeout_ms = (
            self.get_parameter("rpicam_awb_probe_timeout_ms")
            .get_parameter_value()
            .integer_value
        )
        self._selected_rpicam_awb_mode = self.rpicam_awb_mode
        self._awb_autoselect_done = False

        self._stop_event = threading.Event()
        self._rpicam_proc: Optional[subprocess.Popen] = None

        self.image_sub = None
        if self.camera_source == "topic":
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._on_image, 10
            )
        elif self.camera_source != "rpicam":
            self.get_logger().warn(
                f"Unknown camera_source={self.camera_source}; fallback to topic mode."
            )
            self.camera_source = "topic"
            self.image_sub = self.create_subscription(
                Image, self.image_topic, self._on_image, 10
            )

        link_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.ai_link_pub = self.create_publisher(Bool, self.ai_link_topic, link_qos)
        self.compressed_pub = None
        if self.publish_compressed_topic and self.compressed_image_topic:
            self.compressed_pub = self.create_publisher(
                CompressedImage, self.compressed_image_topic, 10
            )

        self._udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._udp_target: Tuple[str, int] = (self.ai_server_ip, self.ai_server_port)
        if self.frame_id_seed >= 0:
            self._frame_id = int(self.frame_id_seed) & 0xFFFFFFFF
        else:
            # Avoid frame-id collisions after restarts and across accidental duplicate senders.
            self._frame_id = (int(time.time() * 1000) ^ os.getpid()) & 0xFFFFFFFF
        self._next_frame_send_at = 0.0
        self._last_udp_warn_at = 0.0
        self._tx_stats_last_log_at = time.monotonic()
        self._tx_frames = 0
        self._tx_packets = 0
        self._tx_bytes = 0
        self._tx_failures = 0
        self._consecutive_ai_failures = 0
        self._consecutive_ai_success = 0
        self._last_ai_dead_log_at = 0.0
        self._ai_link_alive = True

        self._image_queue: Deque[Image] = deque(maxlen=1)
        self._image_event = threading.Event()
        self._image_thread = None
        self._rpicam_thread = None
        if self.camera_source == "topic":
            self._image_thread = threading.Thread(target=self._image_worker, daemon=True)
            self._image_thread.start()
        else:
            self._rpicam_thread = threading.Thread(target=self._rpicam_worker, daemon=True)
            self._rpicam_thread.start()

        self.ai_check_timer = self.create_timer(
            max(0.5, self.ai_healthcheck_period_sec), self._on_ai_healthcheck_timer
        )
        self.ai_link_pub.publish(Bool(data=self._ai_link_alive))

        self.get_logger().info(
            "Communication bridge ready "
            f"(source={self.camera_source}, ai={self.ai_server_ip}:{self.ai_server_port}, "
            f"encoding={self.encoding_mode}, "
            f"max_fps={self.max_fps}, resize={self.resize_width}x{self.resize_height}, "
            f"rotate_180={self.rotate_180}, "
            f"rpicam_awb={self._selected_rpicam_awb_mode}, "
            f"healthcheck={self.ai_healthcheck_mode}:{self.ai_healthcheck_port}, "
            f"frame_id_seed={self._frame_id}, tx_log={self.tx_stats_log_period_sec}s)."
        )

    def _on_image(self, msg: Image) -> None:
        # Save CPU/network when AI is unavailable and streaming should be paused.
        if (
            self.skip_stream_when_ai_dead
            and not self._ai_link_alive
            and self.compressed_pub is None
        ):
            return
        self._image_queue.append(msg)
        self._image_event.set()

    def _image_worker(self) -> None:
        while rclpy.ok() and not self._stop_event.is_set():
            self._image_event.wait(timeout=0.5)
            self._image_event.clear()
            if not self._image_queue:
                continue

            if self.max_fps > 0.0:
                now = time.monotonic()
                if now < self._next_frame_send_at:
                    continue
                self._next_frame_send_at = now + (1.0 / self.max_fps)

            msg = self._image_queue.pop()
            frame = self._ros_image_to_bgr(msg)
            if frame is None:
                continue
            frame = self._resize_frame(frame)
            if self.rotate_180:
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            encoded = self._encode_frame(frame)
            if encoded is None:
                continue
            self._publish_compressed_frame(encoded)
            if self.skip_stream_when_ai_dead and not self._ai_link_alive:
                continue
            self._send_udp_frame(encoded)

    def _build_rpicam_command(self) -> List[str]:
        fps = 8
        if self.max_fps > 0:
            fps = max(1, int(round(self.max_fps)))
        width = self.resize_width if self.resize_width > 0 else 640
        height = self.resize_height if self.resize_height > 0 else 360
        quality = int(max(10, min(95, self.jpeg_quality)))
        cmd = [
            self.rpicam_cmd,
            "--nopreview",
            "--codec",
            "mjpeg",
            "--quality",
            str(quality),
            "--width",
            str(width),
            "--height",
            str(height),
            "--framerate",
            str(fps),
            "--timeout",
            "0",
            "-o",
            "-",
        ]
        if self._selected_rpicam_awb_mode:
            cmd += ["--awb", self._selected_rpicam_awb_mode]
        if self.rotate_180:
            cmd += ["--hflip", "--vflip"]
        return cmd

    def _build_rpicam_env(self) -> dict:
        env = dict(os.environ)
        if self.rpicam_use_system_libs:
            # Prefer distro libcamera stack for stable pisp runtime on PinkyPro.
            env["LD_LIBRARY_PATH"] = "/usr/lib/aarch64-linux-gnu:/lib/aarch64-linux-gnu"
        return env

    def _normalize_awb_mode(self, mode: str) -> str:
        valid = {
            "auto",
            "incandescent",
            "tungsten",
            "fluorescent",
            "indoor",
            "daylight",
            "cloudy",
        }
        normalized = (mode or "auto").strip().lower()
        if normalized in valid:
            return normalized
        if normalized:
            self.get_logger().warn(f"Unknown rpicam_awb_mode={normalized}. Fallback to auto.")
        return "auto"

    def _parse_awb_candidates(self, raw: str) -> List[str]:
        out: List[str] = []
        for token in (raw or "").split(","):
            mode = self._normalize_awb_mode(token)
            if mode not in out:
                out.append(mode)
        return out

    def _awb_balance_score(self, frame: np.ndarray) -> float:
        h, w = frame.shape[:2]
        y0, y1 = h // 6, h - (h // 6)
        x0, x1 = w // 6, w - (w // 6)
        roi = frame[y0:y1, x0:x1]
        if roi.size == 0:
            roi = frame
        means = roi.reshape(-1, 3).mean(axis=0)
        means = means / max(1e-6, float(np.mean(means)))
        return float(np.std(means))

    def _capture_awb_probe_frame(self, awb_mode: str, env: dict) -> Optional[np.ndarray]:
        if not self.rpicam_still_cmd:
            return None
        width = max(160, int(self.rpicam_awb_probe_width))
        height = max(120, int(self.rpicam_awb_probe_height))
        timeout_ms = max(400, int(self.rpicam_awb_probe_timeout_ms))
        cmd = [
            self.rpicam_still_cmd,
            "--nopreview",
            "--immediate",
            "--width",
            str(width),
            "--height",
            str(height),
            "--awb",
            awb_mode,
            "--timeout",
            str(timeout_ms),
            "-o",
            "-",
        ]
        if self.rotate_180:
            cmd += ["--hflip", "--vflip"]
        try:
            result = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                env=env,
                timeout=max(3.0, timeout_ms / 1000.0 + 2.0),
                check=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"[awb] probe failed ({awb_mode}): {exc}")
            return None
        if result.returncode != 0:
            err = result.stderr.decode("utf-8", errors="ignore").strip().splitlines()
            tail = err[-1] if err else "unknown"
            self.get_logger().warn(f"[awb] probe command failed ({awb_mode}): {tail}")
            return None
        if not result.stdout:
            self.get_logger().warn(f"[awb] probe empty output ({awb_mode}).")
            return None
        frame = cv2.imdecode(np.frombuffer(result.stdout, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warn(f"[awb] probe decode failed ({awb_mode}).")
            return None
        return frame

    def _auto_select_awb_mode(self) -> str:
        env = self._build_rpicam_env()
        scores: List[Tuple[float, str]] = []
        for mode in self.rpicam_awb_candidates:
            frame = self._capture_awb_probe_frame(mode, env)
            if frame is None:
                continue
            score = self._awb_balance_score(frame)
            scores.append((score, mode))
            self.get_logger().info(f"[awb] probe mode={mode} score={score:.4f}")
        if not scores:
            self.get_logger().warn(
                "[awb] auto-select failed; keeping configured mode "
                f"{self._selected_rpicam_awb_mode}."
            )
            return self._selected_rpicam_awb_mode
        scores.sort(key=lambda item: item[0])
        selected = scores[0][1]
        self.get_logger().info(
            f"[awb] auto-selected mode={selected} from {len(scores)} candidates."
        )
        return selected

    def _extract_jpeg_frames(self, buffer: bytearray) -> List[bytes]:
        frames: List[bytes] = []
        while True:
            soi = buffer.find(b"\xff\xd8")
            if soi < 0:
                if len(buffer) > 2 * 1024 * 1024:
                    del buffer[:-1024]
                break
            eoi = buffer.find(b"\xff\xd9", soi + 2)
            if eoi < 0:
                if soi > 0:
                    del buffer[:soi]
                break
            frames.append(bytes(buffer[soi : eoi + 2]))
            del buffer[: eoi + 2]
        return frames

    def _rpicam_worker(self) -> None:
        if not self.rpicam_cmd:
            self.get_logger().error("camera_source=rpicam but rpicam_cmd is empty.")
            return

        while rclpy.ok() and not self._stop_event.is_set():
            if self.rpicam_awb_autoselect and not self._awb_autoselect_done:
                self._selected_rpicam_awb_mode = self._auto_select_awb_mode()
                self._awb_autoselect_done = True

            if (
                self.skip_stream_when_ai_dead
                and not self._ai_link_alive
                and self.compressed_pub is None
            ):
                time.sleep(0.5)
                continue

            cmd = self._build_rpicam_command()
            env = self._build_rpicam_env()
            self.get_logger().info(f"Starting rpicam source: {' '.join(cmd)}")
            try:
                self._rpicam_proc = subprocess.Popen(
                    cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    env=env,
                    bufsize=0,
                )
            except Exception as exc:
                self.get_logger().error(f"Failed to start rpicam process: {exc}")
                time.sleep(max(0.5, self.rpicam_restart_backoff_sec))
                continue

            stderr_thread = threading.Thread(
                target=self._drain_rpicam_stderr, args=(self._rpicam_proc,), daemon=True
            )
            stderr_thread.start()

            buffer = bytearray()
            stdout = self._rpicam_proc.stdout
            if stdout is None:
                self.get_logger().error("rpicam process has no stdout pipe.")
                self._stop_rpicam_process()
                time.sleep(max(0.5, self.rpicam_restart_backoff_sec))
                continue

            stopped_for_ai_dead = False
            while rclpy.ok() and not self._stop_event.is_set():
                if self.skip_stream_when_ai_dead and not self._ai_link_alive:
                    if self.compressed_pub is None:
                        stopped_for_ai_dead = True
                        break
                chunk = stdout.read(8192)
                if not chunk:
                    break
                buffer.extend(chunk)
                for frame in self._extract_jpeg_frames(buffer):
                    self._publish_compressed_frame(frame)
                    if self.skip_stream_when_ai_dead and not self._ai_link_alive:
                        continue
                    self._send_udp_frame(frame)

            exit_code = self._rpicam_proc.poll()
            if exit_code is None:
                self._stop_rpicam_process()
            else:
                self.get_logger().warn(f"rpicam process exited with code {exit_code}.")
                self._rpicam_proc = None

            if stopped_for_ai_dead:
                self.get_logger().info("AI link dead: paused rpicam capture.")

            if not self._stop_event.is_set():
                time.sleep(max(0.5, self.rpicam_restart_backoff_sec))

    def _drain_rpicam_stderr(self, proc: subprocess.Popen) -> None:
        if proc.stderr is None:
            return
        try:
            while rclpy.ok() and not self._stop_event.is_set():
                line = proc.stderr.readline()
                if not line:
                    break
                txt = line.decode("utf-8", errors="ignore").strip()
                if txt:
                    self.get_logger().info(f"[rpicam] {txt}")
        except Exception:
            return

    def _stop_rpicam_process(self) -> None:
        proc = self._rpicam_proc
        if proc is None:
            return
        try:
            proc.terminate()
            proc.wait(timeout=2.0)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        finally:
            self._rpicam_proc = None

    def _on_ai_healthcheck_timer(self) -> None:
        if not self.ai_healthcheck_enabled:
            return

        reachable = self._check_ai_reachable()
        previous = self._ai_link_alive

        if reachable:
            self._consecutive_ai_failures = 0
            self._consecutive_ai_success += 1
            if self._consecutive_ai_success >= max(1, self.ai_healthcheck_recover_threshold):
                self._ai_link_alive = True
        else:
            self._consecutive_ai_success = 0
            self._consecutive_ai_failures += 1
            if self._consecutive_ai_failures >= max(1, self.ai_healthcheck_fail_threshold):
                self._ai_link_alive = False

        self.ai_link_pub.publish(Bool(data=self._ai_link_alive))

        if previous != self._ai_link_alive:
            if self._ai_link_alive:
                self.get_logger().info(
                    f"AI link recovered ({self.ai_server_ip}:{self.ai_healthcheck_port})."
                )
            else:
                self._last_ai_dead_log_at = time.monotonic()
                self.get_logger().warn(
                    f"AI link marked dead ({self.ai_server_ip}:{self.ai_healthcheck_port}); "
                    "video streaming will pause until recovered."
                )
        elif not self._ai_link_alive:
            now = time.monotonic()
            if (now - self._last_ai_dead_log_at) >= max(5.0, self.ai_dead_log_period_sec):
                self._last_ai_dead_log_at = now
                self.get_logger().warn(
                    f"AI link still dead ({self.ai_server_ip}:{self.ai_healthcheck_port})."
                )

    def _check_ai_reachable(self) -> bool:
        if self.ai_healthcheck_mode == "none":
            return True
        if self.ai_healthcheck_mode != "tcp_port":
            return False

        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(max(0.1, self.ai_healthcheck_timeout_sec))
        try:
            return sock.connect_ex((self.ai_server_ip, int(self.ai_healthcheck_port))) == 0
        except OSError:
            return False
        finally:
            try:
                sock.close()
            except OSError:
                pass

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

        quality = int(max(10, min(95, self.jpeg_quality)))
        ok, buf = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), quality]
        )
        if not ok:
            self.get_logger().warn("MJPEG encode failed.")
            return None
        return buf.tobytes()

    def _resize_frame(self, frame: np.ndarray) -> np.ndarray:
        if self.resize_width <= 0 and self.resize_height <= 0:
            return frame

        src_h, src_w = frame.shape[:2]
        target_w = self.resize_width
        target_h = self.resize_height

        if target_w > 0 and target_h <= 0:
            target_h = int((src_h * target_w) / max(1, src_w))
        elif target_h > 0 and target_w <= 0:
            target_w = int((src_w * target_h) / max(1, src_h))
        elif target_w <= 0 and target_h <= 0:
            return frame

        target_w = max(1, int(target_w))
        target_h = max(1, int(target_h))
        if target_w == src_w and target_h == src_h:
            return frame
        return cv2.resize(frame, (target_w, target_h), interpolation=cv2.INTER_AREA)

    def _send_udp_frame(self, data: bytes) -> None:
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        payload_max = max(1, min(self.udp_payload_max, 1400))
        total_chunks = (len(data) + payload_max - 1) // payload_max
        if total_chunks == 0:
            return
        if total_chunks > 0xFFFFFFFF:
            self.get_logger().warn("Frame too large to chunk within u32 limit.")
            return

        frame_failed = False
        for idx in range(total_chunks):
            start = idx * payload_max
            end = min(start + payload_max, len(data))
            chunk = data[start:end]
            # ai_server/services/video_receiver.py expects legacy header:
            # [frame_id:u32][packet_id:u32][total_packets:u32]
            header = struct.pack("<III", self._frame_id, idx, total_chunks)
            packet = header + chunk
            try:
                self._udp_sock.sendto(packet, self._udp_target)
                self._tx_packets += 1
                self._tx_bytes += len(packet)
            except OSError as exc:
                self._tx_failures += 1
                frame_failed = True
                now = time.monotonic()
                if (now - self._last_udp_warn_at) >= max(1.0, self.udp_warn_throttle_sec):
                    self._last_udp_warn_at = now
                    self.get_logger().warn(f"UDP send failed: {exc}.")
                break

        if not frame_failed:
            self._tx_frames += 1
        self._maybe_log_tx_stats()

    def _maybe_log_tx_stats(self) -> None:
        period = max(1.0, self.tx_stats_log_period_sec)
        now = time.monotonic()
        elapsed = now - self._tx_stats_last_log_at
        if elapsed < period:
            return
        fps = self._tx_frames / max(1e-6, elapsed)
        kbps = (self._tx_bytes * 8.0 / 1000.0) / max(1e-6, elapsed)
        self.get_logger().info(
            "UDP TX stats: "
            f"frames={self._tx_frames} ({fps:.1f} fps), "
            f"packets={self._tx_packets}, "
            f"bitrate={kbps:.0f} kbps, "
            f"failures={self._tx_failures}, "
            f"target={self.ai_server_ip}:{self.ai_server_port}"
        )
        self._tx_frames = 0
        self._tx_packets = 0
        self._tx_bytes = 0
        self._tx_failures = 0
        self._tx_stats_last_log_at = now

    def _publish_compressed_frame(self, data: bytes) -> None:
        if self.compressed_pub is None:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = "jpeg"
        msg.data = data
        self.compressed_pub.publish(msg)

    def destroy_node(self) -> bool:
        self._stop_event.set()
        self._image_event.set()
        self._stop_rpicam_process()
        try:
            self._udp_sock.close()
        except OSError:
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
