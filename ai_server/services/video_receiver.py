"""
UDP Video Stream Receiver + State-Aware Inference Processor
로봇 카메라로부터 UDP로 영상 프레임을 수신하고,
InferenceStateManager 상태에 따라 활성화된 모델로만 추론 수행
"""

import socket
import logging
import struct
import time
import numpy as np
import cv2
from typing import Optional, Callable, Dict, Any, List
from queue import Queue, Empty
import threading

logger = logging.getLogger(__name__)


class UDPVideoReceiver:
    """
    UDP를 통해 로봇 카메라에서 영상 프레임을 수신하는 클래스.
    항상 수신하되, 프레임 처리는 VideoStreamProcessor에서 상태에 따라 결정.
    """

    def __init__(
        self, host: str = "0.0.0.0", port: int = 54321, buffer_size: int = 65536
    ):
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.socket = None
        self.is_running = False
        self.frame_queue: Queue = Queue(maxsize=30)
        self.receive_thread = None

        logger.info(f"UDP Video Receiver 초기화: {host}:{port}")

    def start(self):
        if self.is_running:
            logger.warning("UDP Receiver가 이미 실행 중입니다")
            return

        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size * 10
            )
            self.socket.bind((self.host, self.port))

            self.is_running = True
            self.receive_thread = threading.Thread(
                target=self._receive_loop, daemon=True
            )
            self.receive_thread.start()

            logger.info(f"UDP Video Receiver 시작: {self.host}:{self.port}")

        except Exception as e:
            logger.error(f"UDP Receiver 시작 실패: {e}")
            raise

    def stop(self):
        self.is_running = False
        if self.socket:
            self.socket.close()
            self.socket = None
        if self.receive_thread:
            self.receive_thread.join(timeout=2)
        logger.info("UDP Video Receiver 중지")

    def _receive_loop(self):
        """
        UDP 패킷 수신 루프.
        패킷 헤더: [frame_id:4B][packet_id:4B][total_packets:4B][data]
        """
        frame_buffer: Dict[int, Dict[int, bytes]] = {}

        while self.is_running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)

                if len(data) < 12:
                    continue

                frame_id = struct.unpack("I", data[0:4])[0]
                packet_id = struct.unpack("I", data[4:8])[0]
                total_packets = struct.unpack("I", data[8:12])[0]
                packet_data = data[12:]

                if frame_id not in frame_buffer:
                    frame_buffer[frame_id] = {}

                frame_buffer[frame_id][packet_id] = packet_data

                # 모든 패킷 도착 → 프레임 조립
                if len(frame_buffer[frame_id]) == total_packets:
                    frame_data = b"".join(
                        [frame_buffer[frame_id][i] for i in range(total_packets)]
                    )

                    frame = self._decode_frame(frame_data)
                    if frame is not None:
                        # 큐가 가득 차면 오래된 프레임 제거
                        if self.frame_queue.full():
                            try:
                                self.frame_queue.get_nowait()
                            except Empty:
                                pass

                        self.frame_queue.put(
                            {
                                "frame": frame,
                                "frame_id": frame_id,
                                "robot_id": addr[0],
                                "timestamp": time.time(),
                            }
                        )

                    del frame_buffer[frame_id]

                # 오래된 불완전 프레임 정리
                if len(frame_buffer) > 100:
                    old_frames = sorted(frame_buffer.keys())[:50]
                    for old_id in old_frames:
                        del frame_buffer[old_id]

            except Exception as e:
                if self.is_running:
                    logger.error(f"UDP 수신 중 오류: {e}")

    def _decode_frame(self, frame_data: bytes) -> Optional[np.ndarray]:
        try:
            nparr = np.frombuffer(frame_data, np.uint8)
            return cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        except Exception as e:
            logger.error(f"프레임 디코딩 실패: {e}")
            return None

    def get_frame(self, timeout: float = 1.0) -> Optional[dict]:
        try:
            return self.frame_queue.get(timeout=timeout)
        except Empty:
            return None


class VideoStreamProcessor:
    """
    상태 기반 비디오 스트림 처리 엔진.

    InferenceStateManager의 상태에 따라 각 로봇의 프레임을
    활성화된 모델(EMPLOYEE/SNACK/OBSTACLE)로만 추론하고,
    결과를 results_queue에 저장하여 gRPC StreamVisionResults로 전달.
    """

    def __init__(
        self,
        receiver: UDPVideoReceiver,
        vision_service,
        state_manager,
        inference_interval: float = 0.3,
    ):
        """
        Args:
            receiver: UDPVideoReceiver 인스턴스
            vision_service: VisionService 인스턴스
            state_manager: InferenceStateManager 인스턴스
            inference_interval: 프레임 처리 최소 간격 (초)
        """
        self.receiver = receiver
        self.vision_service = vision_service
        self.state_manager = state_manager
        self.inference_interval = inference_interval

        self.is_running = False
        self.process_thread = None

        # 추론 결과 큐 — VisionServicer.StreamVisionResults에서 소비
        self.results_queue: Queue = Queue(maxsize=200)

        # 로봇별 마지막 추론 시각 {robot_id: float}
        self._last_inference_time: Dict[str, float] = {}

        logger.info("VideoStreamProcessor 초기화 (상태 기반)")

    def start(self):
        if self.is_running:
            return
        self.is_running = True
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()
        logger.info("VideoStreamProcessor 시작")

    def stop(self):
        self.is_running = False
        if self.process_thread:
            self.process_thread.join(timeout=2)
        logger.info("VideoStreamProcessor 중지")

    def _process_loop(self):
        """
        메인 처리 루프:
        1. UDP 큐에서 프레임 꺼냄
        2. 해당 robot_id의 활성 추론 상태 확인
        3. 활성 모델별 추론 수행
        4. 결과를 results_queue에 push
        """
        while self.is_running:
            try:
                frame_data = self.receiver.get_frame(timeout=0.1)
                if frame_data is None:
                    continue

                robot_id = frame_data["robot_id"]
                frame = frame_data["frame"]
                ts = frame_data["timestamp"]

                # 해당 로봇에 활성 추론이 없으면 스킵
                active_models = self.state_manager.get_active_models(robot_id)
                if not active_models:
                    continue

                # 추론 간격 제어
                last_time = self._last_inference_time.get(robot_id, 0)
                if (ts - last_time) < self.inference_interval:
                    continue
                self._last_inference_time[robot_id] = ts

                # 활성 모델별 추론 수행
                timestamp_ms = int(ts * 1000)

                for model_type in active_models:
                    try:
                        self._run_inference(robot_id, model_type, frame, timestamp_ms)
                    except Exception as e:
                        logger.error(
                            f"추론 실행 오류: robot={robot_id}, "
                            f"model={model_type}, error={e}"
                        )

            except Exception as e:
                if self.is_running:
                    logger.error(f"프레임 처리 루프 오류: {e}")

    def _run_inference(
        self, robot_id: str, model_type: str, frame: np.ndarray, timestamp_ms: int
    ):
        """
        모델 타입에 따라 적절한 추론을 수행하고 결과를 큐에 push.

        Args:
            robot_id: 로봇 식별자
            model_type: EMPLOYEE / SNACK / OBSTACLE
            frame: OpenCV BGR 프레임
            timestamp_ms: 타임스탬프 (ms)
        """
        if model_type == "EMPLOYEE":
            result = self.vision_service.recognize_face_from_frame(frame)
            # 얼굴을 찾았을 때만 결과 전송
            if result["person_type"] != "Unknown":
                self._push_result(
                    {
                        "robot_id": robot_id,
                        "timestamp": timestamp_ms,
                        "type": "face_recognition",
                        "content": result,
                    }
                )

        elif model_type == "SNACK":
            detections = self.vision_service.detect_products_from_frame(frame)
            if detections:
                # 가장 높은 신뢰도의 감지 결과 전송
                best = max(detections, key=lambda d: d["confidence"])
                self._push_result(
                    {
                        "robot_id": robot_id,
                        "timestamp": timestamp_ms,
                        "type": "object_detection",
                        "content": best,
                    }
                )

        elif model_type == "OBSTACLE":
            detections = self.vision_service.detect_obstacles_from_frame(frame)
            if detections:
                self._push_result(
                    {
                        "robot_id": robot_id,
                        "timestamp": timestamp_ms,
                        "type": "multi_objects",
                        "content": detections,
                    }
                )

    def _push_result(self, result: Dict[str, Any]):
        """결과를 큐에 push. 큐가 가득 차면 오래된 결과 제거."""
        if self.results_queue.full():
            try:
                self.results_queue.get_nowait()
            except Empty:
                pass
        self.results_queue.put(result)

    def get_result(self, timeout: float = 1.0) -> Optional[Dict[str, Any]]:
        """결과 큐에서 하나 꺼냄 (VisionServicer에서 호출)."""
        try:
            return self.results_queue.get(timeout=timeout)
        except Empty:
            return None
