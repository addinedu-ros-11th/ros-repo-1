"""
UDP Video Stream Receiver + State-Aware Inference Processor
로봇 카메라로부터 UDP로 영상 프레임을 수신하고,
InferenceStateManager 상태에 따라 활성화된 모델로만 추론 수행
"""

import socket
import logging
import os
import json
import struct
import time
import numpy as np
import cv2
from pathlib import Path
from typing import Optional, Callable, Dict, Any, List
from queue import Queue, Empty
import threading

logger = logging.getLogger(__name__)

# GUI ↔ 서버 프로세스 간 테스트 모드 IPC 파일
TEST_MODE_FLAG_PATH = Path("/tmp/ai_server_test_mode.flag")
TEST_MODE_RESULT_PATH = Path("/tmp/ai_server_test_results.json")


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

        # 최신 프레임 미리보기용 (GUI 연동)
        self._preview_frame_path = Path("/tmp/ai_server_latest_frame.jpg")
        self._preview_interval = 0.1  # 100ms 간격으로 저장
        self._last_preview_time = 0.0

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
        # 미리보기 파일 정리
        try:
            self._preview_frame_path.unlink(missing_ok=True)
        except Exception:
            pass
        logger.info("UDP Video Receiver 중지")

    def _receive_loop(self):
        """
        UDP 패킷 수신 루프.
        패킷 헤더: [frame_id:u32LE][packet_id:u32LE][total_packets:u32LE][data]
        (로봇 bridge_node가 little-endian '<III'로 패킹)
        """
        frame_buffer: Dict[int, Dict[int, bytes]] = {}
        recv_packet_count = 0
        assembled_frame_count = 0
        decoded_frame_count = 0
        last_stats_time = time.time()

        while self.is_running:
            try:
                data, addr = self.socket.recvfrom(self.buffer_size)
                recv_packet_count += 1

                if len(data) < 12:
                    logger.warning(f"UDP 패킷 너무 짧음: {len(data)}B from {addr}")
                    continue

                frame_id, packet_id, total_packets = struct.unpack_from("<III", data, 0)
                packet_data = data[12:]

                # 첫 패킷 수신 시 로그
                if recv_packet_count == 1:
                    logger.info(
                        f"UDP 첫 패킷 수신: from={addr}, "
                        f"frame_id={frame_id}, packet_id={packet_id}, "
                        f"total_packets={total_packets}, data_len={len(packet_data)}"
                    )

                if frame_id not in frame_buffer:
                    frame_buffer[frame_id] = {}

                frame_buffer[frame_id][packet_id] = packet_data

                # 모든 패킷 도착 → 프레임 조립
                if len(frame_buffer[frame_id]) == total_packets:
                    assembled_frame_count += 1
                    frame_data = b"".join(
                        [frame_buffer[frame_id][i] for i in range(total_packets)]
                    )

                    frame = self._decode_frame(frame_data)
                    if frame is not None:
                        decoded_frame_count += 1
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

                        # GUI 미리보기용 프레임 저장 (throttled)
                        now = time.time()
                        if (now - self._last_preview_time) >= self._preview_interval:
                            self._save_preview_frame(frame)
                            self._last_preview_time = now
                    else:
                        logger.warning(
                            f"프레임 디코딩 실패: frame_id={frame_id}, "
                            f"size={len(frame_data)}B, "
                            f"header=0x{frame_data[:4].hex() if len(frame_data) >= 4 else 'N/A'}"
                        )

                    del frame_buffer[frame_id]

                # 주기적 수신 통계 로그 (10초마다)
                now = time.time()
                if (now - last_stats_time) >= 10.0:
                    logger.info(
                        f"UDP 수신 통계: packets={recv_packet_count}, "
                        f"assembled={assembled_frame_count}, "
                        f"decoded={decoded_frame_count}, "
                        f"pending_frames={len(frame_buffer)}"
                    )
                    recv_packet_count = 0
                    assembled_frame_count = 0
                    decoded_frame_count = 0
                    last_stats_time = now

                # 오래된 불완전 프레임 정리
                if len(frame_buffer) > 100:
                    old_frames = sorted(frame_buffer.keys())[:50]
                    for old_id in old_frames:
                        del frame_buffer[old_id]

            except Exception as e:
                if self.is_running:
                    logger.error(f"UDP 수신 중 오류: {e}", exc_info=True)

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

    def _save_preview_frame(self, frame: np.ndarray):
        """최신 프레임을 GUI 미리보기용 JPEG로 저장 (atomic write)"""
        try:
            tmp_path = str(self._preview_frame_path.parent / "_ai_preview_tmp.jpg")
            success = cv2.imwrite(tmp_path, frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not success:
                logger.warning(
                    f"cv2.imwrite 실패: shape={frame.shape}, dtype={frame.dtype}, "
                    f"path={tmp_path}"
                )
                return
            os.replace(tmp_path, str(self._preview_frame_path))
            logger.debug(f"미리보기 프레임 저장: {self._preview_frame_path}")
        except Exception as e:
            logger.error(f"미리보기 프레임 저장 실패: {e}", exc_info=True)


class VideoStreamProcessor:
    """
    상태 기반 비디오 스트림 처리 엔진.

    InferenceStateManager의 상태에 따라 각 로봇의 프레임을
    활성화된 모델(EMPLOYEE/OBSTACLE)로만 추론하고,
    결과를 results_queue에 저장하여 gRPC StreamVisionResults로 전달.
    """

    # 바운딩 박스 색상 (BGR)
    _COLORS = {
        "person": (0, 255, 0),  # 초록
        "chair": (255, 165, 0),  # 파랑+주황
        "potted_plant": (0, 200, 0),  # 녹색
        "bag": (0, 255, 255),  # 노랑
        "robot": (255, 0, 255),  # 핑크
        "Employee": (0, 255, 0),  # 초록 (직원)
        "Guest": (0, 165, 255),  # 주황 (방문객)
        "default": (128, 128, 255),  # 연빨강
    }

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

        # 테스트 모드 상태
        self._test_mode = False
        self._last_test_flag_check = 0.0
        self._test_flag_check_interval = 0.5  # 0.5초마다 플래그 파일 확인

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

    def _check_test_mode(self) -> bool:
        """테스트 모드 플래그 파일 존재 여부로 테스트 모드 확인 (throttled)"""
        now = time.time()
        if (now - self._last_test_flag_check) < self._test_flag_check_interval:
            return self._test_mode
        self._last_test_flag_check = now

        was_test = self._test_mode
        self._test_mode = TEST_MODE_FLAG_PATH.exists()
        if self._test_mode != was_test:
            if self._test_mode:
                logger.info("🧪 테스트 모드 활성화 — 모든 모델 추론 시작")
            else:
                logger.info("🧪 테스트 모드 비활성화")
        return self._test_mode

    def _process_loop(self):
        """
        메인 처리 루프:
        1. UDP 큐에서 프레임 꺼냄
        2. 해당 robot_id의 활성 추론 상태 확인 (테스트 모드면 전체 활성)
        3. 활성 모델별 추론 수행
        4. 결과를 results_queue에 push
        """
        while self.is_running:
            try:
                frame_data = self.receiver.get_frame(timeout=0.1)
                if frame_data is None:
                    # 프레임이 없어도 테스트 모드 플래그는 확인
                    self._check_test_mode()
                    continue

                robot_id = frame_data["robot_id"]
                frame = frame_data["frame"]
                ts = frame_data["timestamp"]

                test_mode = self._check_test_mode()

                # 활성 추론 모델 결정
                if test_mode:
                    active_models = {"EMPLOYEE", "OBSTACLE"}
                else:
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
                test_results = []  # 테스트 모드용 결과 수집

                for model_type in active_models:
                    try:
                        result = self._run_inference(
                            robot_id, model_type, frame, timestamp_ms
                        )
                        if test_mode and result is not None:
                            test_results.append(result)
                    except Exception as e:
                        logger.error(
                            f"추론 실행 오류: robot={robot_id}, "
                            f"model={model_type}, error={e}"
                        )

                # 테스트 모드: 바운딩 박스 그리고 미리보기 저장 + 결과 파일 쓰기
                if test_mode:
                    annotated = self._draw_annotations(frame, test_results)
                    self.receiver._save_preview_frame(annotated)
                    self._save_test_results(test_results)

            except Exception as e:
                if self.is_running:
                    logger.error(f"프레임 처리 루프 오류: {e}")

    def _run_inference(
        self, robot_id: str, model_type: str, frame: np.ndarray, timestamp_ms: int
    ) -> Optional[Dict[str, Any]]:
        """
        모델 타입에 따라 적절한 추론을 수행하고 결과를 큐에 push.
        테스트 모드 시 결과를 반환.

        Args:
            robot_id: 로봇 식별자
            model_type: EMPLOYEE / OBSTACLE
            frame: OpenCV BGR 프레임
            timestamp_ms: 타임스탬프 (ms)

        Returns:
            추론 결과 dict (테스트 모드용) 또는 None
        """
        if model_type == "EMPLOYEE":
            result = self.vision_service.recognize_face_from_frame(frame)
            if result["person_type"] != "Unknown":
                entry = {
                    "robot_id": robot_id,
                    "timestamp": timestamp_ms,
                    "type": "face_recognition",
                    "content": result,
                }
                self._push_result(entry)
                return entry
            return None

        elif model_type == "OBSTACLE":
            detections = self.vision_service.detect_obstacles_from_frame(frame)
            if detections:
                entry = {
                    "robot_id": robot_id,
                    "timestamp": timestamp_ms,
                    "type": "multi_objects",
                    "content": detections,
                }
                self._push_result(entry)
                return entry
            return None

        return None

    def _draw_annotations(
        self, frame: np.ndarray, results: List[Dict[str, Any]]
    ) -> np.ndarray:
        """추론 결과에 바운딩 박스와 라벨을 그린 프레임 반환."""
        annotated = frame.copy()

        for result in results:
            if result["type"] == "multi_objects":
                for det in result["content"]:
                    box = det["box"]
                    x, y, w, h = box["x"], box["y"], box["width"], box["height"]
                    name = det["object_name"]
                    conf = det["confidence"]
                    color = self._COLORS.get(name, self._COLORS["default"])

                    cv2.rectangle(annotated, (x, y), (x + w, y + h), color, 2)

                    label = f"{name} {conf:.0%}"
                    (tw, th), _ = cv2.getTextSize(
                        label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                    )
                    cv2.rectangle(
                        annotated, (x, y - th - 6), (x + tw + 4, y), color, -1
                    )
                    cv2.putText(
                        annotated,
                        label,
                        (x + 2, y - 4),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 0),
                        1,
                    )

            elif result["type"] == "face_recognition":
                content = result["content"]
                person_type = content["person_type"]
                conf = content.get("confidence", 0)
                emp_id = content.get("employee_id", "")
                color = self._COLORS.get(person_type, self._COLORS["default"])

                # 프레임 상단에 얼굴 인식 결과 표시
                if person_type == "Employee":
                    label = f"[FACE] {emp_id} ({conf:.0%})"
                else:
                    label = f"[FACE] {person_type} ({conf:.0%})"

                cv2.putText(
                    annotated,
                    label,
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    color,
                    2,
                )

        return annotated

    def _save_test_results(self, results: List[Dict[str, Any]]):
        """테스트 모드 결과를 JSON 파일로 저장 (GUI에서 읽기 위함)"""
        try:
            summary = {
                "timestamp": time.time(),
                "results": [],
            }
            for r in results:
                if r["type"] == "multi_objects":
                    for det in r["content"]:
                        summary["results"].append(
                            {
                                "type": "obstacle",
                                "name": det["object_name"],
                                "confidence": det["confidence"],
                            }
                        )
                elif r["type"] == "face_recognition":
                    c = r["content"]
                    summary["results"].append(
                        {
                            "type": "face",
                            "person_type": c["person_type"],
                            "employee_id": c.get("employee_id", ""),
                            "confidence": c.get("confidence", 0),
                        }
                    )

            tmp = str(TEST_MODE_RESULT_PATH) + ".tmp"
            with open(tmp, "w") as f:
                json.dump(summary, f)
            os.replace(tmp, str(TEST_MODE_RESULT_PATH))
        except Exception:
            pass

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
