"""
UDP Video Stream Receiver
로봇 카메라로부터 UDP로 영상 프레임을 수신
"""

import socket
import logging
import asyncio
import struct
import numpy as np
import cv2
from typing import Optional, Callable
from queue import Queue
import threading

logger = logging.getLogger(__name__)


class UDPVideoReceiver:
    """
    UDP를 통해 로봇 카메라에서 영상 프레임을 수신하는 클래스
    """

    def __init__(
        self, host: str = "0.0.0.0", port: int = 54321, buffer_size: int = 65536
    ):
        """
        UDP Video Receiver 초기화

        Args:
            host: 수신 호스트
            port: 수신 포트
            buffer_size: UDP 버퍼 크기
        """
        self.host = host
        self.port = port
        self.buffer_size = buffer_size
        self.socket = None
        self.is_running = False
        self.frame_queue = Queue(maxsize=10)
        self.receive_thread = None

        logger.info(f"UDP Video Receiver 초기화: {host}:{port}")

    def start(self):
        """
        UDP 수신 시작
        """
        if self.is_running:
            logger.warning("UDP Receiver가 이미 실행 중입니다")
            return

        try:
            # UDP 소켓 생성
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_RCVBUF, self.buffer_size * 10
            )
            self.socket.bind((self.host, self.port))

            self.is_running = True

            # 수신 스레드 시작
            self.receive_thread = threading.Thread(
                target=self._receive_loop, daemon=True
            )
            self.receive_thread.start()

            logger.info(f"UDP Video Receiver 시작: {self.host}:{self.port}")

        except Exception as e:
            logger.error(f"UDP Receiver 시작 실패: {e}")
            raise

    def stop(self):
        """
        UDP 수신 중지
        """
        self.is_running = False

        if self.socket:
            self.socket.close()
            self.socket = None

        if self.receive_thread:
            self.receive_thread.join(timeout=2)

        logger.info("UDP Video Receiver 중지")

    def _receive_loop(self):
        """
        UDP 패킷 수신 루프 (별도 스레드에서 실행)
        """
        frame_buffer = {}

        while self.is_running:
            try:
                # UDP 패킷 수신
                data, addr = self.socket.recvfrom(self.buffer_size)

                # 패킷 헤더 파싱
                # 형식: [frame_id:4bytes][packet_id:4bytes][total_packets:4bytes][data]
                if len(data) < 12:
                    continue

                frame_id = struct.unpack("I", data[0:4])[0]
                packet_id = struct.unpack("I", data[4:8])[0]
                total_packets = struct.unpack("I", data[8:12])[0]
                packet_data = data[12:]

                # 프레임 버퍼에 패킷 저장
                if frame_id not in frame_buffer:
                    frame_buffer[frame_id] = {}

                frame_buffer[frame_id][packet_id] = packet_data

                # 모든 패킷이 도착했는지 확인
                if len(frame_buffer[frame_id]) == total_packets:
                    # 패킷 순서대로 조합
                    frame_data = b"".join(
                        [frame_buffer[frame_id][i] for i in range(total_packets)]
                    )

                    # JPEG 디코딩
                    frame = self._decode_frame(frame_data)

                    if frame is not None:
                        # 큐에 프레임 추가 (큐가 가득 차면 오래된 프레임 제거)
                        if self.frame_queue.full():
                            try:
                                self.frame_queue.get_nowait()
                            except:
                                pass

                        self.frame_queue.put(
                            {
                                "frame": frame,
                                "frame_id": frame_id,
                                "robot_id": addr[0],  # IP 주소를 robot_id로 사용
                                "timestamp": None,  # TODO: 타임스탬프 추가
                            }
                        )

                    # 완료된 프레임 버퍼 삭제
                    del frame_buffer[frame_id]

                # 오래된 불완전한 프레임 정리 (메모리 누수 방지)
                if len(frame_buffer) > 100:
                    old_frames = sorted(frame_buffer.keys())[:50]
                    for old_id in old_frames:
                        del frame_buffer[old_id]

            except Exception as e:
                if self.is_running:
                    logger.error(f"UDP 수신 중 오류: {e}")

    def _decode_frame(self, frame_data: bytes) -> Optional[np.ndarray]:
        """
        JPEG 데이터를 OpenCV 프레임으로 디코딩

        Args:
            frame_data: JPEG 인코딩된 프레임 데이터

        Returns:
            OpenCV 프레임 (numpy array) 또는 None
        """
        try:
            # JPEG 디코딩
            nparr = np.frombuffer(frame_data, np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            return frame
        except Exception as e:
            logger.error(f"프레임 디코딩 실패: {e}")
            return None

    def get_frame(self, timeout: float = 1.0) -> Optional[dict]:
        """
        큐에서 프레임 가져오기

        Args:
            timeout: 대기 시간 (초)

        Returns:
            프레임 정보 딕셔너리 또는 None
        """
        try:
            return self.frame_queue.get(timeout=timeout)
        except:
            return None

    def has_frame(self) -> bool:
        """
        큐에 프레임이 있는지 확인

        Returns:
            프레임 존재 여부
        """
        return not self.frame_queue.empty()


class VideoStreamProcessor:
    """
    수신한 비디오 스트림을 처리하고 Vision 서비스와 연동
    """

    def __init__(
        self,
        receiver: UDPVideoReceiver,
        vision_service,
        callback: Optional[Callable] = None,
    ):
        """
        Video Stream Processor 초기화

        Args:
            receiver: UDPVideoReceiver 인스턴스
            vision_service: VisionService 인스턴스
            callback: 처리 결과 콜백 함수
        """
        self.receiver = receiver
        self.vision_service = vision_service
        self.callback = callback
        self.is_running = False
        self.process_thread = None

        logger.info("Video Stream Processor 초기화")

    def start(self):
        """
        비디오 스트림 처리 시작
        """
        if self.is_running:
            logger.warning("Video Stream Processor가 이미 실행 중입니다")
            return

        self.is_running = True
        self.process_thread = threading.Thread(target=self._process_loop, daemon=True)
        self.process_thread.start()

        logger.info("Video Stream Processor 시작")

    def stop(self):
        """
        비디오 스트림 처리 중지
        """
        self.is_running = False

        if self.process_thread:
            self.process_thread.join(timeout=2)

        logger.info("Video Stream Processor 중지")

    def _process_loop(self):
        """
        비디오 프레임 처리 루프
        """
        while self.is_running:
            try:
                # 프레임 가져오기
                frame_data = self.receiver.get_frame(timeout=0.1)

                if frame_data is None:
                    continue

                frame = frame_data["frame"]
                robot_id = frame_data["robot_id"]
                frame_id = frame_data["frame_id"]

                # Vision 서비스로 객체 인식 수행
                result = self.vision_service.detect_objects_from_frame(frame)

                # 결과에 메타데이터 추가
                result["robot_id"] = robot_id
                result["frame_id"] = frame_id

                logger.debug(
                    f"프레임 처리 완료: robot={robot_id}, frame={frame_id}, objects={result.get('object_name', 'none')}"
                )

                # 콜백 호출 (Main Server로 결과 전송)
                if self.callback:
                    self.callback(result)

            except Exception as e:
                if self.is_running:
                    logger.error(f"프레임 처리 중 오류: {e}")
