"""
Vision YOLO 추론 결과가 gRPC 스트림으로 메인서버까지 잘 전달되는지 E2E 테스트

테스트 흐름:
  1. Vision 서버가 실행 중이어야 함 (50052 포트)
  2. gRPC 클라이언트로 UpdateInferenceState(robot_id="test", OBSTACLE, True) 전송
  3. 웹캠 프레임 또는 기본 테스트 이미지를 UDP로 전송 (로봇 카메라 시뮬레이션)
  4. StreamVisionResults 구독하여 추론 결과 수신 확인
  5. 결과 출력 및 검증

사용법:
  python tests/test_vision_stream.py              # 웹캠 사용
  python tests/test_vision_stream.py --no-cam     # 합성 이미지 사용 (감지 안 될 수 있음)
"""

import asyncio
import socket
import struct
import sys
import os
import time
import cv2
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

import grpc
from ai_server.grpc_impl import ai_vision_pb2, ai_vision_pb2_grpc

# ── 설정 ──
VISION_GRPC_ADDR = "localhost:50052"
UDP_TARGET = ("127.0.0.1", 54321)
ROBOT_ID = "127.0.0.1"         # UDP 소스 IP가 robot_id로 사용됨
MODEL_TYPE = "OBSTACLE"
TEST_TIMEOUT = 15               # 최대 대기 시간 (초)
MAX_UDP_PACKET = 60000          # UDP 페이로드 최대 크기
NUM_FRAMES = 20                 # 전송할 프레임 수


def create_test_frame() -> np.ndarray:
    """테스트용 컬러 프레임 생성 (YOLO 모델이 감지 못 할 수 있음 — fallback용)"""
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[300:, :] = (180, 180, 180)
    frame[:300, :] = (220, 220, 220)
    cv2.rectangle(frame, (200, 100), (300, 350), (50, 50, 200), -1)
    cv2.circle(frame, (250, 85), 30, (50, 50, 200), -1)
    cv2.rectangle(frame, (450, 200), (550, 350), (139, 90, 43), -1)
    cv2.rectangle(frame, (80, 280), (160, 360), (40, 40, 40), -1)
    return frame


def capture_webcam_frames(num_frames: int = 5) -> list:
    """웹캠에서 프레임 캡처 (사람/의자 등 실제 객체 감지용)"""
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("  ⚠️ 웹캠을 열 수 없습니다. 합성 이미지를 사용합니다.")
        return [create_test_frame() for _ in range(num_frames)]

    frames = []
    # 웹캠 워밍업
    for _ in range(10):
        cap.read()

    for _ in range(num_frames):
        ret, frame = cap.read()
        if ret:
            frames.append(frame)
        else:
            frames.append(create_test_frame())
        time.sleep(0.05)

    cap.release()
    print(f"  📷 웹캠에서 {len(frames)}개 프레임 캡처 완료")
    return frames


def send_frame_udp(frame: np.ndarray, frame_id: int):
    """프레임을 UDP로 전송 (로봇 카메라 시뮬레이션)"""
    # JPEG 인코딩
    _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 90])
    data = buffer.tobytes()

    # 패킷 분할
    total_packets = (len(data) + MAX_UDP_PACKET - 1) // MAX_UDP_PACKET
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    for i in range(total_packets):
        start = i * MAX_UDP_PACKET
        end = min(start + MAX_UDP_PACKET, len(data))
        chunk = data[start:end]

        # 헤더: [frame_id:u32LE][packet_id:u32LE][total_packets:u32LE]
        header = struct.pack("<III", frame_id, i, total_packets)
        sock.sendto(header + chunk, UDP_TARGET)

    sock.close()


async def test_vision_stream():
    """메인 테스트 로직"""
    print("\n" + "=" * 70)
    print("  [테스트] Vision YOLO 추론 → gRPC 스트림 전달 검증")
    print("=" * 70)

    # 1. gRPC 채널 연결
    print(f"\n[1/4] gRPC 연결: {VISION_GRPC_ADDR}")
    try:
        channel = grpc.aio.insecure_channel(VISION_GRPC_ADDR)
        stub = ai_vision_pb2_grpc.VisionServiceStub(channel)
        # 연결 확인 (간단한 RPC 호출)
        await asyncio.wait_for(channel.channel_ready(), timeout=5)
        print("  ✅ Vision 서버 연결 성공")
    except Exception as e:
        print(f"  ❌ Vision 서버 연결 실패: {e}")
        print("  → Vision 서버가 실행 중인지 확인하세요")
        print("  → 실행 방법: python -m ai_server.vision_server")
        return False

    # 2. 추론 상태 활성화
    print(f"\n[2/4] 추론 상태 활성화: robot={ROBOT_ID}, model={MODEL_TYPE}")
    try:
        req = ai_vision_pb2.InferenceStateRequest(
            robot_id=ROBOT_ID, model_type=MODEL_TYPE, is_active=True
        )
        resp = await stub.UpdateInferenceState(req)
        print(f"  ✅ 추론 활성화: success={resp.success}, msg={resp.message}")
    except Exception as e:
        print(f"  ❌ 추론 상태 변경 실패: {e}")
        await channel.close()
        return False

    # 3. UDP로 테스트 프레임 전송 (웹캠 또는 합성)
    use_cam = "--no-cam" not in sys.argv
    if use_cam:
        print(f"\n[3/4] 웹캠에서 프레임 캡처 후 UDP 전송 ({UDP_TARGET})")
        frames = capture_webcam_frames(NUM_FRAMES)
    else:
        print(f"\n[3/4] 합성 테스트 프레임 UDP 전송 ({UDP_TARGET})")
        frames = [create_test_frame() for _ in range(NUM_FRAMES)]

    for i, f in enumerate(frames):
        send_frame_udp(f, frame_id=i)
        await asyncio.sleep(0.12)
    print(f"  ✅ {len(frames)} 프레임 전송 완료")

    # 4. StreamVisionResults 구독 및 결과 수신 대기
    print(f"\n[4/4] gRPC 스트림 구독 — 추론 결과 수신 대기 (최대 {TEST_TIMEOUT}s)...")
    received_results = []

    try:
        stream = stub.StreamVisionResults(ai_vision_pb2.Empty())

        async def read_stream():
            async for result in stream:
                data = {}
                data["robot_id"] = result.robot_id
                data["timestamp"] = result.timestamp

                if result.HasField("object_detection"):
                    data["type"] = "object_detection"
                    data["object_name"] = result.object_detection.object_name
                    data["confidence"] = result.object_detection.confidence
                    box = result.object_detection.box
                    data["box"] = f"({box.x},{box.y},{box.width},{box.height})"

                elif result.HasField("face_recognition"):
                    data["type"] = "face_recognition"
                    data["person_type"] = result.face_recognition.person_type
                    data["confidence"] = result.face_recognition.confidence

                elif result.HasField("multi_objects"):
                    data["type"] = "multi_objects"
                    data["count"] = len(result.multi_objects.objects)
                    data["objects"] = []
                    for obj in result.multi_objects.objects:
                        data["objects"].append({
                            "name": obj.object_name,
                            "conf": f"{obj.confidence:.2f}",
                            "box": f"({obj.box.x},{obj.box.y},{obj.box.width},{obj.box.height})",
                        })

                received_results.append(data)
                print(f"  📦 수신 [{len(received_results)}]: {data}")

                if len(received_results) >= 5:
                    break

        await asyncio.wait_for(read_stream(), timeout=TEST_TIMEOUT)

    except asyncio.TimeoutError:
        print(f"  ⏱️ {TEST_TIMEOUT}초 타임아웃 도달")
    except grpc.aio.AioRpcError as e:
        print(f"  ⚠️ 스트림 오류: {e.code()} - {e.details()}")

    # 추론 비활성화
    try:
        req = ai_vision_pb2.InferenceStateRequest(
            robot_id=ROBOT_ID, model_type=MODEL_TYPE, is_active=False
        )
        await stub.UpdateInferenceState(req)
    except Exception:
        pass

    await channel.close()

    # ── 결과 요약 ──
    print("\n" + "=" * 70)
    if received_results:
        print(f"  ✅ 테스트 성공! {len(received_results)}개 추론 결과 수신 완료")
        for i, r in enumerate(received_results, 1):
            rtype = r.get("type", "unknown")
            if rtype == "multi_objects":
                names = [o["name"] for o in r.get("objects", [])]
                print(f"    [{i}] {rtype}: {r.get('count')}개 객체 → {names}")
            elif rtype == "object_detection":
                print(f"    [{i}] {rtype}: {r.get('object_name')} ({r.get('confidence', 0):.2f})")
            elif rtype == "face_recognition":
                print(f"    [{i}] {rtype}: {r.get('person_type')} ({r.get('confidence', 0):.2f})")
    else:
        print("  ❌ 추론 결과를 수신하지 못했습니다")
        print("  가능한 원인:")
        print("    - YOLO 모델(obstacle.pt)이 테스트 프레임에서 객체를 감지하지 못함")
        print("    - UDP 프레임이 VideoStreamProcessor에 도달하지 못함")
        print("    - inference_interval이 너무 길어서 처리되지 않음")
    print("=" * 70 + "\n")

    return len(received_results) > 0


if __name__ == "__main__":
    success = asyncio.run(test_vision_stream())
    sys.exit(0 if success else 1)
