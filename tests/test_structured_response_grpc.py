"""
gRPC를 통한 구조화된 응답 테스트
LLM 서버와의 실제 통신을 테스트
"""

import grpc
import sys
import os

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ai_server.grpc_impl import ai_services_pb2
from ai_server.grpc_impl import ai_services_pb2_grpc


def print_result(req_id: str, text: str, response):
    """gRPC 응답을 보기 좋게 출력"""
    print(f"\n{'='*70}")
    print(f"요청 ID: {req_id}")
    print(f"입력: {text}")
    print(f"{'-'*70}")
    print(f"🎯 작업 유형: {ai_services_pb2.TaskType.Name(response.task_type)}")
    print(f"✓ 신뢰도: {response.confidence:.2f}")

    struct_msg = response.struct_msg
    print(f"📋 구조화된 메시지:")

    # 공통 필드
    if struct_msg.HasField("location"):
        print(f"  📍 location: {struct_msg.location}")
    if struct_msg.HasField("item"):
        print(f"  📦 item: {struct_msg.item}")
    if struct_msg.HasField("person_name"):
        print(f"  👤 person_name: {struct_msg.person_name}")
    if struct_msg.HasField("person_id"):
        print(f"  🆔 person_id: {struct_msg.person_id}")

    # 배달 관련
    if struct_msg.HasField("source_location"):
        print(f"  🚀 source_location: {struct_msg.source_location}")
    if struct_msg.HasField("dest_location"):
        print(f"  🎯 dest_location: {struct_msg.dest_location}")
    if struct_msg.HasField("quantity"):
        print(f"  🔢 quantity: {struct_msg.quantity}")

    # IoT 제어
    if struct_msg.HasField("device_type"):
        print(
            f"  🔌 device_type: {ai_services_pb2.IoTDeviceType.Name(struct_msg.device_type)}"
        )
    if struct_msg.HasField("command"):
        print(
            f"  ⚙️  command: {ai_services_pb2.IoTCommandType.Name(struct_msg.command)}"
        )
    if struct_msg.HasField("target_value"):
        print(f"  🎚️  target_value: {struct_msg.target_value}")
    if struct_msg.HasField("room_id"):
        print(f"  🏠 room_id: {struct_msg.room_id}")

    # 회의실 관련
    if struct_msg.HasField("meeting_room_id"):
        print(f"  🏢 meeting_room_id: {struct_msg.meeting_room_id}")
    if struct_msg.HasField("start_time"):
        print(f"  ⏰ start_time: {struct_msg.start_time}")
    if struct_msg.HasField("end_time"):
        print(f"  ⏱️  end_time: {struct_msg.end_time}")
    if struct_msg.HasField("attendee_count"):
        print(f"  👥 attendee_count: {struct_msg.attendee_count}")

    # 순찰 관련
    if struct_msg.HasField("area"):
        print(f"  🗺️  area: {struct_msg.area}")
    if len(struct_msg.waypoints) > 0:
        print(f"  🚶 waypoints: {', '.join(struct_msg.waypoints)}")

    # 기타
    if struct_msg.HasField("query_type"):
        print(f"  ❓ query_type: {struct_msg.query_type}")
    if struct_msg.HasField("message"):
        print(f"  💬 message: {struct_msg.message}")
    if len(struct_msg.keywords) > 0:
        print(f"  🏷️  keywords: {', '.join(struct_msg.keywords)}")

    print(f"{'='*70}\n")


def test_grpc_structured_response():
    """gRPC를 통한 구조화된 응답 테스트"""

    print("\n" + "=" * 70)
    print("gRPC 구조화된 응답 테스트")
    print("서버 주소: localhost:50051")
    print("=" * 70)

    # gRPC 채널 및 스텁 생성
    channel = grpc.insecure_channel("localhost:50051")
    stub = ai_services_pb2_grpc.LLMServiceStub(channel)

    # 테스트 케이스
    test_cases = [
        ("req_001", "회의실로 커피 갖다줘"),
        ("req_002", "301호에 서류 전달해줘"),
        ("req_003", "방문객을 3층 회의실로 안내해줘"),
        ("req_004", "로비로 이동해"),
        ("req_005", "여기로 와줘"),
        ("req_006", "충전하러 가"),
        ("req_007", "작업 취소해"),
        ("req_008", "회의실 불 켜줘"),
        ("req_009", "온도 25도로 맞춰줘"),
        ("req_010", "에어컨 켜줘"),
        ("req_011", "로봇 상태 어때?"),
        ("req_012", "회의실이 어디야?"),
        ("req_013", "박영희씨 어디있어?"),
        ("req_014", "오후 2시에 회의실 예약해줘"),
        ("req_015", "2층 사무실 순찰해줘"),
        ("req_016", "안녕하세요"),
        ("req_017", "간식 창고에서 과자 3개 가져와줘"),
        ("req_018", "나를 따라와"),
        ("req_019", "문 잠가줘"),
        ("req_020", "환경 상태 체크해줘"),
    ]

    print("\n테스트 시작...\n")

    success_count = 0
    fail_count = 0

    # 각 테스트 실행
    for i, (req_id, message) in enumerate(test_cases, 1):
        print(f"[테스트 {i}/{len(test_cases)}]")
        try:
            # gRPC 요청
            request = ai_services_pb2.NLRequest(req_id=req_id, message=message)

            # RPC 호출
            response = stub.ParseNaturalLanguage(request)

            # 결과 출력
            print_result(req_id, message, response)
            success_count += 1

        except grpc.RpcError as e:
            print(f"❌ gRPC 오류: {e.code()} - {e.details()}\n")
            fail_count += 1
        except Exception as e:
            print(f"❌ 오류 발생: {e}\n")
            fail_count += 1

    # 결과 요약
    print("\n" + "=" * 70)
    print(f"테스트 완료: 성공 {success_count}, 실패 {fail_count}")
    print("=" * 70)

    channel.close()


if __name__ == "__main__":
    try:
        test_grpc_structured_response()
    except KeyboardInterrupt:
        print("\n테스트 중단됨")
    except Exception as e:
        print(f"\n치명적 오류: {e}")
