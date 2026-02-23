"""
gRPC를 통한 구조화된 응답 테스트
LLM 서버와의 실제 통신을 테스트
"""

import grpc
import sys
import os

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from ai_server.grpc_impl import ai_llm_pb2
from ai_server.grpc_impl import ai_llm_pb2_grpc


def print_result(req_id: str, text: str, response):
    """gRPC 응답을 보기 좋게 출력"""
    print(f"\n{'='*70}")
    print(f"요청 ID: {req_id}")
    print(f"입력: {text}")
    print(f"{'-'*70}")
    print(f"🎯 작업 유형: {ai_llm_pb2.TaskType.Name(response.task_type)}")
    print(f"✓ 신뢰도: {response.confidence:.2f}")

    struct_msg = response.struct_msg
    print(f"📋 구조화된 메시지:")

    # 공통 필드
    if struct_msg.HasField("location"):
        print(f"  📍 location: {struct_msg.location}")

    # 사용자 정보
    if struct_msg.HasField("requester_name"):
        print(f"  👤 requester_name: {struct_msg.requester_name}")
    if struct_msg.HasField("receiver_name"):
        print(f"  👥 receiver_name: {struct_msg.receiver_name}")
    if struct_msg.HasField("visitor_name"):
        print(f"  👋 visitor_name: {struct_msg.visitor_name}")

    # 배달 관련
    if struct_msg.HasField("source_location"):
        print(f"  🚀 source_location: {struct_msg.source_location}")
    if struct_msg.HasField("dest_location"):
        print(f"  🎯 dest_location: {struct_msg.dest_location}")
    if len(struct_msg.items) > 0:
        for item in struct_msg.items:
            print(f"  📦 item: {item.item_name} x{item.quantity}")

    # 기타
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
    stub = ai_llm_pb2_grpc.LLMServiceStub(channel)

    # 테스트 케이스
    test_cases = [
        ("req_001", "회의실로 커피 갖다줘"),
        ("req_002", "301호에 서류 전달해줘"),
        ("req_003", "김철수한테 노트북 전달해줘"),
        ("req_004", "방문객 이영희님을 회의실로 안내해줘"),
        ("req_005", "안녕하세요"),
        ("req_006", "간식 창고에서 과자 3개 가져와줘"),
        ("req_007", "오늘 날씨 어때?"),
    ]

    print("\n테스트 시작...\n")

    success_count = 0
    fail_count = 0

    # 각 테스트 실행
    for i, (req_id, message) in enumerate(test_cases, 1):
        print(f"[테스트 {i}/{len(test_cases)}]")
        try:
            # gRPC 요청
            request = ai_llm_pb2.NLRequest(req_id=req_id, message=message)

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
