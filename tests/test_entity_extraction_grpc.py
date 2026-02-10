#!/usr/bin/env python3
"""
gRPC를 통한 구조화 응답 테스트
LLM 서버가 실행 중이어야 함
"""

import sys
import logging
import grpc
from pathlib import Path

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, str(Path(__file__).parent.parent))

from ai_server.grpc_impl import ai_services_pb2
from ai_server.grpc_impl import ai_services_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


def test_entity_extraction_grpc(server_address: str = "localhost:50051"):
    """
    gRPC를 통한 엔티티 추출 테스트

    Args:
        server_address: LLM 서버 주소
    """
    logger.info("=" * 60)
    logger.info("gRPC 구조화 응답 테스트 시작")
    logger.info(f"서버: {server_address}")
    logger.info("=" * 60)

    # gRPC 채널 생성
    try:
        channel = grpc.insecure_channel(server_address)
        stub = ai_services_pb2_grpc.LLMServiceStub(channel)
        logger.info("✓ gRPC 채널 생성 완료\n")
    except Exception as e:
        logger.error(f"❌ gRPC 채널 생성 실패: {e}")
        logger.error("LLM 서버가 실행 중인지 확인하세요.")
        sys.exit(1)

    # 테스트 케이스들
    test_cases = [
        "회의실로 커피 가져다줘",
        "301호에 서류 전달해줘",
        "로비로 이동해줘",
        "노트북 찾아줘",
        "3층 회의실에 물 한잔 가져다줘",
        "안녕하세요",  # 장소/물품 없음
        "A동 2층으로 가서 박스를 가져와줘",
    ]

    logger.info("=" * 60)
    logger.info("테스트 케이스 실행")
    logger.info("=" * 60 + "\n")

    for i, test_input in enumerate(test_cases, 1):
        logger.info(f"\n[테스트 {i}]")
        logger.info(f"입력: {test_input}")
        logger.info("-" * 40)

        try:
            # gRPC 요청 생성
            request = ai_services_pb2.NLRequest(req_id=f"test_{i}", message=test_input)

            # ParseNaturalLanguage RPC 호출
            response = stub.ParseNaturalLanguage(request)

            # 결과 출력
            logger.info(
                f"🎯 TaskType: {ai_services_pb2.TaskType.Name(response.task_type)}"
            )
            logger.info(f"✓ Confidence: {response.confidence:.2f}")

            struct_msg = response.struct_msg
            fields = []
            for key in (
                "location",
                "item",
                "person_name",
                "source_location",
                "dest_location",
                "room_id",
                "meeting_room_id",
                "area",
            ):
                if struct_msg.HasField(key):
                    fields.append(f"{key}={getattr(struct_msg, key)}")

            if fields:
                logger.info(f"📋 Structured: {', '.join(fields)}")
            else:
                logger.info("📋 Structured: 없음")

        except grpc.RpcError as e:
            logger.error(f"❌ gRPC 오류: {e.code()} - {e.details()}")
        except Exception as e:
            logger.error(f"❌ 처리 실패: {e}")

    # 채널 종료
    channel.close()

    logger.info("\n" + "=" * 60)
    logger.info("테스트 완료")
    logger.info("=" * 60)


def main():
    """메인 함수"""
    import argparse

    parser = argparse.ArgumentParser(description="LLM 엔티티 추출 gRPC 테스트")
    parser.add_argument(
        "--server",
        default="localhost:50051",
        help="LLM 서버 주소 (기본값: localhost:50051)",
    )

    args = parser.parse_args()

    test_entity_extraction_grpc(args.server)


if __name__ == "__main__":
    main()
