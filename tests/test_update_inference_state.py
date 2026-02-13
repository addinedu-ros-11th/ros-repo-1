#!/usr/bin/env python3
"""
UpdateInferenceState RPC 메서드 테스트
Vision 서버와의 실제 gRPC 통신 테스트
"""

import sys
import asyncio
import logging
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

import grpc
from ai_server.grpc_impl import ai_vision_pb2, ai_vision_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def test_update_inference_state(server_address: str = "localhost:50052"):
    """
    UpdateInferenceState RPC 메서드 테스트

    Args:
        server_address: Vision 서버 주소
    """
    logger.info("=" * 70)
    logger.info("UpdateInferenceState RPC 테스트")
    logger.info(f"서버: {server_address}")
    logger.info("=" * 70)

    # gRPC 채널 생성
    try:
        channel = grpc.aio.insecure_channel(server_address)
        stub = ai_vision_pb2_grpc.VisionServiceStub(channel)
        logger.info("✓ gRPC 채널 생성 완료\n")
    except Exception as e:
        logger.error(f"❌ gRPC 채널 생성 실패: {e}")
        logger.error("Vision 서버가 실행 중인지 확인하세요.")
        return False

    # 테스트 케이스들
    test_cases = [
        {
            "robot_id": "robot_001",
            "model_type": "SNACK",
            "is_active": True,
            "description": "간식 배달 시나리오 시작",
        },
        {
            "robot_id": "robot_001",
            "model_type": "EMPLOYEE",
            "is_active": True,
            "description": "직원 인식 시나리오 시작",
        },
        {
            "robot_id": "robot_002",
            "model_type": "OBSTACLE",
            "is_active": True,
            "description": "장애물 감지 시나리오 시작",
        },
        {
            "robot_id": "robot_001",
            "model_type": "SNACK",
            "is_active": False,
            "description": "간식 배달 시나리오 종료",
        },
    ]

    logger.info("테스트 케이스 실행\n")
    success_count = 0
    fail_count = 0

    for i, test_case in enumerate(test_cases, 1):
        logger.info(f"[테스트 {i}/{len(test_cases)}]")
        logger.info(f"설명: {test_case['description']}")
        logger.info(f"robot_id: {test_case['robot_id']}")
        logger.info(f"model_type: {test_case['model_type']}")
        logger.info(f"is_active: {test_case['is_active']}")
        logger.info("-" * 70)

        try:
            # gRPC 요청 생성
            request = ai_vision_pb2.InferenceStateRequest(
                robot_id=test_case["robot_id"],
                model_type=test_case["model_type"],
                is_active=test_case["is_active"],
            )

            # UpdateInferenceState RPC 호출
            response = await stub.UpdateInferenceState(request)

            # 결과 출력
            if response.success:
                logger.info(f"✅ 성공: {response.message}")
                success_count += 1
            else:
                logger.warning(f"⚠️  실패: {response.message}")
                fail_count += 1

        except grpc.RpcError as e:
            logger.error(f"❌ gRPC 오류: {e.code()} - {e.details()}")
            fail_count += 1
        except Exception as e:
            logger.error(f"❌ 오류 발생: {e}")
            fail_count += 1

        logger.info("")

    # 결과 요약
    logger.info("=" * 70)
    logger.info(f"테스트 완료: 성공 {success_count}, 실패 {fail_count}")
    logger.info("=" * 70)

    await channel.close()
    return fail_count == 0


async def main():
    """메인 함수"""
    try:
        success = await test_update_inference_state()
        sys.exit(0 if success else 1)
    except Exception as e:
        logger.error(f"테스트 실행 중 오류: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
