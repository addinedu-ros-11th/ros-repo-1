"""
AI Server Test Client
AI 서버의 gRPC 연결을 테스트하는 간단한 클라이언트
"""

import asyncio
import logging

from main_server.services.ai_inference.grpc_inference_client import AIInferenceService

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def test_ai_server():
    """
    AI 서버 연결 및 기능 테스트
    """
    logger.info("=" * 60)
    logger.info("AI Server 연결 테스트 시작")
    logger.info("=" * 60)

    try:
        # AI Inference 서비스 클라이언트 생성
        ai_service = AIInferenceService()

        # 1. 객체 인식 테스트
        logger.info("\n[테스트 1] 객체 인식 요청")
        logger.info("-" * 60)
        result = await ai_service.request_object_detection("test_image_001")
        logger.info(f"결과: {result}")

        # 2. 얼굴 인식 테스트
        logger.info("\n[테스트 2] 얼굴 인식 요청")
        logger.info("-" * 60)
        result = await ai_service.request_face_recognition("test_image_002")
        logger.info(f"결과: {result}")

        logger.info("\n" + "=" * 60)
        logger.info("모든 테스트 완료!")
        logger.info("=" * 60)

    except Exception as e:
        logger.error(f"테스트 중 오류 발생: {e}", exc_info=True)
        raise


def main():
    """
    메인 함수
    """
    asyncio.run(test_ai_server())


if __name__ == "__main__":
    main()
