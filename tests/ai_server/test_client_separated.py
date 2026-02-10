"""
AI Server Test Client - 분리 아키텍처용
LLM과 Vision 서버의 gRPC 연결을 개별 테스트
"""

import asyncio
import logging

from main_server.services.ai_inference.llm_client import LLMClient
from main_server.services.ai_inference.vision_client import VisionClient

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


async def test_llm_server():
    """
    LLM 서버 테스트
    """
    logger.info("\n" + "=" * 60)
    logger.info("LLM Server 테스트 (Port 50051)")
    logger.info("=" * 60)

    try:
        llm = LLMClient()

        # 1. 텍스트 생성
        logger.info("\n[테스트 1] 텍스트 생성")
        logger.info("-" * 60)
        result = await llm.generate_text("안녕하세요, 오늘 날씨는?")
        logger.info(f"결과: {result}")

        # 2. 대화
        logger.info("\n[테스트 2] 대화")
        logger.info("-" * 60)
        messages = [{"role": "user", "content": "안녕하세요"}]
        result = await llm.chat(messages)
        logger.info(f"결과: {result}")

        # 3. 의도 분석
        logger.info("\n[테스트 3] 의도 분석")
        logger.info("-" * 60)
        result = await llm.analyze_intent("회의실 예약해줘")
        logger.info(f"결과: {result}")

        logger.info("\n✅ LLM Server 테스트 완료!")

    except Exception as e:
        logger.error(f"❌ LLM Server 테스트 실패: {e}", exc_info=True)
        raise


async def test_vision_server():
    """
    Vision 서버 테스트
    """
    logger.info("\n" + "=" * 60)
    logger.info("Vision Server 테스트 (Port 50052)")
    logger.info("=" * 60)

    try:
        vision = VisionClient()

        # 1. 객체 인식
        logger.info("\n[테스트 1] 객체 인식")
        logger.info("-" * 60)
        result = await vision.detect_objects("test_image_001")
        logger.info(f"결과: {result}")

        # 2. 얼굴 인식
        logger.info("\n[테스트 2] 얼굴 인식")
        logger.info("-" * 60)
        result = await vision.recognize_faces("test_image_002")
        logger.info(f"결과: {result}")

        # 3. 복수 객체 인식
        logger.info("\n[테스트 3] 복수 객체 인식")
        logger.info("-" * 60)
        results = await vision.detect_multiple_objects("test_image_003")
        logger.info(f"결과: {len(results)}개 객체 인식")
        for i, obj in enumerate(results):
            logger.info(f"  [{i+1}] {obj['object_name']}: {obj['confidence']:.2f}")

        logger.info("\n✅ Vision Server 테스트 완료!")

    except Exception as e:
        logger.error(f"❌ Vision Server 테스트 실패: {e}", exc_info=True)
        raise


async def test_both_servers():
    """
    LLM과 Vision 서버를 모두 테스트
    """
    logger.info("\n" + "=" * 70)
    logger.info("AI Servers 통합 테스트 시작")
    logger.info("LLM Server (Port 50051) + Vision Server (Port 50052)")
    logger.info("=" * 70)

    try:
        # LLM 테스트
        await test_llm_server()

        # Vision 테스트
        await test_vision_server()

        logger.info("\n" + "=" * 70)
        logger.info("✅ 모든 테스트 완료!")
        logger.info("=" * 70)

    except Exception as e:
        logger.error(f"\n❌ 테스트 중 오류 발생: {e}")
        raise


def main():
    """
    메인 함수
    """
    import sys

    if len(sys.argv) > 1:
        if sys.argv[1] == "llm":
            asyncio.run(test_llm_server())
        elif sys.argv[1] == "vision":
            asyncio.run(test_vision_server())
        else:
            print("Usage: python test_client_separated.py [llm|vision]")
            print("       python test_client_separated.py  (테스트 모두)")
    else:
        asyncio.run(test_both_servers())


if __name__ == "__main__":
    main()
