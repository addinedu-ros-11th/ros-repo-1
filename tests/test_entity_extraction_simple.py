#!/usr/bin/env python3
"""
간단한 구조화 응답 테스트
LLM 서비스만 직접 테스트 (gRPC 서버 없이)
"""

import sys
import logging
from pathlib import Path

# 프로젝트 루트를 Python 경로에 추가
sys.path.insert(0, str(Path(__file__).parent.parent))

from ai_server.services.llm_service import LLMService

# 로깅 설정
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)

logger = logging.getLogger(__name__)


def main():
    """메인 테스트 함수"""

    # LLM 서비스 초기화
    logger.info("=" * 60)
    logger.info("LLM 구조화 응답 테스트 시작")
    logger.info("=" * 60)

    llm_service = LLMService(model_name="qwen3:4b-instruct-2507-q4_K_M")

    try:
        llm_service.initialize()
        logger.info("LLM 서비스 초기화 완료\n")
    except Exception as e:
        logger.error(f"초기화 실패: {e}")
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

    logger.info("\n" + "=" * 60)
    logger.info("테스트 케이스 실행")
    logger.info("=" * 60 + "\n")

    for i, test_input in enumerate(test_cases, 1):
        logger.info(f"\n[테스트 {i}]")
        logger.info(f"입력: {test_input}")
        logger.info("-" * 40)

        try:
            result = llm_service.parse_natural_language(test_input)

            logger.info(f"🎯 TaskType: {result.get('task_type', 'UNKNOWN')}")
            logger.info(f"✓ 신뢰도: {result.get('confidence', 0.0):.2f}")

            fields = result.get("fields", {})
            if fields:
                logger.info(f"📋 Structured: {fields}")
            else:
                logger.info("📋 Structured: 없음")

            if result.get("error"):
                logger.warning(f"⚠ 오류: {result['error']}")

            if result.get("raw_text"):
                logger.debug(f"Raw: {result['raw_text']}")

        except Exception as e:
            logger.error(f"❌ 처리 실패: {e}")

    logger.info("\n" + "=" * 60)
    logger.info("테스트 완료")
    logger.info("=" * 60)


if __name__ == "__main__":
    main()
