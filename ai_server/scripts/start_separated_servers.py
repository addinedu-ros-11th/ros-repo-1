#!/usr/bin/env python3
"""
AI Servers Launcher
LLM 서버와 Vision 서버를 동시에 실행

Note:
- Supports both module execution (python -m ai_server.start_separated_servers)
  and direct script execution (python ai_server/start_separated_servers.py).
"""

import asyncio
import logging
import os
import sys

# 프로젝트 루트를 sys.path에 추가
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from ai_server.llm_server import serve as serve_llm
from ai_server.vision_server import serve as serve_vision

# 로깅 설정
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def main():
    """
    LLM과 Vision 서버를 동시에 실행
    """
    logger.info("=" * 70)
    logger.info("AI Servers (LLM + Vision) 시작")
    logger.info("=" * 70)

    # 두 서버를 동시에 실행
    try:
        await asyncio.gather(
            serve_llm(),
            serve_vision(),
        )
    except KeyboardInterrupt:
        logger.info("\n서버 종료 요청 수신")
        logger.info("모든 서버를 종료합니다...")
    except Exception as e:
        logger.error(f"서버 실행 중 오류 발생: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("정상 종료")
        sys.exit(0)
    except Exception as e:
        logger.error(f"예상치 못한 오류: {e}")
        sys.exit(1)
