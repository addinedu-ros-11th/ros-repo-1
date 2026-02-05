"""
AI Server와 Main Server 통신 테스트
서버 자동 시작 및 종료 포함
"""

import asyncio
import logging
import sys
import os
import subprocess
import time
import signal

# 경로 추가
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService

logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Global variable for server process
server_process = None


def start_ai_server():
    """
    AI 서버를 백그라운드에서 시작
    """
    global server_process

    logger.info("=" * 70)
    logger.info(" AI 서버 시작 중...")
    logger.info("=" * 70)

    try:
        # AI 서버 프로세스 시작
        server_process = subprocess.Popen(
            [sys.executable, "-m", "ai_server.server"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            cwd=os.path.dirname(os.path.abspath(__file__)),
        )

        # 서버가 준비될 때까지 대기
        logger.info("서버 초기화 대기 중... (3초)")
        time.sleep(3)

        # 서버가 살아있는지 확인
        if server_process.poll() is not None:
            # 프로세스가 종료됨
            stdout, stderr = server_process.communicate()
            logger.error("서버 시작 실패!")
            logger.error(f"stdout: {stdout.decode()}")
            logger.error(f"stderr: {stderr.decode()}")
            return False

        logger.info("✓ AI 서버 시작 완료 (PID: {})".format(server_process.pid))
        return True

    except Exception as e:
        logger.error(f"서버 시작 중 오류: {e}", exc_info=True)
        return False


def stop_ai_server():
    """
    AI 서버 종료
    """
    global server_process

    if server_process is None:
        return

    logger.info("\n" + "=" * 70)
    logger.info(" AI 서버 종료 중...")
    logger.info("=" * 70)

    try:
        # SIGTERM 신호 전송
        server_process.terminate()

        # 최대 5초 대기
        try:
            server_process.wait(timeout=5)
            logger.info("✓ AI 서버 정상 종료")
        except subprocess.TimeoutExpired:
            # 강제 종료
            logger.warning("서버가 응답하지 않아 강제 종료합니다...")
            server_process.kill()
            server_process.wait()
            logger.info("✓ AI 서버 강제 종료 완료")

    except Exception as e:
        logger.error(f"서버 종료 중 오류: {e}", exc_info=True)


async def test_grpc_communication():
    """
    gRPC 통신 테스트
    """
    logger.info("=" * 70)
    logger.info(" AI Server <-> Main Server 통신 테스트")
    logger.info("=" * 70)

    try:
        # AI Inference 클라이언트 생성
        logger.info("\n[1] AI Inference 클라이언트 연결 중...")
        ai_client = AIInferenceService(host="localhost", port=50051)
        logger.info("✓ 클라이언트 생성 완료")

        # 객체 인식 테스트
        logger.info("\n[2] 객체 인식 요청 테스트")
        logger.info("-" * 70)
        result = await ai_client.request_object_detection("test_image_001")
        logger.info(f"✓ 결과: {result}")

        # 얼굴 인식 테스트
        logger.info("\n[3] 얼굴 인식 요청 테스트")
        logger.info("-" * 70)
        result = await ai_client.request_face_recognition("test_image_002")
        logger.info(f"✓ 결과: {result}")

        # 채널 종료
        await ai_client.close()

        logger.info("\n" + "=" * 70)
        logger.info(" ✓ 모든 테스트 성공!")
        logger.info("=" * 70)
        return True

    except Exception as e:
        logger.error(f"\n✗ 테스트 실패: {e}", exc_info=True)
        return False


async def main():
    """
    메인 함수
    """
    success = False

    try:
        # 1. AI 서버 시작
        if not start_ai_server():
            logger.error("서버 시작 실패로 테스트를 중단합니다.")
            sys.exit(1)

        # 2. 통신 테스트 실행
        success = await test_grpc_communication()

    except KeyboardInterrupt:
        logger.info("\n사용자에 의해 중단되었습니다.")
    except Exception as e:
        logger.error(f"예상치 못한 오류: {e}", exc_info=True)
    finally:
        # 3. AI 서버 종료
        stop_ai_server()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    asyncio.run(main())
