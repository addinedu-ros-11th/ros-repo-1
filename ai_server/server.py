"""
AI Server - Main Entry Point
gRPC 서버를 구동하여 main_server와 통신
LLM 서비스와 Vision 서비스를 개별 포트에서 동시 실행
"""

import asyncio
import logging
from concurrent import futures

import grpc

from ai_server import config
from ai_server.services.llm_service import LLMService
from ai_server.services.vision_service import VisionService
from ai_server.grpc_impl.llm_servicer import LLMServicer
from ai_server.grpc_impl.vision_servicer import VisionServicer
from ai_server.grpc_impl import ai_llm_pb2_grpc
from ai_server.grpc_impl import ai_vision_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def serve():
    """
    gRPC 서버 시작
    LLM과 Vision 서비스를 각각 독립된 포트에서 실행
    """
    logger.info("=" * 60)
    logger.info("AI Server 시작 (LLM + Vision)")
    logger.info("=" * 60)

    # LLM 서비스 초기화
    logger.info("LLM 서비스 초기화 중...")
    llm_service = LLMService(
        model_name=config.LLM_MODEL_NAME, model_path=config.LLM_MODEL_PATH
    )
    llm_service.initialize()

    # Vision 서비스 초기화
    logger.info("Vision 서비스 초기화 중...")
    vision_service = VisionService(
        model_name=config.VISION_MODEL_NAME, model_path=config.VISION_MODEL_PATH
    )
    vision_service.initialize()

    # LLM gRPC 서버 생성
    llm_server = grpc.aio.server(
        futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS)
    )
    llm_servicer = LLMServicer(llm_service=llm_service)
    ai_llm_pb2_grpc.add_LLMServiceServicer_to_server(llm_servicer, llm_server)
    llm_address = f"{config.LLM_GRPC_HOST}:{config.LLM_GRPC_PORT}"
    llm_server.add_insecure_port(llm_address)

    # Vision gRPC 서버 생성
    vision_server = grpc.aio.server(
        futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS)
    )
    vision_servicer = VisionServicer(vision_service=vision_service)
    ai_vision_pb2_grpc.add_VisionServiceServicer_to_server(
        vision_servicer, vision_server
    )
    vision_address = f"{config.VISION_GRPC_HOST}:{config.VISION_GRPC_PORT}"
    vision_server.add_insecure_port(vision_address)

    # 서버 시작
    logger.info(f"LLM gRPC 서버 시작: {llm_address}")
    await llm_server.start()

    logger.info(f"Vision gRPC 서버 시작: {vision_address}")
    await vision_server.start()

    logger.info("=" * 60)
    logger.info("AI Server 준비 완료")
    logger.info(f"LLM 서비스: {llm_address}")
    logger.info(f"Vision 서비스: {vision_address}")
    logger.info(f"LLM 모델: {config.LLM_MODEL_NAME}")
    logger.info(f"Vision 모델: {config.VISION_MODEL_NAME}")
    logger.info("=" * 60)

    # 서버 종료 대기
    try:
        await asyncio.gather(
            llm_server.wait_for_termination(), vision_server.wait_for_termination()
        )
    except KeyboardInterrupt:
        logger.info("서버 종료 요청 수신")
        await asyncio.gather(llm_server.stop(grace=5), vision_server.stop(grace=5))
        logger.info("서버 종료 완료")


def main():
    """
    메인 함수
    """
    try:
        asyncio.run(serve())
    except Exception as e:
        logger.error(f"서버 실행 중 오류 발생: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    main()
