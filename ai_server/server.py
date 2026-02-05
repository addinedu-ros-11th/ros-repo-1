"""
AI Server - Main Entry Point
gRPC 서버를 구동하여 main_server와 통신
"""

import asyncio
import logging
from concurrent import futures

import grpc

from ai_server import config
from ai_server.services.llm_service import LLMService
from ai_server.services.vision_service import VisionService
from ai_server.grpc_impl.ai_inference_servicer import AIInferenceServicer
from ai_server.grpc_impl import ai_inference_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def serve():
    """
    gRPC 서버 시작
    """
    logger.info("=" * 60)
    logger.info("AI Inference Server 시작")
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

    # gRPC 서버 생성
    server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS))

    # Servicer 등록
    servicer = AIInferenceServicer(
        llm_service=llm_service, vision_service=vision_service
    )
    ai_inference_pb2_grpc.add_AIInferenceServicer_to_server(servicer, server)

    # 서버 주소 설정
    server_address = f"{config.GRPC_SERVER_HOST}:{config.GRPC_SERVER_PORT}"
    server.add_insecure_port(server_address)

    # 서버 시작
    logger.info(f"gRPC 서버 시작: {server_address}")
    await server.start()

    logger.info("=" * 60)
    logger.info("AI Inference Server 준비 완료")
    logger.info(f"주소: {server_address}")
    logger.info(f"LLM 모델: {config.LLM_MODEL_NAME}")
    logger.info(f"Vision 모델: {config.VISION_MODEL_NAME}")
    logger.info("=" * 60)

    # 서버 종료 대기
    try:
        await server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("서버 종료 요청 수신")
        await server.stop(grace=5)
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
