"""
AI Server - Main Entry Point
LLM 서비스와 Vision 서비스를 개별 포트에서 동시 실행
Vision은 상태 기반 추론 엔진 포함 (UDP 영상 수신 + YOLO + face_recognition)
"""

import asyncio
import logging
from concurrent import futures

import grpc

from ai_server import config
from ai_server.services.llm_service import LLMService
from ai_server.services.vision_service import VisionService
from ai_server.services.inference_state_manager import InferenceStateManager
from ai_server.services.video_receiver import UDPVideoReceiver, VideoStreamProcessor
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
    LLM + Vision 통합 서버 시작
    """
    logger.info("=" * 60)
    logger.info("AI Server 시작 (LLM + Vision)")
    logger.info("=" * 60)

    # ============================
    # LLM 서비스 초기화
    # ============================
    logger.info("LLM 서비스 초기화 중...")
    llm_service = LLMService(
        model_name=config.LLM_MODEL_NAME, model_path=config.LLM_MODEL_PATH
    )
    llm_service.initialize()

    # ============================
    # Vision 서비스 + 상태 기반 추론
    # ============================
    logger.info("Vision 서비스 초기화 중...")
    state_manager = InferenceStateManager()

    vision_service = VisionService(
        product_model_path=config.PRODUCT_MODEL_PATH,
        obstacle_model_path=config.OBSTACLE_MODEL_PATH,
        face_match_threshold=config.FACE_MATCH_THRESHOLD,
        yolo_confidence=config.YOLO_CONFIDENCE_THRESHOLD,
    )
    vision_service.initialize()

    # UDP 영상 수신 + 상태 기반 처리
    video_receiver = UDPVideoReceiver(
        host=config.VIDEO_STREAM_HOST,
        port=config.VIDEO_STREAM_PORT,
        buffer_size=config.VIDEO_BUFFER_SIZE,
    )
    video_receiver.start()

    video_processor = VideoStreamProcessor(
        receiver=video_receiver,
        vision_service=vision_service,
        state_manager=state_manager,
        inference_interval=config.INFERENCE_INTERVAL,
    )
    video_processor.start()

    # ============================
    # LLM gRPC 서버
    # ============================
    llm_server = grpc.aio.server(
        futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS)
    )
    llm_servicer = LLMServicer(llm_service=llm_service)
    ai_llm_pb2_grpc.add_LLMServiceServicer_to_server(llm_servicer, llm_server)
    llm_address = f"{config.LLM_GRPC_HOST}:{config.LLM_GRPC_PORT}"
    llm_server.add_insecure_port(llm_address)

    # ============================
    # Vision gRPC 서버
    # ============================
    vision_server = grpc.aio.server(
        futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS)
    )
    vision_servicer = VisionServicer(
        vision_service=vision_service,
        state_manager=state_manager,
        video_processor=video_processor,
    )
    ai_vision_pb2_grpc.add_VisionServiceServicer_to_server(
        vision_servicer, vision_server
    )
    vision_address = f"{config.VISION_GRPC_HOST}:{config.VISION_GRPC_PORT}"
    vision_server.add_insecure_port(vision_address)

    # ============================
    # 서버 시작
    # ============================
    await llm_server.start()
    await vision_server.start()

    logger.info("=" * 60)
    logger.info("AI Server 준비 완료")
    logger.info(f"  LLM gRPC       : {llm_address}")
    logger.info(f"  Vision gRPC    : {vision_address}")
    logger.info(
        f"  UDP 영상 수신  : {config.VIDEO_STREAM_HOST}:{config.VIDEO_STREAM_PORT}"
    )
    logger.info(f"  LLM 모델       : {config.LLM_MODEL_NAME}")
    logger.info(f"  상품 모델      : {config.PRODUCT_MODEL_PATH}")
    logger.info(f"  장애물 모델    : {config.OBSTACLE_MODEL_PATH}")
    logger.info("=" * 60)

    # 서버 종료 대기
    try:
        await asyncio.gather(
            llm_server.wait_for_termination(),
            vision_server.wait_for_termination(),
        )
    except KeyboardInterrupt:
        logger.info("서버 종료 요청 수신")
        video_processor.stop()
        video_receiver.stop()
        await asyncio.gather(llm_server.stop(grace=5), vision_server.stop(grace=5))
        logger.info("서버 종료 완료")


def main():
    try:
        asyncio.run(serve())
    except Exception as e:
        logger.error(f"서버 실행 중 오류: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    main()
