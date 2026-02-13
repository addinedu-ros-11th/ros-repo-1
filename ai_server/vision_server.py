"""
Vision Server
YOLO 객체 인식 + 얼굴 인식 gRPC 서버 + UDP 영상 수신
상태 기반 추론 엔진: 메인서버의 UpdateInferenceState에 따라 추론 ON/OFF
"""

import asyncio
import logging
from concurrent import futures
import grpc

from ai_server import config
from ai_server.services.vision_service import VisionService
from ai_server.services.inference_state_manager import InferenceStateManager
from ai_server.services.video_receiver import UDPVideoReceiver, VideoStreamProcessor
from ai_server.grpc_impl.vision_servicer import VisionServicer
from ai_server.grpc_impl import ai_vision_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)


async def serve():
    """
    Vision gRPC 서버 + UDP 영상 수신 + 상태 기반 추론 엔진 시작
    """
    logger.info("=" * 60)
    logger.info("Vision Server 시작")
    logger.info("=" * 60)

    # 1. 추론 상태 매니저 초기화
    state_manager = InferenceStateManager()

    # 2. Vision 서비스 초기화 (YOLO + face_recognition)
    logger.info("Vision 서비스 초기화 중...")
    vision_service = VisionService(
        product_model_path=config.PRODUCT_MODEL_PATH,
        obstacle_model_path=config.OBSTACLE_MODEL_PATH,
        face_match_threshold=config.FACE_MATCH_THRESHOLD,
        yolo_confidence=config.YOLO_CONFIDENCE_THRESHOLD,
    )
    vision_service.initialize()

    # 3. UDP 영상 수신기 초기화 및 시작
    logger.info("UDP Video Receiver 초기화 중...")
    video_receiver = UDPVideoReceiver(
        host=config.VIDEO_STREAM_HOST,
        port=config.VIDEO_STREAM_PORT,
        buffer_size=config.VIDEO_BUFFER_SIZE,
    )
    video_receiver.start()

    # 4. 상태 기반 비디오 프로세서 초기화 및 시작
    logger.info("VideoStreamProcessor 초기화 중...")
    video_processor = VideoStreamProcessor(
        receiver=video_receiver,
        vision_service=vision_service,
        state_manager=state_manager,
        inference_interval=config.INFERENCE_INTERVAL,
    )
    video_processor.start()

    # 5. gRPC 서버 생성
    server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS))

    # 6. Servicer 등록 (vision_service + state_manager + video_processor 연결)
    servicer = VisionServicer(
        vision_service=vision_service,
        state_manager=state_manager,
        video_processor=video_processor,
    )
    ai_vision_pb2_grpc.add_VisionServiceServicer_to_server(servicer, server)

    # 7. 서버 주소 설정 및 시작
    server_address = f"{config.VISION_GRPC_HOST}:{config.VISION_GRPC_PORT}"
    server.add_insecure_port(server_address)

    await server.start()

    logger.info("=" * 60)
    logger.info("Vision Server 준비 완료")
    logger.info(f"  gRPC 주소      : {server_address}")
    logger.info(
        f"  UDP 영상 수신  : {config.VIDEO_STREAM_HOST}:{config.VIDEO_STREAM_PORT}"
    )
    logger.info(f"  상품 모델      : {config.PRODUCT_MODEL_PATH}")
    logger.info(f"  장애물 모델    : {config.OBSTACLE_MODEL_PATH}")
    logger.info(f"  추론 간격      : {config.INFERENCE_INTERVAL}s")
    logger.info(f"  YOLO 신뢰도   : {config.YOLO_CONFIDENCE_THRESHOLD}")
    logger.info("=" * 60)

    # 서버 종료 대기
    try:
        await server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("서버 종료 요청 수신")
        video_processor.stop()
        video_receiver.stop()
        await server.stop(grace=5)
        logger.info("서버 종료 완료")


def main():
    try:
        asyncio.run(serve())
    except Exception as e:
        logger.error(f"서버 실행 중 오류: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    main()
