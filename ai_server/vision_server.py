"""
Vision Server - YOLOv8n
컴퓨터 비전 전용 gRPC 서버 + UDP 영상 수신
"""

import asyncio
import logging
from concurrent import futures
import grpc

from ai_server import config
from ai_server.services.vision_service import VisionService
from ai_server.services.video_receiver import UDPVideoReceiver, VideoStreamProcessor
from ai_server.grpc_impl.vision_servicer import VisionServicer
from ai_server.grpc_impl import ai_vision_pb2_grpc

# 로깅 설정
logging.basicConfig(
    level=getattr(logging, config.LOG_LEVEL),
    format="%(asctime)s - %(name)s - %(levelname)s - %(message)s",
)
logger = logging.getLogger(__name__)

# 전역 변수로 처리 결과 저장 (Main Server로 전송할 데이터)
detection_results_queue = []


def on_detection_result(result):
    """
    객체 인식 결과 콜백
    Main Server로 전송하기 위해 큐에 저장

    Args:
        result: 객체 인식 결과
    """
    global detection_results_queue

    # 최대 100개까지만 저장 (메모리 관리)
    if len(detection_results_queue) >= 100:
        detection_results_queue.pop(0)

    detection_results_queue.append(result)
    logger.info(
        f"객체 인식 완료: robot={result.get('robot_id')}, object={result.get('object_name')}"
    )


async def serve():
    """
    Vision gRPC 서버 + UDP 영상 수신 시작
    """
    logger.info("=" * 60)
    logger.info("Vision Server (YOLOv8n) 시작")
    logger.info("=" * 60)

    # Vision 서비스 초기화
    logger.info("Vision 서비스 초기화 중...")
    vision_service = VisionService(
        model_name=config.VISION_MODEL_NAME, model_path=config.VISION_MODEL_PATH
    )
    vision_service.initialize()

    # UDP 영상 수신기 초기화 및 시작
    logger.info("UDP Video Receiver 초기화 중...")
    video_receiver = UDPVideoReceiver(
        host=config.VIDEO_STREAM_HOST,
        port=config.VIDEO_STREAM_PORT,
        buffer_size=config.VIDEO_BUFFER_SIZE,
    )
    video_receiver.start()

    # 비디오 스트림 프로세서 초기화 및 시작
    logger.info("Video Stream Processor 초기화 중...")
    video_processor = VideoStreamProcessor(
        receiver=video_receiver,
        vision_service=vision_service,
        callback=on_detection_result,
    )
    video_processor.start()

    # gRPC 서버 생성
    server = grpc.aio.server(futures.ThreadPoolExecutor(max_workers=config.MAX_WORKERS))

    # Servicer 등록
    servicer = VisionServicer(vision_service=vision_service)
    ai_vision_pb2_grpc.add_VisionServiceServicer_to_server(servicer, server)

    # 서버 주소 설정
    server_address = f"{config.VISION_GRPC_HOST}:{config.VISION_GRPC_PORT}"
    server.add_insecure_port(server_address)

    # 서버 시작
    logger.info(f"gRPC 서버 시작: {server_address}")
    await server.start()

    logger.info("=" * 60)
    logger.info("Vision Server 준비 완료")
    logger.info(f"gRPC 주소: {server_address}")
    logger.info(f"UDP 영상 수신: {config.VIDEO_STREAM_HOST}:{config.VIDEO_STREAM_PORT}")
    logger.info(f"모델: {config.VISION_MODEL_NAME}")
    logger.info("=" * 60)

    # 서버 종료 대기
    try:
        await server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("서버 종료 요청 수신")

        # UDP 수신 및 프로세서 중지
        video_processor.stop()
        video_receiver.stop()

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
