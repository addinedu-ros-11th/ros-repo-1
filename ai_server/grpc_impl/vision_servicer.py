"""
Vision gRPC Servicer Implementation
객체 인식, 얼굴 인식 기능 제공
"""

import logging
from typing import Optional
import asyncio
import grpc

from ai_server.grpc_impl import ai_vision_pb2
from ai_server.grpc_impl import ai_vision_pb2_grpc
from ai_server.services.vision_service import VisionService

logger = logging.getLogger(__name__)


class VisionServicer(ai_vision_pb2_grpc.VisionServiceServicer):
    """
    Vision 전용 gRPC Servicer
    ai_vision.proto 기반 VisionServiceServicer 구현
    """

    def __init__(self, vision_service: Optional[VisionService] = None):
        """
        Vision Servicer 초기화

        Args:
            vision_service: Vision 서비스 인스턴스
        """
        self.vision_service = vision_service
        logger.info("Vision Servicer 초기화 완료")

    async def DetectObjects(self, request, context):
        """
        객체 인식 요청 처리

        Args:
            request: ImageRequest 메시지
            context: gRPC context

        Returns:
            ObjectDetectionResponse
        """
        logger.info(f"객체 인식 요청 수신: image_id={request.image_id}")

        try:
            # Vision 서비스를 통해 객체 인식 수행
            result = self.vision_service.detect_objects(request.image_id)

            # 응답 메시지 생성
            response = ai_vision_pb2.ObjectDetectionResponse(
                object_name=result["object_name"],
                confidence=result["confidence"],
                box=ai_vision_pb2.BoundingBox(
                    x=result["box"]["x"],
                    y=result["box"]["y"],
                    width=result["box"]["width"],
                    height=result["box"]["height"],
                ),
            )

            logger.info(
                f"객체 인식 완료: {result['object_name']} (confidence: {result['confidence']})"
            )
            return response

        except Exception as e:
            logger.error(f"객체 인식 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"객체 인식 실패: {str(e)}")
            return ai_vision_pb2.ObjectDetectionResponse()

    async def RecognizeFaces(self, request, context):
        """
        얼굴 인식 요청 처리

        Args:
            request: ImageRequest 메시지
            context: gRPC context

        Returns:
            FaceRecognitionResponse
        """
        logger.info(f"얼굴 인식 요청 수신: image_id={request.image_id}")

        try:
            # Vision 서비스를 통해 얼굴 인식 수행
            image_data = None
            if request.HasField("image_data"):
                image_data = request.image_data
            result = self.vision_service.recognize_face(request.image_id, image_data)

            # 응답 메시지 생성
            response = ai_vision_pb2.FaceRecognitionResponse(
                person_type=result["person_type"], confidence=result["confidence"]
            )

            # employee_id가 있으면 추가
            if "employee_id" in result:
                response.employee_id = result["employee_id"]

            logger.info(
                f"얼굴 인식 완료: {result['person_type']} (confidence: {result['confidence']})"
            )
            return response

        except Exception as e:
            logger.error(f"얼굴 인식 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"얼굴 인식 실패: {str(e)}")
            return ai_vision_pb2.FaceRecognitionResponse()

    async def DetectMultipleObjects(self, request, context):
        """
        복수 객체 인식 요청 처리

        Args:
            request: ImageRequest 메시지
            context: gRPC context

        Returns:
            MultiObjectDetectionResponse
        """
        logger.info(f"복수 객체 인식 요청 수신: image_id={request.image_id}")

        try:
            # Vision 서비스를 통해 복수 객체 인식 수행
            results = self.vision_service.detect_multiple_objects(request.image_id)

            logger.info(f"복수 객체 인식 완료: {len(results)}개 객체")

            # MultiObjectDetectionResponse 생성
            objects = []
            for result in results:
                obj = ai_vision_pb2.ObjectDetectionResponse(
                    object_name=result["object_name"],
                    confidence=result["confidence"],
                    box=ai_vision_pb2.BoundingBox(
                        x=result["box"]["x"],
                        y=result["box"]["y"],
                        width=result["box"]["width"],
                        height=result["box"]["height"],
                    ),
                )
                objects.append(obj)

            return ai_vision_pb2.MultiObjectDetectionResponse(objects=objects)

        except Exception as e:
            logger.error(f"복수 객체 인식 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"복수 객체 인식 실패: {str(e)}")
            return ai_vision_pb2.MultiObjectDetectionResponse(objects=[])

    async def StreamVisionResults(self, request, context):
        """
        실시간 비전 결과 스트리밍

        Args:
            request: Empty 메시지
            context: gRPC context

        Yields:
            VisionResult 스트림
        """
        logger.info("비전 결과 스트리밍 시작")

        try:
            # TODO: 실제 스트리밍 로직 구현
            # 스텁 구현: 5초마다 더미 결과 전송
            for i in range(5):
                await asyncio.sleep(5)

                # 더미 객체 인식 결과 생성
                object_detection = ai_vision_pb2.ObjectDetectionResponse(
                    object_name=f"object_{i}",
                    confidence=0.9 - (i * 0.1),
                    box=ai_vision_pb2.BoundingBox(x=10, y=20, width=100, height=150),
                )

                # VisionResult 메시지 생성
                import time

                result = ai_vision_pb2.VisionResult(
                    robot_id=f"robot_{i % 2}",
                    timestamp=int(time.time()),
                    object_detection=object_detection,
                )

                logger.info(f"스트리밍 결과 전송: robot_id=robot_{i % 2}")
                yield result

        except Exception as e:
            logger.error(f"스트리밍 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"스트리밍 실패: {str(e)}")
