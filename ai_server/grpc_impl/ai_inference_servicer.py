"""
AI Inference gRPC Servicer Implementation
main_server의 요청을 받아 LLM 및 Vision 서비스를 실행
"""

import logging
from typing import Optional
import asyncio
import grpc

from ai_server.grpc_impl import ai_inference_pb2
from ai_server.grpc_impl import ai_inference_pb2_grpc
from ai_server.services.llm_service import LLMService
from ai_server.services.vision_service import VisionService

logger = logging.getLogger(__name__)


class AIInferenceServicer(ai_inference_pb2_grpc.AIInferenceServicer):
    """
    gRPC AIInference 서비스 구현
    """

    def __init__(
        self,
        llm_service: Optional[LLMService] = None,
        vision_service: Optional[VisionService] = None,
    ):
        """
        Servicer 초기화

        Args:
            llm_service: LLM 서비스 인스턴스
            vision_service: Vision 서비스 인스턴스
        """
        self.llm_service = llm_service
        self.vision_service = vision_service
        logger.info("AIInferenceServicer 초기화 완료")

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
            response = ai_inference_pb2.ObjectDetectionResponse(
                object_name=result["object_name"],
                confidence=result["confidence"],
                box=ai_inference_pb2.BoundingBox(
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
            return ai_inference_pb2.ObjectDetectionResponse()

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
            result = self.vision_service.recognize_face(request.image_id)

            # 응답 메시지 생성
            response = ai_inference_pb2.FaceRecognitionResponse(
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
            return ai_inference_pb2.FaceRecognitionResponse()

    async def StreamInferenceResults(self, request, context):
        """
        실시간 추론 결과 스트리밍

        Args:
            request: Empty 메시지
            context: gRPC context

        Yields:
            InferenceResult 스트림
        """
        logger.info("추론 결과 스트리밍 시작")

        try:
            # TODO: 실제 스트리밍 로직 구현
            # 예시: 큐에서 추론 결과를 받아서 스트리밍

            # 스텁 구현: 5초마다 더미 결과 전송
            for i in range(5):
                await asyncio.sleep(5)

                # 더미 객체 인식 결과 생성
                object_detection = ai_inference_pb2.ObjectDetectionResponse(
                    object_name=f"object_{i}",
                    confidence=0.9 - (i * 0.1),
                    box=ai_inference_pb2.BoundingBox(x=10, y=20, width=100, height=150),
                )

                result = ai_inference_pb2.InferenceResult(
                    robot_id=f"robot_{i % 2}", object_detection=object_detection
                )

                logger.info(f"스트리밍 결과 전송: robot_id=robot_{i % 2}")
                yield result

        except Exception as e:
            logger.error(f"스트리밍 처리 중 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(f"스트리밍 실패: {str(e)}")
