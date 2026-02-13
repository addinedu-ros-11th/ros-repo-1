"""
Vision gRPC Servicer Implementation
객체 인식, 얼굴 인식, 실시간 스트리밍, 추론 상태 제어
"""

import logging
import time
import asyncio
from typing import Optional
import grpc

from ai_server.grpc_impl import ai_vision_pb2
from ai_server.grpc_impl import ai_vision_pb2_grpc
from ai_server.services.vision_service import VisionService
from ai_server.services.inference_state_manager import InferenceStateManager

logger = logging.getLogger(__name__)


class VisionServicer(ai_vision_pb2_grpc.VisionServiceServicer):
    """
    Vision gRPC Servicer.
    - DetectObjects / RecognizeFaces / DetectMultipleObjects : 단건 요청
    - StreamVisionResults : 실시간 추론 결과 스트림 (VideoStreamProcessor 결과 큐 소비)
    - UpdateInferenceState : 로봇별 추론 활성/비활성 제어
    """

    def __init__(
        self,
        vision_service: Optional[VisionService] = None,
        state_manager: Optional[InferenceStateManager] = None,
        video_processor=None,
    ):
        """
        Args:
            vision_service: VisionService 인스턴스 (단건 요청용)
            state_manager: InferenceStateManager 인스턴스 (상태 제어)
            video_processor: VideoStreamProcessor 인스턴스 (스트림 결과 큐)
        """
        self.vision_service = vision_service
        self.state_manager = state_manager
        self.video_processor = video_processor
        logger.info("VisionServicer 초기화 완료")

    # ========================================
    # 단건 요청 RPC
    # ========================================

    async def DetectObjects(self, request, context):
        """객체 인식 (단건 요청)"""
        logger.info(f"객체 인식 요청: image_id={request.image_id}")
        try:
            image_data = None
            if request.HasField("image_data"):
                image_data = request.image_data
            result = self.vision_service.detect_objects(request.image_id, image_data)

            return ai_vision_pb2.ObjectDetectionResponse(
                object_name=result["object_name"],
                confidence=result["confidence"],
                box=ai_vision_pb2.BoundingBox(
                    x=result["box"]["x"],
                    y=result["box"]["y"],
                    width=result["box"]["width"],
                    height=result["box"]["height"],
                ),
            )
        except Exception as e:
            logger.error(f"객체 인식 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ai_vision_pb2.ObjectDetectionResponse()

    async def RecognizeFaces(self, request, context):
        """얼굴 인식 (단건 요청)"""
        logger.info(f"얼굴 인식 요청: image_id={request.image_id}")
        try:
            image_data = None
            if request.HasField("image_data"):
                image_data = request.image_data
            result = self.vision_service.recognize_face(request.image_id, image_data)

            response = ai_vision_pb2.FaceRecognitionResponse(
                person_type=result["person_type"],
                confidence=result["confidence"],
            )
            if "employee_id" in result:
                response.employee_id = result["employee_id"]

            return response
        except Exception as e:
            logger.error(f"얼굴 인식 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ai_vision_pb2.FaceRecognitionResponse()

    async def DetectMultipleObjects(self, request, context):
        """복수 객체 인식 (단건 요청)"""
        logger.info(f"복수 객체 인식 요청: image_id={request.image_id}")
        try:
            image_data = None
            if request.HasField("image_data"):
                image_data = request.image_data
            results = self.vision_service.detect_multiple_objects(
                request.image_id, image_data
            )

            objects = []
            for r in results:
                objects.append(
                    ai_vision_pb2.ObjectDetectionResponse(
                        object_name=r["object_name"],
                        confidence=r["confidence"],
                        box=ai_vision_pb2.BoundingBox(
                            x=r["box"]["x"],
                            y=r["box"]["y"],
                            width=r["box"]["width"],
                            height=r["box"]["height"],
                        ),
                    )
                )

            return ai_vision_pb2.MultiObjectDetectionResponse(objects=objects)
        except Exception as e:
            logger.error(f"복수 객체 인식 오류: {e}")
            context.set_code(grpc.StatusCode.INTERNAL)
            context.set_details(str(e))
            return ai_vision_pb2.MultiObjectDetectionResponse(objects=[])

    # ========================================
    # 실시간 스트리밍 RPC
    # ========================================

    async def StreamVisionResults(self, request, context):
        """
        실시간 비전 추론 결과 스트리밍.
        VideoStreamProcessor의 results_queue에서 결과를 소비하여 gRPC 스트림으로 전송.
        메인서버가 구독하면 추론 결과가 계속 전달됨.
        """
        logger.info("비전 결과 스트리밍 시작 (클라이언트 연결)")

        if self.video_processor is None:
            logger.error("VideoStreamProcessor가 설정되지 않음")
            context.set_code(grpc.StatusCode.UNAVAILABLE)
            context.set_details("Video processor not available")
            return

        try:
            while context.is_active():
                # 비동기 루프에서 blocking queue.get()을 실행
                loop = asyncio.get_event_loop()
                result_data = await loop.run_in_executor(
                    None, self.video_processor.get_result, 0.5
                )

                if result_data is None:
                    continue

                # 결과 타입에 따라 VisionResult 메시지 생성
                vision_result = self._build_vision_result(result_data)
                if vision_result:
                    yield vision_result

        except asyncio.CancelledError:
            logger.info("스트리밍 취소됨 (클라이언트 연결 해제)")
        except Exception as e:
            logger.error(f"스트리밍 오류: {e}")

    def _build_vision_result(self, data: dict) -> Optional[ai_vision_pb2.VisionResult]:
        """
        내부 결과 딕셔너리 → gRPC VisionResult 메시지 변환.

        Args:
            data: VideoStreamProcessor가 생성한 결과
                  {"robot_id", "timestamp", "type", "content"}
        """
        robot_id = data.get("robot_id", "")
        timestamp = data.get("timestamp", int(time.time() * 1000))
        result_type = data.get("type")
        content = data.get("content")

        if not content:
            return None

        try:
            if result_type == "face_recognition":
                face_resp = ai_vision_pb2.FaceRecognitionResponse(
                    person_type=content.get("person_type", "Unknown"),
                    confidence=content.get("confidence", 0.0),
                )
                if "employee_id" in content:
                    face_resp.employee_id = content["employee_id"]

                return ai_vision_pb2.VisionResult(
                    robot_id=robot_id,
                    timestamp=timestamp,
                    face_recognition=face_resp,
                )

            elif result_type == "object_detection":
                obj_resp = ai_vision_pb2.ObjectDetectionResponse(
                    object_name=content.get("object_name", ""),
                    confidence=content.get("confidence", 0.0),
                    box=ai_vision_pb2.BoundingBox(
                        x=content.get("box", {}).get("x", 0),
                        y=content.get("box", {}).get("y", 0),
                        width=content.get("box", {}).get("width", 0),
                        height=content.get("box", {}).get("height", 0),
                    ),
                )
                return ai_vision_pb2.VisionResult(
                    robot_id=robot_id,
                    timestamp=timestamp,
                    object_detection=obj_resp,
                )

            elif result_type == "multi_objects":
                objects = []
                for obj in content:
                    objects.append(
                        ai_vision_pb2.ObjectDetectionResponse(
                            object_name=obj.get("object_name", ""),
                            confidence=obj.get("confidence", 0.0),
                            box=ai_vision_pb2.BoundingBox(
                                x=obj.get("box", {}).get("x", 0),
                                y=obj.get("box", {}).get("y", 0),
                                width=obj.get("box", {}).get("width", 0),
                                height=obj.get("box", {}).get("height", 0),
                            ),
                        )
                    )
                multi_resp = ai_vision_pb2.MultiObjectDetectionResponse(objects=objects)
                return ai_vision_pb2.VisionResult(
                    robot_id=robot_id,
                    timestamp=timestamp,
                    multi_objects=multi_resp,
                )

        except Exception as e:
            logger.error(f"VisionResult 생성 오류: {e}")

        return None

    # ========================================
    # 추론 상태 제어 RPC
    # ========================================

    async def UpdateInferenceState(self, request, context):
        """
        추론 상태 업데이트.
        메인서버가 호출하여 특정 로봇의 특정 모델 추론을 시작/중지.
        """
        robot_id = request.robot_id
        model_type = request.model_type
        is_active = request.is_active

        logger.info(
            f"추론 상태 업데이트: robot={robot_id}, "
            f"model={model_type}, active={is_active}"
        )

        try:
            if self.state_manager is None:
                raise RuntimeError("InferenceStateManager가 설정되지 않음")

            success = self.state_manager.update_state(robot_id, model_type, is_active)

            action = "활성화" if is_active else "비활성화"
            if success:
                msg = f"{robot_id} - {model_type} 추론 {action} 완료"
                # 현재 상태 로그
                all_active = self.state_manager.get_active_models(robot_id)
                logger.info(f"  현재 활성 모델: {all_active or '없음'}")
            else:
                msg = f"알 수 없는 모델 타입: {model_type}"

            return ai_vision_pb2.InferenceStateResponse(success=success, message=msg)

        except Exception as e:
            logger.error(f"추론 상태 업데이트 오류: {e}", exc_info=True)
            return ai_vision_pb2.InferenceStateResponse(
                success=False, message=f"Error: {str(e)}"
            )
