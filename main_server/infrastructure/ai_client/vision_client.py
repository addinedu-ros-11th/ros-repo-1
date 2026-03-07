import logging
import asyncio
import grpc
from typing import Dict, Any, Optional
from main_server.config import config

# Generated gRPC files
from main_server.infrastructure.grpc import ai_vision_pb2
from main_server.infrastructure.grpc import ai_vision_pb2_grpc
from main_server.domains.ai.interfaces import IVisionService

logger = logging.getLogger(__name__)


class VisionServiceClient(IVisionService):
    """
    gRPC를 통해 원격 Vision 서버(YOLOv8n)와 통신하는 클라이언트 서비스.
    """

    def __init__(
        self, host: str = config.VISION_GRPC_HOST, port: int = config.VISION_GRPC_PORT
    ):
        self.channel = grpc.aio.insecure_channel(f"{host}:{port}")
        self.stub = ai_vision_pb2_grpc.VisionServiceStub(self.channel)
        logger.info(f"Vision gRPC Client 초기화 완료 (Connecting to {host}:{port}).")
        # 연결 상태 모니터링 태스크 시작
        self._monitor_task = asyncio.create_task(self._monitor_connectivity())

    async def _monitor_connectivity(self):
        """gRPC 채널의 연결 상태를 모니터링하고 로그를 남깁니다."""
        last_state = None
        while True:
            state = self.channel.get_state(try_to_connect=True)
            if state != last_state:
                if state == grpc.ChannelConnectivity.READY:
                    logger.info(f"Vision gRPC 서버에 연결되었습니다. (State: {state})")
                elif state == grpc.ChannelConnectivity.TRANSIENT_FAILURE:
                    logger.warning(
                        f"Vision gRPC 서버 연결 실패 - 재시도 중... (State: {state})"
                    )
                elif state == grpc.ChannelConnectivity.IDLE:
                    logger.info(
                        f"Vision gRPC 서버 연결이 유휴 상태입니다. (State: {state})"
                    )
                elif state == grpc.ChannelConnectivity.CONNECTING:
                    logger.info(f"Vision gRPC 서버에 연결 시도 중... (State: {state})")

                last_state = state

            # 상태가 변경될 때까지 대기
            try:
                await self.channel.wait_for_state_change(last_state)
            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"Vision gRPC 상태 모니터링 오류: {e}")
                await asyncio.sleep(5)

    async def request_object_detection(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        주어진 이미지 ID 또는 데이터로 객체 인식을 요청합니다.
        """
        request = ai_vision_pb2.ImageRequest(image_id=image_id)
        response = await self.stub.DetectObjects(request)

        return {
            "object_name": response.object_name,
            "confidence": response.confidence,
            "box": {
                "x": response.box.x,
                "y": response.box.y,
                "width": response.box.width,
                "height": response.box.height,
            },
        }

    async def request_face_recognition(
        self, image_id: str, image_data: Optional[bytes] = None
    ) -> Dict[str, Any]:
        """
        주어진 이미지 ID 또는 데이터로 얼굴 인식을 요청합니다.
        """
        request = ai_vision_pb2.ImageRequest(image_id=image_id)
        response = await self.stub.RecognizeFaces(request)

        result = {
            "person_type": response.person_type,
            "confidence": response.confidence,
        }
        if response.HasField("employee_id"):
            result["employee_id"] = response.employee_id

        return result

    async def update_inference_state(
        self, robot_id: str, model_type: str, is_active: bool
    ) -> Dict[str, Any]:
        """
        AI 서버에게 특정 로봇에 대한 추론 시작/중지를 명령합니다.
        """
        try:
            # 컴파일된 ai_vision_pb2의 실제 메시지 클래스 사용
            request = ai_vision_pb2.InferenceStateRequest(
                robot_id=robot_id, model_type=model_type, is_active=is_active
            )
            response = await self.stub.UpdateInferenceState(request)
            return {"success": response.success, "message": response.message}
        except Exception as e:
            logger.error(f"Error in UpdateInferenceState: {e}")
            return {"success": False, "message": str(e)}

    async def start_vision_stream(self, callback: Any):
        """
        비전 추론 결과 스트림을 구독합니다.
        robot_id 별로 구분된 결과가 전달됩니다.
        """
        logger.info("Vision 스트림 구독 시작 (StreamVisionResults)...")
        try:
            async for result in self.stub.StreamVisionResults(ai_vision_pb2.Empty()):
                data = {
                    "robot_id": result.robot_id,
                }

                if result.HasField("object_detection"):
                    det = result.object_detection
                    data["type"] = "object_detection"
                    data["content"] = {
                        "object_name": det.object_name,
                        "confidence": det.confidence,
                        "box": {
                            "x": det.box.x,
                            "y": det.box.y,
                            "width": det.box.width,
                            "height": det.box.height,
                        },
                    }
                elif result.HasField("face_recognition"):
                    face = result.face_recognition
                    data["type"] = "face_recognition"
                    data["content"] = {
                        "person_type": face.person_type,
                        "confidence": face.confidence,
                    }
                    if face.HasField("employee_id"):
                        data["content"]["employee_id"] = face.employee_id
                elif result.HasField("multi_objects"):
                    data["type"] = "multi_objects"
                    objects = []
                    for obj in result.multi_objects.objects:
                        objects.append(
                            {
                                "object_name": obj.object_name,
                                "confidence": obj.confidence,
                                "box": {
                                    "x": obj.box.x,
                                    "y": obj.box.y,
                                    "width": obj.box.width,
                                    "height": obj.box.height,
                                },
                            }
                        )
                    data["content"] = objects
                else:
                    continue  # 알 수 없는 결과 타입은 무시

                await callback(data)
        except grpc.aio.AioRpcError as e:
            logger.error(f"Vision 스트림 연결 오류: {e}")

    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
