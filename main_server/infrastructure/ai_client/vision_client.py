import grpc
from typing import Dict, Any, Optional
from main_server.config import config

# Generated gRPC files
from main_server.infrastructure.grpc import ai_inference_pb2
from main_server.infrastructure.grpc import ai_inference_pb2_grpc
from main_server.services.ai_inference.protocols import IVisionService

class VisionServiceClient(IVisionService):
    """
    gRPC를 통해 원격 Vision 서버(YOLOv8n)와 통신하는 클라이언트 서비스.
    """
    def __init__(self, host: str = config.VISION_GRPC_HOST, port: int = config.VISION_GRPC_PORT):
        self.channel = grpc.aio.insecure_channel(f'{host}:{port}')
        self.stub = ai_inference_pb2_grpc.VisionServiceStub(self.channel)
        print(f"Vision gRPC Client 초기화 완료 (Connecting to {host}:{port}).")

    async def request_object_detection(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        """
        주어진 이미지 ID 또는 데이터로 객체 인식을 요청합니다.
        """
        request = ai_inference_pb2.ImageRequest(image_id=image_id, image_data=image_data)
        response = await self.stub.DetectObjects(request)
        
        return {
            "object_name": response.object_name,
            "confidence": response.confidence,
            "box": {
                "x": response.box.x,
                "y": response.box.y,
                "width": response.box.width,
                "height": response.box.height
            }
        }

    async def request_face_recognition(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        """
        주어진 이미지 ID 또는 데이터로 얼굴 인식을 요청합니다.
        """
        request = ai_inference_pb2.ImageRequest(image_id=image_id, image_data=image_data)
        response = await self.stub.RecognizeFaces(request)
        
        result = {
            "person_type": response.person_type,
            "confidence": response.confidence
        }
        if response.HasField("employee_id"):
            result["employee_id"] = response.employee_id
            
        return result

    async def request_multiple_object_detection(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        """
        복수 객체 인식을 요청합니다.
        """
        request = ai_inference_pb2.ImageRequest(image_id=image_id, image_data=image_data)
        response = await self.stub.DetectMultipleObjects(request)
        
        objects = []
        for obj in response.objects:
            objects.append({
                "object_name": obj.object_name,
                "confidence": obj.confidence,
                "box": {
                    "x": obj.box.x,
                    "y": obj.box.y,
                    "width": obj.box.width,
                    "height": obj.box.height
                }
            })
            
        return {"objects": objects}

    async def start_vision_stream(self, callback: Any):
        """
        비전 추론 결과 스트림을 구독합니다.
        """
        print("Vision 스트림 구독 시작...")
        try:
            async for result in self.stub.StreamVisionResults(ai_inference_pb2.Empty()):
                data = {
                    "robot_id": result.robot_id,
                    "timestamp": result.timestamp,
                }
                
                if result.HasField("object_detection"):
                    data["type"] = "object_detection"
                    data["content"] = {
                        "object_name": result.object_detection.object_name,
                        "confidence": result.object_detection.confidence
                    }
                elif result.HasField("face_recognition"):
                    data["type"] = "face_recognition"
                    data["content"] = {
                        "person_type": result.face_recognition.person_type,
                        "confidence": result.face_recognition.confidence
                    }
                elif result.HasField("multi_objects"):
                    data["type"] = "multi_objects"
                    data["content"] = [
                        {
                            "object_name": obj.object_name,
                            "confidence": obj.confidence
                        } for obj in result.multi_objects.objects
                    ]
                
                await callback(data)
        except grpc.aio.AioRpcError as e:
            print(f"Vision 스트림 연결 오류: {e}")

    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
