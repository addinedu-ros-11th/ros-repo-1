import grpc
from typing import Dict, Any
from main_server import config

# Generated gRPC files
from main_server.infrastructure.grpc import ai_inference_pb2
from main_server.infrastructure.grpc import ai_inference_pb2_grpc

class AIInferenceService:
    """
    gRPC를 통해 원격 AI Inference 서버와 통신하는 클라이언트 서비스.
    """
    def __init__(self, host: str = config.AI_INFERENCE_GRPC_HOST, port: int = config.AI_INFERENCE_GRPC_PORT):
        self.channel = grpc.aio.insecure_channel(f'{host}:{port}')
        self.stub = ai_inference_pb2_grpc.AIInferenceStub(self.channel)
        print(f"AI Inference gRPC Client 초기화 완료 (Connecting to {host}:{port}).")

    async def request_object_detection(self, image_id: str) -> Dict[str, Any]:
        """
        주어진 이미지 ID로 객체 인식을 요청합니다.
        """
        request = ai_inference_pb2.ImageRequest(image_id=image_id)
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

    async def request_face_recognition(self, image_id: str) -> Dict[str, Any]:
        """
        주어진 이미지 ID로 얼굴 인식을 요청합니다. (Unary)
        """
        request = ai_inference_pb2.ImageRequest(image_id=image_id)
        response = await self.stub.RecognizeFaces(request)
        
        result = {
            "person_type": response.person_type,
            "confidence": response.confidence
        }
        if response.HasField("employee_id"): 
            result["employee_id"] = response.employee_id
            
        return result

    async def start_inference_stream(self, callback: Any):
        """
        AI 서버로부터 실시간 추론 결과 스트림을 구독합니다. (Server Streaming)
        결과가 수신될 때마다 전달된 콜백 함수를 호출합니다.
        """
        print("AI 추론 결과 스트림 구독 시작...")
        try:
            async for result in self.stub.StreamInferenceResults(ai_inference_pb2.Empty()):
                # 스트림에서 받은 데이터 처리
                data = {
                    "robot_id": result.robot_id,
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
                
                await callback(data)
        except grpc.aio.AioRpcError as e:
            print(f"AI 스트림 연결 오류: {e}")
    
    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
