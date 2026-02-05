import grpc
from typing import Dict, Any, List, Optional
from main_server import config

# Generated gRPC files
from main_server.infrastructure.grpc import ai_inference_pb2
from main_server.infrastructure.grpc import ai_inference_pb2_grpc
from main_server.core_layer.ai_inference.protocols import ILLMService

class LLMServiceClient(ILLMService):
    """
    gRPC를 통해 원격 LLM 서버(Qwen3-4B)와 통신하는 클라이언트 서비스.
    """
    def __init__(self, host: str = config.LLM_GRPC_HOST, port: int = config.LLM_GRPC_PORT):
        self.channel = grpc.aio.insecure_channel(f'{host}:{port}')
        self.stub = ai_inference_pb2_grpc.LLMServiceStub(self.channel)
        print(f"LLM gRPC Client 초기화 완료 (Connecting to {host}:{port}).")

    async def generate_text(self, text: str, max_length: int = 100, temperature: Optional[float] = None) -> Dict[str, Any]:
        """
        주어진 텍스트로 텍스트 생성을 요청합니다.
        """
        request = ai_inference_pb2.TextRequest(
            text=text, 
            max_length=max_length, 
            temperature=temperature
        )
        response = await self.stub.GenerateText(request)
        
        return {
            "generated_text": response.generated_text,
            "confidence": response.confidence
        }

    async def chat(self, messages: List[Dict[str, str]], temperature: Optional[float] = None) -> Dict[str, Any]:
        """
        대화형 응답을 요청합니다.
        """
        chat_messages = [
            ai_inference_pb2.ChatMessage(role=msg["role"], content=msg["content"])
            for msg in messages
        ]
        request = ai_inference_pb2.ChatRequest(
            messages=chat_messages,
            temperature=temperature
        )
        response = await self.stub.Chat(request)
        
        return {
            "response": response.response,
            "confidence": response.confidence
        }

    async def analyze_intent(self, text: str) -> Dict[str, Any]:
        """
        의도 분석을 요청합니다.
        """
        request = ai_inference_pb2.TextRequest(text=text)
        response = await self.stub.AnalyzeIntent(request)
        
        entities = [
            {"type": e.type, "value": e.value, "confidence": e.confidence}
            for e in response.entities
        ]
        
        return {
            "intent": response.intent,
            "confidence": response.confidence,
            "entities": entities
        }

    async def close(self):
        """gRPC 채널을 닫습니다."""
        await self.channel.close()
