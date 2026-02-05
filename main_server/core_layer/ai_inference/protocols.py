from typing import Protocol, Dict, Any, Awaitable

class IAIInferenceService(Protocol):
    """
    AI 추론 서비스와의 통신을 위한 인터페이스(프로토콜).
    Core Layer는 구체적인 통신 방식(gRPC, HTTP 등)을 알 필요 없이 이 인터페이스에 의존합니다.
    """

    async def request_object_detection(self, image_id: str) -> Dict[str, Any]:
        """
        주어진 이미지 ID로 객체 인식을 요청합니다.
        """
        ...

    async def request_face_recognition(self, image_id: str) -> Dict[str, Any]:
        """
        주어진 이미지 ID로 얼굴 인식을 요청합니다.
        """
        ...

    async def start_inference_stream(self, callback: Any) -> None:
        """
        실시간 추론 결과 스트림을 구독합니다.
        """
        ...
    
    async def close(self) -> None:
        """연결을 종료합니다."""
        ...
