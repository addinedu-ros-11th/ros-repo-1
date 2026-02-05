from typing import Protocol, Dict, Any, List, Optional, Awaitable

class ILLMService(Protocol):
    """
    LLM 서비스(Qwen3-4B)와의 통신을 위한 인터페이스.
    """
    async def generate_text(self, text: str, max_length: int = 100, temperature: Optional[float] = None) -> Dict[str, Any]:
        ...

    async def chat(self, messages: List[Dict[str, str]], temperature: Optional[float] = None) -> Dict[str, Any]:
        ...

    async def analyze_intent(self, text: str) -> Dict[str, Any]:
        ...

    async def close(self) -> None:
        ...

class IVisionService(Protocol):
    """
    Vision 서비스(YOLOv8n)와의 통신을 위한 인터페이스.
    """
    async def request_object_detection(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        ...

    async def request_face_recognition(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        ...

    async def request_multiple_object_detection(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        ...

    async def start_vision_stream(self, callback: Any) -> None:
        ...

    async def close(self) -> None:
        ...

class IAIInferenceService(Protocol):
    """
    (Legacy) AI 추론 서비스와의 통신을 위한 인터페이스.
    """
    async def request_object_detection(self, image_id: str) -> Dict[str, Any]:
        ...

    async def request_face_recognition(self, image_id: str) -> Dict[str, Any]:
        ...

    async def start_inference_stream(self, callback: Any) -> None:
        ...
    
    async def close(self) -> None:
        ...
