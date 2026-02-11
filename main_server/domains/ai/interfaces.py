from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

class IVisionService(ABC):
    """
    컴퓨터 비전 서비스(객체 인식, 얼굴 인식 등)를 위한 인터페이스.
    """
    @abstractmethod
    async def request_object_detection(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        pass

    @abstractmethod
    async def request_face_recognition(self, image_id: str, image_data: Optional[bytes] = None) -> Dict[str, Any]:
        pass

    @abstractmethod
    async def update_inference_state(self, robot_id: str, model_type: str, is_active: bool) -> Dict[str, Any]:
        """AI 서버의 추론 상태를 제어합니다 (시작/중지)."""
        pass

    @abstractmethod
    async def start_vision_stream(self, callback: Any):
        pass

    @abstractmethod
    async def close(self):
        pass

class ILLMService(ABC):
    """
    대규모 언어 모델 서비스(자연어 해석 등)를 위한 인터페이스.
    """
    @abstractmethod
    async def parse_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
        pass

    @abstractmethod
    async def close(self):
        pass

class IAIProcessingService(ABC):
    """
    비전과 LLM 기능을 통합하여 메인 서버의 비즈니스 로직을 처리하는 인터페이스.
    """
    @abstractmethod
    async def start_ai_stream(self):
        pass

    @abstractmethod
    async def process_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
        pass
