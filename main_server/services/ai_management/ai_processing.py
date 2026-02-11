import json
import logging
from typing import Any, Dict, Optional
from main_server.infrastructure.ai_client.vision_client import VisionServiceClient
from main_server.infrastructure.ai_client.llm_client import LLMServiceClient
from main_server.web.connection_manager import ConnectionManager

logger = logging.getLogger(__name__)

class AIProcessingService:
    """
    AI 추론 결과를 처리하고 클라이언트에게 전달하며, 
    자연어 해석 요청을 관리하는 통합 AI 서비스.
    """
    def __init__(
        self, 
        vision_service: VisionServiceClient, 
        llm_service: LLMServiceClient,
        connection_manager: ConnectionManager
    ):
        self.vision_service = vision_service
        self.llm_service = llm_service
        self.connection_manager = connection_manager
        logger.info("AIProcessingService 초기화 완료.")

    async def start_ai_stream(self):
        """
        AI 서버로부터 실시간 비전 추론 스트림을 구독하고 웹소켓으로 브로드캐스트합니다.
        """
        logger.info("AI 실시간 추론 스트림 구독 시작...")
        
        async def ai_callback(data: Dict[str, Any]):
            # 수신된 AI 결과를 WebSocket을 통해 모든 관리자에게 전송
            message = {
                "event": "ai_inference",
                "data": data
            }
            await self.connection_manager.broadcast(json.dumps(message))

        try:
            await self.vision_service.start_vision_stream(ai_callback)
        except Exception as e:
            logger.error(f"AI 스트림 처리 중 오류 발생: {e}")

    async def process_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
        """
        사용자의 자연어 명령을 해석하여 구조화된 데이터로 반환합니다.
        """
        logger.info(f"자연어 해석 요청 [ID: {req_id}]: {message}")
        try:
            result = await self.llm_service.parse_natural_language(req_id, message)
            return result
        except Exception as e:
            logger.error(f"자연어 해석 중 오류 발생: {e}")
            return {
                "req_id": req_id,
                "error": str(e),
                "task_type": "UNKNOWN"
            }
