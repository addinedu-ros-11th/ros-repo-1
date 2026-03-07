import logging
import asyncio
from typing import Dict, Any, Callable, Optional
from main_server.infrastructure.ai_client.vision_client import VisionServiceClient

logger = logging.getLogger(__name__)

class AIProcessingService:
    """
    AI 서버(Vision)와의 통신을 관리하고, 스트림 결과를 구독하여 
    각 로봇별 핸들러로 분배(Relay)하는 서비스입니다.
    식별자로 robot_name을 사용합니다.
    """
    def __init__(self, vision_service: VisionServiceClient, llm_service=None, connection_manager=None):
        self.vision_client = vision_service
        self.llm_service = llm_service
        self.connection_manager = connection_manager
        self.callbacks: Dict[str, Dict[str, Callable]] = {} # {robot_name: {type: callback}}
        self._stream_task = None

    async def start_ai_stream(self):
        """서비스 시작 시 AI 서버의 결과 스트림 구독을 시작합니다."""
        if self._stream_task is None:
            self._stream_task = asyncio.create_task(self.vision_client.start_vision_stream(self._dispatch_result))
            logger.info("AI 결과 스트림 분배기(Dispatcher) 시작됨.")

    async def process_natural_language(self, req_id: str, message: str) -> Dict[str, Any]:
        """자연어 명령을 해석하여 구조화된 데이터로 반환합니다."""
        if not self.llm_service:
            logger.error("LLM 서비스가 설정되지 않아 자연어 해석을 수행할 수 없습니다.")
            return {"status": "error", "message": "AI 해석 서비스를 사용할 수 없습니다."}
        
        return await self.llm_service.parse_natural_language(req_id, message)

    async def _dispatch_result(self, data: Dict[str, Any]):
        """AI 서버로부터 온 결과를 분석하여 등록된 콜백(로봇 이름별)으로 전달합니다."""
        # gRPC의 robot_id 필드에 로봇 이름이 들어오는 것으로 간주
        robot_name = data.get("robot_id")
        result_type = data.get("type")
        
        if robot_name in self.callbacks:
            callback = self.callbacks[robot_name].get(result_type)
            if callback:
                if asyncio.iscoroutinefunction(callback):
                    await callback(data)
                else:
                    callback(data)

    async def start_obstacle_detection(self, robot_name: str, callback: Callable):
        """장애물 감지 시작 명령 및 콜백 등록"""
        if robot_name not in self.callbacks:
            self.callbacks[robot_name] = {}
        self.callbacks[robot_name]["object_detection"] = callback
        self.callbacks[robot_name]["multi_objects"] = callback
        
        await self.vision_client.update_inference_state(robot_name, "object", True)
        logger.info(f"[{robot_name}] 장애물 감지 추론 시작 명령 전송")

    async def stop_obstacle_detection(self, robot_name: str):
        """장애물 감지 중지"""
        if robot_name in self.callbacks:
            self.callbacks[robot_name].pop("object_detection", None)
            self.callbacks[robot_name].pop("multi_objects", None)
        
        await self.vision_client.update_inference_state(robot_name, "object", False)
        logger.info(f"[{robot_name}] 장애물 감지 추론 중지 명령 전송")

    async def start_employee_verification(self, robot_name: str, callback: Callable):
        """직원/얼굴 인식 시작 명령 및 콜백 등록"""
        if robot_name not in self.callbacks:
            self.callbacks[robot_name] = {}
        self.callbacks[robot_name]["face_recognition"] = callback
        
        await self.vision_client.update_inference_state(robot_name, "face", True)
        logger.info(f"[{robot_name}] 얼굴 인식 추론 시작 명령 전송")

    async def stop_employee_verification(self, robot_name: str):
        """얼굴 인식 중지"""
        if robot_name in self.callbacks:
            self.callbacks[robot_name].pop("face_recognition", None)
        
        await self.vision_client.update_inference_state(robot_name, "face", False)
        logger.info(f"[{robot_name}] 얼굴 인식 추론 중지 명령 전송")
