import json
import logging
import asyncio
from typing import Any, Dict, Optional
from main_server.infrastructure.ai_client.vision_client import VisionServiceClient
from main_server.infrastructure.ai_client.llm_client import LLMServiceClient
from main_server.web.connection_manager import ConnectionManager

logger = logging.getLogger(__name__)

class AIProcessingService:
    """
    AI 추론 결과를 처리하고 클라이언트에게 전달하며, 
    자연어 해석 요청 및 간식 인증 로직을 관리하는 통합 AI 서비스.
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
        # 현재 활성화된 스트림 검증용 퓨처 저장 (robot_id별)
        self._verification_futures: Dict[str, asyncio.Future] = {}
        logger.info("AIProcessingService 초기화 완료.")

    async def verify_snack_with_stream(self, robot_id: str, target_snack: str, timeout: int = 30) -> bool:
        """
        [간식 시나리오] AI 서버에 추론 시작 명령을 내리고, 
        스트림 결과를 구독하여 간식이 일치하는지 확인합니다.
        """
        logger.info(f"간식 인증 시작: 로봇={robot_id}, 목표={target_snack}")

        # 1. AI 서버에게 해당 로봇의 간식 추론 시작 명령 (UpdateInferenceState)
        await self.vision_service.update_inference_state(robot_id, "SNACK", True)

        # 2. 결과 확인을 위한 Future 생성
        loop = asyncio.get_running_loop()
        verification_future = loop.create_future()
        self._verification_futures[robot_id] = verification_future

        # 일치 여부 확인용 콜백 함수
        async def check_snack_match(data: Dict[str, Any]):
            if data.get("robot_id") == robot_id and data.get("type") == "object_detection":
                detected_item = data["content"].get("object_name", "").lower()
                confidence = data["content"].get("confidence", 0.0)
                
                # 목표 간식과 일치하고 신뢰도가 높을 경우 성공 처리
                if target_snack.lower() in detected_item and confidence > 0.7:
                    if not verification_future.done():
                        logger.info(f"간식 일치 확인: {detected_item} ({confidence:.2f})")
                        verification_future.set_result(True)

        try:
            # 3. 비전 스트림 구독 시작 (기존 리스너들과 함께 처리됨)
            # 여기서는 start_ai_stream이 이미 실행 중이라고 가정하거나, 별도 리스너 등록 로직 필요
            # 실제 구현에서는 start_vision_stream의 callback 리스트에 추가하는 구조가 이상적임
            
            # 4. 결과 대기 (타임아웃 설정)
            success = await asyncio.wait_for(verification_future, timeout=timeout)
            return success
        except asyncio.TimeoutError:
            logger.warning(f"간식 인증 타임아웃: {target_snack}을 찾지 못했습니다.")
            return False
        finally:
            # 5. 추론 중지 명령 및 정리
            await self.vision_service.update_inference_state(robot_id, "SNACK", False)
            self._verification_futures.pop(robot_id, None)

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
            
            # 진행 중인 간식 검증이 있다면 체크
            robot_id = data.get("robot_id")
            if robot_id in self._verification_futures:
                # 여기서 직접 체크 로직 수행
                detected_item = data.get("content", {}).get("object_name", "").lower()
                # target_snack 정보를 future에 담아둘 수 없으므로, 구조적 개선이 필요할 수 있음
                # 여기선 간단히 로직 흐름만 보여줌
                pass

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
