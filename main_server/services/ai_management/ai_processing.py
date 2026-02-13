import json
import logging
import asyncio
from typing import Any, Dict, Optional, Callable, List, Union
from collections import defaultdict
from main_server.infrastructure.ai_client.vision_client import VisionServiceClient
from main_server.infrastructure.ai_client.llm_client import LLMServiceClient
from main_server.web.connection_manager import ConnectionManager

logger = logging.getLogger(__name__)

class AIProcessingService:
    """
    AI 추론 결과를 처리하고 클라이언트에게 전달하며, 
    자연어 해석 요청 및 시나리오별(직원, 간식, 장애물) AI 스트림을 관리하는 통합 서비스.
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
        
        # 시나리오별 콜백 핸들러 저장소
        # 구조: { robot_id: { scenario_type: callback_function } }
        self._scenario_handlers: Dict[str, Dict[str, Union[Callable[[Dict[str, Any]], None], Callable[[Dict[str, Any]], Any]]]] = defaultdict(dict)
        
        # 스트림 태스크 상태 관리
        self._stream_task: Optional[asyncio.Task] = None
        
        logger.info("AIProcessingService 초기화 완료.")

    async def start_ai_stream(self):
        """
        AI 서버로부터 실시간 비전 추론 스트림을 구독하고,
        1. 웹소켓으로 브로드캐스트 (관리자 모니터링용)
        2. 등록된 시나리오 핸들러로 데이터 디스패치 (로직 처리용)
        """
        if self._stream_task and not self._stream_task.done():
            logger.warning("AI 스트림이 이미 실행 중입니다.")
            return

        logger.info("AI 실시간 추론 스트림 리스너 시작...")
        
        async def ai_callback(data: Dict[str, Any]):
            # 1. 수신된 AI 결과를 WebSocket을 통해 모든 관리자에게 전송 (모니터링)
            try:
                message = {
                    "event": "ai_inference",
                    "data": data
                }
                await self.connection_manager.broadcast(json.dumps(message))
            except Exception as e:
                logger.error(f"WebSocket 브로드캐스트 실패: {e}")

            # 2. 로봇 ID 및 데이터 타입 추출
            robot_id = data.get("robot_id")
            if not robot_id:
                return

            # 3. 등록된 시나리오 핸들러에게 디스패치
            # 해당 로봇에 등록된 모든 핸들러를 순회하며 데이터 전달
            handlers = self._scenario_handlers.get(robot_id, {})
            for scenario, handler in handlers.items():
                try:
                    # 시나리오별로 필요한 데이터 필터링은 핸들러 내부에서 수행하거나,
                    # 여기서 타입에 따라 분기할 수 있음. 현재는 모든 데이터를 넘김.
                    if asyncio.iscoroutinefunction(handler):
                        await handler(data)
                    else:
                        handler(data)
                except Exception as e:
                    logger.error(f"시나리오 핸들러 실행 중 오류 ({robot_id}, {scenario}): {e}")

        try:
            # gRPC 스트림 구독 시작 (블로킹 호출이므로 await)
            await self.vision_service.start_vision_stream(ai_callback)
        except Exception as e:
            logger.error(f"AI 스트림 연결 종료 또는 오류 발생: {e}")
        finally:
            logger.info("AI 스트림 리스너가 종료되었습니다.")

    # ==========================================
    # 시나리오별 제어 메서드 (Start/Stop)
    # ==========================================

    async def start_employee_verification(self, robot_id: str, callback: Callable[[Dict[str, Any]], None]):
        """
        [직원 확인 시나리오] 시작
        - AI 서버에 얼굴 인식 활성화 요청
        - 결과 처리 콜백 등록
        """
        logger.info(f"[{robot_id}] 직원 확인 모드 시작")
        
        # 1. 핸들러 등록
        self._scenario_handlers[robot_id]["EMPLOYEE"] = callback
        
        # 2. AI 서버에 추론 시작 명령
        await self.vision_service.update_inference_state(robot_id, "EMPLOYEE", True)

    async def stop_employee_verification(self, robot_id: str):
        """
        [직원 확인 시나리오] 종료
        """
        logger.info(f"[{robot_id}] 직원 확인 모드 종료")
        
        # 1. AI 서버에 추론 중지 명령
        await self.vision_service.update_inference_state(robot_id, "EMPLOYEE", False)
        
        # 2. 핸들러 제거
        if "EMPLOYEE" in self._scenario_handlers[robot_id]:
            del self._scenario_handlers[robot_id]["EMPLOYEE"]

    async def start_obstacle_detection(self, robot_id: str, callback: Callable[[Dict[str, Any]], None]):
        """
        [동적 장애물 판별 시나리오] 시작 (사람/로봇 구분)
        """
        logger.info(f"[{robot_id}] 장애물 감지 모드 시작")
        
        self._scenario_handlers[robot_id]["OBSTACLE"] = callback
        await self.vision_service.update_inference_state(robot_id, "OBSTACLE", True)

    async def stop_obstacle_detection(self, robot_id: str):
        """
        [동적 장애물 판별 시나리오] 종료
        """
        logger.info(f"[{robot_id}] 장애물 감지 모드 종료")
        
        await self.vision_service.update_inference_state(robot_id, "OBSTACLE", False)
        if "OBSTACLE" in self._scenario_handlers[robot_id]:
            del self._scenario_handlers[robot_id]["OBSTACLE"]

    async def verify_snack_with_stream(self, robot_id: str, target_snack: str, timeout: int = 30) -> bool:
        """
        [간식 시나리오] 시작 및 결과 대기 (One-shot verification)
        - 편의를 위해 내부적으로 start/stop을 관리하고 Future로 결과를 반환함.
        """
        logger.info(f"[{robot_id}] 간식 인증 요청: 목표={target_snack}")
        
        loop = asyncio.get_running_loop()
        future = loop.create_future()

        # 임시 콜백 함수 정의
        async def snack_callback(data: Dict[str, Any]):
            if data.get("type") == "object_detection":
                detected_item = data["content"].get("object_name", "").lower()
                confidence = data["content"].get("confidence", 0.0)
                
                # 목표 간식과 일치하고 신뢰도가 높을 경우
                if target_snack.lower() in detected_item and confidence > 0.7:
                    if not future.done():
                        logger.info(f"간식 일치 확인: {detected_item} ({confidence:.2f})")
                        future.set_result(True)

        try:
            # 1. 모니터링 시작 (핸들러 등록 + AI 명령)
            self._scenario_handlers[robot_id]["SNACK"] = snack_callback
            await self.vision_service.update_inference_state(robot_id, "SNACK", True)
            
            # 2. 결과 대기
            result = await asyncio.wait_for(future, timeout=timeout)
            return result
            
        except asyncio.TimeoutError:
            logger.warning(f"간식 인증 타임아웃: {target_snack}")
            return False
        finally:
            # 3. 모니터링 종료 (AI 명령 + 핸들러 제거)
            await self.vision_service.update_inference_state(robot_id, "SNACK", False)
            if "SNACK" in self._scenario_handlers[robot_id]:
                del self._scenario_handlers[robot_id]["SNACK"]

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
