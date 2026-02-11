import asyncio
import logging
from typing import Any, Dict, Optional
from main_server.domains.tasks.schemas import Task, TaskType, TaskStatus
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.services.ai_management.ai_processing import AIProcessingService

logger = logging.getLogger(__name__)

class TaskManager:
    """
    작업의 생명주기를 관리하고 AI 해석 결과를 로봇 작업으로 변환합니다.
    """
    def __init__(self, 
                 task_repo: MySQLTaskRepository, 
                 location_repo: MySQLLocationRepository,
                 fleet_manager: FleetManager,
                 ai_processing_service: AIProcessingService = None):
        self.task_repo = task_repo
        self.location_repo = location_repo
        self.fleet_manager = fleet_manager
        self.ai_processing_service = ai_processing_service

    async def create_task_from_ai(self, ai_result: Dict[str, Any]) -> Optional[Task]:
        # ... (이전 구현 내용 유지)
        pass

    async def execute_snack_task_step(self, task_id: int, robot_id: int, step_event: str):
        """
        [간식 시나리오] 로봇의 보고를 바탕으로 다음 단계 명령을 내립니다.
        """
        task = await self.task_repo.find_by_id(task_id)
        if not task: return

        if step_event == "ARRIVED_AT_PANTRY_ENTRANCE":
            # 6. 창고 진입 가능 확인 후 간식 위치로 이동
            logger.info(f"로봇 {robot_id} 창고 입구 도착. 간식 위치로 이동 명령.")
            snack_name = task.details.get("item", "snack")
            location_data = await self.location_repo.find_by_name(snack_name)
            if location_data:
                await self.fleet_manager.send_move_command(robot_id, location_data)

        elif step_event == "ARRIVED_AT_SNACK_POINT":
            # 7-8. 간식 인증 시작
            logger.info(f"로봇 {robot_id} 간식 앞 도착. AI 인증 시작.")
            success = await self.ai_processing_service.verify_snack_with_stream(
                robot_id=str(robot_id), 
                target_snack=task.details.get("item", "")
            )
            
            if success:
                # 9-10. 로딩 후 사용자에게 이동
                logger.info("간식 인증 성공. 3초 대기(로딩) 후 사용자에게 이동.")
                await asyncio.sleep(3)
                dest_location = await self.location_repo.find_by_name(task.target_location_name)
                if dest_location:
                    await self.fleet_manager.send_move_command(robot_id, dest_location)
            else:
                logger.error("간식 인증 실패. 작업을 중단하거나 재시도해야 합니다.")
