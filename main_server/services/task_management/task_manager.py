import logging
from typing import Any, Dict, Optional, List
from main_server.domains.tasks.schemas import Task, TaskType, TaskStatus
from main_server.domains.robots.schemas import RobotStatus
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.services.task_management.task_processors import SnackProcessor, GuideProcessor, ItemProcessor

logger = logging.getLogger(__name__)

class TaskManager:
    """
    작업의 통합 관리자. 작업 유형에 따라 적절한 Processor를 선택하여 실행합니다.
    """
    def __init__(self, 
                 task_repo: MySQLTaskRepository, 
                 location_repo: MySQLLocationRepository,
                 fleet_manager: FleetManager,
                 ai_processing_service: Any = None):
        self.task_repo = task_repo
        self.location_repo = location_repo
        self.fleet_manager = fleet_manager
        
        # Processor 등록 (시나리오 확장 시 여기에 추가)
        self.processors = {
            TaskType.SNACK_DELIVERY: SnackProcessor(fleet_manager, location_repo, task_repo, ai_processing_service),
            TaskType.GUIDE_GUEST: GuideProcessor(fleet_manager, location_repo, task_repo, ai_processing_service),
            TaskType.ITEM_DELIVERY: ItemProcessor(fleet_manager, location_repo, task_repo, ai_processing_service),
        }

    async def create_task_from_ai(self, ai_result: Dict[str, Any]) -> Optional[Task]:
        """AI 해석 결과로 태스크를 생성하고 로봇을 배차합니다."""
        task_type_str = ai_result.get("task_type")
        fields = ai_result.get("fields", {})
        
        dest_name = fields.get("location") or fields.get("dest_location", "lobby")
        location_data = await self.location_repo.find_by_name(dest_name)
        target_pose = (location_data["coordinate_x"], location_data["coordinate_y"]) if location_data else (0.0, 0.0)

        task_data = {
            "task_type": task_type_str,
            "status": TaskStatus.PENDING,
            "details": fields,
            "target_location_name": dest_name,
            "destination_id": location_data["location_id"] if location_data else None
        }

        task = await self.task_repo.create(task_data)
        
        optimal_robot = await self.fleet_manager.find_optimal_robot(target_pose)
        if optimal_robot:
            await self.assign_and_dispatch(optimal_robot, task)
            return task
        return None

    async def assign_and_dispatch(self, robot, task):
        """로봇에게 작업을 할당하고 해당 시나리오의 초기 명령을 전송합니다."""
        await self.fleet_manager.update_robot_task_status(robot.id, task.id, RobotStatus.MOVING)
        
        # 해당 작업 타입의 Processor 가져오기
        processor = self.processors.get(task.task_type)
        if processor:
            actions = await processor.get_initial_actions(task)
            if actions:
                self.fleet_manager.send_action_commands(robot.name, actions)
        else:
            logger.error(f"작업 타입 {task.task_type}에 대한 처리기가 없습니다.")

    async def handle_robot_event(self, task_id: int, robot_id: int, event: str):
        """로봇으로부터 수신된 이벤트(도착 등)를 처리기에 전달합니다."""
        task = await self.task_repo.get_by_id(task_id)
        if not task: return

        processor = self.processors.get(task.task_type)
        if processor:
            await processor.handle_event(task, robot_id, event)
