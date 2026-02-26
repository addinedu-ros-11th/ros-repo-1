import logging
from typing import Any, Dict, Optional

from main_server.domains.robots.schemas import RobotStatus
from main_server.domains.tasks.schemas import Task, TaskType
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository
from main_server.infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository
from main_server.services.ai_management.ai_processing import AIProcessingService
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.services.task_management.scenario_data_handler import ScenarioDataHandler
from main_server.services.task_management.task_processors import GuideProcessor, ItemProcessor, SnackProcessor
from main_server.web.connection_manager import ConnectionManager

logger = logging.getLogger(__name__)


class TaskManager:
    """
    작업의 통합 관리자. 작업 유형에 따라 적절한 Processor를 선택하여 실행합니다.
    """
    def __init__(self,
                 task_repo: MySQLTaskRepository,
                 location_repo: MySQLLocationRepository,
                 user_repo: MySQLUserRepository,
                 product_repo: MySQLProductRepository,
                 fleet_manager: FleetManager,
                 ai_processing_service: AIProcessingService,
                 connection_manager: ConnectionManager):
        """
        참고: 이 변경으로 인해 main_server/container.py에서 TaskManager 생성 시
              user_repo와 product_repo를 추가로 주입해야 합니다.
        """
        self.task_repo = task_repo
        self.location_repo = location_repo
        self.fleet_manager = fleet_manager

        # AI 결과를 시나리오에 맞게 처리하는 핸들러
        self.scenario_handler = ScenarioDataHandler(location_repo, user_repo, product_repo)

        # Processor 등록 (시나리오 확장 시 여기에 추가)
        self.processors = {
            TaskType.SNACK_DELIVERY: SnackProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager),
            TaskType.GUIDE_GUEST: GuideProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager),
            TaskType.ITEM_DELIVERY: ItemProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager),
        }

    async def create_task_from_ai(self, ai_result: Dict[str, Any], caller_name: Optional[str] = None) -> Optional[Task]:
        """AI 해석 결과로 태스크를 생성하고 로봇을 배차합니다."""

        # 1. AI 결과 보정
        if "fields" not in ai_result or ai_result["fields"] is None:
            ai_result["fields"] = {}
        
        fields = ai_result["fields"]
        
        # Requester name이 AI 결과에 없거나 'user' 같은 placeholder일 경우, 쿠키에서 가져온 caller_name을 기본값으로 사용
        req_name = fields.get("requester_name")
        if (not req_name or req_name.lower() == "user") and caller_name:
            fields["requester_name"] = caller_name
            logger.info(f"Requester name missing or placeholder in AI result. Injected caller_name: {caller_name}")
        
        if not fields.get("requester_name"):
            logger.warning("Both requester_name and caller_name are missing.")

        # 2. 시나리오 핸들러를 통해 AI 결과 처리 및 DB 저장용 데이터 준비
        prepared_data = await self.scenario_handler.prepare_task_data(ai_result)

        if not prepared_data:
            logger.error("Failed to prepare task data from AI result.")
            return None

        task_data = prepared_data["task_data"]
        task_items = prepared_data.get("task_items")
        initial_destination_name = prepared_data["initial_destination_name"]

        # 2. 초기 목적지 좌표 결정 (로봇 배차용)
        if initial_destination_name == "MANUAL_COORDINATE":
            # 수동 이동 시나리오: fields에 포함된 좌표를 직접 사용
            initial_target_pose = (fields.get("x", 0.0), fields.get("y", 0.0))
            logger.info(f"Manual move detected. Using direct coordinates: {initial_target_pose}")
        else:
            # 일반 시나리오: 위치 이름을 기반으로 DB에서 좌표 조회
            initial_location_data = await self.location_repo.find_by_name(initial_destination_name)
            if not initial_location_data:
                logger.error(f"초기 목적지 '{initial_destination_name}'를 찾을 수 없습니다.")
                return None
            initial_target_pose = (initial_location_data["coordinate_x"], initial_location_data["coordinate_y"])

        # 3. 최적 로봇 탐색
        optimal_robot = await self.fleet_manager.find_optimal_robot(initial_target_pose)
        if not optimal_robot:
            # 로봇이 없어도 태스크는 PENDING 상태로 생성할 수 있으나,
            # 현재 요구사항은 '가용한 로봇이 없을 때' 즉시 retry를 유도하므로 None을 반환합니다.
            logger.warning(f"태스크 {task_data.get('task_type')}를 처리할 적절한 로봇이 없습니다.")
            return None

        # 로봇 ID를 태스크 데이터에 반영
        task_data["assigned_robot_id"] = optimal_robot.id

        # 4. 태스크 생성 (DB)
        task = await self.task_repo.create(task_data, task_items)
        if not task:
            logger.error("Failed to create task in database.")
            return None

        # 5. 로봇 할당 및 초기 명령 전송
        await self.assign_and_dispatch(optimal_robot, task)
        return task

    async def assign_and_dispatch(self, robot, task):
        """로봇에게 작업을 할당하고 해당 시나리오의 초기 명령을 전송합니다."""
        await self.fleet_manager.update_robot_task_status(robot.id, task.id, RobotStatus.MOVING)

        processor = self.processors.get(task.task_type)
        if processor:
            actions = await processor.get_initial_actions(task)
            if actions:
                self.fleet_manager.send_action_commands(robot.name, actions)
        else:
            logger.error(f"작업 타입 {task.task_type}에 대한 처리기가 없습니다.")

    async def handle_robot_event(self, task_id: int, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        """로봇으로부터 수신된 이벤트(도착 등)를 처리기에 전달합니다."""
        task = await self.task_repo.get_by_id(task_id)
        if not task: return

        processor = self.processors.get(task.task_type)
        if processor:
            await processor.handle_event(task, robot_id, event, data)

    async def confirm_delivery(self, task_id: int, action_type: str):
        """사용자로부터 확인(적재/수령)을 받아 처리합니다."""
        from common.robot_task_events import RobotEvent

        task = await self.task_repo.get_by_id(task_id)
        if not task:
            return False, "Task not found"

        robot_id = task.assigned_robot_id
        if not robot_id:
            return False, "Robot not assigned"

        processor = self.processors.get(task.task_type)
        if not processor:
            return False, "Processor not found"

        event = None
        if action_type == "CONFIRM_SNACK_RECEIPT":
            event = RobotEvent.DELIVERY_CONFIRMED
        elif action_type == "CONFIRM_LOADING":
            event = RobotEvent.LOADING_COMPLETE
        elif action_type == "CONFIRM_ITEM_RECEIPT":
            event = RobotEvent.DELIVERY_CONFIRMED

        if event:
            await processor.handle_event(task, robot_id, event)
            return True, "Confirmed"

        return False, "Invalid action type"
