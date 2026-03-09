import logging
import asyncio
from datetime import datetime, timedelta
from typing import Any, Dict, List, Optional, Set

from main_server.domains.robots.schemas import RobotStatus, RobotEvent
from main_server.domains.tasks.schemas import Task, TaskType, TaskStatus
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository
from main_server.infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository
from main_server.services.ai_management.ai_processing import AIProcessingService
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.services.task_management.scenario_data_handler import ScenarioDataHandler
from main_server.services.task_management.task_processors import GuideProcessor, ItemProcessor, SnackProcessor, ManualMoveProcessor, GuestCheckProcessor
from main_server.web.connection_manager import ConnectionManager
from main_server.infrastructure.database.repositories.mysql_visitor_repository import MySQLVisitorRepository

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
                 visitor_repo: MySQLVisitorRepository,
                 fleet_manager: FleetManager,
                 ai_processing_service: AIProcessingService,
                 connection_manager: ConnectionManager):
        """
        참고: 이 변경으로 인해 main_server/container.py에서 TaskManager 생성 시
              visitor_repo를 추가로 주입해야 합니다.
        """
        self.task_repo = task_repo
        self.location_repo = location_repo
        self.user_repo = user_repo
        self.visitor_repo = visitor_repo
        self.fleet_manager = fleet_manager
        
        # [Addition] 태스크 생성 중인 로봇을 추적하여 중복 처리를 방지합니다.
        self.processing_robots = set()

        # AI 결과를 시나리오에 맞게 처리하는 핸들러
        self.scenario_handler = ScenarioDataHandler(location_repo, user_repo, product_repo)

        # Processor 등록 (시나리오 확장 시 여기에 추가)
        # 각 프로세서가 태스크 완료 시 호출할 콜백 전달
        self.processors = {
            TaskType.SNACK_DELIVERY: SnackProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, product_repo, on_complete=self.on_task_complete),
            TaskType.GUIDE_GUEST: GuideProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, product_repo, on_complete=self.on_task_complete),
            TaskType.ITEM_DELIVERY: ItemProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, product_repo, on_complete=self.on_task_complete),
            TaskType.MANUAL_MOVE: ManualMoveProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, product_repo, on_complete=self.on_task_complete),
            TaskType.GUEST_CHECK: GuestCheckProcessor(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, visitor_repo, product_repo, on_complete=self.on_task_complete),
        }

    async def on_task_complete(self, robot_id: int):
        """로봇이 작업을 마쳤을 때 호출되는 콜백"""
        logger.info(f"[TaskManager] 로봇 {robot_id} 작업 완료. 대기열 확인.")
        await self.process_pending_tasks()

    async def process_pending_tasks(self):
        """대기 중인 태스크(PENDING)를 조회하여 가용한 로봇에 배차합니다."""
        pending_tasks = await self.task_repo.get_all_by_status(TaskStatus.PENDING)
        
        if not pending_tasks:
            return

        logger.info(f"[TaskManager] 대기 중인 태스크 {len(pending_tasks)}개 발견. 배차 시도.")
        for task in pending_tasks:
            assigned = await self._try_assign_robot(task)
            if not assigned:
                # 현재 가용한 로봇이 없으면 이후 태스크도 배차 불가능할 가능성이 높음 (단, 로봇 위치에 따라 다를 수 있음)
                # find_optimal_robot은 IDLE 로봇이 하나라도 있어야 동작하므로, 
                # 여기서 실패했다면 더 이상 시도할 필요가 없음.
                logger.info("[TaskManager] 가용한 로봇이 없어 배차 중단.")
                break

    async def _try_assign_robot(self, task: Task) -> bool:
        """태스크에 대해 로봇 배차를 시도합니다."""
        target_pose = await self._get_initial_target_pose(task)
        
        # 타겟 위치를 특정할 수 없는 경우 (예: GUEST_CHECK 수동 할당 등)
        # 로봇 위치(0,0) 기준이라도 배차를 시도해야 할지 결정 필요. 
        # 일단 (0,0)으로 가정하거나 생략.
        if not target_pose:
            target_pose = (0.0, 0.0)

        optimal_robot = await self.fleet_manager.find_optimal_robot(target_pose)
        if not optimal_robot:
            return False

        # 배차 성공
        logger.info(f"[TaskManager] 태스크 {task.id} -> 로봇 {optimal_robot.name} 배차.")
        
        # 1. 태스크 정보 업데이트 (로봇 ID, 상태)
        # assign_and_dispatch 내부에서 robot status 업데이트 하므로 여기서는 Task Status만 먼저 업데이트?
        # 아니면 assign_and_dispatch가 Task Status도 관리?
        # 기존 로직: create_task_from_ai -> task_data["assigned_robot_id"] set -> task_repo.create
        # -> assign_and_dispatch (update robot status)
        
        # 여기서는 이미 Task가 DB에 있음.
        await self.task_repo.update(task.id, {
            "assigned_robot_id": optimal_robot.id,
            "status": TaskStatus.ASSIGNED # or MOVING? Existing logic used MOVING for Robot, but Task?
        })
        
        # 2. 실제 명령 전송
        await self.assign_and_dispatch(optimal_robot, task)
        return True

    async def _get_initial_target_pose(self, task: Task) -> Optional[tuple]:
        """태스크 유형에 따른 초기 이동 목적지 좌표를 반환합니다."""
        # 1. Manual Move
        if task.task_type == TaskType.MANUAL_MOVE:
            return (task.details.get("x", 0.0), task.details.get("y", 0.0))
        
        target_name = None
        
        # 2. Snack Delivery -> Pantry (snack_waiting_area)
        if task.task_type == TaskType.SNACK_DELIVERY:
            target_name = "snack_waiting_area"

        # 3. Item Delivery -> Sender Location
        elif task.task_type == TaskType.ITEM_DELIVERY:
            target_name = task.details.get("source_location", "office_1")

        # 4. Guide Guest -> Target Location (or Guest Location?)
        # GuideProcessor uses 'location' in details as destination.
        elif task.task_type == TaskType.GUIDE_GUEST:
            target_name = task.details.get("location")

        if target_name:
            loc = await self.location_repo.find_by_name(target_name)
            if loc:
                return (loc["coordinate_x"], loc["coordinate_y"])
        
        return None

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

        # 3. 태스크 생성 (DB) - 상태: PENDING
        task_data["status"] = TaskStatus.PENDING
        task = await self.task_repo.create(task_data, task_items)
        if not task:
            logger.error("Failed to create task in database.")
            return None

        # 4. 로봇 배차 시도
        if await self._try_assign_robot(task):
            logger.info(f"Task {task.id} assigned immediately.")
        else:
            logger.info(f"Task {task.id} queued (No robot available).")

        return task

    async def assign_and_dispatch(self, robot, task):
        """로봇에게 작업을 할당하고 해당 시나리오의 초기 명령을 전송합니다."""
        logger.info(f"🔗 [TaskManager] 배차 및 실행: Task {task.id} ({task.task_type}) -> Robot {robot.name}")
        await self.fleet_manager.update_robot_task_status(robot.id, task.id, RobotStatus.MOVING)

        processor = self.processors.get(task.task_type)
        if processor:
            actions = await processor.get_initial_actions(task)
            if actions:
                self.fleet_manager.send_action_commands(robot.name, actions, task_id=task.id)
            else:
                logger.warning(f"⚠️ [TaskManager] Task {task.id}에 대한 초기 액션이 없습니다.")
        else:
            logger.error(f"❌ [TaskManager] 작업 타입 {task.task_type}에 대한 처리기가 없습니다.")

    async def handle_robot_event(self, task_id: int, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        """로봇으로부터 수신된 이벤트(도착 등)를 처리기에 전달합니다."""
        logger.info(f"[TaskManager] 로봇 이벤트 수신: {event} (Task: {task_id}, Robot: {robot_id})")
        task = await self.task_repo.get_by_id(task_id)
        if not task: 
            logger.error(f"[TaskManager] 해당 태스크를 찾을 수 없습니다: ID {task_id}")
            return

        processor = self.processors.get(task.task_type)
        if processor:
            await processor.handle_event(task, robot_id, event, data)
        else:
            logger.error(f"[TaskManager] 작업 타입 {task.task_type}에 대한 처리기가 없습니다.")

    async def handle_face_recognition_event(self, robot_id: str, face_data: Dict[str, Any]):
        """
        FleetManager로부터 전달받은 얼굴 인식 결과를 처리합니다.
        - 직원(Employee): 환영 메시지 및 LED 제어 명령 전송
        - 외부인/미인식자(Guest/Unknown): QR 인증 태스크(GUEST_CHECK) 생성
        """
        logger.info(f"🔍 [TaskManager] Face Recognition Event Received: robot_id={robot_id}, data={face_data}")
        
        person_type = face_data.get("person_type", "Unknown")
        confidence = face_data.get("confidence", 0.0)
        employee_account = face_data.get("employee_id") # AI 서버가 보내는 식별자

        # robot_id(str)로 로봇 정보 조회
        robot = await self.fleet_manager.robot_repo.get_by_name(robot_id)
        if not robot:
            logger.error(f"❌ [TaskManager] Robot '{robot_id}' not found in DB! Please check if robot name matches.")
            return
            
        if robot.status == RobotStatus.OFFLINE:
            logger.warning(f"⚠️ [TaskManager] Robot '{robot_id}' is OFFLINE. Ignoring recognition event.")
            return

        # 이미 작업 중이거나 생성 중인 로봇은 무시 (Race Condition 방지)
        if robot.current_task_id:
             logger.info(f"ℹ️ [TaskManager] Robot '{robot_id}' is busy (Task ID: {robot.current_task_id}). Skipping face event.")
             return
             
        if robot.name in self.processing_robots:
             logger.info(f"ℹ️ [TaskManager] Robot '{robot_id}' is currently processing another task creation. Skipping.")
             return

        # 1. 직원 인식 시: DB에서 이름을 찾아 환영 메시지 전송
        if person_type == "Employee" and employee_account:
            logger.info(f"👤 [TaskManager] Employee Detected: {employee_account}")
            user = await self.user_repo.get_user_by_username(employee_account)
            user_name = user.name if user else employee_account
            
            logger.info(f"[{robot_id}] 직원 인식됨: {user_name} ({confidence:.2f}) -> 환영 처리 시작")
            actions = [
                {"action": "SET_LED", "params": {"color": "GREEN", "mode": "SOLID"}},
                {"action": "DISPLAY_TEXT", "params": {"text": f"Hello, {user_name}", "duration": 5}},
            ]
            self.fleet_manager.send_action_commands(robot_id, actions)
            logger.info(f"✅ [TaskManager] Employee welcome command sent to {robot_id}")
            
        # 2. 외부인 또는 미인식자 감지 시: QR 인증 태스크 생성 및 명령 전송
        else:
            logger.info(f"🕵️ [TaskManager] Non-employee detected ({person_type}). Starting GUEST_CHECK via Processor.")
            
            self.processing_robots.add(robot.name)
            try:
                # [태스크 생성] 먼저 생성하여 Task ID 확보
                purpose = "VISITOR_SCAN"
                task_data = {
                    "task_type": "GUEST_CHECK",
                    "requester_id": 1, # System
                    "status": "ASSIGNED",
                    "assigned_robot_id": robot.id,
                    "details": {"reason": "stranger_detected", "person_type": person_type, "purpose": purpose}
                }
                task = await self.task_repo.create(task_data)
                
                if task:
                    # [연결] 이제 TaskProcessor(GuestCheckProcessor)가 명령을 생성하고 전송하도록 위임
                    await self.assign_and_dispatch(robot, task)
                    logger.info(f"✅ [TaskManager] GUEST_CHECK task {task.id} assigned to {robot.name}")
            finally:
                self.processing_robots.remove(robot.name)

    async def start_watchdog(self, interval_seconds: int = 10):
        """작업 대기열 및 멈춘 작업을 주기적으로 감시하는 루프를 시작합니다."""
        logger.info(f"🛡️ [TaskManager] Task Watchdog started (Interval: {interval_seconds}s)")
        while True:
            try:
                await asyncio.sleep(interval_seconds)
                await self.cleanup_stuck_tasks()
                await self.process_pending_tasks() # 대기열도 주기적으로 재확인
            except Exception as e:
                logger.error(f"❌ [TaskManager] Watchdog Error: {e}")

    async def cleanup_stuck_tasks(self, timeout_minutes: int = 3):
        """수행 중인 상태로 너무 오래 방치된 작업을 찾아 실패 처리합니다."""
        # 감시할 상태들
        active_statuses = [TaskStatus.ASSIGNED, TaskStatus.IN_PROGRESS]
        
        for status in active_statuses:
            tasks = await self.task_repo.get_all_by_status(status)
            now = datetime.utcnow()
            
            for task in tasks:
                # [특수 처리] GUEST_CHECK은 로봇 스캔 종료(15초)에 맞춰 30초 후 자동 만료
                current_timeout = timedelta(minutes=timeout_minutes)
                if task.task_type == TaskType.GUEST_CHECK:
                    current_timeout = timedelta(seconds=30)
                
                # 생성된 지 타임아웃 이상 지났는지 확인
                if now - task.created_at > current_timeout:
                    logger.warning(f"⏰ [TaskManager] Task {task.id} ({task.task_type}) has been stuck for {now - task.created_at}. Cleaning up.")
                    
                    # 1. 태스크 상태 업데이트 (FAILED)
                    await self.task_repo.update(task.id, {"status": TaskStatus.FAILED})
                    
                    # 2. 로봇 해제 (해당 태스크에 묶여있던 로봇이 있다면)
                    if task.assigned_robot_id:
                        logger.info(f"♻️ [TaskManager] Releasing robot {task.assigned_robot_id} from failed task {task.id}")
                        await self.fleet_manager.update_robot_task_status(
                            task.assigned_robot_id, None, RobotStatus.IDLE
                        )
                        # 로봇에게 취소 명령 전송
                        robot = await self.fleet_manager.robot_repo.get_by_id(task.assigned_robot_id)
                        if robot:
                            self.fleet_manager.cancel_robot_task(robot.name)

    async def confirm_delivery(self, task_id: int, action_type: str):
        """사용자로부터 확인(적재/수령)을 받아 처리합니다."""
        

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
