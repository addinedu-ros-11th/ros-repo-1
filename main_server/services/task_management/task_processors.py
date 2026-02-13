import asyncio
import logging
import json
from abc import ABC, abstractmethod
from typing import Any, Dict, List
from main_server.domains.tasks.schemas import Task, TaskType
from main_server.domains.robots.schemas import RobotStatus
from common.robot_task_events import RobotEvent

logger = logging.getLogger(__name__)

class BaseTaskProcessor(ABC):
    """모든 작업 처리기의 기본 인터페이스"""
    def __init__(self, fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager):
        self.fleet_manager = fleet_manager
        self.location_repo = location_repo
        self.task_repo = task_repo
        self.ai_processing_service = ai_processing_service
        self.connection_manager = connection_manager

    @abstractmethod
    async def get_initial_actions(self, task: Task) -> List[Dict[str, Any]]:
        """작업 시작 시 로봇에게 내릴 첫 번째 명령들을 생성합니다."""
        pass

    @abstractmethod
    async def handle_event(self, task: Task, robot_id: int, event: str):
        """로봇으로부터 수신된 이벤트에 따라 다음 단계를 처리합니다."""
        pass

    async def _complete_task(self, task: Task, robot_id: int):
        """작업을 완료 상태로 변경하고 로봇을 해제합니다."""
        from main_server.domains.tasks.schemas import TaskStatus
        from main_server.domains.robots.schemas import RobotStatus
        
        await self.task_repo.update(task.id, {"status": TaskStatus.COMPLETED})
        await self.fleet_manager.update_robot_task_status(robot_id, None, RobotStatus.IDLE)
        logger.info(f"태스크 {task.id} 완료 및 로봇 {robot_id} 배차 해제.")

class SnackProcessor(BaseTaskProcessor):
    """간식 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        pantry_entrance = await self.location_repo.find_by_name("간식창고 입구")
        if pantry_entrance:
            return [{
                "action": "GOTO", 
                "params": {"x": pantry_entrance["coordinate_x"], "y": pantry_entrance["coordinate_y"]},
                "on_success": RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE
            }]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        if event == RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE:
            snack_name = task.details.get("item", "snack")
            loc = await self.location_repo.find_by_name(snack_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_SNACK_POINT
                }])
        
        elif event == RobotEvent.ARRIVED_AT_SNACK_POINT:
            success = await self.ai_processing_service.verify_snack_with_stream(robot.name, task.details.get("item", ""))
            if success:
                await asyncio.sleep(3) # 로딩 대기
                dest = await self.location_repo.find_by_name(task.target_location_name)
                if dest:
                    self.fleet_manager.send_action_commands(robot.name, [{
                        "action": "GOTO", 
                        "params": {"x": dest["coordinate_x"], "y": dest["coordinate_y"]},
                        "on_success": RobotEvent.ARRIVED_AT_DESTINATION
                    }])

        elif event == RobotEvent.ARRIVED_AT_DESTINATION:
            # 도착 알림 및 수령 확인 요청 전송
            logger.info(f"로봇 {robot.name} 목적지 도착. 사용자 수령 확인 대기 중.")
            message = {
                "event": "user_action_required",
                "data": {
                    "task_id": task.id,
                    "robot_id": robot_id,
                    "action_type": "CONFIRM_SNACK_RECEIPT",
                    "message": "간식이 도착했습니다. 수령 확인 버튼을 눌러주세요."
                }
            }
            await self.connection_manager.broadcast(json.dumps(message))

        elif event == RobotEvent.DELIVERY_CONFIRMED:
            await self._complete_task(task, robot_id)

class GuideProcessor(BaseTaskProcessor):
    """방문객 가이드 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        dest_name = task.details.get("location", "meeting_room")
        loc = await self.location_repo.find_by_name(dest_name)
        if loc:
            return [{
                "action": "LEAD_GUEST", 
                "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                "on_success": RobotEvent.ARRIVED_AT_DESTINATION
            }]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        if event == RobotEvent.ARRIVED_AT_DESTINATION:
            self.fleet_manager.send_action_commands(robot.name, [{"action": "DISPLAY_TEXT", "params": {"text": "Welcome!", "duration": 5}}])
            await asyncio.sleep(5)
            base_loc = await self.location_repo.find_by_name("로비")
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_BASE
                }])

        elif event == RobotEvent.ARRIVED_AT_BASE:
            await self._complete_task(task, robot_id)

class ItemProcessor(BaseTaskProcessor):
    """P2P 물품 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        sender_loc_name = task.details.get("source_location") or "발송처"
        loc = await self.location_repo.find_by_name(sender_loc_name)
        if loc:
            return [{
                "action": "GOTO", 
                "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                "on_success": RobotEvent.ARRIVED_AT_SENDER
            }]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        
        if event == RobotEvent.ARRIVED_AT_SENDER:
            logger.info(f"로봇 {robot.name} 발송처 도착. 사용자 로딩 대기 중.")
            message = {
                "event": "user_action_required",
                "data": {
                    "task_id": task.id,
                    "robot_id": robot_id,
                    "action_type": "CONFIRM_LOADING",
                    "message": "로봇이 도착했습니다. 물품을 적재하고 확인 버튼을 눌러주세요."
                }
            }
            await self.connection_manager.broadcast(json.dumps(message))

        elif event == RobotEvent.LOADING_COMPLETE:
            dest_name = task.details.get("dest_location") or task.target_location_name
            loc = await self.location_repo.find_by_name(dest_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_RECEIVER
                }])

        elif event == RobotEvent.ARRIVED_AT_RECEIVER:
            logger.info(f"로봇 {robot.name} 수신처 도착. 수령 확인 대기 중.")
            message = {
                "event": "user_action_required",
                "data": {
                    "task_id": task.id,
                    "robot_id": robot_id,
                    "action_type": "CONFIRM_ITEM_RECEIPT",
                    "message": "물품이 도착했습니다. 수령 확인 버튼을 눌러주세요."
                }
            }
            await self.connection_manager.broadcast(json.dumps(message))

        elif event == RobotEvent.DELIVERY_CONFIRMED:
            base_loc = await self.location_repo.find_by_name("로비")
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_BASE
                }])

        elif event == RobotEvent.ARRIVED_AT_BASE:
            await self._complete_task(task, robot_id)
