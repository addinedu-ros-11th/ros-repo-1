import asyncio
import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List
from main_server.domains.tasks.schemas import Task, TaskType
from main_server.domains.robots.schemas import RobotStatus

logger = logging.getLogger(__name__)

class BaseTaskProcessor(ABC):
    """모든 작업 처리기의 기본 인터페이스"""
    def __init__(self, fleet_manager, location_repo, ai_processing_service=None):
        self.fleet_manager = fleet_manager
        self.location_repo = location_repo
        self.ai_processing_service = ai_processing_service

    @abstractmethod
    async def get_initial_actions(self, task: Task) -> List[Dict[str, Any]]:
        """작업 시작 시 로봇에게 내릴 첫 번째 명령들을 생성합니다."""
        pass

    @abstractmethod
    async def handle_event(self, task: Task, robot_id: int, event: str):
        """로봇으로부터 수신된 이벤트에 따라 다음 단계를 처리합니다."""
        pass

class SnackProcessor(BaseTaskProcessor):
    """간식 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        pantry_entrance = await self.location_repo.find_by_name("간식창고 입구")
        if pantry_entrance:
            return [{"action": "GOTO", "params": {"x": pantry_entrance["coordinate_x"], "y": pantry_entrance["coordinate_y"]}}]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        if event == "ARRIVED_AT_PANTRY_ENTRANCE":
            snack_name = task.details.get("item", "snack")
            loc = await self.location_repo.find_by_name(snack_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{"action": "GOTO", "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]}}])
        
        elif event == "ARRIVED_AT_SNACK_POINT":
            success = await self.ai_processing_service.verify_snack_with_stream(robot.name, task.details.get("item", ""))
            if success:
                await asyncio.sleep(3) # 로딩 대기
                dest = await self.location_repo.find_by_name(task.target_location_name)
                if dest:
                    self.fleet_manager.send_action_commands(robot.name, [{"action": "GOTO", "params": {"x": dest["coordinate_x"], "y": dest["coordinate_y"]}}])

class GuideProcessor(BaseTaskProcessor):
    """방문객 가이드 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        # 4. 특정 회의실로 가이딩 이동 명령
        dest_name = task.details.get("location", "meeting_room")
        loc = await self.location_repo.find_by_name(dest_name)
        if loc:
            return [{"action": "LEAD_GUEST", "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]}}]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        if event == "ARRIVED_AT_DESTINATION":
            # 9. LCD 5초 디스플레이 명령
            self.fleet_manager.send_action_commands(robot.name, [{"action": "DISPLAY_TEXT", "params": {"text": "Welcome!", "duration": 5}}])
            await asyncio.sleep(5)
            # 10. 복귀 명령
            base_loc = await self.location_repo.find_by_name("로비")
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{"action": "GOTO", "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]}}])

class ItemProcessor(BaseTaskProcessor):
    """P2P 물품 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        # 5. 발송자 사무실로 이동 명령
        sender_loc_name = task.details.get("source_location") or "발송처"
        loc = await self.location_repo.find_by_name(sender_loc_name)
        if loc:
            return [{"action": "GOTO", "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]}}]
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.find_by_id(robot_id)
        
        if event == "ARRIVED_AT_SENDER":
            # 6. 발송자 도착 알림 (사용자가 물건을 싣고 '로딩 완료'를 누를 때까지 대기)
            logger.info(f"로봇 {robot.name} 발송처 도착. 사용자 로딩 대기 중.")
            # 실제로는 웹소켓/푸시알림을 통해 사용자에게 버튼 클릭 유도

        elif event == "LOADING_COMPLETE":
            # 7. 배달 위치로 이동 명령
            dest_name = task.details.get("dest_location") or task.target_location_name
            loc = await self.location_repo.find_by_name(dest_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{"action": "GOTO", "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]}}])

        elif event == "ARRIVED_AT_RECEIVER":
            # 9. 수신자 도착 알림 및 수령 확인 대기
            logger.info(f"로봇 {robot.name} 수신처 도착. 수령 확인 대기 중.")

        elif event == "DELIVERY_CONFIRMED":
            # 10. 수령 완료 시 복귀 명령
            base_loc = await self.location_repo.find_by_name("로비")
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{"action": "GOTO", "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]}}])
