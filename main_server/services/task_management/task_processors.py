import asyncio
import logging
import json
from abc import ABC, abstractmethod
from typing import Any, Dict, List
from main_server.domains.tasks.schemas import Task, TaskType, TaskStatus
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
        await self.task_repo.update(task.id, {"status": TaskStatus.COMPLETED})
        await self.fleet_manager.update_robot_task_status(robot_id, None, RobotStatus.IDLE)
        logger.info(f"태스크 {task.id} 완료 및 로봇 {robot_id} 배차 해제.")

class SnackProcessor(BaseTaskProcessor):
    """간식 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        # AI 결과와 DB 데이터를 기반으로 'snack_entrance' 사용
        pantry_entrance = await self.location_repo.find_by_name("snack_entrance")
        if pantry_entrance:
            return [{
                "action": "GOTO", 
                "params": {"x": pantry_entrance["coordinate_x"], "y": pantry_entrance["coordinate_y"]},
                "on_success": RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE
            }]
        
        logger.error("'snack_entrance' 위치를 찾을 수 없습니다.")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return

        if event == RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE:
            # AI 결과는 items 리스트로 제공됨 (ScenarioDataHandler 처리 결과)
            items = task.details.get("items", [])
            item_name = "snack" # 기본값

            if items:
                # 첫 번째 아이템의 위치로 이동 (단순화)
                item_name = items[0].get("item_name")
                
            # 창고 내 세부 위치 조회 (없으면 창고 입구에서 대기)
            loc = await self.location_repo.find_by_name(item_name)
            
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_SNACK_POINT
                }])
            else:
                 # 위치 못 찾으면 바로 픽업 단계로 간주 (혹은 에러 처리)
                 logger.warning(f"아이템 '{item_name}'의 위치를 찾을 수 없어 픽업 절차를 진행합니다.")
                 await self.handle_event(task, robot_id, RobotEvent.ARRIVED_AT_SNACK_POINT)
        
        elif event == RobotEvent.ARRIVED_AT_SNACK_POINT:
            items = task.details.get("items", [])
            item_name = items[0].get("item_name") if items else "snack"
            
            # 카메라 스트림을 통한 아이템 검증
            success = await self.ai_processing_service.verify_snack_with_stream(robot.name, item_name)
            
            if success: # 검증 성공 시
                await asyncio.sleep(3) # 로딩 대기 (가상)
                
                # 최종 목적지 (요청자 위치)로 이동
                dest_name = task.target_location_name
                dest = await self.location_repo.find_by_name(dest_name)
                if dest:
                    self.fleet_manager.send_action_commands(robot.name, [{
                        "action": "GOTO", 
                        "params": {"x": dest["coordinate_x"], "y": dest["coordinate_y"]},
                        "on_success": RobotEvent.ARRIVED_AT_DESTINATION
                    }])
                else:
                    logger.error(f"목적지 '{dest_name}'를 찾을 수 없습니다.")

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
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return

        if event == RobotEvent.ARRIVED_AT_DESTINATION:
            self.fleet_manager.send_action_commands(robot.name, [{"action": "DISPLAY_TEXT", "params": {"text": "Welcome!", "duration": 5}}])
            await asyncio.sleep(5)
            # 복귀 위치 (예: 대기 구역)
            base_loc = await self.location_repo.find_by_name("waiting_area") 
            
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_BASE
                }])
            else:
                await self._complete_task(task, robot_id)

        elif event == RobotEvent.ARRIVED_AT_BASE:
            await self._complete_task(task, robot_id)

class ItemProcessor(BaseTaskProcessor):
    """P2P 물품 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        # ScenarioDataHandler에서 주입한 source_location 사용
        sender_loc_name = task.details.get("source_location")
        
        if not sender_loc_name:
            logger.warning("출발지 정보가 없어 기본 위치(office_1)를 탐색합니다.")
            sender_loc_name = "office_1" # Fallback

        loc = await self.location_repo.find_by_name(sender_loc_name)
        if loc:
            return [{
                "action": "GOTO", 
                "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                "on_success": RobotEvent.ARRIVED_AT_SENDER
            }]
        
        logger.error(f"출발지 '{sender_loc_name}'를 찾을 수 없습니다.")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return
        
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
            # 수신자 위치로 이동
            dest_name = task.target_location_name # ScenarioDataHandler가 설정함
            loc = await self.location_repo.find_by_name(dest_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": loc["coordinate_x"], "y": loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_RECEIVER
                }])
            else:
                logger.error(f"수신처 '{dest_name}'를 찾을 수 없습니다.")

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
            # 복귀 (대기 구역)
            base_loc = await self.location_repo.find_by_name("waiting_area")
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {"x": base_loc["coordinate_x"], "y": base_loc["coordinate_y"]},
                    "on_success": RobotEvent.ARRIVED_AT_BASE
                }])
            else:
                await self._complete_task(task, robot_id)

        elif event == RobotEvent.ARRIVED_AT_BASE:
            await self._complete_task(task, robot_id)
