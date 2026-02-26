import asyncio
import logging
import json
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional
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
    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        """로봇으로부터 수신된 이벤트에 따라 다음 단계를 처리합니다."""
        pass

    async def _complete_task(self, task: Task, robot_id: int):
        """작업을 완료 상태로 변경하고 로봇을 해제합니다."""
        await self.task_repo.update(task.id, {"status": TaskStatus.COMPLETED})
        await self.fleet_manager.update_robot_task_status(robot_id, None, RobotStatus.IDLE)
        logger.info(f"태스크 {task.id} 완료 및 로봇 {robot_id} 배차 해제.")
        await self.broadcast_task_update(f"작업(ID:{task.id})이 완료되었습니다.")

    async def broadcast_task_update(self, message: str):
        """작업 진행 상황을 클라이언트에게 알립니다."""
        try:
            payload = {
                "event": "task_update",
                "data": {"message": message}
            }
            await self.connection_manager.broadcast(json.dumps(payload))
        except Exception as e:
            logger.error(f"진행 상황 브로드캐스트 실패: {e}")

class SnackProcessor(BaseTaskProcessor):
    """간식 배달 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        # AI 결과와 DB 데이터를 기반으로 'snack_entrance' 사용
        pantry_entrance = await self.location_repo.find_by_name("snack_entrance")
        if pantry_entrance:
            actions = [{
                "action": "GOTO", 
                "params": {
                    "x": pantry_entrance["coordinate_x"], 
                    "y": pantry_entrance["coordinate_y"],
                    "theta": pantry_entrance.get("theta", 0.0)
                },
                "on_success": RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE
            }]
            logger.info(f"[SnackProcessor] 초기 액션 생성: {actions}")
            return actions
        
        logger.error("[SnackProcessor] 'snack_entrance' 위치를 찾을 수 없습니다.")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: 
            logger.error(f"[SnackProcessor] 로봇 ID {robot_id}를 찾을 수 없습니다.")
            return

        logger.info(f"[SnackProcessor] 이벤트 수신: {event} (Robot: {robot.name}, Task ID: {task.id})")
        if data:
            logger.info(f"[SnackProcessor] 이벤트 데이터: {data}")

        if event == RobotEvent.ARRIVED_AT_PANTRY_ENTRANCE:
            await self.broadcast_task_update("로봇이 간식 창고 입구에 도착했습니다.")
            items = task.details.get("items", [])
            item_name = items[0].get("item_name") if items else "snack"
            
            logger.info(f"[SnackProcessor] 탕비실 입구 도착. 아이템 '{item_name}' 위치 탐색 시작.")
            loc = await self.location_repo.find_by_name(item_name)
            
            if loc:
                logger.info(f"[SnackProcessor] 아이템 위치 확인: {loc['name']} ({loc['coordinate_x']}, {loc['coordinate_y']})")
                commands = [{
                    "action": "GOTO", 
                    "params": {
                        "x": loc["coordinate_x"], 
                        "y": loc["coordinate_y"],
                        "theta": loc.get("theta", 0.0)
                    },
                    "on_success": RobotEvent.ARRIVED_AT_SNACK_POINT
                }]
                self.fleet_manager.send_action_commands(robot.name, commands)
                logger.info(f"[SnackProcessor] GOTO 명령 전송 완료: {commands}")
            else:
                 logger.warning(f"[SnackProcessor] '{item_name}'의 위치를 찾을 수 없어 픽업 단계로 바로 건너뜁니다.")
                 await self.handle_event(task, robot_id, RobotEvent.ARRIVED_AT_SNACK_POINT)
        
        elif event == RobotEvent.ARRIVED_AT_SNACK_POINT:
            logger.info(f"[SnackProcessor] 간식 포인트 도착. QR 스캔 요청 전송.")
            await self.broadcast_task_update("간식 진열대에 도착했습니다. QR 코드를 스캔합니다.")
            commands = [{
                "action": "QR_SCAN", 
                "params": {}, 
                "on_success": RobotEvent.QR_SCANNED 
            }]
            self.fleet_manager.send_action_commands(robot.name, commands)
            logger.info(f"[SnackProcessor] QR_SCAN 명령 전송 완료.")

        elif event == RobotEvent.QR_SCANNED:
            items = task.details.get("items", [])
            target_item_name = items[0].get("item_name") if items else "snack"
            scanned_data = data.get("scanned_data") if data else None
            
            logger.info(f"[SnackProcessor] QR 스캔 결과 수신: {scanned_data} (목표: {target_item_name})")
            
            if not scanned_data:
                logger.warning("[SnackProcessor] QR 데이터가 비어 있습니다. 재시도 명령을 내립니다.")
                await self.broadcast_task_update("QR 인식이 되지 않았습니다. 재시도합니다.")
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "QR_SCAN", 
                    "params": {}, 
                    "on_success": RobotEvent.QR_SCANNED 
                }])
                return

            if target_item_name in scanned_data:
                logger.info("[SnackProcessor] QR 검증 성공. 목적지 이동 시작.")
                await self.broadcast_task_update(f"물품({target_item_name}) 확인 완료. 배달을 시작합니다.")
                await asyncio.sleep(1) 
                
                dest_name = task.target_location_name
                dest = await self.location_repo.find_by_name(dest_name)
                if dest:
                    logger.info(f"[SnackProcessor] 목적지 좌표 확인: {dest_name} ({dest['coordinate_x']}, {dest['coordinate_y']})")
                    commands = [{
                        "action": "GOTO", 
                        "params": {
                            "x": dest["coordinate_x"], 
                            "y": dest["coordinate_y"],
                            "theta": dest.get("theta", 0.0)
                        },
                        "on_success": RobotEvent.ARRIVED_AT_DESTINATION
                    }]
                    self.fleet_manager.send_action_commands(robot.name, commands)
                    logger.info(f"[SnackProcessor] GOTO 명령(목적지) 전송 완료.")
                else:
                    logger.error(f"[SnackProcessor] 목적지 '{dest_name}'를 찾을 수 없습니다.")
            else:
                logger.warning(f"[SnackProcessor] QR 검증 실패. 불일치: {scanned_data}")
                await self.broadcast_task_update("잘못된 물품이 감지되었습니다.")
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "DISPLAY_TEXT", 
                    "params": {"text": "Wrong Item!", "duration": 3}
                }])

        elif event == RobotEvent.ARRIVED_AT_DESTINATION:
            logger.info(f"[SnackProcessor] 목적지 도착 완료. 수령 확인 요청 대기.")
            await self.broadcast_task_update("목적지에 도착했습니다. 간식을 수령해주세요.")
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
            logger.info(f"[SnackProcessor] 수령 확인 이벤트 수신. 태스크 종료 절차 진입.")
            await self._complete_task(task, robot_id)

class GuideProcessor(BaseTaskProcessor):
    """방문객 가이드 시나리오 처리기"""
    async def get_initial_actions(self, task: Task):
        dest_name = task.details.get("location", "meeting_room")
        loc = await self.location_repo.find_by_name(dest_name)
        if loc:
            actions = [{
                "action": "LEAD_GUEST", 
                "params": {
                    "x": loc["coordinate_x"], 
                    "y": loc["coordinate_y"],
                    "theta": loc.get("theta", 0.0)
                },
                "on_success": RobotEvent.ARRIVED_AT_DESTINATION
            }]
            logger.info(f"[GuideProcessor] 초기 액션 생성: {actions}")
            return actions
        logger.error(f"[GuideProcessor] 목적지 '{dest_name}'를 찾을 수 없습니다.")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return

        if event == RobotEvent.ARRIVED_AT_DESTINATION:
            await self.broadcast_task_update("안내 목적지에 도착했습니다.")
            self.fleet_manager.send_action_commands(robot.name, [{"action": "DISPLAY_TEXT", "params": {"text": "Welcome!", "duration": 5}}])
            await asyncio.sleep(5)
            # 복귀 위치 (예: 대기 구역)
            base_loc = await self.location_repo.find_by_name("waiting_area") 
            
            if base_loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {
                        "x": base_loc["coordinate_x"], 
                        "y": base_loc["coordinate_y"],
                        "theta": base_loc.get("theta", 0.0)
                    },
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
            logger.warning("[ItemProcessor] 출발지 정보가 없어 기본 위치(office_1)를 탐색합니다.")
            sender_loc_name = "office_1" # Fallback

        loc = await self.location_repo.find_by_name(sender_loc_name)
        if loc:
            actions = [{
                "action": "GOTO", 
                "params": {
                    "x": loc["coordinate_x"], 
                    "y": loc["coordinate_y"],
                    "theta": loc.get("theta", 0.0)
                },
                "on_success": RobotEvent.ARRIVED_AT_SENDER
            }]
            logger.info(f"[ItemProcessor] 초기 액션 생성: {actions}")
            return actions
        
        logger.error(f"[ItemProcessor] 출발지 '{sender_loc_name}'를 찾을 수 없습니다.")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return
        
        if event == RobotEvent.ARRIVED_AT_SENDER:
            logger.info(f"로봇 {robot.name} 발송처 도착. 사용자 로딩 대기 중.")
            await self.broadcast_task_update("발송 위치에 도착했습니다. 물품을 적재해주세요.")
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
            await self.broadcast_task_update("적재 확인 완료. 수신처로 이동합니다.")
            # 수신자 위치로 이동
            dest_name = task.target_location_name # ScenarioDataHandler가 설정함
            loc = await self.location_repo.find_by_name(dest_name)
            if loc:
                self.fleet_manager.send_action_commands(robot.name, [{
                    "action": "GOTO", 
                    "params": {
                        "x": loc["coordinate_x"], 
                        "y": loc["coordinate_y"],
                        "theta": loc.get("theta", 0.0)
                    },
                    "on_success": RobotEvent.ARRIVED_AT_RECEIVER
                }])
            else:
                logger.error(f"수신처 '{dest_name}'를 찾을 수 없습니다.")

        elif event == RobotEvent.ARRIVED_AT_RECEIVER:
            logger.info(f"로봇 {robot.name} 수신처 도착. 수령 확인 대기 중.")
            await self.broadcast_task_update("수신 위치에 도착했습니다. 물품을 수령해주세요.")
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

        elif event == RobotEvent.ARRIVED_AT_BASE:
            await self._complete_task(task, robot_id)

class ManualMoveProcessor(BaseTaskProcessor):
    """수동 좌표 이동 시나리오 처리기 (테스트 및 디버깅용)"""
    async def get_initial_actions(self, task: Task):
        # task.details에 저장된 x, y, theta 좌표 사용
        x = task.details.get("x")
        y = task.details.get("y")
        theta = task.details.get("theta", 0.0) # 기본값 0.0
        
        if x is not None and y is not None:
            actions = [{
                "action": "GOTO", 
                "params": {"x": x, "y": y, "theta": theta},
                "on_success": RobotEvent.ARRIVED_AT_DESTINATION
            }]
            logger.info(f"[ManualMoveProcessor] 초기 액션 생성: {actions}")
            return actions
        
        logger.error(f"[ManualMoveProcessor] 수동 이동 좌표가 누락되었습니다: {task.details}")
        return []

    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        if event == RobotEvent.ARRIVED_AT_DESTINATION:
            logger.info(f"로봇 {robot_id} 수동 이동 목적지 도착.")
            await self.broadcast_task_update("수동 이동 목적지에 도착했습니다.")
            await self._complete_task(task, robot_id)
