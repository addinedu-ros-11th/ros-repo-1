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

class GuestCheckProcessor(BaseTaskProcessor):
    """외부인 감지 시 QR 인증 및 가이드 전환 처리기"""
    def __init__(self, fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager, visitor_repo):
        super().__init__(fleet_manager, location_repo, task_repo, ai_processing_service, connection_manager)
        self.visitor_repo = visitor_repo

    async def get_initial_actions(self, task: Task) -> List[Dict[str, Any]]:
        # 초기 동작: 빨간색 LED, 안내 문구, QR 스캔 시작
        logger.info("[GuestCheckProcessor] 외부인 감지 대응 시작.")
        return [
            {"action": "SET_LED", "params": {"color": "RED", "mode": "BLINK", "rate": 1.0}},
            {"action": "DISPLAY_TEXT", "params": {"text": "Please scan your QR code", "duration": 0}}, # 0=무한 유지
            {"action": "QR_SCAN", "params": {}, "on_success": RobotEvent.QR_SCANNED}
        ]

    async def handle_event(self, task: Task, robot_id: int, event: str, data: Optional[Dict[str, Any]] = None):
        robot = await self.fleet_manager.robot_repo.get_by_id(robot_id)
        if not robot: return

        if event == RobotEvent.QR_SCANNED:
            scanned_data = data.get("scanned_data") if data else None
            # scanned_data가 문자열(QR 내용) 리스트이거나 단일 문자열일 수 있음
            if not scanned_data:
                logger.warning("[GuestCheckProcessor] QR 데이터 없음. 재시도.")
                # 재시도 로직 (단순 반복)
                self.fleet_manager.send_action_commands(robot.name, [{"action": "QR_SCAN", "params": {}, "on_success": RobotEvent.QR_SCANNED}])
                return
            
            # scanned_data가 리스트라면 첫 번째 값 사용 (간단화)
            qr_code = scanned_data[0] if isinstance(scanned_data, list) and scanned_data else str(scanned_data)
            
            logger.info(f"[GuestCheckProcessor] QR 코드 확인 중: {qr_code}")
            visitor = await self.visitor_repo.get_by_qr_code(qr_code)
            
            if visitor and visitor.status in ["PENDING", "VERIFIED", "APPROVED"]: # APPROVED 상태도 허용 (예약 승인 시)
                logger.info(f"[GuestCheckProcessor] 방문객 확인됨: {visitor.name}")
                
                # 1. 방문객 상태 업데이트 (DB)
                await self.visitor_repo.update(visitor.visitor_id, {"status": "CHECKED_IN"}) # 입장 처리
                
                # 2. 안내 메시지 및 LED 변경
                self.fleet_manager.send_action_commands(robot.name, [
                    {"action": "SET_LED", "params": {"color": "GREEN", "mode": "SOLID"}},
                    {"action": "DISPLAY_TEXT", "params": {"text": f"Welcome {visitor.name}. Guiding...", "duration": 3}}
                ])
                
                # 3. 현재 GUEST_CHECK 태스크 완료 처리
                await self._complete_task(task, robot_id)
                
                # 4. GUIDE_GUEST 태스크 생성 및 즉시 할당
                # 목적지 결정 로직: 
                # 1순위: 방문객 정보에 명시된 destination_id
                # 2순위: 담당자(host_user_id)의 근무 위치
                # 3순위: 기본값 (large_meeting_room)
                
                target_loc_name = "large_meeting_room" # 기본값
                target_loc_id = None

                if visitor.destination_id:
                    loc = await self.location_repo.get_by_id(visitor.destination_id)
                    if loc:
                        target_loc_name = loc.name
                        target_loc_id = loc.location_id
                        logger.info(f"[GuestCheckProcessor] 방문객 목적지(ID:{target_loc_id}) 사용: {target_loc_name}")
                elif visitor.host_user_id:
                    # 담당자 정보 조회 (user_repo 접근 필요)
                    # TaskManager가 user_repo를 가지고 있으므로 fleet_manager.task_manager를 통해 접근 시도
                    if self.fleet_manager.task_manager and self.fleet_manager.task_manager.user_repo:
                        host_user = await self.fleet_manager.task_manager.user_repo.get_by_id(visitor.host_user_id)
                        if host_user and host_user.location_id:
                            loc = await self.location_repo.get_by_id(host_user.location_id)
                            if loc:
                                target_loc_name = loc.name
                                target_loc_id = loc.location_id
                                logger.info(f"[GuestCheckProcessor] 담당자({host_user.name}) 위치 사용: {target_loc_name}")

                new_task_data = {
                    "task_type": "GUIDE_GUEST",
                    "requester_id": 1, # System
                    "status": "ASSIGNED",
                    "assigned_robot_id": robot.id,
                    "visitor_id": visitor.visitor_id,
                    "destination_id": target_loc_id,
                    "target_location_name": target_loc_name,
                    "details": {"location": target_loc_name, "guest_name": visitor.name}
                }
                
                if self.fleet_manager.task_manager:
                    new_task = await self.task_repo.create(new_task_data)
                    if new_task:
                        logger.info(f"[GuestCheckProcessor] GUIDE_GUEST 태스크(ID:{new_task.id}) 자동 생성 및 전환.")
                        await self.fleet_manager.task_manager.assign_and_dispatch(robot, new_task)
            else:
                logger.warning(f"[GuestCheckProcessor] 유효하지 않은 방문객 QR: {qr_code}")
                self.fleet_manager.send_action_commands(robot.name, [
                    {"action": "DISPLAY_TEXT", "params": {"text": "Invalid QR. Try again.", "duration": 3}},
                    {"action": "QR_SCAN", "params": {}, "on_success": RobotEvent.QR_SCANNED} # 재시도
                ])
                # 무한 재시도 대신 카운트를 셀 수도 있음.

