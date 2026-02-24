import asyncio
import logging
from typing import Dict, List, Optional, Set
from collections import deque

from main_server.domains.robots.schemas import Robot
from main_server.domains.tasks.schemas import RobotActionType

logger = logging.getLogger(__name__)

class MutexZone:
    """Mutex 구역 상태 정보를 관리하는 데이터 모델"""
    def __init__(self, zone_id: int, name: str):
        self.zone_id = zone_id
        self.name = name
        self.current_robot_id: Optional[int] = None
        # 대기 큐 (robot_id, priority)
        self.waiting_queue = deque() 

class MutexZoneManager:
    """
    교차로, 좁은 통로 등 1대의 로봇만 진입 가능한 Mutex Zone을 관리하는 서비스.
    """
    def __init__(self, fleet_manager):
        self.fleet_manager = fleet_manager
        # zone_id -> MutexZone 객체
        self.zones: Dict[int, MutexZone] = {}
        # 초기 구역 데이터 로드 (실제로는 DB나 설정에서 가져옴)
        self._initialize_zones()

    def _initialize_zones(self):
        # 예시 구역 데이터 (실무에서는 DB Table: Map_Zones 에서 type='MUTEX'인 것을 로드)
        sample_zones = [
            (1, "Narrow Corridor A"),
            (2, "Intersection B")
        ]
        for zid, name in sample_zones:
            self.zones[zid] = MutexZone(zid, name)
        logger.info(f"MutexZoneManager initialized with {len(self.zones)} zones.")

    async def request_entry(self, robot_id: int, zone_id: int, priority: int = 3):
        """
        로봇으로부터 진입 요청을 처리합니다.
        이미 점유 중이면 대기(PAUSE) 명령을 내리고 큐에 추가합니다.
        """
        zone = self.zones.get(zone_id)
        if not zone:
            logger.error(f"Zone {zone_id} not found.")
            return

        robot_list = await self.fleet_manager.get_all_robot_status()
        robot = next((r for r in robot_list if r.id == robot_id), None)
        if not robot:
            logger.error(f"Robot {robot_id} not found.")
            return

        logger.info(f"Robot '{robot.name}' requests entry to Mutex Zone '{zone.name}' (Priority: {priority})")

        # 1. 구역이 비어 있고 대기 중인 다른 로봇이 없는 경우 즉시 승인
        if zone.current_robot_id is None and len(zone.waiting_queue) == 0:
            await self._grant_entry(robot, zone)
        else:
            # 2. 점유 중이거나 대기 중인 로봇이 있으면 대기 큐에 추가하고 PAUSE 명령
            # 간단하게 FIFO 큐 사용 (우선순위 고려 로직 추가 가능)
            if robot_id not in [r_id for r_id, _ in zone.waiting_queue]:
                zone.waiting_queue.append((robot_id, priority))
            
            logger.info(f"Zone '{zone.name}' occupied or has queue. Robot '{robot.name}' must wait.")
            self.fleet_manager.send_action_commands(robot.name, [{"action": RobotActionType.PAUSE, "params": {}}])

    async def release_zone(self, robot_id: int, zone_id: int):
        """
        로봇이 구역을 빠져나갔을 때 점유를 해제하고 다음 대기 로봇을 승인합니다.
        """
        zone = self.zones.get(zone_id)
        if not zone: return

        if zone.current_robot_id == robot_id:
            logger.info(f"Robot {robot_id} released Mutex Zone '{zone.name}'.")
            zone.current_robot_id = None
            
            # 다음 대기 로봇 처리
            await self._process_next_in_queue(zone)
        else:
            # 혹시 큐에 들어있던 로봇이 나간 경우(취소 등) 큐에서 제거
            zone.waiting_queue = deque([(rid, p) for rid, p in zone.waiting_queue if rid != robot_id])

    async def _process_next_in_queue(self, zone: MutexZone):
        """대기 큐에서 다음 로봇을 꺼내 진입을 승인합니다."""
        if not zone.waiting_queue:
            return

        # 우선순위가 가장 높은 로봇 선택 (단순 구현은 FIFO)
        next_robot_id, _ = zone.waiting_queue.popleft()
        
        robot_list = await self.fleet_manager.get_all_robot_status()
        next_robot = next((r for r in robot_list if r.id == next_robot_id), None)
        
        if next_robot:
            await self._grant_entry(next_robot, zone)
        else:
            # 로봇 정보를 찾을 수 없으면 다음 로봇 시도
            await self._process_next_in_queue(zone)

    async def _grant_entry(self, robot: Robot, zone: MutexZone):
        """로봇에게 진입을 허용하고 RESUME 명령을 내립니다."""
        zone.current_robot_id = robot.id
        logger.info(f"Granting entry to Robot '{robot.name}' for Zone '{zone.name}'.")
        self.fleet_manager.send_action_commands(robot.name, [{"action": RobotActionType.RESUME, "params": {}}])
