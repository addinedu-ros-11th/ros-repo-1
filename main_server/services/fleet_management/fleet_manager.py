import math
import logging
from typing import List, Optional, Dict, Any

from main_server.domains.robots.schemas import Robot, RobotStatus
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository
from main_server.infrastructure.robot_bridge.ros_bridge import ROSBridgeCommunicator
from main_server.web.connection_manager import ConnectionManager
from main_server.services.navigation.path_planner import PathPlannerService

logger = logging.getLogger(__name__)

class FleetManager:
    """
    로봇 자원 관리 및 물리적 제어를 담당하는 서비스.
    최적 로봇 검색, 로봇 상태 업데이트, 직접 명령 전송을 수행합니다.
    """
    def __init__(self,
                 robot_repo: MySQLRobotRepository,
                 robot_communicator: ROSBridgeCommunicator,
                 connection_manager: ConnectionManager):
        self.robot_repo = robot_repo
        self.robot_communicator = robot_communicator
        self.connection_manager = connection_manager
        self.path_planner = PathPlannerService('./main_server/domains/map/mymap.yaml')
        logger.info("FleetManager 초기화 완료.")

    async def find_optimal_robot(self, target_pose: tuple) -> Optional[Robot]:
        """목적지에 가장 적합한 로봇을 검색합니다."""
        idle_robots = await self.robot_repo.find_by_status(RobotStatus.IDLE)
        available_robots = [r for r in idle_robots if r.battery_level > 20]
        
        if not available_robots: return None
        robot_distances = []

        for robot in available_robots:
            # 1. 각 로봇에서 목적지까지의 실제 Global Path를 계산
            path = await self.path_planner.plan_global_path(
                robot, target_pose[0], target_pose[1]
            )
            
            if path:
                # 2. 경로가 존재하면 경로의 노드 개수(또는 실제 거리 합산)를 저장
                # path는 [{'x':...}, {'y':...}] 형태의 리스트이므로 len(path)가 곧 비용입니다.
                robot_distances.append((robot, len(path)))
            else:
                # 경로가 없는 로봇(도달 불가능)은 제외
                continue

        if not robot_distances:
            logger.warning("목적지에 도달 가능한 로봇이 없습니다.")
            return None

        # 3. 경로 길이가 가장 짧은 로봇 반환
        best_robot = min(robot_distances, key=lambda x: x[1])[0]

        return best_robot

    async def update_robot_task_status(self, robot_id: int, task_id: Optional[int], status: RobotStatus) -> Optional[Robot]:
        """로봇의 작업 할당 상태를 DB에 반영하고 알립니다."""
        update_data = {"status": status, "current_task_id": task_id}
        updated_robot = await self.robot_repo.update(robot_id, update_data)
        
        if updated_robot:
            await self.connection_manager.broadcast(updated_robot.model_dump_json())
        return updated_robot

    def send_action_commands(self, robot_name: str, actions: List[Dict[str, Any]]):
        """실제 로봇에게 액션 시퀀스를 전송합니다."""
        self.robot_communicator.send_action_sequence(robot_name, actions)
        logger.info(f"로봇 '{robot_name}'에게 {len(actions)}개의 액션 전송 완료.")

    async def update_robot_status(self, robot_id: int, status: RobotStatus, location: tuple, battery: float) -> Optional[Robot]:
        """로봇으로부터 수신된 텔레메트리 정보를 DB에 갱신합니다."""
        update_data = {
            "status": status,
            "pose_x": location[0],
            "pose_y": location[1],
            "battery_level": battery
        }
        updated_robot = await self.robot_repo.update(robot_id, update_data)
        if updated_robot:
            await self.connection_manager.broadcast(updated_robot.model_dump_json())
        return updated_robot

    async def get_all_robot_status(self) -> List[Robot]:
        return await self.robot_repo.get_all()
