import math
import logging
from typing import List, Optional, Dict, Any

from main_server.domains.robots.schemas import Robot, RobotStatus
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository
from main_server.infrastructure.robot_bridge.ros_bridge import ROSBridgeCommunicator
from main_server.web.connection_manager import ConnectionManager
from main_server.services.navigation.path_planner import PathPlannerService
from main_server.services.ai_management.ai_processing import AIProcessingService

logger = logging.getLogger(__name__)

class FleetManager:
    """
    로봇 자원 관리 및 물리적 제어를 담당하는 서비스.
    최적 로봇 검색, 로봇 상태 업데이트, 직접 명령 전송을 수행합니다.
    """
    def __init__(self,
                 robot_repo: MySQLRobotRepository,
                 robot_communicator: ROSBridgeCommunicator,
                 connection_manager: ConnectionManager,
                 ai_processing_service: Optional[AIProcessingService] = None):
        self.robot_repo = robot_repo
        self.robot_communicator = robot_communicator
        self.connection_manager = connection_manager
        self.ai_processing_service = ai_processing_service
        self.path_planner = PathPlannerService('./main_server/domains/map/mymap.yaml')
        self.forbidden_zones: List[Dict] = []
        logger.info("FleetManager 초기화 완료.")

    async def enable_obstacle_relay(self, robot_id: str):
        """
        특정 로봇에 대해 AI 장애물 감지 스트림을 활성화하고, 
        결과를 로봇에게 실시간으로 전달(Relay)합니다.
        """
        if not self.ai_processing_service:
            logger.error("AIProcessingService가 설정되지 않아 장애물 릴레이를 시작할 수 없습니다.")
            return

        async def _relay_callback(data: Dict[str, Any]):
            # AI 결과에서 장애물 정보 추출 (VisionResult 구조 참조)
            # data 구조 예상: {'robot_id': '...', 'result': {'object_detection': {...}}} 
            # 또는 oneof 필드에 따라 다름.
            
            # 여기서 필요한 데이터만 필터링하거나 가공하여 전송
            # 로봇 쪽에서는 이 데이터를 받아 Local Costmap에 반영하거나 회피 기동 수행
            try:
                # 단순 릴레이 (전체 데이터 전송)
                # 만약 포맷 변환이 필요하면 여기서 수행
                self.robot_communicator.publish_obstacle_info(robot_id, data)
            except Exception as e:
                logger.error(f"[{robot_id}] 장애물 정보 릴레이 실패: {e}")

        logger.info(f"[{robot_id}] 장애물 정보 릴레이 활성화 요청")
        await self.ai_processing_service.start_obstacle_detection(robot_id, _relay_callback)

    async def disable_obstacle_relay(self, robot_id: str):
        """특정 로봇의 장애물 감지 및 릴레이를 중단합니다."""
        if not self.ai_processing_service:
            return
            
        logger.info(f"[{robot_id}] 장애물 정보 릴레이 중단 요청")
        await self.ai_processing_service.stop_obstacle_detection(robot_id)

    async def enable_employee_relay(self, robot_id: str):
        """
        특정 로봇에 대해 AI 직원/얼굴 인식 스트림을 활성화하고,
        결과를 로봇에게 실시간으로 전달(Relay)합니다.
        """
        if not self.ai_processing_service:
            logger.error("AIProcessingService가 설정되지 않아 직원 인식 릴레이를 시작할 수 없습니다.")
            return

        async def _relay_callback(data: Dict[str, Any]):
            try:
                # 직원 인식 결과 릴레이
                self.robot_communicator.publish_employee_result(robot_id, data)
            except Exception as e:
                logger.error(f"[{robot_id}] 직원 인식 정보 릴레이 실패: {e}")

        logger.info(f"[{robot_id}] 직원 인식 릴레이 활성화 요청")
        await self.ai_processing_service.start_employee_verification(robot_id, _relay_callback)

    async def disable_employee_relay(self, robot_id: str):
        """특정 로봇의 직원 인식 및 릴레이를 중단합니다."""
        if not self.ai_processing_service:
            return

        logger.info(f"[{robot_id}] 직원 인식 릴레이 중단 요청")
        await self.ai_processing_service.stop_employee_verification(robot_id)

    def update_forbidden_zones(self, zones: List[Dict]):
        """금지 구역 목록을 저장하고 경로 계획기에 반영합니다."""
        self.forbidden_zones = zones
        self.path_planner.update_forbidden_zones(zones)
        logger.info(f"FleetManager: 금지 구역 {len(zones)}개 업데이트 완료.")

    def get_forbidden_zones(self) -> List[Dict]:
        """현재 설정된 금지 구역 목록을 반환합니다."""
        return self.forbidden_zones

    async def find_optimal_robot(self, target_pose: tuple) -> Optional[Robot]:
        """목적지에 가장 적합한 로봇을 검색합니다."""
        idle_robots = await self.robot_repo.find_by_status(RobotStatus.IDLE)
        # 배터리 충분하고 위치 정보가 유효한 로봇만 필터링
        available_robots = [
            r for r in idle_robots 
            if r.battery_level > 20 and r.pose_x is not None and r.pose_y is not None
        ]
        
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
            # 로봇 상태에 따른 AI 추론 모드(장애물/직원) 자동 제어
            
            # 1. 장애물 감지 (이동 중일 때 활성화)
            if status in [RobotStatus.MOVING, RobotStatus.GUIDING, RobotStatus.ASSIGNED]:
                await self.enable_obstacle_relay(updated_robot.name)
            else:
                # IDLE, WAITING, CHARGING, ERROR, OFFLINE 등
                await self.disable_obstacle_relay(updated_robot.name)
            
            # 2. 직원 인식 (베이스/충전소 대기 중일 때 활성화)
            # WAITING은 작업 중 대기이므로 제외, IDLE/CHARGING일 때만 활성화
            if status in [RobotStatus.IDLE, RobotStatus.CHARGING]:
                await self.enable_employee_relay(updated_robot.name)
            else:
                # MOVING, GUIDING, ASSIGNED, WAITING, ERROR, OFFLINE 등
                await self.disable_employee_relay(updated_robot.name)

            await self.connection_manager.broadcast(updated_robot.model_dump_json())
        return updated_robot

    async def get_all_robot_status(self) -> List[Robot]:
        return await self.robot_repo.get_all()
