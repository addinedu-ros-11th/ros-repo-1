import math
import logging
from typing import List, Optional, Dict, Any, Union

from main_server.config import config
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
        self.path_planner = PathPlannerService(config.MAP_YAML_PATH)
        self.forbidden_zones: List[Dict] = []
        logger.info(f"FleetManager 초기화 완료. (Map: {config.MAP_YAML_PATH})")

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

        logger.debug(f"[{robot_id}] 장애물 정보 릴레이 활성화 요청")
        await self.ai_processing_service.start_obstacle_detection(robot_id, _relay_callback)

    async def disable_obstacle_relay(self, robot_id: str):
        """특정 로봇의 장애물 감지 및 릴레이를 중단합니다."""
        if not self.ai_processing_service:
            return
            
        logger.debug(f"[{robot_id}] 장애물 정보 릴레이 중단 요청")
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

        logger.debug(f"[{robot_id}] 직원 인식 릴레이 활성화 요청")
        await self.ai_processing_service.start_employee_verification(robot_id, _relay_callback)

    async def disable_employee_relay(self, robot_id: str):
        """특정 로봇의 직원 인식 및 릴레이를 중단합니다."""
        if not self.ai_processing_service:
            return

        logger.debug(f"[{robot_id}] 직원 인식 릴레이 중단 요청")
        await self.ai_processing_service.stop_employee_verification(robot_id)

    async def update_forbidden_zones(self, zones: List[Dict]):
        """
        금지 구역 목록을 저장하고 경로 계획기 및 로봇들에게 반영합니다.
        (Service Call 방식)
        """
        self.forbidden_zones = zones
        
        # 1. 서버 측 PathPlanner 업데이트 (배차용)
        self.path_planner.update_forbidden_zones(zones)
        
        # 2. 모든 로봇에게 금지 구역 설정 전파 (내비게이션용)
        try:
            robots = await self.robot_repo.get_all()
            for robot in robots:
                if robot.status != RobotStatus.OFFLINE:
                    # 서비스 호출 (동기 방식이지만 짧은 타임아웃)
                    self.robot_communicator.set_forbidden_zones(robot.name, zones)
            
            logger.info(f"FleetManager: 금지 구역 {len(zones)}개 업데이트 및 {len(robots)}대 로봇 전파 완료.")
        except Exception as e:
            logger.error(f"금지 구역 전파 중 오류: {e}")

    def get_forbidden_zones(self) -> List[Dict]:
        """현재 설정된 금지 구역 목록을 반환합니다."""
        return self.forbidden_zones

    async def find_optimal_robot(self, target_pose: tuple) -> Optional[Robot]:
        """목적지에 가장 적합한 로봇을 검색합니다."""
        idle_robots = await self.robot_repo.find_by_status(RobotStatus.IDLE)
        if not idle_robots: 
            logger.warning("현재 IDLE 상태인 로봇이 없습니다.")
            return None
        # 배터리 충분하고 위치 정보가 유효한 로봇만 필터링
        available_robots = [
            r for r in idle_robots 
            if r.battery_level > 20 and r.pose_x is not None and r.pose_y is not None
        ]
        
        if not available_robots: 
            logger.warning("배터리 잔량이 충분한 상태의 로봇이 없습니다.")
            return None
        robot_distances = []

        for robot in available_robots:
            # 1. 각 로봇에서 목적지까지의 실제 Global Path를 계산
            path = await self.path_planner.plan_global_path(
                robot, target_pose[0], target_pose[1]
            )
            
            if path:
                # 2. 경로가 존재하면 실제 거리 합산을 비용으로 사용
                # path는 [{'x':...}, {'y':...}] 형태의 리스트입니다.
                path_length = 0.0
                if len(path) > 1:
                    for i in range(len(path) - 1):
                        p1 = path[i]
                        p2 = path[i+1]
                        path_length += math.hypot(p2['x'] - p1['x'], p2['y'] - p1['y'])
                
                robot_distances.append((robot, path_length))
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

    async def update_robot_status(self, robot_id: Union[int, str], status: RobotStatus, location: Optional[tuple] = None, battery: Optional[float] = None) -> Optional[Robot]:
        """로봇으로부터 수신된 텔레메트리 정보를 DB에 갱신합니다."""
        
        # 1. 기존 로봇 정보를 가져와 현재 상태 확인 (상태 변경 감지용)
        if isinstance(robot_id, str):
            robot = await self.robot_repo.get_by_name(robot_id)
        else:
            robot = await self.robot_repo.get_by_id(robot_id)

        if not robot:
            logger.warning(f"로봇 '{robot_id}'를 찾을 수 없어 상태 업데이트를 건너뜁니다.")
            return None

        # 실제 변경 여부 확인을 위해 이전 상태 저장
        old_status = robot.status
        robot_id = robot.id # ID 확정 (int)
            
        # 2. DB 업데이트 수행
        # location이나 battery가 None이면 기존 값 유지
        new_x = location[0] if location else robot.pose_x
        new_y = location[1] if location else robot.pose_y
        new_battery = battery if battery is not None else robot.battery_level

        # [Optimization] 변경 사항이 없으면 DB 업데이트 건너뛰기
        if (old_status == status and 
            math.isclose(robot.pose_x, new_x, abs_tol=1e-9) and 
            math.isclose(robot.pose_y, new_y, abs_tol=1e-9) and 
            math.isclose(robot.battery_level, new_battery, abs_tol=1e-9)):
            return robot

        update_data = {
            "status": status,
            "pose_x": new_x,
            "pose_y": new_y,
            "battery_level": new_battery
        }
        updated_robot = await self.robot_repo.update(robot_id, update_data)
        
        if updated_robot:
            # 3. 로봇 상태가 실제로 변경되었을 때만 AI 추론 모드(장애물/직원) 제어
            if status != old_status:
                logger.info(f"[{updated_robot.name}] 상태 변경 감지: {old_status} -> {status}. AI 스트림 제어를 업데이트합니다.")
                
                # 3-1. 장애물 감지 제어 (이동 중일 때 활성화)
                if status in [RobotStatus.MOVING, RobotStatus.GUIDING, RobotStatus.ASSIGNED]:
                    await self.enable_obstacle_relay(updated_robot.name)
                else:
                    await self.disable_obstacle_relay(updated_robot.name)
                
                # 3-2. 직원 인식 제어 (IDLE/충전 시 활성화)
                if status in [RobotStatus.IDLE, RobotStatus.CHARGING]:
                    await self.enable_employee_relay(updated_robot.name)
                else:
                    await self.disable_employee_relay(updated_robot.name)

            await self.connection_manager.broadcast(updated_robot.model_dump_json())
        return updated_robot

    async def get_all_robot_status(self) -> List[Robot]:
        return await self.robot_repo.get_all()
