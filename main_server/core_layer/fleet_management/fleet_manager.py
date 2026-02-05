import math
import json
from typing import List, Optional, Dict, Any

from main_server.domains.robots.robot import Robot, RobotStatus
from main_server.domains.robots.robot_repository import IRobotRepository
from main_server.domains.tasks.task import Task, TaskType
from main_server.infrastructure.communication.protocols import IRobotCommunicator
from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService
from main_server.web.connection_manager import ConnectionManager


class FleetManager:
    """
    다중 로봇의 상태를 관리하고, 최적의 로봇을 배차하며,
    교착 상태를 제어하는 Fleet Management System.
    데이터베이스 및 로봇 통신과 연동하여 로봇을 관리합니다.
    """
    def __init__(self,
                 robot_repo: IRobotRepository,
                 robot_communicator: IRobotCommunicator,
                 ai_service: AIInferenceService,
                 connection_manager: ConnectionManager):
        """
        리포지토리, 커뮤니케이터, AI 서비스, 웹소켓 연결 관리자를 주입받습니다.
        """
        self.robot_repo = robot_repo
        self.robot_communicator = robot_communicator
        self.ai_service = ai_service
        self.connection_manager = connection_manager
        print("Fleet Manager 초기화 완료 (AI 서비스 연동).")

    async def start_ai_stream(self):
        """
        AI 서버로부터의 실시간 추론 스트림을 구독하고 처리를 시작합니다.
        """
        async def handle_ai_result(data: Dict[str, Any]):
            # AI 추론 결과에 따른 로직 처리 (예: 특정 객체 발견 시 정지, 안내 등)
            robot_id = data.get("robot_id")
            result_type = data.get("type")
            content = data.get("content")
            
            print(f"[AI Stream] 로봇({robot_id})로부터 {result_type} 결과 수신: {content}")
            
            # TODO: 여기에 추론 결과에 따른 구체적인 비즈니스 로직 추가
            # 예: 간식 배달 중 장애물 발견 시 경로 재탐색 요청 등

        await self.ai_service.start_inference_stream(handle_ai_result)

    async def find_optimal_robot(self, target_pose: tuple) -> Optional[Robot]:
        """
        주어진 목적지에 가장 적합한 로봇을 찾습니다.
        (거리, 배터리, 현재 상태 고려)
        """
        print(f"{target_pose}로의 작업을 위한 최적 로봇 탐색...")
        
        idle_robots = await self.robot_repo.find_by_status(RobotStatus.IDLE)
        
        available_robots = [
            robot for robot in idle_robots if robot.battery_level > 20
        ]
        
        if not available_robots:
            print("현재 가용한 로봇이 없습니다.")
            return None
        
        best_robot = None
        min_score = float('inf')

        for robot in available_robots:
            # L2 거리 계산
            dist = math.sqrt((robot.pose_x - target_pose[0])**2 + (robot.pose_y - target_pose[1])**2)
            # 점수가 낮을수록 좋음 (현재는 거리만 고려)
            score = dist 
            if score < min_score:
                min_score = score
                best_robot = robot
        
        print(f"최적 로봇으로 '{best_robot.name}' 선택됨 (거리: {min_score:.2f}).")
        return best_robot

    async def assign_task_to_robot(self, robot: Robot, task: Task) -> Optional[Robot]:
        """
        선택된 로봇에게 작업을 할당하고 상태를 변경하며, 실제 로봇에게 명령을 전송합니다.
        변경사항은 WebSocket으로 브로드캐스트됩니다.
        """
        update_data = {
            "status": RobotStatus.MOVING,
            "current_task_id": task.id
        }
        updated_robot = await self.robot_repo.update(robot.id, update_data)
        
        if updated_robot:
            print(f"로봇 '{updated_robot.name}'에게 작업 ID {task.id} 할당 (DB 업데이트).")
            
            # 로봇이 수행할 Action Sequence 생성
            actions = self._generate_action_sequence(task)
            
            # 실제 로봇에게 명령 전송
            self.robot_communicator.send_action_sequence(robot.name, actions)
            print(f"로봇 '{robot.name}'에게 실제 작업 명령 전송 완료.")

            # 변경된 상태를 모든 관리자 클라이언트에게 브로드캐스트
            await self.connection_manager.broadcast(updated_robot.model_dump_json())

        return updated_robot

    def _generate_action_sequence(self, task: Task) -> List[Dict[str, Any]]:
        """작업 유형에 따라 로봇이 수행할 Action Command 리스트를 생성합니다."""
        print(f"작업 ID {task.id} (유형: {task.task_type})에 대한 Action Sequence 생성 중...")
        actions = []
        
        if task.task_type == TaskType.SNACK_DELIVERY:
            pantry_loc = task.details.get("pantry_location", {"x": 5, "y": 5}) # 예시: 간식창고 위치
            user_loc = task.details.get("destination", {"x": 0, "y": 0})
            actions.append({"action": "GOTO", "params": pantry_loc})
            actions.append({"action": "PICKUP", "params": {"item": task.details.get("item_name")}})
            actions.append({"action": "GOTO", "params": user_loc})
            actions.append({"action": "DROPOFF"})
            
        elif task.task_type == TaskType.ITEM_DELIVERY:
            pickup_loc = task.details.get("source")
            dropoff_loc = task.details.get("destination")
            actions.append({"action": "GOTO", "params": pickup_loc})
            actions.append({"action": "PICKUP"})
            actions.append({"action": "GOTO", "params": dropoff_loc})
            actions.append({"action": "DROPOFF"})

        elif task.task_type == TaskType.GUIDE_GUEST:
            destination = task.details.get("destination")
            actions.append({"action": "LEAD_GUEST", "params": destination})
        
        print(f"생성된 Actions: {actions}")
        return actions

    async def update_robot_status(self, robot_id: int, status: RobotStatus, location: tuple, battery: float) -> Optional[Robot]:
        """
        로봇으로부터 주기적으로 상태를 보고받아 DB를 갱신하고,
        변경사항을 WebSocket으로 브로드캐스트합니다.
        """
        update_data = {
            "status": status,
            "pose_x": location[0],
            "pose_y": location[1],
            "battery_level": battery
        }
        updated_robot = await self.robot_repo.update(robot_id, update_data)
        
        if updated_robot:
            # 변경된 상태를 모든 관리자 클라이언트에게 브로드캐스트
            await self.connection_manager.broadcast(updated_robot.model_dump_json())

        return updated_robot

    async def get_all_robot_status(self) -> List[Robot]:
        """
        모든 로봇의 현재 상태를 DB에서 조회하여 반환합니다.
        """
        return await self.robot_repo.get_all()
