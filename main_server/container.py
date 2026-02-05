"""
애플리케이션의 모든 주요 구성 요소(서비스, 리포지토리)를 중앙에서 관리하는
의존성 주입(DI) 컨테이너입니다.
"""
from main_server.infrastructure.database.connection import Database
from main_server.web.connection_manager import manager as connection_manager

# --- Repository Instances ---
from main_server.domains.robots.robot_repository import IRobotRepository
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository

from main_server.domains.tasks.task_repository import ITaskRepository
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository

# --- Communication Instances ---
from main_server.infrastructure.communication.protocols import IRobotCommunicator
from main_server.infrastructure.communication.ros_bridge import ROSBridgeCommunicator

# --- Core Service Instances ---
from main_server.core_layer.ai_inference.grpc_inference_client import AIInferenceService
from main_server.core_layer.office_iot.iot_controller import IoTController
from main_server.core_layer.fleet_management.fleet_manager import FleetManager
from main_server.core_layer.task_management.task_manager import TaskManager


class Container:
    """
    애플리케이션의 싱글턴 서비스와 의존성을 관리하는 컨테이너 클래스.
    FastAPI 앱의 생명주기와 함께 초기화됩니다.
    """
    def __init__(self):
        print("DI 컨테이너 초기화 시작...")
        # 이 변수들은 services()가 호출될 때 채워집니다.
        self.robot_repo = None
        self.task_repo = None
        self.robot_communicator = None
        self.ai_service = None
        self.iot_controller = None
        self.fleet_manager = None
        self.task_manager = None
        self.connection_manager = None

    def services(self):
        """
        모든 싱글턴 서비스 인스턴스를 초기화하고 의존성을 주입합니다.
        """
        if self.task_manager:
            # 이미 초기화되었으면 그대로 반환
            return self

        print("서비스 인스턴스 생성 및 의존성 주입...")
        
        # 1. Infrastructure Layer
        self.robot_repo: IRobotRepository = MySQLRobotRepository()
        self.task_repo: ITaskRepository = MySQLTaskRepository()
        self.robot_communicator: IRobotCommunicator = ROSBridgeCommunicator()
        self.connection_manager = connection_manager # WebSocket 관리자

        # 2. Core Layer
        self.ai_service = AIInferenceService()
        self.iot_controller = IoTController()
        
        self.fleet_manager = FleetManager(
            robot_repo=self.robot_repo,
            robot_communicator=self.robot_communicator,
            ai_service=self.ai_service,
            connection_manager=self.connection_manager
        )
        self.task_manager = TaskManager(
            task_repo=self.task_repo,
            fleet_manager=self.fleet_manager
        )

        print("모든 서비스가 성공적으로 초기화되었습니다.")
        return self

# FastAPI 앱 전체에서 사용될 전역 컨테이너 인스턴스
container = Container()
