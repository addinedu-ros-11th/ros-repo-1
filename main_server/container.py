"""
애플리케이션의 모든 주요 구성 요소(서비스, 리포지토리)를 중앙에서 관리하는
의존성 주입(DI) 컨테이너입니다.
"""
from main_server.infrastructure.database.connection import Database
from main_server.web.connection_manager import manager as connection_manager

# --- Repository Instances ---
from main_server.domains.robots.repository import IRobotRepository
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository

from main_server.domains.tasks.repository import ITaskRepository
from main_server.infrastructure.database.repositories.mysql_task_repository import MySQLTaskRepository
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_admin_repository import MySQLAdminRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository
from main_server.infrastructure.database.repositories.mysql_log_repository import MySQLLogRepository
from main_server.infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository
from main_server.infrastructure.database.repositories.mysql_reservation_repository import MySQLReservationRepository

# --- Communication Instances ---
from main_server.infrastructure.robot_bridge.robot_communicator import IRobotCommunicator
from main_server.infrastructure.robot_bridge.ros_bridge import ROSBridgeCommunicator

# --- Core Service Instances ---
from main_server.infrastructure.ai_client import LLMServiceClient, VisionServiceClient
from main_server.services.ai_management.ai_processing import AIProcessingService
from main_server.services.office_iot.iot_controller import IoTController
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.services.task_management.task_manager import TaskManager


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
        self.location_repo = None
        self.user_repo = None
        self.product_repository = None
        self.admin_repository = None
        self.log_repository = None
        
        self.robot_communicator = None
        self.ai_processing_service = None
        self.llm_service = None
        self.vision_service = None
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
        
        # 1. Infrastructure Layer (Repositories)
        self.robot_repo: IRobotRepository = MySQLRobotRepository()
        self.task_repo: ITaskRepository = MySQLTaskRepository()
        self.location_repo = MySQLLocationRepository()
        self.user_repo = MySQLUserRepository()
        self.product_repository = MySQLProductRepository()
        self.admin_repository = MySQLAdminRepository()
        self.log_repository = MySQLLogRepository()
        self.reservation_repository = MySQLReservationRepository()
        
        self.robot_communicator: IRobotCommunicator = ROSBridgeCommunicator()
        self.connection_manager = connection_manager # WebSocket 관리자

        # 2. Core Layer (Services)
        self.llm_service = LLMServiceClient()
        self.vision_service = VisionServiceClient()
        
        self.ai_processing_service = AIProcessingService(
            vision_service=self.vision_service,
            llm_service=self.llm_service,
            connection_manager=self.connection_manager
        )
        
        self.iot_controller = IoTController()
        
        self.fleet_manager = FleetManager(
            robot_repo=self.robot_repo,
            robot_communicator=self.robot_communicator,
            connection_manager=self.connection_manager
        )
        
        # TaskManager 생성 시 필요한 모든 리포지토리와 서비스 주입
        self.task_manager = TaskManager(
            task_repo=self.task_repo,
            location_repo=self.location_repo,
            user_repo=self.user_repo,
            product_repo=self.product_repository,
            fleet_manager=self.fleet_manager,
            ai_processing_service=self.ai_processing_service,
            connection_manager=self.connection_manager
        )

        print("모든 서비스가 성공적으로 초기화되었습니다.")
        return self

# FastAPI 앱 전체에서 사용될 전역 컨테이너 인스턴스
container = Container()
