import json
import roslibpy
import asyncio
import logging
from typing import List, Dict, Any, Optional
from main_server.infrastructure.robot_bridge.robot_communicator import IRobotCommunicator
from main_server.config import config

logger = logging.getLogger(__name__)

class ROSBridgeCommunicator(IRobotCommunicator):
    """
    rosbridge_suite를 통해 실제 ROS 로봇과 통신하는 구체적인 구현체.
    로봇별 네임스페이스(/{robot_name}/...)를 사용하여 개별 통신을 지원합니다.
    """
    def __init__(self, host: str = config.ROS_BRIDGE_HOST, port: int = config.ROS_BRIDGE_PORT):
        self.host = host
        self.port = port
        self.client = roslibpy.Ros(host=self.host, port=self.port)
        self.command_topics: Dict[str, roslibpy.Topic] = {}
        self.status_topics: Dict[str, roslibpy.Topic] = {}
        logger.info(f"ROSBridgeCommunicator 설정 완료 ({self.host}:{self.port})")

    def connect(self):
        if not self.client.is_connected:
            try:
                self.client.run()
                logger.info(f"ROS Bridge 연결 성공: {self.host}:{self.port}")
            except Exception as e:
                logger.error(f"ROS Bridge 연결 실패: {e}")

    def disconnect(self):
        if self.client.is_connected:
            for topic in self.command_topics.values():
                topic.unadvertise()
            for topic in self.status_topics.values():
                topic.unsubscribe()
            self.client.terminate()
            logger.info("ROS Bridge 연결 종료.")

    def _get_command_topic(self, robot_name: str) -> roslibpy.Topic:
        if robot_name not in self.command_topics:
            self.command_topics[robot_name] = roslibpy.Topic(
                self.client, f"/{robot_name}/commands", "std_msgs/String"
            )
        return self.command_topics[robot_name]

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        if not self.client.is_connected:
            logger.warning("ROS Bridge 미연결 상태로 명령 발행 불가.")
            return

        topic = self._get_command_topic(robot_name)
        message = {
            "robot_name": robot_name,
            "type": "ACTION_SEQUENCE",
            "payload": actions
        }
        topic.publish(roslibpy.Message({"data": json.dumps(message)}))
        logger.info(f"[{robot_name}] 액션 시퀀스 발행 완료 (Topic: {topic.name})")

    def listen_for_status(self, callback: Any):
        """
        [Deprecated] 하위 호환성을 위해 유지하거나 모든 로봇의 상태를 통합 처리할 때 사용 가능.
        네임스페이스 기반 통신에서는 listen_for_robot_status 사용을 권장합니다.
        """
        logger.warning("listen_for_status(callback)는 네임스페이스 환경에서 권장되지 않습니다. listen_for_robot_status를 사용하세요.")

    def listen_for_robot_status(self, robot_name: str, callback: Any):
        """특정 로봇의 네임스페이스 하위 status 토픽을 구독합니다."""
        if robot_name in self.status_topics:
            return

        topic = roslibpy.Topic(self.client, f"/{robot_name}/status", "std_msgs/String")
        
        def _callback(msg):
            try:
                data = json.loads(msg["data"])
                # 데이터에 robot_name이 없을 경우를 대비해 추가
                if "robot_name" not in data:
                    data["robot_name"] = robot_name
                callback(data)
            except Exception as e:
                logger.error(f"[{robot_name}] 상태 메시지 파싱 오류: {e}")

        topic.subscribe(_callback)
        self.status_topics[robot_name] = topic
        logger.info(f"[{robot_name}] 상태 구독 시작 (Topic: {topic.name})")

class ROSBridge:
    """
    애플리케이션 시작 시 백그라운드에서 실행되어 
    ROS 통신 및 FleetManager 상태 동기화를 담당하는 고수준 브리지 서비스.
    """
    def __init__(self, host: str, port: int, fleet_manager: Any, task_manager: Any = None):
        self.communicator = ROSBridgeCommunicator(host, port)
        self.fleet_manager = fleet_manager
        self.task_manager = task_manager
        # 임시 로봇 목록 (나중에 DB나 설정 파일에서 가져오도록 변경 가능)
        self.managed_robots = ["robot_1", "robot_2"]

    async def start(self):
        """ROS Bridge 연결 및 상태 수신 루프 실행"""
        self.communicator.connect()
        
        def status_handler(data: Dict[str, Any]):
            try:
                # data 예시: {"robot_id": 1, "status": "IDLE", "location": [1.2, 3.4], "battery": 85.0, "event": "ARRIVED_AT_DESTINATION"}
                robot_id = data.get("robot_id")
                # robot_name 정보를 활용할 수도 있음
                robot_name = data.get("robot_name")
                status = data.get("status")
                location = tuple(data.get("location", [0, 0]))
                battery = data.get("battery", 0.0)
                event = data.get("event")
                
                logger.debug(f"[{robot_name or robot_id}] 상태 업데이트 수신")

                # 비동기 업데이트 및 이벤트 처리를 메인 루프에서 실행
                asyncio.run_coroutine_threadsafe(
                    self._handle_status_update(robot_id, status, location, battery, event),
                    asyncio.get_event_loop()
                )
            except Exception as e:
                logger.error(f"상태 동기화 핸들러 오류: {e}")

        # 등록된 모든 로봇에 대해 구독 설정
        for robot_name in self.managed_robots:
            self.communicator.listen_for_robot_status(robot_name, status_handler)
        
        try:
            # 연결 유지 대기
            while self.communicator.client.is_connected:
                await asyncio.sleep(1)
        finally:
            self.communicator.disconnect()

    async def _handle_status_update(self, robot_id: int, status: str, location: tuple, battery: float, event: Optional[str]):
        """로봇 상태를 업데이트하고, 이벤트가 있으면 TaskManager에 전달합니다."""
        # 1. FleetManager를 통해 상태 업데이트 (DB 반영)
        updated_robot = await self.fleet_manager.update_robot_status(robot_id, status, location, battery)
        
        # 2. 이벤트가 있고 로봇이 현재 진행 중인 작업이 있다면 TaskManager 호출
        if event and updated_robot and updated_robot.current_task_id and self.task_manager:
            logger.info(f"로봇 {robot_id}로부터 이벤트 수신: {event} (Task ID: {updated_robot.current_task_id})")
            await self.task_manager.handle_robot_event(updated_robot.current_task_id, robot_id, event)
