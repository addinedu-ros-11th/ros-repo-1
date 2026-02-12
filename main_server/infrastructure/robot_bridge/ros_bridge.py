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
    """
    def __init__(self, host: str = config.ROS_BRIDGE_HOST, port: int = config.ROS_BRIDGE_PORT):
        self.host = host
        self.port = port
        self.client = roslibpy.Ros(host=self.host, port=self.port)
        self.command_topic = roslibpy.Topic(self.client, '/robot/commands', 'std_msgs/String')
        self.status_topic = roslibpy.Topic(self.client, '/robot/status', 'std_msgs/String')
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
            self.client.terminate()
            logger.info("ROS Bridge 연결 종료.")

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        if not self.client.is_connected:
            logger.warning("ROS Bridge 미연결 상태로 명령 발행 불가.")
            return

        message = {
            "robot_name": robot_name,
            "type": "ACTION_SEQUENCE",
            "payload": actions
        }
        self.command_topic.publish(roslibpy.Message({'data': json.dumps(message)}))
        logger.info(f"[{robot_name}] 액션 시퀀스 발행 완료.")

    def listen_for_status(self, callback: Any):
        def _callback(msg):
            try:
                data = json.loads(msg['data'])
                callback(data)
            except Exception as e:
                logger.error(f"로봇 상태 메시지 파싱 오류: {e}")

        self.status_topic.subscribe(_callback)
        logger.info("로봇 상태 구독 시작 (/robot/status)")

class ROSBridge:
    """
    애플리케이션 시작 시 백그라운드에서 실행되어 
    ROS 통신 및 FleetManager 상태 동기화를 담당하는 고수준 브리지 서비스.
    """
    def __init__(self, host: str, port: int, fleet_manager: Any, task_manager: Any = None):
        self.communicator = ROSBridgeCommunicator(host, port)
        self.fleet_manager = fleet_manager
        self.task_manager = task_manager

    async def start(self):
        """ROS Bridge 연결 및 상태 수신 루프 실행"""
        self.communicator.connect()
        
        def status_handler(data: Dict[str, Any]):
            try:
                # data 예시: {"robot_id": 1, "status": "IDLE", "location": [1.2, 3.4], "battery": 85.0, "event": "ARRIVED_AT_DESTINATION"}
                robot_id = data.get("robot_id")
                status = data.get("status")
                location = tuple(data.get("location", [0, 0]))
                battery = data.get("battery", 0.0)
                event = data.get("event")
                
                # 비동기 업데이트 및 이벤트 처리를 메인 루프에서 실행
                asyncio.run_coroutine_threadsafe(
                    self._handle_status_update(robot_id, status, location, battery, event),
                    asyncio.get_event_loop()
                )
            except Exception as e:
                logger.error(f"상태 동기화 핸들러 오류: {e}")

        self.communicator.listen_for_status(status_handler)
        
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
