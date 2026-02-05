import json
import roslibpy
import asyncio
from typing import List, Dict, Any, Optional, Callable
from .protocols import IRobotCommunicator
from main_server import config

class ROSBridgeCommunicator(IRobotCommunicator):
    """
    rosbridge_suite를 통해 실제 ROS 로봇과 통신하는 구현체입니다.
    """
    def __init__(self, host: str = config.ROS_BRIDGE_HOST, port: int = config.ROS_BRIDGE_PORT):
        self.host = host
        self.port = port
        self.client = roslibpy.Ros(host=self.host, port=self.port)
        self.command_topic = roslibpy.Topic(self.client, '/robot/commands', 'std_msgs/String')
        self.status_topic = roslibpy.Topic(self.client, '/robot/status', 'std_msgs/String')
        
        print(f"ROSBridgeCommunicator: {self.host}:{self.port} 연결 준비 중...")

    def connect(self):
        if not self.client.is_connected:
            try:
                self.client.run()
                print(f"ROS Bridge 연결 성공: {self.host}:{self.port}")
            except Exception as e:
                print(f"ROS Bridge 연결 실패: {e}")

    def disconnect(self):
        self.client.terminate()
        print("ROS Bridge 연결 종료.")

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        if not self.client.is_connected:
            print("ROS Bridge가 연결되어 있지 않아 명령을 보낼 수 없습니다.")
            return

        message = {
            "robot_name": robot_name,
            "type": "ACTION_SEQUENCE",
            "payload": actions
        }
        self.command_topic.publish(roslibpy.Message({'data': json.dumps(message)}))
        print(f"[{robot_name}] 명령 발행 완료.")

    def listen_for_status(self, callback: Any):
        def _callback(msg):
            try:
                data = json.loads(msg['data'])
                callback(data)
            except Exception as e:
                print(f"상태 메시지 처리 오류: {e}")

        self.status_topic.subscribe(_callback)
        print("로봇 상태 구독 시작 (/robot/status)")

class ROSBridge:
    """
    애플리케이션과 ROS 간의 고수준 가교 역할을 수행하는 클래스.
    FleetManager와 연동하여 수신된 상태를 시스템에 반영합니다.
    """
    def __init__(self, host: str, port: int, fleet_manager: Any):
        self.communicator = ROSBridgeCommunicator(host, port)
        self.fleet_manager = fleet_manager

    async def start(self):
        """ROS Bridge와의 통신을 시작하고 수신 대기 루프를 유지합니다."""
        self.communicator.connect()
        
        # 상태 업데이트 시 FleetManager에 반영하도록 콜백 설정
        def status_handler(data: Dict[str, Any]):
            # 로봇 상태 업데이트 로직 (비동기 함수를 동기 콜백에서 호출하기 위해 이벤트 루프 사용)
            # data 예시: {"robot_id": 1, "status": "IDLE", "location": [1.2, 3.4], "battery": 85.0}
            try:
                robot_id = data.get("robot_id")
                status = data.get("status")
                location = tuple(data.get("location", [0, 0]))
                battery = data.get("battery", 0.0)
                
                # asyncio.run_coroutine_threadsafe를 사용하여 메인 루프에서 실행
                asyncio.run_coroutine_threadsafe(
                    self.fleet_manager.update_robot_status(robot_id, status, location, battery),
                    asyncio.get_event_loop()
                )
            except Exception as e:
                print(f"상태 핸들러 오류: {e}")

        self.communicator.listen_for_status(status_handler)
        
        try:
            while self.communicator.client.is_connected:
                await asyncio.sleep(1)
        finally:
            self.communicator.disconnect()

class MockRobotCommunicator(IRobotCommunicator):
    """테스트용 Mock 구현체"""
    def __init__(self, host: str = "localhost", port: int = 6000):
        print(f"Mock Robot Communicator: {host}:{port} 시뮬레이션 모드.")

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        print(f"--- [Mock] '{robot_name}' Action Sequence ---")
        for action in actions:
            print(f"  - {action}")
        print("------------------------------------------")

    def listen_for_status(self, callback: Any):
        print("[Mock] 로봇 상태 수신 대기 중...")