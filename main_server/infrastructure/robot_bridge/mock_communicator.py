from typing import List, Dict, Any
from main_server.infrastructure.robot_bridge.robot_communicator import IRobotCommunicator

class MockRobotCommunicator(IRobotCommunicator):
    """테스트 및 개발용 Mock 구현체"""
    def __init__(self, host: str = "localhost", port: int = 6000):
        self.host = host
        self.port = port
        self.is_connected = False
        print(f"Mock Robot Communicator: {host}:{port} 시뮬레이션 모드.")

    def connect(self):
        self.is_connected = True
        print("Mock Robot Communicator 연결됨.")

    def disconnect(self):
        self.is_connected = False
        print("Mock Robot Communicator 연결 해제.")

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        print(f"--- [Mock] '{robot_name}' Action Sequence ---")
        for action in actions:
            print(f"  - {action}")
        print("------------------------------------------")

    def listen_for_status(self, callback: Any):
        print("[Mock] 로봇 상태 수신 대기 시작...")
