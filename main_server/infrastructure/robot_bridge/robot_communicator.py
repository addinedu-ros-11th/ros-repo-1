from typing import Protocol, List, Dict, Any

class IRobotCommunicator(Protocol):
    """
    로봇과의 통신을 위한 인터페이스(프로토콜).
    ROS Bridge, MQTT, TCP 소켓 등 구체적인 구현을 추상화합니다.
    """

    def connect(self):
        """통신 채널을 연결합니다."""
        ...

    def disconnect(self):
        """통신 채널을 닫습니다."""
        ...

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        """
        로봇에게 수행할 액션 시퀀스를 전송합니다.
        
        Args:
            robot_name (str): 명령을 수신할 로봇의 이름.
            actions (List[Dict[str, Any]]): 로봇이 순차적으로 수행할 액션 목록.
        """
        ...

    def publish_obstacle_info(self, robot_name: str, obstacle_data: Dict[str, Any]):
        """
        AI에서 감지된 장애물 정보를 로봇에게 발행합니다.
        
        Args:
            robot_name (str): 정보를 수신할 로봇의 이름.
            obstacle_data (Dict[str, Any]): 장애물 정보 데이터.
        """
        ...

    def publish_employee_result(self, robot_name: str, result_data: Dict[str, Any]):
        """
        AI에서 인식된 직원/얼굴 정보를 로봇에게 발행합니다.
        
        Args:
            robot_name (str): 정보를 수신할 로봇의 이름.
            result_data (Dict[str, Any]): 얼굴 인식 결과 데이터.
        """
        ...

    def listen_for_status(self, callback: Any):
        """
        [Deprecated] 로봇으로부터 통합 상태 업데이트를 비동기적으로 수신 대기합니다.
        
        Args:
            callback (Callable): 상태 데이터를 수신했을 때 호출할 함수.
        """
        ...

    def listen_for_robot_status(self, robot_name: str, callback: Any):
        """
        특정 로봇으로부터 상태 업데이트를 비동기적으로 수신 대기합니다. (네임스페이스 지원)
        
        Args:
            robot_name (str): 구독할 로봇의 이름.
            callback (Callable): 상태 데이터를 수신했을 때 호출할 함수.
        """
        ...
        
    def cancel_robot_task(self, robot_name: str):
        """
        특정 로봇의 현재 수행 중인 태스크를 취소(중단)합니다.
        
        Args:
            robot_name (str): 취소 명령을 수신할 로봇의 이름.
        """
        ...
