from typing import Protocol, List, Dict, Any

class IRobotCommunicator(Protocol):
    """
    로봇과의 통신을 위한 인터페이스(프로토콜).
    ROS Bridge, MQTT, TCP 소켓 등 구체적인 구현을 추상화합니다.
    """

    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]]):
        """
        로봇에게 수행할 액션 시퀀스를 전송합니다.
        
        Args:
            robot_name (str): 명령을 수신할 로봇의 이름.
            actions (List[Dict[str, Any]]): 로봇이 순차적으로 수행할 액션 목록.
                예: [{'action': 'GOTO', 'params': {'x': 1.0, 'y': 2.5}}, {'action': 'PICKUP'}]
        """
        ...

    def listen_for_status(self, callback: Any):
        """
        로봇으로부터 상태 업데이트를 비동기적으로 수신 대기합니다.
        
        Args:
            callback (Callable): 상태 데이터를 수신했을 때 호출할 함수.
        """
        ...
