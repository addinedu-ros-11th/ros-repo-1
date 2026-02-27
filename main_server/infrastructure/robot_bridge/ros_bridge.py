import json
import roslibpy
import asyncio
import logging
import time
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

    def _get_obstacle_topic(self, robot_name: str) -> roslibpy.Topic:
        topic_name = f"/{robot_name}/obstacles"
        # 딕셔너리 키를 topic_name으로 하거나 별도 obstacle_topics 딕셔너리를 관리할 수 있음.
        # 여기서는 간단히 command_topics와 구분하기 위해 별도 관리는 하지 않되,
        # 재사용성을 위해 객체 속성으로 저장하는 것이 좋음.
        if not hasattr(self, 'obstacle_topics'):
            self.obstacle_topics = {}
            
        if robot_name not in self.obstacle_topics:
            self.obstacle_topics[robot_name] = roslibpy.Topic(
                self.client, topic_name, "std_msgs/String"
            )
        return self.obstacle_topics[robot_name]

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

    def publish_obstacle_info(self, robot_name: str, obstacle_data: Dict[str, Any]):
        """AI에서 감지된 장애물 정보를 로봇에게 발행합니다."""
        if not self.client.is_connected:
            return

        topic = self._get_obstacle_topic(robot_name)
        # obstacle_data는 이미 딕셔너리 형태라고 가정
        message = {
            "robot_name": robot_name,
            "type": "OBSTACLE_INFO",
            "payload": obstacle_data
        }
        topic.publish(roslibpy.Message({"data": json.dumps(message)}))

    def _get_employee_topic(self, robot_name: str) -> roslibpy.Topic:
        topic_name = f"/{robot_name}/employee_verification"
        if not hasattr(self, 'employee_topics'):
            self.employee_topics = {}
            
        if robot_name not in self.employee_topics:
            self.employee_topics[robot_name] = roslibpy.Topic(
                self.client, topic_name, "std_msgs/String"
            )
        return self.employee_topics[robot_name]

    def publish_employee_result(self, robot_name: str, result_data: Dict[str, Any]):
        """AI에서 인식된 직원/얼굴 정보를 로봇에게 발행합니다."""
        if not self.client.is_connected:
            return

        topic = self._get_employee_topic(robot_name)
        message = {
            "robot_name": robot_name,
            "type": "EMPLOYEE_RESULT",
            "payload": result_data
        }
        topic.publish(roslibpy.Message({"data": json.dumps(message)}))

    def set_forbidden_zones(self, robot_name: str, zones: List[Dict[str, Any]]) -> bool:
        """
        로봇의 금지 구역 설정 서비스(Service)를 호출합니다.
        Service: /{robot_name}/set_forbidden_zones
        Type: office_robot_msgs/SetForbiddenZones (가정)
        Request: { "json_data": <zones_json_string> }
        """
        if not self.client.is_connected:
            logger.warning("ROS Bridge 미연결 상태. 서비스 호출 불가.")
            return False

        service_name = f"/{robot_name}/set_forbidden_zones"
        service = roslibpy.Service(self.client, service_name, 'office_robot_msgs/SetForbiddenZones')
        
        try:
            # JSON 문자열로 변환하여 요청
            request = roslibpy.ServiceRequest({'json_data': json.dumps(zones)})
            # 동기 호출 (timeout 2초)
            response = service.call(request, timeout=2.0)
            
            success = response.get('success', False)
            msg = response.get('message', '')
            
            if success:
                logger.info(f"[{robot_name}] 금지 구역 설정 성공: {msg}")
            else:
                logger.warning(f"[{robot_name}] 금지 구역 설정 실패: {msg}")
                
            return success
        except Exception as e:
            logger.error(f"[{robot_name}] 금지 구역 서비스 호출 중 오류: {e}")
            return False

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
            # 원시 메시지 수신 로그 추가 (통신 여부 확인용)
            logger.info(f"[{robot_name}] Raw message received: {msg}")
            try:
                data = json.loads(msg["data"])
                # 데이터에 robot_name이 없을 경우를 대비해 추가
                if "robot_name" not in data:
                    data["robot_name"] = robot_name
                callback(data)
            except Exception as e:
                logger.error(f"[{robot_name}] 상태 메시지 파싱 오류: {e} (Raw: {msg})")

        topic.subscribe(_callback)
        self.status_topics[robot_name] = topic
        logger.info(f"[{robot_name}] 상태 구독 시작 (Topic: {topic.name})")

class ROSBridge:
    """
    애플리케이션 시작 시 백그라운드에서 실행되어 
    ROS 통신 및 FleetManager 상태 동기화를 담당하는 고수준 브리지 서비스.
    """
    def __init__(self, host: str, port: int, fleet_manager: Any, task_manager: Any = None, mutex_manager: Any = None, log_repo: Any = None):
        self.communicator = ROSBridgeCommunicator(host, port)
        self.fleet_manager = fleet_manager
        self.task_manager = task_manager
        self.mutex_manager = mutex_manager
        self.log_repo = log_repo
        # DB에서 동적으로 로드하기 위해 초기화 시에는 빈 리스트
        self.managed_robots = []
        self.last_heartbeat: Dict[str, float] = {}
        self.last_known_status: Dict[str, str] = {} # [Optimization] 로컬 상태 캐시

    async def start(self):
        """ROS Bridge 연결 및 상태 수신 루프 실행"""
        # 현재 실행 중인 메인 이벤트 루프를 캡처합니다.
        loop = asyncio.get_running_loop()
        self.communicator.connect()
        
        def status_handler(data: Dict[str, Any]):
            try:
                # 캡처한 메인 루프(loop)에 코루틴을 안전하게 전달합니다.
                asyncio.run_coroutine_threadsafe(
                    self._handle_status_update(data),
                    loop
                )
            except Exception as e:
                logger.error(f"상태 동기화 핸들러 오류: {e}")

        # DB에서 관리 대상 로봇 목록 동적 로드
        try:
            robots = await self.fleet_manager.get_all_robot_status()
            if robots:
                self.managed_robots = [r.name for r in robots]
                logger.info(f"관리 대상 로봇 {len(self.managed_robots)}대 로드 완료: {self.managed_robots}")
                
                # 초기 상태 캐싱
                for r in robots:
                    self.last_known_status[r.name] = r.status.value if hasattr(r.status, 'value') else r.status
            else:
                logger.warning("관리 대상 로봇이 DB에 없습니다.")
        except Exception as e:
            logger.error(f"로봇 목록 로드 실패: {e}")
            self.managed_robots = []

        # 등록된 모든 로봇에 대해 구독 설정
        for robot_name in self.managed_robots:
            self.communicator.listen_for_robot_status(robot_name, status_handler)
            
        # 하트비트 모니터링 시작 (백그라운드)
        asyncio.create_task(self._monitor_heartbeats())
        
        try:
            # 연결 유지 대기
            while self.communicator.client.is_connected:
                await asyncio.sleep(1)
        finally:
            self.communicator.disconnect()

    async def _monitor_heartbeats(self):
        """주기적으로 로봇의 마지막 통신 시간을 확인하여 연결이 끊긴 로봇을 OFFLINE 처리합니다."""
        logger.info("Heartbeat Monitor Started")
        while True:
            await asyncio.sleep(5) # 5초마다 검사
            current_time = time.time()
            
            for robot_name in self.managed_robots:
                last_seen = self.last_heartbeat.get(robot_name, 0)
                
                # 10초 이상 통신 없으면 OFFLINE 처리
                if current_time - last_seen > 10:
                    # [Optimization] 이미 OFFLINE으로 알고 있다면 DB 호출 생략
                    if self.last_known_status.get(robot_name) == "OFFLINE":
                        continue

                    try:
                        await self.fleet_manager.update_robot_status(
                            robot_id=robot_name, 
                            status="OFFLINE"
                        )
                        self.last_known_status[robot_name] = "OFFLINE" # 캐시 업데이트
                        logger.warning(f"[{robot_name}] Connection timed out. Status set to OFFLINE.")
                    except Exception as e:
                        logger.error(f"[{robot_name}] OFFLINE 전환 실패: {e}")

    async def _handle_status_update(self, data: Dict[str, Any]):
        """로봇 상태를 업데이트하고, 이벤트가 있으면 TaskManager 또는 MutexZoneManager에 전달합니다."""
        robot_id = data.get("robot_id")
        status = data.get("status")

        # [Heartbeat Update]
        r_name = data.get("robot_name") or robot_id
        if r_name:
            self.last_heartbeat[str(r_name)] = time.time()
            if status:
                self.last_known_status[str(r_name)] = status # 캐시 업데이트

        logger.debug(f"[{robot_id}] 로봇 상태 데이터 수신: {data}")
        
        location = tuple(data.get("location", [0, 0]))
        battery = data.get("battery", 0.0)
        event = data.get("event")

        # 1. FleetManager를 통해 상태 업데이트 (DB 반영)
        updated_robot = await self.fleet_manager.update_robot_status(robot_id, status, location, battery)
        
        # 2. 로봇 에러 상태 별도 로깅 (System_Logs)
        if status == "ERROR" and updated_robot and self.log_repo:
            try:
                await self.log_repo.create({
                    "log_level": "ERROR",
                    "event_type": "ROBOT_ERROR",
                    "robot_id": updated_robot.id,
                    "message": f"로봇 {updated_robot.name} (ID:{updated_robot.id}) 상태 오류 보고됨."
                })
            except Exception as e:
                logger.error(f"시스템 로그 기록 실패: {e}")

        # 3. Mutex Zone 관련 이벤트 처리
        if event and self.mutex_manager:
            zone_id = data.get("zone_id")
            if event == "MUTEX_ZONE_ENTRY_REQUEST" and zone_id:
                await self.mutex_manager.request_entry(robot_id, zone_id)
            elif event == "MUTEX_ZONE_EXIT" and zone_id:
                await self.mutex_manager.release_zone(robot_id, zone_id)

        # 3. 일반 태스크 이벤트 (목적지 도착 등) 처리
        if event and updated_robot and updated_robot.current_task_id and self.task_manager:
            logger.info(f"로봇 {robot_id}로부터 이벤트 수신: {event} (Task ID: {updated_robot.current_task_id})")
            await self.task_manager.handle_robot_event(updated_robot.current_task_id, robot_id, event)
