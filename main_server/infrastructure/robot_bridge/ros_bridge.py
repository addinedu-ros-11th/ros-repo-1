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
    다중 로봇 연결을 지원하는 ROS Bridge 통신 모듈.
    """
    def __init__(self):
        self.clients: Dict[str, roslibpy.Ros] = {}
        self.topics: Dict[str, Dict[str, roslibpy.Topic]] = {}
        logger.info("Multi-Robot ROSBridgeCommunicator Initialized")

    def add_robot(self, robot_name: str, host: str, port: int = 9090):
        if robot_name in self.clients:
            return

        try:
            logger.info(f"Adding robot: {robot_name} -> {host}:{port}")
            client = roslibpy.Ros(host=host, port=port)
            self.clients[robot_name] = client
            self.topics[robot_name] = {}
            client.on_ready(lambda: logger.info(f"[{robot_name}] ROS Bridge Connected!"))
        except Exception as e:
            logger.error(f"Failed to create client for ({robot_name}): {e}")

    def connect(self):
        for name, client in self.clients.items():
            if not client.is_connected:
                try:
                    logger.info(f"[{name}] Connecting to ROS Bridge...")
                    client.run(timeout=5)
                except Exception as e:
                    logger.error(f"[{name}] Connection failed: {e}")

    def disconnect(self):
        for name, client in self.clients.items():
            if client.is_connected:
                client.terminate()
        self.clients.clear()
        self.topics.clear()

    def _get_client(self, robot_name: str) -> Optional[roslibpy.Ros]:
        client = self.clients.get(robot_name)
        if not client:
            logger.warning(f"Client not found for robot: {robot_name}. Available: {list(self.clients.keys())}")
            return None
        if not client.is_connected:
             logger.warning(f"Client for {robot_name} is not connected.")
             return None
        return client

    def _get_topic(self, robot_name: str, topic_name: str, message_type: str) -> Optional[roslibpy.Topic]:
        client = self._get_client(robot_name)
        if not client: return None
        
        topic_key = f"{topic_name}_{message_type}"
        if topic_key not in self.topics[robot_name]:
            self.topics[robot_name][topic_key] = roslibpy.Topic(client, topic_name, message_type)
        return self.topics[robot_name][topic_key]

    # --- IRobotCommunicator Implementation ---
    def send_action_sequence(self, robot_name: str, actions: List[Dict[str, Any]], task_id: Optional[int] = None):
        """로봇에게 액션 시퀀스와 Task ID를 전송합니다."""
        topic_name = f"/{robot_name}/commands"
        topic = self._get_topic(robot_name, topic_name, "std_msgs/String")
        if topic:
            payload = {
                "robot_name": robot_name,
                "type": "ACTION_SEQUENCE",
                "payload": actions,
                "task_id": task_id
            }
            json_payload = json.dumps(payload)
            logger.info(f"📡 [ROSBridge -> {robot_name}] Publishing to {topic_name}: {json_payload[:100]}...")
            topic.publish(roslibpy.Message({"data": json_payload}))
            logger.info(f"📤 [ROSBridge -> {robot_name}] Published ACTION_SEQUENCE successfully.")
        else:
            logger.error(f"❌ [ROSBridge -> {robot_name}] Failed to get topic {topic_name}")

    def publish_obstacle_info(self, robot_name: str, obstacle_data: Dict[str, Any]):
        topic_name = f"/{robot_name}/obstacles"
        topic = self._get_topic(robot_name, topic_name, "std_msgs/String")
        if topic:
            payload = {"robot_name": robot_name, "type": "OBSTACLE_INFO", "payload": obstacle_data}
            topic.publish(roslibpy.Message({"data": json.dumps(payload)}))
            logger.debug(f"📡 [ROSBridge -> {robot_name}] Published OBSTACLE_INFO to {topic_name}")

    def publish_employee_result(self, robot_name: str, result_data: Dict[str, Any]):
        topic_name = f"/{robot_name}/employee_verification"
        topic = self._get_topic(robot_name, topic_name, "std_msgs/String")
        if topic:
            payload = {"robot_name": robot_name, "type": "EMPLOYEE_RESULT", "payload": result_data}
            logger.info(f"📡 [ROSBridge -> {robot_name}] Publishing EMPLOYEE_RESULT to {topic_name}")
            topic.publish(roslibpy.Message({"data": json.dumps(payload)}))

    def set_forbidden_zones(self, robot_name: str, zones: List[Dict[str, Any]]) -> bool:
        client = self._get_client(robot_name)
        if not client: return False
        service = roslibpy.Service(client, f"/{robot_name}/set_forbidden_zones", 'office_robot_msgs/SetForbiddenZones')
        try:
            response = service.call(roslibpy.ServiceRequest({'json_data': json.dumps(zones)}), timeout=2.0)
            return response.get('success', False)
        except Exception as e: 
            logger.error(f"[{robot_name}] Failed to set forbidden zones: {e}")
            return False

    def listen_for_robot_status(self, robot_name: str, callback: Any):
        target_topic = f"/{robot_name}/status"
        topic = self._get_topic(robot_name, target_topic, "std_msgs/String")
        if topic:
            def _cb(msg):
                try:
                    data = json.loads(msg["data"])
                    if "robot_name" not in data: data["robot_name"] = robot_name
                    callback(data)
                except Exception as e: 
                    logger.error(f"[{robot_name}] Error processing status msg: {e}")
            if not topic.is_subscribed:
                topic.subscribe(_cb)
                logger.info(f"[{robot_name}] Subscribed to {target_topic}")

    def cancel_robot_task(self, robot_name: str):
        topic = self._get_topic(robot_name, f"/{robot_name}/commands", "std_msgs/String")
        if topic:
            topic.publish(roslibpy.Message({"data": json.dumps({"robot_name": robot_name, "type": "CANCEL", "payload": {}})}))


class ROSBridge:
    def __init__(self, fleet_manager: Any, task_manager: Any = None, mutex_manager: Any = None, log_repo: Any = None, communicator: Any = None):
        self.communicator = communicator if communicator else ROSBridgeCommunicator()
        self.fleet_manager = fleet_manager
        self.task_manager = task_manager
        self.mutex_manager = mutex_manager
        self.log_repo = log_repo
        
        self.managed_robots: List[str] = []
        self.last_heartbeat: Dict[str, float] = {}
        self.last_known_status: Dict[str, str] = {}

    async def start(self):
        loop = asyncio.get_running_loop()
        try:
            robots = await self.fleet_manager.get_all_robot_status()
            self.managed_robots = [r.name for r in robots]
            for robot in robots:
                host = getattr(robot, 'ip_address', None) 
                if not host:
                    host = f"{robot.name}.local"
                    logger.info(f"Robot {robot.name} has no IP in DB. Using mDNS: {host}")
                self.communicator.add_robot(robot.name, host, port=9090)
                self.last_known_status[robot.name] = robot.status.value if hasattr(robot.status, 'value') else robot.status

            def status_handler(data: Dict[str, Any]):
                asyncio.run_coroutine_threadsafe(self._handle_status_update(data), loop)

            self.communicator.connect()
            await asyncio.sleep(2)
            for name in self.managed_robots:
                self.communicator.listen_for_robot_status(name, status_handler)
            asyncio.create_task(self._monitor_heartbeats())
            while True: await asyncio.sleep(5)
        except Exception as e:
            logger.error(f"ROSBridge critical error: {e}")
        finally:
            self.communicator.disconnect()

    async def _monitor_heartbeats(self):
        while True:
            await asyncio.sleep(5)
            now = time.time()
            for name in self.managed_robots:
                last_hb = self.last_heartbeat.get(name, 0)
                if last_hb > 0 and (now - last_hb > 15):
                    if self.last_known_status.get(name) != "OFFLINE":
                        logger.warning(f"[{name}] Heartbeat lost. Marking OFFLINE.")
                        await self.fleet_manager.update_robot_status(robot_id=name, status="OFFLINE")
                        self.last_known_status[name] = "OFFLINE"

    async def _handle_status_update(self, data: Dict[str, Any]):
        robot_id_raw = data.get("robot_id")
        robot_name_raw = data.get("robot_name")
        identifier = robot_name_raw or robot_id_raw
        if not identifier: return
        self.last_heartbeat[str(identifier)] = time.time()
        
        status = data.get("status")
        location = tuple(data.get("location", [0, 0]))
        battery = data.get("battery", 0.0)
        event = data.get("event")

        updated_robot = await self.fleet_manager.update_robot_status(
            robot_id=identifier, status=status, location=location, battery=battery
        )
        if updated_robot:
            self.last_known_status[updated_robot.name] = status
            if event:
                logger.info(f"[{updated_robot.name}] Event received: {event}")
                if self.task_manager and updated_robot.current_task_id:
                    await self.task_manager.handle_robot_event(updated_robot.current_task_id, updated_robot.id, event, data)
            if status == "ERROR" and self.log_repo:
                await self.log_repo.create({"log_level": "ERROR", "event_type": "ROBOT_ERROR", "robot_id": updated_robot.id, "message": f"Robot {updated_robot.name} error: {data.get('reason','')}"})
