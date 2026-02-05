from enum import Enum

# =================
# General Enums
# =================

class RobotStatus(str, Enum):
    """로봇의 현재 상태 (SR-011)"""
    IDLE = "idle"
    MOVING = "moving"
    PERFORMING_TASK = "performing_task"
    RETURNING = "returning"
    CHARGING = "charging"
    ERROR = "error"

class TaskType(str, Enum):
    """사용자가 요청하는 작업 종류 (시나리오 1-4)"""
    SNACK_DELIVERY = "snack_delivery"
    ITEM_DELIVERY = "item_delivery"
    GUIDE_GUEST = "guide_guest"

class ServerNotification(str, Enum):
    """서버가 사용자 앱으로 보내는 알림 (SR-007)"""
    TASK_ASSIGNED = "task_assigned"
    ROBOT_ARRIVED = "robot_arrived"
    TASK_COMPLETED = "task_completed"
    GUEST_DETECTED = "guest_detected"
    ERROR_OCCURRED = "error_occurred"

class IoTDevice(str, Enum):
    """제어 가능한 IoT 장치의 종류"""
    LIGHT = "light"
    THERMOSTAT = "thermostat"
    AIR_CONDITIONER = "air_conditioner"
    DOOR_LOCK = "door_lock"

class IoTCommand(str, Enum):
    """IoT 장치에 내릴 수 있는 명령의 종류"""
    TURN_ON = "turn_on"
    TURN_OFF = "turn_off"
    SET_VALUE = "set_value" # e.g., for temperature
    LOCK = "lock"
    UNLOCK = "unlock"

# =================
# Communication Protocols (Server <-> Robot)
# =================

class ServerCommand(str, Enum):
    """서버가 로봇에게 내리는 명령 (SR-013, SR-014)"""
    GOTO = "goto"
    PICKUP = "pickup"
    DROPOFF = "dropoff"
    LEAD_GUEST = "lead_guest"
    PAUSE = "pause"
    RESUME = "resume"
    RETURN_TO_STATION = "return_to_station"