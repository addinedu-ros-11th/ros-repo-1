from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class TaskType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    UNKNOWN: _ClassVar[TaskType]
    SNACK_DELIVERY: _ClassVar[TaskType]
    ITEM_DELIVERY: _ClassVar[TaskType]
    PICKUP_ITEM: _ClassVar[TaskType]
    GUIDE_GUEST: _ClassVar[TaskType]
    NAVIGATE_TO_LOCATION: _ClassVar[TaskType]
    FOLLOW_PERSON: _ClassVar[TaskType]
    CALL_ROBOT: _ClassVar[TaskType]
    RETURN_TO_BASE: _ClassVar[TaskType]
    CANCEL_TASK: _ClassVar[TaskType]
    PAUSE_TASK: _ClassVar[TaskType]
    RESUME_TASK: _ClassVar[TaskType]
    CONTROL_LIGHT: _ClassVar[TaskType]
    CONTROL_TEMPERATURE: _ClassVar[TaskType]
    CONTROL_AC: _ClassVar[TaskType]
    CONTROL_DOOR: _ClassVar[TaskType]
    QUERY_ROBOT_STATUS: _ClassVar[TaskType]
    QUERY_LOCATION: _ClassVar[TaskType]
    QUERY_AVAILABILITY: _ClassVar[TaskType]
    FIND_PERSON: _ClassVar[TaskType]
    FIND_ITEM: _ClassVar[TaskType]
    RESERVE_MEETING_ROOM: _ClassVar[TaskType]
    CANCEL_RESERVATION: _ClassVar[TaskType]
    CHECK_ROOM_STATUS: _ClassVar[TaskType]
    PATROL_AREA: _ClassVar[TaskType]
    MONITOR_ENVIRONMENT: _ClassVar[TaskType]
    GENERAL_QUESTION: _ClassVar[TaskType]
    GREETING: _ClassVar[TaskType]

class IoTDeviceType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    IOT_UNKNOWN: _ClassVar[IoTDeviceType]
    LIGHT: _ClassVar[IoTDeviceType]
    THERMOSTAT: _ClassVar[IoTDeviceType]
    AIR_CONDITIONER: _ClassVar[IoTDeviceType]
    DOOR_LOCK: _ClassVar[IoTDeviceType]

class IoTCommandType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = ()
    IOT_CMD_UNKNOWN: _ClassVar[IoTCommandType]
    TURN_ON: _ClassVar[IoTCommandType]
    TURN_OFF: _ClassVar[IoTCommandType]
    SET_VALUE: _ClassVar[IoTCommandType]
    LOCK: _ClassVar[IoTCommandType]
    UNLOCK: _ClassVar[IoTCommandType]
UNKNOWN: TaskType
SNACK_DELIVERY: TaskType
ITEM_DELIVERY: TaskType
PICKUP_ITEM: TaskType
GUIDE_GUEST: TaskType
NAVIGATE_TO_LOCATION: TaskType
FOLLOW_PERSON: TaskType
CALL_ROBOT: TaskType
RETURN_TO_BASE: TaskType
CANCEL_TASK: TaskType
PAUSE_TASK: TaskType
RESUME_TASK: TaskType
CONTROL_LIGHT: TaskType
CONTROL_TEMPERATURE: TaskType
CONTROL_AC: TaskType
CONTROL_DOOR: TaskType
QUERY_ROBOT_STATUS: TaskType
QUERY_LOCATION: TaskType
QUERY_AVAILABILITY: TaskType
FIND_PERSON: TaskType
FIND_ITEM: TaskType
RESERVE_MEETING_ROOM: TaskType
CANCEL_RESERVATION: TaskType
CHECK_ROOM_STATUS: TaskType
PATROL_AREA: TaskType
MONITOR_ENVIRONMENT: TaskType
GENERAL_QUESTION: TaskType
GREETING: TaskType
IOT_UNKNOWN: IoTDeviceType
LIGHT: IoTDeviceType
THERMOSTAT: IoTDeviceType
AIR_CONDITIONER: IoTDeviceType
DOOR_LOCK: IoTDeviceType
IOT_CMD_UNKNOWN: IoTCommandType
TURN_ON: IoTCommandType
TURN_OFF: IoTCommandType
SET_VALUE: IoTCommandType
LOCK: IoTCommandType
UNLOCK: IoTCommandType

class NLRequest(_message.Message):
    __slots__ = ("req_id", "message")
    REQ_ID_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    req_id: str
    message: str
    def __init__(self, req_id: _Optional[str] = ..., message: _Optional[str] = ...) -> None: ...

class StructuredResponse(_message.Message):
    __slots__ = ("req_id", "task_type", "confidence", "struct_msg", "raw_text")
    REQ_ID_FIELD_NUMBER: _ClassVar[int]
    TASK_TYPE_FIELD_NUMBER: _ClassVar[int]
    CONFIDENCE_FIELD_NUMBER: _ClassVar[int]
    STRUCT_MSG_FIELD_NUMBER: _ClassVar[int]
    RAW_TEXT_FIELD_NUMBER: _ClassVar[int]
    req_id: str
    task_type: TaskType
    confidence: float
    struct_msg: StructuredMessage
    raw_text: str
    def __init__(self, req_id: _Optional[str] = ..., task_type: _Optional[_Union[TaskType, str]] = ..., confidence: _Optional[float] = ..., struct_msg: _Optional[_Union[StructuredMessage, _Mapping]] = ..., raw_text: _Optional[str] = ...) -> None: ...

class StructuredMessage(_message.Message):
    __slots__ = ("location", "item", "person_name", "person_id", "source_location", "dest_location", "quantity", "device_type", "command", "target_value", "room_id", "meeting_room_id", "start_time", "end_time", "attendee_count", "area", "waypoints", "query_type", "message", "keywords")
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    ITEM_FIELD_NUMBER: _ClassVar[int]
    PERSON_NAME_FIELD_NUMBER: _ClassVar[int]
    PERSON_ID_FIELD_NUMBER: _ClassVar[int]
    SOURCE_LOCATION_FIELD_NUMBER: _ClassVar[int]
    DEST_LOCATION_FIELD_NUMBER: _ClassVar[int]
    QUANTITY_FIELD_NUMBER: _ClassVar[int]
    DEVICE_TYPE_FIELD_NUMBER: _ClassVar[int]
    COMMAND_FIELD_NUMBER: _ClassVar[int]
    TARGET_VALUE_FIELD_NUMBER: _ClassVar[int]
    ROOM_ID_FIELD_NUMBER: _ClassVar[int]
    MEETING_ROOM_ID_FIELD_NUMBER: _ClassVar[int]
    START_TIME_FIELD_NUMBER: _ClassVar[int]
    END_TIME_FIELD_NUMBER: _ClassVar[int]
    ATTENDEE_COUNT_FIELD_NUMBER: _ClassVar[int]
    AREA_FIELD_NUMBER: _ClassVar[int]
    WAYPOINTS_FIELD_NUMBER: _ClassVar[int]
    QUERY_TYPE_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    KEYWORDS_FIELD_NUMBER: _ClassVar[int]
    location: str
    item: str
    person_name: str
    person_id: str
    source_location: str
    dest_location: str
    quantity: int
    device_type: IoTDeviceType
    command: IoTCommandType
    target_value: float
    room_id: str
    meeting_room_id: str
    start_time: str
    end_time: str
    attendee_count: int
    area: str
    waypoints: _containers.RepeatedScalarFieldContainer[str]
    query_type: str
    message: str
    keywords: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, location: _Optional[str] = ..., item: _Optional[str] = ..., person_name: _Optional[str] = ..., person_id: _Optional[str] = ..., source_location: _Optional[str] = ..., dest_location: _Optional[str] = ..., quantity: _Optional[int] = ..., device_type: _Optional[_Union[IoTDeviceType, str]] = ..., command: _Optional[_Union[IoTCommandType, str]] = ..., target_value: _Optional[float] = ..., room_id: _Optional[str] = ..., meeting_room_id: _Optional[str] = ..., start_time: _Optional[str] = ..., end_time: _Optional[str] = ..., attendee_count: _Optional[int] = ..., area: _Optional[str] = ..., waypoints: _Optional[_Iterable[str]] = ..., query_type: _Optional[str] = ..., message: _Optional[str] = ..., keywords: _Optional[_Iterable[str]] = ...) -> None: ...
