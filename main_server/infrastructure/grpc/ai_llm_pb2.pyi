from google.protobuf.internal import containers as _containers
from google.protobuf.internal import enum_type_wrapper as _enum_type_wrapper
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from typing import ClassVar as _ClassVar, Iterable as _Iterable, Mapping as _Mapping, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor
GENERAL_QUESTION: TaskType
GREETING: TaskType
GUIDE_GUEST: TaskType
ITEM_DELIVERY: TaskType
SNACK_DELIVERY: TaskType
UNKNOWN: TaskType

class ItemInfo(_message.Message):
    __slots__ = ["item_name", "quantity"]
    ITEM_NAME_FIELD_NUMBER: _ClassVar[int]
    QUANTITY_FIELD_NUMBER: _ClassVar[int]
    item_name: str
    quantity: int
    def __init__(self, item_name: _Optional[str] = ..., quantity: _Optional[int] = ...) -> None: ...

class NLRequest(_message.Message):
    __slots__ = ["message", "req_id"]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    REQ_ID_FIELD_NUMBER: _ClassVar[int]
    message: str
    req_id: str
    def __init__(self, req_id: _Optional[str] = ..., message: _Optional[str] = ...) -> None: ...

class StructuredMessage(_message.Message):
    __slots__ = ["dest_location", "items", "keywords", "location", "message", "receiver_name", "requester_name", "source_location", "visitor_name"]
    DEST_LOCATION_FIELD_NUMBER: _ClassVar[int]
    ITEMS_FIELD_NUMBER: _ClassVar[int]
    KEYWORDS_FIELD_NUMBER: _ClassVar[int]
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    RECEIVER_NAME_FIELD_NUMBER: _ClassVar[int]
    REQUESTER_NAME_FIELD_NUMBER: _ClassVar[int]
    SOURCE_LOCATION_FIELD_NUMBER: _ClassVar[int]
    VISITOR_NAME_FIELD_NUMBER: _ClassVar[int]
    dest_location: str
    items: _containers.RepeatedCompositeFieldContainer[ItemInfo]
    keywords: _containers.RepeatedScalarFieldContainer[str]
    location: str
    message: str
    receiver_name: str
    requester_name: str
    source_location: str
    visitor_name: str
    def __init__(self, location: _Optional[str] = ..., requester_name: _Optional[str] = ..., receiver_name: _Optional[str] = ..., visitor_name: _Optional[str] = ..., source_location: _Optional[str] = ..., dest_location: _Optional[str] = ..., items: _Optional[_Iterable[_Union[ItemInfo, _Mapping]]] = ..., message: _Optional[str] = ..., keywords: _Optional[_Iterable[str]] = ...) -> None: ...

class StructuredResponse(_message.Message):
    __slots__ = ["confidence", "raw_text", "req_id", "struct_msg", "task_type"]
    CONFIDENCE_FIELD_NUMBER: _ClassVar[int]
    RAW_TEXT_FIELD_NUMBER: _ClassVar[int]
    REQ_ID_FIELD_NUMBER: _ClassVar[int]
    STRUCT_MSG_FIELD_NUMBER: _ClassVar[int]
    TASK_TYPE_FIELD_NUMBER: _ClassVar[int]
    confidence: float
    raw_text: str
    req_id: str
    struct_msg: StructuredMessage
    task_type: TaskType
    def __init__(self, req_id: _Optional[str] = ..., task_type: _Optional[_Union[TaskType, str]] = ..., confidence: _Optional[float] = ..., struct_msg: _Optional[_Union[StructuredMessage, _Mapping]] = ..., raw_text: _Optional[str] = ...) -> None: ...

class TaskType(int, metaclass=_enum_type_wrapper.EnumTypeWrapper):
    __slots__ = []
