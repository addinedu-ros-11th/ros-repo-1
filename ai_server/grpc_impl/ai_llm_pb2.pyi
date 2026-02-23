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
    GUIDE_GUEST: _ClassVar[TaskType]
    GENERAL_QUESTION: _ClassVar[TaskType]
    GREETING: _ClassVar[TaskType]
UNKNOWN: TaskType
SNACK_DELIVERY: TaskType
ITEM_DELIVERY: TaskType
GUIDE_GUEST: TaskType
GENERAL_QUESTION: TaskType
GREETING: TaskType

class NLRequest(_message.Message):
    __slots__ = ("req_id", "message")
    REQ_ID_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    req_id: str
    message: str
    def __init__(self, req_id: _Optional[str] = ..., message: _Optional[str] = ...) -> None: ...

class ItemInfo(_message.Message):
    __slots__ = ("item_name", "quantity")
    ITEM_NAME_FIELD_NUMBER: _ClassVar[int]
    QUANTITY_FIELD_NUMBER: _ClassVar[int]
    item_name: str
    quantity: int
    def __init__(self, item_name: _Optional[str] = ..., quantity: _Optional[int] = ...) -> None: ...

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
    __slots__ = ("location", "requester_name", "receiver_name", "visitor_name", "source_location", "dest_location", "items", "message", "keywords")
    LOCATION_FIELD_NUMBER: _ClassVar[int]
    REQUESTER_NAME_FIELD_NUMBER: _ClassVar[int]
    RECEIVER_NAME_FIELD_NUMBER: _ClassVar[int]
    VISITOR_NAME_FIELD_NUMBER: _ClassVar[int]
    SOURCE_LOCATION_FIELD_NUMBER: _ClassVar[int]
    DEST_LOCATION_FIELD_NUMBER: _ClassVar[int]
    ITEMS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    KEYWORDS_FIELD_NUMBER: _ClassVar[int]
    location: str
    requester_name: str
    receiver_name: str
    visitor_name: str
    source_location: str
    dest_location: str
    items: _containers.RepeatedCompositeFieldContainer[ItemInfo]
    message: str
    keywords: _containers.RepeatedScalarFieldContainer[str]
    def __init__(self, location: _Optional[str] = ..., requester_name: _Optional[str] = ..., receiver_name: _Optional[str] = ..., visitor_name: _Optional[str] = ..., source_location: _Optional[str] = ..., dest_location: _Optional[str] = ..., items: _Optional[_Iterable[_Union[ItemInfo, _Mapping]]] = ..., message: _Optional[str] = ..., keywords: _Optional[_Iterable[str]] = ...) -> None: ...
