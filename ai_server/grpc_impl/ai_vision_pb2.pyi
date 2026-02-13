from google.protobuf.internal import containers as _containers
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Iterable as _Iterable, Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class InferenceStateRequest(_message.Message):
    __slots__ = ("robot_id", "model_type", "is_active")
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    MODEL_TYPE_FIELD_NUMBER: _ClassVar[int]
    IS_ACTIVE_FIELD_NUMBER: _ClassVar[int]
    robot_id: str
    model_type: str
    is_active: bool
    def __init__(self, robot_id: _Optional[str] = ..., model_type: _Optional[str] = ..., is_active: bool = ...) -> None: ...

class InferenceStateResponse(_message.Message):
    __slots__ = ("success", "message")
    SUCCESS_FIELD_NUMBER: _ClassVar[int]
    MESSAGE_FIELD_NUMBER: _ClassVar[int]
    success: bool
    message: str
    def __init__(self, success: bool = ..., message: _Optional[str] = ...) -> None: ...

class ImageRequest(_message.Message):
    __slots__ = ("image_id", "image_data")
    IMAGE_ID_FIELD_NUMBER: _ClassVar[int]
    IMAGE_DATA_FIELD_NUMBER: _ClassVar[int]
    image_id: str
    image_data: bytes
    def __init__(self, image_id: _Optional[str] = ..., image_data: _Optional[bytes] = ...) -> None: ...

class ObjectDetectionResponse(_message.Message):
    __slots__ = ("object_name", "confidence", "box")
    OBJECT_NAME_FIELD_NUMBER: _ClassVar[int]
    CONFIDENCE_FIELD_NUMBER: _ClassVar[int]
    BOX_FIELD_NUMBER: _ClassVar[int]
    object_name: str
    confidence: float
    box: BoundingBox
    def __init__(self, object_name: _Optional[str] = ..., confidence: _Optional[float] = ..., box: _Optional[_Union[BoundingBox, _Mapping]] = ...) -> None: ...

class BoundingBox(_message.Message):
    __slots__ = ("x", "y", "width", "height")
    X_FIELD_NUMBER: _ClassVar[int]
    Y_FIELD_NUMBER: _ClassVar[int]
    WIDTH_FIELD_NUMBER: _ClassVar[int]
    HEIGHT_FIELD_NUMBER: _ClassVar[int]
    x: int
    y: int
    width: int
    height: int
    def __init__(self, x: _Optional[int] = ..., y: _Optional[int] = ..., width: _Optional[int] = ..., height: _Optional[int] = ...) -> None: ...

class FaceRecognitionResponse(_message.Message):
    __slots__ = ("person_type", "employee_id", "confidence")
    PERSON_TYPE_FIELD_NUMBER: _ClassVar[int]
    EMPLOYEE_ID_FIELD_NUMBER: _ClassVar[int]
    CONFIDENCE_FIELD_NUMBER: _ClassVar[int]
    person_type: str
    employee_id: str
    confidence: float
    def __init__(self, person_type: _Optional[str] = ..., employee_id: _Optional[str] = ..., confidence: _Optional[float] = ...) -> None: ...

class MultiObjectDetectionResponse(_message.Message):
    __slots__ = ("objects",)
    OBJECTS_FIELD_NUMBER: _ClassVar[int]
    objects: _containers.RepeatedCompositeFieldContainer[ObjectDetectionResponse]
    def __init__(self, objects: _Optional[_Iterable[_Union[ObjectDetectionResponse, _Mapping]]] = ...) -> None: ...

class VisionResult(_message.Message):
    __slots__ = ("robot_id", "timestamp", "object_detection", "face_recognition", "multi_objects")
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    TIMESTAMP_FIELD_NUMBER: _ClassVar[int]
    OBJECT_DETECTION_FIELD_NUMBER: _ClassVar[int]
    FACE_RECOGNITION_FIELD_NUMBER: _ClassVar[int]
    MULTI_OBJECTS_FIELD_NUMBER: _ClassVar[int]
    robot_id: str
    timestamp: int
    object_detection: ObjectDetectionResponse
    face_recognition: FaceRecognitionResponse
    multi_objects: MultiObjectDetectionResponse
    def __init__(self, robot_id: _Optional[str] = ..., timestamp: _Optional[int] = ..., object_detection: _Optional[_Union[ObjectDetectionResponse, _Mapping]] = ..., face_recognition: _Optional[_Union[FaceRecognitionResponse, _Mapping]] = ..., multi_objects: _Optional[_Union[MultiObjectDetectionResponse, _Mapping]] = ...) -> None: ...
