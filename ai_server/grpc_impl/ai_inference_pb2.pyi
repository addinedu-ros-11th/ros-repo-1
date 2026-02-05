from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from collections.abc import Mapping as _Mapping
from typing import ClassVar as _ClassVar, Optional as _Optional, Union as _Union

DESCRIPTOR: _descriptor.FileDescriptor

class Empty(_message.Message):
    __slots__ = ()
    def __init__(self) -> None: ...

class InferenceResult(_message.Message):
    __slots__ = ("robot_id", "object_detection", "face_recognition")
    ROBOT_ID_FIELD_NUMBER: _ClassVar[int]
    OBJECT_DETECTION_FIELD_NUMBER: _ClassVar[int]
    FACE_RECOGNITION_FIELD_NUMBER: _ClassVar[int]
    robot_id: str
    object_detection: ObjectDetectionResponse
    face_recognition: FaceRecognitionResponse
    def __init__(self, robot_id: _Optional[str] = ..., object_detection: _Optional[_Union[ObjectDetectionResponse, _Mapping]] = ..., face_recognition: _Optional[_Union[FaceRecognitionResponse, _Mapping]] = ...) -> None: ...

class ImageRequest(_message.Message):
    __slots__ = ("image_id",)
    IMAGE_ID_FIELD_NUMBER: _ClassVar[int]
    image_id: str
    def __init__(self, image_id: _Optional[str] = ...) -> None: ...

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
