from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from main_server.domains.tasks.schemas import TaskType

# ==========================================
# 1. Vision 관련 데이터 모델 (Inference)
# ==========================================

class BoundingBox(BaseModel):
    """이미지 내 객체 위치 정보"""
    x: int
    y: int
    width: int
    height: int

class ObjectDetectionResult(BaseModel):
    """단일 객체 인식 결과"""
    object_name: str
    confidence: float
    box: Optional[BoundingBox] = None

class FaceRecognitionResult(BaseModel):
    """얼굴 인식 결과"""
    person_type: str  # "Employee", "Guest", "Unknown"
    employee_id: Optional[str] = None
    confidence: float

class AIInferenceStreamResult(BaseModel):
    """실시간 AI 스트리밍 데이터 구조"""
    robot_id: str
    type: str  # "object_detection", "face_recognition"
    content: Any  # ObjectDetectionResult 또는 FaceRecognitionResult

# ==========================================
# 2. LLM 관련 데이터 모델 (NL Interpretation)
# ==========================================

class AIStructuredFields(BaseModel):
    """
    자연어에서 추출된 구조화된 파라미터들.
    TaskType에 따라 사용하는 필드가 달라짐.
    """
    location: Optional[str] = None
    item: Optional[str] = None
    person_name: Optional[str] = None
    person_id: Optional[str] = None
    source_location: Optional[str] = None
    dest_location: Optional[str] = None
    quantity: Optional[int] = None
    device_type: Optional[str] = None
    command: Optional[str] = None
    target_value: Optional[float] = None
    room_id: Optional[str] = None
    meeting_room_id: Optional[str] = None
    start_time: Optional[str] = None
    end_time: Optional[str] = None
    attendee_count: Optional[int] = None

class LLMInterpretationResult(BaseModel):
    """자연어 해석 최종 결과 모델"""
    req_id: str
    task_type: TaskType
    confidence: float
    fields: AIStructuredFields
    raw_text: Optional[str] = None

# ==========================================
# 3. AI 제어 명령 모델
# ==========================================

class InferenceStateControl(BaseModel):
    """AI 서버 추론 상태 제어 요청"""
    robot_id: str
    model_type: str  # "FACE", "OBSTACLE", "SNACK"
    is_active: bool
