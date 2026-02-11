from pydantic import BaseModel, Field
from enum import Enum
from typing import Optional
from datetime import datetime

class TaskType(str, Enum):
    """작업의 종류를 나타내는 열거형"""
    SNACK_DELIVERY = "snack_delivery"
    ITEM_DELIVERY = "item_delivery"
    GUIDE_GUEST = "guide_guest"

class TaskStatus(str, Enum):
    """작업의 현재 상태를 나타내는 열거형"""
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_PROGRESS = "in_progress"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELED = "canceled"

class Task(BaseModel):
    """작업(Task)의 데이터 모델"""
    id: int = Field(..., description="작업의 고유 ID")
    task_type: TaskType = Field(..., description="작업의 종류")
    status: TaskStatus = Field(default=TaskStatus.PENDING, description="작업의 현재 상태")
    requester_id: int = Field(..., description="작업을 요청한 직원의 ID")
    robot_id: Optional[int] = Field(None, description="작업에 할당된 로봇의 ID")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="작업 생성 시간")
    completed_at: Optional[datetime] = Field(None, description="작업 완료 시간")
    details: dict = Field({}, description="작업 관련 추가 정보 (e.g., 목적지, 간식 종류)")

    class Config:
        orm_mode = True
        use_enum_values = True
