from pydantic import BaseModel, Field
from enum import Enum
from typing import Optional

class RobotStatus(str, Enum):
    """로봇의 현재 상태를 나타내는 열거형"""
    IDLE = "idle"
    MOVING = "moving"
    WORKING = "working"
    CHARGING = "charging"
    ERROR = "error"
    OFFLINE = "offline"

class Robot(BaseModel):
    """로봇의 데이터 모델"""
    id: int = Field(..., description="로봇의 고유 ID")
    name: str = Field(..., description="로봇의 이름 (예: R2D2-1)")
    status: RobotStatus = Field(default=RobotStatus.IDLE, description="로봇의 현재 상태")
    battery_level: float = Field(..., description="배터리 잔량 (%)", ge=0, le=100)
    pose_x: float = Field(default=0.0, description="로봇의 현재 X 좌표")
    pose_y: float = Field(default=0.0, description="로봇의 현재 Y 좌표")
    current_task_id: Optional[int] = Field(None, description="현재 수행 중인 작업의 ID")

    class Config:
        orm_mode = True
        use_enum_values = True
