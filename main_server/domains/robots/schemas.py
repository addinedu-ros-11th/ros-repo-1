from pydantic import BaseModel, Field
from enum import Enum
from typing import Optional

# ==========================================
# 1. 로봇 관련 Enum (상태)
# ==========================================

class RobotStatus(str, Enum):
    """
    로봇의 현재 물리적/논리적 상태 (SR-008, SR-010)
    DB(Robots 테이블) 및 시나리오 명세와 일치시킴
    """
    IDLE = "IDLE"           # 대기 중 (가용 상태)
    ASSIGNED = "ASSIGNED"   # 작업 배정됨
    MOVING = "MOVING"       # 목적지로 이동 중
    GUIDING = "GUIDING"     # 게스트 가이딩 중
    CHARGING = "CHARGING"   # 충전 중
    ERROR = "ERROR"         # 에러 발생
    OFFLINE = "OFFLINE"     # 연결 끊김 (시스템 내부용)

# ==========================================
# 2. 로봇 데이터 모델 (Schemas)
# ==========================================

class RobotBase(BaseModel):
    name: str = Field(..., description="로봇의 고유 이름 (예: robot_01)")
    status: RobotStatus = Field(default=RobotStatus.IDLE)
    battery_level: float = Field(..., description="배터리 잔량 (%)", ge=0, le=100)
    pose_x: float = Field(default=0.0, description="현재 X 좌표")
    pose_y: float = Field(default=0.0, description="현재 Y 좌표")

class RobotCreate(RobotBase):
    """로봇 등록 시 필요한 데이터"""
    pass

class Robot(RobotBase):
    """로봇 상세 정보 모델"""
    id: int = Field(..., description="데이터베이스 PK")
    current_task_id: Optional[int] = Field(None, description="현재 할당된 작업 ID")
    last_heartbeat: Optional[str] = Field(None, description="마지막 통신 시간")

    class Config:
        from_attributes = True  # Pydantic v2 스타일 (ORM 연동)
        use_enum_values = True

# ==========================================
# 3. 로봇 통신 데이터 구조 (WebSocket/API 전송용)
# ==========================================

"""
[로봇 상태 업데이트 메시지 예시 - WebSocket]
{
    "event": "robot_status_update",
    "data": {
        "robot_id": 1,
        "name": "robot_01",
        "status": "MOVING",
        "location": [12.5, 5.0],
        "battery": 85.5
    }
}
"""
