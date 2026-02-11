from enum import Enum
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any, List
from datetime import datetime

# ==========================================
# 1. 태스크 관련 Enum (상태 및 유형)
# ==========================================

class TaskType(str, Enum):
    """로봇이 수행하는 서비스 시나리오 유형 (SR-011)"""
    SNACK_DELIVERY = "SNACK_DELIVERY"   # 간식 배달
    ITEM_DELIVERY = "ITEM_DELIVERY"     # P2P 물품 배달
    GUIDE_GUEST = "GUIDE_GUEST"         # 방문객 가이드
    RETURN = "RETURN"                   # 복귀 모드
    PATROL = "PATROL"                   # 순찰 모드

class TaskStatus(str, Enum):
    """태스크의 생명주기 상태 (SR-012)"""
    WAITING = "WAITING"                 # 대기 중 (배차 전)
    ASSIGNED = "ASSIGNED"               # 로봇 배차 완료
    IN_PROGRESS = "IN_PROGRESS"         # 수행 중
    ARRIVED = "ARRIVED"                 # 목적지 도착
    COMPLETED = "COMPLETED"             # 완료
    FAILED = "FAILED"                   # 실패
    CANCELLED = "CANCELLED"             # 취소됨

# ==========================================
# 2. 로봇 제어 액션 Enum (Action Sequence용)
# ==========================================

class RobotActionType(str, Enum):
    """로봇에게 내리는 개별 명령 명령어 (SR-013)"""
    GOTO = "GOTO"                       # 좌표로 이동
    LEAD_GUEST = "LEAD_GUEST"           # 게스트와 함께 이동
    PICKUP = "PICKUP"                   # 물품 집기/적재 대기
    DROPOFF = "DROPOFF"                 # 물품 내려놓기/수령 대기
    DISPLAY_TEXT = "DISPLAY_TEXT"       # LCD 텍스트 표시
    PLAY_SOUND = "PLAY_SOUND"           # 부저/음성 출력
    STOP = "STOP"                       # 긴급 정지

# ==========================================
# 3. 데이터 스키마 (Pydantic Models)
# ==========================================

class TaskBase(BaseModel):
    task_type: TaskType
    target_location_name: Optional[str] = None
    priority: int = 3
    details: Dict[str, Any] = Field(default_factory=dict)

class TaskCreate(TaskBase):
    requester_id: int
    destination_id: Optional[int] = None

class Task(TaskBase):
    id: int
    requester_id: int
    assigned_robot_id: Optional[int] = None
    status: TaskStatus
    created_at: datetime
    completed_at: Optional[datetime] = None

    class Config:
        from_attributes = True

class ActionCommand(BaseModel):
    """로봇에게 전달되는 개별 액션 구조"""
    action: RobotActionType
    params: Dict[str, Any] = Field(default_factory=dict)
