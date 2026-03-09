from pydantic import BaseModel, Field
from enum import Enum
from typing import Optional
from datetime import datetime

# ==========================================
# 1. 로봇 관련 Enum (상태)
# ==========================================

class RobotStatus(str, Enum):
    """
    로봇의 현재 물리적/논리적 상태 (SR-008, SR-010)
    """
    IDLE = "IDLE"           # 대기 중 (가용 상태)
    WAITING = "WAITING"     # 사용자 응답 대기 중
    ASSIGNED = "ASSIGNED"   # 작업 배정됨
    MOVING = "MOVING"       # 목적지로 이동 중
    GUIDING = "GUIDING"     # 게스트 가이딩 중
    CHARGING = "CHARGING"   # 충전 중
    ERROR = "ERROR"         # 에러 발생
    OFFLINE = "OFFLINE"     # 연결 끊김

class RobotEvent(str, Enum):
    """
    로봇이 서버로 송신하는 주요 이벤트 (결과 보고)
    """
    # 이동 관련
    ARRIVED = "ARRIVED"                         # 일반 목적지 도착
    ARRIVED_AT_DESTINATION = "ARRIVED_AT_DESTINATION" # 최종 목적지 도착
    ARRIVED_AT_BASE = "ARRIVED_AT_BASE"         # 복귀 완료
    
    # 작업 단계별 (시나리오용)
    ARRIVED_AT_PANTRY_ENTRANCE = "ARRIVED_AT_PANTRY_ENTRANCE"
    ARRIVED_AT_SNACK_POINT = "ARRIVED_AT_SNACK_POINT"
    ARRIVED_AT_SENDER = "ARRIVED_AT_SENDER"
    ARRIVED_AT_RECEIVER = "ARRIVED_AT_RECEIVER"
    
    # 하드웨어/센서 결과
    QR_SCANNED = "QR_SCANNED"                   # QR 스캔 완료 (데이터 포함)
    QR_SCAN_FAILED = "QR_SCAN_FAILED"           # QR 스캔 실패
    OBSTACLE_DETECTED = "OBSTACLE_DETECTED"     # 장애물 감지
    BATTERY_LOW = "BATTERY_LOW"                 # 배터리 부족
    
    # 사용자 상호작용 (로봇 Local 확인)
    LOADING_COMPLETE = "LOADING_COMPLETE"       # 물품 적재 완료 (로봇 버튼 등)
    DELIVERY_CONFIRMED = "DELIVERY_CONFIRMED"   # 수령 확인 완료

# ==========================================
# 2. 로봇 데이터 모델 (Schemas)
# ==========================================

class RobotBase(BaseModel):
    name: str = Field(..., description="로봇의 고유 이름 (예: robot01)")
    status: RobotStatus = Field(default=RobotStatus.IDLE)
    battery_level: float = Field(..., description="배터리 잔량 (%)", ge=0, le=100)
    pose_x: Optional[float] = Field(default=0.0, alias="current_x", description="현재 X 좌표")
    pose_y: Optional[float] = Field(default=0.0, alias="current_y", description="현재 Y 좌표")

class RobotCreate(RobotBase):
    """로봇 등록 시 필요한 데이터"""
    pass

class Robot(RobotBase):
    """로봇 상세 정보 모델"""
    id: int = Field(..., alias="robot_id", description="데이터베이스 PK")
    current_task_id: Optional[int] = Field(None, description="현재 할당된 작업 ID")
    last_heartbeat: Optional[datetime] = Field(None, description="마지막 통신 시간")

    class Config:
        from_attributes = True  # Pydantic v2 스타일 (ORM 연동)
        use_enum_values = True
        populate_by_name = True

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
