from typing import List
from fastapi import APIRouter, Depends, HTTPException, Body
from pydantic import BaseModel

from main_server.domains.robots.schemas import Robot, RobotStatus
from main_server.services.fleet_management.fleet_manager import FleetManager
from main_server.container import container

# FastAPI 라우터 생성
router = APIRouter(
    prefix="/api/v1/admin",
    tags=["Admin/Control"], # 관제 기능을 위해 태그를 조금 더 명확히 함
)

# [추가] 금지 구역 설정을 위한 데이터 스키마
class ForbiddenZoneCreate(BaseModel):
    account: str   # 등록자 아이디
    left: int      # 시작 X
    top: int       # 시작 Y
    width: int     # 가로 길이
    height: int    # 세로 길이
    reason: str    # 설정 사유 (예: "청소 구역", "위험 지역")

# 1. 모든 로봇 상태 조회 (기존 유지)
@router.get("/robots/status", response_model=List[Robot])
async def get_all_robots_status(
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    """모든 로봇의 현재 상태(좌표 포함)를 조회합니다. (SR-011)"""
    return await fleet_manager.get_all_robot_status()

# 2. 로봇 실시간 좌표 업데이트 (관제용 추가)
@router.post("/robots/{robot_id}/location")
async def update_robot_location(
    robot_id: int, 
    x: float, 
    y: float,
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    """로봇이 이동할 때마다 실시간으로 DB에 좌표를 찍어줍니다."""
    # 리포지토리의 update_location을 직접 호출하거나 FleetManager에 메서드 추가 필요
    await fleet_manager.robot_repo.update_location(robot_id, x, y)
    return {"status": "success", "robot_id": robot_id, "new_pos": [x, y]}

# 3. 금지 구역 설정 (관제 지도에서 그린 구역 저장)
@router.post("/forbidden-zones")
async def set_forbidden_zone(
    zone: ForbiddenZoneCreate,
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    # 실제 구현 시 fleet_manager.location_repository.create_zone(zone) 호출
    return {"status": "success", "message": "금지 구역이 저장되었습니다.", "zone": zone}

@router.get("/forbidden-zones")
async def get_all_forbidden_zones(
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    # zones = await fleet_manager.location_repository.get_all_zones()
    return [] # 임시 반환

@router.delete("/forbidden-zones/{zone_id}")
async def delete_forbidden_zone(
    zone_id: int,
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    # await fleet_manager.location_repository.delete_zone(zone_id)
    return {"status": "success", "message": f"구역 {zone_id} 삭제 완료"}

# 4. 시스템 로그 조회 (구현 업데이트)
@router.get('/logs')
async def get_system_logs(
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    """시스템 로그 및 로봇 텔레메트리 로그를 조회합니다. (SR-020)"""
    # TODO: fleet_manager 내부에 log_repository 연결 필요
    return {"message": "로그 조회 기능은 LogRepository 연동 후 활성화됩니다."}

#  @app 수정해야함. db를 안정했음.
# @app.get("/api/office-status")
# async def get_office_status():
#     # 실제 환경에서는 DB에서 조회 (SELECT * FROM rooms...)
#     meeting_rooms = [
#         {"name": "제 1 회의실", "is_occupied": True, "user": "김철수", "time": "14:00 - 16:00"},
#         {"name": "제 2 회의실", "is_occupied": False, "user": "-", "time": "-"}
#     ]
#     snacks = [
#         {"name": "캡슐 커피", "quantity": 45, "status": "충분"},
#         {"name": "생수", "quantity": 5, "status": "부족"}
#     ]
#     return {"rooms": meeting_rooms, "snacks": snacks}