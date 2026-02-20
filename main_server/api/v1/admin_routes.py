from typing import List, Dict, Any
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from datetime import datetime

from main_server.container import container
# 기존 리포지토리 및 신규 리포지토리 임포트
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_admin_repository import MySQLAdminRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository
from main_server.infrastructure.database.repositories.mysql_log_repository import MySQLLogRepository

router = APIRouter(
    prefix="/api/v1/admin",
    tags=["Admin/Control"],
)

# ---------------------------------------------------------
# 1. 사무실 관리 (UI가 바로 렌더링할 수 있게 가공)
# ---------------------------------------------------------
@router.get("/office-status")
async def get_office_status():
    """프론트엔드 테이블에 바로 꽂을 수 있는 형태로 데이터를 정제해서 반환"""
    # DB에서 원본 데이터 가져오기
    raw_rooms = await container.admin_repository.get_meeting_room_status()
    raw_snacks = await container.product_repository.get_snack_inventory()
    
    # 백엔드에서 미리 UI용으로 가공 (데이터 정제 로직)
    processed_rooms = []
    for r in raw_rooms:
        processed_rooms.append({
            "name": r['name'],
            "status": r['status'],  # '사용 중' or '비어 있음'
            "user": r.get('user', '-'),
            "time": r.get('time', '-')
        })

    return {
        "rooms": processed_rooms,
        "snacks": raw_snacks  # SnackRepo에서 이미 가공됨
    }

# ---------------------------------------------------------
# 2. 방문객 예약 관리 (PENDING과 나머지를 백엔드에서 분리)
# ---------------------------------------------------------
@router.get("/visitors")
async def get_visitor_management():
    """프론트에서 필터링할 필요 없게 아예 나눠서 전달"""
    all_visitors = await container.admin_repository.get_visitor_dashboard_data()
    
    # 백엔드에서 비즈니스 로직 처리 (상태별 분류)
    return {
        "pending": [v for v in all_visitors if v['status'] == 'PENDING'],
        "confirmed": [v for v in all_visitors if v['status'] != 'PENDING']
    }

# ---------------------------------------------------------
# 3. 시스템 동작 로그 (가독성 좋게 포맷팅)
# ---------------------------------------------------------
@router.get("/logs")
async def get_system_logs():
    """로그 데이터를 시간순으로 정렬하고 UI 규격에 맞춰 반환"""
    logs = await container.log_repository.get_recent_system_logs(limit=50)
    return logs

# ---------------------------------------------------------
# 4. 로봇 실시간 관제 (후순위 유지)
# ---------------------------------------------------------
@router.get("/robots/status")
async def get_all_robots_status(
    robot_repo: MySQLRobotRepository = Depends(lambda: container.robot_repository)
):
    """로봇의 위치, 배터리, 상태 정보를 실시간으로 반환합니다. (SR-017, SR-018)"""
    return await robot_repo.get_all()