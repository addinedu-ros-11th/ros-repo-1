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
# 1. 사무실 관리 (회의실 & 간식 현황)
# ---------------------------------------------------------
@router.get("/office-status")
async def get_office_status():
    # 수정: location_repo가 아닌 admin_repo를 사용
    rooms = await container.admin_repository.get_meeting_room_status()
    snacks = await container.product_repository.get_snack_inventory()
    
    return {
        "rooms": rooms,
        "snacks": snacks
    }

# ---------------------------------------------------------
# 2. 방문객 예약 관리 (대기 & 확정 목록)
# ---------------------------------------------------------
@router.get("/visitors")
async def get_visitor_management():
    # 수정: visitor 전용 대신 통합 관리하는 admin_repo 사용
    all_visitors = await container.admin_repository.get_visitor_dashboard_data()
    
    pending = [v for v in all_visitors if v['status'] == 'PENDING']
    confirmed = [v for v in all_visitors if v['status'] != 'PENDING']
    
    return {
        "pending": pending,
        "confirmed": confirmed
    }

# ---------------------------------------------------------
# 3. 시스템 동작 로그 (SR-019)
# ---------------------------------------------------------
@router.get("/logs")
async def get_system_logs():
    """
    System_Logs 및 Robots 테이블을 연동하여 최신 로그를 반환합니다. (SR-018 관련)
    """
    # MySQLLogRepository(신규)에서 최신 로그 100개 조회
    logs = await container.log_repository.get_recent_system_logs(limit=100)
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