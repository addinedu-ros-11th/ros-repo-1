from typing import List, Dict, Any
from fastapi import APIRouter, Depends, HTTPException, Cookie # Cookie 추가
from pydantic import BaseModel
from datetime import datetime
from PIL import Image
import yaml
import io
from fastapi import Response
import os
import numpy as np

from main_server.container import container
# 기존 리포지토리 및 신규 리포지토리 임포트
from main_server.infrastructure.database.repositories.mysql_robot_repository import MySQLRobotRepository
from main_server.infrastructure.database.repositories.mysql_location_repository import MySQLLocationRepository
from main_server.infrastructure.database.repositories.mysql_admin_repository import MySQLAdminRepository
from main_server.infrastructure.database.repositories.mysql_product_repository import MySQLProductRepository
from main_server.infrastructure.database.repositories.mysql_log_repository import MySQLLogRepository

async def verify_admin(user_role: str = Cookie(None)):
    if user_role != "ADMIN":
        raise HTTPException(
            status_code=403, 
            detail="관리자 권한이 없습니다."
        )

# 라우터 전체에 이 문지기(dependencies)를 적용
router = APIRouter(
    prefix="/api/v1/admin",
    tags=["Admin/Control"],
    dependencies=[Depends(verify_admin)] # 모든 admin API 접근 전 실행됨
)

# ---------------------------------------------------------
# 0. 지도
# ---------------------------------------------------------
MAP_DIR = "./main_server/domains/map/"

@router.get("/map/image")
async def get_map_image():
    # 파일명은 hawkes1.png로 고정해서 테스트
    test_file_path = os.path.join(MAP_DIR, "map.png") 
    
    # 서버 로그에서 실제 경로를 확인해보기 위한 출력 (터미널 확인용)
    print(f"Checking file at: {test_file_path}")

    if not os.path.exists(test_file_path):
        raise HTTPException(status_code=404, detail=f"File not found at {test_file_path}")

    with open(test_file_path, "rb") as f:
        content = f.read()
        
    # 확장자에 따라 media_type 자동 지정
    return Response(content=content, media_type="image/png")

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
    # 1. 리포지토리에서 각각의 목록을 가져옴
    pending_raw = await container.reservation_repository.get_pending_list() # status='PENDING'만 조회
    approved_raw = await container.reservation_repository.get_approved_list() # status='APPROVED'만 조회
    
    # 2. 프론트엔드 필드명에 맞게 매핑
    def transform(res_list):
        return [{
            "id": r['id'],
            "visitor": r['visitor_name'],
            "purpose": r['purpose'],
            "date": str(r['visit_date']),
            "time": str(r['visit_time']) if r.get('visit_time') else "-",
            "host": r['manager_name'],
            "status": r['status']
        } for r in res_list]

    return {
        "pending": transform(pending_raw),
        "confirmed": transform(approved_raw)
    }

@router.post("/reservations/decision")
async def decide_reservation(request_data: Dict[str, Any]):
    try:
        # 1. ID를 반드시 정수(int)로 변환 (리포지토리 요구사항)
        res_id = int(request_data.get("id"))
        status = request_data.get("status")

        # 2. 리포지토리 호출
        await container.reservation_repository.update_status(res_id, status)
        
        return {"status": "success", "updated_id": res_id}
    except Exception as e:
        # 에러 발생 시 서버 터미널에서 확인 가능하도록 출력
        print(f"!!! DB 업데이트 실패 원인: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ---------------------------------------------------------
# 3. 시스템 동작 로그 (가독성 좋게 포맷팅)
# ---------------------------------------------------------
@router.get("/logs")
async def get_system_logs():
    """로그 데이터를 시간순으로 정렬하고 UI 규격에 맞춰 반환"""
    logs = await container.log_repository.get_recent_system_logs(limit=50)
    return logs

# ---------------------------------------------------------
# 4. 로봇 실시간 관제 (1초마다 호출용)
# ---------------------------------------------------------
@router.get("/robots/telemetry")
async def get_robots_telemetry():
    """기존 리포지토리를 사용하여 로봇 위치 정보 반환"""
    # 1. 모든 로봇 정보 가져오기 (이미 robot_repo가 주입되어 있음)
    robots = await container.robot_repo.get_all()
    
    # 2. 새로운 mymap.yaml 기반 설정값 (정밀지도 버전)
    RESOLUTION = 0.020
    ORIGIN_X = -2.283
    ORIGIN_Y = -2.550
    IMG_H = 255  # 원본 pgm 세로 픽셀

    processed_robots = []
    for r in robots:
        # r.pose_x, r.pose_y는 FleetManager가 갱신해주는 실제 미터 좌표입니다.
        # 이를 지도 이미지의 픽셀 좌표로 변환합니다.
        raw_px = (r.pose_x - ORIGIN_X) / RESOLUTION
        raw_py = IMG_H - ((r.pose_y - ORIGIN_Y) / RESOLUTION)

        processed_robots.append({
            "id": r.id,
            "name": r.name,
            "status": r.status.value if hasattr(r.status, 'value') else r.status,
            "battery": r.battery_level,
            "px": raw_px,  # 원본 픽셀 좌표
            "py": raw_py
        })
    
    return processed_robots

# ---------------------------------------------------------
# 5. 금지구역 관리
# ---------------------------------------------------------
@router.post("/zones")
async def add_zone(zone_data: Dict[str, Any]):
    try:
        # 1. DB 저장
        await container.location_repo.create_forbidden_zone(zone_data)
        
        # 2. FleetManager 동기화 (전체 리스트를 다시 가져와서 업데이트)
        all_zones = await container.location_repo.get_all_forbidden_zones()
        container.fleet_manager.update_forbidden_zones(all_zones)
        
        return {"status": "success"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/zones")
async def get_zones():
    # 리포지토리에서 DB 데이터를 가져옴
    zones = await container.location_repo.get_all_forbidden_zones()
    return zones

@router.delete("/zones/{zone_id}")
async def delete_zone(zone_id: int):
    try:
        # 1. DB 삭제
        success = await container.location_repo.delete_forbidden_zone(zone_id)
        
        if success:
            # 2. FleetManager 동기화 (삭제 후 남은 리스트를 다시 전달)
            all_zones = await container.location_repo.get_all_forbidden_zones()
            container.fleet_manager.update_forbidden_zones(all_zones)
            return {"status": "success"}
        else:
            raise HTTPException(status_code=404, detail="구역을 찾을 수 없습니다.")
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))