from typing import List, Dict, Any
from fastapi import APIRouter, Depends, HTTPException
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

router = APIRouter(
    prefix="/api/v1/admin",
    tags=["Admin/Control"],
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

# --- admin_routes.py ---

# ---------------------------------------------------------
# 4. 로봇 실시간 관제 (1초마다 호출용)
# ---------------------------------------------------------
# @router.get("/robots/status")
# async def get_all_robots_status():
#     """백엔드에서 캔버스 좌표 및 상태 배지까지 계산해서 반환"""
#     robots = await container.robot_repo.get_all()
    
#     processed_robots = []
#     for r in robots:
#         processed_robots.append({
#             "id": r.id,
#             "name": r.name,
#             "status": r.status,
#             # UI에서 조건문 없이 바로 쓸 수 있도록 배지 색상 결정
#             "status_color": "red" if r.status == "WORKING" else "green",
#             "battery": f"{r.battery}%",
#             # 지도 픽셀 좌표 변환 로직이 있다면 여기서 계산 후 전송 가능
#             "pos_x": r.x, 
#             "pos_y": r.y
#         })
#     return processed_robots

# # ---------------------------------------------------------
# # 5. 금지구역 관리 (1초마다 호출용)
# # ---------------------------------------------------------
# @router.get("/zones")
# async def get_forbidden_zones():
#     """프론트엔드 테이블과 지도 렌더링을 위한 구역 데이터"""
#     zones = await container.location_repo.get_forbidden_zones()
    
#     processed_zones = []
#     for z in zones:
#         processed_zones.append({
#             "id": z.id,
#             "name": z.name,
#             # 테이블용 텍스트 가공을 백엔드에서 수행
#             "display_coords": f"({z.x1}, {z.y1}) → ({z.x2}, {z.y2})"
#         })
#     return processed_zones