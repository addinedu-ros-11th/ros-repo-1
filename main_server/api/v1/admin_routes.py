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
    # 1. 파일 경로 설정 (mymap.pgm)
    file_path = os.path.join(MAP_DIR, "mymap.pgm")
    
    if not os.path.exists(file_path):
        raise HTTPException(status_code=404, detail="Map file not found")

    # 2. PGM 파일 열기
    with Image.open(file_path) as img:
        # numpy 배열로 변환하여 분석
        data = np.array(img)
        
        # 3. 유효한 영역(알 수 없는 영역 205, 254 제외) 찾기
        # 0(벽) 또는 255(길)인 부분의 인덱스를 찾습니다.
        mask = (data == 0) | (data == 255)
        coords = np.argwhere(mask)

        if coords.size > 0:
            # 데이터가 있는 최소/최대 좌표 계산 (Crop 범위)
            y_min, x_min = coords.min(axis=0)
            y_max, x_max = coords.max(axis=0)
            
            # 약간의 여백(padding) 추가
            padding = 5
            y_min = max(0, y_min - padding)
            x_min = max(0, x_min - padding)
            y_max = min(data.shape[0], y_max + padding)
            x_max = min(data.shape[1], x_max + padding)
            
            # 이미지 자르기
            img = img.crop((x_min, y_min, x_max, y_max))

        # 4. PNG로 변환하여 메모리에 저장
        img_byte_arr = io.BytesIO()
        img.save(img_byte_arr, format='PNG')
        img_byte_arr = img_byte_arr.getvalue()

    # 5. 브라우저가 인식할 수 있도록 반환
    return Response(content=img_byte_arr, media_type="image/png")

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