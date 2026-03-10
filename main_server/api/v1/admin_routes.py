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
import uuid

from main_server.config import config
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

@router.get("/map/image")
async def get_map_image():
    # config에서 설정된 YAML 파일 이름을 기반으로 PNG 파일을 찾습니다.
    # (PGM은 브라우저에서 직접 표시할 수 없으므로 같은 이름의 PNG가 있다고 가정)
    yaml_filename = os.path.basename(config.MAP_YAML_PATH)
    base_name = os.path.splitext(yaml_filename)[0] # 'office_map' or 'mymap'
    
    # 1순위: YAML과 이름이 같은 PNG (office_map.png 등)
    # 2순위: 기본 map.png
    image_path = os.path.join(config.MAP_DIR, f"{base_name}.png")
    
    if not os.path.exists(image_path):
        image_path = os.path.join(config.MAP_DIR, "map.png")

    if not os.path.exists(image_path):
        raise HTTPException(status_code=404, detail=f"Map image not found in {config.MAP_DIR}")

    with open(image_path, "rb") as f:
        content = f.read()
        
    return Response(content=content, media_type="image/png")

# ---------------------------------------------------------
# 1. 사무실 관리 (UI가 바로 렌더링할 수 있게 가공)
# ---------------------------------------------------------

@router.get("/office-status")
async def get_office_status():
    """프론트엔드 테이블에 회의실(5,6번) 및 간식 현황 반환"""
    
    # 1. 리포지토리를 통해 회의실 5, 6번 데이터 가져오기
    # (container 설정에 따라 location_repo 접근 방식 확인 필요)
    raw_rooms = await container.location_repo.get_admin_meeting_room_status()
    raw_snacks = await container.product_repository.get_snack_inventory()
    
    # 2. UI 규격(admin_dashboard.html)에 맞춰 데이터 매핑
    processed_rooms = []
    for r in raw_rooms:
        processed_rooms.append({
            "name": r['room_name'],
            "status": r['status'],  # PENDING, APPROVED 등 DB Enum 값
            "user": r['user_name'],
            "time": r['res_time']
        })

    # 만약 예약이 없는 경우에도 목록에 회의실 이름은 나오게 하고 싶다면 
    # 별도의 '비어 있음' 처리 로직을 추가할 수 있습니다.

    return {
        "rooms": processed_rooms,
        "snacks": raw_snacks 
    }

# ---------------------------------------------------------
# 2. 방문객 예약 관리 (PENDING과 나머지를 백엔드에서 분리)
# ---------------------------------------------------------
@router.get("/visitors")
async def get_visitors():
    # 1. DB에서 데이터 가져오기 (VisitorRepository 사용)
    pending_raw = await container.visitor_repository.get_pending_list()
    approved_raw = await container.visitor_repository.get_approved_list()
    rejected_raw = await container.visitor_repository.get_rejected_list()

    # 2. 정보 매핑 준비 (host_user_id, destination_id)
    user_ids = set()
    loc_ids = set()
    for v in pending_raw + approved_raw:
        if v.host_user_id: user_ids.add(v.host_user_id)
        if v.destination_id: loc_ids.add(v.destination_id)
    
    user_map = {}
    for uid in user_ids:
        user = await container.user_repo.get_by_id(uid)
        if user: user_map[uid] = user.name

    loc_map = {}
    for lid in loc_ids:
        loc = await container.location_repo.get_by_id(lid)
        if loc: loc_map[lid] = loc['name'] if isinstance(loc, dict) else loc.name

    # 3. PENDING 데이터 가공
    pending_data = []
    for v in pending_raw:
        host_name = user_map.get(v.host_user_id, "-")
        dest_name = loc_map.get(v.destination_id, "담당자 위치")
        pending_data.append({
            "id": v.visitor_id,
            "visitor": v.name,
            "purpose": v.purpose,
            "date": str(v.visit_date),
            "host": host_name,
            "destination": dest_name,
            "status": v.status
        })

    # 4. APPROVED 데이터 가공
    confirmed_data = []
    for v in approved_raw:
        host_name = user_map.get(v.host_user_id, "-")
        dest_name = loc_map.get(v.destination_id, "담당자 위치")
        confirmed_data.append({
            "id": v.visitor_id,
            "visitor": v.name,
            "purpose": v.purpose,
            "date": str(v.visit_date),
            "time": str(v.visit_time) if v.visit_time else "-",
            "host": host_name,
            "destination": dest_name,
            "status": v.status
        })

    # 5. REJECTED 데이터 가공
    rejected_data = []
    for v in rejected_raw:
        host_name = user_map.get(v.host_user_id, "-")
        dest_name = loc_map.get(v.destination_id, "담당자 위치")
        rejected_data.append({
            "id": v.visitor_id,
            "visitor": v.name,
            "purpose": v.purpose,
            "date": str(v.visit_date),
            "time": str(v.visit_time) if v.visit_time else "-",
            "host": host_name,
            "destination": dest_name,
            "status": v.status
        })

    return {
        "pending": pending_data,
        "confirmed": confirmed_data,
        "rejected": rejected_data
    }

@router.post("/reservations/decision")
async def decide_reservation(request_data: Dict[str, Any]):
    try:
        res_id = int(request_data.get("id"))
        status = request_data.get("status")

        update_data = {"status": status}

        # [추가] 승인(APPROVED)일 경우 QR 코드 생성 로직 실행
        if status == "APPROVED":
            # 1. 고유한 식별자 생성 (UUID 활용)
            # 형식 예시: VISITOR_ID_UUID (v_12_a1b2c3...)
            unique_token = str(uuid.uuid4())[:13] # 짧고 고유한 토큰 생성
            qr_payload = f"v_{res_id}_{unique_token}"
            
            # 2. 업데이트 데이터에 qr_code 필드 추가
            update_data["qr_code"] = qr_payload

        # 리포지토리 호출 (status와 필요시 qr_code를 함께 업데이트)
        await container.visitor_repository.update(res_id, update_data)
        
        return {
            "status": "success", 
            "updated_id": res_id, 
            "qr_generated": "qr_code" in update_data
        }
    except Exception as e:
        print(f"!!! DB 업데이트 실패 원인: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# ---------------------------------------------------------
# 3. 시스템 동작 로그 (가독성 좋게 포맷팅)
# ---------------------------------------------------------
@router.get("/system-logs")
async def get_system_logs(date: str = None):
    if not date:
        date = datetime.now().strftime('%Y-%m-%d')
    
    logs = await container.log_repository.get_system_task_logs(date)
    
    results = []
    for l in logs:
        results.append({
            "start_time": l['created_at'].strftime('%H:%M:%S') if l['created_at'] else "-",
            "robot_name": l['robot_name'] or "Unknown",
            "robot_status": l['robot_status'] or "IDLE",
            "battery": f"{int(l['battery_level'])}%" if l['battery_level'] is not None else "0%",
            "task_type": l['task_type'],  # 리포지토리에서 가져온 값을 추가
            "task_status": l['task_status'],
            "end_time": l['completed_at'].strftime('%H:%M:%S') if l['completed_at'] else "-"
        })
    return results

# ---------------------------------------------------------
# 4. 지도 메타데이터 및 로봇 실시간 관제
# ---------------------------------------------------------

@router.get("/map/metadata")
async def get_map_metadata():
    """맵 메타데이터 (해상도, 원점 등) 반환"""
    # FleetManager의 PathPlanner에서 로드된 정보를 사용
    planner = container.fleet_manager.path_planner
    return {
        "resolution": planner.resolution,
        "origin_x": planner.origin[0],
        "origin_y": planner.origin[1],
        "width": planner.width,
        "height": planner.height
    }

@router.get("/robots/telemetry")
async def get_robots_telemetry():
    """기존 리포지토리를 사용하여 로봇 위치 정보 반환"""
    # 1. 모든 로봇 정보 가져오기 (이미 robot_repo가 주입되어 있음)
    robots = await container.robot_repo.get_all()
    
    # PathPlanner에서 메타데이터 가져오기
    planner = container.fleet_manager.path_planner
    RESOLUTION = planner.resolution
    ORIGIN_X = planner.origin[0]
    ORIGIN_Y = planner.origin[1]
    IMG_H = planner.height

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
            "py": raw_py,
            "current_task_id": r.current_task_id
        })
    
    return processed_robots

# ---------------------------------------------------------
# 5. 금지구역 관리
# ---------------------------------------------------------
@router.post("/zones")
async def add_zone(zone_data: Dict[str, Any]):
    try:
        # 1. DB 저장 (리포지토리 구현에 따라 create_forbidden_zone이 없을 수도 있으니 주의)
        # 만약 없다면 location_repo에 추가 필요. 일단 있다고 가정하고 진행.
        await container.location_repo.create_forbidden_zone(zone_data)
        
        # 2. FleetManager 동기화 (전체 리스트를 다시 가져와서 업데이트)
        # location_repo.get_all_forbidden_zones()가 DB에서 목록을 반환한다고 가정
        all_zones = await container.location_repo.get_all_forbidden_zones()
        await container.fleet_manager.update_forbidden_zones(all_zones)
        
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
        # location_repo.delete_forbidden_zone(zone_id)가 있다고 가정
        success = await container.location_repo.delete_forbidden_zone(zone_id)
        
        if success:
            # 2. FleetManager 동기화 (삭제 후 남은 리스트를 다시 전달)
            all_zones = await container.location_repo.get_all_forbidden_zones()
            await container.fleet_manager.update_forbidden_zones(all_zones)
            return {"status": "success"}
        else:
            raise HTTPException(status_code=404, detail="구역을 찾을 수 없습니다.")
            
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))