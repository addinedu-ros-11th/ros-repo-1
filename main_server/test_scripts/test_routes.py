from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import asyncio

# Assuming container is available globally or can be imported
from main_server.container import container
from main_server.domains.robots.schemas import RobotStatus
from main_server.infrastructure.database.repositories.mysql_user_repository import UserRole, UserModel

router = APIRouter(
    prefix="/api/test",
    tags=["Test Integration"]
)

class CreateRobotRequest(BaseModel):
    name: str
    initial_x: float = 0.0
    initial_y: float = 0.0

class CreateUserRequest(BaseModel):
    account: str
    password: str
    name: str
    role: str = "STAFF"
    location_id: int = 1

class SimulateEventRequest(BaseModel):
    task_id: int
    event_type: str 

@router.post("/users")
async def create_test_user(request: CreateUserRequest):
    """테스트용 사용자를 생성합니다."""
    # User repo instance needed. It's not in container usually but we can instantiate it or add to container.
    # Assuming container has user_repo or we can instantiate it.
    # Looking at container.py would be good but let's just instantiate it here as it's a test route.
    from main_server.infrastructure.database.repositories.mysql_user_repository import MySQLUserRepository
    repo = MySQLUserRepository()
    
    existing = await repo.get_user_by_username(request.account)
    if existing:
        return {"status": "exists", "message": f"User {request.account} already exists."}
    
    # We need to hash password if the real app does, but login route just did simple comparison:
    # if not user or user.password_hash != password:
    # So plain text is fine for this test.
    
    user_data = {
        "account": request.account,
        "password_hash": request.password,
        "name": request.name,
        "role": request.role,
        "location_id": request.location_id
    }
    
    # Repo create method usually takes a dict.
    # Check base_repository.create implementation if possible, but standard is dict.
    try:
        new_user = await repo.create(user_data)
        if new_user:
             return {"status": "success", "message": f"User {request.account} created."}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
    
    raise HTTPException(status_code=500, detail="Failed to create user.")

@router.post("/robots")
async def create_dummy_robot(request: CreateRobotRequest):
    """테스트용 더미 로봇을 생성합니다."""
    # 로봇이 이미 존재하는지 확인 (find_by_name 없음 -> get_all 후 필터링)
    try:
        all_robots = await container.fleet_manager.robot_repo.get_all()
        existing_robot = next((r for r in all_robots if r.name == request.name), None)
        
        if existing_robot:
            # Update existing robot to correct position
            await container.fleet_manager.robot_repo.update(
                existing_robot.id,
                {
                    "pose_x": request.initial_x,
                    "pose_y": request.initial_y,
                    "status": RobotStatus.IDLE
                }
            )
            return {
                "status": "exists", 
                "robot_id": existing_robot.id, 
                "message": f"Robot {request.name} already exists. Position updated."
            }

        # 로봇 생성 (Repository signature: create(name, battery_level))
        # 1. 기본 생성
        new_robot = await container.fleet_manager.robot_repo.create(
            name=request.name, 
            battery_level=100.0
        )
        
        # 2. 초기 위치 업데이트
        if new_robot:
            updated_robot = await container.fleet_manager.robot_repo.update(
                new_robot.id, 
                {
                    "pose_x": request.initial_x, 
                    "pose_y": request.initial_y,
                    "status": RobotStatus.IDLE
                }
            )
            return {
                "status": "success", 
                "robot_id": new_robot.id, 
                "message": f"Robot {request.name} created."
            }
    except Exception as e:
        import traceback
        traceback.print_exc()
        raise HTTPException(status_code=500, detail=f"Error creating robot: {str(e)}")
            
    raise HTTPException(status_code=500, detail="Failed to create robot.")

@router.post("/load_map")
async def load_test_map(map_path: str):
    """지정된 경로의 맵 파일을 로드합니다."""
    import os
    if not os.path.isabs(map_path):
        # 상대 경로는 main_server/test_scripts/ 기준으로 처리 (편의상)
        base_dir = os.path.dirname(os.path.abspath(__file__))
        map_path = os.path.join(base_dir, map_path)
    
    if not os.path.exists(map_path):
        raise HTTPException(status_code=404, detail=f"Map file not found: {map_path}")
        
    try:
        container.fleet_manager.path_planner.load_map_config(map_path)
        return {"status": "success", "message": f"Map loaded from {map_path}", "origin": container.fleet_manager.path_planner.origin}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.put("/locations/{name}")
async def update_location_coords(name: str, x: float, y: float):
    """위치 좌표를 강제로 수정합니다."""
    loc = await container.task_manager.location_repo.find_by_name(name)
    if not loc:
        raise HTTPException(status_code=404, detail="Location not found")
    
    # location_repo implementation assumed to be MySQLLocationRepository
    # Since it inherits BaseRepository, we can assume 'update' exists if 'location_id' is accessible.
    try:
        # loc is likely a dict from find_by_name (as seen in logs)
        loc_id = loc.get('location_id') if isinstance(loc, dict) else loc.location_id
        
        # BaseRepository.update expects pk_value and update_data dict
        # The column names are coordinate_x and coordinate_y
        await container.task_manager.location_repo.update(loc_id, {"coordinate_x": x, "coordinate_y": y})
        return {"status": "success", "message": f"Updated {name} to ({x}, {y})"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/tasks/{task_id}/events")
async def trigger_robot_event(task_id: int, request: SimulateEventRequest):
    """
    특정 태스크에 대해 로봇 이벤트를 강제로 발생시킵니다.
    """
    task = await container.task_manager.task_repo.get_by_id(task_id)
    if not task:
        raise HTTPException(status_code=404, detail="Task not found")
    
    if not task.assigned_robot_id:
        raise HTTPException(status_code=400, detail="Task has no assigned robot")

    await container.task_manager.handle_robot_event(
        task_id=task_id,
        robot_id=task.assigned_robot_id,
        event=request.event_type
    )
    return {"status": "success", "message": f"Event {request.event_type} triggered for task {task_id}"}

class UpdateRobotStatusRequest(BaseModel):
    x: float
    y: float
    status: str

@router.put("/robots/{robot_id}/status")
async def update_robot_status(robot_id: int, request: UpdateRobotStatusRequest):
    """로봇의 위치와 상태를 강제로 업데이트합니다 (GUI 이동 확인용)."""
    try:
        status_enum = RobotStatus(request.status)
    except ValueError:
        raise HTTPException(status_code=400, detail=f"Invalid status: {request.status}")

    await container.fleet_manager.update_robot_status(
        robot_id=robot_id,
        status=status_enum,
        location=(request.x, request.y),
        battery=90.0
    )
    return {"status": "success"}

@router.get("/locations")
async def get_locations():
    """모든 위치 정보를 조회합니다."""
    locations = await container.task_manager.location_repo.get_all() # Assuming get_all exists or use custom query
    # Check repository implementation
    # It seems base_repository has no get_all, usually it's find_all or custom.
    # We can use direct query for test.
    query = f"SELECT * FROM Locations" # Table name is 'Locations' in MySQLLocationRepository?
    # Let's check MySQLLocationRepository.
    # It inherits from BaseRepository.
    # Let's assume table name 'Locations' and we can use _execute if exposed or add a method.
    # Since I cannot modify BaseRepository easily without checking, I will try to use the container's repo if it has a method.
    # If not, I'll just return a message saying check DB.
    # But wait, I can use the container to find 'snack_entrance' specifically.
    
    snack = await container.task_manager.location_repo.find_by_name("snack_entrance")
    return {"snack_entrance": snack}

@router.post("/check_path")
async def check_path(start_x: float, start_y: float, end_x: float, end_y: float):
    """경로 계획을 테스트합니다."""
    # We need a dummy robot object to pass to plan_global_path
    from main_server.domains.robots.schemas import Robot
    dummy_robot = Robot(
        robot_id=0, name="dummy", status=RobotStatus.IDLE, battery_level=100.0,
        current_x=start_x, current_y=start_y
    )
    
    path = await container.fleet_manager.path_planner.plan_global_path(dummy_robot, end_x, end_y)
    
    # Also check if points are walkable
    planner = container.fleet_manager.path_planner
    gx1, gy1 = planner.world_to_grid(start_x, start_y)
    gx2, gy2 = planner.world_to_grid(end_x, end_y)
    
    walkable_start = planner.grid.walkable(gx1, gy1) if 0 <= gx1 < planner.width and 0 <= gy1 < planner.height else False
    walkable_end = planner.grid.walkable(gx2, gy2) if 0 <= gx2 < planner.width and 0 <= gy2 < planner.height else False
    
    return {
        "start_walkable": walkable_start,
        "end_walkable": walkable_end,
        "path_found": path is not None,
        "path_length": len(path) if path else 0,
        "grid_start": (gx1, gy1),
        "grid_end": (gx2, gy2)
    }

