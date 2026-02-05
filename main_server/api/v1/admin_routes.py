from typing import List
from fastapi import APIRouter, Depends

from main_server.domains.robots.robot import Robot
from main_server.core_layer.fleet_management.fleet_manager import FleetManager
from main_server.container import container

# FastAPI 라우터 생성
router = APIRouter(
    prefix="/api/v1/admin",
    tags=["Admin"],
)

@router.get("/robots/status", response_model=List[Robot])
async def get_all_robots_status(
    fleet_manager: FleetManager = Depends(lambda: container.fleet_manager)
):
    """
    모든 로봇의 현재 상태를 데이터베이스에서 조회합니다. (SR-011)
    """
    all_robots_status = await fleet_manager.get_all_robot_status()
    return all_robots_status

@router.get('/logs')
def get_system_logs():
    """
    시스템 로그를 조회합니다. (SR-020)
    """
    # TODO: LogRepository를 만들어 로그 조회 기능 구현
    return {"message": "Log feature not implemented."}
