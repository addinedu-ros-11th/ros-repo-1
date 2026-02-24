import logging
from typing import List, Optional, Dict, Any
from pydantic import BaseModel
from main_server.infrastructure.database.connection import Database
from main_server.domains.map.location import LocationName, Pose, WAYPOINTS
from .base_repository import BaseRepository
import aiomysql

logger = logging.getLogger(__name__)

class LocationModel(BaseModel):
    location_id: int
    name: str
    type: Optional[str] = None
    coordinate_x: float
    coordinate_y: float
    theta: float
    is_restricted: int = 0

class MySQLLocationRepository(BaseRepository):
    """
    MySQL 데이터베이스를 사용하여 POI(Point of Interest) 위치 정보를 관리하는 리포지토리.
    """
    def __init__(self):
        super().__init__(table_name="Locations", model=LocationModel, pk_name="location_id")

    async def find_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        """
        장소 이름을 기반으로 좌표 정보를 조회합니다.
        DB에 없을 경우 location.py의 WAYPOINTS에서 보완합니다.
        """
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                # 1. DB에서 완전 일치 검색
                sql = "SELECT * FROM Locations WHERE name = %s LIMIT 1"
                await cur.execute(sql, (name,))
                result = await cur.fetchone()
                
                if not result:
                    # 2. DB에서 부분 일치 검색
                    sql = "SELECT * FROM Locations WHERE name LIKE %s LIMIT 1"
                    await cur.execute(sql, (f"%{name}%",))
                    result = await cur.fetchone()
                
                if result:
                    return result

        # 3. DB에 없을 경우 하드코딩된 WAYPOINTS 확인 (Fallback)
        try:
            # name이 Enum의 value나 name과 일치하는지 확인
            for loc_enum, pose in WAYPOINTS.items():
                if name.lower() in [loc_enum.name.lower(), loc_enum.value.lower()]:
                    return {
                        "location_id": 0,
                        "name": loc_enum.value,
                        "coordinate_x": pose.x,
                        "coordinate_y": pose.y,
                        "theta": pose.theta
                    }
        except Exception as e:
            logger.error(f"WAYPOINTS Fallback 조회 중 오류: {e}")

        return None

    async def find_by_id(self, location_id: int) -> Optional[Dict[str, Any]]:
        """ScenarioDataHandler 호환용: ID로 위치 조회"""
        model = await super().get_by_id(location_id)
        if model:
            return model.model_dump() # dict 형태로 반환
        return None

    async def get_all_locations(self) -> List[Dict[str, Any]]:
        """
        모든 등록된 장소 정보를 가져옵니다.
        """
        return await super().get_all()

    async def create_forbidden_zone(self, data: Dict[str, Any]):
        """금지구역을 Map_Zones 테이블에 저장"""
        async with Database.get_connection() as conn:
            async with conn.cursor() as cur:
                sql = """
                    INSERT INTO Map_Zones (name, type, x1, y1, x2, y2, active)
                    VALUES (%s, %s, %s, %s, %s, %s, 1)
                """
                await cur.execute(sql, (
                    data['name'], data.get('type', 'forbidden'),
                    data['x1'], data['y1'], data['x2'], data['y2']
                ))
                await conn.commit()

    async def get_all_forbidden_zones(self) -> List[Dict[str, Any]]:
        """모든 금지구역 리스트 조회"""
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                sql = "SELECT * FROM Map_Zones WHERE active = 1"
                await cur.execute(sql)
                return await cur.fetchall()
            
    async def delete_forbidden_zone(self, zone_id: int) -> bool:
        """금지구역 ID를 기반으로 DB에서 삭제"""
        async with Database.get_connection() as conn:
            async with conn.cursor() as cur:
                sql = "DELETE FROM Map_Zones WHERE zone_id = %s"
                await cur.execute(sql, (zone_id,))
                await conn.commit()
                return cur.rowcount > 0