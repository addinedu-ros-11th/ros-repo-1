import logging
from typing import List, Optional, Dict, Any
from main_server.infrastructure.database.connection import Database
from main_server.domains.map.location import LocationName, Pose, WAYPOINTS
import aiomysql

logger = logging.getLogger(__name__)

class MySQLLocationRepository:
    """
    MySQL 데이터베이스를 사용하여 POI(Point of Interest) 위치 정보를 관리하는 리포지토리.
    """
    def __init__(self):
        pass

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

    async def get_all_locations(self) -> List[Dict[str, Any]]:
        """
        모든 등록된 장소 정보를 가져옵니다.
        """
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                sql = "SELECT * FROM Locations"
                await cur.execute(sql)
                return await cur.fetchall()
