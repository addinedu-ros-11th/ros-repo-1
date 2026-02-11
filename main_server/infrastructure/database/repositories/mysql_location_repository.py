import logging
from typing import List, Optional, Dict, Any
from main_server.infrastructure.database.connection import Database
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
        """
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                # 간단한 부분 일치 검색 또는 완전 일치 검색
                sql = "SELECT * FROM Locations WHERE name = %s LIMIT 1"
                await cur.execute(sql, (name,))
                result = await cur.fetchone()
                
                if not result:
                    # 부분 일치로 재시도 (예: "회의실" -> "%회의실%")
                    sql = "SELECT * FROM Locations WHERE name LIKE %s LIMIT 1"
                    await cur.execute(sql, (f"%{name}%",))
                    result = await cur.fetchone()
                
                return result

    async def get_all_locations(self) -> List[Dict[str, Any]]:
        """
        모든 등록된 장소 정보를 가져옵니다.
        """
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                sql = "SELECT * FROM Locations"
                await cur.execute(sql)
                return await cur.fetchall()
