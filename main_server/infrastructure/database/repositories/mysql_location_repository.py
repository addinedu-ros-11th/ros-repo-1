import logging
import json
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
                # data['polygon']은 [[x1,y1], [x2,y2], ...] 형태여야 함
                # 만약 프론트엔드에서 x1,y1,x2,y2로 준다면 사각형 폴리곤으로 변환
                if 'x1' in data and 'y1' in data:
                    x1, y1 = data['x1'], data['y1']
                    x2, y2 = data['x2'], data['y2']
                    # UI에서 Rect로 그릴 경우 4개 점으로 변환 (픽셀 좌표계)
                    # FleetManager나 로봇이 이해하기 쉬운 형태로 저장
                    polygon = [[x1, y1], [x2, y1], [x2, y2], [x1, y2]]
                    
                    # 동시에 width, height, left, top 같은 메타데이터도 저장하고 싶다면
                    # polygon_data에 함께 넣거나 별도 컬럼이 필요하지만,
                    # 현재 스키마는 polygon_data(Text)만 있으므로 여기에 다 넣거나 표준 포맷 사용
                    polygon_json = json.dumps(polygon)
                    
                    # 추가: PathPlanner가 width/height/left/top을 쓰므로 이를 맞추기 위해
                    # 별도 메타데이터 필드가 없다면 polygon_data를 확장해서 저장하는 방법 고려.
                    # 하지만 지금은 DB 스키마가 단순하므로 polygon_data에 리스트만 저장하고
                    # 조회 시 변환하거나, 아예 data 자체를 통째로 JSON으로 저장하는 것이 나을 수 있음.
                    # 여기서는 data 전체(메타 포함)를 저장하는 것으로 변경하여 유연성 확보.
                    polygon_json = json.dumps(data) 
                else:
                    polygon_json = json.dumps(data.get('polygon', []))

                sql = """
                    INSERT INTO Map_Zones (name, type, polygon_data, active)
                    VALUES (%s, %s, %s, 1)
                """
                await cur.execute(sql, (
                    data.get('name', 'Forbidden Zone'), 
                    data.get('type', 'RESTRICTED'),
                    polygon_json
                ))
                await conn.commit()

    async def get_all_forbidden_zones(self) -> List[Dict[str, Any]]:
        """모든 금지구역 리스트 조회"""
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cur:
                sql = "SELECT zone_id, name, polygon_data FROM Map_Zones WHERE active = 1"
                await cur.execute(sql)
                rows = await cur.fetchall()
                
                zones = []
                for row in rows:
                    p_data = row.get('polygon_data')
                    if p_data:
                        try:
                            # DB에 저장된 JSON 문자열 파싱
                            data = json.loads(p_data)
                            
                            # 기본 데이터 구성 (DB 컬럼 우선)
                            zone_obj = {
                                'id': row['zone_id'],
                                'zone_id': row['zone_id'], # 프론트엔드 호환성
                                'name': row['name']
                            }
                            
                            # JSON 데이터 병합 (좌표 등)
                            if isinstance(data, dict):
                                zone_obj.update(data)
                                # ID와 Name은 DB 컬럼 값으로 강제 덮어쓰기 (무결성 보장)
                                zone_obj['id'] = row['zone_id']
                                zone_obj['zone_id'] = row['zone_id']
                                zone_obj['name'] = row['name']
                                zones.append(zone_obj)
                            else:
                                # 리스트 형태라면 polygon 필드에 할당
                                zone_obj['polygon'] = data
                                zones.append(zone_obj)
                                
                        except json.JSONDecodeError:
                            logger.error(f"존 ID {row['zone_id']} JSON 파싱 실패")
                return zones
            
    async def delete_forbidden_zone(self, zone_id: int) -> bool:
        """금지구역 ID를 기반으로 DB에서 삭제"""
        async with Database.get_connection() as conn:
            async with conn.cursor() as cur:
                sql = "DELETE FROM Map_Zones WHERE zone_id = %s"
                await cur.execute(sql, (zone_id,))
                await conn.commit()
                return cur.rowcount > 0
            
    #room_reservation을 따로 repository를 만들지않고 추가하였음
    async def create_room_reservation(self, user_pk: int, location_id: int, res_date: str, start_t: str, end_t: str):
        """room_reservation 테이블에 예약 기록 저장"""
        query = """
            INSERT INTO room_reservation 
            (user_id, location_id, reservation_date, start_time, end_time)
            VALUES (%s, %s, %s, %s, %s)
        """
        # BaseRepository에 정의된 _execute 메서드를 사용하며, 쓰기 작업이므로 is_write=True 설정
        return await self._execute(query, (user_pk, location_id, res_date, start_t, end_t), is_write=True)
    
    async def get_room_reservations_by_user(self, user_pk: int):
        """특정 유저의 회의실 예약 현황 조회 (Locations 조인 및 status 포함)"""
        query = """
            SELECT 
                r.reservation_id,
                l.name as room_name,
                r.reservation_date,
                r.start_time,
                r.end_time,
                r.status
            FROM room_reservation r
            JOIN Locations l ON r.location_id = l.location_id
            WHERE r.user_id = %s
            ORDER BY r.reservation_date DESC, r.start_time DESC
        """
        # BaseRepository의 _execute는 결과를 Dict 형태로 반환합니다.
        return await self._execute(query, (user_pk,))

    async def cancel_room_reservation(self, res_id: int, user_pk: int):
        """예약 상태를 CANCLE로 변경 (삭제 대신 업데이트)"""
        query = """
            UPDATE room_reservation 
            SET status = 'CANCLE' 
            WHERE reservation_id = %s AND user_id = %s
        """
        return await self._execute(query, (res_id, user_pk), is_write=True)