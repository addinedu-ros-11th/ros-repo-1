from typing import List, Dict, Any
from pydantic import BaseModel
from .base_repository import BaseRepository

class AdminDashboardModel(BaseModel):
    # 조인 결과를 담기 위한 임시 ID 모델
    id: int

class MySQLAdminRepository(BaseRepository):
    def __init__(self):
        # 기본 테이블을 Locations로 설정하지만, 복잡한 조인 쿼리를 주로 사용함
        super().__init__(table_name="Locations", model=AdminDashboardModel)

    async def get_meeting_room_status(self) -> List[Dict[str, Any]]:
        query = """
            SELECT l.name, 
                   CASE WHEN t.status IN ('MOVING', 'GUIDING') THEN '사용 중' ELSE '비어 있음' END as status,
                   COALESCE(u.name, '-') as user,
                   COALESCE(DATE_FORMAT(t.created_at, '%%H:%%i'), '-') as time
            FROM Locations l
            LEFT JOIN Tasks t ON l.location_id = t.destination_id AND t.status != 'COMPLETED'
            LEFT JOIN Users u ON t.requester_id = u.user_id
            WHERE l.type = 'MEETING_ROOM'
        """
        return await self._execute(query, fetch="all")

    async def get_visitor_dashboard_data(self) -> List[Dict[str, Any]]:
        query = """
            SELECT v.visitor_id as id, v.name as visitor, v.purpose, 
                   DATE_FORMAT(v.visit_date, '%%Y-%%m-%%d') as date, 
                   DATE_FORMAT(v.visit_time, '%%H:%%i') as time, 
                   u.name as host, v.status
            FROM Visitors v
            JOIN Users u ON v.host_id = u.user_id
            ORDER BY v.visit_date DESC, v.visit_time DESC
        """
        return await self._execute(query, fetch="all")