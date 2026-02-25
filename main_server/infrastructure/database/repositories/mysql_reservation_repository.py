from typing import List, Dict, Any
from .base_repository import BaseRepository
from pydantic import BaseModel
from typing import Optional
from datetime import date, time, datetime

class ReservationModel(BaseModel):
    id: Optional[int] = None
    manager_name: str
    visitor_name: str
    purpose: str
    visit_date: date
    visit_time: Optional[time] = None
    visitor_phone: str = "010-0000-0000"
    status: str = "PENDING"
    created_at: Optional[datetime] = None

class MySQLReservationRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="reservations", model=ReservationModel, pk_name="id")

    async def get_pending_list(self) -> List[Dict[str, Any]]:
        """승인 대기 중인 목록 조회 (신규 신청)"""
        query = "SELECT * FROM reservations WHERE status = 'PENDING' ORDER BY created_at DESC"
        return await self._execute(query, fetch="all")

    async def get_approved_list(self) -> List[Dict[str, Any]]:
        """최종 예약 확정 현황 조회"""
        query = "SELECT * FROM reservations WHERE status = 'APPROVED' ORDER BY visit_date ASC"
        return await self._execute(query, fetch="all")

    async def update_status(self, res_id: int, status: str):
        """승인 또는 반려 처리"""
        query = "UPDATE reservations SET status = %s WHERE id = %s"
        await self._execute(query, (status, res_id), fetch="none")