import asyncio
from typing import List, Dict, Any
from .base_repository import BaseRepository

class MySQLReservationRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="reservations", model=None, pk_name="id")

    # [수정] 아래 함수들 앞에 들여쓰기를 추가하세요
    async def get_pending_list(self) -> List[Dict[str, Any]]:
        query = "SELECT * FROM reservations WHERE status = 'PENDING' ORDER BY created_at DESC"
        # 이제 부모 클래스(BaseRepository)의 통합된 _execute를 사용합니다.
        return await self._execute(query, fetch="all")

    async def get_approved_list(self) -> List[Dict[str, Any]]:
        query = "SELECT * FROM reservations WHERE status = 'APPROVED' ORDER BY visit_date ASC"
        return await self._execute(query, fetch="all")

    async def update_status(self, res_id: int, status: str):
        query = "UPDATE reservations SET status = %s WHERE id = %s"
        # 쓰기 작업은 is_write=True를 인자로 전달합니다.
        await self._execute(query, (status, res_id), is_write=True)