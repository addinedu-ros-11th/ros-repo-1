from typing import Optional, Any
from pydantic import BaseModel
from datetime import datetime
from main_server.infrastructure.database.repositories.base_repository import BaseRepository

class Visitor(BaseModel):
    visitor_id: int
    host_user_id: Optional[int] = None
    name: Optional[str] = None
    phone: Optional[str] = None          # 통합된 필드
    purpose: Optional[str] = None        # 통합된 필드
    destination_id: Optional[int] = None # 목적지 정보 추가
    qr_code: Optional[str] = None
    status: Optional[str] = None
    visit_date: Optional[datetime] = None
    visit_time: Optional[Any] = None     # MySQL TIME 타입 대응
    created_at: Optional[datetime] = None

class MySQLVisitorRepository(BaseRepository):
    """방문객 정보(QR코드, 예약 포함)를 관리하는 리포지토리"""
    def __init__(self):
        super().__init__(table_name="Visitors", model=Visitor, pk_name="visitor_id")

    async def get_by_qr_code(self, qr_code: str) -> Optional[Visitor]:
        """QR 코드로 방문객 정보를 조회합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE qr_code = %s"
        result = await self._execute(query, (qr_code,), fetch="one")
        if result:
            return self.model(**result)
        return None

    async def get_pending_list(self):
        """승인 대기 중인 방문 예약 목록 조회"""
        query = f"SELECT * FROM {self.table_name} WHERE status = 'PENDING' ORDER BY created_at DESC"
        results = await self._execute(query, fetch="all")
        return [self.model(**row) for row in results]

    async def get_approved_list(self):
        """승인된 방문 예약 목록 조회"""
        query = f"SELECT * FROM {self.table_name} WHERE status IN ('APPROVED', 'CHECKED_IN') ORDER BY visit_date ASC, visit_time ASC"
        results = await self._execute(query, fetch="all")
        return [self.model(**row) for row in results]
    
    async def get_rejected_list(self):
        """반려된 방문 예약 목록 조회"""
        query = f"SELECT * FROM {self.table_name} WHERE status = 'REJECTED' ORDER BY created_at DESC"
        results = await self._execute(query, fetch="all")
        return [self.model(**row) for row in results]