from typing import Optional
from enum import Enum
from pydantic import BaseModel, Field
from .base_repository import BaseRepository
from datetime import datetime

# [1] 데이터 구조 정의
class UserRole(str, Enum):
    STAFF = "STAFF"
    ADMIN = "ADMIN"

class UserModel(BaseModel):
    user_id: int
    account: str
    password_hash: str
    role: UserRole
    name: str
    department: Optional[str] = None
    location_id: Optional[int] = None # location_id 추가
    created_at: Optional[datetime] = None

# [2] 리포지토리 클래스
class MySQLUserRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="Users", model=UserModel, pk_name="user_id")

    async def get_user_by_username(self, account: str) -> Optional[UserModel]:
        query = f"SELECT * FROM {self.table_name} WHERE account = %s"
        result = await self._execute(query, (account,), fetch="one")
        if result:
            return self.model(**result)
        return None

    async def find_by_name(self, name: str) -> Optional[dict]:
        """
        ScenarioDataHandler 호환용: 이름(name) 또는 계정(account)으로 사용자 조회.
        dict 형태로 반환하여 handler가 ['user_id'] 등으로 접근 가능하게 함.
        """
        # 1. 이름으로 검색
        query = f"SELECT * FROM {self.table_name} WHERE name = %s"
        result = await self._execute(query, (name,), fetch="one")
        
        if not result:
            # 2. 계정으로 검색 (Fallback)
            query = f"SELECT * FROM {self.table_name} WHERE account = %s"
            result = await self._execute(query, (name,), fetch="one")

        if result:
            # Pydantic 모델로 변환했다가 다시 dict로 주거나, 그냥 result(dict) 반환
            # 핸들러가 dict 접근을 하므로 dict 반환이 편함
            return result
        return None
