from typing import Optional
from enum import Enum
from pydantic import BaseModel
# 여기서 BaseRepository를 가져와야 DB 연결 기능을 공짜로 쓸 수 있습니다.
from .base_repository import BaseRepository

# [1] 데이터 구조 정의 (Schema_diagram.txt 반영)
class UserRole(str, Enum):
    STAFF = "STAFF"
    ADMIN = "ADMIN"

class UserModel(BaseModel):
    user_id: int
    account: str  # username -> account로 변경
    password_hash: str
    role: UserRole
    name: str
    department: Optional[str] = None

# [2] 리포지토리 클래스
class UserRepository(BaseRepository):
    def __init__(self):
        # BaseRepository의 기능을 가져오면서 'Users' 테이블을 쓰겠다고 설정함
        super().__init__(table_name="Users", model=UserModel)

    async def get_user_by_username(self, account: str) -> Optional[UserModel]:
        # 쿼리문에서도 username -> account로 변경
        query = f"SELECT * FROM {self.table_name} WHERE account = %s"
        result = await self._execute(query, (account,), fetch="one")
        
        if result:
            return self.model(**result)
        return None