from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar
from pydantic import BaseModel
import aiomysql

from .connection import Database

ModelType = TypeVar("ModelType", bound=BaseModel)

class BaseRepository:
    """
    모든 리포지토리를 위한 기본 클래스입니다.
    비동기 CRUD 작업을 위한 공통 메서드를 제공합니다.
    """
    def __init__(self, table_name: str, model: Type[ModelType]):
        """
        리포지토리를 초기화합니다.

        :param table_name: 데이터베이스 테이블 이름
        :param model: Pydantic/SQLModel과 같은 데이터 모델 클래스
        """
        self.table_name = table_name
        self.model = model

    async def _execute(self, query: str, params: Optional[Tuple] = None, fetch: str = "all") -> Any:
        """
        주어진 쿼리를 실행하고 결과를 반환하는 내부 메서드입니다.

        :param query: 실행할 SQL 쿼리
        :param params: 쿼리에 바인딩할 파라미터
        :param fetch: 'one', 'all', 'none' 중 하나
        """
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cursor:
                await cursor.execute(query, params or ())
                if fetch == "one":
                    return await cursor.fetchone()
                elif fetch == "all":
                    return await cursor.fetchall()
                # fetch == 'none'의 경우, 아무것도 반환하지 않음 (e.g., INSERT, UPDATE, DELETE)

    async def get_by_id(self, item_id: int) -> Optional[ModelType]:
        """ID로 단일 항목을 조회합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE id = %s"
        result = await self._execute(query, (item_id,), fetch="one")
        if result:
            return self.model(**result)
        return None

    async def get_all(self, limit: int = 100, offset: int = 0) -> List[ModelType]:
        """테이블의 모든 항목을 페이지네이션하여 조회합니다."""
        query = f"SELECT * FROM {self.table_name} LIMIT %s OFFSET %s"
        results = await self._execute(query, (limit, offset), fetch="all")
        return [self.model(**row) for row in results]

    async def create(self, data: Dict[str, Any]) -> int:
        """새로운 항목을 생성합니다."""
        columns = ", ".join(data.keys())
        placeholders = ", ".join(["%s"] * len(data))
        query = f"INSERT INTO {self.table_name} ({columns}) VALUES ({placeholders})"
        
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, tuple(data.values()))
                await conn.commit()
                return cursor.lastrowid

    async def update(self, item_id: int, data: Dict[str, Any]) -> None:
        """ID로 기존 항목을 업데이트합니다."""
        set_clause = ", ".join([f"{key} = %s" for key in data.keys()])
        query = f"UPDATE {self.table_name} SET {set_clause} WHERE id = %s"
        params = list(data.values()) + [item_id]
        
        async with Database.get_connection() as conn:
            await self._execute(query, tuple(params), fetch="none")
            await conn.commit()

    async def delete(self, item_id: int) -> None:
        """ID로 항목을 삭제합니다."""
        query = f"DELETE FROM {self.table_name} WHERE id = %s"
        async with Database.get_connection() as conn:
            await self._execute(query, (item_id,), fetch="none")
            await conn.commit()

