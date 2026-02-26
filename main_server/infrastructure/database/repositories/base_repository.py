from typing import Any, Dict, List, Optional, Tuple, Type, TypeVar
from pydantic import BaseModel
import aiomysql

from ..connection import Database

ModelType = TypeVar("ModelType", bound=BaseModel)

class BaseRepository:
    """
    모든 리포지토리를 위한 기본 클래스입니다.
    비동기 CRUD 작업을 위한 공통 메서드를 제공합니다.
    """
    def __init__(self, table_name: str, model: Type[ModelType], pk_name: str = "id"):
        """
        리포지토리를 초기화합니다.

        :param table_name: 데이터베이스 테이블 이름
        :param model: Pydantic/SQLModel과 같은 데이터 모델 클래스
        :param pk_name: 기본 키 컬럼 이름 (default: 'id')
        """
        self.table_name = table_name
        self.model = model
        self.pk_name = pk_name

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

    async def _execute_write(self, query: str, params: Optional[Tuple] = None) -> None:
        """
        데이터 변경(UPDATE, INSERT, DELETE) 전용 메서드.
        실행 후 반드시 commit()을 수행합니다.
        """
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, params or ())
                await conn.commit()  # 변경 사항 확정
                print(f"DEBUG: Execute Write Success - Query: {query}")

    async def _execute_with_retry(self, query: str, params: Optional[Tuple] = None, fetch: str = "all") -> Any:
        """1412 에러 발생 시 재시도하는 전용 메서드"""
        import asyncio
        max_retries = 3
        for attempt in range(max_retries):
            try:
                # 기존 _execute 로직과 동일하지만 내부에서 호출
                async with Database.get_connection() as conn:
                    async with conn.cursor(aiomysql.DictCursor) as cursor:
                        await cursor.execute(query, params or ())
                        if fetch == "one": return await cursor.fetchone()
                        return await cursor.fetchall()
            except aiomysql.OperationalError as e:
                if e.args[0] == 1412 and attempt < max_retries - 1:
                    await asyncio.sleep(0.2)
                    continue
                raise e

    async def get_by_id(self, item_id: int) -> Optional[ModelType]:
        """ID로 단일 항목을 조회합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE {self.pk_name} = %s"
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
        if not data:
            return
            
        set_clause = ", ".join([f"{key} = %s" for key in data.keys()])
        query = f"UPDATE {self.table_name} SET {set_clause} WHERE {self.pk_name} = %s"
        params = list(data.values()) + [item_id]
        
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, tuple(params))
                await conn.commit()

    async def delete(self, item_id: int) -> None:
        """ID로 항목을 삭제합니다."""
        query = f"DELETE FROM {self.table_name} WHERE {self.pk_name} = %s"
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, (item_id,))
                await conn.commit()

