import aiomysql
from typing import Optional, AsyncGenerator
from contextlib import asynccontextmanager
from main_server import config

class Database:
    _pool: Optional[aiomysql.Pool] = None

    @classmethod
    async def initialize(cls):
        """
        데이터베이스 연결 풀을 초기화합니다.
        애플리케이션 시작 시 호출되어야 합니다.
        """
        if cls._pool is None:
            cls._pool = await aiomysql.create_pool(
                host=config.DB_HOST,
                port=config.DB_PORT,
                user=config.DB_USER,
                password=config.DB_PASSWORD,
                db=config.DB_NAME,
                autocommit=False,
                loop=None,  # aiomysql will use the current event loop
            )

    @classmethod
    async def close(cls):
        """
        데이터베이스 연결 풀을 닫습니다.
        애플리케이션 종료 시 호출되어야 합니다.
        """
        if cls._pool:
            cls._pool.close()
            await cls._pool.wait_closed()
            cls._pool = None

    @classmethod
    @asynccontextmanager
    async def get_connection(cls) -> AsyncGenerator[aiomysql.Connection, None]:
        """
        연결 풀에서 비동기 데이터베이스 연결을 가져오는 컨텍스트 관리자를 제공합니다.
        """
        if cls._pool is None:
            raise ConnectionError("Database pool is not initialized. Please call Database.initialize() first.")
        
        async with cls._pool.acquire() as conn:
            try:
                yield conn
            finally:
                pass # The connection is automatically released back to the pool

# FastAPI 등에서 사용할 수 있는 의존성 주입용 함수
async def get_db_connection() -> AsyncGenerator[aiomysql.Connection, None]:
    """
    FastAPI 의존성 주입 시스템을 위한 데이터베이스 연결 생성기입니다.
    """
    async with Database.get_connection() as connection:
        yield connection
