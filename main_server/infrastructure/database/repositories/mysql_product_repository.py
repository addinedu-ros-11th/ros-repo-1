from typing import List, Dict, Any, Optional
from pydantic import BaseModel
from .base_repository import BaseRepository

class ProductModel(BaseModel):
    product_id: int
    name: str
    type: str
    stock_quantity: int

class MySQLProductRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="Products", model=ProductModel, pk_name="product_id")

    async def get_snack_inventory(self) -> List[Dict[str, Any]]:
        # SR-007: 재고가 10개 미만이면 '부족'으로 표시
        query = """
            SELECT name, stock_quantity as quantity,
                   CASE WHEN stock_quantity < 10 THEN '부족' ELSE '정상' END as status
            FROM Products 
            WHERE type = 'SNACK'
        """
        return await self._execute(query, fetch="all")
        
    async def find_by_name(self, name: str) -> Optional[Dict[str, Any]]:
        """ScenarioDataHandler 호환용: 이름으로 제품 조회 (부분 일치 지원)"""
        # 1. 완전 일치 검색
        query = f"SELECT * FROM {self.table_name} WHERE name = %s"
        result = await self._execute(query, (name,), fetch="one")
        
        if not result:
            # 2. 부분 일치 검색 (AI가 '초코파이'라고 했을 때 'Choco Pie'와 매칭되지는 않겠지만, 최소한 대소문자나 부분 문자열 대응)
            query = f"SELECT * FROM {self.table_name} WHERE name LIKE %s"
            result = await self._execute(query, (f"%{name}%",), fetch="one")

        if result:
            return result
        return None
