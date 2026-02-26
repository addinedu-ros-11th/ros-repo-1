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
        if not name: return None

        # 1. 완전 일치 검색
        query = f"SELECT * FROM {self.table_name} WHERE name = %s"
        result = await self._execute(query, (name,), fetch="one")
        
        if not result:
            # 2. 부분 일치 검색 (앞뒤 상관없이 포함 여부)
            query = f"SELECT * FROM {self.table_name} WHERE name LIKE %s"
            result = await self._execute(query, (f"%{name}%",), fetch="one")

        if not result and "(" in name:
            # 3. 괄호가 포함된 경우 (예: 'Choco Pie (초코파이)') 앞부분만으로 재검색
            short_name = name.split("(")[0].strip()
            query = f"SELECT * FROM {self.table_name} WHERE name LIKE %s"
            result = await self._execute(query, (f"%{short_name}%",), fetch="one")

        if result:
            return result
        return None
