from typing import List, Dict, Any
from pydantic import BaseModel
from .base_repository import BaseRepository

class ProductModel(BaseModel):
    product_id: int
    name: str
    type: str
    stock_quantity: int

class MySQLProductRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="Products", model=ProductModel)

    async def get_snack_inventory(self) -> List[Dict[str, Any]]:
        # SR-007: 재고가 10개 미만이면 '부족'으로 표시
        query = """
            SELECT name, stock_quantity as quantity,
                   CASE WHEN stock_quantity < 10 THEN '부족' ELSE '정상' END as status
            FROM Products 
            WHERE type = 'SNACK'
        """
        return await self._execute(query, fetch="all")