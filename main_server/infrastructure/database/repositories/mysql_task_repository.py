import json
import logging
from typing import List, Optional, Dict, Any
from datetime import datetime

from main_server.domains.tasks.schemas import Task, TaskStatus
from main_server.domains.tasks.repository import ITaskRepository
from main_server.infrastructure.database.repositories.base_repository import BaseRepository
from main_server.infrastructure.database.connection import Database

logger = logging.getLogger(__name__)

class MySQLTaskRepository(BaseRepository, ITaskRepository):
    """
    MySQL 데이터베이스에서 Task 데이터를 관리하는 구체적인 리포지토리 클래스입니다.
    """
    def __init__(self):
        super().__init__(table_name="Tasks", model=Task, pk_name="task_id")

    async def get_by_id(self, task_id: int) -> Optional[Task]:
        """ID로 단일 항목을 조회하고 JSON 필드를 처리합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE {self.pk_name} = %s"
        result = await self._execute(query, (task_id,), fetch="one")
        if not result:
            return None
            
        # DB의 JSON 문자열을 dict로 변환 (Pydantic 모델 생성 전)
        if "details" in result and isinstance(result["details"], str):
            try:
                result["details"] = json.loads(result["details"])
            except json.JSONDecodeError:
                result["details"] = {}
                
        return self.model(**result)

    async def get_all_by_status(self, status: TaskStatus) -> List[Task]:
        query = f"SELECT * FROM {self.table_name} WHERE status = %s ORDER BY created_at ASC"
        results = await self._execute(query, (status.value,), fetch="all")
        
        processed_results = []
        for row in results:
            if "details" in row and isinstance(row["details"], str):
                try:
                    row["details"] = json.loads(row["details"])
                except json.JSONDecodeError:
                    row["details"] = {}
            processed_results.append(self.model(**row))
        return processed_results

    async def get_all_for_user(self, user_id: int) -> List[Task]:
        query = f"SELECT * FROM {self.table_name} WHERE requester_id = %s ORDER BY created_at DESC"
        results = await self._execute(query, (user_id,), fetch="all")
        
        processed_results = []
        for row in results:
            if "details" in row and isinstance(row["details"], str):
                try:
                    row["details"] = json.loads(row["details"])
                except json.JSONDecodeError:
                    row["details"] = {}
            processed_results.append(self.model(**row))
        return processed_results

    async def create(self, data: Dict[str, Any], items: Optional[List[Dict[str, Any]]] = None) -> Optional[Task]:
        """
        새로운 작업을 생성하고, 연관된 아이템(Task_Items)이 있다면 함께 저장합니다.
        트랜잭션을 사용하여 원자성을 보장합니다.
        """
        # Enum 객체 처리 (SR-011, SR-012 호환성 보장)
        for key in ["task_type", "status"]:
            if key in data and hasattr(data[key], "value"):
                data[key] = data[key].value

        # 'details' 필드를 JSON 문자열로 변환
        if 'details' in data and isinstance(data['details'], dict):
            data['details'] = json.dumps(data['details'])
        
        # 실제 DB 컬럼에 해당하는 필드만 추출
        # Task.model_fields는 alias를 포함하므로, 실제 DB 컬럼명으로 변환하거나 직접 지정이 필요할 수 있음
        # 여기서는 data에 들어있는 키 중 DB schema(Tasks table)에 있는 것들을 주로 사용
        db_fields = {
            "requester_id", "receiver_id", "assigned_robot_id", "task_type", 
            "priority", "status", "destination_id", "target_location_name", 
            "visitor_id", "details"
        }
        task_db_data = {k: v for k, v in data.items() if k in db_fields}
        
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                try:
                    # 1. Task 생성
                    columns = ", ".join(task_db_data.keys())
                    placeholders = ", ".join(["%s"] * len(task_db_data))
                    query = f"INSERT INTO {self.table_name} ({columns}) VALUES ({placeholders})"
                    await cursor.execute(query, tuple(task_db_data.values()))
                    task_id = cursor.lastrowid
                    
                    # 2. 연관 아이템(Task_Items) 생성
                    if items:
                        item_query = "INSERT INTO Task_Items (task_id, product_id, quantity) VALUES (%s, %s, %s)"
                        for item in items:
                            await cursor.execute(item_query, (task_id, item['product_id'], item['quantity']))
                    
                    await conn.commit()
                    logger.info(f"Task {task_id} created successfully with {len(items) if items else 0} items.")
                except Exception as e:
                    await conn.rollback()
                    logger.error(f"Failed to create task (Step: {'Items' if 'task_id' in locals() else 'Task'}): {e}", exc_info=True)
                    return None

        return await self.get_by_id(task_id)

    async def update(self, task_id: int, update_data: Dict[str, Any]) -> Optional[Task]:
        if 'details' in update_data and isinstance(update_data['details'], dict):
            update_data['details'] = json.dumps(update_data['details'])
        
        # 완료 시간을 자동으로 설정
        if update_data.get("status") == TaskStatus.COMPLETED.value:
            update_data["completed_at"] = datetime.utcnow()

        await super().update(task_id, update_data)
        return await self.get_by_id(task_id)

def get_task_repository() -> ITaskRepository:
    return MySQLTaskRepository()
