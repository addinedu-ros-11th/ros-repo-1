import json
import logging
import aiomysql
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
        logger.debug(f"[MySQLTaskRepository] get_by_id 호출: task_id={task_id}")
        query = f"SELECT * FROM {self.table_name} WHERE {self.pk_name} = %s"
        result = await self._execute(query, (task_id,), fetch="one")
        
        if not result:
            logger.warning(f"[MySQLTaskRepository] ID {task_id}에 해당하는 작업을 DB에서 찾을 수 없습니다.")
            return None
            
        logger.debug(f"[MySQLTaskRepository] DB 조회 결과 성공: {result}")

        # DB의 JSON 문자열을 dict로 변환 (Pydantic 모델 생성 전)
        if "details" in result and isinstance(result["details"], str):
            try:
                result["details"] = json.loads(result["details"])
            except json.JSONDecodeError:
                logger.warning(f"[MySQLTaskRepository] Task {task_id}의 details JSON 파싱 실패")
                result["details"] = {}
                
        try:
            task_obj = self.model(**result)
            return task_obj
        except Exception as e:
            logger.error(f"[MySQLTaskRepository] Task 모델 변환 실패 (ID: {task_id}): {e}")
            logger.error(f"원인 데이터: {result}")
            return None

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
        트랜잭션을 사용하여 원자성을 보장하며, 생성 직후 동일 연결에서 데이터를 조회합니다.
        """
        # Enum 객체 처리 (SR-011, SR-012 호환성 보장)
        for key in ["task_type", "status"]:
            if key in data and hasattr(data[key], "value"):
                data[key] = data[key].value

        # 'details' 필드를 JSON 문자열로 변환
        if 'details' in data and isinstance(data['details'], dict):
            data['details'] = json.dumps(data['details'])
        
        db_fields = {
            "requester_id", "receiver_id", "assigned_robot_id", "task_type", 
            "priority", "status", "destination_id", "target_location_name", 
            "visitor_id", "details"
        }
        task_db_data = {k: v for k, v in data.items() if k in db_fields}
        
        async with Database.get_connection() as conn:
            async with conn.cursor(aiomysql.DictCursor) as cursor:
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
                    logger.info(f"[MySQLTaskRepository] Task {task_id} committed successfully.")

                    # 3. 동일 연결에서 즉시 재조회 (가시성 문제 해결)
                    select_query = f"SELECT * FROM {self.table_name} WHERE {self.pk_name} = %s"
                    await cursor.execute(select_query, (task_id,))
                    result = await cursor.fetchone()
                    
                    if not result:
                        logger.error(f"[MySQLTaskRepository] 생성 직후 ID {task_id} 조회 실패 (동일 연결)")
                        return None

                    # JSON 복구 및 모델 생성
                    if "details" in result and isinstance(result["details"], str):
                        try:
                            result["details"] = json.loads(result["details"])
                        except:
                            result["details"] = {}
                    
                    return self.model(**result)

                except Exception as e:
                    await conn.rollback()
                    logger.error(f"Failed to create task (Step: {'Items' if 'task_id' in locals() else 'Task'}): {e}", exc_info=True)
                    return None

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
