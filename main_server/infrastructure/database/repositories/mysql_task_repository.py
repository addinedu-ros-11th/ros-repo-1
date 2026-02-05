import json
from typing import List, Optional, Dict, Any
from datetime import datetime

from main_server.domains.tasks.task import Task, TaskStatus
from main_server.domains.tasks.task_repository import ITaskRepository
from main_server.infrastructure.database.base_repository import BaseRepository

class MySQLTaskRepository(BaseRepository, ITaskRepository):
    """
    MySQL 데이터베이스에서 Task 데이터를 관리하는 구체적인 리포지토리 클래스입니다.
    """
    def __init__(self):
        super().__init__(table_name="tasks", model=Task)

    async def get_by_id(self, task_id: int) -> Optional[Task]:
        task_data = await super().get_by_id(task_id)
        if task_data:
            # DB에 JSON 문자열로 저장된 'details' 필드를 dict로 변환
            if isinstance(task_data.details, str):
                task_data.details = json.loads(task_data.details)
        return task_data

    async def get_all_by_status(self, status: TaskStatus) -> List[Task]:
        query = f"SELECT * FROM {self.table_name} WHERE status = %s ORDER BY created_at ASC"
        results = await self._execute(query, (status.value,), fetch="all")
        return [self.model(**row) for row in results]

    async def get_all_for_user(self, user_id: int) -> List[Task]:
        query = f"SELECT * FROM {self.table_name} WHERE requester_id = %s ORDER BY created_at DESC"
        results = await self._execute(query, (user_id,), fetch="all")
        return [self.model(**row) for row in results]

    async def create(self, data: Dict[str, Any]) -> Task:
        # 'details' 필드를 JSON 문자열로 변환
        if 'details' in data and isinstance(data['details'], dict):
            data['details'] = json.dumps(data['details'])
        
        # Pydantic 모델에 정의된 필드만 추출
        task_data = {k: v for k, v in data.items() if k in Task.__fields__}
        
        new_task_id = await super().create(task_data)
        
        # 다시 dict로 변환하여 모델 생성
        if 'details' in data:
            data['details'] = json.loads(data['details'])

        return Task(id=new_task_id, **data)

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
