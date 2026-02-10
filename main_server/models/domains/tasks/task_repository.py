from abc import ABC, abstractmethod
from typing import List, Optional, Dict, Any

from .task import Task, TaskStatus

class ITaskRepository(ABC):
    """
    작업(Task) 데이터에 접근하기 위한 리포지토리 인터페이스입니다.
    """

    @abstractmethod
    async def get_by_id(self, task_id: int) -> Optional[Task]:
        """ID로 특정 작업을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def get_all_by_status(self, status: TaskStatus) -> List[Task]:
        """특정 상태의 모든 작업을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def get_all_for_user(self, user_id: int) -> List[Task]:
        """특정 사용자가 요청한 모든 작업을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def create(self, data: Dict[str, Any]) -> Task:
        """새로운 작업을 생성합니다."""
        raise NotImplementedError

    @abstractmethod
    async def update(self, task_id: int, update_data: Dict[str, Any]) -> Optional[Task]:
        """작업 정보를 업데이트합니다."""
        raise NotImplementedError