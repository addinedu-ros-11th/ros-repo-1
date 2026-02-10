from abc import ABC, abstractmethod
from typing import List, Optional

from .robot import Robot, RobotStatus

class IRobotRepository(ABC):
    """
    로봇 데이터에 접근하기 위한 리포지토리 인터페이스입니다.
    이 클래스는 애플리케이션의 서비스 계층이 데이터베이스 기술에 독립적으로
    로봇 데이터와 상호작용할 수 있도록 하는 추상화를 제공합니다.
    """

    @abstractmethod
    async def get_by_id(self, robot_id: int) -> Optional[Robot]:
        """ID로 특정 로봇을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def get_all(self) -> List[Robot]:
        """모든 로봇 목록을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def find_by_status(self, status: RobotStatus) -> List[Robot]:
        """특정 상태에 있는 모든 로봇을 조회합니다."""
        raise NotImplementedError

    @abstractmethod
    async def create(self, name: str, battery_level: float) -> Robot:
        """새로운 로봇을 생성합니다."""
        raise NotImplementedError

    @abstractmethod
    async def update(self, robot_id: int, update_data: dict) -> Optional[Robot]:
        """로봇의 정보를 업데이트합니다."""
        raise NotImplementedError

    @abstractmethod
    async def delete(self, robot_id: int) -> bool:
        """ID로 로봇을 삭제합니다."""
        raise NotImplementedError