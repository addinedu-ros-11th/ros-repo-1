from typing import List, Optional, Dict, Any

from main_server.domains.robots.robot import Robot, RobotStatus
from main_server.domains.robots.robot_repository import IRobotRepository
from main_server.infrastructure.database.base_repository import BaseRepository

class MySQLRobotRepository(BaseRepository, IRobotRepository):
    """
    MySQL 데이터베이스에서 로봇 데이터를 관리하는 구체적인 리포지토리 클래스입니다.
    """
    def __init__(self):
        super().__init__(table_name="robots", model=Robot)

    async def get_by_id(self, robot_id: int) -> Optional[Robot]:
        return await super().get_by_id(robot_id)

    async def get_all(self) -> List[Robot]:
        return await super().get_all()

    async def find_by_status(self, status: RobotStatus) -> List[Robot]:
        """특정 상태의 모든 로봇을 조회합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE status = %s"
        results = await self._execute(query, (status.value,), fetch="all")
        return [self.model(**row) for row in results]

    async def create(self, name: str, battery_level: float) -> Robot:
        """새로운 로봇을 생성하고 생성된 객체를 반환합니다."""
        data_dict = {
            "name": name,
            "battery_level": battery_level,
            "status": RobotStatus.IDLE.value  # 기본 상태
        }
        new_robot_id = await super().create(data_dict)
        return Robot(id=new_robot_id, **data_dict)

    async def update(self, robot_id: int, update_data: Dict[str, Any]) -> Optional[Robot]:
        """로봇 정보를 업데이트하고 업데이트된 객체를 반환합니다."""
        # Pydantic 모델의 기본값이 아닌 명시적으로 설정된 값만 포함
        update_values = {k: v for k, v in update_data.items() if v is not None}
        if not update_values:
            return await self.get_by_id(robot_id) # 업데이트할 내용이 없으면 현재 상태 반환

        await super().update(robot_id, update_values)
        return await self.get_by_id(robot_id)

    async def delete(self, robot_id: int) -> bool:
        """ID로 로봇을 삭제하고 성공 여부를 반환합니다."""
        robot = await self.get_by_id(robot_id)
        if not robot:
            return False
        
        await super().delete(robot_id)
        return True

# 이 리포지토리를 사용하기 위한 의존성 주입용 팩토리 함수
def get_robot_repository() -> IRobotRepository:
    return MySQLRobotRepository()
