from typing import List, Optional, Dict, Any

from main_server.domains.robots.schemas import Robot, RobotStatus
from main_server.domains.robots.repository import IRobotRepository
from main_server.infrastructure.database.repositories.base_repository import BaseRepository

class MySQLRobotRepository(BaseRepository, IRobotRepository):
    """
    MySQL 데이터베이스에서 로봇 데이터를 관리하는 구체적인 리포지토리 클래스입니다.
    """
    def __init__(self):
        super().__init__(table_name="Robots", model=Robot, pk_name="robot_id")

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

        # DB 컬럼명으로 매핑 (pose_x -> current_x, pose_y -> current_y)
        if "pose_x" in update_values:
            update_values["current_x"] = update_values.pop("pose_x")
        if "pose_y" in update_values:
            update_values["current_y"] = update_values.pop("pose_y")

        await super().update(robot_id, update_values)
        
        # 업데이트 후 현재 상태를 가져와 텔레메트리 로그 기록
        updated_robot = await self.get_by_id(robot_id)
        if updated_robot:
            await self.log_telemetry(robot_id, {
                "status": updated_robot.status,
                "current_x": updated_robot.pose_x,
                "current_y": updated_robot.pose_y,
                "battery_level": updated_robot.battery_level
            })
        
        return updated_robot

    async def delete(self, robot_id: int) -> bool:
        """ID로 로봇을 삭제하고 성공 여부를 반환합니다."""
        robot = await self.get_by_id(robot_id)
        if not robot:
            return False
        
        await super().delete(robot_id)
        return True

    async def update_location(self, robot_id: int, x: float, y: float):
        """실시간 좌표 업데이트 (관제 지도 표시용)"""
        await self.update(robot_id, {"pose_x": x, "pose_y": y})

    async def log_telemetry(self, robot_id: int, data: dict):
        """로봇의 모든 센서 데이터를 로그 테이블에 기록 (분석용)"""
        query = """
            INSERT INTO Robot_Telemetry_Logs 
            (robot_id, status, location_x, location_y, battery_level, timestamp)
            VALUES (%s, %s, %s, %s, %s, NOW())
        """
        params = (
            robot_id,
            data.get("status"),
            data.get("current_x"),
            data.get("current_y"),
            data.get("battery_level")
        )
        await self._execute_and_commit(query, params)

    async def _execute_and_commit(self, query: str, params: tuple):
        from main_server.infrastructure.database.connection import Database
        async with Database.get_connection() as conn:
            async with conn.cursor() as cursor:
                await cursor.execute(query, params)
                await conn.commit()

# 이 리포지토리를 사용하기 위한 의존성 주입용 팩토리 함수
def get_robot_repository() -> IRobotRepository:
    return MySQLRobotRepository()
