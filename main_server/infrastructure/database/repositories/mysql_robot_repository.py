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

    async def get_by_name(self, name: str) -> Optional[Robot]:
        """이름으로 특정 로봇을 조회합니다."""
        query = f"SELECT * FROM {self.table_name} WHERE name = %s"
        result = await self._execute(query, (name,), fetch="one")
        if result:
            return self.model(**result)
        return None

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
            "status": RobotStatus.IDLE.value,  # 기본 상태
            "current_x": 0.0,
            "current_y": 0.0
        }
        new_robot_id = await super().create(data_dict)
        return Robot(id=new_robot_id, **data_dict)

    async def update(self, robot_id: int, update_data: Dict[str, Any]) -> Optional[Robot]:
        """로봇 정보를 업데이트하고 업데이트된 객체를 반환합니다."""
        # [Fix] current_task_id는 None으로 설정될 수 있어야 하므로 필터링에서 예외 처리
        update_values = {}
        for k, v in update_data.items():
            if v is not None or k == "current_task_id":
                update_values[k] = v
        
        # 하트비트 시간 명시적 추가 (실시간 통신 확인용)
        update_values["last_heartbeat"] = "NOW()" 
        
        # DB 컬럼명으로 매핑 (pose_x -> current_x, pose_y -> current_y)
        if "pose_x" in update_values:
            update_values["current_x"] = update_values.pop("pose_x")
        if "pose_y" in update_values:
            update_values["current_y"] = update_values.pop("pose_y")

        # BaseRepository.update는 placeholder %s를 사용하므로 SQL 함수 NOW()를 직접 넣으려면 쿼리를 직접 작성하거나
        # datetime.now()를 사용해야 합니다. 여기서는 datetime.now()를 사용하도록 수정합니다.
        from datetime import datetime
        update_values["last_heartbeat"] = datetime.now()

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
