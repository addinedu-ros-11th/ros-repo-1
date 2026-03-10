# /repositories/mysql_log_repository.py
from typing import List, Dict, Any
from pydantic import BaseModel
from .base_repository import BaseRepository

class LogModel(BaseModel):
    log_id: int
    timestamp: Any
    log_level: str
    message: str

class MySQLLogRepository(BaseRepository):
    def __init__(self):
        super().__init__(table_name="System_Logs", model=LogModel)

    async def get_recent_system_logs(self, limit: int = 50) -> List[Dict[str, Any]]:
        query = """
            SELECT DATE_FORMAT(timestamp, '%%Y-%%m-%%d') as date,
                   DATE_FORMAT(timestamp, '%%H:%%i:%%s') as time,
                   COALESCE(r.name, 'System') as robot_name,
                   log_level as status,
                   message
            FROM System_Logs s
            LEFT JOIN Robots r ON s.robot_id = r.robot_id
            ORDER BY s.timestamp DESC
            LIMIT %s
        """
        return await self._execute(query, (limit,), fetch="all")
    
    async def get_system_task_logs(self, target_date: str) -> List[Dict[str, Any]]:
        """
        특정 날짜의 Task 수행 로그와 로봇 상태를 조인하여 조회
        """
        query = """
            SELECT 
                t.created_at,
                r.name AS robot_name,
                r.status AS robot_status,
                r.battery_level,
                t.status AS task_status,
                t.completed_at
            FROM Tasks t
            LEFT JOIN Robots r ON t.assigned_robot_id = r.robot_id
            WHERE DATE(t.created_at) = %s
            ORDER BY t.created_at DESC
        """
        return await self._execute(query, (target_date,))