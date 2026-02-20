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