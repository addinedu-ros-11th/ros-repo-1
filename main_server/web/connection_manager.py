from typing import List
import asyncio
from fastapi import WebSocket


class ConnectionManager:
    """
    활성 WebSocket 연결을 관리하는 중앙 관리자 클래스.
    - 새로운 클라이언트의 연결 및 연결 해제를 처리합니다.
    - 모든 활성 클라이언트에게 메시지를 브로드캐스트합니다.
    """
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        """새로운 WebSocket 연결을 수락하고 목록에 추가합니다."""
        await websocket.accept()
        self.active_connections.append(websocket)
        print(f"새로운 클라이언트 연결: {websocket.client}. 총 {len(self.active_connections)} 명 접속 중.")

    def disconnect(self, websocket: WebSocket):
        """WebSocket 연결을 목록에서 제거합니다."""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
            print(f"클라이언트 연결 해제: {websocket.client}. 총 {len(self.active_connections)} 명 접속 중.")

    async def broadcast(self, message: str):
        """모든 활성 WebSocket 연결에 텍스트 메시지를 브로드캐스트합니다."""
        # 모든 클라이언트에 대한 전송 작업을 비동기적으로 동시에 실행합니다.
        await asyncio.gather(
            *[connection.send_text(message) for connection in self.active_connections],
            return_exceptions=False # 오류가 발생한 전송은 무시
        )

# 애플리케이션 전체에서 사용할 단일 ConnectionManager 인스턴스
manager = ConnectionManager()
