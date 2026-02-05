from main_server import config
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

# --- DI 컨테이너 및 서비스 초기화 ---
from main_server.container import container
from main_server.infrastructure.database.connection import Database
from main_server.web.connection_manager import manager as connection_manager
from main_server.infrastructure.communication.ros_bridge import ROSBridge

# 전역 변수로 백그라운드 태스크 저장
background_tasks = set()

# --- FastAPI 애플리케이션 설정 ---
async def startup_event():
    """애플리케이션 시작 시 모든 서비스를 초기화하고 백그라운드 서버를 가동합니다."""
    # 1. 데이터베이스 연결 풀 생성
    await Database.initialize()
    print("Database pool initialized.")
    
    # 2. DI 컨테이너 초기화 (서비스, 리포지토리 등)
    container.services()
    print("DI container and services initialized.")

    # 3. 통신 서버(ROS 브리지)를 백그라운드 태스크로 시작
    ros_bridge = ROSBridge(host=config.ROS_BRIDGE_HOST, port=config.ROS_BRIDGE_PORT, fleet_manager=container.fleet_manager)
    bridge_task = asyncio.create_task(ros_bridge.start())
    background_tasks.add(bridge_task)

    # 4. AI 실시간 추론 결과 구독 시작
    ai_stream_task = asyncio.create_task(container.fleet_manager.start_ai_stream())
    background_tasks.add(ai_stream_task)
    
    print("ROS Bridge server and AI Stream subscriber started.")


async def shutdown_event():
    """애플리케이션 종료 시 모든 백그라운드 태스크를 취소하고 리소스를 정리합니다."""
    for task in background_tasks:
        task.cancel()
    
    await asyncio.gather(*background_tasks, return_exceptions=True)
    print("Background servers stopped.")
    
    await Database.close()
    print("Database pool closed.")


# FastAPI 앱 인스턴스 생성
app = FastAPI(
    title=config.APP_TITLE,
    description=config.APP_DESCRIPTION,
    version=config.APP_VERSION,
    on_startup=[startup_event],
    on_shutdown=[shutdown_event]
)

# --- 정적 파일 마운트 ---
app.mount("/static", StaticFiles(directory=config.STATIC_FILES_DIR), name="static")

# --- API 및 웹 라우터 등록 ---
from main_server.api.v1 import admin_routes, employee_routes, guest_routes
from main_server.web import routes as web_router
app.include_router(admin_routes.router)
app.include_router(employee_routes.router)
app.include_router(guest_routes.router)
app.include_router(web_router.router) # 웹 UI 라우터 추가

# --- WebSocket 엔드포인트 ---
@app.websocket("/ws/admin/status")
async def websocket_endpoint(websocket: WebSocket):
    """관리자 페이지의 실시간 상태 업데이트를 위한 WebSocket 엔드포인트"""
    await connection_manager.connect(websocket)
    try:
        while True:
            # 현재는 클라이언트로부터 메시지를 받지 않고, 서버 푸시만 사용합니다.
            # receive_text()는 연결 유지를 위해 필요하며, 클라이언트가 연결을 닫으면 예외를 발생시킵니다.
            await websocket.receive_text()
    except WebSocketDisconnect:
        connection_manager.disconnect(websocket)


@app.get("/")
def read_root():
    """API 서버의 상태를 확인하고 UI 링크를 제공하는 기본 엔드포인트"""
    return {
        "message": "Office Robot Service API is running!",
        "admin_dashboard": config.ADMIN_DASHBOARD_PATH,
        "employee_app": config.EMPLOYEE_APP_PATH,
        "api_docs": "/docs"
    }

# uvicorn으로 이 앱을 실행하려면 터미널에서 다음 명령어를 사용하세요:
# uvicorn main_server.app:app --reload
