from contextlib import asynccontextmanager
import logging
import asyncio
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from main_server.config import config

# --- 로깅 설정 ---
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)

# --- DI 컨테이너 및 서비스 초기화 ---
from main_server.container import container
from main_server.infrastructure.database.connection import Database
from main_server.web.connection_manager import manager as connection_manager
from main_server.infrastructure.robot_bridge.ros_bridge import ROSBridge

# 전역 변수로 백그라운드 태스크 저장
background_tasks = set()


@asynccontextmanager
async def lifespan(app: FastAPI):
    """애플리케이션 생명주기 관리 (시작 및 종료)"""
    try:
        # [Startup]
        # 1. 데이터베이스 연결 풀 생성
        await Database.initialize()
        logger.info("Database pool initialized.")

        # 2. DI 컨테이너 초기화
        container.services()
        logger.info("DI container and services initialized.")

        # 3. 통신 서버 시작
        ros_bridge = ROSBridge(
            fleet_manager=container.fleet_manager,
            task_manager=container.task_manager,
            # mutex_manager는 현재 컨테이너에 없으므로 생략 (내부적으로 None 처리됨)
            log_repo=container.log_repository,
            communicator=container.robot_communicator,  # [Fix] 공유 인스턴스 주입
        )
        bridge_task = asyncio.create_task(ros_bridge.start())
        bridge_task.add_done_callback(
            lambda t: (
                logger.error(f"ROS Bridge Task Failed: {t.exception()}")
                if t.exception()
                else None
            )
        )
        background_tasks.add(bridge_task)

        # 4. AI 실시간 스트림 시작
        ai_stream_task = asyncio.create_task(
            container.ai_processing_service.start_ai_stream()
        )
        background_tasks.add(ai_stream_task)

        # 5. DB 로봇 상태 기반 AI 콜백 복원 (재시작 시 콜백 누락 방지)
        await container.fleet_manager.restore_ai_relay_from_db()

        logger.info("ROS Bridge server and AI Stream subscriber started.")

        yield  # 앱 실행 중

    finally:
        # [Shutdown]
        logger.info("Cleaning up resources...")
        await container.ai_processing_service.stop_ai_stream()
        for task in background_tasks:
            task.cancel()

        if background_tasks:
            await asyncio.gather(*background_tasks, return_exceptions=True)

        await Database.close()
        logger.info("Background servers stopped and Database pool closed.")


# FastAPI 앱 인스턴스 생성
app = FastAPI(
    title=config.APP_TITLE,
    description=config.APP_DESCRIPTION,
    version=config.APP_VERSION,
    lifespan=lifespan,
)

# --- 정적 파일 마운트 ---
app.mount("/static", StaticFiles(directory=config.STATIC_FILES_DIR), name="static")

# --- API 및 웹 라우터 등록 ---
from main_server.api.v1 import admin_routes, employee_routes, guest_routes, login_routes
from main_server.web import routes as web_router
from main_server.test_scripts import test_routes

app.include_router(admin_routes.router)
app.include_router(employee_routes.router)
app.include_router(guest_routes.router)
app.include_router(web_router.router)
app.include_router(login_routes.router)
app.include_router(test_routes.router)


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
        "api_docs": "/docs",
    }


# uvicorn으로 이 앱을 실행하려면 터미널에서 다음 명령어를 사용하세요:
# uvicorn main_server.app:app --reload
# 127.0.0.1:8000
