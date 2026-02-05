import os
from dotenv import load_dotenv

# .env 파일 로드
load_dotenv()

# Server configuration
SERVER_HOST = os.getenv("SERVER_HOST", "0.0.0.0")
SERVER_PORT = int(os.getenv("SERVER_PORT", 8000))

# Database configuration
DB_HOST = os.getenv("DB_HOST", "localhost")
DB_USER = os.getenv("DB_USER", "root")
DB_PASSWORD = os.getenv("DB_PASSWORD", "")
DB_NAME = os.getenv("DB_NAME", "office_robot_db")
DB_PORT = int(os.getenv("DB_PORT", 3306))

# Database URL (SQLAlchemy 등에서 필요할 경우 사용)
DATABASE_URL = f"mysql+mysqlconnector://{DB_USER}:{DB_PASSWORD}@{DB_HOST}:{DB_PORT}/{DB_NAME}"

# ROS Bridge configuration
ROS_BRIDGE_HOST = os.getenv("ROS_BRIDGE_HOST", "localhost")
ROS_BRIDGE_PORT = int(os.getenv("ROS_BRIDGE_PORT", 9090))

# AI Inference service configuration
AI_INFERENCE_GRPC_HOST = os.getenv("AI_INFERENCE_GRPC_HOST", "localhost")
AI_INFERENCE_GRPC_PORT = int(os.getenv("AI_INFERENCE_GRPC_PORT", 50051))

# Video Stream configuration
VIDEO_STREAM_HOST = os.getenv("VIDEO_STREAM_HOST", "0.0.0.0")
VIDEO_STREAM_PORT = int(os.getenv("VIDEO_STREAM_PORT", 54321))

# FastAPI Application Metadata
APP_TITLE = "Office Robot Service API"
APP_DESCRIPTION = "[v3.0] UI와 API가 통합된 오피스 로봇 서비스"
APP_VERSION = "3.0.0"

# Web configurations
STATIC_FILES_DIR = "main_server/web/static"
ADMIN_DASHBOARD_PATH = "/web/admin"
EMPLOYEE_APP_PATH = "/web/employee"

