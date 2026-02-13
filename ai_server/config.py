"""
AI Server Configuration
"""

import os
from pathlib import Path
from dotenv import load_dotenv

load_dotenv()

# 프로젝트 루트 경로
_BASE_DIR = Path(__file__).resolve().parent

# gRPC Server Configuration
GRPC_SERVER_HOST = os.getenv("AI_INFERENCE_GRPC_HOST", "0.0.0.0")
GRPC_SERVER_PORT = int(os.getenv("AI_INFERENCE_GRPC_PORT", 50051))

# LLM Server (Qwen3-4B)
LLM_GRPC_HOST = os.getenv("LLM_GRPC_HOST", "0.0.0.0")
LLM_GRPC_PORT = int(os.getenv("LLM_GRPC_PORT", 50051))

# Vision Server (YOLOv8n)
VISION_GRPC_HOST = os.getenv("VISION_GRPC_HOST", "0.0.0.0")
VISION_GRPC_PORT = int(os.getenv("VISION_GRPC_PORT", 50052))

# LLM Configuration (Qwen3-4B via Ollama)
LLM_MODEL_NAME = os.getenv("LLM_MODEL_NAME", "qwen3-4b")
LLM_MODEL_PATH = os.getenv("LLM_MODEL_PATH", str(_BASE_DIR / "models" / "qwen3-4b"))

# Vision YOLO Model Paths
PRODUCT_MODEL_PATH = os.getenv(
    "PRODUCT_MODEL_PATH", str(_BASE_DIR / "models" / "product.pt")
)
OBSTACLE_MODEL_PATH = os.getenv(
    "OBSTACLE_MODEL_PATH", str(_BASE_DIR / "models" / "obstacle.pt")
)

# Face Recognition Configuration
FACE_MATCH_THRESHOLD = float(os.getenv("FACE_MATCH_THRESHOLD", "0.3"))

# Video Stream Configuration (UDP from Robot Camera)
VIDEO_STREAM_HOST = os.getenv("VIDEO_STREAM_HOST", "0.0.0.0")
VIDEO_STREAM_PORT = int(os.getenv("VIDEO_STREAM_PORT", 54321))
VIDEO_BUFFER_SIZE = int(os.getenv("VIDEO_BUFFER_SIZE", 65536))

# Inference Processing Configuration
# 프레임 처리 간격 (초) — YOLO/얼굴 인식을 매 프레임 돌리지 않고 간격을 둠
INFERENCE_INTERVAL = float(os.getenv("INFERENCE_INTERVAL", "0.3"))
# YOLO 신뢰도 임계값
YOLO_CONFIDENCE_THRESHOLD = float(os.getenv("YOLO_CONFIDENCE_THRESHOLD", "0.5"))

# General Settings
MAX_WORKERS = int(os.getenv("MAX_WORKERS", 10))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
