"""
AI Server Configuration
"""

import os
from dotenv import load_dotenv

load_dotenv()

# gRPC Server Configuration
GRPC_SERVER_HOST = os.getenv("AI_INFERENCE_GRPC_HOST", "0.0.0.0")
GRPC_SERVER_PORT = int(os.getenv("AI_INFERENCE_GRPC_PORT", 50051))

# LLM Server (Qwen3-4B)
LLM_GRPC_HOST = os.getenv("LLM_GRPC_HOST", "0.0.0.0")
LLM_GRPC_PORT = int(os.getenv("LLM_GRPC_PORT", 50051))

# Vision Server (YOLOv8n)
VISION_GRPC_HOST = os.getenv("VISION_GRPC_HOST", "0.0.0.0")
VISION_GRPC_PORT = int(os.getenv("VISION_GRPC_PORT", 50052))

# LLM Configuration (Qwen3-4B)
LLM_MODEL_NAME = os.getenv("LLM_MODEL_NAME", "qwen3-4b")
LLM_MODEL_PATH = os.getenv("LLM_MODEL_PATH", "./models/qwen3-4b")

# Vision Configuration (YOLOv8n)
VISION_MODEL_NAME = os.getenv("VISION_MODEL_NAME", "yolov8n")
VISION_MODEL_PATH = os.getenv("VISION_MODEL_PATH", "./models/yolov8n.pt")

# Video Stream Configuration (UDP from Robot Camera)
VIDEO_STREAM_HOST = os.getenv("VIDEO_STREAM_HOST", "0.0.0.0")
VIDEO_STREAM_PORT = int(os.getenv("VIDEO_STREAM_PORT", 54321))
VIDEO_BUFFER_SIZE = int(os.getenv("VIDEO_BUFFER_SIZE", 65536))
VIDEO_FPS = int(os.getenv("VIDEO_FPS", 30))

# General Settings
MAX_WORKERS = int(os.getenv("MAX_WORKERS", 10))
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO")
