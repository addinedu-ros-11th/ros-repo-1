#!/bin/bash

# Vision Server 시작 스크립트

echo "================================"
echo "Vision Server (YOLOv8n) 시작"
echo "Port: 50052"
echo "================================"

# 프로젝트 루트로 이동
cd "$(dirname "$0")/.."

# Python 경로 설정
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# Vision Server 실행
python3 -m ai_server.vision_server
