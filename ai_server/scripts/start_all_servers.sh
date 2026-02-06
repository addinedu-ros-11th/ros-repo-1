#!/bin/bash

# 모든 AI 서버를 시작하는 통합 스크립트

echo "========================================"
echo "AI Servers 시작"
echo "========================================"

# 프로젝트 루트로 이동
cd "$(dirname "$0")/.."

# Python 경로 설정
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

echo ""
echo "[1/2] LLM Server 시작 중... (Port 50051)"
python3 -m ai_server.llm_server &
LLM_PID=$!

echo "[2/2] Vision Server 시작 중... (Port 50052)"
python3 -m ai_server.vision_server &
VISION_PID=$!

echo ""
echo "========================================"
echo "모든 서버 시작 완료!"
echo "========================================"
echo "LLM Server PID: $LLM_PID (Port 50051)"
echo "Vision Server PID: $VISION_PID (Port 50052)"
echo ""
echo "종료하려면 Ctrl+C를 누르세요"
echo "========================================"

# 시그널 핸들링
trap "echo '서버 종료 중...'; kill $LLM_PID $VISION_PID; exit" INT TERM

# 대기
wait
