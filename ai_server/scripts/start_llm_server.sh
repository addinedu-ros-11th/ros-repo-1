#!/bin/bash

# LLM Server 시작 스크립트

echo "================================"
echo "LLM Server (Qwen3-4B) 시작"
echo "Port: 50051"
echo "================================"

# 프로젝트 루트로 이동
cd "$(dirname "$0")/.."

# Python 경로 설정
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# LLM Server 실행
python3 -m ai_server.llm_server
