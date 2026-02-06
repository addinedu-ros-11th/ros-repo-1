#!/bin/bash

# AI Server 시작 스크립트

echo "================================"
echo "AI Inference Server 시작"
echo "================================"

# 프로젝트 루트로 이동
cd "$(dirname "$0")/.."

# Python 경로 설정
export PYTHONPATH="${PYTHONPATH}:$(pwd)"

# AI Server 실행
python3 -m ai_server.server
